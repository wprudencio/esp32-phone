/*
 * esp32-phone — ESP32-S3 USB-headphone internet radio
 *
 * Streams Rádio USP (https://flow.emm.usp.br:8008/radiousp-128.mp3,
 * MP3 128 kbps, 44.1 kHz stereo, Icecast over HTTPS) over Wi-Fi and
 * plays it on a USB Audio Class 1.0 headphone attached to the ESP32-S3
 * native USB port.
 *
 * Audio path:
 *   HTTPS stream -> MP3 simple decoder -> stereo -> 48 kHz resampler
 *   -> PCM ring buffer (PSRAM) -> USB host UAC driver -> headphone
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "esp_crt_bundle.h"
#include "esp_http_client.h"

#include "esp_audio_dec_default.h"
#include "esp_audio_simple_dec.h"
#include "esp_audio_simple_dec_default.h"

#include "usb/usb_host.h"
#include "usb/uac_host.h"

#include "secrets.h"

static const char *TAG = "radio";

/* ---------------------------------------------------------------- */
/* Configuration                                                    */
/* ---------------------------------------------------------------- */

#define STREAM_URL          "https://flow.emm.usp.br:8008/radiousp-128.mp3"
#define STREAM_USER_AGENT   "esp32-phone/1.0"

#define OUT_SAMPLE_RATE     48000   /* headphone native rate (UAC 1.0) */
#define OUT_BIT_RES         16

/* PCM ring buffer: decoded 48 kHz stereo 16-bit samples (~1.4 s) */
#define PCM_RING_BYTES      (256 * 1024)

/* HTTP chunk pulled from the network per read */
#define HTTP_CHUNK_BYTES    2048

/* Decoder output scratch */
#define DEC_OUT_BYTES       (8 * 1024)
#define STEREO_BUF_BYTES    (16 * 1024)
#define RESAMP_BUF_BYTES    (64 * 1024)
#define RESAMP_MAX_FRAMES   (RESAMP_BUF_BYTES / 4)

#define PLAYER_CHUNK_BYTES  2048    /* 512 frames @ 48 kHz stereo */

#define USB_HOST_TASK_PRIORITY  5
#define UAC_TASK_PRIORITY       5
#define PLAYER_TASK_PRIORITY    4
#define NET_TASK_PRIORITY       4

/* ---------------------------------------------------------------- */
/* USB / UAC plumbing (same shape as the sine-wave version)         */
/* ---------------------------------------------------------------- */

typedef enum {
    APP_EVT_DRIVER,
    APP_EVT_DEVICE,
} app_evt_group_t;

typedef struct {
    app_evt_group_t group;
    union {
        struct {
            uint8_t addr;
            uint8_t iface_num;
            uac_host_driver_event_t event;
        } driver;
        struct {
            uac_host_device_handle_t handle;
            uac_host_device_event_t event;
        } device;
    };
} app_evt_t;

#define UAC_RING_BUFFER_BYTES   32768
#define UAC_RING_THRESHOLD      8192

static QueueHandle_t s_evt_queue;
static SemaphoreHandle_t s_spk_lock;

/* Current speaker, guarded by s_spk_lock */
static uac_host_device_handle_t s_spk;
static bool s_out_ready;
static uint8_t s_out_channels;

/* ---------------------------------------------------------------- */
/* Audio pipeline state                                             */
/* ---------------------------------------------------------------- */

static StreamBufferHandle_t s_pcm;

/* Decoder + scratch buffers, owned by the network task */
static esp_audio_simple_dec_handle_t s_dec;
static uint8_t *s_dec_out;
static uint8_t *s_stereo;
static uint8_t *s_resamp;

/* Wi-Fi */
static EventGroupHandle_t s_wifi_events;
#define WIFI_CONNECTED_BIT BIT0

/* Resampler: 44.1k (or whatever) -> 48k, linear interpolation, stereo.
 * pos is in units where one input frame = OUT_SAMPLE_RATE units. */
typedef struct {
    uint32_t src_rate;
    int64_t pos;
    int32_t prev[2];
    int32_t cur[2];
    bool primed;
} resamp_t;

static resamp_t s_rs;

/* ---------------------------------------------------------------- */
/* Resampler                                                         */
/* ---------------------------------------------------------------- */

static void resampler_reset(uint32_t src_rate)
{
    memset(&s_rs, 0, sizeof(s_rs));
    s_rs.src_rate = src_rate;
}

static size_t resample_stereo(const int16_t *in, size_t in_frames,
                              int16_t *out, size_t out_cap_frames)
{
    size_t out_n = 0;
    for (size_t i = 0; i < in_frames; i++) {
        int32_t l = in[2 * i];
        int32_t r = in[2 * i + 1];

        if (!s_rs.primed) {
            s_rs.cur[0] = l;
            s_rs.cur[1] = r;
            s_rs.primed = true;
            continue;
        }

        s_rs.prev[0] = s_rs.cur[0];
        s_rs.prev[1] = s_rs.cur[1];
        s_rs.cur[0] = l;
        s_rs.cur[1] = r;

        while (s_rs.pos < OUT_SAMPLE_RATE) {
            if (out_n >= out_cap_frames) {
                return out_n; /* caller must drain before continuing */
            }
            int32_t f = (int32_t)((s_rs.pos << 15) / OUT_SAMPLE_RATE); /* 0..32767 */
            out[2 * out_n]     = (int16_t)(s_rs.prev[0] +
                                           (((s_rs.cur[0] - s_rs.prev[0]) * f) >> 15));
            out[2 * out_n + 1] = (int16_t)(s_rs.prev[1] +
                                           (((s_rs.cur[1] - s_rs.prev[1]) * f) >> 15));
            out_n++;
            s_rs.pos += s_rs.src_rate;
        }
        s_rs.pos -= OUT_SAMPLE_RATE;
    }
    return out_n;
}

/* ---------------------------------------------------------------- */
/* PCM ring helpers                                                  */
/* ---------------------------------------------------------------- */

static void pcm_push(const uint8_t *data, size_t len)
{
    while (len > 0) {
        size_t w = xStreamBufferSend(s_pcm, data, len, pdMS_TO_TICKS(500));
        data += w;
        len -= w;
    }
}

/* Audible status tone so a dead network is distinguishable from silence */
static void push_beep(uint32_t freq_hz, uint32_t duration_ms, float amplitude)
{
    if (s_resamp == NULL || !s_out_ready) {
        return;
    }
    int16_t *out = (int16_t *)s_resamp;
    size_t frames = ((OUT_SAMPLE_RATE / 1000) * duration_ms) / 10 * 10;
    if (frames > RESAMP_MAX_FRAMES) {
        frames = RESAMP_MAX_FRAMES;
    }
    for (size_t i = 0; i < frames; i++) {
        int16_t v = (int16_t)(amplitude * 32767.0f *
                              sinf(2.0f * (float)M_PI * (float)freq_hz * (float)i / (float)OUT_SAMPLE_RATE));
        out[2 * i] = v;
        out[2 * i + 1] = v;
    }
    pcm_push(s_resamp, frames * 4);
}

/* ---------------------------------------------------------------- */
/* Decoding                                                          */
/* ---------------------------------------------------------------- */

static void handle_decoded(const uint8_t *pcm, size_t len)
{
    esp_audio_simple_dec_info_t info;
    if (esp_audio_simple_dec_get_info(s_dec, &info) != ESP_AUDIO_ERR_OK) {
        return;
    }
    if (info.bits_per_sample != 16 || info.channel == 0) {
        ESP_LOGE(TAG, "unsupported decoded format: %"PRIu32" Hz, %u ch, %u bits",
                 info.sample_rate, info.channel, info.bits_per_sample);
        return;
    }

    if (info.sample_rate != s_rs.src_rate) {
        ESP_LOGI(TAG, "stream format: %"PRIu32" Hz, %u ch, %u bits, %"PRIu32" bps",
                 info.sample_rate, info.channel, info.bits_per_sample, info.bitrate);
        resampler_reset(info.sample_rate);
    }

    uint32_t src_ch = info.channel > 2 ? 2 : info.channel;
    size_t frames = len / (info.channel * 2);
    if (frames * 4 > STEREO_BUF_BYTES) {
        frames = STEREO_BUF_BYTES / 4;
    }

    /* normalize to stereo */
    int16_t *st = (int16_t *)s_stereo;
    if (src_ch == 2) {
        memcpy(st, pcm, frames * 4);
    } else {
        const int16_t *in = (const int16_t *)pcm;
        for (size_t i = 0; i < frames; i++) {
            st[2 * i] = in[i];
            st[2 * i + 1] = in[i];
        }
    }

    size_t out_frames = resample_stereo(st, frames, (int16_t *)s_resamp, RESAMP_MAX_FRAMES);
    if (out_frames > 0) {
        pcm_push(s_resamp, out_frames * 4);
    }
}

static void decode_chunk(uint8_t *data, size_t len)
{
    esp_audio_simple_dec_raw_t raw = {
        .buffer = data,
        .len = len,
        .eos = false,
    };

    while (raw.len > 0) {
        esp_audio_simple_dec_out_t out = {
            .buffer = s_dec_out,
            .len = DEC_OUT_BYTES,
        };

        esp_audio_err_t ret = esp_audio_simple_dec_process(s_dec, &raw, &out);
        if (ret == ESP_AUDIO_ERR_BUFF_NOT_ENOUGH) {
            ESP_LOGW(TAG, "decoder output buffer too small, growing to %"PRIu32,
                     out.needed_size);
            uint8_t *grown = heap_caps_realloc(s_dec_out, out.needed_size,
                                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (grown == NULL) {
                ESP_LOGE(TAG, "cannot grow decoder output buffer");
                return;
            }
            s_dec_out = grown;
            continue;
        }
        if (ret != ESP_AUDIO_ERR_OK) {
            /* Skip a byte to let the parser resync on junk/partial frames */
            raw.buffer++;
            raw.len--;
            continue;
        }

        if (out.decoded_size > 0) {
            handle_decoded(s_dec_out, out.decoded_size);
        }
        if (raw.consumed == 0) {
            break; /* parser needs more input */
        }
        raw.buffer += raw.consumed;
        raw.len -= raw.consumed;
    }
}

/* ---------------------------------------------------------------- */
/* HTTP streaming                                                    */
/* ---------------------------------------------------------------- */

/* Returns true when the connection was established and then ended,
 * false when it never worked. */
static bool stream_once(void)
{
    esp_http_client_config_t cfg = {
        .url = STREAM_URL,
        .user_agent = STREAM_USER_AGENT,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 8000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        ESP_LOGE(TAG, "esp_http_client_init failed");
        return false;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP open failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return false;
    }

    esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if (status != 200) {
        ESP_LOGE(TAG, "HTTP status %d from " STREAM_URL, status);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }

    char *val = NULL;
    if (esp_http_client_get_header(client, "icy-name", &val) == ESP_OK && val) {
        ESP_LOGI(TAG, "station: %s", val);
    }
    if (esp_http_client_get_header(client, "ice-audio-info", &val) == ESP_OK && val) {
        ESP_LOGI(TAG, "audio info: %s", val);
    }

    ESP_LOGI(TAG, "streaming from " STREAM_URL);

    uint8_t buf[HTTP_CHUNK_BYTES];
    int idle_reads = 0;
    while (true) {
        int n = esp_http_client_read(client, (char *)buf, sizeof(buf));
        if (n > 0) {
            idle_reads = 0;
            decode_chunk(buf, n);
            continue;
        }
        if (n == 0) {
            ESP_LOGW(TAG, "stream closed by server");
            break;
        }
        int e = esp_http_client_get_errno(client);
        if (e == EAGAIN || e == EWOULDBLOCK) {
            if (++idle_reads > 3) {
                ESP_LOGW(TAG, "stream stalled");
                break;
            }
            continue;
        }
        ESP_LOGW(TAG, "read error, errno=%d", e);
        break;
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return true;
}

/* ---------------------------------------------------------------- */
/* Networking task: Wi-Fi + stream + decode                          */
/* ---------------------------------------------------------------- */

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *event_data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected, retrying");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got IP " IPSTR, IP2STR(&evt->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi = { 0 };
    strlcpy((char *)wifi.sta.ssid, WIFI_SSID, sizeof(wifi.sta.ssid));
    strlcpy((char *)wifi.sta.password, WIFI_PASS, sizeof(wifi.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void net_task(void *arg)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    s_wifi_events = xEventGroupCreate();
    wifi_init_sta();

    ESP_LOGI(TAG, "connecting to Wi-Fi \"" WIFI_SSID "\"...");
    xEventGroupWaitBits(s_wifi_events, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    /* Decoder + scratch */
    /* Register frame decoders first, then the simple-decoder wrappers on top */
    if (esp_audio_dec_register_default() != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(TAG, "failed to register default audio decoders");
        vTaskDelete(NULL);
    }
    if (esp_audio_simple_dec_register_default() != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(TAG, "failed to register default simple audio decoders");
        vTaskDelete(NULL);
    }

    esp_audio_simple_dec_cfg_t dec_cfg = {
        .dec_type = ESP_AUDIO_SIMPLE_DEC_TYPE_MP3,
        .use_frame_dec = false, /* parser finds frames in arbitrary chunks */
    };
    if (esp_audio_simple_dec_open(&dec_cfg, &s_dec) != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(TAG, "failed to open MP3 decoder");
        vTaskDelete(NULL);
    }

    s_dec_out = heap_caps_calloc(1, DEC_OUT_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    s_stereo = heap_caps_calloc(1, STEREO_BUF_BYTES, MALLOC_CAP_SPIRAM);
    s_resamp = heap_caps_calloc(1, RESAMP_BUF_BYTES, MALLOC_CAP_SPIRAM);
    if (s_dec_out == NULL || s_stereo == NULL || s_resamp == NULL) {
        ESP_LOGE(TAG, "scratch buffer allocation failed");
        vTaskDelete(NULL);
    }
    resampler_reset(0);

    /* Short confirmation blip once the pipeline is ready */
    push_beep(1200, 120, 0.25f);

    int failures = 0;
    while (true) {
        bool connected = stream_once();

        failures = connected ? 0 : (failures + 1);
        uint32_t backoff_ms = connected ? 250 : 1000 * (failures < 15 ? failures : 15);

        if (failures > 0) {
            ESP_LOGW(TAG, "stream failed (%d), retrying in %"PRIu32" ms",
                     failures, backoff_ms);
            push_beep(880, 150, 0.25f);
        }
        vTaskDelay(pdMS_TO_TICKS(backoff_ms));
    }
}

/* ---------------------------------------------------------------- */
/* Player task: PCM ring -> USB headphone                           */
/* ---------------------------------------------------------------- */

static void player_task(void *arg)
{
    static uint8_t chunk[PLAYER_CHUNK_BYTES];

    ESP_LOGI(TAG, "player ready");

    while (true) {
        size_t n = xStreamBufferReceive(s_pcm, chunk, sizeof(chunk), pdMS_TO_TICKS(250));

        esp_err_t err = ESP_OK;
        xSemaphoreTake(s_spk_lock, portMAX_DELAY);
        if (s_out_ready && s_spk != NULL) {
            if (n == 0) {
                /* Underrun: feed silence so the device keeps clocking */
                memset(chunk, 0, sizeof(chunk));
                n = sizeof(chunk);
            }
            if (s_out_channels == 1) {
                /* downmix stereo -> mono in place */
                int16_t *pcm = (int16_t *)chunk;
                size_t frames = n / 4;
                for (size_t i = 0; i < frames; i++) {
                    pcm[i] = (int16_t)(((int32_t)pcm[2 * i] + pcm[2 * i + 1]) / 2);
                }
                n = frames * 2;
            }
            err = uac_host_device_write(s_spk, chunk, n, pdMS_TO_TICKS(200));
        }
        xSemaphoreGive(s_spk_lock);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "uac write failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

/* ---------------------------------------------------------------- */
/* USB host / UAC driver plumbing                                   */
/* ---------------------------------------------------------------- */

static void uac_device_callback(uac_host_device_handle_t handle,
                                const uac_host_device_event_t event, void *arg)
{
    app_evt_t evt = {
        .group = APP_EVT_DEVICE,
        .device = { .handle = handle, .event = event },
    };
    xQueueSend(s_evt_queue, &evt, 0);
}

static void uac_host_callback(uint8_t addr, uint8_t iface_num,
                              const uac_host_driver_event_t event, void *arg)
{
    app_evt_t evt = {
        .group = APP_EVT_DRIVER,
        .driver = { .addr = addr, .iface_num = iface_num, .event = event },
    };
    xQueueSend(s_evt_queue, &evt, 0);
}

static bool alt_supports_freq(const uac_host_dev_alt_param_t *p, uint32_t freq)
{
    if (p->sample_freq_type > 0) {
        for (int i = 0; i < p->sample_freq_type && i < UAC_FREQ_NUM_MAX; i++) {
            if (p->sample_freq[i] == freq) {
                return true;
            }
        }
        return false;
    }
    return freq >= p->sample_freq_lower && freq <= p->sample_freq_upper;
}

static void handle_tx_connected(uint8_t addr, uint8_t iface_num)
{
    uac_host_device_handle_t handle = NULL;
    const uac_host_device_config_t dev_config = {
        .addr = addr,
        .iface_num = iface_num,
        .buffer_size = UAC_RING_BUFFER_BYTES,
        .buffer_threshold = UAC_RING_THRESHOLD,
        .callback = uac_device_callback,
        .callback_arg = NULL,
    };

    esp_err_t err = uac_host_device_open(&dev_config, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "could not open UAC speaker interface: %s", esp_err_to_name(err));
        return;
    }

    uac_host_dev_info_t info;
    if (uac_host_get_device_info(handle, &info) == ESP_OK) {
        ESP_LOGI(TAG, "USB audio device connected: VID:PID %04X:%04X, iface %u",
                 info.VID, info.PID, iface_num);
    }
    uac_host_printf_device_param(handle);

    /* We always render 48 kHz / 16-bit; accept 1 or 2 channel devices. */
    uint8_t out_channels = 0;
    for (uint8_t alt = 1; alt <= info.iface_alt_num; alt++) {
        uac_host_dev_alt_param_t p;
        if (uac_host_get_device_alt_param(handle, alt, &p) != ESP_OK || p.format != 1) {
            continue;
        }
        if (p.bit_resolution == OUT_BIT_RES &&
                (p.channels == 1 || p.channels == 2) &&
                alt_supports_freq(&p, OUT_SAMPLE_RATE)) {
            out_channels = p.channels;
            break;
        }
    }
    if (out_channels == 0) {
        ESP_LOGE(TAG, "device does not support 48 kHz / 16-bit PCM output");
        uac_host_device_close(handle);
        return;
    }

    const uac_host_stream_config_t stream_config = {
        .channels = out_channels,
        .bit_resolution = OUT_BIT_RES,
        .sample_freq = OUT_SAMPLE_RATE,
    };
    err = uac_host_device_start(handle, &stream_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "could not start UAC stream: %s", esp_err_to_name(err));
        uac_host_device_close(handle);
        return;
    }

    if ((err = uac_host_device_set_volume(handle, 13)) != ESP_OK) {
        ESP_LOGW(TAG, "volume control not available: %s", esp_err_to_name(err));
    }
    if ((err = uac_host_device_set_mute(handle, false)) != ESP_OK) {
        ESP_LOGW(TAG, "mute control not available: %s", esp_err_to_name(err));
    }

    xSemaphoreTake(s_spk_lock, portMAX_DELAY);
    s_spk = handle;
    s_out_channels = out_channels;
    s_out_ready = true;
    xSemaphoreGive(s_spk_lock);

    ESP_LOGI(TAG, "headphone streaming at %d Hz / %u ch / %d-bit",
             OUT_SAMPLE_RATE, out_channels, OUT_BIT_RES);
}

static void handle_disconnected(uac_host_device_handle_t handle)
{
    bool was_speaker = false;

    xSemaphoreTake(s_spk_lock, portMAX_DELAY);
    if (handle == s_spk) {
        s_out_ready = false;
        s_spk = NULL;
        was_speaker = true;
    }
    xSemaphoreGive(s_spk_lock);

    if (was_speaker) {
        ESP_LOGI(TAG, "USB audio device disconnected");
        uac_host_device_close(handle);
    }
}

static void uac_lib_task(void *arg)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    const uac_host_driver_config_t uac_config = {
        .create_background_task = true,
        .task_priority = UAC_TASK_PRIORITY,
        .stack_size = 4096,
        .core_id = 0,
        .callback = uac_host_callback,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(uac_host_install(&uac_config));
    ESP_LOGI(TAG, "UAC host driver installed");

    app_evt_t evt;
    while (xQueueReceive(s_evt_queue, &evt, portMAX_DELAY)) {
        if (evt.group == APP_EVT_DRIVER) {
            switch (evt.driver.event) {
            case UAC_HOST_DRIVER_EVENT_TX_CONNECTED:
                handle_tx_connected(evt.driver.addr, evt.driver.iface_num);
                break;
            case UAC_HOST_DRIVER_EVENT_RX_CONNECTED:
                ESP_LOGI(TAG, "microphone interface found (ignored)");
                break;
            default:
                break;
            }
        } else {
            switch (evt.device.event) {
            case UAC_HOST_DRIVER_EVENT_DISCONNECTED:
                handle_disconnected(evt.device.handle);
                break;
            default:
                break;
            }
        }
    }
}

static void usb_lib_task(void *arg)
{
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };

    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG, "USB host installed");
    xTaskNotifyGive((TaskHandle_t)arg);

    while (true) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
            break;
        }
    }

    ESP_LOGI(TAG, "USB host shutdown");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(NULL);
}

/* ---------------------------------------------------------------- */
/* App start                                                         */
/* ---------------------------------------------------------------- */

void app_main(void)
{
    /* USB host enumeration details are useful while bringing up devices */
    esp_log_level_set("USBH", ESP_LOG_DEBUG);

    s_evt_queue = xQueueCreate(10, sizeof(app_evt_t));
    s_spk_lock = xSemaphoreCreateMutex();
    assert(s_evt_queue != NULL);
    assert(s_spk_lock != NULL);

    /* PCM ring buffer in PSRAM */
    uint8_t *ring_storage = heap_caps_calloc(1, PCM_RING_BYTES, MALLOC_CAP_SPIRAM);
    StaticStreamBuffer_t *ring_struct = heap_caps_calloc(1, sizeof(StaticStreamBuffer_t), MALLOC_CAP_SPIRAM);
    assert(ring_storage != NULL && ring_struct != NULL);
    s_pcm = xStreamBufferCreateStatic(PCM_RING_BYTES, 1, ring_storage, ring_struct);
    assert(s_pcm != NULL);

    TaskHandle_t uac_task = NULL;
    BaseType_t ret = xTaskCreatePinnedToCore(uac_lib_task, "uac_events", 4096, NULL,
                                             UAC_TASK_PRIORITY, &uac_task, 0);
    assert(ret == pdTRUE);

    ret = xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096, (void *)uac_task,
                                  USB_HOST_TASK_PRIORITY, NULL, 0);
    assert(ret == pdTRUE);

    ret = xTaskCreatePinnedToCore(player_task, "player", 4096, NULL,
                                  PLAYER_TASK_PRIORITY, NULL, 0);
    assert(ret == pdTRUE);

    ret = xTaskCreatePinnedToCore(net_task, "net", 32768, NULL,
                                  NET_TASK_PRIORITY, NULL, 1);
    assert(ret == pdTRUE);

    ESP_LOGI(TAG, "esp32-phone radio started");
}
