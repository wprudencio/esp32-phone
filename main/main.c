/*
 * ESP32-S3 USB Audio Class (UAC 1.0) host demo.
 *
 * Plays a 440 Hz sine wave through a USB headphone/speaker connected to the
 * ESP32-S3 native USB port (the second USB port on two-port dev kits).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "usb/usb_host.h"
#include "usb/uac_host.h"

static const char *TAG = "sine";

#define SINE_FREQUENCY_HZ       440.0f
#define SINE_AMPLITUDE          0.20f       /* 20% full scale: gentle on ears */

#define PREFERRED_SAMPLE_RATE   48000
#define PREFERRED_CHANNELS      2
#define PREFERRED_BIT_RES       16

#define USB_HOST_TASK_PRIORITY  5
#define UAC_TASK_PRIORITY       5
#define AUDIO_TASK_PRIORITY     4

#define AUDIO_CHUNK_FRAMES      256         /* ~5.3 ms at 48 kHz */
#define PCM_BUFFER_BYTES        4096

#define UAC_RING_BUFFER_BYTES   32768       /* ~341 ms of 48k/16-bit stereo */
#define UAC_RING_THRESHOLD      8192

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

static QueueHandle_t s_evt_queue;
static SemaphoreHandle_t s_spk_lock;

/* Current speaker stream. Guarded by s_spk_lock. */
static uac_host_device_handle_t s_spk;
static bool s_streaming;
static uint32_t s_sample_rate;
static uint8_t s_channels;
static uint8_t s_subframe_size;
static uint8_t s_bit_resolution;

static double s_phase;

/* -------------------------------------------------------------------------- */
/* Format selection                                                           */
/* -------------------------------------------------------------------------- */

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

static uint32_t alt_fallback_freq(const uac_host_dev_alt_param_t *p, uint32_t preferred)
{
    if (p->sample_freq_type > 0) {
        return p->sample_freq[0];
    }

    uint32_t lower = p->sample_freq_lower;
    uint32_t upper = p->sample_freq_upper;

    if (lower == 0 && upper == 0) {
        return 0;
    }
    if (lower == 0) {
        return upper;
    }
    if (upper == 0) {
        return lower;
    }
    if (lower > upper) {
        uint32_t tmp = lower;
        lower = upper;
        upper = tmp;
    }
    if (preferred == 0 || preferred < lower) {
        return lower;
    }
    if (preferred > upper) {
        return upper;
    }
    return preferred;
}

/*
 * Pick an alternate setting. Preference order:
 *   1. 48 kHz, stereo, 16-bit
 *   2. 48 kHz, 16-bit (any channel count)
 *   3. 48 kHz (any format)
 *   4. first usable alternate setting, with a sample rate close to 48 kHz
 */
static esp_err_t select_stream_format(uac_host_device_handle_t handle,
                                      uint32_t *rate, uint8_t *channels,
                                      uint8_t *subframe_size, uint8_t *bit_resolution)
{
    uac_host_dev_info_t info;
    esp_err_t err = uac_host_get_device_info(handle, &info);
    if (err != ESP_OK) {
        return err;
    }

    for (int pass = 0; pass < 3; pass++) {
        for (uint8_t alt = 1; alt <= info.iface_alt_num; alt++) {
            uac_host_dev_alt_param_t p;
            if (uac_host_get_device_alt_param(handle, alt, &p) != ESP_OK) {
                continue;
            }
            if (p.format != 1) { /* PCM only */
                continue;
            }
            if (!alt_supports_freq(&p, PREFERRED_SAMPLE_RATE)) {
                continue;
            }
            if (pass == 0 && !(p.channels == PREFERRED_CHANNELS && p.bit_resolution == PREFERRED_BIT_RES)) {
                continue;
            }
            if (pass == 1 && p.bit_resolution != PREFERRED_BIT_RES) {
                continue;
            }

            *rate = PREFERRED_SAMPLE_RATE;
            *channels = p.channels;
            *subframe_size = p.subframe_size;
            *bit_resolution = p.bit_resolution;
            ESP_LOGI(TAG, "Using alt %u: %"PRIu32" Hz, %u ch, %u-bit (%u-byte subframe)",
                     alt, *rate, *channels, *bit_resolution, *subframe_size);
            return ESP_OK;
        }
    }

    /* No 48 kHz setting: take the first usable one and its first rate. */
    for (uint8_t alt = 1; alt <= info.iface_alt_num; alt++) {
        uac_host_dev_alt_param_t p;
        if (uac_host_get_device_alt_param(handle, alt, &p) != ESP_OK || p.format != 1) {
            continue;
        }
        uint32_t fallback = alt_fallback_freq(&p, PREFERRED_SAMPLE_RATE);
        if (fallback == 0) {
            continue;
        }
        *rate = fallback;
        *channels = p.channels;
        *subframe_size = p.subframe_size;
        *bit_resolution = p.bit_resolution;
        ESP_LOGW(TAG, "48 kHz unsupported, falling back to alt %u: %"PRIu32" Hz, %u ch, %u-bit",
                 alt, *rate, *channels, *bit_resolution);
        return ESP_OK;
    }

    return ESP_ERR_NOT_FOUND;
}

/* -------------------------------------------------------------------------- */
/* Sine generation / playback                                                 */
/* -------------------------------------------------------------------------- */

static void fill_sine(uint8_t *buf, size_t frames)
{
    const double two_pi = 2.0 * M_PI;
    const double step = two_pi * SINE_FREQUENCY_HZ / (double)s_sample_rate;
    const size_t bytes_per_frame = (size_t)s_channels * s_subframe_size;

    for (size_t i = 0; i < frames; i++) {
        double v = sin(s_phase) * SINE_AMPLITUDE;
        s_phase += step;
        if (s_phase >= two_pi) {
            s_phase -= two_pi;
        }

        int32_t sample;
        if (s_bit_resolution <= 16) {
            sample = (int32_t)(v * 32767.0);
        } else if (s_bit_resolution <= 24) {
            sample = (int32_t)(v * 8388607.0);
        } else {
            sample = (int32_t)(v * 2147483647.0);
        }

        uint8_t *frame = buf + i * bytes_per_frame;
        for (uint8_t ch = 0; ch < s_channels; ch++) {
            uint8_t *p = frame + (size_t)ch * s_subframe_size;
            uint32_t u = (uint32_t)sample;
            for (uint8_t b = 0; b < s_subframe_size; b++) {
                p[b] = (u >> (8 * b)) & 0xFF;
            }
        }
    }
}

static void audio_task(void *arg)
{
    static uint8_t pcm[PCM_BUFFER_BYTES];

    ESP_LOGI(TAG, "Audio task ready, waiting for a USB audio device...");

    while (true) {
        xSemaphoreTake(s_spk_lock, portMAX_DELAY);

        if (!s_streaming || s_spk == NULL) {
            xSemaphoreGive(s_spk_lock);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        const size_t bytes_per_frame = (size_t)s_channels * s_subframe_size;
        size_t frames = sizeof(pcm) / bytes_per_frame;
        if (frames > AUDIO_CHUNK_FRAMES) {
            frames = AUDIO_CHUNK_FRAMES;
        }

        fill_sine(pcm, frames);

        esp_err_t err = uac_host_device_write(s_spk, pcm, frames * bytes_per_frame,
                                              pdMS_TO_TICKS(200));
        xSemaphoreGive(s_spk_lock);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "uac_host_device_write failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

/* -------------------------------------------------------------------------- */
/* UAC / USB host plumbing                                                    */
/* -------------------------------------------------------------------------- */

static void uac_device_callback(uac_host_device_handle_t handle,
                                const uac_host_device_event_t event, void *arg)
{
    app_evt_t evt = {
        .group = APP_EVT_DEVICE,
        .device = {
            .handle = handle,
            .event = event,
        },
    };
    xQueueSend(s_evt_queue, &evt, 0);
}

static void uac_host_callback(uint8_t addr, uint8_t iface_num,
                              const uac_host_driver_event_t event, void *arg)
{
    app_evt_t evt = {
        .group = APP_EVT_DRIVER,
        .driver = {
            .addr = addr,
            .iface_num = iface_num,
            .event = event,
        },
    };
    xQueueSend(s_evt_queue, &evt, 0);
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
        ESP_LOGE(TAG, "Could not open UAC speaker interface: %s", esp_err_to_name(err));
        return;
    }

    uac_host_dev_info_t info;
    if (uac_host_get_device_info(handle, &info) == ESP_OK) {
        ESP_LOGI(TAG, "USB audio device connected: VID:PID %04X:%04X, iface %u",
                 info.VID, info.PID, iface_num);
    }
    uac_host_printf_device_param(handle);

    uint32_t rate;
    uint8_t channels, subframe, bits;
    err = select_stream_format(handle, &rate, &channels, &subframe, &bits);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No usable PCM output format found (%s)", esp_err_to_name(err));
        uac_host_device_close(handle);
        return;
    }

    const uac_host_stream_config_t stream_config = {
        .channels = channels,
        .bit_resolution = bits,
        .sample_freq = rate,
    };

    err = uac_host_device_start(handle, &stream_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start UAC stream: %s", esp_err_to_name(err));
        uac_host_device_close(handle);
        return;
    }

    /* These are optional; not every headset implements them. */
    if ((err = uac_host_device_set_volume(handle, 60)) != ESP_OK) {
        ESP_LOGW(TAG, "Volume control not available: %s", esp_err_to_name(err));
    }
    if ((err = uac_host_device_set_mute(handle, false)) != ESP_OK) {
        ESP_LOGW(TAG, "Mute control not available: %s", esp_err_to_name(err));
    }

    xSemaphoreTake(s_spk_lock, portMAX_DELAY);
    s_spk = handle;
    s_sample_rate = rate;
    s_channels = channels;
    s_subframe_size = subframe;
    s_bit_resolution = bits;
    s_phase = 0.0;
    s_streaming = true;
    xSemaphoreGive(s_spk_lock);

    ESP_LOGI(TAG, "Playing %.0f Hz sine at %"PRIu32" Hz, %u ch, %u-bit",
             (double)SINE_FREQUENCY_HZ, rate, channels, bits);
}

static void handle_disconnected(uac_host_device_handle_t handle)
{
    bool was_speaker = false;

    xSemaphoreTake(s_spk_lock, portMAX_DELAY);
    if (handle == s_spk) {
        s_streaming = false;
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
                ESP_LOGI(TAG, "Microphone interface found (ignored)");
                break;
            default:
                break;
            }
        } else {
            switch (evt.device.event) {
            case UAC_HOST_DRIVER_EVENT_DISCONNECTED:
                handle_disconnected(evt.device.handle);
                break;
            case UAC_HOST_DEVICE_EVENT_TX_DONE:
            case UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR:
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

void app_main(void)
{
    /* USB host enumeration details are useful while bringing up a new device. */
    esp_log_level_set("USBH", ESP_LOG_DEBUG);

    s_evt_queue = xQueueCreate(10, sizeof(app_evt_t));
    s_spk_lock = xSemaphoreCreateMutex();
    assert(s_evt_queue != NULL);
    assert(s_spk_lock != NULL);

    TaskHandle_t uac_task = NULL;
    BaseType_t ret = xTaskCreatePinnedToCore(uac_lib_task, "uac_events", 4096, NULL,
                                             UAC_TASK_PRIORITY, &uac_task, 0);
    assert(ret == pdTRUE);

    ret = xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096, (void *)uac_task,
                                  USB_HOST_TASK_PRIORITY, NULL, 0);
    assert(ret == pdTRUE);

    ret = xTaskCreatePinnedToCore(audio_task, "sine_audio", 4096, NULL,
                                  AUDIO_TASK_PRIORITY, NULL, 0);
    assert(ret == pdTRUE);
}
