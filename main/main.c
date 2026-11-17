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
#include "esp_timer.h"
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

/* Playback only starts once this much decoded audio is buffered (and
 * re-buffers to this level after an underrun), so Wi-Fi jitter never
 * starves the USB isochronous stream. ~0.5 s of audio. */
#define PCM_PREBUFFER_BYTES (96 * 1024)

/* HTTP chunk pulled from the network per read */
#define HTTP_CHUNK_BYTES    2048

/* MP3 input accumulator: the simple decoder consumes only complete
 * frames and leaves partial-frame bytes unconsumed. Those bytes must
 * be carried over and re-fed with the next HTTP read; dropping them
 * (as the first version did) halved the effective data rate and made
 * playback run at half speed. */
#define MP3_ACCUM_BYTES     (HTTP_CHUNK_BYTES + 2048)

/* Decoder output scratch */
#define DEC_OUT_BYTES       (8 * 1024)
/* Resampler work buffer: RES_HALF tail frames + up to RES_MAX_IN_FRAMES
 * new stereo frames, 4 bytes per frame */
#define RES_TAIL_BYTES      ((size_t)RES_HALF * 4)
#define RES_WORK_BYTES      (RES_TAIL_BYTES + (size_t)RES_MAX_IN_FRAMES * 4)
#define RESAMP_BUF_BYTES    (128 * 1024)
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

/* Set while the PCM ring is above the prebuffer level; cleared on
 * underrun and device disconnect so playback re-buffers. */
static volatile bool s_play_primed;

/* ---------------------------------------------------------------- */
/* Audio pipeline state                                             */
/* ---------------------------------------------------------------- */

static StreamBufferHandle_t s_pcm;

/* Decoder + scratch buffers, owned by the network task */
static esp_audio_simple_dec_handle_t s_dec;
static uint8_t *s_dec_out;
static uint8_t *s_work;    /* resampler: RES_HALF tail + new stereo frames */
static uint8_t *s_resamp;

/* Wi-Fi */
static EventGroupHandle_t s_wifi_events;
#define WIFI_CONNECTED_BIT BIT0

/* Pipeline health counters, logged periodically by the player task.
 * 32-bit writes are atomic on the S3, so no locking for these. */
static volatile uint32_t s_stat_frames;    /* decoded MP3 frames        */
static volatile uint32_t s_stat_underruns; /* PCM ring ran dry         */
static volatile uint32_t s_stat_resyncs;   /* decoder parse-error skips */
static volatile uint32_t s_stat_drops;     /* chunks dropped on write   */
static volatile uint32_t s_stat_min_ring;  /* min PCM ring fill (bytes) */
static volatile uint32_t s_stat_written;   /* bytes handed to the UAC   */
static volatile uint32_t s_stat_net;       /* bytes read from HTTP      */
static volatile uint32_t s_stat_dec_in;    /* bytes the decoder consumed*/
static volatile uint32_t s_stat_read_us;   /* us spent blocked in read  */
static volatile uint32_t s_stat_loop;      /* read loop iterations      */
static volatile uint32_t s_time_decode;    /* us inside decode_chunk    */
static volatile uint32_t s_time_resamp;    /* us inside resample_stereo */
static volatile uint32_t s_time_push;     /* us inside pcm_push        */
static volatile uint32_t s_stat_rscalls;  /* resample_stereo calls     */
static volatile uint32_t s_stat_inframes; /* input frames fed to it    */
static volatile uint32_t s_bench_us;      /* canary loop time, us      */

/* Resampler: arbitrary src rate -> 48 kHz, Kaiser-windowed-sinc
 * polyphase interpolation, stereo. Quality (measured on the algorithm):
 * 0.0 dB ripple to 20 kHz, images below -35 dBc.
 *
 * Performance notes (learned the hard way, measured on-device):
 * - process decoded chunks as blocks with a contiguous tap window;
 *   shifting a history per input frame cost a ROM memmove each
 * - keep the coefficient lerp hoisted out of the per-channel MACs
 * - the first version of this loop needed 1.69 s of CPU per 1 s of
 *   audio at -Os; this structure plus -O2 runs many times faster
 */
#define RES_TAPS        64      /* FIR taps per output sample          */
#define RES_HALF        (RES_TAPS / 2)
#define RES_PHASES      256     /* fractional-delay table rows (+1)    */
#define RES_KAISER_BETA 10.0f
#define RES_MAX_IN_FRAMES 4096  /* cap on decoded frames per chunk      */

typedef struct {
    uint32_t src_rate;
    int64_t pos;            /* next output's phase: 1 input frame = OUT_RATE units */
    int32_t pending_m;      /* interval index of next output, relative to next chunk */
} resamp_t;

static resamp_t s_rs;
/* Polyphase coefficients in Q15 int16. */
static int16_t s_res_tbl[RES_PHASES + 1][RES_TAPS];

/* ---------------------------------------------------------------- */
/* Resampler                                                         */
/* ---------------------------------------------------------------- */

/* Modified Bessel I0 by series (for the Kaiser window) */
static float res_i0(float x)
{
    float sum = 1.0f, term = 1.0f;
    for (int k = 1; k < 20; k++) {
        term *= (x * x / 4.0f) / (float)(k * k);
        sum += term;
    }
    return sum;
}

static void resampler_reset(uint32_t src_rate)
{
    memset(&s_rs, 0, sizeof(s_rs));
    s_rs.src_rate = src_rate;
    if (src_rate == 0) {
        return;
    }

    /* (Re)build the polyphase table: Kaiser-windowed sinc cutting at
     * min(src, out) Nyquist, each phase normalized to unity DC gain,
     * then quantized to Q15 int16 for integer-only playback math. */
    float fc = (float)(src_rate < OUT_SAMPLE_RATE ? src_rate : OUT_SAMPLE_RATE) * 0.5f;
    float sinc_scale = 2.0f * fc / (float)src_rate;
    float i0_beta = res_i0(RES_KAISER_BETA);
    for (int ph = 0; ph <= RES_PHASES; ph++) {
        float p = (float)ph / (float)RES_PHASES;
        float row[RES_TAPS];
        float sum = 0.0f;
        for (int j = 0; j < RES_TAPS; j++) {
            float d = (float)(j - (RES_HALF - 1)) - p;
            float arg = sinc_scale * d;
            float sinc = (arg == 0.0f) ? 1.0f
                        : sinf((float)M_PI * arg) / ((float)M_PI * arg);
            float u = d / (float)RES_HALF;
            float w = 0.0f;
            if (u >= -1.0f && u <= 1.0f) {
                w = res_i0(RES_KAISER_BETA * sqrtf(1.0f - u * u)) / i0_beta;
            }
            row[j] = sinc * w;
            sum += row[j];
        }
        for (int j = 0; j < RES_TAPS; j++) {
            int32_t q = (int32_t)lrintf(row[j] / sum * 32768.0f);
            if (q > 32767) {
                q = 32767;
            } else if (q < -32767) {
                q = -32767;
            }
            s_res_tbl[ph][j] = (int16_t)q;
        }
    }
}

static size_t resample_stereo(size_t in_frames, int16_t *out, size_t out_cap_frames)
{
    const int16_t *w = (const int16_t *)s_work; /* tail + new, contiguous */
    size_t out_n = 0;
    int32_t m = s_rs.pending_m; /* interval of next output, rel. to new frames */

    while (m + (int32_t)RES_HALF < (int32_t)in_frames) {
        while (s_rs.pos < OUT_SAMPLE_RATE) {
            if (out_n >= out_cap_frames) {
                goto done;
            }
            /* Fractional position in [0,1): phase row + lerp weight */
            uint32_t p32 = (uint32_t)s_rs.pos;      /* < OUT_RATE */
            uint32_t x = p32 * RES_PHASES;
            uint32_t ph = x / OUT_SAMPLE_RATE;
            uint32_t rem = x - ph * OUT_SAMPLE_RATE;
            int t8 = (int)((rem * 256U) / OUT_SAMPLE_RATE);

            const int16_t *c0 = s_res_tbl[ph];
            const int16_t *c1 = s_res_tbl[ph + 1];
            /* Tap j = h[2j], h[2j+1]: input frames m+1 .. m+RES_TAPS */
            const int16_t *h = &w[2 * (m + 1)];

            /* Interpolate the coefficient row once, share it for both
             * channels, then plain int16 x Q15 MACs. */
            int16_t c[RES_TAPS];
            for (int j = 0; j < RES_TAPS; j++) {
                c[j] = (int16_t)(c0[j] + ((((int32_t)c1[j] - (int32_t)c0[j]) * t8) >> 8));
            }
            int32_t al = 0, ar = 0;
            for (int j = 0; j < RES_TAPS; j++) {
                al += (int32_t)h[2 * j] * c[j];
                ar += (int32_t)h[2 * j + 1] * c[j];
            }
            int32_t l = (al + (1 << 14)) >> 15;
            int32_t r = (ar + (1 << 14)) >> 15;
            out[2 * out_n]     = (int16_t)(l > 32767 ? 32767 : (l < -32768 ? -32768 : l));
            out[2 * out_n + 1] = (int16_t)(r > 32767 ? 32767 : (r < -32768 ? -32768 : r));
            out_n++;
            s_rs.pos += s_rs.src_rate;
        }
        s_rs.pos -= OUT_SAMPLE_RATE;
        m++;
    }

done:
    /* Rebase the interval index for the next chunk */
    s_rs.pending_m = m - (int32_t)in_frames;
    /* Keep the last RES_HALF frames as the next chunk's tail */
    memmove(s_work, s_work + in_frames * 4, (size_t)RES_HALF * 4);
    return out_n;
}

/* ---------------------------------------------------------------- */
/* PCM ring helpers                                                  */
/* ---------------------------------------------------------------- */

static void pcm_push(const uint8_t *data, size_t len)
{
    int64_t t0 = esp_timer_get_time();
    while (len > 0) {
        size_t w = xStreamBufferSend(s_pcm, data, len, pdMS_TO_TICKS(500));
        data += w;
        len -= w;
    }
    s_time_push += (uint32_t)(esp_timer_get_time() - t0);
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

/* Canary benchmark: 1M iterations of a trivial multiply-add loop.
 * On a healthy 160 MHz core this takes ~10-30 ms. If it measures much
 * more while streaming, something is stealing or stalling the core. */
static void bench_canary(void)
{
    volatile int32_t sink = 0;
    int64_t t0 = esp_timer_get_time();
    for (int32_t i = 1; i <= 1000000; i++) {
        sink += i * 3;
    }
    (void)sink;
    s_bench_us = (uint32_t)(esp_timer_get_time() - t0);
}

/* Resampler benchmark: 38 MP3-frames' worth (1 s of 44.1 kHz audio)
 * through the real resample_stereo path. */
static void bench_resamp(void)
{
    int16_t *st = (int16_t *)(s_work + RES_TAIL_BYTES);
    for (int i = 0; i < 1152; i++) {
        st[2 * i] = (int16_t)(i * 7);
        st[2 * i + 1] = (int16_t)(i * 5);
    }
    resampler_reset(44100);
    int64_t t0 = esp_timer_get_time();
    size_t outs = 0;
    for (int c = 0; c < 38; c++) {
        outs += resample_stereo(1152, (int16_t *)s_resamp, RESAMP_MAX_FRAMES);
    }
    int64_t t1 = esp_timer_get_time();
    ESP_LOGI(TAG, "resamp bench: %d us for 1s of audio (%d outs, %d us/frame)",
             (int)(t1 - t0), (int)outs, (int)((t1 - t0) / 38));
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
    if (frames > RES_MAX_IN_FRAMES) {
        frames = RES_MAX_IN_FRAMES;
    }

    /* Normalize to stereo, writing after the resampler's tail */
    int16_t *st = (int16_t *)(s_work + RES_TAIL_BYTES);
    if (src_ch == 2) {
        memcpy(st, pcm, frames * 4);
    } else {
        const int16_t *in = (const int16_t *)pcm;
        for (size_t i = 0; i < frames; i++) {
            st[2 * i] = in[i];
            st[2 * i + 1] = in[i];
        }
    }

    int64_t rs_t0 = esp_timer_get_time();
    s_stat_rscalls++;
    s_stat_inframes += frames;
    size_t out_frames = resample_stereo(frames, (int16_t *)s_resamp, RESAMP_MAX_FRAMES);
    s_time_resamp += (uint32_t)(esp_timer_get_time() - rs_t0);
    if (out_frames > 0) {
        pcm_push(s_resamp, out_frames * 4);
    }
}

static uint8_t s_mp3_buf[MP3_ACCUM_BYTES];
static size_t s_mp3_len;

/* Feed accumulated input to the decoder. Unconsumed partial-frame
 * bytes are kept at the front of the buffer; the return value is
 * how many bytes to carry over for the next call. */
static size_t decode_chunk(uint8_t *data, size_t len)
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
                break;
            }
            s_dec_out = grown;
            continue;
        }
        if (ret != ESP_AUDIO_ERR_OK) {
            /* Parser error: drop one byte and let it hunt for the next
             * frame sync. Counted so glitches of this kind are visible
             * in the periodic stats line. */
            s_stat_resyncs++;
            raw.buffer++;
            raw.len--;
            continue;
        }

        if (out.decoded_size > 0) {
            s_stat_frames++;
            handle_decoded(s_dec_out, out.decoded_size);
        }
        if (raw.consumed == 0) {
            break; /* parser needs more input to complete the frame */
        }
        s_stat_dec_in += raw.consumed;
        raw.buffer += raw.consumed;
        raw.len -= raw.consumed;
    }

    if (raw.len > 0 && raw.buffer != data) {
        memmove(data, raw.buffer, raw.len);
    }
    return raw.len;
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
    s_mp3_len = 0; /* fresh connection: drop any stale partial frame */
    int idle_reads = 0;
    int64_t win_start = esp_timer_get_time();
    uint32_t win_reads = 0, win_bytes = 0, win_read_us = 0;
    while (true) {
        int64_t t0 = esp_timer_get_time();
        int n = esp_http_client_read(client, (char *)buf, sizeof(buf));
        int64_t t1 = esp_timer_get_time();
        s_stat_read_us += (uint32_t)(t1 - t0);
        s_stat_loop++;
        win_read_us += (uint32_t)(t1 - t0);
        win_reads++;
        if (n > 0) {
            idle_reads = 0;
            s_stat_net += n;
            win_bytes += n;

            size_t space = sizeof(s_mp3_buf) - s_mp3_len;
            if ((size_t)n > space) {
                /* Parser is badly stuck: flush and resync */
                ESP_LOGW(TAG, "mp3 accumulator overflow, flushing");
                s_mp3_len = 0;
                space = sizeof(s_mp3_buf);
            }
            memcpy(s_mp3_buf + s_mp3_len, buf, n);
            s_mp3_len += n;

            int64_t dc_t0 = esp_timer_get_time();
            s_mp3_len = decode_chunk(s_mp3_buf, s_mp3_len);
            s_time_decode += (uint32_t)(esp_timer_get_time() - dc_t0);
        } else if (n < 0) {
            int e = esp_http_client_get_errno(client);
            if ((e == EAGAIN || e == EWOULDBLOCK) && ++idle_reads <= 1) {
                /* 8 s timeout: give it one more chance */
                continue;
            }
            ESP_LOGW(TAG, "read error, errno=%d", e);
            break;
        } else {
            ESP_LOGW(TAG, "stream closed by server");
            break;
        }

        int64_t now_us = esp_timer_get_time();
        if (now_us - win_start >= 15000000) {
            wifi_ap_record_t ap;
            int8_t rssi = 0;
            if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                rssi = ap.rssi;
            }
            uint32_t win_ms = (uint32_t)((now_us - win_start) / 1000);
            bench_canary();
            ESP_LOGI(TAG,
                     "net: %.1fkB/s reads=%u avg=%uB wait=%u%% dec=%u%% rs=%u%% push=%u%% "
                     "rscalls=%u in=%u bench=%uus rssi=%d",
                     (float)win_bytes / 1024.0f / (float)(win_ms / 1000 ? win_ms / 1000 : 1),
                     (unsigned)win_reads,
                     win_reads ? (unsigned)(win_bytes / win_reads) : 0,
                     (unsigned)((uint64_t)win_read_us * 100 / (uint64_t)(now_us - win_start)),
                     (unsigned)((uint64_t)s_time_decode * 100 / (uint64_t)(now_us - win_start)),
                     (unsigned)((uint64_t)s_time_resamp * 100 / (uint64_t)(now_us - win_start)),
                     (unsigned)((uint64_t)s_time_push * 100 / (uint64_t)(now_us - win_start)),
                     (unsigned)s_stat_rscalls,
                     (unsigned)s_stat_inframes,
                     (unsigned)s_bench_us,
                     (int)rssi);
            win_start = now_us;
            win_reads = 0;
            win_bytes = 0;
            win_read_us = 0;
            s_time_decode = 0;
            s_time_resamp = 0;
            s_time_push = 0;
            s_stat_rscalls = 0;
            s_stat_inframes = 0;
        }
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
    /* Keep the radio always on. Default modem-sleep (MIN_MODEM) parks the
     * radio between beacons and adds hundreds of milliseconds of jitter
     * to the TCP stream, which the player hears as dropouts. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
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
    s_work = heap_caps_calloc(1, RES_WORK_BYTES, MALLOC_CAP_SPIRAM);
    s_resamp = heap_caps_calloc(1, RESAMP_BUF_BYTES, MALLOC_CAP_SPIRAM);
    if (s_dec_out == NULL || s_work == NULL || s_resamp == NULL) {
        ESP_LOGE(TAG, "scratch buffer allocation failed");
        vTaskDelete(NULL);
    }
    resampler_reset(0);

    /* Short confirmation blip once the pipeline is ready */
    push_beep(1200, 120, 0.25f);

    bench_canary();
    bench_resamp();
    ESP_LOGI(TAG, "canary (idle network): %u us", (unsigned)s_bench_us);

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
    TickType_t last_stats = xTaskGetTickCount();
    uint32_t last_net = 0, last_dec = 0, last_frames = 0;

    s_stat_min_ring = PCM_RING_BYTES;

    ESP_LOGI(TAG, "player ready");

    while (true) {
        /* Prebuffer gate: don't start (or resume) playback until enough
         * decoded audio is queued to ride out network jitter. The UAC
         * driver stops its isochronous stream whenever its own ring
         * drops below one URB and only restarts it on the next write,
         * so a starving feeder produces audible start/stop clicks. */
        if (!s_play_primed) {
            size_t avail = xStreamBufferBytesAvailable(s_pcm);
            if (s_stat_min_ring > avail) {
                s_stat_min_ring = avail;
            }
            if (avail >= PCM_PREBUFFER_BYTES) {
                s_play_primed = true;
                ESP_LOGI(TAG, "buffer primed, playback starts");
            } else {
                /* Keep the DAC clocked with silence while waiting so
                 * the isochronous stream never stalls mid-audio. */
                memset(chunk, 0, sizeof(chunk));
                esp_err_t err = ESP_OK;
                bool wrote = false;
                xSemaphoreTake(s_spk_lock, portMAX_DELAY);
                if (s_out_ready && s_spk != NULL) {
                    err = uac_host_device_write(s_spk, chunk, sizeof(chunk), pdMS_TO_TICKS(200));
                    wrote = (err == ESP_OK);
                }
                xSemaphoreGive(s_spk_lock);
                if (!wrote) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                continue;
            }
        }

        /* Accumulate a full chunk; a dry spell of 100 ms while primed
         * is a real underrun (falls back to the gate above). */
        size_t have = 0;
        while (have < sizeof(chunk)) {
            size_t r = xStreamBufferReceive(s_pcm, chunk + have,
                                            sizeof(chunk) - have, pdMS_TO_TICKS(100));
            have += r;
            if (r == 0) {
                break;
            }
        }

        esp_err_t err = ESP_OK;
        xSemaphoreTake(s_spk_lock, portMAX_DELAY);
        if (s_out_ready && s_spk != NULL) {
            if (have == 0) {
                /* Underrun: silence-fill and re-buffer */
                s_stat_underruns++;
                s_play_primed = false;
                memset(chunk, 0, sizeof(chunk));
                have = sizeof(chunk);
                ESP_LOGW(TAG, "PCM ring underrun, re-buffering");
            }
            size_t n = have;
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
            if (err != ESP_OK) {
                s_stat_drops++;
            } else {
                s_stat_written += n;
            }
        } else {
            have = 0; /* no device: drop and keep draining */
        }
        xSemaphoreGive(s_spk_lock);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "uac write failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        /* Track the minimum ring fill and print a health line */
        size_t avail = xStreamBufferBytesAvailable(s_pcm);
        if (s_stat_min_ring > avail) {
            s_stat_min_ring = avail;
        }
        TickType_t now = xTaskGetTickCount();
        if (now - last_stats >= pdMS_TO_TICKS(15000)) {
            uint32_t dt = (now - last_stats) / pdMS_TO_TICKS(1000) ?: 1;
            ESP_LOGI(TAG,
                     "stats: ring=%u min=%u (%u%%) net=%uB/s dec=%uB/s mp3=%.1ff/s "
                     "underruns=%u resyncs=%u drops=%u",
                     (unsigned)avail, (unsigned)s_stat_min_ring,
                     (unsigned)((uint64_t)s_stat_min_ring * 100 / PCM_RING_BYTES),
                     (unsigned)((s_stat_net - last_net) / dt),
                     (unsigned)((s_stat_dec_in - last_dec) / dt),
                     (float)(s_stat_frames - last_frames) / (float)dt,
                     (unsigned)s_stat_underruns, (unsigned)s_stat_resyncs,
                     (unsigned)s_stat_drops);
            last_net = s_stat_net;
            last_dec = s_stat_dec_in;
            last_frames = s_stat_frames;
            s_stat_min_ring = PCM_RING_BYTES;
            last_stats = now;
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
        s_play_primed = false; /* re-buffer before resuming on replug */
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
