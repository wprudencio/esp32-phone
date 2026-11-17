# esp32-phone — ESP32-S3 USB-headphone internet radio

An [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) firmware that turns an
**ESP32-S3** dev kit into an **internet radio played through a USB headphone**:
it joins Wi-Fi, streams **Rádio USP** (Icecast, MP3 128 kbps / 44.1 kHz stereo
over HTTPS), decodes and resamples it to 48 kHz, and plays it on a USB Audio
Class (UAC 1.0) headphone attached to the board's native USB port.

```
                 Wi-Fi "CATNIP"
                      │  HTTPS (TLS)
                      ▼
┌─────────────────────────────────────────────────────────────┐
│ esp_http_client ─► MP3 simple decoder ─► 44.1k→48k resampler │
│ (espressif/esp_audio_codec)                                  │
│                      │                                       │
│                      ▼                                       │
│ PCM ring buffer, 256 KB in PSRAM (~1.4 s jitter buffer)      │
│                      │                                       │
│                      ▼                                       │
│ USB host UAC driver (usb_host_uac) ─► USB headphone, 48 kHz  │
└─────────────────────────────────────────────────────────────┘
```

## Hardware

| Item | Value |
|---|---|
| Board | ESP32-S3 dev kit, **two USB-C ports** (16 MB flash, 8 MB PSRAM) |
| `UART` port | CH343 USB-serial bridge — power, flashing, serial monitor |
| `USB` port | Native USB-OTG (GPIO19 = D-, GPIO20 = D+) — host mode |
| Device under test | USB headphone, `VID:PID 0020:0B21`, UAC 1.0 speaker + mic |

The headphone plugs into the **native USB port** (via a USB-C→USB-A OTG adapter
if it has a USB-A plug). Flashing and logs go through the **UART port**.

> ⚠️ An **ESP32-C3 can't do this** — it has no USB-OTG host. This project needs
> an ESP32-S2 / S3 / P4 (any chip with a USB-OTG peripheral in host mode).

### If a device doesn't enumerate at all

Some dev kits don't feed 5 V back out of the native USB-C port, so the
headphone never powers up. If the log shows nothing after
`UAC host driver installed`, power the headphone externally:

```
ESP32-S3              USB-A female (OTG adapter)
5V    ───────────────  VBUS (red)
GND   ───────────────  GND  (black)
GPIO20 (D+) ────────  D+    (green)
GPIO19 (D-) ────────  D-    (white)
```

(A powered USB hub also works. GND must be common.)

## Setup

Wi-Fi credentials live in `main/secrets.h`, which is **gitignored**:

```sh
cp main/secrets_example.h main/secrets.h
# then edit: WIFI_SSID / WIFI_PASS
```

Stream URL and volume are in `main/main.c` (`STREAM_URL`, and
`uac_host_device_set_volume(handle, 13)` in `handle_tx_connected()`).

## Build & flash

Requires ESP-IDF v5.5+ with the `esp32s3` tools installed:

```sh
. ~/esp/esp-idf/export.sh
cd esp32-phone
idf.py set-target esp32s3
idf.py -p /dev/cu.usbmodem5AE80117911 flash monitor
```

`Ctrl+]` exits the monitor. First build downloads `usb_host_uac`, `usb` and
`esp_audio_codec` via the IDF component manager (versions pinned in
`dependencies.lock`).

Expected log:

```
I (1570) radio: headphone streaming at 48000 Hz / 2 ch / 16-bit
I (3336) radio: got IP 192.168.0.144
I (3748) esp-x509-crt-bundle: Certificate validated
I (4253) radio: streaming from https://flow.emm.usp.br:8008/radiousp-128.mp3
I (4465) radio: stream format: 44100 Hz, 2 ch, 16 bits, 128 bps
```

`tools/monitor.py` is a dependency-free alternative to `idf.py monitor` that
resets the board and captures N seconds of log:

```sh
python3 tools/monitor.py /dev/cu.usbmodem5AE80117911 25
```

## How it works

Tasks (all created in `app_main`):

| Task | Core | Job |
|---|---|---|
| `usb_events` | 0 | `usb_host_install()`, pumps USB host library events |
| `uac_events` | 0 | UAC driver events: open/start headphone on connect |
| `player` | 0 | drains the PCM ring into the UAC driver (silence on underrun) |
| `net` | 1 | Wi-Fi, HTTPS stream, MP3 decode, resample, fills the PCM ring |

Details worth knowing:

- **Format negotiation**: the app always renders 48 kHz / 16-bit PCM because
  that is what the headphone advertises. The MP3 stream is 44.1 kHz, so decoded
  audio goes through a 64-tap Kaiser-windowed-sinc polyphase resampler
  (measured 0.0 dB ripple to 20 kHz, images below -35 dBc; 256 phase rows
  with linear interpolation, int16 x Q15 MACs — ~32% of one 240 MHz core).
  Sources at other rates (or mono) are handled too.
- **The decoder** is the `esp_audio_codec` *simple decoder*: its parser scans
  arbitrary chunk boundaries, but **unconsumed partial-frame bytes must be
  carried over to the next read** — dropping them halves the data rate.
- **Registration gotcha**: both `esp_audio_dec_register_default()` *and*
  `esp_audio_simple_dec_register_default()` must be called — the simple
  decoder wraps the frame decoders registered by the first call.
- **Jitter handling**: the player only starts once ~0.5 s of decoded audio is
  buffered (`PCM_PREBUFFER_BYTES`) and re-buffers to that level after an
  underrun. That matters because the UAC host driver halts its isochronous
  stream whenever its ring drops below one URB, and only restarts it on the
  next `uac_host_device_write()` — a starved feeder produces audible clicks.
- **Wi-Fi**: power save is disabled (`esp_wifi_set_ps(WIFI_PS_NONE)`);
  modem-sleep adds hundreds of ms of jitter that the player hears as dropouts.
- **Reconnect**: any read error/close triggers `esp_http_client` re-init with
  linear backoff. TLS uses the standard certificate bundle.
- **Health stats**: every 15 s the firmware logs ring fill level, network and
  decoder throughput, resampler CPU share, underruns, decoder resyncs and
  dropped chunks — plus a start-up benchmark of the resampler.

## sdkconfig notes

- `CONFIG_COMPILER_OPTIMIZATION_PERF=y` (-O2) and 240 MHz CPU — the audio
  pipeline (TLS + MP3 decode + resampling) needs the headroom; at the -Os
  default the resampler loop measured ~4x slower.
- `CONFIG_SPIRAM_SPEED_40M=y` — the PCM ring moves ~200 kB/s so PSRAM
  bandwidth is irrelevant, while 80 MHz ran marginal on this board (random
  memtest failures and cache stalls). 40 MHz has been rock solid.
- `CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODIC_OUT=y` — the S3's 1.25 KB OTG FIFO
  is split RX / non-periodic TX / periodic TX, and isochronous audio OUT
  streams from the periodic-TX slice. The default *balanced* bias caps it at
  **128 bytes**, while USB headphones commonly need 256–512-byte isochronous
  OUT packets (ours needs 384), failing with
  `E HCD DWC: EP MPS (384) exceeds supported limit (128)`.
  Trade-off: the RX (microphone) slice shrinks to ~132 bytes.
- `CONFIG_SPIRAM=y`, octal — PCM ring + resampler scratch live in PSRAM.
- All codecs except the MP3 decoder are disabled
  (`CONFIG_AUDIO_DECODER_*_SUPPORT=n`); with everything enabled the app grew
  past the default 1.5 MB app partition.
- `partitions.csv` gives the factory app 8 MB of the 16 MB flash.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Audio plays slow / half speed, constant glitches | Pipeline starved: check the 15 s stats line — `mp3=` should read ~38 f/s for a 44.1 kHz stream, `underruns=0` |
| Nothing after "UAC host driver installed" | Device has no power (see *VBUS* above) or dead OTG adapter |
| `EP MPS (...) exceeds supported limit (128)` | The FIFO-bias fix is missing — keep `CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODIC_OUT=y` |
| `Decoder MP3 not registered` | `esp_audio_dec_register_default()` must be called before the simple decoder registration |
| Boot aborts with `External RAM failed memory test!` | PSRAM timing marginal — run `CONFIG_SPIRAM_SPEED_40M=y` (this repo does) |
| Interrupt watchdog / random reboots | Same as above: marginal PSRAM stalled both cores on this board |
| `app partition is too small` | Either trim codecs (see sdkconfig notes) or grow the partition |
| Volume/mute control warnings | Headset doesn't implement those requests — harmless |
| Clicks/dropouts | Increase `PCM_RING_BYTES` / `PCM_PREBUFFER_BYTES` or check `rsyncs`/`underruns` in the stats |
| UAC 2.0-only device (e.g. Apple USB-C dongle) | This driver is UAC 1.0 only; needs a UAC2 host driver or quirk fork |

## Repo layout

```
├── CMakeLists.txt           # ESP-IDF project boilerplate
├── partitions.csv           # 8 MB factory app on 16 MB flash
├── sdkconfig.defaults       # target-independent config (FIFO fix, PSRAM, codecs)
├── main/
│   ├── main.c               # Wi-Fi + HTTP stream + MP3 decode + resample + UAC
│   ├── idf_component.yml    # component manager deps (usb_host_uac, usb, esp_audio_codec)
│   ├── secrets_example.h    # copy to secrets.h and fill in Wi-Fi credentials
│   └── secrets.h            # gitignored
└── tools/
    └── monitor.py           # board reset + serial log capture
```

## Possible next steps

- Open the mic interface (RX) and loop it back to the speaker (needs a custom
  FIFO split for full duplex)
- Volume / station selection over a small web UI or buttons
- An actual "phone": SIP or WebRTC call over the same audio pipeline
