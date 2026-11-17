# esp32-phone — ESP32-S3 USB Audio Host

An [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) firmware that turns an
**ESP32-S3** dev kit into a **USB audio host**: it enumerates a USB Audio Class
(UAC 1.0) headphone connected to the board's native USB port and plays a
generated **440 Hz sine wave** on it (48 kHz / 16-bit / stereo, ~20 % amplitude).

Built as the first brick of a bigger "esp32-phone" project — speaker output
works, and the same stack is the foundation for mic capture and real calls.

```
┌────────────┐   USB-C "UART" port   ┌──────────────┐
│   Mac / PC ├───────────────────────┤ CH343 bridge │── power, flash, logs
└────────────┘                       └──────────────┘
                                     ┌──────────────┐
┌────────────┐   USB-C "USB" port    │ ESP32-S3     │
│ USB        ├───────────────────────┤ GPIO19 D-    │── USB host (OTG)
│ headphone  │        (OTG adapter)  │ GPIO20 D+   │
└────────────┘                       └──────────────┘
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

## Software architecture

Three FreeRTOS tasks plus the USB host stack:

```
app_main
 ├─ usb_events task   ── usb_host_install(), pumps usb_host_lib_handle_events()
 ├─ uac_events task   ── uac_host_install(), dispatches driver/device events
 └─ sine_audio task   ── generates PCM, uac_host_device_write() into the
                          driver's ring buffer (blocking write = pacing)

USB stack:  esp-usb host lib (espressif/usb)  →  usb_host_uac 1.5.0  →  device
```

Flow when a headphone is plugged in:

1. USB host library enumerates the device and parses descriptors.
2. `usb_host_uac` fires `UAC_HOST_DRIVER_EVENT_TX_CONNECTED` (speaker found;
   the mic fires `RX_CONNECTED`, which we currently ignore).
3. The app opens the speaker interface (32 KB ring buffer, 8 KB threshold).
4. Format negotiation: prefer **48 kHz / stereo / 16-bit**, fall back to any
   supported PCM alt setting (`select_stream_format()` in `main/main.c`).
5. `uac_host_device_start()` sets the interface alt setting + sample rate,
   then we set volume (60) and unmute.
6. The sine task generates 256-frame chunks with continuous phase and writes
   them into the ring buffer; TX-done callbacks drain it over isochronous OUT.

The sine generator packs signed little-endian PCM for any channel count and
16/24/32-bit resolution, so it works with less typical headsets too.

## The one non-obvious fix: USB FIFO bias

The S3's USB-OTG core has a small internal FIFO (1.25 KB) split between RX,
non-periodic TX and **periodic TX** — and isochronous audio OUT streams from
the *periodic TX* slice. With the default *balanced* bias that slice is only
**128 bytes**, while USB headphones commonly need 256–512-byte isochronous OUT
packets (ours needs 384). The stream then fails with:

```
E (1084) HCD DWC: EP MPS (384) exceeds supported limit (128)
E (1094) USB HOST: EP allocation error ESP_ERR_NOT_SUPPORTED
E (1105) uac-host: Unable to claim Interface
```

`sdkconfig.defaults` therefore sets:

```
CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODIC_OUT=y
```

which grows the periodic-TX slice to ~1 KB — audio flows immediately.

Trade-off: the RX (microphone) slice shrinks to ~132 bytes. A headset mic with
large isochronous IN packets will need a custom FIFO configuration instead of
this Kconfig shortcut.

## Build & flash

Requires ESP-IDF v5.5+ with the `esp32s3` tools installed:

```sh
. ~/esp/esp-idf/export.sh
cd esp32-phone
idf.py set-target esp32s3
idf.py -p /dev/cu.usbmodem5AE80117911 flash monitor
```

`Ctrl+]` exits the monitor. First build downloads
`espressif/usb_host_uac` + `espressif/usb` via the IDF component manager
(versions pinned in `dependencies.lock`).

Expected log:

```
I (300) sine: USB host installed
I (300) uac-host: Install Succeed, Version: 1.5.0
I (1059) sine: USB audio device connected: VID:PID 0020:0B21, iface 1
I (1073) sine: Using alt 1: 48000 Hz, 2 ch, 16-bit (2-byte subframe)
I (1088) uac-host: Set volume 60%, Interface 1-1
I (1091) uac-host: Unmute Interface 1-1
I (1094) sine: Playing 440 Hz sine at 48000 Hz, 2 ch, 16-bit
```

`tools/monitor.py` is a dependency-free alternative to `idf.py monitor` that
resets the board and captures N seconds of log:

```sh
python3 tools/monitor.py /dev/cu.usbmodem5AE80117911 25
```

## Tuning knobs

All in `main/main.c`:

| Macro | Default | Meaning |
|---|---|---|
| `SINE_FREQUENCY_HZ` | `440.0f` | Tone frequency |
| `SINE_AMPLITUDE` | `0.20f` | 0.0–1.0, software level (stay low!) |
| `PREFERRED_SAMPLE_RATE` | `48000` | Asked first during negotiation |
| `UAC_RING_BUFFER_BYTES` | `32768` | Driver ring buffer (~170 ms at 48k/st/16-bit) |

Hardware volume is set in `handle_tx_connected()`:
`uac_host_device_set_volume(handle, 60)` (0–100, optional per device).

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Nothing after "UAC host driver installed" | Device has no power (see *VBUS* above) or dead OTG adapter |
| `EP MPS (...) exceeds supported limit (128)` | The FIFO-bias fix is missing — keep `CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODIC_OUT=y` |
| Volume/mute control warnings | Headset doesn't implement those requests — harmless |
| Sound but clicks/dropouts | Increase `UAC_RING_BUFFER_BYTES` or raise `AUDIO_TASK_PRIORITY` |
| UAC 2.0-only device (e.g. Apple USB-C dongle) | This driver is UAC 1.0 only; needs a UAC2 host driver or quirk fork |

## Repo layout

```
├── CMakeLists.txt           # ESP-IDF project boilerplate
├── sdkconfig.defaults       # target-independent config (incl. the FIFO fix)
├── main/
│   ├── main.c               # USB host + UAC app + sine generator
│   └── idf_component.yml    # component manager deps (usb_host_uac, usb)
└── tools/
    └── monitor.py           # board reset + serial log capture
```

## Possible next steps

- Open the mic interface (RX) and loop it back to the speaker (needs a custom
  FIFO split for full duplex)
- Play real audio: WAV from SPIFFS, or MP3 through an ESP-ADF decoder
- Wi-Fi transport (SIP? websocket?) for an actual "phone"
