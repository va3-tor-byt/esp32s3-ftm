# ESP32-S3 Wi-Fi FTM Indoor Positioning

Indoor positioning using Wi-Fi **Fine Timing Measurement** (IEEE 802.11mc
round-trip-time ranging) on ESP32-S3 boards. Three fixed **anchors** act as FTM
responders; a mobile **tag** ranges to each one, trilaterates its (x, y)
position, and shows it on a built-in 1.14" TFT.

## Hardware

| Role | Board | Notes |
|---|---|---|
| Anchors ×3 | ESP32-S3 DevKitC-style, N16R8 (dual USB-C) | Flash/monitor through the **COM** port |
| Tag ×1 | TENSTAR TS-ESP32-S3 1.14" TFT (ESP32-S3FH4R2) | Adafruit Feather ESP32-S3 TFT pin-compatible; native USB only; LiPo connector |

## How it works

- Each anchor runs a soft AP named `FTM_ANCHOR_<id>` on channel 1 with the FTM
  responder enabled.
- The tag scans for those SSIDs, then calls `esp_wifi_ftm_initiate_session()`
  for each anchor's BSSID **without connecting**, so one tag can range to many
  anchors.
- Each distance is offset-corrected, smoothed (exponential moving average) and
  fed into a linearised three-circle trilateration.

## Project layout

```
platformio.ini     one project, four environments
src/anchor.cpp     FTM responder (ANCHOR_ID set per environment)
src/tag.cpp        FTM initiator + smoothing + trilateration + TFT + calibration
```

| Environment | Board | Source |
|---|---|---|
| `anchor1`, `anchor2`, `anchor3` | `esp32-s3-devkitc-1` | `anchor.cpp` |
| `tag` | `adafruit_feather_esp32s3_tft` | `tag.cpp` |

## Building and flashing

Requires VS Code with the PlatformIO IDE extension.

1. Open this folder in VS Code.
2. Plug in **one board at a time**.
3. In the PlatformIO sidebar, expand the matching environment and click
   **Upload**. Do not use the tasks under *Default*; that runs every environment.

Command-line equivalent:

```
pio run -e anchor1 -t upload
pio run -e anchor2 -t upload
pio run -e anchor3 -t upload
pio run -e tag -t upload
pio device monitor -e tag
```

If the tag won't connect for upload: hold **BOOT**, tap **RST**, release
**BOOT**, upload, then press **RST** to run.

## Setup

Default anchor layout (metres), set in `ANCHOR_POS` in `src/tag.cpp`:

```
A3 (0, 3)
 |
A1 (0, 0) -------- A2 (4, 0)
```

Mount the anchors at the same height (about 1.5–2 m) with clear line of sight.

## Calibration

FTM timestamps are taken inside the chip, so RF front-end and antenna delays
appear as a constant per-pair distance offset (1 ns ≈ 15 cm). The tag has a
built-in wizard:

1. Hold **BOOT** on the tag, or send `c` over Serial.
2. For each anchor, place the tag `CAL_DIST_CM` (default 200 cm) away and press
   **BOOT**.
3. The tag takes 40 readings and stores `offset = median − true distance` in
   NVS flash. The interquartile spread is shown as a quality check
   (>50 cm usually means multipath).

Serial commands: `c` calibrate, `p` print offsets, `x` clear offsets.

## Roadmap

- [ ] Web portal hosted by the tag: live floor-plan view, calibration, settings,
      CSV export
- [ ] Anchor positions and settings stored in NVS (no re-flash to change layout)
- [ ] Better filtering (median / Kalman)
- [ ] Fourth anchor + least-squares solver
- [ ] Anchor-to-anchor self-calibration and auto-survey
- [ ] Optional MQTT bridge for logging
