# ESPurifier

DIY industrial-grade ceiling-mounted air purifier for a woodworking shop. An ESP32-S3 monitors particulate levels via a Sharp optical dust sensor and automatically controls a 220V fan through a solid-state relay. The unit accepts commands from a 433 MHz RF remote and shows live status on a small OLED display.

## Hardware

| Component | Part | ESP32-S3 Pin |
|---|---|---|
| MCU | Freenove ESP32-S3 Dev Board | — |
| Dust sensor | Sharp GP2Y1010AU0F | LED → GPIO 4, ADC out → GPIO 5 |
| Fan relay | SSR-25DA (3–32V DC trigger) | GPIO 6 |
| RF receiver | 433 MHz superheterodyne + EV1527 2-button fob | GPIO 7 |
| OLED display | 0.96" SSD1306 (I2C) | SDA → GPIO 8, SCL → GPIO 9 |
| AC-DC supply | HLK-PM01 (220V → 5V) | — |
| Filtration | Xiaomi HEPA cylinder + polyester pre-filter | — |

The Sharp sensor LED circuit requires a 150 Ω series resistor and a 220 µF bypass capacitor.

## Firmware

Written in **C++20** targeting **ESP-IDF v5.2+** (GCC 13.2, `-std=gnu++20`).

### Dependencies (managed via `idf_component.yml`)

| Library | Purpose |
|---|---|
| [esp-cpp/espp](https://github.com/esp-cpp/espp) | `espp::Task`, `espp::OneshotAdc`, `espp::Nvs`, `espp::Logger` |
| [espressif/esp_lcd_ssd1306](https://components.espressif.com/) | SSD1306 panel driver via `esp_lcd` |

### Component layout

```
components/
├── dust_sensor/   — Sharp GP2Y1010AU0F sampling (Core 1, 280 µs pulse timing)
├── rf_remote/     — EV1527 decoder via ESP32 RMT peripheral
├── fan_control/   — AUTO / OFF / MANUAL HIGH state machine + SSR GPIO
├── display/       — SSD1306 1 Hz refresh, heartbeat indicator
└── nvs_config/    — NVS persistence for mode and dust thresholds
```

### Fan control modes

| Mode | Behaviour |
|---|---|
| **AUTO** (default) | Fan ON when dust > `thr_hi`; OFF when dust < `thr_lo`. Minimum 5-minute run time once triggered. |
| **MANUAL HIGH** | Fan forced ON for 30 minutes, then reverts to AUTO. |
| **OFF** | Fan forced off regardless of dust level. |

### RF remote (EV1527 2-button fob)

| Button | Action |
|---|---|
| A | Toggle AUTO ↔ MANUAL HIGH |
| B | Toggle current mode ↔ OFF |

### Safety features

- **Task Watchdog (TWDT):** 10-second timeout — device reboots on firmware hang.
- **OTA updates:** WiFi-based over-the-air updates; credentials set via `idf.py menuconfig`.
- **NVS persistence:** Current mode and dust thresholds survive power cuts.

## Building

### Prerequisites

- [ESP-IDF v5.2+](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/) installed and sourced
- 4 MB flash on the target board (standard for Freenove ESP32-S3)

### Configure

```sh
idf.py set-target esp32s3
idf.py menuconfig
# Set: ESPurifier Configuration → WiFi SSID / Password / OTA URL
```

### Build and flash

```sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### OTA update

Host the built firmware binary and point `CONFIG_OTA_URL` at it:

```sh
python3 -m http.server 8080 --directory build/
# Set CONFIG_OTA_URL = "http://<your-ip>:8080/ESPurifier.bin"
```

## Calibrating dust thresholds

The default thresholds (`thr_hi = 0.05 mg/m³`, `thr_lo = 0.02 mg/m³`) are starting points. The Sharp GP2Y1010AU0F output voltage varies with sensor age and supply voltage. To recalibrate:

1. Observe the live dust reading on the OLED during known clean-air and dusty conditions.
2. Update `thr_hi` / `thr_lo` in NVS (a future CLI command or BLE characteristic can expose this without reflashing).

## License

MIT — see [LICENSE](LICENSE).
