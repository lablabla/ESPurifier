# ESPurifier

DIY industrial-grade ceiling-mounted air purifier for a woodworking shop. An ESP32-S3 monitors particulate levels via a Sharp optical dust sensor, controls a 220V fan through a solid-state relay, and provides high-visibility status via a WS2812B 8-LED ring. Commands come from a 433 MHz EV1527 RF remote.

## Hardware

| Component | Part | ESP32-S3 Pin |
|---|---|---|
| MCU | Freenove ESP32-S3 Dev Board | — |
| Dust sensor | Sharp GP2Y1010AU0F | LED → GPIO 4, ADC out → GPIO 5 |
| Fan relay | SSR-25DA (3–32V DC trigger) | GPIO 6 |
| RF receiver | RX480E-4 433 MHz + EV1527 2-button fob | GPIO 7 |
| LED ring | WS2812B 8-LED NeoPixel ring | GPIO 18 (via 74HCT125 level shifter) |
| AC-DC supply | HLK-PM01 (220V → 5V) | — |
| Filtration | Xiaomi HEPA cylinder + polyester pre-filter | — |
| Safety | 1A fuse, PG9 cable glands, IP65 project box | — |

### Wiring notes

**WS2812B LED ring**
- Drive the data line through a **74HCT125** level shifter (3.3V → 5V logic).
- Power the ring directly from the HLK-PM01 5V rail — do **not** draw from the ESP32.
- Place a **470 Ω** resistor in series on the data line and a **1000 µF** capacitor across the 5V/GND power rails at the ring.

**Sharp dust sensor**
- Place a **150 Ω** resistor in series with the LED and a **220 µF** capacitor across the LED supply.

## LED status ring

The 8-LED ring is split into two zones visible from the floor:

### Zone A — LED 0: Operational mode
| Colour | Mode |
|---|---|
| Solid Red | OFF |
| Breathing Yellow | AUTO (sensor-driven) |
| Solid Green | MANUAL HIGH (fan forced on) |

### Zone B — LEDs 1–7: Air quality bar
| LEDs lit | Colour | PM2.5 concentration |
|---|---|---|
| 1–2 | Green | < 35 µg/m³ (clean) |
| 3–4 | Yellow | 35–75 µg/m³ (moderate) |
| 5–6 | Orange | 75–150 µg/m³ (high) |
| 7 | Flashing Red | > 150 µg/m³ (hazardous) |

## Firmware

Written in **C++20** targeting **ESP-IDF v5.2+** (GCC 13.2, `-std=gnu++20`).

### Dependencies (managed via `idf_component.yml`)

| Library | Purpose |
|---|---|
| [esp-cpp/espp](https://github.com/esp-cpp/espp) | `espp::Task`, `espp::OneshotAdc`, `espp::Nvs`, `espp::Logger` |
| [espressif/led_strip](https://components.espressif.com/components/espressif/led_strip) | WS2812B driver via RMT TX peripheral |

### Component layout

```
components/
├── dust_sensor/   — Sharp GP2Y1010AU0F sampling (Core 1, 280 µs pulse timing, µg/m³ output)
├── rf_remote/     — EV1527 decoder via ESP32 RMT RX peripheral
├── fan_control/   — AUTO / OFF / MANUAL HIGH state machine, SSR GPIO, 5-min min-run
├── led_ring/      — WS2812B ring UI via RMT TX; Zone A mode + Zone B air-quality bar
└── nvs_config/    — NVS persistence for mode and dust thresholds
```

> The ESP32-S3 RMT peripheral has separate TX and RX channels — `rf_remote` uses an RX channel (GPIO 7) and `led_ring` uses a TX channel (GPIO 18). They do not interfere.

### Fan control modes

| Mode | Behaviour |
|---|---|
| **AUTO** (default) | Fan ON when dust > `thr_hi` (default 50 µg/m³); OFF when dust < `thr_lo` (default 20 µg/m³). Minimum 5-minute run time once triggered. |
| **MANUAL HIGH** | Fan forced ON for 30 minutes, then reverts to AUTO. |
| **OFF** | Fan forced off. Pressing Button B again restores the previous active mode. |

### RF remote (EV1527 2-button fob)

| Button | Action |
|---|---|
| A | Toggle AUTO ↔ MANUAL HIGH |
| B | Toggle current mode ↔ OFF (restores previous mode when leaving OFF) |

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

```sh
python3 -m http.server 8080 --directory build/
# Set CONFIG_OTA_URL = "http://<your-ip>:8080/ESPurifier.bin"
```

## Calibrating dust thresholds

The Sharp GP2Y1010AU0F calibration used (`density = (V − 0.9 V) / 0.5 mV·m³·µg⁻¹`) gives a linear estimate. Real-world accuracy depends on the sensor age and supply voltage. Default fan thresholds (`thr_hi = 50 µg/m³`, `thr_lo = 20 µg/m³`) are conservative starting points — adjust via NVS once you observe the sensor behaviour in your shop.

## License

MIT — see [LICENSE](LICENSE).
