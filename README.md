# ESP32-C6 Zigbee Climate & CO₂ Sensor Node

> A Zigbee End Device that streams real-time temperature, humidity, and CO₂ concentration to your smart-home hub — no Wi-Fi needed.

[![](https://img.shields.io/badge/Platform-PlatformIO-blue)](https://platformio.org)
[![](https://img.shields.io/badge/Framework-Arduino-green)](https://arduino.cc)
[![](https://img.shields.io/badge/MCU-ESP32--C6-purple)](https://www.espressif.com/en/products/socs/esp32-c6)
[![](https://img.shields.io/badge/Zigbee-ED%20(End%20Device)-yellow)](https://en.wikipedia.org/wiki/Zigbee)
[![](https://img.shields.io/badge/Sensors-AHT21%20%C2%B7%20ENS160-orange)](https://github.com/Muhammad-Yunus/ESP32C6_ZIGBEE_DEVICE)
[![](https://img.shields.io/badge/I2C-200kHz-cyan)]()
[![](https://img.shields.io/badge/Reporting-10s-red)]()

---

## Project Structure

```
ESP32C6_ZIGBEE_DEVICE/
├── include/
│   └── README.md
├── lib/
│   └── README.md
├── src/
│   └─��� main.cpp          # Main application (Zigbee + sensor I2C)
├── test/
│   └── README.md
├── .pio/
│   ├── libdeps/          # Auto-installed PlatformIO libraries
│   └── build/
└── platformio.ini        # Build & library configuration
```

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                    ESP32-C6                         │
│                                                     │
│  ┌───────────────┐       ┌────────────────────────┐ │
│  │ Zigbee ED     │       │  I2C Bus (200 kHz)     │ │
│  │ (RF radio)    │       │  SCL: GPIO6  SDA: GPIO5│ │
│  │               │       └──────┬─────────┬───────┘ │
│  │  EP10: Climate│              │         │         │
│  │  EP11: CO2    │       ┌──────┴───┐  ┌──┴────────┐│
│  └───────────────┘       │  AHT21   │  │  ENS160   ││
│                          │  (0x38)  │  │  (0x53)   ││
│  ┌──────────────┐        │  Temp/Hum│  │   CO₂ ppm ││
│  │  LED: GPIO8  │        └──────────┘  └───────────┘│
│  └──────────────┘                                   │
└─────────────────────────────────────────────────────┘
        │
        ▼ Zigbee 802.15.4
┌──────────────────────┐
│  Zigbee Coordinator  │
│  (e.g. CC2530,       │
│   ZNP, ESP32-C5,     │
│   Home Assistant)    │
└──────────────────────┘
```

**Reporting flow:**

1. **Every 5 s** — AHT21 is polled via I2C (`getTemperature()`, `getHumidity()`)
2. **Every 1 s** — ENS160 measurement trigger + result read (CO₂ + accuracy)
3. **Every 10 s** — All three values are written to Zigbee endpoints:
   - `climateSensor` (EP10): temperature + humidity
   - `carbonDioxideSensor` (EP11): CO₂ ppm
4. Zigbee automatic reporting pushes values to the coordinator

---

## Tech Stack

| Layer | Component |
|-------|-----------|
| **MCU** | ESP32-C6 (RISC-V, Wi-Fi 6 + 802.15.4) |
| **Framework** | Arduino (PlatformIO `espressif32`) |
| **Zigbee** | ESP32-C6 Zigbee ED mode (`Zigbee.h`) |
| **I2C Sensors** | AHT21 (0x38), ENS160 (0x53) |
| **I2C Lib** | `Wire` (ESP32 built-in), `AHT20` (adrianmacech) |
| **Build** | PlatformIO |

---

## How to Build & Flash

### Prerequisites
- [PlatformIO](https://platformio.org) installed
- USB-C cable connected to your ESP32-C6 DevKitC-1

### Build
```bash
platformio run
```

### Flash
```bash
platformio run --target upload
```

### Monitor (serial log)
```bash
platformio device monitor
```
Expected output on successful boot:
```
AHT21 initialized
ENS160 detected, sending soft reset...
ENS160 ready
Starting Zigbee End Device...
Connected to Zigbee network
Automatic Zigbee reporting configured for 10 seconds
Temp: 25.3 C  Hum: 54.2%  CO2: 412 ppm (+- 12.0%)
```

### Clean build (if needed)
```bash
platformio run -t clean
```

---

## Wiring & Pin Mapping

### I2C Bus (shared by both sensors)

| ESP32-C6 Pin | Signal | Connected To |
|:---:|:---|:---|
| **GPIO5** | SDA | AHT21 `SDA` · ENS160 `SDA` |
| **GPIO6** | SCL | AHT21 `SCL` · ENS160 `SCL` |
| **3.3 V** | VCC | AHT21 `VDD` · ENS160 `VCC` |
| **GND** | GND | AHT21 `GND` · ENS160 `GND` |

### Sensor Details

| Sensor | I2C Address | Function |
|--------|:-----------:|----------|
| **AHT21** | `0x38` | Temperature + Humidity |
| **ENS160** | `0x53` | CO₂ concentration (ppm) + accuracy (%) |

### LED

| ESP32-C6 Pin | Function |
|:---:|---|
| **GPIO8** | RGB LED (status blink every 500 ms) |

### Schematic (simple)

```
  VCC (3.3V) ──┬──────────────────────┐
                │                      │
              AHT21 VDD              ENS160 VCC
                │                      │
   GPIO5 (SDA) ─┴──── AHT21 SDA       │
                │                      │
   GPIO6 (SCL) ─┴──── AHT21 SCL       │
                │                      │
   GND ─────────┴──────────────────────┴──── GND
```

### Notes
- I2C bus runs at **200 kHz** — both sensors support this speed
- No pull-up resistors needed on most AHT21/ENS160 modules (built-in)
- ENS160 needs **~3 minutes warm-up** after power-on for accurate CO₂ readings
- ENS160 soft reset is sent on boot to recalibrate baseline
