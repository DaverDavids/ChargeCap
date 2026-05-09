DISCLAIMER: This code was made with the help of AI/LLMs.

# ChargeCap

A USB inline charge limiter for lithium-ion battery devices. Set the current when you start charging, and the device will cut power when the current has dropped to half.

## How It Works

ChargeCap exploits the Li-ion charge curve: charge current stays high during bulk charge, then tapers linearly as voltage approaches the cell ceiling (~4.2V/cell). By sampling peak current at the start of a charge cycle and cutting off at a set fraction of that peak (default: 50%), the device stops charging at roughly 80–90% — regardless of pack size or cell count.

- **Long-press button:** samples current peak and sets the cutoff threshold
- **Short-press button:** toggles output on/off (manual override)
- **OLED display:** shows live current, cutoff threshold, charge elapsed time, and status
- **Auto-cutoff:** MOSFET cuts USB power when current drops to threshold; display inverts to signal completion

## Hardware

- **MCU:** ESP32-C3
- **Current sensing:** Shunt resistor (0.027 Ω) + ADC (GPIO 0)
- **Output control:** GPIO 10 driving a MOSFET/relay
- **Display:** SSD1306 128×32 OLED (I²C: SDA GPIO 7, SCL GPIO 6)
- **Button:** GPIO 8

## Firmware Features

- Web dashboard (Highcharts) showing live current + full charge history
- Tiered ring-buffer data storage: 100 recent points (100ms), 60 medium (1 min), 100 long (adaptive)
- EMA smoothing on ADC data. Built in esp32-c3 ADC is not sensitive enough to be precise, but precision is not needed here. Smoothing eliminates large noise spikes.

## Setup

1. Create a `Secrets.h` file alongside `ChargeCap.ino`:
   ```cpp
   #define MYSSID "your_wifi_ssid"
   #define MYPSK  "your_wifi_password"
   ```
2. Install libraries: `Adafruit GFX`, `Adafruit SSD1306`, `ArduinoOTA`, `ESPmDNS`
3. Flash to ESP32-C3 via Arduino IDE
4. Tune `SHUNT_RESISTANCE`, `ADC_REFERENCE_VOLTAGE`, and `CURRENT_OFFSET` constants for your hardware
