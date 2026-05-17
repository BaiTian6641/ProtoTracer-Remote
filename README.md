# ProtoTracer Remote Firmware Framework

This directory contains an ESP-IDF scaffold for the ProtoTracer remote controller based on the ESP32-C6 hardware map extracted from the schematic and EasyEDA netlist.

## Current scope

- Board support generated from the verified ESP32-C6 net mapping
- Boot flow that prioritizes configuration sources in this order:
  1. main board
  2. remote repo
  3. filesystem image
- IDF-native service boundaries for networking, pairing, OTA, sensors, inputs, power, LEDs, and low-power-core orchestration
- A sample manifest in the SPIFFS image for later runtime configuration

## Build

From the repository root:

```powershell
idf.py -C firmware set-target esp32c6
idf.py -C firmware build
```

For first flash:

```powershell
idf.py -C firmware -p COMx flash monitor
```

## Project layout

- `main`: app entrypoint
- `components/common`: shared types and manifest parser
- `components/board_support`: verified pin map and expander layout
- `components/controller_core`: boot orchestration and config storage
- `components/connectivity`: Wi-Fi, pairing, repo sync, and local update server
- `components/drivers`: sensor, input, power, LED, and LP-core-facing stubs
- `data`: SPIFFS image contents, including the default manifest
- `docs`: architecture notes and hardware constraints

## Key documents

- `docs/architecture.md`: current scaffold architecture
- `docs/implementation_process.md`: detailed next-step implementation process for drivers, BLE binding, OTA, display DMA, and security
- `docs/datasheets/README.md`: local reference archive index and download status

## Important hardware note

The current schematic routes `VBAT_MON` through U17 as a comparator output. That gives the ESP32-C6 a digital presence/threshold signal, not a raw ADC battery voltage. Firmware therefore cannot make a fully authoritative chemistry decision from `VBAT_MON` alone. The current charger policy assumes the shipped pack is 3 AAA NiMH cells in series: `CHR_EN` defaults off, then turns on the TPS22917 charger input switch when MAX17055 telemetry classifies the pack as 3-cell NiMH. The TPS22917 input is the `+5V` rail and its output is `CHR_5V` into the BQ25172, so actual charging still requires external 5 V to be present. Clearly high pack voltage is treated as alkaline and charging remains disabled.