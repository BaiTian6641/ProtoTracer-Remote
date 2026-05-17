# Architecture Notes

## Boot and configuration order

The scaffold resolves configuration in the same priority you requested:

1. `main_board`
2. `remote_repo`
3. `filesystem_image`

At boot the controller loads the filesystem manifest from the SPIFFS image, then tries to replace it with a higher-priority source.

## Mapping the reference code to ESP-IDF

The reference code in `Reference code` is Arduino-oriented, but the useful ideas map well into ESP-IDF:

- `NetWizard` becomes a SoftAP provisioning path plus a future captive-portal HTTP flow.
- `ElegantOTAPro` becomes an IDF-native HTTP service plus later `esp_https_ota` integration.
- `UserConfigManager`, `AnimationDownloader`, and `FaceModelUpdater` establish the right behavior for remote JSON, checksums, and private repo access.

This framework keeps those behaviors but does not depend on Arduino libraries.

## Private repo access model

The repo client reads an auth token from NVS namespace `secrets` using the key specified in the manifest field `repo.auth_token_nvs_key`.

The manifest also carries:

- `auth_scheme`: `bearer` or `token`
- `accept_header`: optional header for raw API or alternate endpoints

That mirrors the patterns in the reference code where GitHub and Gitee use different authorization styles.

## Hardware-driven constraints

### Buttons and interrupts

Buttons and several sensor alerts do not land directly on ESP32-C6 GPIOs. They pass through the TCA9534 expander and fan into `IO_INT` on GPIO2.

### Battery chemistry detection

The schematic exposes `VBAT_MON` through U17 as a comparator output. That means the controller currently sees only a threshold result, not a raw battery voltage waveform. Firmware can monitor low-battery or threshold events, but it cannot robustly distinguish NiMH from alkaline chemistry from this signal alone.

The charger path is gated by `CHR_EN` on GPIO4 and defaults off. `CHR_EN` drives the active-high TPS22917 load switch from `+5V` to `CHR_5V`, which feeds the BQ25172 charger input. Current firmware assumes the product pack is 3 AAA NiMH cells in series. It uses MAX17055 voltage/current telemetry to classify the pack; the charge-path switch is enabled when the pack is classified as NiMH. Actual charging still requires external 5 V on the TPS22917 input. A high pack voltage above the 3-cell NiMH full range is treated as alkaline and keeps charging disabled.

### Low-power core

The framework reserves LP-core orchestration for later implementation. Based on the current wiring, likely wake candidates are:

- `IO_INT` on GPIO2
- `VBAT_MON` on GPIO12

## Recommended next implementation steps

1. Add the real TCA9534, VCNL4035X01, and LSM6DSOWTR drivers.
2. Implement the BLE or Wi-Fi transport for main-board discovery and binding.
3. Replace the OTA upload stubs with real firmware and SPIFFS update handlers.
4. Add signed manifest validation before accepting remote config or assets.
5. Add an asset manager for animation files, UI resources, and versioned content.