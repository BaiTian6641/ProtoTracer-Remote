# ProtoTracer Remote Implementation Process

This document translates the current ESP-IDF scaffold into an execution plan for the real remote-controller firmware.

## 1. Baseline and constraints

The current scaffold already defines the project structure, configuration priority, and board mapping.

Relevant files:

- `components/board_support/include/prototracer_board.hpp`
- `components/controller_core/controller_app.cpp`
- `components/connectivity/connectivity_services.cpp`
- `components/drivers/driver_services.cpp`
- `data/config/default_manifest.json`

Hardware constraints that must shape the implementation:

1. `IO_INT` on GPIO2 is the only direct interrupt line from the TCA9534 expander, so buttons and multiple sensor interrupts are indirect.
2. `IMU_EN` is actually the IMU `CS` line and should be treated as `IMU_CS`. Because `SDO/SA0` is tied high in the netlist, the IMU should come up on I2C address `0x6B`.
3. `VBAT_MON` is a comparator output from U17, not a raw battery ADC node. Firmware can detect threshold status but cannot reliably identify NiMH vs alkaline chemistry from this board alone.
4. The OLED panel is SSD1351 over 4-line SPI and should be driven with DMA-backed transfers to avoid wasting HP core time on pixel copies.
5. The low-power core should be used conservatively at first. The current board makes LP-core wake supervision realistic, but not full sensor-stack ownership.

## 2. Reference-code conclusions

The attached reference code is useful as a behavior reference, not as drop-in source.

Use as reference:

1. `NetWizard` provides a good user flow for provisioning, setup, and Wi-Fi credential capture.
2. `UserConfigManager`, `AnimationDownloader`, and `FaceModelUpdater` show the intended pattern for repo-backed JSON and asset pulls, optional tokens, and checksum checks.
3. `ElegantOTAPro` shows the expected local update UX and metadata fields such as hardware ID and firmware version.

Do not directly reuse without review:

1. `NetWizard` is AGPL-3.0, so copying or adapting its code into closed product firmware can trigger source-disclosure obligations.
2. `ElegantOTAPro-3-1-4` is under a commercial license. Its code and UI assets should not be copied into this project unless you have confirmed license rights for your distribution model.

Recommended approach:

1. Re-implement the behavior natively in ESP-IDF.
2. Keep the UX ideas, not the code or bundled portal assets.
3. Store repo tokens only in NVS, not in SPIFFS or checked-in manifests.

## 3. Implementation order

Recommended work order:

1. Finish the shared I2C and interrupt foundation.
2. Implement the TCA9534 expander driver and input event routing.
3. Implement the LSM6DSOWTR, VCNL4035X01, and MAX17055 drivers.
4. Implement the SSD1351 DMA display driver.
5. Add BLE discovery, bind, and configuration exchange with the main board.
6. Replace OTA stubs with real firmware and filesystem update handlers.
7. Add manifest signature or checksum validation.
8. Add limited low-power-core behavior around wake policy and background supervision.

## 4. Shared bus and event foundation

Before adding real device drivers, split the current `drivers` stub component into concrete files:

- `components/drivers/include/i2c_bus.hpp`
- `components/drivers/i2c_bus.cpp`
- `components/drivers/include/event_router.hpp`
- `components/drivers/event_router.cpp`

Process:

1. Initialize a single I2C master bus on GPIO22 and GPIO23.
2. Wrap bus access in one mutex or command queue so drivers do not interleave transactions unsafely.
3. Create a small event router task that owns the expander interrupt response path.
4. Configure GPIO2 as an interrupt input and signal the router task from the ISR using a queue or task notification.
5. Keep sensor drivers free of direct GPIO ISR code where possible. Route all expander-driven events through the same event path.

Expected result:

1. One consistent I2C API for TCA9534, LSM6DSOWTR, VCNL4035X01, and MAX17055.
2. One shared event fan-in for buttons and sensor alerts.

## 5. TCA9534 driver process

Suggested new files:

- `components/drivers/include/tca9534.hpp`
- `components/drivers/tca9534.cpp`

Implementation process:

1. Implement register access for Input Port, Output Port, Polarity Inversion, and Configuration.
2. On boot, configure all eight channels as inputs because the current schematic uses the expander only as an input concentrator.
3. Keep polarity inversion disabled unless a later board spin requires it.
4. Read the input state once after initialization to establish the baseline.
5. On each GPIO2 interrupt, read the input register and compare against the cached previous state.
6. Publish semantic events using the verified map:
   `P0=JOTSTICK_BTN`, `P1=BTN0`, `P2=BTN1`, `P3=BTN2`, `P4=GES_INT`, `P5=IMU_INT1`, `P6=IMU_INT2`, `P7=ALARM`.
7. Add debounce only to the human buttons and joystick button, not to sensor interrupt lines.

Expected behavior:

1. Button handling moves out of polling code.
2. Gesture, IMU, and fuel-gauge alerts can be triaged by reading the expander state after `IO_INT` fires.

## 6. LSM6DSOWTR driver process

Suggested new files:

- `components/drivers/include/lsm6dso.hpp`
- `components/drivers/lsm6dso.cpp`

Implementation process:

1. Treat the BOM part `LSM6DSOWTR` as the `LSM6DSO` register-compatible family member for software purposes.
2. Keep `IMU_CS` high to maintain I2C mode.
3. Probe address `0x6B` because `SDO/SA0` is tied to `SENSOR_3V3` in the netlist.
4. Read `WHO_AM_I` first and fail loudly if it does not match the expected value.
5. Enable register auto-increment and block data update.
6. Set initial accelerometer and gyro ranges conservatively, then tune after data review.
7. Configure interrupt routing to the pins already wired into the expander: `INT1` and `INT2`.
8. Expose both raw sample APIs and higher-level event hooks for motion states such as vertical shake, horizontal shake, tilt, or wake-up detection.
9. Prefer the device FIFO for burst reads if you want to reduce HP-core wake frequency.

Suggested firmware responsibilities:

1. Raw motion acquisition for UI and control logic.
2. Shake direction or activity classification.
3. Optional use of embedded functions if they reduce host processing enough to justify the configuration complexity.

## 7. VCNL4035X01 driver process

Suggested new files:

- `components/drivers/include/vcnl4035.hpp`
- `components/drivers/vcnl4035.cpp`

Implementation process:

1. Probe the default I2C address from the datasheet during bring-up and confirm against live hardware.
2. Configure proximity sensing, ambient light sensing, integration time, and measurement rate.
3. Configure the interrupt thresholds and clear behavior.
4. Bring up the external IRED path carefully because the design uses external emitters driven through the sensor-related nets.
5. Expose both raw ALS or proximity readings and higher-level gesture helper states.

Important design note:

1. The VCNL4035X01 is not a full turnkey gesture engine. Gesture behavior on this board will come from proximity timing and threshold logic combined with the external emitters and application code.
2. Treat `GES_INT` as a wake or event hint, then sample the sensor and classify the event in firmware.

## 8. MAX17055 driver process

Suggested new files:

- `components/drivers/include/max17055.hpp`
- `components/drivers/max17055.cpp`

Implementation process:

1. Probe the fixed MAX17055 address and verify identity or expected power-on state.
2. Implement register access for voltage, current, state of charge, temperature, status, and alert handling.
3. Read and clear alert state after the expander reports `ALARM`.
4. Persist any needed learned parameters only if they fit your product behavior; otherwise stick to the ModelGauge EZ workflow.
5. Expose a higher-level battery service API to the rest of the controller.

Critical constraint:

1. The MAX17055 can report battery state and estimate charge, but the current hardware does not provide the ESP32-C6 with a raw analog battery sense suitable for fully authoritative chemistry classification.
2. Current firmware uses a conservative 3-cell NiMH policy for the shipped AAA NiMH pack: keep `CHR_EN` off by default, classify from MAX17055 telemetry, and enable the TPS22917 charge-path switch when the pack is classified as NiMH. Actual charging still depends on external 5 V being present on the TPS22917 input.
3. If true chemistry detection is required later, add an ADC path to VBAT in a board revision.

## 9. SSD1351 DMA display driver process

Suggested new files:

- `components/drivers/include/ssd1351_panel.hpp`
- `components/drivers/ssd1351_panel.cpp`
- `components/drivers/include/display_service.hpp`
- `components/drivers/display_service.cpp`

Implementation process:

1. Use ESP-IDF `spi_bus_initialize` with DMA enabled.
2. Use `esp_lcd_panel_io_spi` for command and pixel transport if it fits the command model cleanly.
3. If `esp_lcd` does not fit the SSD1351 command flow cleanly enough, implement a thin custom panel layer on top of `spi_device_transmit` and DMA descriptors.
4. Drive the panel in 4-line SPI mode with the verified pins:
   `SCLK=GPIO20`, `MOSI=GPIO21`, `CS=GPIO19`, `DC=GPIO18`, `RES=GPIO14`, display rail enable `GPIO13`.
5. Follow the SSD1351 power-on and reset sequence from the datasheet before the first RAM write.
6. Use DMA-capable line buffers rather than full-frame flushes in ordinary UI updates.
7. Implement double-buffered line or tile rendering so the CPU prepares the next chunk while DMA pushes the current chunk.
8. Keep the panel in RGB565 internally unless there is a visual requirement to move to a wider format.
9. Add a display-service queue so UI code submits rectangles or scenes rather than writing panel commands directly.

Expected result:

1. The display no longer blocks the HP core for long pixel-copy loops.
2. UI updates are bounded and compatible with concurrent sensor and BLE work.

## 10. BLE discovery and binding process

Suggested new files:

- `components/connectivity/include/bind_service.hpp`
- `components/connectivity/bind_service.cpp`
- `components/connectivity/include/ble_transport.hpp`
- `components/connectivity/ble_transport.cpp`

Recommended transport:

1. Use NimBLE on ESP-IDF.
2. Advertise the controller with a compact manufacturer payload containing device ID, firmware version, and bind state.
3. Discover the main board using a dedicated custom GATT service UUID.
4. Store the bound peer ID and link metadata in NVS.

Recommended bind sequence:

1. Remote boots with filesystem-image config.
2. Remote scans for the main board service UUID.
3. If a known or approved main board is found, the remote opens a BLE connection.
4. Main board sends a signed or hashed manifest header first.
5. Remote validates version, source priority, and authenticity.
6. Remote receives the configuration blob over chunked BLE characteristics or an advertised local HTTP endpoint.
7. Remote stores the result in NVS or SPIFFS and reboots or reconfigures services in place.

Recommended GATT characteristics:

1. `device_info` read
2. `bind_request` write
3. `bind_status` notify
4. `config_manifest` read or notify
5. `config_chunk` read or notify
6. `command` write for explicit pull, reboot, or sync actions

Security requirements:

1. Pairing should not blindly trust any broadcaster with the correct UUID.
2. At minimum, use a challenge-response exchange bound to a shared secret or signed manifest.
3. Store long-lived trust anchors in NVS, not in SPIFFS.

## 11. Config-source priority and repo sync process

The requested runtime priority is:

1. Pull from main board
2. Pull from remote repo
3. User manual upload filesystem image

Recommended resolution logic:

1. Boot with the filesystem image manifest as the seed configuration.
2. If a bound main board is discovered, request its manifest and compare version or hash.
3. If no usable main board data is available, connect using stored Wi-Fi credentials.
4. Download the remote manifest over HTTPS.
5. If neither source is available, keep the filesystem-image manifest and open the provisioning or maintenance path.

Remote repo requirements:

1. Use HTTPS only.
2. Support private repos by reading tokens from NVS.
3. Keep the manifest small and signed or at least hashed.
4. Separate config, assets, and firmware metadata so small updates do not require reflashing firmware.

## 12. OTA and filesystem-update process

Suggested new files:

- `components/connectivity/include/update_server.hpp`
- `components/connectivity/update_server.cpp`
- `components/connectivity/include/manifest_verifier.hpp`
- `components/connectivity/manifest_verifier.cpp`

Replace the current HTTP stubs with real handlers:

1. `POST /api/update/firmware`
2. `POST /api/update/filesystem`
3. `GET /api/update/status`

Firmware update process:

1. Authenticate the request.
2. Stream the uploaded image into the next OTA partition using `esp_ota_begin`, `esp_ota_write`, and `esp_ota_end`.
3. Verify the image header and full SHA-256 before marking the partition bootable.
4. Use rollback support so the device can recover from a bad image.

Filesystem image update process:

1. Stream the SPIFFS image to the `storage` partition.
2. Verify content length and SHA-256 while writing.
3. Unmount and remount the partition after success.
4. Reject uploads that do not match the manifest version or declared checksum.

Validation requirements:

1. Minimum acceptable validation is SHA-256 per artifact.
2. Better validation is a signed manifest using a built-in public key.
3. Validate the manifest first, then firmware image, then filesystem image, then asset files.

## 13. Manifest security process

Recommended manifest fields:

1. `version`
2. `firmware_url`
3. `firmware_sha256`
4. `filesystem_url`
5. `filesystem_sha256`
6. `assets` list with name, url, hash, and size
7. `signature`
8. `key_id`

Recommended validation flow:

1. Download the manifest.
2. Verify the signature using a compiled-in public key.
3. Verify monotonic version rules to prevent downgrade attacks unless explicitly allowed.
4. Verify each downloaded artifact against its hash before activation.

## 14. Low-power core process

Initial LP-core scope should be intentionally small.

Recommended first use:

1. Wake policy and sleep coordination.
2. Monitoring of simple wake-capable inputs such as `IO_INT` and `VBAT_MON` if supported by the chosen sleep mode.
3. Periodic low-duty background bookkeeping rather than full sensor ownership.

Avoid in the first pass:

1. Full I2C sensor polling from the LP core.
2. Display refresh logic on the LP core.
3. Any complex state machine that depends on services still running on the HP core.

## 15. Test and validation process

Bring-up sequence:

1. Validate rail enables for `SENSOR_EN`, `LCD_EN`, and LED rail control.
2. Validate I2C addressing and `WHO_AM_I` or equivalent identity reads.
3. Validate the expander interrupt path end-to-end.
4. Validate SSD1351 reset, init, and DMA flush.
5. Validate BLE advertising, binding, and configuration transfer.
6. Validate firmware OTA and SPIFFS image update separately.
7. Validate rollback on failed firmware image.

Bench tests to add early:

1. Repeated expander IRQ storm test while updating the display.
2. Concurrent BLE plus display DMA plus sensor reads.
3. Power-cycle and rollback behavior after interrupted updates.
4. Asset-manifest validation with corrupted hash or signature.

## 16. Datasheet and reference usage

Local reference archive:

- `docs/datasheets/TCA9534_ti.pdf`
- `docs/datasheets/VCNL4035X01_vishay.pdf`
- `docs/datasheets/SSD1351_solomon_via_crystalfontz.pdf`
- `docs/datasheets/ESP32-C6_Datasheet_espressif.pdf`
- `docs/datasheets/ESP32-C6_TRM_espressif.pdf`
- `docs/datasheets/ESP32-C6_HW_Guidelines_espressif.pdf`

Blocked or partial from this environment:

1. Official ST `LSM6DSO` PDF endpoint timed out repeatedly.
2. Official Analog Devices `MAX17055` datasheet and user-guide PDF endpoints timed out or reset repeatedly.
3. Local HTML wrapper pages were kept for reference:
   `docs/datasheets/LSM6DSO_for_LSM6DSOWTR_alldatasheet_wrapper.html`
   `docs/datasheets/MAX17055_alldatasheet_wrapper.html`

Re-run the local script when network conditions are better:

- `docs/datasheets/download_datasheets.ps1`

## 17. Immediate coding next steps

Suggested first concrete implementation tasks:

1. Split `driver_services.cpp` into real driver files and add the shared I2C bus.
2. Implement TCA9534 first so buttons and interrupt routing are stable.
3. Implement SSD1351 DMA next so UI bring-up and provisioning flows are visible.
4. Implement MAX17055 and LSM6DSO after the event and display path is stable.
5. Implement BLE bind and config exchange before full repo sync, because source priority requires it.
6. Replace update stubs only after manifest validation rules are finalized.