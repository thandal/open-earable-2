# OpenEarable v2 Firmware — System Description

This is a **Zephyr RTOS** firmware project for the **nRF5340** dual-core SoC, built with the **nRF Connect SDK (v3.0.1)** and CMake. It's an open-source ear-worn sensing platform with BLE LE Audio, multiple sensors, and SD card logging.

## Top-Level Layout

| Path | Purpose |
|------|---------|
| `src/` | Main source code (~700 files) |
| `src/main.cpp` | **Application entry point** — initializes power, USB, audio, sensors, BLE services |
| `boards/teco/openearable_v2/` | Board definitions, device trees, pin configs |
| `dts/bindings/` | Custom device tree bindings (fuel gauge, codec, etc.) |
| `include/` | Shared headers (notably `zbus_common.h` for the message bus) |
| `tools/` | Flash scripts, UART terminal utilities |
| `doc/` | RST documentation (building, FOTA, architecture) |

## Key Source Modules (`src/`)

- **`audio/`** — LE Audio streaming pipeline: I2S, PDM mic, LC3 codec, audio datapath, equalization, SD card playback
- **`bluetooth/`** — BLE stack: unicast/broadcast streams, GATT services (sensor, battery, LED, button), volume control, advertising, DFU
- **`SensorManager/`** — Hardware sensor abstraction for 5 sensors: BMA580 (accel), BMX160 (IMU), BMP388 (baro/temp), MAXM86161 (PPG), MLX90632 (IR thermometer)
- **`Battery/`** — Power management with TI BQ25120A (charger) and BQ27220 (fuel gauge) drivers
- **`SD_Card/`** — FAT/exFAT filesystem, high-speed logging in `.oe` binary format
- **`drivers/`** — Hardware drivers: Cirrus ADAU1860 codec, I2S, USB audio, LED controller, DSP filters
- **`Wire/`** — Arduino-like I2C/TWI abstraction
- **`buttons/`** — Button input state machine
- **`time_sync/`** — Multi-device time synchronization
- **`ParseInfo/`** — Dynamic sensor configuration schemas
- **`utils/`** — LED state indicator, UICR access, board version detection

## Inter-Module Communication

Modules are decoupled via **Zephyr ZBUS** — a publish/subscribe message bus. Key message types include `button_msg`, `le_audio_msg`, `sensor_msg`, `battery_data`, `volume_msg`, and `sd_msg`.

## Configuration Files

- **`prj.conf`** — Default build with full logging, BLE, sensors, MCUboot FOTA support
- **`prj_release.conf`** — Production build (no debug, minimal logging)
- **`Kconfig` / `Kconfig.defaults`** — Kernel-style config system
- **`*.overlay`** files — Device tree overlays per build variant

## Data Flow

- **Audio:** BLE LE Audio ISO → LC3 decode → audio datapath → I2S → ADAU1860 codec (and reverse for mic capture)
- **Sensors:** I2C hardware → SensorManager → ZBUS → BLE GATT notifications + SD card logging
- **Power:** BQ25120A charger → BQ27220 fuel gauge → PowerManager → ZBUS → LED status + BLE notifications

## Build & Flash

Built with `west build` (Zephyr's meta-tool). Flash scripts in `tools/flash/` support J-Link, FOTA, and MCU Manager upload. The `west.yml` manifest pins the nRF Connect SDK version.

## Hardware

- **SoC:** nRF5340 dual-core ARM Cortex-M33 (Application + Network cores)
- **Audio Codec:** Analog Devices ADAU1860
- **Sensors:** BMA580, BMX160, BMP388, MAXM86161, MLX90632
- **Battery:** TI BQ25120A (charger) + BQ27220 (fuel gauge)
- **LED Controller:** KTD2026 (3-channel RGB)
- **Interfaces:** I2S, I2C/TWI, SPI, USB, ADC, GPIO, PWM, UART
- **Storage:** SD card (FAT/exFAT)
- **Bootloader:** MCUboot

## I2C Bus Topology

| Bus | Address | Role | Speed | Devices |
|-----|---------|------|-------|---------|
| I2C1 | i2c@9000 | Power | 400 kHz | KTD2026 (0x30), BQ27220 (0x55), BQ25120A (0x6A) |
| I2C2 | i2c@b000 | Audio | 400 kHz | MAXM86161 (0x62), ADAU1860 (0x64) |
| I2C3 | i2c@c000 | Sensors | 1 MHz (FM+) | BMA580 (0x18), BMX160 (0x68), BMP388 (0x76), MLX90632 (0x3A) |

## Optimization Opportunities (from datasheet review)

- **BMP388:** Running with no oversampling and no IIR filter. The datasheet recommends at least x4 pressure oversampling + IIR coeff 3 for indoor navigation (5 cm altitude noise vs 55 cm with current defaults). See BMP388 datasheet Table "Recommended Filter Settings."
- **BMX160:** Has a 1024-byte hardware FIFO that is not utilized. At higher ODRs, the current polling approach risks data loss. Enabling header-mode FIFO with watermark interrupt would reduce I2C transactions and CPU wakeups.
- **MLX90632:** Calibration constants are global `double` variables (104 bytes). These should be `float` class members — the Cortex-M33 FPU is single-precision only, so `double` math is emulated in software.
