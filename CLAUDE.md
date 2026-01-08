# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

BarbellIMU is an embedded IoT project for velocity-based resistance training using an MPU-6050 6-axis IMU on an ESP32-C3 with BLE connectivity. It has two components:
- **Firmware** (C/ESP-IDF): BLE peripheral running on ESP32-C3, streams IMU data
- **Host Tool** (Python): BLE client that connects, receives data, and logs to CSV

## Build Commands

### Firmware (ESP-IDF)
```bash
# Initialize ESP-IDF environment (required before idf.py commands)
. ~/esp/esp-idf/export.sh

cd firmware
idf.py build                      # Compile
idf.py -p /dev/ttyUSB0 flash      # Flash to device
idf.py -p /dev/ttyUSB0 monitor    # Serial monitor
idf.py -p /dev/ttyUSB0 flash monitor  # Flash and monitor in one command
```

### Host Tool (Python)
```bash
cd host
pip install -r requirements.txt
python barbell_imu.py             # Run with defaults (100Hz)
python barbell_imu.py -s -r 100   # Show live samples at 100Hz
python barbell_imu.py -c          # Run calibration before streaming
python barbell_imu.py -d AA:BB:CC:DD:EE:FF  # Connect to specific device

# VBT (Velocity-Based Training) mode
python barbell_imu.py --vbt                           # Enable VBT processing
python barbell_imu.py --vbt --exercise squat          # With exercise type
python barbell_imu.py --vbt --load-kg 100             # Track load
python barbell_imu.py --vbt --loss-threshold 20       # Warn if velocity drops >20%
python barbell_imu.py --vbt --invert                  # Flip velocity sign (inverted mount)
```

## Architecture

```
firmware/
├── main/
│   ├── main.c           # Entry point, initializes NVS, starts tasks
│   ├── ble_service.c/h  # NimBLE peripheral, handles commands/notifications
│   ├── imu_task.c/h     # FreeRTOS task for IMU sampling, calibration
│   └── protocol.h       # Binary packet format, UUIDs, opcodes
└── components/mpu6050/  # Custom I2C driver for MPU-6050

host/
├── barbell_imu.py       # Async BLE client using bleak library
├── protocol.py          # Mirror of firmware protocol definitions
└── vbt.py               # VBT processor: velocity integration, rep detection
```

**Key data flow**: IMU task samples MPU6050 → builds 20-byte packet → BLE service sends notification → Host receives via bleak → logs to CSV

## Protocol

The protocol is defined identically in `firmware/main/protocol.h` (C) and `host/protocol.py` (Python). Stream packets are 20 bytes with timestamp (µs), 3-axis accel (milli-g), 3-axis gyro (dps×16), sequence number, and flags.

## Hardware Configuration

- **Target**: ESP32-C3
- **I2C**: SDA=GPIO8, SCL=GPIO9
- **MPU6050 address**: 0x68
- **Stream rate**: 50-200 Hz (firmware-constrained)

## Key Constraints

- Timestamp is 32-bit microseconds, wraps every ~71 minutes
- Gyro calibration offsets are stored in RAM only (not persisted to flash)
- BLE packets fit default MTU (20 bytes payload + 3 bytes overhead = 23)
- Sequence number is 8-bit, wraps at 256 (used for packet loss detection)

## VBT Processing

The host tool supports real-time Velocity-Based Training metrics via `--vbt`:

**Algorithm overview:**
1. **Gravity estimation**: Low-pass filter on accelerometer (τ=0.5s default) to find "down"
2. **Linear acceleration**: Subtract gravity estimate from measured acceleration
3. **Vertical velocity**: Integrate acceleration projected onto gravity vector
4. **ZUPT**: Zero Velocity Update resets velocity when stationary (accel ~1g, low gyro)
5. **Rep detection**: State machine detects stationary→moving→stationary transitions
6. **Drift correction**: Linear ramp subtraction at rep end enforces v_end=0

**Outputs per rep:**
- Mean Concentric Velocity (MCV): time-weighted average during upward motion
- Peak Concentric Velocity (PCV): maximum velocity during concentric phase
- Concentric duration, total rep duration
- Velocity loss %: tracks fatigue relative to best rep in set

**Tuning parameters** (CLI flags):
- `--tau` / `--alpha`: Gravity filter responsiveness
- `--stationary-accel-g`, `--stationary-gyro-dps`: ZUPT sensitivity
- `--stationary-hold-ms`: Time to confirm rest (rep boundary detection)
- `--concentric-v-min`: Threshold for concentric phase (default 0.05 m/s)

**Output files:**
- `imu_data_<ts>.csv`: Raw IMU samples (unchanged)
- `reps_<ts>.csv`: Per-rep metrics (when VBT enabled)
