# Barbell IMU Sensor MVP

ESP32 + MPU-6050 motion sensor for velocity-based training data collection.

## Hardware

- ESP32 development board (BLE capable)
- GY-521 breakout (MPU-6050, 6-DoF)
- I2C connections: SDA=GPIO21, SCL=GPIO22

## Firmware Build

```bash
# Initialize ESP-IDF environment
. ~/esp/esp-idf/export.sh

# Build
cd firmware
idf.py build

# Flash
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor
```

## Python Host Tool

```bash
cd host
pip install -r requirements.txt

# Run (scans for device automatically)
python barbell_imu.py

# With options
python barbell_imu.py --rate 100 --calibrate --show-samples

# VBT mode for real-time velocity tracking
python barbell_imu.py --vbt --exercise squat --load-kg 100

# Help
python barbell_imu.py --help
```

### CLI Options

**Basic options:**
- `-r, --rate HZ`: Stream rate 50-200 Hz (default: 100)
- `-o, --output DIR`: Output directory for logs (default: ./data)
- `-c, --calibrate`: Run gyro calibration before streaming
- `-s, --show-samples`: Show live acceleration values
- `-d, --device ADDR`: Skip scanning, connect to specific address

**VBT (Velocity-Based Training) options:**
- `--vbt`: Enable real-time velocity tracking and rep detection
- `--exercise {squat,bench,deadlift}`: Exercise type (metadata)
- `--load-kg KG`: Barbell load in kg (shown in output)
- `--loss-threshold PCT`: Warn when velocity loss exceeds threshold
- `--invert`: Invert velocity sign (if sensor mounted upside down)
- `--no-bell`: Disable terminal bell on rep completion

**VBT tuning parameters:**
- `--tau SEC`: Gravity filter time constant (default: 0.5)
- `--stationary-accel-g G`: Max accel deviation from 1g for rest detection (default: 0.05)
- `--stationary-gyro-dps DPS`: Max gyro for rest detection (default: 5.0)
- `--stationary-hold-ms MS`: Time to confirm stationary state (default: 250)
- `--move-confirm-ms MS`: Time to confirm movement started (default: 50)
- `--concentric-v-min M/S`: Min velocity for concentric phase (default: 0.05)

## BLE Protocol

### Service UUID
`000001bb-0000-0010-8000-00805f9b34fb`

### Characteristics
- Stream (notify): `010001bb-0000-0010-8000-00805f9b34fb`
- Command (write): `020001bb-0000-0010-8000-00805f9b34fb`
- Status (read): `030001bb-0000-0010-8000-00805f9b34fb`

### Commands
| Opcode | Name | Payload |
|--------|------|---------|
| 0x01 | START_STREAM | uint16 rate_hz |
| 0x02 | STOP_STREAM | none |
| 0x03 | CALIBRATE | none |

### Stream Packet (20 bytes)
| Offset | Field | Type | Units |
|--------|-------|------|-------|
| 0 | timestamp | uint32 | microseconds |
| 4 | ax | int16 | milli-g |
| 6 | ay | int16 | milli-g |
| 8 | az | int16 | milli-g |
| 10 | gx | int16 | (deg/s)*16 |
| 12 | gy | int16 | (deg/s)*16 |
| 14 | gz | int16 | (deg/s)*16 |
| 16 | seq | uint8 | counter |
| 17 | flags | uint8 | bit0=streaming, bit1=calibrated |
| 18 | reserved | uint16 | 0 |

## Output Files

**Raw IMU data** (`imu_data_<timestamp>.csv`):
```
timestamp_us,ax_mg,ay_mg,az_mg,gx_dps16,gy_dps16,gz_dps16,seq,flags
```

**Per-rep metrics** (`reps_<timestamp>.csv`, VBT mode only):
```
rep_index,rep_duration_s,concentric_duration_s,peak_velocity_m_s,mean_velocity_m_s,peak_accel_m_s2,peak_gyro_dps,velocity_loss_pct
```

**Metadata JSON** includes recording timestamp, firmware version, sensor ranges, exercise type, and load.

## VBT Mode

When `--vbt` is enabled, the tool computes real-time velocity-based training metrics:

- **Live dashboard**: Current velocity, rep count, stationary/moving state
- **Rep detection**: Automatically segments reps based on rest periods
- **Per-rep metrics**:
  - Mean Concentric Velocity (MCV): Time-weighted average during upward motion
  - Peak Concentric Velocity (PCV): Maximum velocity in concentric phase
  - Velocity Loss %: Fatigue indicator relative to best rep in set

Example output:
```
Rep 1 | MCV 0.52 m/s | PCV 0.68 m/s | Con 0.65s | Loss 0%
Rep 2 | MCV 0.48 m/s | PCV 0.63 m/s | Con 0.71s | Loss 8%
Rep 3 | MCV 0.41 m/s | PCV 0.55 m/s | Con 0.82s | Loss 21%
  WARNING: Velocity loss 21% exceeds threshold 20%
```
