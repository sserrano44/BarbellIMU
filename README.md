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

# Help
python barbell_imu.py --help
```

### CLI Options

- `-r, --rate HZ`: Stream rate 50-200 Hz (default: 100)
- `-o, --output DIR`: Output directory for logs (default: ./data)
- `-c, --calibrate`: Run gyro calibration before streaming
- `-s, --show-samples`: Show live acceleration values
- `-d, --device ADDR`: Skip scanning, connect to specific address

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

CSV files with columns:
```
timestamp_us,ax_mg,ay_mg,az_mg,gx_dps16,gy_dps16,gz_dps16,seq,flags
```

Metadata JSON includes recording timestamp, firmware version, sensor ranges, and axis convention notes.
