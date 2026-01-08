#pragma once

#include <stdint.h>

// BLE Service and Characteristic UUIDs
// Custom 128-bit UUIDs for Barbell IMU service
// UUIDs in string form: 000001bb-0000-0010-8000-00805f9b34fb (service)
// NimBLE BLE_UUID128_INIT expects bytes in little-endian (reversed) order
#define BARBELL_IMU_SERVICE_UUID        0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, \
                                        0x10, 0x00, 0x00, 0x00, 0xBB, 0x01, 0x00, 0x00

#define BARBELL_IMU_STREAM_CHAR_UUID    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, \
                                        0x10, 0x00, 0x00, 0x00, 0xBB, 0x01, 0x00, 0x01

#define BARBELL_IMU_COMMAND_CHAR_UUID   0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, \
                                        0x10, 0x00, 0x00, 0x00, 0xBB, 0x01, 0x00, 0x02

#define BARBELL_IMU_STATUS_CHAR_UUID    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, \
                                        0x10, 0x00, 0x00, 0x00, 0xBB, 0x01, 0x00, 0x03

// Command opcodes
#define CMD_START_STREAM    0x01
#define CMD_STOP_STREAM     0x02
#define CMD_CALIBRATE       0x03
#define CMD_SET_CONFIG      0x10

// Stream packet flags
#define FLAG_STREAMING      (1 << 0)
#define FLAG_CALIBRATED     (1 << 1)

// Stream packet structure (20 bytes, little-endian)
// Offset  Field     Type     Units
// 0       t_us      uint32   microseconds since boot
// 4       ax        int16    milli-g
// 6       ay        int16    milli-g
// 8       az        int16    milli-g
// 10      gx        int16    (deg/s) * 16
// 12      gy        int16    (deg/s) * 16
// 14      gz        int16    (deg/s) * 16
// 16      seq       uint8    sequence counter
// 17      flags     uint8    status flags
// 18      reserved  uint16   reserved (0)

#define STREAM_PACKET_SIZE  20

typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;
    int16_t ax_mg;
    int16_t ay_mg;
    int16_t az_mg;
    int16_t gx_dps16;
    int16_t gy_dps16;
    int16_t gz_dps16;
    uint8_t seq;
    uint8_t flags;
    uint16_t reserved;
} stream_packet_t;

_Static_assert(sizeof(stream_packet_t) == STREAM_PACKET_SIZE, "Packet size mismatch");

// Status structure for status characteristic
typedef struct __attribute__((packed)) {
    uint16_t stream_rate_hz;
    uint8_t accel_fs;       // 0=2g, 1=4g, 2=8g, 3=16g
    uint8_t gyro_fs;        // 0=250, 1=500, 2=1000, 3=2000 dps
    uint8_t dlpf;
    uint8_t flags;
    uint8_t fw_version_major;
    uint8_t fw_version_minor;
} status_payload_t;

#define FW_VERSION_MAJOR    1
#define FW_VERSION_MINOR    0
