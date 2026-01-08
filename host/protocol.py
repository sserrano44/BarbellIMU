"""
Barbell IMU Protocol Definitions
Matches firmware/main/protocol.h
"""

import struct
from dataclasses import dataclass
from typing import Optional

# BLE UUIDs
# The bytes in protocol.h are arranged as:
# 0xBB, 0x01, 0x00, 0x0X, 0x00, 0x00, 0x10, 0x00,
# 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB
# UUID format: xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx
# Converting the bytes (reversing each segment for little-endian):
SERVICE_UUID = "000001bb-0000-0010-8000-00805f9b34fb"
STREAM_CHAR_UUID = "010001bb-0000-0010-8000-00805f9b34fb"
COMMAND_CHAR_UUID = "020001bb-0000-0010-8000-00805f9b34fb"
STATUS_CHAR_UUID = "030001bb-0000-0010-8000-00805f9b34fb"

# Command opcodes
CMD_START_STREAM = 0x01
CMD_STOP_STREAM = 0x02
CMD_CALIBRATE = 0x03
CMD_SET_CONFIG = 0x10

# Stream packet flags
FLAG_STREAMING = 0x01
FLAG_CALIBRATED = 0x02

# Packet size
STREAM_PACKET_SIZE = 20


@dataclass
class StreamPacket:
    """Decoded stream packet from firmware"""
    timestamp_us: int       # Microseconds since ESP32 boot
    ax_mg: int              # Acceleration X in milli-g
    ay_mg: int              # Acceleration Y in milli-g
    az_mg: int              # Acceleration Z in milli-g
    gx_dps16: int           # Gyro X in (deg/s) * 16
    gy_dps16: int           # Gyro Y in (deg/s) * 16
    gz_dps16: int           # Gyro Z in (deg/s) * 16
    seq: int                # Sequence counter (0-255)
    flags: int              # Status flags

    @property
    def is_streaming(self) -> bool:
        return bool(self.flags & FLAG_STREAMING)

    @property
    def is_calibrated(self) -> bool:
        return bool(self.flags & FLAG_CALIBRATED)

    @property
    def ax_g(self) -> float:
        """Acceleration X in g"""
        return self.ax_mg / 1000.0

    @property
    def ay_g(self) -> float:
        """Acceleration Y in g"""
        return self.ay_mg / 1000.0

    @property
    def az_g(self) -> float:
        """Acceleration Z in g"""
        return self.az_mg / 1000.0

    @property
    def gx_dps(self) -> float:
        """Gyro X in deg/s"""
        return self.gx_dps16 / 16.0

    @property
    def gy_dps(self) -> float:
        """Gyro Y in deg/s"""
        return self.gy_dps16 / 16.0

    @property
    def gz_dps(self) -> float:
        """Gyro Z in deg/s"""
        return self.gz_dps16 / 16.0

    @property
    def accel_magnitude_g(self) -> float:
        """Total acceleration magnitude in g"""
        import math
        return math.sqrt(self.ax_g**2 + self.ay_g**2 + self.az_g**2)

    @classmethod
    def from_bytes(cls, data: bytes) -> Optional["StreamPacket"]:
        """Decode packet from raw bytes"""
        if len(data) != STREAM_PACKET_SIZE:
            return None

        # Little-endian: uint32, 6x int16, uint8, uint8, uint16
        try:
            values = struct.unpack("<I6hBBH", data)
            return cls(
                timestamp_us=values[0],
                ax_mg=values[1],
                ay_mg=values[2],
                az_mg=values[3],
                gx_dps16=values[4],
                gy_dps16=values[5],
                gz_dps16=values[6],
                seq=values[7],
                flags=values[8],
            )
        except struct.error:
            return None


@dataclass
class StatusPayload:
    """Decoded status from firmware"""
    stream_rate_hz: int
    accel_fs: int           # 0=2g, 1=4g, 2=8g, 3=16g
    gyro_fs: int            # 0=250, 1=500, 2=1000, 3=2000 dps
    dlpf: int
    flags: int
    fw_version_major: int
    fw_version_minor: int

    @property
    def accel_range_g(self) -> int:
        return [2, 4, 8, 16][self.accel_fs]

    @property
    def gyro_range_dps(self) -> int:
        return [250, 500, 1000, 2000][self.gyro_fs]

    @property
    def fw_version(self) -> str:
        return f"{self.fw_version_major}.{self.fw_version_minor}"

    @classmethod
    def from_bytes(cls, data: bytes) -> Optional["StatusPayload"]:
        """Decode status from raw bytes"""
        if len(data) < 8:
            return None

        try:
            values = struct.unpack("<HBBBBBB", data[:8])
            return cls(
                stream_rate_hz=values[0],
                accel_fs=values[1],
                gyro_fs=values[2],
                dlpf=values[3],
                flags=values[4],
                fw_version_major=values[5],
                fw_version_minor=values[6],
            )
        except struct.error:
            return None


def make_start_stream_cmd(rate_hz: int) -> bytes:
    """Create START_STREAM command"""
    return struct.pack("<BH", CMD_START_STREAM, rate_hz)


def make_stop_stream_cmd() -> bytes:
    """Create STOP_STREAM command"""
    return bytes([CMD_STOP_STREAM])


def make_calibrate_cmd() -> bytes:
    """Create CALIBRATE command"""
    return bytes([CMD_CALIBRATE])
