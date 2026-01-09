#!/usr/bin/env python3
"""
Barbell IMU Test Harness
Connects to the ESP32 BLE sensor and logs IMU data to disk.

Supports optional VBT (Velocity-Based Training) metrics for real-time
rep tracking with velocity loss monitoring.
"""

import argparse
import asyncio
import csv
import json
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, List

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice

from protocol import (
    SERVICE_UUID,
    STREAM_CHAR_UUID,
    COMMAND_CHAR_UUID,
    STATUS_CHAR_UUID,
    StreamPacket,
    StatusPayload,
    make_start_stream_cmd,
    make_stop_stream_cmd,
    make_calibrate_cmd,
    STREAM_PACKET_SIZE,
)
from vbt import VBTProcessor, VBTConfig, RepMetrics


# Queue size for packet buffering
# At 200Hz, 1000 packets = 5 seconds buffer
PACKET_QUEUE_SIZE = 1000


class Statistics:
    """Track stream statistics"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.packets_received = 0
        self.packets_lost = 0
        self.last_seq: Optional[int] = None
        self.start_time = time.time()
        self.last_report_time = time.time()
        self.last_report_packets = 0
        self.last_timestamp_us: Optional[int] = None
        self.dt_sum = 0
        self.dt_count = 0

    def update(self, packet: StreamPacket):
        self.packets_received += 1

        # Check sequence for packet loss
        if self.last_seq is not None:
            expected = (self.last_seq + 1) & 0xFF
            if packet.seq != expected:
                # Calculate lost packets (handle wrap)
                if packet.seq > expected:
                    lost = packet.seq - expected
                else:
                    lost = (256 - expected) + packet.seq
                self.packets_lost += lost
        self.last_seq = packet.seq

        # Track timing
        if self.last_timestamp_us is not None:
            dt = packet.timestamp_us - self.last_timestamp_us
            if dt > 0:  # Handle timestamp wrap
                self.dt_sum += dt
                self.dt_count += 1
        self.last_timestamp_us = packet.timestamp_us

    @property
    def elapsed(self) -> float:
        return time.time() - self.start_time

    @property
    def effective_rate(self) -> float:
        if self.elapsed > 0:
            return self.packets_received / self.elapsed
        return 0

    @property
    def loss_rate(self) -> float:
        total = self.packets_received + self.packets_lost
        if total > 0:
            return self.packets_lost / total * 100
        return 0

    @property
    def avg_dt_ms(self) -> float:
        if self.dt_count > 0:
            return (self.dt_sum / self.dt_count) / 1000.0
        return 0

    def get_interval_rate(self) -> float:
        """Get rate since last report"""
        now = time.time()
        dt = now - self.last_report_time
        if dt > 0:
            rate = (self.packets_received - self.last_report_packets) / dt
        else:
            rate = 0
        self.last_report_time = now
        self.last_report_packets = self.packets_received
        return rate


class BarbellIMU:
    """BLE client for Barbell IMU sensor with optional VBT processing"""

    def __init__(
        self,
        output_dir: Path,
        stream_rate: int = 100,
        show_samples: bool = False,
        vbt_enabled: bool = False,
        vbt_config: Optional[VBTConfig] = None,
        exercise: Optional[str] = None,
        load_kg: Optional[float] = None,
        loss_threshold: Optional[float] = None,
        bell_on_rep: bool = True,
    ):
        self.output_dir = output_dir
        self.stream_rate = stream_rate
        self.show_samples = show_samples
        self.vbt_enabled = vbt_enabled
        self.exercise = exercise
        self.load_kg = load_kg
        self.loss_threshold = loss_threshold
        self.bell_on_rep = bell_on_rep

        self.client: Optional[BleakClient] = None
        self.device: Optional[BLEDevice] = None
        self.status: Optional[StatusPayload] = None
        self.stats = Statistics()

        self.csv_file = None
        self.csv_writer = None
        self.reps_csv_file = None
        self.reps_csv_writer = None
        self.metadata = {}
        self.running = False
        self.latest_packet: Optional[StreamPacket] = None

        # Async queue for decoupling BLE callback from processing
        self.packet_queue: asyncio.Queue = asyncio.Queue(maxsize=PACKET_QUEUE_SIZE)
        self.consumer_task: Optional[asyncio.Task] = None

        # VBT processor
        self.vbt: Optional[VBTProcessor] = None
        if vbt_enabled:
            self.vbt = VBTProcessor(
                config=vbt_config,
                on_rep_complete=self._on_rep_complete,
            )

        # Dashboard state (updated by consumer, read by display loop)
        self.current_velocity = 0.0
        self.last_rep_metrics: Optional[RepMetrics] = None
        self.pending_rep_display: Optional[RepMetrics] = None  # For one-time display

    async def scan(self, timeout: float = 10.0) -> Optional[BLEDevice]:
        """Scan for BarbellIMU device"""
        print(f"Scanning for BarbellIMU device (timeout: {timeout}s)...")

        devices = await BleakScanner.discover(timeout=timeout, return_adv=True)

        for device, adv_data in devices.values():
            if device.name and "BarbellIMU" in device.name:
                print(f"Found: {device.name} ({device.address})")
                return device

            # Also check for service UUID in advertisement
            if adv_data.service_uuids:
                for uuid in adv_data.service_uuids:
                    if SERVICE_UUID.lower() in uuid.lower():
                        print(f"Found by UUID: {device.name} ({device.address})")
                        return device

        print("No BarbellIMU device found")
        return None

    async def connect(self, device: BLEDevice) -> bool:
        """Connect to device"""
        print(f"Connecting to {device.address}...")

        self.client = BleakClient(device)
        try:
            await self.client.connect()
            print("Connected!")

            # Discover services
            services = self.client.services
            for service in services:
                if SERVICE_UUID.lower() in service.uuid.lower():
                    print(f"Found BarbellIMU service: {service.uuid}")
                    break
            else:
                print("Warning: BarbellIMU service not found in discovered services")

            # Read status
            await self.read_status()

            self.device = device
            return True

        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    async def disconnect(self):
        """Disconnect from device"""
        if self.client and self.client.is_connected:
            try:
                await self.send_command(make_stop_stream_cmd())
            except Exception:
                pass
            await self.client.disconnect()
            print("Disconnected")

    async def read_status(self):
        """Read current status from device"""
        if not self.client:
            return

        try:
            data = await self.client.read_gatt_char(STATUS_CHAR_UUID)
            self.status = StatusPayload.from_bytes(data)
            if self.status:
                print(f"Firmware: v{self.status.fw_version}")
                print(f"Accel range: +/-{self.status.accel_range_g}g")
                print(f"Gyro range: +/-{self.status.gyro_range_dps} dps")
        except Exception as e:
            print(f"Failed to read status: {e}")

    async def send_command(self, cmd: bytes):
        """Send command to device"""
        if self.client:
            await self.client.write_gatt_char(COMMAND_CHAR_UUID, cmd)

    def _handle_notification(self, sender, data: bytearray):
        """
        Handle incoming stream notification.

        IMPORTANT: This runs in the BLE callback context. Keep it lightweight!
        Only parse the packet and push to queue. All processing happens in consumer.
        """
        packet = StreamPacket.from_bytes(bytes(data))
        if packet:
            try:
                # Non-blocking put; drop packet if queue is full
                self.packet_queue.put_nowait(packet)
            except asyncio.QueueFull:
                # Queue full, drop packet (will show in loss stats)
                pass

    async def _consumer_loop(self):
        """
        Async consumer task that processes packets from the queue.

        Handles:
        - Statistics updates
        - CSV writing
        - VBT processing
        """
        while self.running:
            try:
                # Wait for packet with timeout to allow graceful shutdown
                packet = await asyncio.wait_for(
                    self.packet_queue.get(),
                    timeout=0.1
                )

                # Update statistics
                self.stats.update(packet)
                self.latest_packet = packet

                # Write to CSV
                if self.csv_writer:
                    self.csv_writer.writerow([
                        packet.timestamp_us,
                        packet.ax_mg,
                        packet.ay_mg,
                        packet.az_mg,
                        packet.gx_dps16,
                        packet.gy_dps16,
                        packet.gz_dps16,
                        packet.seq,
                        packet.flags,
                    ])

                # VBT processing
                if self.vbt:
                    self.current_velocity = self.vbt.process_packet(packet)

            except asyncio.TimeoutError:
                # No packet received, continue loop to check running flag
                continue
            except asyncio.CancelledError:
                break

    def _on_rep_complete(self, metrics: RepMetrics):
        """
        Callback when VBT detects a completed rep.
        Called from consumer loop context.
        """
        self.last_rep_metrics = metrics
        self.pending_rep_display = metrics  # Flag for display

        # Write to reps CSV
        if self.reps_csv_writer:
            self.reps_csv_writer.writerow([
                metrics.rep_index,
                f"{metrics.rep_duration_s:.3f}",
                f"{metrics.concentric_duration_s:.3f}",
                f"{metrics.peak_concentric_velocity_m_s:.3f}",
                f"{metrics.mean_concentric_velocity_m_s:.3f}",
                f"{metrics.peak_accel_m_s2:.2f}",
                f"{metrics.peak_gyro_dps:.1f}",
                f"{metrics.velocity_loss_pct:.1f}",
            ])
            # Flush to ensure data is written
            if self.reps_csv_file:
                self.reps_csv_file.flush()

    async def start_streaming(self):
        """Start streaming data"""
        if not self.client:
            return

        # Setup output files
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = self.output_dir / f"imu_data_{timestamp}.csv"
        meta_path = self.output_dir / f"imu_data_{timestamp}_meta.json"

        # Create metadata
        self.metadata = {
            "recording_start": datetime.now().isoformat(),
            "device_address": self.device.address if self.device else "unknown",
            "stream_rate_hz": self.stream_rate,
            "firmware_version": self.status.fw_version if self.status else "unknown",
            "accel_range_g": self.status.accel_range_g if self.status else 16,
            "gyro_range_dps": self.status.gyro_range_dps if self.status else 2000,
            "vbt_enabled": self.vbt_enabled,
            "exercise": self.exercise,
            "load_kg": self.load_kg,
            "packet_format": {
                "timestamp_us": "uint32, microseconds since ESP32 boot",
                "ax_mg": "int16, milli-g",
                "ay_mg": "int16, milli-g",
                "az_mg": "int16, milli-g",
                "gx_dps16": "int16, (deg/s) * 16",
                "gy_dps16": "int16, (deg/s) * 16",
                "gz_dps16": "int16, (deg/s) * 16",
                "seq": "uint8, sequence counter",
                "flags": "uint8, bit0=streaming, bit1=calibrated",
            },
            "axis_convention": "TODO: Document mounting orientation",
        }

        # Open CSV file
        self.csv_file = open(csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "timestamp_us", "ax_mg", "ay_mg", "az_mg",
            "gx_dps16", "gy_dps16", "gz_dps16", "seq", "flags"
        ])

        # Open reps CSV if VBT enabled
        if self.vbt_enabled:
            reps_csv_path = self.output_dir / f"reps_{timestamp}.csv"
            self.reps_csv_file = open(reps_csv_path, "w", newline="")
            self.reps_csv_writer = csv.writer(self.reps_csv_file)
            self.reps_csv_writer.writerow([
                "rep_index", "rep_duration_s", "concentric_duration_s",
                "peak_velocity_m_s", "mean_velocity_m_s",
                "peak_accel_m_s2", "peak_gyro_dps", "velocity_loss_pct"
            ])
            print(f"Rep metrics: {reps_csv_path}")

        # Save metadata
        with open(meta_path, "w") as f:
            json.dump(self.metadata, f, indent=2)

        print(f"Logging to: {csv_path}")

        # Start consumer task
        self.running = True
        self.consumer_task = asyncio.create_task(self._consumer_loop())

        # Subscribe to notifications
        await self.client.start_notify(STREAM_CHAR_UUID, self._handle_notification)

        # Send start command
        self.stats.reset()
        await self.send_command(make_start_stream_cmd(self.stream_rate))
        print(f"Streaming started at {self.stream_rate} Hz")

        if self.vbt_enabled:
            print("\nVBT Mode Active")
            if self.exercise:
                print(f"Exercise: {self.exercise}")
            if self.load_kg:
                print(f"Load: {self.load_kg} kg")
            print("Place barbell at rest to begin tracking reps...")

    async def stop_streaming(self):
        """Stop streaming data"""
        self.running = False

        # Cancel consumer task
        if self.consumer_task:
            self.consumer_task.cancel()
            try:
                await self.consumer_task
            except asyncio.CancelledError:
                pass

        if self.client and self.client.is_connected:
            await self.send_command(make_stop_stream_cmd())
            await self.client.stop_notify(STREAM_CHAR_UUID)

        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

        if self.reps_csv_file:
            self.reps_csv_file.close()
            self.reps_csv_file = None
            self.reps_csv_writer = None

        print("Streaming stopped")

    async def calibrate(self):
        """Run calibration"""
        if not self.client:
            return

        print("Starting calibration - keep sensor stationary...")
        await self.send_command(make_calibrate_cmd())

        # Wait for calibration to complete
        await asyncio.sleep(2.0)

        # Check status
        await self.read_status()
        if self.status and (self.status.flags & 0x02):
            print("Calibration complete!")
        else:
            print("Calibration may still be in progress")

    def print_diagnostics(self):
        """Print current diagnostics and VBT dashboard"""
        rate = self.stats.get_interval_rate()
        loss = self.stats.loss_rate
        pkt = self.latest_packet

        # Check for pending rep display (print before status line)
        if self.pending_rep_display:
            rep = self.pending_rep_display
            self.pending_rep_display = None

            # Clear current line and print rep summary
            print("\r" + " " * 100 + "\r", end="")  # Clear line

            # Format rep summary
            rep_line = (
                f"Rep {rep.rep_index} | "
                f"MCV {rep.mean_concentric_velocity_m_s:.2f} m/s | "
                f"PCV {rep.peak_concentric_velocity_m_s:.2f} m/s | "
                f"Con {rep.concentric_duration_s:.2f}s | "
                f"Loss {rep.velocity_loss_pct:.0f}%"
            )

            # Add load if specified
            if self.load_kg:
                rep_line = f"[{self.load_kg}kg] " + rep_line

            print(rep_line)

            # Velocity loss warning
            if self.loss_threshold and rep.velocity_loss_pct > self.loss_threshold:
                print(f"  WARNING: Velocity loss {rep.velocity_loss_pct:.0f}% exceeds threshold {self.loss_threshold:.0f}%")

            # Terminal bell
            if self.bell_on_rep:
                print("\a", end="")

        # Build status line
        status_line = (
            f"\rPkts: {self.stats.packets_received:6d} | "
            f"Rate: {rate:5.1f} Hz | "
            f"Loss: {loss:4.1f}% | "
            f"Avg dt: {self.stats.avg_dt_ms:5.2f}ms"
        )

        if self.show_samples and pkt:
            status_line += (
                f" | Acc: [{pkt.ax_g:+5.2f},{pkt.ay_g:+5.2f},{pkt.az_g:+5.2f}]g"
                f" | Cal: {'Y' if pkt.is_calibrated else 'N'}"
            )

        # VBT dashboard line
        if self.vbt_enabled and self.vbt:
            # Show velocity and rep info
            v = self.current_velocity
            rep_count = self.vbt.rep_count
            state_char = "S" if self.vbt.is_stationary else "M"  # Stationary/Moving

            vbt_line = f" | V: {v:+5.2f} m/s [{state_char}] | Reps: {rep_count}"

            # Show last rep MCV/PCV if available
            if self.last_rep_metrics:
                last = self.last_rep_metrics
                vbt_line += f" | Last: {last.mean_concentric_velocity_m_s:.2f}/{last.peak_concentric_velocity_m_s:.2f}"
                if last.velocity_loss_pct > 0:
                    vbt_line += f" ({last.velocity_loss_pct:.0f}%)"

            status_line += vbt_line

        print(status_line, end="", flush=True)


async def main():
    parser = argparse.ArgumentParser(description="Barbell IMU Test Harness")

    # Existing options
    parser.add_argument(
        "-r", "--rate",
        type=int,
        default=100,
        help="Stream rate in Hz (default: 100, range: 50-200)"
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        default="./data",
        help="Output directory for log files (default: ./data)"
    )
    parser.add_argument(
        "-c", "--calibrate",
        action="store_true",
        help="Run calibration before streaming"
    )
    parser.add_argument(
        "-s", "--show-samples",
        action="store_true",
        help="Show latest sample values in diagnostics"
    )
    parser.add_argument(
        "-d", "--device",
        type=str,
        help="Device address to connect to (skip scanning)"
    )

    # VBT options
    parser.add_argument(
        "--vbt",
        action="store_true",
        help="Enable VBT (Velocity-Based Training) processing"
    )
    parser.add_argument(
        "--exercise",
        type=str,
        choices=["squat", "bench", "deadlift"],
        help="Exercise type (metadata only for now)"
    )
    parser.add_argument(
        "--alpha",
        type=float,
        help="Gravity low-pass filter alpha (overrides --tau)"
    )
    parser.add_argument(
        "--tau",
        type=float,
        default=0.5,
        help="Gravity filter time constant in seconds (default: 0.5)"
    )
    parser.add_argument(
        "--stationary-accel-g",
        type=float,
        default=0.05,
        help="Stationary detection: max accel deviation from 1g (default: 0.05)"
    )
    parser.add_argument(
        "--stationary-gyro-dps",
        type=float,
        default=5.0,
        help="Stationary detection: max gyro magnitude (default: 5.0)"
    )
    parser.add_argument(
        "--stationary-hold-ms",
        type=float,
        default=250,
        help="Time to confirm stationary state in ms (default: 250)"
    )
    parser.add_argument(
        "--move-confirm-ms",
        type=float,
        default=50,
        help="Time to confirm movement started in ms (default: 50)"
    )
    parser.add_argument(
        "--concentric-v-min",
        type=float,
        default=0.05,
        help="Minimum velocity for concentric phase in m/s (default: 0.05)"
    )
    parser.add_argument(
        "--invert",
        action="store_true",
        help="Invert velocity sign (if IMU mounted upside down)"
    )
    parser.add_argument(
        "--load-kg",
        type=float,
        help="Load in kg (metadata, shown in rep output)"
    )
    parser.add_argument(
        "--loss-threshold",
        type=float,
        help="Velocity loss %% threshold for warning (e.g., 20)"
    )
    parser.add_argument(
        "--no-bell",
        action="store_true",
        help="Disable terminal bell on rep completion"
    )
    parser.add_argument(
        "--max-rep-gyro",
        type=float,
        default=100.0,
        help="Maximum peak gyro (dps) to accept as valid rep - rejects racking (default: 100)"
    )
    parser.add_argument(
        "--gravity-warmup-s",
        type=float,
        default=1.5,
        help="Gravity convergence warmup time in seconds (default: 1.5)"
    )

    args = parser.parse_args()

    # Clamp rate
    rate = max(50, min(200, args.rate))
    if rate != args.rate:
        print(f"Rate clamped to {rate} Hz")

    # Setup output directory
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Build VBT config if enabled
    vbt_config = None
    if args.vbt:
        vbt_config = VBTConfig(
            alpha=args.alpha,
            tau_s=args.tau,
            gravity_convergence_time_s=args.gravity_warmup_s,
            stationary_accel_g=args.stationary_accel_g,
            stationary_gyro_dps=args.stationary_gyro_dps,
            stationary_hold_ms=args.stationary_hold_ms,
            move_confirm_ms=args.move_confirm_ms,
            concentric_v_min=args.concentric_v_min,
            max_rep_peak_gyro_dps=args.max_rep_gyro,
            invert=args.invert,
        )

    # Create client
    imu = BarbellIMU(
        output_dir=output_dir,
        stream_rate=rate,
        show_samples=args.show_samples,
        vbt_enabled=args.vbt,
        vbt_config=vbt_config,
        exercise=args.exercise,
        load_kg=args.load_kg,
        loss_threshold=args.loss_threshold,
        bell_on_rep=not args.no_bell,
    )

    # Handle Ctrl+C
    stop_event = asyncio.Event()

    def signal_handler():
        print("\nStopping...")
        stop_event.set()

    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, signal_handler)

    try:
        # Find device
        if args.device:
            from bleak.backends.device import BLEDevice
            device = BLEDevice(args.device, "BarbellIMU", {}, 0)
        else:
            device = await imu.scan()
            if not device:
                return 1

        # Connect
        if not await imu.connect(device):
            return 1

        # Calibrate if requested
        if args.calibrate:
            await imu.calibrate()

        # Start streaming
        await imu.start_streaming()

        # Diagnostics loop (~5Hz update for dashboard)
        print("\nPress Ctrl+C to stop\n")
        while not stop_event.is_set():
            imu.print_diagnostics()
            try:
                await asyncio.wait_for(stop_event.wait(), timeout=0.2)
            except asyncio.TimeoutError:
                pass

    except Exception as e:
        print(f"\nError: {e}")
        return 1

    finally:
        print()  # New line after status line
        await imu.stop_streaming()
        await imu.disconnect()

        # Print final stats
        print(f"\n--- Session Summary ---")
        print(f"Duration: {imu.stats.elapsed:.1f}s")
        print(f"Packets received: {imu.stats.packets_received}")
        print(f"Packets lost: {imu.stats.packets_lost}")
        print(f"Loss rate: {imu.stats.loss_rate:.2f}%")
        print(f"Effective rate: {imu.stats.effective_rate:.1f} Hz")

        # VBT summary
        if imu.vbt_enabled and imu.vbt and imu.vbt.rep_count > 0:
            print(f"\n--- VBT Summary ---")
            if imu.exercise:
                print(f"Exercise: {imu.exercise}")
            if imu.load_kg:
                print(f"Load: {imu.load_kg} kg")
            print(f"Total reps: {imu.vbt.rep_count}")
            print(f"Best MCV: {imu.vbt.best_mean_velocity:.2f} m/s")

            # Print all rep summaries
            print("\nRep Details:")
            print("-" * 70)
            print(f"{'Rep':>3} {'Duration':>8} {'Con Time':>8} {'MCV':>7} {'PCV':>7} {'Loss':>6}")
            print("-" * 70)
            for rep in imu.vbt.completed_reps:
                print(
                    f"{rep.rep_index:>3} "
                    f"{rep.rep_duration_s:>7.2f}s "
                    f"{rep.concentric_duration_s:>7.2f}s "
                    f"{rep.mean_concentric_velocity_m_s:>6.2f} "
                    f"{rep.peak_concentric_velocity_m_s:>6.2f} "
                    f"{rep.velocity_loss_pct:>5.0f}%"
                )

    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
