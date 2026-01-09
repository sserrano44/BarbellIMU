"""
Velocity-Based Training (VBT) Processor for Barbell IMU

This module computes real-time VBT metrics from IMU stream data:
- Gravity estimation via low-pass filter
- Vertical velocity integration with ZUPT (Zero Velocity Update)
- Rep segmentation based on stationary/moving transitions
- Per-rep drift correction and concentric phase detection

Assumptions and Limitations:
- Assumes barbell moves primarily along the gravity vector (vertical lifts)
- Gravity filter needs time to converge (~0.5s default)
- ZUPT works best when barbell is truly stationary at rep boundaries
- Single-axis integration accumulates drift; corrected at rep end
- Works best for squat/bench/deadlift patterns with clear pause points
"""

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, List, Callable

from protocol import StreamPacket


# Physical constants
G_M_S2 = 9.80665  # Standard gravity in m/s^2


@dataclass
class RepMetrics:
    """Metrics for a completed rep"""
    rep_index: int
    rep_duration_s: float
    concentric_duration_s: float
    peak_concentric_velocity_m_s: float
    mean_concentric_velocity_m_s: float
    peak_accel_m_s2: float
    peak_gyro_dps: float
    velocity_loss_pct: float  # Loss relative to best rep in set


@dataclass
class VBTSample:
    """A single sample in the rep buffer for drift correction"""
    timestamp_us: int
    velocity_raw: float  # Uncorrected velocity estimate
    a_up: float  # Vertical acceleration for this sample


class RepState(Enum):
    """State machine for rep detection"""
    WAITING_FOR_STATIONARY = auto()  # Initial state, finding first rest
    STATIONARY = auto()              # Barbell at rest
    MAYBE_MOVING = auto()            # Potentially starting a rep
    MOVING = auto()                  # Rep in progress
    MAYBE_STATIONARY = auto()        # Potentially ending a rep


@dataclass
class VBTConfig:
    """Configuration for VBT processing"""
    # Gravity low-pass filter: alpha = 1 - exp(-dt/tau)
    # Default tau=0.5s at 100Hz gives alpha ~ 0.02
    alpha: Optional[float] = None  # If None, computed from tau
    tau_s: float = 0.5             # Time constant in seconds

    # ZUPT thresholds
    stationary_accel_g: float = 0.05   # Max deviation from 1g to be stationary
    stationary_gyro_dps: float = 5.0   # Max gyro magnitude to be stationary
    stationary_hold_ms: float = 250    # Time to confirm stationary state

    # Rep detection
    move_confirm_ms: float = 50        # Time to confirm movement started
    min_rep_duration_ms: float = 300   # Minimum rep duration to be valid

    # Concentric phase detection
    concentric_v_min: float = 0.05     # Minimum velocity for concentric phase (m/s)

    # Rep validation - reject reps that don't meet these thresholds
    min_rep_mcv: float = 0.1           # Minimum MCV to count as valid rep (m/s)
    min_concentric_duration_ms: float = 100  # Minimum concentric duration (ms)

    # Orientation
    invert: bool = False               # Invert velocity sign if IMU mounted inverted


class VBTProcessor:
    """
    Real-time VBT processor for IMU stream data.

    Accepts sequential StreamPackets and produces:
    - Real-time velocity estimates
    - Rep detection and metrics
    - Velocity loss tracking
    """

    def __init__(self, config: Optional[VBTConfig] = None,
                 on_rep_complete: Optional[Callable[[RepMetrics], None]] = None):
        """
        Initialize VBT processor.

        Args:
            config: VBT configuration parameters
            on_rep_complete: Callback when a rep completes
        """
        self.config = config or VBTConfig()
        self.on_rep_complete = on_rep_complete

        # State
        self._reset_state()

    def _reset_state(self):
        """Reset all processing state"""
        # Timing
        self.last_timestamp_us: Optional[int] = None

        # Gravity estimate (initially assume Z-up, 1g)
        # Will converge to actual orientation via low-pass filter
        self.g_est = [0.0, 0.0, G_M_S2]  # m/s^2
        self.gravity_initialized = False

        # Velocity integration
        self.velocity = 0.0  # Current vertical velocity estimate (m/s)

        # ZUPT state
        self.stationary_start_us: Optional[int] = None
        self.is_stationary = False

        # Rep detection state machine
        self.rep_state = RepState.WAITING_FOR_STATIONARY
        self.state_enter_us: Optional[int] = None

        # Current rep buffer
        self.rep_buffer: List[VBTSample] = []
        self.rep_start_us: Optional[int] = None

        # Rep tracking
        self.rep_count = 0
        self.completed_reps: List[RepMetrics] = []
        self.best_mean_velocity = 0.0  # Best MCV in current set

        # Peak tracking for current rep
        self.rep_peak_accel = 0.0
        self.rep_peak_gyro = 0.0

        # Last computed values for dashboard
        self.last_a_up = 0.0

    def reset(self):
        """Reset processor for new set"""
        self._reset_state()

    def _compute_alpha(self, dt_s: float) -> float:
        """
        Compute low-pass filter alpha from time constant.
        alpha = 1 - exp(-dt/tau)

        This ensures consistent filtering regardless of sample rate.
        """
        if self.config.alpha is not None:
            return self.config.alpha
        if self.config.tau_s <= 0:
            return 1.0  # No filtering
        return 1.0 - math.exp(-dt_s / self.config.tau_s)

    def _compute_dt_us(self, timestamp_us: int) -> int:
        """
        Compute time delta handling uint32 wrap.
        Timestamp wraps every ~71 minutes (2^32 microseconds).
        """
        if self.last_timestamp_us is None:
            return 0

        dt = timestamp_us - self.last_timestamp_us
        # Handle wrap: if dt is negative, add 2^32
        if dt < 0:
            dt += 2**32

        # Sanity check: if dt is huge (> 1 second), something is wrong
        # Treat as discontinuity
        if dt > 1_000_000:
            return 0

        return dt

    def _update_gravity_estimate(self, a_meas: List[float], alpha: float):
        """
        Update gravity estimate using low-pass filter.

        The gravity vector is estimated by low-pass filtering the
        accelerometer readings. Over time, dynamic accelerations
        average out and the filter converges to the gravity direction.

        To reduce drift, we normalize the result to exactly 1g magnitude.
        """
        if not self.gravity_initialized:
            # Initialize with first measurement
            self.g_est = a_meas.copy()
            self.gravity_initialized = True
            return

        # Low-pass filter: g_est = (1-alpha)*g_est + alpha*a_meas
        for i in range(3):
            self.g_est[i] = (1 - alpha) * self.g_est[i] + alpha * a_meas[i]

        # Normalize to exactly g magnitude to prevent drift
        mag = math.sqrt(sum(x*x for x in self.g_est))
        if mag > 0:
            scale = G_M_S2 / mag
            for i in range(3):
                self.g_est[i] *= scale

    def _check_stationary(self, accel_mag_g: float, gyro_mag_dps: float,
                          timestamp_us: int) -> bool:
        """
        Check if sensor is stationary using ZUPT criteria.

        Stationary when BOTH conditions met for stationary_hold_ms:
        1. Accel magnitude close to 1g: |mag - 1| < threshold
        2. Gyro magnitude small: mag < threshold
        """
        accel_ok = abs(accel_mag_g - 1.0) < self.config.stationary_accel_g
        gyro_ok = gyro_mag_dps < self.config.stationary_gyro_dps

        if accel_ok and gyro_ok:
            if self.stationary_start_us is None:
                self.stationary_start_us = timestamp_us

            # Check if we've been stationary long enough
            dt_ms = self._compute_dt_us(timestamp_us) / 1000.0
            if self.stationary_start_us is not None:
                hold_time = timestamp_us - self.stationary_start_us
                if hold_time < 0:
                    hold_time += 2**32
                if hold_time / 1000.0 >= self.config.stationary_hold_ms:
                    return True
        else:
            self.stationary_start_us = None

        return False

    def _process_rep_state_machine(self, is_currently_stationary: bool,
                                   timestamp_us: int):
        """
        State machine for rep detection.

        Transitions:
        WAITING_FOR_STATIONARY -> STATIONARY (when stationary detected)
        STATIONARY -> MAYBE_MOVING (when movement starts)
        MAYBE_MOVING -> MOVING (after move_confirm_ms)
        MAYBE_MOVING -> STATIONARY (if movement stops quickly)
        MOVING -> MAYBE_STATIONARY (when stationary starts)
        MAYBE_STATIONARY -> STATIONARY (after stationary_hold_ms) -> rep complete
        MAYBE_STATIONARY -> MOVING (if movement resumes)
        """
        prev_state = self.rep_state

        if self.rep_state == RepState.WAITING_FOR_STATIONARY:
            if is_currently_stationary:
                self.rep_state = RepState.STATIONARY
                self.state_enter_us = timestamp_us
                self.velocity = 0.0  # Reset velocity when first stationary

        elif self.rep_state == RepState.STATIONARY:
            if not is_currently_stationary:
                self.rep_state = RepState.MAYBE_MOVING
                self.state_enter_us = timestamp_us

        elif self.rep_state == RepState.MAYBE_MOVING:
            if is_currently_stationary:
                # False alarm, go back to stationary
                self.rep_state = RepState.STATIONARY
                self.state_enter_us = timestamp_us
                self.velocity = 0.0
            else:
                # Check if we've been moving long enough to confirm rep start
                if self.state_enter_us is not None:
                    hold_time = timestamp_us - self.state_enter_us
                    if hold_time < 0:
                        hold_time += 2**32
                    if hold_time / 1000.0 >= self.config.move_confirm_ms:
                        self.rep_state = RepState.MOVING
                        self.state_enter_us = timestamp_us
                        self._start_rep(timestamp_us)

        elif self.rep_state == RepState.MOVING:
            if is_currently_stationary:
                self.rep_state = RepState.MAYBE_STATIONARY
                self.state_enter_us = timestamp_us

        elif self.rep_state == RepState.MAYBE_STATIONARY:
            if not is_currently_stationary:
                # Movement resumed, back to moving
                self.rep_state = RepState.MOVING
                self.state_enter_us = timestamp_us
            else:
                # Check if we've been stationary long enough to confirm rep end
                if self.state_enter_us is not None:
                    hold_time = timestamp_us - self.state_enter_us
                    if hold_time < 0:
                        hold_time += 2**32
                    if hold_time / 1000.0 >= self.config.stationary_hold_ms:
                        self._end_rep(timestamp_us)
                        self.rep_state = RepState.STATIONARY
                        self.state_enter_us = timestamp_us
                        self.velocity = 0.0

    def _start_rep(self, timestamp_us: int):
        """Start tracking a new rep"""
        self.rep_buffer = []
        self.rep_start_us = timestamp_us
        self.rep_peak_accel = 0.0
        self.rep_peak_gyro = 0.0

    def _end_rep(self, timestamp_us: int):
        """
        Complete a rep and compute metrics.

        Applies per-rep drift correction by enforcing v_end = 0
        via linear ramp subtraction.
        """
        if self.rep_start_us is None or len(self.rep_buffer) < 2:
            return

        # Check minimum rep duration
        rep_duration_us = timestamp_us - self.rep_start_us
        if rep_duration_us < 0:
            rep_duration_us += 2**32
        rep_duration_s = rep_duration_us / 1_000_000.0

        if rep_duration_s * 1000 < self.config.min_rep_duration_ms:
            # Too short, not a valid rep
            self.rep_buffer = []
            return

        # Apply drift correction: subtract linear ramp to make v_end = 0
        # The last raw velocity in buffer represents drift
        if len(self.rep_buffer) > 0:
            drift_total = self.rep_buffer[-1].velocity_raw
            n_samples = len(self.rep_buffer)

            corrected_velocities = []
            for i, sample in enumerate(self.rep_buffer):
                # Linear ramp: drift at sample i = drift_total * (i / (n-1))
                if n_samples > 1:
                    drift_at_sample = drift_total * (i / (n_samples - 1))
                else:
                    drift_at_sample = 0
                v_corrected = sample.velocity_raw - drift_at_sample
                corrected_velocities.append(v_corrected)
        else:
            corrected_velocities = []

        # Find concentric phase: samples where v > concentric_v_min
        # Concentric = moving upward (positive velocity after orientation handling)
        concentric_samples = []
        concentric_durations = []

        prev_ts = None
        for i, v in enumerate(corrected_velocities):
            ts = self.rep_buffer[i].timestamp_us
            if v > self.config.concentric_v_min:
                concentric_samples.append(v)
                if prev_ts is not None:
                    dt = ts - prev_ts
                    if dt < 0:
                        dt += 2**32
                    concentric_durations.append(dt / 1_000_000.0)
            prev_ts = ts

        # Calculate metrics
        if len(concentric_samples) > 0:
            peak_v = max(concentric_samples)
            # Time-weighted average for MCV
            if len(concentric_durations) > 0 and sum(concentric_durations) > 0:
                # Weight each velocity by its duration
                weighted_sum = sum(v * dt for v, dt in
                                   zip(concentric_samples[1:], concentric_durations))
                total_time = sum(concentric_durations)
                mean_v = weighted_sum / total_time if total_time > 0 else peak_v
            else:
                mean_v = sum(concentric_samples) / len(concentric_samples)
            concentric_duration = sum(concentric_durations)
        else:
            # No clear concentric phase detected
            # Fall back to max positive velocity
            positive_v = [v for v in corrected_velocities if v > 0]
            peak_v = max(positive_v) if positive_v else 0.0
            mean_v = sum(positive_v) / len(positive_v) if positive_v else 0.0
            concentric_duration = 0.0

        # Validate rep: must have meaningful concentric velocity and duration
        # This filters out false positives from small movements/bumps
        if mean_v < self.config.min_rep_mcv:
            # Not a real rep - MCV too low
            self.rep_buffer = []
            return

        if concentric_duration * 1000 < self.config.min_concentric_duration_ms:
            # Not a real rep - concentric phase too short
            self.rep_buffer = []
            return

        # Update best MCV for velocity loss calculation
        self.rep_count += 1
        if mean_v > self.best_mean_velocity:
            self.best_mean_velocity = mean_v

        # Calculate velocity loss
        if self.best_mean_velocity > 0:
            velocity_loss = 100.0 * (1.0 - mean_v / self.best_mean_velocity)
        else:
            velocity_loss = 0.0

        # Create rep metrics
        metrics = RepMetrics(
            rep_index=self.rep_count,
            rep_duration_s=rep_duration_s,
            concentric_duration_s=concentric_duration,
            peak_concentric_velocity_m_s=peak_v,
            mean_concentric_velocity_m_s=mean_v,
            peak_accel_m_s2=self.rep_peak_accel,
            peak_gyro_dps=self.rep_peak_gyro,
            velocity_loss_pct=velocity_loss,
        )

        self.completed_reps.append(metrics)

        # Callback
        if self.on_rep_complete:
            self.on_rep_complete(metrics)

        # Clear buffer
        self.rep_buffer = []

    def process_packet(self, packet: StreamPacket) -> float:
        """
        Process a single stream packet.

        Args:
            packet: StreamPacket from IMU

        Returns:
            Current velocity estimate in m/s (positive = up)
        """
        timestamp_us = packet.timestamp_us

        # Compute dt
        dt_us = self._compute_dt_us(timestamp_us)
        dt_s = dt_us / 1_000_000.0
        self.last_timestamp_us = timestamp_us

        # Convert accelerometer to m/s^2
        # mg -> g -> m/s^2
        a_meas = [
            packet.ax_mg / 1000.0 * G_M_S2,
            packet.ay_mg / 1000.0 * G_M_S2,
            packet.az_mg / 1000.0 * G_M_S2,
        ]

        # Compute accelerometer and gyro magnitudes for ZUPT
        accel_mag_g = packet.accel_magnitude_g
        gyro_mag_dps = math.sqrt(
            packet.gx_dps**2 + packet.gy_dps**2 + packet.gz_dps**2
        )

        # Update gravity estimate
        if dt_s > 0:
            alpha = self._compute_alpha(dt_s)
            self._update_gravity_estimate(a_meas, alpha)

        # Compute linear acceleration (remove gravity)
        a_lin = [a_meas[i] - self.g_est[i] for i in range(3)]

        # Project onto gravity direction to get vertical/up acceleration
        # a_up = dot(a_lin, unit(g_est))
        g_mag = math.sqrt(sum(x*x for x in self.g_est))
        if g_mag > 0:
            g_unit = [x / g_mag for x in self.g_est]
            # Negative because gravity points down, we want "up" positive
            a_up = -sum(a_lin[i] * g_unit[i] for i in range(3))
        else:
            a_up = 0.0

        # Apply inversion if configured
        if self.config.invert:
            a_up = -a_up

        self.last_a_up = a_up

        # Check ZUPT
        is_currently_stationary = self._check_stationary(
            accel_mag_g, gyro_mag_dps, timestamp_us
        )

        # Integrate velocity
        if dt_s > 0:
            if is_currently_stationary:
                # ZUPT: reset velocity to zero
                self.velocity = 0.0
            else:
                self.velocity += a_up * dt_s

        # Update rep state machine
        self._process_rep_state_machine(is_currently_stationary, timestamp_us)

        # If in a rep, buffer the sample and track peaks
        if self.rep_state in (RepState.MOVING, RepState.MAYBE_STATIONARY):
            self.rep_buffer.append(VBTSample(
                timestamp_us=timestamp_us,
                velocity_raw=self.velocity,
                a_up=a_up,
            ))

            # Track peaks
            accel_mag_m_s2 = accel_mag_g * G_M_S2
            if accel_mag_m_s2 > self.rep_peak_accel:
                self.rep_peak_accel = accel_mag_m_s2
            if gyro_mag_dps > self.rep_peak_gyro:
                self.rep_peak_gyro = gyro_mag_dps

        self.is_stationary = is_currently_stationary

        return self.velocity

    @property
    def current_velocity(self) -> float:
        """Get current velocity estimate in m/s"""
        return self.velocity

    @property
    def in_rep(self) -> bool:
        """Check if currently in a rep"""
        return self.rep_state in (RepState.MOVING, RepState.MAYBE_STATIONARY,
                                  RepState.MAYBE_MOVING)

    @property
    def last_rep(self) -> Optional[RepMetrics]:
        """Get the most recent completed rep"""
        return self.completed_reps[-1] if self.completed_reps else None
