#!/usr/bin/env python3
"""
VBT Training Analysis Tool
Analyzes multiple training sessions to estimate 1RM and recommend training loads.
"""

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


# Minimum Velocity Thresholds by exercise (m/s at 1RM)
MVT = {
    "squat": 0.30,
    "bench": 0.17,
    "deadlift": 0.32,
    "default": 0.30,
}

# RIR velocity thresholds (m/s) - approximate
RIR_THRESHOLDS = [
    (0.50, "4+"),
    (0.40, "2-3"),
    (0.30, "1-2"),
    (0.00, "0-1"),
]


@dataclass
class Rep:
    """Single rep data from CSV."""
    rep_index: int
    rep_duration_s: float
    concentric_duration_s: float
    peak_velocity_m_s: float
    mean_velocity_m_s: float
    peak_accel_m_s2: float
    peak_gyro_dps: float
    velocity_loss_pct: float


@dataclass
class Session:
    """A single training session (one set)."""
    filepath: Path
    load_kg: float
    exercise: str
    reps: List[Rep]
    valid_reps: List[Rep] = field(default_factory=list)
    filtered_reasons: List[str] = field(default_factory=list)
    best_velocity: Optional[float] = None


@dataclass
class LoadVelocityProfile:
    """Linear regression results for load-velocity relationship."""
    slope: float
    intercept: float
    r_squared: float
    data_points: List[Tuple[float, float]]  # (load, velocity) pairs


@dataclass
class AnalysisResult:
    """Complete analysis results."""
    exercise: str
    sessions: List[Session]
    profile: Optional[LoadVelocityProfile]
    e1rm_kg: Optional[float]
    mvt: float
    warnings: List[str] = field(default_factory=list)


def load_rep_csv(filepath: Path) -> List[Rep]:
    """Load reps from a CSV file."""
    reps = []
    with open(filepath, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                rep = Rep(
                    rep_index=int(row["rep_index"]),
                    rep_duration_s=float(row["rep_duration_s"]),
                    concentric_duration_s=float(row["concentric_duration_s"]),
                    peak_velocity_m_s=float(row["peak_velocity_m_s"]),
                    mean_velocity_m_s=float(row["mean_velocity_m_s"]),
                    peak_accel_m_s2=float(row["peak_accel_m_s2"]),
                    peak_gyro_dps=float(row["peak_gyro_dps"]),
                    velocity_loss_pct=float(row["velocity_loss_pct"]),
                )
                reps.append(rep)
            except (KeyError, ValueError) as e:
                print(f"Warning: Skipping malformed row in {filepath}: {e}", file=sys.stderr)
    return reps


def find_meta_json(reps_csv: Path) -> Optional[Path]:
    """Find the corresponding metadata JSON for a reps CSV file.

    reps_20260109_101616.csv -> imu_data_20260109_101616_meta.json
    """
    name = reps_csv.stem  # reps_20260109_101616
    if name.startswith("reps_"):
        timestamp = name[5:]  # 20260109_101616
        meta_name = f"imu_data_{timestamp}_meta.json"
        meta_path = reps_csv.parent / meta_name
        if meta_path.exists():
            return meta_path
    return None


def load_session(reps_csv: Path) -> Optional[Session]:
    """Load a session from reps CSV and its metadata JSON."""
    reps = load_rep_csv(reps_csv)
    if not reps:
        print(f"Warning: No reps found in {reps_csv}", file=sys.stderr)
        return None

    # Find and load metadata
    meta_path = find_meta_json(reps_csv)
    load_kg = 0.0
    exercise = "unknown"

    if meta_path:
        try:
            with open(meta_path) as f:
                meta = json.load(f)
                load_kg = meta.get("load_kg", 0.0)
                exercise = meta.get("exercise", "unknown")
        except (json.JSONDecodeError, IOError) as e:
            print(f"Warning: Could not load metadata from {meta_path}: {e}", file=sys.stderr)
    else:
        print(f"Warning: No metadata found for {reps_csv}", file=sys.stderr)

    return Session(
        filepath=reps_csv,
        load_kg=load_kg,
        exercise=exercise,
        reps=reps,
    )


def filter_reps(session: Session) -> None:
    """Filter outlier reps from a session. Modifies session in place."""
    valid = []
    reasons = []

    for i, rep in enumerate(session.reps):
        # Skip first rep if too long (gravity convergence issues)
        if i == 0 and rep.rep_duration_s > 5.0:
            reasons.append(f"rep {rep.rep_index}: first rep too long ({rep.rep_duration_s:.1f}s)")
            continue

        # Skip unrealistic velocities
        if rep.mean_velocity_m_s > 1.5:
            reasons.append(f"rep {rep.rep_index}: velocity too high ({rep.mean_velocity_m_s:.2f} m/s)")
            continue
        if rep.mean_velocity_m_s < 0.05:
            reasons.append(f"rep {rep.rep_index}: velocity too low ({rep.mean_velocity_m_s:.2f} m/s)")
            continue

        # Skip abnormal durations
        if rep.rep_duration_s > 10.0:
            reasons.append(f"rep {rep.rep_index}: duration too long ({rep.rep_duration_s:.1f}s)")
            continue
        if rep.rep_duration_s < 0.5:
            reasons.append(f"rep {rep.rep_index}: duration too short ({rep.rep_duration_s:.1f}s)")
            continue

        # Skip high-gyro reps (likely racking)
        if rep.peak_gyro_dps > 100:
            reasons.append(f"rep {rep.rep_index}: high gyro ({rep.peak_gyro_dps:.0f} dps, likely racking)")
            continue

        valid.append(rep)

    session.valid_reps = valid
    session.filtered_reasons = reasons


def get_representative_velocity(session: Session) -> Optional[float]:
    """Get best representative velocity for load-velocity profile.

    Uses: Best MCV from first 3 valid reps (freshest, least fatigued)
    """
    if not session.valid_reps:
        return None

    first_three = session.valid_reps[:3]
    best = max(r.mean_velocity_m_s for r in first_three)
    session.best_velocity = best
    return best


def linear_regression(x: List[float], y: List[float]) -> Tuple[float, float, float]:
    """Simple linear regression. Returns (slope, intercept, r_squared)."""
    n = len(x)
    if n < 2:
        return 0.0, 0.0, 0.0

    sum_x = sum(x)
    sum_y = sum(y)
    sum_xy = sum(xi * yi for xi, yi in zip(x, y))
    sum_x2 = sum(xi * xi for xi in x)
    sum_y2 = sum(yi * yi for yi in y)

    denom = n * sum_x2 - sum_x * sum_x
    if abs(denom) < 1e-10:
        return 0.0, sum_y / n, 0.0

    slope = (n * sum_xy - sum_x * sum_y) / denom
    intercept = (sum_y - slope * sum_x) / n

    # R-squared
    y_mean = sum_y / n
    ss_tot = sum((yi - y_mean) ** 2 for yi in y)
    ss_res = sum((yi - (slope * xi + intercept)) ** 2 for xi, yi in zip(x, y))

    r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

    return slope, intercept, r_squared


def compute_load_velocity_profile(sessions: List[Session]) -> Optional[LoadVelocityProfile]:
    """Compute linear regression of load vs best MCV per set."""
    data_points = []

    for session in sessions:
        if session.load_kg > 0 and session.best_velocity is not None:
            data_points.append((session.load_kg, session.best_velocity))

    if len(data_points) < 2:
        return None

    loads = [p[0] for p in data_points]
    velocities = [p[1] for p in data_points]

    slope, intercept, r_squared = linear_regression(loads, velocities)

    return LoadVelocityProfile(
        slope=slope,
        intercept=intercept,
        r_squared=r_squared,
        data_points=data_points,
    )


def estimate_1rm(profile: LoadVelocityProfile, mvt: float) -> Optional[float]:
    """Calculate e1RM where velocity = MVT.

    velocity = slope * load + intercept
    mvt = slope * e1rm + intercept
    e1rm = (mvt - intercept) / slope
    """
    if abs(profile.slope) < 1e-10:
        return None

    e1rm = (mvt - profile.intercept) / profile.slope
    return e1rm if e1rm > 0 else None


def estimate_rir(velocity: float) -> str:
    """Estimate Reps in Reserve from velocity."""
    for threshold, rir in RIR_THRESHOLDS:
        if velocity >= threshold:
            return rir
    return "0"


def analyze_sessions(sessions: List[Session], exercise: Optional[str] = None) -> AnalysisResult:
    """Perform full analysis on a list of sessions."""
    warnings = []

    # Determine exercise type
    if exercise:
        ex = exercise
    else:
        exercises = set(s.exercise for s in sessions if s.exercise != "unknown")
        if len(exercises) == 1:
            ex = exercises.pop()
        elif len(exercises) > 1:
            warnings.append(f"Mixed exercises found: {exercises}. Using first: {list(exercises)[0]}")
            ex = list(exercises)[0]
        else:
            ex = "unknown"
            warnings.append("No exercise type found in metadata. Using default MVT.")

    mvt = MVT.get(ex, MVT["default"])

    # Filter reps in each session
    for session in sessions:
        filter_reps(session)
        get_representative_velocity(session)

    # Check for sessions with too few valid reps
    valid_sessions = []
    for session in sessions:
        if len(session.valid_reps) < 3:
            warnings.append(f"{session.filepath.name}: Only {len(session.valid_reps)} valid reps (need 3+)")
        else:
            valid_sessions.append(session)

    # Check for missing load data
    sessions_with_load = [s for s in valid_sessions if s.load_kg > 0]
    if len(sessions_with_load) < len(valid_sessions):
        warnings.append("Some sessions missing load_kg in metadata")

    # Compute load-velocity profile
    profile = compute_load_velocity_profile(sessions_with_load)

    e1rm = None
    if profile:
        # Quality checks
        if profile.slope >= 0:
            warnings.append("Positive slope: velocity should decrease with load. Data may be unreliable.")
        if profile.r_squared < 0.7:
            warnings.append(f"Low R² ({profile.r_squared:.2f}): poor linear fit. Consider more data points.")

        # Estimate 1RM
        e1rm = estimate_1rm(profile, mvt)

        if e1rm:
            max_tested = max(s.load_kg for s in sessions_with_load)
            if e1rm > max_tested * 1.5:
                warnings.append(f"e1RM ({e1rm:.0f}kg) is >1.5x heaviest tested load ({max_tested:.0f}kg). May be overestimated.")
            if e1rm < max_tested:
                warnings.append(f"e1RM ({e1rm:.0f}kg) is less than tested load ({max_tested:.0f}kg). Data may be unreliable.")
    else:
        warnings.append("Could not compute load-velocity profile. Need at least 2 sessions with load data.")

    return AnalysisResult(
        exercise=ex,
        sessions=sessions,
        profile=profile,
        e1rm_kg=e1rm,
        mvt=mvt,
        warnings=warnings,
    )


def format_report(result: AnalysisResult) -> str:
    """Format analysis results as a text report."""
    lines = []
    lines.append("=" * 50)
    lines.append("VBT Training Analysis Report")
    lines.append("=" * 50)
    lines.append("")
    lines.append(f"Exercise: {result.exercise}")
    lines.append(f"Sessions analyzed: {len(result.sessions)}")
    lines.append("")

    # Data Quality
    lines.append("--- Data Quality ---")
    for session in result.sessions:
        total = len(session.reps)
        valid = len(session.valid_reps)
        filtered = total - valid
        line = f"  {session.filepath.name} ({session.load_kg:.0f}kg): {total} reps, {valid} valid"
        if filtered > 0:
            line += f" ({filtered} filtered)"
        lines.append(line)
        for reason in session.filtered_reasons:
            lines.append(f"    - {reason}")
    lines.append("")

    # Load-Velocity Profile
    lines.append("--- Load-Velocity Profile ---")
    if result.profile:
        lines.append("Data points:")
        for load, vel in sorted(result.profile.data_points):
            lines.append(f"  {load:5.0f} kg -> {vel:.3f} m/s")
        lines.append("")
        lines.append(f"Linear fit: MCV = {result.profile.slope:.5f} * load + {result.profile.intercept:.3f}")

        r2_quality = "excellent" if result.profile.r_squared >= 0.9 else \
                     "good" if result.profile.r_squared >= 0.7 else "poor"
        lines.append(f"R² = {result.profile.r_squared:.2f} ({r2_quality} fit)")
    else:
        lines.append("  Insufficient data for profile")
    lines.append("")

    # Estimated 1RM
    lines.append("--- Estimated 1RM ---")
    if result.e1rm_kg:
        lines.append(f"e1RM: {result.e1rm_kg:.0f} kg (at MVT = {result.mvt:.2f} m/s for {result.exercise})")
    else:
        lines.append("  Could not estimate 1RM")
    lines.append("")

    # Training Zone Recommendations
    if result.e1rm_kg and result.profile:
        lines.append("--- Training Zone Recommendations ---")
        lines.append(f"{'Zone':<12} | {'%1RM':>5} | {'Load':>8} | {'Target MCV':<15}")
        lines.append("-" * 50)

        zones = [
            ("Strength", 0.85, 0.30, 0.40),
            ("Hypertrophy", 0.70, 0.40, 0.55),
            ("Power", 0.50, 0.55, 0.75),
        ]

        for zone_name, pct, vel_low, vel_high in zones:
            load = result.e1rm_kg * pct
            lines.append(f"{zone_name:<12} | {pct*100:4.0f}% | {load:6.0f} kg | {vel_low:.2f}-{vel_high:.2f} m/s")
        lines.append("")

    # Per-Set Fatigue Analysis
    lines.append("--- Per-Set Fatigue Analysis ---")
    for session in result.sessions:
        if len(session.valid_reps) < 2:
            lines.append(f"  {session.filepath.name}: Not enough valid reps for fatigue analysis")
            continue

        first_vel = session.valid_reps[0].mean_velocity_m_s
        last_vel = session.valid_reps[-1].mean_velocity_m_s
        loss_pct = (first_vel - last_vel) / first_vel * 100 if first_vel > 0 else 0

        first_rir = estimate_rir(first_vel)
        last_rir = estimate_rir(last_vel)

        lines.append(f"  {session.filepath.name} ({session.load_kg:.0f}kg):")
        lines.append(f"    Started {first_vel:.2f} m/s (~{first_rir} RIR), ended {last_vel:.2f} m/s (~{last_rir} RIR)")
        lines.append(f"    Velocity loss: {loss_pct:.0f}%")

        if loss_pct < 20:
            assessment = "Moderate fatigue, could push harder"
        elif loss_pct < 35:
            assessment = "Good hypertrophy stimulus"
        else:
            assessment = "High fatigue, near failure"
        lines.append(f"    -> {assessment}")
    lines.append("")

    # Warnings
    if result.warnings:
        lines.append("--- Warnings ---")
        for warning in result.warnings:
            lines.append(f"  ! {warning}")
        lines.append("")

    # Summary
    if result.e1rm_kg:
        lines.append("--- Summary Recommendations ---")
        lines.append(f"Estimated 1RM: ~{result.e1rm_kg:.0f} kg for {result.exercise}")
        lines.append("")
        lines.append("For hypertrophy training:")
        hyp_low = result.e1rm_kg * 0.65
        hyp_high = result.e1rm_kg * 0.80
        lines.append(f"  - Optimal load: {hyp_low:.0f}-{hyp_high:.0f} kg (65-80% 1RM)")
        lines.append("  - Target rep range: 6-12 reps")
        lines.append("  - Stop set when velocity drops below 0.25 m/s")
        lines.append("  - Target velocity loss: 20-35% per set")

    return "\n".join(lines)


def format_json(result: AnalysisResult) -> str:
    """Format analysis results as JSON."""
    data = {
        "exercise": result.exercise,
        "sessions_analyzed": len(result.sessions),
        "mvt": result.mvt,
        "e1rm_kg": result.e1rm_kg,
        "profile": None,
        "sessions": [],
        "warnings": result.warnings,
    }

    if result.profile:
        data["profile"] = {
            "slope": result.profile.slope,
            "intercept": result.profile.intercept,
            "r_squared": result.profile.r_squared,
            "data_points": [{"load_kg": l, "velocity_m_s": v} for l, v in result.profile.data_points],
        }

    for session in result.sessions:
        session_data = {
            "file": session.filepath.name,
            "load_kg": session.load_kg,
            "total_reps": len(session.reps),
            "valid_reps": len(session.valid_reps),
            "best_velocity": session.best_velocity,
            "filtered_reasons": session.filtered_reasons,
        }
        data["sessions"].append(session_data)

    return json.dumps(data, indent=2)


def main():
    parser = argparse.ArgumentParser(
        description="Analyze VBT training sessions to estimate 1RM and recommend training loads.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s data/reps_*.csv
  %(prog)s --exercise squat data/reps_*.csv
  %(prog)s --json -o report.json data/reps_*.csv
        """,
    )
    parser.add_argument("files", nargs="+", type=Path, help="Rep CSV files to analyze")
    parser.add_argument(
        "--exercise", "-e",
        choices=["squat", "bench", "deadlift"],
        help="Exercise type (overrides metadata)",
    )
    parser.add_argument(
        "--output", "-o",
        type=Path,
        help="Output file (default: stdout)",
    )
    parser.add_argument(
        "--json", "-j",
        action="store_true",
        help="Output as JSON instead of text report",
    )

    args = parser.parse_args()

    # Load all sessions
    sessions = []
    for filepath in args.files:
        if not filepath.exists():
            print(f"Warning: File not found: {filepath}", file=sys.stderr)
            continue
        session = load_session(filepath)
        if session:
            sessions.append(session)

    if not sessions:
        print("Error: No valid sessions to analyze", file=sys.stderr)
        sys.exit(1)

    # Analyze
    result = analyze_sessions(sessions, args.exercise)

    # Format output
    if args.json:
        output = format_json(result)
    else:
        output = format_report(result)

    # Write output
    if args.output:
        with open(args.output, "w") as f:
            f.write(output)
            f.write("\n")
        print(f"Report written to {args.output}")
    else:
        print(output)


if __name__ == "__main__":
    main()
