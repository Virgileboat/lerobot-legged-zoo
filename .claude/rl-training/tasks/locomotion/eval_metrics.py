#!/usr/bin/env python3
"""Compute detailed gait quality metrics from policy evaluation trajectories.

Called by evaluate_policy.py after each scenario rollout.
Not run standalone — imported and called via analyze_trajectory().
"""

import re
from pathlib import Path

import numpy as np


def parse_tier2_config(config_path):
    """Parse Tier 2 Normalization section from monitor_config.md.

    Returns dict of {metric: {"min": float, "max": float, "type": str, "weight": float}}
    """
    if config_path is None:
        return None
    text = Path(config_path).read_text() if isinstance(config_path, (str, Path)) else None
    if text is None:
        return None
    config = {}
    for line in text.splitlines():
        m = re.match(r"^- (\w+):\s*([\d.]+),\s*([\d.]+),\s*(\w+),\s*([\d.]+)", line)
        if m:
            config[m.group(1)] = {
                "min": float(m.group(2)),
                "max": float(m.group(3)),
                "type": m.group(4),
                "weight": float(m.group(5)),
            }
    return config if config else None


def compute_joint_jerk(joint_positions, dt):
    """Mean jerk (third derivative of position) across all joints.

    Lower jerk = smoother motion. Returns mean absolute jerk.
    joint_positions: np.array of shape (timesteps, num_joints)
    """
    if len(joint_positions) < 4:
        return None
    vel = np.diff(joint_positions, axis=0) / dt
    acc = np.diff(vel, axis=0) / dt
    jerk = np.diff(acc, axis=0) / dt
    return float(np.mean(np.abs(jerk)))


def compute_step_periodicity(contact_states, dt):
    """Dominant frequency strength from FFT on foot contact signals.

    contact_states: np.array of shape (timesteps, num_feet), binary (0/1)
    Returns strength of dominant frequency relative to total power (0-1).
    Higher = more periodic/rhythmic gait.
    """
    if len(contact_states) < 16:
        return None
    scores = []
    for foot_idx in range(contact_states.shape[1]):
        signal = contact_states[:, foot_idx].astype(float)
        signal = signal - signal.mean()
        fft = np.fft.rfft(signal)
        power = np.abs(fft) ** 2
        if power[1:].sum() < 1e-6:
            scores.append(0.0)
            continue
        dominant = power[1:].max()
        total = power[1:].sum()
        scores.append(float(dominant / total))
    return float(np.mean(scores))


def compute_stance_swing_ratio(contact_states):
    """Per-foot fraction of time spent in stance (on ground).

    Natural walking is ~60% stance, 40% swing.
    Returns list of ratios per foot.
    """
    if len(contact_states) < 2:
        return None
    ratios = []
    for foot_idx in range(contact_states.shape[1]):
        stance_frac = contact_states[:, foot_idx].mean()
        ratios.append(float(stance_frac))
    return ratios


def compute_phase_offset(contact_states):
    """Cross-correlation based phase offset between left and right foot.

    For bipeds: returns offset as fraction of gait cycle (0-1).
    0.5 = perfect alternation (walking), 0.0 = in-phase (hopping).
    """
    if contact_states.shape[1] < 2 or len(contact_states) < 16:
        return None
    left = contact_states[:, 0].astype(float) - contact_states[:, 0].mean()
    right = contact_states[:, 1].astype(float) - contact_states[:, 1].mean()
    corr = np.correlate(left, right, mode="full")
    mid = len(corr) // 2
    half = len(left) // 2
    corr_half = corr[mid : mid + half]
    if len(corr_half) == 0 or corr_half.max() < 1e-6:
        return None
    peak_idx = np.argmax(corr_half)
    fft_left = np.fft.rfft(contact_states[:, 0].astype(float))
    power = np.abs(fft_left) ** 2
    if power[1:].max() < 1e-6:
        return None
    dominant_freq_idx = np.argmax(power[1:]) + 1
    period_samples = len(contact_states) / dominant_freq_idx
    if period_samples < 1:
        return None
    offset = (peak_idx % period_samples) / period_samples
    return float(min(offset, 1.0 - offset) * 2)


def compute_grf_profile_score(contact_forces, contact_states):
    """Score how close vertical ground reaction forces match natural gait pattern.

    Natural walking has a double-hump M-shape in vertical GRF per stride.
    Returns 0-1 (1 = natural pattern).
    """
    if contact_forces is None or len(contact_forces) < 16:
        return None
    scores = []
    for foot_idx in range(contact_forces.shape[1]):
        force = contact_forces[:, foot_idx]
        contact = contact_states[:, foot_idx]
        stance_forces = force[contact > 0.5]
        if len(stance_forces) < 4:
            scores.append(0.0)
            continue
        mid = len(stance_forces) // 2
        first_half_max = stance_forces[:mid].max() if mid > 0 else 0
        second_half_max = stance_forces[mid:].max() if mid < len(stance_forces) else 0
        mid_val = stance_forces[mid] if mid < len(stance_forces) else 0
        peak_avg = (first_half_max + second_half_max) / 2
        if peak_avg < 1e-6:
            scores.append(0.0)
            continue
        dip_ratio = 1.0 - (mid_val / peak_avg)
        scores.append(float(max(0.0, min(1.0, dip_ratio * 2))))
    return float(np.mean(scores)) if scores else None


def normalize_score(value, good_min, good_max, metric_type="higher_better"):
    """Normalize a metric value to 0-1 based on good range."""
    if value is None:
        return None
    if metric_type == "target_range":
        # Value is best when inside [good_min, good_max], worse as it deviates
        mid = (good_min + good_max) / 2
        half_range = (good_max - good_min) / 2
        if half_range < 1e-6:
            return 1.0 if abs(value - mid) < 1e-6 else 0.0
        deviation = abs(value - mid) / half_range
        return max(0.0, min(1.0, 1.0 - (deviation - 1.0))) if deviation > 1.0 else 1.0
    if metric_type == "lower_better":
        if value <= good_min:
            return 1.0
        if value >= good_max:
            return 0.0
        return 1.0 - (value - good_min) / (good_max - good_min)
    # higher_better (default)
    if value >= good_max:
        return 1.0
    if value <= good_min:
        return 0.0
    return (value - good_min) / (good_max - good_min)


def analyze_trajectory(joint_positions, joint_velocities, contact_states, contact_forces, dt, config_path=None):
    """Main entry point - called by evaluate_policy.py per scenario.

    Args:
        joint_positions: np.array (timesteps, num_joints)
        joint_velocities: np.array (timesteps, num_joints)
        contact_states: np.array (timesteps, num_feet) - binary 0/1
        contact_forces: np.array (timesteps, num_feet) - vertical force magnitude, or None
        dt: float - simulation timestep
        config_path: optional path to monitor_config.md for normalization ranges and weights

    Returns:
        dict with raw metrics and normalized scores
    """
    tier2 = parse_tier2_config(config_path)

    jerk = compute_joint_jerk(joint_positions, dt)
    periodicity = compute_step_periodicity(contact_states, dt)
    stance_swing = compute_stance_swing_ratio(contact_states)
    phase = compute_phase_offset(contact_states)
    grf = compute_grf_profile_score(contact_forces, contact_states)

    def get_range(metric, default_min, default_max, default_type):
        if tier2 and metric in tier2:
            return tier2[metric]["min"], tier2[metric]["max"], tier2[metric]["type"]
        return default_min, default_max, default_type

    jerk_min, jerk_max, jerk_type = get_range("joint_jerk", 0, 100, "lower_better")
    period_min, period_max, period_type = get_range("step_periodicity", 0, 1, "higher_better")
    phase_min, phase_max, phase_type = get_range("phase_offset", 0, 1, "higher_better")
    grf_min, grf_max, grf_type = get_range("grf_profile_score", 0, 1, "higher_better")

    stance_score = None
    if stance_swing is not None:
        stance_score = float(np.mean([1.0 - 2.0 * abs(r - 0.6) for r in stance_swing]))
        stance_score = max(0.0, min(1.0, stance_score))

    sub_scores = {
        "joint_jerk_score": normalize_score(jerk, jerk_min, jerk_max, jerk_type),
        "periodicity_score": normalize_score(periodicity, period_min, period_max, period_type),
        "stance_swing_score": stance_score,
        "phase_offset_score": normalize_score(phase, phase_min, phase_max, phase_type) if phase is not None else None,
        "grf_score": normalize_score(grf, grf_min, grf_max, grf_type) if grf is not None else None,
    }

    score_to_metric = {
        "joint_jerk_score": "joint_jerk",
        "periodicity_score": "step_periodicity",
        "stance_swing_score": "stance_swing_ratio",
        "phase_offset_score": "phase_offset",
        "grf_score": "grf_profile_score",
    }

    if tier2:
        weighted_sum = 0.0
        total_weight = 0.0
        for score_name, val in sub_scores.items():
            metric_name = score_to_metric.get(score_name)
            if val is not None and metric_name and metric_name in tier2:
                w = tier2[metric_name]["weight"]
                weighted_sum += val * w
                total_weight += w
        detailed_quality_score = float(weighted_sum / total_weight) if total_weight > 1e-6 else None
    else:
        valid = [s for s in sub_scores.values() if s is not None]
        detailed_quality_score = float(np.mean(valid)) if valid else None

    return {
        "joint_jerk": jerk,
        "step_periodicity": periodicity,
        "stance_swing_ratio": stance_swing,
        "phase_offset": phase,
        "grf_profile_score": grf,
        "sub_scores": sub_scores,
        "detailed_quality_score": detailed_quality_score,
    }


def format_markdown(results, scenario_name):
    """Format eval quality metrics as markdown."""
    lines = [f"\n### Gait Quality — {scenario_name}"]
    for key in ("joint_jerk", "step_periodicity", "phase_offset", "grf_profile_score"):
        val = results.get(key)
        lines.append(f"- **{key}**: {val:.4f}" if val is not None else f"- **{key}**: N/A")
    if results.get("stance_swing_ratio") is not None:
        ratios = ", ".join(f"{r:.2f}" for r in results["stance_swing_ratio"])
        lines.append(f"- **stance_swing_ratio**: [{ratios}]")
    dqs = results.get("detailed_quality_score")
    lines.append(f"- **detailed_quality_score**: {dqs:.3f}" if dqs is not None else "- **detailed_quality_score**: N/A")
    return "\n".join(lines)
