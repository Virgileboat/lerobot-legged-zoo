# Monitoring Configuration — locomotion

## Task Type
locomotion

## Quality Metrics — Tier 1 (Derived from WandB)

Computed every monitor tick from raw WandB metrics. No training code changes needed.

| Metric | Source | Weight | Description |
|--------|--------|--------|-------------|
| symmetry_ratio | Torque/* L vs R joints | 0.25 | 1.0 = perfectly symmetric, 0.0 = fully asymmetric |
| action_smoothness | Episode_Reward/action_fft_band_le_3hz_ratio | 0.30 | Fraction of action energy below 3Hz (physics motion proxy) |
| survival_ratio | Episode_Termination/* | 0.25 | Fraction of episodes ending in timeout vs fell |
| reward_balance | Episode_Reward/* ratios | 0.20 | Balance between locomotion and regularization rewards |

Composite quality_score = weighted sum of above, clamped to [0, 1].

## Quality Metrics — Tier 2 (Eval-Time)

Computed during policy evaluation from full simulation trajectories.

| Metric | Description | Good Range |
|--------|-------------|------------|
| joint_jerk | Mean jerk (d³pos/dt³) across joints | < 50.0 |
| step_periodicity | FFT dominant frequency strength on foot contacts | > 0.5 |
| stance_swing_ratio | Per-foot ground/air time ratio | 0.5 — 0.7 stance |
| phase_offset | L/R foot contact timing offset | 0.4 — 0.6 |
| grf_profile_score | Vertical force pattern naturalness | > 0.5 |

Composite detailed_quality_score = mean of normalized sub-scores, clamped to [0, 1].

## Actuator Mapping

Left/right pairs for symmetry computation:
- left: [hipz_left, hipx_left, hipy_left, knee_left, ankley_left, anklex_left]
- right: [hipz_right, hipx_right, hipy_right, knee_right, ankley_right, anklex_right]

Locomotion reward prefixes (for reward_balance):
- locomotion: [track_linear_velocity, track_angular_velocity]
- regularization: [upright, pose, action_rate_l2, foot_slip, action_fft_band_le_3hz_ratio]

## Decision Rules

- quality_score_bad_threshold: 0.4
- quality_declining_monitors: 2
- quality_finish_minimum: 0.7
- reward_vs_quality_divergence: true

When reward_vs_quality_divergence is true:
- If reward is improving (> 5% gain) but quality_score is declining (> 5% drop) for quality_declining_monitors consecutive monitors → BAD

## Human Feedback Tags

- asymmetric_gait: Left/right legs behave differently
- jerky_motion: Abrupt, non-smooth movements (high frequency)
- shuffling: Feet barely leave the ground
- stumbling: Frequent near-falls or recovery motions
- reward_hacking: Achieves reward through unintended behavior
- too_conservative: Overly cautious, barely moving
- unstable_at_speed: Falls or wobbles at higher commanded velocities
- unnatural_posture: Body orientation or lean looks wrong
- lateral_lean: Robot consistently leans to one side

## Tag-to-Metric Mapping

Maps human feedback tags to the metrics that should improve when the issue is addressed.

- asymmetric_gait → symmetry_ratio
- jerky_motion → action_smoothness
- shuffling → survival_ratio
- stumbling → survival_ratio
- reward_hacking → reward_balance
- too_conservative → reward_balance
- unstable_at_speed → (no metric)
- unnatural_posture → (no metric)
- lateral_lean → symmetry_ratio

## Tier 2 Normalization

Format: metric, min, max, type, weight
Used by eval_metrics.py to normalize raw values and compute weighted composite.

- joint_jerk: 0, 100, lower_better, 0.15
- step_periodicity: 0, 1, higher_better, 0.25
- stance_swing_ratio: 0.5, 0.7, target_range, 0.20
- phase_offset: 0, 1, higher_better, 0.25
- grf_profile_score: 0, 1, higher_better, 0.15
