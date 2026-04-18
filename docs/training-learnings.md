# Training Learnings

Actionable tips and insights accumulated from training experiments.
Each entry is backed by observed evidence from training runs.

## Reward Design

- **Foot-position symmetry reward over joint-space**: Joint-space bilateral symmetry metrics are unreliable indicators of visual gait quality. Use world-space foot site positions (y-lean, max height, mean height, swing range) instead. Run_013+: user confirmed gait looked better with foot-position metrics.
- **air_time=0 is a decisive failure signal**: When `Episode_Reward/air_time` stays at 0.0 for 3+ monitors, the robot is shuffling/standing on one leg and no amount of tuning recovers it without explicit stepping incentive. Raise air_time reward weight (0→1.5) and reduce hipz/hipx penalties to allow stepping.
- **bilateral_symmetry reward weight must stay low (≤0.3) initially**: Too-high bilateral symmetry weight causes the robot to stand on one leg rather than walk. Introduce it gradually after a walking gait is established.

## Observation Space

## Training Hyperparameters

## Physical Limits & Robot Capabilities

- **anklex right_mean ≈ +0.203 is a structural bias**: The right ankle lateral joint consistently reads +0.203 rad mean across all scenarios and all runs from run_015 onward. This appears to be a physical/hardware asymmetry (asymmetric frictionloss per identification) rather than a policy failure. Do not expect bilateral symmetry at anklex to reach zero.

## Common Failure Modes

- **Silent reward failure from wrong attribute access**: `robot.data.site_names` throws `AttributeError` silently swallowed by `except Exception: pass`, causing bilateral_symmetry reward to return 0.0 throughout training. Fix: use hardcoded site indices (torso=0, foot_right=1, foot_left=2 per MJCF order). Always surface exceptions with `traceback.print_exc()` instead of bare `pass`.
- **Eval metrics using raw actions instead of joint_pos**: Computing bilateral asymmetry from `env.action_manager.action` (unscaled network output) produces phantom values (e.g. hipy_mean=-11.36, anklex_mean=7.65). Use `robot.data.joint_pos` for physically meaningful symmetry metrics.

## What Doesn't Work

- **Joint-space bilateral symmetry as primary metric**: Misleads evaluation — raw actions are not in joint-space and produce nonsensical asymmetry numbers (run_012–014). Switch to foot-position metrics.
- **Broad `except Exception: pass` blocks in reward functions**: Hides all errors; the reward silently returns zero for the entire run. Always at minimum call `traceback.print_exc()`.
