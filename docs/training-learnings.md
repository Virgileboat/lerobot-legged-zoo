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
- **`site_pos_w`/`root_pos_w` unavailable with EntityArticulationInfoCfg**: These attributes do not exist on the wrapped robot data object. Any reward that reads them silently returns 0.0 (runs 015–018 all had bilateral_symmetry=0.0 for this reason). Fix: use `robot.data.joint_pos` — always available for articulated robots. Index lateral joints by name pattern (`hipx_right`, `anklex_right`, etc.) rather than hardcoded site indices.
- **Worktree commits landing in detached HEAD**: When the main repo is on the training branch, `launch.sh` uses it directly (no worktree). Commits made in the separate worktree go to detached HEAD and are never seen by training. Always commit training changes to the main repo directly or verify the branch HEAD is updated before launch.

## What Doesn't Work

- **Joint-space bilateral symmetry as primary metric**: Misleads evaluation — raw actions are not in joint-space and produce nonsensical asymmetry numbers (run_012–014). Switch to foot-position metrics.
- **Broad `except Exception: pass` blocks in reward functions**: Hides all errors; the reward silently returns zero for the entire run. Always at minimum call `traceback.print_exc()`.
- **Structural hipx/anklex asymmetry is not fully correctable by reward alone**: Even with a working bilateral_symmetry reward (weight=2.0), hipx bias only improved from 0.323→0.304 (6%) over a full 20k-step run (run_019). The asymmetry appears to be partly structural (different hardware frictionloss per leg). Increasing reward weight further may help, but may conflict with walking quality.
