# lerobot-legged-zoo

Legged robot models (MJCF + assets) plus MJLab training examples in one place.
Run from the repo root with `uv`.

Status by robot:
- LeRobot Humanoid family: in development (common CAD source):
  `https://cad.onshape.com/documents/fb645318a27646d1d8840be6/w/d1cae8805fb652b4d1614997/e/621ef473e7cde4418aaec2ed`
- Leggy: WIP concept; no hardware yet.
- Open Duck: reference repo `https://github.com/apirrone/Open_Duck_Playground`

## Installation

This project uses `uv`. A CUDA-capable GPU is required for training/play.

```bash
uv sync
```

## Run Training

```bash
# Train with default settings
uv run train Mjlab-Velocity-Flat-Open-Duck-v2

# Adjust number of parallel environments
uv run train Mjlab-Velocity-Flat-Open-Duck-v2 --env.scene.num-envs 2048

# See all options
uv run train Mjlab-Velocity-Flat-Open-Duck-v2 --help
```

## Run Play/Evaluation

```bash
# Play with a wandb checkpoint
uv run play Mjlab-Velocity-Flat-Open-Duck-v2 --wandb-run-path <your-wandb-path>

# Play with a local checkpoint
uv run play Mjlab-Velocity-Flat-Open-Duck-v2 --checkpoint-file logs/rsl_rl/open_duck_v2/<run>/model_*.pt

# Use random actions (sanity check)
uv run play Mjlab-Velocity-Flat-Open-Duck-v2 --agent random
```

## Available Tasks

| Task Name | Description |
|-----------|-------------|
| `Mjlab-Velocity-Flat-Leggy` | Train Leggy to walk on flat ground |
| `Mjlab-Velocity-Rough-Leggy` | Train Leggy to walk on rough terrain |
| `Mjlab-Velocity-Flat-Open-Duck-v2` | Train Open Duck v2 to walk on flat ground |
| `Mjlab-Velocity-Rough-Open-Duck-v2` | Train Open Duck v2 to walk on rough terrain |
| `Mjlab-Velocity-Flat-LeRobot-Humanoid` | Train LeRobot Humanoid to walk on flat ground |
| `Mjlab-Velocity-Rough-LeRobot-Humanoid` | Train LeRobot Humanoid to walk on rough terrain |
| `Mjlab-Velocity-Flat-LeRobot-Humanoid-full` | Train LeRobot Humanoid (full) on flat ground |
| `Mjlab-Velocity-Rough-LeRobot-Humanoid-full` | Train LeRobot Humanoid (full) on rough terrain |
| `Mjlab-Velocity-Flat-LeRobot-Humanoid-no-arms` | Train LeRobot Humanoid (no arms) on flat ground |
| `Mjlab-Velocity-Rough-LeRobot-Humanoid-no-arms` | Train LeRobot Humanoid (no arms) on rough terrain |

## Robots (Details)

Each robot has a model under `models/` and a matching training example under
`training_exemples/`.

### Leggy
- Model: `models/leggy/robot.xml`
- Training: `training_exemples/leggy/env_cfgs.py`
- Constants: `training_exemples/leggy/leggy_constants.py`
- Tasks: `Mjlab-Velocity-Flat-Leggy`, `Mjlab-Velocity-Rough-Leggy`
- Small biped with parallel legs, designed for compact locomotion experiments.
![Leggy](media/leggy.svg)

### Open Duck v2 (Backlash)
- Model: `models/open_duck_v2/open_duck_v2_backlash.xml`
- Training: `training_exemples/open_duck_v2/env_cfgs.py`
- Constants: `training_exemples/open_duck_v2/open_duck_v2_constants.py`
- Tasks: `Mjlab-Velocity-Flat-Open-Duck-v2`, `Mjlab-Velocity-Rough-Open-Duck-v2`
- Updated Open Duck with backlash joints and improved geometry.
![Open Duck v2](media/open_duck_v2.svg)

### LeRobot Humanoid Full (20-DOF)
- Model: `models/lerobot_humanoide/mjcf/robot.xml`
- Training: `training_exemples/lerobot_humanoid_full/env_cfgs.py`
- Constants: `training_exemples/lerobot_humanoid_full/lerobot_humanoid_full_constants.py`
- Tasks: `Mjlab-Velocity-Flat-LeRobot-Humanoid-full`, `Mjlab-Velocity-Rough-LeRobot-Humanoid-full`
- Full humanoid with upper‑body joints for whole‑body control research.
![LeRobot Humanoid Full](media/lerobot_humanoid_full.png)

### LeRobot Humanoid (12-DOF)
- Model: `models/bipedal_plateform/mjcf/robot.xml`
- Training: `training_exemples/lerobot_humanoid/env_cfgs.py`
- Constants: `training_exemples/lerobot_humanoid/lerobot_humanoid_constants.py`
- Tasks: `Mjlab-Velocity-Flat-LeRobot-Humanoid`, `Mjlab-Velocity-Rough-LeRobot-Humanoid`
- 12‑DOF lower‑body humanoid for fast velocity‑tracking experiments.
[![LeRobot Humanoid (rough terrain policy)](media/lerobot_humanoid.png)](media/lerobot_humanoide.mp4)

### LeRobot Humanoid No-Arms
- Model: `models/bipedal_plateform_no_arms/mjcf/robot.xml`
- Training: `training_exemples/lerobot_humanoid_no_arms/env_cfgs.py`
- Constants: `training_exemples/lerobot_humanoid_no_arms/lerobot_humanoid_no_arms_constants.py`
- Tasks: `Mjlab-Velocity-Flat-LeRobot-Humanoid-no-arms`, `Mjlab-Velocity-Rough-LeRobot-Humanoid-no-arms`
- Lower‑body‑only humanoid variant without arms.
![LeRobot Humanoid No-Arms](media/lerobot_humanoid_no_arms.png)

## Modify Rewards / Cost Function

Edit `training_exemples/<robot>/env_cfgs.py` and tune `cfg.rewards`:

```python
# Example: change weights
cfg.rewards["upright"].weight = 1.0
cfg.rewards["body_ang_vel"].weight = -0.05

# Example: add a new cost term
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.tasks.velocity import mdp

cfg.rewards["self_collisions"] = RewardTermCfg(
  func=mdp.self_collision_cost,
  weight=-1.0,
  params={"sensor_name": "self_collision"},
)
```

After edits, rerun your task with `uv run train ...`.
