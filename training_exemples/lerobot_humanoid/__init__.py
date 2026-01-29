from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfgs import (
  lerobot_humanoid_flat_env_cfg,
  lerobot_humanoid_rough_env_cfg,
)
from .rl_cfg import lerobot_humanoid_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-Velocity-Rough-LeRobot-Humanoid",
  env_cfg=lerobot_humanoid_rough_env_cfg(),
  play_env_cfg=lerobot_humanoid_rough_env_cfg(play=True),
  rl_cfg=lerobot_humanoid_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Mjlab-Velocity-Flat-LeRobot-Humanoid",
  env_cfg=lerobot_humanoid_flat_env_cfg(),
  play_env_cfg=lerobot_humanoid_flat_env_cfg(play=True),
  rl_cfg=lerobot_humanoid_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

