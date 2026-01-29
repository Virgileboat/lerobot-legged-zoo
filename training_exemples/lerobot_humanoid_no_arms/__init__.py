from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfgs import (
  lerobot_humanoid_no_arms_flat_env_cfg,
  lerobot_humanoid_no_arms_rough_env_cfg,
)
from .rl_cfg import lerobot_humanoid_no_arms_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-Velocity-Rough-LeRobot-Humanoid-no-arms",
  env_cfg=lerobot_humanoid_no_arms_rough_env_cfg(),
  play_env_cfg=lerobot_humanoid_no_arms_rough_env_cfg(play=True),
  rl_cfg=lerobot_humanoid_no_arms_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Mjlab-Velocity-Flat-LeRobot-Humanoid-no-arms",
  env_cfg=lerobot_humanoid_no_arms_flat_env_cfg(),
  play_env_cfg=lerobot_humanoid_no_arms_flat_env_cfg(play=True),
  rl_cfg=lerobot_humanoid_no_arms_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

