from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from ..lerobot_humanoid_no_arms.rl_cfg import lerobot_humanoid_no_arms_ppo_runner_cfg
from .env_cfgs import (
  lerobot_humanoid_no_arms_hg_flat_env_cfg,
  lerobot_humanoid_no_arms_hg_rough_env_cfg,
)

register_mjlab_task(
  task_id="Mjlab-Velocity-Rough-LeRobot-Humanoid-no-arms-high-gain",
  env_cfg=lerobot_humanoid_no_arms_hg_rough_env_cfg(),
  play_env_cfg=lerobot_humanoid_no_arms_hg_rough_env_cfg(play=True),
  rl_cfg=lerobot_humanoid_no_arms_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

# Ablation variants: with and without joint torque observation.
register_mjlab_task(
  task_id="Mjlab-Velocity-Flat-LeRobot-Humanoid-no-arms-high-gain-torque-obs",
  env_cfg=lerobot_humanoid_no_arms_hg_flat_env_cfg(torque_obs=True),
  play_env_cfg=lerobot_humanoid_no_arms_hg_flat_env_cfg(play=True, torque_obs=True),
  rl_cfg=lerobot_humanoid_no_arms_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Mjlab-Velocity-Flat-LeRobot-Humanoid-no-arms-high-gain-no-torque-obs",
  env_cfg=lerobot_humanoid_no_arms_hg_flat_env_cfg(torque_obs=False),
  play_env_cfg=lerobot_humanoid_no_arms_hg_flat_env_cfg(play=True, torque_obs=False),
  rl_cfg=lerobot_humanoid_no_arms_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
