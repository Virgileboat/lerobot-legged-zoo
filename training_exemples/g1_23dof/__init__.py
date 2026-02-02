from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfgs import g1_23dof_flat_env_cfg, g1_23dof_rough_env_cfg
from .rl_cfg import g1_23dof_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-Velocity-Rough-Unitree-G1-23dof",
  env_cfg=g1_23dof_rough_env_cfg(),
  play_env_cfg=g1_23dof_rough_env_cfg(play=True),
  rl_cfg=g1_23dof_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Mjlab-Velocity-Flat-Unitree-G1-23dof",
  env_cfg=g1_23dof_flat_env_cfg(),
  play_env_cfg=g1_23dof_flat_env_cfg(play=True),
  rl_cfg=g1_23dof_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
