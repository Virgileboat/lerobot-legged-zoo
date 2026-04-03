"""LeRobot Humanoid high-gain velocity environment configurations.

Reuses all reward shaping, domain randomization and observation logic from the
base lerobot_humanoid_no_arms config. Only the robot model (3c56aee
identification) and the actuator gains differ.
"""

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.utils.noise import UniformNoiseCfg as Unoise

from ..lerobot_humanoid_no_arms.env_cfgs import (
  _joint_torques_obs,
  lerobot_humanoid_no_arms_flat_env_cfg,
  lerobot_humanoid_no_arms_rough_env_cfg,
)
from .lerobot_humanoid_no_arms_high_gain_constants import (
  LEROBOT_HG_ACTION_SCALE,
  get_lerobot_hg_robot_cfg,
)


def _apply_high_gain_robot(cfg: ManagerBasedRlEnvCfg, torque_obs: bool) -> ManagerBasedRlEnvCfg:
  """Swap in the high-gain robot and update dependent config."""
  cfg.scene.entities = {"robot": get_lerobot_hg_robot_cfg()}

  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = LEROBOT_HG_ACTION_SCALE

  # Sync torque obs with torque_obs flag (base config already added/skipped it).
  if torque_obs and "joint_torques" not in cfg.observations["policy"].terms:
    cfg.observations["policy"].terms["joint_torques"] = ObservationTermCfg(
      func=_joint_torques_obs,
      noise=Unoise(n_min=-5.0, n_max=5.0),
      scale=1.0 / 88.0,
    )
  elif not torque_obs:
    cfg.observations["policy"].terms.pop("joint_torques", None)

  return cfg


def lerobot_humanoid_no_arms_hg_flat_env_cfg(
  play: bool = False,
  torque_obs: bool = True,
) -> ManagerBasedRlEnvCfg:
  """High-gain flat terrain config (3c56aee model, recommended gains)."""
  cfg = lerobot_humanoid_no_arms_flat_env_cfg(play=play, torque_obs=torque_obs)
  return _apply_high_gain_robot(cfg, torque_obs)


def lerobot_humanoid_no_arms_hg_rough_env_cfg(
  play: bool = False,
  torque_obs: bool = True,
) -> ManagerBasedRlEnvCfg:
  """High-gain rough terrain config (3c56aee model, recommended gains)."""
  cfg = lerobot_humanoid_no_arms_rough_env_cfg(play=play, torque_obs=torque_obs)
  return _apply_high_gain_robot(cfg, torque_obs)
