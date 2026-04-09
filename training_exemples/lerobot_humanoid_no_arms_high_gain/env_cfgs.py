"""LeRobot Humanoid high-gain velocity environment configurations.

Reuses all reward shaping, domain randomization and observation logic from the
base lerobot_humanoid_no_arms config. Only the robot model (3c56aee
identification) and the actuator gains differ.
"""

import math

from mjlab.actuator.delayed_actuator import DelayedActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg
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

ACTION_RATE_PENALTY_SCALE = 0.5


def _scale_high_gain_action_rate_penalties(cfg: ManagerBasedRlEnvCfg) -> None:
  """Reduce action-rate penalties for the high-gain variants only."""
  for reward_name in ("action_rate_l2", "action_rate_hipz_hipx_l2"):
    reward_term = cfg.rewards.get(reward_name)
    if reward_term is not None:
      reward_term.weight *= ACTION_RATE_PENALTY_SCALE

  for curriculum_name in ("action_rate_weight", "action_rate_hipz_hipx_weight"):
    curriculum_term = cfg.curriculum.get(curriculum_name)
    if curriculum_term is None:
      continue
    weight_stages = curriculum_term.params.get("weight_stages", [])
    for stage in weight_stages:
      if "weight" in stage:
        stage["weight"] *= ACTION_RATE_PENALTY_SCALE


def _apply_high_gain_robot(cfg: ManagerBasedRlEnvCfg, torque_obs: bool) -> ManagerBasedRlEnvCfg:
  """Swap in the high-gain robot and update dependent config."""
  robot_cfg = get_lerobot_hg_robot_cfg()
  orig_artic = robot_cfg.articulation
  robot_cfg.articulation = EntityArticulationInfoCfg(
    actuators=tuple(
      DelayedActuatorCfg(base_cfg=a, delay_min_lag=0, delay_max_lag=8)
      for a in orig_artic.actuators
    ),
    soft_joint_pos_limit_factor=orig_artic.soft_joint_pos_limit_factor,
  )
  cfg.scene.entities = {"robot": robot_cfg}

  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = LEROBOT_HG_ACTION_SCALE
  _scale_high_gain_action_rate_penalties(cfg)
  joint_pos_obs = cfg.observations["policy"].terms.get("joint_pos")
  if joint_pos_obs is not None and getattr(joint_pos_obs, "noise", None) is not None:
    joint_pos_noise_rad = math.radians(3.0)
    joint_pos_obs.noise.n_min = -joint_pos_noise_rad
    joint_pos_obs.noise.n_max = joint_pos_noise_rad

  # Sync torque obs with torque_obs flag (base config already added/skipped it).
  if torque_obs and "joint_torques" not in cfg.observations["policy"].terms:
    cfg.observations["policy"].terms["joint_torques"] = ObservationTermCfg(
      func=_joint_torques_obs,
      noise=Unoise(n_min=-0.1, n_max=0.1),
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
