"""MDP functions for open duck mini v2 tasks"""

import math
from typing import TYPE_CHECKING, Optional

import numpy as np
import torch

from mjlab.envs.manager_based_rl_env import ManagerBasedRlEnv
from mjlab.envs.mdp.actions import JointPositionActionCfg as _JointPositionActionCfg
from mjlab.entity import Entity
from mjlab.managers.reward_manager import RewardManager as _RewardManager
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.tasks.velocity.mdp.velocity_command import (
    UniformVelocityCommand,
    UniformVelocityCommandCfg,
)
from mjlab.utils.lab_api.math import matrix_from_quat

if TYPE_CHECKING:
    from mjlab.viewer.debug_visualizer import DebugVisualizer


# ---------------------------------------------------------------------------
# NaN-safe training patches. Reward/advantage/std can go NaN under extreme
# contact impulses or sudden curriculum steps; these patches sanitize them
# before they propagate into gradients. Guarded with try/except because
# rsl_rl's internal class layout shifts between versions.
# ---------------------------------------------------------------------------
_orig_reward_compute = _RewardManager.compute

def _nan_safe_reward_compute(self, dt: float) -> torch.Tensor:
    result = _orig_reward_compute(self, dt)
    for key in self._episode_sums:
        torch.nan_to_num_(self._episode_sums[key], nan=0.0)
    return torch.nan_to_num(result, nan=0.0)

_RewardManager.compute = _nan_safe_reward_compute


try:
    from rsl_rl.algorithms.ppo import PPO as _PPO

    _orig_compute_returns = _PPO.compute_returns

    def _safe_compute_returns(self, obs) -> None:
        _orig_compute_returns(self, obs)
        st = self.storage
        torch.nan_to_num_(st.advantages, nan=0.0, posinf=0.0, neginf=0.0)
        torch.nan_to_num_(st.returns, nan=0.0, posinf=0.0, neginf=0.0)

    _PPO.compute_returns = _safe_compute_returns
    _PPO_PATCH_OK = True
except Exception as _e:
    _PPO_PATCH_OK = False
    print(f"[mdp] PPO.compute_returns patch skipped: {_e}")


_STD_PATCH_OK = False
try:
    # rsl-rl-lib 3.x: patch ActorCritic._update_distribution
    from rsl_rl.modules.actor_critic import ActorCritic as _ActorCritic

    _orig_update_dist = _ActorCritic._update_distribution

    def _safe_update_distribution(self, obs: torch.Tensor) -> None:
        if not self.state_dependent_std:
            with torch.no_grad():
                if self.noise_std_type == "scalar" and hasattr(self, "std"):
                    self.std.data.clamp_(min=1e-3).nan_to_num_(nan=1e-3)
                elif self.noise_std_type == "log" and hasattr(self, "log_std"):
                    self.log_std.data.nan_to_num_(nan=math.log(1e-3))
        _orig_update_dist(self, obs)

    _ActorCritic._update_distribution = _safe_update_distribution
    _STD_PATCH_OK = True
except Exception:
    # rsl-rl-lib 5.x: patch GaussianDistribution.update
    try:
        from rsl_rl.modules.distribution import GaussianDistribution as _GaussianDistribution

        _orig_update = _GaussianDistribution.update

        def _safe_update(self, mlp_output: torch.Tensor) -> None:
            with torch.no_grad():
                if self.std_type == "scalar" and hasattr(self, "std_param"):
                    self.std_param.data.clamp_(min=1e-3).nan_to_num_(nan=1e-3)
                elif self.std_type == "log" and hasattr(self, "log_std_param"):
                    self.log_std_param.data.nan_to_num_(nan=math.log(1e-3))
            _orig_update(self, mlp_output)

        _GaussianDistribution.update = _safe_update
        _STD_PATCH_OK = True
    except Exception as _e:
        print(f"[mdp] Gaussian std patch skipped: {_e}")

print(
    "[mdp] NaN-safe patches active: reward=yes, "
    f"advantages={'yes' if _PPO_PATCH_OK else 'skipped'}, "
    f"std={'yes' if _STD_PATCH_OK else 'skipped'}"
)


_DEFAULT_ASSET_CFG = SceneEntityCfg("robot")


# ---------------------------------------------------------------------------
# Neck offset action: adds a randomised offset to neck/head joint targets so
# the policy learns to walk through head disturbance and a user can inject
# head motion on top at deployment.
# ---------------------------------------------------------------------------

_NECK_JOINT_PATTERNS = [r".*neck_pitch.*", r".*head_pitch.*", r".*head_yaw.*", r".*head_roll.*"]
_NECK_OFFSET_SMOOTHING_TAU = 0.5


class NeckOffsetJointPositionAction(_JointPositionActionCfg.class_type):
    """JointPositionAction that adds a random offset to neck/head joint targets."""

    def apply_actions(self) -> None:
        super().apply_actions()

        env = self._env

        if not hasattr(env, "_neck_offset"):
            env._neck_offset = torch.zeros(env.num_envs, 4, device=env.device)
            env._neck_offset_target = torch.zeros(env.num_envs, 4, device=env.device)

        if not hasattr(self, "_neck_joint_ids"):
            ids, _ = self._entity.find_joints_by_actuator_names(_NECK_JOINT_PATTERNS)
            self._neck_joint_ids = torch.tensor(ids, device=env.device, dtype=torch.long)

        alpha = min(1.0, env.step_dt / _NECK_OFFSET_SMOOTHING_TAU)
        env._neck_offset.lerp_(env._neck_offset_target, alpha)

        self._entity.data.joint_pos_target[:, self._neck_joint_ids] += env._neck_offset


class NeckOffsetJointPositionActionCfg(_JointPositionActionCfg):
    def build(self, env: ManagerBasedRlEnv) -> "NeckOffsetJointPositionAction":
        return NeckOffsetJointPositionAction(self, env)


def reset_neck_offset(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor,
):
    """Reset neck joint offsets to zero at episode start."""
    if not hasattr(env, "_neck_offset"):
        env._neck_offset = torch.zeros(env.num_envs, 4, device=env.device)
        env._neck_offset_target = torch.zeros(env.num_envs, 4, device=env.device)

    if len(env_ids) > 0:
        env._neck_offset[env_ids] = 0.0
        env._neck_offset_target[env_ids] = 0.0


def randomize_neck_offset_target(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor,
    max_offset: float = 0.3,
):
    """Sample new random neck offset targets at a given interval."""
    if not hasattr(env, "_neck_offset_target"):
        env._neck_offset = torch.zeros(env.num_envs, 4, device=env.device)
        env._neck_offset_target = torch.zeros(env.num_envs, 4, device=env.device)

    if len(env_ids) > 0:
        env._neck_offset_target[env_ids] = (
            torch.rand(len(env_ids), 4, device=env.device) * 2 - 1
        ) * max_offset


def reset_action_history(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
):
    """Reset cached action history for environments that are being reset."""
    if len(env_ids) == 0:
        return

    asset: Entity = env.scene[asset_cfg.name]

    if hasattr(env, '_prev_leg_actions'):
        if hasattr(env, 'action_manager') and env.action_manager.action is not None:
            leg_joint_indices = list(range(0, 5)) + list(range(9, 14))
            env._prev_leg_actions[env_ids] = env.action_manager.action[env_ids][:, leg_joint_indices]
        else:
            env._prev_leg_actions[env_ids] = 0.0

    if hasattr(env, '_prev_neck_actions'):
        if hasattr(env, 'action_manager') and env.action_manager.action is not None:
            neck_joint_indices = list(range(5, 9))
            env._prev_neck_actions[env_ids] = env.action_manager.action[env_ids][:, neck_joint_indices]
        else:
            env._prev_neck_actions[env_ids] = 0.0

    if hasattr(env, '_prev_leg_actions_for_acc'):
        if hasattr(env, 'action_manager') and env.action_manager.action is not None:
            leg_joint_indices = list(range(0, 5)) + list(range(9, 14))
            current_action = env.action_manager.action[env_ids][:, leg_joint_indices]
            env._prev_leg_actions_for_acc[env_ids] = current_action
            env._prev_prev_leg_actions_for_acc[env_ids] = current_action
        else:
            env._prev_leg_actions_for_acc[env_ids] = 0.0
            env._prev_prev_leg_actions_for_acc[env_ids] = 0.0

    if hasattr(env, '_prev_neck_actions_for_acc'):
        if hasattr(env, 'action_manager') and env.action_manager.action is not None:
            neck_joint_indices = list(range(5, 9))
            current_action = env.action_manager.action[env_ids][:, neck_joint_indices]
            env._prev_neck_actions_for_acc[env_ids] = current_action
            env._prev_prev_neck_actions_for_acc[env_ids] = current_action
        else:
            env._prev_neck_actions_for_acc[env_ids] = 0.0
            env._prev_prev_neck_actions_for_acc[env_ids] = 0.0

    if hasattr(asset.data, '_prev_joint_vel'):
        joint_vel = asset.data.joint_vel[env_ids, :][:, asset_cfg.joint_ids]
        asset.data._prev_joint_vel[env_ids] = joint_vel

    if hasattr(env, '_prev_actuator_forces'):
        env._prev_actuator_forces[env_ids] = asset.data.actuator_force[env_ids].clone()


def leg_action_rate_l2(
    env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG
) -> torch.Tensor:
    """Penalize the rate of change of leg actions (action_t - action_{t-1})."""
    leg_joint_indices = list(range(0, 5)) + list(range(9, 14))

    if not hasattr(env, 'action_manager'):
        return torch.zeros(env.num_envs, device=env.device)

    actions = env.action_manager.action
    if actions.shape[1] < 14:
        return torch.zeros(env.num_envs, device=env.device)

    leg_actions = actions[:, leg_joint_indices]

    if not hasattr(env, '_prev_leg_actions'):
        env._prev_leg_actions = leg_actions.clone()
        return torch.zeros(env.num_envs, device=env.device)

    action_rate = leg_actions - env._prev_leg_actions
    env._prev_leg_actions = leg_actions.clone()

    return torch.sum(torch.square(action_rate), dim=1)


def neck_action_rate_l2(
    env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG
) -> torch.Tensor:
    """Penalize the rate of change of neck actions (action_t - action_{t-1})."""
    neck_joint_indices = list(range(5, 9))

    if not hasattr(env, 'action_manager'):
        return torch.zeros(env.num_envs, device=env.device)

    actions = env.action_manager.action
    if actions.shape[1] < 14:
        return torch.zeros(env.num_envs, device=env.device)

    neck_actions = actions[:, neck_joint_indices]

    if not hasattr(env, '_prev_neck_actions'):
        env._prev_neck_actions = neck_actions.clone()
        return torch.zeros(env.num_envs, device=env.device)

    action_rate = neck_actions - env._prev_neck_actions
    env._prev_neck_actions = neck_actions.clone()

    return torch.sum(torch.square(action_rate), dim=1)


def com_height_target(
    env: ManagerBasedRlEnv,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
    target_height_min: float = 0.13,
    target_height_max: float = 0.18,
) -> torch.Tensor:
    """Reward for keeping the CoM within a target height range."""
    asset: Entity = env.scene[asset_cfg.name]

    com_height = torch.nan_to_num(
        asset.data.root_link_pos_w[:, 2] - env.scene.terrain.env_origins[:, 2], nan=0.0
    )

    below_min = com_height < target_height_min
    above_max = com_height > target_height_max
    in_range = ~(below_min | above_max)

    penalty_below = torch.square(com_height - target_height_min) * below_min.float()
    penalty_above = torch.square(com_height - target_height_max) * above_max.float()

    return in_range.float() - (penalty_below + penalty_above)


def joint_torques_l2(
    env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG
) -> torch.Tensor:
    """Penalize actuator forces (torques) to encourage energy-efficient motion."""
    asset: Entity = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.actuator_force), dim=1)


def robot_state_is_nan(
    env: ManagerBasedRlEnv,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """Terminate environments where MuJoCo produced NaN joint positions."""
    asset: Entity = env.scene[asset_cfg.name]
    return torch.any(torch.isnan(asset.data.joint_pos), dim=1)


def randomize_delayed_actuator_gains(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor,
    kp_range: tuple[float, float],
    kd_range: tuple[float, float],
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
    operation: str = "scale",
):
    """Randomize PD gains.

    Handles:
    - XmlPositionActuator (possibly wrapped in DelayedActuator): modifies MuJoCo
      model actuator_gainprm/biasprm directly.
    - BamM6Actuator: uses its built-in set_gains()/reset_gains() API.
    """
    from mjlab.actuator import XmlPositionActuator
    from mjlab.actuator.delayed_actuator import DelayedActuator
    from .actuator import BamM6Actuator

    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device, dtype=torch.int)
    else:
        env_ids = env_ids.to(env.device, dtype=torch.int)

    asset: Entity = env.scene[asset_cfg.name]

    if not hasattr(env, '_original_actuator_gains'):
        env._original_actuator_gains = {}

    for actuator in asset.actuators:
        if isinstance(actuator, DelayedActuator):
            base_actuator = actuator._base_actuator
        else:
            base_actuator = actuator

        ctrl_ids = base_actuator.ctrl_ids
        ctrl_key = tuple(ctrl_ids.tolist())

        if not isinstance(base_actuator, BamM6Actuator):
            if ctrl_key not in env._original_actuator_gains:
                env._original_actuator_gains[ctrl_key] = {
                    'gainprm': env.sim.model.actuator_gainprm[0, ctrl_ids, 0].clone(),
                    'biasprm1': env.sim.model.actuator_biasprm[0, ctrl_ids, 1].clone(),
                    'biasprm2': env.sim.model.actuator_biasprm[0, ctrl_ids, 2].clone(),
                }

        if isinstance(base_actuator, BamM6Actuator):
            base_actuator.reset_gains(env_ids)
        else:
            original = env._original_actuator_gains[ctrl_key]
            env.sim.model.actuator_gainprm[env_ids[:, None], ctrl_ids, 0] = original['gainprm'].unsqueeze(0).expand(len(env_ids), -1)
            env.sim.model.actuator_biasprm[env_ids[:, None], ctrl_ids, 1] = original['biasprm1'].unsqueeze(0).expand(len(env_ids), -1)
            env.sim.model.actuator_biasprm[env_ids[:, None], ctrl_ids, 2] = original['biasprm2'].unsqueeze(0).expand(len(env_ids), -1)

        kp_samples = torch.rand(len(env_ids), len(ctrl_ids), device=env.device) * (kp_range[1] - kp_range[0]) + kp_range[0]
        kd_samples = torch.rand(len(env_ids), len(ctrl_ids), device=env.device) * (kd_range[1] - kd_range[0]) + kd_range[0]

        if isinstance(base_actuator, XmlPositionActuator):
            if operation == "scale":
                env.sim.model.actuator_gainprm[env_ids[:, None], ctrl_ids, 0] *= kp_samples
                env.sim.model.actuator_biasprm[env_ids[:, None], ctrl_ids, 1] *= kp_samples
                env.sim.model.actuator_biasprm[env_ids[:, None], ctrl_ids, 2] *= kd_samples
            elif operation == "abs":
                env.sim.model.actuator_gainprm[env_ids[:, None], ctrl_ids, 0] = kp_samples
                env.sim.model.actuator_biasprm[env_ids[:, None], ctrl_ids, 1] = -kp_samples
                env.sim.model.actuator_biasprm[env_ids[:, None], ctrl_ids, 2] = -kd_samples
        elif isinstance(base_actuator, BamM6Actuator):
            kp_mean = kp_samples.mean(dim=1, keepdim=True)
            kd_mean = kd_samples.mean(dim=1, keepdim=True)
            base_actuator.set_gains(env_ids, kp_scale=kp_mean, kd_scale=kd_mean)


def randomize_mass_and_inertia(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor,
    scale_range: tuple[float, float],
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
):
    """Randomize body mass and inertia together with the same scaling factor."""
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device, dtype=torch.int)
    else:
        env_ids = env_ids.to(env.device, dtype=torch.int)

    asset: Entity = env.scene[asset_cfg.name]

    body_ids = asset_cfg.body_ids
    if isinstance(body_ids, slice):
        body_ids = list(range(asset.num_bodies))[body_ids]
    body_indices = asset.indexing.body_ids[body_ids]

    num_envs = len(env_ids)
    num_bodies = len(body_indices)
    scales = torch.rand(num_envs, num_bodies, device=env.device) * (scale_range[1] - scale_range[0]) + scale_range[0]

    if not hasattr(env, '_original_mass_inertia'):
        env._original_mass_inertia = {
            'mass': env.sim.model.body_mass[0, body_indices].clone(),
            'inertia': env.sim.model.body_inertia[0, body_indices].clone(),
        }

    original = env._original_mass_inertia
    env.sim.model.body_mass[env_ids[:, None], body_indices] = original['mass'].unsqueeze(0).expand(num_envs, -1)
    env.sim.model.body_inertia[env_ids[:, None], body_indices] = original['inertia'].unsqueeze(0).expand(num_envs, -1, -1)

    env.sim.model.body_mass[env_ids[:, None], body_indices] *= scales
    env.sim.model.body_inertia[env_ids[:, None], body_indices] *= scales.unsqueeze(-1)


def velocity_command_ranges_curriculum(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor,
    command_name: str,
    velocity_stages: list[dict],
) -> torch.Tensor:
    """Update velocity command ranges based on training progress."""
    del env_ids

    from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
    from typing import cast

    command_term = env.command_manager.get_term(command_name)
    assert command_term is not None, f"Command term '{command_name}' not found"

    cfg = cast(UniformVelocityCommandCfg, command_term.cfg)

    current_lin_vel = velocity_stages[0]["lin_vel_range"]
    current_ang_vel = velocity_stages[0]["ang_vel_range"]

    for stage in velocity_stages:
        if env.common_step_counter > stage["step"]:
            current_lin_vel = stage["lin_vel_range"]
            current_ang_vel = stage["ang_vel_range"]

    cfg.ranges.lin_vel_x = (-current_lin_vel, current_lin_vel)
    cfg.ranges.lin_vel_y = (-current_lin_vel, current_lin_vel)
    cfg.ranges.ang_vel_z = (-current_ang_vel, current_ang_vel)

    return torch.tensor([current_lin_vel])


def neck_offset_curriculum(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor,
    event_name: str,
    offset_stages: list[dict],
) -> torch.Tensor:
    """Update neck offset magnitude based on training progress."""
    del env_ids

    assert event_name in env.cfg.events, f"Event '{event_name}' not found"
    event_cfg = env.cfg.events[event_name]

    current_offset = offset_stages[0]["max_offset"]
    for stage in offset_stages:
        if env.common_step_counter > stage["step"]:
            current_offset = stage["max_offset"]

    event_cfg.params["max_offset"] = current_offset
    return torch.tensor([current_offset])


def com_range_curriculum(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor,
    event_name: str,
    range_stages: list[dict],
) -> torch.Tensor:
    """Update CoM randomization range based on training progress."""
    del env_ids

    assert event_name in env.cfg.events, f"Event '{event_name}' not found"
    event_cfg = env.cfg.events[event_name]

    current_range = range_stages[0]["range"]
    for stage in range_stages:
        if env.common_step_counter > stage["step"]:
            current_range = stage["range"]

    event_cfg.params["ranges"] = (-current_range, current_range)
    return torch.tensor([current_range])


def projected_gravity(
    env: ManagerBasedRlEnv,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """Projected gravity vector in body frame."""
    asset: Entity = env.scene[asset_cfg.name]
    return asset.data.projected_gravity_b


def raw_accelerometer(
    env: ManagerBasedRlEnv,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """Raw accelerometer reading (gravity + linear acceleration), normalised."""
    asset: Entity = env.scene[asset_cfg.name]

    mj_model = asset.data.model
    sensor_adr_array = mj_model.sensor_adr
    sensor_id = 4  # imu_accel
    sensor_adr = int(sensor_adr_array[sensor_id].item())

    accel_raw = asset.data.data.sensordata[:, sensor_adr:sensor_adr+3]

    accel_negated = -accel_raw
    accel_norm = torch.norm(accel_negated, dim=-1, keepdim=True)
    accel_normalized = torch.where(
        accel_norm > 0.1,
        accel_negated / accel_norm,
        asset.data.projected_gravity_b
    )

    return accel_normalized


def randomize_imu_orientation(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor,
    max_angle_deg: float = 2.0,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
):
    """Randomize IMU sensor mounting orientation by small angles."""
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device, dtype=torch.int)
    else:
        env_ids = env_ids.to(env.device, dtype=torch.int)

    asset: Entity = env.scene[asset_cfg.name]
    site_id = 0  # IMU site

    if not hasattr(env, '_original_imu_quat'):
        env._original_imu_quat = env.sim.model.site_quat[0, site_id].clone()

    num_envs = len(env_ids)
    max_angle_rad = max_angle_deg * torch.pi / 180.0
    angles = (torch.rand(num_envs, 3, device=env.device) * 2 - 1) * max_angle_rad

    half_angles = angles / 2.0
    quats_delta = torch.zeros(num_envs, 4, device=env.device)
    quats_delta[:, 0] = 1.0
    quats_delta[:, 1:] = half_angles
    quats_delta = quats_delta / torch.norm(quats_delta, dim=1, keepdim=True)

    original_quat = env._original_imu_quat.unsqueeze(0).expand(num_envs, -1)
    w1, x1, y1, z1 = quats_delta[:, 0], quats_delta[:, 1], quats_delta[:, 2], quats_delta[:, 3]
    w2, x2, y2, z2 = original_quat[:, 0], original_quat[:, 1], original_quat[:, 2], original_quat[:, 3]

    new_quat = torch.stack([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ], dim=1)

    env.sim.model.site_quat[env_ids, site_id] = new_quat


def randomize_base_orientation(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor,
    max_pitch_deg: float = 10.0,
    max_roll_deg: float = 5.0,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
):
    """Randomize base orientation at episode start to force reactive behavior."""
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device, dtype=torch.int)
    else:
        env_ids = env_ids.to(env.device, dtype=torch.int)

    asset: Entity = env.scene[asset_cfg.name]
    num_envs = len(env_ids)

    max_pitch_rad = max_pitch_deg * torch.pi / 180.0
    max_roll_rad = max_roll_deg * torch.pi / 180.0

    pitch = (torch.rand(num_envs, device=env.device) * 2 - 1) * max_pitch_rad
    roll = (torch.rand(num_envs, device=env.device) * 2 - 1) * max_roll_rad
    yaw = torch.zeros(num_envs, device=env.device)

    cy = torch.cos(yaw * 0.5)
    sy = torch.sin(yaw * 0.5)
    cp = torch.cos(pitch * 0.5)
    sp = torch.sin(pitch * 0.5)
    cr = torch.cos(roll * 0.5)
    sr = torch.sin(roll * 0.5)

    quat_w = cr * cp * cy + sr * sp * sy
    quat_x = sr * cp * cy - cr * sp * sy
    quat_y = cr * sp * cy + sr * cp * sy
    quat_z = cr * cp * sy - sr * sp * cy

    new_quat = torch.stack([quat_w, quat_x, quat_y, quat_z], dim=1)
    new_quat = new_quat / torch.norm(new_quat, dim=1, keepdim=True)

    root_quat_idx = 3
    env.sim.data.qpos[env_ids, root_quat_idx:root_quat_idx+4] = new_quat


def stillness_at_zero_command(
    env: ManagerBasedRlEnv,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
    command_name: str = "twist",
    command_threshold: float = 0.01,
    vel_std: float = 0.1,
) -> torch.Tensor:
    """Reward staying still when velocity command is near zero."""
    asset: Entity = env.scene[asset_cfg.name]

    command = env.command_manager.get_command(command_name)
    total_speed = torch.norm(command[:, :2], dim=1) + torch.abs(command[:, 2])
    is_standing_cmd = (total_speed < command_threshold).float()

    body_vel = torch.norm(asset.data.root_link_vel_w[:, :2], dim=1)
    stillness = torch.exp(-body_vel ** 2 / vel_std ** 2)

    return is_standing_cmd * stillness


def standing_pose(
    env: ManagerBasedRlEnv,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
    command_name: str = "twist",
    command_threshold: float = 0.01,
    std: float = 0.15,
) -> torch.Tensor:
    """Extra pose reward active only when command is near zero."""
    asset: Entity = env.scene[asset_cfg.name]

    command = env.command_manager.get_command(command_name)
    total_speed = torch.norm(command[:, :2], dim=1) + torch.abs(command[:, 2])
    is_standing = (total_speed < command_threshold).float()

    joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]
    default_pos = asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    error = joint_pos - default_pos
    pose_reward = torch.exp(-torch.mean(error ** 2, dim=1) / std ** 2)

    return is_standing * pose_reward


def standing_envs_curriculum(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor,
    command_name: str,
    standing_stages: list[dict],
) -> torch.Tensor:
    """Update the fraction of standing environments based on training progress."""
    from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
    from typing import cast

    del env_ids

    command_term = env.command_manager.get_term(command_name)
    assert command_term is not None, f"Command term '{command_name}' not found"
    cfg = cast(UniformVelocityCommandCfg, command_term.cfg)

    for stage in standing_stages:
        if env.common_step_counter > stage["step"]:
            cfg.rel_standing_envs = stage["rel_standing_envs"]

    return torch.tensor([cfg.rel_standing_envs])


class VelocityCommandCommandOnly(UniformVelocityCommand):
    """UniformVelocityCommand that only draws the commanded arrows in debug viz."""

    def _debug_vis_impl(self, visualizer: "DebugVisualizer") -> None:
        batch = visualizer.env_idx
        if batch >= self.num_envs:
            return

        cmds = self.command.cpu().numpy()
        base_pos_ws = self.robot.data.root_link_pos_w.cpu().numpy()
        base_quat_w = self.robot.data.root_link_quat_w
        base_mat_ws = matrix_from_quat(base_quat_w).cpu().numpy()

        base_pos_w = base_pos_ws[batch]
        base_mat_w = base_mat_ws[batch]
        cmd = cmds[batch]

        if np.linalg.norm(base_pos_w) < 1e-6:
            return

        def local_to_world(vec: np.ndarray) -> np.ndarray:
            return base_pos_w + base_mat_w @ vec

        scale = self.cfg.viz.scale * 2.0
        z_offset = self.cfg.viz.z_offset

        cmd_lin_from = local_to_world(np.array([0, 0, z_offset]) * scale)
        cmd_lin_to = local_to_world(
            (np.array([0, 0, z_offset]) + np.array([cmd[0], cmd[1], 0])) * scale
        )
        visualizer.add_arrow(cmd_lin_from, cmd_lin_to, color=(0.2, 0.2, 0.6, 0.6), width=0.015)


class VelocityCommandCommandOnlyCfg(UniformVelocityCommandCfg):
    def build(self, env: ManagerBasedRlEnv) -> "VelocityCommandCommandOnly":
        return VelocityCommandCommandOnly(self, env)
