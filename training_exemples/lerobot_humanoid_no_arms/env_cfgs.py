"""LeRobot Humanoid velocity environment configurations."""

import csv
from copy import deepcopy
import json
import math
import re
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any

from .lerobot_humanoid_no_arms_constants import (
  LEROBOT_HUMANOID_NO_ARMS_ACTION_SCALE,
  get_lerobot_humanoid_no_arms_robot_cfg,
)
from mjlab.actuator.delayed_actuator import DelayedActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.curriculum_manager import CurriculumTermCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.utils.noise import UniformNoiseCfg as Unoise
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg
import sys
import torch


_CSV_LOG_STATE_BY_ENV: dict[int, dict[str, Any]] = {}


class _SelectiveActionRateL2Penalty:
  """L2 action-rate penalty computed on a selected subset of action dimensions."""

  def __init__(self) -> None:
    self._state_by_env: dict[int, dict[str, Any]] = {}

  def _resolve_joint_indices(
    self,
    env,
    num_actions: int,
    joint_name_patterns: tuple[str, ...],
    device: torch.device,
  ) -> torch.Tensor:
    if num_actions <= 0:
      return torch.empty((0,), dtype=torch.long, device=device)

    asset = env.scene["robot"]
    candidate_name_lists = [
      ("actuator_names", getattr(asset, "actuator_names", None)),
      ("joint_names", getattr(asset, "joint_names", None)),
      ("dof_names", getattr(asset, "dof_names", None)),
    ]
    compiled_patterns = [re.compile(p) for p in joint_name_patterns]

    checked_sources: list[str] = []
    for source_name, names in candidate_name_lists:
      if names is None or len(names) != num_actions:
        size = "None" if names is None else str(len(names))
        checked_sources.append(f"{source_name}={size}")
        continue
      checked_sources.append(f"{source_name}={len(names)}")
      ids = [
        i
        for i, name in enumerate(names)
        if any(pattern.fullmatch(name) for pattern in compiled_patterns)
      ]
      if ids:
        return torch.tensor(ids, dtype=torch.long, device=device)
    raise ValueError(
      "No action dimensions matched joint_name_patterns="
      f"{joint_name_patterns} with num_actions={num_actions}. "
      f"Checked sources: {', '.join(checked_sources)}."
    )

  def __call__(
    self,
    env,
    joint_name_patterns: tuple[str, ...] = (r".*hipz.*", r".*hipx.*"),
  ) -> torch.Tensor:
    actions = env.action_manager.action
    prev_actions = env.action_manager.prev_action
    env_key = id(env)

    state = self._state_by_env.get(env_key)
    needs_init = (
      state is None
      or state["joint_ids"].device != actions.device
      or state["num_actions"] != actions.shape[1]
      or state["joint_name_patterns"] != joint_name_patterns
    )
    if needs_init:
      state = {
        "joint_ids": self._resolve_joint_indices(
          env=env,
          num_actions=actions.shape[1],
          joint_name_patterns=joint_name_patterns,
          device=actions.device,
        ),
        "num_actions": actions.shape[1],
        "joint_name_patterns": joint_name_patterns,
      }
      self._state_by_env[env_key] = state

    joint_ids = state["joint_ids"]

    if joint_ids.numel() == 0:
      return torch.zeros(actions.shape[0], device=actions.device, dtype=actions.dtype)
    return torch.sum(torch.square(actions[:, joint_ids] - prev_actions[:, joint_ids]), dim=1)

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    del env_ids


class _SelectiveJointLimitMarginPenalty:
  """Penalize selected joints when operating near hard joint limits."""

  def __init__(self) -> None:
    self._state_by_env: dict[int, dict[str, Any]] = {}

  def _resolve_joint_indices(
    self,
    env,
    joint_name_patterns: tuple[str, ...],
    device: torch.device,
  ) -> torch.Tensor:
    asset = env.scene["robot"]
    joint_names = getattr(asset, "joint_names", None)
    if joint_names is None:
      raise ValueError("Robot joint names are unavailable for limit-margin penalty.")
    compiled_patterns = [re.compile(p) for p in joint_name_patterns]
    ids = [
      i
      for i, name in enumerate(joint_names)
      if any(pattern.fullmatch(name) for pattern in compiled_patterns)
    ]
    if not ids:
      raise ValueError(
        f"No joints matched joint_name_patterns={joint_name_patterns} "
        f"in available joints={tuple(joint_names)}."
      )
    return torch.tensor(ids, dtype=torch.long, device=device)

  def __call__(
    self,
    env,
    joint_name_patterns: tuple[str, ...] = (r".*ankley.*", r".*anklex.*"),
    margin_ratio: float = 0.1,
  ) -> torch.Tensor:
    asset = env.scene["robot"]
    joint_pos = asset.data.joint_pos
    joint_pos_limits = asset.data.joint_pos_limits
    env_key = id(env)

    state = self._state_by_env.get(env_key)
    needs_init = (
      state is None
      or state["joint_ids"].device != joint_pos.device
      or state["num_joints"] != joint_pos.shape[1]
      or state["joint_name_patterns"] != joint_name_patterns
    )
    if needs_init:
      state = {
        "joint_ids": self._resolve_joint_indices(
          env=env,
          joint_name_patterns=joint_name_patterns,
          device=joint_pos.device,
        ),
        "num_joints": joint_pos.shape[1],
        "joint_name_patterns": joint_name_patterns,
      }
      self._state_by_env[env_key] = state

    joint_ids = state["joint_ids"]
    if joint_ids.numel() == 0:
      return torch.zeros(joint_pos.shape[0], device=joint_pos.device, dtype=joint_pos.dtype)

    pos = joint_pos[:, joint_ids]
    lower = joint_pos_limits[:, joint_ids, 0]
    upper = joint_pos_limits[:, joint_ids, 1]
    span = (upper - lower).clamp(min=1e-6)
    margin_ratio = float(max(0.0, min(margin_ratio, 0.49)))

    # Start penalizing inside a margin band near each hard bound.
    margin = margin_ratio * span
    lower_band = lower + margin
    upper_band = upper - margin
    near_lower = (lower_band - pos).clamp(min=0.0) / span
    near_upper = (pos - upper_band).clamp(min=0.0) / span
    return torch.sum(near_lower.square() + near_upper.square(), dim=1)

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    del env_ids


class _ActionFftBandRatioReward:
  """Reward the fraction of action spectral energy inside a low-frequency band."""

  def __init__(self) -> None:
    self._state_by_env: dict[int, dict[str, Any]] = {}
    self._last_env_key: int | None = None
    self._freq_masks_cache: dict[
      tuple[int, float, float, str], tuple[torch.Tensor, torch.Tensor]
    ] = {}

  def _get_freq_masks(
    self,
    history_len: int,
    step_dt: float,
    cutoff_hz: float,
    device: torch.device,
  ) -> tuple[torch.Tensor, torch.Tensor]:
    key = (history_len, round(step_dt, 8), cutoff_hz, str(device))
    masks = self._freq_masks_cache.get(key)
    if masks is not None:
      return masks
    freqs = torch.fft.rfftfreq(history_len, d=step_dt, device=device)
    valid_mask = freqs > 0.0
    inband_mask = (freqs <= cutoff_hz) & valid_mask
    masks = (inband_mask, valid_mask)
    self._freq_masks_cache[key] = masks
    return masks

  def __call__(
    self,
    env,
    history_len: int = 50,
    min_history: int = 50,
    cutoff_hz: float = 3.0,
  ) -> torch.Tensor:
    actions = env.action_manager.action
    if history_len < 4:
      raise ValueError(f"history_len must be >= 4 for FFT reward, got {history_len}")
    env_key = id(env)
    self._last_env_key = env_key

    state = self._state_by_env.get(env_key)
    needs_init = (
      state is None
      or state["buf"].shape[0] != actions.shape[0]
      or state["buf"].shape[2] != actions.shape[1]
      or state["buf"].shape[1] != history_len
    )
    if needs_init:
      state = {
        "buf": torch.zeros(
          (actions.shape[0], history_len, actions.shape[1]),
          device=actions.device,
          dtype=actions.dtype,
        ),
        "pos": 0,
        "count": torch.zeros(
          actions.shape[0], device=actions.device, dtype=torch.long
        ),
      }
      self._state_by_env[env_key] = state

    buf = state["buf"]
    pos = int(state["pos"])
    count = state["count"]

    buf[:, pos, :] = actions
    state["pos"] = (pos + 1) % history_len
    count.add_(1).clamp_(max=history_len)

    idx = torch.tensor(
      [((state["pos"] - history_len + i) % history_len) for i in range(history_len)],
      device=actions.device,
      dtype=torch.long,
    )
    hist = buf.index_select(1, idx)
    hist = hist - hist.mean(dim=1, keepdim=True)
    spec = torch.fft.rfft(hist, dim=1)
    power = spec.real.square() + spec.imag.square()
    inband_mask, valid_mask = self._get_freq_masks(
      history_len, float(env.step_dt), cutoff_hz, actions.device
    )
    if not bool(valid_mask.any()):
      reward = torch.ones(actions.shape[0], device=actions.device, dtype=actions.dtype)
    else:
      total_power = power[:, valid_mask, :].sum(dim=(1, 2))
      inband_power = power[:, inband_mask, :].sum(dim=(1, 2))
      reward = torch.where(
        total_power > 1e-12,
        inband_power / total_power,
        torch.ones_like(total_power),
      )

    if min_history > 0:
      ready = count >= min(min_history, history_len)
      reward = torch.where(ready, reward, torch.zeros_like(reward))

    return reward

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    if self._last_env_key is None:
      return
    state = self._state_by_env.get(self._last_env_key)
    if state is None:
      return
    buf = state["buf"]
    count = state["count"]
    if env_ids is None or isinstance(env_ids, slice):
      buf.zero_()
      count.zero_()
      state["pos"] = 0
      return
    buf[env_ids] = 0.0
    count[env_ids] = 0


def _find_right_left_foot_site_indices(env) -> tuple[int, int] | None:
  """Resolve world-space foot site indices as (right_idx, left_idx)."""
  asset = env.scene["robot"]
  site_names = getattr(asset, "site_names", None)
  if site_names:
    right_idx = None
    left_idx = None
    for i, site_name in enumerate(site_names):
      lower_name = site_name.lower()
      if right_idx is None and ("foot_right" in lower_name or "right_foot" in lower_name):
        right_idx = i
      if left_idx is None and ("foot_left" in lower_name or "left_foot" in lower_name):
        left_idx = i
    if right_idx is not None and left_idx is not None:
      return right_idx, left_idx
  # Fallback for legacy MJCF ordering (torso=0, foot_right=1, foot_left=2).
  # Keeping this fallback avoids silently zeroing the reward if site names change.
  if asset.data.site_pos_w.shape[1] >= 3:
    return 1, 2
  return None


class _BilateralSymmetryReward:
  """Penalize bilateral asymmetry from world-space foot trajectories.

  Uses foot-space metrics from docs/training-learnings.md:
  - y-lean (lateral midpoint bias),
  - max foot-height mismatch,
  - mean foot-height mismatch,
  - swing-range mismatch.
  """

  def __init__(self) -> None:
    self._state_by_env: dict[int, dict[str, Any]] = {}
    self._last_env_key: int | None = None

  def __call__(
    self,
    env,
    history_len: int = 60,
    min_history: int = 30,
    lateral_cmd_scale: float = 1.0,
  ) -> torch.Tensor:
    try:
      asset = env.scene["robot"]
      site_pos_w = asset.data.site_pos_w  # [N, num_sites, 3]
      N = site_pos_w.shape[0]
      env_key = id(env)
      self._last_env_key = env_key

      state = self._state_by_env.get(env_key)
      needs_init = (
        state is None
        or state["buf"].shape[0] != N
        or state["buf"].shape[1] != history_len
        or state["buf"].device != site_pos_w.device
        or state["buf"].dtype != site_pos_w.dtype
      )
      if needs_init:
        state = {
          "site_ids": _find_right_left_foot_site_indices(env),
          "buf": torch.zeros(
            (N, history_len, 2, 3),
            device=site_pos_w.device,
            dtype=site_pos_w.dtype,
          ),
          "pos": 0,
          "count": torch.zeros(N, device=site_pos_w.device, dtype=torch.long),
        }
        self._state_by_env[env_key] = state

      site_ids = state["site_ids"]
      if site_ids is None:
        return torch.zeros(N, device=site_pos_w.device, dtype=site_pos_w.dtype)
      right_idx, left_idx = site_ids
      if max(right_idx, left_idx) >= site_pos_w.shape[1]:
        return torch.zeros(N, device=site_pos_w.device, dtype=site_pos_w.dtype)

      right = site_pos_w[:, right_idx, :]
      left = site_pos_w[:, left_idx, :]

      buf = state["buf"]
      pos = int(state["pos"])
      count = state["count"]
      buf[:, pos, 0, :] = right
      buf[:, pos, 1, :] = left
      state["pos"] = (pos + 1) % history_len
      count.add_(1).clamp_(max=history_len)

      idx = torch.tensor(
        [((state["pos"] - history_len + i) % history_len) for i in range(history_len)],
        device=site_pos_w.device,
        dtype=torch.long,
      )
      hist = buf.index_select(1, idx)

      right_y = hist[:, :, 0, 1]
      left_y = hist[:, :, 1, 1]
      right_z = hist[:, :, 0, 2]
      left_z = hist[:, :, 1, 2]

      root_y = asset.data.root_link_pos_w[:, 1].unsqueeze(1)
      # y-lean: lateral midpoint of both feet should stay centered under the torso.
      y_lean = (0.5 * (right_y + left_y) - root_y).abs().mean(dim=1)

      # Foot-height symmetry metrics recommended in training learnings.
      max_height_diff = (right_z.max(dim=1).values - left_z.max(dim=1).values).abs()
      mean_height_diff = (right_z.mean(dim=1) - left_z.mean(dim=1)).abs()
      swing_right = right_z.max(dim=1).values - right_z.min(dim=1).values
      swing_left = left_z.max(dim=1).values - left_z.min(dim=1).values
      swing_range_diff = (swing_right - swing_left).abs()

      penalty = y_lean + max_height_diff + mean_height_diff + swing_range_diff

      if min_history > 0:
        ready = count >= min(min_history, history_len)
        penalty = torch.where(ready, penalty, torch.zeros_like(penalty))

      # During strong lateral commands, perfect left/right symmetry is not expected.
      # Keep the penalty active near vy=0 and fade it out toward |vy|=1.0 (G1 command range).
      cmd = env.command_manager.get_command("twist")
      if cmd is not None:
        forward_weight = (1.0 - cmd[:, 1].abs() / max(lateral_cmd_scale, 1e-6)).clamp(
          min=0.0
        )
        penalty = penalty * forward_weight

      return -penalty

    except Exception:
      traceback.print_exc()
      N = env.num_envs if hasattr(env, "num_envs") else env.action_manager.action.shape[0]
      return torch.zeros(N, device=env.action_manager.action.device)

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    if self._last_env_key is None:
      return
    state = self._state_by_env.get(self._last_env_key)
    if state is None:
      return
    buf = state["buf"]
    count = state["count"]
    if env_ids is None or isinstance(env_ids, slice):
      buf.zero_()
      count.zero_()
      state["pos"] = 0
      return
    buf[env_ids] = 0.0
    count[env_ids] = 0


class _FootLeadAlternationReward:
  """Encourage lead-foot alternation during forward locomotion.

  The reward combines:
  - a discrete bonus when the lead foot swaps in +x/-x,
  - a small dense bonus for maintaining non-zero fore-aft foot separation,
  - a penalty if the same foot stays in front for too many steps.
  """

  def __init__(self) -> None:
    self._state_by_env: dict[int, dict[str, Any]] = {}
    self._last_env_key: int | None = None

  def __call__(
    self,
    env,
    lead_margin: float = 0.02,
    target_separation: float = 0.08,
    max_same_lead_steps: int = 20,
    min_forward_cmd: float = 0.15,
    forward_cmd_full: float = 0.60,
    lateral_cmd_scale: float = 1.0,
  ) -> torch.Tensor:
    try:
      asset = env.scene["robot"]
      site_pos_w = asset.data.site_pos_w  # [N, num_sites, 3]
      N = site_pos_w.shape[0]
      env_key = id(env)
      self._last_env_key = env_key

      state = self._state_by_env.get(env_key)
      needs_init = (
        state is None
        or state["prev_lead_sign"].shape[0] != N
        or state["prev_lead_sign"].device != site_pos_w.device
      )
      if needs_init:
        state = {
          "site_ids": _find_right_left_foot_site_indices(env),
          "prev_lead_sign": torch.zeros(N, device=site_pos_w.device, dtype=torch.int8),
          "steps_same_lead": torch.zeros(N, device=site_pos_w.device, dtype=torch.long),
          "initialized": torch.zeros(N, device=site_pos_w.device, dtype=torch.bool),
        }
        self._state_by_env[env_key] = state

      site_ids = state["site_ids"]
      if site_ids is None:
        return torch.zeros(N, device=site_pos_w.device, dtype=site_pos_w.dtype)
      right_idx, left_idx = site_ids
      if max(right_idx, left_idx) >= site_pos_w.shape[1]:
        return torch.zeros(N, device=site_pos_w.device, dtype=site_pos_w.dtype)

      right_x = site_pos_w[:, right_idx, 0]
      left_x = site_pos_w[:, left_idx, 0]
      delta_x = right_x - left_x

      lead_sign = torch.zeros(N, device=site_pos_w.device, dtype=torch.int8)
      lead_sign = torch.where(delta_x > lead_margin, torch.ones_like(lead_sign), lead_sign)
      lead_sign = torch.where(delta_x < -lead_margin, -torch.ones_like(lead_sign), lead_sign)

      prev_lead_sign = state["prev_lead_sign"]
      initialized = state["initialized"]
      switched = initialized & (lead_sign != 0) & (prev_lead_sign != 0) & (lead_sign != prev_lead_sign)

      cmd = env.command_manager.get_command("twist")
      if cmd is not None:
        forward_mag = cmd[:, 0].abs()
        forward_ramp = max(float(forward_cmd_full) - float(min_forward_cmd), 1e-6)
        forward_gate = ((forward_mag - float(min_forward_cmd)) / forward_ramp).clamp(
          min=0.0, max=1.0
        )
        lateral_gate = (1.0 - cmd[:, 1].abs() / max(float(lateral_cmd_scale), 1e-6)).clamp(
          min=0.0
        )
        active = forward_gate * lateral_gate
      else:
        active = torch.ones(N, device=site_pos_w.device, dtype=site_pos_w.dtype)

      should_count = (lead_sign != 0) & (active > 0.0)
      steps_same_lead = state["steps_same_lead"]
      steps_same_lead = torch.where(
        should_count, steps_same_lead + 1, torch.zeros_like(steps_same_lead)
      )
      steps_same_lead = torch.where(switched, torch.zeros_like(steps_same_lead), steps_same_lead)
      state["steps_same_lead"] = steps_same_lead

      # Update remembered lead sign only when outside dead-zone.
      state["prev_lead_sign"] = torch.where(lead_sign != 0, lead_sign, prev_lead_sign)
      state["initialized"] = initialized | (lead_sign != 0)

      switch_bonus = switched.to(site_pos_w.dtype)
      sep_denom = max(float(target_separation) - float(lead_margin), 1e-6)
      separation_bonus = ((delta_x.abs() - float(lead_margin)) / sep_denom).clamp(
        min=0.0, max=1.0
      )
      stale_penalty = (
        (steps_same_lead.to(site_pos_w.dtype) - float(max_same_lead_steps))
        / max(float(max_same_lead_steps), 1.0)
      ).clamp(min=0.0, max=1.0)

      reward = (switch_bonus + 0.25 * separation_bonus - stale_penalty) * active
      return reward

    except Exception:
      traceback.print_exc()
      N = env.num_envs if hasattr(env, "num_envs") else env.action_manager.action.shape[0]
      return torch.zeros(N, device=env.action_manager.action.device)

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    if self._last_env_key is None:
      return
    state = self._state_by_env.get(self._last_env_key)
    if state is None:
      return
    prev_lead_sign = state["prev_lead_sign"]
    steps_same_lead = state["steps_same_lead"]
    initialized = state["initialized"]
    if env_ids is None or isinstance(env_ids, slice):
      prev_lead_sign.zero_()
      steps_same_lead.zero_()
      initialized.zero_()
      return
    prev_lead_sign[env_ids] = 0
    steps_same_lead[env_ids] = 0
    initialized[env_ids] = False


class _BilateralTorqueBalanceReward:
  """Penalize sustained left/right torque-energy imbalance between legs.

  Uses actuator torque squared (tau^2) as a proxy for effort and compares
  right-vs-left rolling averages over a short history window.
  """

  def __init__(self) -> None:
    self._state_by_env: dict[int, dict[str, Any]] = {}
    self._last_env_key: int | None = None

  def _resolve_leg_indices(
    self,
    env,
    num_actuators: int,
    device: torch.device,
  ) -> tuple[torch.Tensor, torch.Tensor]:
    asset = env.scene["robot"]
    candidate_name_lists = [
      ("actuator_names", getattr(asset, "actuator_names", None)),
      ("joint_names", getattr(asset, "joint_names", None)),
      ("dof_names", getattr(asset, "dof_names", None)),
    ]

    checked_sources: list[str] = []
    for source_name, names in candidate_name_lists:
      if names is None or len(names) != num_actuators:
        size = "None" if names is None else str(len(names))
        checked_sources.append(f"{source_name}={size}")
        continue
      checked_sources.append(f"{source_name}={len(names)}")
      right_ids: list[int] = []
      left_ids: list[int] = []
      for i, name in enumerate(names):
        lname = str(name).lower()
        if "_right" in lname or lname.endswith("right"):
          right_ids.append(i)
        elif "_left" in lname or lname.endswith("left"):
          left_ids.append(i)
      if right_ids and left_ids:
        return (
          torch.tensor(right_ids, dtype=torch.long, device=device),
          torch.tensor(left_ids, dtype=torch.long, device=device),
        )

    raise ValueError(
      "Could not resolve left/right actuator indices for torque balance. "
      f"num_actuators={num_actuators}, checked sources: {', '.join(checked_sources)}."
    )

  def __call__(
    self,
    env,
    history_len: int = 40,
    min_history: int = 20,
    epsilon: float = 1e-6,
  ) -> torch.Tensor:
    try:
      torques = env.scene["robot"].data.actuator_force
      if history_len < 1:
        raise ValueError(f"history_len must be >= 1, got {history_len}")
      env_key = id(env)
      self._last_env_key = env_key

      state = self._state_by_env.get(env_key)
      needs_init = (
        state is None
        or state["buf"].shape[0] != torques.shape[0]
        or state["buf"].shape[1] != history_len
        or state["buf"].device != torques.device
        or state["buf"].dtype != torques.dtype
        or state["num_actuators"] != torques.shape[1]
      )
      if needs_init:
        right_ids, left_ids = self._resolve_leg_indices(
          env=env,
          num_actuators=torques.shape[1],
          device=torques.device,
        )
        state = {
          "right_ids": right_ids,
          "left_ids": left_ids,
          "num_actuators": torques.shape[1],
          "buf": torch.zeros(
            (torques.shape[0], history_len, 2),
            device=torques.device,
            dtype=torques.dtype,
          ),
          "pos": 0,
          "count": torch.zeros(torques.shape[0], device=torques.device, dtype=torch.long),
        }
        self._state_by_env[env_key] = state

      right_ids = state["right_ids"]
      left_ids = state["left_ids"]
      if right_ids.numel() == 0 or left_ids.numel() == 0:
        return torch.zeros(torques.shape[0], device=torques.device, dtype=torques.dtype)
      if (
        int(right_ids.max().item()) >= torques.shape[1]
        or int(left_ids.max().item()) >= torques.shape[1]
      ):
        return torch.zeros(torques.shape[0], device=torques.device, dtype=torques.dtype)

      right_energy = torques[:, right_ids].square().sum(dim=1)
      left_energy = torques[:, left_ids].square().sum(dim=1)

      buf = state["buf"]
      pos = int(state["pos"])
      count = state["count"]
      buf[:, pos, 0] = right_energy
      buf[:, pos, 1] = left_energy
      state["pos"] = (pos + 1) % history_len
      count.add_(1).clamp_(max=history_len)

      idx = torch.tensor(
        [((state["pos"] - history_len + i) % history_len) for i in range(history_len)],
        device=torques.device,
        dtype=torch.long,
      )
      hist = buf.index_select(1, idx)
      denom = count.clamp(min=1).to(torques.dtype)
      right_avg = hist[:, :, 0].sum(dim=1) / denom
      left_avg = hist[:, :, 1].sum(dim=1) / denom
      eps = max(float(epsilon), 1e-12)
      imbalance = (right_avg - left_avg).abs() / (right_avg + left_avg + eps)

      if min_history > 0:
        ready = count >= min(min_history, history_len)
        imbalance = torch.where(ready, imbalance, torch.zeros_like(imbalance))

      # This term is a penalty; reward manager multiplies by a positive weight.
      return -imbalance

    except Exception:
      traceback.print_exc()
      N = env.num_envs if hasattr(env, "num_envs") else env.action_manager.action.shape[0]
      return torch.zeros(N, device=env.action_manager.action.device)

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    if self._last_env_key is None:
      return
    state = self._state_by_env.get(self._last_env_key)
    if state is None:
      return
    buf = state["buf"]
    count = state["count"]
    if env_ids is None or isinstance(env_ids, slice):
      buf.zero_()
      count.zero_()
      state["pos"] = 0
      return
    buf[env_ids] = 0.0
    count[env_ids] = 0


_ACTION_FFT_BAND_RATIO_REWARD = _ActionFftBandRatioReward()
_SELECTIVE_ACTION_RATE_L2_PENALTY = _SelectiveActionRateL2Penalty()
_SELECTIVE_JOINT_LIMIT_MARGIN_PENALTY = _SelectiveJointLimitMarginPenalty()
_BILATERAL_SYMMETRY_REWARD = _BilateralSymmetryReward()
_FOOT_LEAD_ALTERNATION_REWARD = _FootLeadAlternationReward()
_BILATERAL_TORQUE_BALANCE_REWARD = _BilateralTorqueBalanceReward()


def _flatten_obs_policy(
  obs_policy: torch.Tensor | dict[str, torch.Tensor],
  env_index: int,
) -> list[float]:
  """Flatten policy observation for one environment into a 1D list."""
  if isinstance(obs_policy, torch.Tensor):
    return obs_policy[env_index].detach().cpu().reshape(-1).tolist()

  flat_values: list[float] = []
  for term in obs_policy.values():
    flat_values.extend(term[env_index].detach().cpu().reshape(-1).tolist())
  return flat_values


def _log_obs_action_csv(
  env,
  env_ids=None,
  env_index: int = 0,
  csv_path: str | None = None,
) -> None:
  """Log policy observation and action to CSV at every environment step.

  Output format matches the real-robot debug CSV so sim and real logs can be
  compared directly:  time_s, step, observation (JSON), action_pre_scale (JSON).
  """
  del env_ids  # Unused for global interval events.
  if env_index < 0 or env_index >= env.num_envs:
    return

  obs_dict = env.observation_manager.compute(update_history=False)
  obs_policy = obs_dict.get("policy")
  if obs_policy is None:
    return

  obs_values = _flatten_obs_policy(obs_policy, env_index)
  action_values = (
    env.action_manager.action[env_index].detach().cpu().reshape(-1).tolist()
  )

  env_key = id(env)
  state = _CSV_LOG_STATE_BY_ENV.get(env_key)
  if state is None:
    if csv_path is None:
      stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
      csv_file = (
        Path("logs")
        / "csv"
        / f"lerobot_humanoid_no_arms_play_{stamp}.csv"
      )
    else:
      csv_file = Path(csv_path)
    csv_file.parent.mkdir(parents=True, exist_ok=True)
    state = {"path": csv_file, "header_written": False}
    _CSV_LOG_STATE_BY_ENV[env_key] = state
    print(f"[csv] Logging observations/actions to: {csv_file.resolve()}", flush=True)

  csv_file = state["path"]

  if not state["header_written"]:
    with csv_file.open("w", newline="") as f:
      csv.writer(f).writerow(["time_s", "step", "observation", "action_pre_scale"])
    state["header_written"] = True

  sim_time = float(env.sim.data.time.item() if hasattr(env.sim.data.time, "item") else env.sim.data.time[0])
  obs_json = json.dumps(obs_values, separators=(",", ":"))
  act_json = json.dumps(action_values, separators=(",", ":"))
  with csv_file.open("a", newline="") as f:
    csv.writer(f).writerow([f"{sim_time:.6f}", int(env.common_step_counter), obs_json, act_json])


def _joint_torques_obs(env) -> torch.Tensor:
  """Return actuator torques for all joints. Shape: [num_envs, 12]."""
  return env.scene["robot"].data.actuator_force


def _contact_dr_ranges_curriculum(
  env,
  env_ids,
  ranges_stages: list[dict[str, Any]],
) -> dict[str, torch.Tensor]:
  """Progressively widen contact DR ranges as training improves.

  This keeps early training easier (narrow DR around baseline), then broadens
  ranges for robustness once a gait is established.
  """

  del env_ids  # Global schedule based on common training step.
  step = int(env.common_step_counter)
  stage = ranges_stages[0]
  for candidate in ranges_stages:
    if step >= int(candidate["step"]):
      stage = candidate

  foot_friction = env.event_manager.get_term_cfg("foot_friction")
  foot_friction.params["ranges"] = tuple(stage["mu"])

  foot_torsional = env.event_manager.get_term_cfg("foot_friction_torsional")
  foot_torsional.params["ranges"] = {1: tuple(stage["torsional"])}

  foot_rolling = env.event_manager.get_term_cfg("foot_friction_rolling")
  foot_rolling.params["ranges"] = {2: tuple(stage["rolling"])}

  return {
    "mu_min": torch.tensor(stage["mu"][0], dtype=torch.float),
    "mu_max": torch.tensor(stage["mu"][1], dtype=torch.float),
    "torsional_min": torch.tensor(stage["torsional"][0], dtype=torch.float),
    "torsional_max": torch.tensor(stage["torsional"][1], dtype=torch.float),
    "rolling_min": torch.tensor(stage["rolling"][0], dtype=torch.float),
    "rolling_max": torch.tensor(stage["rolling"][1], dtype=torch.float),
  }


def _print_actuator_torques(env, env_ids=None) -> None:
  """Print mean/max actuator torque (absolute) for quick debugging during play."""
  asset = env.scene["robot"]
  torques = asset.data.actuator_force
  if env_ids is not None:
    torques = torques[env_ids]
  mean_abs = torques.abs().mean(dim=0).tolist()
  max_abs = torques.abs().max(dim=0).values.tolist()
  names = getattr(asset, "actuator_names", None)
  if names is None:
    names = [f"a{i}" for i in range(len(mean_abs))]
  rows = [
    f"{n}: mean {m:.2f} max {x:.2f}"
    for n, m, x in zip(names, mean_abs, max_abs, strict=False)
  ]
  print("[torque]\n  " + "\n  ".join(rows), flush=True)
  sys.stdout.flush()


def lerobot_humanoid_no_arms_rough_env_cfg(play: bool = False, torque_obs: bool = False) -> ManagerBasedRlEnvCfg:
  """Create LeRobot Humanoid rough terrain velocity configuration."""
  cfg = make_velocity_env_cfg()

  cfg.sim.mujoco.ccd_iterations = 500
  cfg.sim.contact_sensor_maxmatch = 500
  cfg.sim.nconmax = 100

  # Wrap all actuators with delay around the measured 1 control-tick latency
  # (20 ms on the real robot, physics dt = 5 ms -> 1 tick = 4 physics steps).
  # Keep delay mostly stable: per-step resampling behaves like high-frequency jitter,
  # which is less realistic than a slowly varying transport delay.
  # Use a moderate 5-35 ms range (1-7 physics steps) and update infrequently.
  robot_cfg = get_lerobot_humanoid_no_arms_robot_cfg()
  orig_artic = robot_cfg.articulation
  robot_cfg.articulation = EntityArticulationInfoCfg(
    actuators=tuple(
      DelayedActuatorCfg(
        base_cfg=a,
        delay_min_lag=1,
        delay_max_lag=7,
        delay_update_period=4,
        delay_hold_prob=0.95,
      )
      for a in orig_artic.actuators
    ),
    soft_joint_pos_limit_factor=orig_artic.soft_joint_pos_limit_factor,
  )
  cfg.scene.entities = {"robot": robot_cfg}

  site_names = ("foot_right", "foot_left")
  geom_names = ("left_foot_collision", "right_foot_collision")

  feet_ground_cfg = ContactSensorCfg(
    name="feet_ground_contact",
    primary=ContactMatch(
      mode="subtree",
      pattern=r"^(foot_subassembly|foot_subassembly_2)$",
      entity="robot",
    ),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
    track_air_time=True,
  )
  self_collision_cfg = ContactSensorCfg(
    name="self_collision",
    primary=ContactMatch(mode="subtree", pattern="torso_subassembly", entity="robot"),
    secondary=ContactMatch(mode="subtree", pattern="torso_subassembly", entity="robot"),
    fields=("found",),
    reduce="none",
    num_slots=1,
  )
  cfg.scene.sensors = (feet_ground_cfg, self_collision_cfg)

  if cfg.scene.terrain is not None and cfg.scene.terrain.terrain_generator is not None:
    cfg.scene.terrain.terrain_generator.curriculum = True
    cfg.scene.terrain.terrain_generator.difficulty_range = (0.0, 0.3)
    if hasattr(cfg.scene.terrain, "max_init_terrain_level"):
      cfg.scene.terrain.max_init_terrain_level = 2

  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = dict(LEROBOT_HUMANOID_NO_ARMS_ACTION_SCALE)
  # Keep ankle command authority narrower than hips/knees (f86d8bc behavior).
  for joint_name in ("ankley_right", "ankley_left"):
    joint_pos_action.scale[joint_name] = 0.22
  for joint_name in ("anklex_right", "anklex_left"):
    joint_pos_action.scale[joint_name] = 0.18

  cfg.viewer.body_name = "torso_subassembly"

  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  twist_cmd.viz.z_offset = 0.9  # Adjust based on robot height.
  # Use the full command envelope from the beginning (no velocity curriculum).
  twist_cmd.ranges.lin_vel_x = (-1.0, 1.0)
  twist_cmd.ranges.lin_vel_y = (-1.0, 1.0)
  twist_cmd.ranges.ang_vel_z = (-0.5, 0.5)
  cfg.curriculum.pop("command_vel", None)

  # Stronger observation randomization for sim-to-real robustness.
  policy_obs = cfg.observations["policy"]
  # Base velocity cfg builds critic terms from policy terms; clone policy terms that
  # we customize so policy-only history settings do not leak into critic.
  for term_name in ("projected_gravity", "joint_pos", "joint_vel", "actions"):
    if term_name in policy_obs.terms:
      policy_obs.terms[term_name] = deepcopy(policy_obs.terms[term_name])
  base_lin_vel_term = policy_obs.terms.get("base_lin_vel")
  if base_lin_vel_term is not None and getattr(base_lin_vel_term, "noise", None) is not None:
    base_lin_vel_term.noise.n_min = -0.075
    base_lin_vel_term.noise.n_max = 0.075
  policy_obs.terms.pop("base_lin_vel", None)
  policy_obs.terms.pop("base_ang_vel", None)
  projected_gravity_term = policy_obs.terms.get("projected_gravity")
  projected_gravity_term.noise.n_min = -0.012
  projected_gravity_term.noise.n_max = 0.012
  projected_gravity_term.history_length = 0
  projected_gravity_term.flatten_history_dim = True
  joint_pos_term = policy_obs.terms.get("joint_pos")
  if joint_pos_term is not None and getattr(joint_pos_term, "noise", None) is not None:
    # Keep position corruption present but tighter to match runtime behavior.
    joint_pos_noise_rad = math.radians(1.0)
    joint_pos_term.noise.n_min = -joint_pos_noise_rad
    joint_pos_term.noise.n_max = joint_pos_noise_rad
  joint_vel_term = policy_obs.terms.get("joint_vel")
  # Real hardware finite-difference velocity is noisy; use a slightly wider
  # envelope to cover estimator spikes seen in logs.
  joint_vel_noise_rad_s = 20.0 * math.pi / 180.0
  joint_vel_term.noise.n_min = -joint_vel_noise_rad_s
  joint_vel_term.noise.n_max = joint_vel_noise_rad_s
  joint_vel_term.history_length = 0
  joint_vel_term.flatten_history_dim = True
  actions_term = policy_obs.terms.get("actions")
  if actions_term is not None:
    actions_term.history_length = 0
    actions_term.flatten_history_dim = True
  # Joint torques with noise (sim2real: real actuators have torque measurement noise).
  if torque_obs:
    policy_obs.terms["joint_torques"] = ObservationTermCfg(
      func=_joint_torques_obs,
      noise=Unoise(n_min=-0.05, n_max=0.05),  # ±0.05 Nm — matches real torque readout noise
      scale=1.0 / 88.0,  # Normalize by a representative effort limit.
    )

  cfg.observations["critic"].terms["foot_height"].params[
    "asset_cfg"
  ].site_names = site_names
  # Keep critic and actor consistent: base_ang_vel removed from actor for sim2real.
  cfg.observations["critic"].terms.pop("base_ang_vel", None)
  # Widen startup encoder bias randomization a bit to better cover real
  # zero-offset drift while remaining in a plausible range.
  if "encoder_bias" in cfg.events:
    cfg.events["encoder_bias"].params["bias_range"] = (-0.02, 0.02)

  # ---------------------------------------------------------------------------
  # Domain Randomization: Contact Parameters
  # ---------------------------------------------------------------------------
  cfg.events["foot_friction"].params["asset_cfg"].geom_names = geom_names
  # Randomize around measured hardware slip behavior while keeping a higher
  # upper bound for robustness to grippier real terrains/materials.
  cfg.events["foot_friction"].params["ranges"] = (0.52, 0.75)  # geom_friction[0]
  # Keep feet independent to model real left/right contact asymmetries.
  cfg.events["foot_friction"].params["shared_random"] = False
  cfg.events["foot_friction"].mode = "reset"
  cfg.events["foot_friction"].domain_randomization = True
  cfg.events["foot_friction_torsional"] = EventTermCfg(
    func=envs_mdp.randomize_field,
    mode="reset",
    domain_randomization=True,
    params={
      "field": "geom_friction",
      "ranges": {1: (0.004, 0.009)},  # geom_friction[1]
      "operation": "abs",
      "asset_cfg": SceneEntityCfg("robot", geom_names=geom_names),
      "shared_random": False,
    },
  )
  cfg.events["foot_friction_rolling"] = EventTermCfg(
    func=envs_mdp.randomize_field,
    mode="reset",
    domain_randomization=True,
    params={
      "field": "geom_friction",
      "ranges": {2: (0.00008, 0.00025)},  # geom_friction[2]
      "operation": "abs",
      "asset_cfg": SceneEntityCfg("robot", geom_names=geom_names),
      "shared_random": False,
    },
  )

  cfg.curriculum["contact_dr_ranges"] = CurriculumTermCfg(
    func=_contact_dr_ranges_curriculum,
    params={
      "ranges_stages": [
        {
          "step": 0,
          "mu": (0.52, 0.75),
          "torsional": (0.004, 0.009),
          "rolling": (0.00008, 0.00025),
        },
        {
          "step": 7_500 * 24,
          "mu": (0.48, 0.84),
          "torsional": (0.0035, 0.0105),
          "rolling": (0.00006, 0.00033),
        },
        {
          "step": 15_000 * 24,
          "mu": (0.45, 0.90),
          "torsional": (0.003, 0.012),
          "rolling": (0.00005, 0.00040),
        },
      ]
    },
  )

  # ---------------------------------------------------------------------------
  # Domain Randomization: Joint Dynamics
  # ---------------------------------------------------------------------------
  joint_groups = {
    "hipz": ("hipz_right", "hipz_left"),
    "hipx": ("hipx_right", "hipx_left"),
    "hipy": ("hipy_right", "hipy_left"),
    "knee": ("knee_right", "knee_left"),
    "ankley": ("ankley_right", "ankley_left"),
    "anklex": ("anklex_right", "anklex_left"),
  }
  joint_armature_scales = {
    "hipz": (0.80, 1.20),
    "hipx": (0.80, 1.20),
    "hipy": (0.80, 1.20),
    "knee": (0.80, 1.20),
    "ankley": (0.80, 1.20),
    "anklex": (0.80, 1.20),
  }
  joint_damping_scales = {
    "hipz": (0.80, 1.20),
    "hipx": (0.80, 1.20),
    "hipy": (0.80, 1.20),
    "knee": (0.80, 1.20),
    "ankley": (0.80, 1.20),
    "anklex": (0.80, 1.20),
  }
  # "Static friction" approximation at the joint level (MuJoCo frictionloss).
  joint_frictionloss_scales = {
    "hipz": (0.80, 1.20),
    "hipx": (0.80, 1.20),
    "hipy": (0.80, 1.20),
    "knee": (0.80, 1.20),
    "ankley": (0.80, 1.20),
    "anklex": (0.80, 1.20),
  }
  for group_name, joint_names in joint_groups.items():
    asset_cfg = SceneEntityCfg("robot", joint_names=list(joint_names))
    cfg.events[f"joint_armature_{group_name}"] = EventTermCfg(
      func=envs_mdp.randomize_field,
      mode="startup",
      domain_randomization=True,
      params={
        "field": "dof_armature",
        "ranges": joint_armature_scales[group_name],
        "operation": "scale",
        "asset_cfg": asset_cfg,
      },
    )
    cfg.events[f"joint_damping_{group_name}"] = EventTermCfg(
      func=envs_mdp.randomize_field,
      mode="startup",
      domain_randomization=True,
      params={
        "field": "dof_damping",
        "ranges": joint_damping_scales[group_name],
        "operation": "scale",
        "asset_cfg": asset_cfg,
      },
    )
    cfg.events[f"joint_frictionloss_{group_name}"] = EventTermCfg(
      func=envs_mdp.randomize_field,
      mode="startup",
      domain_randomization=True,
      params={
        "field": "dof_frictionloss",
        "ranges": joint_frictionloss_scales[group_name],
        "operation": "scale",
        "asset_cfg": asset_cfg,
      },
    )

  # ---------------------------------------------------------------------------
  # Domain Randomization: Body Inertial Parameters (Torso + Limbs)
  # ---------------------------------------------------------------------------
  dr_body_names = (
    "torso_subassembly",
    "hipx_subassembly",
    "hipy_subassembly",
    "hipx_subassemby_sym",
    "hipy_subassembly_sym",
    "tigh_subassembly",
    "tigh_subassembly_sym",
    "shin_subassembly",
    "shin_subassembly_sym",
    "ankle_subassembly",
    "ankle_subassembly_2",
    "foot_subassembly",
    "foot_subassembly_2",
  )

  # COM: +/- 3 cm per body on x/y/z.
  cfg.events["base_com"].params["asset_cfg"].body_names = dr_body_names
  cfg.events["base_com"].params["ranges"] = {
    0: (-0.03, 0.03),
    1: (-0.03, 0.03),
    2: (-0.03, 0.03),
  }
  cfg.events["base_com"].domain_randomization = True

  # Mass: +/- 20% per body.
  cfg.events["body_mass"] = EventTermCfg(
    func=envs_mdp.randomize_field,
    mode="startup",
    domain_randomization=True,
    params={
      "field": "body_mass",
      "ranges": (0.80, 1.20),
      "operation": "scale",
      "asset_cfg": SceneEntityCfg("robot", body_names=dr_body_names),
    },
  )
  cfg.events["body_inertia"] = EventTermCfg(
    func=envs_mdp.randomize_field,
    mode="startup",
    domain_randomization=True,
    params={
      "field": "body_inertia",
      "ranges": (0.80, 1.20),
      "operation": "scale",
      "asset_cfg": SceneEntityCfg("robot", body_names=dr_body_names),
    },
  )

  # Remove legacy no-arms-specific DR terms that were tied to older pipelines.
  for event_name in (
    "robot_weight",
    "joint_coulomb_friction",
    "joint_viscous_friction",
    "joint_armature",
  ):
    cfg.events.pop(event_name, None)

  # Pose reward std values for the 12-DOF humanoid.
  # The variable_posture reward targets the robot default joint pose
  # (asset.data.default_joint_pos), which comes from KNEES_BENT_KEYFRAME.
  # Hip joints get more freedom, ankle roll is tight for balance.
  cfg.rewards["pose"].params["std_standing"] = {
    # Lower body - 12 DOF.
    # Tighter hipz/hipx stds encourage stillness when command velocity is zero.
    r".*hipy.*": 0.8,
    r".*hipx.*": 0.05,
    r".*hipz.*": 0.05,
    r".*knee.*": 0.8,
    r".*ankley.*": 0.35,
    r".*anklex.*": 0.2,
  }

  cfg.rewards["pose"].params["std_walking"] = {
    # Lower body - 12 DOF.
    r".*hipy.*": 0.8,
    r".*hipx.*": 0.1,
    r".*hipz.*": 0.1,
    r".*knee.*": 0.8,
    r".*ankley.*": 0.35,
    r".*anklex.*": 0.2,
  }

  cfg.rewards["pose"].params["std_running"] = {
    # Lower body - 12 DOF.
    r".*hipy.*": 0.8,
    r".*hipx.*": 0.1,
    r".*hipz.*": 0.1,
    r".*knee.*": 0.8,
    r".*ankley.*": 0.2,
    r".*anklex.*": 0.1,
  }
  # Keep pose regularization meaningful for no-arms while allowing stepping.
  cfg.rewards["pose"].weight = 1.5

  cfg.rewards["upright"].params["asset_cfg"].body_names = ("torso_subassembly",)
  cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("torso_subassembly",)
  cfg.rewards["upright"].weight = 1.0

  for reward_name in ["foot_clearance", "foot_swing_height", "foot_slip"]:
    cfg.rewards[reward_name].params["asset_cfg"].site_names = site_names

  # Slightly increase task-tracking priority.
  cfg.rewards["track_linear_velocity"].weight = 2.2
  cfg.rewards["track_angular_velocity"].weight = 2.2

  cfg.rewards["body_ang_vel"].weight = -0.05
  # Replace default soft-limit crossing cost with an explicit near-limit cost.
  # This starts penalizing in a margin band before hard bounds (not only after
  # crossing a soft-limit threshold), which better discourages boundary-seeking.
  cfg.rewards["dof_pos_limits"] = RewardTermCfg(
    func=_SELECTIVE_JOINT_LIMIT_MARGIN_PENALTY,
    weight=-1.5,
    params={
      "joint_name_patterns": (r".*",),
      "margin_ratio": 0.1,
    },
  )
  cfg.rewards["angular_momentum"].weight = -0.02
  cfg.rewards["air_time"].weight = 1.5
  cfg.rewards["foot_slip"].weight = -0.1
  cfg.rewards["soft_landing"].weight = -1e-5

  cfg.rewards["action_fft_band_le_3hz_ratio"] = RewardTermCfg(
    func=_ACTION_FFT_BAND_RATIO_REWARD,
    weight=1.5,
    params={"history_len": 50, "min_history": 50, "cutoff_hz": 2.5},
  )

  cfg.rewards["bilateral_symmetry"] = RewardTermCfg(
    func=_BILATERAL_SYMMETRY_REWARD,
    # Keep this low at the beginning: too-strong symmetry early in training tends
    # to collapse exploration into one-leg standing instead of learning to walk.
    weight=0.2,
    params={
      "history_len": 60,
      "min_history": 30,
      # Must match twist_cmd.ranges.lin_vel_y magnitude above.
      "lateral_cmd_scale": 1.0,
    },
  )
  cfg.rewards["bilateral_torque_balance"] = RewardTermCfg(
    func=_BILATERAL_TORQUE_BALANCE_REWARD,
    weight=0.0,
    params={
      # Smooth over roughly one gait sub-phase to avoid penalizing normal
      # stance/swing alternation at each single timestep.
      "history_len": 40,
      "min_history": 20,
      "epsilon": 1e-6,
    },
  )
  cfg.rewards["action_rate_l2"].weight = -0.1
  # Additional smoothness/effort regularizers (kept conservative and ramped via
  # curriculum below so they do not block early task learning).
  cfg.rewards["action_acc_l2"] = RewardTermCfg(func=mdp.action_acc_l2, weight=-0.03)
  cfg.rewards["joint_vel_l2"] = RewardTermCfg(func=mdp.joint_vel_l2, weight=-1e-3)
  cfg.rewards["joint_acc_l2"] = RewardTermCfg(func=mdp.joint_acc_l2, weight=-2e-7)
  cfg.rewards["joint_torques_l2"] = RewardTermCfg(
    func=mdp.joint_torques_l2, weight=-2e-5
  )
  cfg.rewards["action_rate_hipz_hipx_l2"] = RewardTermCfg(
    func=_SELECTIVE_ACTION_RATE_L2_PENALTY,
    weight=-0.5,
    params={"joint_name_patterns": (r".*hipz.*", r".*hipx.*")},
  )
  cfg.curriculum.pop("pose_weight", None)
  cfg.curriculum["action_rate_weight"] = CurriculumTermCfg(
    func=mdp.reward_weight,
    params={
      "reward_name": "action_rate_l2",
      "weight_stages": [
        {"step": 0, "weight": -0.1},
        # Curriculum uses env.common_step_counter (env steps), while W&B "Step"
        # is PPO iterations. Here num_steps_per_env=24, so multiply by 24.
        {"step": 5_000 * 24, "weight": -0.5},
        {"step": 10_000 * 24, "weight": -1.5},
        {"step": 15_000 * 24, "weight": -3.0},
      ],
    },
  )
  cfg.curriculum["action_acc_weight"] = CurriculumTermCfg(
    func=mdp.reward_weight,
    params={
      "reward_name": "action_acc_l2",
      "weight_stages": [
        {"step": 0, "weight": 0.0},
        {"step": 5_000 * 24, "weight": -0.01},
        {"step": 10_000 * 24, "weight": -0.02},
        {"step": 15_000 * 24, "weight": -0.03},
      ],
    },
  )
  cfg.curriculum["joint_vel_l2_weight"] = CurriculumTermCfg(
    func=mdp.reward_weight,
    params={
      "reward_name": "joint_vel_l2",
      "weight_stages": [
        {"step": 0, "weight": 0.0},
        {"step": 5_000 * 24, "weight": -3e-4},
        {"step": 10_000 * 24, "weight": -6e-4},
        {"step": 15_000 * 24, "weight": -1e-3},
      ],
    },
  )
  cfg.curriculum["joint_acc_l2_weight"] = CurriculumTermCfg(
    func=mdp.reward_weight,
    params={
      "reward_name": "joint_acc_l2",
      "weight_stages": [
        {"step": 0, "weight": 0.0},
        {"step": 5_000 * 24, "weight": -5e-8},
        {"step": 10_000 * 24, "weight": -1e-7},
        {"step": 15_000 * 24, "weight": -2e-7},
      ],
    },
  )
  cfg.curriculum["joint_torques_l2_weight"] = CurriculumTermCfg(
    func=mdp.reward_weight,
    params={
      "reward_name": "joint_torques_l2",
      "weight_stages": [
        {"step": 0, "weight": 0.0},
        {"step": 5_000 * 24, "weight": -5e-6},
        {"step": 10_000 * 24, "weight": -1e-5},
        {"step": 15_000 * 24, "weight": -2e-5},
      ],
    },
  )
  cfg.curriculum["bilateral_symmetry_weight"] = CurriculumTermCfg(
    func=mdp.reward_weight,
    params={
      "reward_name": "bilateral_symmetry",
      "weight_stages": [
        {"step": 0, "weight": 0.2},
        # Ramp symmetry only after gait emerges; this avoids over-constraining
        # early exploration while still enforcing cleaner, more symmetric walking later.
        {"step": 7_500 * 24, "weight": 0.35},
        {"step": 12_500 * 24, "weight": 0.5},
        {"step": 17_500 * 24, "weight": 0.7},
      ],
    },
  )
  cfg.curriculum["bilateral_torque_balance_weight"] = CurriculumTermCfg(
    func=mdp.reward_weight,
    params={
      "reward_name": "bilateral_torque_balance",
      "weight_stages": [
        {"step": 0, "weight": 0.0},
        {"step": 7_500 * 24, "weight": 0.05},
        {"step": 12_500 * 24, "weight": 0.10},
        {"step": 17_500 * 24, "weight": 0.15},
      ],
    },
  )
  cfg.rewards["self_collisions"] = RewardTermCfg(
    func=mdp.self_collision_cost,
    weight=-1.0,
    params={"sensor_name": self_collision_cfg.name},
  )
  # Terrain-side stiff contact settings to limit foot penetration.
  # Keep a fixed friction triplet to reduce contact jitter with mesh feet.
  cfg.scene.terrain.friction = "1.0 0.005 0.0001"
  cfg.scene.terrain.solref = "0.005 1"
  cfg.scene.terrain.solimp = "0.995 0.9995 0.001 0.5 2"
  cfg.scene.terrain.contact = "enable"
  # Apply play mode overrides.
  if play:
    # Effectively infinite episode length.
    cfg.episode_length_s = int(1e9)

    cfg.viewer.azimuth = 135.0
    cfg.viewer.elevation = -20.0

    cfg.observations["policy"].enable_corruption = False
    cfg.events.pop("push_robot", None)
    cfg.curriculum.pop("push_recovery", None)
    # Keep play mode diagnostic and repeatable: disable domain-randomized events.
    for event_name in list(cfg.events.keys()):
      if event_name in ("reset_base", "reset_robot_joints"):
        continue
      event_cfg = cfg.events[event_name]
      if getattr(event_cfg, "domain_randomization", False):
        cfg.events.pop(event_name, None)
    # Encoder bias is a startup perturbation (not tagged as domain_randomization).
    cfg.events.pop("encoder_bias", None)
    # Remove DR curriculum terms tied to removed events.
    cfg.curriculum.pop("contact_dr_ranges", None)

    if cfg.scene.terrain is not None:
      if cfg.scene.terrain.terrain_generator is not None:
        cfg.scene.terrain.terrain_generator.curriculum = False
        cfg.scene.terrain.terrain_generator.num_cols = 5
        cfg.scene.terrain.terrain_generator.num_rows = 5
        cfg.scene.terrain.terrain_generator.border_width = 10.0

    # cfg.events["log_obs_action_csv"] = EventTermCfg(
    #   func=_log_obs_action_csv,
    #   mode="interval",
    #   interval_range_s=(0.0, 0.0),
    #   is_global_time=True,
    #   params={"env_index": 0},
    # )

  return cfg


def lerobot_humanoid_no_arms_flat_env_cfg(play: bool = False, torque_obs: bool = False) -> ManagerBasedRlEnvCfg:
  """Create LeRobot Humanoid flat terrain velocity configuration."""
  cfg = lerobot_humanoid_no_arms_rough_env_cfg(play=play, torque_obs=torque_obs)

  cfg.sim.njmax = 300
  cfg.sim.mujoco.ccd_iterations = 50
  cfg.sim.contact_sensor_maxmatch = 64
  cfg.sim.nconmax = None

  # Switch to flat terrain.
  assert cfg.scene.terrain is not None
  cfg.scene.terrain.terrain_type = "plane"
  cfg.scene.terrain.terrain_generator = None

  # Disable terrain curriculum.
  assert "terrain_levels" in cfg.curriculum
  del cfg.curriculum["terrain_levels"]

  if play:
    twist_cmd = cfg.commands["twist"]
    assert isinstance(twist_cmd, UniformVelocityCommandCfg)
    twist_cmd.ranges.lin_vel_x = (0.3, 0.8)
    twist_cmd.ranges.lin_vel_y = (0.0, 0.0)
    twist_cmd.ranges.ang_vel_z = (0.0, 0.0)

    # cfg.events["print_actuator_torques"] = EventTermCfg(
    #   func=_print_actuator_torques,
    #   mode="interval",
    #   interval_range_s=(0.5, 0.5),
    #   is_global_time=True,
    # )

  return cfg
