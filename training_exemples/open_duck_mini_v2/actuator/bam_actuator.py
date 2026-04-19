"""BAM M6 actuator model for MuJoCo Warp.

Implements the full BAM (Better Actuator Model) M6 friction model inside mjlab's
actuator framework. This replaces MuJoCo's built-in kp+damping+frictionloss with:

  - XL330 firmware voltage control law (position error → duty cycle → voltage)
  - DC motor torque equation (voltage → torque, with back-EMF)
  - M6 load-dependent friction: Coulomb + Stribeck + motor-load + external-load + quadratic

Reference: Duclusaud et al., "Extended Friction Models for the Physics Simulation
of Servo Actuators", 2024. https://arxiv.org/abs/2410.08650
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import mujoco
import mujoco_warp as mjwarp
import torch

from mjlab.actuator.actuator import Actuator, ActuatorCfg, ActuatorCmd
from mjlab.utils.spec import create_motor_actuator

if TYPE_CHECKING:
    from mjlab.entity import Entity


@dataclass(kw_only=True)
class BamM6ActuatorCfg(ActuatorCfg):
    """Configuration for a BAM M6 actuator (XL330)."""

    # --- Electrical parameters ---
    kt: float
    """Motor torque constant [Nm/A]."""
    R: float
    """Motor winding resistance [Ohm]."""

    # --- Firmware control law ---
    vin: float = 7.4
    """Supply voltage [V]."""
    kp_fw: float = 200.0
    """Firmware position P gain (register value)."""
    error_gain: float = (4096 / (2 * math.pi)) / (256 * 885)
    """XL330 firmware scaling: (encoder_counts/2pi) / (KP_divisor * PWM_limit)."""
    max_pwm: float = 1.0
    """Maximum duty cycle (clipped)."""

    # --- M6 friction parameters ---
    friction_base: float = 0.0
    """Coulomb friction [Nm]."""
    friction_stribeck: float = 0.0
    """Stribeck friction component [Nm]."""
    dtheta_stribeck: float = 0.1
    """Stribeck velocity threshold [rad/s]."""
    alpha: float = 2.0
    """Stribeck curvature exponent."""
    friction_viscous: float = 0.0
    """Viscous friction coefficient [Nm·s/rad]."""
    load_friction_motor: float = 0.0
    """Friction proportional to motor torque."""
    load_friction_external: float = 0.0
    """Friction proportional to external (gravity) torque."""
    load_friction_motor_stribeck: float = 0.0
    """Stribeck component of motor-load friction."""
    load_friction_external_stribeck: float = 0.0
    """Stribeck component of external-load friction."""
    load_friction_motor_quad: float = 0.0
    """Quadratic motor-load friction."""
    load_friction_external_quad: float = 0.0
    """Quadratic external-load friction."""

    def build(
        self, entity: Entity, target_ids: list[int], target_names: list[str]
    ) -> BamM6Actuator:
        return BamM6Actuator(self, entity, target_ids, target_names)


class BamM6Actuator(Actuator):
    """BAM M6 actuator: full voltage control + load-dependent friction."""

    def __init__(
        self,
        cfg: BamM6ActuatorCfg,
        entity: Entity,
        target_ids: list[int],
        target_names: list[str],
    ) -> None:
        super().__init__(entity, target_ids, target_names)
        self.cfg = cfg
        self._model: mjwarp.Model | None = None
        self._data: mjwarp.Data | None = None
        self._dt: float = 0.0
        self._dof_ids: torch.Tensor | None = None
        # Per-env gain tensors (initialized in initialize(), randomized by DR)
        self.kp_scale: torch.Tensor | None = None
        self.kd_scale: torch.Tensor | None = None
        self.default_kp_scale: torch.Tensor | None = None
        self.default_kd_scale: torch.Tensor | None = None

    def edit_spec(self, spec: mujoco.MjSpec, target_names: list[str]) -> None:
        """Convert existing XML position actuators to motor (torque) mode and
        zero out joint friction. We handle all friction ourselves.
        """
        target_set = set(target_names)
        force_limit = self.cfg.vin * self.cfg.kt / self.cfg.R

        # Convert existing actuators on our joints from <position> to <motor>
        converted = set()
        for act in spec.actuators:
            tgt = act.target
            tgt_name = tgt.name if hasattr(tgt, "name") else str(tgt) if tgt else None
            if tgt_name in target_set:
                act.set_to_motor()
                act.forcelimited = True
                act.forcerange = (-force_limit, force_limit)
                act.gear = [1.0, 0, 0, 0, 0, 0]
                # Set armature on the joint via the spec
                for joint in spec.joints:
                    if joint.name == tgt_name:
                        joint.armature = self.cfg.armature
                        joint.damping = 0.0
                        joint.frictionloss = 0.0
                        break
                self._mjs_actuators.append(act)
                converted.add(tgt_name)

        # For any targets not already in the XML, create new motor actuators
        for target_name in target_names:
            if target_name not in converted:
                actuator = create_motor_actuator(
                    spec,
                    target_name,
                    effort_limit=force_limit,
                    armature=self.cfg.armature,
                    frictionloss=0.0,
                    transmission_type=self.cfg.transmission_type,
                )
                self._mjs_actuators.append(actuator)
                for joint in spec.joints:
                    if joint.name == target_name:
                        joint.damping = 0.0
                        joint.frictionloss = 0.0
                        break

    def initialize(
        self,
        mj_model: mujoco.MjModel,
        model: mjwarp.Model,
        data: mjwarp.Data,
        device: str,
    ) -> None:
        super().initialize(mj_model, model, data, device)
        self._model = model
        self._data = data
        self._dt = mj_model.opt.timestep
        self._device = device

        # Map from target (joint) ids to DOF ids in the full qvel vector.
        # For 1-DOF hinge joints, jnt_dofadr[joint_id] gives the qvel index.
        jnt_dofadr = mj_model.jnt_dofadr
        entity_joint_ids = self.entity.indexing.joint_ids
        dof_ids = []
        for tid in self._joint_ids_list:
            global_joint_id = entity_joint_ids[tid].item()
            dof_ids.append(jnt_dofadr[global_joint_id])
        self._dof_ids = torch.tensor(dof_ids, dtype=torch.long, device=device)

        # Per-env gain scaling factors (1.0 = nominal, randomized by DR)
        num_envs = data.nworld
        self.kp_scale = torch.ones(num_envs, 1, dtype=torch.float, device=device)
        self.kd_scale = torch.ones(num_envs, 1, dtype=torch.float, device=device)
        self.default_kp_scale = self.kp_scale.clone()
        self.default_kd_scale = self.kd_scale.clone()

    def set_gains(
        self,
        env_ids: torch.Tensor | slice,
        kp_scale: torch.Tensor | None = None,
        kd_scale: torch.Tensor | None = None,
    ) -> None:
        """Set per-env gain scaling factors.

        Args:
            env_ids: Environment indices to update.
            kp_scale: Multiplicative scale for kp_fw (shape: (len(env_ids), 1)).
            kd_scale: Multiplicative scale for back-EMF damping (shape: (len(env_ids), 1)).
        """
        if kp_scale is not None:
            assert self.kp_scale is not None
            self.kp_scale[env_ids] = kp_scale
        if kd_scale is not None:
            assert self.kd_scale is not None
            self.kd_scale[env_ids] = kd_scale

    def reset_gains(self, env_ids: torch.Tensor | slice) -> None:
        """Reset gains to nominal for specified environments."""
        assert self.kp_scale is not None and self.default_kp_scale is not None
        assert self.kd_scale is not None and self.default_kd_scale is not None
        self.kp_scale[env_ids] = self.default_kp_scale[env_ids]
        self.kd_scale[env_ids] = self.default_kd_scale[env_ids]

    def compute(self, cmd: ActuatorCmd) -> torch.Tensor:
        """Compute actuator torques using the full BAM M6 model.

        1. XL330 firmware control law: position error → duty cycle → voltage
        2. DC motor equation: voltage → motor torque (with back-EMF)
        3. M6 friction: load-dependent Coulomb + Stribeck + viscous
        4. Static friction clipping (prevents zero-velocity oscillation)
        5. Return: motor_torque + friction_torque
        """
        cfg = self.cfg
        pos_error = cmd.position_target - cmd.joint_pos
        vel = cmd.joint_vel

        # ── 1. XL330 firmware voltage control law ──
        # kp_scale is per-env (shape: num_envs, 1), broadcast over joints
        assert self.kp_scale is not None and self.kd_scale is not None
        duty_cycle = pos_error * cfg.kp_fw * self.kp_scale * cfg.error_gain
        duty_cycle = torch.clamp(duty_cycle, -cfg.max_pwm, cfg.max_pwm)
        voltage = cfg.vin * duty_cycle

        # ── 2. DC motor torque ──
        motor_torque = cfg.kt * voltage / cfg.R - (cfg.kt ** 2) * vel * self.kd_scale / cfg.R

        # ── 3. External (bias) torque on each joint ──
        # qfrc_bias contains gravity + Coriolis forces per DOF.
        # BAM convention: bias_torque = m*g*l*sin(q) with g = -9.81
        # MuJoCo convention: qfrc_bias has the opposite sign.
        # We negate to match BAM's sign convention for the friction model.
        assert self._data is not None and self._dof_ids is not None
        qfrc_bias_all = self._data.qfrc_bias  # (nworld, nv)
        if isinstance(qfrc_bias_all, torch.Tensor):
            external_torque = -qfrc_bias_all[:, self._dof_ids]
        else:
            external_torque = -torch.as_tensor(
                qfrc_bias_all, device=self._device
            )[:, self._dof_ids]

        # ── 4. M6 friction budget ──
        abs_vel = torch.abs(vel)

        # Stribeck coefficient: 1 when stopped, decays with velocity
        stribeck_coeff = torch.exp(
            -torch.pow(abs_vel / cfg.dtheta_stribeck, cfg.alpha)
        )

        # Directional gearbox torque (M5/M6: separate motor vs external)
        gearbox_torque = torch.abs(
            external_torque * cfg.load_friction_external
            - motor_torque * cfg.load_friction_motor
        )
        gearbox_torque_stribeck = torch.abs(
            external_torque * cfg.load_friction_external_stribeck
            - motor_torque * cfg.load_friction_motor_stribeck
        )

        # Quadratic terms (M6)
        abs_ext = torch.abs(external_torque)
        abs_mot = torch.abs(motor_torque)
        # drive regime: motor dominates; backdrive regime: external dominates
        drive_mask = (abs_mot > abs_ext).float()
        backdrive_mask = 1.0 - drive_mask
        quad_term = (
            drive_mask * cfg.load_friction_external_quad * abs_ext ** 2
            + backdrive_mask * cfg.load_friction_motor_quad * abs_mot ** 2
        )

        # Total friction budget (Coulomb-like, velocity-independent part)
        frictionloss = (
            cfg.friction_base
            + gearbox_torque
            + stribeck_coeff * (cfg.friction_stribeck + gearbox_torque_stribeck)
            + stribeck_coeff * quad_term
        )

        # Viscous (velocity-proportional) damping
        damping = cfg.friction_viscous

        # Total friction budget
        friction_budget = frictionloss + damping * abs_vel

        # ── 5. Static friction clipping (BAM's Algorithm 1) ──
        # tau_stop = torque needed to bring joint to rest in one timestep
        # Uses effective inertia from dof_invweight0 (1/effective_inertia)
        assert self._model is not None
        dof_invweight = self._model.dof_invweight0  # (nv,) or (nworld, nv)
        if isinstance(dof_invweight, torch.Tensor):
            invweight = dof_invweight
        else:
            invweight = torch.as_tensor(dof_invweight, device=self._device)
        # Handle both 1D (shared) and 2D (per-env) layouts
        if invweight.ndim == 1:
            eff_inertia = 1.0 / invweight[self._dof_ids].unsqueeze(0)
        else:
            eff_inertia = 1.0 / invweight[:, self._dof_ids]

        # Net torque without friction (motor + gravity/coriolis)
        # Note: external_torque is in BAM convention (negated qfrc_bias).
        # For tau_stop we need the actual MuJoCo net force, which is
        # motor_torque (our ctrl) + qfrc_bias (added by MuJoCo) = motor_torque - external_torque
        qfrc_bias_mujoco = -external_torque  # un-negate to get MuJoCo convention
        net_no_friction = motor_torque + qfrc_bias_mujoco
        tau_stop = (eff_inertia / self._dt) * vel + net_no_friction

        # Friction opposes tau_stop, clipped to budget
        abs_tau_stop = torch.abs(tau_stop)
        friction_magnitude = torch.minimum(abs_tau_stop, friction_budget)
        friction_torque = -torch.sign(tau_stop) * friction_magnitude

        # ── 6. Return total actuator torque ──
        # MuJoCo adds qfrc_bias separately. We return motor + friction only.
        return motor_torque + friction_torque
