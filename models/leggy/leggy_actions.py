"""Custom action term for Leggy robot with motor-to-knee conversion.

This module provides a simplified action term that only handles motor-to-knee conversion:
- Inherits from JointPositionAction for standard joint control
- Overrides process_actions to convert motor commands → knee angles
- Passive joints are handled automatically by MuJoCo's constraint solver
- Observations compute motor space using joint_pos_motor/joint_vel_motor

Usage in your task configuration:
    from mjlab_leggy.leggy.leggy_actions import LeggyJointActionCfg

    cfg.actions = {
        "joint_pos": LeggyJointActionCfg()
    }
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import torch

from mjlab.envs.mdp.actions.joint_actions import JointPositionAction
from mjlab.envs.mdp.actions.actions_config import JointPositionActionCfg

if TYPE_CHECKING:
    from mjlab.envs import ManagerBasedRlEnv


# =============================================================================
# Kinematic Conversion Functions
# those 2 functions are validated by there usage in leggy_constants.py
# =============================================================================

def motor_to_knee(motor: torch.Tensor | float, hipX: torch.Tensor | float) -> torch.Tensor | float:
    """Convert motor command to knee angle.

    Leggy's knee is mechanically offset by the hipX angle.
    The motor command must be adjusted to get the actual knee angle.

    Args:
        motor: Motor command angle [rad] (tensor or scalar)
        hipX: Current hipX joint angle [rad] (tensor or scalar)

    Returns:
        Knee angle [rad] (same type as input)
    """
    result = hipX + motor
    return result


def knee_to_motor(knee: torch.Tensor | float, hipX: torch.Tensor | float) -> torch.Tensor | float:
    """Convert knee angle to motor command.

    Used for observations: convert actual knee angle to motor representation.

    Args:
        knee: Knee joint angle [rad] (tensor or scalar)
        hipX: Current hipX joint angle [rad] (tensor or scalar)

    Returns:
        Motor command angle [rad] (same type as input)
    """
    result = knee - hipX
    return result


# =============================================================================
# Action Term Implementation
# =============================================================================

class LeggyJointAction(JointPositionAction):
    """Action term that handles Leggy's motor-to-knee conversion.

    Inherits from JointPositionAction and only overrides process_actions to convert
    motor commands to knee angles. The parent class handles apply_actions.

    Policy outputs: [LhipY, LhipX, Lmotor, RhipY, RhipX, Rmotor]
    Converted to: [LhipY, LhipX, Lknee, RhipY, RhipX, Rknee]

    Note: Passive joints are handled automatically by MuJoCo's constraint solver.
    Observations compute motor space from knee angles using joint_pos_motor/joint_vel_motor.
    """

    cfg: LeggyJointActionCfg

    def process_actions(self, actions: torch.Tensor) -> None:
        """Convert motor commands to knee angles and store in processed_actions.

        Args:
            actions: Raw actions from policy with shape [num_envs, 6]
                     Columns: [LhipY, LhipX, Lmotor, RhipY, RhipX, Rmotor]
        """
        # Store raw actions
        self._raw_actions[:] = actions

        # Apply scale and offset (from parent class - includes HOME_FRAME default offset)
        self._processed_actions[:] = self._raw_actions * self._scale + self._offset

        # Convert motor commands to knee commands (indices 2 and 5)
        # Use target hipX values (indices 1 and 4) for the conversion
        # After this, _processed_actions contains: [LhipY, LhipX, Lknee, RhipY, RhipX, Rknee]
        hipX_left_target = self._processed_actions[:, 1]
        hipX_right_target = self._processed_actions[:, 4]
        self._processed_actions[:, 2] = motor_to_knee(self._processed_actions[:, 2], hipX_left_target)
        self._processed_actions[:, 5] = motor_to_knee(self._processed_actions[:, 5], hipX_right_target)


# =============================================================================
# Action Term Configuration
# =============================================================================

@dataclass(kw_only=True)
class LeggyJointActionCfg(JointPositionActionCfg):
    """Configuration for Leggy joint action with motor conversion.

    The policy outputs 6 actions for: [LhipY, LhipX, Lmotor, RhipY, RhipX, Rmotor]
    This action term automatically:
    - Converts motor commands → knee angles using current hipX
    - Sets actuated joint targets (via parent JointPositionAction)

    Passive joints are handled by MuJoCo's constraint solver.
    Use joint_pos_motor/joint_vel_motor for observations in motor space.

    Attributes:
        class_type: The action term class (automatically set to LeggyJointAction)
        asset_name: Name of the robot asset (default: "robot")
        actuator_names: Names of the actuators (defaults to the 6 knee actuators)
    """
    class_type: type[JointPositionAction] = LeggyJointAction
    asset_name: str = "robot"
    actuator_names: tuple[str, ...] = ("LhipY", "LhipX", "Lknee", "RhipY", "RhipX", "Rknee")


# =============================================================================
# Observation Helper Functions
# =============================================================================

def joint_pos_motor(env, asset_cfg=None) -> torch.Tensor:
    """Get joint positions in motor-space representation.

    Computes motor space knee angles from current knee positions using knee_to_motor conversion.
    This allows MuJoCo to handle the passive joint loop automatically.

    Args:
        env: The environment instance
        asset_cfg: Asset configuration (not used, kept for API compatibility)

    Returns:
        Joint positions [num_envs, 6]
        Layout: [LhipY_pos, LhipX_pos, Lmotor_pos, RhipY_pos, RhipX_pos, Rmotor_pos]
    """
    # Get the robot asset
    asset = env.scene["robot"]

    # Find joint indices in the order: [LhipY, LhipX, Lknee, RhipY, RhipX, Rknee]
    joint_names = ["LhipY", "LhipX", "Lknee", "RhipY", "RhipX", "Rknee"]
    joint_ids = asset.find_joints(joint_names)[0]

    # Read current joint positions
    joint_positions = asset.data.joint_pos[:, joint_ids]

    # Extract components
    LhipY = joint_positions[:, 0]
    LhipX = joint_positions[:, 1]
    Lknee = joint_positions[:, 2]
    RhipY = joint_positions[:, 3]
    RhipX = joint_positions[:, 4]
    Rknee = joint_positions[:, 5]

    # Convert knee positions to motor space
    Lmotor = knee_to_motor(Lknee, LhipX)
    Rmotor = knee_to_motor(Rknee, RhipX)

    # Return in motor space layout
    motor_positions = torch.stack([LhipY, LhipX, Lmotor, RhipY, RhipX, Rmotor], dim=1)

    return motor_positions


def joint_vel_motor(env, asset_cfg=None) -> torch.Tensor:
    """Get joint velocities in motor-space representation.

    Computes motor space knee velocities from current knee and hipX velocities.

    Args:
        env: The environment instance
        asset_cfg: Asset configuration (not used, kept for API compatibility)

    Returns:
        Joint velocities [num_envs, 6]
        Layout: [LhipY_vel, LhipX_vel, Lmotor_vel, RhipY_vel, RhipX_vel, Rmotor_vel]
    """
    # Get the robot asset
    asset = env.scene["robot"]

    # Find joint indices in the order: [LhipY, LhipX, Lknee, RhipY, RhipX, Rknee]
    joint_names = ["LhipY", "LhipX", "Lknee", "RhipY", "RhipX", "Rknee"]
    joint_ids = asset.find_joints(joint_names)[0]

    # Read current joint velocities
    joint_velocities = asset.data.joint_vel[:, joint_ids]

    # Extract components
    LhipY_vel = joint_velocities[:, 0]
    LhipX_vel = joint_velocities[:, 1]
    Lknee_vel = joint_velocities[:, 2]
    RhipY_vel = joint_velocities[:, 3]
    RhipX_vel = joint_velocities[:, 4]
    Rknee_vel = joint_velocities[:, 5]

    # Motor velocity = d/dt(knee - hipX) = knee_vel - hipX_vel
    Lmotor_vel = Lknee_vel - LhipX_vel
    Rmotor_vel = Rknee_vel - RhipX_vel

    # Return in motor space layout
    motor_velocities = torch.stack([LhipY_vel, LhipX_vel, Lmotor_vel, RhipY_vel, RhipX_vel, Rmotor_vel], dim=1)

    return motor_velocities


def joint_torques_motor(env, asset_cfg=None) -> torch.Tensor:
    """Get actuator torques in motor-space representation.

    Since Leggy has torque sensors at the motor outputs, we want to observe
    torques as the robot would measure them:
    - Hip torques: directly from hip actuators
    - Motor torques: from knee actuators (which drive the motors through the mechanism)

    Args:
        env: The environment instance
        asset_cfg: Asset configuration (not used, kept for API compatibility)

    Returns:
        Actuator torques [num_envs, 6]
        Layout: [LhipY_torque, LhipX_torque, Lmotor_torque, RhipY_torque, RhipX_torque, Rmotor_torque]
    """
    # Get the robot asset
    asset = env.scene["robot"]

    # Find actuator indices in the order: [LhipY, LhipX, Lknee, RhipY, RhipX, Rknee]
    actuator_names = ["LhipY", "LhipX", "Lknee", "RhipY", "RhipX", "Rknee"]
    actuator_ids = asset.find_actuators(actuator_names)[0]

    # Read actuator forces (torques) from simulation
    # data.actuator_force contains the force/torque each actuator is producing
    actuator_torques = asset.data.actuator_force[:, actuator_ids]

    # The layout matches our observation joints perfectly:
    # - Index 0: LhipY actuator → LhipY torque
    # - Index 1: LhipX actuator → LhipX torque
    # - Index 2: Lknee actuator → Lmotor torque (knee drives motor through mechanism)
    # - Index 3: RhipY actuator → RhipY torque
    # - Index 4: RhipX actuator → RhipX torque
    # - Index 5: Rknee actuator → Rmotor torque (knee drives motor through mechanism)

    return actuator_torques


# =============================================================================
# Convenience Exports
# =============================================================================

__all__ = [
    "motor_to_knee",
    "knee_to_motor",
    "joint_pos_motor",
    "joint_vel_motor",
    "joint_torques_motor",
    "LeggyJointAction",
    "LeggyJointActionCfg",
]
