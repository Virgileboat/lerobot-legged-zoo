"""Open Duck Mini v2 velocity environment configuration."""

import math
from copy import deepcopy

# Domain randomization toggles
ENABLE_COM_RANDOMIZATION = True
ENABLE_KP_RANDOMIZATION = True
ENABLE_KD_RANDOMIZATION = True
ENABLE_MASS_INERTIA_RANDOMIZATION = True
ENABLE_VELOCITY_PUSHES = True
ENABLE_IMU_ORIENTATION_RANDOMIZATION = True
ENABLE_BASE_ORIENTATION_RANDOMIZATION = False

# Observation configuration
USE_PROJECTED_GRAVITY = True  # If True, use projected gravity instead of raw accelerometer

# Domain randomization ranges
COM_RANDOMIZATION_RANGE = 0.005  # ±5mm
MASS_INERTIA_RANDOMIZATION_RANGE = (0.95, 1.05)  # ±5%
KP_RANDOMIZATION_RANGE = (0.85, 1.15)  # ±15%
KD_RANDOMIZATION_RANGE = (0.9, 1.1)  # ±10%
VELOCITY_PUSH_INTERVAL_S = (3.0, 6.0)
VELOCITY_PUSH_RANGE = (-0.3, 0.3)
IMU_ORIENTATION_RANDOMIZATION_ANGLE = 1.0  # ±1° IMU mounting error
BASE_ORIENTATION_MAX_PITCH_DEG = 10.0
BASE_ORIENTATION_MAX_ROLL_DEG = 5.0

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.curriculum_manager import CurriculumTermCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg
from mjlab.utils.noise import UniformNoiseCfg as Unoise

from .open_duck_mini_v2_constants import get_open_duck_mini_v2_robot_cfg
from . import mdp as open_duck_mdp


def open_duck_mini_v2_velocity_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
    """Create Open Duck Mini v2 velocity tracking environment configuration."""

    std_standing = {
        r".*hip_yaw.*": 0.05,
        r".*hip_roll.*": 0.05,
        r".*hip_pitch.*": 0.05,
        r".*knee.*": 0.05,
        r".*ankle.*": 0.05,
        r".*neck.*": 0.05,
        r".*head.*": 0.05,
    }

    std_walking = {
        r".*hip_yaw.*": 0.3,
        r".*hip_roll.*": 0.1,
        r".*hip_pitch.*": 0.4,
        r".*knee.*": 0.4,
        r".*ankle.*": 0.25,
        r".*neck.*": 0.1,
        r".*head.*": 0.1,
    }

    std_running = {
        r".*hip_yaw.*": 0.5,
        r".*hip_roll.*": 0.2,
        r".*hip_pitch.*": 0.8,
        r".*knee.*": 0.8,
        r".*ankle.*": 0.5,
        r".*neck.*": 0.1,
        r".*head.*": 0.1,
    }

    site_names = ["left_foot", "right_foot"]

    feet_ground_cfg = ContactSensorCfg(
        name="feet_ground_contact",
        primary=ContactMatch(
            mode="subtree",
            pattern=r"^(foot_assembly|foot_assembly_2)$",
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
        primary=ContactMatch(mode="subtree", pattern="trunk_assembly", entity="robot"),
        secondary=ContactMatch(mode="subtree", pattern="trunk_assembly", entity="robot"),
        fields=("found",),
        reduce="none",
        num_slots=1,
    )

    foot_frictions_geom_names = (
        "left_foot_collision",
        "right_foot_collision",
    )

    cfg = make_velocity_env_cfg()

    cfg.observations["critic"].terms["foot_height"].params[
        "asset_cfg"
    ].site_names = site_names

    # Robot setup
    cfg.scene.entities = {"robot": get_open_duck_mini_v2_robot_cfg()}
    cfg.scene.sensors = (feet_ground_cfg, self_collision_cfg)
    cfg.viewer.body_name = "trunk_assembly"

    # Action configuration
    joint_pos_action = cfg.actions["joint_pos"]
    assert isinstance(joint_pos_action, JointPositionActionCfg)
    joint_pos_action.scale = 1.0

    # === REWARDS ===
    cfg.rewards["pose"].params["std_standing"] = std_standing
    cfg.rewards["pose"].params["std_walking"] = std_walking
    cfg.rewards["pose"].params["std_running"] = std_running
    cfg.rewards["pose"].params["walking_threshold"] = 0.01
    cfg.rewards["pose"].params["running_threshold"] = 0.5
    cfg.rewards["pose"].weight = 2.0

    cfg.rewards["self_collisions"] = RewardTermCfg(
        func=mdp.self_collision_cost,
        weight=-1.0,
        params={"sensor_name": self_collision_cfg.name},
    )

    cfg.rewards["upright"].params["asset_cfg"].body_names = ("trunk_assembly",)
    cfg.rewards["upright"].weight = 1.0

    for reward_name in ["foot_clearance", "foot_swing_height", "foot_slip"]:
        cfg.rewards[reward_name].params["asset_cfg"].site_names = site_names

    cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("trunk_assembly",)

    cfg.rewards["foot_slip"].weight = -0.1
    cfg.rewards["foot_slip"].params["command_threshold"] = 0.01

    cfg.rewards["soft_landing"].weight = -1e-05

    cfg.rewards["air_time"].weight = 5.0
    cfg.rewards["air_time"].params["command_threshold"] = 0.01
    cfg.rewards["air_time"].params["threshold_min"] = 0.15
    cfg.rewards["air_time"].params["threshold_max"] = 0.4

    cfg.rewards["stillness_at_zero_command"] = RewardTermCfg(
        func=open_duck_mdp.stillness_at_zero_command,
        weight=3.0,
        params={
            "command_name": "twist",
            "command_threshold": 0.01,
            "vel_std": 0.1,
        },
    )

    cfg.rewards["body_ang_vel"].weight = -0.05
    cfg.rewards["angular_momentum"].weight = -0.02

    cfg.rewards["track_linear_velocity"].weight = 3.0
    cfg.rewards["track_linear_velocity"].params["std"] = math.sqrt(0.15)
    cfg.rewards["track_angular_velocity"].weight = 3.0
    cfg.rewards["track_angular_velocity"].params["std"] = math.sqrt(0.40)

    cfg.rewards["action_rate_l2"].weight = -0.6

    cfg.rewards["foot_clearance"].params["command_threshold"] = 0.01
    cfg.rewards["foot_clearance"].params["target_height"] = 0.02

    cfg.rewards["foot_swing_height"].params["command_threshold"] = 0.01
    cfg.rewards["foot_swing_height"].params["target_height"] = 0.02

    cfg.rewards["neck_action_rate_l2"] = RewardTermCfg(
        func=open_duck_mdp.neck_action_rate_l2, weight=-0.1
    )

    cfg.rewards["com_height_target"] = RewardTermCfg(
        func=open_duck_mdp.com_height_target,
        weight=1.2,
        params={
            "target_height_min": 0.13,
            "target_height_max": 0.18,
        },
    )

    cfg.rewards["joint_torques_l2"] = RewardTermCfg(
        func=open_duck_mdp.joint_torques_l2, weight=-1e-3
    )

    # === EVENTS ===
    cfg.events["reset_action_history"] = EventTermCfg(
        func=open_duck_mdp.reset_action_history,
        mode="reset",
    )

    cfg.events["foot_friction"].params["asset_cfg"].geom_names = foot_frictions_geom_names
    cfg.events["reset_base"].params["pose_range"]["z"] = (0.20, 0.21)

    if ENABLE_VELOCITY_PUSHES:
        interval = (0.5, 1.0) if play else VELOCITY_PUSH_INTERVAL_S
        cfg.events["push_robot"] = EventTermCfg(
            func=mdp.push_by_setting_velocity,
            mode="interval",
            interval_range_s=interval,
            params={
                "velocity_range": {
                    "x": VELOCITY_PUSH_RANGE,
                    "y": VELOCITY_PUSH_RANGE,
                },
                "asset_cfg": SceneEntityCfg("robot"),
            },
        )

    if ENABLE_COM_RANDOMIZATION:
        cfg.events["randomize_com"] = EventTermCfg(
            func=mdp.randomize_field,
            mode="reset",
            domain_randomization=True,
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=("trunk_assembly",)),
                "operation": "add",
                "field": "body_ipos",
                "ranges": (-COM_RANDOMIZATION_RANGE, COM_RANDOMIZATION_RANGE),
            },
        )

    if ENABLE_KP_RANDOMIZATION or ENABLE_KD_RANDOMIZATION:
        kp_range = KP_RANDOMIZATION_RANGE if ENABLE_KP_RANDOMIZATION else (1.0, 1.0)
        kd_range = KD_RANDOMIZATION_RANGE if ENABLE_KD_RANDOMIZATION else (1.0, 1.0)
        cfg.events["randomize_motor_gains"] = EventTermCfg(
            func=open_duck_mdp.randomize_delayed_actuator_gains,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "operation": "scale",
                "kp_range": kp_range,
                "kd_range": kd_range,
            },
        )

    if ENABLE_MASS_INERTIA_RANDOMIZATION:
        cfg.events["randomize_mass_inertia"] = EventTermCfg(
            func=open_duck_mdp.randomize_mass_and_inertia,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=("trunk_assembly",)),
                "scale_range": MASS_INERTIA_RANDOMIZATION_RANGE,
            },
        )

    if ENABLE_IMU_ORIENTATION_RANDOMIZATION:
        cfg.events["randomize_imu_orientation"] = EventTermCfg(
            func=open_duck_mdp.randomize_imu_orientation,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "max_angle_deg": IMU_ORIENTATION_RANDOMIZATION_ANGLE,
            },
        )

    if ENABLE_BASE_ORIENTATION_RANDOMIZATION:
        cfg.events["randomize_base_orientation"] = EventTermCfg(
            func=open_duck_mdp.randomize_base_orientation,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "max_pitch_deg": BASE_ORIENTATION_MAX_PITCH_DEG,
                "max_roll_deg": BASE_ORIENTATION_MAX_ROLL_DEG,
            },
        )

    # === OBSERVATIONS ===
    del cfg.observations["policy"].terms["base_lin_vel"]

    gravity_term_name = "projected_gravity" if USE_PROJECTED_GRAVITY else "raw_accelerometer"

    if not USE_PROJECTED_GRAVITY:
        del cfg.observations["policy"].terms["projected_gravity"]
        cfg.observations["policy"].terms["raw_accelerometer"] = ObservationTermCfg(
            func=open_duck_mdp.raw_accelerometer,
            scale=1.0,
        )

    cfg.observations["policy"].terms[gravity_term_name] = deepcopy(
        cfg.observations["policy"].terms[gravity_term_name]
    )
    cfg.observations["policy"].terms["base_ang_vel"] = deepcopy(
        cfg.observations["policy"].terms["base_ang_vel"]
    )

    cfg.observations["policy"].terms["base_ang_vel"].delay_min_lag = 0
    cfg.observations["policy"].terms["base_ang_vel"].delay_max_lag = 3
    cfg.observations["policy"].terms["base_ang_vel"].delay_update_period = 64

    cfg.observations["policy"].terms[gravity_term_name].delay_min_lag = 0
    cfg.observations["policy"].terms[gravity_term_name].delay_max_lag = 3
    cfg.observations["policy"].terms[gravity_term_name].delay_update_period = 64

    cfg.observations["policy"].terms["base_ang_vel"].noise = Unoise(n_min=-0.024, n_max=0.024)
    cfg.observations["policy"].terms[gravity_term_name].noise = Unoise(n_min=-0.007, n_max=0.007)
    cfg.observations["policy"].terms["joint_pos"].noise = Unoise(n_min=-0.0006, n_max=0.0006)
    cfg.observations["policy"].terms["joint_vel"].noise = Unoise(n_min=-0.024, n_max=0.024)

    # === COMMANDS ===
    command: UniformVelocityCommandCfg = cfg.commands["twist"]
    command.rel_standing_envs = 0.02
    command.rel_heading_envs = 0.0
    command.ranges.lin_vel_x = (-0.3, 0.3)
    command.ranges.lin_vel_y = (-0.3, 0.3)
    command.ranges.ang_vel_z = (-1.5, 1.5)
    command.viz.z_offset = 0.8

    # === TERRAIN ===
    cfg.scene.terrain.terrain_type = "plane"
    cfg.scene.terrain.terrain_generator = None

    # === CURRICULUM ===
    cfg.curriculum["action_rate_weight"] = CurriculumTermCfg(
        func=mdp.reward_weight,
        params={
            "reward_name": "action_rate_l2",
            "weight_stages": [
                {"step": 0, "weight": -0.4},
                {"step": 250 * 24, "weight": -0.8},
                {"step": 500 * 24, "weight": -1.0},
            ],
        },
    )

    cfg.curriculum["standing_envs"] = CurriculumTermCfg(
        func=open_duck_mdp.standing_envs_curriculum,
        params={
            "command_name": "twist",
            "standing_stages": [
                {"step": 0,          "rel_standing_envs": 0.02},
                {"step": 500 * 24,   "rel_standing_envs": 0.05},
                {"step": 750 * 24,   "rel_standing_envs": 0.1},
                {"step": 1000 * 24,  "rel_standing_envs": 0.15},
                {"step": 1500 * 24,  "rel_standing_envs": 0.2},
                {"step": 2000 * 24,  "rel_standing_envs": 0.25},
            ],
        },
    )

    cfg.curriculum["velocity_command_ranges"] = CurriculumTermCfg(
        func=open_duck_mdp.velocity_command_ranges_curriculum,
        params={
            "command_name": "twist",
            "velocity_stages": [
                {"step": 0,          "lin_vel_range": 0.3,  "ang_vel_range": 1.5},
                {"step": 500 * 24,   "lin_vel_range": 0.35, "ang_vel_range": 1.6},
                {"step": 1000 * 24,  "lin_vel_range": 0.4,  "ang_vel_range": 1.7},
            ],
        },
    )

    del cfg.curriculum["terrain_levels"]
    del cfg.curriculum["command_vel"]

    return cfg
