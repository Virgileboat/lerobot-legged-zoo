import os
from pathlib import Path

import mujoco
from mjlab.actuator import XmlPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg
import numpy as np
from mjlab_leggy.leggy.leggy_actions import motor_to_knee, knee_to_motor

LEGGY_XML: Path = Path(os.path.dirname(__file__)) / "robot.xml"
assert LEGGY_XML.exists(), f"XML not found: {LEGGY_XML}"


def get_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_file(str(LEGGY_XML))


# NOTE: Motor-to-knee conversion is done via the LeggyJointAction action term.
# Passive joints are handled automatically by MuJoCo's constraint solver.
# Observations compute motor space from knee angles using joint_pos_motor/joint_vel_motor.
#
# For usage in tasks, see: from mjlab_leggy.leggy.leggy_actions import LeggyJointActionCfg


stand_pose = {
    "hipY": 6 * np.pi / 180.0,
    "hipX": 25 * np.pi / 180.0,
    "kneeMotor": 45 * np.pi / 180.0,
    "knee": (25+45) * np.pi / 180.0
}

HOME_FRAME = EntityCfg.InitialStateCfg(
    # Starting position (robot body center height)
    pos=(0.0, 0.0, 0.18),
    rot=(1.0, 0.0, 0.0, 0.0),
    joint_pos={
        # ".*hipY.*": stand_pose["hipY"],
        # ".*hipX.*": stand_pose["hipX"],#25
        # ".*knee.*": motor_to_knee(stand_pose["kneeMotor"], stand_pose["hipX"]),# so delta is 45+25=70
        # "LpassiveMotor": stand_pose["kneeMotor"], #this one is the actual motor knee
        # "RpassiveMotor": stand_pose["kneeMotor"],#this one is the actual motor knee
        # "Lpassive2": motor_to_knee(stand_pose["kneeMotor"], stand_pose["hipX"]), #70
        # "Rpassive2": motor_to_knee(stand_pose["kneeMotor"], stand_pose["hipX"]) #70

        ".*hipY.*": stand_pose["hipY"],
        ".*hipX.*": stand_pose["hipX"],#25
        ".*knee.*": stand_pose["knee"],# so delta is 45+25=70
        "LpassiveMotor": knee_to_motor(stand_pose["knee"], stand_pose["hipX"]), #this one is the actual motor knee
        "RpassiveMotor": knee_to_motor(stand_pose["knee"], stand_pose["hipX"]),#this one is the actual motor knee
        "Lpassive2": stand_pose["knee"], #70
        "Rpassive2": stand_pose["knee"] #70
    },
    joint_vel={".*": 0.0},
    # Explicitly set root (freejoint) velocities to zero
    lin_vel=(0.0, 0.0, 0.0),
    ang_vel=(0.0, 0.0, 0.0),
)

FULL_COLLISION = CollisionCfg(
    geom_names_expr=[".*_collision"],
    condim={r"^(left|right)_foot_collision$": 3, ".*_collision": 1},
    priority={r"^(left|right)_foot_collision$": 1},
    # Friction: (sliding, torsional, rolling) - higher sliding = less slip
    friction={r"^(left|right)_foot_collision$": (1.5, 0.005, 0.0001)},
    # solref: (timeconst, dampratio) - smaller timeconst = stiffer, less bouncy
    solref={r"^(left|right)_foot_collision$": (0.005, 1.0)},
    # solimp: (dmin, dmax, width, midpoint, power) - higher values = less penetration
    solimp={r"^(left|right)_foot_collision$": (0.995, 0.9995, 0.001, 0.5, 2)},
)

# Use existing position actuators defined in robot.xml
# (LhipY, LhipX, Lknee, RhipY, RhipX, Rknee with kp=10, forcerange=0.236)
LEGGY_ACTUATORS = XmlPositionActuatorCfg(
    joint_names_expr=("LhipY", "LhipX", "Lknee", "RhipY", "RhipX", "Rknee"),
)

LEGGY_ROBOT_CFG = EntityCfg(
    spec_fn=get_spec,
    init_state=HOME_FRAME,
    collisions=(FULL_COLLISION,),
    articulation=EntityArticulationInfoCfg(
        actuators=(LEGGY_ACTUATORS,),
        soft_joint_pos_limit_factor=0.9,
    ),
)

if __name__ == "__main__":
    import mujoco.viewer as viewer
    from mjlab.scene import Scene, SceneCfg
    from mjlab.terrains import TerrainImporterCfg
    import torch

    device = "cuda:0" if torch.cuda.is_available() else "cpu"

    SCENE_CFG = SceneCfg(
        terrain=TerrainImporterCfg(terrain_type="plane"),
        entities={"robot": LEGGY_ROBOT_CFG},
    )

    scene = Scene(SCENE_CFG, device=device)
    model = scene.compile()

    # NOTE: For proper simulation with motor conversion and passive joints,
    # use the full environment with LeggyJointActionCfg instead of this viewer.
    print("Launching basic viewer. Passive joints will not be automatically updated.")
    print("For full functionality, run the environment with LeggyJointActionCfg.")

    with viewer.launch_passive(model, scene.data) as v:
        while v.is_running():
            mujoco.mj_step(model, scene.data)
            v.sync()
