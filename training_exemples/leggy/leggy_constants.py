"""Leggy robot constants (parallel mechanism)."""

from pathlib import Path

import mujoco
import numpy as np

from mjlab.actuator import XmlPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import CollisionCfg

##
# MJCF and assets.
##

def _find_repo_root() -> Path:
  here = Path(__file__).resolve()
  for parent in here.parents:
    if (parent / "models").is_dir():
      return parent
  raise FileNotFoundError("Could not locate repo root containing 'models' directory.")


LEGGY_MESH_DIR: Path = _find_repo_root() / "models" / "leggy"
LEGGY_XML: Path = LEGGY_MESH_DIR / "robot.xml"

assert LEGGY_XML.exists(), f"MJCF file not found: {LEGGY_XML}"


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, LEGGY_MESH_DIR, meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(LEGGY_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


##
# Initial pose.
##

stand_pose = {
  "hipY": 6 * np.pi / 180.0,
  "hipX": 25 * np.pi / 180.0,
  "kneeMotor": 45 * np.pi / 180.0,
  "knee": (25 + 45) * np.pi / 180.0,
}

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0.0, 0.0, 0.18),
  rot=(1.0, 0.0, 0.0, 0.0),
  joint_pos={
    ".*hipY.*": stand_pose["hipY"],
    ".*hipX.*": stand_pose["hipX"],
    ".*knee.*": stand_pose["knee"],
    "LpassiveMotor": stand_pose["kneeMotor"],
    "RpassiveMotor": stand_pose["kneeMotor"],
    "Lpassive2": stand_pose["knee"],
    "Rpassive2": stand_pose["knee"],
  },
  joint_vel={".*": 0.0},
  lin_vel=(1.8, 0.0, 1.5),
  ang_vel=(0.0, 0.0, 0.0),
)

##
# Collision config (mesh-based, no capsules).
##

FULL_COLLISION = CollisionCfg(
  geom_names_expr=(
    "left_foot_collision",
    "right_foot_collision",
  ),
  condim={
    "left_foot_collision": 3,
    "right_foot_collision": 3,
  },
  priority={
    "left_foot_collision": 1,
    "right_foot_collision": 1,
  },
  friction={
    "left_foot_collision": (1.5, 0.005, 0.0001),
    "right_foot_collision": (1.5, 0.005, 0.0001),
  },
  solref={
    "left_foot_collision": (0.005, 1.0),
    "right_foot_collision": (0.005, 1.0),
  },
  solimp={
    "left_foot_collision": (0.995, 0.9995, 0.001, 0.5, 2),
    "right_foot_collision": (0.995, 0.9995, 0.001, 0.5, 2),
  },
)

##
# Actuators (XML-defined).
##

LEGGY_ACTUATORS = XmlPositionActuatorCfg(
  target_names_expr=("LhipY", "LhipX", "Lknee", "RhipY", "RhipX", "Rknee"),
)

LEGGY_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(LEGGY_ACTUATORS,),
  soft_joint_pos_limit_factor=0.9,
)


def get_leggy_robot_cfg() -> EntityCfg:
  """Get a fresh Leggy robot configuration instance."""
  return EntityCfg(
    spec_fn=get_spec,
    init_state=HOME_KEYFRAME,
    collisions=(FULL_COLLISION,),
    articulation=LEGGY_ARTICULATION,
  )
