"""Unitree G1 29-DOF constants (LeRobot model)."""

from pathlib import Path

import mujoco

from mjlab.asset_zoo.robots.unitree_g1.g1_constants import (
  FULL_COLLISION,
  G1_ACTUATOR_5020,
  G1_ACTUATOR_7520_14,
  G1_ACTUATOR_7520_22,
  G1_ACTUATOR_4010,
  G1_ACTUATOR_ANKLE,
  G1_ACTUATOR_WAIST,
  KNEES_BENT_KEYFRAME,
)
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg
from mjlab.entity import EntityCfg
from mjlab.utils.os import update_assets

##
# MJCF and assets.
##

def _find_repo_root() -> Path:
  here = Path(__file__).resolve()
  for parent in here.parents:
    if (parent / "models").is_dir():
      return parent
  raise FileNotFoundError("Could not locate repo root containing 'models' directory.")


G1_29DOF_MODEL_DIR: Path = _find_repo_root() / "models" / "g1_description"
G1_29DOF_XML: Path = G1_29DOF_MODEL_DIR / "g1_29dof.xml"

assert G1_29DOF_XML.exists(), f"MJCF file not found: {G1_29DOF_XML}"


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, G1_29DOF_MODEL_DIR / "meshes", meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(G1_29DOF_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


# 29-DOF variant includes waist roll/pitch joints.
G1_29DOF_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    G1_ACTUATOR_5020,
    G1_ACTUATOR_7520_14,
    G1_ACTUATOR_7520_22,
    G1_ACTUATOR_4010,
    G1_ACTUATOR_WAIST,
    G1_ACTUATOR_ANKLE,
  ),
  soft_joint_pos_limit_factor=0.9,
)


def get_g1_29dof_robot_cfg() -> EntityCfg:
  """Get a fresh G1 29-DOF robot configuration instance."""
  return EntityCfg(
    init_state=KNEES_BENT_KEYFRAME,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=G1_29DOF_ARTICULATION,
  )


G1_29DOF_ACTION_SCALE: dict[str, float] = {}
for a in G1_29DOF_ARTICULATION.actuators:
  assert isinstance(a, BuiltinPositionActuatorCfg)
  e = a.effort_limit
  s = a.stiffness
  names = a.target_names_expr
  assert e is not None
  for n in names:
    G1_29DOF_ACTION_SCALE[n] = 0.25 * e / s


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_g1_29dof_robot_cfg())
  viewer.launch(robot.spec.compile())
