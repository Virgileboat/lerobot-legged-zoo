"""Unitree G1 23-DOF constants (LeRobot model)."""

from pathlib import Path

import mujoco

from mjlab.asset_zoo.robots.unitree_g1.g1_constants import (
  FULL_COLLISION,
  G1_ACTUATOR_5020,
  G1_ACTUATOR_7520_14,
  G1_ACTUATOR_7520_22,
  G1_ACTUATOR_ANKLE,
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


G1_23DOF_MODEL_DIR: Path = _find_repo_root() / "models" / "g1_description"
G1_23DOF_XML: Path = G1_23DOF_MODEL_DIR / "g1_23dof.xml"

assert G1_23DOF_XML.exists(), f"MJCF file not found: {G1_23DOF_XML}"


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, G1_23DOF_MODEL_DIR / "meshes", meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(G1_23DOF_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


# 23-DOF variant does not include wrist pitch/yaw or waist roll/pitch joints.
G1_23DOF_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    G1_ACTUATOR_5020,
    G1_ACTUATOR_7520_14,
    G1_ACTUATOR_7520_22,
    G1_ACTUATOR_ANKLE,
  ),
  soft_joint_pos_limit_factor=0.9,
)


def get_g1_23dof_robot_cfg() -> EntityCfg:
  """Get a fresh G1 23-DOF robot configuration instance."""
  return EntityCfg(
    init_state=KNEES_BENT_KEYFRAME,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=G1_23DOF_ARTICULATION,
  )


G1_23DOF_ACTION_SCALE: dict[str, float] = {}
for a in G1_23DOF_ARTICULATION.actuators:
  assert isinstance(a, BuiltinPositionActuatorCfg)
  e = a.effort_limit
  s = a.stiffness
  names = a.target_names_expr
  assert e is not None
  for n in names:
    G1_23DOF_ACTION_SCALE[n] = 0.25 * e / s


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_g1_23dof_robot_cfg())
  viewer.launch(robot.spec.compile())
