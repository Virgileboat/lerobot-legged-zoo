"""LeRobot Humanoid (12-DOF bipedal) constants."""

from pathlib import Path

import mujoco

from mjlab.actuator import BuiltinPositionActuatorCfg
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


LEROBOT_HUMANOID_NO_ARMS_MESH_DIR: Path = _find_repo_root() / "models" / "bipedal_plateform_no_arms" / "mjcf"
LEROBOT_HUMANOID_NO_ARMS_XML: Path = (
  LEROBOT_HUMANOID_NO_ARMS_MESH_DIR / "ientified_lrotobot_humanoid_no_arms.xml"
)

assert LEROBOT_HUMANOID_NO_ARMS_XML.exists(), f"MJCF file not found: {LEROBOT_HUMANOID_NO_ARMS_XML}"


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, LEROBOT_HUMANOID_NO_ARMS_MESH_DIR, meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(LEROBOT_HUMANOID_NO_ARMS_XML))
  spec.assets = get_assets(spec.meshdir)

  return spec


##
# Actuator config.
#
# The robot uses position-controlled actuators.
# We define conservative stiffness and damping values.
# Adjust based on your actual motor specifications.
##

# Identified joint-level actuator/joint parameters.
# Note: BuiltinPositionActuatorCfg rewrites MuJoCo joint armature/frictionloss,
# so these must match the identified MJCF values.
IDENTIFIED_ACTUATOR_PARAMS: dict[str, dict[str, float]] = {
  "hipz_right": {
    "kp": 10.0,
    "kv": 0.5,
    "effort_limit": 150.0,
    "armature": 0.03399100371098966,
    "frictionloss": 0.3390972681587539,
  },
  "hipx_right": {
    "kp": 20.0,
    "kv": 0.5,
    "effort_limit": 88.0,
    "armature": 0.021821226609500758,
    "frictionloss": 0.20146538999966387,
  },
  "hipy_right": {
    "kp": 20.0,
    "kv": 0.5,
    "effort_limit": 88.0,
    "armature": 0.2940956388058932,
    "frictionloss": 0.3996136801880859,
  },
  "knee_right": {
    "kp": 20.0,
    "kv": 0.5,
    "effort_limit": 88.0,
    "armature": 0.4063800985351495,
    "frictionloss": 0.20104603637816845,
  },
  "ankley_right": {
    "kp": 30.0,
    "kv": 1.0,
    "effort_limit": 44.0,
    "armature": 0.2106689119646011,
    "frictionloss": 0.13500359482883856,
  },
  "anklex_right": {
    "kp": 30.0,
    "kv": 1.0,
    "effort_limit": 44.0,
    "armature": 0.14840707882276186,
    "frictionloss": 0.4250925528502693,
  },
  "hipz_left": {
    "kp": 10.0,
    "kv": 0.5,
    "effort_limit": 150.0,
    "armature": 0.023188684778642967,
    "frictionloss": 0.2537584343329908,
  },
  "hipx_left": {
    "kp": 20.0,
    "kv": 0.5,
    "effort_limit": 88.0,
    "armature": 0.030387147982065633,
    "frictionloss": 0.6492191098255814,
  },
  "hipy_left": {
    "kp": 20.0,
    "kv": 0.5,
    "effort_limit": 88.0,
    "armature": 0.005084764344399466,
    "frictionloss": 0.9692185316442714,
  },
  "knee_left": {
    "kp": 20.0,
    "kv": 0.5,
    "effort_limit": 88.0,
    "armature": 0.09622722581638446,
    "frictionloss": 0.400036527536521,
  },
  "ankley_left": {
    "kp": 30.0,
    "kv": 1.0,
    "effort_limit": 44.0,
    "armature": 0.058959933130334555,
    "frictionloss": 0.6458502425774141,
  },
  "anklex_left": {
    "kp": 30.0,
    "kv": 1.0,
    "effort_limit": 44.0,
    "armature": 0.060897965838140784,
    "frictionloss": 0.3589253212548187,
  },
}

_ACTUATOR_ORDER = (
  "hipz_right",
  "hipx_right",
  "hipy_right",
  "knee_right",
  "ankley_right",
  "anklex_right",
  "hipz_left",
  "hipx_left",
  "hipy_left",
  "knee_left",
  "ankley_left",
  "anklex_left",
)


def _make_identified_actuator(joint_name: str) -> BuiltinPositionActuatorCfg:
  p = IDENTIFIED_ACTUATOR_PARAMS[joint_name]
  return BuiltinPositionActuatorCfg(
    target_names_expr=(joint_name,),
    stiffness=p["kp"],
    damping=p["kv"],
    effort_limit=p["effort_limit"],
    armature=p["armature"],
    frictionloss=p["frictionloss"],
  )

##
# Keyframe config.
##

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.72),
  joint_pos={
    # Slight crouch reference pose used as the robot default posture.
    # This becomes the target posture for the variable_posture reward
    # (it reads asset.data.default_joint_pos).
    "hipz_right": 0.0,
    "hipx_right": 0.0,
    "hipy_right": 0.0,
    "knee_right": 0.0,
    "ankley_right": 0.0,
    "anklex_right": 0.0,
    "hipz_left": 0.0,
    "hipx_left": 0.0,
    "hipy_left": -0.0,
    "knee_left": 0.0,
    "ankley_left": -0.0,
    "anklex_left": 0.0,
  },
  joint_vel={".*": 0.0},
)

# KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
#   pos=(0, 0, 0.77),
#   joint_pos={
#     # Slight crouch reference pose used as the robot default posture.
#     # This becomes the target posture for the variable_posture reward
#     # (it reads asset.data.default_joint_pos).
#     "hipz_right": 0.0,
#     "hipx_right": 0.0,
#     "hipy_right": 0.35,
#     "knee_right": 0.70,
#     "ankley_right": 0.35,
#     "anklex_right": 0.0,
#     "hipz_left": 0.0,
#     "hipx_left": 0.0,
#     "hipy_left": -0.35,
#     "knee_left": 0.70,
#     "ankley_left": -0.35,
#     "anklex_left": 0.0,
#   },
#   joint_vel={".*": 0.0},
# )


KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.72),
  joint_pos={
    # Slight crouch reference pose used as the robot default posture.
    # This becomes the target posture for the variable_posture reward
    # (it reads asset.data.default_joint_pos).
    "hipz_right": 0.0,
    "hipx_right": 0.0,
    "hipy_right": 0.0,
    "knee_right": 0.0,
    "ankley_right": 0.0,
    "anklex_right": 0.0,
    "hipz_left": 0.0,
    "hipx_left": 0.0,
    "hipy_left": -0.0,
    "knee_left": 0.0,
    "ankley_left": -0.0,
    "anklex_left": 0.0,
  },
  joint_vel={".*": 0.0},
)

##
# Collision config.
##

# Enable foot collisions with appropriate friction.
FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=(r"^(left|right)_foot_collision$",),
  # contype=0 disables contacts entirely; keep it enabled for ground contact.
  contype=1,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
  # Stiffer contact settings to reduce penetration.
  solref=(0.005, 1.0),
  solimp=(0.995, 0.9995, 0.001, 0.5, 2),
)

# Full collision including self-collisions.
FULL_COLLISION = CollisionCfg(
  geom_names_expr=(".*_collision",),
  condim={r"^(left|right)_foot_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot_collision$": 1},
  friction={r"^(left|right)_foot_collision$": (0.6,)},
  # Foot-specific stiff contact settings to reduce penetration.
  solref={r"^(left|right)_foot_collision$": (0.005, 1.0)},
  solimp={r"^(left|right)_foot_collision$": (0.995, 0.9995, 0.001, 0.5, 2)},
)
##
# Final config.
##

LEROBOT_HUMANOID_NO_ARMS_ARTICULATION = EntityArticulationInfoCfg(
  actuators=tuple(_make_identified_actuator(name) for name in _ACTUATOR_ORDER),
  soft_joint_pos_limit_factor=0.9,
)


def get_lerobot_humanoid_no_arms_robot_cfg() -> EntityCfg:
  """Get a fresh LeRobot Humanoid robot configuration instance.

  Returns a new EntityCfg instance each time to avoid mutation issues when
  the config is shared across multiple places.
  """
  return EntityCfg(
    init_state=KNEES_BENT_KEYFRAME,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=LEROBOT_HUMANOID_NO_ARMS_ARTICULATION,
  )


# Action scale: scales normalized actions to joint position offsets.
LEROBOT_HUMANOID_NO_ARMS_ACTION_SCALE: dict[str, float] = {}
for a in LEROBOT_HUMANOID_NO_ARMS_ARTICULATION.actuators:
  assert isinstance(a, BuiltinPositionActuatorCfg)
  e = a.effort_limit
  s = a.stiffness
  names = a.target_names_expr
  assert e is not None
  for n in names:
    LEROBOT_HUMANOID_NO_ARMS_ACTION_SCALE[n] = 0.25 * e / s


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_lerobot_humanoid_no_arms_robot_cfg())
  viewer.launch(robot.spec.compile())
