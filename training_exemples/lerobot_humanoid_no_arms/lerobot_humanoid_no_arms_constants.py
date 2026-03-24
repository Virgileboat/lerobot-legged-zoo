"""LeRobot Humanoid (12-DOF bipedal) constants."""

import math
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
LEROBOT_HUMANOID_NO_ARMS_XML: Path = LEROBOT_HUMANOID_NO_ARMS_MESH_DIR / "robot.xml"

assert LEROBOT_HUMANOID_NO_ARMS_XML.exists(), f"MJCF file not found: {LEROBOT_HUMANOID_NO_ARMS_XML}"


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, LEROBOT_HUMANOID_NO_ARMS_MESH_DIR, meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(LEROBOT_HUMANOID_NO_ARMS_XML))
  spec.assets = get_assets(spec.meshdir)
  # Set symmetric hinge limits (in radians) so joint-limit rewards and reset
  # logic use meaningful bounds instead of the MJCF default [0, 0] ranges.
  joint_limit_deg_by_name = {
    # Hip yaw / roll.
    "hipz_right": 10.0,
    "hipz_left": 10.0,
    "hipx_right": 10.0,
    "hipx_left": 10.0,
    # Hip pitch and knee.
    "hipy_right": 45.0,
    "hipy_left": 45.0,
    "knee_right": 45.0,
    "knee_left": 45.0,
    # Ankle pitch / roll.
    "ankley_right": 10.0,
    "ankley_left": 10.0,
    "anklex_right": 5.0,
    "anklex_left": 5.0,
  }
  for joint_name, half_range_deg in joint_limit_deg_by_name.items():
    half_range_rad = math.radians(half_range_deg)
    spec.joint(joint_name).range = [-half_range_rad, half_range_rad]

  return spec


##
# Actuator config.
#
# The robot uses position-controlled actuators.
# We define conservative stiffness and damping values.
# Adjust based on your actual motor specifications.
##

# Base actuator gains (no additional scaling).
BASE_KP_HIPZ = 20.0
BASE_KV_HIPZ = 2.0
BASE_KP_HIPX = 40.0
BASE_KV_HIPX = 2.0
BASE_KP_HIP = 200.0
BASE_KV_HIP = 10.0
BASE_KP_KNEE = 200.0
BASE_KV_KNEE = 10.0
BASE_KP_ANKLE = 30.0
BASE_KV_ANKLE = 1.0

# Joint armature/frictionloss applied through actuator config.
# Keep these aligned with the robot MJCF values.
HIPZ_ARMATURE = 0.00226871374002
HIPX_ARMATURE = 0.0134012810769
HIPY_ARMATURE = 0.0134012810769
KNEE_ARMATURE = 0.0134012810769
ANKLEY_ARMATURE = 0.0209776768142
ANKLEX_ARMATURE = 0.00485638046015

HIPZ_FRICTIONLOSS = 0.135520223994
HIPX_FRICTIONLOSS = 0.116870607367
HIPY_FRICTIONLOSS = 0.116870607367
KNEE_FRICTIONLOSS = 0.116870607367
ANKLEY_FRICTIONLOSS = 0.0171195170804
ANKLEX_FRICTIONLOSS = 0.191719474184

# Effort limits (Nm) - adjust based on your motors.
HIP_EFFORT_LIMIT = 88.0
KNEE_EFFORT_LIMIT = 88.
ANKLE_EFFORT_LIMIT = 44.


LEROBOT_ACTUATOR_HIP1 = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "hipz_.*",
  ),
  stiffness=BASE_KP_HIPZ,
  damping=BASE_KV_HIPZ,
  effort_limit=150.0,
  armature=HIPZ_ARMATURE,
  frictionloss=HIPZ_FRICTIONLOSS,
)


LEROBOT_ACTUATOR_HIP2 = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "hipx_.*",
  ),
  stiffness=BASE_KP_HIPX,
  damping=BASE_KV_HIPX,
  effort_limit=88.0,
  armature=HIPX_ARMATURE,
  frictionloss=HIPX_FRICTIONLOSS,
)


LEROBOT_ACTUATOR_HIP3 = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "hipy_.*",
  ),
  stiffness=BASE_KP_HIP,
  damping=BASE_KV_HIP,
  effort_limit=88.0,
  armature=HIPY_ARMATURE,
  frictionloss=HIPY_FRICTIONLOSS,
)


LEROBOT_ACTUATOR_KNEE = BuiltinPositionActuatorCfg(
  target_names_expr=("knee_.*",),
  stiffness=BASE_KP_KNEE,
  damping=BASE_KV_KNEE,
  effort_limit=88.0,
  armature=KNEE_ARMATURE,
  frictionloss=KNEE_FRICTIONLOSS,
)

LEROBOT_ACTUATOR_ANKLEY = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "ankley_.*",
  ),
  stiffness=BASE_KP_ANKLE,
  damping=BASE_KV_ANKLE,
  effort_limit=44.0,
  armature=ANKLEY_ARMATURE,
  frictionloss=ANKLEY_FRICTIONLOSS,
)

LEROBOT_ACTUATOR_ANKLEX = BuiltinPositionActuatorCfg(
  target_names_expr=("anklex_.*",),
  stiffness=BASE_KP_ANKLE,
  damping=BASE_KV_ANKLE,
  effort_limit=44.0,
  armature=ANKLEX_ARMATURE,
  frictionloss=ANKLEX_FRICTIONLOSS,
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
  actuators=(
    LEROBOT_ACTUATOR_HIP1,
    LEROBOT_ACTUATOR_HIP2,
    LEROBOT_ACTUATOR_HIP3,
    LEROBOT_ACTUATOR_KNEE,
    LEROBOT_ACTUATOR_ANKLEY,
    LEROBOT_ACTUATOR_ANKLEX,
  ),
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
