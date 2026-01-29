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


LEROBOT_HUMANOID_MESH_DIR: Path = _find_repo_root() / "models" / "bipedal_plateform" / "mjcf"
LEROBOT_HUMANOID_XML: Path = LEROBOT_HUMANOID_MESH_DIR / "robot.xml"

assert LEROBOT_HUMANOID_XML.exists(), f"MJCF file not found: {LEROBOT_HUMANOID_XML}"


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, LEROBOT_HUMANOID_MESH_DIR, meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(LEROBOT_HUMANOID_XML))
  spec.assets = get_assets(spec.meshdir)

  return spec


##
# Actuator config.
#
# The robot uses position-controlled actuators.
# We define conservative stiffness and damping values.
# Adjust based on your actual motor specifications.
##

# Conservative actuator parameters for humanoid locomotion.
# These should be tuned based on actual motor specs.
HIP_ARMATURE = 0.02  # Reflected inertia for hip joints.
KNEE_ARMATURE = 0.015  # Reflected inertia for knee joints.
ANKLE_ARMATURE = 0.01  # Reflected inertia for ankle joints.

NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0

# Stiffness and damping derived from armature.
HIP_STIFFNESS = HIP_ARMATURE * NATURAL_FREQ**2
KNEE_STIFFNESS = KNEE_ARMATURE * NATURAL_FREQ**2
ANKLE_STIFFNESS = ANKLE_ARMATURE * NATURAL_FREQ**2

HIP_DAMPING = 2.0 * DAMPING_RATIO * HIP_ARMATURE * NATURAL_FREQ
KNEE_DAMPING = 2.0 * DAMPING_RATIO * KNEE_ARMATURE * NATURAL_FREQ
ANKLE_DAMPING = 2.0 * DAMPING_RATIO * ANKLE_ARMATURE * NATURAL_FREQ

# Effort limits (Nm) - adjust based on your motors.
HIP_EFFORT_LIMIT = 88.0
KNEE_EFFORT_LIMIT = 88.0
ANKLE_EFFORT_LIMIT = 40.0

LEROBOT_ACTUATOR_HIP = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "hipz_.*",
    "hipx_.*",
    "hipy_.*",
  ),
  stiffness=HIP_STIFFNESS,
  damping=HIP_DAMPING,
  effort_limit=HIP_EFFORT_LIMIT,
  armature=HIP_ARMATURE,
)

LEROBOT_ACTUATOR_KNEE = BuiltinPositionActuatorCfg(
  target_names_expr=("knee_.*",),
  stiffness=KNEE_STIFFNESS,
  damping=KNEE_DAMPING,
  effort_limit=KNEE_EFFORT_LIMIT,
  armature=KNEE_ARMATURE,
)

LEROBOT_ACTUATOR_ANKLE = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "ankley_.*",
    "anklex_.*",
  ),
  stiffness=ANKLE_STIFFNESS,
  damping=ANKLE_DAMPING,
  effort_limit=ANKLE_EFFORT_LIMIT,
  armature=ANKLE_ARMATURE,
)

##
# Keyframe config.
##

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.75),
  joint_pos={
    # Standing pose - all joints at zero.
    ".*": 0.0,
  },
  joint_vel={".*": 0.0},
)

KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.75),
  joint_pos={
    # Slightly bent knees for better initial stability.
    ".*": -0.,
    ".*": 0.,
    ".*": -0.,
    ".*": 0.0,
    ".*": 0.0,
    ".*": 0.0,
  },
  joint_vel={".*": 0.0},
)

##
# Collision config.
##

# Enable foot collisions with appropriate friction.
FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=(r"^(left|right)_foot[1-6]_collision$",),
  # contype=0 disables contacts entirely; keep it enabled for ground contact.
  contype=1,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
)

# Full collision including self-collisions.
FULL_COLLISION = CollisionCfg(
  geom_names_expr=(".*_collision",),
  condim={r"^(left|right)_foot[1-6]_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot[1-6]_collision$": 1},
  friction={r"^(left|right)_foot[1-6]_collision$": (0.6,)},
  # solref: (timeconst, dampratio) - smaller timeconst = stiffer, less bouncy
  solref={r"^(left|right)_foot[1-6]_collision$": (0.005, 1.0)},
  # solimp: (dmin, dmax, width, midpoint, power) - higher values = less penetration
  solimp={r"^(left|right)_foot[1-6]_collision$": (0.995, 0.9995, 0.001, 0.5, 2)},
)

##
# Final config.
##

LEROBOT_HUMANOID_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    LEROBOT_ACTUATOR_HIP,
    LEROBOT_ACTUATOR_KNEE,
    LEROBOT_ACTUATOR_ANKLE,
  ),
  soft_joint_pos_limit_factor=0.9,
)


def get_lerobot_humanoid_robot_cfg() -> EntityCfg:
  """Get a fresh LeRobot Humanoid robot configuration instance.

  Returns a new EntityCfg instance each time to avoid mutation issues when
  the config is shared across multiple places.
  """
  return EntityCfg(
    init_state=KNEES_BENT_KEYFRAME,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=LEROBOT_HUMANOID_ARTICULATION,
  )


# Action scale: scales normalized actions to joint position offsets.
LEROBOT_HUMANOID_ACTION_SCALE: dict[str, float] = {}
for a in LEROBOT_HUMANOID_ARTICULATION.actuators:
  assert isinstance(a, BuiltinPositionActuatorCfg)
  e = a.effort_limit
  s = a.stiffness
  names = a.target_names_expr
  assert e is not None
  for n in names:
    LEROBOT_HUMANOID_ACTION_SCALE[n] = 0.25 * e / s


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_lerobot_humanoid_robot_cfg())
  viewer.launch(robot.spec.compile())
