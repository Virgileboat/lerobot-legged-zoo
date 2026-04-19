"""MJLab Hf zoo - Training robot to move."""

# NOTE: the other training examples (lerobot_humanoid*, leggy, g1_*) were
# written against an older mjlab commit. They no longer load cleanly against
# the mjlab revision pinned in pyproject.toml (d1d32d8b...) because the
# velocity env no longer has a "base_com" event, which those examples still
# reference. Re-enable these imports once they've been migrated.
# from . import lerobot_humanoid  # noqa: F401
# from . import lerobot_humanoid_full  # noqa: F401
# from . import lerobot_humanoid_no_arms  # noqa: F401
# from . import leggy  # noqa: F401
# from . import g1_23dof  # noqa: F401
# from . import g1_29dof  # noqa: F401

from . import open_duck_mini_v2  # noqa: F401
