"""No-arms velocity runner with entropy annealing."""

from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner


class LerobotNoArmsOnPolicyRunner(VelocityOnPolicyRunner):
  """Velocity runner with per-update entropy coefficient decay.

  RSL-RL's PPO config exposes a fixed entropy coefficient. We apply a lightweight
  schedule here so exploration starts high and decays toward exploitation.
  """

  _ENTROPY_DECAY_RATE = 0.9998
  _ENTROPY_MIN = 3e-4

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._entropy_start = float(self.alg.entropy_coef)
    self._entropy_schedule_iter = int(self.current_learning_iteration)
    self._set_entropy_for_iteration(self._entropy_schedule_iter)

  def _set_entropy_for_iteration(self, iteration: int) -> None:
    entropy_coef = self._entropy_start * (self._ENTROPY_DECAY_RATE ** iteration)
    self.alg.entropy_coef = max(self._ENTROPY_MIN, entropy_coef)

  def learn(self, num_learning_iterations: int, init_at_random_ep_len: bool = False):
    self._entropy_schedule_iter = int(self.current_learning_iteration)
    original_update = self.alg.update

    def _update_with_entropy_schedule(*args, **kwargs):
      self._set_entropy_for_iteration(self._entropy_schedule_iter)
      out = original_update(*args, **kwargs)
      self._entropy_schedule_iter += 1
      return out

    self.alg.update = _update_with_entropy_schedule
    try:
      return super().learn(
        num_learning_iterations=num_learning_iterations,
        init_at_random_ep_len=init_at_random_ep_len,
      )
    finally:
      self.alg.update = original_update

  def log(self, locs: dict, width: int = 80, pad: int = 35):
    super().log(locs=locs, width=width, pad=pad)
    if self.writer is not None:
      self.writer.add_scalar("Loss/entropy_coef", float(self.alg.entropy_coef), locs["it"])
