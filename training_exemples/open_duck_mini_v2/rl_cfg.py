"""RL configuration for Open Duck Mini v2 (backlash) velocity task."""

from mjlab.rl import (
  RslRlOnPolicyRunnerCfg,
  RslRlPpoActorCriticCfg,
  RslRlPpoAlgorithmCfg,
)


def open_duck_mini_v2_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  """Create RL runner configuration for Open Duck Mini v2 (backlash) velocity task.

  Matches microduck configuration:
  - No observation normalization (we use observation noise instead)
  - ELU activation
  - 512-256-128 hidden layers
  """
  return RslRlOnPolicyRunnerCfg(
    policy=RslRlPpoActorCriticCfg(
      init_noise_std=1.0,
      actor_obs_normalization=False,  # Disabled like microduck - we use obs noise instead
      critic_obs_normalization=False,  # Disabled like microduck
      actor_hidden_dims=(512, 256, 128),
      critic_hidden_dims=(512, 256, 128),
      activation="elu",
    ),
    algorithm=RslRlPpoAlgorithmCfg(
      value_loss_coef=1.0,
      use_clipped_value_loss=True,
      clip_param=0.2,
      entropy_coef=0.01,
      num_learning_epochs=5,
      num_mini_batches=4,
      learning_rate=1.0e-3,  # Same as microduck
      schedule="adaptive",
      gamma=0.99,
      lam=0.95,
      desired_kl=0.01,
      max_grad_norm=1.0,
    ),
    wandb_project="mjlab_open_duck_mini_v2",
    experiment_name="velocity",  # Directory name
    run_name="velocity",  # Appended to datetime in wandb
    save_interval=250,
    num_steps_per_env=24,
    max_iterations=50_000,  # Same as microduck
  )
