import argparse
from dataclasses import asdict
from pathlib import Path

import torch
import mjlab.scripts.play as play  # 👈 on réutilise les imports de play.py


class ExportPolicy(torch.nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        # RSL-RL style (souvent)
        if hasattr(self.policy, "act_inference") and callable(self.policy.act_inference):
            out = self.policy.act_inference(obs)
        else:
            out = self.policy(obs)

        if isinstance(out, (tuple, list)):
            out = out[0]
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True)
    ap.add_argument("--checkpoint-file", required=True)
    ap.add_argument("--out", default="policy.onnx")
    ap.add_argument("--device", default="cpu")   # "cpu" / "cuda:0" / "auto"
    ap.add_argument("--opset", type=int, default=17)
    ap.add_argument("--num-envs", type=int, default=1)
    args = ap.parse_args()

    # même call que run_play()
    play.configure_torch_backends()

    device = args.device
    if device == "auto":
        device = "cuda:0" if torch.cuda.is_available() else "cpu"

    env_cfg = play.load_env_cfg(args.task, play=True)
    agent_cfg = play.load_rl_cfg(args.task)

    env_cfg.scene.num_envs = args.num_envs

    resume_path = Path(args.checkpoint_file)
    if not resume_path.exists():
        raise FileNotFoundError(f"Checkpoint file not found: {resume_path}")

    # on reprend les mêmes classes que run_play()
    env = play.ManagerBasedRlEnv(cfg=env_cfg, device=device, render_mode=None)
    env = play.RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    runner_cls = play.load_runner_cls(args.task) or play.OnPolicyRunner
    runner = runner_cls(env, asdict(agent_cfg), device=device)
    runner.load(str(resume_path), map_location=device)
    policy = runner.get_inference_policy(device=device)

    # obs réelle
    obs = env.reset()
    if isinstance(obs, tuple) and len(obs) >= 1:
        obs = obs[0]
    if isinstance(obs, dict):
        obs = obs.get("obs", None) or obs.get("observations", None)
        if obs is None:
            raise RuntimeError("reset() a renvoyé un dict, mais pas de clé 'obs'/'observations'.")

    obs = torch.as_tensor(obs, dtype=torch.float32, device="cpu")
    if obs.ndim == 1:
        obs = obs.unsqueeze(0)

    export_model = ExportPolicy(policy).to("cpu").eval()

    torch.onnx.export(
        export_model,
        obs,
        args.out,
        opset_version=args.opset,
        input_names=["obs"],
        output_names=["action"],
        dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
    )

    print(f"✅ Exported: {args.out}")
    print("obs shape:", tuple(obs.shape))

    env.close()


if __name__ == "__main__":
    main()
