#!/usr/bin/env python3
"""Evaluate the lerobot_humanoid_no_arms policy on locomotion scenarios.

Usage:
    uv run .claude/rl-training/tasks/locomotion/evaluate_policy.py <wandb_run_path> \
        --output-dir run_001/ --config .claude/rl-training/config.md

    # Dry run (random policy, fewer steps — for pipeline validation):
    uv run .claude/rl-training/tasks/locomotion/evaluate_policy.py --dry-run \
        --output-dir /tmp/eval-dry-run/ --config .claude/rl-training/config.md

Produces:
    - eval_report.md     — behavioral report with symmetry and physics-motion analysis
    - eval_metrics.json  — machine-readable metrics
    - eval_raw_data.json — per-step rewards and actions
    - eval_video.mp4     — video of forward_slow scenario (if render available)
"""

import argparse
import importlib.util
import json
import sys
from dataclasses import asdict
from pathlib import Path

import numpy as np
import torch


SCENARIOS = [
    {"name": "forward_slow", "vx": 0.3, "vy": 0.0, "wz": 0.0},
    {"name": "forward_fast", "vx": 0.8, "vy": 0.0, "wz": 0.0},
    {"name": "lateral",      "vx": 0.0, "vy": 0.4, "wz": 0.0},
    {"name": "combined",     "vx": 0.5, "vy": 0.2, "wz": 0.1},
]

# Action vector layout: pairs alternate right/left per joint type
# [hipz_r, hipz_l, hipx_r, hipx_l, hipy_r, hipy_l, knee_r, knee_l,
#  ankley_r, ankley_l, anklex_r, anklex_l]
LATERAL_PAIRS = [
    ("hipz",   0, 1),
    ("hipx",   2, 3),
    ("anklex", 10, 11),
]

# Sagittal joints: symmetric gait means both legs oscillate with equal amplitude
# (right and left do the same movement but half-period apart)
# Asymmetry here = one foot goes higher than the other (different swing amplitudes)
SAGITTAL_PAIRS = [
    ("hipy",   4, 5),
    ("knee",   6, 7),
    ("ankley", 8, 9),
]

EVAL_STEPS = 500   # ~10 seconds at 50Hz control
DRY_RUN_STEPS = 20


def load_eval_metrics_module():
    script_dir = Path(__file__).parent
    path = script_dir / "eval_metrics.py"
    if not path.exists():
        return None
    spec = importlib.util.spec_from_file_location("eval_metrics", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def download_checkpoint(run_path: str, output_dir: Path) -> Path:
    import wandb
    api = wandb.Api()
    run = api.run(run_path)
    files = [f for f in run.files() if f.name.endswith(".pt") and "model_" in f.name]
    if not files:
        raise RuntimeError(f"No .pt checkpoint found in run {run_path}")

    def step_num(f):
        try:
            return int(Path(f.name).stem.split("_")[-1])
        except ValueError:
            return 0

    latest = max(files, key=step_num)
    print(f"Downloading checkpoint: {latest.name} (step {step_num(latest)})")
    latest.download(root=str(output_dir), replace=True)
    # WandB may nest the file under subdirs — find it
    candidates = list(output_dir.rglob(Path(latest.name).name))
    return candidates[0] if candidates else output_dir / latest.name


def set_velocity_command(raw_env, vx, vy, wz, device):
    """Override the velocity command in the mjlab env."""
    try:
        cmd_manager = raw_env.command_manager
        cmd_term = cmd_manager.get_term("twist")
        # vel_command_b is [num_envs, 3] in body frame
        cmd_tensor = torch.tensor([[vx, vy, wz]], device=device, dtype=torch.float32)
        cmd_term.vel_command_b[:] = cmd_tensor
    except Exception:
        pass  # Fall through — env will sample its own command


def run_scenario(env, policy, scenario, num_steps, device, record_video=False):
    raw_env = env.unwrapped if hasattr(env, "unwrapped") else env

    obs_dict, _ = env.reset()
    set_velocity_command(raw_env, scenario["vx"], scenario["vy"], scenario["wz"], device)

    joint_pos_hist, joint_vel_hist = [], []
    contact_hist, force_hist = [], []
    reward_hist, action_hist = [], []
    frames = []
    episode_breaks = 0

    for _ in range(num_steps):
        with torch.inference_mode():
            actions = policy(obs_dict)

        step_out = env.step(actions)
        # RSL-RL VecEnvWrapper returns (obs, rewards, dones, extras)
        obs_dict, rewards, dones, extras = step_out[0], step_out[1], step_out[2], step_out[-1]

        # Collect per-step data
        try:
            robot = raw_env.scene["robot"]
            joint_pos_hist.append(robot.data.joint_pos[0].cpu().numpy())
            joint_vel_hist.append(robot.data.joint_vel[0].cpu().numpy())

            sensor = raw_env.scene["feet_ground_contact"]
            contact_hist.append(sensor.data.found[0, :, 0].cpu().numpy().astype(float))
            force_hist.append(sensor.data.force[0, :, 0, 2].abs().cpu().numpy())
        except Exception:
            pass

        reward_hist.append(float(rewards[0].item()))
        action_hist.append(actions[0].detach().cpu().numpy())

        if record_video:
            try:
                frame = raw_env.render()
                if frame is not None:
                    frames.append(frame)
            except Exception:
                pass

        if bool(dones[0]):
            episode_breaks += 1
            obs_dict, _ = env.reset()
            set_velocity_command(raw_env, scenario["vx"], scenario["vy"], scenario["wz"], device)

    return {
        "joint_pos":      np.array(joint_pos_hist) if joint_pos_hist else None,
        "joint_vel":      np.array(joint_vel_hist) if joint_vel_hist else None,
        "contacts":       np.array(contact_hist)   if contact_hist   else None,
        "forces":         np.array(force_hist)     if force_hist     else None,
        "rewards":        np.array(reward_hist),
        "actions":        np.array(action_hist)    if action_hist    else None,
        "frames":         frames,
        "episode_breaks": episode_breaks,
    }


def compute_bilateral_asymmetry(actions):
    """Compute symmetry metrics for lateral and sagittal joint pairs.

    Lateral (hipz, hipx, anklex): symmetric gait means mean(r) + mean(l) ≈ 0
    (they mirror each other — one yaws/rolls one way, the other the opposite).

    Sagittal (hipy, knee, ankley): symmetric gait means equal oscillation amplitudes
    (both legs swing with same height/range, half-period apart).
    Asymmetry = one foot goes higher than the other = std(r) ≠ std(l).
    """
    if actions is None or len(actions) == 0:
        return {}, {}
    lateral = {}
    for name, r_idx, l_idx in LATERAL_PAIRS:
        r_mean = float(np.mean(actions[:, r_idx]))
        l_mean = float(np.mean(actions[:, l_idx]))
        lateral[name] = {
            "right_mean": round(r_mean, 4),
            "left_mean":  round(l_mean, 4),
            "bias":       round(abs(r_mean + l_mean), 4),
        }
    sagittal = {}
    for name, r_idx, l_idx in SAGITTAL_PAIRS:
        r_std = float(np.std(actions[:, r_idx]))
        l_std = float(np.std(actions[:, l_idx]))
        r_mean = float(np.mean(actions[:, r_idx]))
        l_mean = float(np.mean(actions[:, l_idx]))
        sagittal[name] = {
            "right_std":  round(r_std, 4),
            "left_std":   round(l_std, 4),
            "amp_bias":   round(abs(r_std - l_std), 4),   # swing amplitude asymmetry
            "right_mean": round(r_mean, 4),
            "left_mean":  round(l_mean, 4),
        }
    return lateral, sagittal


def compute_fft_ratio_3hz(actions, dt=0.02):
    """Fraction of action power at or below 3Hz — physics motion proxy for sim2real."""
    if actions is None or len(actions) < 50:
        return None
    centered = actions - actions.mean(axis=0, keepdims=True)
    fft = np.fft.rfft(centered, axis=0)
    freqs = np.fft.rfftfreq(len(centered), d=dt)
    power = np.abs(fft) ** 2
    valid = freqs > 0
    inband = (freqs <= 3.0) & valid
    total = power[valid].sum()
    return float(power[inband].sum() / total) if total > 1e-12 else 1.0


def format_report(scenario_results, eval_mod, monitor_config_path):
    lines = ["# Behavioral Evaluation Report", ""]

    # Aggregate summary stats
    all_rewards, all_biases, all_amp_biases, all_fft = [], [], [], []
    for data in scenario_results.values():
        all_rewards.append(float(np.mean(data["rewards"])))
        for p in data["asymmetry"].values():
            all_biases.append(p["bias"])
        for p in data["sagittal_asymmetry"].values():
            all_amp_biases.append(p["amp_bias"])
        if data["fft_ratio"] is not None:
            all_fft.append(data["fft_ratio"])

    mean_reward    = np.mean(all_rewards)    if all_rewards    else 0.0
    mean_bias      = np.mean(all_biases)     if all_biases     else 0.0
    mean_amp_bias  = np.mean(all_amp_biases) if all_amp_biases else 0.0
    mean_fft       = np.mean(all_fft)        if all_fft        else 0.0

    asym_status     = "OK" if mean_bias < 0.05     else ("!" if mean_bias < 0.15     else "!! asymmetric_gait")
    amp_bias_status = "OK" if mean_amp_bias < 0.05 else ("!" if mean_amp_bias < 0.15 else "!! foot_height_asymmetry")
    fft_status      = "physics-feasible" if mean_fft > 0.8 else ("borderline" if mean_fft > 0.6 else "!! HIGH-FREQ sim2real risk")

    lines += [
        "## Summary",
        f"- Mean episode reward: {mean_reward:.3f}",
        f"- Bilateral bias (lateral joints, lower=better): {mean_bias:.3f} → {asym_status}",
        f"- Sagittal amp bias (foot height, lower=better): {mean_amp_bias:.3f} → {amp_bias_status}",
        f"- Action energy ≤3Hz (physics motion): {mean_fft:.3f} → {fft_status}",
        "",
    ]

    raw_metrics = {}
    all_quality = []

    lines.append("## Per-Scenario Results")
    for sname, data in scenario_results.items():
        sc = next((s for s in SCENARIOS if s["name"] == sname), {})
        vx, vy, wz = sc.get("vx", 0), sc.get("vy", 0), sc.get("wz", 0)
        lines.append(f"### {sname} (vx={vx}, vy={vy}, wz={wz})")

        mean_r = float(np.mean(data["rewards"]))
        lines.append(f"- Mean reward: {mean_r:.3f}")
        lines.append(f"- Episode resets (falls): {data['episode_breaks']}")

        # Lateral symmetry (mean-based)
        lines.append("- Bilateral symmetry (lateral joints):")
        for joint, pd in data["asymmetry"].items():
            flag = "" if pd["bias"] < 0.05 else (" !" if pd["bias"] < 0.15 else " !!")
            lines.append(
                f"  - {joint}: right={pd['right_mean']:+.3f}  left={pd['left_mean']:+.3f}"
                f"  bias={pd['bias']:.3f}{flag}"
            )
        # Sagittal symmetry (amplitude-based: detects foot height asymmetry)
        lines.append("- Sagittal symmetry (foot height — amp_bias=|std_r - std_l|):")
        for joint, pd in data["sagittal_asymmetry"].items():
            flag = "" if pd["amp_bias"] < 0.05 else (" !" if pd["amp_bias"] < 0.15 else " !!")
            lines.append(
                f"  - {joint}: std_r={pd['right_std']:.3f}  std_l={pd['left_std']:.3f}"
                f"  amp_bias={pd['amp_bias']:.3f}{flag}"
            )

        # FFT
        if data["fft_ratio"] is not None:
            flag = "" if data["fft_ratio"] > 0.8 else (" !" if data["fft_ratio"] > 0.6 else " !!")
            lines.append(f"- Action energy ≤3Hz: {data['fft_ratio']:.3f}{flag}")

        # Gait quality (Tier 2)
        quality_score = None
        if eval_mod and data.get("traj_quality"):
            q = data["traj_quality"]
            quality_score = q.get("detailed_quality_score")
            lines.append(eval_mod.format_markdown(q, sname))

        if quality_score is not None:
            lines.append(f"- Quality score: {quality_score:.3f}")
            all_quality.append(quality_score)

        lines.append("")

        raw_metrics[sname] = {
            "mean_reward":           mean_r,
            "fft_ratio_3hz":         data["fft_ratio"],
            "bilateral_asymmetry":   data["asymmetry"],
            "sagittal_asymmetry":    data["sagittal_asymmetry"],
            "episode_breaks":        data["episode_breaks"],
            "quality_score":         quality_score,
        }

    # Cross-scenario analysis
    lines.append("## Cross-Scenario Analysis")
    hipx_biases  = [scenario_results[s]["asymmetry"].get("hipx", {}).get("bias", 0.0) for s in scenario_results]
    hipz_biases  = [scenario_results[s]["asymmetry"].get("hipz", {}).get("bias", 0.0) for s in scenario_results]
    hipy_amps    = [scenario_results[s]["sagittal_asymmetry"].get("hipy",   {}).get("amp_bias", 0.0) for s in scenario_results]
    knee_amps    = [scenario_results[s]["sagittal_asymmetry"].get("knee",   {}).get("amp_bias", 0.0) for s in scenario_results]
    ankley_amps  = [scenario_results[s]["sagittal_asymmetry"].get("ankley", {}).get("amp_bias", 0.0) for s in scenario_results]

    if np.mean(hipx_biases) > 0.10:
        lines.append(
            f"- **Persistent hipx asymmetry** across all scenarios "
            f"(mean bias {np.mean(hipx_biases):.3f}) — lateral weight shift. "
            "Tag: lateral_lean / asymmetric_gait."
        )
    elif np.mean(hipx_biases) > 0.05:
        lines.append(f"- Mild hipx asymmetry (mean bias {np.mean(hipx_biases):.3f})")
    else:
        lines.append("- hipx symmetry OK")

    if np.mean(hipz_biases) > 0.05:
        lines.append(f"- hipz asymmetry detected (mean bias {np.mean(hipz_biases):.3f})")

    # Sagittal (foot height) analysis
    mean_hipy_amp   = np.mean(hipy_amps)
    mean_knee_amp   = np.mean(knee_amps)
    mean_ankley_amp = np.mean(ankley_amps)
    if mean_knee_amp > 0.05 or mean_hipy_amp > 0.05:
        lines.append(
            f"- **Foot height asymmetry detected** — "
            f"hipy amp_bias {mean_hipy_amp:.3f}, knee amp_bias {mean_knee_amp:.3f}, "
            f"ankley amp_bias {mean_ankley_amp:.3f}. Tag: foot_height_asymmetry."
        )
    else:
        lines.append(
            f"- Sagittal symmetry OK — hipy amp_bias {mean_hipy_amp:.3f}, "
            f"knee amp_bias {mean_knee_amp:.3f}"
        )

    if all_fft:
        lines.append(f"- Physics motion ratio (≤3Hz): {np.mean(all_fft):.3f} (target >0.8 for sim2real)")

    if all_quality:
        lines.append(f"- Mean Tier-2 quality score: {np.mean(all_quality):.3f}")

    lines += ["", f"<!-- RAW_METRICS:{json.dumps(raw_metrics)}-->"]
    return "\n".join(lines), raw_metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_path", nargs="?", default=None)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--config", default=".claude/rl-training/config.md")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Register tasks so mjlab knows about lerobot_humanoid_no_arms
    import training_exemples.lerobot_humanoid_no_arms  # noqa: F401
    from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import RslRlVecEnvWrapper
    from rsl_rl.runners import OnPolicyRunner

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    task_id = "Mjlab-Velocity-Flat-LeRobot-Humanoid-no-arms"
    num_steps = DRY_RUN_STEPS if args.dry_run else EVAL_STEPS

    env_cfg = load_env_cfg(task_id, play=True)
    agent_cfg = load_rl_cfg(task_id)
    env_cfg.scene.num_envs = 1

    env = ManagerBasedRlEnv(cfg=env_cfg, device=device, render_mode="rgb_array")
    env = RslRlVecEnvWrapper(env, clip_actions=getattr(agent_cfg, "clip_actions", False))

    if args.dry_run:
        print("[dry-run] Using random policy")
        def policy(obs):
            return torch.randn(1, 12, device=device) * 0.1
    else:
        if args.run_path is None:
            print("ERROR: run_path required unless --dry-run", file=sys.stderr)
            env.close()
            sys.exit(1)
        checkpoint_path = download_checkpoint(args.run_path, output_dir)
        runner = OnPolicyRunner(env, asdict(agent_cfg), device=device)
        runner.load(str(checkpoint_path), map_location=device)
        policy = runner.get_inference_policy(device=device)

    eval_mod = load_eval_metrics_module()
    monitor_cfg_path = Path(".claude/rl-training/tasks/locomotion/monitor_config.md")

    scenario_results = {}
    video_frames = []

    for scenario in SCENARIOS:
        print(f"Scenario: {scenario['name']}  vx={scenario['vx']} vy={scenario['vy']} wz={scenario['wz']}")
        record = (len(video_frames) == 0)  # only record first scenario
        data = run_scenario(env, policy, scenario, num_steps, device, record_video=record)

        # Use actual joint positions (not raw network output) for meaningful symmetry metrics
        sym_src = data["joint_pos"] if data["joint_pos"] is not None else data["actions"]
        data["asymmetry"], data["sagittal_asymmetry"] = compute_bilateral_asymmetry(sym_src)
        data["fft_ratio"] = compute_fft_ratio_3hz(data["actions"])

        if eval_mod and data["joint_pos"] is not None and data["contacts"] is not None:
            try:
                data["traj_quality"] = eval_mod.analyze_trajectory(
                    joint_positions=data["joint_pos"],
                    joint_velocities=data["joint_vel"],
                    contact_states=data["contacts"],
                    contact_forces=data["forces"],
                    dt=0.02,
                    config_path=str(monitor_cfg_path) if monitor_cfg_path.exists() else None,
                )
            except Exception as e:
                print(f"WARNING: eval_metrics failed for {scenario['name']}: {e}", file=sys.stderr)
                data["traj_quality"] = None
        else:
            data["traj_quality"] = None

        if data["frames"] and not video_frames:
            video_frames = data["frames"]

        scenario_results[scenario["name"]] = data

    env.close()

    # Write outputs
    report, raw_metrics = format_report(scenario_results, eval_mod, monitor_cfg_path)
    (output_dir / "eval_report.md").write_text(report)
    (output_dir / "eval_metrics.json").write_text(json.dumps(raw_metrics, indent=2))

    raw_data = {
        name: {
            "rewards": data["rewards"].tolist(),
            "actions": data["actions"].tolist() if data["actions"] is not None else [],
        }
        for name, data in scenario_results.items()
    }
    (output_dir / "eval_raw_data.json").write_text(json.dumps(raw_data))

    # Video
    if video_frames:
        video_path = output_dir / "eval_video.mp4"
        try:
            import imageio
            imageio.mimwrite(str(video_path), video_frames, fps=30, quality=8)
            print(f"Video: {video_path}")
        except Exception as e:
            print(f"WARNING: video save failed: {e}", file=sys.stderr)
    else:
        print("No video frames (headless mode)")

    print("Evaluation complete.")


if __name__ == "__main__":
    main()
