#!/usr/bin/env python3
"""Run ONNX policy inference for Open Duck Mini v2 in MuJoCo with rendering."""

import argparse
import math
import time
import numpy as np
import mujoco
import mujoco.viewer

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SCENE_XML = str(REPO_ROOT / "models" / "open_duck_mini_v2" / "scene.xml")

# Default pose matches KNEES_BENT_KEYFRAME from training constants
DEFAULT_POSE = np.array([
    0.0,   # left_hip_yaw
    0.0,   # left_hip_roll
   -0.4,   # left_hip_pitch
    0.8,   # left_knee
   -0.4,   # left_ankle
    0.0,   # neck_pitch
    0.0,   # head_pitch
    0.0,   # head_yaw
    0.0,   # head_roll
    0.0,   # right_hip_yaw
    0.0,   # right_hip_roll
    0.4,   # right_hip_pitch
    0.8,   # right_knee
   -0.4,   # right_ankle
], dtype=np.float32)


class PolicyInference:
    def __init__(self, model, data, onnx_path, action_scale=1.0, use_projected_gravity=True,
                 delay_min_lag=0, delay_max_lag=0):
        import onnxruntime as ort

        self.model = model
        self.data = data
        self.action_scale = action_scale
        self.use_projected_gravity = use_projected_gravity
        self.delay_min_lag = delay_min_lag
        self.delay_max_lag = delay_max_lag

        print(f"Loading policy from: {onnx_path}")
        self.session = ort.InferenceSession(onnx_path)
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        print(f"  Input:  {self.input_name}, shape: {self.session.get_inputs()[0].shape}")
        print(f"  Output: {self.output_name}, shape: {self.session.get_outputs()[0].shape}")

        # Sensor IDs
        self.gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_ang_vel")
        self.accel_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_accel")
        self.trunk_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "trunk_assembly")

        if self.gyro_id < 0:
            raise RuntimeError("Sensor 'imu_ang_vel' not found in model")

        self.n_joints = model.nu
        self.default_pose = DEFAULT_POSE[:self.n_joints]
        print(f"Actuators: {self.n_joints}")

        self.last_action = np.zeros(self.n_joints, dtype=np.float32)
        self.vel_cmd = np.zeros(3, dtype=np.float32)

        # Action delay buffer
        self.use_delay = delay_max_lag > 0
        if self.use_delay:
            buf = delay_max_lag + 1
            self.action_buffer = [np.zeros(self.n_joints, dtype=np.float32) for _ in range(buf)]
            self.buffer_index = 0
            self.current_lag = np.random.randint(delay_min_lag, delay_max_lag + 1)
            print(f"Actuator delay: lag={self.current_lag} steps (min={delay_min_lag}, max={delay_max_lag})")

    def quat_rotate_inverse(self, quat, vec):
        """Rotate vec by inverse of quaternion [w, x, y, z]."""
        w = quat[0]
        xyz = quat[1:4]
        t = np.cross(xyz, vec) * 2
        return vec - w * t + np.cross(xyz, t)

    def get_projected_gravity(self):
        quat = self.data.xquat[self.trunk_id].copy().astype(np.float32)
        return self.quat_rotate_inverse(quat, np.array([0.0, 0.0, -1.0], dtype=np.float32))

    def get_raw_accelerometer(self):
        adr = self.model.sensor_adr[self.accel_id]
        raw = -self.data.sensordata[adr:adr + 3].copy().astype(np.float32)
        mag = np.linalg.norm(raw)
        if mag > 0.1:
            return raw / mag
        quat = self.data.xquat[self.trunk_id].copy().astype(np.float32)
        return self.quat_rotate_inverse(quat, np.array([0.0, 0.0, -1.0], dtype=np.float32))

    def get_base_ang_vel(self):
        adr = self.model.sensor_adr[self.gyro_id]
        return self.data.sensordata[adr:adr + 3].copy().astype(np.float32)

    def get_joint_pos_relative(self):
        return self.data.qpos[7:7 + self.n_joints].copy().astype(np.float32) - self.default_pose

    def get_joint_vel(self):
        return self.data.qvel[6:6 + self.n_joints].copy().astype(np.float32)

    def get_observations(self):
        """Build 51D observation matching training policy:
          base_ang_vel (3) + projected_gravity (3) + joint_pos (14) + joint_vel (14) + last_action (14) + command (3)
        """
        grav = self.get_projected_gravity() if self.use_projected_gravity else self.get_raw_accelerometer()
        return np.concatenate([
            self.get_base_ang_vel(),
            grav,
            self.get_joint_pos_relative(),
            self.get_joint_vel(),
            self.last_action,
            self.vel_cmd,
        ]).astype(np.float32)

    def set_vel_cmd(self, x=0.0, y=0.0, yaw=0.0):
        self.vel_cmd = np.array([x, y, yaw], dtype=np.float32)
        print(f"Vel cmd: [{x:.2f}, {y:.2f}, {yaw:.2f}]")

    def infer(self):
        obs = self.get_observations().reshape(1, -1)
        action = self.session.run([self.output_name], {self.input_name: obs})[0].squeeze(0).astype(np.float32)
        self.last_action = action.copy()
        return action

    def apply_action(self, action):
        if self.use_delay:
            self.action_buffer[self.buffer_index] = action.copy()
            delayed = self.action_buffer[(self.buffer_index - self.current_lag) % len(self.action_buffer)]
            self.buffer_index = (self.buffer_index + 1) % len(self.action_buffer)
            self.data.ctrl[:] = self.default_pose + delayed * self.action_scale
        else:
            self.data.ctrl[:] = self.default_pose + action * self.action_scale


def main():
    parser = argparse.ArgumentParser(description="Run ONNX policy for Open Duck Mini v2 in MuJoCo")
    parser.add_argument("--policy", "-p", type=str, required=True, help="Path to policy ONNX file")
    parser.add_argument("--scene", type=str, default=SCENE_XML, help=f"Path to scene XML (default: {SCENE_XML})")
    parser.add_argument("--lin-vel-x", type=float, default=0.0)
    parser.add_argument("--lin-vel-y", type=float, default=0.0)
    parser.add_argument("--ang-vel-z", type=float, default=0.0)
    parser.add_argument("--action-scale", type=float, default=1.0)
    parser.add_argument("--raw-accelerometer", action="store_true", help="Use raw accelerometer instead of projected gravity")
    parser.add_argument("--delay", type=int, nargs="*", default=None, help="Actuator delay: --delay MIN MAX or --delay LAG")
    parser.add_argument("--debug", action="store_true", help="Print obs/action each step")
    args = parser.parse_args()

    delay_min, delay_max = 0, 0
    if args.delay is not None:
        if len(args.delay) == 0:
            delay_min, delay_max = 1, 2
        elif len(args.delay) == 1:
            delay_min = delay_max = args.delay[0]
        else:
            delay_min, delay_max = args.delay[0], args.delay[1]

    print(f"Loading MuJoCo model: {args.scene}")
    model = mujoco.MjModel.from_xml_path(args.scene)
    model.opt.timestep = 0.005
    data = mujoco.MjData(model)

    policy = PolicyInference(
        model, data,
        onnx_path=args.policy,
        action_scale=args.action_scale,
        use_projected_gravity=not args.raw_accelerometer,
        delay_min_lag=delay_min,
        delay_max_lag=delay_max,
    )
    policy.set_vel_cmd(args.lin_vel_x, args.lin_vel_y, args.ang_vel_z)

    # Set initial state to default pose
    freejoint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "trunk_assembly_freejoint")
    qpos_adr = model.jnt_qposadr[freejoint_id]
    data.qpos[qpos_adr:qpos_adr + 3] = [0.0, 0.0, 0.22]
    data.qpos[qpos_adr + 3:qpos_adr + 7] = [1, 0, 0, 0]
    data.qpos[7:7 + policy.n_joints] = policy.default_pose
    data.ctrl[:] = policy.default_pose
    mujoco.mj_forward(model, data)

    obs_size = policy.get_observations().size
    expected = 3 + 3 + policy.n_joints * 3 + 3  # ang_vel + grav + joints*3 + cmd
    print(f"\nObservation size: {obs_size} (expected {expected})")
    if obs_size != expected:
        print("WARNING: size mismatch — check DEFAULT_POSE and joint count")

    # Keyboard controls
    try:
        from pynput import keyboard as kb

        def on_press(key):
            try:
                if key == kb.Key.up:
                    policy.set_vel_cmd(0.3, policy.vel_cmd[1], policy.vel_cmd[2])
                elif key == kb.Key.down:
                    policy.set_vel_cmd(-0.3, policy.vel_cmd[1], policy.vel_cmd[2])
                elif key == kb.Key.right:
                    policy.set_vel_cmd(policy.vel_cmd[0], -0.3, policy.vel_cmd[2])
                elif key == kb.Key.left:
                    policy.set_vel_cmd(policy.vel_cmd[0], 0.3, policy.vel_cmd[2])
                elif key == kb.Key.space:
                    policy.set_vel_cmd(0.0, 0.0, 0.0)
                elif hasattr(key, "char"):
                    if key.char in ("a", "A"):
                        policy.set_vel_cmd(policy.vel_cmd[0], policy.vel_cmd[1], 1.0)
                    elif key.char in ("e", "E"):
                        policy.set_vel_cmd(policy.vel_cmd[0], policy.vel_cmd[1], -1.0)
            except Exception as exc:
                print(f"Key error: {exc}")

        listener = kb.Listener(on_press=on_press)
        listener.start()

        print("\nKeyboard controls:")
        print("  UP / DOWN:    lin_vel_x  ±0.3 m/s")
        print("  LEFT / RIGHT: lin_vel_y  ±0.3 m/s")
        print("  A / E:        ang_vel_z  ±1.0 rad/s")
        print("  SPACE:        stop")
    except ImportError:
        print("\nKeyboard control unavailable: install pynput")

    decimation = 4
    control_dt = decimation * model.opt.timestep
    step = 0

    print("\nOpen Duck Mini v2 Policy Inference — close viewer to exit\n")

    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
        viewer.sync()
        while viewer.is_running():
            t0 = time.time()

            action = policy.infer()
            policy.apply_action(action)
            step += 1

            if args.debug and (step <= 5 or step % 50 == 0):
                obs = policy.get_observations()
                pos = data.qpos[qpos_adr:qpos_adr + 3]
                print(f"\nStep {step}  pos={pos}  cmd={policy.vel_cmd}")
                print(f"  ang_vel={obs[0:3]}")
                print(f"  gravity={obs[3:6]}")
                print(f"  joint_pos={obs[6:20]}")
                print(f"  action={action}")

            for _ in range(decimation):
                mujoco.mj_step(model, data)
            viewer.sync()

            elapsed = time.time() - t0
            if control_dt - elapsed > 0:
                time.sleep(control_dt - elapsed)

    print("Done.")


if __name__ == "__main__":
    main()
