#!/usr/bin/env python3
"""[ADDED] SG2-only Isaac Lab DDS bringup, adapted from sh5_dds_bringup.py.

Keep the existing Vuer -> SG2 arm retarget -> Cyclo VRController unmodified.
This process replaces *only* the robot's joint command / measured joint_states
and TF endpoints.  It neither starts a follower nor activates the VR controller.

Use an isolated ROS/DDS domain; domain 30 requires explicit --allow_domain_30.
"""

import argparse
import importlib.util
import math
import os
import sys
import threading
import time
from copy import deepcopy
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from common.sg2_bridge_contract import (
    ARM_JOINTS, ARM_TOPIC, GRIPPER_MASTER, JOINT_STATES_TOPIC, TF_TOPIC,
    READY_ELBOW_RAD, allowed_names, bounded_target, parse_arm_trajectory,
)


def _parse_args_and_launch():
    # [DEBUG] Pure preflight runs without Isaac Sim or the DDS Python packages.
    quick = argparse.ArgumentParser(add_help=False)
    quick.add_argument("--check-only", action="store_true")
    quick_options, _ = quick.parse_known_args()
    if quick_options.check_only:
        usd = ROOT / "source/cyclo_lab/data/robots/FFW/FFW_SG2.usd"
        print("[CHECK] SG2 USD:", usd, "exists=", usd.is_file())
        for package in ("isaaclab", "cyclonedds", "robotis_dds_python"):
            print(f"[CHECK] {package}:", "FOUND" if importlib.util.find_spec(package) else "MISSING")
        print("[CHECK] Arm DDS topics:", ARM_TOPIC)
        print("[CHECK] Gripper masters:", GRIPPER_MASTER)
        return None, None

    # Validate dependencies BEFORE starting the GPU-heavy Isaac application.
    missing = [p for p in ("isaaclab", "cyclonedds", "robotis_dds_python")
               if importlib.util.find_spec(p) is None]
    if missing:
        raise SystemExit(
            "[ERROR] Missing SG2 DDS Python dependencies: " + ", ".join(missing) +
            ". Set up an isolated CycloneDDS 0.10.2 + robotis_dds_python Python "
            "environment; Isaac Sim alone is not the DDS bridge.")

    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(description="SG2 Isaac Sim DDS arm+gripper bridge")
    parser.add_argument("--domain_id", type=int, default=int(os.getenv("ROS_DOMAIN_ID", "231")))
    parser.add_argument("--allow_domain_30", action="store_true",
                        help="Explicitly permit physical-robot domain 30 (only on isolated network).")
    parser.add_argument("--enable_gravity", action="store_true")
    parser.add_argument("--enable_environment", action="store_true")
    parser.add_argument("--enable_camera_views", action="store_true",
                        help="List SG2 camera prims while keeping a usable overview viewport.")
    parser.add_argument("--max_arm_speed_rad_s", type=float, default=1.0)
    parser.add_argument("--command_timeout_s", type=float, default=0.5)
    parser.add_argument("--step_hz", type=float, default=60.0)
    parser.add_argument("--publish_hz", type=float, default=30.0)
    parser.add_argument("--ready_elbow_rad", type=float, default=READY_ELBOW_RAD)
    parser.add_argument("--disable_gripper", action="store_true")
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()

    # [DEBUG] Fail closed if accidentally using the live robot's default domain.
    if not 0 <= args.domain_id <= 232:
        parser.error("DDS domain must be between 0 and 232")
    if args.domain_id == 30 and not args.allow_domain_30:
        parser.error("Domain 30 may reach the real robot. Use domain 231 or "
                     "--allow_domain_30 only after isolating real robot DDS traffic.")
    if not 0.05 <= args.max_arm_speed_rad_s <= 2.0:
        parser.error("max_arm_speed_rad_s must be in [0.05, 2.0]")
    if not 0.1 <= args.command_timeout_s <= 5.0:
        parser.error("command_timeout_s must be in [0.1, 5.0]")
    if not 10 <= args.step_hz <= 240 or not 1 <= args.publish_hz <= args.step_hz:
        parser.error("require 10 <= step_hz <= 240 and 1 <= publish_hz <= step_hz")
    if not math.isfinite(args.ready_elbow_rad):
        parser.error("ready_elbow_rad must be finite")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    app = AppLauncher(args).app
    return args, app


args_cli, simulation_app = _parse_args_and_launch()
if args_cli is None:
    raise SystemExit(0)

# [IMPORTANT] Do not import USD, Isaac Lab simulation helpers or Cyclo tasks
# until AppLauncher has initialized Omni/Kit.
import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from cyclonedds.core import Policy, Qos
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from pxr import UsdGeom, UsdPhysics

from cyclo_lab.assets.robots import FFW_SG2_CFG
from robotis_dds_python.idl.builtin_interfaces.msg import Time_
from robotis_dds_python.idl.geometry_msgs.msg import Quaternion_, Transform_, TransformStamped_, Vector3_
from robotis_dds_python.idl.sensor_msgs.msg import JointState_
from robotis_dds_python.idl.std_msgs.msg import Header_
from robotis_dds_python.idl.tf2_msgs.msg import TFMessage_
from robotis_dds_python.idl.trajectory_msgs.msg import JointTrajectory_
from robotis_dds_python.tools.topic_manager import TopicManager

from common.environment import make_simple_warehouse_environment_cfg


@configclass
class SG2BringupSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )
    environment: AssetBaseCfg = None
    robot: ArticulationCfg = None


def make_robot_cfg():
    robot_cfg = deepcopy(FFW_SG2_CFG)
    robot_cfg.spawn.rigid_props.disable_gravity = not args_cli.enable_gravity
    # [CHANGED] Start the simulated SG2 in the same elbow-bent ready pose
    # used by the physical SG2 teleop controller. No real-robot action.
    for side in ("l", "r"):
        robot_cfg.init_state.joint_pos[f"arm_{side}_joint4"] = args_cli.ready_elbow_rad
    return robot_cfg


def _now_stamp():
    nanoseconds = time.time_ns()
    return Time_(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


def _trajectory_qos():
    return Qos(Policy.Reliability.BestEffort, Policy.Durability.Volatile,
               Policy.History.KeepLast(10))


class SG2DdsBridge:
    """[ADDED] SH5 DDS topic pattern with SG2-only joints and measured TF."""

    def __init__(self, robot, scene, manager):
        self.robot = robot
        self.scene = scene
        self.manager = manager
        self.lock = threading.Lock()
        self.running = True
        self.pending = {side: {} for side in ("l", "r")}
        self.last_update = {side: None for side in ("l", "r")}
        self.stale_logged = set()
        self.warned = set()
        self.joint_index = {name: index for index, name in enumerate(robot.data.joint_names)}
        self.allowed = {side: allowed_names(side, not args_cli.disable_gripper) for side in ("l", "r")}
        for side, required in ARM_JOINTS.items():
            missing = set(required) - set(self.joint_index)
            if missing:
                raise RuntimeError(f"SG2 articulation missing {side} arm joints: {sorted(missing)}")
        if not args_cli.disable_gripper:
            missing = set(GRIPPER_MASTER.values()) - set(self.joint_index)
            if missing:
                raise RuntimeError(f"SG2 articulation missing master gripper joints: {sorted(missing)}")

        self._limits = robot.data.joint_pos_limits[0].detach().cpu().tolist()
        for side in ("l", "r"):
            for name in self.allowed[side]:
                lower, upper = self._limits[self.joint_index[name]]
                if not all(map(math.isfinite, (lower, upper))) or lower >= upper:
                    raise RuntimeError(f"USD invalid position limits for {name}: [{lower}, {upper}]")

        # [FIX] In SG2 USD 'base_link' is a fixed transform BELOW rigid 'world'.
        # Resolve its authored offset once, then compose it with measured PhysX
        # rigid-body world pose on every publish. Never pretend base_link is a body.
        self._base_body_index, self._base_offset_pos, self._base_offset_quat = self._base_anchor()
        self.joint_writer = manager.topic_writer(
            topic_name=JOINT_STATES_TOPIC, topic_type=JointState_)
        self.tf_writer = manager.topic_writer(topic_name=TF_TOPIC, topic_type=TFMessage_)
        self.readers = []
        self.threads = []
        for side in ("l", "r"):
            reader = manager.topic_reader(
                topic_name=ARM_TOPIC[side], topic_type=JointTrajectory_, qos=_trajectory_qos())
            worker = threading.Thread(target=self._read_loop, args=(side, reader),
                                      daemon=True, name=f"sg2-{side}-dds")
            self.readers.append(reader)
            self.threads.append(worker)
            worker.start()
            print(f"[DDS] {side} arm input: {ARM_TOPIC[side]}", flush=True)

    def _base_anchor(self):
        from isaacsim.core.utils.stage import get_current_stage
        stage = get_current_stage()
        bases = [prim for prim in stage.Traverse()
                 if prim.GetName() == "base_link" and "/Robot/" in str(prim.GetPath())]
        if len(bases) != 1:
            raise RuntimeError(f"Expected exactly one SG2 Robot/base_link, found {[str(p.GetPath()) for p in bases]}")
        base = bases[0]
        rigid = base
        while rigid.IsValid() and not rigid.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid = rigid.GetParent()
        if not rigid or not rigid.IsValid():
            raise RuntimeError("SG2 USD base_link has no rigid ancestor")
        body_name = rigid.GetName()
        if body_name not in self.robot.data.body_names:
            raise RuntimeError(f"SG2 base rigid body '{body_name}' absent from {self.robot.data.body_names}")
        cache = UsdGeom.XformCache()
        offset = cache.GetLocalToWorldTransform(base) * cache.GetLocalToWorldTransform(rigid).GetInverse()
        position = list(map(float, offset.ExtractTranslation()))
        quaternion = offset.ExtractRotationQuat()
        qwxyz = [float(quaternion.GetReal())] + list(map(float, quaternion.GetImaginary()))
        if not all(map(math.isfinite, position + qwxyz)):
            raise RuntimeError("SG2 base frame has a nonfinite authored offset")
        print(f"[TF] fixed base_link under rigid {body_name}; authored offset={position}", flush=True)
        return self.robot.data.body_names.index(body_name), position, qwxyz

    def _warn_once(self, key, message):
        if key not in self.warned:
            self.warned.add(key)
            print("[WARN] " + message, flush=True)

    def _read_loop(self, side, reader):
        try:
            while self.running:
                for msg in reader.take_iter():
                    if not self.running:
                        break
                    if msg is None or not msg.points:
                        continue
                    parsed = parse_arm_trajectory(
                        side, msg.joint_names, msg.points[-1].positions,
                        self.joint_index, not args_cli.disable_gripper)
                    if parsed is None:
                        self._warn_once(f"invalid_{side}",
                            f"Rejected invalid/unexpected {side} arm trajectory (joint names or positions).")
                        continue
                    with self.lock:
                        self.pending[side].update(parsed)
                        self.last_update[side] = time.monotonic()
                        self.stale_logged.discard(side)
                time.sleep(0.001)
        except Exception as error:
            self._warn_once(f"reader_error_{side}", f"DDS reader {side} stopped: {error}")

    def apply_latest_targets(self, dt):
        with self.lock:
            commands = {side: dict(self.pending[side]) for side in ("l", "r")}
            stamps = dict(self.last_update)
        target = self.robot.data.joint_pos_target.clone()
        measured = self.robot.data.joint_pos
        now = time.monotonic()
        max_step = args_cli.max_arm_speed_rad_s * dt
        for side in ("l", "r"):
            last = stamps[side]
            if last is None:
                continue
            if now - last > args_cli.command_timeout_s:
                # [FIX] Tracking pause/dropout cannot leave an indefinitely
                # moving arm target. Hold the *measured* pose, not the stale goal.
                for name in self.allowed[side]:
                    index = self.joint_index[name]
                    target[0, index] = measured[0, index]
                if side not in self.stale_logged:
                    self.stale_logged.add(side)
                    print(f"[SAFETY] {side} arm command stale: holding measured joints", flush=True)
                continue
            for name, requested in commands[side].items():
                index = self.joint_index[name]
                current = float(target[0, index])
                lower, upper = self._limits[index]
                commanded = bounded_target(current, requested, lower, upper, max_step)
                if commanded is None:
                    self._warn_once(f"limit_{name}",
                                    f"Rejected {name} target={requested:.4f}, USD limits=[{lower:.4f},{upper:.4f}]")
                    continue
                target[0, index] = commanded
        self.robot.set_joint_position_target(target)

    def publish_feedback(self):
        from torch import tensor
        stamp = _now_stamp()
        names = list(self.robot.data.joint_names)
        measured = self.robot.data
        joint_msg = JointState_(
            header=Header_(stamp=stamp, frame_id="base_link"),
            name=names,
            position=measured.joint_pos[0].detach().cpu().tolist(),
            velocity=measured.joint_vel[0].detach().cpu().tolist(),
            effort=[0.0] * len(names),
        )
        self.joint_writer.write(joint_msg)

        body_pose = measured.body_link_state_w[0, :, :7]
        idx = self._base_body_index
        base_pos, base_q = math_utils.combine_frame_transforms(
            body_pose[idx, :3].unsqueeze(0), body_pose[idx, 3:7].unsqueeze(0),
            tensor(self._base_offset_pos, device=body_pose.device).unsqueeze(0),
            tensor(self._base_offset_quat, device=body_pose.device).unsqueeze(0),
        )
        transforms = []
        for body_id, name in enumerate(measured.body_names):
            if body_id == idx or name == "base_link":
                continue
            position, quaternion = math_utils.subtract_frame_transforms(
                base_pos, base_q, body_pose[body_id, :3].unsqueeze(0),
                body_pose[body_id, 3:7].unsqueeze(0))
            xyz = position[0].detach().cpu().tolist()
            wxyz = quaternion[0].detach().cpu().tolist()
            transforms.append(TransformStamped_(
                header=Header_(stamp=stamp, frame_id="base_link"), child_frame_id=name,
                transform=Transform_(
                    translation=Vector3_(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])),
                    rotation=Quaternion_(x=float(wxyz[1]), y=float(wxyz[2]),
                                         z=float(wxyz[3]), w=float(wxyz[0])))))
        self.tf_writer.write(TFMessage_(transforms=transforms))

    def shutdown(self):
        self.running = False
        for reader in self.readers:
            try:
                reader.Close()
            except Exception:
                pass
        for worker in self.threads:
            worker.join(timeout=1.0)
        for writer in (self.joint_writer, self.tf_writer):
            try:
                writer.Close()
            except Exception:
                pass


def main():
    usd_path = Path(FFW_SG2_CFG.spawn.usd_path)
    if not usd_path.is_file():
        raise FileNotFoundError(f"SG2 USD missing: {usd_path}")
    print("[INFO] SG2 DDS bridge domain=", args_cli.domain_id, flush=True)
    print("[INFO] Arm controller remains responsible for IK, startup/reference checks.", flush=True)
    print("[INFO] Sim does not publish to any motor/hardware-specific controller.", flush=True)
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
        device=args_cli.device, dt=1.0 / args_cli.step_hz, render_interval=1))
    sim.set_camera_view((2.5, -2.1, 1.7), (0.0, 0.0, 0.8))
    cfg = SG2BringupSceneCfg(num_envs=1, env_spacing=2.0)
    cfg.robot = make_robot_cfg().replace(prim_path="{ENV_REGEX_NS}/Robot")
    if args_cli.enable_environment:
        cfg.environment = make_simple_warehouse_environment_cfg()
    scene = InteractiveScene(cfg)
    sim.reset()
    scene.reset()
    scene.update(sim.get_physics_dt())
    robot = scene["robot"]
    # [FIX] ArticulationCfg.init_state is only a configured default until it is
    # written to PhysX. Without this step, the measured elbow starts at zero
    # even though the intended SG2 controller ready pose is -1.57 rad.
    initial_q = robot.data.default_joint_pos.clone()
    initial_dq = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(initial_q, initial_dq)
    robot.set_joint_position_target(initial_q)
    robot.set_joint_velocity_target(initial_dq)
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim.get_physics_dt())
    print("[READY POSE] SG2 simulated elbow joint4 initialised to",
          args_cli.ready_elbow_rad, "rad (no physical robot movement)", flush=True)
    if args_cli.enable_camera_views:
        from isaacsim.core.utils.stage import get_current_stage
        cameras = [str(prim.GetPath()) for prim in get_current_stage().Traverse()
                   if prim.GetTypeName() == "Camera"]
        print("[INFO] Available SG2 USD camera prims:", cameras, flush=True)
    manager = TopicManager(domain_id=args_cli.domain_id)
    bridge = SG2DdsBridge(robot, scene, manager)
    print("[READY] SG2 DDS simulation: joint_states, TF and arm JointTrajectory connected", flush=True)
    sim_dt = sim.get_physics_dt()
    publish_interval = 1.0 / args_cli.publish_hz
    last_publish = 0.0
    try:
        while simulation_app.is_running():
            start = time.monotonic()
            bridge.apply_latest_targets(sim_dt)
            scene.write_data_to_sim()
            sim.step()
            scene.update(sim_dt)
            if time.monotonic() - last_publish >= publish_interval:
                bridge.publish_feedback()
                last_publish = time.monotonic()
            remaining = sim_dt - (time.monotonic() - start)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        bridge.shutdown()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
