#!/usr/bin/env python3
"""[ADDED 2026-10-14] Isolated native Cyclo + real gate + synthetic follower test.

No physical follower is launched. Required domain232 + localhost-only prevent
connection to the robot domain30. The first-order follower is a timing fixture,
NOT a physical motor model. This executable is never copied by install_robot.sh.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    required = {'ROS_DOMAIN_ID': '232', 'ROS_LOCALHOST_ONLY': '1',
                'AI_WORKER_XR_EXECUTION': 'validation', 'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}
    if any(os.environ.get(k) != v for k, v in required.items()):
        raise SystemExit('Synthetic runtime test requires validation/domain232/localhost/FastDDS')
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--follower-tau', type=float, default=.15)
    args = p.parse_args()
    if not 0 <= args.follower_tau <= .5:
        p.error('fixture tau must be 0..0.5s')

    import numpy as np
    import pinocchio as pin
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseStamped
    from dynamixel_interfaces.msg import DynamixelState
    from std_msgs.msg import String, Bool
    from trajectory_msgs.msg import JointTrajectory
    from ament_index_python.packages import get_package_prefix
    from hardware.command_gate import HardwareCommandGate
    from hardware.xr_contract import ARM_DXL_IDS, ARM_NAMES, FOLLOWER_TOPICS

    profile = json.loads(args.profile.read_text())
    if profile['model'] != 'sg2':
        raise ValueError('This concrete failure reproducer is SG2-only')
    # [TEST ONLY] Copy manifest into a temporary directory; mark as synthetic.
    profile['validation_fixture'] = True
    profile['readiness_policy'] = 'driver_managed'
    model = pin.buildModelFromUrdf(profile['urdf'])
    data = model.createData()
    joints = json.loads((args.profile.parent/'snapshot/joints.json').read_text())
    names = list(profile['controller_joint_names'])
    targets = dict(joints)
    q = pin.neutral(model)
    def poses():
        for i in range(1, model.njoints):
            q[model.joints[i].idx_q] = joints[model.names[i]]
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        return {s: data.oMf[model.getFrameId(f'arm_{v}_link7')].copy()
                for s, v in (('left','l'),('right','r'))}
    held = poses()
    rclpy.init(args=[])
    synthetic = Node('ai_worker_xr_hardware')
    executor = SingleThreadedExecutor()
    executor.add_node(synthetic)
    js = synthetic.create_publisher(JointState, '/joint_states', 10)
    motor = synthetic.create_publisher(DynamixelState, '/ffw_follower/dxl_state', 10)
    desc = synthetic.create_publisher(String, '/robot_description',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    desc.publish(String(data=(args.profile.parent/'snapshot/robot_description.urdf').read_text()))
    intent = synthetic.create_publisher(String, '/xr_hardware/intent', 10)
    activate = synthetic.create_publisher(Bool, '/reactivate', 10)
    goals = {s: synthetic.create_publisher(PoseStamped, '/'+v+'_goal_pose', 10)
             for s,v in (('left','l'),('right','r'))}
    first_command = {}
    output_count = 0
    timing_ok = True
    def output(msg, side):
        nonlocal output_count, timing_ok
        output_count += 1
        first_command.setdefault(side, time.monotonic())
        timing_ok &= (len(msg.points) == 1 and msg.header.stamp.sec == 0
                      and msg.header.stamp.nanosec == 0
                      and msg.points[0].time_from_start.sec == 0
                      and msg.points[0].time_from_start.nanosec == 0
                      and list(msg.joint_names) == list(ARM_NAMES[side]))
        if msg.points:
            targets.update(zip(msg.joint_names, msg.points[0].positions))
    subs = [synthetic.create_subscription(JointTrajectory, FOLLOWER_TOPICS[s],
            lambda msg, side=s: output(msg,side), 10) for s in goals]
    processes = []
    gate = None
    result = {'physical_hardware_test': False, 'backend': 'synthetic_first_order_follower',
              'native_cyclo': True, 'real_rclpy_gate': True, 'follower_tau_s': args.follower_tau,
              'domain': 232, 'localhost_only': True}
    trace = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log_handles = []
    try:
        with tempfile.TemporaryDirectory(prefix='sg2_gate_profile_') as temporary:
            profile_file = Path(temporary)/'profile.json'
            profile_file.write_text(json.dumps(profile))
            gate = HardwareCommandGate(profile_file)
            executor.add_node(gate)
            prefix = Path(get_package_prefix('cyclo_motion_controller_ros'))
            for executable in ('vr_controller_node', 'reference_checker_node'):
                log = args.output.with_name(args.output.stem+'_'+executable+'.log').open('w')
                log_handles.append(log)
                cmd = [str(prefix/'lib/cyclo_motion_controller_ros'/executable), '--ros-args',
                       '--params-file', profile['config']]
                if executable == 'vr_controller_node':
                    cmd += ['-p', 'urdf_path:='+profile['urdf'], '-p', 'srdf_path:='+profile['srdf']]
                processes.append(subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT))
            start = last_tick = time.monotonic()
            enabled = False
            sequence = 0
            max_hold_drift = max_hold_angle = 0.
            initial = {s: value.translation.copy() for s,value in held.items()}
            while time.monotonic()-start < 28.:
                for _ in range(12):
                    executor.spin_once(timeout_sec=0.)
                now = time.monotonic()
                dt = now-last_tick
                if dt < .01:
                    time.sleep(.001)
                    continue
                last_tick = now
                factor = 1. if args.follower_tau == 0 else -math.expm1(-dt/args.follower_tau)
                previous = dict(joints)
                for name in names:
                    joints[name] += factor*(targets[name]-joints[name])
                stamp = synthetic.get_clock().now().to_msg()
                state = JointState()
                state.header.stamp = stamp
                state.name = names
                state.position = [joints[n] for n in names]
                state.velocity = [(joints[n]-previous[n])/dt for n in names]
                js.publish(state)
                health = DynamixelState()
                health.header.stamp = stamp
                health.id = ARM_DXL_IDS['right']+ARM_DXL_IDS['left']
                health.torque_state = [False]*14  # Explicit unsupported telemetry fixture, not proof ON.
                health.dxl_hw_state = [0]*14
                health.comm_state = 0
                motor.publish(health)
                warm = now-max(first_command.values()) if len(first_command)==2 else -1.
                u = min(1., max(0., (warm-8.)/3.))
                shift = .02*(3*u*u-2*u*u*u)
                actual = poses()
                for side in goals:
                    goal = held[side].copy()
                    goal.translation[0] += shift
                    msg = PoseStamped()
                    msg.header.stamp = stamp
                    msg.header.frame_id = 'base_link'
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float,goal.translation)
                    quat = pin.Quaternion(goal.rotation)
                    msg.pose.orientation.x = float(quat.x)
                    msg.pose.orientation.y = float(quat.y)
                    msg.pose.orientation.z = float(quat.z)
                    msg.pose.orientation.w = float(quat.w)
                    goals[side].publish(msg)
                    if enabled and warm < 8.:
                        max_hold_drift = max(max_hold_drift, float(np.linalg.norm(actual[side].translation-initial[side])))
                        max_hold_angle = max(max_hold_angle, float(np.linalg.norm(pin.log3(actual[side].rotation@held[side].rotation.T))))
                if not enabled and now-start > 2.:
                    try:
                        gate.graph()
                        if gate.description_ok and gate.motor_ok and now-gate.measured_rx < .1:
                            enabled = True
                            activate.publish(Bool(data=True))
                    except ValueError:
                        pass
                sequence += 1
                intent.publish(String(data=json.dumps({'model':'sg2', 'enabled':enabled,
                    'session_id':'isolated_runtime_fixture', 'sequence':sequence,
                    'ros_time_s':synthetic.get_clock().now().nanoseconds*1e-9})))
                trace.append({'t':now-start, 'warm_s':warm, 'left_q6_actual':joints['arm_l_joint6'],
                              'left_q6_target':targets['arm_l_joint6'], 'gate_fault':gate.fault})
                if any(proc.poll() is not None for proc in processes):
                    raise RuntimeError('Native test process exited unexpectedly')
                if gate.fault:
                    raise RuntimeError(gate.fault)
                if max_hold_drift > .02 or max_hold_angle > math.radians(10):
                    raise RuntimeError('Existing PC startup 20mm/10deg bound would reject the synthetic follower')
                if warm > 12.:
                    displacement = {s: float(actual[s].translation[0]-initial[s][0]) for s in goals}
                    if not timing_ok or any(v < .015 for v in displacement.values()):
                        raise RuntimeError('Timing contract or 20mm forward tracking fixture failed')
                    # Explicit manual disarm for fixture teardown. Lease-loss
                    # behavior is separately covered by test_command_gate.
                    saved_fault = gate.fault
                    intent.publish(String(data=json.dumps({'model':'sg2', 'enabled':False,
                        'session_id':'isolated_runtime_fixture', 'sequence':sequence+1,
                        'ros_time_s':synthetic.get_clock().now().nanoseconds*1e-9})))
                    result.update(passed=True, gate_fault_before_stop=saved_fault,
                        max_startup_wrist_drift_mm=max_hold_drift*1000,
                        max_startup_wrist_angle_deg=math.degrees(max_hold_angle),
                        forward_displacement_m=displacement, forwarded_frames=output_count,
                        immediate_stream_timing=timing_ok)
                    break
            else:
                raise RuntimeError('Runtime fixture timed out before stationary startup and forward stroke completed')
    except Exception as error:
        result.update(passed=False, error=repr(error), gate_detail=getattr(gate,'fault_detail',None))
    finally:
        activate.publish(Bool(data=False))
        if gate is not None:
            gate.hold('isolated test finished')
        for proc in processes:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
        for proc in processes:
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.wait(timeout=3)
        for handle in log_handles:
            handle.close()
        if gate is not None:
            executor.remove_node(gate); gate.destroy_node()
        executor.remove_node(synthetic); synthetic.destroy_node()
        executor.shutdown(); rclpy.try_shutdown()
    args.output.write_text(json.dumps({**result, 'trace':trace}, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result.get('passed') else 1)


if __name__ == '__main__':
    main()
