"""Run the installed VR controller in an isolated ROS domain with synthetic input."""

import math
import os
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


assert os.environ.get('ROS_DOMAIN_ID') == '231', 'Only run in isolated domain 231'
assert os.environ.get('RMW_IMPLEMENTATION') == 'rmw_fastrtps_cpp'
root = Path('/root/ros2_ws/install')
exe = root / 'cyclo_motion_controller_ros/lib/cyclo_motion_controller_ros/vr_controller_node'
model = root / 'cyclo_motion_controller_models/share/cyclo_motion_controller_models/models/ai_worker'
log = open('/tmp/quest_gripper_forwarding_test.log', 'w')
process = subprocess.Popen([
    str(exe), '--ros-args',
    '-p', f'urdf_path:={model}/ffw_sg2_follower.urdf',
    '-p', f'srdf_path:={model}/ffw_sg2_follower_default.srdf',
], stdout=log, stderr=subprocess.STDOUT)
rclpy.init(args=[])
node = Node('quest_gripper_forwarding_test')
latest, measured = {}, {}
raw, goals = {}, {}
for side, short in (('left', 'l'), ('right', 'r')):
    prefix = f'/leader/joint_trajectory_command_broadcaster_{side}'
    raw[side] = node.create_publisher(JointTrajectory, prefix + '/raw_joint_trajectory', 10)
    goals[side] = node.create_publisher(PoseStamped, f'/{short}_goal_pose', 10)
    node.create_subscription(JointTrajectory, prefix + '/joint_trajectory',
                             lambda msg, side=side: latest.update({side: msg}), 10)
    node.create_subscription(PoseStamped, f'/{short}_gripper_pose',
                             lambda msg, side=side: measured.update({side: msg}), 10)
joint_pub = node.create_publisher(JointState, '/joint_states', 10)
activate_pub = node.create_publisher(Bool, '/reactivate', 10)
joint_msg = JointState()
joint_msg.name = ['joint'] + [f'arm_{s}_joint{i}' for s in ('l', 'r') for i in range(1, 8)]
joint_msg.name += ['gripper_l_joint1', 'gripper_r_joint1']
joint_msg.position = [0.] + [-.1, .25, 0., -.9, 0., 0., 0.] + [-.1, -.25, 0., -.9, 0., 0., 0.] + [0., 0.]
joint_msg.velocity = [0.] * len(joint_msg.name)


def tick(duration, values=None, activate=None):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        assert process.poll() is None, 'Controller exited; check test log'
        joint_msg.header.stamp = node.get_clock().now().to_msg()
        joint_pub.publish(joint_msg)
        for side in goals:
            if side in measured:
                goals[side].publish(measured[side])
            if values is not None:
                cmd = JointTrajectory()
                cmd.joint_names = [f'gripper_{side[0]}_joint1']
                cmd.points = [JointTrajectoryPoint(positions=[values[side]])]
                raw[side].publish(cmd)
        if activate is not None:
            activate_pub.publish(Bool(data=activate))
        rclpy.spin_once(node, timeout_sec=.01)


try:
    tick(3.)
    assert len(measured) == 2, 'No measured wrist poses'
    tick(4., activate=True)
    assert len(latest) == 2, 'Controller did not activate'
    for msg in latest.values():
        assert len(msg.joint_names) == 7, 'No trigger input should not synthesize a gripper command'
    print('PASS: no gripper command before first trigger input', flush=True)
    for left, right in ((0., 1.3), (.65, .25), (1.3, 0.)):
        values = {'left': left, 'right': right}
        tick(.6, values=values)
        for side, expected in values.items():
            msg = latest[side]
            assert len(msg.joint_names) == len(msg.points[0].positions) == 8
            assert msg.joint_names[:7] == [f'arm_{side[0]}_joint{i}' for i in range(1, 8)]
            idx = msg.joint_names.index(f'gripper_{side[0]}_joint1')
            assert math.isclose(msg.points[0].positions[idx], expected, abs_tol=1e-9)
        print(f'PASS: left={left}, right={right} forwarded with 7 arm joints', flush=True)
    print('PASS: installed controller forwards open / partial / closed commands in domain 231', flush=True)
finally:
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
    node.destroy_node()
    rclpy.shutdown()
    log.close()
