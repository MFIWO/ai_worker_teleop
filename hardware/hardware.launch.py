"""[ADDED] Existing Cyclo + checker + robot-side lease gate. No follower bringup."""
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

from hardware.xr_contract import load_profile


def generate_launch_description():
    profile_file = os.environ['AI_WORKER_XR_PROFILE']
    p = load_profile(profile_file)
    root = Path(__file__).resolve().parents[1]
    gate = ExecuteProcess(cmd=[sys.executable, '-m', 'hardware.command_gate', '--profile', profile_file], output='screen')
    hands = p.get('hand_control') == 'robotis_native_measured_fingers'
    build_root = 'ai_worker_xr_sh5_hands' if hands else 'ai_worker_xr_startup_steady'
    controller = Node(package='cyclo_motion_controller_ros',
                      executable=f'/workspace/{build_root}/build/cyclo_motion_controller_ros/vr_controller_node',
                      name='vr_controller', parameters=[p['config'], {'urdf_path': p['urdf'], 'srdf_path': p['srdf'],
                      'activation_delay_seconds': 0.1, 'startup_ramp_seconds': 0.1,
                      'joint_state_timeout': 5.0}],
                      output='screen')
    checker = Node(package='cyclo_motion_controller_ros', executable='reference_checker_node',
                   name='reference_checker', parameters=[p['config']], output='screen')
    actions = [gate, controller, checker]
    if hands:
        actions.append(Node(package='cyclo_motion_controller_ros_py', executable='retargeting_teleop',
                            name='retargeting_teleop', output='screen'))
    for process in tuple(actions):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='hardware control component exited'))])))
    return LaunchDescription(actions)
