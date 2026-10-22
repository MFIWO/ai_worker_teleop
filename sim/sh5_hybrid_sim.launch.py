"""One simulation stack: native arm QP, checker, original hand IK, watchdog relay."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
import sys
from sim.prepare_sh5_sim import PROFILE, require_sim


def generate_launch_description():
    require_sim()
    config = str(PROFILE / 'controller.yaml')
    processes = [
        ExecuteProcess(cmd=[sys.executable, '-m', 'sim.sh5_sim_bridge'], output='screen'),
        Node(package='cyclo_motion_controller_ros',
             executable='/workspace/sh5_hybrid_sim_build/build/cyclo_motion_controller_ros/vr_controller_node',
             name='vr_controller', parameters=[config], output='screen'),
        Node(package='cyclo_motion_controller_ros', executable='reference_checker_node',
             name='reference_checker', parameters=[config], output='screen'),
        Node(package='cyclo_motion_controller_ros_py', executable='retargeting_teleop',
             name='retargeting_teleop', output='screen', remappings=[
                 (f'/leader/joint_trajectory_command_broadcaster_{s}_hand/joint_trajectory',
                  f'/sh5_sim/raw/{s}_hand') for s in ('left', 'right')]),
    ]
    return LaunchDescription(processes + [RegisterEventHandler(OnProcessExit(
        target_action=p, on_exit=[EmitEvent(event=Shutdown(reason='SH5 sim component exited'))]))
        for p in processes])
