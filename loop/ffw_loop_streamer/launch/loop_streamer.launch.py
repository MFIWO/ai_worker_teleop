#!/usr/bin/env python3
# Copyright 2026 Config Intelligence Inc.
# Licensed under the Apache License, Version 2.0.
"""Standalone Config Loop streamer for the ROBOTIS AI Worker.

Subscribes to the topics ffw_bringup produces (follower /joint_states, /odom,
leader /leader/*, /cmd_vel, camera compressed images) and streams them to
Config Loop. Read-only: never publishes to the robot.
"""
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ffw_loop_streamer')
    # Config files are keyed /** so they apply to this node regardless of its name.
    config_files = sorted(glob.glob(os.path.join(pkg_dir, 'config', '*.yaml')))

    loop_streamer = Node(
        package='ffw_loop_streamer',
        executable='loop_streamer',
        name='loop_streamer',
        output='screen',
        # Fails fast when Loop is unreachable; respawn retries every 5 s so the
        # streamer may start before Loop (or survive a Loop restart).
        respawn=LaunchConfiguration('respawn'),
        respawn_delay=5.0,
        parameters=config_files + [{
            'robot_type': LaunchConfiguration('robot_type'),
            'loop_addr': LaunchConfiguration('loop_addr'),
            'fps': LaunchConfiguration('fps'),
            'source_key': LaunchConfiguration('source_key'),
            'enable_camera': LaunchConfiguration('enable_camera'),
            'camera_probe_timeout_s': LaunchConfiguration('camera_probe_timeout'),
            'rtsp_advertise_host': LaunchConfiguration('rtsp_advertise_host'),
            'rtsp_base_port': LaunchConfiguration('rtsp_base_port'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_type', default_value=os.environ.get('LOOP_ROBOT_TYPE', 'ffw_sg2_rev1'),
            description='Robot config name (see ffw_loop_streamer/config): ffw_sg2_rev1, ffw_bg2_rev4'),
        DeclareLaunchArgument(
            'loop_addr', default_value=os.environ.get('LOOP_ADDR', 'localhost:50051'),
            description='Config Loop gRPC address host:port (env LOOP_ADDR)'),
        DeclareLaunchArgument(
            'source_key', default_value='robotis',
            description='Loop robot source identity; use robotis-vr for the separate VR layout'),
        DeclareLaunchArgument(
            'fps', default_value='30.0',
            description='robot-step rate and camera RTSP frame rate; match the Loop cell config Hz'),
        DeclareLaunchArgument(
            'enable_camera', default_value='true',
            description='Also stream the configured cameras over RTSP/RTP-JPEG'),
        DeclareLaunchArgument(
            'camera_probe_timeout', default_value='120.0',
            description='Seconds to wait for the first frame of each camera before streaming without it'),
        DeclareLaunchArgument(
            'rtsp_advertise_host', default_value='',
            description='Host Loop uses to pull RTSP; empty = detect from the route to Loop'),
        DeclareLaunchArgument(
            'rtsp_base_port', default_value='8554',
            description='First RTSP listen port; one port per camera'),
        DeclareLaunchArgument(
            'respawn', default_value='true',
            description='Restart the streamer every 5 s while Config Loop is unreachable'),
        loop_streamer,
    ])
