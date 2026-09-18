#!/usr/bin/env python3
# Copyright 2026 Config Intelligence Inc.
# Licensed under the Apache License, Version 2.0.
"""ffw_sg2_ai (follower + leader + cameras) plus the Config Loop streamer.

Equivalent to the ``ffw_sg2_ai`` alias, with the streamer started after the
cameras (follower t=0, cameras t=10..20 s, leader t=30 s in ffw_bringup).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = os.path.join(get_package_share_directory('ffw_bringup'), 'launch')
    here = os.path.join(get_package_share_directory('ffw_loop_streamer'), 'launch')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'ffw_sg2_ai.launch.py')))
    streamer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'loop_streamer.launch.py')),
        launch_arguments={
            'robot_type': 'ffw_sg2_rev1',
            'loop_addr': LaunchConfiguration('loop_addr'),
            'fps': LaunchConfiguration('fps'),
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument(
            'loop_addr', default_value=os.environ.get('LOOP_ADDR', 'localhost:50051'),
            description='Config Loop gRPC address host:port (env LOOP_ADDR)'),
        DeclareLaunchArgument('fps', default_value='30.0'),
        DeclareLaunchArgument(
            'streamer_delay', default_value='25.0',
            description='Seconds after bringup before the streamer starts (cameras need 10-20 s)'),
        robot,
        TimerAction(period=LaunchConfiguration('streamer_delay'), actions=[streamer]),
    ])
