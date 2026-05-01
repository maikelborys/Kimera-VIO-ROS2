"""Mono+IMU launch wrapper.

Slice B sanity wrapper: requires a Kimera-VIO param folder configured for
the mono+IMU frontend (`vio_params.frontend_type_ == kMonoImu`). The wrapper
itself only reroutes left_cam_topic + imu_topic; the frontend choice is
controlled by the params folder, NOT by this launch file.

A reference mono frontend isn't shipped by Kimera-VIO upstream — bring your
own param folder. Use this launch as a topology check (does the package
load and create a Mono pipeline? do the topics show up?).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base = os.path.join(
        get_package_share_directory('kimera_vio_ros'),
        'launch', 'kimera_vio_ros.launch.py')

    args = [
        DeclareLaunchArgument(
            'params_folder',
            description='Path to a Kimera-VIO param folder (mono+IMU).'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('left_cam_topic', default_value='/cam0/image_raw'),
        DeclareLaunchArgument('imu_topic',      default_value='/imu0'),
    ]

    return LaunchDescription(args + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base),
            launch_arguments={
                'params_folder':       LaunchConfiguration('params_folder'),
                'use_sim_time':        LaunchConfiguration('use_sim_time'),
                'left_cam_topic':      LaunchConfiguration('left_cam_topic'),
                'imu_topic':           LaunchConfiguration('imu_topic'),
                # Mono frontend ignores the right cam topic, but the param
                # is required by the data provider's CHECK so give it a
                # placeholder.
                'right_cam_topic':     '/unused/right_cam',
                'right_cam_frame_id':  'cam1',
            }.items(),
        ),
    ])
