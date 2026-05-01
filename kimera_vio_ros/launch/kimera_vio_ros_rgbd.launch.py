"""RGBD+IMU launch wrapper.

Slice B sanity wrapper: requires a Kimera-VIO param folder configured for
the RGBD+IMU frontend (`vio_params.frontend_type_ == kRgbdImu`). The frontend
choice is controlled by the params folder, NOT by this launch file.

Topology mapped onto a typical RealSense pipeline:
  /camera/color/image_raw  -> left_cam/image_raw  (color frame)
  /camera/depth/image_raw  -> depth_cam/image_raw (registered depth)
  /camera/imu              -> imu

Use this launch as a topology check; full RGBD validation needs an RGBD
dataset which is out of scope for the EuRoC-only Phase 2 PAUSE.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base = os.path.join(
        get_package_share_directory('kimera_vio_ros'),
        'launch', 'kimera_vio_ros.launch.py')

    args = [
        DeclareLaunchArgument(
            'params_folder',
            description='Path to a Kimera-VIO param folder (RGBD+IMU).'),
        DeclareLaunchArgument('use_sim_time',     default_value='true'),
        DeclareLaunchArgument('color_cam_topic',  default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_cam_topic',  default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('imu_topic',        default_value='/camera/imu'),
    ]

    rgbd_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base),
        launch_arguments={
            'params_folder':       LaunchConfiguration('params_folder'),
            'use_sim_time':        LaunchConfiguration('use_sim_time'),
            'left_cam_topic':      LaunchConfiguration('color_cam_topic'),
            'depth_cam_topic':     LaunchConfiguration('depth_cam_topic'),
            'imu_topic':           LaunchConfiguration('imu_topic'),
            # right_cam_topic is unused on the RGBD path; placeholder.
            'right_cam_topic':     '/unused/right_cam',
            'right_cam_frame_id':  'cam1',
        }.items(),
    )

    return LaunchDescription(args + [rgbd_node])
