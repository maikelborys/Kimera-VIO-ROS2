"""EuRoC wrapper around the base kimera_vio_ros launch.

Defaults assume a `ros2 bag play <V1_01_easy converted bag> --clock` is
running in another terminal. The EuRoC bag publishes:
  /cam0/image_raw, /cam1/image_raw, /imu0, /leica/position
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
            default_value='/home/maikel/coding/MITSPARK/Kimera-VIO/params/Euroc',
            description='Kimera-VIO param folder (Euroc by default).'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_lcd',      default_value='false'),
        DeclareLaunchArgument('use_lcd_registration_server',
                              default_value='false'),
        DeclareLaunchArgument('visualize',    default_value='false'),
    ]

    return LaunchDescription(args + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base),
            launch_arguments={
                'params_folder':       LaunchConfiguration('params_folder'),
                'use_sim_time':        LaunchConfiguration('use_sim_time'),
                'use_lcd':             LaunchConfiguration('use_lcd'),
                'use_lcd_registration_server':
                    LaunchConfiguration('use_lcd_registration_server'),
                'visualize':           LaunchConfiguration('visualize'),
                'left_cam_topic':      '/cam0/image_raw',
                'right_cam_topic':     '/cam1/image_raw',
                'imu_topic':           '/imu0',
                'base_link_frame_id':  'base_link',
                'odom_frame_id':       'world',
                'map_frame_id':        'map',
                'left_cam_frame_id':   'cam0',
                'right_cam_frame_id':  'cam1',
            }.items(),
        ),
    ])
