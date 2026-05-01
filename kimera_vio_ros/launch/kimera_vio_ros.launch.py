"""Base launch file for Kimera-VIO-ROS 2.

Slice B: stereo / mono / RGBD frontends supported (frontend choice comes
from the params folder, NOT this launch file). Loads gflags flagfiles
from `<params_folder>/flags/*.flags` so the Mesher and backend pick up the
parameters the original ROS 1 wrapper passed via `--flagfile=...`.

Args of interest:

- ``params_folder``       — path to a Kimera-VIO param dir (e.g. Euroc)
- ``vocabulary_path``     — ORB vocabulary file (default vendored ORBvoc.yml)
- ``left_cam_topic``      — remap target for ``left_cam/image_raw``
- ``right_cam_topic``     — remap target for ``right_cam/image_raw`` (stereo)
- ``depth_cam_topic``     — remap target for ``depth_cam/image_raw`` (RGBD)
- ``imu_topic``           — remap target for ``imu``
- ``base_link_frame_id``, ``odom_frame_id``, ``map_frame_id``,
  ``left_cam_frame_id``, ``right_cam_frame_id`` — TF frame names
- ``use_sim_time``        — true for bag replay
- ``viz_type``            — 0=Mesh2dTo3dSparse (mesh), 1=Pointcloud, 2=None
- ``log_gt_data``         — write GT CSV
- ``use_lcd``             — enable Loop-Closure Detection (Slice C)
- ``use_lcd_registration_server`` — advertise ~/register_lcd_frames service
- ``visualize``           — start mesh_to_marker.py + rviz2 with the
                            bundled `rviz/kimera_vio_euroc.rviz` config
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_FLAG_FILES = (
    'VioBackend.flags',
    'RegularVioBackend.flags',
    'Mesher.flags',
    'Visualizer3D.flags',
)


def _resolve_flagfiles(params_folder: str) -> list[str]:
    flags_dir = os.path.join(params_folder, 'flags')
    args: list[str] = []
    for name in _FLAG_FILES:
        path = os.path.join(flags_dir, name)
        if os.path.exists(path):
            args.append(f'--flagfile={path}')
    return args


def _build_node(context, *args, **kwargs):
    params_folder = LaunchConfiguration('params_folder').perform(context)
    vocab = LaunchConfiguration('vocabulary_path').perform(context)
    viz_type = LaunchConfiguration('viz_type').perform(context)
    use_lcd = LaunchConfiguration('use_lcd').perform(context)
    visualize = (
        LaunchConfiguration('visualize').perform(context).lower() == 'true')

    arguments = _resolve_flagfiles(params_folder) + [
        f'--vocabulary_path={vocab}',
        f'--viz_type={viz_type}',
        f'--use_lcd={use_lcd}',
        '--logtostderr',
        '--colorlogtostderr',
    ]

    nodes = [Node(
        package='kimera_vio_ros',
        executable='kimera_vio_ros_node',
        name='kimera_vio_ros',
        namespace='kimera_vio_ros',
        output='screen',
        emulate_tty=True,
        arguments=arguments,
        parameters=[{
            'use_sim_time':              LaunchConfiguration('use_sim_time'),
            'params_folder_path':        LaunchConfiguration('params_folder'),
            'sensor_params_folder_path': '',
            'use_rviz':                  LaunchConfiguration('use_rviz'),
            'log_gt_data':               LaunchConfiguration('log_gt_data'),
            'use_external_odom':         LaunchConfiguration('use_external_odom'),
            'force_same_image_timestamp':
                LaunchConfiguration('force_same_image_timestamp'),
            'use_lcd_registration_server':
                LaunchConfiguration('use_lcd_registration_server'),
            'base_link_frame_id': LaunchConfiguration('base_link_frame_id'),
            'odom_frame_id':      LaunchConfiguration('odom_frame_id'),
            'map_frame_id':       LaunchConfiguration('map_frame_id'),
            'left_cam_frame_id':  LaunchConfiguration('left_cam_frame_id'),
            'right_cam_frame_id': LaunchConfiguration('right_cam_frame_id'),
        }],
        remappings=[
            ('left_cam/image_raw',  LaunchConfiguration('left_cam_topic')),
            ('right_cam/image_raw', LaunchConfiguration('right_cam_topic')),
            ('depth_cam/image_raw', LaunchConfiguration('depth_cam_topic')),
            ('imu',                 LaunchConfiguration('imu_topic')),
        ],
    )]

    if visualize:
        nodes.append(Node(
            package='kimera_vio_ros',
            executable='mesh_to_marker.py',
            name='mesh_to_marker',
            namespace='kimera_vio_ros',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id':     LaunchConfiguration('odom_frame_id'),
            }],
            remappings=[
                ('mesh',        '/kimera_vio_ros/mesh'),
                ('mesh_marker', '/kimera_vio_ros/mesh_marker'),
            ],
        ))
        rviz_config_path = os.path.join(
            get_package_share_directory('kimera_vio_ros'),
            'rviz', 'kimera_vio_euroc.rviz')
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='kimera_vio_rviz',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            condition=IfCondition(LaunchConfiguration('visualize')),
        ))
    return nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument('params_folder', default_value=''),
        DeclareLaunchArgument(
            'vocabulary_path',
            default_value='/home/maikel/coding/MITSPARK/Kimera-VIO/vocabulary/ORBvoc.yml'),
        DeclareLaunchArgument('left_cam_topic',  default_value='/cam0/image_raw'),
        DeclareLaunchArgument('right_cam_topic', default_value='/cam1/image_raw'),
        DeclareLaunchArgument('depth_cam_topic',
                              default_value='/depth/image_raw',
                              description='RGBD only; ignored for stereo.'),
        DeclareLaunchArgument('imu_topic',       default_value='/imu0'),
        DeclareLaunchArgument('base_link_frame_id', default_value='base_link'),
        DeclareLaunchArgument('odom_frame_id',      default_value='odom'),
        DeclareLaunchArgument('map_frame_id',       default_value='map'),
        DeclareLaunchArgument('left_cam_frame_id',  default_value='cam0'),
        DeclareLaunchArgument('right_cam_frame_id', default_value='cam1'),
        DeclareLaunchArgument('use_sim_time',  default_value='true'),
        DeclareLaunchArgument('use_rviz',      default_value='true'),
        DeclareLaunchArgument('log_gt_data',   default_value='false'),
        DeclareLaunchArgument('use_external_odom', default_value='false'),
        DeclareLaunchArgument('force_same_image_timestamp', default_value='true'),
        DeclareLaunchArgument(
            'viz_type', default_value='0',
            description='0=Mesh2dTo3dSparse (mesh on /mesh), '
                        '1=Pointcloud (no mesh), 2=None.'),
        DeclareLaunchArgument(
            'use_lcd', default_value='false',
            description='Enable Loop-Closure Detection (Slice C).'),
        DeclareLaunchArgument(
            'use_lcd_registration_server', default_value='false',
            description='Advertise ~/register_lcd_frames service '
                        '(requires use_lcd=true).'),
        DeclareLaunchArgument(
            'visualize', default_value='false',
            description='Spawn mesh_to_marker.py to publish '
                        '/kimera_vio_ros/mesh_marker for RViz.'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_build_node)])
