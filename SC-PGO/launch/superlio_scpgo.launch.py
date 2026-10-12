from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from pathlib import Path
import os
import tempfile


def prepare_output(context):
    temporary = Path('/mnt/e/code/codex/point-lio-sc/tmp')
    temporary.mkdir(parents=True, exist_ok=True)
    os.environ['TMPDIR'] = str(temporary)
    tempfile.tempdir = str(temporary)
    directory = LaunchConfiguration('save_directory').perform(context)
    if not directory.endswith('/'):
        raise RuntimeError('save_directory must end with /')
    Path(directory).mkdir(parents=True, exist_ok=True)
    return []


def generate_launch_description():
    package = get_package_share_directory('aloam_velodyne')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('save_directory',
                             default_value='/mnt/e/code/codex/point-lio-sc/superlio_pgo_data/'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution(
            [package, 'config', 'sc_pgo.yaml'])),
        OpaqueFunction(function=prepare_output),
        Node(package='aloam_velodyne', executable='superlio_scpgo_bridge.py',
             name='superlio_scpgo_bridge', output='screen'),
        Node(package='aloam_velodyne', executable='alaserPGO',
             name='laserPGO', output='screen',
             parameters=[LaunchConfiguration('config'), {
                 'save_directory': LaunchConfiguration('save_directory'),
                 'sub_lidar_topic': '/superlio/scpgo/cloud_body',
                 'sub_odom_topic': '/superlio/scpgo/odom',
                 'use_gps': False,
                 'num_cores_transform': 1, 'num_cores_icp': 1,
                 'viz_path_frequency': 5.0,
             }]),
        # The two names describe the same initial coordinate system; this also
        # lets the PGO RViz show Super-LIO's original world-frame path.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='superlio_pgo_world_alias',
             arguments=['--frame-id', 'camera_init', '--child-frame-id', 'world'],
             output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz_superlio_scpgo',
             arguments=['-d', PathJoinSubstitution(
                 [package, 'rviz_cfg', 'superlio_scpgo.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
