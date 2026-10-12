from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package = get_package_share_directory('super_lio')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution(
            [package, 'config', 'unitree_l1.yaml'])),
        Node(package='super_lio', executable='super_lio_node',
             name='super_lio', output='screen',
             parameters=[LaunchConfiguration('config')]),
        Node(package='rviz2', executable='rviz2', name='rviz_super_lio',
             arguments=['-d', PathJoinSubstitution([package, 'rviz', 'lio.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
