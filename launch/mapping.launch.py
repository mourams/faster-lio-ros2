import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('faster_lio')
    default_config_path = os.path.join(package_path, 'config', 'avia.yaml')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'faster_lio.rviz')

    config_path = LaunchConfiguration('config_path')
    use_rviz = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    faster_lio_node = Node(
        package='faster_lio',
        executable='run_mapping_online',
        name='faster_lio',
        parameters=[config_path],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(use_rviz)
    )

    ld = LaunchDescription()
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(faster_lio_node)
    ld.add_action(rviz_node)

    return ld