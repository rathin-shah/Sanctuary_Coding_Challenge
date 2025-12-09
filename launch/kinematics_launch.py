from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('planar_kinematics')
    config_file = os.path.join(pkg_share, 'config', 'fk_params.yaml')

    declare_config_arg = DeclareLaunchArgument(
        'config',
        default_value=config_file,
        description='Path to FK config YAML'
    )
    # ----------------------------------------------------------

    fk_node = Node(
        package='planar_kinematics',
        executable='fk_node',
        name='fk_node',
        parameters=[LaunchConfiguration('config')]
    )

    ik_node = Node(
        package='planar_kinematics',
        executable='ik_node',
        name='ik_node'
    )

    return LaunchDescription([
        declare_config_arg,
        fk_node,
        ik_node
    ])
