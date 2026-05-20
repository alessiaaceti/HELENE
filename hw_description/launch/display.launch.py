import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('hw_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'helene_hw.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')
    package_dir = get_package_share_directory('controller_helene')

    return LaunchDescription([
        # Node 1: Publish robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),
        # Node 2: GUI for joinsts removal
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        # Node 3: RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
        # Node 4. spawn trajectory controller for MoveIt 2
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["helene_trajectory_controller"],
        ),
    ])
