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

    # Common parameters for simulation stability
    sim_time_param = {'use_sim_time': True}

    return LaunchDescription([
        # Node 1: Publish robot state (with simulation time enabled)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description_config},
                sim_time_param
            ]
        ),
        
        # REMOVED: joint_state_publisher_gui 
        # (It conflicts with helene_trajectory_controller on the /joint_states topic)

        # Node 2: RViz2 (with simulation time enabled)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[sim_time_param]
        ),
        
        # Node 3: Spawn trajectory controller for MoveIt 2
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["helene_trajectory_controller"],
            parameters=[sim_time_param]
        ),
    ])