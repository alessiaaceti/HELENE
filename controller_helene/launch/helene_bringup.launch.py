from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # path to files
    pkg_share = get_package_share_directory('hw_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'helene_hw.urdf.xacro')

    # process the xacro file to get the robot description in XML format
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_content = robot_description_config.toxml()

    controller_config = os.path.join(
        get_package_share_directory('controller_helene'), 'config', 'helene_controllers.yaml'
    )

    return LaunchDescription([
        # 0. micro-ROS Agent (Endpoint ESP32)
        # NOTE: Substitute '--dev', '/dev/ttyUSB0' with 'udp4', '--port', '8888' if using Wi-Fi instead of USB
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', '/dev/ttyUSB0']
        ),

        # 1. publish robot model
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        # 2. main controller manager (runs C++ code)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description_content}, controller_config],
            output='screen',
        ),
        # 3. spawn broadcaster (to see the robot moving in rviz)
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        # 4. spawn custom velocity controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["helene_velocity_controller"],
        ),
        # 5. spawn custom trajectory controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["helene_trajectory_controller"],
        ),
        # 6. spawn force torque sensor broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["helene_force_broadcaster"],
        ),
    ])