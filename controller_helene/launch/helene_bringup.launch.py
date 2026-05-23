from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # path to files
    pkg_hw_description = get_package_share_directory('hw_description')
    pkg_controller_helene = get_package_share_directory('controller_helene')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_hw_description, 'urdf', 'helene_hw.urdf.xacro')

    use_mock_hardware = 'true'  # Set to 'false' to use real hardware instead of Gazebo mock hardware

    # process the xacro file to get the robot description in XML format
    robot_description_config = xacro.process_file(
        xacro_file, 
        mappings={'use_mock_hardware': use_mock_hardware}
    )
    robot_description_content = robot_description_config.toxml()

    controller_config = os.path.join(
        get_package_share_directory('controller_helene'), 'config', 'helene_controllers.yaml'
    )

    # Include Gazebo launch file to start the simulation environment
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')]),
    )

    # Node to spawn the robot in Gazebo using the robot description from the xacro file
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'helene'],
        output='screen'
    )

    force_feedback_node = Node(
        package='controller_helene',
        executable='force_feedback_node.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,         # start Gazebo simulation
        spawn_entity,   # spawn the robot in Gazebo using the robot description from the xacro file

        # 0. micro-ROS Agent (Endpoint ESP32)
        # NOTE: Substitute '--dev', '/dev/ttyUSB0' with 'udp4', '--port', '8888' if using Wi-Fi instead of USB
        #Node(
            #package='micro_ros_agent',
            #executable='micro_ros_agent',
            #name='micro_ros_agent',
            #output='screen',
            #arguments=['serial', '--dev', '/dev/ttyUSB0']
        #),

        # 1. publish robot model
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}, {'use_sim_time': True}]
        ),
        # 2. main controller manager (runs C++ code)
        #Node(
        #    package='controller_manager',
        #    executable='ros2_control_node',
        #    parameters=[{'robot_description': robot_description_content}, controller_config],
        #    output='screen',
        #),
        # 3. spawn broadcaster (to see the robot moving in rviz)
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            parameters=[{'use_sim_time': True}]
        ),
        # 4. spawn custom velocity controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["helene_velocity_controller"],
            parameters=[{'use_sim_time': True}]
        ),
        # 5. spawn custom trajectory controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["helene_trajectory_controller", "--inactive"],
            parameters=[{'use_sim_time': True}]
        ),
        # 6. spawn force torque sensor broadcaster
        #Node(
        #   package="controller_manager",
        #    executable="spawner",
        #    arguments=["helene_force_broadcaster"],
        #    parameters=[{'use_sim_time': True}]
        #),
        # 7. spawn force feedback node
        force_feedback_node,
    ])