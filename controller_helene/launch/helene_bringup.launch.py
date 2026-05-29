from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to required packages
    pkg_hw_description = get_package_share_directory('hw_description')
    pkg_controller_helene = get_package_share_directory('controller_helene')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_hw_description, 'urdf', 'helene_hw.urdf.xacro')
    use_mock_hardware = 'true'

    # Correct ROS 2 method: use Command to process xacro dynamically
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_mock_hardware:=', use_mock_hardware]),
        value_type=str
    )

    # Ignition Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Spawn robot in Ignition
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'helene'],
        output='screen'
    )

    # Bridge for Force-Torque sensor (CRITICAL)
    # This translates Ignition messages to ROS 2 messages
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/force_torque_sensor@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench'],
        output='screen'
    )

    # Publish robot state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Spawn joint state broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': True}]
    )

    # Controllers start ONLY AFTER joint_state_broadcaster is successfully loaded
    # This prevents race conditions during startup
    load_velocity_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=["helene_velocity_controller"],
                parameters=[{'use_sim_time': True}]
            )],
        )
    )

    # Custom force feedback node
    force_feedback_node = Node(
        package='controller_helene',
        executable='force_feedback_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        # Start with a small delay to give Gazebo time to load the ros2_control plugin
        TimerAction(
            period=3.0,
            actions=[joint_state_broadcaster]
        ),
        
        # This will automatically trigger when joint_state_broadcaster exits its spawning phase
        load_velocity_controller,
        
        # Ensure force_feedback starts after all controllers are up and running
        TimerAction(
            period=5.0, 
            actions=[force_feedback_node]
        )
    ])