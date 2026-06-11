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
    hole_board_model_path = os.path.join(pkg_hw_description, 'urdf', 'hole_board.urdf')

    xacro_file = os.path.join(pkg_hw_description, 'urdf', 'helene_hw.urdf.xacro')
    world_file = os.path.join(get_package_share_directory('controller_helene'), 'worlds', 'helene_world.sdf')
    use_mock_hardware = 'true' # Set to 'true' to use the mock hardware interface, which is compatible with Gazebo. Change to 'false' if you want to use the real hardware interface (not recommended for simulation).

    # Correct ROS 2 method: use Command to process xacro dynamically
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_mock_hardware:=', use_mock_hardware]),
        value_type=str
    )

    # Ignition Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(), # Pass the world file as an argument to Gazebo
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
    # Controllers start ONLY AFTER joint_state_broadcaster is successfully loaded
    load_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=["helene_trajectory_controller"], # <-- Switched to the trajectory controller
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

    # spawn hole
    #spawn_hole_board = Node(
    #    package='ros_gz_sim',
    #    executable='create',
    #    arguments=[
    #        '-name', 'hole_board',
    #        '-file', hole_board_model_path,
    #        '-x', '0.4',
    #        '-y', '0.0',
    #        '-z', '0.1',
    #    ],
    #    output='screen',
    #)

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        #spawn_hole_board,
        # Start with a small delay to give Gazebo time to load the ros2_control plugin
        TimerAction(
            period=3.0,
            actions=[joint_state_broadcaster]
        ),
        
        # This will automatically trigger when joint_state_broadcaster exits its spawning phase
        load_trajectory_controller,
        
        # Ensure force_feedback starts after all controllers are up and running
        TimerAction(
            period=5.0, 
            actions=[force_feedback_node]
        )
    ])