import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

# Utility function to load YAML files as dictionaries for MoveIt 2 parameters
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Paths to required packages
    pkg_hw_description = get_package_share_directory('hw_description')
    pkg_controller_helene = get_package_share_directory('controller_helene')
    pkg_helene_moveit_config = get_package_share_directory('helene_moveit_config')
    
    xacro_file = os.path.join(pkg_hw_description, 'urdf', 'helene_hw.urdf.xacro')
    
    # Load the SRDF Semantic file for MoveIt 2 from the correct package
    srdf_file = os.path.join(pkg_helene_moveit_config, 'config', 'helene.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_config = f.read()
    
    # Simulation time configuration for all nodes
    sim_time_param = {'use_sim_time': True}
    
    # Use mock hardware interface for Gazebo compatibility
    use_mock_hardware = 'true'  # Set to 'true' if using mock hardware interface

    # Process Xacro dynamically to generate robot description
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_mock_hardware:=', use_mock_hardware,
                 ' use_ft_sensor:=true',
                 ' sensor_stiffness:=', LaunchConfiguration('sensor_stiffness')]),
        value_type=str
    )

    # MoveIt and Servo plan on the rigid arm (q7 fixed): the sensor spring is only a
    # simulation detail of Gazebo and robot_state_publisher, not a joint to control.
    robot_description_rigid = ParameterValue(
        Command(['xacro ', xacro_file, ' use_mock_hardware:=', use_mock_hardware,
                 ' use_ft_sensor:=true']),
        value_type=str
    )

    # 1. Ignition Gazebo Simulation Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': [LaunchConfiguration('gz_flags'), ' ', LaunchConfiguration('world')]}.items(),
    )

    # 2. Spawn Robot Entity in Ignition Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'helene'],
        output='screen'
    )

    # 3. ROS 2 - Ignition Gazebo Parameter Bridge (Force-Torque Sensor & Clock)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/force_torque_sensor@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        output='screen'
    )

    # 4. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}, sim_time_param]
    )

    # --- MoveIt 2 Configuration (Formato Isolato e Sicuro per Humble) ---
    kinematics_yaml = load_yaml('helene_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('helene_moveit_config', 'config/joint_limits.yaml')
    ompl_yaml = load_yaml('helene_moveit_config', 'config/ompl_planning.yaml')
    servo_yaml = load_yaml('controller_helene', 'config/servo_params.yaml')

    servo_params_flat = {}
    if servo_yaml:
        if 'servo_node' in servo_yaml and 'ros__parameters' in servo_yaml['servo_node']:
            servo_params_flat = servo_yaml['servo_node']['ros__parameters']
        elif 'moveit_servo' in servo_yaml and 'ros__parameters' in servo_yaml['moveit_servo']:
            servo_params_flat = servo_yaml['moveit_servo']['ros__parameters']
        elif 'ros__parameters' in servo_yaml:
            servo_params_flat = servo_yaml['ros__parameters']
        else:
            servo_params_flat = servo_yaml

    servo_params_final = {}
    if isinstance(servo_params_flat, dict):
        for k, v in servo_params_flat.items():
            servo_params_final[k] = v
            servo_params_final[f'moveit_servo.{k}'] = v

    servo_forced_params = {
        'move_group_name': 'helene_arm',
        'planning_frame': 'base_link',
        'ee_frame_name': 'axis_6',
        'robot_link_command_frame': 'base_link',
        'check_collisions': False,
        
        'moveit_servo.move_group_name': 'helene_arm',
        'moveit_servo.planning_frame': 'base_link',
        'moveit_servo.ee_frame_name': 'axis_6',
        'moveit_servo.robot_link_command_frame': 'base_link',
        'moveit_servo.check_collisions': False,
    }
    servo_params_final.update(servo_forced_params)

    # Struttura esplicita richiesta per evitare i Segmentation Fault
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml if kinematics_yaml else {}}

    planning_pipelines_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
            # RRTConnect specific parameters
            'planner_configs': ['RRTConnectkConfigDefault'],
            'RRTConnectkConfigDefault': {
                'type': 'geometric::RRTConnect',
                'range': 0.0,  # Default range, can be tuned for performance
            }
        }
    }

    moveit_controllers_config = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager.controller_names': ['helene_trajectory_controller'],
        'moveit_simple_controller_manager.helene_trajectory_controller.type': 'FollowJointTrajectory',
        'moveit_simple_controller_manager.helene_trajectory_controller.action_ns': 'follow_joint_trajectory',
        'moveit_simple_controller_manager.helene_trajectory_controller.default': True,
        'moveit_simple_controller_manager.helene_trajectory_controller.joints': ['q1', 'q2', 'q3', 'q4', 'q5', 'q6'],
    }

    # 5. MoveGroup Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description_rigid},
            {'robot_description_semantic': robot_description_semantic_config},
            robot_description_kinematics,
            joint_limits_yaml if joint_limits_yaml else {},
            ompl_yaml if ompl_yaml else {},
            planning_pipelines_config,
            moveit_controllers_config,
            sim_time_param
        ],
    )

    # 6. RViz2 Node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {'robot_description': robot_description_rigid},
            {'robot_description_semantic': robot_description_semantic_config},
            robot_description_kinematics,
            joint_limits_yaml if joint_limits_yaml else {},
            ompl_yaml if ompl_yaml else {},
            planning_pipelines_config,
            moveit_controllers_config,
            sim_time_param
        ],
        arguments=['-d', os.path.join(pkg_helene_moveit_config, 'config', 'moveit.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # 7. Joint State Broadcaster Spawner
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[sim_time_param]
    )

    # 8. Post-Broadcaster Event Handler
    # servo_mode:=true loads the velocity controller that MoveIt Servo writes to
    # (/helene_velocity_controller/commands) instead of the trajectory controller:
    # needed for hand guiding and screw insertion. Both claim the same joint velocity
    # interfaces, so only one of them is started.
    trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["helene_trajectory_controller"],
        parameters=[sim_time_param],
        condition=UnlessCondition(LaunchConfiguration('servo_mode'))
    )
    velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["helene_velocity_controller"],
        parameters=[sim_time_param],
        condition=IfCondition(LaunchConfiguration('servo_mode'))
    )
    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[trajectory_controller, velocity_controller],
        )
    )

    # 9. MoveIt Servo Main Node - Configuration with explicit parameters to avoid Segmentation Faults and ensure proper initialization order
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description_rigid},
            {'robot_description_semantic': robot_description_semantic_config},
            robot_description_kinematics,
            joint_limits_yaml if joint_limits_yaml else {},
            servo_params_final, # Il super dizionario protetto
            sim_time_param
        ],
    )

    # 10. Admittance Teleoperation Translator (Force Sensor -> Cartesian Twist)
    helene_force_servo_teleop = Node(
        package='controller_helene',
        executable='helene_force_servo_teleop',
        output='screen',
        parameters=[sim_time_param],
        condition=IfCondition(LaunchConfiguration('teleop'))
    )

    # MoveIt Servo (Humble) ignores commands until it is started
    start_servo = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger', '{}'],
        output='screen'
    )

    # 11. Force-guided screw insertion (approach, spiral search, insert), off by default
    screw_insertion = Node(
        package='controller_helene',
        executable='screw_insertion_node',
        output='screen',
        parameters=[sim_time_param, {'auto_start': False,
                    'direct_joint_control': True,
                    'robot_description': robot_description_rigid,
                    # the Gazebo F/T sensor reports the force of the tool on the arm
                    # (child_to_parent) in the sensor frame: no sign flip
                    'sensor_sign_x': 1.0, 'sensor_sign_y': 1.0, 'sensor_sign_z': 1.0,
                    'tool_tip_offset_z': 0.06,
                    'expected_surface_z': ParameterValue(LaunchConfiguration('surface_z'), value_type=float),
                    'sensor_delay': ParameterValue(LaunchConfiguration('sensor_delay'), value_type=float),
                    }],
        condition=IfCondition(LaunchConfiguration('insertion'))
    )

    return LaunchDescription([
        # rviz:=false and gz_flags:='-r -s' run everything headless (CI / no display)
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gz_flags', default_value='-r'),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg_controller_helene, 'worlds', 'helene_world.sdf')),
        # force sensor stiffness [N/m]; 0 = rigid. Without compliance the simulated contact
        # is infinitely stiff: use ~20000 (20 N/mm) for force-control tests
        DeclareLaunchArgument('sensor_stiffness', default_value='0'),
        # known plate height for the insertion node ('nan' = unknown: spiral search from contact)
        DeclareLaunchArgument('surface_z', default_value='0.05'),
        # extra delay [s] on the force signal, to test the loop with the latency of the real chain
        DeclareLaunchArgument('sensor_delay', default_value='0.0'),
        DeclareLaunchArgument('servo_mode', default_value='false'),
        DeclareLaunchArgument('teleop', default_value='true'),
        DeclareLaunchArgument('insertion', default_value='false'),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        move_group_node,
        rviz2_node,
        
        # Start core broadcaster with a small delay for Gazebo framework readiness
        TimerAction(
            period=3.0,
            actions=[joint_state_broadcaster]
        ),
        
        # Event triggers controller spawning automatically
        load_controllers,
        
        # Safety delay to allow hardware controllers to fully activate 
        # before starting MoveIt Servo and the Teleop Translator node
        TimerAction(
            period=8.0, 
            actions=[servo_node, helene_force_servo_teleop]
        ),
        # Servo needs a moment to load the robot model before it accepts start_servo
        TimerAction(period=24.0, actions=[start_servo]),
        # The insertion node tares at start: it waits for ~/start, which
        # sim_go_to_start.py --start calls once the arm is at rest above the plate
        TimerAction(period=25.0, actions=[screw_insertion])
    ])