import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    pkg_hw_description = get_package_share_directory('hw_description')
    pkg_controller_helene = get_package_share_directory('controller_helene')
    pkg_helene_moveit_config = get_package_share_directory('helene_moveit_config')
    
    xacro_file = os.path.join(pkg_hw_description, 'urdf', 'helene_hw.urdf.xacro')
    srdf_file = os.path.join(pkg_helene_moveit_config, 'config', 'helene.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_config = f.read()
    
    # Real robot parameters
    sim_time_param = {'use_sim_time': False}
    use_mock_hardware = 'false' # Activate real hardware mode in xacro with this parameter

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_mock_hardware:=', use_mock_hardware]),
        value_type=str
    )

    # 1. Controller Manager Hardware
    controller_manager_yaml = os.path.join(pkg_controller_helene, 'config', 'helene_controllers.yaml')
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, controller_manager_yaml],
        output="screen",
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}, sim_time_param]
    )

    # --- MoveIt 2 Configuration ---
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
            'planner_configs': ['RRTConnectkConfigDefault'],
            'RRTConnectkConfigDefault': { 'type': 'geometric::RRTConnect', 'range': 0.0 }
        }
    }

    moveit_controllers_config = {
        'moveit_manage_controllers': True,

        # Diciamo a MoveIt di usare il controller manager intelligente che parla con ros2_control
        'moveit_controller_manager': 'moveit_ros_control_interface/MoveItControllerManager',
        
        # Mappiamo i controller che ros2_control ha effettivamente a disposizione
        'moveit_ros_control_interface': {
            'ros_control_namespace': '/',
            'ros_control_node_name': 'controller_manager',
            'controller_names': ['helene_trajectory_controller', 'helene_velocity_controller'],
            
            # Configuriamo il controller di traiettoria
            'helene_trajectory_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': ['q1', 'q2', 'q3', 'q4', 'q5', 'q6'],
            },
            
            # Diciamo a MoveIt che esiste anche il controller di velocità
            'helene_velocity_controller': {
                'type': 'JointGroupVelocityController',
                'default': False,
                'joints': ['q1', 'q2', 'q3', 'q4', 'q5', 'q6'],
            }
        }
    }

    # 3. MoveGroup Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic_config},
            robot_description_kinematics,
            joint_limits_yaml if joint_limits_yaml else {},
            ompl_yaml if ompl_yaml else {},
            planning_pipelines_config,
            moveit_controllers_config,
            sim_time_param
        ],
    )

    # 4. RViz2 Node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic_config},
            robot_description_kinematics,
            joint_limits_yaml if joint_limits_yaml else {},
            ompl_yaml if ompl_yaml else {},
            planning_pipelines_config,
            moveit_controllers_config,
            sim_time_param
        ],
        arguments=['-d', os.path.join(pkg_helene_moveit_config, 'config', 'moveit.rviz')],
    )

    # 5. Joint State Broadcaster Spawner
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[sim_time_param]
    )

    # 6. Event Handler to Load Controllers After Joint State Broadcaster
    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                # 1. Controller per lo SpaceMouse (Velocità)
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["helene_velocity_controller"],
                    parameters=[sim_time_param]
                ),
                # 2. Controller per RViz (Traiettorie) - AGGIUNTO!
                Node(
                    package="controller_manager",
                    executable="spawner",
                    # Nota: mettiamo --inactive per evitare che litighi con il controller di velocità all'avvio. 
                    # MoveIt lo attiverà automaticamente quando premi "Plan and Execute".
                    arguments=["helene_trajectory_controller", "--inactive"],
                    parameters=[sim_time_param]
                ),
                #Node(
                #    package="controller_manager",
                #    executable="spawner",
                #    arguments=["helene_force_broadcaster"],
                #    parameters=[sim_time_param]
                #)
            ],
        )
    )

    # 7. MoveIt Servo Main Node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic_config},
            robot_description_kinematics,
            joint_limits_yaml if joint_limits_yaml else {},
            servo_params_final,
            sim_time_param
        ],
    )

    # 8. Micro-ROS Agent Node
    microros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/helene_esp', '-b', '460800']
    )

    # 9. SpaceNav Driver Node (Ripristinato originale che sputa Twist)
    spacenav_driver_node = Node(
        package='spacenav',
        executable='spacenav_node',
        name='spacenav_node',
        output='screen',
        parameters=[sim_time_param, {'zero_when_static': True}]
    )

    # 9.1 Convertitore Nativo Python (Sintassi lineare compatta)
    from launch.actions import ExecuteProcess
    import sys
    
    python_transformer = ExecuteProcess(
        cmd=[
            sys.executable, '-c',
            'import rclpy; '
            'from rclpy.node import Node; '
            'from geometry_msgs.msg import Twist, TwistStamped; '
            'from std_msgs.msg import Header; '
            'rclpy.init(); '
            'node = Node("inline_transformer"); '
            'pub = node.create_publisher(TwistStamped, "/spacenav/twist_stamped", 10); '
            'sub = node.create_subscription(Twist, "/spacenav/twist", lambda msg: pub.publish(TwistStamped(header=Header(stamp=node.get_clock().now().to_msg(), frame_id="base_link"), twist=msg)), 10); '
            'rclpy.spin(node)'
        ],
        output='screen'
    )

    # 9.2 Convertitore per l'ESP32 (Da MultiArray a JointState)
    esp32_bridge_script = """
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

def main():
    rclpy.init()
    node = Node('esp32_command_bridge')
    pub = node.create_publisher(JointState, '/helene_trajectory_controller/joint_commands', 10)
    
    def cb(msg):
        js = JointState()
        js.header.stamp = node.get_clock().now().to_msg()
        js.name = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6']
        js.velocity = list(msg.data)
        pub.publish(js)
        
    sub = node.create_subscription(Float64MultiArray, '/helene_velocity_controller/commands', cb, 10)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
"""

    esp32_bridge = ExecuteProcess(
        cmd=[sys.executable, '-c', esp32_bridge_script],
        output='screen'
    )
    
    return LaunchDescription([
        #microros_agent_node,
        spacenav_driver_node,
        python_transformer,
        esp32_bridge,
        # Load the main nodes after a delay to ensure the micro-ROS agent is ready
        TimerAction(
            period=2.0,
            actions=[
                ros2_control_node,
                robot_state_publisher,
                move_group_node,
                rviz2_node,
            ]
        ),
        
        # Load Joint State Broadcaster first, then the rest of the controllers after a delay
        TimerAction(
            period=3.0,
            actions=[joint_state_broadcaster]
        ),
        load_controllers,
        
        # Security delay 
        TimerAction(
            period=8.0, 
            actions=[servo_node]
        )
    ])