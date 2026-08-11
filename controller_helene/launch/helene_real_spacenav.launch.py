import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, ExecuteProcess
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
    use_mock_hardware = 'false'

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
            'RRTConnectkConfigDefault': {'type': 'geometric::RRTConnect', 'range': 0.0}
        }
    }

    moveit_controllers_config = {
        'moveit_manage_controllers': True,
        'moveit_controller_manager': 'moveit_ros_control_interface/MoveItControllerManager',
        'moveit_ros_control_interface': {
            'ros_control_namespace': '/',
            'ros_control_node_name': 'controller_manager',
            'controller_names': ['helene_trajectory_controller', 'helene_velocity_controller'],
            'helene_trajectory_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': ['q1', 'q2', 'q3', 'q4', 'q5', 'q6'],
            },
            'helene_velocity_controller': {
                'type': 'JointGroupVelocityController',
                'default': False,
                'joints': ['q1', 'q2', 'q3', 'q4', 'q5', 'q6'],
            }
        }
    }

    planning_scene_monitor_config = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
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
            planning_scene_monitor_config,
            sim_time_param,
            # Disable strict execution time monitoring to prevent TIMED_OUT errors
            {'trajectory_execution.execution_duration_monitoring': False}
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

    # 6. Event Handler: Load controllers after Joint State Broadcaster completes
    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["helene_velocity_controller"],
                    parameters=[sim_time_param]
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["helene_trajectory_controller", "--inactive"],
                    parameters=[sim_time_param]
                )
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

    # 8. Micro-ROS Agent Node (ESP32 serial communication)
    microros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/helene_esp', '-b', '460800']
    )

    # 9. SpaceNav Driver Node
    spacenav_driver_node = Node(
        package='spacenav',
        executable='spacenav_node',
        name='spacenav_node',
        output='screen',
        parameters=[sim_time_param, {'zero_when_static': True}]
    )

    # 10. Transformer with Smart Automatic Controller Switching
    transformer_script = """
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from controller_manager_msgs.srv import SwitchController, ListControllers

class SpaceNavTransformer(Node):
    def __init__(self):
        super().__init__('spacenav_transformer')
        self.pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.sub = self.create_subscription(Twist, '/spacenav/twist', self.cb, 10)

        # Clients for automatic ros2_control manager switching
        self.cli_list = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.cli_switch = self.create_client(SwitchController, '/controller_manager/switch_controller')

        self.is_switching = False

        # --- AXIS MAPPING & INVERSION ---
        self.scale_x = -1.0  # Inverted: Push forward -> Move forward (+X)
        self.scale_y = -1.0  # Inverted: Push right -> Move right (-Y)
        self.scale_z = 1.0   # Pull up -> Move up (+Z)

        self.scale_rx = 1.0
        self.scale_ry = -1.0
        self.scale_rz = 1.0

    def check_and_switch_controller(self):
        if self.is_switching or not self.cli_list.service_is_ready():
            return

        req = ListControllers.Request()
        future = self.cli_list.call_async(req)
        future.add_done_callback(self._on_list_controllers_done)

    def _on_list_controllers_done(self, future):
        try:
            res = future.result()
            vel_active = False
            traj_active = False

            for ctrl in res.controller:
                if ctrl.name == 'helene_velocity_controller' and ctrl.state == 'active':
                    vel_active = True
                if ctrl.name == 'helene_trajectory_controller' and ctrl.state == 'active':
                    traj_active = True

            # If velocity controller is not active, activate it on the fly
            if not vel_active and self.cli_switch.service_is_ready():
                self.is_switching = True
                self.get_logger().info('SpaceMouse movement detected: Switching to helene_velocity_controller...')
                
                switch_req = SwitchController.Request()
                switch_req.activate_controllers = ['helene_velocity_controller']
                if traj_active:
                    switch_req.deactivate_controllers = ['helene_trajectory_controller']
                switch_req.strictness = SwitchController.Request.BEST_EFFORT

                switch_future = self.cli_switch.call_async(switch_req)
                switch_future.add_done_callback(lambda f: setattr(self, 'is_switching', False))
        except Exception as e:
            self.get_logger().error(f"Error during controller switch: {e}")
            self.is_switching = False

    def cb(self, msg):
        # Check if user is physically touching/moving SpaceMouse knob
        moving = any([
            abs(msg.linear.x) > 0.02, abs(msg.linear.y) > 0.02, abs(msg.linear.z) > 0.02,
            abs(msg.angular.x) > 0.02, abs(msg.angular.y) > 0.02, abs(msg.angular.z) > 0.02
        ])

        if moving:
            self.check_and_switch_controller()

        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'

        ts.twist.linear.x = msg.linear.x * self.scale_x
        ts.twist.linear.y = msg.linear.y * self.scale_y
        ts.twist.linear.z = msg.linear.z * self.scale_z

        ts.twist.angular.x = msg.angular.x * self.scale_rx
        ts.twist.angular.y = msg.angular.y * self.scale_ry
        ts.twist.angular.z = msg.angular.z * self.scale_rz

        self.pub.publish(ts)

def main():
    rclpy.init()
    node = SpaceNavTransformer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
"""

    python_transformer = ExecuteProcess(
        cmd=[sys.executable, '-c', transformer_script],
        output='screen'
    )

    # 11. ESP32 Command Bridge: Converts Float64MultiArray -> JointState for ESP32
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

    # 12. Activate MoveIt Servo service
    start_servo_service = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger', '{}'],
        output='screen'
    )

    return LaunchDescription([
        # Immediate launch of hardware drivers and bridges
        microros_agent_node,
        spacenav_driver_node,
        python_transformer,
        esp32_bridge,

        # Launch core ROS 2 and MoveIt nodes after brief delay
        TimerAction(
            period=2.0,
            actions=[
                ros2_control_node,
                robot_state_publisher,
                move_group_node,
                rviz2_node,
            ]
        ),

        # Launch Joint State Broadcaster first, triggering controller loading on exit
        TimerAction(
            period=3.0,
            actions=[joint_state_broadcaster]
        ),
        load_controllers,

        # Launch MoveIt Servo node once controllers are ready
        TimerAction(
            period=8.0,
            actions=[servo_node]
        ),

        # Final trigger to activate Servo mode
        TimerAction(
            period=10.0,
            actions=[start_servo_service]
        )
    ])