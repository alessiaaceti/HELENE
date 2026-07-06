#!/usr/bin/env python3
import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

class SpaceNavBridge(Node):
    def __init__(self):
        super().__init__('spacenav_to_servo_bridge')
        
        # Subscribe to SpaceMouse buttons
        self.joy_sub = self.create_subscription(Joy, '/spacenav/joy', self.joy_callback, 10)

        # Subscribe to SpaceMouse movements
        self.subscription = self.create_subscription(Twist, '/spacenav/twist', self.listener_callback, 10)
            
        # Publisher to MoveIt Servo
        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
            
        self.deadman_pressed = False
        self.was_pressed = False 
        self.zero_sent = False 
        
        self.get_logger().info('Bridge Started! Run with telemetry check.')

        # Ascolta i calcoli di MoveIt Servo
        self.traj_sub = self.create_subscription(
            JointTrajectory, 
            '/helene_trajectory_controller/joint_trajectory', 
            self.trajectory_callback, 
            10
        )

        # Parla direttamente con l'ESP32 nel formato che preferisce
        self.joint_cmd_pub = self.create_publisher(
            JointState, 
            '/helene_trajectory_controller/joint_commands', 
            10
        )

    def joy_callback(self, msg):
        if msg.buttons and msg.buttons[0] == 1:
            self.deadman_pressed = True
        else:
            self.deadman_pressed = False

    def listener_callback(self, msg):
        if not self.deadman_pressed:
            if self.was_pressed:
                self.send_zero_command()
                self.was_pressed = False
                self.zero_sent = True
            return

        self.was_pressed = True

        # Deadzone filter
        deadzone = 0.05
        is_still = (abs(msg.linear.x) < deadzone and 
                    abs(msg.linear.y) < deadzone and 
                    abs(msg.linear.z) < deadzone and
                    abs(msg.angular.x) < deadzone and 
                    abs(msg.angular.y) < deadzone and 
                    abs(msg.angular.z) < deadzone)

        if is_still:
            if not self.zero_sent:
                self.send_zero_command()
                self.zero_sent = True 
            return

        self.zero_sent = False

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        # Apply deadzone filtering
        msg.linear.x = 0.0 if abs(msg.linear.x) < deadzone else msg.linear.x
        msg.linear.y = 0.0 if abs(msg.linear.y) < deadzone else msg.linear.y
        msg.linear.z = 0.0 if abs(msg.linear.z) < deadzone else msg.linear.z
        #msg.angular.x = 0.0 if abs(msg.angular.x) < deadzone else msg.angular.x
        #msg.angular.y = 0.0 if abs(msg.angular.y) < deadzone else msg.angular.y
        #msg.angular.z = 0.0 if abs(msg.angular.z) < deadzone else msg.angular.z
        
        # TELEMETRY LOG: Print outgoing values to verify they are between -1.0 and 1.0
        self.get_logger().info(f'Publishing Twist -> Lin X: {msg.linear.x:.3f}, Lin Z: {msg.linear.z:.3f}')

        # Pack and publish
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link' 
        stamped_msg.twist = msg
        self.publisher.publish(stamped_msg)

    def send_zero_command(self):
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'
        stamped_msg.twist = Twist() 
        self.publisher.publish(stamped_msg)

    def trajectory_callback(self, msg):
        # Se il messaggio è vuoto, ignora
        if len(msg.points) == 0:
            return

        # Impacchetta i dati per l'ESP32
        js_msg = JointState()
        js_msg.header = msg.header
        js_msg.name = msg.joint_names
        js_msg.velocity = msg.points[0].velocities # Estrae le velocità pure calcolate da MoveIt

        # Spara i dati verso Micro-ROS!
        self.joint_cmd_pub.publish(js_msg)

def main(args=None):
    rclpy.init(args=sys.argv if args is None else args)
    bridge = SpaceNavBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()