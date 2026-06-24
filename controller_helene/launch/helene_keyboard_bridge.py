#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class KeyboardToServoBridge(Node):
    def __init__(self):
        super().__init__('keyboard_to_servo_bridge')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.get_logger().info("Bridge Tastiera -> MoveIt Servo Avviato!")

    def callback(self, msg):
        stamped_twist = TwistStamped()
        stamped_twist.header.stamp = self.get_clock().now().to_msg()
        stamped_twist.header.frame_id = "base_link"
        stamped_twist.twist = msg
        self.pub.publish(stamped_twist)

def main():
    rclpy.init()
    rclpy.spin(KeyboardToServoBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
