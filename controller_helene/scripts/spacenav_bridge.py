#!/usr/bin/env python3
import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class SpaceNavBridge(Node):
    def __init__(self):
        super().__init__('spacenav_to_servo_bridge')
        
        # Subscribe to the SpaceMouse input topic
        self.subscription = self.create_subscription(
            Twist,
            '/spacenav/twist',
            self.listener_callback,
            10)
            
        # Publisher for the topic required by MoveIt Servo
        self.publisher = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10)
            
        self.get_logger().info('SpaceMouse <-> MoveIt Servo Bridge Started!')

    def listener_callback(self, msg):
        stamped_msg = TwistStamped()
        
        # Add the timestamp required by MoveIt
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Define the reference frame (use 'base_link' or 'axis_6' if you want to move relative to the end-effector)
        stamped_msg.header.frame_id = 'base_link' 
        
        # Copy the Cartesian movement data
        stamped_msg.twist = msg
        
        self.publisher.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=sys.argv if args is None else args)
    bridge = SpaceNavBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()