#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistBridge(Node):
    def __init__(self):
        super().__init__('keyboard_twist_bridge')
        
        # subscriber for the Twist message
        self.sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.callback, 
            10)
            
        # publisher for the stamped message
        self.pub = self.create_publisher(
            TwistStamped, 
            '/servo_node/delta_twist_cmds', 
            10)

    def callback(self, msg):
        stamped_msg = TwistStamped()
        
        # add timestamp and frame_id
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link' 
        
        # copy the twist data
        stamped_msg.twist = msg
        
        self.pub.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()