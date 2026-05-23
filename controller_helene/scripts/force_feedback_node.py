#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray

class ForceFeedbackNode(Node):
    def __init__(self):
        super().__init__('force_feedback_node')

        # 1. Subscribe to the sensor (data from your colleague)
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/helene_force_broadcaster/wrench', # Topic name from controllers.yaml
            self.force_callback,
            10)

        # 2. Publisher for the motors (velocity commands)
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/helene_velocity_controller/commands',
            10)

        # Gain parameter (admittance)
        # The higher it is, the more sensitive and faster the robot is
        self.gain = 0.005 
        
        # Deadzone threshold (to prevent the robot from moving on its own due to noise)
        self.deadzone = 2.0 # Newton

        self.get_logger().info('Force Feedback node started. Waiting for sensor data...')

    def force_callback(self, msg):
        # Retrieve X, Y, Z forces from the message
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z

        # Initialize velocities for the 6 joints to zero
        velocities = Float64MultiArray()
        v_cmds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # SIMPLIFIED CONTROL LOGIC
        # Here we map force to specific joints.
        # Example: push on X moves joint q1, push on Z moves joint q2
        
        if abs(fx) > self.deadzone:
            v_cmds[0] = fx * self.gain  # Moves joint q1
            
        if abs(fy) > self.deadzone:
            v_cmds[1] = fy * self.gain  # Moves joint q2

        if abs(fz) > self.deadzone:
            v_cmds[2] = fz * self.gain  # Moves joint q3

        # Publish the velocity array
        velocities.data = v_cmds
        self.publisher.publish(velocities)

def main(args=None):
    rclpy.init(args=args)
    node = ForceFeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()