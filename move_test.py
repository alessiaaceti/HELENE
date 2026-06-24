import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

def main():
    rclpy.init()
    node = rclpy.create_node('magic_mover')
    pub = node.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
    
    msg = TwistStamped()
    msg.header.frame_id = 'base_link'
    
    # Mettiamo a zero tutti i movimenti lineari
    msg.twist.linear.x = 0.0
    msg.twist.linear.y = 0.0
    msg.twist.linear.z = 0.0
    
    # Diamo un comando di ROTAZIONE (gira la pinza/polso)
    msg.twist.angular.z = 0.5  # Rotazione a 0.5 rad/s
    
    print("🚀 Invio comandi perfetti a Servo in corso... (Premi Ctrl+C per fermare)")
    
    def publish_cmd():
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        
    # Invia a 100Hz (0.01 secondi)
    timer = node.create_timer(0.01, publish_cmd)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()
