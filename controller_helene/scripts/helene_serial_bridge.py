#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial
import time
from control_msgs.msg import JointTrajectoryControllerState
import math

class HeleneSerialBridge(Node):
    def __init__(self):
        super().__init__('helene_serial_bridge_node')
        
        self.target_velocities = [0.0] * 6
        self.last_sent_velocities = [0] * 6
        
        # --- Multithreading Configuration ---
        from rclpy.callback_groups import ReentrantCallbackGroup
        self.callback_group = ReentrantCallbackGroup()
        
        # --- QoS Profile Selection ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- Publishers ---
        self.pub_angles = []
        self.pub_velocities = []
        for i in range(1, 7):
            self.pub_angles.append(self.create_publisher(Float32, f'/joint{i}/actual_angle', 10))
            self.pub_velocities.append(self.create_publisher(Float32, f'/joint{i}/actual_velocity', 10))
            
        # --- Subscriber UNICO a MoveIt ---
        self.sub_state = self.create_subscription(
            JointTrajectoryControllerState,
            '/helene_trajectory_controller/controller_state',
            self.trajectory_state_cb,
            qos_profile,
            callback_group=self.callback_group
        )
        
        # --- Serial Connection Setup ---
        try:
            self.ser = serial.Serial()
            self.ser.port = '/dev/ttyUSB0'
            self.ser.baudrate = 576000
            self.ser.timeout = 0  # Non-blocking mode
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
            
            self.get_logger().info("Serial port opened. Waiting 2 seconds for initialization...")
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.get_logger().info("Serial bridge connected and ready.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            exit(1)
            
        # --- Timers ---
        self.create_timer(0.01, self.read_serial_callback, callback_group=self.callback_group)   # 100 Hz
        self.create_timer(0.05, self.write_serial_callback, callback_group=self.callback_group)  # 20 Hz

    # --- CALLBACK MOVEIT ---
    def trajectory_state_cb(self, msg):
        if len(msg.desired.velocities) >= 6:
            for i in range(6):
                # velocity multiplicator 
                # multiply rad/s by 11287 to get motor command in ticks/s (assuming 1 rad/s = 11287 ticks/s for the specific motor/encoder setup)
                velocita_rad_s = msg.desired.velocities[i]
                velocita_motore = velocita_rad_s * 11287.0
                
                self.target_velocities[i] = velocita_motore

    # --- Serial Reading ---
    def read_serial_callback(self):
        try:
            if self.ser.in_waiting > 0:
                raw_data = self.ser.read(self.ser.in_waiting)
                if b'\n' in raw_data:
                    lines = raw_data.decode('utf-8', errors='ignore').strip().split('\n')
                    line = lines[-1].strip()
                    
                    if not line: 
                        return
                    
                    data = line.split(',')
                    if len(data) == 2:
                        try:
                            msg_a = Float32(); msg_a.data = float(data[0])
                            self.pub_angles[0].publish(msg_a)
                            
                            msg_v = Float32(); msg_v.data = float(data[1])
                            self.pub_velocities[0].publish(msg_v)
                            
                            for i in range(1, 6):
                                msg_fake = Float32(); msg_fake.data = 0.0
                                self.pub_angles[i].publish(msg_fake)
                                self.pub_velocities[i].publish(msg_fake)
                        except ValueError: 
                            pass
                    elif len(data) >= 6: 
                        pass
                    else:
                        self.get_logger().info(f"[Serial Message]: {line}")
        except Exception:
            pass

    # --- Serial Writing ---
    def write_serial_callback(self):
        try:
            vel_ints = [int(v) for v in self.target_velocities]
            serial_cmd = "V:" + ",".join(map(str, vel_ints)) + "\n"
            
            self.ser.write(serial_cmd.encode('utf-8'))
            
            if vel_ints != self.last_sent_velocities:
                self.get_logger().info(f"Sent motor command: {serial_cmd.strip()}")
                self.last_sent_velocities = list(vel_ints)
        except Exception:
            pass
        
def main(args=None):
    rclpy.init(args=args)
    node = HeleneSerialBridge()
    
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try: 
        executor.spin()
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()