#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial
import time

class HeleneSerialBridge(Node):
    def __init__(self):
        super().__init__('helene_serial_bridge_node')
        
        self.target_velocities = [0.0] * 6
        self.last_sent_velocities = [0] * 6
        
        # --- Multithreading Configuration ---
        from rclpy.callback_groups import ReentrantCallbackGroup
        self.callback_group = ReentrantCallbackGroup()
        
        # --- QoS Profile Selection ---
        # Using Best Effort to prevent message queuing delays
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
            
        # --- Subscribers ---
        self.subs = [
            self.create_subscription(Float32, '/joint1/target_velocity', self.cb_j1, qos_profile, callback_group=self.callback_group),
            self.create_subscription(Float32, '/joint2/target_velocity', self.cb_j2, qos_profile, callback_group=self.callback_group),
            self.create_subscription(Float32, '/joint3/target_velocity', self.cb_j3, qos_profile, callback_group=self.callback_group),
            self.create_subscription(Float32, '/joint4/target_velocity', self.cb_j4, qos_profile, callback_group=self.callback_group),
            self.create_subscription(Float32, '/joint5/target_velocity', self.cb_j5, qos_profile, callback_group=self.callback_group),
            self.create_subscription(Float32, '/joint6/target_velocity', self.cb_j6, qos_profile, callback_group=self.callback_group),
        ]
            
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

    # --- Joint Subscriber Callbacks ---
    def cb_j1(self, msg): self.target_velocities[0] = msg.data
    def cb_j2(self, msg): self.target_velocities[1] = msg.data
    def cb_j3(self, msg): self.target_velocities[2] = msg.data
    def cb_j4(self, msg): self.target_velocities[3] = msg.data
    def cb_j5(self, msg): self.target_velocities[4] = msg.data
    def cb_j6(self, msg): self.target_velocities[5] = msg.data

    def read_serial_callback(self):
        try:
            if self.ser.in_waiting > 0:
                raw_data = self.ser.read(self.ser.in_waiting)
                if b'\n' in raw_data:
                    # Extract the latest complete line from the buffer
                    lines = raw_data.decode('utf-8', errors='ignore').strip().split('\n')
                    line = lines[-1].strip()
                    
                    if not line: 
                        return
                    
                    data = line.split(',')
                    # Handle single-joint telemetry data (Angle, Velocity)
                    if len(data) == 2:
                        try:
                            msg_a = Float32(); msg_a.data = float(data[0])
                            self.pub_angles[0].publish(msg_a)
                            
                            msg_v = Float32(); msg_v.data = float(data[1])
                            self.pub_velocities[0].publish(msg_v)
                            
                            # Publish dummy data for remaining joints
                            for i in range(1, 6):
                                msg_fake = Float32(); msg_fake.data = 0.0
                                self.pub_angles[i].publish(msg_fake)
                                self.pub_velocities[i].publish(msg_fake)
                        except ValueError: 
                            pass
                    # Placeholder for multi-joint data format
                    elif len(data) >= 6: 
                        pass
                    else:
                        self.get_logger().info(f"[Serial Message]: {line}")
        except Exception:
            pass

    def write_serial_callback(self):
        try:
            # Convert target velocities to integers and format the command string
            vel_ints = [int(v) for v in self.target_velocities]
            serial_cmd = "V:" + ",".join(map(str, vel_ints)) + "\n"
            
            self.ser.write(serial_cmd.encode('utf-8'))
            
            # Log only when the target velocities change to avoid spamming stdout
            if vel_ints != self.last_sent_velocities:
                self.get_logger().info(f"Sent motor command: {serial_cmd.strip()}")
                self.last_sent_velocities = list(vel_ints)
        except Exception:
            pass
        
def main(args=None):
    rclpy.init(args=args)
    node = HeleneSerialBridge()
    
    # Use MultiThreadedExecutor to handle concurrent execution of timers and subscriptions
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