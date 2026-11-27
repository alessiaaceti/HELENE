#!/usr/bin/env python3
"""Simulation helper: bring the arm to a pose with the tool pointing down, 6 mm above the
insertion test plate (worlds/helene_insertion_world.sdf), a few mm off the hole centre.

Open-loop-free: a P controller on /joint_states writes joint velocities to the velocity
controller (start the simulation with servo_mode:=true). When the pose is reached it
optionally starts the insertion node (--start).

  ros2 run controller_helene sim_go_to_start.py --start
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

JOINTS = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6']
# tool tip at x=0.302, y=0.0015, z=0.062 (plate top at z=0.05, hole centre (0.30, 0.0)), tool pointing down
START = [0.0051, 0.8068, 1.445, 0.0005, 0.8904, -1.1589]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--q', type=float, nargs=6, default=START)
    ap.add_argument('--start', action='store_true', help='call screw_insertion_node/start when done')
    ap.add_argument('--timeout', type=float, default=60.0)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Node('sim_go_to_start', parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = {}
    node.create_subscription(JointState, '/joint_states',
                             lambda m: state.update(zip(m.name, m.position)), 10)
    pub = node.create_publisher(Float64MultiArray, '/helene_velocity_controller/commands', 10)
    t_end = time.time() + args.timeout
    ok = False
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.01)
        if any(j not in state for j in JOINTS):
            continue
        err = [t - state[j] for t, j in zip(args.q, JOINTS)]
        msg = Float64MultiArray()
        msg.data = [max(-0.3, min(0.3, 1.5 * e)) for e in err]
        if max(abs(e) for e in err) < 0.001:
            msg.data = [0.0] * 6
            pub.publish(msg)
            ok = True
            break
        pub.publish(msg)
        time.sleep(0.01)
    if not ok:
        print('start pose not reached', file=sys.stderr)
        sys.exit(1)
    for _ in range(50):          # let the arm settle before the force tare
        pub.publish(Float64MultiArray(data=[0.0] * 6))
        time.sleep(0.02)
    print('start pose reached')
    if args.start:
        cli = node.create_client(Trigger, '/screw_insertion_node/start')
        if cli.wait_for_service(timeout_sec=10.0):
            fut = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            print(fut.result().message if fut.result() else 'start: no answer')
        else:
            print('screw_insertion_node not available', file=sys.stderr)
            sys.exit(1)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
