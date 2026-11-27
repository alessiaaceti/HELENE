#!/usr/bin/env python3
"""Republishes a WrenchStamped topic with a fixed delay. Used in Gazebo to
emulate the latency of the real sensor path (CAN -> ESP -> micro-ROS ->
ros1_bridge), which the simulation does not have.

  ~input (default /force_sensor/wrench_gazebo), ~output (/force_sensor/wrench),
  ~delay [s] (default 0.0)
"""
from collections import deque

import rospy
from geometry_msgs.msg import WrenchStamped


def main():
    rospy.init_node("topic_delay")
    delay = rospy.Duration(float(rospy.get_param("~delay", 0.0)))
    pub = rospy.Publisher(rospy.get_param("~output", "/force_sensor/wrench"),
                          WrenchStamped, queue_size=10)
    queue = deque()

    def cb(msg):
        if delay.to_sec() <= 0.0:
            pub.publish(msg)
        else:
            queue.append((rospy.Time.now() + delay, msg))

    rospy.Subscriber(rospy.get_param("~input", "/force_sensor/wrench_gazebo"),
                     WrenchStamped, cb, queue_size=100, tcp_nodelay=True)
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        while queue and queue[0][0] <= now:
            pub.publish(queue.popleft()[1])
        rate.sleep()


if __name__ == "__main__":
    main()
