#!/usr/bin/env python3
"""Demo 1 - hand guiding: the arm holds still and moves where you push the
force sensor. Sends Cartesian velocities to MoveIt Servo.

Keep your hand off the sensor for the first second (tare)."""

import rospy
from std_msgs.msg import String

from applications_helene.force_control import Admittance, AdmittanceParams
from applications_helene.ros_force_io import ForceIO


def main():
    rospy.init_node("helene_admittance")
    p = AdmittanceParams(
        damping=rospy.get_param("~damping", AdmittanceParams.damping),
        deadband=rospy.get_param("~deadband", AdmittanceParams.deadband),
        max_speed=rospy.get_param("~max_speed", AdmittanceParams.max_speed),
        filter_cutoff_hz=rospy.get_param("~filter_cutoff_hz", AdmittanceParams.filter_cutoff_hz),
        tare_samples=rospy.get_param("~tare_samples", AdmittanceParams.tare_samples),
    )
    rate_hz = rospy.get_param("~rate", 100.0)
    io = ForceIO()
    ctrl = Admittance(p)
    state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
    rospy.on_shutdown(io.stop)

    rospy.loginfo("waiting for force data...")
    while not rospy.is_shutdown() and not io.fresh():
        rospy.sleep(0.05)
    rospy.loginfo("taring - do not touch the sensor")
    state_pub.publish("TARE")

    rate = rospy.Rate(rate_hz)
    dt = 1.0 / rate_hz
    was_tared = False
    while not rospy.is_shutdown():
        if not io.fresh():
            rospy.logwarn_throttle(1.0, "force data stale -> holding still")
            io.stop()
        else:
            try:
                v = ctrl.step(io.force_world(), dt)
            except Exception as e:  # tf not ready yet
                rospy.logwarn_throttle(1.0, "tf: %s", e)
                v = [0.0, 0.0, 0.0]
            io.send_velocity(v)
            if ctrl.tare.done and not was_tared:
                was_tared = True
                rospy.loginfo("bias %s N - push the sensor to move the arm", ctrl.tare.bias)
                state_pub.publish("GUIDING")
        rate.sleep()


if __name__ == "__main__":
    main()
