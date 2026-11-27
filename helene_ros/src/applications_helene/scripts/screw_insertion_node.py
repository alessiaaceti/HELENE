#!/usr/bin/env python3
"""Demo 2 - screw insertion with force-regulated spiral search.

Start with the screw a few mm above the part, tool pointing down, arm still.
TARE -> APPROACH -> SEARCH (spiral, constant light force) -> INSERT -> DONE.
Any problem (stale sensor, tf error, force above abort_force) -> stop.

All parameters are ~private params named like InsertionParams fields, in SI
units (m, s, N). Before raising force_gain read max_stable_force_gain() in
force_control.py: too high a gain is what makes the force explode.
"""

from dataclasses import fields

import rospy
from std_msgs.msg import Float32, String

from applications_helene.force_control import (
    InsertionParams, ScrewInsertion, DONE, FAILED, max_stable_force_gain)
from applications_helene.ros_force_io import ForceIO


def load_params():
    p = InsertionParams()
    for f in fields(InsertionParams):
        if rospy.has_param("~" + f.name):
            setattr(p, f.name, rospy.get_param("~" + f.name))
    return p


def main():
    rospy.init_node("helene_screw_insertion")
    p = load_params()
    rate_hz = rospy.get_param("~rate", 100.0)

    # Sanity check of the gain against the expected contact stiffness/latency.
    k_env = rospy.get_param("~expected_stiffness", 20000.0)   # N/m
    latency = rospy.get_param("~expected_latency", 0.1)       # s
    kmax = max_stable_force_gain(k_env, latency, margin=1.0)
    if p.force_gain > kmax:
        rospy.logwarn("force_gain %.2g m/s/N with %.0f N/mm and %.0f ms latency is "
                      "likely to oscillate (limit ~%.2g). Lower it or add compliance.",
                      p.force_gain, k_env / 1000.0, latency * 1000.0, kmax)

    io = ForceIO()
    ctrl = ScrewInsertion(p)
    state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
    fz_pub = rospy.Publisher("~fz", Float32, queue_size=1)
    rospy.on_shutdown(io.stop)

    rospy.loginfo("waiting for force data...")
    while not rospy.is_shutdown() and not io.fresh():
        rospy.sleep(0.05)

    rate = rospy.Rate(rate_hz)
    dt = 1.0 / rate_hz
    last_state = None
    while not rospy.is_shutdown():
        if not io.fresh():
            ctrl.fail("force sensor data stale (> %.0f ms)" % (1e3 * io.timeout))
            v = [0.0, 0.0, 0.0]
        else:
            try:
                v = ctrl.step(io.force_world(), io.tool_position(), dt)
            except Exception as e:
                ctrl.fail("tf error: %s" % e)
                v = [0.0, 0.0, 0.0]
        io.send_velocity(v)
        st = ctrl.status
        fz_pub.publish(st.fz)
        if st.state != last_state:
            rospy.loginfo("[%s] %s", st.state, st.message)
            state_pub.publish("%s %s" % (st.state, st.message))
            last_state = st.state
        if st.state in (DONE, FAILED):
            for _ in range(10):        # make sure Servo sees the stop
                io.stop()
                rospy.sleep(0.01)
            break
        rate.sleep()


if __name__ == "__main__":
    main()
