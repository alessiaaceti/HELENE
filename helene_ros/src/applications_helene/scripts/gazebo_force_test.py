#!/usr/bin/env python3
"""End-to-end Gazebo test of the force nodes (needs force_simulation.launch).

    rosrun applications_helene gazebo_force_test.py insertion
    rosrun applications_helene gazebo_force_test.py admittance
    rosrun applications_helene gazebo_force_test.py insertion "{force_gain: 0.002}"
          (extra node parameters, e.g. to reproduce the force blow-up)

insertion:  moves the screw 4 mm above the plate, 1.4 mm away from the hole,
            switches to effort_group, starts force_control.launch
            (mode:=insertion, real-robot Servo settings) and checks that the screw ends up in
            the hole without excessive force.
press:      starts 10 mm away from the hole and only searches on the flat
            plate (spiral limited to 3 mm): measures how well the pressing
            force is held. Use it with force_simulation.launch sensor_delay:=0.1
            and a high force_gain to reproduce the force blow-up.
admittance: same start pose (higher up), starts mode:=admittance, pushes the
            screw with a 6 N force in +x via Gazebo and checks that the arm
            moves in +x and stops after the push.
A CSV log is written to ~/.ros/gazebo_force_test_<mode>.csv.
"""

import math
import os
import subprocess
import sys

import moveit_commander
import rospy
import tf2_ros
from controller_manager_msgs.srv import SwitchController
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Point, Pose, Quaternion, Wrench, WrenchStamped, Vector3
from std_msgs.msg import Float32, String

# must match end_effectors/screw_tool/worlds/screw_plate.world
HOLE_XY = (0.2012, -0.0008)
PLATE_TOP = 0.05
START_XY = (0.200, 0.000)
TOOL_LEN = 0.035           # axis_6 -> screw_tip (ft sensor 10 mm + screw 25 mm)


class Recorder:
    def __init__(self):
        self.state = ""
        self.fz = 0.0
        self.rows = []
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber("/force_control/state", String, self._state)
        rospy.Subscriber("/force_control/fz", Float32, self._fz)
        self.raw = (0.0, 0.0, 0.0)
        rospy.Subscriber("/force_sensor/wrench", WrenchStamped, self._raw)

    def _raw(self, msg):
        f = msg.wrench.force
        self.raw = (f.x, f.y, f.z)

    def _state(self, msg):
        self.state = msg.data
        rospy.loginfo("node state: %s", msg.data)

    def _fz(self, msg):
        self.fz = msg.data

    def tip(self):
        t = self.tf_buffer.lookup_transform("world", "screw_tip", rospy.Time(0), rospy.Duration(1.0))
        p = t.transform.translation
        return p.x, p.y, p.z

    def sample(self):
        x, y, z = self.tip()
        self.rows.append((rospy.get_time(), self.state.split(" ")[0], x, y, z, self.fz) + self.raw)
        return x, y, z

    def save(self, mode):
        path = os.path.expanduser("~/.ros/gazebo_force_test_%s.csv" % mode)
        with open(path, "w") as f:
            f.write("t,state,x,y,z,fz,raw_fx,raw_fy,raw_fz\n")
            for r in self.rows:
                f.write("%.3f,%s,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f\n" % r)
        rospy.loginfo("log: %s", path)


def move_above(z_tip, start_xy=START_XY):
    group = moveit_commander.MoveGroupCommander("helene_arm")
    group.set_max_velocity_scaling_factor(0.3)
    group.set_named_target("home")
    group.go(wait=True)
    pose = Pose(position=Point(start_xy[0], start_xy[1], z_tip + TOOL_LEN),
                orientation=Quaternion(1.0, 0.0, 0.0, 0.0))   # tool z pointing down
    group.set_goal_position_tolerance(0.0005)
    group.set_goal_orientation_tolerance(0.005)
    # The trajectory controller has loose goal tolerances (1 rad in
    # ros_controllers.yaml), so re-plan from where it stopped until the
    # flange is within 2.5 mm of the target (the search covers the rest).
    ok = False
    for _ in range(4):
        group.set_pose_target(pose, "axis_6")
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(1.0)
        p = group.get_current_pose("axis_6").pose.position
        err = math.dist((p.x, p.y, p.z), (pose.position.x, pose.position.y, pose.position.z))
        rospy.loginfo("start pose error %.2f mm", err * 1e3)
        if err < 0.0025:
            ok = True
            break
    return ok


def switch_to_streaming():
    rospy.wait_for_service("/helene/controller_manager/switch_controller")
    sw = rospy.ServiceProxy("/helene/controller_manager/switch_controller", SwitchController)
    res = sw(start_controllers=["effort_group"], stop_controllers=["VelocityJointInterface_trajectory_controller"],
             strictness=2, start_asap=False, timeout=0.0)
    return res.ok


def start_nodes(mode, extra):
    cmd = ["roslaunch", "applications_helene", "force_control.launch",
           "mode:=%s" % mode, "load_planning_context:=false", "tool_frame:=screw_tip",
           "force_sign:=1.0", "extra_params:=%s" % extra]
    rospy.loginfo("starting: %s", " ".join(cmd))
    return subprocess.Popen(cmd)


def run_insertion(rec, extra=""):
    params = "expected_surface_z: %.3f, spiral_max_radius: 0.005" % PLATE_TOP
    if extra:
        params += ", " + extra.strip().strip("{}")
    proc = start_nodes("insertion", "{%s}" % params)
    t0 = rospy.get_time()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and rospy.get_time() - t0 < 120.0:
        rec.sample()
        s = rec.state.split(" ")[0]
        if s in ("DONE", "FAILED"):
            break
        rate.sleep()
    for _ in range(50):           # settle
        rec.sample()
        rate.sleep()
    proc.terminate()
    proc.wait()

    x, y, z = rec.tip()
    first = next(r for r in rec.rows if r[1] == "APPROACH")
    rospy.loginfo("misalignment at start: %.2f mm",
                  1e3 * math.hypot(first[2] - HOLE_XY[0], first[3] - HOLE_XY[1]))
    d = math.hypot(x - HOLE_XY[0], y - HOLE_XY[1])
    fmax = max(r[5] for r in rec.rows)
    fsearch = [r[5] for r in rec.rows if r[1] == "SEARCH"]
    rospy.loginfo("RESULT state=%s | tip %.2f mm from hole centre, %.2f mm below plate top | "
                  "max fz %.2f N | search fz mean %.2f N", rec.state, d * 1e3,
                  (PLATE_TOP - z) * 1e3, fmax,
                  sum(fsearch) / len(fsearch) if fsearch else float("nan"))
    return rec.state.startswith("DONE") and d < 0.002 and z < PLATE_TOP - 0.001 and fmax < 10.0


def run_press(rec, extra=""):
    params = "expected_surface_z: %.3f, spiral_max_radius: 0.003" % PLATE_TOP
    if extra:
        params += ", " + extra.strip().strip("{}")
    proc = start_nodes("insertion", "{%s}" % params)
    t0 = rospy.get_time()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and rospy.get_time() - t0 < 90.0:
        rec.sample()
        if rec.state.split(" ")[0] in ("DONE", "FAILED"):
            break
        rate.sleep()
    proc.terminate()
    proc.wait()
    search = [r for r in rec.rows if r[1] == "SEARCH"]
    settled = [r[5] for r in search[len(search) // 5:]]   # skip the contact transient
    if not settled:
        rospy.loginfo("RESULT state=%s, no search phase", rec.state)
        return False
    mean = sum(settled) / len(settled)
    std = math.sqrt(sum((f - mean) ** 2 for f in settled) / len(settled))
    lost = sum(1 for f in settled if f < 0.3)
    rospy.loginfo("RESULT state=%s | search %.1f s | fz mean %.2f N, std %.2f N, "
                  "min %.2f N, max %.2f N, samples without contact %d/%d | peak fz %.2f N",
                  rec.state, search[-1][0] - search[0][0], mean, std, min(settled),
                  max(settled), lost, len(settled), max(r[5] for r in rec.rows))
    return std < 0.5 and lost == 0


def run_admittance(rec):
    proc = start_nodes("admittance", "{}")
    rate = rospy.Rate(50)
    t0 = rospy.get_time()
    while not rospy.is_shutdown() and "GUIDING" not in rec.state and rospy.get_time() - t0 < 30:
        rec.sample()
        rate.sleep()
    rospy.sleep(1.0)
    x0, y0, z0 = rec.sample()

    rospy.wait_for_service("/gazebo/apply_body_wrench")
    push = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
    push(body_name="helene_arm::screw_link", reference_frame="world",
         wrench=Wrench(force=Vector3(6.0, 0.0, 0.0), torque=Vector3(0, 0, 0)),
         start_time=rospy.Time(0), duration=rospy.Duration(2.0))
    t0 = rospy.get_time()
    while rospy.get_time() - t0 < 2.0:
        rec.sample()
        rate.sleep()
    x1, y1, z1 = rec.sample()
    t0 = rospy.get_time()
    while rospy.get_time() - t0 < 2.0:      # after the push: must stop
        rec.sample()
        rate.sleep()
    x2, y2, z2 = rec.sample()
    proc.terminate()
    proc.wait()
    dx, drift = x1 - x0, math.dist((x1, y1, z1), (x2, y2, z2))
    rospy.loginfo("RESULT push +x 6 N for 2 s: moved dx=%.1f mm dy=%.1f mm dz=%.1f mm; "
                  "drift in 2 s after release %.1f mm", dx * 1e3, (y1 - y0) * 1e3,
                  (z1 - z0) * 1e3, drift * 1e3)
    return dx > 0.01 and abs(y1 - y0) < 0.5 * dx and abs(z1 - z0) < 0.5 * dx and drift < 0.005


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "insertion"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("gazebo_force_test")
    rec = Recorder()
    z_start = PLATE_TOP + (0.05 if mode == "admittance" else 0.004)
    start_xy = (START_XY[0], START_XY[1] + 0.010) if mode == "press" else START_XY
    if not move_above(z_start, start_xy):
        rospy.logerr("RESULT FAIL: could not reach the start pose")
        sys.exit(2)
    rospy.loginfo("start tip position: %s", ["%.4f" % c for c in rec.sample()])
    if not switch_to_streaming():
        rospy.logerr("RESULT FAIL: controller switch failed")
        sys.exit(2)
    extra = sys.argv[2] if len(sys.argv) > 2 else ""
    if mode == "insertion":
        ok = run_insertion(rec, extra)
    elif mode == "press":
        ok = run_press(rec, extra)
    else:
        ok = run_admittance(rec)
    rec.save(mode)
    rospy.loginfo("RESULT %s: %s", mode, "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
