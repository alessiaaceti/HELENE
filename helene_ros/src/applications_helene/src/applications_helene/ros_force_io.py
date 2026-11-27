"""ROS (noetic) glue shared by the force nodes: reads the force sensor,
rotates it into the world frame, reads the tool position from TF and sends
Cartesian velocity commands to MoveIt Servo."""

import rospy
import tf2_ros
from geometry_msgs.msg import TwistStamped, Vector3, Vector3Stamped, WrenchStamped


def _quat_rotate(q, v):
    """Rotate vector v by unit quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    # v' = v + w*t + cross(q.xyz, t)
    return [vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx)]


class ForceIO:
    MSG_TYPES = {"wrench": WrenchStamped, "vector3_stamped": Vector3Stamped,
                 "vector3": Vector3}

    def __init__(self):
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.tool_frame = rospy.get_param("~tool_frame", "axis_6")
        # Frame of the force sensor. Empty -> use header.frame_id of the msg.
        self.sensor_frame = rospy.get_param("~sensor_frame", "")
        # +1 or -1: push the tool tip UP with a finger, world fz must go > 0.
        self.force_sign = float(rospy.get_param("~force_sign", 1.0))
        # Raw sensor units -> newtons (e.g. if the board sends grams or counts).
        self.force_scale = float(rospy.get_param("~force_scale", 1.0))
        self.timeout = float(rospy.get_param("~sensor_timeout", 0.1))
        msg_type = rospy.get_param("~msg_type", "wrench")
        topic = rospy.get_param("~force_topic", "/force_sensor/wrench")
        twist_topic = rospy.get_param("~twist_topic", "/servo_server/delta_twist_cmds")

        self._f = None
        self._frame = self.sensor_frame
        self._stamp = rospy.Time(0)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.sub = rospy.Subscriber(topic, self.MSG_TYPES[msg_type], self._cb,
                                    queue_size=1, tcp_nodelay=True)
        self.pub = rospy.Publisher(twist_topic, TwistStamped, queue_size=1)
        rospy.loginfo("force input: %s (%s), sign %+g, scale %g", topic, msg_type,
                      self.force_sign, self.force_scale)

    def _cb(self, msg):
        if isinstance(msg, WrenchStamped):
            v = msg.wrench.force
            frame = msg.header.frame_id
        elif isinstance(msg, Vector3Stamped):
            v = msg.vector
            frame = msg.header.frame_id
        else:
            v = msg
            frame = ""
        k = self.force_sign * self.force_scale
        self._f = [k * v.x, k * v.y, k * v.z]
        if not self.sensor_frame and frame:
            self._frame = frame
        # Stamp with arrival time: the ESP clock is not synchronised.
        self._stamp = rospy.Time.now()

    def fresh(self):
        return self._f is not None and (rospy.Time.now() - self._stamp).to_sec() < self.timeout

    def force_world(self):
        """Latest force rotated into the world frame (bias NOT removed)."""
        if not self._frame:
            return list(self._f)   # no frame given: assume sensor axes == world
        t = self.tf_buffer.lookup_transform(self.world_frame, self._frame,
                                            rospy.Time(0), rospy.Duration(0.05))
        q = t.transform.rotation
        return _quat_rotate((q.x, q.y, q.z, q.w), self._f)

    def tool_position(self):
        t = self.tf_buffer.lookup_transform(self.world_frame, self.tool_frame,
                                            rospy.Time(0), rospy.Duration(0.05))
        p = t.transform.translation
        return [p.x, p.y, p.z]

    def send_velocity(self, v):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.world_frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = v
        self.pub.publish(msg)

    def stop(self):
        self.send_velocity([0.0, 0.0, 0.0])
