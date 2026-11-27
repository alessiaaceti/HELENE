#!/usr/bin/env python3
"""Prints the HELENE URDF for Gazebo with stepper-like joints: the joint
transmissions use VelocityJointInterface (as the real robot driver) instead
of EffortJointInterface.

helene_hw.urdf contains the transmissions twice, the second time with a
gazebo_ros_control instance in the root namespace "/". With effort joints
that instance only adds zero force; with velocity joints it would overwrite
every command with 0, so it is removed here.

Usage (in a launch file):
  <param name="robot_description"
         command="$(find applications_helene)/scripts/sim_velocity_urdf.py end_effector:=screw_tool"/>
"""
import re
import subprocess
import sys

import rospkg

xacro_file = rospkg.RosPack().get_path("hw_description") + "/urdf/helene_with_tool.xacro"
urdf = subprocess.check_output(["xacro", xacro_file] + sys.argv[1:], text=True)
urdf = urdf.replace("hardware_interface/EffortJointInterface",
                    "hardware_interface/VelocityJointInterface")
root_plugin = re.compile(
    r"<gazebo>\s*<plugin[^>]*gazebo_ros_control[^>]*>\s*"
    r"<robotNamespace>/</robotNamespace>\s*</plugin>\s*</gazebo>")
urdf, n = root_plugin.subn("", urdf)
if n != 1:
    sys.stderr.write("sim_velocity_urdf: expected 1 root gazebo_ros_control, found %d\n" % n)
sys.stdout.write(urdf)
