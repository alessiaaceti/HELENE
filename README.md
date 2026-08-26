# HELENE - Low-Cost 6-DoF Robot

HELENE is a low-cost 6-degree-of-freedom robotic arm whose mechanical structure is primarily fabricated using 3D-printed components.

This repository contains the robot description, hardware interface, ROS 2 control packages, MoveIt 2 configuration, simulation files, and hardware assembly documentation.

The custom electronics are documented in the `helene_electronics` directory.

> **Current development branch:** `ros2-migration-v01`
>
> This branch contains the ongoing migration of the HELENE software stack from ROS 1 to ROS 2.

---

## Repository structure

The main components of the repository are:

| Directory | Description |
|---|---|
| `controller_helene/` | ROS 2 hardware interface, controllers and teleoperation nodes |
| `helene_moveit_config/` | MoveIt 2 configuration |
| `helene_msgs/` | Custom ROS 2 messages |
| `hw_description/` | URDF/Xacro robot description and hardware configuration |
| `hardware_build/` | Mechanical parts and assembly instructions |
| `helene_electronics/` | Electronics hardware and ESP32 firmware |
| `documentation/` | Additional documentation and pictures |
| `linux_helpers/` | Linux helper scripts and desktop launchers |
| `screw_driver_description/` | Screwdriver end-effector description |

---

# Requirements

The current ROS 2 software stack is intended to run with:

- Ubuntu/Linux system
- ROS 2 Humble
- ROS 2 `colcon` build system
- MoveIt 2
- `ros2_control`
- Micro-ROS
- Python 3

The exact ROS 2 dependencies are declared in the individual `package.xml` files.

---

# Installation

## 1. Install ROS 2 Humble

Install ROS 2 Humble following the official ROS 2 documentation.

After installing ROS 2, source the environment:

    source /opt/ros/humble/setup.bash

To source ROS 2 automatically in every terminal:

    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

---

## 2. Create a ROS 2 workspace

Create a new workspace:

    mkdir -p ~/helene_ws/src
    cd ~/helene_ws/src

Clone this repository:

    git clone -b ros2-migration-v01 https://github.com/alessiaaceti/HELENE.git

Then return to the workspace:

    cd ~/helene_ws

---

## 3. Install dependencies

Use `rosdep` to install the dependencies of the workspace:

    rosdep update
    rosdep install --from-paths src --ignore-src -r -y

If some dependencies are not available through `rosdep`, install the corresponding ROS 2 packages manually.

---

## 4. Build the workspace

Build the workspace using `colcon`:

    cd ~/helene_ws
    colcon build --symlink-install

After a successful build, source the workspace:

    source install/setup.bash

It is recommended to source the workspace in every new terminal:

    source ~/helene_ws/install/setup.bash

---

# Running HELENE

The repository currently supports both simulation and the physical robot.

## Simulation

The simulation launch file is:

    ros2 launch controller_helene helene_simulation.launch.py

The simulation configuration is provided through the `controller_helene` and `helene_moveit_config` packages.

> **Note:** The current research experiments are performed on the physical robot rather than in simulation.

---

# Running the real robot

## Hardware requirements

To operate the physical HELENE robot you need:

- HELENE robotic arm
- 24 V power supply
- USB connection to the computer
- ESP32-based robot electronics
- Linux computer running ROS 2 Humble

The robot electronics use ESP32-based motor control boards. The boards communicate through CAN, with the master controller providing the interface to the host computer.

For details about the electronics, see:

`helene_electronics/`

---

## Hardware connection

Before starting ROS 2:

1. Connect the HELENE robot to the 24 V power supply.
2. Allow the robot to complete its startup/alignment procedure.
3. Connect the robot to the computer through USB.
4. Make sure the expected serial device is available.

The current real-robot launch configuration expects the Micro-ROS serial device:

    /dev/helene_esp

at:

    460800 baud

If this device is not available, check the Linux helper configuration and the USB/serial connection before starting the robot.

---

## Start the real robot

Source ROS 2 and the workspace:

    source /opt/ros/humble/setup.bash
    source ~/helene_ws/install/setup.bash

Then launch the complete real-robot system:

    ros2 launch controller_helene helene_real.launch.py

This launch file starts the main components required for operation, including:

- `ros2_control_node`
- `robot_state_publisher`
- MoveIt 2
- RViz2
- MoveIt Servo
- joint state broadcaster
- HELENE velocity controller
- HELENE trajectory controller
- force sensor bridge
- force-based admittance controller
- Micro-ROS serial agent

The real-robot launch configuration uses:

    planning frame:       base_link
    end-effector frame:   axis_6
    command frame:        base_link

---

# Force Sensor and Admittance Control

The current branch includes a force-based Cartesian teleoperation pipeline.

The system receives three-axis force measurements from the force sensor mounted at the end-effector.

The processing pipeline is:

    Force sensor
         |
         v
    raw_meas_vector
         |
         v
    raw_meas_to_wrench
         |
         v
    /force_torque_sensor
         |
         v
    helene_force_servo_teleop
         |
         +--> bias compensation
         |
         +--> force scaling
         |
         +--> EMA filtering
         |
         +--> TF transformation
         |    axis_6 -> base_link
         |
         +--> Cartesian admittance control
         |
         v
    TwistStamped
         |
         v
    MoveIt Servo
         |
         v
    HELENE robot

The corresponding ROS 2 nodes are implemented in:

`controller_helene/src/raw_meas_to_wrench.cpp`

`controller_helene/src/helene_force_servo_teleop.cpp`

---

# Force sensor input

The raw sensor measurements are received through:

    raw_meas_vector

and converted to:

    /force_torque_sensor

using a `geometry_msgs/WrenchStamped` message.

The sensor frame is:

    axis_6

The bridge node is:

    raw_meas_to_wrench

The real-robot launch configuration currently sets the force scaling parameter to:

    scale_factor = 1000.0

The exact physical calibration and interpretation of this scale factor should be verified against the force sensor calibration procedure.

---

# Force-based Cartesian control

The node:

    helene_force_servo_teleop

subscribes to:

    /force_torque_sensor

and processes the three force components:

    Fx
    Fy
    Fz

The force measurements are first bias-compensated.

At startup, the controller collects 100 force measurements while the sensor is not being touched and computes the average value for each axis.

The force signal is then filtered using an exponential moving average (EMA):

    F_filtered(k) =
        alpha * F(k)
        + (1 - alpha) * F_filtered(k-1)

The current implementation uses:

    alpha = 0.25

The filtered force vector is transformed from:

    axis_6

to:

    base_link

using TF2.

---

## Admittance controller

The controller implements independent Cartesian mass-damper dynamics for the three translational axes.

The continuous-time model is:

    M * v_dot + B * v = F

or equivalently:

    v_dot = (F / B - v) / tau

with:

    M = B * tau

The controller is updated at a fixed control frequency and integrates the velocity state using the corresponding timestep.

The commanded angular velocity is set to zero.

The resulting Cartesian velocity command is published to MoveIt Servo using:

    geometry_msgs/TwistStamped

The command frame is:

    base_link

and the MoveIt Servo end-effector frame is:

    axis_6

---

# Force control safety features

The admittance controller includes several mechanisms to limit or stabilize the commanded motion:

- force deadzone
- force-data timeout
- Cartesian velocity saturation
- velocity threshold
- separate damping and time constants for X, Y and Z

If fresh force data are not received within the configured timeout, the target force is set to zero, causing the commanded velocity to decelerate toward zero.

---

# ROS 2 topics

The main topics involved in the force-control pipeline are:

| Topic | Message type | Description |
|---|---|---|
| `raw_meas_vector` | `geometry_msgs/Vector3` | Raw three-axis sensor measurements |
| `/force_torque_sensor` | `geometry_msgs/WrenchStamped` | Force measurements expressed in the sensor frame |
| `/servo_node/delta_twist_cmds` | `geometry_msgs/TwistStamped` | Cartesian velocity commands sent to MoveIt Servo |
| `/joint_states` | `sensor_msgs/JointState` | Robot joint states |
| `/tf` | `tf2_msgs/TFMessage` | Robot coordinate transforms |

Additional diagnostic topics may be added to the force controller for experimental data acquisition.

To inspect available topics:

    ros2 topic list

To inspect the force sensor:

    ros2 topic echo /force_torque_sensor

To inspect the Cartesian velocity command:

    ros2 topic echo /servo_node/delta_twist_cmds

---

# Recording experimental data

ROS 2 bags can be used to record the force-control experiments.

First inspect the available topics:

    ros2 topic list

For example:

    ros2 bag record \
      /force_torque_sensor \
      /servo_node/delta_twist_cmds \
      /joint_states \
      /tf \
      -o experiment_name

After the experiment, inspect the recorded bag:

    ros2 bag info experiment_name

The recommended experimental recordings should contain, at minimum:

- three-axis force measurements
- commanded Cartesian velocity
- joint states
- TF information

Additional diagnostic topics should be recorded if available.

---

# Experimental procedure

For interaction-control experiments, the force sensor is mounted on the HELENE end-effector.

The robot is controlled using the force-based Cartesian admittance controller.

For directional tests, apply an external force independently along:

    +X
    -X
    +Y
    -Y
    +Z
    -Z

Repeat each test multiple times to evaluate the consistency of the resulting motion.

For quantitative analysis, record the corresponding ROS 2 bags and compare the measured force with the commanded Cartesian velocity.

---

# Controller parameters

The main force-control parameters are configured in:

`controller_helene/src/helene_force_servo_teleop.cpp`

and/or through ROS 2 parameters.

Important parameters include:

    control_rate
    deadzone
    damping_x
    damping_y
    damping_z
    time_constant_x
    time_constant_y
    time_constant_z
    force_timeout
    velocity_epsilon
    max_linear_vel
    sensor_frame
    command_frame
    scale_factor

Always verify the parameter values used for a specific experiment before comparing experimental results.

---

# Mechanical construction

The mechanical parts of HELENE are primarily 3D printed.

The mechanical parts, CAD files and assembly instructions are available in:

`hardware_build/`

For the original robot design and additional documentation, see the project documentation and the original HELENE repository.

---

# Electronics

The robot uses ESP32-based motor-control boards.

The electronics repository includes:

- motor control PCBs
- ESP32 firmware
- CAN communication
- magnetic encoders
- stepper motor drivers
- end-effector electronics

See:

`helene_electronics/`

for the electronics documentation.

---

# Troubleshooting

## The robot does not move

Check:

1. The 24 V power supply.
2. USB connection.
3. Availability of `/dev/helene_esp`.
4. Whether the Micro-ROS agent started successfully.
5. Whether the controllers are active.

Check the controller state:

    ros2 control list_controllers

---

## Check the serial device

    ls -l /dev/helene_esp

If the device is not available, check the USB connection and the Linux helper/udev configuration.

---

## Check ROS 2 nodes

    ros2 node list

The real-robot launch should start the main HELENE, MoveIt Servo and force-control nodes.

---

## Check controllers

    ros2 control list_controllers

The velocity controller used by MoveIt Servo should be active when Cartesian velocity commands are being sent.

---

## Check TF

The force controller requires the transform:

    axis_6 -> base_link

Check the transform with:

    ros2 run tf2_ros tf2_echo base_link axis_6

---

# Development

The current ROS 2 migration is being developed in the branch:

    ros2-migration-v01

The ROS 2 migration includes changes to:

- package build system
- ROS 2 C++ APIs
- launch files
- `ros2_control`
- MoveIt 2
- ROS 2 message interfaces
- hardware communication
- force-based Cartesian control

The ROS 1 implementation is not the recommended entry point for the current development branch.

---

# License

See the `LICENSE` file for licensing information.
