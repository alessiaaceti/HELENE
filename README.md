# HELENE - low cost 6DOF robot

<a href="url"><img src="documentation/pictures/title.png" width="100%"></a>

Open the 3D model in your browser: https://a360.co/3EGeJgL

## Summary
This repo contains everything to build and use the low cost robot except the custom electronics, which can be [found in the seperate repo right here](https://github.com/SrSupp/Helene_electronics)
All files and an assembly guide can be found in the hardware_build directory.
### Get started
You can either run the robot itself, or a simulation.

## Simulation
Start the simulation environment:
```
roslaunch moveit_helene start_gazebo.launch 
```
Afterwards start moveit: 
```
roslaunch moveit_helene start_moveit.launch 
```


## Real Robot

You need:
-Robot itself. [See building instructions](hardware_build/AssemblyManualOnline.pdf) \
-Ubuntu or Debian machine running ROS melodic or noetic with this package. Noetic is preferred! [See installation guide](ROS_InstallationGuide.md)\
-24v Power supply \
-USB Cable 

To start the robot arm connect Helene to 24V and wait until it is fully aligned. Only then can the robot arm be connected to the computer via the USB cable! 
A terminal can then be started on the computer (Preffered!). Via the command: 

```
roslaunch applications_helene realrobot.launch 
```

the connection is established automatically, the PID controller and MoveIt are started. 

Alternatively, this can also be done separately via the commands: 
```
$ roslaunch controller_helene robot.launch 
$ roslaunch moveit_helene hardware_interface.launch 
$ roslaunch moveit_helene demo.launch 
```
each of which must be executed in a new terminal. 

## Start Demo
In applications_helene you can find 2 simple nodes to control the robot. Run using:
```
$ rosrun applications_helene demoSpiral 
$ rosrun applications_helene demoSpeed
```
demoSpiral draws a spiral, demoSpeed executes the same trajectory in full and reduced speed.

## Python Interface

Inside the applications_helene\scripts folder is the Python library helpy, which allows easy programming of the robot arm. An example is located in main. When the example is executed, the robot arm should move and also the LEDs should light up. Use this script as inspiration and show what the robot arm can do!\
We use the PyCharm IDE for Python programming, with which the created scripts can be executed directly. This is also great for students. 
Without an IDE, the script main can be executed by the command: 
```
python3 main.py
```

## jog Arm with a joystick/spacemouse or published EEF velocities
You probably need to modify /moveit_helene/config/helene_servoing.yaml 

- launch either Gazebo or the real robot
- move out of singularity, if starting in one

**switch from trajectory controller to group controller**
```
rosservice call helene/controller_manager/switch_controller "start_controllers: ['effort_group']
stop_controllers: ['arm_controller']
strictness: 2
start_asap: false
timeout: 0.0" 
```
**run the servo node**
```
roslaunch moveit_helene spacenav_cpp.launch 
```

### Support for custom end effectors

The robot supports the use of custom end effectors. These end effectors can be used by adding the argument "end_effector:=END_EFFECTOR_NAME", when launching the robot.

For more information regarding custom end effectors see the [corresponding documentation](helene_ros/src/end_effectors/custom_end_effectors.md).


### Troubleshooting
In this section we collect tips that can help with installation and operation in the event of errors. This list will be continuously expanded.

#### Firmware
If you encounter problems while compiling the ESP projects and get an error due to missing ros_lib headers, try building your customized libs like this:

```
cd <Path to the failing project>/lib
rm -rf ros_lib 
rosrun rosserial_arduino make_libraries.py .
```
#### Hardware assembly
Incorrect angle measurements: Each joint has its own encoder, which always consists of a sensor and a magnet. Make sure that the polarity of the magnet is diametrically opposed, only then can it work with the sensor. Make sure that the magnet cannot move in the recess. If the press fit with your magnet model is not sufficient, we recommend fixing it with a drop of superglue. Carry out a new calibration after every change to the robot (even after tensioning belts).

Motors move incorrectly/not at all: Make sure that the power supply is sufficient and use a multimeter to test that the supply is maintained even when the motors are energized. Check the wiring of the motors. Some stepper motor manufacturers use different plugs/cable colors. If the phases are mixed up, the motor may change direction or not move smoothly at all.

#### Operation
Robot does not react / does not hold position after loss of voltage: Parts of the electronics are supplied with voltage by both the power supply unit and the USB connection. If the 24V supply is interrupted but the USB connection is maintained, the ESP32 will continue to run. To restart the motors, the start sequence must be carried out again as described above, i.e. first disconnect both connections and then reconnect first 24V, then USB.

