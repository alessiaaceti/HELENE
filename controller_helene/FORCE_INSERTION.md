# Force-guided screw insertion (ROS 2)

`screw_insertion_node` presses the tool on a surface, searches for the hole and inserts, using only the
force sensor (`/force_torque_sensor`) and the tool position. The algorithm is in
`include/controller_helene/insertion_controller.hpp` (no ROS dependency); the node feeds it the
force (tared, rotated into `base_link`) and the tip position and sends the resulting velocity out.

States: `TARE` (average 100 samples, arm still) -> `APPROACH` (down at 1 mm/s until `fz > contact_force`)
-> `SEARCH` (z force-controlled to `target_force`, x/y on an Archimedean spiral) -> `INSERT` (press with
`insert_force`, x/y yield to the lateral force so the hole edge centres the tip) -> `DONE` / `FAILED`.
Hole detection: `fz` drops *and* the tool is lower than `hole_depth` below the contact height, or the tip is
pushed sideways (`max_lateral_force`) against the edge. Safety: `abort_force`, sensor timeout, insertion timeout.

## Why the force "explodes"

The loop `v_z = Kf (F_ref - F)` around a stiff contact (`k`) with dead time `T` is an integrator with a delay and
is unstable when `Kf * k * T > pi/2` (`maxStableForceGain(k, T)` gives a safe gain). With a velocity command into a
rigid surface the force is `k * v * T` per control period. So: use a small gain (default `5e-5 m/s/N`), a slow
approach (1 mm/s) and never command a position into the surface.

## Simulation (Ignition, no hardware needed)

```
# plate with a 10 mm hole and 2 mm chamfer, reachable by the arm; sensor stiffness 20 N/mm
ros2 launch controller_helene helene_simulation.launch.py rviz:=false gz_flags:='-r -s' \
  teleop:=false servo_mode:=true insertion:=true sensor_stiffness:=20000 \
  world:=$(ros2 pkg prefix controller_helene)/share/controller_helene/worlds/helene_insertion_world.sdf
# bring the tool above the hole and start the insertion
ros2 run controller_helene sim_go_to_start.py --start
```

`IGN_GAZEBO_RESOURCE_PATH` must contain the `share` directories of `hw_description` and `controller_helene`
and `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` the library directory of `ign_ros2_control`.

Things this setup needs and why:

* `sensor_stiffness` (xacro arg of `hw_description`): without it the simulated contact is perfectly rigid and any
  velocity into the surface produces kilonewtons (1800 N measured). With it, q7 becomes a 3-axis spring
  (x, y, z, damped at 70 %), like the printed sensor. MoveIt and the insertion node use the rigid model.
* `direct_joint_control`: MoveIt Servo has a dead band of about 1 mm/s (measured: 0.5 mm/s command -> no motion,
  1 mm/s -> 0.03 mm/s, 2 mm/s -> 1.2 mm/s) and adds filter lag, so it cannot execute the sub-mm/s speeds of a force
  loop. The node therefore maps the Cartesian velocity to joint velocities (damped least squares, KDL Jacobian) and
  writes `/helene_velocity_controller/commands`. With `direct_joint_control:=false` it publishes a `TwistStamped` to
  `/servo_node/delta_twist_cmds` like the hand-guiding node.
* The plate of `helene_world.sdf` (x = 0.43 m) is not reachable with the tool pointing down; `helene_insertion_world.sdf`
  puts a plate at x = 0.30 m (`worlds/make_plate.py` regenerates the mesh).
* `sensor_sign_x/y/z`: the default `(-1, -1, +1)` is the one of `helene_force_servo_teleop`. The Gazebo sensor reports the
  force of the tool on the arm and needs `(1, 1, 1)` (the launch file sets it). **Check on the real sensor** by pushing
  the tip up with a finger: `/helene/insertion_fz` must go positive.

## Results in simulation

| case | result |
|---|---|
| start above the countersink, tool 2.7 mm off axis, no delay | contact on the chamfer, tip slides in, DONE at 4 mm |
| same, +0.1 s artificial delay on the force | DONE at 4 mm |
| start on the flat part (11.5 mm off axis), spiral search | **not solved**: the flat tool edge catches the chamfer rim, ends in `FAILED` (insertion stuck) |
| rigid contact (no sensor spring), velocity into plate | 1800 N peak: abort (this is the "explosion") |

The last two rows are limits of the test bench and of the algorithm, not proof it works on the real arm: the contact
model (DART, rigid bodies + one spring) and the flat tool tip are crude. Real data (rosbag of the sensor at rest and
during a push/contact) is needed to tune `filter_cutoff_hz`, `contact_force`, `target_force` and the sign.

## Parameters

All fields of `InsertionParams` are ROS parameters (see the header for units), plus `sensor_sign_*`, `sensor_frame`,
`command_frame`, `tool_frame`, `tool_tip_offset_z`, `control_rate`, `force_timeout`, `max_speed`, `sensor_delay`,
`direct_joint_control`, `dls_lambda`, `auto_start`. Services: `~/start` (re-tare and restart), `~/stop`.
Topics: `/helene/insertion_state`, `/helene/insertion_fz`, `/helene/insertion_depth`, `/helene/insertion_tip`.
