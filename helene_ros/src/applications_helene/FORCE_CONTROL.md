# Force control: hand guiding and screw insertion

Force sensor on the end effector → CAN → ESP → micro-ROS (ROS 2) → `ros1_bridge`
→ this ROS 1 stack → MoveIt Servo → `effort_group` (joint velocity controller)
→ rosserial → steppers.

| File | What |
|---|---|
| `src/applications_helene/force_control.py` | Algorithms, plain Python (no ROS): tare, filter, admittance, insertion state machine, stability formula |
| `src/applications_helene/insertion_sim.py` | Offline simulation: stiff surface with a hole, latency, 30 Hz HW loop, bias, noise |
| `src/applications_helene/ros_force_io.py` | ROS glue: sensor topic, rotation into `world`, tool pose from TF, Twist to Servo |
| `scripts/admittance_node.py` | Demo 1: push the sensor, the arm follows |
| `scripts/screw_insertion_node.py` | Demo 2: approach, spiral search, hole detection, insertion |
| `launch/force_control.launch` | Starts Servo (with the right output for the real robot) and one of the nodes |
| `test/test_force_control.py` | `python3 -m pytest test/` (no ROS needed) |
| `launch/force_simulation.launch` | Gazebo test bench: HELENE + `screw_tool` (simulated force sensor) + plate with a hole, real-robot controllers |
| `scripts/gazebo_force_test.py` | Automatic end-to-end test in Gazebo (`insertion` / `press` / `admittance`) |
| `scripts/topic_delay.py`, `scripts/sim_velocity_urdf.py` | Test bench helpers: sensor latency, stepper-like velocity joints in Gazebo |
| `end_effectors/screw_tool` | Sensor + screw end effector, `screw_plate.world` and the plate mesh |

## Why the force "explodes"

1. **Force loop gain too high for the latency (main cause).** The arm is a stiff
   position/velocity source (steppers) and the screw on the part is a stiff
   contact, `k_env` ≈ 10–100 N/mm. With `v_z = Kf·(F_ref − F)` the loop gain is
   `Kf·k_env` [1/s], and the signal path has a dead time `T` of roughly 0.1 s
   (CAN, ESP, micro-ROS, bridge, the 30 Hz loop in `my_robot_vel.cpp`, rosserial,
   filter lag). The loop is unstable when `Kf·k_env·T > π/2`. Each correction
   then overshoots the previous one, the force oscillates with growing amplitude
   and the screw bounces on and off the surface. Example: 20 N/mm and 0.1 s
   give a limit of 0.8 mm/s per N. A "reasonable looking" 2 mm/s per N
   is 2.5× over it. In Gazebo, 2 mm/s per N with 100 ms of sensor delay took
   the force from 0 to 15 N in 0.4 s (safety abort); the same gain without
   the delay held 2 N with 0.1 N of noise. Commanding a *position* into the surface is the extreme
   case (infinite gain): 0.5 mm of tracking error is already 10–50 N.
   → `max_stable_force_gain(k_env, T)` gives the gain. Use ~1/5 of the limit.
   The simulation reproduces it: `python3 -m applications_helene.insertion_sim`.

2. **MoveIt Servo sends joint positions to a velocity controller (real robot).**
   `helene_servoing.yaml` has `publish_joint_positions: true`. That is right for
   Gazebo, where `effort_group` is a `JointGroupPositionController`. On the real
   robot (`ros_controllers_jog.yaml`) `effort_group` is a
   `JointGroupVelocityController`, so a joint angle of 1.2 rad becomes a speed
   of 1.2 rad/s. The speed equals the angle, the angle keeps growing, and the
   arm runs away exponentially. `force_control.launch` overrides this for the
   real robot (`publish_joint_velocities: true`).

3. **No tare, wrong frame or wrong sign.** The weight of gripper + screw is a
   constant offset of several N. Without removing it the "contact" threshold is
   already exceeded (or never reached). It changes when the tool rotates, so
   tare in the working orientation. The sensor axes must be rotated into the
   world frame (`sensor_frame` via TF). If the sign is wrong the force loop
   becomes positive feedback: pressing makes it press harder. **Check:** push
   the screw tip up with a finger, then `rostopic echo /force_control/fz` must go positive.
   Otherwise set `force_sign:=-1`. (In Gazebo the simulated sensor needs
   `force_sign:=1`; with `-1` the admittance demo moved *against* the push.)

4. **Contact loss with an integrator or a fast "find the surface again".** When
   the screw drops into the hole (or bounces) the force goes to 0. A PI force
   controller, a high gain, or a fast re-approach then drives down at full
   speed, and with 0.1 s of latency it keeps going after the contact is back:
   the overshoot is `k_env·v·T`. In Gazebo, re-approaching at 2 mm/s after a
   contact loss produced 15 N spikes. Here there is no integrator, the
   re-approach uses the approach speed (1 mm/s), and the hole is detected and
   handled as its own state.

Also noticed (not changed):
- `my_robot_vel.cpp`: `read()` converts the measured velocity with
  `40/(71583·2π)` but `write()` uses `71583/π·2/40` (operator precedence).
  The two are not inverse of each other: they differ by a factor of
  π² ≈ 9.9. Check against the firmware which one is right.
- MoveIt Servo resolves `cartesian_command_in_topic` in its private namespace,
  so with `helene_servoing.yaml` it listens on
  `/servo_server/servo_server/delta_twist_cmds`. `force_control.launch` fixes
  that for the force nodes; `spacenav_cpp.launch` probably has the same problem.
- `realrobot.launch` loads `ros_controllers.yaml`, which has no `effort_group`.
- `helene_hw.urdf` declares the transmissions and `gazebo_ros_control` twice,
  the second time in the root namespace. That is harmless with effort joints,
  but with velocity joints it overwrites every command with 0
  (`sim_velocity_urdf.py` removes it for the test bench).

## Algorithm (screw insertion)

```
TARE      arm still, average 100 samples -> bias (weight of tool + screw)
APPROACH  v_z = -1 mm/s (x/y held) until fz > 1 N for 3 samples -> remember contact point
SEARCH    z : v_z = -Kf·(F_ref - fz), clamped, deadband   (holds ~2 N, never position)
              contact lost -> down at the approach speed, not faster
          xy: Archimedean spiral around the contact point, r = pitch·θ/2π,
              constant tangential speed, advances only while fz ≈ F_ref
          hole  <=> fz < 0.6·F_ref  AND  z dropped > 0.8 mm below contact
                    (or: pushed sideways > 4 N with no load below, or while already
                     0.3 mm down = tip caught on the hole edge)
          fail  <=> r > max radius, or blocked sideways on the flat surface
INSERT    push down with ~3 N, x/y yield to the lateral force (the hole edge
          pushes the tip towards the centre), until insert_depth, or no more
          progress for 1 s / fz > 6 N below 1 mm depth (seated, thread reached)
ALWAYS    |F| > abort_force, stale sensor data, TF error  -> stop
```

Choosing the parameters:
- `spiral_pitch` < hole clearance (hole radius − screw tip radius, chamfer
  helps), otherwise the spiral can step over the hole.
- `force_gain` ≤ `max_stable_force_gain(k_env, T)`. Measure `T` (step the
  velocity command, see when the force reacts) and `k_env` (ΔF/Δz just after
  contact).
- `approach_speed`: the overshoot at contact is about `k_env·v·T`
  (20 N/mm · 1 mm/s · 0.1 s = 2 N).
- If you know the height of the part, set `expected_surface_z`. If the screw
  is already above the hole it drops in during the approach. That is then
  recognised and the spiral is skipped.

**The most effective hardware change is compliance:** a spring or a rubber
element in the tool (spring-loaded bit holder) brings `k_env` down to
1–2 N/mm. The allowed gain then goes up 10–50×, the contact overshoot goes
down by the same factor, and everything gets much more forgiving. Lowering
the latency (raising the 30 Hz loop in `my_robot_vel.cpp`, a direct ROS 2
path without the bridge) helps in the same way.

Default `force_gain` is 0.05 mm/s per N. With 0.1 s of latency it is well
damped up to about 60 N/mm and stable up to about 300 N/mm. The price is a
slower force settling (about 1 s at 20 N/mm). Raise it only after measuring `k_env` and `T`.

## Test results

**Offline model** (`python3 -m pytest test/`, 43 tests; stiff surface,
80 ms latency, 30 Hz loop). The insertion succeeds for 5, 20 and 50 N/mm and
hole offsets up to 2 mm, with 50 and 100 ms of latency (36/36 cases, peak
force < 10 N). At 150 ms, 13/36 cases fail, mostly at 50 N/mm. A gain of
2 mm/s per N bounces on and off the surface (63 contact losses).

**Gazebo** (`force_simulation.launch` + `gazebo_force_test.py`). The arm runs
with the real-robot controllers and Servo settings, with stepper-like velocity
joints. The simulated force sensor runs at 100 Hz on the screw. The plate is a
single mesh with a 4 mm hole (0.5 mm chamfer, 3 mm deep, contact kp 20 N/mm).
The start misalignment comes from MoveIt and was 1.8 to 4.4 mm.

| test | sensor delay | result |
|---|---|---|
| insertion ×3 | 0 | 3/3 hole found and inserted, tip 0.5 mm from the hole centre, max 3.4 N |
| insertion ×3 | 100 ms | 3/3 inserted 4 mm, max 4.4 N |
| press (2 N on the flat plate, 24 s) | 0 / 100 ms | mean 2.1 / 2.2 N, std 0.34 / 0.14 N, never lost contact |
| press, Kf = 2 mm/s/N | 0 | stable, std 0.1 N |
| press, Kf = 2 mm/s/N, fast re-approach | 100 ms | **0 → 15 N in 0.4 s, abort** |
| admittance, 6 N push for 2 s | 0 | moved 25 mm along the push, 0 mm sideways, 0.7 mm drift after release |

In earlier runs, a misalignment above the search radius (5 mm in the test)
ended in a clean `FAILED no hole found` at about 3 N.

To run it yourself:
```
roslaunch applications_helene force_simulation.launch [sensor_delay:=0.1] [gazebo_gui:=true]
rosrun applications_helene gazebo_force_test.py insertion   # or press / admittance
```

## Running

See the comment at the top of `launch/force_control.launch`.

```
roslaunch applications_helene force_control.launch mode:=admittance
roslaunch applications_helene force_control.launch mode:=insertion
```

Things to check first, in this order: `rostopic hz /force_sensor/wrench` (rate
and gaps), sign with the finger test, admittance demo (low risk), then
insertion with a low `target_force`.
