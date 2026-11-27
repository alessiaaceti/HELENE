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
   is 2.5× over it. Commanding a *position* into the surface is the extreme
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
   Otherwise set `force_sign:=-1`.

4. **Contact loss with an integrator.** When the screw drops into the hole the
   force goes to 0. A PI force controller (or a high gain) then accelerates down
   at full speed and slams into the bottom. Here the z speed is clamped
   (`max_z_speed`), there is no integrator, and the hole is detected and
   handled as its own state.

Also noticed (not changed): in `my_robot_vel.cpp` `read()` converts the
measured velocity with `40/(71583·2π)` but `write()` uses `71583/π·2/40`
(operator precedence). The two are not inverse of each other: they differ by
a factor of π² ≈ 9.9. Check against the firmware which one is right.

## Algorithm (screw insertion)

```
TARE      arm still, average 100 samples -> bias (weight of tool + screw)
APPROACH  v_z = -1 mm/s until fz > 1 N for 3 samples   -> remember contact point
SEARCH    z : v_z = -Kf·(F_ref - fz), clamped, deadband   (holds ~2 N, never position)
          xy: Archimedean spiral around the contact point, r = pitch·θ/2π,
              constant tangential speed, advances only while fz ≈ F_ref
          hole  <=> fz < 0.35·F_ref  AND  z dropped > 0.8 mm below contact
                    (or: tip pushed sideways with no load below = caught on the hole wall)
          fail  <=> r > max radius, or blocked sideways with load
INSERT    xy velocity 0 (the hole guides the screw), z down slowly
          until insert_depth, or fz > insert_max_force (thread / seated)
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

Simulation results (20 N/mm, 80 ms + 30 Hz loop):

| | result | peak Fz |
|---|---|---|
| tuned, Kf = 0.15 mm/s/N | hole found, inserted | ~6 N (at the bottom of the hole) |
| naive, Kf = 2 mm/s/N | oscillation grows, abort | > 17 N |
| naive, same gain, ~10 ms latency | works | ~6 N |

The tests sweep 5/20/50 N/mm stiffness and hole offsets up to 2 mm. With
50 N/mm *and* 100 ms latency the default gain gets close to the limit and starts
failing: in that case lower `force_gain` or add compliance.

## Running

See the comment at the top of `launch/force_control.launch`.

```
roslaunch applications_helene force_control.launch mode:=admittance
roslaunch applications_helene force_control.launch mode:=insertion
```

Things to check first, in this order: `rostopic hz /force_sensor/wrench` (rate
and gaps), sign with the finger test, admittance demo (low risk), then
insertion with a low `target_force`.
