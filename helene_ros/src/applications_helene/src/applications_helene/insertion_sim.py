"""Offline simulation of the screw insertion loop (no ROS needed).

Models what matters for stability on HELENE:
  * a stiff surface (k_env N/m) with a hole of radius ``hole_radius``,
  * dead time between the force reading and the arm actually moving
    (CAN + micro-ROS + ROS + rosserial), and the 30 Hz hardware loop,
  * sensor bias (weight of gripper+screw) and noise.

Run it to compare a stable gain with a "naive" one:

    python3 -m applications_helene.insertion_sim
"""

import math
import random
from collections import deque

from applications_helene.force_control import (
    InsertionParams, ScrewInsertion, DONE, FAILED)


class StiffSurfaceWithHole:
    def __init__(self, k_env=20000.0, hole_xy=(0.0015, -0.001),
                 hole_radius=0.0004, hole_floor=-0.002, friction=0.2):
        self.k = k_env
        self.hole = hole_xy
        self.r = hole_radius
        self.floor = hole_floor      # where the thread starts / screw stops
        self.mu = friction

    def surface_height(self, x, y):
        d = math.hypot(x - self.hole[0], y - self.hole[1])
        return self.floor if d < self.r else 0.0

    def force(self, pos, vel):
        """Reaction force on the tool, world frame, +z up.

        Inside the hole the walls push the tip back (lateral force), so the
        tool cannot slide out sideways once it has dropped in."""
        x, y, z = pos
        dx, dy = x - self.hole[0], y - self.hole[1]
        d = math.hypot(dx, dy)
        f = [0.0, 0.0, 0.0]
        if d < self.r:                       # above / inside the hole
            pen = self.floor - z
            if pen > 0:
                f[2] = self.k * pen
        elif z < 0.0:                        # touching top surface or wall
            wall_pen = d - self.r
            if -z <= wall_pen:
                f[2] = self.k * (-z)         # pressing on the top surface
            else:                            # pushed into the hole wall
                f[0] = -self.k * wall_pen * dx / d
                f[1] = -self.k * wall_pen * dy / d
                if self.floor - z > 0:
                    f[2] = self.k * (self.floor - z)
                return f
        vxy = math.hypot(vel[0], vel[1])
        if f[2] > 0 and vxy > 1e-6:          # sliding friction
            f[0] -= self.mu * f[2] * vel[0] / vxy
            f[1] -= self.mu * f[2] * vel[1] / vxy
        return f


def simulate(params=None, env=None, delay=0.08, dt=0.01, hw_rate=30.0,
             noise=0.08, bias=(0.3, -0.2, -2.5), start_z=0.003,
             t_max=60.0, seed=1):
    """Returns (controller, trace). trace rows: (t, state, x, y, z, fz_true)."""
    rng = random.Random(seed)
    env = env or StiffSurfaceWithHole()
    ctrl = ScrewInsertion(params or InsertionParams())
    pos = [0.0, 0.0, start_z]
    applied_v = [0.0, 0.0, 0.0]
    n_delay = max(1, int(round(delay / dt)))
    cmd_pipe = deque([[0.0, 0.0, 0.0]] * n_delay)
    hw_every = max(1, int(round(1.0 / (hw_rate * dt))))
    trace = []
    t = 0.0
    k = 0
    while t < t_max and ctrl.status.state not in (DONE, FAILED):
        f_true = env.force(pos, applied_v)
        f_meas = [f + b + rng.gauss(0.0, noise) for f, b in zip(f_true, bias)]
        v_cmd = ctrl.step(f_meas, list(pos), dt)
        cmd_pipe.append(v_cmd)
        delayed = cmd_pipe.popleft()
        if k % hw_every == 0:          # 30 Hz zero-order hold of the HW loop
            applied_v = delayed
        pos = [p + v * dt for p, v in zip(pos, applied_v)]
        trace.append((t, ctrl.status.state, pos[0], pos[1], pos[2], f_true[2]))
        t += dt
        k += 1
    return ctrl, trace


def peak_force(trace, state=None):
    rows = [r for r in trace if state is None or r[1] == state]
    return max((r[5] for r in rows), default=0.0)


def _report(name, ctrl, trace):
    st = ctrl.status
    print("%-28s -> %-6s %-45s peak Fz %5.1f N  (in SEARCH %5.1f N)  t=%.1fs" % (
        name, st.state, st.message, peak_force(trace),
        peak_force(trace, "SEARCH"), trace[-1][0]))


if __name__ == "__main__":
    good = InsertionParams()
    _report("tuned (Kf=0.15 mm/s/N)", *simulate(good))
    naive = InsertionParams(force_gain=0.002, max_z_speed=0.02)
    _report("naive (Kf=2 mm/s/N)", *simulate(naive))
    no_delay = InsertionParams(force_gain=0.002, max_z_speed=0.02)
    _report("naive, zero latency", *simulate(no_delay, delay=0.01, hw_rate=100.0))
