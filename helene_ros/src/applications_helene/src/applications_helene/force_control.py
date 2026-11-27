"""Force-guided motion for HELENE: hand guiding (admittance) and screw insertion.

This module is plain Python (no ROS imports) so it can be unit-tested and
simulated offline. The ROS nodes in ``scripts/`` only feed it measurements and
forward the velocity it returns to MoveIt Servo.

Conventions (read this before tuning!)
--------------------------------------
* Everything is SI: metres, seconds, newtons, m/s.
* Forces are expressed in the *world/base* frame, already rotated from the
  sensor frame and with the bias (tare) removed.
* ``f[2]`` (``fz``) is the reaction force the surface applies on the tool,
  positive UP. When the screw presses on the surface, ``fz > 0``.
  Check it by pushing the screw tip upwards with a finger: ``fz`` must go
  positive. If it goes negative, flip ``force_sign`` in the node.
* Commands are Cartesian velocities in the world frame (what MoveIt Servo
  expects with ``command_in_type: speed_units``).

Why the force "explodes"
------------------------
The arm is driven by steppers through a velocity/position interface: it is a
very stiff *position source*. The screw/surface contact is also very stiff
(k_env ~ 10-100 N/mm). A force loop of the form ``v_z = Kf * (F_ref - F)``
closed around that contact is an integrator with loop gain ``Kf * k_env``
[1/s] and a dead time ``T`` (CAN -> ESP -> micro-ROS -> ROS -> controller ->
rosserial -> motors, plus the 30 Hz hardware loop, plus filter lag).
That loop is unstable as soon as ``Kf * k_env * T > pi/2``: every period the
correction overshoots more than the previous one and the force oscillates with
growing amplitude until it saturates or something gives way. See
``max_stable_force_gain``. Position-commanding the tool *into* the surface is
the limit case Kf -> infinity: any millimetre of error becomes tens of newtons.
"""

import math
from dataclasses import dataclass, field
from typing import Optional


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------

def clamp(x, limit):
    return max(-limit, min(limit, x))


def deadband(x, band):
    if abs(x) <= band:
        return 0.0
    return x - math.copysign(band, x)


def norm(v):
    return math.sqrt(sum(c * c for c in v))


def max_stable_force_gain(k_env, delay, margin=0.3):
    """Largest admittance gain Kf [m/s/N] for a force loop on a stiff contact.

    The loop ``v = Kf*(F_ref - F)``, ``F = k_env * penetration`` with a total
    dead time ``delay`` is ``L(s) = Kf*k_env*exp(-s*delay)/s``. It goes
    unstable at ``Kf*k_env*delay = pi/2``. ``margin`` is the loop gain*delay
    product you accept; 0.3 gives ~70 deg phase margin (no visible
    oscillation), pi/2 is the stability limit.

    Example: k_env = 20 N/mm, delay = 100 ms -> Kf <= 0.15 mm/s per N.
    """
    if k_env <= 0 or delay <= 0:
        raise ValueError("k_env and delay must be positive")
    return margin / (k_env * delay)


class LowPass:
    """First-order low-pass filter on a vector. Its lag adds ~1/(2*pi*fc)
    of delay to the force loop, so do not set the cutoff too low."""

    def __init__(self, cutoff_hz, size=3):
        self.cutoff_hz = cutoff_hz
        self.y = None
        self.size = size

    def reset(self):
        self.y = None

    def __call__(self, x, dt):
        if self.y is None:
            self.y = list(x)
            return list(self.y)
        tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        a = dt / (tau + dt)
        self.y = [yi + a * (xi - yi) for yi, xi in zip(self.y, x)]
        return list(self.y)


class Tare:
    """Averages the first ``n_samples`` readings (robot still, no contact)
    and subtracts that bias afterwards. Re-tare whenever the tool
    orientation changes, because the weight of gripper+screw moves between
    axes when the sensor rotates."""

    def __init__(self, n_samples=100):
        self.n_samples = n_samples
        self._acc = [0.0, 0.0, 0.0]
        self._n = 0
        self.bias = None

    def reset(self):
        self.__init__(self.n_samples)

    @property
    def done(self):
        return self.bias is not None

    def add(self, f):
        self._acc = [a + fi for a, fi in zip(self._acc, f)]
        self._n += 1
        if self._n >= self.n_samples:
            self.bias = [a / self._n for a in self._acc]

    def apply(self, f):
        return [fi - bi for fi, bi in zip(f, self.bias)]


# ---------------------------------------------------------------------------
# Stage 1: hand guiding
# ---------------------------------------------------------------------------

@dataclass
class AdmittanceParams:
    damping: float = 400.0        # N/(m/s): 10 N push -> 25 mm/s
    deadband: float = 0.8         # N, ignores noise and residual bias
    max_speed: float = 0.03       # m/s
    filter_cutoff_hz: float = 8.0
    tare_samples: int = 100


class Admittance:
    """Hand guiding: velocity = force / damping. The robot holds still (zero
    velocity) when nobody touches the sensor and follows the push otherwise.

    Pure damping (no virtual spring, no virtual mass) is the most robust
    choice on a stepper arm: there is no integrator that can wind up and the
    arm stops as soon as you let go.
    """

    def __init__(self, params=None):
        self.p = params or AdmittanceParams()
        self.tare = Tare(self.p.tare_samples)
        self.filt = LowPass(self.p.filter_cutoff_hz)

    def step(self, f_raw, dt):
        if not self.tare.done:
            self.tare.add(f_raw)
            return [0.0, 0.0, 0.0]
        f = self.filt(self.tare.apply(f_raw), dt)
        mag = norm(f)
        if mag <= self.p.deadband:
            return [0.0, 0.0, 0.0]
        # radial deadband: keep direction, remove the band from the magnitude
        scale = (mag - self.p.deadband) / mag
        v = [scale * fi / self.p.damping for fi in f]
        speed = norm(v)
        if speed > self.p.max_speed:
            v = [vi * self.p.max_speed / speed for vi in v]
        return v


# ---------------------------------------------------------------------------
# Stage 2: screw insertion
# ---------------------------------------------------------------------------

TARE, APPROACH, SEARCH, INSERT, DONE, FAILED = (
    "TARE", "APPROACH", "SEARCH", "INSERT", "DONE", "FAILED")


@dataclass
class InsertionParams:
    # --- sensing ---
    tare_samples: int = 100
    filter_cutoff_hz: float = 15.0
    # --- approach ---
    approach_speed: float = 0.001     # m/s. Overshoot force ~ k_env*v*delay
    # Optional: nominal surface height (world z). If contact happens more
    # than hole_depth below it, the screw already fell into the hole while
    # approaching -> skip the spiral and insert directly.
    expected_surface_z: Optional[float] = None
    contact_force: float = 1.0        # N on fz to declare contact
    contact_samples: int = 3          # consecutive samples above threshold
    # --- force regulation while searching ---
    target_force: float = 2.0         # N pressing force during the spiral
    force_gain: float = 0.00005       # m/s per N. Use max_stable_force_gain()
    force_deadband: float = 0.15      # N
    max_z_speed: float = 0.002        # m/s, also the speed when contact is lost
    # --- spiral ---
    spiral_pitch: float = 0.0006      # m between turns (< hole clearance/chamfer)
    spiral_speed: float = 0.002       # m/s tangential speed
    spiral_min_radius: float = 0.0003 # avoids infinite angular speed at r=0
    spiral_max_radius: float = 0.004  # m, give up beyond this
    xy_gain: float = 4.0              # 1/s, P term tracking the spiral
    # --- hole detection ---
    hole_force_ratio: float = 0.6     # fz below this * target_force ...
    hole_depth: float = 0.0008        # ... AND tool this far below contact height
    hole_samples: int = 3
    lost_force_ratio: float = 0.35    # fz below this * target_force: contact lost,
                                      # descend at max_z_speed instead of the slow gain
    # --- insertion ---
    insert_speed: float = 0.001       # m/s down while there is no load
    insert_force: float = 3.0         # N, push force while inserting
    insert_xy_gain: float = 0.0002    # m/s per N: x/y yield to the hole walls
    insert_depth: float = 0.004       # m below contact height, then DONE
    min_insert_depth: float = 0.001   # seated/stalled above this depth -> DONE
    insert_max_force: float = 6.0     # N, at/after min_insert_depth -> DONE (bottom/thread)
    stall_time: float = 1.0           # s without progress at insert_force -> seated
    insert_timeout: float = 15.0      # s
    # --- safety ---
    abort_force: float = 15.0         # N on |F|, anywhere -> FAILED, stop
    max_lateral_force: float = 4.0    # N on |Fxy| while searching: blocked
    rim_depth: float = 0.0003         # blocked AND this far down -> tip is on the hole rim


@dataclass
class InsertionStatus:
    state: str = TARE
    message: str = ""
    contact_xyz: list = field(default_factory=list)
    hole_xy: list = field(default_factory=list)
    spiral_radius: float = 0.0
    fz: float = 0.0


class ScrewInsertion:
    """State machine: TARE -> APPROACH -> SEARCH -> INSERT -> DONE (or FAILED).

    APPROACH: move down slowly until fz > contact_force (debounced).
    SEARCH:   z is force-controlled (admittance, low gain, clamped) to hold
              ``target_force`` while x/y follow an Archimedean spiral around
              the contact point. The z position is NEVER commanded directly.
    Hole:     the screw tip drops into the hole -> fz collapses AND the tool
              moves down by more than ``hole_depth``. Requiring both rejects
              noise spikes (force only) and surface slope (depth only).
    INSERT:   stop the spiral, push down with insert_force while x/y yield to
              the lateral force (the hole edge centres the screw), until the
              depth is reached or the force rises again (thread engaged).

    Call ``step(f_world, pos_world, dt)`` at a fixed rate; it returns the
    Cartesian velocity command [vx, vy, vz] in the world frame.
    """

    def __init__(self, params=None):
        self.p = params or InsertionParams()
        self.tare = Tare(self.p.tare_samples)
        self.filt = LowPass(self.p.filter_cutoff_hz)
        self.status = InsertionStatus()
        self._count = 0
        self._theta = 0.0
        self._contact = None
        self._drop_xy = None
        self._approach_xy = None
        self._t_insert = 0.0
        self._stall_t = 0.0
        self._stall_depth = -1.0

    # -- helpers ----------------------------------------------------------
    def _set(self, state, msg=""):
        self.status.state = state
        self.status.message = msg
        self._count = 0

    def _spiral_point(self, theta):
        p = self.p
        r = max(p.spiral_min_radius, p.spiral_pitch * theta / (2.0 * math.pi))
        cx, cy = self._contact[0], self._contact[1]
        return [cx + r * math.cos(theta), cy + r * math.sin(theta)], r

    def _force_z_velocity(self, fz):
        """v_z from the force error. Positive error (pressing too little)
        -> move down (negative v_z)."""
        p = self.p
        if fz < p.lost_force_ratio * p.target_force:
            # Contact lost (hole, or edge of the part): the low gain would
            # follow the surface down far too slowly, so descend at the
            # approach speed. Not faster: with ~0.1 s of latency the arm
            # keeps going after contact is back, and every mm/s becomes
            # k_env * v * T newtons of overshoot.
            return -p.approach_speed
        err = deadband(p.target_force - fz, p.force_deadband)
        return clamp(-p.force_gain * err, p.max_z_speed)

    def fail(self, msg):
        """External abort (stale sensor, tf error...)."""
        self._set(FAILED, msg)

    def _insert(self, f, fz, pos_world, dt):
        """Push down with a limited force while x/y yield to the lateral
        force (admittance): the hole edge pushes the tip towards the hole
        centre, so the screw centres itself instead of being pressed
        against the wall."""
        p, st = self.p, self.status
        depth = self._contact[2] - pos_world[2]
        self._t_insert += dt
        if depth >= p.insert_depth:
            self._set(DONE, "inserted %.1f mm" % (1e3 * depth))
            return [0.0, 0.0, 0.0]
        if fz > p.insert_max_force and depth >= p.min_insert_depth:
            self._set(DONE, "force %.1f N at depth %.1f mm (seated/thread)" % (fz, 1e3 * depth))
            return [0.0, 0.0, 0.0]
        # stall: no progress while pushing with the insertion force
        if depth > self._stall_depth + 0.0001:
            self._stall_depth, self._stall_t = depth, 0.0
        elif fz > 0.5 * p.insert_force:
            self._stall_t += dt
        if self._stall_t > p.stall_time and depth >= p.min_insert_depth:
            self._set(DONE, "seated at depth %.1f mm (%.1f N)" % (1e3 * depth, fz))
            return [0.0, 0.0, 0.0]
        if self._t_insert > p.insert_timeout:
            self._set(FAILED, "insertion stuck at depth %.1f mm" % (1e3 * depth))
            return [0.0, 0.0, 0.0]
        if fz < p.contact_force:
            vz = -p.insert_speed
        else:
            vz = clamp(-p.force_gain * (p.insert_force - fz), p.insert_speed)
        vx = clamp(p.insert_xy_gain * f[0], p.spiral_speed)
        vy = clamp(p.insert_xy_gain * f[1], p.spiral_speed)
        return [vx, vy, vz]

    # -- main step --------------------------------------------------------
    def step(self, f_world, pos_world, dt):
        p, st = self.p, self.status
        if st.state in (DONE, FAILED):
            return [0.0, 0.0, 0.0]

        if not self.tare.done:
            self.tare.add(f_world)
            if self.tare.done:
                self._set(APPROACH)
            return [0.0, 0.0, 0.0]

        f = self.filt(self.tare.apply(f_world), dt)
        fz = f[2]
        st.fz = fz

        if norm(f) > p.abort_force:
            self._set(FAILED, "abort: |F|=%.1f N > %.1f N" % (norm(f), p.abort_force))
            return [0.0, 0.0, 0.0]

        if st.state == APPROACH:
            # hold x/y where the approach started: otherwise tracking errors
            # of the arm add to the misalignment the spiral has to cover
            if self._approach_xy is None:
                self._approach_xy = [pos_world[0], pos_world[1]]
            vx = clamp(p.xy_gain * (self._approach_xy[0] - pos_world[0]), p.spiral_speed)
            vy = clamp(p.xy_gain * (self._approach_xy[1] - pos_world[1]), p.spiral_speed)
            self._count = self._count + 1 if fz > p.contact_force else 0
            if self._count >= p.contact_samples:
                self._contact = list(pos_world)
                st.contact_xyz = list(pos_world)
                self._theta = 0.0
                ez = p.expected_surface_z
                if ez is not None:
                    # depths are measured from the known surface, not from
                    # where contact happened (the tip may touch the hole rim)
                    self._contact[2] = ez
                if ez is not None and pos_world[2] < ez - p.hole_depth:
                    st.hole_xy = [pos_world[0], pos_world[1]]
                    self._set(INSERT, "contact %.1f mm below the surface: "
                              "already in the hole" % (1e3 * (ez - pos_world[2])))
                    return [0.0, 0.0, 0.0]
                self._set(SEARCH, "contact at z=%.4f" % pos_world[2])
                return [0.0, 0.0, 0.0]
            return [vx, vy, -p.approach_speed]

        if st.state == SEARCH:
            f_lat = math.hypot(f[0], f[1])
            depth = self._contact[2] - pos_world[2]
            if f_lat > p.max_lateral_force and (
                    fz < p.hole_force_ratio * p.target_force or depth > p.rim_depth):
                # Pushed sideways with no load from below (tip dropped into the
                # hole, spiral drags it against the wall), or sideways while
                # already lower than the surface (tip sitting on the rim).
                st.hole_xy = [pos_world[0], pos_world[1]]
                self._set(INSERT, "hole (tip against the hole edge, |Fxy|=%.1f N, "
                          "%.1f mm down)" % (f_lat, 1e3 * depth))
                return [0.0, 0.0, 0.0]
            if f_lat > p.max_lateral_force:
                self._set(FAILED, "blocked sideways: |Fxy|=%.1f N (stuck on an edge, "
                          "or already in the hole: set expected_surface_z)"
                          % f_lat)
                return [0.0, 0.0, 0.0]
            vz = self._force_z_velocity(fz)
            dropped = fz < p.hole_force_ratio * p.target_force
            if dropped and self._drop_xy is None:
                self._drop_xy = [pos_world[0], pos_world[1]]   # where it fell
            elif not dropped:
                self._drop_xy = None
            hole = dropped and depth > p.hole_depth
            self._count = self._count + 1 if hole else 0
            if self._count >= p.hole_samples:
                st.hole_xy = list(self._drop_xy)
                self._set(INSERT, "hole at x=%.4f y=%.4f" % tuple(st.hole_xy))
                return [0.0, 0.0, 0.0]

            # Advance along the spiral at constant tangential speed, but only
            # while the pressing force is roughly right: sliding with no
            # contact would skip over the hole, sliding with too much force
            # makes the screw stick and jump.
            target, r = self._spiral_point(self._theta)
            st.spiral_radius = r
            if r > p.spiral_max_radius:
                self._set(FAILED, "no hole found within r=%.1f mm" % (1e3 * p.spiral_max_radius))
                return [0.0, 0.0, 0.0]
            if abs(fz - p.target_force) < 0.5 * p.target_force:
                self._theta += p.spiral_speed * dt / r
            nxt, _ = self._spiral_point(self._theta)
            vx = (nxt[0] - target[0]) / dt + p.xy_gain * (target[0] - pos_world[0])
            vy = (nxt[1] - target[1]) / dt + p.xy_gain * (target[1] - pos_world[1])
            lim = 2.0 * p.spiral_speed
            return [clamp(vx, lim), clamp(vy, lim), vz]

        if st.state == INSERT:
            return self._insert(f, fz, pos_world, dt)

        return [0.0, 0.0, 0.0]
