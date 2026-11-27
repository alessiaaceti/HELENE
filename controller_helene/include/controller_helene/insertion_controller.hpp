// Force-guided screw insertion state machine (ROS-independent, header only).
//
// Conventions
//  * SI units: m, s, N, m/s.
//  * Forces are expressed in the base frame with the bias removed.
//  * fz is the reaction force of the surface on the tool, positive UP
//    (pressing on the surface gives fz > 0).
//  * The output is a Cartesian velocity in the base frame [vx, vy, vz].
//
// Why the force "explodes": the force loop v_z = Kf (F_ref - F) closed around a
// stiff contact (k_env) with a dead time T is unstable when Kf * k_env * T > pi/2.
// Use max_stable_force_gain() to choose the gain.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace controller_helene
{

using Vec3 = std::array<double, 3>;

inline double clampSym(double x, double limit) { return std::max(-limit, std::min(limit, x)); }

inline double deadband(double x, double band)
{
  if (std::fabs(x) <= band) return 0.0;
  return x - std::copysign(band, x);
}

inline double norm3(const Vec3 & v) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

// Largest gain Kf [m/s/N] of a force loop on a stiff contact. `margin` is the accepted
// Kf*k_env*delay product (0.3 ~ 70 deg phase margin, pi/2 = stability limit).
inline double maxStableForceGain(double k_env, double delay, double margin = 0.3)
{
  if (k_env <= 0.0 || delay <= 0.0) throw std::invalid_argument("k_env and delay must be positive");
  return margin / (k_env * delay);
}

// First-order low-pass on a 3-vector.
class LowPass
{
public:
  explicit LowPass(double cutoff_hz = 15.0) : cutoff_hz_(cutoff_hz) {}
  void reset() { init_ = false; }
  Vec3 operator()(const Vec3 & x, double dt)
  {
    if (!init_) { y_ = x; init_ = true; return y_; }
    const double tau = 1.0 / (2.0 * M_PI * cutoff_hz_);
    const double a = dt / (tau + dt);
    for (int i = 0; i < 3; ++i) y_[i] += a * (x[i] - y_[i]);
    return y_;
  }
private:
  double cutoff_hz_;
  bool init_{false};
  Vec3 y_{0, 0, 0};
};

// Averages the first n samples (robot still, no contact) and subtracts the bias afterwards.
class Tare
{
public:
  explicit Tare(int n_samples = 100) : n_(n_samples) {}
  bool done() const { return done_; }
  void add(const Vec3 & f)
  {
    for (int i = 0; i < 3; ++i) acc_[i] += f[i];
    if (++count_ >= n_) {
      for (int i = 0; i < 3; ++i) bias_[i] = acc_[i] / count_;
      done_ = true;
    }
  }
  Vec3 apply(const Vec3 & f) const { return {f[0] - bias_[0], f[1] - bias_[1], f[2] - bias_[2]}; }
  int count() const { return count_; }
private:
  int n_;
  int count_{0};
  bool done_{false};
  Vec3 acc_{0, 0, 0};
  Vec3 bias_{0, 0, 0};
};

enum class State { TARE, APPROACH, SEARCH, INSERT, DONE, FAILED };

inline const char * stateName(State s)
{
  switch (s) {
    case State::TARE: return "TARE";
    case State::APPROACH: return "APPROACH";
    case State::SEARCH: return "SEARCH";
    case State::INSERT: return "INSERT";
    case State::DONE: return "DONE";
    case State::FAILED: return "FAILED";
  }
  return "?";
}

struct InsertionParams
{
  // sensing
  int tare_samples{100};
  double filter_cutoff_hz{15.0};
  // approach
  double approach_speed{0.001};            // m/s. Overshoot force ~ k_env*v*delay
  std::optional<double> expected_surface_z;  // nominal surface height (world z)
  double contact_force{1.0};               // N on fz to declare contact
  int contact_samples{3};
  // force regulation while searching
  double target_force{2.0};                // N pressing force during the spiral
  double force_gain{0.00005};              // m/s per N (see maxStableForceGain)
  double force_deadband{0.15};             // N
  double max_z_speed{0.002};               // m/s
  // spiral
  double spiral_pitch{0.0006};             // m between turns
  double spiral_speed{0.002};              // m/s tangential
  double spiral_min_radius{0.0003};
  double spiral_max_radius{0.004};         // give up beyond this
  double xy_gain{4.0};                     // 1/s
  // hole detection
  double hole_force_ratio{0.6};
  double hole_depth{0.0008};
  int hole_samples{3};
  double lost_force_ratio{0.35};
  // insertion
  double insert_speed{0.001};
  double insert_force{3.0};
  double insert_xy_gain{0.0002};           // m/s per N
  double insert_depth{0.004};
  double min_insert_depth{0.001};
  double insert_max_force{6.0};
  double stall_time{1.0};
  double insert_timeout{15.0};
  // safety
  double abort_force{15.0};
  double max_lateral_force{4.0};
  double rim_depth{0.0003};
};

struct InsertionStatus
{
  State state{State::TARE};
  std::string message;
  Vec3 contact_xyz{0, 0, 0};
  Vec3 hole_xy{0, 0, 0};
  double spiral_radius{0.0};
  double fz{0.0};
  double depth{0.0};
};

class ScrewInsertion
{
public:
  explicit ScrewInsertion(const InsertionParams & p = InsertionParams())
  : p_(p), tare_(p.tare_samples), filt_(p.filter_cutoff_hz) {}

  const InsertionStatus & status() const { return st_; }
  const InsertionParams & params() const { return p_; }

  void fail(const std::string & msg) { set(State::FAILED, msg); }

  // f_world: raw force in base frame (bias not removed), pos_world: tool tip, dt: s.
  Vec3 step(const Vec3 & f_world, const Vec3 & pos, double dt)
  {
    const Vec3 zero{0, 0, 0};
    if (st_.state == State::DONE || st_.state == State::FAILED) return zero;

    if (!tare_.done()) {
      tare_.add(f_world);
      if (tare_.done()) set(State::APPROACH, "");
      return zero;
    }

    const Vec3 f = filt_(tare_.apply(f_world), dt);
    const double fz = f[2];
    st_.fz = fz;
    if (have_contact_) st_.depth = contact_[2] - pos[2];

    if (norm3(f) > p_.abort_force) {
      set(State::FAILED, "abort: |F|=" + num(norm3(f)) + " N > " + num(p_.abort_force) + " N");
      return zero;
    }

    switch (st_.state) {
      case State::APPROACH: return approach(fz, pos);
      case State::SEARCH: return search(f, fz, pos, dt);
      case State::INSERT: return insert(f, fz, pos, dt);
      default: return zero;
    }
  }

private:
  static std::string num(double v, int prec = 1)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.*f", prec, v);
    return buf;
  }

  void set(State s, const std::string & msg)
  {
    st_.state = s;
    st_.message = msg;
    count_ = 0;
  }

  void spiralPoint(double theta, double & x, double & y, double & r) const
  {
    r = std::max(p_.spiral_min_radius, p_.spiral_pitch * theta / (2.0 * M_PI));
    x = contact_[0] + r * std::cos(theta);
    y = contact_[1] + r * std::sin(theta);
  }

  double forceZVelocity(double fz) const
  {
    if (fz < p_.lost_force_ratio * p_.target_force) {
      // contact lost: descend at the approach speed (not faster: latency * stiffness)
      return -p_.approach_speed;
    }
    const double err = deadband(p_.target_force - fz, p_.force_deadband);
    return clampSym(-p_.force_gain * err, p_.max_z_speed);
  }

  Vec3 approach(double fz, const Vec3 & pos)
  {
    if (!approach_xy_) approach_xy_ = std::array<double, 2>{pos[0], pos[1]};
    const double vx = clampSym(p_.xy_gain * ((*approach_xy_)[0] - pos[0]), p_.spiral_speed);
    const double vy = clampSym(p_.xy_gain * ((*approach_xy_)[1] - pos[1]), p_.spiral_speed);
    count_ = fz > p_.contact_force ? count_ + 1 : 0;
    if (count_ >= p_.contact_samples) {
      contact_ = pos;
      st_.contact_xyz = pos;
      theta_ = 0.0;
      have_contact_ = true;
      if (p_.expected_surface_z) {
        contact_[2] = *p_.expected_surface_z;
        if (pos[2] < *p_.expected_surface_z - p_.hole_depth) {
          st_.hole_xy = {pos[0], pos[1], 0.0};
          set(State::INSERT, "contact " + num(1e3 * (*p_.expected_surface_z - pos[2])) +
              " mm below the surface: already in the hole");
          return {0, 0, 0};
        }
      }
      set(State::SEARCH, "contact at z=" + num(pos[2], 4));
      return {0, 0, 0};
    }
    return {vx, vy, -p_.approach_speed};
  }

  Vec3 search(const Vec3 & f, double fz, const Vec3 & pos, double dt)
  {
    const Vec3 zero{0, 0, 0};
    const double f_lat = std::hypot(f[0], f[1]);
    const double depth = contact_[2] - pos[2];
    if (f_lat > p_.max_lateral_force) {
      // Pushed sideways while pressing: the tip met the edge/chamfer of the hole (or the hole
      // wall after dropping in). Stop the spiral and press down while x/y yield to the lateral
      // force: the edge centres the tip. If it was a false alarm the insertion times out
      // (insert_timeout) without ever pushing harder than insert_force.
      st_.hole_xy = {pos[0], pos[1], 0.0};
      set(State::INSERT, "tip against the hole edge (|Fxy|=" + num(f_lat) + " N, " +
          num(1e3 * depth) + " mm down)");
      return zero;
    }
    const double vz = forceZVelocity(fz);
    const bool dropped = fz < p_.hole_force_ratio * p_.target_force;
    if (dropped && !drop_xy_) drop_xy_ = std::array<double, 2>{pos[0], pos[1]};
    else if (!dropped) drop_xy_.reset();
    const bool hole = dropped && depth > p_.hole_depth;
    count_ = hole ? count_ + 1 : 0;
    if (count_ >= p_.hole_samples) {
      st_.hole_xy = {(*drop_xy_)[0], (*drop_xy_)[1], 0.0};
      set(State::INSERT, "hole at x=" + num(st_.hole_xy[0], 4) + " y=" + num(st_.hole_xy[1], 4));
      return zero;
    }

    double tx, ty, r;
    spiralPoint(theta_, tx, ty, r);
    st_.spiral_radius = r;
    if (r > p_.spiral_max_radius) {
      set(State::FAILED, "no hole found within r=" + num(1e3 * p_.spiral_max_radius) + " mm");
      return zero;
    }
    // advance only while there is contact (above the 'lost' level) and the force is not too high:
    // sliding without contact would skip over the hole, sliding with too much force makes the
    // tip stick and jump. A low (but present) force must not freeze the spiral, otherwise a tip
    // resting on a chamfer never gets to slide towards the hole.
    if (fz > p_.lost_force_ratio * p_.target_force && fz < 1.5 * p_.target_force)
      theta_ += p_.spiral_speed * dt / r;
    double nx, ny, nr;
    spiralPoint(theta_, nx, ny, nr);
    const double vx = (nx - tx) / dt + p_.xy_gain * (tx - pos[0]);
    const double vy = (ny - ty) / dt + p_.xy_gain * (ty - pos[1]);
    const double lim = 2.0 * p_.spiral_speed;
    return {clampSym(vx, lim), clampSym(vy, lim), vz};
  }

  Vec3 insert(const Vec3 & f, double fz, const Vec3 & pos, double dt)
  {
    const Vec3 zero{0, 0, 0};
    const double depth = contact_[2] - pos[2];
    t_insert_ += dt;
    if (depth >= p_.insert_depth) {
      set(State::DONE, "inserted " + num(1e3 * depth) + " mm");
      return zero;
    }
    if (fz > p_.insert_max_force && depth >= p_.min_insert_depth) {
      set(State::DONE, "force " + num(fz) + " N at depth " + num(1e3 * depth) + " mm (seated/thread)");
      return zero;
    }
    if (depth > stall_depth_ + 0.0001) {
      stall_depth_ = depth;
      stall_t_ = 0.0;
    } else if (fz > 0.5 * p_.insert_force) {
      stall_t_ += dt;
    }
    if (stall_t_ > p_.stall_time && depth >= p_.min_insert_depth) {
      set(State::DONE, "seated at depth " + num(1e3 * depth) + " mm (" + num(fz) + " N)");
      return zero;
    }
    if (t_insert_ > p_.insert_timeout) {
      set(State::FAILED, "insertion stuck at depth " + num(1e3 * depth) + " mm");
      return zero;
    }
    const double vz = fz < p_.contact_force ? -p_.insert_speed
                                            : clampSym(-p_.force_gain * (p_.insert_force - fz), p_.insert_speed);
    return {clampSym(p_.insert_xy_gain * f[0], p_.spiral_speed),
            clampSym(p_.insert_xy_gain * f[1], p_.spiral_speed), vz};
  }

  InsertionParams p_;
  Tare tare_;
  LowPass filt_;
  InsertionStatus st_;
  int count_{0};
  double theta_{0.0};
  Vec3 contact_{0, 0, 0};
  bool have_contact_{false};
  std::optional<std::array<double, 2>> drop_xy_;
  std::optional<std::array<double, 2>> approach_xy_;
  double t_insert_{0.0};
  double stall_t_{0.0};
  double stall_depth_{-1.0};
};

}  // namespace controller_helene
