#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include <bspline_opt/uniform_bspline.h>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/msg/bspline.hpp>

namespace ego_planner
{
struct ScheduledTrajectory
{
  int64_t id = 0;
  rclcpp::Time start{0, 0, RCL_ROS_TIME};
  double duration = 0.0;
  std::array<UniformBspline, 3> curves;

  std::array<Eigen::Vector3d, 3> sample(double seconds)
  {
    const double t = std::clamp(seconds, 0.0, duration);
    std::array<Eigen::Vector3d, 3> state{
        curves[0].evaluateDeBoorT(t),
        curves[1].evaluateDeBoorT(t),
        curves[2].evaluateDeBoorT(t)};
    if (seconds >= duration)
    {
      state[1].setZero();
      state[2].setZero();
    }
    return state;
  }
};

inline bool decodeTrajectory(const traj_utils::msg::Bspline &msg,
                             ScheduledTrajectory &out, std::string &reason)
{
  // This planner emits uniform cubic knots. Reject malformed input before
  // entering Eigen/de Boor, including giant messages and invalid IDs.
  if (msg.order != 3 || msg.traj_id <= 0 ||
      msg.traj_id > std::numeric_limits<int32_t>::max() ||
      msg.pos_pts.size() < 6 || msg.pos_pts.size() > 1024 ||
      msg.knots.size() != msg.pos_pts.size() + 4)
  {
    reason = "invalid order, id, or array sizes";
    return false;
  }
  Eigen::MatrixXd points(3, msg.pos_pts.size());
  Eigen::VectorXd knots(msg.knots.size());
  for (size_t i = 0; i < msg.pos_pts.size(); ++i)
    points.col(i) << msg.pos_pts[i].x, msg.pos_pts[i].y, msg.pos_pts[i].z;
  if (!points.allFinite())
  {
    reason = "non-finite control point";
    return false;
  }
  for (size_t i = 0; i < msg.knots.size(); ++i)
  {
    knots(i) = msg.knots[i];
    if (!std::isfinite(knots(i)) ||
        (i && knots(i) - knots(i - 1) <= 1e-8))
    {
      reason = "non-finite or non-increasing knots";
      return false;
    }
  }
  const double duration = knots(points.cols()) - knots(3);
  if (duration <= 0.0 || duration > 600.0 || msg.start_time.sec < 0 ||
      msg.start_time.nanosec >= 1000000000U)
  {
    reason = "invalid duration or timestamp";
    return false;
  }
  out.id = msg.traj_id;
  out.start = rclcpp::Time(msg.start_time, RCL_ROS_TIME);
  out.duration = duration;
  out.curves[0] = UniformBspline(points, 3, 0.1);
  out.curves[0].setKnot(knots);
  out.curves[1] = out.curves[0].getDerivative();
  out.curves[2] = out.curves[1].getDerivative();
  for (double t : {0.0, duration * 0.5, duration})
    for (const auto &value : out.sample(t))
      if (!value.allFinite())
      {
        reason = "non-finite evaluated state";
        return false;
      }
  return true;
}

inline bool continuousAtSwitch(ScheduledTrajectory &old_traj,
                               ScheduledTrajectory &new_traj,
                               double position_tolerance = 0.03,
                               double velocity_tolerance = 0.05,
                               double acceleration_tolerance = 0.20)
{
  const double t = (new_traj.start - old_traj.start).seconds();
  if (t < 0.0)
    return false;
  const auto old_state = old_traj.sample(t);
  const auto new_state = new_traj.sample(0.0);
  return (old_state[0] - new_state[0]).norm() <= position_tolerance &&
         (old_state[1] - new_state[1]).norm() <= velocity_tolerance &&
         (old_state[2] - new_state[2]).norm() <= acceleration_tolerance;
}

// The server stages -> acknowledges -> commits -> activates. The old trajectory
// remains active throughout staging. Uncommitted/late proposals never execute.
class TrajectoryHandover
{
public:
  std::optional<ScheduledTrajectory> active;
  std::optional<ScheduledTrajectory> pending;
  bool committed = false;
  int64_t highest_id = 0;

  bool stage(ScheduledTrajectory next, const rclcpp::Time &now,
             std::string &reason)
  {
    if (pending && next.id == pending->id)
    {
      reason = "duplicate pending id";
      return false;
    }
    if (pending || next.id <= highest_id || next.start <= now)
    {
      reason = "pending occupied, stale id, or expired start";
      return false;
    }
    if (active && !continuousAtSwitch(*active, next))
    {
      reason = "discontinuous position/velocity/acceleration boundary";
      return false;
    }
    highest_id = next.id;
    pending = std::move(next);
    committed = false;
    return true;
  }

  bool commit(int64_t id, const rclcpp::Time &now)
  {
    if (!pending || pending->id != id || pending->start <= now)
      return false;
    committed = true;
    return true;
  }

  bool cancel(int64_t id)
  {
    if (!pending || pending->id != id)
      return false;
    pending.reset();
    committed = false;
    return true;
  }

  // Positive ID: activated. Negative ID: expired without commit. Zero: no change.
  int64_t advance(const rclcpp::Time &now)
  {
    if (!pending || now < pending->start)
      return 0;
    const int64_t id = pending->id;
    if (committed)
      active = std::move(pending);
    pending.reset();
    const bool activated = committed;
    committed = false;
    return activated ? id : -id;
  }

  bool emergency(ScheduledTrajectory stop)
  {
    if (stop.id <= highest_id)
      return false;
    // Only a constant trajectory may bypass the normal continuity gate.
    const auto points = stop.curves[0].getControlPoint();
    for (Eigen::Index i = 1; i < points.cols(); ++i)
      if ((points.col(i) - points.col(0)).norm() > 1e-6)
        return false;
    highest_id = stop.id;
    pending.reset();
    committed = false;
    active = std::move(stop);
    return true;
  }
};
}  // namespace ego_planner
