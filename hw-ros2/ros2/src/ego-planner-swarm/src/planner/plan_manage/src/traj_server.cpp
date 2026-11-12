#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/msg/odometry.hpp"
#include "traj_utils/msg/bspline.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>//加
#include <cmath>//加
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>//加
#include <std_msgs/msg/bool.hpp>//加
#include <ego_planner/trajectory_handover.h>

rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;

quadrotor_msgs::msg::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_ = 0;
rclcpp::Clock::SharedPtr node_clock_;
ego_planner::TrajectoryHandover handover_;
rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr ack_pub_, committed_pub_;

void publishId(const rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr &pub, int64_t id)
{
  std_msgs::msg::Int64 msg;
  msg.data = id;
  pub->publish(msg);
}

void loadActiveTrajectory()
{
  const auto &active = *handover_.active;
  traj_.assign(active.curves.begin(), active.curves.end());
  start_time_ = active.start;
  traj_id_ = static_cast<int>(active.id);
  traj_duration_ = active.duration;
  receive_traj_ = true;
}

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

//加
bool have_odom_yaw_ = false;
double odom_yaw_ = 0.0;
std::chrono::steady_clock::time_point last_odom_receive_;

Eigen::Vector3d actual_position_ = Eigen::Vector3d::Zero();
Eigen::Vector3d actual_velocity_ = Eigen::Vector3d::Zero();
int64_t recovery_epoch_ = 0, recovery_floor_ = 0;
int64_t recovery_anchor_id_ = 0, recovery_accepted_id_ = 0;
bool recovery_running_ = false, hard_hold_ = false;
std::chrono::steady_clock::time_point stopped_receive_{};
rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr offer_pub_, anchor_ack_pub_;

void publishRecovery(const rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr &pub,
                     int64_t id)
{
  std_msgs::msg::Int64MultiArray m;
  m.data = {recovery_epoch_, id};
  pub->publish(m);
}

bool stoppedFresh()
{
  const auto now = std::chrono::steady_clock::now();
  return have_odom_yaw_ && actual_position_.allFinite() && actual_velocity_.allFinite() &&
      now - last_odom_receive_ <= std::chrono::milliseconds(200) &&
      stopped_receive_ != std::chrono::steady_clock::time_point{} &&
      now - stopped_receive_ <= std::chrono::milliseconds(200) &&
      actual_velocity_.norm() < 0.10;
}

void recoveryRequest(const std_msgs::msg::Int64MultiArray &m)
{
  if (hard_hold_ || m.data.size() != 2 || m.data[0] <= recovery_epoch_ ||
      m.data[0] <= 0 || m.data[1] < 0 || m.data[1] >= std::numeric_limits<int32_t>::max() - 1)
    return;
  recovery_epoch_ = m.data[0];
  recovery_floor_ = m.data[1];
  recovery_running_ = true;
  recovery_anchor_id_ = recovery_accepted_id_ = 0;
  stopped_receive_ = std::chrono::steady_clock::time_point{};
  handover_.pending.reset();
  handover_.active.reset();
  handover_.committed = false;
  handover_.highest_id = std::max(handover_.highest_id, recovery_floor_);
  receive_traj_ = false; // bridge owns braking; stop sampling the old route
  RCLCPP_WARN(rclcpp::get_logger("traj_server"), "[RECOVERY] epoch=%ld: old route suspended",
              static_cast<long>(recovery_epoch_));
}
//加

void odomYawCallback(nav_msgs::msg::Odometry::ConstPtr msg)
{
  if (msg->header.frame_id != "world")
    return;

  const auto &q = msg->pose.pose.orientation;
  const double n2 =
      q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;

  if (!std::isfinite(n2) || n2 < 1e-12)
    return;

  const double inv = 1.0 / std::sqrt(n2);
  const double x = q.x * inv;
  const double y = q.y * inv;
  const double z = q.z * inv;
  const double w = q.w * inv;

  // /grid_map/odom_enu 中的 ENU 航向
  odom_yaw_ = std::atan2(
      2.0 * (w * z + x * y),
      1.0 - 2.0 * (y * y + z * z));

  const auto &p = msg->pose.pose.position;
  const auto &v = msg->twist.twist.linear;
  actual_position_ = Eigen::Vector3d(p.x, p.y, p.z);
  actual_velocity_ = Eigen::Vector3d(v.x, v.y, v.z);
  if (!actual_position_.allFinite() || !actual_velocity_.allFinite()) return;
  last_odom_receive_ = std::chrono::steady_clock::now();
  have_odom_yaw_ = true;
}
//加

void bsplineCallback(traj_utils::msg::Bspline::ConstPtr msg)
{
  ego_planner::ScheduledTrajectory next;
  std::string reason;
  bool valid = ego_planner::decodeTrajectory(*msg, next, reason);
  if (valid && !handover_.active && (!have_odom_yaw_ ||
      std::chrono::steady_clock::now() - last_odom_receive_ > std::chrono::milliseconds(500)))
  {
    valid = false;
    reason = "waiting for fresh odometry before first trajectory";
  }
  //加
  if (valid && (hard_hold_ ||
      (recovery_running_ && (recovery_anchor_id_ == 0 || !stoppedFresh()))))
  {
    valid = false;
    reason = "waiting for fresh stopped state and recovery anchor";
  }//加
  if (valid)
    valid = handover_.stage(std::move(next), node_clock_->now(), reason);
  if (!valid)
  {
    RCLCPP_WARN(rclcpp::get_logger("traj_server"), "Rejected trajectory %ld: %s",
                static_cast<long>(msg->traj_id), reason.c_str());
    if (msg->traj_id > 0 && msg->traj_id <= std::numeric_limits<int32_t>::max())
      publishId(ack_pub_, -msg->traj_id);
    return;
  }
  if (!receive_traj_)
  {
    last_yaw_ = odom_yaw_;
    last_yaw_dot_ = 0.0;
  }
  if (recovery_running_)//加
  {
    recovery_accepted_id_ = 0;
    publishRecovery(offer_pub_, msg->traj_id);
  }
  else//加
    publishId(ack_pub_, msg->traj_id);
}

void commitCallback(std_msgs::msg::Int64::ConstSharedPtr msg)
{
  //if (handover_.commit(msg->data, node_clock_->now()))
  //  publishId(committed_pub_, msg->data);
  //else if (msg->data > 0 && msg->data <= std::numeric_limits<int32_t>::max())
  //  publishId(ack_pub_, -msg->data);
  if (!hard_hold_ && (!recovery_running_ ||
      (recovery_accepted_id_ == msg->data && stoppedFresh())) &&
      handover_.commit(msg->data, node_clock_->now()))
    publishId(committed_pub_, msg->data);
  else if (msg->data > 0 && msg->data <= std::numeric_limits<int32_t>::max())
    publishId(ack_pub_, -msg->data);//改
}

void cancelCallback(std_msgs::msg::Int64::ConstSharedPtr msg)
{
  if (handover_.cancel(msg->data))
    publishId(ack_pub_, -msg->data);
}

void emergencyCallback(traj_utils::msg::Bspline::ConstSharedPtr msg)
{
  //ego_planner::ScheduledTrajectory stop;
  //std::string reason;
  //if (!ego_planner::decodeTrajectory(*msg, stop, reason) ||
  //    std::abs((node_clock_->now() - stop.start).seconds()) > 0.5 ||
  //    !handover_.emergency(std::move(stop)))
  //{
  //  RCLCPP_ERROR(rclcpp::get_logger("traj_server"), "Rejected malformed/stale emergency trajectory");
  //  return;
  //}
  //loadActiveTrajectory();
  if (hard_hold_ || (recovery_running_ && !stoppedFresh())) return;
  ego_planner::ScheduledTrajectory stop;
  std::string reason;
  if (!ego_planner::decodeTrajectory(*msg, stop, reason) ||
      std::abs((node_clock_->now() - stop.start).seconds()) > 0.5 ||
      (recovery_running_ && (stop.id <= recovery_floor_ ||
       (stop.sample(0.0)[0] - actual_position_).norm() > 0.05)) ||
      !handover_.emergency(std::move(stop)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("traj_server"), "Rejected malformed/stale emergency trajectory");
    return;
  }
  loadActiveTrajectory();
  if (recovery_running_)
  {
    recovery_anchor_id_ = traj_id_;
    recovery_accepted_id_ = 0;
    last_yaw_ = odom_yaw_;
    last_yaw_dot_ = 0.0;
    publishRecovery(anchor_ack_pub_, recovery_anchor_id_);
    RCLCPP_INFO(rclcpp::get_logger("traj_server"), "[RECOVERY] anchor installed, id=%d", traj_id_);
  }
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, rclcpp::Time &time_now, rclcpp::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).seconds();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback()
{
  //加
  if (hard_hold_) return;
  if (recovery_running_)
  {
    if (recovery_anchor_id_ > 0) publishRecovery(anchor_ack_pub_, recovery_anchor_id_);
    if (handover_.pending && !stoppedFresh())
    {
      const auto id = handover_.pending->id;
      handover_.cancel(id);
      publishId(ack_pub_, -id);
    }
    if (handover_.pending) publishRecovery(offer_pub_, handover_.pending->id);
  }
  //加
  rclcpp::Time time_now = node_clock_->now();
  const int64_t activated = handover_.advance(time_now);
  if (activated > 0)
  {
    loadActiveTrajectory();
    RCLCPP_INFO(rclcpp::get_logger("traj_server"), "Activated trajectory %ld",
                static_cast<long>(activated));
  }
  else if (activated < 0)
    publishId(ack_pub_, activated);

  // While a proposal waits in pending, continue sampling the old active slot.
  // Before the first scheduled start, the PX4 bridge retains its takeoff hover.
  if (!receive_traj_)
    return;
  double t_cur = (time_now - start_time_).seconds();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static rclcpp::Time time_last = time_now - rclcpp::Duration::from_seconds(0.01);
  if ((time_now - time_last).seconds() <= 0.0)
    time_last = time_now - rclcpp::Duration::from_seconds(0.01);
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
    // A future trajectory must never become a READY command at the origin.
    time_last = time_now;//加
    return;//加
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub->publish(cmd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server");
  node_clock_ = node->get_clock();
//加
  offer_pub_ = node->create_publisher<std_msgs::msg::Int64MultiArray>(
      "/drone_0_planning/recovery_offer", rclcpp::QoS(1).reliable().transient_local());
  anchor_ack_pub_ = node->create_publisher<std_msgs::msg::Int64MultiArray>(
      "/drone_0_planning/recovery_anchor_ack", rclcpp::QoS(1).reliable());
  auto request_sub = node->create_subscription<std_msgs::msg::Int64MultiArray>(
      "/drone_0_planning/recovery_request", rclcpp::QoS(1).reliable(),
      [](std_msgs::msg::Int64MultiArray::ConstSharedPtr m) { recoveryRequest(*m); });
  auto stopped_sub = node->create_subscription<std_msgs::msg::Int64MultiArray>(
      "/drone_0_planning/recovery_stopped", rclcpp::QoS(1).reliable(),
      [](std_msgs::msg::Int64MultiArray::ConstSharedPtr m) {
        recoveryRequest(*m);
        if (m->data.size() == 2 && recovery_running_ && m->data[0] == recovery_epoch_ &&
            m->data[1] == recovery_floor_)
          stopped_receive_ = std::chrono::steady_clock::now();
      });
  auto accept_sub = node->create_subscription<std_msgs::msg::Int64MultiArray>(
      "/drone_0_planning/recovery_accept", rclcpp::QoS(1).reliable(),
      [](std_msgs::msg::Int64MultiArray::ConstSharedPtr m) {
        if (!hard_hold_ && recovery_running_ && stoppedFresh() && m->data.size() == 2 &&
            m->data[0] == recovery_epoch_ && handover_.pending &&
            m->data[1] == handover_.pending->id && recovery_anchor_id_ > 0)
        {
          recovery_accepted_id_ = m->data[1];
          publishId(ack_pub_, m->data[1]);
        }
      });
  auto tracking_sub = node->create_subscription<std_msgs::msg::Int64MultiArray>(
      "/drone_0_planning/recovery_tracking", rclcpp::QoS(1).reliable(),
      [](std_msgs::msg::Int64MultiArray::ConstSharedPtr m) {
        if (!hard_hold_ && recovery_running_ && m->data.size() == 2 &&
            m->data[0] == recovery_epoch_ && handover_.active &&
            m->data[1] == handover_.active->id && m->data[1] == recovery_accepted_id_)
        {
          recovery_running_ = false;
          RCLCPP_INFO(rclcpp::get_logger("traj_server"), "[RECOVERY] bridge TRACK confirmed");
        }
      });
  auto hard_hold_sub = node->create_subscription<std_msgs::msg::Bool>(
      "/drone_0_planning/safety_hold", rclcpp::QoS(1).reliable().transient_local(),
      [](std_msgs::msg::Bool::ConstSharedPtr m) {
        if (!m->data) return;
        hard_hold_ = true;
        recovery_running_ = false;
        handover_.pending.reset();
        handover_.active.reset();
        handover_.committed = false;
        receive_traj_ = false;
      });
//加
  ack_pub_ = node->create_publisher<std_msgs::msg::Int64>("planning/trajectory_ack", 10);
  committed_pub_ = node->create_publisher<std_msgs::msg::Int64>("planning/trajectory_committed", 10);
  auto commit_sub = node->create_subscription<std_msgs::msg::Int64>(
      "planning/trajectory_commit", 10, commitCallback);
  auto cancel_sub = node->create_subscription<std_msgs::msg::Int64>(
      "planning/cancel_pending", 10, cancelCallback);
  auto emergency_sub = node->create_subscription<traj_utils::msg::Bspline>(
      "planning/emergency_bspline", 10, emergencyCallback);
  //加
  auto odom_yaw_sub =
      node->create_subscription<nav_msgs::msg::Odometry>(
          "/grid_map/odom_enu",
          rclcpp::SensorDataQoS(),
          odomYawCallback);

  auto bspline_sub = node->create_subscription<traj_utils::msg::Bspline>(
      "planning/bspline",
      10,
      bsplineCallback);

  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "/position_cmd",
      50);

  auto cmd_timer = node->create_wall_timer(
      std::chrono::milliseconds(10),
      cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  node->declare_parameter("traj_server/time_forward", -1.0);
  node->get_parameter("traj_server/time_forward", time_forward_);

  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
