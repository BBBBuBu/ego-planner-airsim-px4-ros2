#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <thread>

#include <ego_planner/trajectory_handover.h>
#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/dyn_a_star.h>

using ego_planner::PlanningBudget;
using ego_planner::ScheduledTrajectory;
using ego_planner::TrajectoryHandover;

namespace
{
rclcpp::Time at(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9)), RCL_ROS_TIME);
}

// Analytic x(t) = x0 + v*t + a*t^2/2 on a uniform cubic spline.
traj_utils::msg::Bspline curve(int id, double start, double x0,
                             double velocity = 0.5, double acceleration = 0.0)
{
  traj_utils::msg::Bspline msg;
  msg.traj_id = id;
  msg.order = 3;
  msg.start_time = at(start);
  constexpr double dt = 0.2;
  for (int i = 0; i < 10; ++i)
  {
    const double n = i - 1;
    geometry_msgs::msg::Point p;
    p.x = x0 + velocity * n * dt + acceleration * (n * n - 1.0 / 3.0) * dt * dt / 2;
    p.z = 2.0;
    msg.pos_pts.push_back(p);
  }
  for (int i = 0; i < 14; ++i)
    msg.knots.push_back((i - 3) * dt);
  return msg;
}

ScheduledTrajectory decode(const traj_utils::msg::Bspline &msg)
{
  ScheduledTrajectory out;
  std::string reason;
  EXPECT_TRUE(ego_planner::decodeTrajectory(msg, out, reason)) << reason;
  return out;
}

TrajectoryHandover activeServer()
{
  TrajectoryHandover server;
  server.active = decode(curve(1, 100.0, 0.0));
  server.highest_id = 1;
  return server;
}
}  // namespace

TEST(Handover, OldCommandsContinueUntilCommittedBoundary)
{
  auto server = activeServer();
  std::string reason;
  ASSERT_TRUE(server.stage(decode(curve(2, 100.2, 0.1)), at(100.05), reason)) << reason;
  EXPECT_EQ(server.active->id, 1);
  ASSERT_TRUE(server.commit(2, at(100.06)));
  for (int i = 0; i < 20; ++i)
  {
    const double t = i * 0.01;
    EXPECT_EQ(server.advance(at(100.0 + t)), 0);
    ASSERT_TRUE(server.active.has_value());
    EXPECT_NEAR(server.active->sample(t)[0].x(), 0.5 * t, 1e-10);
    EXPECT_NEAR(server.active->sample(t)[1].x(), 0.5, 1e-10);
  }
  EXPECT_EQ(server.advance(at(100.2)), 2);
  EXPECT_EQ(server.active->id, 2);
  EXPECT_NEAR(server.active->sample(0.0)[0].x(), 0.1, 1e-10);
  EXPECT_NEAR(server.active->sample(0.0)[1].x(), 0.5, 1e-10);
}

TEST(Handover, UncommittedAndCancelledProposalsNeverExecute)
{
  auto server = activeServer();
  std::string reason;
  ASSERT_TRUE(server.stage(decode(curve(2, 100.2, 0.1)), at(100.0), reason));
  EXPECT_EQ(server.advance(at(100.21)), -2);
  EXPECT_EQ(server.active->id, 1);
  EXPECT_FALSE(server.commit(2, at(100.21)));
  EXPECT_FALSE(server.stage(decode(curve(2, 100.4, 0.2)), at(100.22), reason));
  ASSERT_TRUE(server.stage(decode(curve(3, 100.4, 0.2)), at(100.22), reason));
  EXPECT_FALSE(server.cancel(999));
  ASSERT_TRUE(server.commit(3, at(100.23)));
  EXPECT_TRUE(server.cancel(3));
  EXPECT_EQ(server.advance(at(100.5)), 0);
  EXPECT_EQ(server.active->id, 1);
}

TEST(Handover, ExpiredAndOutOfOrderMessagesDoNotReplaceActive)
{
  auto server = activeServer();
  std::string reason;
  EXPECT_FALSE(server.stage(decode(curve(2, 100.1, 0.05)), at(100.11), reason));
  ASSERT_TRUE(server.stage(decode(curve(3, 100.2, 0.1)), at(100.11), reason));
  EXPECT_FALSE(server.commit(2, at(100.12)));
  EXPECT_FALSE(server.stage(decode(curve(4, 100.3, 0.15)), at(100.12), reason));
  EXPECT_EQ(server.pending->id, 3);
  EXPECT_EQ(server.active->id, 1);
}

TEST(Handover, RejectsPositionVelocityAndAccelerationDiscontinuities)
{
  auto server = activeServer();
  std::string reason;
  EXPECT_FALSE(server.stage(decode(curve(2, 100.2, 0.98)), at(100.0), reason));
  EXPECT_FALSE(server.stage(decode(curve(3, 100.2, 0.1, 1.0)), at(100.0), reason));
  EXPECT_FALSE(server.stage(decode(curve(4, 100.2, 0.1, 0.5, 0.5)), at(100.0), reason));
  EXPECT_EQ(server.active->id, 1);
  EXPECT_FALSE(server.pending);
}

TEST(Handover, EmergencyInvalidatesPendingAndRejectsNonConstantBypass)
{
  auto server = activeServer();
  std::string reason;
  ASSERT_TRUE(server.stage(decode(curve(2, 100.2, 0.1)), at(100.0), reason));
  ASSERT_TRUE(server.commit(2, at(100.01)));
  EXPECT_FALSE(server.emergency(decode(curve(3, 100.1, 0.7, 0.5))));
  ASSERT_TRUE(server.emergency(decode(curve(4, 100.1, 0.7, 0.0))));
  EXPECT_FALSE(server.pending);
  EXPECT_FALSE(server.commit(2, at(100.12)));
  EXPECT_EQ(server.advance(at(100.3)), 0);
  EXPECT_EQ(server.active->id, 4);
  EXPECT_NEAR(server.active->sample(0.2)[0].x(), 0.7, 1e-10);
}

TEST(Handover, EndpointHoldCanHandOverWithoutResettingStartTime)
{
  auto server = activeServer();
  const double end = server.active->duration;
  const double endpoint = server.active->sample(end)[0].x();
  std::string reason;
  EXPECT_FALSE(server.stage(decode(curve(2, 102.0, endpoint, 0.5)), at(101.9), reason));
  ASSERT_TRUE(server.stage(decode(curve(3, 102.0, endpoint, 0.0)), at(101.9), reason));
  ASSERT_TRUE(server.commit(3, at(101.91)));
  EXPECT_EQ(server.advance(at(102.0)), 3);
  EXPECT_NEAR(server.active->sample(0.0)[0].x(), endpoint, 1e-10);
  EXPECT_NEAR(server.active->sample(0.0)[1].norm(), 0.0, 1e-10);
}

TEST(Handover, MalformedSplinesAreRejectedBeforeEvaluation)
{
  ScheduledTrajectory result;
  std::string reason;
  auto msg = curve(1, 100, 0);
  msg.pos_pts[2].x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ego_planner::decodeTrajectory(msg, result, reason));
  msg = curve(1, 100, 0);
  msg.knots[5] = msg.knots[4];
  EXPECT_FALSE(ego_planner::decodeTrajectory(msg, result, reason));
  msg = curve(1, 100, 0);
  msg.knots.pop_back();
  EXPECT_FALSE(ego_planner::decodeTrajectory(msg, result, reason));
  msg = curve(1, 100, 0);
  msg.start_time.nanosec = 1000000000U;
  EXPECT_FALSE(ego_planner::decodeTrajectory(msg, result, reason));
}

TEST(Budget, ExpiredBudgetIsSharedAcrossSearchesAndOptimizerRetries)
{
  auto budget = std::make_shared<PlanningBudget>(0.001);
  const auto original_deadline = budget->deadline();
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  ASSERT_TRUE(budget->expired());
  AStar search;
  search.initGridMap(nullptr, Eigen::Vector3i(3, 3, 3));
  search.setSearchBudget(0.2);
  search.setPlanningBudget(budget);
  for (int i = 0; i < 10; ++i)
  {
    EXPECT_FALSE(search.AstarSearch(0.1, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));
    EXPECT_EQ(budget->deadline(), original_deadline);
  }
  ego_planner::BsplineOptimizer optimizer;
  optimizer.setPlanningBudget(budget);
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 8), output;
  EXPECT_TRUE(optimizer.initControlPoints(points, true).empty());
  EXPECT_TRUE(optimizer.initializationFailed());
  ego_planner::ControlPoints cps;
  cps.resize(8);
  cps.points = points;
  double cost = 0.0;
  EXPECT_FALSE(optimizer.BsplineOptimizeTrajRebound(output, cost, cps, 0.2));
  EXPECT_FALSE(optimizer.BsplineOptimizeTrajRefine(points, 0.2, output));
  EXPECT_TRUE(budget->expired());
}

TEST(Budget, RejectsInvalidConfiguration)
{
  EXPECT_THROW(PlanningBudget(0.0), std::invalid_argument);
  EXPECT_THROW(PlanningBudget(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}
