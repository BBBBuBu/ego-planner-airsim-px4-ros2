#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include <vector>
#include <optional>
#include <chrono>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>//加
#include <ego_planner/trajectory_handover.h>//加
#include "visualization_msgs/msg/marker.hpp"

#include "bspline_opt/bspline_optimizer.h"
#include "plan_env/grid_map.h"
#include "traj_utils/msg/bspline.hpp"
#include "traj_utils/msg/multi_bsplines.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "traj_utils/msg/data_disp.hpp"
#include "ego_planner/planner_manager.h"
#include "traj_utils/planning_visualization.h"

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::msg::DataDisp data_disp_;
    traj_utils::msg::MultiBsplines multi_bspline_msgs_buf_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_, wp_id_;
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> wps_;
    int current_wp_;

    PlanningBudget::Ptr planning_budget_;
    rclcpp::Time scheduled_start_{0, 0, RCL_ROS_TIME};
    double handover_lead_sec_ = 0.20;
    double min_publish_margin_sec_ = 0.05;
    double retry_delay_sec_ = 0.10;
    PlanningBudget::Clock::time_point next_plan_attempt_{};
    PlanningBudget::Clock::time_point last_odom_receive_{};
    std::optional<LocalTrajData> pending_traj_;
    bool pending_accepted_ = false;
    bool pending_commit_sent_ = false;
    bool pending_committed_ = false;
    bool pending_rejected_ = false;
    bool pending_startup_pub_ = false;
    double pending_previous_progress_ = 0.0;
    Eigen::Vector3d pending_previous_target_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d pending_previous_velocity_ = Eigen::Vector3d::Zero();

    //加
    // C：桥进入 HOLD 后锁止本轮任务，忽略后续 false 消息。
    bool bridge_hold_latched_ = false;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bridge_hold_sub_;

    // C：实际到达判据。
    double goal_position_tolerance_ = 0.20;
    double goal_speed_tolerance_ = 0.10;
    double goal_hold_time_ = 1.0;
    bool goal_timer_active_ = false;
    PlanningBudget::Clock::time_point goal_since_{};
    PlanningBudget::Clock::time_point goal_last_check_{};
//加
    enum class RecoveryPhase { IDLE, WAIT_STOP, WAIT_ANCHOR, PLAN, WAIT_TRACKING };
    RecoveryPhase recovery_phase_ = RecoveryPhase::IDLE;
    int64_t recovery_epoch_ = 0, recovery_floor_ = 0;
    int64_t recovery_anchor_id_ = 0, recovery_tracking_id_ = 0;
    bool recovery_anchor_acked_ = false;
    Eigen::Vector3d recovery_anchor_pos_ = Eigen::Vector3d::Zero();
    PlanningBudget::Clock::time_point recovery_stopped_receive_{};
    PlanningBudget::Clock::time_point recovery_anchor_sent_{};
    std::vector<rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr> recovery_subs_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr recovery_needed_pub_;
    void initRecoveryInterfaces();
    void recoveryRequest(const std_msgs::msg::Int64MultiArray &msg);
    bool processRecovery();
//加
    bool actualGoalReached();
    //加
    bool flag_escape_emergency_;
    //加 急停后连续停稳检测，使用单调时钟。
    bool emergency_settle_active_ = false;

    PlanningBudget::Clock::time_point emergency_settle_since_{};
    PlanningBudget::Clock::time_point emergency_last_check_{};

    /* ROS utils */
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_sub_;
    rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr broadcast_bspline_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;

    // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
    // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr new_pub_;
    rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr emergency_bspline_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr commit_pub_, cancel_pending_pub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr trajectory_ack_sub_, trajectory_committed_sub_;
    rclcpp::Publisher<traj_utils::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_pub_;
    rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr broadcast_bspline_pub_;

    /* helper functions */
    bool beginPlanningRound();
    void processHandover();
    void cancelPendingTrajectory();
    bool trajectoryClear(LocalTrajData &trajectory, double begin, double end,
                         bool use_planning_budget = false);
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromCurrentTraj(const int trial_times = 1);

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void readGivenWps();
    void planNextWaypoint(const Eigen::Vector3d next_wp);
    void getLocalTarget();

    /* ROS functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg);
    void triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg);
    void odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg);
    void swarmTrajsCallback(const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg);
    void BroadcastBsplineCallback(const std::shared_ptr<const traj_utils::msg::Bspline> &msg);

    bool checkCollision();
    void publishSwarmTrajs(bool startup_pub);

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    void init(rclcpp::Node::SharedPtr &node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif