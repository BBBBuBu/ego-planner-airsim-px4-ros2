
#include <ego_planner/ego_replan_fsm.h>
#include <cmath>//加
#include <stdexcept>//加

namespace ego_planner
{
  namespace
  {
  struct PlanningRoundTrace
  {
    rclcpp::Logger logger;
    PlanningBudget::Ptr budget;
    double seconds;
    ~PlanningRoundTrace()
    {
      const auto started = budget->deadline() -
          std::chrono::duration_cast<PlanningBudget::Clock::duration>(
              std::chrono::duration<double>(seconds));
      const double ms = std::chrono::duration<double, std::milli>(
          PlanningBudget::Clock::now() - started).count();
      RCLCPP_INFO(logger, "[AB_PLAN] elapsed_ms=%.3f budget_ms=%.3f expired=%d",
                  ms, seconds * 1000.0, budget->expired());
    }
  };
  }

  void EGOReplanFSM::init(rclcpp::Node::SharedPtr &node)
  {
    node_ = node;
    
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    have_new_target_ = false;
    flag_escape_emergency_ = false;
    local_target_pt_.setZero();
    local_target_vel_.setZero();
    handover_lead_sec_ = node_->declare_parameter("fsm/handover_lead_sec", 0.20);
    min_publish_margin_sec_ = node_->declare_parameter("fsm/min_publish_margin_sec", 0.05);
    retry_delay_sec_ = node_->declare_parameter("fsm/retry_delay_sec", 0.10);
    //加
    goal_position_tolerance_ =
        node_->declare_parameter("fsm/goal_position_tolerance", 0.20);
    goal_speed_tolerance_ =
        node_->declare_parameter("fsm/goal_speed_tolerance", 0.10);
    goal_hold_time_ =
        node_->declare_parameter("fsm/goal_hold_time", 1.0);

    if (!std::isfinite(goal_position_tolerance_) ||
        goal_position_tolerance_ <= 0.0 ||
        !std::isfinite(goal_speed_tolerance_) ||
        goal_speed_tolerance_ <= 0.0 ||
        !std::isfinite(goal_hold_time_) ||
        goal_hold_time_ <= 0.0)
    {
      throw std::invalid_argument(
          "Goal tolerances and hold time must be positive");
    }
    //加

    node_->declare_parameter("fsm/flight_type", -1);
    node_->declare_parameter("fsm/thresh_replan_time", -1.0);
    node_->declare_parameter("fsm/thresh_no_replan_meter", -1.0);
    node_->declare_parameter("fsm/planning_horizon", -1.0);
    node_->declare_parameter("fsm/planning_horizen_time", -1.0);
    node_->declare_parameter("fsm/emergency_time", 1.0);
    node_->declare_parameter("fsm/realworld_experiment", false);
    node_->declare_parameter("fsm/fail_safe", true);

    node_->get_parameter("fsm/flight_type", target_type_);
    node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
    node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
    node_->get_parameter("fsm/planning_horizon", planning_horizen_);
    node_->get_parameter("fsm/planning_horizen_time", planning_horizen_time_);
    node_->get_parameter("fsm/emergency_time", emergency_time_);
    node_->get_parameter("fsm/realworld_experiment", flag_realworld_experiment_);
    node_->get_parameter("fsm/fail_safe", enable_fail_safe_);

    have_trigger_ = !flag_realworld_experiment_;

    node_->declare_parameter("fsm/waypoint_num", -1);
    node_->get_parameter("fsm/waypoint_num", waypoint_num_);

    for (int i = 0; i < waypoint_num_; i++)
    {
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_x", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_y", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_z", -1.0);

      node_->get_parameter("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2]);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));

    planner_manager_.reset(new EGOPlannerManager);

    planner_manager_->initPlanModules(node_, visualization_);
    if (!std::isfinite(handover_lead_sec_) || !std::isfinite(min_publish_margin_sec_) ||
        !std::isfinite(retry_delay_sec_) || min_publish_margin_sec_ <= 0.0 ||
        retry_delay_sec_ <= 0.0 || handover_lead_sec_ <=
          planner_manager_->planBudgetSeconds() + min_publish_margin_sec_)
      throw std::invalid_argument("Handover lead must exceed planning budget + publication margin");

    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();
    //加
    bridge_hold_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/drone_0_planning/safety_hold",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::ConstSharedPtr msg)
        {
          if (!msg->data || bridge_hold_latched_)
            return;

          bridge_hold_latched_ = true;
          enable_fail_safe_ = false;
          have_trigger_ = false;
          goal_timer_active_ = false;
          flag_escape_emergency_ = false;

          // changeFSMExecState 内部会取消 pending。
          changeFSMExecState(EMERGENCY_STOP, "BRIDGE_HOLD");

          RCLCPP_ERROR(
              node_->get_logger(),
              "Bridge HOLD latched: pending cancelled; "
              "task blocked until ground restart");
        });
      //加
    initRecoveryInterfaces();//加
    /* callback*/
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&EGOReplanFSM::execFSMCallback, this));

    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&EGOReplanFSM::checkCollisionCallback, this));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom_world",
        1,
        [this](const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
        {
          this->odometryCallback(msg);
        });
    // std::bind(&EGOReplanFSM::odometryCallback, this, std::placeholders::_1));

    if (planner_manager_->pp_.drone_id >= 1)
    {
      string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
      swarm_trajs_sub_ = node_->create_subscription<traj_utils::msg::MultiBsplines>(
          sub_topic_name,
          10,
          [this](const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg)
          {
            this->swarmTrajsCallback(msg);
          });
    }

    // ros2 中topic名字中不能出现负号，单机id是-1需要处理
    // string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    string pub_topic_name;
    if (planner_manager_->pp_.drone_id <= -1)
    {
      RCLCPP_INFO(node_->get_logger(), "single drone:%d", planner_manager_->pp_.drone_id);
      pub_topic_name = string("/drone_") + "single" + string("_planning/swarm_trajs");
    }else
    {
      pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    }
    
    swarm_trajs_pub_ = node_->create_publisher<traj_utils::msg::MultiBsplines>(pub_topic_name, 10);

    broadcast_bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = node_->create_subscription<traj_utils::msg::Bspline>(
        "planning/broadcast_bspline_to_planner",
        100,
        [this](const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
        {
          this->BroadcastBsplineCallback(msg);
        });

    bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/bspline", 10);
    emergency_bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/emergency_bspline", 10);
    commit_pub_ = node_->create_publisher<std_msgs::msg::Int64>("planning/trajectory_commit", 10);
    cancel_pending_pub_ = node_->create_publisher<std_msgs::msg::Int64>("planning/cancel_pending", 10);
    trajectory_ack_sub_ = node_->create_subscription<std_msgs::msg::Int64>(
        "planning/trajectory_ack", 10,
        [this](std_msgs::msg::Int64::ConstSharedPtr msg) {
          if (!pending_traj_) return;
          if (msg->data == pending_traj_->traj_id_) pending_accepted_ = true;
          if (msg->data == -static_cast<int64_t>(pending_traj_->traj_id_))
            pending_rejected_ = true;
        });
    trajectory_committed_sub_ = node_->create_subscription<std_msgs::msg::Int64>(
        "planning/trajectory_committed", 10,
        [this](std_msgs::msg::Int64::ConstSharedPtr msg) {
          if (pending_traj_ && pending_commit_sent_ && msg->data == pending_traj_->traj_id_)
            pending_committed_ = true;
        });
    data_disp_pub_ = node_->create_publisher<traj_utils::msg::DataDisp>("planning/data_display", 100);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/move_base_simple/goal",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->waypointCallback(msg);
          });
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/traj_start_trigger",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->triggerCallback(msg);
          });

      RCLCPP_INFO(node_->get_logger(), "Wait for 1 second.");
      int count = 0;
      while (rclcpp::ok() && count++ < 1000)
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      RCLCPP_WARN(node_->get_logger(), "Waiting for trigger from [n3ctrl] from RC");

      while (rclcpp::ok() && (!have_odom_ || !have_trigger_))
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      readGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::readGivenWps()

  {
    if (waypoint_num_ <= 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    // 用 visualization_->displayGoalPoint() 方法对waypoint进行可视化
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // plan first global waypoint
    wp_id_ = 0;
    planNextWaypoint(wps_[wp_id_]);
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    if (bridge_hold_latched_) return;//加

    goal_timer_active_ = false;//加
    goal_last_check_ = PlanningBudget::Clock::time_point{};//加

    bool success = false;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), next_wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {
      end_pt_ = next_wp;

      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM状态转换 ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          if (bridge_hold_latched_) return;//加
          rclcpp::spin_some(node_);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    if (bridge_hold_latched_ ||
    recovery_phase_ != RecoveryPhase::IDLE) return;//加
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    if (bridge_hold_latched_ ||
    recovery_phase_ != RecoveryPhase::IDLE) return;//加
    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;

    init_pt_ = odom_pos_;

    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);

    planNextWaypoint(end_wp);
  }

  void EGOReplanFSM::odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    last_odom_receive_ = PlanningBudget::Clock::now();
    have_odom_ = true;
  }

  void EGOReplanFSM::BroadcastBsplineCallback(const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
  {
    size_t id = msg->drone_id;
    if ((int)id == planner_manager_->pp_.drone_id)
      return;

    // if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    auto &clock = *node_->get_clock();
    auto msg_time = rclcpp::Time(msg->start_time, clock.get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Clock type: %d", node_->now().get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Start time clock type: %d", rclcpp::Time(msg->start_time).get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "msg_time: %d", msg_time.get_clock_type());
    if (abs((node_->now() - msg_time).seconds()) > 0.25)
    {
      // ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs", msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      RCLCPP_ERROR(node_->get_logger(), "Time difference is too large! Local - Remote Agent %d = %fs",
                   msg->drone_id, (node_->now() - msg_time).seconds());
      return;
    }

    // 路径缓冲区初始化
    if (planner_manager_->swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_trajs_buf_.push_back(blank);
      }
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
      return; // if the current drone is too far to the received agent.
    }

    /* Store data */
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->pos_pts.size(); ++j)
    {
      pos_pts(0, j) = msg->pos_pts[j].x;
      pos_pts(1, j) = msg->pos_pts[j].y;
      pos_pts(2, j) = msg->pos_pts[j].z;
    }

    planner_manager_->swarm_trajs_buf_[id].drone_id = id;

    // 计算路径持续时间
    if (msg->order % 2)
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }

    // 生成bspline并存储
    UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
  }

  void EGOReplanFSM::swarmTrajsCallback(const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg)
  {

    multi_bspline_msgs_buf_.traj.clear();
    multi_bspline_msgs_buf_ = *msg;

    if (!have_odom_)
    {
      RCLCPP_ERROR(node_->get_logger(), "swarmTrajsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong trajectory size!msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      return;
    }

    if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    {
      RCLCPP_ERROR(node_->get_logger(), "Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    planner_manager_->swarm_trajs_buf_.clear();
    planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

    // 处理每条路径
    for (size_t i = 0; i < msg->traj.size(); i++)
    {

      Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
        continue;
      }

      // 存储路径控制点和节点
      Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
      Eigen::VectorXd knots(msg->traj[i].knots.size());
      for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
      {
        knots(j) = msg->traj[i].knots[j];
      }
      for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
        pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
        pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
      }

      planner_manager_->swarm_trajs_buf_[i].drone_id = i;

      // 计算路径持续时间
      if (msg->traj[i].order % 2)
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
      UniformBspline pos_traj(pos_pts, msg->traj[i].order, msg->traj[i].knots[1] - msg->traj[i].knots[0]);
      pos_traj.setKnot(knots);
      planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

      planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

      planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
    }

    have_recv_pre_agent_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    //if (new_state == EMERGENCY_STOP)
    //  cancelPendingTrajectory();
    if (new_state == EMERGENCY_STOP)//加
    {
      cancelPendingTrajectory();

      if (exec_state_ != EMERGENCY_STOP)
        flag_escape_emergency_ = true;
    }
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback()
  {
    if (bridge_hold_latched_) return;//加
    if (processRecovery()) return;
    processHandover();
    exec_timer_->cancel(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (have_odom_ && have_target_ && have_trigger_)
        {
          bool success = planFromGlobalTraj(10); // zx-todo
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");

            publishSwarmTrajs(true);
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(), "Failed to generate the first trajectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM");
          }
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      // local_data_ remains the old active trajectory until confirmed switching.
      if (pending_traj_ || planner_manager_->local_data_.traj_id_ == 0)
        break;
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = std::min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wp_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wp_id_++;
        planNextWaypoint(wps_[wp_id_]);
      }
      else if ((local_target_pt_ - end_pt_).norm() < 1e-3) // close to the global target
      {
        //if (t_cur > info->duration_ - 1e-2)
        if (actualGoalReached())//改
        {
          have_target_ = false;
          have_trigger_ = false;

          if (target_type_ == TARGET_TYPE::PRESET_TARGET)
          {
            wp_id_ = 0;
            planNextWaypoint(wps_[wp_id_]);
          }

          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }
        case EMERGENCY_STOP:
    {
      using SteadyClock = PlanningBudget::Clock;
      const auto now = SteadyClock::now();

      // 初始调试值：速度连续 0.5 秒低于 0.10 m/s。
      constexpr double settle_speed = 0.10;
      constexpr double settle_time = 0.50;

      // 主循环出现明显空档时，重新累计停稳时间。
      const bool check_gap =
          emergency_last_check_ != SteadyClock::time_point{} &&
          now - emergency_last_check_ >
              std::chrono::milliseconds(150);

      emergency_last_check_ = now;

      if (check_gap)
        emergency_settle_active_ = false;

      // 实际状态必须有效、新鲜。
      // 无效反馈不能用来生成新的停止参考或确认停稳。
      const bool odom_valid =
          have_odom_ &&
          odom_pos_.allFinite() &&
          odom_vel_.allFinite() &&
          now - last_odom_receive_ <=
              std::chrono::milliseconds(200);

      if (!odom_valid)
      {
        emergency_settle_active_ = false;
        break;
      }

      // 每次进入急停，先发送一次停止轨迹。
      if (flag_escape_emergency_)
      {
        emergency_settle_active_ = false;

        if (callEmergencyStop(odom_pos_))
        {
          flag_escape_emergency_ = false;

          RCLCPP_WARN(
              node_->get_logger(),
              "Emergency stop sent. Waiting for actual speed "
              "< %.2f m/s for %.2f s.",
              settle_speed, settle_time);
        }

        // 本周期只处理急停发送，不立即开始恢复。
        break;
      }

      auto map = planner_manager_->grid_map_;

      // 保留任务目标；只有任务和地图满足条件时才恢复。
      const bool recovery_allowed =
          enable_fail_safe_ &&
          have_target_ &&
          have_trigger_ &&
          map->odomValid() &&
          map->hasDepthObservation() &&
          !map->getOdomDepthTimeout();

      if (!recovery_allowed)
      {
        emergency_settle_active_ = false;
        break;
      }

      const double actual_speed = odom_vel_.norm();

      // 速度重新超限，重新累计停稳时间。
      if (actual_speed >= settle_speed)
      {
        emergency_settle_active_ = false;
        break;
      }

      if (!emergency_settle_active_)
      {
        emergency_settle_active_ = true;
        emergency_settle_since_ = now;
        break;
      }

      const double settled_seconds =
          std::chrono::duration<double>(
              now - emergency_settle_since_).count();

      if (settled_seconds < settle_time)
        break;

      emergency_settle_active_ = false;

      RCLCPP_INFO(
          node_->get_logger(),
          "Emergency settled: speed=%.3f m/s, duration=%.3f s. "
          "Requesting a new trajectory.",
          actual_speed, settled_seconds);

      //changeFSMExecState(GEN_NEW_TRAJ, "EMERGENCY_SETTLED");
      recovery_needed_pub_->publish(std_msgs::msg::Empty{});//改
      break;
    }
    //case EMERGENCY_STOP:
    //{

    //  if (flag_escape_emergency_) // Avoiding repeated calls
    //  {
    //    callEmergencyStop(odom_pos_);
    //  }
    //  else
    //  {
    //    if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
    //      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
    //  }

    //  flag_escape_emergency_ = false;
    //  break;
    //}
    }

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);

  force_return:;
    // exec_timer_.start();
    if (exec_timer_ && exec_timer_->is_canceled())
    {
      // 取消状态下无需重新创建，可以复用现有计时器
      exec_timer_->reset();
    }
  }
  //加
  bool EGOReplanFSM::actualGoalReached()
  {
    const auto now = PlanningBudget::Clock::now();

    const bool gap =
        goal_last_check_ != PlanningBudget::Clock::time_point{} &&
        now - goal_last_check_ > std::chrono::milliseconds(150);

    goal_last_check_ = now;

    if (bridge_hold_latched_ ||
        pending_traj_ ||
        !have_odom_ ||
        gap ||
        now - last_odom_receive_ > std::chrono::milliseconds(200) ||
        !odom_pos_.allFinite() ||
        !odom_vel_.allFinite() ||
        planner_manager_->grid_map_->getOdomDepthTimeout() ||
        (odom_pos_ - end_pt_).norm() > goal_position_tolerance_ ||
        odom_vel_.norm() > goal_speed_tolerance_)
    {
      goal_timer_active_ = false;
      return false;
    }

    if (!goal_timer_active_)
    {
      goal_timer_active_ = true;
      goal_since_ = now;
    }

    return std::chrono::duration<double>(
               now - goal_since_).count() >= goal_hold_time_;
  }

  void EGOReplanFSM::initRecoveryInterfaces()
  {
    // Bridge repeats current recovery state. Volatile readers avoid replaying
    // a completed request when a node restarts (restart all nodes on ground).
    const auto qos = rclcpp::QoS(1).reliable();
    recovery_needed_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
        "/drone_0_planning/recovery_needed", 10);
    recovery_subs_.push_back(node_->create_subscription<std_msgs::msg::Int64MultiArray>(
        "/drone_0_planning/recovery_request", qos,
        [this](std_msgs::msg::Int64MultiArray::ConstSharedPtr m) { recoveryRequest(*m); }));
    recovery_subs_.push_back(node_->create_subscription<std_msgs::msg::Int64MultiArray>(
        "/drone_0_planning/recovery_stopped", qos,
        [this](std_msgs::msg::Int64MultiArray::ConstSharedPtr m) {
          recoveryRequest(*m); // messages on different topics may be reordered
          if (m->data.size() == 2 && m->data[0] == recovery_epoch_ &&
              m->data[1] == recovery_floor_ && recovery_phase_ != RecoveryPhase::IDLE)
            recovery_stopped_receive_ = PlanningBudget::Clock::now();
        }));
    recovery_subs_.push_back(node_->create_subscription<std_msgs::msg::Int64MultiArray>(
        "/drone_0_planning/recovery_anchor_ack", qos,
        [this](std_msgs::msg::Int64MultiArray::ConstSharedPtr m) {
          if (m->data.size() == 2 && m->data[0] == recovery_epoch_ &&
              m->data[1] == recovery_anchor_id_ && recovery_anchor_id_ > 0)
            recovery_anchor_acked_ = true;
        }));
    recovery_subs_.push_back(node_->create_subscription<std_msgs::msg::Int64MultiArray>(
        "/drone_0_planning/recovery_tracking", qos,
        [this](std_msgs::msg::Int64MultiArray::ConstSharedPtr m) {
          if (m->data.size() != 2 || m->data[0] != recovery_epoch_ ||
              recovery_phase_ != RecoveryPhase::WAIT_TRACKING) return;
          const int64_t expected = pending_traj_ ? pending_traj_->traj_id_ :
              planner_manager_->local_data_.traj_id_;
          if (m->data[1] == expected && expected > recovery_anchor_id_)
            recovery_tracking_id_ = expected;
        }));
  }

  void EGOReplanFSM::recoveryRequest(const std_msgs::msg::Int64MultiArray &m)
  {
    if (bridge_hold_latched_ || m.data.size() != 2 || m.data[0] <= 0 ||
        m.data[0] <= recovery_epoch_ || m.data[1] < 0 ||
        m.data[1] >= std::numeric_limits<int32_t>::max() - 1) return;
    recovery_epoch_ = m.data[0];
    recovery_floor_ = m.data[1];
    recovery_anchor_id_ = recovery_tracking_id_ = 0;
    recovery_anchor_acked_ = false;
    recovery_stopped_receive_ = PlanningBudget::Clock::time_point{};
    recovery_anchor_sent_ = PlanningBudget::Clock::time_point{};
    recovery_phase_ = RecoveryPhase::WAIT_STOP;
    goal_timer_active_ = false;
    emergency_settle_active_ = false;
    changeFSMExecState(EMERGENCY_STOP, "RECOVERY_REQUEST");
    RCLCPP_WARN(node_->get_logger(), "[RECOVERY] epoch=%ld: waiting for stop; goal retained",
                static_cast<long>(recovery_epoch_));
  }

  bool EGOReplanFSM::processRecovery()
  {
    if (recovery_phase_ == RecoveryPhase::IDLE) return false;
    const auto now = PlanningBudget::Clock::now();
    // Keep checking staged routes until activation, including after commit.
    if (pending_traj_ && !trajectoryClear(*pending_traj_, 0.0,
                                          pending_traj_->duration_ * 2.0 / 3.0))
    {
      cancelPendingTrajectory();
      recovery_phase_ = RecoveryPhase::PLAN;
    }
    if (recovery_phase_ == RecoveryPhase::PLAN ||
        recovery_phase_ == RecoveryPhase::WAIT_TRACKING) processHandover();
    if (recovery_tracking_id_ > recovery_anchor_id_ &&
        planner_manager_->local_data_.traj_id_ == recovery_tracking_id_)
    {
      recovery_phase_ = RecoveryPhase::IDLE;
      flag_escape_emergency_ = false;
      changeFSMExecState(EXEC_TRAJ, "RECOVERY_TRACKING");
      RCLCPP_INFO(node_->get_logger(), "[RECOVERY] epoch=%ld: TRACK confirmed, id=%ld",
                  static_cast<long>(recovery_epoch_), static_cast<long>(recovery_tracking_id_));
      return true;
    }
    auto map = planner_manager_->grid_map_;
    if (recovery_phase_ == RecoveryPhase::WAIT_TRACKING)
    {
      if (!pending_traj_ && planner_manager_->local_data_.traj_id_ == recovery_anchor_id_)
        recovery_phase_ = RecoveryPhase::PLAN; // rejected/expired proposal
      else
      {
        // Never let an active recovery route escape collision checks while
        // waiting for the bridge's final acknowledgement.
        auto &active = planner_manager_->local_data_;
        if (active.traj_id_ > recovery_anchor_id_)
        {
          const double t = (node_->now() - active.start_time_).seconds();
          if (map->getOdomDepthTimeout() ||
              now - last_odom_receive_ > std::chrono::milliseconds(200) ||
              !trajectoryClear(active, t, std::min(active.duration_, t + 1.0)))
            recovery_needed_pub_->publish(std_msgs::msg::Empty{});
        }
        return true;
      }
    }
    const bool stopped = recovery_stopped_receive_ != PlanningBudget::Clock::time_point{} &&
        now - recovery_stopped_receive_ <= std::chrono::milliseconds(200) &&
        have_odom_ && now - last_odom_receive_ <= std::chrono::milliseconds(200) &&
        odom_pos_.allFinite() && odom_vel_.allFinite() && odom_vel_.norm() < 0.10;
    if (!stopped)
    {
      cancelPendingTrajectory();
      recovery_phase_ = RecoveryPhase::WAIT_STOP;
      recovery_anchor_id_ = 0;
      recovery_anchor_acked_ = false;
      return true;
    }
    if (!have_target_ || !have_trigger_ || !enable_fail_safe_ ||
        !map->odomValid() || !map->hasDepthObservation() || map->getOdomDepthTimeout() ||
        !map->isInMap(odom_pos_) || map->getInflateOccupancy(odom_pos_) != 0)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "[RECOVERY] waiting: task/map invalid or actual start occupied");
      return true;
    }
    if (recovery_phase_ == RecoveryPhase::WAIT_STOP ||
        (recovery_phase_ == RecoveryPhase::WAIT_ANCHOR && !recovery_anchor_acked_ &&
         now - recovery_anchor_sent_ > std::chrono::milliseconds(250)))
    {
      recovery_anchor_pos_ = odom_pos_;
      recovery_anchor_acked_ = false;
      if (!callEmergencyStop(recovery_anchor_pos_)) return true;
      recovery_anchor_id_ = planner_manager_->local_data_.traj_id_;
      recovery_anchor_sent_ = now;
      recovery_phase_ = RecoveryPhase::WAIT_ANCHOR;
      if (recovery_anchor_id_ <= recovery_floor_)
      {
        RCLCPP_ERROR(node_->get_logger(), "[RECOVERY] ID mismatch; restart all nodes on ground");
        return true;
      }
      RCLCPP_INFO(node_->get_logger(), "[RECOVERY] anchor sent, id=%ld",
                  static_cast<long>(recovery_anchor_id_));
      return true;
    }
    if (recovery_phase_ == RecoveryPhase::WAIT_ANCHOR && recovery_anchor_acked_)
    {
      if (!planner_manager_->planGlobalTraj(recovery_anchor_pos_, Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()))
        return true;
      have_new_target_ = true;
      next_plan_attempt_ = PlanningBudget::Clock::time_point{};
      recovery_phase_ = RecoveryPhase::PLAN;
      RCLCPP_INFO(node_->get_logger(), "[RECOVERY] anchor acknowledged; replan to retained goal");
    }
    if (recovery_phase_ == RecoveryPhase::PLAN)
    {
      if ((odom_pos_ - recovery_anchor_pos_).norm() > 0.05)
      {
        recovery_phase_ = RecoveryPhase::WAIT_STOP;
        return true;
      }
      if (planFromGlobalTraj(10))
      {
        recovery_phase_ = RecoveryPhase::WAIT_TRACKING;
        changeFSMExecState(EXEC_TRAJ, "RECOVERY_PROPOSED");
        publishSwarmTrajs(false);
      }
    }
    return true;
  }
  //加

  bool EGOReplanFSM::beginPlanningRound()
  {
    if (bridge_hold_latched_ || pending_traj_ || PlanningBudget::Clock::now() < next_plan_attempt_ ||
        !have_odom_ || PlanningBudget::Clock::now() - last_odom_receive_ >
          std::chrono::milliseconds(500))//加
      return false;

    planning_budget_ = std::make_shared<PlanningBudget>(planner_manager_->planBudgetSeconds());
    planner_manager_->setPlanningBudget(planning_budget_);
    //scheduled_start_ = node_->now() + rclcpp::Duration::from_seconds(handover_lead_sec_);
    scheduled_start_ = node_->now() + rclcpp::Duration::from_seconds(
        recovery_phase_ == RecoveryPhase::IDLE ? handover_lead_sec_ :
        std::max(0.50, handover_lead_sec_));//加
    auto &old = planner_manager_->local_data_;
    if (old.traj_id_ > 0)
    {
      const double t = (scheduled_start_ - old.start_time_).seconds();
      if (t < 0.0)
        return false;
      // The server keeps publishing the endpoint with zero derivatives after
      // a trajectory ends. Seed from that same commanded hold state.
      start_pt_ = old.position_traj_.evaluateDeBoorT(std::min(t, old.duration_));
      if (t >= old.duration_)//加
      {
        start_vel_.setZero();
        start_acc_.setZero();
      }
      else
      {
        start_vel_ = old.velocity_traj_.evaluateDeBoorT(t);
        start_acc_ = old.acceleration_traj_.evaluateDeBoorT(t);
      }
    }
    else
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
    }
    //加
    if (!odom_pos_.allFinite() || !start_pt_.allFinite())
      return false;

    auto map = planner_manager_->grid_map_;

    const int odom_unknown =
        map->isInMap(odom_pos_)
            ? static_cast<int>(map->isUnknown(odom_pos_))
            : -1;

    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[START_DIAG] active=%d "
        "odom=(%.3f,%.3f,%.3f) raw=%d inflate=%d unknown=%d "
        "start=(%.3f,%.3f,%.3f) inflate=%d gap=%.3f",
        planner_manager_->local_data_.traj_id_,
        odom_pos_.x(), odom_pos_.y(), odom_pos_.z(),
        map->getOccupancy(odom_pos_),
        map->getInflateOccupancy(odom_pos_),
        odom_unknown,
        start_pt_.x(), start_pt_.y(), start_pt_.z(),
        map->getInflateOccupancy(start_pt_),
        (start_pt_ - odom_pos_).norm());
    return true;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times)
  {
    if (!beginPlanningRound()) return false;
    const PlanningRoundTrace trace{node_->get_logger(), planning_budget_,
                                    planner_manager_->planBudgetSeconds()};
    const bool random_init = timesOfConsecutiveStateCalls().first != 1;
    for (int i = 0; i < trial_times && !planning_budget_->expired(); ++i)
      if (callReboundReplan(true, random_init)) return true;
    next_plan_attempt_ = PlanningBudget::Clock::now() +
        std::chrono::duration_cast<PlanningBudget::Clock::duration>(
            std::chrono::duration<double>(retry_delay_sec_));
    return false;
  }

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times)
  {
    if (!beginPlanningRound()) return false;
    const PlanningRoundTrace trace{node_->get_logger(), planning_budget_,
                                    planner_manager_->planBudgetSeconds()};
    if (callReboundReplan(false, false)) return true;
    if (!planning_budget_->expired() && callReboundReplan(true, false)) return true;
    for (int i = 0; i < trial_times && !planning_budget_->expired(); ++i)
      if (callReboundReplan(true, true)) return true;
    next_plan_attempt_ = PlanningBudget::Clock::now() +
        std::chrono::duration_cast<PlanningBudget::Clock::duration>(
            std::chrono::duration<double>(retry_delay_sec_));
    return false;
  }

  bool EGOReplanFSM::trajectoryClear(LocalTrajData &trajectory, double begin,
                                    double end, bool use_planning_budget)
  {
    if (trajectory.traj_id_ <= 0) return true;
    if (!std::isfinite(begin) || !std::isfinite(end)) return false;
    begin = std::clamp(begin, 0.0, trajectory.duration_);
    end = std::clamp(end, begin, trajectory.duration_);
    // Bound spatial steps using the derivative control polygon, not the chord.
    const auto velocity_points = trajectory.velocity_traj_.getControlPoint();
    double max_speed = 0.0;
    for (Eigen::Index i = 0; i < velocity_points.cols(); ++i)
      max_speed = std::max(max_speed, velocity_points.col(i).norm());
    const double step = std::min(0.02, planner_manager_->grid_map_->getResolution() /
                                        (2.0 * std::max(0.01, max_speed)));
    if (!std::isfinite(step) || step < 1e-5) return false;
    const auto deadline = PlanningBudget::Clock::now() + std::chrono::milliseconds(10);
    for (double t = begin; ; t = std::min(end, t + step))
    {
      if (PlanningBudget::Clock::now() >= deadline ||
          (use_planning_budget && planning_budget_ && planning_budget_->expired()))
        return false;
      if (planner_manager_->grid_map_->getInflateOccupancy(
            trajectory.position_traj_.evaluateDeBoorT(t)) != 0)
        return false;
      if (t >= end) break;
    }
    return true;
  }

  void EGOReplanFSM::cancelPendingTrajectory()
  {
    if (!pending_traj_) return;
    std_msgs::msg::Int64 cancel;
    cancel.data = pending_traj_->traj_id_;
    cancel_pending_pub_->publish(cancel);
    planner_manager_->global_data_.last_progress_time_ = pending_previous_progress_;
    local_target_pt_ = pending_previous_target_;
    local_target_vel_ = pending_previous_velocity_;
    pending_traj_.reset();
    pending_accepted_ = pending_commit_sent_ = pending_committed_ = false;
    pending_rejected_ = pending_startup_pub_ = false;
  }

  void EGOReplanFSM::processHandover()
  {
    if (bridge_hold_latched_ || !pending_traj_) return;//改 if (!pending_traj_) return;
    const auto now = node_->now();
    const double remaining = (pending_traj_->start_time_ - now).seconds();
    if (pending_rejected_ || (!pending_committed_ && remaining <= 0.01))
    {
      RCLCPP_ERROR(node_->get_logger(), "Trajectory handover rejected or acknowledgement expired");
      cancelPendingTrajectory();
      if (planner_manager_->local_data_.traj_id_ > 0)
      {
        flag_escape_emergency_ = true;
        changeFSMExecState(EMERGENCY_STOP, "HANDOVER");
      }
      else
        changeFSMExecState(GEN_NEW_TRAJ, "HANDOVER");
      next_plan_attempt_ = PlanningBudget::Clock::now() +
          std::chrono::duration_cast<PlanningBudget::Clock::duration>(
              std::chrono::duration<double>(retry_delay_sec_));
      return;
    }
    if (pending_accepted_ && !pending_commit_sent_)
    {
      auto &old = planner_manager_->local_data_;
      const bool prefix_clear = old.traj_id_ == 0 || trajectoryClear(old,
          (now - old.start_time_).seconds(),
          (pending_traj_->start_time_ - old.start_time_).seconds());
      if (remaining < min_publish_margin_sec_ || !prefix_clear ||
          !trajectoryClear(*pending_traj_, 0.0, pending_traj_->duration_ * 2.0 / 3.0))
      {
        pending_rejected_ = true;
        return;
      }
      std_msgs::msg::Int64 commit;
      commit.data = pending_traj_->traj_id_;
      pending_commit_sent_ = true;
      commit_pub_->publish(commit);
    }
    if (pending_committed_ && remaining <= 0.0)
    {
      const bool startup = pending_startup_pub_;
      planner_manager_->local_data_ = *pending_traj_;
      pending_traj_.reset();
      pending_accepted_ = pending_commit_sent_ = pending_committed_ = false;
      pending_rejected_ = pending_startup_pub_ = false;
      have_new_target_ = false;
      publishSwarmTrajs(startup);
      RCLCPP_INFO(node_->get_logger(), "Activated trajectory %d at scheduled boundary",
                  planner_manager_->local_data_.traj_id_);
    }
  }

  void EGOReplanFSM::checkCollisionCallback()
  {
    if (bridge_hold_latched_ ||
    recovery_phase_ != RecoveryPhase::IDLE) return;//加
    processHandover();

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;
    
    if (exec_state_ == WAIT_TARGET ||
        (info->traj_id_ == 0 && !pending_traj_))
      return;

    /* ---------- check lost of depth ---------- */
    //if (map->getOdomDepthTimeout())
    //{
    //  RCLCPP_ERROR(node_->get_logger(), "Depth Lost! EMERGENCY_STOP");

    //  enable_fail_safe_ = false;
    //  changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    //}
      if (map->getOdomDepthTimeout())
      {
        RCLCPP_ERROR(node_->get_logger(), "Depth Lost! EMERGENCY_STOP");

        enable_fail_safe_ = false;
        changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        return;
      }

      // 急停的执行与恢复由 execFSMCallback 处理，
      // 此处不再沿旧轨迹反复调用重规划。
      if (exec_state_ == EMERGENCY_STOP)
      {
        return;
      }


    if (pending_traj_ && !trajectoryClear(*pending_traj_, 0.0,
                                         pending_traj_->duration_ * 2.0 / 3.0))
    {
      cancelPendingTrajectory();
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "PENDING_SAFETY");
      return;
    }
    if (info->traj_id_ == 0) return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    // double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_cur = (node_->now() - info->start_time_).seconds();

    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);
    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    // double t_cur_global = ros::Time::now().toSec();
    double t_cur_global = node_->now().seconds();

    const double old_end = pending_traj_ ? std::min(info->duration_,
        (pending_traj_->start_time_ - info->start_time_).seconds()) : info->duration_;
    double t_2_3 = pending_traj_ ? old_end : info->duration_ * 2 / 3;
    for (double t = t_cur; t < old_end; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      bool occ = false;
      occ |= map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t));

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.seconds();
        Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
        double dist = (p_cur - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          break;
        }
      }

      if (occ)
      {

        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          publishSwarmTrajs(false);
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);

            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            RCLCPP_WARN(node_->get_logger(), "current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    if (!planning_budget_ || planning_budget_->expired() || pending_traj_)
      return false;
    const double previous_progress = planner_manager_->global_data_.last_progress_time_;
    const Eigen::Vector3d previous_target = local_target_pt_;
    const Eigen::Vector3d previous_velocity = local_target_vel_;
    getLocalTarget();
    LocalTrajData candidate;
    bool success = planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_,
        local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, scheduled_start_, candidate);
    success = success && !planning_budget_->expired() &&
        (scheduled_start_ - node_->now()).seconds() >= min_publish_margin_sec_;
    if (success)
    {
      success = (candidate.start_pos_ - start_pt_).norm() <= 0.03 &&
          (candidate.velocity_traj_.evaluateDeBoorT(0.0) - start_vel_).norm() <= 0.05 &&
          (candidate.acceleration_traj_.evaluateDeBoorT(0.0) - start_acc_).norm() <= 0.20 &&
          trajectoryClear(candidate, 0.0, candidate.duration_ * 2.0 / 3.0, true);
      auto &old = planner_manager_->local_data_;
      if (success && old.traj_id_ > 0)
        success = trajectoryClear(old, (node_->now() - old.start_time_).seconds(),
                                  (scheduled_start_ - old.start_time_).seconds(), true);
    }
    if (!success || planning_budget_->expired())
    {
      planner_manager_->global_data_.last_progress_time_ = previous_progress;
      local_target_pt_ = previous_target;
      local_target_vel_ = previous_velocity;
      return false;
    }
    pending_previous_progress_ = previous_progress;
    pending_previous_target_ = previous_target;
    pending_previous_velocity_ = previous_velocity;
    pending_traj_ = candidate;
    pending_accepted_ = pending_commit_sent_ = pending_committed_ = false;
    pending_rejected_ = pending_startup_pub_ = false;

    traj_utils::msg::Bspline msg;
    msg.drone_id = planner_manager_->pp_.drone_id;
    msg.order = 3;
    msg.start_time = candidate.start_time_;
    msg.traj_id = candidate.traj_id_;
    const auto points = candidate.position_traj_.getControlPoint();
    for (Eigen::Index i = 0; i < points.cols(); ++i)
    {
      geometry_msgs::msg::Point p;
      p.x = points(0, i); p.y = points(1, i); p.z = points(2, i);
      msg.pos_pts.push_back(p);
    }
    const auto knots = candidate.position_traj_.getKnot();
    msg.knots.assign(knots.data(), knots.data() + knots.size());
    bspline_pub_->publish(msg);
    visualization_->displayOptimalList(points, 0);
    return true;
  }

  void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    if (pending_traj_)
    {
      pending_startup_pub_ = pending_startup_pub_ || startup_pub;
      return;
    }
    auto info = &planner_manager_->local_data_;

    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();

    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(bspline);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
        // return plan_and_refine_success;
      }
      // swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
      swarm_trajs_pub_->publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_->publish(bspline);
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    cancelPendingTrajectory();

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    emergency_bspline_pub_->publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // Important cornor case!
        for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();
          if (dist_temp < planning_horizen_)
          {
            pos_t = pos_t_temp;
            dist = (pos_t - start_pt_).norm();
            cout << "Escape cornor case \"getLocalTarget\"" << endl;
            break;
          }
        }
      }

      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }

      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      planner_manager_->global_data_.last_progress_time_ = planner_manager_->global_data_.global_duration_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }
  }

} // namespace ego_planner
