#!/usr/bin/env python3
"""
position_cmd_to_px4.py
======================
EGO-Planner  ->  PX4 的桥接节点（ENU world  ->  NED local）

数据流：
    /drone_0_planning/pos_cmd      (quadrotor_msgs/PositionCommand, ENU, 10Hz)
        --> ENU->NED 转换
        --> /fmu/in/trajectory_setpoint      (px4_msgs/TrajectorySetpoint)
        --> /fmu/in/offboard_control_mode    (20Hz 心跳)

控制流程：
    ego 模式：TAKEOFF -> WAIT_EGO -> TRACK；可恢复故障进入 BRAKE。
    明确的规划器急停：TRACK -> PLANNER_BRAKE -> TRACK；最多等待 3s。
    必须收到匹配急停 ID 的 server 指令，且实际稳定后才接管新 active 轨迹。
    PLANNER_BRAKE 暂缓软误差判定，仍保留硬误差和输入超时保护。
    可恢复故障：BRAKE -> WAIT_RECOVERY -> TRACK，需 FSM/server 协商新轨迹。
    没有配套 FSM 时不会自行前飞；本进程最多三次恢复尝试，之后终止 HOLD。
    到达起飞高度后由用户发送 /traj_start_trigger，桥不自动触发规划。
    fixed_height 是相对首次有效位置的上升距离，不是绝对高度。
    上锁或离开已接管的 OFFBOARD 后停止接管，需从地面重启节点。
    feedback_timeout 检查位置（默认 0.5s）；status_timeout 检查状态（默认 3s）。
    真正反馈丢失时停止控制，由 PX4 配置的 OFFBOARD 丢失动作处理。

运行（务必用系统 python3.10，不要 activate conda）：
    conda deactivate 2>/dev/null || true
    source /opt/ros/humble/setup.bash
    source ~/hw-ros2/ros2/install/setup.bash
    python3 position_cmd_to_px4.py --ros-args -p mode:=ego -p fixed_height:=5.0
    起飞稳定后手动发布 /traj_start_trigger；只允许一个桥发布 PX4 控制指令。
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleStatus, VehicleCommandAck,
)
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int64MultiArray, Empty
from traj_utils.msg import Bspline

# -------------------------------------------------------------
# PX4 uXRCE-DDS 要求的 QoS：BEST_EFFORT + TRANSIENT_LOCAL + depth=1
# 必须和 hw_insight/offboard.py 完全一致，
# 否则订阅/发布都会"发了没反应"
# -------------------------------------------------------------
def px4_qos() -> QoSProfile:
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    return qos


class PositionCmdToPx4(Node):

    TAKEOFF, WAIT_EGO, TRACK, HOLD = range(4)
    PLANNER_BRAKE = 4
    BRAKE, WAIT_RECOVERY = 5, 6

    # 状态机
    S_IDLE, S_STREAM, S_OFFBOARD, S_ACTIVE = 0, 1, 2, 3

    def __init__(self):
        super().__init__('position_cmd_to_px4')

        # ---------------- 参数 ----------------
        self.declare_parameter('mode', 'fixed')          # 'fixed' | 'ego'
        self.declare_parameter('auto_arm', True)         # 是否由本节点解锁
        self.declare_parameter('fixed_height', 2.0)      # 两种模式均相对首次有效位置上升 [m]
        self.declare_parameter('cmd_timeout', 0.2)       # pos_cmd 超时 [s]
        self.declare_parameter('publish_rate', 20.0)     # 心跳频率 [Hz]
        self.declare_parameter('target_system', 1)
        self.declare_parameter('target_component', 1)
        self.declare_parameter('yaw_enu_to_ned', True)   # ENU->NED 的 yaw 变换
        # 本地 PX4 1.15.2 uCDR 对 timestamp=0 使用 hrt_absolute_time()。
        # 保留 ros 默认值；仅对已核实接收语义的 PX4 启用接收端时间戳。
        self.declare_parameter('px4_timestamp_mode', 'ros')
        self.px4_timestamp_mode_ = self.get_parameter('px4_timestamp_mode').value
        if self.px4_timestamp_mode_ not in ('ros', 'receiver'):
            raise ValueError('px4_timestamp_mode 必须为 ros 或 receiver')

        for name, value in [('takeoff_tolerance', 0.2), ('settle_speed', 0.15),
                            ('settle_time', 2.0), ('handover_max_error', 0.3),
                            ('feedback_timeout', 0.5), ('status_timeout', 3.0)]:
            self.declare_parameter(name, value)
            setattr(self, name + '_', float(self.get_parameter(name).value))
            if not math.isfinite(getattr(self, name + '_')) or getattr(self, name + '_') <= 0:
                raise ValueError(name + ' 必须为有限正数')

        # C：跟踪保护。当前数值用于仿真校准，不是最终安全认证参数。
        self.declare_parameter('tracking_guard_enabled', True)
        self.tracking_guard_enabled_ = bool(
            self.get_parameter('tracking_guard_enabled').value)

        for name, value in [
                ('tracking_warn_error', 0.15),
                ('tracking_stop_error', 0.20),
                ('tracking_hard_error', 0.30),
                ('tracking_warn_time', 0.20),
                ('tracking_stop_time', 0.10),
                ('tracking_gap_reset', 0.15)]:
            self.declare_parameter(name, value)
            number = float(self.get_parameter(name).value)
            if not math.isfinite(number) or number <= 0:
                raise ValueError(name + ' 必须为有限正数')
            setattr(self, name + '_', number)

        if not (
                self.tracking_warn_error_
                < self.tracking_stop_error_
                < self.tracking_hard_error_):
            raise ValueError('跟踪阈值必须满足 warn < stop < hard')

        for name, value in [('planner_brake_timeout', 3.0),
                            ('planner_brake_settle_time', 0.15),
                            ('planner_brake_settle_speed', 0.10),
                            ('clock_jump_tolerance', 0.05),
                            ('clock_stable_time', 2.0),
                            ('reference_position_tolerance', 0.05),
                            ('reference_velocity_tolerance', 0.25),
                            ('recovery_settle_time', 0.30),
                            ('recovery_timeout', 15.0)]:
            self.declare_parameter(name, value)
            number = float(self.get_parameter(name).value)
            if not math.isfinite(number) or number <= 0:
                raise ValueError(name + ' 必须为有限正数')
            setattr(self, name + '_', number)
        self.declare_parameter('auto_recovery_enabled', True)
        self.auto_recovery_enabled_ = bool(self.get_parameter('auto_recovery_enabled').value)
        self.declare_parameter('recovery_max_attempts', 3)
        attempts = self.get_parameter('recovery_max_attempts').value
        if isinstance(attempts, bool) or not isinstance(attempts, int) or not 1 <= attempts <= 10:
            raise ValueError('recovery_max_attempts 必须为 1 到 10 的整数')
        self.recovery_max_attempts_ = attempts

        self.mode_ = self.get_parameter('mode').value
        self.auto_arm_ = bool(self.get_parameter('auto_arm').value)
        self.fixed_height_ = float(self.get_parameter('fixed_height').value)
        self.cmd_timeout_ = float(self.get_parameter('cmd_timeout').value)
        if self.mode_ not in ('fixed', 'ego'):
            raise ValueError('mode 必须是 fixed 或 ego')
        if not math.isfinite(self.fixed_height_) or self.fixed_height_ <= 0:
            raise ValueError('fixed_height 必须为有限正数')
        if not math.isfinite(self.cmd_timeout_) or self.cmd_timeout_ <= 0:
            raise ValueError('cmd_timeout 必须为有限正数')
        rate = float(self.get_parameter('publish_rate').value)
        if not math.isfinite(rate) or rate <= 2.0:
            raise ValueError('publish_rate 必须大于 2 Hz')
        self.prestream_count_ = math.ceil(2.0 * rate)
        self.tgt_sys_ = int(self.get_parameter('target_system').value)
        self.tgt_comp_ = int(self.get_parameter('target_component').value)
        self.yaw_xform_ = bool(self.get_parameter('yaw_enu_to_ned').value)

        # ---------------- 状态 ----------------
        self.state_ = self.S_IDLE
        self.stream_count_ = 0
        self.arm_sent_ = False        # ARM 指令是否已发（用标志而非帧计数）
        self.offb_sent_ = False       # DO_SET_MODE(OFFBOARD) 是否已发
        self.offb_wait_count_ = 0     # 等待进入 OFFBOARD 的周期计数（仅用于诊断）

        self.have_local_pos_ = False
        self.local_ = VehicleLocalPosition()          # NED
        self.armed_ = False
        self.nav_state_ = 0

        self.have_cmd_ = False
        self.last_cmd_time_ = self.get_clock().now()
        self.traj_id_ = 0

        # 待发布的 setpoint（始终是 NED），None 表示"还没准备好，先跟当前位姿"
        self.sp_ = None

        # 固定航点（fixed 模式下第一次收到 local_position 时确定）
        self.fixed_sp_ = None
        self.fixed_yaw_ = 0.0
        self.phase_ = self.TAKEOFF
        self.settle_since_ = None
        self.last_local_time_ = None
        self.last_status_time_ = None
        self.last_cmd_monotonic_ = None
        self.wait_stamp_ns_ = None
        self.trigger_stamp_ns_ = None
        self.hold_sp_ = None
        self.inhibited_ = False
        self.ever_active_ = False
        self.last_timer_monotonic_ = None
        self.last_ocm_monotonic_ = None
        self.max_ocm_gap_ = 0.0

        self.tracking_warn_since_ = None
        self.tracking_stop_since_ = None
        self.tracking_last_check_ = None
        self.tracking_last_feedback_ = None

        # 记录实际调用发布接口发送的参考，不能拿尚未发送的指令比较。
        self.last_sent_sp_ = None
        self.last_sent_monotonic_ = None
        self.emergency_token_ = None
        self.emergency_highest_id_ = 0
        self.brake_started_ = None
        self.brake_settled_since_ = None
        self.recovery_id_ = None
        self.recovery_first_seen_ = None
        self.clock_sample_ = (self.get_clock().now().nanoseconds, time.monotonic())
        self.clock_stable_since_ = self.clock_sample_[1]
        self.last_reference_ = None
        self.highest_cmd_id_ = 0
        self.recovery_epoch_ = time.time_ns()
        self.recovery_attempts_ = 0  # 本进程总次数；成功接管不清零，避免无限重试。
        self.recovery_floor_ = 0
        self.recovery_offer_id_ = None
        self.recovery_started_at_ = None
        self.recovery_settled_since_ = None
        self.recovery_ready_stamp_ = None
        self.recovery_cmd_first_ = {}
        self.recovery_tracking_id_ = None

        qos = px4_qos()

        # ---------------- 发布 ----------------
        self.pub_ocm_ = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_tsp_ = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_vcmd_ = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # 向 FSM 通知桥已进入保持；周期发布，同时保留最后状态。
        self.pub_safety_hold_ = self.create_publisher(
            Bool,
            '/drone_0_planning/safety_hold',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # 所有恢复消息 data=[epoch, trajectory_id]；request/stopped 的第二项为旧 ID 上界。
        recovery_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for suffix in ('request', 'stopped', 'accept', 'tracking'):
            setattr(self, 'pub_recovery_' + suffix + '_', self.create_publisher(
                Int64MultiArray, '/drone_0_planning/recovery_' + suffix, recovery_qos))
        # ---------------- 订阅 ----------------
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.cb_local_pos, qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.cb_status, qos)
        self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack',
            self.cb_command_ack, qos)

        if self.mode_ == 'ego':
            self.create_subscription(Empty, '/drone_0_planning/recovery_needed',
                                     self.cb_recovery_needed, 10)
            self.create_subscription(Int64MultiArray, '/drone_0_planning/recovery_offer',
                                     self.cb_recovery_offer, recovery_qos)
            self.create_subscription(PoseStamped, '/traj_start_trigger', self.cb_trigger, 10)
            self.create_subscription(
                PositionCommand, '/drone_0_planning/pos_cmd',
                self.cb_pos_cmd, 10)
            self.create_subscription(
                Bspline, '/drone_0_planning/emergency_bspline',
                self.cb_emergency_bspline, 10)

        # ---------------- 主循环 ----------------
        # 心跳周期不应随系统时钟校准或 ROS 时间跳变暂停。
        self.control_clock_ = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer_ = self.create_timer(1.0 / rate, self.on_timer, clock=self.control_clock_)

        self.get_logger().info(
            'position_cmd_to_px4 启动: mode=%s, auto_arm=%s, fixed_height=%s, '
            'rate=%sHz, cmd_timeout=%ss'
            % (self.mode_, self.auto_arm_, self.fixed_height_,
               rate, self.cmd_timeout_))
        self.get_logger().info(
            '控制定时器=STEADY_TIME，PX4 时间戳模式=%s' % self.px4_timestamp_mode_)
        self.get_logger().info(
            '桥保护版本=recovery-v2，自动恢复=%s；需要 FSM/server 恢复握手配套'
            % self.auto_recovery_enabled_)

        if self.mode_ == 'fixed':
            self.get_logger().warn(
                'fixed 模式：仅悬停。收到 local_position 后锁定为当前 XY + '
                '%.1fm 高度（ENU 向上）；本轮只执行固定悬停'
                % self.fixed_height_)

    # =================================================================
    # 按当前 ROS 时钟发送微秒时间戳，配合现有 PX4 DDS 时间同步
    # =================================================================
    def _us(self) -> int:
        return int(self.get_clock().now().nanoseconds // 1000)

    def _px4_timestamp(self) -> int:
        """仅用于 PX4 输入；EGO 指令新鲜度仍使用 ROS 时间检查。"""
        return 0 if self.px4_timestamp_mode_ == 'receiver' else self._us()

    # =================================================================
    # 回调
    # =================================================================
    def cb_local_pos(self, msg: VehicleLocalPosition):
        self.local_ = msg
        self.last_local_time_ = time.monotonic()
        self.have_local_pos_ = (
            msg.xy_valid and msg.z_valid and msg.v_xy_valid and msg.v_z_valid
            and all(math.isfinite(v) for v in
                    (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.heading)))
        if not self.have_local_pos_:
            return

        self._check_tracking_error()

        # 用每条有效反馈观察速度，避免仅依赖 20Hz 定时器判断已稳定。
        self._check_planner_brake()
        self._update_recovery()

        if self.fixed_sp_ is None:
            # 首次收到位置：记录固定航点（当前 XY + 抬升 fixed_height）
            # NED 里"向上"= z 减小
            self.fixed_sp_ = (msg.x, msg.y, msg.z - self.fixed_height_)
            self.fixed_yaw_ = msg.heading
            self.get_logger().info(
                'fixed 悬停点锁定 (NED): (%.2f, %.2f, %.2f)  '
                '<- 当前 z=%.2f 抬升 %.2fm'
                % (self.fixed_sp_[0], self.fixed_sp_[1], self.fixed_sp_[2],
                   msg.z, self.fixed_height_))

    def cb_status(self, msg: VehicleStatus):
        self.last_status_time_ = time.monotonic()
        if (not self.inhibited_ and self.armed_
                and msg.arming_state != VehicleStatus.ARMING_STATE_ARMED):
            self.inhibited_ = True
            self.get_logger().warn('PX4 已上锁；禁止自动再次起飞，请从地面重启桥')
        if (not self.inhibited_ and self.ever_active_
                and msg.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.inhibited_ = True
            now = time.monotonic()
            ocm_age = (now - self.last_ocm_monotonic_
                       if self.last_ocm_monotonic_ is not None else float('inf'))
            self.get_logger().warn(
                '已离开 OFFBOARD：nav_state=%d->%d, intention=%d, '
                'failsafe=%s, arming_state=%d, status_timestamp=%d, '
                'nav_state_timestamp=%d, status_ros_age=%.3fs, '
                'last_heartbeat_age=%.3fs, max_publish_gap=%.3fs；'
                '停止接管，不自动抢回模式（发布间隔仅表示本进程调用，不代表 PX4 已收到）'
                % (self.nav_state_, msg.nav_state, msg.nav_state_user_intention,
                   msg.failsafe, msg.arming_state, msg.timestamp,
                   msg.nav_state_timestamp, self._us() / 1e6 - msg.timestamp / 1e6,
                   ocm_age, self.max_ocm_gap_))
        self.armed_ = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state_ = msg.nav_state

    def cb_trigger(self, msg):
        if self.phase_ == self.WAIT_EGO:
            self.trigger_stamp_ns_ = self.get_clock().now().nanoseconds
            self.get_logger().info('收到规划触发，等待新鲜且连续的 EGO 指令')

    def _feedback_valid(self):
        now = time.monotonic()
        return (self.have_local_pos_ and self.last_local_time_ is not None
                and self.last_status_time_ is not None
                and now - self.last_local_time_ <= self.feedback_timeout_
                and now - self.last_status_time_ <= self.status_timeout_)

    def _publish_safety_hold(self):
        msg = Bool()
        msg.data = self.phase_ == self.HOLD or self.inhibited_
        self.pub_safety_hold_.publish(msg)

    def _enter_hold(self, reason):
        if (self.auto_recovery_enabled_ and self.mode_ == 'ego'
                and self.phase_ in (self.TRACK, self.PLANNER_BRAKE)):
            self._begin_recovery(reason)
            return
        self._hard_hold(reason)

    def _hard_hold(self, reason):
        if self.phase_ != self.HOLD:
            self.hold_sp_ = {
                'pos': (self.local_.x, self.local_.y, self.local_.z),
                'vel': (0.0, 0.0, 0.0),
                'acc': (0.0, 0.0, 0.0),
                'yaw': self.local_.heading,
            }
            self.phase_ = self.HOLD
            self.sp_ = None
            self._publish_safety_hold()
            self.get_logger().error(
                reason + '；恢复未完成，保持停止，需落地后处理/重启')

    def _publish_recovery(self, suffix, trajectory_id):
        msg = Int64MultiArray()
        msg.data = [self.recovery_epoch_, int(trajectory_id)]
        getattr(self, 'pub_recovery_' + suffix + '_').publish(msg)

    def _check_clock(self):
        ros, mono = self.get_clock().now().nanoseconds, time.monotonic()
        old_ros, old_mono = self.clock_sample_
        self.clock_sample_ = (ros, mono)
        delta = (ros - old_ros) / 1e9 - (mono - old_mono)
        if abs(delta) > self.clock_jump_tolerance_:
            self.clock_stable_since_ = mono
            self.trigger_stamp_ns_ = None
            self.last_reference_ = None
            reason = 'ROS/单调时间变化不一致：%+.6fs，拒绝旧时间轴参考' % delta
            if self.phase_ in (self.TRACK, self.PLANNER_BRAKE, self.BRAKE, self.WAIT_RECOVERY):
                self._begin_recovery(reason)
            else:
                self.get_logger().warn(reason)
            return False
        return mono - self.clock_stable_since_ >= self.clock_stable_time_

    def _begin_recovery(self, reason):
        if self.phase_ == self.HOLD or self.inhibited_:
            return
        if self.recovery_attempts_ >= self.recovery_max_attempts_:
            self._hard_hold(reason + '；已达到本轮自动恢复次数上限')
            return
        if not self.auto_recovery_enabled_:
            self._hard_hold(reason)
            return
        self.recovery_attempts_ += 1
        self.recovery_epoch_ += 1
        self.recovery_floor_ = max(self.highest_cmd_id_, self.traj_id_,
                                   self.recovery_offer_id_ or 0)
        self.recovery_offer_id_ = None
        self.recovery_tracking_id_ = None
        self.recovery_started_at_ = time.monotonic()
        self.recovery_settled_since_ = None
        self.recovery_ready_stamp_ = None
        self.recovery_cmd_first_.clear()
        self.hold_sp_ = {'pos': (self.local_.x, self.local_.y, self.local_.z),
                         'vel': (0.0, 0.0, 0.0), 'acc': (0.0, 0.0, 0.0),
                         'yaw': self.local_.heading}
        self.sp_ = None
        self.last_reference_ = None
        self.phase_ = self.BRAKE
        self._publish_recovery('request', self.recovery_floor_)
        self._publish_safety_hold()
        self.get_logger().warn(
            '%s；进入 BRAKE，恢复 epoch=%d，尝试 %d/%d；等待 FSM 新轨迹握手'
            % (reason, self.recovery_epoch_, self.recovery_attempts_, self.recovery_max_attempts_))

    def _update_recovery(self):
        if self.phase_ not in (self.BRAKE, self.WAIT_RECOVERY):
            return
        now = time.monotonic()
        if now - self.recovery_started_at_ >= self.recovery_timeout_:
            self._begin_recovery('恢复握手超时；检查 FSM/server 是否已配套及地图是否有效')
            return
        if not self._feedback_valid():
            self.recovery_settled_since_ = None
            return
        speed = math.sqrt(self.local_.vx ** 2 + self.local_.vy ** 2 + self.local_.vz ** 2)
        settled = (speed <= self.planner_brake_settle_speed_
                   and now - self.clock_stable_since_ >= self.clock_stable_time_)
        if not settled:
            self.recovery_settled_since_ = None
            if self.phase_ == self.WAIT_RECOVERY:
                # 已发 stopped 后重新移动：旧 offer/accept 全部作废。
                self._begin_recovery('静止接管条件失效')
            return
        if self.recovery_settled_since_ is None:
            self.recovery_settled_since_ = now
        if (self.phase_ == self.BRAKE
                and now - self.recovery_settled_since_ >= self.recovery_settle_time_):
            # 刹停位移后以已停稳的实际位置为接管基准，避免继续追赶故障瞬间的位置。
            self.hold_sp_['pos'] = (self.local_.x, self.local_.y, self.local_.z)
            self.hold_sp_['yaw'] = self.local_.heading
            self.phase_ = self.WAIT_RECOVERY
            self.recovery_ready_stamp_ = self.get_clock().now().nanoseconds
            self.get_logger().info('BRAKE -> WAIT_RECOVERY：静止且时钟稳定，等待新轨迹')

    def cb_recovery_needed(self, _msg):
        if self.inhibited_ or not self._feedback_valid() or not self._check_clock():
            return
        if (self.phase_ in (self.TRACK, self.PLANNER_BRAKE)
                or (self.phase_ == self.WAIT_EGO and self.trigger_stamp_ns_ is not None)):
            self._begin_recovery('规划器请求停稳后重新规划')

    def cb_recovery_offer(self, msg):
        if self.inhibited_ or not self._check_clock():
            return
        self._update_recovery()
        if (self.phase_ != self.WAIT_RECOVERY or not self._feedback_valid()
                or not self.armed_ or self.nav_state_ != VehicleStatus.NAVIGATION_STATE_OFFBOARD
                or len(msg.data) != 2 or msg.data[0] != self.recovery_epoch_
                or not self.recovery_floor_ < msg.data[1] <= 2147483647
                or (self.recovery_offer_id_ is not None
                    and msg.data[1] < self.recovery_offer_id_)):
            return
        self.recovery_offer_id_ = int(msg.data[1])
        self._publish_recovery('accept', self.recovery_offer_id_)

    def _accept_recovery_command(self, msg, sp, stamp_ns):
        self._update_recovery()
        if self.phase_ != self.WAIT_RECOVERY or msg.trajectory_id != self.recovery_offer_id_:
            return False
        now = time.monotonic()
        first = self.recovery_cmd_first_.get(msg.trajectory_id, now)
        if (stamp_ns <= self.recovery_ready_stamp_ or now - first > self.cmd_timeout_
                or math.dist(sp['pos'], (self.local_.x, self.local_.y, self.local_.z))
                    > min(self.tracking_warn_error_, self.handover_max_error_)
                or math.dist(sp['vel'], (0, 0, 0)) > self.planner_brake_settle_speed_
                or math.dist(sp['acc'], (0, 0, 0)) > 0.5):
            return False
        self.phase_ = self.TRACK
        self.last_reference_ = None
        self.tracking_warn_since_ = self.tracking_stop_since_ = None
        self.tracking_last_check_ = self.tracking_last_feedback_ = None
        self.recovery_tracking_id_ = msg.trajectory_id
        self._publish_safety_hold()
        self._publish_recovery('tracking', msg.trajectory_id)
        self.get_logger().info('WAIT_RECOVERY -> TRACK：epoch=%d，新轨迹=%d'
                               % (self.recovery_epoch_, msg.trajectory_id))
        return True

    def _reference_continuous(self, sp):
        if self.last_reference_ is None:
            return True
        previous, when = self.last_reference_
        dt = time.monotonic() - when
        if not 0 <= dt <= self.cmd_timeout_:
            return False
        predicted = tuple(p + v * dt + 0.5 * a * dt * dt
                          for p, v, a in zip(previous['pos'], previous['vel'], previous['acc']))
        velocity = tuple(v + a * dt for v, a in zip(previous['vel'], previous['acc']))
        return (math.dist(sp['pos'], predicted) <= self.reference_position_tolerance_
                and math.dist(sp['vel'], velocity) <= self.reference_velocity_tolerance_)

    def _check_tracking_error(self):
        if (
                not self.tracking_guard_enabled_
                or self.phase_ not in (self.TRACK, self.PLANNER_BRAKE)
                or not self._feedback_valid()
                or self.last_sent_sp_ is None):
            self.tracking_warn_since_ = None
            self.tracking_stop_since_ = None
            self.tracking_last_check_ = None
            self.tracking_last_feedback_ = None
            return

        now = time.monotonic()

        if (
                self.last_sent_monotonic_ is None
                or now - self.last_sent_monotonic_ > self.cmd_timeout_):
            self._enter_hold('已发送参考指令长时间未更新')
            return

        # 同一条反馈不重复累计持续时间。
        if self.tracking_last_feedback_ == self.last_local_time_:
            return
        self.tracking_last_feedback_ = self.last_local_time_

        # 观察存在较大空档时，不能把空档当作持续超限证据。
        if (
                self.tracking_last_check_ is not None
                and now - self.tracking_last_check_
                > self.tracking_gap_reset_):
            self.tracking_warn_since_ = None
            self.tracking_stop_since_ = None
        self.tracking_last_check_ = now

        # 两者均为 PX4 本地 NED，避免混用 AirSim header 时间。
        error = math.dist(
            self.last_sent_sp_['pos'],
            (self.local_.x, self.local_.y, self.local_.z))

        if (
                not math.isfinite(error)
                or error >= self.tracking_hard_error_):
            self._enter_hold(
                '跟踪位置误差达到硬阈值：%.3fm' % error)
            return

        if self.phase_ == self.PLANNER_BRAKE:
            # 规划器把参考瞬间改为常值时，刹停位移不是普通轨迹跟踪误差。
            # 只暂缓软阈值；硬阈值、反馈/指令新鲜度和总等待期限仍有效。
            self.tracking_warn_since_ = None
            self.tracking_stop_since_ = None
            return

        if error >= self.tracking_stop_error_:
            if self.tracking_stop_since_ is None:
                self.tracking_stop_since_ = now

            if (
                    now - self.tracking_stop_since_
                    >= self.tracking_stop_time_):
                self._enter_hold(
                    '跟踪位置误差持续超限：%.3fm' % error)
                return
        else:
            self.tracking_stop_since_ = None

        if error >= self.tracking_warn_error_:
            if self.tracking_warn_since_ is None:
                self.tracking_warn_since_ = now

            if (
                    now - self.tracking_warn_since_
                    >= self.tracking_warn_time_):
                self.get_logger().warn(
                    '跟踪位置误差 %.3fm' % error,
                    throttle_duration_sec=1.0)
        else:
            self.tracking_warn_since_ = None

    def cb_command_ack(self, msg: VehicleCommandAck):
        if msg.command in (VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                           VehicleCommand.VEHICLE_CMD_DO_SET_MODE):
            self.get_logger().info(
                'PX4 command ACK: command=%d result=%d（0=接受）'
                % (msg.command, msg.result))

    def cb_emergency_bspline(self, msg):
        """只登记明确的常值急停；必须再与 server 输出的 pos_cmd 匹配。"""
        if (self.inhibited_ or self.phase_ != self.TRACK
                or not self._feedback_valid()):
            return
        points = [(p.y, p.x, -p.z) for p in msg.pos_pts]
        knots = list(msg.knots)
        stamp = msg.start_time.sec * 1000000000 + msg.start_time.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if (msg.order != 3 or not 6 <= len(points) <= 1024
                or len(knots) != len(points) + 4
                or not 0 < msg.traj_id <= 2147483647
                or msg.traj_id <= self.emergency_highest_id_
                or msg.traj_id < self.traj_id_ or abs(age) > 0.5
                or not all(math.isfinite(v) for p in points for v in p)
                or not all(math.isfinite(k) for k in knots)
                or any(b <= a for a, b in zip(knots, knots[1:]))
                or not 0 < knots[-4] - knots[3] <= 600
                or any(math.dist(p, points[0]) > 1e-6 for p in points)):
            return
        self.emergency_highest_id_ = msg.traj_id
        self.emergency_token_ = (msg.traj_id, points[0], time.monotonic())
        # DDS 不同话题之间没有顺序保证。
        if self.sp_ is not None:
            self._confirm_planner_brake(self.traj_id_, self.sp_)

    def _confirm_planner_brake(self, trajectory_id, sp):
        token = self.emergency_token_
        if (self.phase_ != self.TRACK or token is None
                or trajectory_id != token[0]
                or time.monotonic() - token[2] > 0.5
                or math.dist(sp['pos'], token[1]) > 1e-5
                or math.dist(sp['vel'], (0, 0, 0)) > 1e-6
                or math.dist(sp['acc'], (0, 0, 0)) > 1e-6):
            return
        self.phase_ = self.PLANNER_BRAKE
        self.brake_started_ = time.monotonic()
        self.brake_settled_since_ = None
        self.recovery_id_ = None
        self.recovery_first_seen_ = None
        self.tracking_warn_since_ = self.tracking_stop_since_ = None
        self.get_logger().warn(
            'TRACK -> PLANNER_BRAKE：确认急停轨迹 %d；最多等待 %.2fs，保留硬误差保护'
            % (trajectory_id, self.planner_brake_timeout_))

    def _check_planner_brake(self):
        if self.phase_ != self.PLANNER_BRAKE:
            return
        now = time.monotonic()
        if now - self.brake_started_ >= self.planner_brake_timeout_:
            self._enter_hold('规划器急停后未在期限内完成新轨迹接管')
            return
        speed = math.sqrt(self.local_.vx ** 2 + self.local_.vy ** 2 + self.local_.vz ** 2)
        if speed <= self.planner_brake_settle_speed_:
            if self.brake_settled_since_ is None:
                self.brake_settled_since_ = now
        else:
            self.brake_settled_since_ = None

    def _accept_after_brake(self, trajectory_id, sp):
        self._check_planner_brake()
        if self.phase_ != self.PLANNER_BRAKE:
            return False
        stop_id = self.emergency_token_[0]
        if trajectory_id == stop_id:
            # 同一急停 ID 不能被换成移动指令，也不能无限延长等待期限。
            return (math.dist(sp['pos'], self.emergency_token_[1]) <= 1e-5
                    and math.dist(sp['vel'], (0, 0, 0)) <= 1e-6
                    and math.dist(sp['acc'], (0, 0, 0)) <= 1e-6)
        if trajectory_id < stop_id:
            return False
        now = time.monotonic()
        if self.recovery_id_ is None or trajectory_id > self.recovery_id_:
            self.recovery_id_ = trajectory_id
            self.recovery_first_seen_ = now
        # 只在新 active 轨迹的起步阶段接管；不能追赶已经运行很久的轨迹。
        if (trajectory_id != self.recovery_id_
                or now - self.recovery_first_seen_ > self.cmd_timeout_
                or self.brake_settled_since_ is None
                or now - self.brake_settled_since_ < self.planner_brake_settle_time_
                or math.dist(sp['pos'], (self.local_.x, self.local_.y, self.local_.z))
                    > min(self.tracking_warn_error_, self.handover_max_error_)
                or math.dist(sp['vel'], (0, 0, 0)) > self.planner_brake_settle_speed_
                or math.dist(sp['acc'], (0, 0, 0)) > 0.5
                or math.dist(sp['pos'], self.sp_['pos']) >= self.tracking_hard_error_):
            return False
        self.phase_ = self.TRACK
        self.last_reference_ = None
        self.tracking_warn_since_ = self.tracking_stop_since_ = None
        self.tracking_last_check_ = self.tracking_last_feedback_ = None
        self.get_logger().info(
            'PLANNER_BRAKE -> TRACK：实际速度已稳定，接管新 active 轨迹 %d'
            % trajectory_id)
        return True

    def cb_pos_cmd(self, msg: PositionCommand):
        if not self._check_clock():
            return
        if (self.inhibited_ or not self._feedback_valid()
                or not self.armed_
                or self.nav_state_ != VehicleStatus.NAVIGATION_STATE_OFFBOARD
                or self.phase_ not in (self.WAIT_EGO, self.TRACK, self.PLANNER_BRAKE,
                                       self.BRAKE, self.WAIT_RECOVERY)):
            return
        if self.phase_ == self.WAIT_EGO and self.trigger_stamp_ns_ is None:
            return
        if msg.trajectory_flag != PositionCommand.TRAJECTORY_STATUS_READY or msg.header.frame_id != 'world':
            return
        values = (msg.position.x, msg.position.y, msg.position.z,
                  msg.velocity.x, msg.velocity.y, msg.velocity.z,
                  msg.acceleration.x, msg.acceleration.y, msg.acceleration.z, msg.yaw)
        if not all(math.isfinite(v) for v in values):
            return
        if not 0 < msg.trajectory_id <= 2147483647:
            return
        stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if not 0.0 <= age <= self.cmd_timeout_:
            self.get_logger().warn('拒绝过期或未来时间戳的 EGO 指令', throttle_duration_sec=2.0)
            return
        self.highest_cmd_id_ = max(self.highest_cmd_id_, msg.trajectory_id)
        if self.phase_ == self.WAIT_EGO:
            if stamp_ns <= self.trigger_stamp_ns_:
                return
            distance = math.dist((msg.position.y, msg.position.x, -msg.position.z),
                                 (self.local_.x, self.local_.y, self.local_.z))
            if distance > self.handover_max_error_:
                self.get_logger().warn('首条 EGO 指令距离当前位置 %.2fm，拒绝跳变接管' % distance,
                                       throttle_duration_sec=2.0)
                return
            self.phase_ = self.TRACK
            self.get_logger().info('WAIT_EGO -> TRACK：开始跟踪 EGO')

        # ---------------- ENU(world) -> NED(local) ----------------
        #   x_ned =  y_enu
        #   y_ned =  x_enu
        #   z_ned = -z_enu
        px, py, pz = msg.position.x, msg.position.y, msg.position.z
        vx, vy, vz = msg.velocity.x, msg.velocity.y, msg.velocity.z
        ax, ay, az = msg.acceleration.x, msg.acceleration.y, msg.acceleration.z

        if self.yaw_xform_:
            # ENU yaw: 从东逆时针；NED yaw: 从北顺时针
            yaw_ned = self._wrap_pi(-msg.yaw + math.pi / 2.0)
        else:
            yaw_ned = self._wrap_pi(msg.yaw)

        sp = {
            'pos': (py, px, -pz),
            'vel': (vy, vx, -vz),
            'acc': (ay, ax, -az),
            'yaw': yaw_ned,
        }
        if self.phase_ in (self.BRAKE, self.WAIT_RECOVERY):
            if msg.trajectory_id != self.recovery_offer_id_:
                return
            if msg.trajectory_id not in self.recovery_cmd_first_:
                if len(self.recovery_cmd_first_) >= 128:
                    return
                self.recovery_cmd_first_[msg.trajectory_id] = time.monotonic()
            if not self._accept_recovery_command(msg, sp, stamp_ns):
                return
        if msg.trajectory_id < self.traj_id_:
            return
        # 绝不能先发给 PX4 再检查这条参考是否突然远离实际位置。
        distance = math.dist(sp['pos'], (self.local_.x, self.local_.y, self.local_.z))
        if distance >= self.tracking_hard_error_:
            self._enter_hold('发布前拒绝远离实际位置的参考：%.3fm' % distance)
            return
        if self.phase_ == self.PLANNER_BRAKE:
            if not self._accept_after_brake(msg.trajectory_id, sp):
                return
        else:
            self._confirm_planner_brake(msg.trajectory_id, sp)
        if self.phase_ == self.TRACK and not self._reference_continuous(sp):
            self._enter_hold('发布前拒绝位置/速度不连续或跨越超时的参考')
            return
        self.sp_ = sp
        self.last_cmd_monotonic_ = time.monotonic()
        self.last_reference_ = (sp, self.last_cmd_monotonic_)
        self.have_cmd_ = True
        self.last_cmd_time_ = self.get_clock().now()
        self.traj_id_ = msg.trajectory_id

    @staticmethod
    def _wrap_pi(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    # =================================================================
    # 发送 PX4 指令
    # =================================================================
    def send_vehicle_command(self, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0,
                             p5=0.0, p6=0.0, p7=0.0):
        msg = VehicleCommand()
        msg.timestamp = self._px4_timestamp()
        msg.command = command
        msg.param1, msg.param2, msg.param3, msg.param4 = p1, p2, p3, p4
        msg.param5, msg.param6, msg.param7 = p5, p6, p7
        msg.target_system = self.tgt_sys_
        msg.target_component = self.tgt_comp_
        msg.source_system = 1
        msg.source_component = 1
        msg.confirmation = 0
        msg.from_external = True
        self.pub_vcmd_.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self._px4_timestamp()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.pub_ocm_.publish(msg)
        now = time.monotonic()
        if self.last_ocm_monotonic_ is not None:
            self.max_ocm_gap_ = max(self.max_ocm_gap_, now - self.last_ocm_monotonic_)
        self.last_ocm_monotonic_ = now

    def publish_trajectory_setpoint(self, sp):
        self._check_clock()
        # timer 选好参考后仍可能被校时；发布入口再做一次最终拦截。
        if self.phase_ == self.TRACK and math.dist(
                sp['pos'], (self.local_.x, self.local_.y, self.local_.z)) >= self.tracking_hard_error_:
            self._enter_hold('发布入口拦截超出硬阈值的参考')
        if self.phase_ in (self.BRAKE, self.WAIT_RECOVERY, self.HOLD):
            sp = self.hold_sp_
        if sp is None or self.inhibited_:
            return
        msg = TrajectorySetpoint()
        msg.timestamp = self._px4_timestamp()
        # 位置（NED）
        msg.position = [float(sp['pos'][0]), float(sp['pos'][1]),
                        float(sp['pos'][2])]
        # 速度前馈（NED）
        msg.velocity = [float(sp['vel'][0]), float(sp['vel'][1]),
                        float(sp['vel'][2])]
        # 加速度前馈（NED）
        msg.acceleration = [float(sp['acc'][0]), float(sp['acc'][1]),
                            float(sp['acc'][2])]
        msg.jerk = [float('nan')] * 3
        # yaw 与 yawspeed 同时有效时行为不明确，明确置 NaN 让 PX4 用 yaw
        msg.yaw = float(sp['yaw'])
        msg.yawspeed = float('nan')
        self.pub_tsp_.publish(msg)
        self.last_sent_sp_ = {'pos': tuple(sp['pos'])}
        self.last_sent_monotonic_ = time.monotonic()

    # =================================================================
    # 主循环
    # =================================================================
    def on_timer(self):
        self._check_clock()
        self._update_recovery()
        self._publish_safety_hold()
        if self.phase_ in (self.BRAKE, self.WAIT_RECOVERY):
            self._publish_recovery('request', self.recovery_floor_)
            if self.phase_ == self.WAIT_RECOVERY:
                self._publish_recovery('stopped', self.recovery_floor_)
                if self.recovery_offer_id_ is not None:
                    self._publish_recovery('accept', self.recovery_offer_id_)
        elif self.phase_ == self.TRACK and self.recovery_tracking_id_ is not None:
            self._publish_recovery('tracking', self.recovery_tracking_id_)
        now = time.monotonic()
        if (not self.inhibited_ and self.ever_active_
                and self.last_timer_monotonic_ is not None
                and now - self.last_timer_monotonic_ > 0.2):
            self.get_logger().warn(
                '控制定时器间隔 %.3fs（预期约 %.3fs），检查进程调度/阻塞'
                % (now - self.last_timer_monotonic_, self.timer_.timer_period_ns / 1e9),
                throttle_duration_sec=2.0)
        self.last_timer_monotonic_ = now
        if self.inhibited_:
            return
        if not self._feedback_valid():
            self.settle_since_ = None
            now = time.monotonic()
            local_age = (now - self.last_local_time_
                         if self.last_local_time_ is not None else float('inf'))
            status_age = (now - self.last_status_time_
                          if self.last_status_time_ is not None else float('inf'))
            self.get_logger().warn(
                'PX4 反馈未就绪/失效：position_age=%.3fs (limit=%.3fs), '
                'status_age=%.3fs (limit=%.3fs), position_valid=%s；'
                '停止控制；已接管时将交由 PX4 OFFBOARD 丢失处理，需从地面重启桥'
                % (local_age, self.feedback_timeout_, status_age,
                   self.status_timeout_, self.have_local_pos_),
                throttle_duration_sec=2.0)
            if self.ever_active_:
                self.inhibited_ = True
            return
        if self.fixed_sp_ is None:
            return
            
        self._check_tracking_error()
        self._check_planner_brake()

        if self.ever_active_ and self.phase_ == self.TAKEOFF:
            distance = math.dist(self.fixed_sp_, (self.local_.x, self.local_.y, self.local_.z))
            speed = math.sqrt(self.local_.vx ** 2 + self.local_.vy ** 2 + self.local_.vz ** 2)
            if distance <= self.takeoff_tolerance_ and speed <= self.settle_speed_:
                if self.settle_since_ is None:
                    self.settle_since_ = time.monotonic()
                if time.monotonic() - self.settle_since_ >= self.settle_time_:
                    self.phase_ = self.WAIT_EGO
                    self.sp_ = None
                    self.wait_stamp_ns_ = self.get_clock().now().nanoseconds
                    self.get_logger().info(
                        '起飞完成，稳定悬停；请发送 /traj_start_trigger'
                        if self.mode_ == 'ego' else
                        '起飞完成，fixed 模式持续悬停，无需发送规划触发')
            else:
                self.settle_since_ = None

        # ---- 1. 未解锁：持续预流，计数不能在每次回调中清零 ----
        if not self.armed_:
            if self.state_ == self.S_IDLE:
                self.stream_count_ = 0
                self.arm_sent_ = False
                self.offb_sent_ = False
                self.offb_wait_count_ = 0
            if not self.auto_arm_:
                return
            self.state_ = self.S_STREAM

            # 必须先有 setpoint 流，PX4 才接受 ARM / OFFBOARD
            self.publish_offboard_control_mode()
            sp = self._select_setpoint()
            if sp is not None:
                self.publish_trajectory_setpoint(sp)
            self.stream_count_ += 1

            # (a) 预流约 2s 后发 ARM
            if self.stream_count_ >= self.prestream_count_ and not self.arm_sent_:
                self.send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)
                self.arm_sent_ = True
                self.get_logger().info('发送 ARM 指令')
            return

        # ---- 1b. 已解锁但还没请求 OFFBOARD：立刻请求 ----
        # 不要用"帧计数"来判断时机（之前 == 60 的分支永远命中不到），用状态标志
        if not self.offb_sent_:
            # DO_SET_MODE: param1=1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
            #              param2=6 (PX4 custom mode = OFFBOARD)
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0)
            self.offb_sent_ = True
            self.get_logger().info('发送 DO_SET_MODE(OFFBOARD)')
            # 注意：不 return，本周期继续发 setpoint，避免切换瞬间出现空档

        # ---- 2. 已解锁：持续发心跳 + setpoint ----
        self.publish_offboard_control_mode()

        sp = self._select_setpoint()
        if sp is not None:
            self.publish_trajectory_setpoint(sp)

        # ---- 3. 状态迁移 ----
        if self.nav_state_ == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.state_ != self.S_ACTIVE:
                self.ever_active_ = True
                self.state_ = self.S_ACTIVE
                self.offb_wait_count_ = 0
                self.get_logger().info(
                    '>>> 已进入 OFFBOARD，开始接管位置控制 <<<')
        else:
            self.state_ = self.S_STREAM
            # 已经请求过 OFFBOARD 但一直没切过去 -> 报精确诊断，不刷屏
            self.offb_wait_count_ += 1
            if self.offb_sent_ and self.offb_wait_count_ == 25:   # ~1.25s
                self.get_logger().error(
                    'DO_SET_MODE(OFFBOARD) 已发送但 nav_state 仍为 %d。'
                    '检查：① ros2 topic hz /fmu/in/trajectory_setpoint 是否 ~20Hz；'
                    '② ros2 topic echo /fmu/out/vehicle_command_ack；'
                    '③ 是否被 QGC/遥控抢走控制权' % self.nav_state_)
            self.get_logger().warn(
                '已解锁但 nav_state=%d（OFFBOARD=14），等待切换...'
                % self.nav_state_, throttle_duration_sec=2.0)

    def _select_setpoint(self):
        """返回本周期要发布的 setpoint（NED dict），或 None"""
        if self.mode_ == 'fixed' or self.phase_ in (self.TAKEOFF, self.WAIT_EGO):
            if self.fixed_sp_ is None:
                return None
            return {'pos': self.fixed_sp_, 'vel': (0.0, 0.0, 0.0),
                    'acc': (0.0, 0.0, 0.0), 'yaw': self.fixed_yaw_}
        if self.phase_ in (self.TRACK, self.PLANNER_BRAKE):
            self._check_planner_brake()
            if self.phase_ in (self.BRAKE, self.WAIT_RECOVERY, self.HOLD):
                return self.hold_sp_
            if (self.sp_ is not None and self.last_cmd_monotonic_ is not None
                    and time.monotonic() - self.last_cmd_monotonic_ <= self.cmd_timeout_):
                return self.sp_
            self._enter_hold('EGO 指令超时')
        return self.hold_sp_


def main(args=None):
    rclpy.init(args=args)
    node = PositionCmdToPx4()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
