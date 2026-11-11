#!/usr/bin/env python3
"""
Bridge: ego-planner PositionCommand → AirSim velocity commands.
Uses subprocess + ros2 topic pub to avoid airsim_interfaces import issues.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
import subprocess


class PosCmdToVelBridge(Node):
    def __init__(self):
        super().__init__('poscmd_to_vel_bridge')

        self.declare_parameter('kp_xy', 1.0)
        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('max_vel_xy', 2.0)
        self.declare_parameter('max_vel_z', 1.5)
        self.declare_parameter('target_timeout', 0.5)

        self.kp_xy = self.get_parameter('kp_xy').value
        self.kp_z = self.get_parameter('kp_z').value
        self.max_vel_xy = self.get_parameter('max_vel_xy').value
        self.max_vel_z = self.get_parameter('max_vel_z').value
        self.target_timeout = self.get_parameter('target_timeout').value

        self.target_pos = np.zeros(3)
        self.target_vel = np.zeros(3)
        self.target_yaw = 0.0
        self.current_pos = np.zeros(3)
        self.has_odom = False
        self.has_target = False
        self.last_vel = np.zeros(4)  # x, y, z, yaw

        self.odom_sub = self.create_subscription(
            Odometry, '/grid_map/odom_enu', self.odom_callback, 10)

        self.cmd_sub = self.create_subscription(
            Odometry, '/drone_0_planning/pos_cmd', self.cmd_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.last_target_time = self.get_clock().now()

        self.get_logger().info(
            f'poscmd_to_vel_bridge started (subprocess mode) '
            f'kp_xy={self.kp_xy}, kp_z={self.kp_z}')

    def cmd_callback(self, msg):
        try:
            self.target_pos = np.array([
                msg.position.x, msg.position.y, msg.position.z])
            self.target_vel = np.array([
                msg.velocity.x, msg.velocity.y, msg.velocity.z])
            self.target_yaw = float(getattr(msg, 'yaw', 0.0))
        except AttributeError:
            try:
                self.target_pos = np.array([
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z])
                self.target_vel = np.zeros(3)
            except AttributeError:
                return

        self.has_target = True
        self.last_target_time = self.get_clock().now()

    def odom_callback(self, msg):
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z])
        self.has_odom = True

    def control_loop(self):
        if not self.has_odom:
            return

        dt = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
        if dt > self.target_timeout:
            self.has_target = False
            self._send_vel(np.zeros(4))
            return

        if not self.has_target:
            self._send_vel(np.zeros(4))
            return

        error = self.target_pos - self.current_pos
        vel_des = np.array([
            self.kp_xy * error[0] + self.target_vel[0],
            self.kp_xy * error[1] + self.target_vel[1],
            self.kp_z * error[2] + self.target_vel[2],
        ])

        vel_xy = vel_des[:2]
        n = np.linalg.norm(vel_xy)
        if n > self.max_vel_xy:
            vel_xy = vel_xy / n * self.max_vel_xy

        vel_z = np.clip(vel_des[2], -self.max_vel_z, self.max_vel_z)
        vel = np.array([vel_xy[0], vel_xy[1], vel_z, self.target_yaw])

        # Only publish if velocity changed meaningfully (>1cm/s)
        if np.linalg.norm(vel - self.last_vel) > 0.01:
            self._send_vel(vel)
            self.last_vel = vel

    def _send_vel(self, vel):
        """Publish velocity via ros2 topic pub (subprocess)."""
        yaml = (
            f'{{twist: {{linear: {{x: {vel[0]:.3f}, y: {vel[1]:.3f}, '
            f'z: {vel[2]:.3f}}}, angular: {{x: 0.0, y: 0.0, z: {vel[3]:.3f}}}}}}}'
        )
        try:
            subprocess.run(
                ['ros2', 'topic', 'pub', '--once', '-t', '1',
                 '/airsim_node/PX4/vel_cmd_world_frame',
                 'airsim_interfaces/msg/VelCmd', yaml],
                capture_output=True,
                timeout=0.05)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PosCmdToVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()