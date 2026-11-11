#!/usr/bin/env python3
"""Exercise A/B node communication in isolated domain 193; no PX4 publishers."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Int64
from traj_utils.msg import Bspline


def stamp(seconds):
    ns = round(seconds * 1e9)
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


def trajectory(identifier, start, x0, velocity=0.2):
    msg = Bspline()
    msg.traj_id = identifier
    msg.order = 3
    msg.start_time = stamp(start)
    from geometry_msgs.msg import Point
    msg.pos_pts = [Point(x=x0 + velocity * (i - 1) * 0.2, z=2.0)
                   for i in range(40)]
    msg.knots = [(i - 3) * 0.2 for i in range(44)]
    return msg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '193':
        raise RuntimeError('This test requires isolated ROS_DOMAIN_ID=193')
    args.output.mkdir(parents=True, exist_ok=True)
    prefix = Path(get_package_prefix('ego_planner')) / 'lib/ego_planner'
    processes = []
    logs = []
    rclpy.init()
    node = rclpy.create_node('ego_ab_wire_test')
    commands, acknowledgements, committed, converted = [], [], [], []
    subscriptions = [
        node.create_subscription(PositionCommand, '/position_cmd',
                                 lambda m: commands.append((time.monotonic(), m)), 1000),
        node.create_subscription(Int64, '/planning/trajectory_ack',
                                 lambda m: acknowledgements.append(m.data), 10),
        node.create_subscription(Int64, '/planning/trajectory_committed',
                                 lambda m: committed.append(m.data), 10),
        node.create_subscription(Odometry, '/grid_map/odom_enu',
                                 lambda m: converted.append(m), 1000),
    ]
    publishers = {
        'spline': node.create_publisher(Bspline, '/planning/bspline', 10),
        'emergency': node.create_publisher(Bspline, '/planning/emergency_bspline', 10),
        'commit': node.create_publisher(Int64, '/planning/trajectory_commit', 10),
        'cancel': node.create_publisher(Int64, '/planning/cancel_pending', 10),
    }
    odom_pub = node.create_publisher(Odometry, '/grid_map/odom_enu', 10)

    def ros_now():
        return node.get_clock().now().nanoseconds / 1e9

    def spin_for(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.01)

    def wait_until(condition, timeout=5.0):
        until = time.monotonic() + timeout
        while not condition():
            if time.monotonic() >= until:
                raise AssertionError('Timed out waiting for test condition')
            rclpy.spin_once(node, timeout_sec=0.01)

    def start(executable):
        log = (args.output / (executable + '.log')).open('w')
        logs.append(log)
        process = subprocess.Popen([str(prefix / executable)], stdout=log,
                                   stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    def stop(process):
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)

    def publish_odom():
        msg = Odometry()
        msg.header.frame_id = 'world'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.z = 2.0
        msg.pose.pose.orientation.w = 1.0
        odom_pub.publish(msg)

    timer = node.create_timer(0.01, publish_odom)
    report = {}
    try:
        server = start('traj_server')
        wait_until(lambda: publishers['spline'].get_subscription_count() > 0)
        spin_for(1.2)
        first_start = ros_now() + 0.30
        publishers['spline'].publish(trajectory(1, first_start, 0.0))
        wait_until(lambda: 1 in acknowledgements)
        publishers['commit'].publish(Int64(data=1))
        wait_until(lambda: 1 in committed)
        wait_until(lambda: len(commands) >= 20)

        second_start = ros_now() + 0.35
        x0 = 0.2 * (second_start - first_start)
        publishers['spline'].publish(trajectory(2, second_start, x0))
        wait_until(lambda: 2 in acknowledgements)
        publishers['commit'].publish(Int64(data=2))
        wait_until(lambda: 2 in committed)
        wait_until(lambda: any(m.trajectory_id == 2 for _, m in commands))
        spin_for(0.15)
        for _, msg in commands:
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if ts < second_start:
                assert msg.trajectory_id == 1
        transition = next(i for i, (_, m) in enumerate(commands)
                          if m.trajectory_id == 2)
        previous, current = commands[transition - 1], commands[transition]
        report['switch_gap_sec'] = current[0] - previous[0]
        report['switch_position_step_m'] = abs(current[1].position.x - previous[1].position.x)
        assert report['switch_gap_sec'] < 0.1
        assert report['switch_position_step_m'] < 0.03

        publishers['spline'].publish(trajectory(3, ros_now() - 0.1, 0.0))
        wait_until(lambda: -3 in acknowledgements)
        publishers['spline'].publish(trajectory(4, ros_now() + 0.3, 9.0))
        wait_until(lambda: -4 in acknowledgements)
        uncommitted_start = ros_now() + 0.25
        publishers['spline'].publish(trajectory(
            5, uncommitted_start, 0.2 * (uncommitted_start - first_start)))
        wait_until(lambda: 5 in acknowledgements)
        wait_until(lambda: -5 in acknowledgements)
        cancelled_start = ros_now() + 0.30
        publishers['spline'].publish(trajectory(
            6, cancelled_start, 0.2 * (cancelled_start - first_start)))
        wait_until(lambda: 6 in acknowledgements)
        publishers['commit'].publish(Int64(data=6))
        wait_until(lambda: 6 in committed)
        publishers['cancel'].publish(Int64(data=6))
        wait_until(lambda: -6 in acknowledgements)
        spin_for(0.35)
        assert not any(m.trajectory_id in (3, 4, 5, 6) for _, m in commands)

        publishers['emergency'].publish(trajectory(7, ros_now(), 0.7, 0.0))
        wait_until(lambda: any(m.trajectory_id == 7 for _, m in commands))
        malformed = trajectory(8, ros_now() + 0.25, 0.7, 0.0)
        malformed.pos_pts[2].x = math.nan
        publishers['spline'].publish(malformed)
        wait_until(lambda: -8 in acknowledgements)
        spin_for(0.10)
        assert server.poll() is None
        assert commands[-1][1].trajectory_id == 7
        assert abs(commands[-1][1].position.x - 0.7) < 1e-6
        report['late_discontinuous_uncommitted_cancelled_malformed_rejected'] = True
        report['command_count'] = len(commands)
        stop(server)

        timer.cancel()
        node.destroy_publisher(odom_pub)
        converter = start('airsim_odom_to_enu')
        raw_pub = node.create_publisher(Odometry, '/airsim_node/PX4/odom_local_ned', 10)
        wait_until(lambda: raw_pub.get_subscription_count() > 0)
        raw = Odometry()
        raw.header.stamp = Time(sec=123, nanosec=456)
        raw.pose.pose.position.x = 1.0
        raw.pose.pose.position.y = 2.0
        raw.pose.pose.position.z = -3.0
        raw.pose.pose.orientation.w = 1.0
        raw.twist.twist.linear.x = 4.0
        raw.twist.twist.linear.y = 5.0
        raw.twist.twist.linear.z = 6.0
        raw_pub.publish(raw)
        wait_until(lambda: any(m.header.stamp.sec == 123 for m in converted))
        result = next(m for m in converted if m.header.stamp.sec == 123)
        assert (result.pose.pose.position.x, result.pose.pose.position.y,
                result.pose.pose.position.z) == (2.0, 1.0, 3.0)
        assert (result.twist.twist.linear.x, result.twist.twist.linear.y,
                result.twist.twist.linear.z) == (5.0, 4.0, -6.0)
        assert result.header.stamp.nanosec == 456
        assert result.header.frame_id == 'world'
        assert abs(abs(result.pose.pose.orientation.w) - math.sqrt(0.5)) < 1e-6
        assert abs(abs(result.pose.pose.orientation.z) - math.sqrt(0.5)) < 1e-6
        raw.header.stamp.sec = 124
        raw.pose.pose.orientation.w = 0.0
        raw_pub.publish(raw)
        spin_for(0.15)
        assert not any(m.header.stamp.sec == 124 for m in converted)
        assert converter.poll() is None
        report['independent_converter_position_velocity_attitude_stamp_and_invalid_input'] = True
        (args.output / 'wire_test.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2), flush=True)
    finally:
        for process in processes:
            if process.poll() is None:
                stop(process)
        for log in logs:
            log.close()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
