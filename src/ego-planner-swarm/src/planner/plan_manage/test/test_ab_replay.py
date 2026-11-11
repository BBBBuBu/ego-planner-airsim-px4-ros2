#!/usr/bin/env python3
"""Replay only recorded sensors in domain 193; never start a PX4 bridge."""

import argparse
import json
import os
from pathlib import Path
import re
import signal
import sqlite3
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '193':
        raise RuntimeError('This test requires isolated ROS_DOMAIN_ID=193')
    db = args.bag if args.bag.suffix == '.db3' else next(args.bag.glob('*.db3'))
    con = sqlite3.connect('file:' + str(db) + '?mode=ro', uri=True)
    topics = {name: identifier for identifier, name in con.execute('select id,name from topics')}
    trigger_time = con.execute('select min(timestamp) from messages where topic_id=?',
                               (topics['/traj_start_trigger'],)).fetchone()[0]
    begin = trigger_time - 4_000_000_000
    end = trigger_time + 13_000_000_000
    raw_id = topics['/airsim_node/PX4/odom_local_ned']
    depth_id = topics['/airsim_node/PX4/CameraDepth/DepthPlanar']
    rows = con.execute(
        'select timestamp,topic_id,data from messages where topic_id in (?,?) '
        'and timestamp between ? and ? order by timestamp',
        (raw_id, depth_id, begin, end)).fetchall()
    con.close()
    assert rows, 'No sensor records in selected interval'
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('ego_ab_recorded_sensor_test')
    raw_pub = node.create_publisher(Odometry, '/airsim_node/PX4/odom_local_ned', 100)
    depth_pub = node.create_publisher(Image, '/airsim_node/PX4/CameraDepth/DepthPlanar', 20)
    trigger_pub = node.create_publisher(PoseStamped, '/traj_start_trigger', 10)
    enu_times, commands, plan_rounds = [], [], []

    def log_callback(msg):
        match = re.search(r'\[AB_PLAN\] elapsed_ms=([\d.]+).*expired=(\d)', msg.msg)
        if match:
            plan_rounds.append((float(match.group(1)), int(match.group(2))))

    subscriptions = [
        node.create_subscription(Odometry, '/grid_map/odom_enu',
                                 lambda m: enu_times.append(time.monotonic()), 1000),
        node.create_subscription(PositionCommand, '/drone_0_planning/pos_cmd',
                                 lambda m: commands.append((time.monotonic(), m.trajectory_id)), 1000),
        node.create_subscription(Log, '/rosout', log_callback, 1000),
    ]
    log = (args.output / 'planner_replay.log').open('w')
    process = subprocess.Popen(
        ['/opt/ros/humble/bin/ros2', 'launch', 'ego_planner', 'single_run_in_sim.launch.py'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def spin_until(deadline):
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.005, deadline - time.monotonic())))

    try:
        deadline = time.monotonic() + 10
        while raw_pub.get_subscription_count() == 0 or depth_pub.get_subscription_count() == 0:
            assert time.monotonic() < deadline, 'Planner/converter did not discover sensor publishers'
            assert process.poll() is None, 'Planner launch exited'
            rclpy.spin_once(node, timeout_sec=0.01)
        replay_started = time.monotonic()
        triggered = False
        for ns, topic, data in rows:
            due = replay_started + (ns - begin) / 1e9
            spin_until(due)
            if not triggered and ns >= trigger_time:
                trigger = PoseStamped()
                trigger.header.frame_id = 'world'
                trigger.pose.orientation.w = 1.0
                trigger_pub.publish(trigger)
                triggered = True
            if topic == raw_id:
                raw_pub.publish(deserialize_message(data, Odometry))
            else:
                depth_pub.publish(deserialize_message(data, Image))
            rclpy.spin_once(node, timeout_sec=0.0)
        spin_until(time.monotonic() + 0.2)
        assert process.poll() is None, 'Planner launch crashed during replay'
        assert len(enu_times) > 500, 'Insufficient converted odometry'
        assert commands, 'No executable trajectories during replay'
        assert plan_rounds, 'No planning budget measurements'
        gaps = [b - a for a, b in zip(enu_times, enu_times[1:])]
        report = {
            'source_bag': str(db),
            'scope': 'recorded sensors, not a closed-loop flight or collision acceptance test',
            'enu_count': len(enu_times),
            'enu_max_receipt_gap_sec': max(gaps),
            'command_count': len(commands),
            'trajectory_ids': sorted(set(identifier for _, identifier in commands)),
            'planning_round_count': len(plan_rounds),
            'planning_round_max_ms': max(ms for ms, _ in plan_rounds),
            'expired_round_count': sum(expired for _, expired in plan_rounds),
            'planning_rounds_ms': [ms for ms, _ in plan_rounds],
        }
        (args.output / 'replay_test.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2), flush=True)
        assert max(gaps) < 0.5, 'Converted odometry stalled'
        assert max(ms for ms, _ in plan_rounds) < 200, 'Planning still has a long blocking round'
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        log.close()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
