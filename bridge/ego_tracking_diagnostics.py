#!/usr/bin/env python3
"""Read-only EGO/PX4 tracking capture. No control publishers or arming calls.

Run before triggering, then stop with Ctrl+C. CSV coordinates use ENU.
Errors use latest samples, not interpolated or time-aligned ground truth.
"""
import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import Image


def ned_to_enu(x, y, z):
    return y, x, -z


def distance(a, b):
    return math.dist(a, b)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--duration', type=float, default=180.0)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error('--duration must be positive')
    path = args.output or Path.home() / ('ego_tracking_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv')
    rclpy.init()
    node = rclpy.create_node('ego_tracking_diagnostics')
    values, received, max_gaps = {}, {}, {}
    def remember(key, value):
        now = time.monotonic()
        if key in received:
            max_gaps[key] = max(max_gaps.get(key, 0), now - received[key])
        received[key], values[key] = now, value
    subs = [
        node.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            lambda m: remember('px4', ned_to_enu(m.x, m.y, m.z)), qos_profile_sensor_data),
        node.create_subscription(Odometry, '/grid_map/odom_enu',
            lambda m: remember('odom', (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)), qos_profile_sensor_data),
        node.create_subscription(PositionCommand, '/drone_0_planning/pos_cmd',
            lambda m: remember('cmd', (m.position.x, m.position.y, m.position.z,
                m.trajectory_id, m.header.stamp.sec + m.header.stamp.nanosec / 1e9,
                math.sqrt(m.velocity.x**2 + m.velocity.y**2 + m.velocity.z**2))), qos_profile_sensor_data),
        node.create_subscription(Image, '/airsim_node/PX4/CameraDepth/DepthPlanar',
            lambda m: remember('depth', m.header.stamp.sec + m.header.stamp.nanosec / 1e9), qos_profile_sensor_data),
    ]
    fields = ['elapsed_s', 'ros_s', 'px4_x', 'px4_y', 'px4_z', 'odom_x', 'odom_y', 'odom_z',
        'cmd_x', 'cmd_y', 'cmd_z', 'tracking_error_m', 'origin_difference_m', 'traj_id',
        'cmd_speed_mps', 'cmd_ros_age_s', 'depth_ros_age_s']
    fields += [f'{key}_{suffix}_s' for key in ('px4', 'odom', 'cmd', 'depth') for suffix in ('receive_age', 'max_receive_gap')]
    start = time.monotonic()
    last_sample = last_print = start
    print('Read-only capture:', path, flush=True)
    try:
        with path.open('x', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            while rclpy.ok() and time.monotonic() - start < args.duration:
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.monotonic()
                if now - last_sample < 0.1:
                    continue
                last_sample = now
                ros_now = node.get_clock().now().nanoseconds / 1e9
                row = dict(elapsed_s=now-start, ros_s=ros_now)
                for key in ('px4', 'odom', 'cmd', 'depth'):
                    row[key+'_receive_age_s'] = now-received[key] if key in received else float('inf')
                    row[key+'_max_receive_gap_s'] = max_gaps.get(key, 0)
                    if key in values and key != 'depth':
                        row.update({key+'_'+axis: v for axis, v in zip('xyz', values[key][:3])})
                if all(k in values for k in ('px4', 'cmd')) and row['px4_receive_age_s'] < .5 and row['cmd_receive_age_s'] < .25:
                    row['tracking_error_m'] = distance(values['px4'], values['cmd'][:3])
                if all(k in values for k in ('px4', 'odom')) and max(row['px4_receive_age_s'], row['odom_receive_age_s']) < .5:
                    row['origin_difference_m'] = distance(values['px4'], values['odom'])
                if 'cmd' in values:
                    row.update(traj_id=values['cmd'][3], cmd_speed_mps=values['cmd'][5], cmd_ros_age_s=ros_now-values['cmd'][4])
                if 'depth' in values:
                    row['depth_ros_age_s'] = ros_now-values['depth']
                writer.writerow(row)
                if now-last_print >= 2:
                    last_print = now
                    print('t=%.1f error=%s m; PX4/AirSim difference=%s m; cmd age=%.3fs depth age=%.3fs' % (
                        now-start, row.get('tracking_error_m', 'unavailable'), row.get('origin_difference_m', 'unavailable'),
                        row['cmd_receive_age_s'], row['depth_receive_age_s']), flush=True)
                    f.flush()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('Saved:', path)


if __name__ == '__main__':
    main()
