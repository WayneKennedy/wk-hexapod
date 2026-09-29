#!/usr/bin/env python3
"""
Report the lidar's nearest returns by body bearing, to find returns from the
robot itself (legs, head, cables) in the scan plane (OQ-31).

Listens to /scan for a few seconds with the stack running and prints, per 30
degree sector of body bearing (0 = ahead, + = left), the number of returns and
the nearest one, then every bearing with a return nearer than --near.

Usage (on the robot, ROS sourced as scripts/launch.sh does):
  python3 scripts/scan-near.py [--seconds 5] [--near 0.35]
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--near', type=float, default=0.35)
    args = ap.parse_args()

    rclpy.init()
    node = Node('scan_near')
    buf = Buffer()
    TransformListener(buf, node)
    scans = []
    node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)

    end = time.monotonic() + args.seconds + 10.0  # discovery takes up to ~10 s
    first = None
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if scans and first is None:
            first = time.monotonic()
        if first is not None and time.monotonic() - first >= args.seconds:
            break
    if not scans:
        print('no /scan received')
        return

    # Yaw of the laser in base_link, from TF (pi on this robot)
    yaw = None
    try:
        q = buf.lookup_transform('base_link', scans[0].header.frame_id, rclpy.time.Time()).transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    except Exception as e:  # noqa: BLE001
        print(f'no TF base_link -> {scans[0].header.frame_id} ({e}); bearings are in the laser frame')
        yaw = 0.0

    sectors = {s: [] for s in range(-180, 180, 30)}
    near = {}
    total = 0
    for m in scans:
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            total += 1
            b = math.degrees(m.angle_min + i * m.angle_increment + yaw)
            b = (b + 180.0) % 360.0 - 180.0
            sectors[int(math.floor(b / 30.0)) * 30].append(r)
            if r < args.near:
                k = int(round(b / 5.0)) * 5
                lo, n = near.get(k, (r, 0))
                near[k] = (min(lo, r), n + 1)

    m = scans[0]
    print(f'{len(scans)} scans, {len(m.ranges)} beams each, {total} returns, '
          f'range_min {m.range_min:.2f} m, laser yaw in base_link {math.degrees(yaw):.0f} deg')
    print('body sector (deg)   returns   nearest (m)')
    for s in sorted(sectors):
        v = sectors[s]
        print(f'  {s:+4d} .. {s + 30:+4d}     {len(v):6d}   {min(v):.3f}' if v
              else f'  {s:+4d} .. {s + 30:+4d}          0   -')
    print(f'returns nearer than {args.near} m, by body bearing (5 deg bins): nearest, count')
    for k in sorted(near):
        print(f'  {k:+4d}   {near[k][0]:.3f}   {near[k][1]}')
    if not near:
        print('  none')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
