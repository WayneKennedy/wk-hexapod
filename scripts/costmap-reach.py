#!/usr/bin/env python3
"""
Report what the planner can reach in the global costmap (OQ-34).

Takes one /global_costmap/costmap with the stack running, flood-fills from the
robot's cell over cells the planner may cross, and prints the reachable area,
whether it touches unknown space, the cost at each --goal, and a character map
around the robot.

Usage (on the robot, ROS sourced as scripts/launch.sh does):
  python3 scripts/costmap-reach.py [--goal X Y]... [--span 5.0] [--save FILE.npz]

Map characters: R robot, G goal, '.' reachable, ' ' free but cut off,
'+' inflated (cost below inscribed), 'o' inscribed, '#' lethal, '?' unknown.
"""

import argparse
import collections
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

INSCRIBED = 99   # OccupancyGrid scale: 100 lethal, 99 inscribed, -1 unknown


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goal', nargs=2, type=float, action='append', default=[],
                    metavar=('X', 'Y'))
    ap.add_argument('--span', type=float, default=5.0, help='side of the character map, m')
    ap.add_argument('--topic', default='/global_costmap/costmap')
    ap.add_argument('--save', help='write the grid and the robot pose to this .npz')
    args = ap.parse_args()

    rclpy.init()
    node = Node('costmap_reach')
    buf = Buffer()
    TransformListener(buf, node)
    got = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, args.topic, got.append, qos)

    pose = None
    end = time.monotonic() + 25.0   # discovery takes up to ~10 s
    while time.monotonic() < end and (not got or pose is None):
        rclpy.spin_once(node, timeout_sec=0.2)
        if got:
            try:
                t = buf.lookup_transform(got[-1].header.frame_id, 'base_link',
                                         rclpy.time.Time()).transform.translation
                pose = (t.x, t.y)
            except Exception:  # noqa: BLE001
                pass
    if not got or pose is None:
        print(f'costmap received: {bool(got)}; robot pose: {pose}')
        return

    m = got[-1]
    w, h, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    grid = np.array(m.data, dtype=np.int16).reshape(h, w)
    if args.save:
        np.savez_compressed(args.save, grid=grid, res=res, ox=ox, oy=oy, pose=pose)

    def cell(x, y):
        return int((x - ox) / res), int((y - oy) / res)

    rx, ry = cell(*pose)
    passable = (grid >= 0) & (grid < INSCRIBED)
    reach = np.zeros_like(passable)
    if 0 <= rx < w and 0 <= ry < h:
        # The robot's own cell may be costly; start from it regardless
        queue = collections.deque([(rx, ry)])
        reach[ry, rx] = True
        while queue:
            x, y = queue.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h and passable[ny, nx] and not reach[ny, nx]:
                    reach[ny, nx] = True
                    queue.append((nx, ny))

    unknown = grid < 0
    edge = np.zeros_like(unknown)
    edge[1:, :] |= unknown[:-1, :]
    edge[:-1, :] |= unknown[1:, :]
    edge[:, 1:] |= unknown[:, :-1]
    edge[:, :-1] |= unknown[:, 1:]
    touching = reach & edge

    ys, xs = np.nonzero(reach)
    print(f'costmap {w} x {h} cells at {res:.3f} m, origin ({ox:.2f}, {oy:.2f}), '
          f'frame {m.header.frame_id}')
    print(f'robot at ({pose[0]:.2f}, {pose[1]:.2f}), cost at its cell {grid[ry, rx]}')
    print(f'cells: unknown {int(unknown.sum())}, lethal {int((grid == 100).sum())}, '
          f'inscribed {int((grid == INSCRIBED).sum())}, passable {int(passable.sum())}')
    print(f'reachable: {int(reach.sum())} cells, {reach.sum() * res * res:.2f} m2, '
          f'x {ox + xs.min() * res:.2f}..{ox + (xs.max() + 1) * res:.2f}, '
          f'y {oy + ys.min() * res:.2f}..{oy + (ys.max() + 1) * res:.2f}')
    print(f'reachable cells beside unknown space: {int(touching.sum())}')

    goals = []
    for gx, gy in args.goal:
        cx, cy = cell(gx, gy)
        goals.append((cx, cy))
        if not (0 <= cx < w and 0 <= cy < h):
            print(f'goal ({gx:.2f}, {gy:.2f}): outside the costmap')
            continue
        d = np.hypot(xs - cx, ys - cy) * res
        print(f'goal ({gx:.2f}, {gy:.2f}): cost {grid[cy, cx]}, '
              f'reachable {bool(reach[cy, cx])}, nearest reachable cell {d.min():.2f} m away')

    # Character map, x ahead is up the page only if the robot faces +x; this is the
    # map frame: +x right, +y up
    step = max(1, int(round(0.1 / res)))
    half = int(args.span / 2 / res)
    print(f'map frame, +x right, +y up, {step * res:.2f} m per character')
    for y in range(min(h - 1, ry + half), max(0, ry - half) - 1, -step):
        row = []
        for x in range(max(0, rx - half), min(w, rx + half + 1), step):
            block = grid[y:y + step, x:x + step]
            if abs(x - rx) < step and abs(y - ry) < step:
                c = 'R'
            elif any(abs(x - gx) < step and abs(y - gy) < step for gx, gy in goals):
                c = 'G'
            elif (block == 100).any():
                c = '#'
            elif (block == INSCRIBED).any():
                c = 'o'
            elif reach[y:y + step, x:x + step].any():
                c = '.'
            elif (block < 0).all():
                c = '?'
            elif (block > 0).any():
                c = '+'
            else:
                c = ' '
            row.append(c)
        print(''.join(row))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
