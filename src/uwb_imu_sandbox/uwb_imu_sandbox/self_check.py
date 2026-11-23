#!/usr/bin/env python3
"""Self check: listens to the running sandbox and reports whether it is healthy.

    ros2 run uwb_imu_sandbox self_check --ros-args -p duration:=20.0

Exit code 0 when every check passes, 1 otherwise.
"""
import math
import statistics
import sys
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

from uwb_imu_sandbox_msgs.msg import UwbRange, UwbTruth


class SelfCheck(Node):

    def __init__(self):
        super().__init__('self_check')
        self.declare_parameter('duration', 20.0)
        self.declare_parameter('expected_anchors', [1, 2, 3, 4])
        self.duration = float(self.get_parameter('duration').value)
        self.expected = list(self.get_parameter('expected_anchors').value)
        self.ranges, self.truths, self.arrays, self.imu, self.odom = [], [], [], [], []
        self.clock = []
        self.create_subscription(UwbRange, '/uwb/range', lambda m: self.ranges.append(m), 200)
        self.create_subscription(UwbTruth, '/uwb/truth', lambda m: self.truths.append(m), 200)
        self.create_subscription(Float64MultiArray, '/uwb/ranges',
                                 lambda m: self.arrays.append(m), qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', lambda m: self.imu.append(m), 500)
        self.create_subscription(Odometry, '/ground_truth/odom',
                                 lambda m: self.odom.append(m), 200)
        self.create_subscription(Clock, '/clock', lambda m: self.clock.append(m), 10)


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    n = SelfCheck()
    t_end = time.time() + n.duration
    while time.time() < t_end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)
    results = []

    def check(name, ok, detail):
        results.append((name, bool(ok), detail))

    # more than one /clock publisher means more than one simulation (or bridge)
    # is running: every rate below would then be a multiple of the real one
    clocks = n.count_publishers('/clock')
    hint = '' if clocks == 1 else ' - stop all old runs: pkill -f "gz sim"'
    check('only one simulation running', clocks == 1, f'{clocks} /clock publisher(s){hint}')

    sim = [c.clock.sec + c.clock.nanosec * 1e-9 for c in n.clock]
    sim_span = (max(sim) - min(sim)) if len(sim) > 1 else 0.0
    check('simulation clock runs', sim_span > 1.0, f'{sim_span:.1f} s of sim time seen')

    if n.imu and sim_span > 0:
        rate = len(n.imu) / sim_span
        check('IMU rate ~100 Hz', 70 <= rate <= 130, f'{rate:.1f} Hz')
        az = statistics.fmean(m.linear_acceleration.z for m in n.imu)
        check('IMU sees gravity', 9.0 < az < 10.6, f'mean accel z {az:.3f} m/s^2')
        finite = all(math.isfinite(v) for m in n.imu for v in (
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
            m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z))
        check('IMU values finite', finite, '')
        check('IMU frame', all(m.header.frame_id == 'base_link' for m in n.imu),
              n.imu[-1].header.frame_id)
    else:
        check('IMU messages', False, 'no /imu messages')

    if n.ranges and sim_span > 0:
        for a in n.expected:
            rs = [r for r in n.ranges if r.anchor_id == a]
            slots = sorted(stamp(t) for t in n.truths if t.anchor_id == a)
            # slot rate while the anchor is in the tag's network (it leaves the
            # network when it is more than 12 m away)
            if len(slots) > 2:
                gaps = [b - c for b, c in zip(slots[1:], slots[:-1])]
                in_net = sum(g for g in gaps if g < 1.0)
                rate = (len(slots) - 1) / in_net if in_net > 0 else 0.0
            else:
                rate = 0.0
            check(f'UWB anchor {a} reports', len(rs) > 0 and 4.0 <= rate <= 11.0,
                  f'{len(rs)} ranges, {rate:.2f} slots/s while in range')
        net = max(r.network_count for r in n.ranges)
        check('UWB network size', net == len(n.expected), f'max network_count {net}')
        finite = all(math.isfinite(v) for r in n.ranges for v in (
            r.range, r.rx_level, r.fp_level, r.quality, r.delta_level))
        check('UWB values finite', finite, '')
        check('UWB range limit', all(0.0 <= r.range < 16.0 for r in n.ranges), '')
    else:
        check('UWB messages', False, 'no /uwb/range messages')

    if n.truths:
        ok = [t for t in n.truths if t.outcome == 'ok']
        los = [t for t in n.truths if t.line_of_sight]
        check('UWB ground truth', len(ok) > 0,
              f'{len(n.truths)} slots, {len(ok)} ok, {len(los)} LOS')
        # pair each range with the truth of the same slot (same anchor, the
        # two are stamped by uwb_node within a few ms of each other)
        errs_los, errs_nlos = [], []
        tr_anchor = {}
        for t in ok:
            tr_anchor.setdefault(t.anchor_id, []).append((stamp(t), t))
        for r in n.ranges:
            cands = tr_anchor.get(r.anchor_id, [])
            if not cands:
                continue
            ts, tt = min(cands, key=lambda c: abs(c[0] - stamp(r)))
            if abs(ts - stamp(r)) > 0.006:
                continue
            target = errs_los if tt.nlos_weight < 0.05 else errs_nlos
            target.append(r.range - tt.true_range)
        if errs_los:
            med = statistics.median(errs_los)
            check('LOS range error looks like a DW1000', -0.15 < med < 0.35,
                  f'median {med * 100:.1f} cm over {len(errs_los)}')
        if errs_nlos:
            check('NLOS ranges present', True,
                  f'median error {statistics.median(errs_nlos) * 100:.1f} cm '
                  f'over {len(errs_nlos)}')
    else:
        check('UWB ground truth', False, 'no /uwb/truth messages')

    if n.arrays:
        a = n.arrays[-1]
        check('/uwb/ranges layout', len(a.data) == len(n.expected), f'{len(a.data)} entries')
    else:
        check('/uwb/ranges', False, 'no messages')
    check('ground truth odometry', len(n.odom) > 0, f'{len(n.odom)} messages')

    width = max(len(r[0]) for r in results)
    print('\nUWB-IMU Sandbox self check')
    print('-' * (width + 30))
    for name, ok, detail in results:
        print(f'{"PASS" if ok else "FAIL"}  {name.ljust(width)}  {detail}')
    failed = [r for r in results if not r[1]]
    print('-' * (width + 30))
    print('ALL CHECKS PASSED' if not failed else f'{len(failed)} CHECK(S) FAILED')
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if not failed else 1)


if __name__ == '__main__':
    main()
