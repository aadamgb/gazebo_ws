#!/usr/bin/env python3
# usage: odom_rate_probe.py [seconds] [odometry topics...]  (run with the session RMW_IMPLEMENTATION)
# Per topic: count, rate over sim time (header stamps) and wall time, gap stats
# for stamps vs. arrival, and how many stamps were skipped (lost upstream).
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
qos_profile_sensor_data = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

TOPICS = sys.argv[2:] or ["/uav1/mavros/local_position/odom", "/uav1/mavros/odometry/in", "/uav1/hw_api/odometry"]
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0


class Probe(Node):
    def __init__(self):
        super().__init__("odom_probe")
        self.data = {t: [] for t in TOPICS}
        self.clock = []
        for t in TOPICS:
            self.create_subscription(Odometry, t, lambda m, t=t: self.data[t].append(
                (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, time.monotonic())), qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", lambda m: self.clock.append(
            (m.clock.sec + m.clock.nanosec * 1e-9, time.monotonic())), qos_profile_sensor_data)


def stats(name, arr):
    if len(arr) < 3:
        print(f"{name}: {len(arr)} msgs")
        return
    a = np.array(arr)
    st, rx = a[:, 0], a[:, 1]
    dst, drx = np.diff(st) * 1e3, np.diff(rx) * 1e3
    print(f"{name}\n  msgs {len(a)}  rate(stamp) {len(a) / (st[-1] - st[0]):.1f} Hz  rate(wall) {len(a) / (rx[-1] - rx[0]):.1f} Hz")
    print(f"  stamp gaps ms: min {dst.min():.1f} med {np.median(dst):.1f} p99 {np.percentile(dst, 99):.1f} max {dst.max():.1f}  dup/backwards {np.sum(dst <= 0)}")
    print(f"  arrival gaps ms: min {drx.min():.1f} med {np.median(drx):.1f} p99 {np.percentile(drx, 99):.1f} max {drx.max():.1f}")
    vals, cnt = np.unique(np.round(dst, 0), return_counts=True)
    top = sorted(zip(cnt, vals), reverse=True)[:6]
    print("  stamp gap histogram (ms:count): " + ", ".join(f"{v:g}:{c}" for c, v in top))


def main():
    rclpy.init()
    n = Probe()
    t0 = time.monotonic()
    while time.monotonic() - t0 < DURATION:
        rclpy.spin_once(n, timeout_sec=0.05)
    for t in TOPICS:
        stats(t, n.data[t])
    if len(n.clock) > 2:
        c = np.array(n.clock)
        print(f"/clock: msgs {len(c)}  RTF {(c[-1, 0] - c[0, 0]) / (c[-1, 1] - c[0, 1]):.3f}  arrival gap max {np.diff(c[:, 1]).max() * 1e3:.1f} ms")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
