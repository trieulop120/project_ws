#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

def show(tag, t):
    q = t.transform.rotation
    yaw = math.degrees(2 * math.atan2(q.z, q.w))
    print(f"{tag:22s} x={t.transform.translation.x:+.3f} y={t.transform.translation.y:+.3f} yaw={yaw:+6.1f}°")

rclpy.init(); n = Node('tf_snap')
buf = Buffer(); TransformListener(buf, n)
for _ in range(30):
    rclpy.spin_once(n, timeout_sec=0.1)
for a, b in (('map', 'odom'), ('odom', 'base_footprint'), ('map', 'base_footprint')):
    try:
        show(f"{a}->{b}", buf.lookup_transform(a, b, rclpy.time.Time()))
    except Exception as e:
        print(f"{a}->{b}: {e}")