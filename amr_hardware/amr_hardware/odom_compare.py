#!/usr/bin/env python3
"""
odom_compare.py - so sánh các nguồn odom + gyro trên 1 terminal.

Nguồn: wheel (/odom_raw), rf2o (/odom_rf2o), VO (/rtabmap/odom),
       EKF (/odometry/filtered), gyro (/imu/data_raw).
Nguồn chưa publish sẽ hiện NO DATA, không làm script lỗi.

Cách dùng:
  python3 odom_compare.py
  python3 odom_compare.py --real-dist 2.0     # sau khi chạy thẳng 2.0 m thật
  python3 odom_compare.py --real-yaw 360      # sau khi quay 360 độ thật
  python3 odom_compare.py --label A_thang_2m_lan1 --csv runs.csv
      # Dừng robot -> Ctrl+C -> script hỏi số đo thật (thước/góc) rồi ghi
      # 1 dòng/nguồn vào runs.csv để so nhiều lần chạy

Ctrl+C để dừng, bảng cuối cùng được in lại để chép số liệu.
Muốn đo lại từ 0: Ctrl+C rồi chạy lại.
"""
import argparse
import csv
import math
import os
import statistics
import time
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

# (tên, topic). Sửa topic ở đây nếu cần.
SOURCES = [
    ('wheel', '/odom_raw'),
    ('rf2o', '/odom_rf2o'),
    ('VO', '/rtabmap/odom'),
    ('EKF', '/odometry/filtered'),
]
WIN = 50          # số mẫu dùng để tính phương sai khi đứng yên
DT_MAX = 0.5      # bỏ qua bước tích phân gyro nếu hở quá lâu [s]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def var_of(win):
    return statistics.pvariance(win) if len(win) >= 5 else float('nan')


class Track:
    def __init__(self, name, topic):
        self.name, self.topic = name, topic
        self.arrivals = deque(maxlen=30)
        self.count = 0
        self.x0 = self.y0 = None
        self.disp = 0.0
        self.yaw = 0.0
        self.last_yaw = None
        self.vx = self.wz = 0.0
        self.vx_win = deque(maxlen=WIN)
        self.wz_win = deque(maxlen=WIN)
        self.cov_vx = self.cov_wz = self.cov_pos = float('nan')

    def hz(self):
        if len(self.arrivals) < 2 or time.time() - self.arrivals[-1] > 2.0:
            return 0.0
        return (len(self.arrivals) - 1) / (self.arrivals[-1] - self.arrivals[0])


class OdomCompare(Node):
    def __init__(self, args):
        super().__init__('odom_compare')
        self.args = args
        self.tracks = [Track(n, t) for n, t in SOURCES]
        for tr in self.tracks:
            self.create_subscription(
                Odometry, tr.topic,
                lambda m, tr=tr: self.odom_cb(tr, m), qos_profile_sensor_data)

        # gyro
        self.gyro = Track('gyro', '/imu/data_raw')
        self.last_t = None
        self.create_subscription(Imu, self.gyro.topic, self.imu_cb,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self.report)

    def odom_cb(self, tr, m):
        tr.arrivals.append(time.time())
        tr.count += 1
        p = m.pose.pose
        yaw = yaw_of(p.orientation)
        if tr.x0 is None:
            tr.x0, tr.y0, tr.last_yaw = p.position.x, p.position.y, yaw
        tr.disp = math.hypot(p.position.x - tr.x0, p.position.y - tr.y0)
        tr.yaw += wrap(yaw - tr.last_yaw)
        tr.last_yaw = yaw
        tr.vx = m.twist.twist.linear.x
        tr.wz = m.twist.twist.angular.z
        tr.vx_win.append(tr.vx)
        tr.wz_win.append(tr.wz)
        tr.cov_vx = m.twist.covariance[0]
        tr.cov_wz = m.twist.covariance[35]
        tr.cov_pos = m.pose.covariance[0]

    def imu_cb(self, m):
        g = self.gyro
        g.arrivals.append(time.time())
        g.count += 1
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.last_t is not None:
            dt = t - self.last_t
            if 0.0 < dt < DT_MAX:
                g.yaw += m.angular_velocity.z * dt
        self.last_t = t
        g.wz = m.angular_velocity.z
        g.wz_win.append(g.wz)
        g.cov_wz = m.angular_velocity_covariance[8]

    def row(self, tr, is_gyro=False):
        if tr.hz() == 0.0 and not tr.arrivals:
            return f"{tr.name:<6} {tr.topic:<20} NO DATA"
        status = 'OK'
        if tr.hz() == 0.0:
            status = 'STALE'
        elif tr.name == 'VO' and tr.cov_pos > 1.0:
            status = 'LOST?'

        def f(v, w, p):
            return f"{'-':>{w}}" if (v != v) else f"{v:{w}.{p}f}"

        disp = '-' if is_gyro else f"{tr.disp:.3f}"
        yaw = math.degrees(tr.yaw)
        vx = '-' if is_gyro else f"{tr.vx:.3f}"
        line = (f"{tr.name:<6} {tr.topic:<20} {tr.hz():5.1f} "
                f"{disp:>8} {yaw:9.1f} {vx:>7} {tr.wz:8.4f} "
                f"{'-' if is_gyro else f(var_of(tr.vx_win), 9, 6):>9} "
                f"{f(var_of(tr.wz_win), 9, 6)} "
                f"{'-' if is_gyro else f(tr.cov_vx, 8, 4):>8} "
                f"{f(tr.cov_wz, 8, 4)} {status}")

        a = self.args
        extra = ''
        if a.real_dist and not is_gyro:
            extra += f" | dist err {(tr.disp - a.real_dist) / a.real_dist * 100:+.1f}%"
        if a.real_yaw:
            e = (math.degrees(tr.yaw) - a.real_yaw) / a.real_yaw * 100
            extra += f" | yaw err {e:+.1f}%"
        return line + extra

    def render(self):
        head = (f"{'src':<6} {'topic':<20} {'Hz':>5} {'disp[m]':>8} "
                f"{'yaw[deg]':>9} {'vx':>7} {'wz':>8} {'var_vx':>9} "
                f"{'var_wz':>9} {'cov_vx':>8} {'cov_wz':>8} status")
        lines = [head, '-' * len(head)]
        lines += [self.row(t) for t in self.tracks]
        lines.append(self.row(self.gyro, is_gyro=True))
        lines += [
            '-' * len(head),
            'disp = khoảng cách thẳng từ điểm bắt đầu | yaw = góc cộng dồn (unwrap)',
            'var_* = phương sai đo thực tế trong ~%d mẫu gần nhất (đọc khi ĐỨNG YÊN)' % WIN,
            'cov_*  = covariance mà chính message đang khai báo',
        ]
        return '\n'.join(lines)

    def save_csv(self):
        a = self.args
        if not a.csv:
            return
        new = not os.path.exists(a.csv)
        with open(a.csv, 'a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(['label', 'source', 'msgs', 'disp_m', 'yaw_deg',
                            'real_dist_m', 'real_yaw_deg', 'dist_err_pct',
                            'yaw_err_pct', 'var_vx', 'var_wz', 'cov_pos_last'])
            for tr in self.tracks + [self.gyro]:
                is_g = tr is self.gyro
                yaw = math.degrees(tr.yaw)
                d_err = ((tr.disp - a.real_dist) / a.real_dist * 100
                         if a.real_dist and not is_g else '')
                y_err = ((yaw - a.real_yaw) / a.real_yaw * 100
                         if a.real_yaw else '')
                w.writerow([a.label, tr.name, tr.count,
                            '' if is_g else f"{tr.disp:.4f}", f"{yaw:.2f}",
                            a.real_dist or '', a.real_yaw or '',
                            d_err if d_err == '' else f"{d_err:.2f}",
                            y_err if y_err == '' else f"{y_err:.2f}",
                            '' if is_g else var_of(tr.vx_win),
                            var_of(tr.wz_win),
                            '' if is_g else tr.cov_pos])
        print(f"Đã ghi kết quả vào {a.csv} (label={a.label})")

    def report(self):
        print('\033[H\033[J' + self.render(), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--real-dist', type=float, default=0.0,
                    help='quãng đường thẳng thật đo bằng thước [m]')
    ap.add_argument('--real-yaw', type=float, default=0.0,
                    help='góc quay thật [độ]')
    ap.add_argument('--label', default='run',
                    help='tên lần chạy, ví dụ A_thang_2m_lan1')
    ap.add_argument('--csv', default='',
                    help='file CSV để ghi thêm kết quả khi thoát')
    args = ap.parse_args()

    rclpy.init()
    node = OdomCompare(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print('\n' + node.render())
    if args.csv:
        # số đo thật chỉ biết sau khi dừng robot, nên hỏi tại đây (Enter = bỏ qua)
        try:
            if not args.real_dist:
                v = input('Khoảng cách thật đo được [m] (Enter bỏ qua): ').strip()
                args.real_dist = float(v) if v else 0.0
            if not args.real_yaw:
                v = input('Góc quay thật [độ] (Enter bỏ qua): ').strip()
                args.real_yaw = float(v) if v else 0.0
        except (ValueError, EOFError):
            pass
        node.save_csv()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()