#!/usr/bin/env python3
"""
odom_calib.py - đo, so sánh và TÍNH HỆ SỐ HIỆU CHỈNH cho encoder / camera VO / IMU gyro.
Gộp odom_compare.py + tf_snap: 1 file duy nhất.

Nguồn: wheel (/odom_raw), rf2o (/odom_rf2o), VO (/rtabmap/odom), EKF (/odometry/filtered),
       gyro (/imu/data_raw), TF map->odom, odom->base, map->base (tham chiếu, KHÔNG phải sự thật tuyệt đối).
Nguồn chưa publish sẽ hiện NO DATA, không làm script lỗi.

=== QUY TRÌNH (làm theo thứ tự, mỗi loại >= 3 lần) ===
 1) Đứng yên (đo bias gyro + nhiễu):
      python3 odom_calib.py run --kind static --label static_1 --duration 30
 2) Chạy thẳng 1 m / 2 m / 3 m (đánh dấu điểm đầu/cuối, đo bằng thước):
      python3 odom_calib.py run --kind straight --label S1m_1
      -> chạy script, robot ĐỨNG YÊN, bấm Enter để zero, lái robot, dừng, Ctrl+C,
         nhập số đo thật (thước). Có thể truyền sẵn: --real-dist 2.0
 3) Đi sát tường (đo khoảng cách tới tường lúc đầu/cuối -> độ lệch ngang thật):
      python3 odom_calib.py run --kind wall --label W3m_1
      -> nhập: quãng đường dọc tường, độ lệch ngang (+ là lệch TRÁI), góc lệch cuối (nếu đo được)
 4) Quay tại chỗ 360 độ (đánh dấu hướng robot lên sàn), cả CCW (+360) và CW (-360):
      python3 odom_calib.py run --kind rotate --label R360_ccw_1
 5) Xem báo cáo tổng hợp (tự cập nhật sau mỗi lần chạy):
      python3 odom_calib.py report --wheel-radius 0.0825 --wheelbase 0.477

Dữ liệu lưu trong --out-dir (mặc định calib_data/):
   runs.jsonl   mọi lần chạy (dữ liệu thô)      report.md   báo cáo tổng hợp + hệ số đề xuất
   <label>.txt  bảng chụp lại của từng lần chạy
Sau khi sửa tham số (bán kính bánh, wheelbase, bias...) hãy dùng --out-dir MỚI (vd calib_v2)
để không trộn dữ liệu cũ/mới.
"""
import argparse
import json
import math
import os
import select
import statistics
import sys
import threading
import time
from collections import deque
from datetime import datetime

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.utilities import remove_ros_args
    from sensor_msgs.msg import Imu
    from tf2_ros import Buffer, TransformListener
    HAVE_ROS = True
except ImportError:          # cho phép chạy `report` trên máy không có ROS
    HAVE_ROS = False
    Node = object

# (tên, topic). Sửa topic ở đây nếu cần.
SOURCES = [
    ('wheel', '/odom_raw'),
    ('rf2o', '/odom_rf2o'),
    ('VO', '/rtabmap/odom'),
    ('EKF', '/odometry/filtered'),
]
GYRO_TOPIC = '/imu/data_raw'
WIN = 50            # số mẫu tính phương sai "trực tiếp"
DT_MAX = 0.5        # bỏ bước tích phân gyro nếu hở quá lâu [s]
KINDS = ('static', 'straight', 'wall', 'rotate')
NAN = float('nan')
CV_MAX = 3.0        # % - độ lặp lại tối đa để tin một hệ số
NAMES = ['wheel', 'rf2o', 'VO', 'EKF', 'gyro', 'map_tf']


# --------------------------------------------------------------------------- tiện ích
def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def ok(v):
    return v is not None and v == v


def pvar(win):
    return statistics.pvariance(win) if len(win) >= 5 else NAN


def fmt(v, w, p, sign=False):
    if not ok(v):
        return f"{'-':>{w}}"
    return f"{v:{'+' if sign else ''}{w}.{p}f}"


def clean(o):
    """NaN -> None để json hợp lệ."""
    if isinstance(o, float) and o != o:
        return None
    if isinstance(o, dict):
        return {k: clean(v) for k, v in o.items()}
    if isinstance(o, list):
        return [clean(v) for v in o]
    return o


class Stat:
    """Welford: mean/std chạy toàn bộ lần đo."""
    def __init__(self):
        self.n, self.mean, self.m2 = 0, 0.0, 0.0

    def add(self, x):
        self.n += 1
        d = x - self.mean
        self.mean += d / self.n
        self.m2 += d * (x - self.mean)

    @property
    def std(self):
        return math.sqrt(self.m2 / self.n) if self.n > 1 else NAN


class Track:
    def __init__(self, name, topic, is_gyro=False):
        self.name, self.topic, self.is_gyro = name, topic, is_gyro
        self.arrivals = deque(maxlen=30)
        self.reset()

    def reset(self):
        self.count = 0
        self.x0 = self.y0 = self.yaw0 = None
        self.x = self.y = 0.0
        self.last_xy = None
        self.path = 0.0
        self.yaw = 0.0
        self.last_yaw = None
        self.vx = self.wz = 0.0
        self.vx_win, self.wz_win = deque(maxlen=WIN), deque(maxlen=WIN)
        self.vx_st, self.wz_st = Stat(), Stat()
        self.cov_vx = self.cov_wz = self.cov_pos = NAN
        self.lost = 0
        self.t_first = self.t_last = None
        self.last_t = None
        self.T = 0.0                      # tổng dt đã tích phân (gyro)

    def hz(self):
        if len(self.arrivals) < 2 or time.time() - self.arrivals[-1] > 2.0:
            return 0.0
        return (len(self.arrivals) - 1) / (self.arrivals[-1] - self.arrivals[0])

    def rel(self):
        """(tiến, ngang-trái) trong hệ tọa độ của tư thế bắt đầu."""
        if self.x0 is None:
            return 0.0, 0.0
        dx, dy = self.x - self.x0, self.y - self.y0
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        return dx * c + dy * s, -dx * s + dy * c


# --------------------------------------------------------------------------- node đo
class Calib(Node):
    def __init__(self, args):
        super().__init__('odom_calib')
        self.args = args
        self.tracks = [Track(n, t) for n, t in SOURCES]
        self.gyro = Track('gyro', GYRO_TOPIC, True)
        for tr in self.tracks:
            self.create_subscription(
                Odometry, tr.topic, lambda m, tr=tr: self.odom_cb(tr, m),
                qos_profile_sensor_data)
        self.create_subscription(Imu, GYRO_TOPIC, self.imu_cb, qos_profile_sensor_data)

        self.pairs = {
            'map->odom': (args.map_frame, args.odom_frame),
            'odom->base': (args.odom_frame, args.base_frame),
            'map->base': (args.map_frame, args.base_frame),
        }
        self.tfbuf = Buffer()
        self.tfl = TransformListener(self.tfbuf, self)
        self.reset_req = False
        self.live = True
        self.real = {'dist': args.real_dist, 'yaw': args.real_yaw, 'lat': args.real_lat}
        self.reset()
        self.create_timer(0.2, self.tick_tf)
        self.create_timer(1.0, self.report)

    # ---- TF
    def tf_pose(self, pair):
        try:
            t = self.tfbuf.lookup_transform(pair[0], pair[1], Time())
        except Exception:
            return None
        p = t.transform.translation
        return (p.x, p.y, yaw_of(t.transform.rotation))

    def tick_tf(self):
        for k, pr in self.pairs.items():          # nếu lúc reset chưa có TF thì bù muộn
            if self.tf0.get(k) is None:
                self.tf0[k] = self.tf_pose(pr)
        p = self.tf_pose(self.pairs['map->base'])
        if not p:
            return
        if self.map_last_yaw is not None:
            self.map_yaw_acc += wrap(p[2] - self.map_last_yaw)
        self.map_last_yaw = p[2]

    def reset(self):
        for t in self.tracks + [self.gyro]:
            t.reset()
        self.t_start = time.time()
        self.tf0 = {k: self.tf_pose(v) for k, v in self.pairs.items()}
        self.map_yaw_acc = 0.0
        p = self.tf0.get('map->base')
        self.map_last_yaw = p[2] if p else None

    # ---- callbacks
    def odom_cb(self, tr, m):
        now = time.time()
        tr.arrivals.append(now)
        tr.count += 1
        tr.t_last = now
        if tr.t_first is None:
            tr.t_first = now
        p = m.pose.pose
        yaw = yaw_of(p.orientation)
        x, y = p.position.x, p.position.y
        if tr.x0 is None:
            tr.x0, tr.y0, tr.yaw0 = x, y, yaw
            tr.last_yaw, tr.last_xy = yaw, (x, y)
        tr.path += math.hypot(x - tr.last_xy[0], y - tr.last_xy[1])
        tr.last_xy = (x, y)
        tr.x, tr.y = x, y
        tr.yaw += wrap(yaw - tr.last_yaw)
        tr.last_yaw = yaw
        tr.vx = m.twist.twist.linear.x
        tr.wz = m.twist.twist.angular.z
        tr.vx_win.append(tr.vx)
        tr.wz_win.append(tr.wz)
        tr.vx_st.add(tr.vx)
        tr.wz_st.add(tr.wz)
        tr.cov_vx = m.twist.covariance[0]
        tr.cov_wz = m.twist.covariance[35]
        tr.cov_pos = m.pose.covariance[0]
        if tr.name == 'VO' and tr.cov_pos > 1.0:
            tr.lost += 1

    def imu_cb(self, m):
        g, now = self.gyro, time.time()
        g.arrivals.append(now)
        g.count += 1
        g.t_last = now
        if g.t_first is None:
            g.t_first = now
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = now
        wz = m.angular_velocity.z
        if g.last_t is not None:
            dt = t - g.last_t
            if 0.0 < dt < DT_MAX:
                g.yaw += 0.5 * (wz + g.wz) * dt        # hình thang
                g.T += dt
        g.last_t = t
        g.wz = wz
        g.wz_win.append(wz)
        g.wz_st.add(wz)
        g.cov_wz = m.angular_velocity_covariance[8]

    # ---- số liệu
    def status(self, tr):
        if not tr.arrivals:
            return 'NO DATA'
        if tr.hz() == 0.0:
            return 'STALE'
        if tr.name == 'VO' and ok(tr.cov_pos) and tr.cov_pos > 1.0:
            return 'LOST?'
        return 'OK'

    def metrics(self, tr):
        el = (tr.t_last - tr.t_first) if tr.t_first else 0.0
        m = dict(source=tr.name, topic=tr.topic, msgs=tr.count, hz=tr.hz(),
                 rate=(tr.count / el if el > 0 else NAN), T=tr.T if tr.is_gyro else el,
                 yaw=math.degrees(tr.yaw), wz=tr.wz,
                 wz_mean=tr.wz_st.mean if tr.wz_st.n else NAN, wz_std=tr.wz_st.std,
                 var_wz=pvar(tr.wz_win), cov_wz=tr.cov_wz, status=self.status(tr))
        if tr.is_gyro:
            m.update(fwd=None, lat=None, disp=None, path=None, vx=None, vx_mean=None,
                     vx_std=None, var_vx=None, cov_vx=None, cov_pos=None, lost_frac=None)
        else:
            fwd, lat = tr.rel()
            m.update(fwd=fwd, lat=lat, disp=math.hypot(fwd, lat), path=tr.path, vx=tr.vx,
                     vx_mean=tr.vx_st.mean if tr.vx_st.n else NAN, vx_std=tr.vx_st.std,
                     var_vx=pvar(tr.vx_win), cov_vx=tr.cov_vx, cov_pos=tr.cov_pos,
                     lost_frac=(tr.lost / tr.count if tr.count else 0.0))
        return m

    def map_metrics(self):
        a, b = self.tf0.get('map->base'), self.tf_pose(self.pairs['map->base'])
        if not a or not b:
            return None
        dx, dy = b[0] - a[0], b[1] - a[1]
        c, s = math.cos(a[2]), math.sin(a[2])
        fwd, lat = dx * c + dy * s, -dx * s + dy * c
        return dict(source='map_tf', topic=f"TF {self.args.map_frame}->{self.args.base_frame}",
                    msgs=0, hz=None, rate=None, T=time.time() - self.t_start, fwd=fwd, lat=lat,
                    disp=math.hypot(fwd, lat), path=None, yaw=math.degrees(self.map_yaw_acc),
                    vx=None, wz=None, wz_mean=None, wz_std=None, vx_mean=None, vx_std=None,
                    var_vx=None, var_wz=None, cov_vx=None, cov_wz=None, cov_pos=None,
                    lost_frac=None, status='REF')

    def tf_snapshot(self):
        lines, jump = [], None
        for k, pr in self.pairs.items():
            now, st = self.tf_pose(pr), self.tf0.get(k)
            if not now:
                lines.append(f"  {k:<11} n/a")
                continue
            s = f"  {k:<11} x={now[0]:+.3f} y={now[1]:+.3f} yaw={math.degrees(now[2]):+7.1f}°"
            if st:
                d = (now[0] - st[0], now[1] - st[1], math.degrees(wrap(now[2] - st[2])))
                s += f"   | Δ từ lúc zero: dx={d[0]:+.3f} dy={d[1]:+.3f} dyaw={d[2]:+.1f}°"
                if k == 'map->odom':
                    jump = dict(dx=d[0], dy=d[1], dyaw=d[2], dist=math.hypot(d[0], d[1]))
            lines.append(s)
        return lines, jump

    def snapshot(self):
        ms = [self.metrics(t) for t in self.tracks] + [self.metrics(self.gyro)]
        mm = self.map_metrics()
        if mm:
            ms.append(mm)
        lines, jump = self.tf_snapshot()
        return dict(metrics=ms, tf_lines=lines, jump=jump,
                    elapsed=time.time() - self.t_start)

    def report(self):
        if self.live:
            print('\033[H\033[J' + render(self.snapshot(), self.real, self.args.kind), flush=True)


# --------------------------------------------------------------------------- hiển thị
def err_text(m, real):
    parts = []
    if real.get('dist') and ok(m.get('fwd')):
        parts.append(f"dist {(m['fwd'] - real['dist']) / real['dist'] * 100:+.1f}%")
    ry = real.get('yaw')
    if ry is not None and ok(m.get('yaw')):
        if abs(ry) >= 45:
            parts.append(f"yaw {(m['yaw'] - ry) / ry * 100:+.1f}%")
        else:
            parts.append(f"yaw {m['yaw'] - ry:+.1f}°")
    if real.get('lat') is not None and ok(m.get('lat')):
        parts.append(f"lat {m['lat'] - real['lat']:+.3f}m")
    return (' | ' + ' | '.join(parts)) if parts else ''


def row_text(m, real):
    if m['status'] == 'NO DATA':
        return f"{m['source']:<7} {m['topic']:<22} NO DATA"
    line = (f"{m['source']:<7} {m['topic']:<22} {fmt(m['hz'], 5, 1)} {fmt(m['fwd'], 8, 3)} "
            f"{fmt(m['lat'], 8, 3)} {fmt(m['yaw'], 9, 1)} {fmt(m['vx'], 7, 3)} "
            f"{fmt(m['wz'], 8, 4)} {fmt(m['var_vx'], 9, 6)} {fmt(m['var_wz'], 9, 6)} "
            f"{fmt(m['cov_wz'], 8, 4)} {m['status']}")
    return line + err_text(m, real)


def render(snap, real, kind):
    head = (f"{'src':<7} {'topic':<22} {'Hz':>5} {'fwd[m]':>8} {'lat[m]':>8} {'yaw[deg]':>9} "
            f"{'vx':>7} {'wz':>8} {'var_vx':>9} {'var_wz':>9} {'cov_wz':>8} status")
    out = [f"[{kind}] đã đo {snap['elapsed']:.1f}s   (Enter = zero lại, Ctrl+C = kết thúc)",
           head, '-' * len(head)]
    out += [row_text(m, real) for m in snap['metrics']]
    out += ['-' * len(head),
            'fwd = quãng tiến theo hướng ban đầu | lat = lệch ngang (+ là TRÁI) | yaw = góc cộng dồn',
            'var_* = phương sai ~%d mẫu gần nhất (đọc khi ĐỨNG YÊN) | cov_wz = covariance message khai báo' % WIN,
            'map_tf = tham chiếu từ TF (phụ thuộc chất lượng định vị), không phải sự thật tuyệt đối',
            'TF:'] + snap['tf_lines']
    return '\n'.join(out)


# --------------------------------------------------------------------------- lưu kết quả
def ask(prompt):
    try:
        v = input(prompt).strip().replace(',', '.')
    except EOFError:
        return None
    if not v:
        return None
    try:
        return float(v)
    except ValueError:
        print('  (bỏ qua: không phải số)')
        return None


def prompt_real(args, real):
    k = args.kind
    if args.no_prompt or k == 'static':
        return
    print('\nNhập số đo THẬT (Enter = bỏ qua):')
    if k in ('straight', 'wall'):
        if real['dist'] is None:
            real['dist'] = ask('  Quãng đường thật đi được [m]: ')
        if real['lat'] is None:
            real['lat'] = ask('  Lệch ngang thật so với đường thẳng ban đầu [m] (+ trái, - phải): ')
        if real['yaw'] is None:
            real['yaw'] = ask('  Góc lệch hướng cuối thật [độ] (+ ngược chiều kim đồng hồ; Enter nếu không đo): ')
    elif k == 'rotate':
        if real['yaw'] is None:
            real['yaw'] = ask('  Góc quay thật [độ] (CCW dương, CW âm, vd 360 hoặc -360): ')


def save_run(args, snap, real):
    os.makedirs(args.out_dir, exist_ok=True)
    rec = dict(label=args.label, kind=args.kind, time=datetime.now().isoformat(timespec='seconds'),
               duration=snap['elapsed'], real_dist=real['dist'], real_yaw=real['yaw'],
               real_lat=real['lat'], note=args.note, sources=snap['metrics'],
               map_odom_jump=snap['jump'])
    with open(os.path.join(args.out_dir, 'runs.jsonl'), 'a', encoding='utf-8') as f:
        f.write(json.dumps(clean(rec), ensure_ascii=False) + '\n')
    txt = os.path.join(args.out_dir, f"{args.label}.txt")
    with open(txt, 'w', encoding='utf-8') as f:
        f.write(f"label={args.label} kind={args.kind} time={rec['time']} note={args.note}\n")
        f.write(f"real: dist={real['dist']} yaw={real['yaw']} lat={real['lat']}\n\n")
        f.write(render(snap, real, args.kind) + '\n')
    print(f"\nĐã lưu: {os.path.abspath(txt)}\n        {os.path.abspath(os.path.join(args.out_dir, 'runs.jsonl'))}")


# --------------------------------------------------------------------------- phân tích / báo cáo
def get(run, name):
    for s in run['sources']:
        if s['source'] == name and s.get('status') != 'NO DATA':
            return s
    return None


def summarize(vals):
    vals = [v for v in vals if ok(v)]
    if not vals:
        return None
    n, med = len(vals), statistics.median(vals)
    sd = statistics.stdev(vals) if n > 1 else NAN
    return dict(n=n, med=med, sd=sd, cv=(sd / abs(med) * 100 if n > 1 and med else NAN))


def yaw_eff(s, bias):
    y = s.get('yaw')
    if not ok(y):
        return None
    if s['source'] == 'gyro' and bias:
        y -= math.degrees(bias * (s.get('T') or 0.0))      # trừ bias đã đo ở lần đứng yên
    return y


def analyze(runs):
    A = {}
    stat_runs = [r for r in runs if r['kind'] == 'static']
    gb = summarize([get(r, 'gyro')['wz_mean'] for r in stat_runs if get(r, 'gyro')])
    bias = gb['med'] if gb else 0.0
    A['bias'], A['bias_sum'] = bias, gb

    dist = {n: [] for n in NAMES}
    yaw = {n: [] for n in NAMES}
    yaw_dir = {'ccw': {n: [] for n in NAMES}, 'cw': {n: [] for n in NAMES}}
    for r in runs:
        rd, ry = r.get('real_dist'), r.get('real_yaw')
        for n in NAMES:
            s = get(r, n)
            if not s:
                continue
            if r['kind'] in ('straight', 'wall') and rd and ok(s.get('fwd')) and s['fwd'] > 0.05:
                dist[n].append(rd / s['fwd'])
            if r['kind'] == 'rotate' and ry:
                y = yaw_eff(s, bias)
                if ok(y) and abs(y) > 5 and y * ry > 0:
                    yaw[n].append(ry / y)
                    yaw_dir['ccw' if ry > 0 else 'cw'][n].append(ry / y)
    A['dist'] = {n: summarize(v) for n, v in dist.items()}
    A['yaw'] = {n: summarize(v) for n, v in yaw.items()}
    A['yaw_dir'] = {d: {n: summarize(v) for n, v in dd.items()} for d, dd in yaw_dir.items()}

    # độ lệch hướng không bù được khi chạy thẳng -> mất cân bằng bánh trái/phải
    lr = []
    for r in runs:
        s, rd = get(r, 'wheel'), r.get('real_dist')
        if r['kind'] not in ('straight', 'wall') or not s or not rd:
            continue
        if r.get('real_yaw') is not None:
            th = r['real_yaw']
        elif r.get('real_lat') is not None:
            th = math.degrees(2.0 * r['real_lat'] / rd)    # cung tròn đều: hướng = 2*lat/s
        else:
            continue
        d = th - s['yaw']
        lr.append(dict(label=r['label'], delta=d, per_m=d / rd))
    A['lr'] = lr
    A['lr_sum'] = summarize([x['per_m'] for x in lr])
    if lr and A['lr_sum']:
        sg = 1 if A['lr_sum']['med'] >= 0 else -1
        A['lr_sign'] = sum(1 for x in lr if x['per_m'] * sg > 0) / len(lr)
    return A


def reliable(sm):
    return bool(sm) and sm['n'] >= 3 and ok(sm['cv']) and sm['cv'] <= CV_MAX


def verdict(sm):
    if not sm:
        return '-'
    if sm['n'] < 3:
        return f"ít dữ liệu (n={sm['n']}, cần >=3)"
    if not ok(sm['cv']) or sm['cv'] > CV_MAX:
        return 'KHÔNG ổn định - chưa nên chỉnh (kiểm tra trượt bánh, cơ khí, tốc độ, cách đo)'
    if abs(sm['med'] - 1.0) < 0.01:
        return 'đã đủ tốt (<1%), không cần chỉnh'
    return f"nên chỉnh x{sm['med']:.4f} ({(sm['med'] - 1) * 100:+.1f}%)"


def nf(v, p=3, sign=False):
    if not ok(v):
        return '-'
    return f"{v:+.{p}f}" if sign else f"{v:.{p}f}"


def md_table(headers, rows):
    out = ['| ' + ' | '.join(headers) + ' |', '|' + '|'.join(['---'] * len(headers)) + '|']
    out += ['| ' + ' | '.join(str(c) for c in r) + ' |' for r in rows]
    return '\n'.join(out)


def spread_rows(runs, A):
    rows = []
    for r in runs:
        if r['kind'] in ('straight', 'wall'):
            key, srcs, unit = 'fwd', ['wheel', 'rf2o', 'VO'], 'm'
        elif r['kind'] == 'rotate':
            key, srcs, unit = 'yaw', ['wheel', 'rf2o', 'VO', 'gyro'], 'deg'
        else:
            continue
        before, after, used = [], [], []
        for n in srcs:
            s = get(r, n)
            if not s:
                continue
            v = s['fwd'] if key == 'fwd' else yaw_eff(s, A['bias'])
            if not ok(v):
                continue
            sm = (A['dist'] if key == 'fwd' else A['yaw']).get(n)
            before.append(v)
            after.append(v * (sm['med'] if sm else 1.0))
            used.append(n)
        if len(before) < 2:
            continue
        rows.append([r['label'], r['kind'], '/'.join(used),
                     f"{max(before) - min(before):.3f} {unit}",
                     f"{max(after) - min(after):.3f} {unit}"])
    return rows


def build_report(runs, wheel_radius=None, wheelbase=None):
    A = analyze(runs)
    kinds = {}
    for r in runs:
        kinds[r['kind']] = kinds.get(r['kind'], 0) + 1
    L = ['# Báo cáo hiệu chỉnh odom (encoder / camera VO / IMU gyro)',
         f"_Tạo lúc {datetime.now():%Y-%m-%d %H:%M:%S} · {len(runs)} lần chạy: "
         + ', '.join(f"{k}={v}" for k, v in kinds.items()) + '_', '']

    # --- 1. hệ số
    L += ['## 1. Hệ số hiệu chỉnh đo được',
          'Hệ số = giá trị THẬT / giá trị ĐO (nhân vào số đo để ra số đúng). '
          f"Chỉ tin khi n>=3 và độ lặp lại CV<={CV_MAX:.0f}%.", '']
    rows = []
    for n in ['wheel', 'rf2o', 'VO', 'EKF', 'map_tf']:
        sm = A['dist'].get(n)
        if sm:
            rows.append([n, 'quãng đường (thẳng)', nf(sm['med'], 4), sm['n'], nf(sm['cv'], 1), verdict(sm)])
    for n in ['wheel', 'rf2o', 'VO', 'EKF', 'gyro', 'map_tf']:
        sm = A['yaw'].get(n)
        if sm:
            rows.append([n + (' (đã trừ bias)' if n == 'gyro' and A['bias'] else ''),
                         'góc quay (rotate)', nf(sm['med'], 4), sm['n'], nf(sm['cv'], 1), verdict(sm)])
    gb = A['bias_sum']
    if gb:
        rows.append(['gyro', 'bias z khi đứng yên [rad/s]', nf(gb['med'], 5, True), gb['n'],
                     nf(gb['sd'], 5), f"= {math.degrees(gb['med']):+.4f} °/s; trừ vào angular_velocity.z"])
    L += [md_table(['Nguồn', 'Đại lượng', 'Hệ số (median)', 'n', 'CV% / sd', 'Kết luận'], rows)
          if rows else '_Chưa đủ dữ liệu có số đo thật._', '']

    # --- 2. áp dụng
    L += ['## 2. Đề xuất áp dụng (mỗi lần chỉ đổi 1 thứ rồi đo lại)', '']
    sd_w, sy_w = A['dist'].get('wheel'), A['yaw'].get('wheel')
    if reliable(sd_w):
        s = f"- **Encoder - bán kính bánh**: nhân x{sd_w['med']:.4f}"
        if wheel_radius:
            s += f" -> wheel_radius {wheel_radius:g} -> **{wheel_radius * sd_w['med']:.5f} m**"
        L.append(s + ' (odom ngắn hơn thật => bán kính thực lớn hơn khai báo).')
    elif sd_w:
        L.append(f"- Encoder - bán kính bánh: CHƯA tin cậy ({verdict(sd_w)}).")
    if reliable(sy_w):
        k = sd_w['med'] if reliable(sd_w) else 1.0
        fac = k / sy_w['med']       # b_new = b * odom_yaw/real_yaw (đã tính ảnh hưởng của bán kính mới)
        s = f"- **Encoder - wheelbase**: nhân x{fac:.4f}"
        if wheelbase:
            s += f" -> wheelbase {wheelbase:g} -> **{wheelbase * fac:.4f} m**"
        L.append(s + ' (đã gộp ảnh hưởng của hệ số bán kính ở trên vì yaw_odom tỉ lệ với bán kính).')
        ccw, cw = A['yaw_dir']['ccw']['wheel'], A['yaw_dir']['cw']['wheel']
        if ccw and cw and abs(ccw['med'] - cw['med']) / ((ccw['med'] + cw['med']) / 2) * 100 > CV_MAX:
            L.append(f"  - Cảnh báo: CCW ({ccw['med']:.4f}) khác CW ({cw['med']:.4f}) > {CV_MAX:.0f}% "
                     '-> trượt bánh không đối xứng / ma sát sàn; đừng chỉ lấy trung bình mù quáng.')
    elif sy_w:
        L.append(f"- Encoder - wheelbase: CHƯA tin cậy ({verdict(sy_w)}).")
    if A['lr_sum']:
        pm = A['lr_sum']['med']
        s = (f"- **Encoder - cân bằng trái/phải**: khi chạy thẳng, hướng thật lệch so với odom "
             f"{pm:+.2f} °/m (dương = robot thật quay trái hơn odom nghĩ); "
             f"cùng dấu {A.get('lr_sign', 0) * 100:.0f}% số lần.")
        L.append(s)
        if wheelbase and A.get('lr_sign', 0) >= 0.8 and len(A['lr']) >= 3:
            x = wheelbase * math.radians(pm)
            L.append(f"  - Ước lượng cR - cL = {x:+.4f} => nhân bán kính phải x{1 + x / 2:.4f}, trái x{1 - x / 2:.4f}. "
                     'Dấu phụ thuộc quy ước encoder của bạn: sau khi đổi, chạy lại thẳng để xác nhận lệch giảm.')
        else:
            L.append('  - Chưa đủ nhất quán để chỉnh (cần >=3 lần, >=80% cùng dấu, và --wheelbase). '
                     'Nếu dấu đổi lung tung = nhiễu trượt/sàn gạch, KHÔNG phải sai hiệu chuẩn.')
    if gb:
        L.append(f"- **Gyro - bias**: trừ {gb['med']:+.5f} rad/s ({math.degrees(gb['med']):+.4f} °/s) "
                 f"(n={gb['n']}, sd={nf(gb['sd'], 5)}). Chỉnh ở firmware/IMU driver thay vì dùng ngưỡng 'chết'.")
    sg = A['yaw'].get('gyro')
    if sg:
        L.append(f"- **Gyro - hệ số góc**: x{sg['med']:.4f} -> {verdict(sg)}.")
    for nm, label in (('VO', 'Camera VO'), ('rf2o', 'rf2o')):
        d, y = A['dist'].get(nm), A['yaw'].get(nm)
        for sm, what in ((d, 'quãng đường'), (y, 'góc')):
            if sm and abs(sm['med'] - 1) > 0.03:
                L.append(f"- **{label} - {what}** lệch {(sm['med'] - 1) * 100:+.1f}%: không có 'bánh xe' để chỉnh; "
                         'kiểm tra hiệu chuẩn camera/ngoại tham số TF (camera->base), độ sâu, đặc điểm cảnh '
                         '(rf2o: hành lang/tường trơn).' if nm == 'VO' else
                         f"- **{label} - {what}** lệch {(sm['med'] - 1) * 100:+.1f}%: kiểm tra TF laser->base, "
                         'scan nhiễu, môi trường ít đặc trưng.')
    lost = [get(r, 'VO')['lost_frac'] for r in runs if get(r, 'VO') and ok(get(r, 'VO').get('lost_frac'))]
    if lost and max(lost) > 0.05:
        L.append(f"- VO mất theo dõi tới {max(lost) * 100:.0f}% số mẫu ở ít nhất 1 lần -> cần gate bằng covariance trước khi vào EKF.")
    L.append('')

    # --- 3. static
    st = [r for r in runs if r['kind'] == 'static']
    L += ['## 3. Nhiễu & trôi khi đứng yên', '']
    if st:
        rows = []
        for n in ['wheel', 'rf2o', 'VO', 'EKF', 'gyro', 'map_tf']:
            ss = [(r, get(r, n)) for r in st if get(r, n)]
            if not ss:
                continue
            f = lambda k, per_min=False: summarize(  # noqa: E731
                [(s[k] / r['duration'] * 60 if per_min else s[k]) for r, s in ss if ok(s.get(k))])
            dr, yr = f('disp', True), f('yaw', True)
            vs, ws = f('vx_std'), f('wz_std')
            rows.append([n, nf(dr['med'], 4) if dr else '-', nf(yr['med'], 3) if yr else '-',
                         nf(vs['med'] ** 2, 7) if vs else '-', nf(ws['med'] ** 2, 7) if ws else '-'])
        L += [md_table(['Nguồn', 'trôi vị trí [m/phút]', 'trôi góc [°/phút]', 'var vx', 'var wz'], rows),
              '', 'var = nhiễu NỀN khi đứng yên. Covariance trong EKF nên >= các giá trị này (thực tế thường 2-5 lần, '
              'vì khi chạy còn trượt bánh). Giá trị 0 hoặc quá nhỏ làm EKF tin quá mức.', '']
    else:
        L += ['_Chưa có lần chạy static (khuyên chạy trước: `--kind static --duration 30`)._', '']

    # --- 4. thống nhất
    sp = spread_rows(runs, A)
    L += ['## 4. Độ thống nhất giữa các nguồn (chênh max-min)', '']
    if sp:
        L += [md_table(['Lần chạy', 'Loại', 'Nguồn', 'Trước hiệu chỉnh', 'Sau (áp hệ số trên)'], sp),
              '', 'Cột "Sau" dùng hệ số tính từ chính dữ liệu này (lạc quan); phải chạy lại sau khi thực sự đổi tham số để xác nhận.', '']
    else:
        L += ['_Chưa đủ dữ liệu._', '']

    # --- 5. từng lần chạy
    L += ['## 5. Từng lần chạy', '']
    for r in runs:
        real = []
        for k, lab in (('real_dist', 'dist'), ('real_yaw', 'yaw'), ('real_lat', 'lat')):
            if r.get(k) is not None:
                real.append(f"{lab}={r[k]}")
        L.append(f"### {r['label']} - {r['kind']} ({r['duration']:.0f}s, {r['time']})"
                 + (f" - {r['note']}" if r.get('note') else ''))
        L.append('Số đo thật: ' + (', '.join(real) if real else '(không có)'))
        if r.get('map_odom_jump'):
            j = r['map_odom_jump']
            L.append(f"map->odom nhảy {j['dist']:.3f} m, {j['dyaw']:+.1f}° trong lúc chạy "
                     '(localizer phải sửa bấy nhiêu = odom đã trôi so với bản đồ).')
        rows = []
        for s in r['sources']:
            if s.get('status') == 'NO DATA':
                rows.append([s['source'], 'NO DATA'] + [''] * 6)
                continue
            y = yaw_eff(s, A['bias'])
            e = err_text(dict(s, yaw=y), dict(dist=r.get('real_dist'), yaw=r.get('real_yaw'), lat=r.get('real_lat')))
            rows.append([s['source'], s.get('status', ''), nf(s.get('fwd'), 3, True), nf(s.get('lat'), 3, True),
                         nf(y, 1, True), nf(s.get('wz_std'), 5) if r['kind'] == 'static' else
                         nf(s.get('lost_frac', None) and s['lost_frac'] * 100, 0),
                         e.replace(' | ', '', 1) if e else '', nf(s.get('rate'), 1)])
        L += [md_table(['Nguồn', 'Trạng thái', 'fwd [m]', 'lat [m]', 'yaw [°]',
                        'wz_std' if r['kind'] == 'static' else 'VO mất %', 'sai số vs thật', 'Hz'], rows), '']

    L += ['## 6. Lưu ý để không chỉnh mù quáng',
          '- Mỗi loại thử >= 3 lần, đổi tốc độ; nếu CV > 3% thì nguyên nhân là cơ khí/sàn/cách đo, không phải hệ số.',
          '- Thứ tự: bias gyro -> bán kính bánh (chạy thẳng) -> wheelbase (quay) -> cân bằng trái/phải -> VO/rf2o.',
          '- Sau mỗi thay đổi tham số: tạo --out-dir mới và chạy lại cùng bộ test để so sánh.',
          '- map_tf chỉ là tham chiếu từ localizer; số đo bằng thước/góc thật luôn ưu tiên hơn.',
          '- Sàn gạch có khe: trượt bánh ngẫu nhiên không bù được bằng hệ số; xử lý bằng covariance + fuse IMU/rf2o.']
    return '\n'.join(L)


def load_runs(out_dir):
    path = os.path.join(out_dir, 'runs.jsonl')
    runs = []
    if os.path.exists(path):
        with open(path, encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        runs.append(json.loads(line))
                    except ValueError:
                        pass
    return runs


def write_report(args, quiet=False):
    runs = load_runs(args.out_dir)
    if not runs:
        print(f"Chưa có dữ liệu trong {args.out_dir}/runs.jsonl")
        return
    md = build_report(runs, args.wheel_radius, args.wheelbase)
    os.makedirs(args.out_dir, exist_ok=True)
    path = os.path.join(args.out_dir, 'report.md')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(md + '\n')
    if not quiet:
        print(md)
    print(f"\nĐã ghi báo cáo: {path}")


# --------------------------------------------------------------------------- main
def key_thread(node, stop):
    while not stop.is_set():
        r, _, _ = select.select([sys.stdin], [], [], 0.2)
        if r:
            if sys.stdin.readline() == '':
                return
            node.reset_req = True


def run(args):
    if not HAVE_ROS:
        sys.exit('Không import được rclpy: hãy source ROS 2 trước (source /opt/ros/humble/setup.bash).')
    if not args.label:
        args.label = f"{args.kind}_{datetime.now():%H%M%S}"
    rclpy.init()
    node = Calib(args)
    for _ in range(10):                      # làm nóng TF buffer
        rclpy.spin_once(node, timeout_sec=0.1)
    node.reset()
    stop = threading.Event()
    th = threading.Thread(target=key_thread, args=(node, stop), daemon=True)
    th.start()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.reset_req:
                node.reset()
                node.reset_req = False
            if args.duration and time.time() - node.t_start >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    node.live = False
    stop.set()
    th.join(timeout=0.5)
    node.tick_tf()
    snap = node.snapshot()                   # chụp NGAY lúc dừng
    print('\n' + render(snap, node.real, args.kind))
    prompt_real(args, node.real)
    snap_txt = render(snap, node.real, args.kind)
    print('\n' + snap_txt)
    if args.no_save:
        print('\n[--no-save] Không lưu file nào.')
    else:
        save_run(args, snap, node.real)
        write_report(args, quiet=True)
    node.destroy_node()
    rclpy.try_shutdown()


def main():
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--out-dir', default='calib_data', help='thư mục lưu dữ liệu & báo cáo')
    common.add_argument('--wheel-radius', type=float, default=None, help='bán kính bánh hiện tại [m] (để in giá trị mới)')
    common.add_argument('--wheelbase', type=float, default=None, help='wheelbase hiện tại [m] (vd 0.477)')

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    r = sub.add_parser('run', parents=[common], help='đo 1 lần chạy và lưu')
    r.add_argument('--kind', choices=KINDS, default='straight')
    r.add_argument('--label', default='', help='vd S2m_1, W3m_2, R360_ccw_1')
    r.add_argument('--real-dist', type=float, default=None, help='quãng đường thật [m]')
    r.add_argument('--real-yaw', type=float, default=None, help='góc thật [độ] (rotate: tổng góc quay; thẳng: góc lệch cuối)')
    r.add_argument('--real-lat', type=float, default=None, help='lệch ngang thật [m], + là trái')
    r.add_argument('--duration', type=float, default=0.0, help='tự dừng sau N giây (dùng cho static)')
    r.add_argument('--note', default='', help='ghi chú (tốc độ, sàn, ...)')
    r.add_argument('--no-save', action='store_true', help='chỉ xem/so sánh, KHÔNG ghi file nào')
    r.add_argument('--no-prompt', action='store_true', help='không hỏi số đo thật khi kết thúc')
    r.add_argument('--map-frame', default='map')
    r.add_argument('--odom-frame', default='odom')
    r.add_argument('--base-frame', default='base_footprint')
    r.set_defaults(func=run)

    p = sub.add_parser('report', parents=[common], help='tạo lại báo cáo tổng hợp từ runs.jsonl')
    p.set_defaults(func=lambda a: write_report(a))

    argv = sys.argv[1:]
    if HAVE_ROS:                              # bỏ --ros-args ... do `ros2 run` truyền vào
        argv = remove_ros_args(sys.argv)[1:]
    if not argv or argv[0] not in ('run', 'report', '-h', '--help'):
        argv = ['run'] + argv                 # không ghi 'run' cũng được
    args = ap.parse_args(argv)
    args.out_dir_abs = os.path.abspath(args.out_dir)
    args.func(args)


if __name__ == '__main__':
    main()