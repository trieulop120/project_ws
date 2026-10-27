#!/usr/bin/env python3
"""
pallet_gui.py
=============

GUI Tkinter cho pallet_mission_node (cung kieu voi mission_gui.py).

Nut bam:
    1. HOME -> P_1        goi mission_manager/trigger_home_to_p1 (Empty)
    2. Detect ON / OFF    goi pallet_mission/detect (SetBool)
                          ON  = bat detector + ha lift ve Home
                          OFF = tat detector + chot x, y, yaw
    3. Go to pallet       goi pallet_mission/go_to_pallet (Trigger)
                          (vao pallet -> nang 5 cm -> lui -> ve Home -> ha lift -> DONE)
    4. Return HOME        goi mission_manager/trigger_return_home (Empty)
    5. STOP               goi pallet_mission/stop (Trigger)

Hien thi (doc /pallet_mission/status):
    trang thai, thong bao, x/y/yaw pallet, diem dem, sieu am, chieu cao lift.

Usage:
    ros2 run amr_mission_manager pallet_gui
"""

import json
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool, Trigger

HOME_P1_SERVICE = 'mission_manager/trigger_home_to_p1'
RETURN_HOME_SERVICE = 'mission_manager/trigger_return_home'
STATUS_TOPIC = '/pallet_mission/status'
STATUS_TIMEOUT_S = 2.0

# dang chay chu trinh: khoa cac nut di chuyen (chi con STOP)
BUSY_STATES = ('NAVIGATING', 'SETTLING', 'ALIGNING', 'INSERTING',
               'LIFTING', 'BACKING', 'HOMING', 'LOWERING')
IDLE_LIKE = ('IDLE', 'READY', 'DONE', 'FAILED', 'STOPPED')

STATE_COLORS = {
    'IDLE': '#555555',
    'DETECTING': '#1565c0',
    'READY': '#2e7d32',
    'NAVIGATING': '#ef6c00',
    'SETTLING': '#ef6c00',
    'ALIGNING': '#ef6c00',
    'INSERTING': '#ef6c00',
    'LIFTING': '#6a1b9a',
    'BACKING': '#ef6c00',
    'HOMING': '#ef6c00',
    'LOWERING': '#6a1b9a',
    'DONE': '#2e7d32',
    'FAILED': '#c62828',
    'STOPPED': '#c62828',
    'OFFLINE': '#c62828',
}


class PalletGUINode(Node):
    """ROS2 node: service clients + subscribe status."""

    def __init__(self):
        super().__init__('pallet_gui')
        self._lock = threading.Lock()
        self._status = None
        self._status_t = 0.0
        self.events = queue.Queue()       # (label, ok, message) tu ROS thread -> GUI

        self.cli_home = self.create_client(Empty, HOME_P1_SERVICE)
        self.cli_return_home = self.create_client(Empty, RETURN_HOME_SERVICE)
        self.cli_detect = self.create_client(SetBool, 'pallet_mission/detect')
        self.cli_go = self.create_client(Trigger, 'pallet_mission/go_to_pallet')
        self.cli_stop = self.create_client(Trigger, 'pallet_mission/stop')

        self.create_subscription(String, STATUS_TOPIC, self._on_status, 10)

    def _on_status(self, msg: String):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            self._status = d
            self._status_t = time.monotonic()

    def get_status(self):
        """Return (dict | None, tuoi cua status tinh bang giay)."""
        with self._lock:
            if self._status is None:
                return None, float('inf')
            return dict(self._status), time.monotonic() - self._status_t

    def call(self, client, request, label: str):
        """Goi service khong chan; ket qua day vao self.events."""
        if not client.service_is_ready():
            self.events.put((label, False, 'service chua san sang'))
            return
        try:
            fut = client.call_async(request)
        except Exception as e:
            self.events.put((label, False, f'loi goi service: {e}'))
            return
        fut.add_done_callback(lambda f: self._done(label, f))

    def _done(self, label, fut):
        try:
            res = fut.result()
        except Exception as e:
            self.events.put((label, False, str(e)))
            return
        if hasattr(res, 'success'):
            self.events.put((label, bool(res.success), res.message))
        else:
            self.events.put((label, True, 'da gui'))


class PalletGUI(tk.Tk):
    """Giao dien dieu khien pallet mission."""

    def __init__(self, node: PalletGUINode):
        super().__init__()
        self._node = node
        self.title('AMR Pallet Mission')
        self.geometry('460x790')

        main = ttk.Frame(self, padding='15')
        main.pack(fill=tk.BOTH, expand=True)

        ttk.Label(main, text='AMR Pallet Mission',
                  font=('Arial', 14, 'bold')).pack(pady=(0, 5))

        # ---- 1. HOME -> P_1 ----
        self._section(main, '1. HOME → P_1')
        self.btn_home = ttk.Button(main, text='HOME → P_1', width=34,
                                   command=self._on_home)
        self.btn_home.pack(pady=3)

        # ---- 2. Detect ----
        self._section(main, '2. DETECT PALLET (Detect ON: hạ lift về Home)')
        row = ttk.Frame(main)
        row.pack()
        self.btn_det_on = ttk.Button(row, text='Detect ON', width=16,
                                     command=self._on_detect_on)
        self.btn_det_on.pack(side='left', padx=3)
        self.btn_det_off = ttk.Button(row, text='Detect OFF (chốt)', width=16,
                                      command=self._on_detect_off)
        self.btn_det_off.pack(side='left', padx=3)

        # ---- 3. Go to pallet ----
        self._section(main, '3. ĐI VÀO PALLET → NÂNG → LÙI → VỀ HOME')
        self.btn_go = ttk.Button(main, text='Go to pallet', width=34,
                                 command=self._on_go)
        self.btn_go.pack(pady=3)

        # ---- 4. Return HOME ----
        self._section(main, '4. VỀ HOME')
        self.btn_return = ttk.Button(main, text='Return HOME', width=34,
                                     command=self._on_return_home)
        self.btn_return.pack(pady=3)

        # ---- STOP ----
        tk.Button(main, text='STOP', width=30, height=2, bg='#c62828', fg='white',
                  activebackground='#8e0000', activeforeground='white',
                  font=('Arial', 11, 'bold'),
                  command=self._on_stop).pack(pady=(12, 3))

        ttk.Separator(main, orient='horizontal').pack(fill='x', pady=12)

        # ---- Trang thai ----
        self._state_var = tk.StringVar(value='OFFLINE')
        self._state_lbl = tk.Label(main, textvariable=self._state_var,
                                   font=('Arial', 16, 'bold'), fg=STATE_COLORS['OFFLINE'])
        self._state_lbl.pack()

        self._msg_var = tk.StringVar(value='')
        ttk.Label(main, textvariable=self._msg_var, wraplength=410,
                  justify='center', font=('Arial', 10)).pack(pady=(3, 8))

        self._live_var = tk.StringVar(value='')
        ttk.Label(main, textvariable=self._live_var, font=('Arial', 10),
                  foreground='#1565c0').pack()

        self._pallet_var = tk.StringVar(value='Pallet: chưa có kết quả')
        ttk.Label(main, textvariable=self._pallet_var, wraplength=410,
                  justify='center', font=('Arial', 10, 'bold')).pack(pady=(6, 0))

        self._pre_var = tk.StringVar(value='')
        ttk.Label(main, textvariable=self._pre_var, wraplength=410,
                  justify='center', font=('Arial', 9)).pack()

        self._us_var = tk.StringVar(value='Siêu âm: chưa có dữ liệu')
        ttk.Label(main, textvariable=self._us_var,
                  font=('Arial', 10)).pack(pady=(8, 0))

        self._lift_var = tk.StringVar(value='Lift: chưa có dữ liệu')
        ttk.Label(main, textvariable=self._lift_var,
                  font=('Arial', 10)).pack(pady=(2, 0))

        ttk.Separator(main, orient='horizontal').pack(fill='x', pady=12)

        self._last_var = tk.StringVar(value='Lệnh gần nhất: -')
        ttk.Label(main, textvariable=self._last_var, wraplength=410,
                  justify='center', font=('Arial', 9),
                  foreground='gray').pack()

        self.after(100, self._refresh)

    # ------------------------------------------------------------------
    @staticmethod
    def _section(parent, text):
        ttk.Label(parent, text=text, font=('Arial', 10, 'bold')).pack(pady=(10, 3))

    # ---------------- Button handlers ----------------
    def _on_home(self):
        self._node.call(self._node.cli_home, Empty.Request(), 'HOME → P_1')

    def _on_return_home(self):
        self._node.call(self._node.cli_return_home, Empty.Request(), 'Return HOME')

    def _on_detect_on(self):
        req = SetBool.Request()
        req.data = True
        self._node.call(self._node.cli_detect, req, 'Detect ON')

    def _on_detect_off(self):
        req = SetBool.Request()
        req.data = False
        self._node.call(self._node.cli_detect, req, 'Detect OFF')

    def _on_go(self):
        self._node.call(self._node.cli_go, Trigger.Request(), 'Go to pallet')

    def _on_stop(self):
        self._node.call(self._node.cli_stop, Trigger.Request(), 'STOP')

    # ---------------- Refresh ----------------
    @staticmethod
    def _enable(btn, enabled: bool):
        btn.configure(state='normal' if enabled else 'disabled')

    def _refresh(self):
        # ket qua lenh
        while True:
            try:
                label, ok, msg = self._node.events.get_nowait()
            except queue.Empty:
                break
            self._last_var.set(f"Lệnh gần nhất: {label} → {'OK' if ok else 'LỖI'}: {msg}")

        st, age = self._node.get_status()
        online = st is not None and age <= STATUS_TIMEOUT_S

        if not online:
            self._set_state_label('OFFLINE')
            self._msg_var.set('Chưa nhận được /pallet_mission/status. '
                              'Node pallet_mission đã chạy chưa?')
            self._live_var.set('')
            self._enable(self.btn_det_on, False)
            self._enable(self.btn_det_off, False)
            self._enable(self.btn_go, False)
            self._enable(self.btn_home, True)
            self._enable(self.btn_return, True)
            self.after(100, self._refresh)
            return

        state = st.get('state', '?')
        self._set_state_label(state)
        self._msg_var.set(st.get('msg', ''))

        # live khi dang detect
        live = st.get('live')
        if state == 'DETECTING':
            n = st.get('samples', 0)
            if live:
                self._live_var.set(
                    f"Đang thấy: x={live['x']:.2f}  y={live['y']:.2f}  "
                    f"yaw={live['yaw_deg']:.0f}°   ({n} mẫu)")
            else:
                self._live_var.set(f'Chưa thấy pallet ({n} mẫu)')
        else:
            self._live_var.set('')

        # ket qua pallet da chot
        p = st.get('pallet')
        if p:
            tag = 'ổn định' if p.get('stable') else 'KHÔNG ỔN ĐỊNH'
            self._pallet_var.set(
                f"Pallet: x={p['x']:.3f} m   y={p['y']:.3f} m   "
                f"yaw={p['yaw_deg']:.1f}°  ({tag})")
        else:
            self._pallet_var.set('Pallet: chưa có kết quả')

        pre = st.get('pre')
        if pre:
            self._pre_var.set(f"Điểm đệm: x={pre['x']:.3f}  y={pre['y']:.3f}  "
                              f"yaw={pre['yaw_deg']:.1f}°")
        else:
            self._pre_var.set('')

        # sieu am
        us = st.get('us_cm')
        cnt = st.get('us_count', 0)
        if us is None:
            self._us_var.set('Siêu âm: không có echo / chưa có dữ liệu')
        else:
            self._us_var.set(f'Siêu âm: {us:.1f} cm   '
                             f'(mẫu liên tiếp dưới ngưỡng: {cnt})')

        # lift
        lift = st.get('lift_cm')
        if lift is None:
            self._lift_var.set('Lift: không có dữ liệu /joint_states')
        else:
            self._lift_var.set(f'Lift: {lift:.1f} cm')

        # bat/tat nut theo trang thai
        busy = state in BUSY_STATES
        self._enable(self.btn_det_on, state in IDLE_LIKE)
        self._enable(self.btn_det_off, state == 'DETECTING')
        self._enable(self.btn_go, state == 'READY')
        self._enable(self.btn_home, not busy)
        self._enable(self.btn_return, not busy and state != 'DETECTING')

        self.after(100, self._refresh)

    def _set_state_label(self, state: str):
        self._state_var.set(state)
        self._state_lbl.configure(fg=STATE_COLORS.get(state, '#555555'))


def main():
    rclpy.init(args=sys.argv)
    ros_node = PalletGUINode()

    app = PalletGUI(ros_node)

    def spin_ros():
        while rclpy.ok():
            try:
                rclpy.spin_once(ros_node, timeout_sec=0.1)
            except Exception:
                break

    threading.Thread(target=spin_ros, daemon=True).start()

    try:
        app.mainloop()
    finally:
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()