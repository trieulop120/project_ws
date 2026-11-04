#!/usr/bin/env python3
"""
pallet_gui.py
=============

GUI Tkinter cho pallet_mission_node (cung kieu voi mission_gui.py).

Nut bam (trai -> phai theo thu tu lam viec):
    1. HOME -> P_1 / HOME -> P_2   mission_manager/trigger_home_to_p1|p2 (Empty)
                                    (edge -> BT ComputeRoute, khong co edge -> set goal: do BT cua mission_manager)
    2. Detect ON / OFF              pallet_mission/detect (SetBool)
                                    ON  = bat detector + ha lift ve Home
                                    OFF = tat detector + chot x, y, yaw + PHAN LOAI HANG (A/B)
    3. Go to pallet                 pallet_mission/go_to_pallet (Trigger)
                                    (vao pallet -> nang -> lui. after_pick=hold: dung, giu pallet = CARRYING)
    4. P_1 -> D_1 / P_2 -> D_1      mission_manager/trigger_p1_to_d1|p2_to_d1 (Empty)
    5. DROP                         pallet_mission/drop (Trigger)
                                    (can chinh o D_1 -> nang lift -> vao ke -> sieu am dung -> ha lift -> lui -> ha ve 0)
       Chon loai hang (Tu dong / A / B) = param pallet_mission cargo_type_override
       Chon o ke (Tu chon / A1 / A2 / B1 / B2) = param pallet_mission drop_slot
    6. Return HOME / Ha lift (bo pallet tai cho) / STOP

Hien thi (doc /pallet_mission/status):
    trang thai, thong bao, x/y/yaw pallet, diem dem, sieu am, lift, loai hang, o ke.

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
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool, Trigger

SVC_HOME_P1 = 'mission_manager/trigger_home_to_p1'
SVC_HOME_P2 = 'mission_manager/trigger_home_to_p2'
SVC_P1_D1 = 'mission_manager/trigger_p1_to_d1'
SVC_P2_D1 = 'mission_manager/trigger_p2_to_d1'
SVC_RETURN_HOME = 'mission_manager/trigger_return_home'
PARAM_SVC = '/pallet_mission/set_parameters'
STATUS_TOPIC = '/pallet_mission/status'
STATUS_TIMEOUT_S = 2.0
SEL_HOLD_S = 1.5          # sau khi bam chon: bo qua gia tri status trong ngan nay (tranh nhay nguoc)

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
    'CARRYING': '#6a1b9a',
    'DONE': '#2e7d32',
    'FAILED': '#c62828',
    'STOPPED': '#c62828',
    'OFFLINE': '#c62828',
}

SLOT_TXT = {'full': 'CÓ HÀNG', 'empty': 'trống', 'unknown': '?'}


class PalletGUINode(Node):
    """ROS2 node: service clients + subscribe status."""

    def __init__(self):
        super().__init__('pallet_gui')
        self._lock = threading.Lock()
        self._status = None
        self._status_t = 0.0
        self.events = queue.Queue()       # (label, ok, message) tu ROS thread -> GUI

        self.cli_home_p1 = self.create_client(Empty, SVC_HOME_P1)
        self.cli_home_p2 = self.create_client(Empty, SVC_HOME_P2)
        self.cli_p1_d1 = self.create_client(Empty, SVC_P1_D1)
        self.cli_p2_d1 = self.create_client(Empty, SVC_P2_D1)
        self.cli_return_home = self.create_client(Empty, SVC_RETURN_HOME)
        self.cli_detect = self.create_client(SetBool, 'pallet_mission/detect')
        self.cli_go = self.create_client(Trigger, 'pallet_mission/go_to_pallet')
        self.cli_drop = self.create_client(Trigger, 'pallet_mission/drop')
        self.cli_lift_home = self.create_client(Trigger, 'pallet_mission/lift_home')
        self.cli_stop = self.create_client(Trigger, 'pallet_mission/stop')
        self.cli_param = self.create_client(SetParameters, PARAM_SVC)

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

    def set_str_param(self, name: str, value: str, label: str):
        """Dat param string cua pallet_mission (khong chan)."""
        if not self.cli_param.service_is_ready():
            self.events.put((label, False, 'pallet_mission chua chay (set_parameters)'))
            return
        req = SetParameters.Request()
        # rcl_interfaces/srv/SetParameters uses the `parameters` field.
        req.parameters = [Parameter(name, Parameter.Type.STRING, value).to_parameter_msg()]
        try:
            fut = self.cli_param.call_async(req)
        except Exception as e:
            self.events.put((label, False, f'loi goi service: {e}'))
            return

        def _cb(f):
            try:
                res = f.result()
                r = res.results[0]
                self.events.put((label, bool(r.successful),
                                 f'{name} = "{value}"' if r.successful else (r.reason or 'bi tu choi')))
            except Exception as e:
                self.events.put((label, False, str(e)))
        fut.add_done_callback(_cb)


class PalletGUI(tk.Tk):
    """Giao dien dieu khien pallet mission."""

    def __init__(self, node: PalletGUINode):
        super().__init__()
        self._node = node
        self._sel_t = 0.0
        self.title('AMR Pallet Mission')
        self.geometry('980x720')

        root = ttk.Frame(self, padding='12')
        root.pack(fill=tk.BOTH, expand=True)
        left = ttk.Frame(root)
        left.pack(side='left', fill='y', padx=(0, 14))
        ttk.Separator(root, orient='vertical').pack(side='left', fill='y')
        right = ttk.Frame(root)
        right.pack(side='left', fill=tk.BOTH, expand=True, padx=(14, 0))

        ttk.Label(left, text='AMR Pallet Mission',
                  font=('Arial', 14, 'bold')).pack(pady=(0, 2))

        # ---- 1. HOME -> P ----
        self._section(left, '1. ĐI TỚI ĐIỂM LẤY')
        r = ttk.Frame(left)
        r.pack()
        self.btn_home_p1 = ttk.Button(r, text='HOME → P_1', width=17, command=self._on_home_p1)
        self.btn_home_p1.pack(side='left', padx=3)
        self.btn_home_p2 = ttk.Button(r, text='HOME → P_2', width=17, command=self._on_home_p2)
        self.btn_home_p2.pack(side='left', padx=3)

        # ---- 2. Detect ----
        self._section(left, '2. DETECT PALLET + PHÂN LOẠI HÀNG')
        r = ttk.Frame(left)
        r.pack()
        self.btn_det_on = ttk.Button(r, text='Detect ON', width=17, command=self._on_detect_on)
        self.btn_det_on.pack(side='left', padx=3)
        self.btn_det_off = ttk.Button(r, text='Detect OFF (chốt)', width=17,
                                      command=self._on_detect_off)
        self.btn_det_off.pack(side='left', padx=3)

        # ---- 3. Go to pallet ----
        self._section(left, '3. VÀO PALLET → NÂNG → LÙI (giữ pallet)')
        self.btn_go = ttk.Button(left, text='Go to pallet', width=36, command=self._on_go)
        self.btn_go.pack(pady=2)

        # ---- 4. -> D_1 ----
        self._section(left, '4. ĐI TỚI KỆ (D_1)')
        r = ttk.Frame(left)
        r.pack()
        self.btn_p1_d1 = ttk.Button(r, text='P_1 → D_1', width=17, command=self._on_p1_d1)
        self.btn_p1_d1.pack(side='left', padx=3)
        self.btn_p2_d1 = ttk.Button(r, text='P_2 → D_1', width=17, command=self._on_p2_d1)
        self.btn_p2_d1.pack(side='left', padx=3)

        # ---- 5. Drop ----
        self._section(left, '5. DROP VÀO KỆ')
        self._cargo_var = tk.StringVar(value='')
        r = ttk.Frame(left)
        r.pack(anchor='w')
        ttk.Label(r, text='Loại hàng:', font=('Arial', 9)).pack(side='left', padx=(0, 6))
        for txt, val in (('Tự động', ''), ('A (vừa)', 'A'), ('B (cao)', 'B')):
            ttk.Radiobutton(r, text=txt, value=val, variable=self._cargo_var,
                            command=self._on_cargo_sel).pack(side='left', padx=2)
        self._slot_var = tk.StringVar(value='')
        r = ttk.Frame(left)
        r.pack(anchor='w', pady=(3, 0))
        ttk.Label(r, text='Ô kệ:      ', font=('Arial', 9)).pack(side='left', padx=(0, 6))
        for txt, val in (('Tự chọn', ''), ('A1', 'A1'), ('A2', 'A2'), ('B1', 'B1'), ('B2', 'B2')):
            ttk.Radiobutton(r, text=txt, value=val, variable=self._slot_var,
                            command=self._on_slot_sel).pack(side='left', padx=2)
        self.btn_drop = ttk.Button(left, text='DROP', width=36, command=self._on_drop)
        self.btn_drop.pack(pady=(6, 2))

        # ---- 6. Khac ----
        self._section(left, '6. KHÁC')
        self.btn_return = ttk.Button(left, text='Return HOME', width=36, command=self._on_return_home)
        self.btn_return.pack(pady=2)
        self.btn_lift_home = ttk.Button(left, text='Hạ lift (bỏ pallet tại chỗ)', width=36,
                                        command=self._on_lift_home)
        self.btn_lift_home.pack(pady=2)
        tk.Button(left, text='STOP', width=32, height=2, bg='#c62828', fg='white',
                  activebackground='#8e0000', activeforeground='white',
                  font=('Arial', 11, 'bold'),
                  command=self._on_stop).pack(pady=(10, 2))

        # ================= Phai: trang thai =================
        self._state_var = tk.StringVar(value='OFFLINE')
        self._state_lbl = tk.Label(right, textvariable=self._state_var,
                                   font=('Arial', 18, 'bold'), fg=STATE_COLORS['OFFLINE'])
        self._state_lbl.pack()

        self._msg_var = tk.StringVar(value='')
        ttk.Label(right, textvariable=self._msg_var, wraplength=480,
                  justify='center', font=('Arial', 10)).pack(pady=(3, 6))

        self._err_var = tk.StringVar(value='')
        ttk.Label(right, textvariable=self._err_var, font=('Arial', 10, 'bold'),
                  foreground='#ef6c00').pack()

        self._live_var = tk.StringVar(value='')
        ttk.Label(right, textvariable=self._live_var, font=('Arial', 10),
                  foreground='#1565c0').pack()

        ttk.Separator(right, orient='horizontal').pack(fill='x', pady=8)

        self._pallet_var = tk.StringVar(value='Pallet: chưa có kết quả')
        ttk.Label(right, textvariable=self._pallet_var, wraplength=480,
                  justify='center', font=('Arial', 10, 'bold')).pack()
        self._pre_var = tk.StringVar(value='')
        ttk.Label(right, textvariable=self._pre_var, wraplength=480,
                  justify='center', font=('Arial', 9)).pack()

        self._cargo_info_var = tk.StringVar(value='Hàng: chưa detect')
        ttk.Label(right, textvariable=self._cargo_info_var, wraplength=480,
                  justify='center', font=('Arial', 11, 'bold'),
                  foreground='#6a1b9a').pack(pady=(8, 0))
        self._drop_info_var = tk.StringVar(value='')
        ttk.Label(right, textvariable=self._drop_info_var, wraplength=480,
                  justify='center', font=('Arial', 10)).pack()
        self._shelf_var = tk.StringVar(value='Kệ: chưa có dữ liệu')
        ttk.Label(right, textvariable=self._shelf_var, wraplength=480,
                  justify='center', font=('Arial', 10)).pack(pady=(4, 0))

        ttk.Separator(right, orient='horizontal').pack(fill='x', pady=8)

        self._us_var = tk.StringVar(value='Siêu âm: chưa có dữ liệu')
        ttk.Label(right, textvariable=self._us_var, font=('Arial', 10)).pack()
        self._lift_var = tk.StringVar(value='Lift: chưa có dữ liệu')
        ttk.Label(right, textvariable=self._lift_var, font=('Arial', 10)).pack(pady=(2, 0))

        ttk.Separator(right, orient='horizontal').pack(fill='x', pady=8)

        self._last_var = tk.StringVar(value='Lệnh gần nhất: -')
        ttk.Label(right, textvariable=self._last_var, wraplength=480,
                  justify='center', font=('Arial', 9), foreground='gray').pack()

        self.after(100, self._refresh)

    # ------------------------------------------------------------------
    @staticmethod
    def _section(parent, text):
        ttk.Label(parent, text=text, font=('Arial', 10, 'bold')).pack(pady=(9, 3))

    # ---------------- Button handlers ----------------
    def _on_home_p1(self):
        self._node.call(self._node.cli_home_p1, Empty.Request(), 'HOME → P_1')

    def _on_home_p2(self):
        self._node.call(self._node.cli_home_p2, Empty.Request(), 'HOME → P_2')

    def _on_p1_d1(self):
        self._node.call(self._node.cli_p1_d1, Empty.Request(), 'P_1 → D_1')

    def _on_p2_d1(self):
        self._node.call(self._node.cli_p2_d1, Empty.Request(), 'P_2 → D_1')

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

    def _on_drop(self):
        self._node.call(self._node.cli_drop, Trigger.Request(), 'DROP')

    def _on_lift_home(self):
        self._node.call(self._node.cli_lift_home, Trigger.Request(), 'Hạ lift')

    def _on_stop(self):
        self._node.call(self._node.cli_stop, Trigger.Request(), 'STOP')

    def _on_cargo_sel(self):
        self._sel_t = time.monotonic()
        self._node.set_str_param('cargo_type_override', self._cargo_var.get(), 'Loại hàng')

    def _on_slot_sel(self):
        self._sel_t = time.monotonic()
        self._node.set_str_param('drop_slot', self._slot_var.get(), 'Ô kệ')

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
            self._err_var.set('')
            for b in (self.btn_det_on, self.btn_det_off, self.btn_go, self.btn_p1_d1,
                      self.btn_p2_d1, self.btn_drop, self.btn_lift_home):
                self._enable(b, False)
            for b in (self.btn_home_p1, self.btn_home_p2, self.btn_return):
                self._enable(b, True)
            self.after(100, self._refresh)
            return

        state = st.get('state', '?')
        self._set_state_label(state)
        self._msg_var.set(st.get('msg', ''))

        # lech khi dang canh / vao
        err = st.get('err')
        if err and state in ('ALIGNING', 'INSERTING'):
            self._err_var.set(f"Phase {st.get('phase') or '-'}:  lệch ngang {err['lat_cm']:+.1f} cm   "
                              f"lệch góc {err['yaw_deg']:+.1f}°   cách mặt {err['to_face_cm']:.0f} cm")
        else:
            self._err_var.set('')

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

        # loai hang
        cc, ct, ch = st.get('cargo_class'), st.get('cargo_type'), st.get('cargo_h_cm')
        if cc is None:
            self._cargo_info_var.set('Hàng: chưa detect')
        elif ct:
            h = '' if ch is None else f' {ch:.0f} cm'
            self._cargo_info_var.set(f'Hàng: {cc}{h} → loại {ct}')
        else:
            self._cargo_info_var.set(f"Hàng: KHÔNG RÕ ({st.get('cargo_why') or '?'}) → chọn A/B")

        dr = st.get('drop')
        if dr:
            self._drop_info_var.set(f"Drop: ô {dr['slot']} (tầng {dr['level']}, loại {dr['type']}) "
                                    f"nâng {dr['insert_cm']:.0f} cm khi vào, đặt ở {dr['release_cm']:.0f} cm")
        else:
            self._drop_info_var.set('')

        sh = st.get('shelf')
        if sh and sh.get('states'):
            s = sh['states']
            txt = '   '.join(f"{k}={SLOT_TXT.get(s.get(k), '?')}" for k in ('A1', 'A2', 'B1', 'B2'))
            raw = sh.get('raw')
            bits = '----' if not raw else ''.join('?' if v < 0 else ('1' if v else '0') for v in raw)
            self._shelf_var.set(f"Kệ: {txt}\n(thô {bits}{'' if sh.get('fresh') else ' - DỮ LIỆU CŨ'})")
        else:
            self._shelf_var.set('Kệ: chưa có dữ liệu')

        # dong bo lua chon voi param that cua node (tru luc vua bam)
        sel = st.get('sel') or {}
        if time.monotonic() - self._sel_t > SEL_HOLD_S:
            self._cargo_var.set(sel.get('cargo', ''))
            self._slot_var.set(sel.get('slot', ''))

        # sieu am
        us = st.get('us_cm')
        cnt = st.get('us_count', 0)
        if us is None:
            self._us_var.set('Siêu âm: không có echo / chưa có dữ liệu')
        else:
            self._us_var.set(f'Siêu âm: {us:.1f} cm   (mẫu liên tiếp dưới ngưỡng: {cnt})')

        lift = st.get('lift_cm')
        if lift is None:
            self._lift_var.set('Lift: không có dữ liệu /joint_states')
        else:
            self._lift_var.set(f'Lift: {lift:.1f} cm')

        # bat/tat nut theo trang thai
        busy = state in BUSY_STATES
        holding = st.get('cargo') is True
        can_move = not busy and state != 'DETECTING'
        self._enable(self.btn_det_on, state in IDLE_LIKE and not holding)
        self._enable(self.btn_det_off, state == 'DETECTING')
        self._enable(self.btn_go, state == 'READY')
        self._enable(self.btn_home_p1, can_move and not holding)
        self._enable(self.btn_home_p2, can_move and not holding)
        self._enable(self.btn_p1_d1, can_move and holding)
        self._enable(self.btn_p2_d1, can_move and holding)
        self._enable(self.btn_drop, holding and state in ('CARRYING', 'FAILED', 'STOPPED'))
        self._enable(self.btn_return, can_move)
        self._enable(self.btn_lift_home, can_move)

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
