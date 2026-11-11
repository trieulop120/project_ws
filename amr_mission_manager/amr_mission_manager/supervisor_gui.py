#!/usr/bin/env python3
"""
supervisor_gui.py
=================
GUI Tkinter cho Mission Supervisor: nut START / STOP + theo doi trang thai.

Chuc nang:
    - Chọn chế độ single/auto rồi START chu trình
    - RESUME từ checkpoint RECOVERY sau khi xác nhận điều kiện an toàn
    - STOP luôn khả dụng để hủy Nav2 và dừng pallet_mission
    - Theo doi /mission_supervisor/status (JSON 0.5 Hz):
        * state supervisor (IDLE / GOTO_PICK / PICKING / DONE / FAILED / ...)
        * state + phase + lift cua pallet_mission
        * trang thai 4 o ke (A1/A2/B1/B2) tu /shelf/occupancy theo
          anh xa trong shelf.yaml: co hang / trong / chua ro (unknown)
        * dong gia tri tho /shelf/occupancy (de che cam bien xac nhan)

Soho 4 o (nhin tu phia robot dung o D_1):
    C1 (A2 tren, A1 duoi) | C2 B1 | C3 B2

Chay (mission_supervisor phai dang chay - mission_supervisor.launch.py):
    ros2 run amr_mission_manager supervisor_gui
"""

import json
import sys
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger

START_SVC = '/mission/start'
STOP_SVC = '/mission/stop'
RESUME_SVC = '/mission/resume'
PARAM_SVC = '/mission_supervisor/set_parameters'
STATUS_TOPIC = '/mission_supervisor/status'

# 4 o: (id, cot, row) - row 2 = tren
SLOTS = [('A1', 0, 0), ('A2', 0, 1), ('B1', 1, 0), ('B2', 2, 0)]

# Mau: full / empty / unknown
C_FULL = '#2e7d32'
C_EMPTY = '#e8e8e8'
C_UNKNOWN = '#f0a500'
C_SEL = '#1565c0'
T_FULL = '#ffffff'
T_DARK = '#222222'


class SupervisorGUINode(Node):
    """ROS2 node: service client start/stop + set_parameters + status."""

    def __init__(self):
        super().__init__('supervisor_gui')
        self._cli_start = self.create_client(Trigger, START_SVC)
        self._cli_stop = self.create_client(Trigger, STOP_SVC)
        self._cli_resume = self.create_client(Trigger, RESUME_SVC)
        self._cli_param = self.create_client(SetParameters, PARAM_SVC)
        self.create_subscription(String, STATUS_TOPIC, self._on_status, 10)
        # Ban tin status JSON cuoi (node luu, GUI doc qua self.last_status)
        self.last_status = None

    def call_start(self) -> bool:
        return self._trigger(self._cli_start, START_SVC)

    def call_stop(self) -> bool:
        return self._trigger(self._cli_stop, STOP_SVC)

    def call_resume(self) -> bool:
        return self._trigger(self._cli_resume, RESUME_SVC)

    def call_set_mode(self, mode: str) -> bool:
        if not self._cli_param.wait_for_service(timeout_sec=2.0):
            return False
        p = Parameter('run_mode', Parameter.Type.STRING, mode)
        req = SetParameters.Request()
        req.parameters = [p.to_parameter_msg()]
        res = self._call(self._cli_param, req)
        return bool(res and res.results and res.results[0].successful)

    def _call(self, cli, req, timeout: float = 3.0):
        """Goi service tu thread GUI va CHO ket qua (ROS spin chay thread khac).
        rclpy Future khong co get_result(); dung Event + add_done_callback."""
        fut = cli.call_async(req)
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            self.get_logger().warn('Service tra loi qua han')
            return None
        return fut.result()

    def _trigger(self, cli, name) -> bool:
        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'Service {name} chua san sang '
                                   f'(mission_supervisor dang chay?)')
            return False
        res = self._call(cli, Trigger.Request())
        return res is not None and bool(res.success)

    def _on_status(self, msg: String):
        try:
            self.last_status = json.loads(msg.data)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Vẽ ô kệ
# ---------------------------------------------------------------------------
def _raw_bits_str(raw) -> str:
    """shelf_raw (list 4 bit hoac None) -> chuoi 4 ky tu '1000' (ESP32-style).

    - '-' = chua nhan du lieu; '?' = bit < 0 (mat lien lac);
    '1' = co hang, '0' = trong.
    """
    if not raw:
        return '----'
    return ''.join('?' if v is None or v < 0 else ('1' if v else '0')
                   for v in raw)


class SupervisorGUI(tk.Tk):
    """Cua so START/STOP + chon loai hang + panel 4 o ke + dong gia tri tho."""

    # So do o (khung canvas): cot rong = 3 cot
    COL_W, GAP, H_TOP = 76, 8, 10
    CELL_H = 52          # o cot 1 (A1/A2 chong nhau)
    FULL_H = 2 * CELL_H + GAP   # cot 2/3 (B1/B2 can chieu toan)

    def __init__(self, node: SupervisorGUINode):
        super().__init__()
        self._node = node
        self.title('AMR Mission Supervisor')
        self._window_width = 560
        self.geometry(f'{self._window_width}x620')

        main = ttk.Frame(self, padding='12')
        main.pack(fill='both', expand=True)

        ttk.Label(main, text='Mission Supervisor',
                  font=('Arial', 14, 'bold')).pack(pady=(0, 8))

        # ---- Nut START / STOP ----
        btns = ttk.Frame(main)
        btns.pack(fill='x')
        ttk.Button(btns, text='START', width=14,
                   command=self._on_start).pack(side='left', padx=4)
        ttk.Button(btns, text='RESUME', width=10,
                   command=self._on_resume).pack(side='left', padx=4)
        ttk.Button(btns, text='STOP', width=14,
                   command=self._on_stop).pack(side='left', padx=4)

        mode_row = ttk.Frame(main)
        mode_row.pack(fill='x', pady=(8, 0))
        ttk.Label(mode_row, text='Chế độ chạy:').pack(side='left', padx=4)
        self._mode_var = tk.StringVar(value='single')
        ttk.Combobox(mode_row, textvariable=self._mode_var, state='readonly',
                     values=('single', 'auto'), width=12).pack(side='left', padx=4)
        ttk.Label(mode_row, text='single: 1 pallet rồi về HOME · auto: chạy tới khi hết pallet/ô phù hợp',
                  font=('Arial', 8), foreground='gray').pack(side='left', padx=4)

        # ---- Loai pallet detector da phan loai ----
        ttk.Separator(main, orient='horizontal').pack(fill='x', pady=10)
        cargo = ttk.Frame(main)
        cargo.pack(fill='x')
        ttk.Label(cargo, text='Pallet detector:',
                  font=('Arial', 10)).pack(side='left')
        self._cargo_var = tk.StringVar(value='chưa detect')
        self._cargo_label = ttk.Label(cargo, textvariable=self._cargo_var,
                                       font=('Arial', 9), foreground='gray')
        self._cargo_label.pack(side='left', padx=8)

        # ---- Panel 4 o ke (soto) ----
        ttk.Separator(main, orient='horizontal').pack(fill='x', pady=10)
        ttk.Label(main, text='Ke (trung gian = cot 2; A: cot 1, '
                             'A2 tren / A1 duoi)',
                  font=('Arial', 10, 'bold')).pack(anchor='w')
        canvas_w = 3 * self.COL_W + 2 * self.GAP + 8
        self._cv = tk.Canvas(main, width=canvas_w,
                             height=self.FULL_H + 2 * self.H_TOP + 4,
                             bg='white', highlightthickness=0)
        self._cv.pack(anchor='w', pady=(4, 2))

        # Dong gia tri tho /shelf/occupancy (de che cam bien xac nhan)
        self._raw_var = tk.StringVar(value='gia tri tho /shelf/occupancy: '
                                           '----  (bit0->A1, bit1->A2, '
                                           'bit2->B1, bit3->B2; 1=co hang)')
        ttk.Label(main, textvariable=self._raw_var,
                  font=('monospace', 9),
                  foreground='#444444').pack(anchor='w')

        # Truyen do
        self._slot_sel_var = tk.StringVar(value='o da chon: -')
        self._pm_var = tk.StringVar(value='pm_state: -  phase: -')
        self._state_var = tk.StringVar(value='IDLE')
        self._msg_var = tk.StringVar(value='Doi du lieu status...')
        self._run_var = tk.StringVar(value='Mode: -   Chu ky: 0')
        self._detector_var = tk.StringVar(value='Detector: -')
        self._pick_queue_var = tk.StringVar(value='Điểm lấy: -')
        for var in (self._slot_sel_var, self._pm_var):
            ttk.Label(main, textvariable=var, font=('Arial', 10)).pack(anchor='w')
        self._state_label = ttk.Label(main, textvariable=self._state_var,
                                      font=('Arial', 10, 'bold'))
        self._state_label.pack(anchor='w')
        for var in (self._msg_var, self._run_var, self._detector_var,
                    self._pick_queue_var):
            ttk.Label(main, textvariable=var, font=('Arial', 10)).pack(anchor='w')

        ttk.Label(main,
                  text='mau o:  xanh=CÓ HÀNG  ·  xám=TRỐNG  ·  '
                       'vàng=CHƯA RÕ (mất liên lạc / -1 / stale)  ·  '
                       'viền xanh dương=ô đã chọn',
                  font=('Arial', 8), foreground='gray').pack(anchor='w',
                                                              pady=(6, 0))

        # Sub status (ROS chay thread rieng -> update UI qua after)
        self._poll_status()

    # ------------------------------------------------------------------
    # Vẽ o
    # ------------------------------------------------------------------
    def _cell_rect(self, sid: str):
        """(x0, y0, x1, y1) trong khung canvas cho 1 o.

        Cot 0 (A): A2 = ô TRÊN (nửa trên), A1 = ô DƯỚI (nửa dưới, cách
        GAP). Cot 1/2 (B): ô cao = cả cột (B1 cột giữa, B2 cột phải).
        """
        for sid2, col, _row in SLOTS:
            if sid2 != sid:
                continue
            x0 = self.H_TOP + col * (self.COL_W + self.GAP)
            x1 = x0 + self.COL_W
            if sid2 == 'A2':                 # ô trên, nửa trên cột 0
                y0, y1 = self.H_TOP, self.H_TOP + self.CELL_H
            elif sid2 == 'A1':               # ô dưới, nửa dưới cột 0
                y0 = self.H_TOP + self.CELL_H + self.GAP
                y1 = y0 + self.CELL_H
            else:                            # B1 / B2: cả cột
                y0, y1 = self.H_TOP, self.H_TOP + self.FULL_H
            return x0, y0, x1, y1
        return None

    def _draw_slots(self, slots: dict, selected, cargo_type: str):
        cv = self._cv
        cv.delete('all')
        for sid, col, row in SLOTS:
            x0, y0, x1, y1 = self._cell_rect(sid)
            st = (slots or {}).get(sid, 'unknown')
            if st == 'full':
                fill, txt = C_FULL, T_FULL
            elif st == 'empty':
                fill, txt = C_EMPTY, T_DARK
            else:
                fill, txt = C_UNKNOWN, T_DARK
            sel = (selected == sid)
            cv.create_rectangle(x0, y0, x1, y1, fill=fill,
                                outline=C_SEL if sel else '#666666',
                                width=3 if sel else 1)
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            state_txt = {'full': 'CÓ HÀNG', 'empty': 'TRỐNG'}.get(st, 'CHƯA RÕ')
            cv.create_text(cx, cy - 10, text=sid, fill=txt,
                           font=('Arial', 13, 'bold'))
            cv.create_text(cx, cy + 8, text=state_txt, fill=txt,
                           font=('Arial', 8))
        cv.update_idletasks()

    # ------------------------------------------------------------------
    def _poll_status(self):
        d = self._node.last_status
        if d:
            state = str(d.get('state', '-'))
            self._state_var.set(state)
            self._state_label.configure(
                foreground='#b77900' if state == 'WAITING_SLOT'
                else '#c62828' if state == 'RECOVERY' else '#222222')
            self._msg_var.set(str(d.get('msg', '')))
            cyc = d.get('cycle') or {}
            self._run_var.set(f"Mode: {cyc.get('run_mode', '-')}   Chu ky: {cyc.get('number', 0)}")
            queue = d.get('pick_queue') or []
            queue_text = ' → '.join(str(x) for x in queue) or 'đã hết điểm lấy'
            waiting = ' · đang chờ ô kệ trống' if d.get('waiting_slot') else ''
            self._pick_queue_var.set(f'Điểm lấy còn lại: {queue_text}{waiting}')
            pm_state = d.get('pm_state') or '-'
            pm_phase = d.get('pm_phase') or '-'
            n, nmin = d.get('pm_samples', 0), d.get('pm_min_samples', 0)
            self._pm_var.set(f'Pallet: {pm_state}   phase: {pm_phase}   detect samples: {n}/{nmin}')
            det = d.get('detector') or {}
            self._detector_var.set(
                f"Detector: {det.get('state', '-')} | seen {det.get('seen_pallets', 0)} "
                f"loaded {det.get('loaded_pallets', 0)} compatible {det.get('compatible_pallets', 0)} "
                f"types {','.join(det.get('target_cargo_classes') or [])} "
                f"{'fresh' if det.get('fresh') else 'stale'}")

            slots = d.get('slots')
            selected = d.get('drop_slot')
            cargo = d.get('cargo_type') or ''
            self._draw_slots(slots or {}, selected, cargo)

            self._raw_var.set('giá trị thô /shelf/occupancy: '
                              + _raw_bits_str(d.get('shelf_raw'))
                              + '   (bit0→A1, bit1→A2, bit2→B1, bit3→B2; '
                                '1=có hàng, ?=mất liên lạc)')
            self._slot_sel_var.set(
                f'ô đã chọn cho đặt: {selected or "-"}'
                + (f'  (khung TF {d.get("drop_slot_frame")})'
                   if d.get('drop_slot_frame') else ''))
            self._cargo_var.set(
                {'A': 'hàng vừa (loại A)',
                 'B': 'hàng cao (loại B)'}.get(cargo, 'chưa phân loại'))
        else:
            self._cargo_var.set('chưa có dữ liệu status')
        self.after(500, self._poll_status)

    # ------------------------------------------------------------------
    def _on_start(self):
        mode = self._mode_var.get()
        def start():
            return self._node.call_set_mode(mode) and self._node.call_start()
        self._run_command(start, 'START ' + mode.upper())

    def _on_resume(self):
        status = self._node.last_status or {}
        if status.get('state') == 'RECOVERY':
            ok = messagebox.askyesno(
                'Xác nhận tiếp tục',
                'Hãy xác nhận robot, pallet và lift đang ở trạng thái an toàn. '
                'Nếu lỗi xảy ra trong lúc đưa pallet vào kệ, chỉ Resume sau khi kiểm tra trực tiếp.')
            if not ok:
                return
        self._run_command(self._node.call_resume, 'RESUME')

    def _on_stop(self):
        self._run_command(self._node.call_stop, 'STOP')

    def _run_command(self, fn, label):
        def worker():
            ok = fn()
            self.after(0, lambda: self._cmd_done(ok, label))
        threading.Thread(target=worker, daemon=True).start()

    def _cmd_done(self, ok, label):
        self._msg_var.set(f'{label} - {"ĐÃ GỬI" if ok else "SERVICE CHƯA SẴN"}')


def main():
    rclpy.init(args=sys.argv)
    node = SupervisorGUINode()
    app = SupervisorGUI(node)

    def spin_ros():
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        except ExternalShutdownException:
            # The Tk window may close while spin_once is waiting for work.
            pass

    threading.Thread(target=spin_ros, daemon=True).start()
    app.mainloop()
    if rclpy.ok():
        rclpy.shutdown()
    node.destroy_node()


if __name__ == '__main__':
    main()
