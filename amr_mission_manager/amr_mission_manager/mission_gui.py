#!/usr/bin/env python3
"""
mission_gui.py
==============

Simple GUI for Mission Manager using Tkinter.

Missions:
    - HOME → P_1, HOME → P_2
    - P_1 → D_1, P_2 → D_1
    - D_1 → P_1, D_1 → P_2
    - Return HOME (NavigateToPose direct)

Usage:
    ros2 run amr_mission_manager mission_gui

Author: AMR System
"""

import sys
import threading
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class MissionGUINode(Node):
    """ROS2 node for mission GUI."""

    def __init__(self):
        super().__init__('mission_gui')
        self._services = {}
        self._create_services()

    def _create_services(self):
        """Create service clients."""
        services = [
            'mission_manager/trigger_home_to_p1',
            'mission_manager/trigger_home_to_p2',
            'mission_manager/trigger_p1_to_d1',
            'mission_manager/trigger_p2_to_d1',
            'mission_manager/trigger_d1_to_p1',
            'mission_manager/trigger_d1_to_p2',
            'mission_manager/trigger_return_home',
        ]
        for svc in services:
            self._services[svc] = self.create_client(Empty, svc)

    def call_service(self, service_name: str) -> bool:
        """Call mission service."""
        if service_name not in self._services:
            self.get_logger().error(f'Unknown service: {service_name}')
            return False
        client = self._services[service_name]
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service not ready: {service_name}')
            return False
        try:
            future = client.call_async(Empty.Request())
            return True
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return False


class MissionGUI(tk.Tk):
    """GUI for Mission Manager."""

    def __init__(self, node: MissionGUINode):
        super().__init__()

        self._node = node
        self.title("AMR Mission Manager")
        # Auto size to fit all buttons
        self.geometry("600x700")

        # Main frame
        main_frame = ttk.Frame(self, padding="15")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Title
        title = ttk.Label(main_frame, text="AMR Mission Manager",
                          font=("Arial", 14, "bold"))
        title.pack(pady=(0, 5))

        # Layout: HOME row
        section_home = ttk.Label(main_frame, text="HOME → PICKUP",
                               font=("Arial", 10, "bold"))
        section_home.pack(pady=(10, 3))

        btn_frame_home = ttk.Frame(main_frame)
        btn_frame_home.pack(fill='x', pady=(0, 5))

        for label, service in [
            ("HOME → P_1", "mission_manager/trigger_home_to_p1"),
            ("HOME → P_2", "mission_manager/trigger_home_to_p2"),
        ]:
            btn = ttk.Button(btn_frame_home, text=label, width=18)
            btn.pack(side='left', padx=3)
            btn.configure(command=lambda s=service, l=label, b=btn:
                        self._on_click(s, l, b))

        # Layout: PICKUP → DROPOFF row
        section_pick_drop = ttk.Label(main_frame, text="PICKUP → DROPOFF",
                                     font=("Arial", 10, "bold"))
        section_pick_drop.pack(pady=(10, 3))

        btn_frame_pick = ttk.Frame(main_frame)
        btn_frame_pick.pack(fill='x', pady=(0, 5))

        for label, service in [
            ("P_1 → D_1", "mission_manager/trigger_p1_to_d1"),
            ("P_2 → D_1", "mission_manager/trigger_p2_to_d1"),
        ]:
            btn = ttk.Button(btn_frame_pick, text=label, width=18)
            btn.pack(side='left', padx=3)
            btn.configure(command=lambda s=service, l=label, b=btn:
                        self._on_click(s, l, b))

        # Layout: DROPOFF → PICKUP row
        section_drop_pick = ttk.Label(main_frame, text="DROPOFF → PICKUP",
                                     font=("Arial", 10, "bold"))
        section_drop_pick.pack(pady=(10, 3))

        btn_frame_drop = ttk.Frame(main_frame)
        btn_frame_drop.pack(fill='x', pady=(0, 5))

        for label, service in [
            ("D_1 → P_1", "mission_manager/trigger_d1_to_p1"),
            ("D_1 → P_2", "mission_manager/trigger_d1_to_p2"),
        ]:
            btn = ttk.Button(btn_frame_drop, text=label, width=18)
            btn.pack(side='left', padx=3)
            btn.configure(command=lambda s=service, l=label, b=btn:
                        self._on_click(s, l, b))

        # Separator
        ttk.Separator(main_frame, orient='horizontal').pack(fill='x', pady=15)

        # Return Home
        btn_home = ttk.Button(main_frame, text="Return HOME",
                              width=25, style='Accent.TButton')
        btn_home.pack(pady=5)
        btn_home.configure(command=lambda: self._on_click(
            'mission_manager/trigger_return_home', 'Return HOME', btn_home))

        # Separator
        ttk.Separator(main_frame, orient='horizontal').pack(fill='x', pady=15)

        # Status
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill='x', pady=5)

        self._status = tk.StringVar(value="Status: Ready")
        status_label = ttk.Label(status_frame, textvariable=self._status,
                                 font=("Arial", 10))
        status_label.pack()

        # Legend
        legend = ttk.Label(main_frame,
                          text="Robot: 1 HOME, 2 PICKUP, 1 DROPOFF",
                          font=("Arial", 8), foreground='gray')
        legend.pack(pady=(10, 0))

    def _on_click(self, service: str, label: str, button: ttk.Button):
        """Handle button click."""
        button.configure(state='disabled')
        self._status.set(f"Executing: {label}...")

        def call_service():
            success = self._node.call_service(service)
            self.after(0, lambda: self._on_complete(success, label, button))

        thread = threading.Thread(target=call_service)
        thread.start()

    def _on_complete(self, success: bool, label: str, button: ttk.Button):
        """Handle service complete."""
        button.configure(state='normal')
        if success:
            self._status.set(f"Status: {label} - SENT")
        else:
            self._status.set(f"Status: {label} - FAILED")


def main():
    # Init ROS
    rclpy.init(args=sys.argv)
    ros_node = MissionGUINode()

    # Create GUI
    app = MissionGUI(ros_node)

    # Spin ROS in thread
    def spin_ros():
        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.1)

    spin_thread = threading.Thread(target=spin_ros, daemon=True)
    spin_thread.start()

    # Run GUI
    app.mainloop()

    # Cleanup
    rclpy.shutdown()


if __name__ == '__main__':
    main()
