#!/usr/bin/env python3
"""
mission_gui.py
==============

Simple GUI for Mission Manager using Tkinter.

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
            'mission_manager/trigger_home_to_pick',
            'mission_manager/trigger_pick_to_drop',
            'mission_manager/trigger_drop_to_pick',
            'mission_manager/trigger_pick_to_home',
            'mission_manager/trigger_drop_to_home',
        ]
        for svc in services:
            self._services[svc] = self.create_client(Empty, svc)

    def call_service(self, service_name: str) -> bool:
        """Call mission service."""
        if service_name not in self._services:
            return False
        client = self._services[service_name]
        if not client.wait_for_service(timeout_sec=1.0):
            return False
        try:
            future = client.call_async(Empty.Request())
            return True
        except Exception:
            return False


class MissionGUI(tk.Tk):
    """GUI for Mission Manager."""

    def __init__(self, node: MissionGUINode):
        super().__init__()

        self._node = node
        self.title("AMR Mission Manager")
        self.geometry("500x650")

        # Main frame
        main_frame = ttk.Frame(self, padding="20")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Title
        title = ttk.Label(main_frame, text="AMR Mission Manager",
                          font=("Arial", 16, "bold"))
        title.pack(pady=10)

        # Missions
        missions = [
            ("HOME → PICK", "mission_manager/trigger_home_to_pick"),
            ("PICK → DROP", "mission_manager/trigger_pick_to_drop"),
            ("DROP → PICK", "mission_manager/trigger_drop_to_pick"),
            ("PICK → HOME", "mission_manager/trigger_pick_to_home"),
            ("DROP → HOME", "mission_manager/trigger_drop_to_home"),
        ]

        for i, (label, service) in enumerate(missions):
            btn = ttk.Button(main_frame, text=label, width=20)
            btn.pack(pady=5)
            btn.configure(command=lambda s=service, l=label, b=btn:
                        self._on_click(s, l, b))

        # Status
        self._status = tk.StringVar(value="Status: Ready")
        status_label = ttk.Label(main_frame, textvariable=self._status,
                                 font=("Arial", 10))
        status_label.pack(pady=20)

    def _on_click(self, service: str, label: str, button: ttk.Button):
        """Handle button click."""
        button.configure(state='disabled')
        self._status.set(f"Status: Executing {label}...")

        def call_service():
            success = self._node.call_service(service)
            self.after(0, lambda: self._on_complete(success, label, button))

        thread = threading.Thread(target=call_service)
        thread.start()

    def _on_complete(self, success: bool, label: str, button: ttk.Button):
        """Handle service complete."""
        button.configure(state='normal')
        if success:
            self._status.set(f"Status: {label} - DONE")
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
