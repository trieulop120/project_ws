#!/usr/bin/env python3
"""
amr_web/bridge_node.py
======================
ROS 2 Entry Point Node - Optimized for Overview Dashboard
- Heartbeat monitoring cho 4 thiết bị phần cứng
- Tắt camera stream (không encode/broadcast ảnh)
- MultiThreadedExecutor cho ROS callbacks
- SLAM web control: start/stop SLAM, save map, teleop, lift
"""

import asyncio
import json
import math
import os
import signal
import subprocess
import threading
import time
from typing import Optional
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import BatteryState, Image, LaserScan, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import uvicorn

from amr_web.websocket_manager import WebSocketManager
from amr_web.state_bridge import StateBridge
from amr_web.converters import odom_to_json, map_to_json, laser_to_json
from amr_web.web_server import create_app


def sensor_qos():
    """QoS Profile cho sensor data: Best Effort, depth=1, không giữ history."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE
    )


class WebBridge(Node):
    """
    ROS 2 Web Bridge Node - Optimized for Overview Dashboard.
    Giám sát heartbeat 4 thiết bị: ESP32, LiDAR, Camera, Lift.
    """

    def __init__(self):
        super().__init__('web_bridge')

        # Parameters
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8000)
        self.declare_parameter('pose_topic', '/odometry/filtered')
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('camera_topic', '/color/image_raw')
        self.declare_parameter('lidar_topic', '/scan_filtered')
        self.declare_parameter('joint_topic', '/joint_states')
        self.declare_parameter('pose_rate', 5.0)
        self.declare_parameter('battery_rate', 1.0)
        self.declare_parameter('system_status_rate', 1.0)
        self.declare_parameter('map_on_change', True)
        self.declare_parameter('verbose', True)

        self._host = self.get_parameter('host').value
        self._port = self.get_parameter('port').value
        self._pose_topic = self.get_parameter('pose_topic').value
        self._battery_topic = self.get_parameter('battery_topic').value
        self._map_topic = self.get_parameter('map_topic').value
        self._camera_topic = self.get_parameter('camera_topic').value
        self._lidar_topic = self.get_parameter('lidar_topic').value
        self._joint_topic = self.get_parameter('joint_topic').value
        self._pose_rate = self.get_parameter('pose_rate').value
        self._battery_rate = self.get_parameter('battery_rate').value
        self._system_status_rate = self.get_parameter('system_status_rate').value
        self._map_on_change = self.get_parameter('map_on_change').value
        self._verbose = self.get_parameter('verbose').value

        # State Managers
        self._state_bridge = StateBridge()
        self._ws_manager = WebSocketManager()
        self._last_map_hash = None

        # Lift state tracking
        self._lift_height_mm: int = 0
        self._lift_initialized: bool = False

        # SLAM Process Management
        self._slam_process: Optional[subprocess.Popen] = None
        self._slam_active: bool = False

        # Publishers cho teleop và lift
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._esp32_cmd_pub = self.create_publisher(String, '/esp32_cmd', 10)

        # Laser scan subscription
        self._laser_sub = self.create_subscription(
            LaserScan, self._lidar_topic, self._on_laser, qos_profile=sensor_qos())

        # FastAPI Server Setup
        self._app = create_app(
            state_bridge=self._state_bridge,
            ws_manager=self._ws_manager,
            node_logger=self.get_logger() if self._verbose else None,
            bridge_node=self  # Pass self để xử lý commands
        )

        # ROS Subscriptions
        self._pose_sub = self.create_subscription(
            Odometry, self._pose_topic, self._on_pose, qos_profile=sensor_qos())
        self._battery_sub = self.create_subscription(
            BatteryState, self._battery_topic, self._on_battery, qos_profile=sensor_qos())
        self._map_sub = self.create_subscription(
            OccupancyGrid, self._map_topic, self._on_map, qos_profile=sensor_qos())
        self._camera_sub = self.create_subscription(
            Image, self._camera_topic, self._on_camera, qos_profile=sensor_qos())
        self._lidar_sub = self.create_subscription(
            LaserScan, self._lidar_topic, self._on_lidar, qos_profile=sensor_qos())
        self._joint_sub = self.create_subscription(
            JointState, self._joint_topic, self._on_joint, qos_profile=10)

        # Threading
        self._running = True
        self._ws_thread: Optional[threading.Thread] = None
        self._broadcast_thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None

        # Rate control
        self._pose_interval = 1.0 / self._pose_rate
        self._battery_interval = 1.0 / self._battery_rate
        self._system_status_interval = 1.0 / self._system_status_rate
        self._laser_interval = 0.1  # 10 Hz for laser scan
        self._last_pose_send = 0.0
        self._last_battery_send = 0.0
        self._last_system_status_send = 0.0
        self._last_laser_send = 0.0
        self._last_map_send = 0.0
        self._map_interval = 0.5  # 2 Hz for map broadcast
        self._map_pending = False  # Flag: map data changed, needs broadcast

        if self._verbose:
            self.get_logger().info(f'Web Bridge Running on {self._host}:{self._port}')
            self.get_logger().info(f'Lift tracking: joint={self._joint_topic} (position in meters → mm)')

    def _on_pose(self, msg: Odometry) -> None:
        try:
            pose_data = odom_to_json(msg)
            self._state_bridge.update_pose(pose_data)
        except Exception as e:
            if self._verbose:
                self.get_logger().warn(f'Pose error: {e}')

    def _on_battery(self, msg: BatteryState) -> None:
        try:
            percentage = None
            if not math.isnan(msg.percentage):
                percentage = round(msg.percentage)

            battery_data = {
                "type": "battery",
                "timestamp": time.time(),
                "percentage": percentage,
                "voltage": round(msg.voltage, 1) if not math.isnan(msg.voltage) else None,
                "current": round(msg.current, 1) if not math.isnan(msg.current) else None,
                "charging": msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING,
            }
            self._state_bridge.update_battery(battery_data)
            self._state_bridge.update_heartbeat('esp32')
        except Exception as e:
            if self._verbose:
                self.get_logger().warn(f'Battery error: {e}')

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            map_data = map_to_json(msg)
            self._state_bridge.update_map(map_data)
            self._map_pending = True  # Flag: map changed, needs broadcast
        except Exception as e:
            if self._verbose:
                self.get_logger().warn(f'Map error: {e}')

    def _on_camera(self, msg: Image) -> None:
        """Camera callback - CHỈ cập nhật heartbeat."""
        self._state_bridge.update_heartbeat('camera')

    def _on_lidar(self, msg: LaserScan) -> None:
        """Callback cho LiDAR - chỉ cập nhật heartbeat."""
        self._state_bridge.update_heartbeat('lidar')

    def _on_laser(self, msg: LaserScan) -> None:
        """Callback cho laser scan - broadcast lên WebSocket."""
        try:
            laser_data = laser_to_json(msg)
            self._state_bridge.update_laser(laser_data)
        except Exception as e:
            if self._verbose:
                self.get_logger().warn(f'Laser error: {e}')

    def _on_joint(self, msg: JointState) -> None:
        """Callback cho Joint States - cập nhật lift height từ joint_states position."""
        if 'encoder_lift_joint' in msg.name:
            self._state_bridge.update_heartbeat('lift')

            try:
                joint_idx = msg.name.index('encoder_lift_joint')

                # Lấy vị trí theo mét (JointState cho Prismatic Joint dùng đơn vị Mét)
                lift_height_m = max(0.0, float(msg.position[joint_idx]))

                # Đổi sang mm nguyên (ép kiểu int tròn)
                height_mm = int(round(lift_height_m * 1000.0))

                # Đánh dấu đã initialized (đã home)
                if not self._lift_initialized:
                    self._lift_initialized = True
                    self._state_bridge.set_lift_initialized(True)

                self._lift_height_mm = height_mm

                # Cập nhật state bridge
                self._state_bridge.update_lift_height(self._lift_height_mm)
            except (ValueError, IndexError):
                pass

    def _broadcast_loop(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        async def run_broadcasts():
            while self._running:
                current_time = time.time()

                if current_time - self._last_pose_send >= self._pose_interval:
                    pose = self._state_bridge.get_latest_pose()
                    if pose:
                        await self._ws_manager.broadcast(json.dumps(pose))
                        self._last_pose_send = current_time

                if current_time - self._last_battery_send >= self._battery_interval:
                    battery = self._state_bridge.get_latest_battery()
                    if battery:
                        await self._ws_manager.broadcast(json.dumps(battery))
                        self._last_battery_send = current_time

                if current_time - self._last_system_status_send >= self._system_status_interval:
                    system_status = self._state_bridge.get_system_overview_state()
                    await self._ws_manager.broadcast(json.dumps(system_status))
                    self._last_system_status_send = current_time

                if current_time - self._last_laser_send >= self._laser_interval:
                    laser = self._state_bridge.get_latest_laser()
                    if laser:
                        await self._ws_manager.broadcast(json.dumps(laser))
                        self._last_laser_send = current_time

                # Broadcast map khi có thay đổi (rate-limited)
                if self._map_pending and (current_time - self._last_map_send) >= self._map_interval:
                    map_data = self._state_bridge.get_latest_map()
                    if map_data:
                        await self._ws_manager.broadcast(json.dumps(map_data))
                        self._last_map_send = current_time
                        self._map_pending = False

                await asyncio.sleep(0.05)

        try:
            self._loop.run_until_complete(run_broadcasts())
        except Exception as e:
            self.get_logger().error(f'Broadcast error: {e}')
        finally:
            self._loop.close()

    def start_websocket_server(self) -> None:
        self._ws_thread = threading.Thread(target=self._run_server, daemon=True)
        self._ws_thread.start()

    def _run_server(self) -> None:
        config = uvicorn.Config(
            self._app, host=self._host, port=self._port,
            log_level="info" if self._verbose else "warning"
        )
        server = uvicorn.Server(config)
        server.run()

    # ============================================================
    # SLAM Process Management
    # ============================================================

    def start_slam(self) -> bool:
        """Khởi chạy SLAM launch file."""
        # Check if process is actually running
        if self._slam_process and self._slam_process.poll() is None:
            self.get_logger().warn('SLAM process still running')
            return False

        try:
            # Source workspace properly
            ws_setup = os.path.expanduser('~/project_ws/install/setup.bash')

            cmd = f'source {ws_setup} && ros2 launch amr_mapping slam_web.launch.py'

            self._slam_process = subprocess.Popen(
                ['bash', '-c', cmd],
                stdout=subprocess.DEVNULL,  # Don't capture output
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self._slam_active = True
            self.get_logger().info('SLAM started: amr_mapping slam_web.launch.py')

            # Small delay to let process initialize
            time.sleep(0.5)

            # Verify process is still running
            if self._slam_process.poll() is not None:
                self.get_logger().error('SLAM process exited immediately')
                self._slam_active = False
                return False

            return True
        except Exception as e:
            self.get_logger().error(f'Failed to start SLAM: {e}')
            self._slam_active = False
            self._slam_process = None
            return False

    def get_slam_state(self) -> bool:
        """Trả về trạng thái SLAM đang chạy hay không."""
        # Kiểm tra cả process và flag
        if self._slam_process and self._slam_process.poll() is None:
            return True
        return self._slam_active

    def stop_slam(self) -> None:
        """Dừng SLAM - kill process group và các ROS nodes."""
        # Kill ROS nodes trước
        try:
            subprocess.run(
                ['bash', '-c',
                 'source ~/project_ws/install/setup.bash 2>/dev/null && '
                 'ros2 node kill /slam_toolbox 2>/dev/null; '
                 'ros2 node kill /octomap_server 2>/dev/null; '
                 'ros2 lifecycle set /slam_toolbox shutdown 2>/dev/null || true'],
                capture_output=True, timeout=5.0
            )
            self.get_logger().info('ROS nodes killed')
        except Exception as e:
            self.get_logger().warn(f'Node kill warning: {e}')

        # Kill process group
        if self._slam_process:
            try:
                pgid = os.getpgid(self._slam_process.pid)
                os.killpg(pgid, signal.SIGKILL)
                self._slam_process.wait(timeout=2.0)
                self.get_logger().info('SLAM process killed (SIGKILL)')
            except ProcessLookupError:
                self.get_logger().warn('SLAM process already dead')
            except Exception as e:
                self.get_logger().error(f'Failed to kill SLAM: {e}')
                try:
                    self._slam_process.kill()
                except:
                    pass
            finally:
                self._slam_process = None
                self._slam_active = False

    def save_map(self, map_name: str) -> tuple:
        """
        Lưu bản đồ sử dụng slam_toolbox service.
        Trả về (success, path).
        """
        # Tạo thư mục maps nếu chưa có
        maps_dir = os.path.expanduser('~/project_ws/maps')
        os.makedirs(maps_dir, exist_ok=True)

        # Đường dẫn tuyệt đối cho map
        map_path = os.path.join(maps_dir, map_name)
        map_path_abs = os.path.expanduser(map_path)

        try:
            # Dùng service call của slam_toolbox
            result = subprocess.run(
                [
                    'ros2', 'service', 'call',
                    '/slam_toolbox/save_map',
                    'slam_toolbox/srv/SaveMap',
                    f'{{name: {{data: "{map_path_abs}"}}}}'
                ],
                capture_output=True,
                text=True,
                timeout=30.0
            )

            if result.returncode == 0:
                yaml_path = f"{map_path_abs}.yaml"
                self.get_logger().info(f'Map saved: {yaml_path}')
                return True, yaml_path
            else:
                self.get_logger().error(f'Map save failed: {result.stderr}')
                return False, ""

        except subprocess.TimeoutExpired:
            self.get_logger().error('Map save timeout')
            return False, ""
        except Exception as e:
            self.get_logger().error(f'Map save error: {e}')
            return False, ""

    def get_available_maps(self) -> list:
        """
        Lấy danh sách maps từ ~/project_ws/maps/
        """
        import time
        maps = []

        try:
            maps_dir = os.path.expanduser('~/project_ws/maps')
            if os.path.exists(maps_dir):
                for f in sorted(os.listdir(maps_dir)):
                    if f.endswith('.yaml'):
                        full_path = os.path.join(maps_dir, f)
                        map_id = f.replace('.yaml', '')
                        # Lấy thời gian tạo thực tế từ file
                        try:
                            created = time.strftime('%Y-%m-%d', time.localtime(os.path.getctime(full_path)))
                        except:
                            created = time.strftime('%Y-%m-%d')
                        maps.append({
                            'id': map_id,
                            'name': map_id,
                            'path': full_path,
                            'created': created
                        })
        except Exception as e:
            self.get_logger().warn(f'Maps error: {e}')

        return maps

    def get_available_routes(self, map_id: str = None) -> list:
        """
        Lấy danh sách routes từ ~/project_ws/maps/{map_id}/routes/
        """
        import time
        routes = []

        try:
            if map_id:
                routes_dir = os.path.expanduser(f'~/project_ws/maps/{map_id}/routes')
                if os.path.exists(routes_dir):
                    for f in sorted(os.listdir(routes_dir)):
                        if f.endswith('.yaml'):
                            full_path = os.path.join(routes_dir, f)
                            route_id = f.replace('.yaml', '')
                            # Lấy thời gian tạo thực tế
                            try:
                                created = time.strftime('%Y-%m-%d', time.localtime(os.path.getctime(full_path)))
                            except:
                                created = time.strftime('%Y-%m-%d')
                            routes.append({
                                'id': route_id,
                                'name': route_id,
                                'path': full_path,
                                'created': created
                            })
        except Exception as e:
            self.get_logger().warn(f'Routes error: {e}')

        return routes

    # ============================================================
    # Teleop & Lift Control
    # ============================================================

    def publish_cmd_vel(self, linear: float, angular: float) -> None:
        """Publish cmd_vel cho teleop."""
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self._cmd_vel_pub.publish(twist)

    def publish_lift(self, action: str, target_mm: int) -> None:
        """Publish lift commands qua /esp32_cmd."""
        cmd = String()

        if action == "UP":
            cmd.data = "M30"  # Default lift up speed
        elif action == "DOWN":
            cmd.data = "M-30"  # Default lift down speed
        elif action == "HOME":
            cmd.data = "HOME"
        elif action == "SET_HEIGHT":
            # target_mm -> cm cho ESP32 protocol
            cmd.data = f"H{target_mm / 10:.1f}"
        else:
            return

        self._esp32_cmd_pub.publish(cmd)
        if self._verbose:
            self.get_logger().info(f'Lift cmd: {cmd.data}')


def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    bridge = WebBridge()
    executor.add_node(bridge)

    bridge.start_websocket_server()
    time.sleep(1.0)

    try:
        bridge._broadcast_thread = threading.Thread(
            target=bridge._broadcast_loop, daemon=True)
        bridge._broadcast_thread.start()
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge._running = False
        bridge.stop_slam()  # Cleanup SLAM process
        executor.remove_node(bridge)
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
