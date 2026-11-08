#!/usr/bin/env python3
"""
mission_manager_node.py
======================

AMR Mission Manager Node.

Responsibilities:
- Resolve destination from route_graph.yaml
- Select mission-specific BT
- Send NavigateToPose with selected BT

Flow:
    GUI button / Service call
        ↓
    Resolve destination from route_graph.yaml
        ↓
    Select mission-specific BT
        ↓
    NavigateToPose(pose, BT=<mission>.xml)
        ↓
    BT: ComputeRoute → FollowPath → ComputePathToPose → FollowPath
    (or NavigateToPose for return_home)

Usage:
    ros2 run amr_mission_manager mission_manager

Author: AMR System
"""

import json
import os
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputeRoute, NavigateToPose
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_srvs.srv import Empty


NODE_NAME = 'mission_manager'

# Mission -> (start_node, dest_node, BT_file)
# - start_node/dest_node: ten node trong graph do Nav2 launch dang publish
# - return_home: khong can probe route (BT la Nav2 thuan, khong co ComputeRoute)
MISSION_BT_MAP = {
    'home_to_p1': ('NODE_HOME', 'P_1', 'home_to_p1.xml'),   # HOME -> P_1
    'home_to_p2': ('NODE_HOME', 'P_2', 'home_to_p2.xml'),   # HOME -> P_2
    'p1_to_d1':   ('P_1',       'D_1', 'p1_to_d1.xml'),     # P_1 -> D_1
    'p2_to_d1':   ('P_2',       'D_1', 'p2_to_d1.xml'),     # P_2 -> D_1
    'd1_to_p1':   ('D_1',       'P_1', 'd1_to_p1.xml'),     # D_1 -> P_1
    'd1_to_p2':   ('D_1',       'P_2', 'd1_to_p2.xml'),     # D_1 -> P_2
    'return_home': ('',          'NODE_HOME', 'return_home.xml'),
}

# Probe route_server (Q2): toi da 0.5s - khong de robot chon lan khi server chet.
PROBE_WAIT_SERVER_S = 0.5
PROBE_RESULT_TIMEOUT_S = 5.0


class RouteProbe:
    """Probe /compute_route tren rclpy Context rieng, chay trong thread rieng.

    LY DO tach rieng: probe duoc goi tu trong service callback cua node chinh.
    Neu dung chung executor voi node chinh se DEADLOCK (executor dang ban
    xu ly callback thi khong the spin them). Vay vay probe dung:
      - 1 Context rieng
      - 1 Node rieng ('mission_route_probe')
      - 1 Executor rieng, spin trong thread daemon
    """

    def __init__(self):
        self._ctx = rclpy.Context()
        self._ctx.init()
        self._node = Node('mission_route_probe', context=self._ctx)
        self._client = ActionClient(self._node, ComputeRoute, 'compute_route')
        self._executor = SingleThreadedExecutor(context=self._ctx)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        # Warm-up DDS discovery: action client vua tao can thoi gian de
        # tim thay server. Neu khong warm-up, probe DAU TIEN goi
        # wait_for_server(0.5s) se fail (chua co server trong graph cua
        # client) -> mission dau tien roi ve FREE du graph hoan toan hop le.
        # Warm-up chay o background, khong chan mission.
        self._warmup_done = False
        self._warmup_thread = threading.Thread(
            target=self._warmup, daemon=True)
        self._warmup_thread.start()

    def _warmup(self):
        """Cho action client discovery server (khong chan mission)."""
        try:
            self._client.wait_for_server(timeout_sec=10.0)
            self._warmup_done = True
        except Exception:
            pass

    def probe(self, start_id: int, dest_id: int,
              wait_server_s: float, timeout_s: float) -> tuple:
        """Tra (usable: bool, reason: str).

        wait_server_s = cho toi da 0.5s (yeu cau Q2). Neu discovery chua xong
        (warm-up) thi cho them toi da 2s MOT LAN dau tien - sau do moi
        that su dung ngan 0.5s cho cac mission tiep theo.
        """
        # Lan dau tien: cho discovery (neu warm-up chua xong) - toi da 2s.
        if not self._warmup_done:
            self._client.wait_for_server(timeout_sec=2.0)
            self._warmup_done = True

        if not self._client.wait_for_server(timeout_sec=wait_server_s):
            return False, ('route_server khong phan ung (chua active, hoac graph '
                           'malformed khong load duoc)')

        goal = ComputeRoute.Goal()
        goal.start_id = start_id
        goal.goal_id = dest_id
        goal.use_start = False
        goal.use_poses = False

        try:
            handle = self._send_and_wait(goal, timeout_s)
            if handle is None:
                return False, 'route_server khong tra ket qua trong thoi gian'
            if not handle.accepted:
                return False, 'route_server tu choi goal'

            wrapped = self._get_result(handle, timeout_s)
            if wrapped is None:
                return False, 'route_server khong tra ket qua trong thoi gian'
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                return False, (f'ComputeRoute that bai (status={wrapped.status})')
            if not wrapped.result.path.poses:
                return False, 'ComputeRoute tra path rong'
            return True, 'route OK'
        except Exception as e:
            return False, f'probe route loi: {e}'

    def _send_and_wait(self, goal, timeout_s):
        """Gui goal va cho handle (accept/reject) trong timeout."""
        done = threading.Event()
        box = {}

        def _on_sent(fut):
            try:
                box['handle'] = fut.result()
            except Exception as e:
                box['error'] = e
            finally:
                done.set()

        self._client.send_goal_async(goal).add_done_callback(_on_sent)
        if not done.wait(timeout_s):
            return None
        if 'error' in box:
            raise box['error']
        return box.get('handle')

    def _get_result(self, handle, timeout_s):
        """Cho result cua goal, tra wrapped result hoac None neu timeout."""
        done = threading.Event()
        box = {}

        def _on_result(fut):
            try:
                box['result'] = fut.result()
            except Exception as e:
                box['error'] = e
            finally:
                done.set()

        handle.get_result_async().add_done_callback(_on_result)
        if not done.wait(timeout_s):
            return None
        if 'error' in box:
            raise box['error']
        return box.get('result')

    def close(self):
        """Dung executor va huy node/context."""
        try:
            self._executor.shutdown()
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
        try:
            self._ctx.shutdown()
        except Exception:
            pass


class MissionManager(Node):
    """AMR Mission Manager - sends navigation goals with mission-specific BT."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter('bt_package', 'amr_navigation')

        self._bt_package = self.get_parameter('bt_package').value

        # Route graph source is announced by route_graph_publisher, which is
        # configured by nav2.launch. Do not choose a package-local graph here.
        self._loader = None
        route_qos = QoSProfile(depth=1)
        route_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        route_qos.reliability = ReliabilityPolicy.RELIABLE
        self._route_path_sub = self.create_subscription(
            String, '/navigation/route_yaml_path', self._on_route_path, route_qos)

        # NavigateToPose action client
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # ComputeRoute's BT node may fail while Nav2 is loading the custom BT
        # if /compute_route is not active. In that case, send the same pose
        # with Nav2's configured default BT so navigation can still proceed.
        self._route_client = ActionClient(self, ComputeRoute, 'compute_route')

        # Probe /compute_route tren Context+thread rieng (xem class RouteProbe).
        # Dung de quyet dinh ROUTE vs FREE ma khong deadlock executor chinh.
        self._probe = None
        try:
            self._probe = RouteProbe()
        except Exception as e:
            self.get_logger().warn(f'Khong tao duoc route probe: {e}')

        # --- Trang thai cho LLM (JSON) ---
        # /mission/status: mot topic JSON duy nhat, noi dung:
        #   {mission, state, nav_mode, reason, route_reason, ts}
        # state: EXECUTING | SUCCEEDED | ABORTED | CANCELED
        # nav_mode: ROUTE (di theo route graph) | FREE (Nav2 free navigation)
        self._status_pub = self.create_publisher(String, '/mission/status', 10)
        self._nav_mode = 'FREE'
        self._route_reason = ''
        self._active_mission = None

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('NavigateToPose server not ready')
        else:
            self.get_logger().info('NavigateToPose server connected')

        # Create mission services
        self._create_services()

        self.get_logger().info('Mission Manager initialized')

    def _on_route_path(self, msg: String):
        """Load the route selected by the Nav2 launch."""
        try:
            from amr_navigation.route_graph_loader import RouteGraphLoader
            self._loader = RouteGraphLoader(
                yaml_path=msg.data
            )
            self.get_logger().info(f'Using active Nav2 route graph: {msg.data}')
            self.get_logger().info(self._loader.get_summary())
        except Exception as e:
            self._loader = None
            self.get_logger().error(f'Failed to load active Nav2 route graph {msg.data}: {e}')

    def _get_bt_path(self, bt_file: str) -> str:
        """Get BT path for mission."""
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory(self._bt_package)
            return os.path.join(pkg, 'config', 'bt', bt_file)
        except Exception as e:
            self.get_logger().error(f'Failed to get BT path: {e}')
            return ''

    def _create_services(self):
        """Create all mission trigger services."""
        for svc_name in MISSION_BT_MAP.keys():
            service_name = f'mission_manager/trigger_{svc_name}'
            self.create_service(
                Empty,
                service_name,
                lambda req, res, svc=svc_name: self._on_mission(req, res, svc)
            )

        self.get_logger().info(f'Services: {", ".join(MISSION_BT_MAP.keys())}')

    def _on_mission(self, request, response, mission_name: str):
        """Handle mission request."""
        if mission_name not in MISSION_BT_MAP:
            self.get_logger().error(f'Unknown mission: {mission_name}')
            return response

        start_node, dest_node, bt_file = MISSION_BT_MAP[mission_name]
        self.get_logger().info(f'=== {mission_name} -> {dest_node} ===')
        self._active_mission = mission_name
        self._execute_mission(mission_name, start_node, dest_node, bt_file)
        return response

    def _execute_mission(self, mission_name: str, start_node: str,
                         destination: str, bt_file: str) -> bool:
        """Execute mission to destination.

        Chien luoc hybrid (Q1 - duyet):
          - Co route (graph hop le + route_server phan ung + BT khop ID)
            -> gui BT mission (ComputeRoute + FollowPath, co Fallback ben trong).
          - Khong route -> gui Nav2 default BT (ComputePathToPose + FollowPath).
        Ca hai duong deu toi duoc dich; chi khac che do duoc bao cao cho LLM.
        """
        if self._loader is None:
            self.get_logger().error('Route loader not available')
            self._publish_status(mission_name, 'ABORTED', 'route loader khong khoi dong')
            return False

        try:
            # Resolve destination node
            dst_node = self._loader.resolve(destination)
            if not dst_node:
                self.get_logger().error(f'Cannot resolve: {destination}')
                self._publish_status(mission_name, 'ABORTED',
                                    f'graph thieu node {destination}')
                return False

            self.get_logger().info(f'  Destination: {dst_node}')

            # Get destination pose
            pose = self._loader.get_node_pose(dst_node)
            if pose is None:
                self.get_logger().error(f'No pose for {dst_node}')
                self._publish_status(mission_name, 'ABORTED',
                                    f'graph thieu pose cua {dst_node}')
                return False

            self.get_logger().info(f'  Goal: x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}')

            # --- Quyet dinh che do: ROUTE hay FREE ---
            bt_path, nav_mode, route_reason = self._select_navigation_mode(
                mission_name, start_node, dst_node, bt_file)

            self._nav_mode = nav_mode
            self._route_reason = route_reason
            if nav_mode == 'FREE':
                self.get_logger().warn(
                    f'Route khong dung duoc ({route_reason}) -> '
                    f'fallback Nav2 free navigation')
            else:
                self.get_logger().info('  BT: %s (route)' % bt_file)

            self._navigate_to_pose(pose, dst_node, bt_path,
                                   use_home_pose=(mission_name == 'return_home'))
            return True

        except Exception as e:
            self.get_logger().error(f'Mission failed: {e}')
            import traceback
            traceback.print_exc()
            self._publish_status(mission_name, 'ABORTED', str(e))
            return False

    def _select_navigation_mode(self, mission_name: str, start_node: str,
                                dest_node: str, bt_file: str) -> tuple:
        """Chon BT va che do di chuyen. Tra (bt_path, nav_mode, route_reason).

        nav_mode:
          'ROUTE' -> dung BT mission (ComputeRoute + FollowPath)
          'FREE'  -> dung Nav2 default BT (ComputePathToPose + FollowPath)
        """
        # return_home: BT rieng, khong co ComputeRoute -> luon FREE.
        if mission_name == 'return_home':
            bt = self._get_bt_path(bt_file)
            if not bt:
                self.get_logger().error(f'BT not found: {bt_file}')
                return '', 'FREE', 'BT khong tim thay'
            return bt, 'FREE', 'BT return_home khong dung route'

        # 1) BT file co ton tai?
        bt = self._get_bt_path(bt_file)
        if not bt:
            self.get_logger().error(f'BT not found: {bt_file}')
            return '', 'FREE', 'BT khong tim thay'

        # 2) Validate ID trong BT co khop thu tu node trong YAML khong?
        ok, why = self._validate_bt_node_ids(bt, start_node, dest_node)
        if not ok:
            self.get_logger().error(
                f'[BT/Graph ID mismatch] {why} -> BO QUA route, dung Nav2 free '
                f'navigation (khong xoa hay doi BT).')
            return '', 'FREE', why

        # 3) Probe route_server: route co dung duoc khong?
        route_ok, reason = self._probe_route(start_node, dest_node)
        if not route_ok:
            return '', 'FREE', reason

        return bt, 'ROUTE', 'route OK'

    def _validate_bt_node_ids(self, bt_path: str, start_node: str,
                              dest_node: str) -> tuple:
        """Kiem tra ID trong BT (SetBlackboard my_start_id/my_goal_id) co khop
        voi thu tu node trong route graph YAML khong.

        BT dinh nghia ID cung (uint16) cho ComputeRoute, nen ID phai khop
        thu tu node trong YAML. Neu user them/xoa node -> ID lech -> bo qua
        route (dung Nav2 free navigation) thay vi sua BT tu dong.

        Returns (ok: bool, reason: str)
        """
        if not start_node:
            # Mission khong co diem xuat phat (return_home) -> khong can check.
            return True, ''
        try:
            start_id = self._loader.get_node_id(start_node)
            dest_id = self._loader.get_node_id(dest_node)
            if start_id is None or dest_id is None:
                missing = start_node if start_id is None else dest_node
                return False, f'graph thieu node {missing}'

            tree = ET.parse(bt_path)
            root = tree.getroot()
            bt_start = root.find('.//SetBlackboard[@output_key="my_start_id"]')
            bt_goal = root.find('.//SetBlackboard[@output_key="my_goal_id"]')
            if bt_start is None or bt_goal is None:
                return False, 'BT khong co SetBlackboard my_start_id/my_goal_id'

            bt_start_id = int(bt_start.get('value'))
            bt_goal_id = int(bt_goal.get('value'))
            if bt_start_id != start_id or bt_goal_id != dest_id:
                order = self._loader.get_all_node_names()
                return False, (
                    f'BT ID khong khop graph: BT({bt_start_id}->{bt_goal_id}) vs '
                    f'YAML({start_id}->{dest_id}) [{start_node}->{dest_node}]. '
                    f'Thu tu node YAML hien tai: {order}')
            return True, ''
        except Exception as e:
            return False, f'khong doc duoc BT/graph: {e}'

    def _probe_route(self, start_node: str, dest_node: str) -> tuple:
        """Thu route_server xem route (start_node -> dest_node) co dung duoc khong.

        Returns (usable: bool, reason: str)

        Luu y: ComputeRoute.result khong co field error_code, nen khong phan
        biet duoc 'graph malformed' voi '2 diem khong noi duoc' - chi biet
        route that bai hay khong.
        """
        start_id = self._loader.get_node_id(start_node)
        dest_id = self._loader.get_node_id(dest_node)
        if start_id is None or dest_id is None:
            return False, 'graph thieu node'

        if self._probe is None:
            return False, 'route probe khong khoi dong'
        return self._probe.probe(start_id, dest_id,
                                  PROBE_WAIT_SERVER_S, PROBE_RESULT_TIMEOUT_S)

    def _navigate_to_pose(self, pose: PoseStamped, node_name: str, bt_path: str, use_home_pose: bool = False):
        """Send NavigateToPose goal with custom BT (bt_path rong = Nav2 default BT)."""
        pose.header.stamp = self.get_clock().now().to_msg()

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = bt_path

        # For return_home, the BT expects {home_pose} instead of {goal}
        # We pass pose as both to handle this
        if use_home_pose:
            self.get_logger().info('  Mode: NavigateToPose (direct to HOME)')
        else:
            self.get_logger().info(
                f'  Mode: {"ComputeRoute + FollowPath" if bt_path else "Nav2 free navigation"}')

        self.get_logger().info(f'  Sending NavigateToPose...')

        if not self._nav_client.server_is_ready():
            self.get_logger().error('NavigateToPose server not ready')
            return

        send_future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: None
        )
        send_future.add_done_callback(
            lambda f: self._on_goal_sent(f, node_name)
        )

    def _on_goal_sent(self, future, node_name: str):
        """Handle goal sent result."""
        try:
            handle = future.result()
            if handle and handle.accepted:
                self.get_logger().info(f'  Goal accepted: {node_name}')
                # Robot dang chay -> bao EXECUTING cho LLM ngay.
                self._publish_status(self._active_mission, 'EXECUTING',
                                     'robot dang di')
                result_future = handle.get_result_async()
                result_future.add_done_callback(
                    lambda f: self._on_complete(f, node_name)
                )
            else:
                self.get_logger().error('  Goal rejected')
                self._publish_status(self._active_mission, 'ABORTED',
                                     'Nav2 tu choi goal')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            self._publish_status(self._active_mission, 'ABORTED', str(e))

    def _on_complete(self, future, node_name: str):
        """Handle navigation complete.

        FIX (cho LLM): rclpy tra GetResult.Response co field `.status`
        (uint8, theo action_msgs/GoalStatus). Truoc day code kiem tra
        `hasattr(result, 'goal_handle')` - nhung ActionResult khong co
        attribute do -> luon rơi vao nhanh 'COMPLETED', kể cả khi goal
        ABORTED/CANCELED. Bay go doc `status` that.
        """
        try:
            wrapped = future.result()
            if wrapped is None:
                self.get_logger().warn(f'  === {node_name} FAILED ===')
                self._publish_status(self._active_mission, 'ABORTED',
                                     'khong nhan duoc ket qua goal')
                return

            status = wrapped.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'  === {node_name} SUCCEEDED ===')
                self._publish_status(self._active_mission, 'SUCCEEDED',
                                     'da den noi')
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn(f'  === {node_name} ABORTED ===')
                self._publish_status(
                    self._active_mission, 'ABORTED',
                    'ke chay: route va Nav2 free navigation deu that bai')
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn(f'  === {node_name} CANCELED ===')
                self._publish_status(self._active_mission, 'CANCELED',
                                     'goal bi huy')
            else:
                self.get_logger().warn(f'  === {node_name} status={status} ===')
                self._publish_status(self._active_mission, 'ABORTED',
                                     f'goal status={status}')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            self._publish_status(self._active_mission, 'ABORTED', str(e))

    def _publish_status(self, mission_name, state: str, reason: str = ''):
        """Publish trang thai JSON cho LLM tren /mission/status."""
        if mission_name is None:
            return
        payload = {
            'mission': mission_name,
            'state': state,
            'nav_mode': self._nav_mode,
            'reason': reason,
            'route_reason': self._route_reason,
            'timestamp': time.time(),
        }
        try:
            m = String()
            m.data = json.dumps(payload, ensure_ascii=False)
            self._status_pub.publish(m)
        except Exception as e:
            self.get_logger().warn(f'Khong publish status duoc: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._probe is not None:
            node._probe.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
