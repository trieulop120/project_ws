#!/usr/bin/env python3
"""Operator-selected shelf pallet retrieval coordinator.

The node owns retrieval sequencing and consumes the detector's opt-in,
full-frame candidate stream. The operator selects the source slot; this
coordinator owns the complete retrieval mission once started.

Change log (this revision):
  * D_1 navigation is a plain Nav2 NavigateToPose goal (default BT), like pick.
  * Nav2 handoff (same logic as pick): if the robot is within `d1_handoff_m` of the D_1 goal for
    `d1_handoff_hold_s` and Nav2 has still not finished, cancel Nav2 and let
    `retrieval_align` finish the job (it tolerates 40 cm / 25 deg).
  * Nav2 abort/timeout is accepted when the robot is within `d1_abort_ok_m`.

Step 1 supersedes the navigation notes above: selected-slot stances use the
slot TF at a fixed 1.20 m standoff. Handoff/timeout must await the terminal
Nav2 result and a measured stop. Side-column integration with retrieval_align
is explicitly deferred to step 2; its centered-D_1 admission is unchanged.

Step 2 integrates all columns: retrieval_align admits the selected slot's
configured waiting pose and only corrects small residual errors. Camera-based
retrieval_pick navigates to the detected pallet's 0.65 m staging pose after
detection, then performs its separate alignment before raising the forks.
"""

import math
import os
import threading
import time
import uuid

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int8MultiArray, String
from std_srvs.srv import Empty, Trigger
import tf2_ros

from amr_mission_manager.shelf_model import ShelfModel, FULL


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PalletRetrievalNode(Node):
    def __init__(self):
        super().__init__('pallet_retrieval')
        self.declare_parameter('target_slot', '')
        self.declare_parameter('shelf_config', '')
        self.declare_parameter('stable_frames', 5)
        self.declare_parameter('candidate_timeout_s', 15.0)
        self.declare_parameter('detector_startup_timeout_s', 30.0)
        self.declare_parameter('pick_timeout_s', 490.0)  # staging 180 + settle 10 + mechanical pick 300
        # --- Nav2 to D_1 ---
        self.declare_parameter('d1_nav_timeout_s', 180.0)
        self.declare_parameter('d1_handoff_m', 0.20)         # 0 = disable handoff
        self.declare_parameter('d1_handoff_hold_s', 4.0)   # same as pick (handoff_hold_s)
        self.declare_parameter('d1_abort_ok_m', 0.35)
        self.declare_parameter('stance_yaw_tol_deg', 25.0)
        self.declare_parameter('stance_settle_s', 1.0)
        self.declare_parameter('stance_stop_timeout_s', 10.0)
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('base_frame', 'base_footprint')
        self._model = self._load_model()
        self._lock = threading.RLock()
        self._run_lock = threading.Lock()
        self._bits = None
        self._shelf_t = 0.0
        self._candidates = None
        self._candidate_t = 0.0
        self._detection_status = None
        self._detection_status_t = 0.0
        self._pm_status = None
        self._pm_status_t = 0.0
        self._supervisor = None
        self._supervisor_t = 0.0
        self._mission_status = None
        self._mission_status_t = 0.0
        self._worker = None
        self._resume_stage = None
        self._resume_slot = None
        self._issued_stages = {}
        self._saved_target = None
        self._recovery_since = 0.0
        self._recovery_pm_session = None
        self._recovery_stop_future = None
        self._route_request_pending = None
        self._resumed_pm_after = 0.0
        self._resumed_pm_id = 0
        self._last_status = None
        self._run_id = None
        self._resume_revision = 0
        self._resume_in_progress = False
        self._discard_pending = False
        self._discard_future = None
        self._cancel_event = threading.Event()
        self._active_nav_goal = None
        self._stance_pending = None
        self._odom_t = self._nav_status_t = 0.0
        self._odom_speed = None
        self._nav_goal_active = None
        self._stance_stopped_since = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._selected_pose_pub = self.create_publisher(
            PoseStamped, '/pallet_retrieval/selected_pose', 10)
        self._selected_slot_pub = self.create_publisher(
            String, '/pallet_retrieval/selected_slot', 10)
        detector_qos = QoSProfile(depth=1,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                  reliability=ReliabilityPolicy.RELIABLE)
        self._candidate_enable_pub = self.create_publisher(
            Bool, '/pallet_retrieval/enable', detector_qos)
        self._status_pub = self.create_publisher(
            String, '/pallet_retrieval/status', 10)
        self.create_subscription(PoseArray, '/pallet_retrieval/candidates',
                                 self._on_candidates, 10)
        self.create_subscription(String, '/pallet_retrieval/detection_status',
                                 self._on_detection_status, 10)
        self.create_subscription(Int8MultiArray, '/shelf/occupancy',
                                 self._on_shelf, 10)
        self.create_subscription(String, '/pallet_mission/status',
                                 self._on_pm_status, 10)
        self.create_subscription(String, '/mission_supervisor/status',
                                 self._on_supervisor, 10)
        self.create_subscription(String, '/mission/status',
                                 self._on_mission_status, 10)
        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value),
                                 self._on_stance_odom, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 self._on_stance_nav_status, 10)

        self._start_srv = self.create_service(Trigger,
                                              '/pallet_retrieval/start',
                                              self._srv_start)
        self._resume_srv = self.create_service(Trigger, '/pallet_retrieval/resume', self._srv_resume)
        self._resume_cli = self.create_client(Trigger, '/pallet_mission/resume')
        self._cancel_srv = self.create_service(Trigger,
                                               '/pallet_retrieval/cancel',
                                               self._srv_cancel)
        self._manager_stop_cli = self.create_client(Trigger, '/mission_manager/stop')
        self._manager_stop_request = None
        self._expected_pm_session = None
        self._pick_cli = self.create_client(Trigger, '/pallet_mission/retrieval_pick')
        self._lift_home_cli = self.create_client(Trigger, '/pallet_mission/retrieval_home')
        self._align_cli = self.create_client(Trigger, '/pallet_mission/retrieval_align')
        self._stop_cli = self.create_client(Trigger, '/pallet_mission/stop')
        self._discard_cli = self.create_client(Trigger, '/pallet_mission/discard_recovery')
        self._detector_start_cli = self.create_client(
            Trigger, '/pallet_mission/retrieval_detector_start')
        self._detector_stop_cli = self.create_client(
            Trigger, '/pallet_mission/retrieval_detector_stop')
        self._release_cli = self.create_client(Trigger, '/pallet_mission/retrieval_release')
        self._backoff_cli = self.create_client(Trigger, '/pallet_mission/retrieval_backoff')
        self._d1_d2_cli = self.create_client(Empty,
                                             '/mission_manager/trigger_d1_to_d2')
        self._home_cli = self.create_client(Empty,
                                            '/mission_manager/trigger_return_home')
        self.create_timer(1.0, self._publish_status_heartbeat)
        self.get_logger().info('Operator-triggered Auto pallet retrieval coordinator ready')

    def _load_model(self):
        path = str(self.get_parameter('shelf_config').value or '').strip()
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('amr_mission_manager'),
                                'config', 'shelf.yaml')
        return ShelfModel.from_yaml(path)

    def _on_shelf(self, msg):
        with self._lock:
            self._bits = [int(v) for v in msg.data]
            self._shelf_t = time.monotonic()

    def _on_candidates(self, msg):
        with self._lock:
            self._candidates = msg
            self._candidate_t = time.monotonic()

    def _on_detection_status(self, msg):
        try:
            import json
            data = json.loads(msg.data)
            with self._lock:
                self._detection_status = data
                self._detection_status_t = time.monotonic()
        except Exception:
            pass

    def _on_pm_status(self, msg):
        try:
            with self._lock:
                self._pm_status = __import__('json').loads(msg.data)
                self._pm_status_t = time.monotonic()
        except Exception:
            pass

    def _on_supervisor(self, msg):
        try:
            with self._lock:
                self._supervisor = __import__('json').loads(msg.data)
                self._supervisor_t = time.monotonic()
        except Exception:
            pass

    def _on_mission_status(self, msg):
        try:
            with self._lock:
                self._mission_status = __import__('json').loads(msg.data)
                self._mission_status_t = time.monotonic()
        except Exception:
            pass

    def _status(self, state, message, slot=None, nav_leg=None, resuming=None, discarded=False):
        with self._lock:
            previous = self._last_status or {}
            paused = state in ('RECOVERY', 'CANCELING')
            from_state = previous.get('recovery_from') or previous.get('state')
            from_leg = previous.get('recovery_nav_leg') or previous.get('nav_leg')
            if not paused or resuming is False:
                self._resume_in_progress = False
            self._last_status = {
                'state': state, 'message': message, 'slot': slot,
                'nav_leg': nav_leg if state == 'NAVIGATING_TO_D1' else None,
                'recovery_from': from_state if paused else None,
                'recovery_nav_leg': from_leg if paused else None,
                'discarded': bool(discarded),
            }
        self._publish_status_heartbeat()

    def _publish_status_heartbeat(self):
        import json
        with self._lock:
            if self._last_status is None:
                return
            pm = self._pm_status or {}
            cargo = pm.get('cargo') if time.monotonic() - self._pm_status_t <= 2. else None
            cargo = cargo if isinstance(cargo, bool) else None
            payload = dict(self._last_status)
            payload['run_id'] = self._run_id
            payload['resume_revision'] = self._resume_revision
            payload['resuming'] = self._resume_in_progress and self._run_lock.locked()
            payload['discard_pending'] = self._discard_pending
            payload['cargo'] = cargo
            payload['recovery_required'] = payload['state'] == 'RECOVERY' or (
                payload['state'] in ('FAILED', 'CANCELED') and cargo is not False)
            reason = self._resume_blocked() if payload['state'] == 'RECOVERY' else ''
            payload['resume_available'] = payload['state'] == 'RECOVERY' and not reason
            payload['resume_reason'] = reason
            payload['operation_checkpoint'] = (pm.get('recovery') or {}).get('checkpoint')
            payload['resume_step'] = self._resume_stage
            if payload['recovery_required'] and cargo is not False and 'inspect cargo/lift' not in payload['message']:
                payload['message'] += (' | Pallet may still be on the forks; inspect cargo/lift '
                                       'before starting another mission.')
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)

    def _srv_start(self, _req, res):
        if self._discard_pending:
            res.success, res.message = False, 'Previous retrieval discard is still pending'
            return res
        if self._resume_stage is not None:
            res.success, res.message = False, 'Retrieval has an interrupted operation; Resume it before starting a new retrieval'
            return res
        with self._lock:
            if self._stance_pending is not None or self._active_nav_goal is not None:
                res.success, res.message = False, 'Previous retrieval navigation has not ended'
                return res
        slot = str(self.get_parameter('target_slot').value or '').strip().upper()
        if slot not in self._model._slot_by_id:
            res.success, res.message = False, f'Invalid retrieval slot: {slot!r}'
            return res
        with self._lock:
            bits, shelf_t = self._bits, self._shelf_t
            supervisor = self._supervisor
            supervisor_t = self._supervisor_t
            pm = self._pm_status
            pm_t = self._pm_status_t
        if bits is None or time.monotonic() - shelf_t > 2.0:
            res.success, res.message = False, 'Shelf occupancy is missing or stale'
            return res
        states = self._model.slot_states(bits, fresh=True)
        if states.get(slot) != FULL:
            res.success, res.message = False, f'Slot {slot} is {states.get(slot, "unknown")}'
            return res
        if supervisor is None or time.monotonic() - supervisor_t > 2.0:
            res.success, res.message = False, 'Mission supervisor status is missing or stale'
            return res
        if supervisor.get('state') not in ('IDLE', 'DONE', 'FAILED', 'STOPPED'):
            res.success, res.message = False, 'Mission supervisor is active'
            return res
        if pm is None or time.monotonic() - pm_t > 2.0:
            res.success, res.message = False, 'Pallet mission status is missing or stale'
            return res
        recovery = pm.get('recovery') or {}
        if recovery.get('checkpoint') or recovery.get('stop_pending'):
            res.success, res.message = False, 'Resolve the saved pallet operation or pending Stop before a new retrieval'
            return res
        if pm.get('cargo') is not False or pm.get('state') not in (
                'IDLE', 'DONE', 'FAILED', 'STOPPED'):
            res.success, res.message = False, 'Pallet handling is active or empty forks are not confirmed'
            return res
        if not self._run_lock.acquire(blocking=False):
            res.success, res.message = False, 'Retrieval is already active'
            return res
        with self._lock:
            self._run_id = uuid.uuid4().hex
            self._resume_revision = 0
            self._resume_in_progress = False
            self._discard_pending = False
            self._discard_future = None
            self._expected_pm_session = pm.get('session_id')
            self._resume_stage, self._resume_slot = 0, slot
            self._issued_stages = {}
            self._saved_target = None
            self._recovery_pm_session = None
            self._resumed_pm_after = 0.0
            self._cancel_event.clear()
            self._status('HOMING_LIFT', 'Preparing retrieval; homing lift first', slot)
        self._worker = threading.Thread(target=self._run, args=(slot,), daemon=True)
        self._worker.start()
        res.success, res.message = True, f'Retrieval started for {slot}'
        return res

    def _on_discard_result(self, future):
        """Advance to terminal only after pallet_mission confirms discard."""
        try:
            result = future.result()
            ok = bool(result.success)
            detail = result.message or ''
        except Exception as exc:
            ok, detail = False, str(exc)
        with self._lock:
            if future is not self._discard_future:
                return  # late result from a superseded run
            self._discard_pending = False
            self._discard_future = None
            slot = self._resume_slot
            if ok:
                self._resume_stage = None
                self._issued_stages.clear()
                self._saved_target = None
                self._recovery_pm_session = None
                self._status('CANCELED', 'Retrieval discarded', slot, discarded=True)
                self.get_logger().info('Retrieval discard confirmed by pallet_mission')
            else:
                # Recovery is retained: no false IDLE, no false permission to start.
                self._status('RECOVERY', 'Retrieval discard not confirmed', slot, resuming=False)
                self.get_logger().warn(f'Retrieval discard rejected: {detail}')

    def _srv_cancel(self, _req, res):
        if not self._run_lock.locked():
            if self._discard_pending:
                res.success, res.message = True, 'Retrieval discard is pending'
                return res
            if self._resume_stage is not None:
                if not self._discard_cli.service_is_ready():
                    res.success, res.message = False, 'Pallet discard service is unavailable'
                    return res
                with self._lock:
                    self._discard_pending = True
                    self._status('CANCELING', 'Discarding retrieval recovery', self._resume_slot)
                try:
                    future = self._discard_cli.call_async(Trigger.Request())
                    self._discard_future = future
                    future.add_done_callback(self._on_discard_result)
                except Exception as exc:
                    self._discard_pending = False
                    self._discard_future = None
                    self._status('RECOVERY', 'Retrieval discard not confirmed', self._resume_slot)
                    res.success, res.message = False, str(exc)
                    return res
                res.success, res.message = True, 'Retrieval discard requested'
                return res
            res.success, res.message = False, 'No retrieval mission is active'
            return res
        self._cancel_event.set()
        active_goal = self._active_nav_goal
        if active_goal is not None:
            try:
                active_goal.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f'Could not cancel retrieval Nav2 goal: {exc}')
        slot = (self._last_status or {}).get('slot')
        self._status('CANCELING', 'Operator requested retrieval cancellation', slot)
        res.success, res.message = True, 'Retrieval cancellation requested'
        return res

    def _raise_if_cancelled(self):
        expected = getattr(self, '_expected_pm_session', None)
        if expected is not None and (self._pm_status or {}).get('session_id') != expected:
            raise RuntimeError('Pallet node restarted or reset during retrieval')
        if self._cancel_event.is_set():
            raise RuntimeError('RETRIEVAL_CANCELED')

    def _lookup_shelf_front(self):
        tf = self._tf_buffer.lookup_transform('map', 'shelf_front', Time(),
                                              timeout=Duration(seconds=1.0))
        t, q = tf.transform.translation, tf.transform.rotation
        return float(t.x), float(t.y), yaw_from_quaternion(q)

    # ------------------------------------------------------------------
    # Nav2 helpers (BT path + proximity test for the handoff)
    # ------------------------------------------------------------------
    def _on_stance_odom(self, msg):
        twist = msg.twist.twist
        speed = (math.hypot(twist.linear.x, twist.linear.y), abs(twist.angular.z))
        now = time.monotonic()
        with self._lock:
            if (now - self._odom_t > 0.5 or not all(math.isfinite(v) for v in speed)
                    or speed[0] > 0.02 or speed[1] > 0.04):
                self._stance_stopped_since = None
            self._odom_speed, self._odom_t = speed, now

    def _on_stance_nav_status(self, msg):
        with self._lock:
            self._nav_goal_active = any(s.status in (1, 2, 3) for s in msg.status_list)
            self._nav_status_t = time.monotonic()
            if self._nav_goal_active:
                self._stance_stopped_since = None

    def _slot_stance(self, slot):
        if slot is None:
            # Keep the centered D_1 leg using the configured anchor distance.
            x, y, yaw = self._lookup_shelf_front()
            distance = self._model.front_distance_m
        else:
            frame = self._model.slot_tf_frame(slot)
            if frame is None:
                raise RuntimeError(f'Invalid retrieval slot: {slot!r}')
            tf = self._tf_buffer.lookup_transform(
                'map', frame, Time(), timeout=Duration(seconds=1.0))
            t, q = tf.transform.translation, tf.transform.rotation
            x, y, yaw = float(t.x), float(t.y), yaw_from_quaternion(q)
            distance = self._model.front_distance_m
        if not all(math.isfinite(v) for v in (x, y, yaw, distance)):
            raise RuntimeError('Invalid rack TF for retrieval stance')
        return x - distance * math.cos(yaw), y - distance * math.sin(yaw), yaw

    def _robot_map_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', str(self.get_parameter('base_frame').value), Time())
            age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
            if age < 0.0 or age > 1.0:
                return None
            t, q = tf.transform.translation, tf.transform.rotation
            pose = float(t.x), float(t.y), yaw_from_quaternion(q)
            return pose if all(math.isfinite(v) for v in pose) else None
        except Exception:
            return None

    def _stance_reached(self, target, radius):
        pose = self._robot_map_pose()
        if pose is None:
            return False
        dyaw = math.atan2(math.sin(pose[2] - target[2]), math.cos(pose[2] - target[2]))
        return (math.hypot(pose[0] - target[0], pose[1] - target[1]) <= radius
                and abs(math.degrees(dyaw)) <= float(self.get_parameter('stance_yaw_tol_deg').value))

    def _wait_stance_stopped(self, target, radius):
        # let the robot come to rest
        started = time.monotonic()
        end = started + float(self.get_parameter('stance_stop_timeout_s').value)
        with self._lock:
            self._stance_stopped_since = None
        while rclpy.ok() and time.monotonic() < end:
            self._raise_if_cancelled()
            now = time.monotonic()
            reached = self._stance_reached(target, radius)
            with self._lock:
                speed = self._odom_speed
                stopped = (self._odom_t >= started and now - self._odom_t <= 0.5
                           # Action status may publish only on state changes;
                           # a terminal status need not arrive after settle starts.
                           and self._nav_status_t > 0.0
                           and self._nav_goal_active is False
                           and self._active_nav_goal is None and self._stance_pending is None
                           and speed is not None and all(math.isfinite(v) for v in speed)
                           and speed[0] <= 0.02 and speed[1] <= 0.04)
                if not stopped or not reached:
                    self._stance_stopped_since = None
                elif self._stance_stopped_since is None:
                    self._stance_stopped_since = now
                elif now - self._stance_stopped_since >= float(self.get_parameter('stance_settle_s').value):
                    return
            time.sleep(0.05)
        raise RuntimeError('Retrieval stance stop/position/yaw confirmation timed out')

    def _release_stance_goal(self, handle, future):
        try:
            terminal = future.result().status in (4, 5, 6)
        except Exception:
            terminal = False
        if terminal:
            with self._lock:
                if self._active_nav_goal is handle:
                    self._active_nav_goal = None

    def _cancel_late_stance_goal(self, future):
        try:
            handle = future.result()
            with self._lock:
                if self._stance_pending is future:
                    self._stance_pending = None
                if handle is not None and handle.accepted:
                    self._active_nav_goal = handle
            if handle is not None and handle.accepted:
                result = handle.get_result_async()
                result.add_done_callback(lambda done: self._release_stance_goal(handle, done))
                handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(f'Late retrieval goal cancellation unconfirmed: {exc}')

    def _stop_stance_navigation(self, handle=None, result=None):
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f'Retrieval Nav2 cancel failed: {exc}')
        # Existing pallet stop publishes drive zero and M0; failures never hand
        # control to align/detector/pick, even if Nav2 cancellation is uncertain.
        try:
            if self._stop_cli.service_is_ready():
                self._stop_cli.call_async(Trigger.Request())
        except Exception as exc:
            self.get_logger().warn(f'Retrieval stop request failed: {exc}')
        if result is not None:
            end = time.monotonic() + float(self.get_parameter('stance_stop_timeout_s').value)
            while rclpy.ok() and not result.done() and time.monotonic() < end:
                time.sleep(0.05)
            if result.done():
                self._release_stance_goal(handle, result)

    def _robot_xy(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', str(self.get_parameter('base_frame').value), Time())
            return float(tf.transform.translation.x), float(tf.transform.translation.y)
        except Exception:
            return None

    def _near(self, gx, gy, radius):
        p = self._robot_xy()
        return p is not None and math.hypot(p[0] - gx, p[1] - gy) <= radius

    def _navigate_to_slot_stance(self, slot):
        """Navigate to D_1's rack-facing pose, with optional lateral column offset.

        shelf_front is derived from D_1 + front_distance_m, so subtracting
        that same configured distance resolves back to D_1. A selected column
        adds only its TF lateral offset; it does not add another standoff.

        Handoff: Nav2 often dithers within a few cm of a tight goal (xy 7 cm,
        yaw 0.04 rad) next to the rack. When the robot has been within
        `d1_handoff_m` of the goal for `d1_handoff_hold_s` the Nav2 goal is
        cancelled and `retrieval_align` (tolerant to 40 cm / 25 deg) takes over.

        Step 1: the selected slot now comes directly from its own map TF at
        1.20 m. The legacy alignment admission is NOT changed here. Every
        successful return also requires terminal Nav2 ownership and odometry.
        Step 2 uses front_distance_m for both this goal and align admission;
        the source shelf configuration remains 1.20 m.
        """
        self._raise_if_cancelled()
        target = self._slot_stance(slot)
        gx, gy, yaw = target
        handoff_m = float(self.get_parameter('d1_handoff_m').value)
        hold_s = float(self.get_parameter('d1_handoff_hold_s').value)
        abort_ok_m = float(self.get_parameter('d1_abort_ok_m').value)
        timeout_s = float(self.get_parameter('d1_nav_timeout_s').value)
        # B1 equals D_1 with the current geometry. Reuse that arrival only
        # after confirming position, yaw, idle Nav2 and a measured stable stop.
        if slot == 'B1' and self._stance_reached(target, max(0.0, handoff_m)):
            self._wait_stance_stopped(target, max(0.0, handoff_m))
            return
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        if not self._nav.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('NavigateToPose server is unavailable')
        self._raise_if_cancelled()
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        future = self._nav.send_goal_async(nav_goal)
        self._stance_pending = future
        try:
            handle = self._wait_future(future, 10.0)
        except Exception:
            future.add_done_callback(self._cancel_late_stance_goal)
            self._stop_stance_navigation()
            raise
        if handle is None:
            future.add_done_callback(self._cancel_late_stance_goal)
            self._stop_stance_navigation()
            self._raise_if_cancelled()
            raise RuntimeError('Retrieval stance goal response timed out')
        self._stance_pending = None
        if not handle or not handle.accepted:
            raise RuntimeError('Nav2 rejected retrieval stance goal')
        self._active_nav_goal = handle

        result_fut = handle.get_result_async()
        result_fut.add_done_callback(lambda done: self._release_stance_goal(handle, done))
        end = time.monotonic() + timeout_s
        near_t0 = None
        handed_off = False
        try:
            while rclpy.ok() and not result_fut.done() and time.monotonic() < end:
                self._raise_if_cancelled()
                if handoff_m > 0.0 and self._stance_reached(target, handoff_m):
                    if near_t0 is None:
                        near_t0 = time.monotonic()
                    elif time.monotonic() - near_t0 >= hold_s:
                        self.get_logger().info(
                            f'Nav2 within {handoff_m * 100:.0f} cm of stance {slot or "D_1"} for '
                            f'{hold_s:.0f}s -> cancel Nav2, retrieval_align takes over')
                        handle.cancel_goal_async()
                        handed_off = True
                        break
                else:
                    near_t0 = None
                time.sleep(0.05)
            self._raise_if_cancelled()
            if not result_fut.done():
                # Cancel on handoff/timeout, then wait for the terminal result.
                if not handed_off:
                    handle.cancel_goal_async()
                cancel_end = time.monotonic() + float(self.get_parameter('stance_stop_timeout_s').value)
                while rclpy.ok() and not result_fut.done() and time.monotonic() < cancel_end:
                    self._raise_if_cancelled()
                    time.sleep(0.05)
                if not result_fut.done():
                    raise RuntimeError('Retrieval Nav2 cancellation was not confirmed')
                handed_off = True  # Includes the old near-goal timeout fallback.
            self._raise_if_cancelled()
            status = result_fut.result().status
            if status == 4:                       # SUCCEEDED
                pass
            elif status == 6 or (status == 5 and handed_off):
                if not self._stance_reached(target, abort_ok_m):
                    raise RuntimeError(f'Nav2 did not reach retrieval stance (status={status})')
            else:
                raise RuntimeError(f'Retrieval stance navigation ended with status={status}')
            self._release_stance_goal(handle, result_fut)
            self._wait_stance_stopped(target, abort_ok_m)
        except Exception:
            self._stop_stance_navigation(handle, result_fut)
            raise

    def _stable_target(self, slot, enabled_at):
        timeout = float(self.get_parameter('candidate_timeout_s').value)
        startup_timeout = float(self.get_parameter('detector_startup_timeout_s').value)
        needed = int(self.get_parameter('stable_frames').value)
        deadline = time.monotonic() + startup_timeout
        previous = None
        stable = 0
        matches = []
        last_seen = 0.0
        saw_any_candidate = False
        saw_nonmatching_candidate = False
        saw_ambiguous_candidate = False
        saw_raw_detection = False
        saw_unlocalized_detection = False
        saw_inference_frame = False
        while time.monotonic() < deadline:
            self._raise_if_cancelled()
            with self._lock:
                msg, seen = self._candidates, self._candidate_t
                pm, pm_t = self._pm_status, self._pm_status_t
                supervisor, supervisor_t = self._supervisor, self._supervisor_t
                detector_status = self._detection_status
                detector_status_t = self._detection_status_t
            if detector_status and detector_status_t >= enabled_at:
                if not saw_inference_frame:
                    saw_inference_frame = True
                    # Do not spend the pallet-search window waiting for the
                    # detector process/model to finish starting. Search gets
                    # its full timeout after the first inference result arrives.
                    deadline = time.monotonic() + timeout
                detected_count = int(detector_status.get('detected_pallets', 0))
                localized_count = int(detector_status.get('localized_pallets', 0))
                saw_raw_detection |= detected_count > 0
                saw_unlocalized_detection |= detected_count > 0 and localized_count == 0
            if (pm and pm_t >= enabled_at and pm.get('state') in ('STOPPED', 'FAILED')):
                raise RuntimeError('Pallet retrieval was stopped by the operator')
            if (supervisor and supervisor_t >= enabled_at and
                    supervisor.get('state') == 'RECOVERY'):
                raise RuntimeError('Mission supervisor entered recovery')
            if msg is None or seen < enabled_at or time.monotonic() - seen > 1.0:
                time.sleep(0.05)
                continue
            if seen <= last_seen:
                time.sleep(0.02)
                continue
            last_seen = seen
            if msg.poses:
                saw_any_candidate = True
            try:
                shelf_pose = self._lookup_shelf_front()
            except Exception:
                time.sleep(0.05)
                continue
            matches = []
            for pose in msg.poses:
                detected = (pose.position.x, pose.position.y, pose.position.z,
                            yaw_from_quaternion(pose.orientation))
                ok, _why = self._model.retrieval_pose_match(slot, detected, shelf_pose)
                if ok:
                    matches.append(detected)
                else:
                    saw_nonmatching_candidate = True
            if len(matches) > 1:
                saw_ambiguous_candidate = True
            if len(matches) == 1:
                candidate = matches[0]
                if previous is not None:
                    distance = math.dist(candidate[:3], previous[:3])
                    dyaw = abs(math.atan2(math.sin(candidate[3] - previous[3]),
                                          math.cos(candidate[3] - previous[3])))
                    stable = stable + 1 if distance <= 0.05 and dyaw <= math.radians(5.0) else 1
                else:
                    stable = 1
                previous = candidate
                if stable >= needed:
                    out = PoseStamped()
                    out.header.frame_id = 'map'
                    out.header.stamp = self.get_clock().now().to_msg()
                    out.pose.position.x, out.pose.position.y, out.pose.position.z = candidate[:3]
                    out.pose.orientation.z = math.sin(candidate[3] / 2.0)
                    out.pose.orientation.w = math.cos(candidate[3] / 2.0)
                    return out
            else:
                # Zero means the selected pallet is not visible; more than one
                # means the slot match is ambiguous. Neither can start a pick.
                previous, stable = None, 0
            time.sleep(0.05)
        timeout = float(self.get_parameter('candidate_timeout_s').value)
        if saw_ambiguous_candidate:
            reason = f'Multiple pallet detections matched {slot}; target is ambiguous'
        elif saw_unlocalized_detection:
            reason = (f'Pallet detections were visible at {slot}, but depth/yaw '
                      'localization produced no valid pose')
        elif saw_nonmatching_candidate:
            reason = (f'No valid pallet matched {slot} within {timeout:.0f}s; '
                      'a pallet may be in another slot/level, or an object may be '
                      'blocking the selected slot')
        elif saw_any_candidate:
            reason = (f'No pallet matched {slot} within {timeout:.0f}s; '
                      'the selected slot may contain a non-pallet object or be occluded')
        elif saw_raw_detection:
            reason = f'Pallet was detected at {slot}, but no pose matched its rack TF'
        elif not saw_inference_frame:
            reason = (f'Detector start was accepted, but no inference frames were received '
                      f'within {startup_timeout:.0f}s; check detector readiness and camera '
                      'topic/QoS compatibility')
        else:
            reason = (f'Detector processed camera frames but found no pallet at {slot} '
                      f'within {timeout:.0f}s; the slot may be empty, occluded, or contain '
                      'a non-pallet object')
        raise RuntimeError(reason)

    def _wait_future(self, future, timeout):
        end = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            if self._cancel_event.is_set():
                break
            time.sleep(0.02)
        if not future.done():
            future.add_done_callback(lambda _: self._stop_for_recovery())
            return None
        return future.result()

    def _call(self, client, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        result = self._wait_future(client.call_async(Trigger.Request()), timeout + 5.0)
        if result is None or not result.success:
            raise RuntimeError(result.message if result else f'Service timed out: {client.srv_name}')
        return result.message

    def _run_route_mission(self, client, name, timeout=180.0):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'Mission service unavailable: {client.srv_name}')
        started_at = time.monotonic()
        started_wall = time.time()
        future = client.call_async(Empty.Request())
        self._route_request_pending = future
        result = self._wait_future(future, 15.0)
        if future.done():
            self._route_request_pending = None
        if result is None:
            raise RuntimeError(f'Mission request timed out: {name}')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._raise_if_cancelled()
            with self._lock:
                status = self._mission_status
                status_t = self._mission_status_t
            if (status and status_t >= started_at and status.get('mission') == name
                    and float(status.get('timestamp', 0.)) >= started_wall):
                state = status.get('state')
                if state == 'SUCCEEDED':
                    return
                if state in ('ABORTED', 'CANCELED', 'FAILED', 'RECOVERY', 'STOPPED'):
                    raise RuntimeError(status.get('reason') or f'{name} {state.lower()}')
            time.sleep(0.1)
        raise RuntimeError(f'Mission {name} did not report success in time')

    def _wait_pm_state(self, states, timeout, after_t=0.0, after_operation_id=None, after_resume_id=0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._raise_if_cancelled()
            with self._lock:
                data = self._pm_status
                status_t = self._pm_status_t
            if (status_t < after_t or (data or {}).get('last_resumed_checkpoint_id', 0) < after_resume_id
                    or (after_operation_id is not None
                    and (data or {}).get('operation_id', -1) <= after_operation_id)):
                time.sleep(0.1)
                continue
            state = data.get('state') if data else None
            if state in states:
                return data
            if state in ('FAILED', 'STOPPED'):
                raise RuntimeError(data.get('msg') or 'Pallet mission failed')
            time.sleep(0.1)
        raise RuntimeError(f'Pallet mission did not reach {states}')

    def _stop_for_recovery(self):
        if self._recovery_pm_session is None:
            self._recovery_pm_session = (self._pm_status or {}).get('session_id')
        self._recovery_since = time.monotonic()
        self._recovery_stop_future = None
        self._manager_stop_request = None
        try:
            if self._manager_stop_cli.service_is_ready():
                self._manager_stop_request = self._manager_stop_cli.call_async(Trigger.Request())
            if self._stop_cli.service_is_ready():
                self._recovery_stop_future = self._stop_cli.call_async(Trigger.Request())
        except Exception as exc:
            self.get_logger().error(f'Retrieval recovery STOP failed: {exc}')
        self._stop_stance_navigation(self._active_nav_goal)

    def _resume_blocked(self):
        now = time.monotonic()
        if self._resume_stage is None:
            return 'No retrieval checkpoint in this session'
        if self._run_lock.locked():
            return 'Retrieval worker has not stopped yet'
        if now - self._recovery_since > 120.:
            return 'Retrieval checkpoint expired; inspect robot'
        if self._stance_pending is not None or self._active_nav_goal is not None:
            return 'Previous retrieval navigation has not ended'
        if getattr(self, '_manager_stop_request', None) is not None and not self._manager_stop_request.done():
            return 'Waiting for mission_manager STOP request acknowledgement'
        if self._route_request_pending is not None and not self._route_request_pending.done():
            return 'Previous route trigger has not ended'
        future = self._recovery_stop_future
        if future is None or not future.done():
            return 'Waiting for pallet STOP response'
        try:
            if not future.result().success:
                return 'Pallet STOP unconfirmed'
        except Exception:
            return 'Pallet STOP failed'
        if ((self._mission_status or {}).get('busy') is not False
                or self._mission_status_t < self._recovery_since):
            return 'Waiting for mission_manager to release pending navigation'
        if self._nav_goal_active is not False or self._nav_status_t <= 0:
            return 'Nav2 has not confirmed idle'
        if (self._supervisor is None or now - self._supervisor_t > 2.
                or self._supervisor.get('state') not in ('IDLE', 'DONE', 'FAILED', 'STOPPED')):
            return 'Supervisor is busy or stale'
        pm = self._pm_status or {}
        if now - self._pm_status_t > 2. or self._pm_status_t < self._recovery_since:
            return 'Waiting for fresh pallet status after STOP'
        if pm.get('session_id') != self._recovery_pm_session:
            return 'Pallet node restarted during retrieval recovery; inspect and reset'
        recovery = pm.get('recovery') or {}
        if recovery.get('stop_pending'):
            return 'Waiting for all pallet STOP requests to finish'
        if recovery.get('checkpoint'):
            if not recovery.get('available'):
                return recovery.get('reason') or 'Pallet checkpoint blocked'
        elif pm.get('state') not in ('IDLE', 'READY', 'CARRYING', 'DONE', 'STOPPED'):
            return 'Pallet state cannot resume'
        if (now - self._odom_t > .5 or self._odom_speed is None
                or not all(math.isfinite(v) for v in self._odom_speed)
                or self._odom_speed[0] > .02 or self._odom_speed[1] > .04):
            return 'Fresh stationary odometry required'
        if not isinstance(pm.get('cargo'), bool):
            return 'Cargo state unknown'
        return ''

    def _srv_resume(self, _req, res):
        reason = self._resume_blocked()
        if reason:
            res.success, res.message = False, reason
            return res
        if not self._run_lock.acquire(blocking=False):
            res.success, res.message = False, 'Retrieval is busy'
            return res
        with self._lock:
            self._resume_revision += 1
            self._resume_in_progress = True
        self._cancel_event.clear()
        self._worker = threading.Thread(target=self._run, args=(self._resume_slot, True), daemon=True)
        self._worker.start()
        res.success, res.message = True, 'Retrieval Resume accepted'
        return res

    def _pm_step(self, client, states, timeout):
        stage = self._resume_stage
        issued = self._issued_stages.get(stage)
        if issued is None:
            if not client.wait_for_service(timeout_sec=1.):
                raise RuntimeError(f'Service unavailable before issuing mechanical step: {client.srv_name}')
            issued = time.monotonic()
            self._issued_stages[stage] = {'time': issued, 'operation_id': (self._pm_status or {}).get('operation_id', -1)}
            result = self._wait_future(client.call_async(Trigger.Request()), 15.)
            if result is not None and not result.success:
                self._issued_stages.pop(stage, None)  # explicit refusal means no accepted step
                raise RuntimeError(result.message)
            if result is None:
                raise RuntimeError('Mechanical request outcome unknown; inspect its checkpoint')
        elif not self._resumed_pm_after:
            pm = self._pm_status or {}
            if (pm.get('state') not in states or self._pm_status_t < issued['time']
                    or pm.get('operation_id', -1) <= issued['operation_id']):
                raise RuntimeError('Interrupted mechanical step has no validated checkpoint')
        record = self._issued_stages[stage]
        return self._wait_pm_state(states, timeout, after_t=max(record['time'], self._resumed_pm_after),
                                   after_operation_id=record['operation_id'], after_resume_id=self._resumed_pm_id)

    def _run_step(self, stage, slot):
        if stage == 0:
            self._status('HOMING_LIFT', 'Homing the lift before retrieval', slot)
            data = self._pm_step(self._lift_home_cli, {'IDLE'}, 160.)
            h = data.get('lift_cm')
            if h is None or not math.isfinite(float(h)) or abs(float(h)) > 1.:
                raise RuntimeError('Lift HOME not confirmed by joint_states')
        elif stage == 1:
            self._status('NAVIGATING_TO_D1', 'Moving to centered D_1', slot, nav_leg='d1')
            self._navigate_to_slot_stance(None)
        elif stage == 2:
            self._status('NAVIGATING_TO_D1', f'Moving to waiting pose for {slot}', slot, nav_leg='slot')
            self._navigate_to_slot_stance(slot)
        elif stage == 3:
            self._status('ALIGNING_TO_SLOT', f'Aligning to {slot}', slot)
            self._pm_step(self._align_cli, {'READY'}, 180.)
            self._wait_stance_stopped(self._slot_stance(slot), float(self.get_parameter('d1_abort_ok_m').value))
        elif stage == 4:
            self._status('LOADING_DETECTOR', 'Loading pallet detector for fresh samples', slot)
            self._call(self._detector_start_cli)
            enabled_at = time.monotonic()
            self._candidate_enable_pub.publish(Bool(data=True))
            self._status('DETECTING', f'Detecting pallet at {slot}', slot)
            try:
                self._saved_target = self._stable_target(slot, enabled_at)
            finally:
                self._candidate_enable_pub.publish(Bool(data=False))
                self._call(self._detector_stop_cli, timeout=2.)
        elif stage == 5:
            if self._saved_target is None:
                raise RuntimeError('No saved retrieval target')
            self._selected_slot_pub.publish(String(data=slot))
            self._selected_pose_pub.publish(self._saved_target)
            time.sleep(.15)
            self._status('PICKING', f'Approaching detected pallet, then retrieving from {slot}', slot)
            self._pm_step(self._pick_cli, {'CARRYING'}, float(self.get_parameter('pick_timeout_s').value))
        elif stage == 6:
            self._status('NAVIGATING_TO_D2', 'Carrying pallet to D_2', slot)
            self._run_route_mission(self._d1_d2_cli, 'd1_to_d2', timeout=180.)
        elif stage == 7:
            self._status('PLACING_AT_D2', 'Lowering pallet at D_2', slot)
            self._pm_step(self._release_cli, {'DONE'}, 180.)
        elif stage == 8:
            self._status('PLACING_AT_D2', 'Backing clear of released pallet', slot)
            self._pm_step(self._backoff_cli, {'DONE'}, 30.)
        elif stage == 9:
            self._status('RETURNING_HOME', 'Returning to HOME', slot)
            self._run_route_mission(self._home_cli, 'return_home', timeout=180.)

    def _run(self, slot, resuming=False):
        try:
            if resuming:
                recovery = (self._pm_status or {}).get('recovery') or {}
                if recovery.get('checkpoint'):
                    self._resumed_pm_after = time.monotonic()
                    self._resumed_pm_id = recovery['checkpoint']['id']
                    self._call(self._resume_cli)
            self._selected_slot_pub.publish(String(data=slot))
            while self._resume_stage is not None and self._resume_stage < 10:
                self._raise_if_cancelled()
                self._run_step(self._resume_stage, slot)
                self._resume_stage += 1
                self._resumed_pm_after = 0.
                self._resumed_pm_id = 0
            self._resume_stage = None
            self._status('DONE', f'Retrieval from {slot} complete', slot)
        except Exception as exc:
            self._candidate_enable_pub.publish(Bool(data=False))
            self._stop_for_recovery()
            self._status('RECOVERY', f'{exc}; operation retained for Resume after inspection', slot, resuming=False)
            self.get_logger().warn(f'Retrieval recovery: {exc}')
        finally:
            self._resume_in_progress = False
            self._run_lock.release()
            self._publish_status_heartbeat()


def main(args=None):
    rclpy.init(args=args)
    node = PalletRetrievalNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
