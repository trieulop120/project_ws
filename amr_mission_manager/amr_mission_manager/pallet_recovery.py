"""Session-local recovery for the existing pallet state machine (no ROS I/O types).

The pallet node remains the sole owner of controller, fork and backoff state.
STOP freezes that state; resuming never creates a replacement controller.
"""
import math
import time


def finite_pose(pose):
    return pose is not None and len(pose) == 3 and all(math.isfinite(v) for v in pose)


def pose_error(a, b):
    return (math.hypot(a[0] - b[0], a[1] - b[1]),
            abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2]))))


def navigation_stopped(active, stamp, stopped_at, cancel_future=None):
    """Action status is event based; idle is not a periodic sensor heartbeat."""
    if active is True:
        return False
    if active is False and stamp >= stopped_at:
        return True
    if cancel_future is not None and cancel_future.done():
        try:
            result = cancel_future.result()
            return result.return_code == 0 and not result.goals_canceling
        except Exception:
            return False
    return False


class PalletRecovery:
    RECOVERABLE = {'DETECTING', 'READY', 'NAVIGATING', 'SETTLING', 'ALIGNING',
                   'AUTO_RECOVERING', 'WAITING_FOR_SENSOR', 'INSERTING', 'LIFTING',
                   'LOWERING', 'BACKING', 'HOMING', 'CARRYING', 'DONE'}

    def _stop_confirmation_blocked(self):
        """Measured stop admission, independent of checkpoint resumability."""
        if self._state not in ('STOPPED', 'FAILED', 'IDLE', 'CARRYING', 'DONE'):
            return 'Pallet operation is still active'
        if (getattr(self, '_nav_pending', None) is not None or
                any(not f.done() for f in getattr(self, '_nav_results', [])) or
                any(not f.done() for f in getattr(self, '_stop_request_futures', []))):
            return 'Pallet navigation or STOP requests are pending'
        stopped_at = (getattr(self, '_checkpoint', None) or {}).get('time', 0.)
        if (self._manager_busy is not False or not navigation_stopped(
                self._retrieval_nav_active, self._retrieval_nav_t, stopped_at,
                getattr(self, '_nav_cancel_future', None))):
            return 'Navigation ownership is not released'
        now = time.monotonic()
        speed = self._drop_odom_speed
        if (now - self._drop_odom_t > self.p['drop_odom_stale_s'] or speed is None or
                not all(math.isfinite(v) for v in speed) or
                speed[0] > self.p['drop_stop_linear_mps'] or speed[1] > self.p['drop_stop_angular_rps'] or
                self._resume_still_since is None or now - self._resume_still_since < self.p['settle_s']):
            return 'Waiting for stationary odometry'
        if (self._lift_height_now() is None or self._lift_still_since is None or
                now - self._lift_still_since < self.p['lift_settle_s']):
            return 'Waiting for stable lift feedback'
        return ''

    def _srv_discard_recovery(self, _req, res):
        reason = self._stop_confirmation_blocked()
        if reason:
            res.success, res.message = False, reason
            return res
        # Only discard operation logic. Cargo, joint/odom/range samples, lift
        # position and commands are deliberately left with their ROS producers.
        self._nav_epoch += 1
        self._checkpoint = None
        self._resume_operation_id = None
        self._drop_nav = None
        self._goal_handle = None
        self._set_state('CARRYING' if self._cargo is True else 'IDLE', 'Operation checkpoint discarded')
        res.success, res.message = True, 'Operation discarded; physical observations preserved'
        return res

    def _capture_recovery(self):
        if getattr(self, '_checkpoint', None) is not None or self._state not in self.RECOVERABLE:
            return
        self._checkpoint_serial = getattr(self, '_checkpoint_serial', 0) + 1
        self._checkpoint = {
            'id': self._checkpoint_serial, 'state': self._state,
            'operation_id': self._operation_id,
            'phase': self._ctl.phase, 'time': time.monotonic(),
            'pose': self._robot_pose(self.p['insertion_frame']),
            'lift_cm': self._lift_height_now(), 'cargo': self._cargo,
            'route': getattr(self, '_route_path_loaded', None),
            'alignment_retries': self._ctl.alignment_retries,
            'gate_retries': self._ctl.gate_retries,
            'sidesteps': self._ctl.sidesteps,
            'sidestep_distance_m': self._ctl.sidestep_distance_m,
            'controller_error': tuple(self._ctl.err),
            'alignment_error_before_retry': self._ctl._alignment_error_before_retry,
            'backoff_reference': tuple(getattr(self, '_bo_ref', (0.0, 0.0))),
            'backoff_distance_m': float(getattr(self, '_bo_dist', 0.0)),
            'insertion_only': self._ctl.insertion_only,
        }
        self._resume_still_since = None
        self._lift_still_since = None
        self._nav_cancel_future = None  # an earlier operation's cancel cannot prove this STOP

    def _resume_drop_navigation(self):
        if (not self._drop or not finite_pose(self._drop.get('target_odom'))
                or self._pre is None or not self._nav.server_is_ready()):
            return False, 'Missing DROP staging context or Nav2 server'
        if self._ctl.insertion_only or self._drop_lifted:
            return False, 'Raised forks cannot restart DROP navigation'
        if (self._nav_pending is not None or any(not f.done() for f in self._nav_results)
                or not self._retrieval_navigation_idle()):
            return False, 'Previous navigation ownership has not been released'
        # Re-send the same staging goal, retaining the original DROP/controller.
        ctx = {'started': time.monotonic(), 'handoff': False,
               'cancel_at': None, 'stopped_since': None}
        self._drop_nav = ctx
        self._handoff_t0 = None
        self._set_state('NAVIGATING', 'Resuming saved DROP staging goal')
        try:
            future = self._nav.send_goal_async(self._staging_goal())
            self._nav_pending = future
            future.add_done_callback(lambda future: self._on_drop_goal_sent(future, ctx))
        except Exception as exc:
            return False, str(exc)
        return True, 'DROP staging resumed'

    def _recovery_blocked(self):
        cp = getattr(self, '_checkpoint', None)
        if cp is None:
            return 'No operation checkpoint in this node session'
        if self._state not in ('STOPPED', 'FAILED'):
            return 'Pallet handling is not stopped'
        if cp.get('invalidated'):
            return cp['invalidated']
        now = time.monotonic()
        if now - cp['time'] > float(self.p['resume_max_pause_s']):
            return 'Checkpoint expired; inspect robot before restarting the operation'
        if self.pub_cmd.get_subscription_count() == 0 or self.pub_esp32.get_subscription_count() == 0:
            return 'Drive or lift command subscriber is unavailable'
        if getattr(self, '_nav_pending', None) is not None:
            return 'Waiting for the previous Nav2 goal response'
        if any(not future.done() for future in getattr(self, '_stop_request_futures', [])):
            return 'Waiting for all STOP service requests to finish'
        if any(not future.done() for future in getattr(self, '_nav_results', [])):
            return 'Waiting for the previous Nav2 goal to end'
        if self._manager_busy is not False or self._manager_status_t < cp['time']:
            return 'Waiting for mission_manager to release pending navigation'
        if not navigation_stopped(self._retrieval_nav_active, self._retrieval_nav_t,
                                  cp['time'], getattr(self, '_nav_cancel_future', None)):
            return 'Nav2 has not confirmed idle'
        speed = self._drop_odom_speed
        if (now - self._drop_odom_t > float(self.p['drop_odom_stale_s'])
                or speed is None or not all(math.isfinite(v) for v in speed)
                or speed[0] > self.p['drop_stop_linear_mps']
                or speed[1] > self.p['drop_stop_angular_rps']
                or getattr(self, '_resume_still_since', None) is None
                or now - self._resume_still_since < float(self.p['settle_s'])):
            return 'Waiting for fresh odometry and a stationary robot'
        lift = self._lift_height_now()
        if (lift is None or getattr(self, '_lift_still_since', None) is None
                or now - self._lift_still_since < float(self.p['lift_settle_s'])):
            return 'Waiting for fresh, stable lift joint_states'
        if self._cargo != cp['cargo']:
            return 'Cargo changed since STOP; inspect the pallet'
        if cp['route'] != getattr(self, '_route_path_loaded', None):
            return 'Route changed since STOP'
        pose = self._robot_pose(self.p['insertion_frame'])
        if not finite_pose(pose) or not finite_pose(cp['pose']):
            return 'Missing odom-frame pose at STOP or Resume'
        distance, yaw = pose_error(pose, cp['pose'])
        if distance > self.p['resume_pose_tol_m'] or yaw > math.radians(self.p['resume_yaw_tol_deg']):
            return 'Robot moved or odometry changed since STOP; checkpoint cannot be reused'
        if cp['state'] not in ('LIFTING', 'LOWERING'):
            if cp['lift_cm'] is None or abs(lift - cp['lift_cm']) > float(self.p['lift_tol_cm']):
                return 'Fork height changed outside the interrupted lift operation'
        if cp['state'] in ('READY', 'NAVIGATING', 'SETTLING') and self._mode == 'pick':
            if self._result is None or self._pallet_ins is None or self._pre is None:
                return 'No locked pallet pose to resume'
            current = self._map_to_ins(self._result['x'], self._result['y'], self._result['yaw'])
            if not finite_pose(current):
                return 'Missing map to odom transform'
            distance, yaw = pose_error(current, self._pallet_ins)
            if distance > self.p['max_tf_jump_m'] or yaw > math.radians(self.p['max_tf_jump_deg']):
                return 'Locked pallet pose no longer agrees with localization; detect again'
        if cp['state'] in ('ALIGNING', 'INSERTING', 'AUTO_RECOVERING', 'WAITING_FOR_SENSOR'):
            if self._ctl.t_start is not None and cp['time'] - self._ctl.t_start >= self._ctl.p['total_timeout_s']:
                return 'Approach active-time budget exhausted; inspect before restarting'
            if not finite_pose(self._line):
                return 'Missing frozen insertion line'
            if self._ctl.insertion_only and self._ctl.phase in self._ctl.ALIGN_PHASES:
                return 'Raised forks cannot resume a rotation or side-step'
            if self._ctl.phase in ('GATE', 'LOCK') and now - self._us_t > self.p['range_stale_s']:
                return 'Fresh ultrasonic readings are required at the insertion gate'
            if (self._mode == 'drop' and self._ctl.insertion_only
                    and self._ctl.phase == 'GATE'):
                live = self._drop_live_alignment_error()
                if live is None:
                    return 'DROP GATE Resume requires a fresh robot pose and frozen rack line'
                _pose, (lat, yaw, _a) = live
                if (abs(lat) * 100.0 > float(self.p['gate_lat_cm'])
                        or abs(math.degrees(yaw)) > float(self.p['gate_yaw_deg'])):
                    return ('DROP GATE remains outside tolerance with raised forks; '
                            'no sensor confirms pallet/fork clearance for retreat, keep STOP and inspect')
        if cp['state'] in ('LIFTING', 'LOWERING') and cp['time'] - self._lift_t0 >= self.p['lift_timeout_s']:
            return 'Lift active-time budget exhausted; inspect mechanism'
        if cp['state'] == 'BACKING' and cp['time'] - self._bo_t0 >= self._bo_timeout:
            return 'Backoff active-time budget exhausted; inspect robot'
        return ''

    def _recovery_status(self):
        cp = getattr(self, '_checkpoint', None)
        if cp is None:
            return {'available': False, 'reason': 'No operation checkpoint', 'checkpoint': None,
                    'stop_confirmed': not self._stop_confirmation_blocked(),
                    'stop_pending': any(not f.done() for f in getattr(self, '_stop_request_futures', []))}
        reason = self._recovery_blocked()
        return {'available': not reason, 'reason': reason,
                'stop_confirmed': not self._stop_confirmation_blocked(),
                'stop_pending': any(not f.done() for f in getattr(self, '_stop_request_futures', [])),
                'checkpoint': {'id': cp['id'], 'state': cp['state'], 'phase': cp['phase'],
                               'operation_id': cp['operation_id'],
                               'lift_target_cm': self._lift_goal if cp['state'] in ('LIFTING', 'LOWERING') else None,
                               'lift_next': self._lift_next, 'insertion_only': self._ctl.insertion_only,
                               'sidesteps': self._ctl.sidesteps,
                               'sidestep_distance_m': self._ctl.sidestep_distance_m,
                               'alignment_retries': self._ctl.alignment_retries,
                               'gate_retries': self._ctl.gate_retries}}

    def _srv_resume(self, _req, res):
        reason = self._recovery_blocked()
        if reason:
            res.success, res.message = False, reason
            return res
        cp = self._checkpoint
        now = time.monotonic()
        pause = now - cp['time']
        resumed_drop_gate = None
        if (cp['state'] in ('ALIGNING', 'INSERTING', 'AUTO_RECOVERING', 'WAITING_FOR_SENSOR')
                and self._mode == 'drop'
                and self._ctl.insertion_only and self._ctl.phase == 'GATE'):
            # _recovery_blocked already checked this current pose against GATE.
            resumed_drop_gate = self._drop_live_alignment_error()
        # Retain elapsed ACTIVE time and all geometric/controller budgets.
        if cp['state'] in ('ALIGNING', 'INSERTING', 'AUTO_RECOVERING',
                           'WAITING_FOR_SENSOR', 'LIFTING', 'LOWERING', 'BACKING'):
            for key in ('t_start', 't_phase', '_wait_t0', '_pulse_until'):
                value = getattr(self._ctl, key, None)
                if value is not None:
                    setattr(self._ctl, key, value + pause)
            self._ctl._wait_t0 = None
            self._ctl._pulse_until = None  # never resume a stale timed turn pulse
            for attr in ('_tf_wait_since', '_range_wait_since'):
                value = getattr(self, attr, None)
                if value is not None:
                    setattr(self, attr, value + pause)
        self._phase_shown = None
        self._us_count = 0  # confirmation must use samples from the resumed operation
        self._resume_operation_id = cp['operation_id']
        self._checkpoint = None
        state = cp['state']
        if state == 'DETECTING':
            ok, why = self._start_detect()
            if not ok:
                self._checkpoint = cp
                self._resume_operation_id = None
                res.success, res.message = False, why
                return res
        elif state in ('LIFTING', 'LOWERING'):
            self._lift_t0 += pause
            self._lift_settle_t0 = None
            # A cached pre-Resume measurement must not complete the operation.
            self._lift_resume_after = self._lift_t
            self._set_state(state, f'Resuming lift to {self._lift_goal:g} cm')
            tol = self.p['lift_home_tol_cm'] if self._lift_is_home else self.p['lift_tol_cm']
            if abs(self._lift_height_now() - self._lift_goal) > tol:
                self._esp32(self.p['lift_home_cmd'] if self._lift_is_home else f'H{self._lift_goal:g}')
        elif state == 'BACKING':
            self._bo_t0 += pause  # original reference and total distance are retained
            self._set_state(state, 'Resuming remaining backoff distance')
        elif state in ('READY', 'NAVIGATING', 'SETTLING'):
            # NAVIGATING/SETTLING can safely return to the stored staging goal.
            # DROP uses its existing staging admission on the explicit replay.
            if self._mode == 'drop':
                ok, why = self._resume_drop_navigation()
            elif self._retrieval_mode:
                ok, why = self._start_retrieval_pick_navigation()
            else:
                self._set_state('READY', 'Locked pallet pose retained; resuming staging')
                ok, why = (True, 'Locked pallet pose ready') if state == 'READY' else self._go_to_pallet()
            if not ok:
                self._checkpoint = cp
                self._set_state('STOPPED', why)
                self._resume_operation_id = None
                res.success, res.message = False, why
                return res
        elif state == 'HOMING':
            self._begin_home()
            if self._state != 'HOMING':
                self._checkpoint = cp
                self._resume_operation_id = None
                res.success, res.message = False, 'Could not resume HOME navigation'
                return res
        elif resumed_drop_gate is not None:
            _pose, (_lat, _yaw, a) = resumed_drop_gate
            self._set_state('INSERTING', 'Resume rechecked current DROP GATE within tolerance; continuing LOCK')
            self.get_logger().info(
                f'[DROP_GATE_RESUME] revalidated: lat={_lat*100:+.3f} cm, '
                f'yaw={math.degrees(_yaw):+.3f} deg, a={a:.3f} m; '
                'GATE controller will recheck before LOCK; no retreat required')
        else:
            self._set_state(state, f'Resuming {state} / {self._ctl.phase}')
        self._resume_operation_id = None
        self._last_resumed_checkpoint_id = cp['id']
        res.success, res.message = True, f'Resumed {state} from operation checkpoint {cp["id"]}'
        return res
