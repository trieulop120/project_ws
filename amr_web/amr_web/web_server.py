#!/usr/bin/env python3
# amr_web/web_server.py
"""
FastAPI Web Server with WebSocket support.
Phase 1 Fix: asyncio.to_thread for blocking commands, proper error handling.
"""

import os
import json
import asyncio
import traceback
import mimetypes
import time
from contextlib import asynccontextmanager

from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from typing import Optional
from amr_web.websocket_manager import WebSocketManager
from amr_web.state_bridge import StateBridge
from amr_web.navigation_validation import valid_resource_id, map_image_path
# History (HISTORY_RESULTS_PLAN.md §6): log every WS command and connection
# edge. Imports are guarded by try/except at the use site so a missing
# history package (e.g. older install) cannot break the server.
try:
    from amr_web.history import events as _history_events
    _HISTORY_OK = True
except Exception:
    _HISTORY_OK = False

mimetypes.add_type('text/css', '.css')
mimetypes.add_type('application/javascript', '.js')


def _client_ip(websocket: WebSocket) -> Optional[str]:
    """Best-effort client IP for history logging. Never raises."""
    try:
        client = getattr(websocket, "client", None)
        if client is not None:
            host = getattr(client, "host", None)
            if host:
                return str(host)
    except Exception:
        pass
    return None


def _reject_json_constant(value):
    raise ValueError(f'Non-finite JSON number: {value}')


def create_app(state_bridge: StateBridge, ws_manager: WebSocketManager, node_logger=None, bridge_node=None) -> FastAPI:
    """Create FastAPI app with lifespan for loop attachment."""

    @asynccontextmanager
    async def lifespan(app):
        """Attach bridge_node to uvicorn's event loop."""
        if bridge_node and hasattr(bridge_node, 'attach_server_loop'):
            loop = asyncio.get_running_loop()
            bridge_node.attach_server_loop(loop)
        try:
            yield
        finally:
            if bridge_node and hasattr(bridge_node, 'detach_server_loop'):
                bridge_node.detach_server_loop()

    app = FastAPI(title="AMR Web Bridge", lifespan=lifespan)

    pkg_share_dir = get_package_share_directory('amr_web')
    web_dir = os.path.join(pkg_share_dir, 'resources', 'web')

    if node_logger:
        node_logger.info(f"Serving web directory from: {web_dir}")

    @app.get("/health")
    async def health():
        return {"status": "ok", "service": "amr_web", "version": "0.3.0"}

    @app.get("/api/maps")
    def get_maps():
        if bridge_node:
            maps = bridge_node.get_available_maps()
            return {"status": "ok", "maps": maps}
        return {"status": "error", "message": "Bridge node not available"}

    @app.get("/api/map/image/{map_id}")
    def get_map_image(map_id: str):
        if not bridge_node:
            return {"status": "error", "message": "Bridge node not available"}

        import yaml
        import base64

        if not valid_resource_id(map_id):
            return {'status': 'error', 'message': 'Invalid map identifier'}
        maps_dir = os.path.expanduser('~/project_ws/maps')
        yaml_path = os.path.join(maps_dir, f'{map_id}.yaml')

        if not os.path.exists(yaml_path):
            return {"status": "error", "message": f"Map {map_id} not found"}

        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                yaml_data = yaml.safe_load(f)

            if not yaml_data:
                return {"status": "error", "message": "Empty YAML data"}

            pgm_filename = yaml_data.get('image', f'{map_id}.pgm')
            pgm_path = map_image_path(maps_dir, map_id, pgm_filename)
            # PGM permits comments, arbitrary whitespace and P2/P5 encodings.
            # Pillow is already a bridge dependency; use its parser, not newline counting.
            from PIL import Image
            with Image.open(pgm_path) as image:
                image = image.convert('L')
                width, height = image.size
                img_data = image.tobytes()
            origin = yaml_data.get('origin', [0, 0, 0])
            expected_size = width * height
            encoded = base64.b64encode(img_data[:expected_size]).decode('ascii')

            return {
                "status": "ok",
                "info": {
                    "resolution": yaml_data.get('resolution', 0.05),
                    "width": width,
                    "height": height,
                    "frame_id": getattr(bridge_node, '_map_frame', 'map'),
                    "mode": yaml_data.get('mode', 'trinary'),
                    "negate": yaml_data.get('negate', 0),
                    "occupied_thresh": yaml_data.get('occupied_thresh', 0.65),
                    "free_thresh": yaml_data.get('free_thresh', 0.25),
                    "origin": {
                        "x": origin[0],
                        "y": origin[1],
                        "theta": origin[2]
                    }
                },
                "image_data": encoded,
                "is_pgm_raw": True,  # Danh dau day la PGM thuong, can render truc tiep
            }
        except Exception as e:
            if node_logger:
                node_logger.error(f'Map image {map_id} failed: {e}')
            return {"status": "error", "message": "Map image unavailable; check the selected map files"}

    @app.get("/api/routes")
    def get_routes(map_id: str = None):
        if map_id is not None and not valid_resource_id(map_id):
            return {'status': 'error', 'message': 'Invalid map identifier'}
        if bridge_node:
            routes = bridge_node.get_available_routes(map_id)
            return {"status": "ok", "routes": routes}
        return {"status": "error", "message": "Bridge node not available"}

    @app.post("/api/routes")
    async def save_route(payload: dict):
        if not bridge_node:
            return {"status": "error", "message": "Bridge node not available"}

        map_id = payload.get('map_id')
        route_data = payload.get('route')

        if not map_id or not route_data:
            return {"status": "error", "message": "Missing map_id or route payload"}

        success, msg = await asyncio.to_thread(bridge_node.save_route, map_id, route_data)
        # [history] Record route save (plan §6).
        if _HISTORY_OK:
            try:
                route_name = (route_data.get("graph_name")
                              or route_data.get("name") or "")
                _history_events.record_system_event(
                    category="ROUTE",
                    action="route_saved" if success else "route_save_failed",
                    level="info" if success else "warn",
                    detail=f"map={map_id} name={route_name}",
                    params={"map_id": map_id, "route_name": route_name,
                            "result": "success" if success else "failed"},
                    result="success" if success else "failed",
                )
            except Exception:
                pass
        if success:
            return {"status": "ok", "message": msg}
        return {"status": "error", "message": msg}

    @app.delete("/api/maps/{map_id}")
    async def delete_map(map_id: str):
        if not bridge_node:
            return {"status": "error", "message": "Bridge node not available"}

        success, msg = await asyncio.to_thread(bridge_node.delete_map_and_routes, map_id)
        # [history] Record map delete (plan §6).
        if _HISTORY_OK:
            try:
                _history_events.record_system_event(
                    category="MAP",
                    action="map_deleted" if success else "map_delete_failed",
                    level="info" if success else "warn",
                    detail=f"map={map_id}",
                    params={"map_id": map_id,
                            "result": "success" if success else "failed"},
                    result="success" if success else "failed",
                )
            except Exception:
                pass
        if success:
            return {"status": "ok", "message": msg}
        return {"status": "error", "message": msg}

    async def handle_command(cmd: dict, websocket: WebSocket) -> dict:
        command = cmd.get('command', '')
        if not isinstance(command, str):
            return {'type': 'command_response', 'request_id': cmd.get('request_id'),
                    'status': 'failed', 'message': 'Command must be a string'}
        result = {"type": "command_response", "command": command}
        if cmd.get('request_id') is not None:
            result['request_id'] = cmd['request_id']

        # [history] Record incoming command (plan §6). Soft-fail: any error
        # is swallowed inside the helper; we never raise from here.
        client_ip = _client_ip(websocket) if _HISTORY_OK else None
        history_event_id = (
            _history_events.record_command(cmd, client_ip=client_ip)
            if _HISTORY_OK else None
        )
        if command == "teleop" and _HISTORY_OK:
            try:
                _history_events.record_teleop_tick(
                    client_ip,
                    float(cmd.get("linear", 0.0)),
                    float(cmd.get("angular", 0.0)),
                )
            except Exception:
                pass

        # P1-1 Fix: Run blocking commands in thread pool
        if command == "start_slam":
            if bridge_node:
                if bridge_node.is_navigation_running():
                    result["status"] = "failed"
                    result["message"] = "Navigation is running – stop it first"
                else:
                    success, message = await asyncio.to_thread(bridge_node.web_control_command, command)
                    result["status"] = "started" if success else "failed"
                    result["message"] = message
                    await ws_manager.broadcast(json.dumps({
                        "type": "slam_state",
                        "is_slam_active": bridge_node.get_slam_state()
                    }))
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command == "stop_slam":
            if bridge_node:
                await asyncio.to_thread(bridge_node.stop_slam)
                result["status"] = "stopped"
                result["message"] = "SLAM stopped"
                await ws_manager.broadcast(json.dumps({
                    "type": "slam_state",
                    "is_slam_active": False
                }))
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command == "save_map":
            map_name = cmd.get('map_name', 'map_01')
            if node_logger:
                node_logger.info(f'save_map received: map_name={map_name}')
            if bridge_node:
                success, path = await asyncio.to_thread(bridge_node.save_map, map_name)
                if node_logger:
                    node_logger.info(f'save_map result: success={success}, path={path}')
                result["status"] = "success" if success else "failed"
                result["path"] = path
                result["message"] = f"Map saved to {path}" if success else "Save failed"
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command in ["get_map", "request_map"]:
            if state_bridge:
                latest_map = state_bridge.get_latest_map()
                if latest_map:
                    # P1-9 Fix: Use ws_manager.send instead of direct send_text
                    await ws_manager.send(websocket, json.dumps(latest_map))
                    result["status"] = "found"
                    result["message"] = "Map sent"
                else:
                    result["status"] = "not_found"
                    result["message"] = "No map available"
            else:
                result["status"] = "error"
                result["message"] = "State bridge not available"

        # Navigation Commands (P1-1: run blocking calls in thread)
        elif command == "start_navigation":
            map_id = cmd.get('map_id', '')
            route_id = cmd.get('route_id')
            if bridge_node:
                success, msg = await asyncio.to_thread(
                    bridge_node.web_control_command, command, map_id, route_id)
                result["status"] = "started" if success else "failed"
                result["message"] = msg
                result['nav_state'] = bridge_node.get_nav_state()
                # Always broadcast nav_state after start
                if success:
                    await ws_manager.broadcast(json.dumps(bridge_node.get_nav_state()))
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command == "stop_navigation":
            if bridge_node:
                success, msg = await asyncio.to_thread(bridge_node.stop_navigation)
                result["status"] = "stopped" if success else "failed"
                result["message"] = msg
                result['nav_state'] = bridge_node.get_nav_state()
                # Broadcast full navigation state
                await ws_manager.broadcast(json.dumps(bridge_node.get_nav_state()))
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command == "nav_goal":
            pose = cmd.get('pose', {})
            route_id = cmd.get('route_id')
            if bridge_node:
                success, msg = await asyncio.to_thread(
                    bridge_node.web_control_command, command, pose, route_id, client_ip=client_ip)
                result["status"] = "sent" if success else "failed"
                result["message"] = msg
                result['nav_state'] = bridge_node.get_nav_state()
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command == "nav_cancel":
            if bridge_node:
                success, msg = await asyncio.to_thread(bridge_node.cancel_nav_goal)
                result["status"] = "canceled" if success else "failed"
                result["message"] = msg
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command == "set_initial_pose":
            pose = cmd.get('pose', {})
            if bridge_node:
                success, msg = await asyncio.to_thread(bridge_node.web_control_command, command, pose)
                result["status"] = "ok" if success else "failed"
                result["message"] = msg
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command == "get_nav_state":
            if bridge_node:
                nav_state = bridge_node.get_nav_state()
                result["status"] = "ok"
                result["nav_state"] = nav_state
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command == "waypoint_mission":
            waypoints = cmd.get('waypoints', [])
            mission_type = cmd.get('mission_type', 'waypoints')
            if bridge_node:
                success, msg = await asyncio.to_thread(
                    bridge_node.web_control_command, command, waypoints, mission_type, client_ip=client_ip)
                result["status"] = "started" if success else "failed"
                result["message"] = msg
                result['nav_state'] = bridge_node.get_nav_state()
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command == "emergency_stop":
            if bridge_node:
                success, msg = await asyncio.to_thread(bridge_node.emergency_stop_robot)
                result["status"] = "ok" if success else "failed"
                result["message"] = msg
                # P1-4 Fix: Broadcast full nav_state instead of manual dict
                await ws_manager.broadcast(json.dumps(bridge_node.get_nav_state()))
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        # Vấn đề 3 Fix: Xử lý lệnh set_op_mode để đồng bộ trạng thái AUTO/MANUAL
        elif command == "set_op_mode":
            mode = cmd.get('mode', 'MANUAL')
            if bridge_node and bridge_node._nav_bridge:
                success, message = await asyncio.to_thread(
                    bridge_node.web_control_command, command, mode)
                result["status"] = "ok" if success else "error"
                result["mode"] = mode
                result["message"] = message
            else:
                result["status"] = "error"
                result["message"] = "NavBridge not available"

        elif command in ("mission_cycle_start", "mission_cycle_stop",
                         "mission_cycle_resume", "mission_cycle_cancel"):
            if bridge_node:
                run_mode = cmd.get('run_mode', 'auto')
                success, message = await asyncio.to_thread(
                    bridge_node.mission_cycle_command, command, run_mode)
                result['status'] = 'started' if success and command == 'mission_cycle_start' else (
                    'ok' if success else 'error')
                if not success and node_logger:
                    node_logger.warning(f'Auto command {command}: {message}')
                user_errors = {'mission_cycle_start': 'Cannot start mission; check readiness',
                    'mission_cycle_stop': 'Stop not confirmed; check the robot',
                    'mission_cycle_cancel': 'Cancellation not confirmed; check the robot',
                    'mission_cycle_resume': 'Resume unavailable; check the robot'}
                result['message'] = message if success else user_errors[command]
                result['mission_supervisor'] = bridge_node.get_mission_supervisor_status()
                result['run_mode'] = run_mode if command == 'mission_cycle_start' else None
            else:
                result['status'] = 'error'
                result['message'] = 'Bridge node not available'

        elif command == "pallet_retrieval_start":
            if bridge_node:
                slot = cmd.get('slot', '')
                success, message = await asyncio.to_thread(
                    bridge_node.pallet_retrieval_start, slot)
                result['status'] = 'started' if success else 'error'
                result['message'] = message
                result['slot'] = slot
            else:
                result['status'] = 'error'
                result['message'] = 'Bridge node not available'

        elif command == "pallet_retrieval_cancel":
            if bridge_node:
                success, message = await asyncio.to_thread(
                    bridge_node.pallet_retrieval_cancel)
                result['status'] = 'canceled' if success else 'error'
                result['message'] = message
            else:
                result['status'] = 'error'
                result['message'] = 'Bridge node not available'

        elif command == "confirm_forks_empty":
            if bridge_node:
                success, message = await asyncio.to_thread(
                    bridge_node.confirm_forks_empty, cmd.get('confirmed_empty') is True)
                result['status'] = 'ok' if success else 'error'
                result['message'] = message
            else:
                result['status'] = 'error'
                result['message'] = 'Bridge node not available'

        elif command == "start_camera":
            mode = cmd.get('mode', 'rgb')
            if bridge_node:
                bridge_node.set_camera_stream(True, mode)
                result["status"] = "ok"
                result["mode"] = mode
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command == "stop_camera":
            if bridge_node:
                bridge_node.set_camera_stream(False, 'off')
                result["status"] = "ok"
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command == "teleop":
            linear = cmd.get('linear', 0.0)
            angular = cmd.get('angular', 0.0)
            if bridge_node:
                success, msg = await asyncio.to_thread(
                    bridge_node.web_control_command, command, linear, angular)
                result["status"] = "ok" if success else "error"
                result["message"] = msg
                result["linear"] = linear
                result["angular"] = angular
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command == "lift":
            action = cmd.get('action', '')
            target_mm = cmd.get('target_mm', 0)
            if bridge_node:
                success, msg = await asyncio.to_thread(
                    bridge_node.web_control_command, command, action, target_mm)
                result["status"] = "ok" if success else "error"
                result["message"] = msg
                result["action"] = action
                result["target_mm"] = target_mm
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command.startswith("llm_"):
            llm = getattr(bridge_node, "llm", None) if bridge_node else None
            if llm:
                result.update(llm.handle(cmd))
            else:
                result["status"] = "error"
                result["message"] = "LLM bridge not available"

        else:
            result["status"] = "unknown"
            result["message"] = f"Unknown command: {command}"

        # [history] Update the recorded 'requested' row with the final
        # success/failed status. Non-blocking; ignored on any error.
        if _HISTORY_OK and history_event_id is not None:
            try:
                _history_events.update_command_result(
                    history_event_id,
                    success=str(result.get("status", "")) in (
                        "ok", "success", "started", "sent", "canceled", "stopped", "found"),
                    message=result.get("message"),
                )
            except Exception:
                pass

        return result

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()
        ws_manager.add(websocket)

        # [history] Record WS connect (plan §6).
        if _HISTORY_OK:
            try:
                _history_events.record_ws_connect(_client_ip(websocket))
            except Exception:
                pass

        if node_logger:
            node_logger.info(f'Client connected. Total: {ws_manager.connection_count}')

        try:
            # P1-13 Fix: Send initial state using ws_manager.send
            for _, data in state_bridge.get_all_state().items():
                await ws_manager.send(websocket, json.dumps(data))

            initial_system = state_bridge.get_system_overview_state()
            if bridge_node:
                initial_system.update(bridge_node.get_web_mission_status())
            await ws_manager.send(websocket, json.dumps(initial_system))

            if bridge_node:
                await ws_manager.send(websocket, json.dumps({
                    "type": "slam_state",
                    "is_slam_active": bridge_node.get_slam_state()
                }))
                await ws_manager.send(websocket, json.dumps(bridge_node.get_nav_state()))
                _llm = getattr(bridge_node, "llm", None)
                if _llm and _llm.last_status is not None:
                    await ws_manager.send(websocket, json.dumps(
                        {"type": "llm_status", "data": _llm.last_status}))

            # Keep command order for normal work. Safety requests bypass that
            # queue so a slow service request on this socket cannot delay Stop.
            tasks = set()
            previous = None
            closed = False
            requests = {}
            interrupt_tasks = {}
            command_generation = 0
            interrupt_commands = {'nav_cancel', 'emergency_stop', 'stop_navigation',
                                  'mission_cycle_stop', 'mission_cycle_cancel', 'pallet_retrieval_cancel', 'llm_stop'}

            motion_commands = {'nav_goal', 'waypoint_mission', 'start_navigation', 'set_initial_pose',
                               'set_op_mode', 'mission_cycle_start', 'mission_cycle_resume',
                               'pallet_retrieval_start', 'teleop', 'lift'}

            async def execute(cmd, predecessor, generation):
                if predecessor is not None:
                    await predecessor
                if closed:
                    return
                request_id = cmd.get('request_id')
                fingerprint = json.dumps(cmd, sort_keys=True)
                if request_id is not None:
                    if not isinstance(request_id, str) or len(request_id) > 200:
                        await ws_manager.send(websocket, json.dumps({'type': 'command_response',
                            'command': cmd.get('command'), 'status': 'failed', 'message': 'Invalid request identifier'}))
                        return
                    existing = requests.get(request_id)
                    if existing:
                        if existing[0] != fingerprint:
                            response = {'type': 'command_response', 'command': cmd.get('command'),
                                'request_id': request_id, 'status': 'failed', 'message': 'Request identifier already used'}
                        else:
                            response = await asyncio.shield(existing[1])
                        await ws_manager.send(websocket, json.dumps(response))
                        return
                    completion = asyncio.get_running_loop().create_future()
                    requests[request_id] = (fingerprint, completion)
                try:
                    if cmd['command'] in motion_commands and generation != command_generation:
                        response = {'type': 'command_response', 'command': cmd['command'],
                            'request_id': request_id, 'status': 'failed', 'message': 'Command superseded by Stop'}
                    else:
                        response = await handle_command(cmd, websocket)
                except Exception as e:
                    if node_logger:
                        node_logger.error(f"Command '{cmd.get('command')}' request={request_id} failed: {e}\n{traceback.format_exc()}")
                    response = {'type': 'command_response', 'command': cmd.get('command'),
                        'request_id': request_id, 'status': 'error',
                        'message': 'Command outcome unconfirmed. Check robot state before retrying; see bridge logs.'}
                if request_id is not None:
                    completion.set_result(response)
                    for key in list(requests):
                        if len(requests) <= 128:
                            break
                        if requests[key][1].done():
                            del requests[key]
                await ws_manager.send(websocket, json.dumps(response))

            def completed(task):
                tasks.discard(task)
                if not task.cancelled() and task.exception() is not None and node_logger:
                    node_logger.error(f'Web command task failed: {task.exception()}')

            try:
                while True:
                    raw = await websocket.receive_text()
                    try:
                        cmd = json.loads(raw, parse_constant=_reject_json_constant)
                    except (json.JSONDecodeError, ValueError):
                        await ws_manager.send(websocket, json.dumps({'type': 'command_response',
                            'status': 'failed', 'message': 'Invalid command JSON'}))
                        continue
                    if not isinstance(cmd, dict) or not isinstance(cmd.get('command'), str):
                        await ws_manager.send(websocket, json.dumps({'type': 'command_response',
                            'status': 'failed', 'message': 'Command must be a string'}))
                        continue
                    interrupt = cmd['command'] in interrupt_commands
                    if (interrupt and cmd['command'] in interrupt_tasks and
                            not interrupt_tasks[cmd['command']].done()):
                        await ws_manager.send(websocket, json.dumps({'type': 'command_response',
                            'command': cmd['command'], 'request_id': cmd.get('request_id'),
                            'status': 'failed', 'message': 'Stop request already pending; await confirmation'}))
                        continue
                    if len(tasks) >= 32 and not interrupt:
                        await ws_manager.send(websocket, json.dumps({'type': 'command_response',
                            'command': cmd['command'], 'request_id': cmd.get('request_id'),
                            'status': 'failed', 'message': 'Too many pending commands; wait for acknowledgement'}))
                        continue
                    if interrupt:
                        command_generation += 1
                    task = asyncio.create_task(execute(cmd, None if interrupt else previous, command_generation))
                    tasks.add(task)
                    task.add_done_callback(completed)
                    if interrupt:
                        interrupt_tasks[cmd['command']] = task
                    if not interrupt:
                        previous = task
            except WebSocketDisconnect:
                pass
            finally:
                closed = True
                # Do not cancel to_thread work: it may already be affecting ROS.
                # Queued work skips execution once disconnected.
                if tasks:
                    outcomes = await asyncio.gather(*tasks, return_exceptions=True)
                    for outcome in outcomes:
                        if isinstance(outcome, Exception) and node_logger:
                            node_logger.error(f'Command task failed: {outcome}')

        except Exception as e:
            if node_logger:
                node_logger.error(f'WS endpoint error: {e}')
        finally:
            ws_manager.remove(websocket)
            # [history] Record WS disconnect + close any open teleop session.
            if _HISTORY_OK:
                try:
                    ip = _client_ip(websocket)
                    _history_events.end_teleop_session(ip)
                    _history_events.record_ws_disconnect(ip)
                except Exception:
                    pass
            if node_logger:
                node_logger.info('Client disconnected')

    # [history] REST API for the History page. Must be included BEFORE the "/" static
    # mount below, otherwise the mount would swallow /api/history/*. Soft-fail.
    if _HISTORY_OK:
        try:
            from amr_web.history import api as _history_api
            app.include_router(_history_api.router)
        except Exception as e:
            if node_logger:
                node_logger.warn(f'History API disabled: {e}')

    if os.path.exists(web_dir):
        app.mount("/", StaticFiles(directory=web_dir, html=True), name="static")

    return app
