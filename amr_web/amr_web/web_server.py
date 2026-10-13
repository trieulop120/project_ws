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

from amr_web.websocket_manager import WebSocketManager
from amr_web.state_bridge import StateBridge

mimetypes.add_type('text/css', '.css')
mimetypes.add_type('application/javascript', '.js')


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
    async def get_maps():
        if bridge_node:
            maps = bridge_node.get_available_maps()
            return {"status": "ok", "maps": maps}
        return {"status": "error", "message": "Bridge node not available"}

    @app.get("/api/map/image/{map_id}")
    async def get_map_image(map_id: str):
        if not bridge_node:
            return {"status": "error", "message": "Bridge node not available"}

        import yaml
        import base64

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
            pgm_path = os.path.join(maps_dir, pgm_filename)
            if not os.path.exists(pgm_path):
                return {"status": "error", "message": f"Image file not found: {pgm_path}"}

            with open(pgm_path, 'rb') as f:
                pgm_data = f.read()

            header_end = pgm_data.find(b'\n', pgm_data.find(b'\n', pgm_data.find(b'\n') + 1) + 1) + 1
            img_data = pgm_data[header_end:]

            header = pgm_data[:header_end].decode('ascii', errors='ignore')
            parts = header.split()
            width = 0
            height = 0
            for p in parts:
                if p.isdigit() and int(p) > 0:
                    if width == 0:
                        width = int(p)
                    elif height == 0:
                        height = int(p)
                        break

            if width == 0 or height == 0:
                return {"status": "error", "message": f"Invalid PGM dimensions: {width}x{height}"}

            origin = yaml_data.get('origin', [0, 0, 0])
            expected_size = width * height
            if len(img_data) < expected_size:
                return {"status": "error", "message": "PGM pixel data is incomplete"}
            encoded = base64.b64encode(img_data[:expected_size]).decode('ascii')

            return {
                "status": "ok",
                "info": {
                    "resolution": yaml_data.get('resolution', 0.05),
                    "width": width,
                    "height": height,
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
            return {"status": "error", "message": str(e)}

    @app.get("/api/routes")
    async def get_routes(map_id: str = None):
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

        success, msg = bridge_node.save_route(map_id, route_data)
        if success:
            return {"status": "ok", "message": msg}
        return {"status": "error", "message": msg}

    @app.delete("/api/maps/{map_id}")
    async def delete_map(map_id: str):
        if not bridge_node:
            return {"status": "error", "message": "Bridge node not available"}

        success, msg = bridge_node.delete_map_and_routes(map_id)
        if success:
            return {"status": "ok", "message": msg}
        return {"status": "error", "message": msg}

    async def handle_command(cmd: dict, websocket: WebSocket) -> dict:
        command = cmd.get('command', '')
        result = {"type": "command_response", "command": command}

        # P1-1 Fix: Run blocking commands in thread pool
        if command == "start_slam":
            if bridge_node:
                if bridge_node.is_navigation_running():
                    result["status"] = "failed"
                    result["message"] = "Navigation đang chạy – hãy dừng Navigation trước"
                else:
                    success = await asyncio.to_thread(bridge_node.start_slam)
                    result["status"] = "started" if success else "failed"
                    result["message"] = "SLAM started" if success else "SLAM already running or failed"
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
                if bridge_node.get_slam_state():
                    result["status"] = "failed"
                    result["message"] = "SLAM đang chạy – hãy dừng SLAM trước"
                else:
                    success, msg = await asyncio.to_thread(bridge_node.start_navigation, map_id, route_id)
                    result["status"] = "started" if success else "failed"
                    result["message"] = msg
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
                # Broadcast full navigation state
                await ws_manager.broadcast(json.dumps(bridge_node.get_nav_state()))
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command == "nav_goal":
            pose = cmd.get('pose', {})
            route_id = cmd.get('route_id')
            if bridge_node:
                success, msg = await asyncio.to_thread(bridge_node.send_nav_goal, pose, route_id)
                result["status"] = "sent" if success else "failed"
                result["message"] = msg
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
                success, msg = await asyncio.to_thread(bridge_node.set_nav_initial_pose, pose)
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
                success, msg = await asyncio.to_thread(bridge_node.send_waypoint_mission, waypoints, mission_type)
                result["status"] = "started" if success else "failed"
                result["message"] = msg
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
                bridge_node._nav_bridge.set_op_mode(mode)
                result["status"] = "ok"
                result["mode"] = mode
            else:
                result["status"] = "error"
                result["message"] = "NavBridge not available"

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
                bridge_node.publish_cmd_vel(linear, angular)
                result["status"] = "ok"
                result["linear"] = linear
                result["angular"] = angular
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command == "lift":
            action = cmd.get('action', '')
            target_mm = cmd.get('target_mm', 0)
            if bridge_node:
                bridge_node.publish_lift(action, target_mm)
                result["status"] = "ok"
                result["action"] = action
                result["target_mm"] = target_mm
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        else:
            result["status"] = "unknown"
            result["message"] = f"Unknown command: {command}"

        return result

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()
        ws_manager.add(websocket)

        if node_logger:
            node_logger.info(f'Client connected. Total: {ws_manager.connection_count}')

        try:
            # P1-13 Fix: Send initial state using ws_manager.send
            for _, data in state_bridge.get_all_state().items():
                await ws_manager.send(websocket, json.dumps(data))

            await ws_manager.send(websocket, json.dumps(state_bridge.get_system_overview_state()))

            if bridge_node:
                await ws_manager.send(websocket, json.dumps({
                    "type": "slam_state",
                    "is_slam_active": bridge_node.get_slam_state()
                }))
                await ws_manager.send(websocket, json.dumps(bridge_node.get_nav_state()))

            # P1-9 Fix: Proper error handling, don't swallow exceptions
            while True:
                try:
                    raw = await websocket.receive_text()
                    try:
                        cmd = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    if not isinstance(cmd, dict) or 'command' not in cmd:
                        continue
                    try:
                        response = await handle_command(cmd, websocket)
                    except Exception as e:
                        if node_logger:
                            node_logger.error(f"Command '{cmd.get('command')}' failed: {e}\n{traceback.format_exc()}")
                        response = {"type": "command_response", "command": cmd.get('command'),
                                    "status": "error", "message": str(e)}
                    await ws_manager.send(websocket, json.dumps(response))
                except WebSocketDisconnect:
                    break

        except Exception as e:
            if node_logger:
                node_logger.error(f'WS endpoint error: {e}')
        finally:
            ws_manager.remove(websocket)
            if node_logger:
                node_logger.info('Client disconnected')

    if os.path.exists(web_dir):
        app.mount("/", StaticFiles(directory=web_dir, html=True), name="static")

    return app
