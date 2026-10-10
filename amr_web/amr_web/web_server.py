#!/usr/bin/env python3
# amr_web/web_server.py

import os
import json
import asyncio
import mimetypes

from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from amr_web.websocket_manager import WebSocketManager
from amr_web.state_bridge import StateBridge


# BẮT BUỘC: Ép Python nhận đúng MIME Type chuẩn cho Firefox/Chrome
mimetypes.add_type('text/css', '.css')
mimetypes.add_type('application/javascript', '.js')


def create_app(state_bridge: StateBridge, ws_manager: WebSocketManager, node_logger=None, bridge_node=None) -> FastAPI:
    app = FastAPI(title="AMR Web Bridge")

    # Lấy đường dẫn cài đặt gốc trong install/
    pkg_share_dir = get_package_share_directory('amr_web')
    web_dir = os.path.join(pkg_share_dir, 'resources', 'web')

    if node_logger:
        node_logger.info(f"Serving web directory from: {web_dir}")

    # Health Check Endpoint
    @app.get("/health")
    async def health():
        return {"status": "ok", "service": "amr_web", "version": "0.2.0"}

    # ============================================================
    # Map & Route API Endpoints
    # ============================================================

    @app.get("/api/maps")
    async def get_maps():
        """Lấy danh sách maps từ ~/project_ws/maps/."""
        if bridge_node:
            maps = bridge_node.get_available_maps()
            return {"status": "ok", "maps": maps}
        return {"status": "error", "message": "Bridge node not available"}

    @app.get("/api/map/image/{map_id}")
    async def get_map_image(map_id: str):
        """Lấy dữ liệu hình ảnh map từ file YAML + PGM."""
        if not bridge_node:
            return {"status": "error", "message": "Bridge node not available"}

        import os
        import yaml
        import base64

        maps_dir = os.path.expanduser('~/project_ws/maps')
        yaml_path = os.path.join(maps_dir, f'{map_id}.yaml')

        if not os.path.exists(yaml_path):
            return {"status": "error", "message": f"Map {map_id} not found"}

        try:
            # Đọc YAML metadata
            with open(yaml_path, 'r', encoding='utf-8') as f:
                yaml_data = yaml.safe_load(f)

            if not yaml_data:
                return {"status": "error", "message": "Empty YAML data"}

            # Đọc PGM image
            pgm_filename = yaml_data.get('image', f'{map_id}.pgm')
            pgm_path = os.path.join(maps_dir, pgm_filename)
            if not os.path.exists(pgm_path):
                return {"status": "error", "message": f"Image file not found: {pgm_path}"}

            with open(pgm_path, 'rb') as f:
                pgm_data = f.read()

            # Parse PGM header - simple approach
            # Find the newline after maxval to know where data starts
            header_end = pgm_data.find(b'\n', pgm_data.find(b'\n', pgm_data.find(b'\n') + 1) + 1) + 1
            img_data = pgm_data[header_end:]

            # Parse width/height from header
            header = pgm_data[:header_end].decode('ascii', errors='ignore')
            parts = header.split()
            # parts should be like ['P5', '108', '70', '255']
            width = 0
            height = 0
            for i, p in enumerate(parts):
                if p.isdigit() and int(p) > 0:
                    if width == 0:
                        width = int(p)
                    elif height == 0:
                        height = int(p)
                        break

            if width == 0 or height == 0:
                return {"status": "error", "message": f"Invalid PGM dimensions: {width}x{height}"}

            origin = yaml_data.get('origin', [0, 0, 0])

            # Map & Route is an image editor: retain every original PGM gray
            # value and its top-to-bottom row order.  It must not be converted
            # to the three-state OccupancyGrid representation used by live SLAM.
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
                "image_data": encoded
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}

    @app.get("/api/routes")
    async def get_routes(map_id: str = None):
        """Lấy danh sách routes từ ~/project_ws/maps/{map_id}/routes/."""
        if bridge_node:
            routes = bridge_node.get_available_routes(map_id)
            return {"status": "ok", "routes": routes}
        return {"status": "error", "message": "Bridge node not available"}

    @app.post("/api/routes")
    async def save_route(payload: dict):
        """API tiếp nhận dữ liệu POST từ nút Save Route trên Web SCADA."""
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
        """API tiếp nhận yêu cầu xóa Map và các Route liên quan."""
        if not bridge_node:
            return {"status": "error", "message": "Bridge node not available"}

        success, msg = bridge_node.delete_map_and_routes(map_id)
        if success:
            return {"status": "ok", "message": msg}
        return {"status": "error", "message": msg}
    
    # ============================================================
    # Command Handlers
    # ============================================================

    async def handle_command(cmd: dict) -> dict:
        """Xử lý commands từ WebSocket client."""
        command = cmd.get('command', '')
        result = {"type": "command_response", "command": command}

        if command == "start_slam":
            if bridge_node:
                success = bridge_node.start_slam()
                result["status"] = "started" if success else "failed"
                result["message"] = "SLAM started" if success else "SLAM already running or failed"

                # Broadcast slam_state to ALL clients
                await ws_manager.broadcast(json.dumps({
                    "type": "slam_state",
                    "is_slam_active": bridge_node.get_slam_state()
                }))
            else:
                result["status"] = "error"
                result["message"] = "Bridge node not available"

        elif command == "stop_slam":
            if bridge_node:
                bridge_node.stop_slam()
                result["status"] = "stopped"
                result["message"] = "SLAM stopped"

                # Broadcast slam_state to ALL clients
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
                success, path = bridge_node.save_map(map_name)
                if node_logger:
                    node_logger.info(f'save_map result: success={success}, path={path}')
                result["status"] = "success" if success else "failed"
                result["path"] = path
                result["message"] = f"Map saved to {path}" if success else "Save failed"
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

        elif command == "get_map":
            # Push current map data immediately to client
            if state_bridge:
                latest_map = state_bridge.get_latest_map()
                if latest_map:
                    await websocket.send_text(json.dumps(latest_map))
                    result["status"] = "found"
                    result["message"] = "Map sent"
                else:
                    result["status"] = "not_found"
                    result["message"] = "No map available"
            else:
                result["status"] = "error"
                result["message"] = "State bridge not available"

        elif command == "request_map":
            # Alias for get_map - push current map data immediately to client
            if state_bridge:
                latest_map = state_bridge.get_latest_map()
                if latest_map:
                    await websocket.send_text(json.dumps(latest_map))
                    result["status"] = "found"
                    result["message"] = "Map sent"
                else:
                    result["status"] = "not_found"
                    result["message"] = "No map available"
            else:
                result["status"] = "error"
                result["message"] = "State bridge not available"

        else:
            result["status"] = "unknown"
            result["message"] = f"Unknown command: {command}"

        return result

    # ============================================================
    # WebSocket Endpoint
    # ============================================================
    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()
        ws_manager.add(websocket)

        if node_logger:
            node_logger.info(f'Client connected. Total: {ws_manager.connection_count}')

        try:
            state = state_bridge.get_all_state()
            for key, data in state.items():
                await websocket.send_text(json.dumps(data))

            # Gửi system_status ngay khi client kết nối
            system_status = state_bridge.get_system_overview_state()
            await websocket.send_text(json.dumps(system_status))

            # Gửi slam_state hiện tại cho client mới
            if bridge_node:
                await websocket.send_text(json.dumps({
                    "type": "slam_state",
                    "is_slam_active": bridge_node.get_slam_state()
                }))

            while True:
                try:
                    data = await asyncio.wait_for(websocket.receive_text(), timeout=5.0)

                    # Xử lý command từ client
                    try:
                        cmd = json.loads(data)
                        if 'command' in cmd:
                            response = await handle_command(cmd)
                            await websocket.send_text(json.dumps(response))
                            continue
                    except json.JSONDecodeError:
                        pass

                except asyncio.TimeoutError:
                    # Gửi system_status thay vì robot_status
                    system_status = state_bridge.get_system_overview_state()
                    await websocket.send_text(json.dumps(system_status))

        except (WebSocketDisconnect, Exception):
            pass
        finally:
            ws_manager.remove(websocket)
            if node_logger:
                node_logger.info('Client disconnected')

    # MOUNT STATIC FILES VỚI DẤU HOẶC AN TOÀN DÀNH CHO FIREFOX
    if os.path.exists(web_dir):
        app.mount("/", StaticFiles(directory=web_dir, html=True), name="static")

    return app
