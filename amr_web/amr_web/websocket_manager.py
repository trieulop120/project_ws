#!/usr/bin/env python3
# amr_web/websocket_manager.py
"""
Thread-safe WebSocket connection manager.
Phase 1 Fix: One send point, per-connection lock, don't auto-remove on send error.
"""
import asyncio
import threading
from typing import Dict, List
from fastapi import WebSocket


class WebSocketManager:
    """Thread-safe WebSocket connection manager."""

    def __init__(self):
        self._connections: Dict[WebSocket, asyncio.Lock] = {}
        self._point_cloud_clients = set()
        self._lock = threading.Lock()

    def add(self, websocket: WebSocket) -> None:
        with self._lock:
            self._connections[websocket] = asyncio.Lock()

    def remove(self, websocket: WebSocket) -> None:
        with self._lock:
            self._connections.pop(websocket, None)
            self._point_cloud_clients.discard(websocket)

    def set_point_cloud_enabled(self, websocket: WebSocket, enabled: bool) -> bool:
        """Opt a connected client into or out of cloud telemetry."""
        with self._lock:
            if websocket not in self._connections:
                return False
            if enabled:
                self._point_cloud_clients.add(websocket)
            else:
                self._point_cloud_clients.discard(websocket)
            return True

    @property
    def has_point_cloud_clients(self) -> bool:
        with self._lock:
            return bool(self._point_cloud_clients)

    async def broadcast_point_cloud(self, message: str) -> None:
        with self._lock:
            clients = list(self._point_cloud_clients)
        if clients:
            await asyncio.gather(*(self.send(ws, message, drop_if_busy=True)
                                   for ws in clients))

    async def send(self, ws: WebSocket, message: str, drop_if_busy: bool = False) -> bool:
        """Send message to a single WebSocket with per-connection lock."""
        with self._lock:
            lock = self._connections.get(ws)
        if lock is None:
            return False
        if drop_if_busy and lock.locked():
            # Client slow: drop telemetry, don't queue
            return True
        try:
            async with lock:
                await asyncio.wait_for(ws.send_text(message), timeout=3.0)
            return True
        except Exception:
            # Don't remove here - endpoint will remove on receive disconnect
            return False

    async def broadcast(self, message: str) -> None:
        """Broadcast message to all connected clients."""
        with self._lock:
            conns: List[WebSocket] = list(self._connections)
        if conns:
            await asyncio.gather(*(self.send(ws, message, drop_if_busy=True) for ws in conns))

    @property
    def connection_count(self) -> int:
        with self._lock:
            return len(self._connections)
