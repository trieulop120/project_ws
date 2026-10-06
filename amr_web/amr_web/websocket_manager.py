#!/usr/bin/env python3
# amr_web/websocket_manager.py
import threading
from typing import List
from fastapi import WebSocket


class WebSocketManager:
    """Thread-safe WebSocket connection manager."""

    def __init__(self):
        self._connections: List[WebSocket] = []
        self._lock = threading.Lock()

    def add(self, websocket: WebSocket) -> None:
        with self._lock:
            self._connections.append(websocket)

    def remove(self, websocket: WebSocket) -> None:
        with self._lock:
            if websocket in self._connections:
                self._connections.remove(websocket)

    async def broadcast(self, message: str) -> None:
        """Broadcast message to all connected clients."""
        disconnected = []

        with self._lock:
            connections_copy = list(self._connections)

        for ws in connections_copy:
            try:
                await ws.send_text(message)
            except Exception:
                disconnected.append(ws)

        if disconnected:
            with self._lock:
                for ws in disconnected:
                    if ws in self._connections:
                        self._connections.remove(ws)

    @property
    def connection_count(self) -> int:
        with self._lock:
            return len(self._connections)