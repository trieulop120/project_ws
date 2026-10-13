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
        self._lock = threading.Lock()

    def add(self, websocket: WebSocket) -> None:
        with self._lock:
            self._connections[websocket] = asyncio.Lock()

    def remove(self, websocket: WebSocket) -> None:
        with self._lock:
            self._connections.pop(websocket, None)

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
