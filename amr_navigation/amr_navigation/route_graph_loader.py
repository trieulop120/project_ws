#!/usr/bin/env python3
"""
route_graph_loader.py
====================
Data access layer cho Mission Manager.

Doc truc tiep tu route_graph.yaml:
- Node position/orientation -> PoseStamped
- Node type -> resolve HOME/PICK/DROP
- Sequential ID (theo thu tu trong YAML)

# Only reads route_graph.yaml - does NOT use route_poses.yaml
"""

import os
import yaml
import math
from typing import Optional, List
from geometry_msgs.msg import PoseStamped, Quaternion
from ament_index_python.packages import get_package_share_directory


class RouteGraphLoader:
    """Load route graph data tu route_graph.yaml."""

    def __init__(self, package_name: str = 'amr_navigation'):
        self._package_name = package_name
        self._yaml_data: dict = {}
        self._node_order: List[str] = []  # Sequential IDs
        self._loaded = False

    def _get_package_share_path(self) -> str:
        try:
            return get_package_share_directory(self._package_name)
        except Exception:
            home = os.path.expanduser("~/project_ws")
            return os.path.join(home, "src", self._package_name)

    def _load(self) -> None:
        """Load route_graph.yaml."""
        if self._loaded:
            return

        yaml_path = os.path.join(
            self._get_package_share_path(),
            'config', 'graphs', 'route_graph.yaml'
        )

        if os.path.exists(yaml_path):
            with open(yaml_path, 'r') as f:
                self._yaml_data = yaml.safe_load(f) or {}

            # Build sequential ID map theo thu tu nodes trong YAML
            nodes = self._yaml_data.get('nodes', {})
            self._node_order = list(nodes.keys())

        self._loaded = True

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """Chuyen yaw (rad) sang quaternion."""
        q = Quaternion()
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        q.x = 0.0
        q.y = 0.0
        q.z = sy
        q.w = cy
        return q

    def _find_by_type(self, node_type: str) -> Optional[str]:
        """Tim node theo type."""
        self._load()
        nodes = self._yaml_data.get('nodes', {})
        for name, data in nodes.items():
            if data.get('type') == node_type:
                return name
        return None

    def resolve(self, logical: str) -> Optional[str]:
        """
        Resolve logical name (HOME/PICK/DROP) thanh actual node name.

        Args:
            logical: HOME, PICK, hoac DROP

        Returns:
            Node name tu route_graph.yaml, hoac None
        """
        self._load()
        upper = logical.upper()

        if upper in ('HOME', 'NODE_HOME'):
            return self._find_by_type('home')
        elif upper in ('PICK', 'PICKUP'):
            return self._find_by_type('pickup_approach')
        elif upper in ('DROP', 'DROPOFF'):
            return self._find_by_type('dropoff_approach')
        elif logical in self._node_order:
            return logical
        return None

    def get_node_id(self, node_name: str) -> Optional[int]:
        """
        Lay sequential ID cua node.

        ID la thu tu trong route_graph.yaml (bat dau tu 0).

        Args:
            node_name: Ten node (VD: "P_1")

        Returns:
            Sequential ID, hoac None
        """
        self._load()
        if node_name in self._node_order:
            return self._node_order.index(node_name)
        return None

    def get_node_pose(self, node_name: str) -> Optional[PoseStamped]:
        """
        Lay PoseStamped tu route_graph.yaml.

        Args:
            node_name: Ten node (VD: "P_1")

        Returns:
            PoseStamped, hoac None
        """
        self._load()
        nodes = self._yaml_data.get('nodes', {})
        if node_name not in nodes:
            return None

        node = nodes[node_name]
        pos = node.get('position', {})
        orient = node.get('orientation', {})

        pose = PoseStamped()
        pose.header.frame_id = 'map'

        pose.pose.position.x = pos.get('x', 0.0)
        pose.pose.position.y = pos.get('y', 0.0)
        pose.pose.position.z = pos.get('z', 0.0)

        yaw = orient.get('yaw', 0.0)
        pose.pose.orientation = self._yaw_to_quaternion(yaw)

        return pose

    def get_all_node_names(self) -> List[str]:
        """Lay tat ca ten node."""
        self._load()
        return self._node_order.copy()

    def get_summary(self) -> str:
        """Tao summary cho log."""
        self._load()
        lines = ["RouteGraphLoader Summary:"]
        lines.append(f"  Total nodes: {len(self._node_order)}")
        lines.append(f"  HOME: {self.resolve('HOME')}")
        lines.append(f"  PICK: {self.resolve('PICK')}")
        lines.append(f"  DROP: {self.resolve('DROP')}")
        lines.append("  Node IDs:")
        for i, name in enumerate(self._node_order):
            lines.append(f"    {i}: {name}")
        return "\n".join(lines)


if __name__ == '__main__':
    loader = RouteGraphLoader()
    print("=" * 50)
    print("RouteGraphLoader Test")
    print("=" * 50)
    print(loader.get_summary())
    print()
    print("Test get_node_pose:")
    for name in ['NODE_HOME', 'P_1', 'D_1']:
        pose = loader.get_node_pose(name)
        if pose:
            print(f"  {name}: x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}")
