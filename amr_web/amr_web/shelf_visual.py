"""Read-only rack geometry from the existing shelf model and active route."""
import math

from geometry_msgs.msg import PoseStamped

# Surveyed dimensions supplied for visualization, separate from mission parameters.
SHELF_WIDTH_M = 1.875
SHELF_HEIGHT_M = 0.90
SLOT_DEPTH_M = 0.35
SUPPORT_HEIGHT_M = {1: 0.20, 2: 0.55}
# Carton heights for shelf visualization by configured product family.
SHELF_CARTON_HEIGHT_M = {'A': 0.15, 'B': 0.40}


def rack_from_route(shelf, route, frame_id):
    name = route.resolve(shelf.anchor_node)
    anchor: PoseStamped = route.get_node_pose(name) if name else None
    if anchor is None or anchor.header.frame_id != frame_id:
        return None
    p, q = anchor.pose.position, anchor.pose.orientation
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    if not all(math.isfinite(v) for v in (p.x, p.y, yaw)):
        return None
    from amr_mission_manager.shelf_model import SHELF_FRONT
    poses = shelf.map_poses(p.x, p.y, yaw)
    slots = []
    supports = []
    columns = {slot.col for slot in shelf.slots}
    if not math.isclose(len(columns) * shelf.col_pitch_m, SHELF_WIDTH_M, abs_tol=0.001):
        return None  # The surveyed geometry does not describe this shelf topology.
    fx, fy, yaw = poses[SHELF_FRONT]
    tx, ty = math.cos(yaw), math.sin(yaw)
    for slot in shelf.slots:
        level = shelf.slot_level.get(slot.id)
        if level not in SUPPORT_HEIGHT_M:
            return None
        sx, sy, syaw = poses[slot.tf_frame]
        z = SUPPORT_HEIGHT_M[level]
        slots.append({'id': slot.id, 'shelf_slot_id': slot.id if slot.sensor_confirmed else None,
                      'pose': {'x': sx + tx*SLOT_DEPTH_M/2, 'y': sy + ty*SLOT_DEPTH_M/2,
                               'yaw': syaw, 'z': z},
                      'cargo_height_m': SHELF_CARTON_HEIGHT_M.get(slot.product_type)})
        supports.append({'id': slot.id, 'x': -(sx-fx)*ty + (sy-fy)*tx,
                         'z': z, 'width': shelf.col_pitch_m})
    return {'id': SHELF_FRONT, 'type': 'rack',
            'pose': {'x': fx + tx*SLOT_DEPTH_M/2, 'y': fy + ty*SLOT_DEPTH_M/2,
                     'yaw': yaw + math.pi/2},
            'width': SHELF_WIDTH_M, 'depth': SLOT_DEPTH_M, 'height': SHELF_HEIGHT_M,
            'supports': supports,
            'partitions': [(i - len(columns)/2)*shelf.col_pitch_m for i in range(1, len(columns))],
            'slots': slots}
