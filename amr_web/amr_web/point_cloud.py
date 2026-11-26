"""Bounded PointCloud2 XYZ decoding and rigid transforms for web telemetry."""

import math
import struct


_FLOAT_FORMAT = {7: 'f', 8: 'd'}  # sensor_msgs/PointField FLOAT32/FLOAT64


def read_xyz(msg, limit=500):
    """Read finite XYZ points, sampling evenly before decoding large clouds."""
    width, height = int(msg.width), int(msg.height)
    point_step, row_step = int(msg.point_step), int(msg.row_step)
    total = width * height
    if total <= 0 or limit <= 0 or point_step <= 0 or row_step < width * point_step:
        return []
    data = memoryview(msg.data)
    if len(data) < (height - 1) * row_step + width * point_step:
        return []
    fields = {field.name: field for field in msg.fields}
    endian = '>' if msg.is_bigendian else '<'
    readers = []
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        fmt = _FLOAT_FORMAT.get(getattr(field, 'datatype', None))
        if fmt is None or field.count < 1:
            return []
        reader = struct.Struct(endian + fmt)
        if field.offset < 0 or field.offset + reader.size > point_step:
            return []
        readers.append((field.offset, reader))

    count = min(total, limit)
    points = []
    for sample in range(count):
        index = sample * total // count
        row, col = divmod(index, width)
        base = row * row_step + col * point_step
        point = tuple(reader.unpack_from(data, base + offset)[0]
                      for offset, reader in readers)
        if all(math.isfinite(value) for value in point):
            points.append(point)
    return points


def transform_xyz(points, translation, rotation):
    """Apply map<-source translation and normalized quaternion rotation."""
    tx, ty, tz = translation
    qx, qy, qz, qw = rotation
    values = (tx, ty, tz, qx, qy, qz, qw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Non-finite transform')
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-12:
        raise ValueError('Invalid transform quaternion')
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    result = []
    for x, y, z in points:
        result.append([
            tx + (1-2*(yy+zz))*x + 2*(xy-wz)*y + 2*(xz+wy)*z,
            ty + 2*(xy+wz)*x + (1-2*(xx+zz))*y + 2*(yz-wx)*z,
            tz + 2*(xz-wy)*x + 2*(yz+wx)*y + (1-2*(xx+yy))*z,
        ])
    return result
