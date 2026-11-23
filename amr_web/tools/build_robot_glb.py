#!/usr/bin/env python3
"""Offline STL/URDF -> GLB conversion for the Navigation Digital Twin.

Requires numpy only while building the asset. No ROS node, web request, or server
process runs this conversion. GLB coordinates are glTF Y-up; the viewer rotates
the root +90 degrees about X to recover ROS X-forward, Y-left, Z-up.
"""

import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


WORKSPACE_SRC = Path(__file__).resolve().parents[2]
DESCRIPTION = WORKSPACE_SRC / 'amr_description'
URDF = DESCRIPTION / 'urdf' / 'amr_description.urdf'
XACRO = DESCRIPTION / 'urdf' / 'amr.gazebo.xacro'
OUTPUT = Path(__file__).resolve().parents[1] / 'resources/web/models/amr_description.glb'
STL_DTYPE = np.dtype([
    ('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')
])


def numbers(text, default):
    return [float(value) for value in (text or default).split()]


def gltf_vector(vector):
    x, y, z = vector
    return [float(x), float(z), float(-y)]


def quaternion_multiply(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return [w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2]


def rpy_to_gltf_quaternion(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    ros = [sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
           cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy]
    basis = [-math.sqrt(0.5), 0, 0, math.sqrt(0.5)]
    inverse = [math.sqrt(0.5), 0, 0, math.sqrt(0.5)]
    return quaternion_multiply(quaternion_multiply(basis, ros), inverse)


def read_stl(path):
    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError(f'Invalid binary STL: {path}')
    triangles = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) != 84 + triangles * 50:
        raise ValueError(f'Invalid binary STL triangle count: {path}')
    facets = np.frombuffer(raw, dtype=STL_DTYPE, count=triangles, offset=84)
    original = facets['vertices'].astype(np.float64)
    face_normals = np.cross(original[:, 1] - original[:, 0],
                            original[:, 2] - original[:, 0])
    norm = np.linalg.norm(face_normals, axis=1)
    zero = norm < 1e-12
    face_normals[zero] = facets['normal'][zero]
    norm = np.linalg.norm(face_normals, axis=1)
    face_normals /= np.maximum(norm[:, None], 1e-12)
    vertices = original.reshape(-1, 3)
    normals = np.repeat(face_normals, 3, axis=0)
    # Weld matching position/normal pairs. 10 um position and 0.001 normal
    # precision are below the source STL's practical manufacturing resolution.
    keys = np.column_stack((np.rint(vertices * 100000).astype(np.int32),
                            np.rint(normals * 1000).astype(np.int32)))
    unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    positions_ros = unique[:, :3].astype(np.float64) / 100000
    normals_ros = unique[:, 3:].astype(np.float64) / 1000
    normals_ros /= np.maximum(np.linalg.norm(normals_ros, axis=1)[:, None], 1e-12)
    positions = np.column_stack((positions_ros[:, 0], positions_ros[:, 2],
                                 -positions_ros[:, 1])).astype('<f4')
    normals_out = np.column_stack((normals_ros[:, 0], normals_ros[:, 2],
                                   -normals_ros[:, 1])).astype('<f4')
    index_type = '<u2' if len(unique) <= 65535 else '<u4'
    indices = inverse.astype(index_type)
    return positions, normals_out, indices, original


def build():
    urdf = ET.parse(URDF).getroot()
    xacro = ET.parse(XACRO).getroot()
    footprint_joint = xacro.find("./joint[@name='base_footprint_joint']")
    if footprint_joint is None or footprint_joint.find('parent').get('link') != 'base_footprint' or \
            footprint_joint.find('child').get('link') != 'base_link':
        raise ValueError('base_footprint -> base_link transform is missing')

    blob = bytearray()
    views = []
    accessors = []
    meshes = []
    materials = []
    nodes = [{'name': 'base_footprint', 'children': []}]
    node_by_link = {'base_footprint': 0}
    bounds = {}

    def add_view(data, target):
        while len(blob) % 4:
            blob.append(0)
        offset = len(blob)
        blob.extend(data)
        views.append({'buffer': 0, 'byteOffset': offset, 'byteLength': len(data),
                      'target': target})
        return len(views) - 1

    def add_accessor(data, component, kind, target, limits=False):
        view = add_view(data.tobytes(), target)
        accessor = {'bufferView': view, 'componentType': component,
                    'count': len(data), 'type': kind}
        if limits:
            accessor['min'] = [float(v) for v in data.min(axis=0)]
            accessor['max'] = [float(v) for v in data.max(axis=0)]
        accessors.append(accessor)
        return len(accessors) - 1

    for link in urdf.findall('link'):
        name = link.get('name')
        visual = link.find('visual')
        mesh_element = visual.find('geometry/mesh') if visual is not None else None
        if mesh_element is None:
            continue
        visual_origin = visual.find('origin')
        if visual_origin is not None and (numbers(visual_origin.get('xyz'), '0 0 0') != [0, 0, 0] or
                                         numbers(visual_origin.get('rpy'), '0 0 0') != [0, 0, 0]):
            raise ValueError(f'Nonzero visual transform needs an explicit child node: {name}')
        if mesh_element.get('scale') not in (None, '1 1 1'):
            raise ValueError(f'Unexpected mesh scale: {name}')
        filename = Path(mesh_element.get('filename')).name
        positions, normals, indices, original = read_stl(DESCRIPTION / 'meshes' / filename)
        bounds[name] = (original.min(axis=(0, 1)), original.max(axis=(0, 1)))
        position_accessor = add_accessor(positions, 5126, 'VEC3', 34962, True)
        normal_accessor = add_accessor(normals, 5126, 'VEC3', 34962)
        index_accessor = add_accessor(indices, 5123 if indices.dtype.itemsize == 2 else 5125,
                                      'SCALAR', 34963)
        color_element = visual.find('material/color')
        color = numbers(color_element.get('rgba'), '0.75 0.78 0.80 1') if color_element is not None \
            else [0.75, 0.78, 0.80, 1]
        materials.append({'name': name, 'pbrMetallicRoughness': {
            'baseColorFactor': color, 'metallicFactor': 0.0, 'roughnessFactor': 0.85},
            'doubleSided': True})
        meshes.append({'name': name, 'primitives': [{'attributes': {
            'POSITION': position_accessor, 'NORMAL': normal_accessor},
            'indices': index_accessor, 'material': len(materials) - 1}]})
        node_by_link[name] = len(nodes)
        nodes.append({'name': name, 'mesh': len(meshes) - 1, 'children': []})

    joints = [footprint_joint] + list(urdf.findall('joint'))
    for joint in joints:
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        if parent not in node_by_link or child not in node_by_link:
            raise ValueError(f'Joint link absent: {joint.get("name")}')
        origin = joint.find('origin')
        xyz = numbers(origin.get('xyz'), '0 0 0') if origin is not None else [0, 0, 0]
        rpy = numbers(origin.get('rpy'), '0 0 0') if origin is not None else [0, 0, 0]
        node = nodes[node_by_link[child]]
        node['translation'] = gltf_vector(xyz)
        if any(abs(v) > 1e-12 for v in rpy):
            node['rotation'] = rpy_to_gltf_quaternion(rpy)
        node['extras'] = {'urdf_joint': joint.get('name'), 'joint_type': joint.get('type')}
        nodes[node_by_link[parent]]['children'].append(node_by_link[child])

    document = {
        'asset': {'version': '2.0', 'generator': 'amr_web offline URDF/STL converter'},
        'scene': 0, 'scenes': [{'nodes': [0]}], 'nodes': nodes, 'meshes': meshes,
        'materials': materials, 'buffers': [{'byteLength': len(blob)}],
        'bufferViews': views, 'accessors': accessors,
    }
    json_chunk = json.dumps(document, separators=(',', ':')).encode('utf-8')
    json_chunk += b' ' * (-len(json_chunk) % 4)
    blob.extend(b'\0' * (-len(blob) % 4))
    total = 12 + 8 + len(json_chunk) + 8 + len(blob)
    output = struct.pack('<4sII', b'glTF', 2, total)
    output += struct.pack('<I4s', len(json_chunk), b'JSON') + json_chunk
    output += struct.pack('<I4s', len(blob), b'BIN\0') + blob
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_bytes(output)
    print(f'{OUTPUT}: {len(output):,} bytes, {len(meshes)} meshes, '
          f'{sum(len(m["primitives"]) for m in meshes)} primitives')
    print('base_link STL bounds (m):', bounds['base_link'])
    print('base_footprint -> base_link (m):',
          numbers(footprint_joint.find('origin').get('xyz'), '0 0 0'))


if __name__ == '__main__':
    build()
