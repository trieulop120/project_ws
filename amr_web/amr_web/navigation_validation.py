"""Validation shared by navigation's HTTP and ROS entry points."""
from pathlib import Path


def valid_resource_id(value):
    return (isinstance(value, str) and bool(value.strip()) and value not in ('.', '..')
            and len(value) <= 200 and not any(c in value for c in ('/', '\\', '\x00')))


def map_image_path(maps_dir, map_id, image_name):
    if not valid_resource_id(map_id):
        raise ValueError('Invalid map identifier')
    path = (Path(maps_dir) / image_name).resolve()
    if not path.is_relative_to(Path(maps_dir).resolve()):
        raise ValueError('Map image must be inside the maps directory')
    return path
