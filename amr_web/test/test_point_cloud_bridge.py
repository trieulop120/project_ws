import struct
import unittest
from threading import Lock
from types import SimpleNamespace as NS

from rclpy.time import Time
from rclpy.clock import ClockType
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformException

from amr_web.bridge_node import WebBridge


def message(stamp_sec=9):
    msg = PointCloud2()
    msg.header.frame_id = 'camera_depth_optical_frame'
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = 500_000_000
    msg.width = 1
    msg.height = 1
    msg.point_step = 12
    msg.row_step = 12
    msg.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                  for name, offset in [('x', 0), ('y', 4), ('z', 8)]]
    msg.data = struct.pack('<fff', 1, 0, 0)
    return msg


class FakeBuffer:
    def __init__(self, fail=False):
        self.calls = []
        self.fail = fail

    def lookup_transform(self, target, source, stamp):
        self.calls.append((target, source, stamp.nanoseconds))
        if self.fail:
            raise TransformException('No TF at sensor timestamp')
        return NS(transform=NS(translation=NS(x=2, y=3, z=4),
                               rotation=NS(x=0, y=0, z=0, w=1)))


class BridgeCloudTest(unittest.TestCase):
    def bridge(self, buffer):
        return NS(_tf_buffer=buffer, _map_frame='map',
                  get_clock=lambda: NS(now=lambda: Time(seconds=10, clock_type=ClockType.ROS_TIME)))

    def test_exact_sensor_stamp_and_map_coordinates(self):
        buffer = FakeBuffer()
        result = WebBridge._convert_point_cloud(self.bridge(buffer), message())
        self.assertEqual(buffer.calls, [('map', 'camera_depth_optical_frame', 9_500_000_000)])
        self.assertEqual(result['frame_id'], 'map')
        self.assertEqual(result['stamp'], {'sec': 9, 'nanosec': 500_000_000})
        self.assertEqual(result['sensor_origin'], [2, 3, 4])
        self.assertEqual(result['points'], [[3.0, 3.0, 4.0]])

    def test_cloud_mailbox_records_the_selected_map_at_capture(self):
        bridge = NS(_ws_manager=NS(has_point_cloud_clients=True),
                    _nav_bridge=NS(get_active_map_id=lambda: None),
                    _cloud_lock=Lock(), _cloud_latest=None)
        frame = message()
        WebBridge._on_point_cloud(bridge, frame)
        self.assertIsNone(bridge._cloud_latest)
        bridge._nav_bridge.get_active_map_id = lambda: 'Warehouse A.2'
        WebBridge._on_point_cloud(bridge, frame)
        self.assertEqual(bridge._cloud_latest, ('Warehouse A.2', frame))

    def test_lidar_height_is_read_from_existing_base_transform(self):
        buffer = FakeBuffer()
        bridge = NS(_tf_buffer=buffer, _base_frame='base_footprint',
                    get_parameter=lambda name: NS(value='lidar_link'))
        self.assertEqual(WebBridge.get_lidar_height(bridge), 4.0)
        self.assertEqual(buffer.calls, [('base_footprint', 'lidar_link', 0)])
        bridge._tf_buffer = FakeBuffer(fail=True)
        self.assertIsNone(WebBridge.get_lidar_height(bridge))

    def test_stale_frame_and_missing_tf_are_dropped(self):
        buffer = FakeBuffer()
        self.assertIsNone(WebBridge._convert_point_cloud(self.bridge(buffer), message(8)))
        self.assertEqual(buffer.calls, [])
        missing = FakeBuffer(fail=True)
        self.assertIsNone(WebBridge._convert_point_cloud(self.bridge(missing), message()))
        self.assertEqual(len(missing.calls), 1)


if __name__ == '__main__':
    unittest.main()
