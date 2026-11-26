import unittest
from types import SimpleNamespace as NS

from rclpy.clock import ClockType
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException

from amr_web.bridge_node import WebBridge


class LaserScanBridgeTest(unittest.TestCase):
    def setUp(self):
        self.calls = []
        self.map_id = 'warehouse'
        self.now = Time(seconds=10, clock_type=ClockType.ROS_TIME)
        self.fail = False
        self.bridge = NS(
            _map_frame='map', _base_frame='base_footprint', _last_laser_convert=0,
            _laser_interval=.1, _verbose=False,
            _tf_buffer=NS(lookup_transform=self.lookup),
            _nav_bridge=NS(get_active_map_id=lambda: self.map_id),
            get_clock=lambda: NS(clock_type=ClockType.ROS_TIME, now=lambda: self.now),
        )
        self.cache = {}
        self.bridge._state_bridge = NS(
            update_laser=lambda data: self.cache.update(data),
            update_heartbeat=lambda key: self.cache.update(heartbeat=key),
        )

    def lookup(self, target, source, stamp):
        self.calls.append((target, source, stamp.nanoseconds))
        if self.fail:
            raise TransformException('TF unavailable')
        return NS(transform=NS(translation=NS(x=1., y=2., z=.23),
                               rotation=NS(x=0., y=0., z=0., w=1.)))

    def capture(self):
        scan = LaserScan()
        scan.header.frame_id = 'lidar_actual_frame'
        scan.header.stamp.sec = 9
        scan.header.stamp.nanosec = 900_000_000
        scan.angle_min = -.7
        scan.angle_increment = .002345678
        scan.range_min = .1
        scan.range_max = 10.
        scan.ranges = [1., float('inf')]
        WebBridge._on_laser(self.bridge, scan)

    def test_metadata_uses_existing_scan_and_first_ray_stamp(self):
        self.capture()
        self.assertEqual(self.cache['ranges'], [1., None])
        self.assertEqual(self.cache['heartbeat'], 'lidar')
        self.assertEqual(self.cache['map_id'], 'warehouse')
        self.assertEqual(self.cache['angle_increment'], .0023)
        self.assertAlmostEqual(self.cache['angle_increment_rad'], .002345678)
        pose = WebBridge._laser_visual_pose(self.bridge, self.cache)
        self.assertEqual(pose, dict(frame_id='map', position=[1.,2.,.23], rotation=[0.,0.,0.,1.], height_m=.23))
        self.assertEqual(self.calls, [('map', 'lidar_actual_frame', 9_900_000_000),
                                      ('base_footprint', 'lidar_actual_frame', 9_900_000_000)])

    def test_foreign_map_stale_or_unavailable_tf_omit_pose(self):
        self.capture()
        self.map_id = 'another-map'
        self.assertIsNone(WebBridge._laser_visual_pose(self.bridge, self.cache))
        self.map_id = 'warehouse'
        self.now = Time(seconds=11, clock_type=ClockType.ROS_TIME)
        self.assertIsNone(WebBridge._laser_visual_pose(self.bridge, self.cache))
        self.assertEqual(self.calls, [])
        self.now = Time(seconds=10, clock_type=ClockType.ROS_TIME)
        self.fail = True
        self.assertIsNone(WebBridge._laser_visual_pose(self.bridge, self.cache))
        self.assertEqual(self.cache['ranges'], [1., None])


if __name__ == '__main__':
    unittest.main()
