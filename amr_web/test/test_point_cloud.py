import math
import asyncio
import struct
import unittest
from types import SimpleNamespace as NS

from amr_web.point_cloud import read_xyz, transform_xyz
from amr_web.websocket_manager import WebSocketManager


def cloud(rows, *, bigendian=False, padding=0):
    endian = '>' if bigendian else '<'
    width, height = len(rows[0]), len(rows)
    point_step = 16  # x,y,z plus an unrelated intensity field
    row_step = width * point_step + padding
    data = bytearray(row_step * height)
    for row, values in enumerate(rows):
        for col, xyz in enumerate(values):
            struct.pack_into(endian + 'fff', data, row*row_step + col*point_step + 4, *xyz)
    fields = [NS(name=name, offset=offset, datatype=7, count=1)
              for name, offset in [('z', 12), ('x', 4), ('y', 8)]]
    return NS(width=width, height=height, point_step=point_step,
              row_step=row_step, data=data, fields=fields, is_bigendian=bigendian)


class PointCloudTest(unittest.TestCase):
    def test_organized_rows_offsets_endianness_and_nonfinite(self):
        for bigendian in (False, True):
            msg = cloud([[(1, 2, 3), (math.nan, 0, 0)],
                         [(4, 5, 6), (math.inf, 0, 0)]],
                        bigendian=bigendian, padding=8)
            self.assertEqual(read_xyz(msg), [(1, 2, 3), (4, 5, 6)])

    def test_cap_samples_across_whole_cloud(self):
        msg = cloud([[(float(i), 0, 0) for i in range(1000)]])
        result = read_xyz(msg, limit=500)
        self.assertEqual(len(result), 500)
        self.assertEqual(result[0], (0, 0, 0))
        self.assertEqual(result[-1], (998, 0, 0))

    def test_rejects_truncated_data_and_missing_fields(self):
        msg = cloud([[(1, 2, 3)]])
        msg.data = msg.data[:4]
        self.assertEqual(read_xyz(msg), [])
        msg = cloud([[(1, 2, 3)]])
        msg.fields = msg.fields[:2]
        self.assertEqual(read_xyz(msg), [])

    def test_full_quaternion_rotation_and_translation(self):
        s = math.sqrt(0.5)
        # +90 degrees about Z: ROS +X becomes +Y.
        result = transform_xyz([(1, 0, 0)], (2, 3, 4), (0, 0, s, s))
        for actual, expected in zip(result[0], (2, 4, 4)):
            self.assertAlmostEqual(actual, expected)
        # +90 degrees about Y: +X becomes -Z (pitch must not be ignored).
        result = transform_xyz([(1, 0, 0)], (2, 3, 4), (0, s, 0, s))
        for actual, expected in zip(result[0], (2, 3, 3)):
            self.assertAlmostEqual(actual, expected)

    def test_invalid_quaternion_rejected(self):
        with self.assertRaises(ValueError):
            transform_xyz([(1, 0, 0)], (0, 0, 0), (0, 0, 0, 0))

    def test_websocket_stream_is_opt_in_and_removed_on_disconnect(self):
        class Client:
            def __init__(self):
                self.messages = []

            async def send_text(self, message):
                self.messages.append(message)

        async def scenario():
            manager = WebSocketManager()
            first, second = Client(), Client()
            manager.add(first)
            manager.add(second)
            self.assertFalse(manager.has_point_cloud_clients)
            await manager.broadcast_point_cloud('cloud')
            self.assertEqual(first.messages, [])
            manager.set_point_cloud_enabled(first, True)
            await manager.broadcast_point_cloud('cloud')
            self.assertEqual(first.messages, ['cloud'])
            self.assertEqual(second.messages, [])
            manager.remove(first)
            self.assertFalse(manager.has_point_cloud_clients)

        asyncio.run(scenario())


if __name__ == '__main__':
    unittest.main()
