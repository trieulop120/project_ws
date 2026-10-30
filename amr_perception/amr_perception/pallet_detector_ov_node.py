#!/usr/bin/env python3
"""
Pallet Detector Node — OpenVINO FP16 backend (Native Engine)
============================================================
Nạp trực tiếp thư mục OpenVINO qua Ultralytics Engine để đảm bảo
độ chính xác 100% giống file best.pt mà vẫn đạt tốc độ ~20 FPS.
"""

import os
import rclpy
from amr_perception.pallet_detector_node import PalletDetectorNode

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class PalletDetectorOVNode(PalletDetectorNode):
    """PalletDetectorNode chạy trên OpenVINO Engine native."""

    def _find_ov_model_dir(self) -> str:
        candidates = []
        if self.model_path:
            candidates.append(self.model_path)
            candidates.append(os.path.join(self.model_path, 'best_openvino_model'))
            candidates.append(os.path.join(os.path.dirname(self.model_path), 'best_openvino_model'))
        try:
            from ament_index_python.packages import get_package_share_directory
            candidates.append(os.path.join(
                get_package_share_directory('amr_perception'),
                'weights', 'best_openvino_model'))
        except Exception:
            pass
        candidates.append(os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'weights', 'best_openvino_model'))

        for d in candidates:
            if d and os.path.isdir(d) and os.path.isfile(os.path.join(d, 'best.xml')):
                return d
        return ''

    def _load_model(self):
        """Nạp thư mục OpenVINO IR bằng Ultralytics OpenVINO backend."""
        if YOLO is None:
            self.get_logger().error('Thư viện Ultralytics chưa được cài đặt!')
            return

        model_dir = self._find_ov_model_dir()
        if not model_dir:
            self.get_logger().error(
                'Không tìm thấy mô hình OpenVINO! (Cần thư mục best_openvino_model chứa best.xml & best.bin)')
            return

        try:
            self.get_logger().info(f'Đang nạp mô hình OpenVINO từ: {model_dir}')
            # Nạp trực tiếp thư mục OpenVINO IR, Ultralytics sẽ tự kích hoạt OpenVINO C++ Engine bên dưới
            self.model = YOLO(model_dir, task='segment')
            self.get_logger().info('Nạp thành công OpenVINO FP16 Model!')
        except Exception as e:
            self.get_logger().error(f'Lỗi nạp mô hình OpenVINO: {e}')
            self.model = None


def main(args=None):
    rclpy.init(args=args)
    node = PalletDetectorOVNode()
    if bool(node.get_parameter('mt_executor').value):
        from rclpy.executors import MultiThreadedExecutor
        exec_ = MultiThreadedExecutor()
        exec_.add_node(node)
        try:
            exec_.spin()
        except KeyboardInterrupt:
            pass
        finally:
            exec_.shutdown()
    else:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()