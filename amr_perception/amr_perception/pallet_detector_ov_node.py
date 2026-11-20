#!/usr/bin/env python3
"""Pallet detector with Ultralytics OpenVINO, forced Intel GPU execution.

This node always uses the Intel GPU for inference. If OpenVINO cannot compile
or run on the GPU, startup fails rather than silently switching to CPU.

The parent PalletDetectorNode still provides all ROS 2 detection, depth,
TF, and mission features. The existing launch file may keep passing
inference_device:=intel:gpu, but doing so is no longer required for this node.
"""

import os
import yaml
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from amr_perception.pallet_detector_node import PalletDetectorNode

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class PalletDetectorOVNode(PalletDetectorNode):
    """Use Ultralytics' native OpenVINO backend on an Intel GPU only."""

    GPU_DEVICE = 'intel:gpu'

    def _force_intel_gpu(self):
        """Override the CPU default inherited from PalletDetectorNode.

        The parent calls _load_model() after declaring and reading the ROS
        inference_device parameter, so this override takes effect before
        warm-up and before all subsequent detection predictions.
        """
        previous_device = getattr(self, 'inference_device', None)
        if previous_device != self.GPU_DEVICE:
            self.get_logger().warning(
                f'Forcing inference_device={self.GPU_DEVICE} '
                f'(previous value: {previous_device!r}); CPU fallback disabled.')
        self.inference_device = self.GPU_DEVICE

        # Keep `ros2 param get ... inference_device` consistent with reality.
        results = self.set_parameters([
            Parameter('inference_device', value=self.GPU_DEVICE)
        ])
        if not results or not results[0].successful:
            reason = results[0].reason if results else 'unknown reason'
            raise RuntimeError(f'Failed to set inference_device parameter: {reason}')

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

    def _warm_up(self):
        """Compile the OpenVINO model and verify actual Intel GPU execution."""
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        self.model.predict(dummy, device=self.GPU_DEVICE, verbose=False)

        predictor = getattr(self.model, 'predictor', None)
        backend = getattr(predictor, 'model', None)
        compiled = getattr(backend, 'ov_compiled_model', None)
        if compiled is None:
            raise RuntimeError(
                'Cannot verify OpenVINO execution: ov_compiled_model is unavailable.')

        execution_devices = list(compiled.get_property('EXECUTION_DEVICES'))
        if not any(str(dev).upper().startswith('GPU') for dev in execution_devices):
            raise RuntimeError(
                'Intel GPU required, but OpenVINO compiled the model on '
                f'{execution_devices}. CPU fallback is not allowed.')

        self.get_logger().info(
            f'OpenVINO GPU warm-up successful '
            f'(requested={self.GPU_DEVICE}, actual={execution_devices})')

    def _load_model(self):
        """Load OpenVINO IR, force GPU, and fail fast if GPU is unavailable."""
        self._force_intel_gpu()

        if YOLO is None:
            raise RuntimeError('Ultralytics is not installed.')

        model_dir = self._find_ov_model_dir()
        if not model_dir:
            raise RuntimeError(
                'OpenVINO model not found. Expected best_openvino_model '
                'directory containing best.xml and best.bin.')

        try:
            self.get_logger().info(f'Loading OpenVINO model: {model_dir}')
            metadata_path = os.path.join(model_dir, 'metadata.yaml')
            task = None
            if os.path.isfile(metadata_path):
                with open(metadata_path, 'r', encoding='utf-8') as stream:
                    metadata = yaml.safe_load(stream) or {}
                task = metadata.get('task')
            self.model = YOLO(model_dir, task=task) if task else YOLO(model_dir)
            self._warm_up()
            self.get_logger().info(
                f'OpenVINO model loaded on Intel GPU (task={task or "auto"}).')
        except Exception as exc:
            self.model = None
            self.get_logger().error(f'Failed to start Intel GPU inference: {exc}')
            raise


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PalletDetectorOVNode()
        if bool(node.get_parameter('mt_executor').value):
            from rclpy.executors import MultiThreadedExecutor
            executor = MultiThreadedExecutor()
            executor.add_node(node)
            try:
                executor.spin()
            finally:
                executor.shutdown()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
