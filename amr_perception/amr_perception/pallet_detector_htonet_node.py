#!/usr/bin/env python3
"""
ROS 2 Node: Pallet Detector using HToNet-v5 (CenterNet-style) with OpenVINO Runtime.
Inherits all 3D depth estimation, RANSAC yaw calculation, TF transform,
target locking, and visualization logic from PalletDetectorNode.
"""

import os
import sys
import traceback
import rclpy
import cv2
import numpy as np

try:
    import openvino as ov
except ImportError:
    print("ERROR: OpenVINO runtime not found. Install with: pip install openvino", file=sys.stderr)
    sys.exit(1)

from amr_perception.pallet_detector_node import PalletDetectorNode


class PalletDetectorHTONetNode(PalletDetectorNode):
    STRIDE = 8.0
    IN_W = 640
    IN_H = 480

    # Mapping đã kiểm chứng thực tế trên htonet_v5_pallet_ov:
    # Channel 0: box | Channel 1: hole | Channel 2: pallet
    HTONET_CLASS_NAMES = {
        0: 'box',
        1: 'hole',
        2: 'pallet',
    }

    def __init__(self):
        super().__init__()
        self.get_logger().info(
            f"HTONet-v5 OpenVINO Detector initialized | device={self.ov_device} | "
            f"classes={self.HTONET_CLASS_NAMES}"
        )

    def _load_model(self):
        """Override _load_model() của PalletDetectorNode để nạp HToNet-v5 OpenVINO IR."""
        self.declare_parameter('ov_device', 'GPU')
        self.declare_parameter('topk', 50)
        self.declare_parameter('pallet_conf', -1.0)
        self.declare_parameter('box_conf', -1.0)
        self.declare_parameter('hole_conf', -1.0)

        self.ov_device = str(self.get_parameter('ov_device').value).upper()
        self.topk = int(self.get_parameter('topk').value)

        p_conf = float(self.get_parameter('pallet_conf').value)
        b_conf = float(self.get_parameter('box_conf').value)
        h_conf = float(self.get_parameter('hole_conf').value)
        self.pallet_conf = p_conf if p_conf >= 0.0 else self.conf_threshold
        self.box_conf = b_conf if b_conf >= 0.0 else self.conf_threshold
        self.hole_conf = h_conf if h_conf >= 0.0 else self.conf_threshold

        model_path = self.model_path
        if os.path.isdir(model_path):
            xml_files = [f for f in os.listdir(model_path) if f.endswith('.xml')]
            if not xml_files:
                raise FileNotFoundError(f"No .xml IR file found in directory: {model_path}")
            model_path = os.path.join(model_path, xml_files[0])

        if not os.path.exists(model_path):
            raise FileNotFoundError(f"HTONet OpenVINO model not found at: {model_path}")

        self.get_logger().info(f"Loading HTONet-v5 OpenVINO IR from: {model_path} on [{self.ov_device}]")
        core = ov.Core()
        available_devices = core.available_devices
        self.get_logger().info(f"Available OpenVINO devices: {available_devices}")

        target_device = self.ov_device
        if target_device == 'GPU' and not any('GPU' in d for d in available_devices):
            self.get_logger().warn("GPU not available in OpenVINO, falling back to CPU.")
            target_device = 'CPU'

        ov_model = core.read_model(model=model_path)
        self.compiled_model = core.compile_model(
            model=ov_model,
            device_name=target_device,
            config={'PERFORMANCE_HINT': 'LATENCY'},
        )
        self.infer_request = self.compiled_model.create_infer_request()
        self.output_names = [o.get_any_name() for o in self.compiled_model.outputs]

        dummy_input = np.zeros((1, 3, self.IN_H, self.IN_W), dtype=np.float32)
        self.infer_request.infer({'images': dummy_input})
        self.get_logger().info(f"HTONet-v5 compiled & warmed up successfully on {target_device}.")

    def _preprocess(self, bgr_image: np.ndarray) -> np.ndarray:
        """Tiền xử lý chuẩn: BGR -> RGB, resize (640, 480), normalize [0, 1], NCHW FP32."""
        h, w = bgr_image.shape[:2]
        if (w, h) != (self.IN_W, self.IN_H):
            img = cv2.resize(bgr_image, (self.IN_W, self.IN_H), interpolation=cv2.INTER_LINEAR)
        else:
            img = bgr_image
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        return np.ascontiguousarray(img_rgb.transpose(2, 0, 1)[None])

    def detect_pallets(self, color_image: np.ndarray,
                       use_full_frame: bool = False,
                       retrieval: bool = False):
        """Return detections in the format consumed by PalletDetectorNode."""
        orig_h, orig_w = color_image.shape[:2]
        # Match the parent detector's ROI behaviour. Keep coordinates relative
        # to the original image so depth sampling and camera intrinsics remain valid.
        x0 = y0 = 0
        inference_image = color_image
        if retrieval:
            roi = self._roi_crop(orig_h, orig_w, retrieval=True)
        elif use_full_frame:
            roi = None
        else:
            roi = self._roi_crop(orig_h, orig_w)
        if roi is not None:
            x0, x1, y0, y1 = roi
            inference_image = color_image[y0:y1, x0:x1]

        crop_h, crop_w = inference_image.shape[:2]
        scale_x = crop_w / float(self.IN_W)
        scale_y = crop_h / float(self.IN_H)

        inp = self._preprocess(inference_image)
        self.infer_request.infer({'images': inp})
        outs = {
            name: self.infer_request.get_output_tensor(i).data
            for i, name in enumerate(self.output_names)
        }

        hm = outs['heatmap'][0]              # (3, 60, 80)
        hmax = outs['heatmap_max'][0]      # (3, 60, 80)
        wh = outs['box_wh'][0]               # (2, 60, 80)
        off = outs['subpixel_offset'][0]     # (2, 60, 80)

        min_thr = min(self.pallet_conf, self.box_conf, self.hole_conf)
        peaks = (hm == hmax) & (hm >= min_thr)
        cs, ys, xs = np.nonzero(peaks)
        scores = hm[cs, ys, xs]

        order = np.argsort(-scores)[:self.topk]

        pallets = []
        boxes = []
        holes = []
        all_dets = []

        for idx in order:
            cls_id = int(cs[idx])
            y, x = int(ys[idx]), int(xs[idx])
            conf = float(scores[idx])

            cls_name = self.HTONET_CLASS_NAMES.get(cls_id, 'unknown')
            if cls_name == 'pallet' and conf < self.pallet_conf:
                continue
            elif cls_name == 'box' and conf < self.box_conf:
                continue
            elif cls_name == 'hole' and conf < self.hole_conf:
                continue
            elif cls_name == 'unknown':
                continue

            ox, oy = float(off[0, y, x]), float(off[1, y, x])
            cx = x0 + (x + ox) * self.STRIDE * scale_x
            cy = y0 + (y + oy) * self.STRIDE * scale_y
            w = float(wh[0, y, x]) * self.STRIDE * scale_x
            h = float(wh[1, y, x]) * self.STRIDE * scale_y

            x1 = int(max(0, round(cx - w / 2.0)))
            y1 = int(max(0, round(cy - h / 2.0)))
            x2 = int(min(orig_w - 1, round(cx + w / 2.0)))
            y2 = int(min(orig_h - 1, round(cy + h / 2.0)))

            det = {
                'class_id': cls_id,
                'class': cls_name,
                'conf': conf,
                # HTONet predicts axis-aligned width/height boxes. Express
                # those as the four-corner polygon expected by the parent.
                'box': np.array([[x1, y1], [x2, y1],
                                 [x2, y2], [x1, y2]], dtype=np.float32),
                'center': ((x1 + x2) // 2, (y1 + y2) // 2),
                # The parent uses these pixel coordinates for depth fallback
                # and camera back-projection.
                'u': (x1 + x2) // 2,
                'v': (y1 + y2) // 2,
                'width': x2 - x1,
                'height': y2 - y1,
                'angle': 0.0,
            }
            all_dets.append(det)

            if cls_name == 'pallet':
                pallets.append(det)
            elif cls_name == 'box':
                boxes.append(det)
            elif cls_name == 'hole':
                holes.append(det)

        return all_dets


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PalletDetectorHTONetNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
