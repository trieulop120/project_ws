#!/usr/bin/env python3
"""
Pallet Detector Node — OpenVINO INT8 backend
=============================================
Same pipeline as `pallet_detector_node` (TF, depth sampling, yaw fit,
calibration correction, and publishing are ALL inherited unchanged), but the
YOLO inference runs on OpenVINO INT8 (CPU) instead of PyTorch:

    PyTorch 4t  : ~78-120 ms/frame  -> ~8-13 fps
    OV  INT8 CPU: ~47 ms/frame      -> ~21 fps   (measured 2026-10-27, i7-11390H)

Model (pre-exported ultralytics INT8 IR, already in the repo):
    weights/best_int8_openvino_model/{best.xml, best.bin}
    input  float32 [1,3,640,640]
    out[0] [1,39,8400]  = cx,cy,w,h (640-px) | 3 class scores | 32 mask coeffs
    out[1] [1,32,160,160] = mask prototypes

Why "resize" / letterbox:
    The exported IR has a STATIC 640x640 input, but the node feeds it a 570x240
    ROI crop. We cannot feed 570x240 to a static-640 model, so we letterbox:
    aspect-preserving scale to fit 640, then gray-pad (value 114) to 640x640 —
    exactly what ultralytics does internally before PyTorch inference. This is
    a fixed input-resolution requirement of the exported model, not a quality
    loss. Inverse mapping (640-space -> full-frame pixels) is done for boxes
    and masks so depth sampling / intrinsics are unchanged.

Run (dev, from the source tree — no rebuild needed):
    source /opt/ros/humble/setup.bash
    source ~/project_ws/install/setup.bash
    cd ~/project_ws/src/amr_perception
    OMP_NUM_THREADS=4 python3 amr_perception/pallet_detector_ov_node.py

To run it as a ROS executable (`ros2 run amr_perception pallet_detector_ov`),
add the install lines in the "CMake" note below.

Recommended environment (measured): OMP_NUM_THREADS=4 — do NOT pin 8 threads
for a 4-physical-core chip; 8t was measured ~2.6x SLOWER than 4t for PyTorch
and OV 4t ~= OV 8t.

Run only ONE detector at a time: the old PyTorch node and this one publish the
same /pallet/* topics.
"""

import math
import os
import time
import traceback
from typing import List, Tuple

import cv2
import numpy as np

# Import the base class (PyTorch node). Works from the source tree and from the
# install tree (when installed as a package module); falls back to the sibling
# module name when both .py files share a directory.
try:
    from amr_perception.pallet_detector_node import PalletDetectorNode
except ImportError:
    from pallet_detector_node import PalletDetectorNode

import rclpy


OV_MODEL_SUBDIR = 'best_int8_openvino_model'
OV_INPUT = 640                 # static input size of the exported IR
OV_PAD_VAL = 114               # ultralytics letterbox padding value
DEFAULT_CLASS_NAMES = {0: 'box', 1: 'hole', 2: 'pallet'}
MAX_DETECTIONS = 5             # cap mask decodes per frame (speed)


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


class PalletDetectorOVNode(PalletDetectorNode):
    """PalletDetectorNode with OpenVINO INT8 inference (same outputs)."""

    # ------------------------------------------------------------------
    # Model loading (replaces the PyTorch _load_model)
    # ------------------------------------------------------------------
    def _find_ov_model_dir(self) -> str:
        candidates = []
        if self.model_path:
            candidates.append(os.path.join(
                os.path.dirname(self.model_path), OV_MODEL_SUBDIR))
            candidates.append(self.model_path)
        try:
            from ament_index_python.packages import get_package_share_directory
            candidates.append(os.path.join(
                get_package_share_directory('amr_perception'),
                'weights', OV_MODEL_SUBDIR))
        except Exception:
            pass
        candidates.append(os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'weights', OV_MODEL_SUBDIR))
        for d in candidates:
            if d and os.path.isfile(os.path.join(d, 'best.xml')):
                return d
        return ''

    @staticmethod
    def _read_class_names(model_dir: str) -> dict:
        try:
            import yaml
            with open(os.path.join(model_dir, 'metadata.yaml')) as f:
                doc = yaml.safe_load(f) or {}
            return {int(k): str(v) for k, v in (doc.get('names') or {}).items()}
        except Exception:
            return dict(DEFAULT_CLASS_NAMES)

    def _load_model(self):
        """Load the OpenVINO INT8 IR (instead of the PyTorch .pt)."""
        model_dir = self._find_ov_model_dir()
        self._ov_req = None
        if not model_dir:
            self.get_logger().error(
                f'OpenVINO model not found (need weights/{OV_MODEL_SUBDIR}/best.xml). '
                f'Searched share dir and src weights dir.')
            return
        try:
            import openvino as ov
        except ImportError:
            self.get_logger().error('openvino not importable (pip3 install openvino)')
            return
        try:
            core = ov.Core()
            ir = os.path.join(model_dir, 'best.xml')
            model = core.read_model(ir)
            self._ov_cm = core.compile_model(model, 'CPU')
            self._ov_req = self._ov_cm.create_infer_request()
            self._ov_in = self._ov_cm.input(0)
            self._ov_out_head = self._ov_cm.output(0)
            self._ov_out_proto = self._ov_cm.output(1)
            self._ov_names = self._read_class_names(model_dir)
            self._ov_ms = 0.0
            # warmup (JIT + weight load)
            self._ov_req.infer(
                {self._ov_in: np.zeros((1, 3, OV_INPUT, OV_INPUT), dtype=np.float32)})
            self.model = None          # keep the PyTorch branch off
            self.get_logger().info(
                f'OpenVINO INT8 model loaded: {ir}\n'
                f'  in  {model.inputs[0].partial_shape}\n'
                f'  out {model.outputs[0].partial_shape}, '
                f'{model.outputs[1].partial_shape}\n'
                f'  classes: {self._ov_names}')
        except Exception as e:
            self.get_logger().error(f'Failed to load OpenVINO model: {e}')

    # ------------------------------------------------------------------
    # Inference + post-processing (letterbox, NMS, mask decode)
    # ------------------------------------------------------------------
    @staticmethod
    def _letterbox(img: np.ndarray) -> Tuple[np.ndarray, float, int, int, int, int]:
        """Aspect-preserving scale + centered gray pad to 640x640.

        Returns (canvas, s, px, py, placed_w, placed_h):
            s        = scale factor (canvas px per crop px)
            px, py   = top-left of the image region inside the canvas
            placed_w, placed_h = image region size in canvas pixels
        """
        h, w = img.shape[:2]
        s = OV_INPUT / max(h, w)
        pw, ph = int(round(w * s)), int(round(h * s))
        resized = cv2.resize(img, (pw, ph), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((OV_INPUT, OV_INPUT, 3), OV_PAD_VAL, dtype=np.uint8)
        px, py = (OV_INPUT - pw) // 2, (OV_INPUT - ph) // 2
        canvas[py:py + ph, px:px + pw] = resized
        return canvas, s, px, py, pw, ph

    @staticmethod
    def _nms_xyxy(boxes_xyxy: np.ndarray, scores: np.ndarray, iou_thr: float,
                   max_keep: int = MAX_DETECTIONS) -> List[int]:
        """Greedy NMS on (n,4) xyxy boxes. Returns kept positions, desc score."""
        order = np.argsort(scores)[::-1]
        keep = []
        while len(order) > 0 and len(keep) < max_keep:
            i = int(order[0])
            keep.append(i)
            if len(order) == 1:
                break
            rest = order[1:]
            ix1 = np.maximum(boxes_xyxy[i, 0], boxes_xyxy[rest, 0])
            iy1 = np.maximum(boxes_xyxy[i, 1], boxes_xyxy[rest, 1])
            ix2 = np.minimum(boxes_xyxy[i, 2], boxes_xyxy[rest, 2])
            iy2 = np.minimum(boxes_xyxy[i, 3], boxes_xyxy[rest, 3])
            inter = np.clip(ix2 - ix1, 0, None) * np.clip(iy2 - iy1, 0, None)
            a_i = (boxes_xyxy[i, 2] - boxes_xyxy[i, 0]) * \
                  (boxes_xyxy[i, 3] - boxes_xyxy[i, 1])
            a_r = (boxes_xyxy[rest, 2] - boxes_xyxy[rest, 0]) * \
                  (boxes_xyxy[rest, 3] - boxes_xyxy[rest, 1])
            iou = inter / np.maximum(a_i + a_r - inter, 1e-9)
            order = rest[iou <= iou_thr]
        return keep

    def _decode_oriented_boxes(self, coeffs_all: np.ndarray, proto: np.ndarray,
                               cand_keep: np.ndarray, fh: int, fw: int,
                               off_x: int, off_y: int, s: float,
                               px: int, py: int, pw: int, ph: int,
                               crop_w: int, crop_h: int,
                               xyxy_full: np.ndarray
                               ) -> Tuple[list, list]:
        """Return (boxes, angles) in FULL-frame pixels, one OBB each.

        Mirrors the base node's segment branch: mask -> upsample to 640 ->
        crop-region -> crop size -> full frame -> minAreaRect. Falls back to the
        axis-aligned corners if a mask is empty.
        """
        boxes, angles = [], []
        m160 = _sigmoid(coeffs_all[cand_keep]) @ proto     # (K,160,160)
        for k in range(len(cand_keep)):
            m = m160[k] > 0.5
            m640 = cv2.resize(m.astype(np.uint8), (OV_INPUT, OV_INPUT),
                              interpolation=cv2.INTER_NEAREST)
            mlb = m640[py:py + ph, px:px + pw].astype(np.uint8)   # (ph, pw)
            mcrop = cv2.resize(mlb, (crop_w, crop_h),
                               interpolation=cv2.INTER_NEAREST)   # crop px
            mfull = np.zeros((fh, fw), dtype=np.uint8)
            mfull[off_y:off_y + crop_h, off_x:off_x + crop_w] = mcrop * 255
            cnts, _ = cv2.findContours(mfull, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            pts = np.concatenate(cnts, axis=0) if len(cnts) else np.zeros((0,), dtype=np.int32)
            if len(pts) >= 4:
                cr, wh, ang = cv2.minAreaRect(pts)
                if wh[0] > 1e-3 and wh[1] > 1e-3:
                    boxes.append(cv2.boxPoints((cr, wh, ang)))
                    angles.append(math.radians(ang))
                    continue
            boxes.append(xyxy_full[k])        # fallback corners
            angles.append(0.0)
        return boxes, angles

    def detect_pallets(self, color_img: np.ndarray) -> List[dict]:
        """OpenVINO INT8 detection; returns dicts in EXACTLY the same format as
        PalletDetectorNode.detect_pallets (center, angle, conf, class, class_id,
        box, u, v), so compute_pallet_poses() / depth / yaw stay untouched."""
        if self._ov_req is None:
            return []

        roi = self._roi_crop(*color_img.shape[:2])
        fh, fw = color_img.shape[:2]
        if roi is not None:
            x0, x1, y0, y1 = roi
            crop_img = color_img[y0:y1, x0:x1]
            off_x, off_y = x0, y0
        else:
            crop_img, off_x, off_y = color_img, 0, 0
        crop_h, crop_w = crop_img.shape[:2]

        try:
            t0 = time.perf_counter()
            canvas, s, px, py, pw, ph = self._letterbox(crop_img)
            tensor = canvas.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
            res = self._ov_req.infer({self._ov_in: tensor})
            head = res[self._ov_out_head][0].T              # (8400, 39)
            proto = res[self._ov_out_proto][0]              # (32,160,160)
            self._ov_ms = (time.perf_counter() - t0) * 1000.0
            self.get_logger().info(f'OV infer {self._ov_ms:.0f} ms',
                                   throttle_duration_sec=5.0)

            n_cls = len(self._ov_names)
            cx, cy, bw, bh = head[:, 0], head[:, 1], head[:, 2], head[:, 3]
            cls_scores = head[:, 4:4 + n_cls]
            if cls_scores.max() > 1.0:                      # defensive: logits
                cls_scores = _sigmoid(cls_scores)
            conf = cls_scores.max(axis=1)
            cls_id = cls_scores.argmax(axis=1)

            cand = np.nonzero(conf > self.conf_threshold)[0]
            if len(cand) == 0:
                return []

            xyxy640 = np.stack([cx[cand] - bw[cand] / 2, cy[cand] - bh[cand] / 2,
                                cx[cand] + bw[cand] / 2, cy[cand] + bh[cand] / 2],
                               axis=1)
            keep = self._nms_xyxy(xyxy640, conf[cand], self.iou_threshold)
            cand_keep = cand[keep]                           # 8400-space idx

            # inverse letterbox: 640-space -> crop px -> full-frame px
            x1f = (xyxy640[keep, 0] - px) / s + off_x
            x2f = (xyxy640[keep, 2] - px) / s + off_x
            y1f = (xyxy640[keep, 1] - py) / s + off_y
            y2f = (xyxy640[keep, 3] - py) / s + off_y
            xyxy_full = np.stack([x1f, y1f, x2f, y2f], axis=1)
            xyxy_full = np.clip(xyxy_full, [0, 0], [fw - 1, fh - 1])

            coeffs_all = head[:, 7:7 + len(proto)]           # (8400,32)
            boxes, angles = self._decode_oriented_boxes(
                coeffs_all, proto, cand_keep, fh, fw, off_x, off_y,
                s, px, py, pw, ph, crop_w, crop_h, xyxy_full)

            dets = []
            for i in range(len(keep)):
                x1b, y1b, x2b, y2b = xyxy_full[i]
                c = int(cls_id[cand_keep[i]])
                dets.append({
                    'center': ((x1b + x2b) / 2, (y1b + y2b) / 2),
                    'angle': float(angles[i]),
                    'conf': float(conf[cand_keep[i]]),
                    'class': self._ov_names.get(c, 'unknown'),
                    'class_id': c,
                    'box': boxes[i],
                    'u': int((x1b + x2b) / 2),
                    'v': int((y1b + y2b) / 2),
                })
            return dets
        except Exception as e:
            self.get_logger().error(f'OV detection error: {e}\n{traceback.format_exc()}')
            return []


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
