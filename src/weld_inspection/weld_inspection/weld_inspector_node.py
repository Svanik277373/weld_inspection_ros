#!/usr/bin/env python3
"""
weld_inspector_node.py — YOLO26-Segmentation version
Uses sigmoid (not softmax) on YOLO26 class scores for correct confidence.

Class mapping (confirmed from model.names):
  0: Bad Welding          -> REJECT
  1: Crack                -> REJECT
  2: Excess Reinforcement -> REJECT
  3: Good Welding         -> PASS
  4: Porosity             -> REJECT
  5: Spatters             -> REJECT
"""

import json
import os
import time
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge

import cv2
import numpy as np

try:
    import onnxruntime as ort
    ONNX_AVAILABLE = True
except ImportError:
    ONNX_AVAILABLE = False


CLASS_LABELS = {
    0: "Bad Welding",
    1: "Crack",
    2: "Excess Reinforcement",
    3: "Good Welding",
    4: "Porosity",
    5: "Spatters",
}

GOOD_LABEL       = "Good Welding"
DEFECTIVE_CLASSES = {"Bad Welding", "Crack", "Excess Reinforcement", "Porosity", "Spatters"}

CLASS_COLOR = {
    "Bad Welding":          (0,   40, 220),
    "Crack":                (0,   20, 180),
    "Excess Reinforcement": (0,  140, 220),
    "Good Welding":         (50, 200,  50),
    "Porosity":             (20, 100, 210),
    "Spatters":             (20, 160, 220),
}

INPUT_SIZE  = 640
CONF_THRESH = 0.25


def letterbox(img, size=640):
    h, w = img.shape[:2]
    r    = size / max(h, w)
    img  = cv2.resize(img, (int(round(w*r)), int(round(h*r))),
                      interpolation=cv2.INTER_LINEAR)
    nh, nw = img.shape[:2]
    top  = (size - nh) // 2
    left = (size - nw) // 2
    img  = cv2.copyMakeBorder(img, top, size-nh-top, left, size-nw-left,
                               cv2.BORDER_CONSTANT, value=(114,114,114))
    return img, r, (left, top)


def preprocess(bgr):
    img, ratio, pad = letterbox(bgr, INPUT_SIZE)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    img = img.transpose(2,0,1)[None]   # NCHW
    return img, ratio, pad


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x.astype(np.float64)))


def parse_detections(output0, conf_thresh):
    """
    output0: (1, 4+num_classes+32, num_anchors)
    YOLO uses sigmoid on class scores — NOT softmax.
    """
    pred      = output0[0].T                        # (anchors, 4+nc+32)
    nc        = len(CLASS_LABELS)
    scores    = sigmoid(pred[:, 4:4+nc])            # sigmoid for correct 0-1 range
    class_ids = scores.argmax(axis=1)
    confs     = scores.max(axis=1)
    mask      = confs >= conf_thresh

    detections = []
    for box, cls_id, conf in zip(pred[mask, :4], class_ids[mask], confs[mask]):
        cx, cy, bw, bh = box
        x1, y1 = int(cx - bw/2), int(cy - bh/2)
        x2, y2 = int(cx + bw/2), int(cy + bh/2)
        label  = CLASS_LABELS.get(int(cls_id), "Unknown")
        detections.append({
            "class_id":   int(cls_id),
            "label":      label,
            "confidence": float(conf),
            "box":        [x1, y1, x2, y2],
        })

    detections.sort(key=lambda d: d["confidence"], reverse=True)
    return detections


class WeldInspectorNode(Node):

    def __init__(self):
        super().__init__("weld_inspector")

        self.declare_parameter("model_path",
                               str(Path.home() / "weld_model.onnx"))
        self.declare_parameter("confidence_threshold", CONF_THRESH)
        self.declare_parameter("diverter_angle",       -1.30)
        self.declare_parameter("diverter_reset_delay",  2.5)
        self.declare_parameter("debug_display",        False)

        model_path       = os.path.expanduser(
            self.get_parameter("model_path").value)
        self.conf_thresh = self.get_parameter("confidence_threshold").value
        self.div_angle   = self.get_parameter("diverter_angle").value
        self.div_delay   = self.get_parameter("diverter_reset_delay").value
        self.debug_disp  = self.get_parameter("debug_display").value

        self._session   = None
        self._demo_mode = False
        self._load_model(model_path)

        self.bridge        = CvBridge()
        self._latest_frame = None
        self._frame_lock   = threading.Lock()
        self._div_busy     = False
        self._div_lock     = threading.Lock()
        self._frame_count  = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Image, "/weld_camera/image_raw", self._image_cb, sensor_qos)

        self.result_pub   = self.create_publisher(String,  "/weld_inspection/result", 10)
        self.diverter_pub = self.create_publisher(Float64, "/diverter/cmd", 10)
        self.viz_pub      = self.create_publisher(Image,   "/weld_inspection/viz", 10)

        self.create_timer(0.5, self._tick)

        self.get_logger().info("WeldInspectorNode ready")
        if self._demo_mode:
            self.get_logger().warn("DEMO mode — no model loaded")

    def _load_model(self, path):
        p = Path(path)
        if not p.exists():
            self.get_logger().warn("Model not found: " + path + " — DEMO mode")
            self._demo_mode = True
            return
        if not ONNX_AVAILABLE:
            self.get_logger().error("onnxruntime not installed — DEMO mode")
            self._demo_mode = True
            return

        providers = (
            ["CUDAExecutionProvider", "CPUExecutionProvider"]
            if "CUDAExecutionProvider" in ort.get_available_providers()
            else ["CPUExecutionProvider"]
        )
        self._session    = ort.InferenceSession(str(p), providers=providers)
        self._input_name = self._session.get_inputs()[0].name
        self.get_logger().info("YOLO26-seg ONNX model loaded ✓  (" + path + ")")

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._frame_lock:
                self._latest_frame = frame
        except Exception as e:
            self.get_logger().error("CvBridge: " + str(e))

    def _tick(self):
        with self._frame_lock:
            frame = self._latest_frame
        if frame is None:
            return

        detections = self._run_inference(frame)
        self._frame_count += 1

        top = detections[0] if detections else None

        if top:
            defective = top["label"] != GOOD_LABEL
        else:
            defective = True

        result = {
            "frame":      self._frame_count,
            "defective":  defective,
            "top_label":  top["label"]      if top else "none",
            "top_conf":   top["confidence"] if top else 0.0,
            "detections": detections,
            "timestamp":  time.time(),
        }

        msg      = String()
        msg.data = json.dumps(result)
        self.result_pub.publish(msg)

        if top:
            self.get_logger().info(
                "[" + str(self._frame_count) + "] " +
                top["label"] + "  (" +
                str(round(top["confidence"] * 100, 1)) + "%)  " +
                ("DEFECT" if defective else "GOOD")
            )

        if defective:
            self._trigger_diverter()

        self._publish_viz(frame, detections, defective)

    def _run_inference(self, bgr_frame):
        if self._demo_mode:
            idx   = self._frame_count % len(CLASS_LABELS)
            label = CLASS_LABELS[idx]
            return [{"class_id": idx, "label": label,
                     "confidence": 0.91, "box": [160, 120, 480, 360]}]

        inp, _, _ = preprocess(bgr_frame)
        outputs   = self._session.run(None, {self._input_name: inp})
        return parse_detections(outputs[0], self.conf_thresh)

    def _trigger_diverter(self):
        with self._div_lock:
            if self._div_busy:
                return
            self._div_busy = True

        self.get_logger().warn("Diverter activated!")
        cmd      = Float64()
        cmd.data = self.div_angle
        self.diverter_pub.publish(cmd)

        def _reset():
            time.sleep(self.div_delay)
            cmd.data = 0.0
            self.diverter_pub.publish(cmd)
            with self._div_lock:
                self._div_busy = False
            self.get_logger().info("Diverter reset")

        threading.Thread(target=_reset, daemon=True).start()

    def _publish_viz(self, frame, detections, defective):
        try:
            vis = frame.copy()

            for det in detections:
                x1, y1, x2, y2 = det["box"]
                label = det["label"]
                conf  = det["confidence"]
                color = CLASS_COLOR.get(label, (200, 200, 200))
                cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
                tag = label + " " + str(round(conf * 100, 1)) + "%"
                (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                cv2.rectangle(vis, (x1, y1-th-6), (x1+tw+4, y1), color, -1)
                cv2.putText(vis, tag, (x1+2, y1-4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (255, 255, 255), 1, cv2.LINE_AA)

            banner = (0, 35, 200) if defective else (35, 170, 35)
            status = "DEFECT — DIVERTING" if defective else "GOOD WELDING — PASS"
            cv2.rectangle(vis, (0, 0), (vis.shape[1], 40), banner, -1)
            cv2.putText(vis, status, (10, 28),
                        cv2.FONT_HERSHEY_DUPLEX, 0.80,
                        (255, 255, 255), 2, cv2.LINE_AA)

            img_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            self.viz_pub.publish(img_msg)

            if self.debug_disp:
                cv2.imshow("Weld Inspection", vis)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().debug("Viz: " + str(e))


def main(args=None):
    rclpy.init(args=args)
    node = WeldInspectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
