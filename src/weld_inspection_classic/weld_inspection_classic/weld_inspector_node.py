#!/usr/bin/env python3
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
GOOD_LABEL = "Good Welding"

def letterbox(img, size=640):
    h, w = img.shape[:2]
    r = size / max(h, w)
    img = cv2.resize(img, (int(round(w*r)), int(round(h*r))), interpolation=cv2.INTER_LINEAR)
    nh, nw = img.shape[:2]
    top = (size - nh) // 2
    left = (size - nw) // 2
    img = cv2.copyMakeBorder(img, top, size-nh-top, left, size-nw-left, cv2.BORDER_CONSTANT, value=(114,114,114))
    return img, r, (left, top)

def preprocess(bgr):
    img, ratio, pad = letterbox(bgr, 640)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    img = img.transpose(2,0,1)[None] 
    return img, ratio, pad

def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x.astype(np.float64)))

def parse_detections(output0, conf_thresh):
    pred = output0[0].T 
    nc = len(CLASS_LABELS)
    scores = sigmoid(pred[:, 4:4+nc]) 
    class_ids = scores.argmax(axis=1)
    confs = scores.max(axis=1)
    mask = confs >= conf_thresh
    
    detections = []
    for box, cls_id, conf in zip(pred[mask, :4], class_ids[mask], confs[mask]):
        cx, cy, bw, bh = box
        x1, y1 = int(cx - bw/2), int(cy - bh/2)
        x2, y2 = int(cx + bw/2), int(cy + bh/2)
        label = CLASS_LABELS.get(int(cls_id), "Unknown")
        detections.append({
            "class_id": int(cls_id),
            "label": label,
            "confidence": float(conf),
            "box": [x1, y1, x2, y2],
        })
    detections.sort(key=lambda d: d["confidence"], reverse=True)
    return detections


class WeldInspectorNode(Node):
    def __init__(self):
        super().__init__("weld_inspector")
        self.declare_parameter("model_path", str(Path.home() / "Downloads" / "weld_model.onnx"))
        self.declare_parameter("confidence_threshold", 0.25)
        
        model_path = os.path.expanduser(self.get_parameter("model_path").value)
        self.conf_thresh = self.get_parameter("confidence_threshold").value
        
        self._demo_mode = False
        self._load_model(model_path)
        
        self.bridge = CvBridge()
        self._latest_frame = None
        self._frame_lock = threading.Lock()
        self._frame_count = 0
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # Subscribing to the Gazebo Classic camera topic
        self.create_subscription(Image, "/weld_camera/image_raw", self._image_cb, sensor_qos)
        
        self.result_pub = self.create_publisher(String, "/weld_inspection/result", 10)
        self.viz_pub = self.create_publisher(Image, "/weld_inspection/viz", 10)
        
        self.create_timer(0.5, self._tick)

    def _load_model(self, path):
        p = Path(path)
        if not p.exists() or not ONNX_AVAILABLE:
            self.get_logger().warn("Model not found or ONNX missing. Entering DEMO mode.")
            self._demo_mode = True
            return
            
        providers = ["CPUExecutionProvider"]
        self._session = ort.InferenceSession(str(p), providers=providers)
        self._input_name = self._session.get_inputs()[0].name
        self.get_logger().info("YOLO ONNX model loaded.")

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._frame_lock:
                self._latest_frame = frame
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")

    def _tick(self):
        with self._frame_lock:
            frame = self._latest_frame
            
        if frame is None:
            return
            
        if self._demo_mode:
            detections = []
        else:
            inp, _, _ = preprocess(frame)
            outputs = self._session.run(None, {self._input_name: inp})
            detections = parse_detections(outputs[0], self.conf_thresh)
            
        self._frame_count += 1
        top = detections[0] if detections else None
        defective = top["label"] != GOOD_LABEL if top else False
        
        result = {
            "frame": self._frame_count,
            "defective": defective,
            "top_label": top["label"] if top else "none",
            "top_conf": top["confidence"] if top else 0.0
        }
        
        msg = String()
        msg.data = json.dumps(result)
        self.result_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WeldInspectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()