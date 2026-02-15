import os
import sys
import time
import json
import glob
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge


def _inject_venv_site_packages(venv_path: str) -> None:
    """
    Add a venv's site-packages to sys.path so this node (running under system Python via ros2 run)
    can import ML deps installed in the venv (ultralytics, torch, cv2, etc.) without touching system Python.
    """
    venv_path = os.path.expanduser(venv_path)
    candidates = glob.glob(os.path.join(venv_path, "lib", "python3.*", "site-packages"))
    if not candidates:
        raise RuntimeError(
            f"Could not find site-packages under venv: {venv_path}\n"
            f"Expected something like: {venv_path}/lib/python3.x/site-packages"
        )
    site_pkgs = candidates[0]
    if site_pkgs not in sys.path:
        sys.path.insert(0, site_pkgs)


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_node")

        # ---- Parameters ----
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("debug_topic", "/perception/debug_image")
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("device", "cuda")  # "cuda" or "cpu"
        self.declare_parameter("venv_path", "~/venvs/task_aware_ml")
        self.declare_parameter("publish_debug_image", True)

        self.image_topic = self.get_parameter("image_topic").value
        self.debug_topic = self.get_parameter("debug_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.model_name = self.get_parameter("model").value
        self.conf = float(self.get_parameter("conf").value)
        self.device = self.get_parameter("device").value
        self.venv_path = self.get_parameter("venv_path").value
        self.publish_debug = bool(self.get_parameter("publish_debug_image").value)

        # ---- Bring in ML deps from venv ----
        _inject_venv_site_packages(self.venv_path)

        # Now safe to import ML deps installed in venv
        import cv2  # noqa: F401
        import torch
        from ultralytics import YOLO

        self.cv2 = cv2
        self.torch = torch
        self.YOLO = YOLO

        # ---- Model ----
        self.get_logger().info(f"Loading YOLO model: {self.model_name}")
        self.model = self.YOLO(self.model_name)

        cuda_ok = False
        cuda_name = "N/A"
        try:
            cuda_ok = bool(self.torch.cuda.is_available())
            if cuda_ok:
                cuda_name = self.torch.cuda.get_device_name(0)
        except Exception:
            pass

        self.get_logger().info(
            f"YOLO ready. device_param={self.device}  torch_cuda={cuda_ok}  gpu={cuda_name}"
        )

        # ---- ROS I/O ----
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.pub_debug = self.create_publisher(Image, self.debug_topic, 10)
        self.pub_det = self.create_publisher(String, self.detections_topic, 10)

        # Simple perf stats
        self._ema_ms: Optional[float] = None

    def on_image(self, msg: Image):
        # Convert ROS Image -> OpenCV (BGR)
        try:
            # incoming is rgb8; convert to bgr8 for OpenCV drawing
            cv_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            cv_bgr = self.cv2.cvtColor(cv_rgb, self.cv2.COLOR_RGB2BGR)
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        # Run YOLO
        t0 = time.perf_counter()
        try:
            results = self.model.predict(
                source=cv_bgr,
                conf=self.conf,
                device=self.device,
                verbose=False
            )
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return
        dt_ms = (time.perf_counter() - t0) * 1000.0
        self._ema_ms = dt_ms if self._ema_ms is None else (0.85 * self._ema_ms + 0.15 * dt_ms)

        # Parse results (ultralytics returns a list; we passed one image)
        r = results[0]
        names = r.names  # dict: class_id -> name

        det_list = []
        if r.boxes is not None and len(r.boxes) > 0:
            boxes_xyxy = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()
            clss = r.boxes.cls.cpu().numpy().astype(int)

            for (x1, y1, x2, y2), conf, cls_id in zip(boxes_xyxy, confs, clss):
                det_list.append({
                    "class_id": int(cls_id),
                    "class_name": str(names.get(int(cls_id), str(cls_id))),
                    "conf": float(conf),
                    "bbox_xyxy": [float(x1), float(y1), float(x2), float(y2)]
                })

        # Publish detections as JSON (v1)
        payload = {
            "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
            "frame_id": msg.header.frame_id,
            "image": {"w": int(msg.width), "h": int(msg.height), "encoding": msg.encoding},
            "inference_ms": float(dt_ms),
            "inference_ms_ema": float(self._ema_ms),
            "num_detections": len(det_list),
            "detections": det_list,
        }
        det_msg = String()
        det_msg.data = json.dumps(payload)
        self.pub_det.publish(det_msg)

        # Draw overlay + publish debug image
        if self.publish_debug:
            overlay = cv_bgr.copy()
            for d in det_list:
                x1, y1, x2, y2 = [int(v) for v in d["bbox_xyxy"]]
                label = f'{d["class_name"]} {d["conf"]:.2f}'
                self.cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
                self.cv2.putText(
                    overlay, label, (x1, max(0, y1 - 8)),
                    self.cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                )

            # BGR -> RGB for ROS image
            overlay_rgb = self.cv2.cvtColor(overlay, self.cv2.COLOR_BGR2RGB)
            out = self.bridge.cv2_to_imgmsg(overlay_rgb, encoding="rgb8")
            out.header = msg.header
            self.pub_debug.publish(out)


def main():
    rclpy.init()
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
