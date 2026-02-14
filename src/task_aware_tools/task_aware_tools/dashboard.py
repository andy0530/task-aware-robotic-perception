import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import json


@dataclass
class StreamStats:
    last_ros_time: Optional[Time] = None
    last_wall_time: Optional[float] = None
    fps: float = 0.0
    width: int = 0
    height: int = 0
    encoding: str = ""
    frame_id: str = ""

@dataclass
class CameraInfoStats:
    last_ros_time: Optional[Time] = None
    last_wall_time: Optional[float] = None
    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    ok: bool = False
    
@dataclass
class YoloStats:
    last_wall_time: Optional[float] = None
    dets: int = 0
    inference_ms: float = 0.0
    inference_ms_ema: float = 0.0
    top_label: str = "N/A"
    top_conf: float = 0.0
    ok: bool = False



class TaskAwareDashboard(Node):
    def __init__(self):
        super().__init__('task_aware_dashboard')

        # ---- Parameters ----
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('fixed_frame', 'world')
        self.declare_parameter('camera_frame', 'fixed_camera/camera_link/rgb_camera')
        self.declare_parameter('refresh_hz', 1.0)
        self.declare_parameter('detections_topic', '/perception/detections')
        self.detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.yolo = YoloStats()

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        refresh_hz = self.get_parameter('refresh_hz').get_parameter_value().double_value

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Stats ----
        self.img = StreamStats()
        self.caminfo = CameraInfoStats()

        # ---- Subscribers ----
        self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, 10)
        self.create_subscription(String, self.detections_topic, self.on_detections, 10)


        # ---- Publisher ----
        self.status_pub = self.create_publisher(String, '/task_aware/status', 10)

        # ---- Timer ----
        period = 1.0 / max(refresh_hz, 0.1)
        self.timer = self.create_timer(period, self.refresh)

        self.get_logger().info(
            f"Dashboard started.\n"
            f"  image_topic: {self.image_topic}\n"
            f"  camera_info_topic: {self.camera_info_topic}\n"
            f"  TF check: {self.fixed_frame} -> {self.camera_frame}\n"
        )

    def on_image(self, msg: Image):
        now_wall = time.time()
        now_ros = self.get_clock().now()

        # FPS estimate from wall time between messages
        if self.img.last_wall_time is not None:
            dt = now_wall - self.img.last_wall_time
            if dt > 0:
                # exponential moving average to smooth
                inst_fps = 1.0 / dt
                self.img.fps = 0.8 * self.img.fps + 0.2 * inst_fps if self.img.fps > 0 else inst_fps

        self.img.last_wall_time = now_wall
        self.img.last_ros_time = now_ros
        self.img.width = msg.width
        self.img.height = msg.height
        self.img.encoding = msg.encoding
        self.img.frame_id = msg.header.frame_id

    def on_camera_info(self, msg: CameraInfo):
        now_wall = time.time()
        now_ros = self.get_clock().now()

        self.caminfo.last_wall_time = now_wall
        self.caminfo.last_ros_time = now_ros

        # K = [fx 0 cx, 0 fy cy, 0 0 1]
        K = msg.k
        fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
        self.caminfo.fx, self.caminfo.fy, self.caminfo.cx, self.caminfo.cy = fx, fy, cx, cy
        self.caminfo.ok = (fx > 0.0 and fy > 0.0)

    def _staleness_ms(self, last_wall: Optional[float]) -> Optional[int]:
        if last_wall is None:
            return None
        return int((time.time() - last_wall) * 1000)

    def _tf_ok(self) -> Tuple[bool, str]:
        try:
            # Use latest available transform
            _ = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.camera_frame,
                Time()
            )
            return True, "OK"
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            return False, str(e).split('\n')[0][:120]

    def _node_exists(self, name_fragment: str) -> bool:
        # Returns True if any node name contains the fragment
        nodes = self.get_node_names()
        return any(name_fragment in n for n in nodes)
    
    def on_detections(self, msg: String):
        self.yolo.last_wall_time = time.time()
        try:
            data = json.loads(msg.data)
            self.yolo.dets = int(data.get("num_detections", 0))
            self.yolo.inference_ms = float(data.get("inference_ms", 0.0))
            self.yolo.inference_ms_ema = float(data.get("inference_ms_ema", 0.0))

            dets = data.get("detections", [])
            if dets:
                # take highest-confidence det
                best = max(dets, key=lambda d: float(d.get("conf", 0.0)))
                self.yolo.top_label = str(best.get("class_name", "N/A"))
                self.yolo.top_conf = float(best.get("conf", 0.0))
            else:
                self.yolo.top_label = "N/A"
                self.yolo.top_conf = 0.0

            self.yolo.ok = True
        except Exception:
            self.yolo.ok = False

    def refresh(self):
        img_stale = self._staleness_ms(self.img.last_wall_time)
        info_stale = self._staleness_ms(self.caminfo.last_wall_time)
        yolo_stale = self._staleness_ms(self.yolo.last_wall_time)
        tf_ok, tf_msg = self._tf_ok()

        # Bridge nodes (best-effort checks)
        has_gz_bridge = self._node_exists('ros_gz_bridge') or self._node_exists('parameter_bridge')

        # Compose status lines
        img_line = (
            f"IMAGE  topic={self.image_topic}  "
            f"fps={self.img.fps:.1f}  stale={img_stale if img_stale is not None else 'N/A'}ms  "
            f"{self.img.width}x{self.img.height} {self.img.encoding}  frame_id={self.img.frame_id or 'N/A'}"
        )
        info_line = (
            f"INFO   topic={self.camera_info_topic}  "
            f"stale={info_stale if info_stale is not None else 'N/A'}ms  "
            f"K: fx={self.caminfo.fx:.1f} fy={self.caminfo.fy:.1f} cx={self.caminfo.cx:.1f} cy={self.caminfo.cy:.1f}  "
            f"ok={'YES' if self.caminfo.ok else 'NO'}"
        )
        tf_line = (
            f"TF     {self.fixed_frame} -> {self.camera_frame}  "
            f"{'OK' if tf_ok else 'FAIL'}"
            f"{'' if tf_ok else f' ({tf_msg})'}"
        )
        sys_line = (
            f"BRIDGE gz_bridge={'YES' if has_gz_bridge else 'NO'}  "
        )
        yolo_line = (
            f"YOLO   topic={self.detections_topic}  "
            f"stale={yolo_stale if yolo_stale is not None else 'N/A'}ms  "
            f"dets={self.yolo.dets}  "
            f"inf={self.yolo.inference_ms:.1f}ms  ema={self.yolo.inference_ms_ema:.1f}ms  "
            f"top={self.yolo.top_label} {self.yolo.top_conf:.2f}  "
            f"ok={'YES' if self.yolo.ok else 'NO'}"
        )

        # Print (clear-ish)
        print("\n" + "=" * 90)
        print("DASHBOARD")
        print(img_line)
        print(info_line)
        print(tf_line)
        print(yolo_line)
        print(sys_line)

        # Publish summary
        summary = String()
        summary.data = f"{img_line} | {info_line} | {tf_line} | {yolo_line} | {sys_line}"
        self.status_pub.publish(summary)



def main():
    rclpy.init()
    node = TaskAwareDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
