#!/usr/bin/env python3
"""
VLA Vision Node
---------------
Runs YOLOv11 on an RGB-D camera stream and publishes detected objects
with 3D positions to /detected_objects.

Subscribes:
  /camera/color/image_raw     (sensor_msgs/Image)
  /camera/depth/image_rect_raw (sensor_msgs/Image)
  /camera/color/camera_info   (sensor_msgs/CameraInfo)

Publishes:
  /detected_objects           (vla_interfaces/ObjectDetection[])
  /vision_debug/image         (sensor_msgs/Image)  -- annotated frame
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from vla_interfaces.msg import ObjectDetection

# Import YOLO — will use ultralytics if available, else mock for testing
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


# ---------------------------------------------------------------------------
# Mock YOLO for testing without a real model
# ---------------------------------------------------------------------------
class MockYOLO:
    """Simulates YOLO output for testing the pipeline without a GPU/model."""
    MOCK_OBJECTS = [
        {"label": "screw",  "conf": 0.91, "box": (120, 180, 60, 40)},
        {"label": "rivet",  "conf": 0.87, "box": (300, 200, 50, 35)},
        {"label": "washer", "conf": 0.78, "box": (480, 160, 55, 30)},
        {"label": "bolt",   "conf": 0.83, "box": (220, 350, 65, 45)},
    ]

    def __call__(self, frame, conf=0.5, verbose=False):
        return [MockResult(self.MOCK_OBJECTS)]


class MockResult:
    def __init__(self, objects):
        self.boxes = MockBoxes(objects)


class MockBoxes:
    def __init__(self, objects):
        self._objects = objects

    def __iter__(self):
        for obj in self._objects:
            yield MockBox(obj)


class MockBox:
    def __init__(self, obj):
        x, y, w, h = obj["box"]
        self.xyxy = np.array([[x, y, x + w, y + h]])
        self.conf = np.array([obj["conf"]])
        self.cls = np.array([0])
        self._label = obj["label"]

    @property
    def label(self):
        return self._label


# ---------------------------------------------------------------------------
# Vision Node
# ---------------------------------------------------------------------------
class VisionNode(Node):

    def __init__(self):
        super().__init__("vision_node")

        # ---- Parameters ----
        self.declare_parameter("model_path", "yolo11n.pt")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("use_mock", not YOLO_AVAILABLE)
        self.declare_parameter("class_filter", ["screw", "rivet", "washer", "bolt", "nut"])

        model_path      = self.get_parameter("model_path").value
        self.conf_thr   = self.get_parameter("conf_threshold").value
        self.use_mock   = self.get_parameter("use_mock").value
        self.class_filter = set(self.get_parameter("class_filter").value)

        # ---- Model ----
        if self.use_mock:
            self.get_logger().warn("YOLO not found — using MockYOLO. Set use_mock:=false for real detection.")
            self.model = MockYOLO()
            self.class_names = {i: name for i, name in enumerate(
                ["screw", "rivet", "washer", "bolt", "nut"])}
        else:
            self.get_logger().info(f"Loading YOLO model: {model_path}")
            self.model = YOLO(model_path)
            self.class_names = self.model.names

        # ---- CV Bridge ----
        self.bridge = CvBridge()

        # ---- Camera intrinsics (filled from CameraInfo) ----
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_info_received = False

        # ---- Publishers ----
        self.pub_objects = self.create_publisher(
            ObjectDetection, "/detected_objects", 10)
        self.pub_debug = self.create_publisher(
            Image, "/vision_debug/image", 10)

        # ---- Subscribers ----
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        if self.use_mock:
            # In mock mode, just run on a timer — no real camera needed
            self.timer = self.create_timer(0.1, self._mock_detection_callback)
        else:
            self.sub_info = self.create_subscription(
                CameraInfo, "/camera/color/camera_info",
                self._camera_info_cb, 10)

            sub_rgb   = Subscriber(self, Image, "/camera/color/image_raw",
                                   qos_profile=qos)
            sub_depth = Subscriber(self, Image, "/camera/depth/image_rect_raw",
                                   qos_profile=qos)
            self.ts = ApproximateTimeSynchronizer(
                [sub_rgb, sub_depth], queue_size=5, slop=0.05)
            self.ts.registerCallback(self._image_callback)

        self.get_logger().info("VisionNode ready.")

    # ------------------------------------------------------------------
    # Camera info
    # ------------------------------------------------------------------
    def _camera_info_cb(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_received = True
            self.get_logger().info(
                f"Camera intrinsics received: fx={self.fx:.1f} fy={self.fy:.1f}")

    # ------------------------------------------------------------------
    # Real RGB-D callback
    # ------------------------------------------------------------------
    def _image_callback(self, rgb_msg: Image, depth_msg: Image):
        try:
            rgb_frame   = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        self._run_detection(rgb_frame, depth_frame)

    # ------------------------------------------------------------------
    # Mock callback (no real camera)
    # ------------------------------------------------------------------
    def _mock_detection_callback(self):
        # Create a blank frame for mock mode
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = np.ones((480, 640), dtype=np.float32) * 0.5  # 50 cm
        self._run_detection(frame, depth)

    # ------------------------------------------------------------------
    # Core detection logic
    # ------------------------------------------------------------------
    def _run_detection(self, rgb_frame: np.ndarray, depth_frame: np.ndarray):
        results = self.model(rgb_frame, conf=self.conf_thr, verbose=False)
        stamp   = self.get_clock().now().to_msg()

        for result in results:
            for box in result.boxes:
                # ---- Label ----
                cls_id = int(box.cls[0])
                if self.use_mock:
                    label = box.label
                else:
                    label = self.class_names.get(cls_id, f"class_{cls_id}")

                if self.class_filter and label not in self.class_filter:
                    continue

                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                # ---- 3D position from depth ----
                cx_px = int((x1 + x2) / 2)
                cy_px = int((y1 + y2) / 2)
                z_m   = self._get_depth_at(depth_frame, cx_px, cy_px)
                x_m, y_m = self._pixel_to_camera(cx_px, cy_px, z_m)

                # ---- Publish ----
                msg = ObjectDetection()
                msg.header = Header()
                msg.header.stamp = stamp
                msg.header.frame_id = "camera_color_optical_frame"
                msg.label      = label
                msg.confidence = conf
                msg.x          = x_m
                msg.y          = y_m
                msg.z          = z_m
                msg.width      = float(x2 - x1)
                msg.height     = float(y2 - y1)

                self.pub_objects.publish(msg)

        # ---- Debug image ----
        annotated = self._draw_detections(rgb_frame.copy(), results)
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            debug_msg.header.stamp = stamp
            self.pub_debug.publish(debug_msg)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _get_depth_at(self, depth: np.ndarray, cx: int, cy: int,
                      window: int = 5) -> float:
        """Median depth in a small window around (cx, cy). Returns metres."""
        h, w = depth.shape[:2]
        x0 = max(0, cx - window)
        x1 = min(w, cx + window)
        y0 = max(0, cy - window)
        y1 = min(h, cy + window)
        patch = depth[y0:y1, x0:x1].astype(float)

        valid = patch[patch > 0]
        if valid.size == 0:
            return 0.5  # fallback 50 cm

        median = float(np.median(valid))
        # RealSense returns mm — convert to m
        return median / 1000.0 if median > 10 else median

    def _pixel_to_camera(self, u: int, v: int, z: float):
        """Back-project pixel to camera X, Y using intrinsics."""
        if not self.camera_info_received or self.fx is None:
            # Rough estimate without intrinsics
            return (u - 320) / 600.0 * z, (v - 240) / 600.0 * z
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return x, y

    def _draw_detections(self, frame, results):
        """Draw bounding boxes and labels on a frame."""
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = box.label if self.use_mock else self.class_names.get(cls_id, "obj")

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 80), 2)
                cv2.putText(frame, f"{label} {conf:.2f}",
                            (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 200, 80), 2)
        return frame


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
