"""YOLO person detector for equirectangular ScoutMini camera images."""

from __future__ import annotations

from pathlib import Path
import json
import time
from typing import List, Optional

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from .track_schema import Detection2D, make_image_markers, make_people_frame


class YoloPeopleDetector(Node):
    def __init__(self) -> None:
        super().__init__('yolo_people_detector')
        self.declare_parameter('image_topic', '/equirectangular/image')
        self.declare_parameter('detections_topic', '/people/detections_2d')
        self.declare_parameter('debug_image_topic', '/people/debug_image')
        self.declare_parameter('marker_topic', '/people/detections_2d/markers')
        self.declare_parameter('metrics_topic', '/people/detector_metrics')
        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('target_fps', 8.0)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('tracker_config', 'bytetrack.yaml')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('frame_id', 'camera_360')
        self.declare_parameter('person_class_id', 0)

        self.bridge = CvBridge()
        self.model = None
        self.model_error_logged = False
        self.last_process_time = self.get_clock().now()

        image_topic = str(self.get_parameter('image_topic').value)
        detections_topic = str(self.get_parameter('detections_topic').value)
        debug_image_topic = str(self.get_parameter('debug_image_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)
        metrics_topic = str(self.get_parameter('metrics_topic').value)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(Image, image_topic, self._image_callback, qos)
        self.detections_pub = self.create_publisher(String, detections_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, debug_image_topic, 1)
        self.metrics_pub = self.create_publisher(String, metrics_topic, 10)

        self._load_model()
        self.get_logger().info(f'Subscribed to {image_topic}; publishing detections on {detections_topic}')

    def _load_model(self) -> None:
        model_path = str(self.get_parameter('model_path').value).strip()
        if not model_path:
            self.get_logger().warn('model_path is empty; YOLO detector will idle until configured.')
            return

        path = Path(model_path).expanduser()
        if not path.exists():
            self.get_logger().error(f'YOLO model_path does not exist: {path}')
            return

        try:
            from ultralytics import YOLO

            self.model = YOLO(str(path))
            self.get_logger().info(f'Loaded YOLO model: {path}')
        except Exception as exc:  # noqa: BLE001 - report dependency/model failures cleanly
            self.model = None
            self.get_logger().error(f'Failed to load YOLO model {path}: {exc}')

    def _should_process(self) -> bool:
        target_fps = float(self.get_parameter('target_fps').value)
        if target_fps <= 0.0:
            return True
        now = self.get_clock().now()
        elapsed = (now - self.last_process_time).nanoseconds * 1e-9
        if elapsed < 1.0 / target_fps:
            return False
        self.last_process_time = now
        return True

    def _image_callback(self, msg: Image) -> None:
        if self.model is None:
            if not self.model_error_logged:
                self.get_logger().warn('No YOLO model loaded; skipping image processing.')
                self.model_error_logged = True
            return
        if not self._should_process():
            return

        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        start_time = time.perf_counter()
        detections = self._run_yolo(image_bgr)
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        frame_id = str(self.get_parameter('frame_id').value) or msg.header.frame_id
        people_frame = make_people_frame(msg.header.stamp, frame_id, detections)

        out = String()
        out.data = people_frame.to_json()
        self.detections_pub.publish(out)
        self.marker_pub.publish(
            make_image_markers(msg.header.stamp, frame_id, people_frame.people, 'yolo_detections')
        )
        self._publish_metrics(msg, len(detections), elapsed_ms)

    def _publish_metrics(self, msg: Image, detection_count: int, elapsed_ms: float) -> None:
        metrics = {
            'stamp_sec': float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
            'frame_id': msg.header.frame_id,
            'image_width': int(msg.width),
            'image_height': int(msg.height),
            'detection_count': int(detection_count),
            'elapsed_ms': round(float(elapsed_ms), 3),
            'model_path': str(self.get_parameter('model_path').value),
            'device': str(self.get_parameter('device').value),
            'target_fps': float(self.get_parameter('target_fps').value),
            'imgsz': int(self.get_parameter('imgsz').value),
            'enable_tracking': bool(self.get_parameter('enable_tracking').value),
        }
        out = String()
        out.data = json.dumps(metrics, separators=(',', ':'), sort_keys=True)
        self.metrics_pub.publish(out)

    def _run_yolo(self, image_bgr) -> List[Detection2D]:
        conf = float(self.get_parameter('confidence_threshold').value)
        iou = float(self.get_parameter('iou_threshold').value)
        imgsz = int(self.get_parameter('imgsz').value)
        device = str(self.get_parameter('device').value)
        person_class_id = int(self.get_parameter('person_class_id').value)
        enable_tracking = bool(self.get_parameter('enable_tracking').value)
        tracker_config = str(self.get_parameter('tracker_config').value)

        try:
            if enable_tracking:
                results = self.model.track(
                    image_bgr,
                    persist=True,
                    classes=[person_class_id],
                    conf=conf,
                    iou=iou,
                    imgsz=imgsz,
                    device=device,
                    tracker=tracker_config,
                    verbose=False,
                )
            else:
                results = self.model.predict(
                    image_bgr,
                    classes=[person_class_id],
                    conf=conf,
                    iou=iou,
                    imgsz=imgsz,
                    device=device,
                    verbose=False,
                )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return []

        if not results:
            return []

        result = results[0]
        if bool(self.get_parameter('publish_debug_image').value):
            try:
                plotted = result.plot()
                debug_msg = self.bridge.cv2_to_imgmsg(plotted, encoding='bgr8')
                debug_msg.header.stamp = self.get_clock().now().to_msg()
                debug_msg.header.frame_id = str(self.get_parameter('frame_id').value)
                self.debug_image_pub.publish(debug_msg)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'Failed to publish YOLO debug image: {exc}')

        boxes = getattr(result, 'boxes', None)
        if boxes is None or boxes.xyxy is None:
            return []

        track_ids: List[Optional[int]] = [None] * len(boxes)
        if getattr(boxes, 'id', None) is not None:
            try:
                track_ids = [int(v) for v in boxes.id.cpu().numpy().tolist()]
            except Exception:
                track_ids = [None] * len(boxes)

        detections = []
        height, width = image_bgr.shape[:2]
        xyxy = boxes.xyxy.cpu().numpy().tolist()
        confs = boxes.conf.cpu().numpy().tolist() if boxes.conf is not None else [0.0] * len(xyxy)
        for idx, bbox in enumerate(xyxy):
            detections.append(
                Detection2D(
                    label='person',
                    confidence=float(confs[idx]),
                    bbox_xyxy=[float(v) for v in bbox],
                    image_width=int(width),
                    image_height=int(height),
                    track_id=track_ids[idx] if idx < len(track_ids) else None,
                    source='yolo',
                )
            )
        return detections


def main() -> None:
    rclpy.init()
    node = YoloPeopleDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
