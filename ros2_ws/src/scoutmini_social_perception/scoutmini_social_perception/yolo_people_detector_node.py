"""YOLO person detection and panorama-aware tracking for ScoutMini."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import tempfile
import time
from typing import List, Optional

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from .track_colors import track_color
from .track_schema import TrackObservation, TrackReconciler


class YoloPeopleDetector(Node):
    """Detect people, track them with BoT-SORT, and publish typed results."""

    def __init__(self) -> None:
        super().__init__('yolo_people_detector')
        self.declare_parameter('image_topic', '/equirectangular/image')
        self.declare_parameter('input_type', 'raw')
        self.declare_parameter('tracks_topic', '/people/tracks_2d')
        self.declare_parameter('debug_image_topic', '/people/debug_image')
        self.declare_parameter('diagnostics_topic', '/people/detector_diagnostics')
        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('target_fps', 8.0)
        self.declare_parameter('imgsz', 960)
        self.declare_parameter('tracker_config', '')
        self.declare_parameter('reid_model_path', '')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('frame_id_override', '360_link')
        self.declare_parameter('person_class_id', 0)
        self.declare_parameter('track_timeout_sec', 1.0)
        self.declare_parameter('max_reconcile_distance_norm', 0.12)

        self.bridge = CvBridge()
        self.model = None
        self._last_process_ns = 0
        self._model_warning_logged = False
        self._runtime_tracker_config: Optional[Path] = None
        self._reconciler = TrackReconciler(
            timeout_sec=float(self.get_parameter('track_timeout_sec').value),
            max_match_distance_norm=float(
                self.get_parameter('max_reconcile_distance_norm').value
            ),
        )

        image_topic = str(self.get_parameter('image_topic').value)
        input_type = str(self.get_parameter('input_type').value).lower()
        if input_type == 'compressed':
            self.subscription = self.create_subscription(
                CompressedImage,
                image_topic,
                self._compressed_callback,
                qos_profile_sensor_data,
            )
        elif input_type == 'raw':
            self.subscription = self.create_subscription(
                Image,
                image_topic,
                self._raw_callback,
                qos_profile_sensor_data,
            )
        else:
            raise ValueError("input_type must be 'raw' or 'compressed'")

        self.tracks_pub = self.create_publisher(
            Detection2DArray,
            str(self.get_parameter('tracks_topic').value),
            10,
        )
        self.debug_pub = self.create_publisher(
            Image,
            str(self.get_parameter('debug_image_topic').value),
            1,
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('diagnostics_topic').value),
            10,
        )
        self._tracker_config = self._prepare_tracker_config()
        self._load_model()
        self.get_logger().info(
            f'Subscribed to {image_topic} ({input_type}); publishing typed tracks'
        )

    def _load_model(self) -> None:
        model_path = Path(str(self.get_parameter('model_path').value)).expanduser()
        if not str(self.get_parameter('model_path').value).strip():
            self.get_logger().warning('model_path is empty; detector will remain idle')
            return
        if not model_path.is_file():
            self.get_logger().error(f'Model does not exist: {model_path}')
            return
        try:
            from ultralytics import YOLO

            self.model = YOLO(str(model_path), task='detect')
            self._warm_model()
            self.get_logger().info(f'Loaded model: {model_path}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to load model {model_path}: {exc}')

    def _warm_model(self) -> None:
        """Initialize the inference backend before live frames can arrive."""
        image_size = int(self.get_parameter('imgsz').value)
        dummy_image = np.zeros((image_size, image_size, 3), dtype=np.uint8)
        self.model.predict(
            source=dummy_image,
            imgsz=image_size,
            device=str(self.get_parameter('device').value),
            verbose=False,
        )

    def _prepare_tracker_config(self) -> str:
        configured = str(self.get_parameter('tracker_config').value).strip()
        source = (
            Path(configured).expanduser()
            if configured
            else Path(get_package_share_directory('scoutmini_social_perception'))
            / 'config'
            / 'botsort.yaml'
        )
        if not source.is_file():
            raise FileNotFoundError(f'Tracker config does not exist: {source}')

        import yaml

        with source.open(encoding='utf-8') as stream:
            tracker = yaml.safe_load(stream)
        if not isinstance(tracker, dict):
            raise ValueError(f'Tracker config must be a mapping: {source}')

        reid_path = str(self.get_parameter('reid_model_path').value).strip()
        if tracker.get('with_reid') and reid_path:
            model_path = Path(reid_path).expanduser()
            if not model_path.is_file():
                raise FileNotFoundError(f'ReID model does not exist: {model_path}')
            tracker['model'] = str(model_path)
            self.get_logger().info(f'BoT-SORT ReID model: {model_path}')
        elif tracker.get('with_reid'):
            tracker['with_reid'] = False
            tracker['model'] = 'auto'
            self.get_logger().warning(
                'reid_model_path is empty; BoT-SORT ReID is disabled'
            )

        with tempfile.NamedTemporaryFile(
            mode='w',
            prefix='scoutmini_botsort_',
            suffix='.yaml',
            delete=False,
            encoding='utf-8',
        ) as stream:
            yaml.safe_dump(tracker, stream, sort_keys=False)
            self._runtime_tracker_config = Path(stream.name)
        return str(self._runtime_tracker_config)

    def _raw_callback(self, msg: Image) -> None:
        if not self._should_process():
            return
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Image conversion failed: {exc}')
            return
        self._process(image_bgr, msg.header)

    def _compressed_callback(self, msg: CompressedImage) -> None:
        if not self._should_process():
            return
        try:
            image_bgr = self.bridge.compressed_imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8',
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Compressed image conversion failed: {exc}')
            return
        self._process(image_bgr, msg.header)

    def _should_process(self) -> bool:
        if self.model is None:
            if not self._model_warning_logged:
                self.get_logger().warning('No model loaded; skipping images')
                self._model_warning_logged = True
            return False
        target_fps = float(self.get_parameter('target_fps').value)
        now_ns = self.get_clock().now().nanoseconds
        if target_fps > 0.0 and self._last_process_ns:
            if now_ns - self._last_process_ns < int(1e9 / target_fps):
                return False
        self._last_process_ns = now_ns
        return True

    def _process(self, image_bgr, source_header) -> None:
        start = time.perf_counter()
        results = self._run_tracker(image_bgr)
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        if results is None:
            self._publish_diagnostics(source_header, elapsed_ms, 0, False)
            return

        header = deepcopy(source_header)
        frame_override = str(self.get_parameter('frame_id_override').value).strip()
        if frame_override:
            header.frame_id = frame_override

        result = results[0]
        boxes = getattr(result, 'boxes', None)
        observations = self._observations(boxes, image_bgr.shape)
        stamp_sec = float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9
        if stamp_sec <= 0.0:
            stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        public_ids = self._reconciler.reconcile(observations, stamp_sec)

        output = Detection2DArray()
        output.header = header
        if boxes is not None and len(observations):
            xyxy = boxes.xyxy.cpu().numpy().tolist()
            scores = boxes.conf.cpu().numpy().tolist()
            for index, coordinates in enumerate(xyxy):
                output.detections.append(
                    self._make_detection(
                        header,
                        coordinates,
                        float(scores[index]),
                        public_ids[index],
                    )
                )
        self.tracks_pub.publish(output)
        self._publish_debug(image_bgr, output)
        self._publish_diagnostics(
            header,
            elapsed_ms,
            len(output.detections),
            True,
        )

    def _run_tracker(self, image_bgr):
        try:
            return self.model.track(
                image_bgr,
                persist=True,
                classes=[int(self.get_parameter('person_class_id').value)],
                conf=float(self.get_parameter('confidence_threshold').value),
                iou=float(self.get_parameter('iou_threshold').value),
                imgsz=int(self.get_parameter('imgsz').value),
                device=str(self.get_parameter('device').value),
                tracker=self._tracker_config,
                verbose=False,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'YOLO tracking failed: {exc}')
            return None

    @staticmethod
    def _observations(boxes, image_shape) -> List[TrackObservation]:
        if boxes is None or boxes.xyxy is None:
            return []
        height, width = image_shape[:2]
        upstream_ids: List[Optional[str]] = [None] * len(boxes)
        if getattr(boxes, 'id', None) is not None:
            upstream_ids = [
                str(int(value))
                for value in boxes.id.cpu().numpy().tolist()
            ]
        observations = []
        for index, (x1, y1, x2, y2) in enumerate(boxes.xyxy.cpu().numpy().tolist()):
            observations.append(
                TrackObservation(
                    upstream_id=upstream_ids[index],
                    center_x=0.5 * (float(x1) + float(x2)),
                    center_y=0.5 * (float(y1) + float(y2)),
                    image_width=int(width),
                    image_height=int(height),
                )
            )
        return observations

    @staticmethod
    def _make_detection(header, coordinates, score: float, track_id: str) -> Detection2D:
        x1, y1, x2, y2 = [float(value) for value in coordinates]
        detection = Detection2D()
        detection.header = header
        detection.id = track_id
        detection.bbox.center.position.x = 0.5 * (x1 + x2)
        detection.bbox.center.position.y = 0.5 * (y1 + y2)
        detection.bbox.size_x = max(0.0, x2 - x1)
        detection.bbox.size_y = max(0.0, y2 - y1)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = 'person'
        hypothesis.hypothesis.score = score
        detection.results.append(hypothesis)
        return detection

    def _publish_debug(self, image_bgr, tracks: Detection2DArray) -> None:
        if not bool(self.get_parameter('publish_debug_image').value):
            return
        try:
            annotated = image_bgr.copy()
            for detection in tracks.detections:
                center = detection.bbox.center.position
                half_width = 0.5 * detection.bbox.size_x
                half_height = 0.5 * detection.bbox.size_y
                top_left = (
                    max(0, int(round(center.x - half_width))),
                    max(0, int(round(center.y - half_height))),
                )
                bottom_right = (
                    min(annotated.shape[1] - 1, int(round(center.x + half_width))),
                    min(annotated.shape[0] - 1, int(round(center.y + half_height))),
                )
                score = (
                    detection.results[0].hypothesis.score
                    if detection.results else 0.0
                )
                label = f'person {detection.id} {score:.2f}'
                color = track_color(detection.id)
                cv2.rectangle(annotated, top_left, bottom_right, color, 3)
                self._draw_label(annotated, label, top_left, color)
            msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            msg.header = tracks.header
            self.debug_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Debug image publishing failed: {exc}')

    @staticmethod
    def _draw_label(image, label: str, top_left, color) -> None:
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.65
        thickness = 2
        (width, height), baseline = cv2.getTextSize(
            label,
            font,
            scale,
            thickness,
        )
        x = top_left[0]
        y = max(height + baseline + 4, top_left[1])
        cv2.rectangle(
            image,
            (x, y - height - baseline - 4),
            (x + width + 8, y + 2),
            color,
            -1,
        )
        cv2.putText(
            image,
            label,
            (x + 4, y - baseline - 1),
            font,
            scale,
            (0, 0, 0),
            thickness,
            cv2.LINE_AA,
        )

    def _publish_diagnostics(
        self,
        header,
        elapsed_ms: float,
        track_count: int,
        healthy: bool,
    ) -> None:
        array = DiagnosticArray()
        array.header = header
        status = DiagnosticStatus()
        status.name = 'scoutmini_social_perception/detector'
        status.hardware_id = 'insta360_x4'
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
        status.message = 'tracking' if healthy else 'inference failed'
        status.values = [
            KeyValue(key='elapsed_ms', value=f'{elapsed_ms:.3f}'),
            KeyValue(key='track_count', value=str(track_count)),
            KeyValue(key='model_path', value=str(self.get_parameter('model_path').value)),
            KeyValue(key='device', value=str(self.get_parameter('device').value)),
        ]
        array.status.append(status)
        self.diagnostics_pub.publish(array)

    def destroy_node(self):
        """Remove the generated tracker config before shutting down."""
        if self._runtime_tracker_config is not None:
            self._runtime_tracker_config.unlink(missing_ok=True)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = YoloPeopleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
