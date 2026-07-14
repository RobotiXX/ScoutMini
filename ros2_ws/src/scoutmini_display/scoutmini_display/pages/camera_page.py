"""Camera page for the dashboard UI."""

from PyQt5.QtCore import Qt, pyqtSlot as Slot
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class CameraPage(QWidget):
    """Page that subscribes to image topics only while it is visible."""

    def __init__(self, ros_node=None):
        super().__init__()
        self._ros_node = ros_node
        self._subscription = None
        self._active = False
        self._current_topic_name = ''
        self._current_topic_type = ''
        self._latest_pixmap = None
        self._received_frame_count = 0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(16)

        title = QLabel('Stream')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('font-size: 28px; font-weight: bold;')
        layout.addWidget(title)

        row = QHBoxLayout()
        self.topic_combo = QComboBox()
        self.topic_combo.setStyleSheet('font-size: 20px;')
        self.topic_combo.currentIndexChanged.connect(self._on_topic_changed)
        row.addWidget(self.topic_combo, stretch=1)

        self.refresh_button = QPushButton('Refresh topics')
        self.refresh_button.setMinimumHeight(48)
        self.refresh_button.clicked.connect(self.refresh_topics)
        row.addWidget(self.refresh_button)
        layout.addLayout(row)

        self.status_label = QLabel('Stream subscription is off until this page is shown.')
        self.status_label.setWordWrap(True)
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet('font-size: 18px;')
        layout.addWidget(self.status_label)

        self.image_label = QLabel('No image yet')
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumHeight(260)
        self.image_label.setStyleSheet('background-color: #111; color: #ddd; font-size: 22px;')
        layout.addWidget(self.image_label, stretch=1)

    def set_ros_node(self, ros_node):
        self._ros_node = ros_node

    def activate(self):
        """Refresh image topics and subscribe to the current selection."""
        self._active = True
        if self._ros_node is not None:
            self._ros_node.get_logger().info('Camera page activated; refreshing image topics.')
        self.refresh_topics()

    def deactivate(self):
        """Unsubscribe from the image topic so the camera stops consuming data."""
        self._active = False
        self._destroy_subscription()
        if self._ros_node is not None:
            self._ros_node.get_logger().info('Stream page deactivated; image subscription removed.')
        self.set_status('Stream subscription is off until this page is shown.')

    def refresh_topics(self, checked=False):
        topics = self._discover_image_topics()
        previous_topic = self._current_topic_name
        self.topic_combo.blockSignals(True)
        self.topic_combo.clear()
        for topic_name, topic_type in topics:
            self.topic_combo.addItem(f'{topic_name}  [{topic_type}]', (topic_name, topic_type))

        chosen_index = -1
        if previous_topic:
            for index in range(self.topic_combo.count()):
                topic_name, _ = self.topic_combo.itemData(index)
                if topic_name == previous_topic:
                    chosen_index = index
                    break
        if chosen_index < 0 and self.topic_combo.count() > 0:
            chosen_index = 0
        if chosen_index >= 0:
            self.topic_combo.setCurrentIndex(chosen_index)
            self._apply_selected_topic(subscribe=self._active)
        else:
            self._current_topic_name = ''
            self._current_topic_type = ''
            self.set_status('No image topics found.')
            self._destroy_subscription()

        self.topic_combo.blockSignals(False)

    def _discover_image_topics(self):
        if self._ros_node is None:
            return []

        topic_entries = []
        seen_names = set()
        for topic_name, topic_types in self._ros_node.get_topic_names_and_types():
            if topic_name in seen_names:
                continue
            seen_names.add(topic_name)
            if any(topic_type in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage') for topic_type in topic_types):
                topic_entries.append((topic_name, topic_types[0] if topic_types else ''))

        return sorted(topic_entries, key=lambda item: item[0])

    def _on_topic_changed(self, index):
        if index < 0:
            return
        self._apply_selected_topic(subscribe=self._active)

    def _apply_selected_topic(self, subscribe=False):
        if self.topic_combo.currentIndex() < 0:
            return

        topic_name, topic_type = self.topic_combo.currentData()
        if topic_name == self._current_topic_name and topic_type == self._current_topic_type:
            return

        self._current_topic_name = topic_name
        self._current_topic_type = topic_type
        self.set_status(f'Selected {topic_name} ({topic_type}).')
        if self._ros_node is not None:
            self._ros_node.get_logger().info(
                f'Stream topic selected: name={topic_name}, type={topic_type}, subscribe={subscribe}'
            )
        self._destroy_subscription()

        if subscribe:
            self._create_subscription()

    def _create_subscription(self):
        if self._ros_node is None or not self._current_topic_name:
            return

        if self._current_topic_type == 'sensor_msgs/msg/CompressedImage':
            msg_type = CompressedImage
        else:
            msg_type = Image

        self._subscription = self._ros_node.create_subscription(
            msg_type,
            self._current_topic_name,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.set_status(f'Subscribed to {self._current_topic_name}.')
        if self._ros_node is not None:
            self._ros_node.get_logger().info(
                f'Subscribed to stream topic {self._current_topic_name} as {msg_type.__name__}.'
            )

    def _destroy_subscription(self):
        if self._ros_node is not None and self._subscription is not None:
            self._ros_node.destroy_subscription(self._subscription)
        self._subscription = None

    def _image_callback(self, msg):
        self._received_frame_count += 1
        if self._ros_node is not None and self._received_frame_count <= 3:
            if isinstance(msg, CompressedImage):
                encoding = 'compressed'
            else:
                encoding = msg.encoding
            self._ros_node.get_logger().info(
                f'Received camera frame #{self._received_frame_count} from {self._current_topic_name} '
                f'(encoding={encoding}).'
            )

        qimage = self._message_to_qimage(msg)
        if qimage is None or qimage.isNull():
            return

        self._render_qimage(qimage)

    def _message_to_qimage(self, msg):
        if isinstance(msg, CompressedImage):
            qimage = QImage.fromData(bytes(msg.data))
            if qimage.isNull() and self._ros_node is not None:
                self._ros_node.get_logger().warning(
                    f'Failed to decode compressed image from {self._current_topic_name}.'
                )
            return qimage

        width = int(msg.width)
        height = int(msg.height)
        encoding = (msg.encoding or '').lower()
        bytes_per_line = int(msg.step)
        raw_bytes = bytes(msg.data)

        if encoding == 'mono8':
            return QImage(raw_bytes, width, height, bytes_per_line, QImage.Format_Grayscale8).copy()
        if encoding == 'rgb8':
            return QImage(raw_bytes, width, height, bytes_per_line, QImage.Format_RGB888).copy()
        if encoding == 'bgr8':
            return QImage(raw_bytes, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        if encoding == 'rgba8':
            return QImage(raw_bytes, width, height, bytes_per_line, QImage.Format_RGBA8888).copy()

        self.set_status(f'Unsupported image encoding: {msg.encoding}')
        return None

    def _render_qimage(self, qimage):
        target_size = self.image_label.size()
        if target_size.width() > 0 and target_size.height() > 0:
            pixmap = QPixmap.fromImage(qimage).scaled(
                target_size,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        else:
            pixmap = QPixmap.fromImage(qimage)

        self._latest_pixmap = pixmap
        self.image_label.setPixmap(pixmap)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._latest_pixmap is not None:
            self.image_label.setPixmap(
                self._latest_pixmap.scaled(
                    self.image_label.size(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )
            )

    def set_status(self, message):
        self.status_label.setText(message)
