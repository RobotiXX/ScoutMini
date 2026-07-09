"""Robot status page for the dashboard UI."""

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QFrame,
    QGridLayout,
    QLabel,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)


class StatusPage(QWidget):
    """Page that displays sensor rates, battery, and Jetson power."""

    def __init__(self, ros_node=None):
        super().__init__()
        self._ros_node = ros_node
        self._sensor_rows = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(16)

        title = QLabel('Status')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('font-size: 28px; font-weight: bold; color: #111827;')
        layout.addWidget(title)

        self.summary_label = QLabel('Waiting for status data.')
        self.summary_label.setAlignment(Qt.AlignCenter)
        self.summary_label.setWordWrap(True)
        self.summary_label.setStyleSheet('font-size: 20px; color: #111827;')
        layout.addWidget(self.summary_label)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)

        content = QWidget()
        self.content_layout = QVBoxLayout(content)
        self.content_layout.setSpacing(14)
        self.content_layout.setContentsMargins(0, 0, 0, 0)

        self.sensor_grid = QGridLayout()
        self.sensor_grid.setHorizontalSpacing(12)
        self.sensor_grid.setVerticalSpacing(8)
        self._add_sensor_header()
        self.content_layout.addLayout(self.sensor_grid)

        self.battery_label = self._make_status_box('Battery', 'Waiting for Scout status.')
        self.content_layout.addWidget(self.battery_label)

        self.jetson_label = self._make_status_box('Jetson Power', 'jetson-stats not checked yet.')
        self.content_layout.addWidget(self.jetson_label)
        self.content_layout.addStretch()

        scroll.setWidget(content)
        layout.addWidget(scroll, stretch=1)

        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self.refresh)
        self._refresh_timer.start(1000)

    def set_ros_node(self, ros_node):
        self._ros_node = ros_node

    def refresh(self):
        if self._ros_node is None:
            self.summary_label.setText('No backend connected.')
            return

        snapshot = self._ros_node.get_status_snapshot()
        sensors = snapshot.get('sensors', [])
        failing_count = sum(1 for sensor in sensors if sensor.get('failing'))
        if failing_count:
            self.summary_label.setText(f'{failing_count} sensor monitor(s) need attention.')
            self.summary_label.setStyleSheet('font-size: 20px; color: #b91c1c; font-weight: bold;')
        else:
            self.summary_label.setText('All configured sensor monitors are healthy.')
            self.summary_label.setStyleSheet('font-size: 20px; color: #166534; font-weight: bold;')

        self._update_sensor_rows(sensors)
        self._update_battery(snapshot.get('battery', {}))
        self._update_jetson(snapshot.get('jetson', {}))

    def _add_sensor_header(self):
        headers = ['Sensor', 'Rate', 'Expected', 'State']
        for column, text in enumerate(headers):
            label = QLabel(text)
            label.setStyleSheet('font-size: 18px; font-weight: bold; color: #111827;')
            self.sensor_grid.addWidget(label, 0, column)

    def _update_sensor_rows(self, sensors):
        seen_names = set()
        for sensor in sensors:
            name = sensor.get('name', 'Unnamed')
            seen_names.add(name)
            if name not in self._sensor_rows:
                row = len(self._sensor_rows) + 1
                self._sensor_rows[name] = [self._make_cell() for _ in range(4)]
                for column, label in enumerate(self._sensor_rows[name]):
                    self.sensor_grid.addWidget(label, row, column)

            rate = sensor.get('rate_hz', 0.0)
            expected = sensor.get('expected_rate_hz', 0.0)
            failing = bool(sensor.get('failing'))
            message = sensor.get('message', '')
            state_text = 'FAIL' if failing else 'OK'
            color = '#b91c1c' if failing else '#166534'

            cells = self._sensor_rows[name]
            cells[0].setText(f'{name}\n{sensor.get("topic", "")}')
            cells[1].setText(f'{rate:.1f} Hz')
            cells[2].setText(f'{expected:.1f} Hz')
            cells[3].setText(f'{state_text}\n{message}')
            cells[3].setStyleSheet(f'font-size: 17px; color: {color}; font-weight: bold;')

        for name, cells in self._sensor_rows.items():
            visible = name in seen_names
            for cell in cells:
                cell.setVisible(visible)

    def _update_battery(self, battery):
        percentage = battery.get('percentage')
        voltage = battery.get('voltage')
        message = battery.get('message', 'No battery data.')

        parts = []
        if percentage is not None:
            parts.append(f'{percentage * 100.0:.0f}%')
        if voltage is not None:
            parts.append(f'{voltage:.1f} V')
        if not parts:
            parts.append(message)

        color = '#166534' if battery.get('available') else '#b91c1c'
        self.battery_label.setText(f'Battery: {"  |  ".join(parts)}')
        self.battery_label.setStyleSheet(self._box_style(color))

    def _update_jetson(self, jetson):
        power_watts = jetson.get('power_watts')
        message = jetson.get('message', 'No Jetson power data.')
        if power_watts is None:
            text = f'Jetson Power: {message}'
        else:
            text = f'Jetson Power: {power_watts:.1f} W'

        color = '#166534' if jetson.get('available') else '#6b7280'
        self.jetson_label.setText(text)
        self.jetson_label.setStyleSheet(self._box_style(color))

    def _make_cell(self):
        label = QLabel('')
        label.setWordWrap(True)
        label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        label.setMinimumHeight(48)
        label.setStyleSheet('font-size: 17px; color: #111827;')
        return label

    def _make_status_box(self, title, text):
        label = QLabel(f'{title}: {text}')
        label.setWordWrap(True)
        label.setMinimumHeight(56)
        label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        label.setStyleSheet(self._box_style('#6b7280'))
        return label

    def _box_style(self, color):
        return (
            'font-size: 20px; font-weight: bold; padding: 12px; '
            f'color: {color}; background-color: #f3f4f6; border: 1px solid #d1d5db;'
        )
