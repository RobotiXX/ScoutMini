"""ScoutMini dashboard application built from the example pages."""

import os
import signal
import sys

import rclpy
from PyQt5.QtCore import Qt, QTimer, pyqtSlot as Slot
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QGridLayout,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from scoutmini_display.dashboard_backend import DashboardBackend
from scoutmini_display.pages import CameraPage, GoToRoomPage, StatusPage, WifiPage


class PinDialog(QDialog):
    """Touchscreen-friendly numeric keypad PIN prompt."""

    def __init__(self, parent=None, page_name='protected page'):
        super().__init__(parent)
        self.setModal(True)
        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)
        self._entered_pin = ''

        layout = QVBoxLayout(self)
        layout.setSpacing(16)
        layout.setContentsMargins(24, 24, 24, 24)
        self.setStyleSheet(
            'QDialog { background-color: white; color: #111827; border: 0px; }'
            'QLabel { color: #111827; }'
        )

        message = QLabel('Enter 4-digit PIN')
        message.setWordWrap(True)
        message.setAlignment(Qt.AlignCenter)
        message.setStyleSheet('font-size: 28px; font-weight: bold;')
        layout.addWidget(message)

        self.pin_display = QLineEdit()
        self.pin_display.setMaxLength(4)
        self.pin_display.setEchoMode(QLineEdit.Password)
        self.pin_display.setAlignment(Qt.AlignCenter)
        self.pin_display.setPlaceholderText('••••')
        self.pin_display.setReadOnly(True)
        self.pin_display.setStyleSheet(
            'font-size: 30px; padding: 12px; background-color: #f3f4f6; color: #111827;'
        )
        layout.addWidget(self.pin_display)

        keypad = QGridLayout()
        keypad.setHorizontalSpacing(14)
        keypad.setVerticalSpacing(14)

        digits = [
            ('1', 0, 0), ('2', 0, 1), ('3', 0, 2),
            ('4', 1, 0), ('5', 1, 1), ('6', 1, 2),
            ('7', 2, 0), ('8', 2, 1), ('9', 2, 2),
        ]

        for digit, row, column in digits:
            button = QPushButton(digit)
            button.setMinimumSize(90, 70)
            button.setStyleSheet('font-size: 34px;')
            button.clicked.connect(lambda checked=False, value=digit: self.append_digit(value))
            keypad.addWidget(button, row, column)

        self.clear_button = QPushButton('Clear')
        self.clear_button.setMinimumHeight(70)
        self.clear_button.setStyleSheet('font-size: 24px;')
        self.clear_button.clicked.connect(self.clear_pin)
        keypad.addWidget(self.clear_button, 3, 0)

        zero_button = QPushButton('0')
        zero_button.setMinimumSize(90, 70)
        zero_button.setStyleSheet('font-size: 34px;')
        zero_button.clicked.connect(lambda checked=False, value='0': self.append_digit(value))
        keypad.addWidget(zero_button, 3, 1)

        self.enter_button = QPushButton('Enter')
        self.enter_button.setMinimumHeight(70)
        self.enter_button.setStyleSheet('font-size: 24px; font-weight: bold;')
        self.enter_button.clicked.connect(self.accept)
        keypad.addWidget(self.enter_button, 3, 2)

        layout.addLayout(keypad)

        self.cancel_button = QPushButton('Cancel')
        self.cancel_button.setMinimumHeight(60)
        self.cancel_button.setStyleSheet('font-size: 24px;')
        self.cancel_button.clicked.connect(self.reject)
        layout.addWidget(self.cancel_button)

        self.pin_display.setFocus()

    def append_digit(self, digit):
        if len(self._entered_pin) >= 4:
            return
        self._entered_pin += digit
        self.pin_display.setText('*' * len(self._entered_pin))

    def clear_pin(self, checked=False):
        self._entered_pin = ''
        self.pin_display.clear()

    def pin(self):
        return self._entered_pin


class RobotDashboard(QWidget):
    """Main controller for page switching and the ROS/Qt event loop."""

    def __init__(self, ros_node):
        super().__init__()
        self._ros_node = ros_node
        self._pin_code = self._load_pin_code()
        self.setWindowTitle("ScoutMini Dashboard")

        self.pages = QStackedWidget()
        self.room_page = GoToRoomPage()
        self.camera_page = CameraPage(self._ros_node)
        self.wifi_page = WifiPage()
        self.status_page = StatusPage(self._ros_node)

        self.pages.addWidget(self.room_page)
        self.pages.addWidget(self.camera_page)
        self.pages.addWidget(self.wifi_page)
        self.pages.addWidget(self.status_page)

        self.room_page.room_requested.connect(self.go_to_room)

        self.page_buttons = [
            self._create_nav_button("Go to Room", self.show_room_page),
            self._create_nav_button("\U0001F512 Stream", self.show_camera_page),
            self._create_nav_button("\U0001F512 Network", self.show_wifi_page),
            self._create_nav_button("\U0001F512 Status", self.show_status_page),
        ]

        footer = QHBoxLayout()
        footer.setSpacing(12)
        for button in self.page_buttons:
            footer.addWidget(button, stretch=1)

        self.pin_feedback_label = QLabel('')
        self.pin_feedback_label.setAlignment(Qt.AlignCenter)
        self.pin_feedback_label.setStyleSheet('color: #b91c1c; font-size: 20px; font-weight: bold;')

        footer_wrapper = QVBoxLayout()
        footer_wrapper.setSpacing(8)
        footer_wrapper.addWidget(self.pin_feedback_label)
        footer_wrapper.addLayout(footer)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.addWidget(self.pages, stretch=1)
        layout.addLayout(footer_wrapper, stretch=0)

        self._current_page_index = 0
        self.show_page(0)
        self.wifi_page.refresh_networks()

        self._ros_spin_timer = QTimer(self)
        self._ros_spin_timer.timeout.connect(self._spin_ros_once)
        self._ros_spin_timer.start(20)

    def _spin_ros_once(self):
        if not rclpy.ok():
            QApplication.quit()
            return

        rclpy.spin_once(self._ros_node, timeout_sec=0.0)

    @Slot(bool)
    def show_room_page(self, checked=False):
        self.show_page(0)

    @Slot(bool)
    def show_camera_page(self, checked=False):
        self._request_protected_page(1, "Stream")

    @Slot(bool)
    def show_wifi_page(self, checked=False):
        self._request_protected_page(2, "Network")

    @Slot(bool)
    def show_status_page(self, checked=False):
        self._request_protected_page(3, "Status")

    def _create_nav_button(self, label, slot):
        button = QPushButton(label)
        button.setMinimumHeight(64)
        button.setMaximumHeight(64)
        button.setCheckable(True)
        button.setStyleSheet(
            "QPushButton { font-size: 24px; }"
            "QPushButton:checked { background-color: #2d6cdf; color: white; font-weight: bold; }"
        )
        button.clicked.connect(slot)
        return button

    def show_page(self, index):
        """Switch visible page and keep page-specific behavior in sync."""
        if self._current_page_index == 1 and index != 1:
            self.camera_page.deactivate()

        self.pages.setCurrentIndex(index)
        self._current_page_index = index
        self._ros_node.get_logger().info(f'Switched dashboard page to index {index}.')

        if index == 1:
            self.camera_page.activate()
        elif index == 2:
            self.wifi_page.refresh_networks()
        elif index == 3:
            self.status_page.refresh()

        for page_index, button in enumerate(self.page_buttons):
            button.setChecked(index == page_index)

    def _sync_nav_buttons(self):
        for page_index, button in enumerate(self.page_buttons):
            button.setChecked(self._current_page_index == page_index)

    def _request_protected_page(self, index, page_name):
        self.pin_feedback_label.clear()
        if self._prompt_for_pin(page_name):
            self._ros_node.get_logger().info(f'PIN accepted for {page_name} page.')
            self.show_page(index)
            return

        self._ros_node.get_logger().warning(f'PIN denied for {page_name} page.')
        self.pin_feedback_label.setText(f'Incorrect PIN for {page_name}.')
        self._sync_nav_buttons()

    def _prompt_for_pin(self, page_name):
        dialog = PinDialog(self, page_name=page_name)
        if dialog.exec() != QDialog.Accepted:
            self._ros_node.get_logger().info(f'PIN dialog cancelled for {page_name} page.')
            return False

        entered_pin = dialog.pin().strip()
        if len(entered_pin) != 4:
            self._ros_node.get_logger().warning(f'Incomplete PIN entered for {page_name} page.')
            return False

        if entered_pin == self._pin_code:
            return True

        return False

    def _load_pin_code(self):
        pin_code = os.environ.get('ROBOT_UI_PIN', '5555').strip()
        if len(pin_code) == 4 and pin_code.isdigit():
            return pin_code

        self._ros_node.get_logger().warning(
            f'Invalid ROBOT_UI_PIN value {pin_code!r}; falling back to 5555.'
        )
        return '5555'

    @Slot(str)
    def go_to_room(self, room_name):
        new_room_name = room_name.replace("_", " ")
        self.room_page.set_status(f'Requesting navigation to {new_room_name}...')
        self._ros_node.get_logger().info(f'Requested navigation to room {new_room_name}.')
        self._ros_node.navigate_to_room(room_name, self.room_page.set_status)


def main(args=None):
    """Entry point for the ScoutMini dashboard application."""
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    ros_node = DashboardBackend()
    window = RobotDashboard(ros_node)

    exit_code = 0

    def handle_sigint(signum, frame):
        nonlocal exit_code
        exit_code = 130
        ros_node.get_logger().info('Ctrl+C received; shutting down dashboard.')
        window.close()
        app.quit()

    signal.signal(signal.SIGINT, handle_sigint)

    window.resize(1024, 600)
    # window.show()
    window.showFullScreen()
    try:
        app_exit_code = app.exec()
        if exit_code == 0:
            exit_code = app_exit_code
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
