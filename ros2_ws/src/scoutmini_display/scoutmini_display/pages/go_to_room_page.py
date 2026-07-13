"""Go-to-room page for the dashboard UI."""

from functools import partial

from PyQt5.QtCore import QRegularExpression, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QRegularExpressionValidator
from PyQt5.QtWidgets import (
    QGridLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


# Touchscreen layout settings. Keep these together so the page can be tuned for
# a different display without searching through the widget construction code.
PAGE_SPACING = 10
STATUS_FONT_SIZE = 24
FEEDBACK_FONT_SIZE = 18
ROOM_INPUT_FONT_SIZE = 28
ROOM_INPUT_PADDING = 8
ROOM_INPUT_MAX_LENGTH = 20
CONTENT_HORIZONTAL_SPACING = 12
CONTENT_VERTICAL_SPACING = 10
BUTTON_HEIGHT = 62
BUILDING_BUTTON_FONT_SIZE = 26
KEYPAD_HORIZONTAL_SPACING = 12
KEYPAD_VERTICAL_SPACING = 10
KEYPAD_BUTTON_WIDTH = 86
KEYPAD_BUTTON_FONT_SIZE = 32
ACTION_BUTTON_FONT_SIZE = 25
MAX_ROOM_DIGITS = 4
TRANSIENT_MESSAGE_TIMEOUT_MS = 10_000


class GoToRoomPage(QWidget):
    """Page that collects a room name and asks the backend to navigate there."""

    room_requested = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.selected_building = 'FUSE'
        self.room_digits = ''
        self.max_room_digits = MAX_ROOM_DIGITS
        self.default_status_text = 'Enter a room number, then press Go.'
        self._transient_status_text = ''

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(PAGE_SPACING)

        self.status_label = QLabel(self.default_status_text)
        self.status_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet(
            f'font-size: {STATUS_FONT_SIZE}px; color: black; font-weight: bold;'
        )
        layout.addWidget(self.status_label)

        self.feedback_label = QLabel('')
        self.feedback_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.feedback_label.setWordWrap(True)
        self.feedback_label.setStyleSheet(
            f'font-size: {FEEDBACK_FONT_SIZE}px; color: #b91c1c; font-weight: bold;'
        )
        layout.addWidget(self.feedback_label)
        self._transient_feedback_text = ''

        self.room_input = QLineEdit()
        self.room_input.setMaxLength(ROOM_INPUT_MAX_LENGTH)
        self.room_input.setAlignment(Qt.AlignCenter)
        self.room_input.setPlaceholderText('Building 0000')
        self.room_input.setStyleSheet(
            f'font-size: {ROOM_INPUT_FONT_SIZE}px; padding: {ROOM_INPUT_PADDING}px;'
        )
        room_pattern = rf'\d{{0,{MAX_ROOM_DIGITS}}}'
        self.room_input.setValidator(QRegularExpressionValidator(QRegularExpression(room_pattern), self))
        self.room_input.setReadOnly(True)
        layout.addWidget(self.room_input)

        content = QGridLayout()
        content.setHorizontalSpacing(CONTENT_HORIZONTAL_SPACING)
        content.setVerticalSpacing(CONTENT_VERTICAL_SPACING)

        self.building_buttons = {}
        building_specs = [
            ('FUSE', True, 0),
            ('Van Metre', False, 1),
            ('Vernon Smith', False, 2),
            ('Hazel', False, 3),
        ]
        for label, enabled, row in building_specs:
            button = QPushButton(label)
            button.setFixedHeight(BUTTON_HEIGHT)
            button.setCheckable(True)
            button.setEnabled(enabled)
            button.setStyleSheet(
                f'QPushButton {{ font-size: {BUILDING_BUTTON_FONT_SIZE}px; }}'
                'QPushButton:checked { background-color: #2d6cdf; color: white; font-weight: bold; }'
                'QPushButton:disabled { background-color: #d1d5db; color: #6b7280; }'
            )
            if enabled:
                button.clicked.connect(partial(self.select_building, label))
            content.addWidget(button, row, 0)
            self.building_buttons[label] = button
            content.setRowMinimumHeight(row, BUTTON_HEIGHT)

        keypad = QGridLayout()
        keypad.setHorizontalSpacing(KEYPAD_HORIZONTAL_SPACING)
        keypad.setVerticalSpacing(KEYPAD_VERTICAL_SPACING)

        digits = [
            ('1', 0, 0), ('2', 0, 1), ('3', 0, 2),
            ('4', 1, 0), ('5', 1, 1), ('6', 1, 2),
            ('7', 2, 0), ('8', 2, 1), ('9', 2, 2),
        ]

        for digit, row, column in digits:
            button = QPushButton(digit)
            button.setFixedSize(KEYPAD_BUTTON_WIDTH, BUTTON_HEIGHT)
            button.setStyleSheet(f'font-size: {KEYPAD_BUTTON_FONT_SIZE}px;')
            button.clicked.connect(partial(self.append_digit, digit))
            keypad.addWidget(button, row, column)

        self.clear_button = QPushButton('Clear')
        self.clear_button.setFixedHeight(BUTTON_HEIGHT)
        self.clear_button.setStyleSheet(f'font-size: {ACTION_BUTTON_FONT_SIZE}px;')
        self.clear_button.clicked.connect(self.clear_room)
        keypad.addWidget(self.clear_button, 3, 0)

        zero_button = QPushButton('0')
        zero_button.setFixedSize(KEYPAD_BUTTON_WIDTH, BUTTON_HEIGHT)
        zero_button.setStyleSheet(f'font-size: {KEYPAD_BUTTON_FONT_SIZE}px;')
        zero_button.clicked.connect(partial(self.append_digit, '0'))
        keypad.addWidget(zero_button, 3, 1)

        self.go_button = QPushButton('Go')
        self.go_button.setFixedHeight(BUTTON_HEIGHT)
        self.go_button.setStyleSheet(
            f'font-size: {ACTION_BUTTON_FONT_SIZE}px; font-weight: bold;'
        )
        self.go_button.clicked.connect(self.request_room)
        keypad.addWidget(self.go_button, 3, 2)

        content.addLayout(keypad, 0, 1, 4, 3)
        layout.addLayout(content)

        layout.addStretch()

        self.select_building('FUSE')
        self._refresh_room_display()

    def append_digit(self, digit):
        if len(self.room_digits) >= self.max_room_digits:
            return
        self.room_digits += digit
        self._refresh_room_display()

    def clear_room(self, checked=False):
        self.room_digits = ''
        self._refresh_room_display()

    def select_building(self, building_name):
        self.selected_building = building_name
        for name, button in self.building_buttons.items():
            button.setChecked(name == building_name)
        self._refresh_room_display()

    def _refresh_room_display(self):
        self.room_input.setText(f'{self.selected_building} {self.room_digits}')

    def request_room(self, checked=False):
        if not self.room_digits:
            self.set_status('Enter a room number first.')
            return

        self.room_requested.emit(f'{self.selected_building}_{self.room_digits}')

    def set_status(self, message):
        if message == 'Waypoint service is not available yet.':
            self.set_transient_feedback(message)
            return

        self._transient_status_text = ''
        self.status_label.setText(message)
        if message.startswith('Requesting navigation to '):
            self.set_transient_status(message)

    def set_transient_status(self, message):
        self._transient_status_text = message
        QTimer.singleShot(
            TRANSIENT_MESSAGE_TIMEOUT_MS,
            lambda: self.clear_transient_status(message),
        )

    def clear_transient_status(self, message=''):
        if message and message != self._transient_status_text:
            return
        self._transient_status_text = ''
        self.status_label.setText(self.default_status_text)

    def set_transient_feedback(self, message):
        self._transient_feedback_text = message
        self.feedback_label.setText(message)
        QTimer.singleShot(
            TRANSIENT_MESSAGE_TIMEOUT_MS,
            lambda: self.clear_transient_feedback(message),
        )

    def clear_transient_feedback(self, message=''):
        if message and message != self._transient_feedback_text:
            return
        self._transient_feedback_text = ''
        self.feedback_label.clear()

    def set_last_room(self, room_name):
        self._transient_status_text = ''
        self.status_label.setText(f'Last requested room: {room_name}')
