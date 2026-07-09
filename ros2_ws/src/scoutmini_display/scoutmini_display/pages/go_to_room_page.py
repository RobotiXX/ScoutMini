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


class GoToRoomPage(QWidget):
    """Page that collects a room name and asks the backend to navigate there."""

    room_requested = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.selected_building = 'FUSE'
        self.room_digits = ''
        self.max_room_digits = 4
        self.default_status_text = 'Enter a room number, then press Go.'
        self._transient_status_text = ''

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(18)

        self.status_label = QLabel(self.default_status_text)
        self.status_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet('font-size: 26px; color: black; font-weight: bold;')
        layout.addWidget(self.status_label)

        self.feedback_label = QLabel('')
        self.feedback_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.feedback_label.setWordWrap(True)
        self.feedback_label.setStyleSheet('font-size: 18px; color: #b91c1c; font-weight: bold;')
        layout.addWidget(self.feedback_label)
        self._transient_feedback_text = ''

        self.room_input = QLineEdit()
        self.room_input.setMaxLength(20)
        self.room_input.setAlignment(Qt.AlignCenter)
        self.room_input.setPlaceholderText('Building 0000')
        self.room_input.setStyleSheet('font-size: 30px; padding: 10px;')
        self.room_input.setValidator(QRegularExpressionValidator(QRegularExpression(r'\d{0,4}'), self))
        self.room_input.setReadOnly(True)
        layout.addWidget(self.room_input)

        content = QGridLayout()
        content.setHorizontalSpacing(16)
        content.setVerticalSpacing(16)

        self.building_buttons = {}
        building_specs = [
            ('FUSE', True, 0),
            ('Van Metre', False, 1),
            ('Vernon Smith', False, 2),
            ('Hazel', False, 3),
        ]
        for label, enabled, row in building_specs:
            button = QPushButton(label)
            button.setFixedHeight(70)
            button.setCheckable(True)
            button.setEnabled(enabled)
            button.setStyleSheet(
                'QPushButton { font-size: 28px; }'
                'QPushButton:checked { background-color: #2d6cdf; color: white; font-weight: bold; }'
                'QPushButton:disabled { background-color: #d1d5db; color: #6b7280; }'
            )
            if enabled:
                button.clicked.connect(partial(self.select_building, label))
            content.addWidget(button, row, 0)
            self.building_buttons[label] = button
            content.setRowMinimumHeight(row, 70)

        keypad = QGridLayout()
        keypad.setHorizontalSpacing(16)
        keypad.setVerticalSpacing(16)

        digits = [
            ('1', 0, 0), ('2', 0, 1), ('3', 0, 2),
            ('4', 1, 0), ('5', 1, 1), ('6', 1, 2),
            ('7', 2, 0), ('8', 2, 1), ('9', 2, 2),
        ]

        for digit, row, column in digits:
            button = QPushButton(digit)
            button.setFixedSize(90, 70)
            button.setStyleSheet('font-size: 34px;')
            button.clicked.connect(partial(self.append_digit, digit))
            keypad.addWidget(button, row, column)

        self.clear_button = QPushButton('Clear')
        self.clear_button.setFixedHeight(70)
        self.clear_button.setStyleSheet('font-size: 24px;')
        self.clear_button.clicked.connect(self.clear_room)
        keypad.addWidget(self.clear_button, 3, 0)

        zero_button = QPushButton('0')
        zero_button.setFixedSize(90, 70)
        zero_button.setStyleSheet('font-size: 34px;')
        zero_button.clicked.connect(partial(self.append_digit, '0'))
        keypad.addWidget(zero_button, 3, 1)

        self.go_button = QPushButton('Go')
        self.go_button.setFixedHeight(70)
        self.go_button.setStyleSheet('font-size: 24px; font-weight: bold;')
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
        QTimer.singleShot(10000, lambda: self.clear_transient_status(message))

    def clear_transient_status(self, message=''):
        if message and message != self._transient_status_text:
            return
        self._transient_status_text = ''
        self.status_label.setText(self.default_status_text)

    def set_transient_feedback(self, message):
        self._transient_feedback_text = message
        self.feedback_label.setText(message)
        QTimer.singleShot(10000, lambda: self.clear_transient_feedback(message))

    def clear_transient_feedback(self, message=''):
        if message and message != self._transient_feedback_text:
            return
        self._transient_feedback_text = ''
        self.feedback_label.clear()

    def set_last_room(self, room_name):
        self._transient_status_text = ''
        self.status_label.setText(f'Last requested room: {room_name}')
