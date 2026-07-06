"""PyQt multi-page demo without ROS.

Use this file to learn basic Qt layout/page patterns before adding ROS topics.
"""

import sys
from functools import partial

from PyQt5.QtCore import QRegularExpression, Qt, QTimer, pyqtSlot as Slot
from PyQt5.QtGui import QRegularExpressionValidator
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)


class SecondsCounter(QWidget):
    """Example touchscreen-style UI with page switching and a keypad."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Seconds Counter")

        # Shared state used by widgets on different pages.
        self.seconds = 0
        self.last_room_number = "----"

        # QStackedWidget keeps multiple "pages" and shows one at a time.
        self.pages = QStackedWidget()
        self.counter_page = self.create_counter_page()
        self.keypad_page = self.create_keypad_page()
        self.pages.addWidget(self.counter_page)
        self.pages.addWidget(self.keypad_page)

        self.time_button = QPushButton("Time")
        self.keypad_button = QPushButton("Keypad")
        self.time_button.setMinimumSize(180, 80)
        self.keypad_button.setMinimumSize(180, 80)
        self.time_button.setCheckable(True)
        self.keypad_button.setCheckable(True)
        self.time_button.setStyleSheet(
            "QPushButton { font-size: 36px; }"
            "QPushButton:checked { background-color: #2d6cdf; color: white; font-weight: bold; }"
        )
        self.keypad_button.setStyleSheet(
            "QPushButton { font-size: 36px; }"
            "QPushButton:checked { background-color: #2d6cdf; color: white; font-weight: bold; }"
        )
        self.time_button.clicked.connect(self.show_time_page)
        self.keypad_button.clicked.connect(self.show_keypad_page)

        self.footer = QHBoxLayout()
        self.footer.addWidget(self.time_button, alignment=Qt.AlignLeft)
        self.footer.addStretch()
        self.footer.addWidget(self.keypad_button, alignment=Qt.AlignRight)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.addWidget(self.pages, stretch=1)
        layout.addLayout(self.footer)

        self.show_page(0)

        # Regular UI timer updates the counter every second.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_counter)
        self.timer.start(1000)  # loop period in milliseconds

    def create_counter_page(self):
        """Build the page that shows and edits elapsed seconds."""
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addStretch()

        # The counter stays visually centered while the +/- buttons sit on the same horizontal line.
        self.label = QLabel("Seconds since start: 0")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 64px;")

        self.minus_button = QPushButton("-10")
        self.plus_button = QPushButton("+10")
        self.minus_button.setStyleSheet("font-size: 32px;")
        self.plus_button.setStyleSheet("font-size: 32px;")
        self.minus_button.setMinimumSize(240, 120)
        self.plus_button.setMinimumSize(240, 120)
        self.minus_button.clicked.connect(self.decrease_by_ten)
        self.plus_button.clicked.connect(self.increase_by_ten)

        counter_row = QHBoxLayout()
        counter_row.addWidget(self.minus_button, alignment=Qt.AlignVCenter)
        counter_row.addStretch()
        counter_row.addWidget(self.label, alignment=Qt.AlignCenter)
        counter_row.addStretch()
        counter_row.addWidget(self.plus_button, alignment=Qt.AlignVCenter)

        layout.addLayout(counter_row)

        layout.addStretch()
        return page

    def create_keypad_page(self):
        """Build a numeric keypad page for entering a 4-digit value."""
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(18)
        layout.addStretch()

        # This page collects a 4-digit room number, then stores the last submitted value below the keypad.
        title = QLabel("Enter 4-digit room number")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 28px;")
        layout.addWidget(title)

        self.room_input = QLineEdit()
        self.room_input.setMaxLength(4)
        self.room_input.setAlignment(Qt.AlignCenter)
        self.room_input.setPlaceholderText("____")
        self.room_input.setStyleSheet("font-size: 30px; padding: 10px;")
        self.room_input.setValidator(QRegularExpressionValidator(QRegularExpression(r"\d{0,4}"), self))
        layout.addWidget(self.room_input)

        keypad = QGridLayout()
        keypad.setHorizontalSpacing(16)
        keypad.setVerticalSpacing(16)

        digits = [
            ("1", 0, 0), ("2", 0, 1), ("3", 0, 2),
            ("4", 1, 0), ("5", 1, 1), ("6", 1, 2),
            ("7", 2, 0), ("8", 2, 1), ("9", 2, 2),
            ("0", 3, 1),
        ]

        for digit, row, column in digits:
            button = QPushButton(digit)
            button.setFixedSize(90, 70)
            button.setStyleSheet("font-size: 26px;")
            button.clicked.connect(partial(self.append_digit, digit))
            keypad.addWidget(button, row, column)

        layout.addLayout(keypad)

        self.enter_button = QPushButton("Enter")
        self.enter_button.setFixedHeight(64)
        self.enter_button.setStyleSheet("font-size: 24px;")
        self.enter_button.clicked.connect(self.store_room_number)
        layout.addWidget(self.enter_button)

        self.last_room_label = QLabel(f"Last room number entered: {self.last_room_number}")
        self.last_room_label.setAlignment(Qt.AlignCenter)
        self.last_room_label.setStyleSheet("font-size: 24px;")
        layout.addWidget(self.last_room_label)

        layout.addStretch()
        return page

    @Slot(bool)
    def show_time_page(self, checked=False):
        self.show_page(0)

    @Slot(bool)
    def show_keypad_page(self, checked=False):
        self.show_page(1)

    def show_page(self, index):
        """Switch visible page and keep footer buttons in sync."""
        self.pages.setCurrentIndex(index)
        self.time_button.setChecked(index == 0)
        self.keypad_button.setChecked(index == 1)

    @Slot(str, bool)
    def append_digit(self, digit, checked=False):
        if len(self.room_input.text()) < 4:
            self.room_input.setText(self.room_input.text() + digit)

    @Slot()
    def store_room_number(self):
        # Require exactly 4 digits before accepting input.
        room_number = self.room_input.text()
        if len(room_number) != 4:
            return

        self.last_room_number = room_number
        self.last_room_label.setText(f"Last room number entered: {self.last_room_number}")
        self.room_input.clear()

    @Slot()
    def refresh_label(self):
        self.label.setText(f"Seconds since start: {self.seconds}")

    @Slot()
    def increase_by_ten(self):
        self.seconds += 10
        self.refresh_label()

    @Slot()
    def decrease_by_ten(self):
        self.seconds = max(0, self.seconds - 10)
        self.refresh_label()

    @Slot()
    def update_counter(self):
        self.seconds += 1
        self.refresh_label()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SecondsCounter()
    # Use full-screen because this package targets touchscreen robot displays.
    window.showFullScreen()
    sys.exit(app.exec())
