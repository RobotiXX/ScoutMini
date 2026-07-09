import sys

from PyQt5.QtCore import Qt, QTimer, pyqtSlot as Slot
from PyQt5.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from scoutmini_display.pages import CounterPage, KeypadPage, WifiPage


class SecondsCounter(QWidget):
    """Main controller for the demo pages, interactions, and timer loop."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Seconds Counter")

        # Shared state used by widgets on different pages.
        self.seconds = 0
        self.last_room_number = "----"

        # QStackedWidget keeps multiple "pages" and shows one at a time.
        self.pages = QStackedWidget()
        self.counter_page = CounterPage()
        self.keypad_page = KeypadPage(self.last_room_number)
        self.wifi_page = WifiPage()
        self.pages.addWidget(self.counter_page)
        self.pages.addWidget(self.keypad_page)
        self.pages.addWidget(self.wifi_page)

        # Wire page-local widgets to the controller methods.
        self.counter_page.minus_button.clicked.connect(self.decrease_by_ten)
        self.counter_page.plus_button.clicked.connect(self.increase_by_ten)
        self.keypad_page.enter_button.clicked.connect(self.store_room_number)
        self.keypad_page.digit_pressed.connect(self.append_digit)

        self.page_buttons = [
            self._create_nav_button("Time", self.show_time_page),
            self._create_nav_button("Keypad", self.show_keypad_page),
            self._create_nav_button("WiFi", self.show_wifi_page),
        ]

        self.footer = QHBoxLayout()
        self.footer.addWidget(self.page_buttons[0], alignment=Qt.AlignLeft)
        self.footer.addStretch()
        self.footer.addWidget(self.page_buttons[1], alignment=Qt.AlignCenter)
        self.footer.addStretch()
        self.footer.addWidget(self.page_buttons[2], alignment=Qt.AlignRight)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.addWidget(self.pages, stretch=1)
        layout.addLayout(self.footer)

        self.show_page(0)
        self.wifi_page.refresh_networks()

        # Regular UI timer updates the counter every second.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_counter)
        self.timer.start(1000)  # loop period in milliseconds

    @Slot(bool)
    def show_time_page(self, checked=False):
        self.show_page(0)

    @Slot(bool)
    def show_keypad_page(self, checked=False):
        self.show_page(1)

    @Slot(bool)
    def show_wifi_page(self, checked=False):
        self.show_page(2)
        self.wifi_page.refresh_networks()

    def _create_nav_button(self, label, slot):
        button = QPushButton(label)
        button.setMinimumSize(180, 80)
        button.setCheckable(True)
        button.setStyleSheet(
            "QPushButton { font-size: 36px; }"
            "QPushButton:checked { background-color: #2d6cdf; color: white; font-weight: bold; }"
        )
        button.clicked.connect(slot)
        return button

    def show_page(self, index):
        """Switch visible page and keep footer buttons in sync."""
        self.pages.setCurrentIndex(index)
        for page_index, button in enumerate(self.page_buttons):
            button.setChecked(index == page_index)

    @Slot(str)
    def append_digit(self, digit):
        if len(self.keypad_page.room_input.text()) < 4:
            self.keypad_page.room_input.setText(self.keypad_page.room_input.text() + digit)

    @Slot()
    def store_room_number(self):
        # Require exactly 4 digits before accepting input.
        room_number = self.keypad_page.room_input.text()
        if len(room_number) != 4:
            return

        self.last_room_number = room_number
        self.keypad_page.set_last_room_number(self.last_room_number)
        self.keypad_page.room_input.clear()

    @Slot()
    def refresh_label(self):
        self.counter_page.set_seconds(self.seconds)

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
    window.resize(1024, 600)
    window.show()
    # window.showFullScreen()
    sys.exit(app.exec())
