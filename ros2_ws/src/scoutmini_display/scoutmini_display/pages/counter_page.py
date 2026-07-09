"""Counter page for the multi-page Qt demo."""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget


class CounterPage(QWidget):
    """Page that shows the elapsed seconds and exposes +/- buttons."""

    def __init__(self):
        super().__init__()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addStretch()

        # The controller updates this label with the current elapsed value.
        self.label = QLabel("Seconds since start: 0")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 64px;")

        from PyQt5.QtWidgets import QPushButton

        self.minus_button = QPushButton("-10")
        self.plus_button = QPushButton("+10")
        self.minus_button.setStyleSheet("font-size: 32px;")
        self.plus_button.setStyleSheet("font-size: 32px;")
        self.minus_button.setMinimumSize(240, 120)
        self.plus_button.setMinimumSize(240, 120)

        counter_row = QHBoxLayout()
        counter_row.addWidget(self.minus_button, alignment=Qt.AlignVCenter)
        counter_row.addStretch()
        counter_row.addWidget(self.label, alignment=Qt.AlignCenter)
        counter_row.addStretch()
        counter_row.addWidget(self.plus_button, alignment=Qt.AlignVCenter)

        layout.addLayout(counter_row)
        layout.addStretch()

    def set_seconds(self, seconds):
        """Refresh the label text from the controller state."""
        self.label.setText(f"Seconds since start: {seconds}")