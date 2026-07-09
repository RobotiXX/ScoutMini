"""WiFi page for the multi-page Qt demo."""

import subprocess

from PyQt5.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class WifiPage(QWidget):
    """Page that shows saved WiFi profiles and lets the user reconnect to one."""

    def __init__(self):
        super().__init__()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(18)
        layout.addStretch()

        title = QLabel("Network")
        title.setStyleSheet("font-size: 28px; font-weight: bold;")
        layout.addWidget(title)

        description = QLabel("Only saved NetworkManager Wi-Fi profiles are shown here.")
        description.setStyleSheet("font-size: 18px;")
        layout.addWidget(description)

        self.active_label = QLabel("Active connection: None")
        self.active_label.setStyleSheet("font-size: 22px;")
        layout.addWidget(self.active_label)

        self.network_combo = QComboBox()
        self.network_combo.setMinimumHeight(48)
        self.network_combo.setStyleSheet("font-size: 22px;")
        layout.addWidget(self.network_combo)

        button_row = QHBoxLayout()
        self.refresh_button = QPushButton("Refresh")
        self.connect_button = QPushButton("Connect")
        self.refresh_button.setMinimumHeight(60)
        self.connect_button.setMinimumHeight(60)
        self.refresh_button.setStyleSheet("font-size: 24px;")
        self.connect_button.setStyleSheet("font-size: 24px;")
        self.refresh_button.clicked.connect(self.refresh_networks)
        self.connect_button.clicked.connect(self.connect_selected_network)
        button_row.addWidget(self.refresh_button)
        button_row.addWidget(self.connect_button)
        layout.addLayout(button_row)

        self.status_label = QLabel("Press Refresh to load saved network profiles.")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("font-size: 18px;")
        layout.addWidget(self.status_label)

        layout.addStretch()
        self.connect_button.setEnabled(False)
        self.refresh_networks()

    def set_networks(self, network_names, active_name=None):
        """Replace the combo box items with the available saved Wi-Fi profiles."""
        previous_selection = self.network_combo.currentText().strip()
        self.network_combo.blockSignals(True)
        self.network_combo.clear()
        self.network_combo.addItems(network_names)

        selected_name = None
        if active_name and active_name in network_names:
            selected_name = active_name
        elif previous_selection in network_names:
            selected_name = previous_selection
        elif network_names:
            selected_name = network_names[0]

        if selected_name:
            self.network_combo.setCurrentText(selected_name)

        self.network_combo.blockSignals(False)
        self.connect_button.setEnabled(bool(network_names))

    def set_active_connection(self, active_name):
        """Update the label that shows the currently active connection."""
        if active_name:
            self.active_label.setText(f"Active connection: {active_name}")
        else:
            self.active_label.setText("Active connection: None")

    def set_status(self, message):
        """Show feedback for refresh/connect actions."""
        self.status_label.setText(message)

    def refresh_networks(self):
        """Load saved Wi-Fi profiles from NetworkManager and update the page."""
        known_networks, active_network, status_message = self._get_wifi_profiles()
        self.set_networks(known_networks, active_network)
        self.set_active_connection(active_network)
        self.set_status(status_message)

    def connect_selected_network(self):
        """Bring the currently selected saved Wi-Fi profile up."""
        network_name = self.network_combo.currentText().strip()
        if not network_name:
            self.set_status("Select a saved Wi-Fi profile first.")
            return

        success, message = self._run_nmcli(["connection", "up", network_name])
        self.refresh_networks()
        self.set_status(message)

    def _get_wifi_profiles(self):
        success, output = self._run_nmcli(["-t", "-f", "NAME,TYPE", "connection", "show"])
        if not success:
            return [], None, output

        known_networks = []
        for line in output.splitlines():
            if not line.strip() or ":" not in line:
                continue
            name, connection_type = line.split(":", 1)
            if connection_type.strip() == "802-11-wireless":
                known_networks.append(name.strip())

        active_success, active_output = self._run_nmcli(["-t", "-f", "NAME,TYPE", "connection", "show", "--active"])
        active_network = None
        if active_success:
            for line in active_output.splitlines():
                if not line.strip() or ":" not in line:
                    continue
                name, connection_type = line.split(":", 1)
                if connection_type.strip() == "802-11-wireless":
                    active_network = name.strip()
                    break

        if known_networks:
            status_message = f"Found {len(known_networks)} saved network profiles."
        else:
            status_message = "No saved network profiles found."

        return known_networks, active_network, status_message

    def _run_nmcli(self, args):
        try:
            completed = subprocess.run(
                ["nmcli", *args],
                check=False,
                capture_output=True,
                text=True,
                timeout=15,
            )
        except FileNotFoundError:
            return False, "nmcli is not installed on this system."
        except subprocess.TimeoutExpired:
            return False, "nmcli timed out while talking to NetworkManager."

        output = (completed.stdout or completed.stderr or "").strip()
        if completed.returncode == 0:
            return True, output

        if not output:
            output = f"nmcli failed with exit code {completed.returncode}."
        return False, output
