"""Smallest PyQt app used to verify that Qt is working on the robot."""

import sys

from PyQt5.QtWidgets import QApplication, QLabel


def main():
	"""Entry point for `ros2 run scoutmini_display example_no_ros_hello_world`."""
	app = QApplication(sys.argv)
	label = QLabel("Hello World")
	label.show()
	sys.exit(app.exec())


if __name__ == '__main__':
	main()
