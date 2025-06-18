# Usage with a USB CAN adapter (preferred)
sudo modprobe gs_usb
sudo ip link set can2 up type can bitrate 500000

# Usage with a CAN transceiver and Jetson pins
# sudo busybox devmem 0x0c303018 w 0xc458
# sudo busybox devmem 0x0c303010 w 0xc400
# sudo ip link set can0 up type can bitrate 500000
