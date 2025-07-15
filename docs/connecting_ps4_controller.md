# Connecting PS4 controller via CLI for Scout Mini
Maintainer: Nhat Le 

Our AgileX Scout Mini can be controlled with a PS4 DualShock Controller with `ds4drv` and `ds4_driver` packages

Requirements: our ROS 2 software stack for Scout Mini is installed on the robot's onboard computer

## 1. Pair a new controller to Scout Mini

### 1.1 Get controller MAC address
In a terminal on robot
```bash
$ ds4drv
# expected output: [info][bluetooth] Scanning for devices
```
Hold the PS and SHARE buttons on the controller together for 3 seconds until the light starts double blinking (pairing mode), you should see in the terminal `[info][bluetooth] Found device <MAC_address>`, for example `Found device 84:30:95:2F:E8:CC`.
Copy the controller MAC address for later use

# 1.3 Connect to the controller with `bluetoothctl`

In a terminal on robot
```bash
$ bluetoothctl # you will now enter the bluetooth shell
$ scan on # turn on bluetooth pairing mode, ignore numerous devices you might see
$ trust <MAC_address> # enter the copied MAC address here
$ connect <MAC_address>
$ exit
```
Now the controller is connected to the robot and you should see a solid light on it.

## 2. Reconnect controller to Scout Mini later
Each time you turn on the robot and after bringing up the ROS 2 stack completely, just press the PS button once on the controller, and then it will connect to the robot onboard computer and you can start controlling from there.
