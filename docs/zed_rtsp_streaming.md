# ZED RTSP Stream for Lab Viewers

Goal: expose the onboard ZED RGB feed as an RTSP stream so lab users can view
it from another computer on the lab LAN, or through an SSH tunnel if they have
robot credentials.

Viewer access is one command after the robot stream has been started manually.
Do not expose the RTSP port publicly.

## Quick Start

Robot operator:

1. SSH into the robot as `nvidia`.
2. Run `./scripts/zed_rtsp/start_zed_rtsp_stack.sh`.
3. Tell viewers to open `rtsp://192.168.0.159:8554/zed`.

LAN viewer:

```bash
./scripts/zed_rtsp/view_zed_rtsp_lan.sh 192.168.0.159
```

SSH tunnel viewer:

```bash
./scripts/zed_rtsp/view_zed_rtsp_ssh_tunnel.sh nvidia@192.168.0.159
```

## Known Working State

- Robot lab-router IP: `192.168.0.159`
- Robot hotspot IP, if unchanged: `10.42.0.1`
- Robot ScoutMini repo: `/home/nvidia/repos/ScoutMini`
- RTSP workspace: `/home/nvidia/image2rtsp_ws`
- RTSP URL: `rtsp://192.168.0.159:8554/zed`
- Source topic: `/zed/zed_node/rgb/color/rect/image/compressed`
- Source type: `sensor_msgs/msg/CompressedImage`
- Observed source rate: about `14.9 Hz`
- Observed source bandwidth: about `7.8 MB/s`
- ZED SDK version: `5.2.2`
- ZED camera: ZED 2 on USB, with `/dev/video0` and `/dev/video1`
- One-command robot start script: `./scripts/zed_rtsp/start_zed_rtsp_stack.sh`

The `image2rtsp` configuration should contain:

```yaml
compressed:       True
topic:            "/zed/zed_node/rgb/color/rect/image/compressed"
camera:           False
mountpoint:       "/zed"
port:             "8554"
local_only:       False
```

## Start the Robot Stream Manually

Run this on the robot:

```bash
cd /home/nvidia/repos/ScoutMini
./scripts/zed_rtsp/start_zed_rtsp_stack.sh
```

Leave this terminal running.

The script starts both the ZED ROS wrapper and `image2rtsp`. Expected
`image2rtsp` output includes:

```text
Subscribing to sensor_msgs::msg::CompressedImage
Stream available at rtsp://0.0.0.0:8554/zed
```

If you need to debug the two parts separately, start the ZED wrapper:

```bash
cd /home/nvidia/repos/ScoutMini
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 camera_name:=zed
```

Then start RTSP in another terminal:

```bash
cd /home/nvidia/image2rtsp_ws
source /opt/ros/humble/setup.bash
source /home/nvidia/repos/ScoutMini/ros2_ws/install/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch image2rtsp image2rtsp.launch.py
```

## View on the Lab LAN

Anyone on the same lab network can view the stream if port `8554` is reachable:

```bash
./scripts/zed_rtsp/view_zed_rtsp_lan.sh 192.168.0.159
```

If `ffplay` is missing, the script offers to install `ffmpeg` with apt. Users
can also open the RTSP URL in VLC.

The script uses this low-latency `ffplay` command:

```bash
ffplay -fflags nobuffer -flags low_delay -framedrop -probesize 32 -analyzeduration 0 -sync ext rtsp://<robot-ip>:8554/zed
```

VLC also works:

```bash
vlc rtsp://192.168.0.159:8554/zed
```

If the robot IP changes, find the active address on the robot:

```bash
hostname -I
```

Then replace `192.168.0.159` in the viewer command.

## View Through SSH

Use this when the viewer has SSH credentials but cannot reach port `8554`
directly.

```bash
./scripts/zed_rtsp/view_zed_rtsp_ssh_tunnel.sh nvidia@192.168.0.159
```

If `ffplay` is missing, the script offers to install `ffmpeg` with apt.

The script opens a tunnel on local port `18554`, then plays:

```bash
rtsp://127.0.0.1:18554/zed
```

If local port `18554` is already in use, choose another local port:

```bash
LOCAL_PORT=18555 ./scripts/zed_rtsp/view_zed_rtsp_ssh_tunnel.sh nvidia@192.168.0.159
```

## Check the Stream

Run on the robot:

```bash
cd /home/nvidia/repos/ScoutMini
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ZED_RTSP_TOPIC="/zed/zed_node/rgb/color/rect/image/compressed"

echo "Robot IPs:"
hostname -I

echo
echo "ZED topic:"
ros2 topic info "$ZED_RTSP_TOPIC"
ros2 topic hz "$ZED_RTSP_TOPIC" --window 10

echo
echo "image2rtsp config:"
grep -E "compressed:|topic:|camera:|mountpoint:|port:|local_only:" \
  /home/nvidia/image2rtsp_ws/install/image2rtsp/share/image2rtsp/config/parameters.yaml

echo
echo "RTSP port:"
ss -ltnp | grep ':8554' || true
```

From a viewer laptop, test connectivity:

```bash
nc -vz 192.168.0.159 8554
```

## Permissions

- Runtime start/stop does not require `sudo`.
- The robot user must be able to access the ZED camera. The `nvidia` user is
  expected to be in `video`, `render`, and `zed`.
- A fresh setup needs `sudo` for GStreamer and RTSP development packages.
- LAN viewers need network reachability to `192.168.0.159:8554`.
- SSH tunnel viewers need robot SSH credentials.
- Do not expose port `8554` to the public internet. This RTSP setup does not
  provide authentication.

## Troubleshooting

If the viewer cannot connect:

1. Confirm the robot stream is running.
2. Confirm the robot IP with `hostname -I`.
3. Confirm `local_only: False` in the installed `image2rtsp` config.
4. Confirm port `8554` is listening with `ss -ltnp | grep ':8554'`.
5. From the viewer laptop, run `nc -vz 192.168.0.159 8554`.

If video is delayed:

1. Use the low-latency `ffplay` command from this document.
2. Prefer lab LAN or Ethernet over weak WiFi.
3. Check robot load with `tegrastats`.
4. Consider lowering ZED resolution or FPS before changing GStreamer pipelines.

If `ZED_Diagnostic` fails over SSH with a Qt or `xcb` display error, that is a
GUI display issue. Validate the camera through USB detection and the ROS wrapper
instead.

## Future Tailscale Option

Tailscale can make remote viewing easier later by giving authorized users a
private VPN IP for the robot. This should be admin-approved because it requires
installing and managing a VPN client and access policy on the shared robot.

Once approved and configured, viewers should be able to use the same LAN viewer
command with the robot's Tailscale IP:

```bash
ffplay -fflags nobuffer -flags low_delay -framedrop -probesize 32 -analyzeduration 0 -sync ext rtsp://<robot-tailscale-ip>:8554/zed
```
