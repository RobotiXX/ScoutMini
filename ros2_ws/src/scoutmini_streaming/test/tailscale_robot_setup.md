# Tailscale Setup for Scout ZED WebRTC

Use this after the WebRTC stream works on the LAN.

This is the only system-level part of the ZED WebRTC path. It installs the
Tailscale package and starts the `tailscaled` daemon. It does not modify ROS,
navigation, robot motion, or the ScoutMini workspace.

## Install

Run from a local terminal where you can enter the `nvidia` sudo password:

```bash
cd "$HOME/repos/ScoutMini"
curl -fsSL https://tailscale.com/install.sh -o /tmp/tailscale-install.sh
sh /tmp/tailscale-install.sh
```

The official installer for Ubuntu 22.04 Jammy adds:

- `/usr/share/keyrings/tailscale-archive-keyring.gpg`
- `/etc/apt/sources.list.d/tailscale.list`
- apt packages: `tailscale`, `tailscale-archive-keyring`
- systemd service: `tailscaled`

Use `<robot_tailscale_ip>` for the private robot address in shared commands and
diagnostic notes.

## Authenticate

After install:

```bash
sudo tailscale up --ssh --hostname=scoutmini-zed
```

Open the printed login URL, authenticate with the approved lab/user account,
then check:

```bash
tailscale status
tailscale ip -4
```

## Test WebRTC over Tailscale

Start the stream on the robot:

```bash
systemctl --user start scoutmini-webrtc.service
```

From a laptop that is also on the same tailnet, open:

```text
http://<robot_tailscale_ip>:8889/zed/
```

If the page loads but video does not start, check whether UDP `8189` is
reachable over the Tailscale path. The MediaMTX HTTP viewer is on TCP `8889`;
WebRTC media negotiation uses ICE on UDP `8189`.

## Diagnostics

```bash
cd "$HOME/repos/ScoutMini"
ros2_ws/src/scoutmini_streaming/scripts/check_zed_stream_stack.sh
```

For a bounded robot-side startup test:

```bash
cd "$HOME/repos/ScoutMini"
ros2_ws/src/scoutmini_streaming/test/smoke_test_zed_webrtc_stack.sh
```

If `tailscale status` shows an iptables `--restore-mark` health warning, keep
testing connectivity first. It may not block this private viewer path. Avoid
changing firewall mode unless the robot cannot be reached over Tailscale.

The browser may label the page as "Not secure" because this first private
viewer uses plain HTTP on the tailnet. That is expected for this phase. Add
HTTPS only if this viewer later needs browser features that require a secure
context or if access expands beyond the private tailnet.
