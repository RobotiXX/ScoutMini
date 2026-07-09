# Rohan Work Branch Handoff

This branch collects Rohan's current ScoutMini work so it is easy to inspect,
test, and split into focused pull requests later.

## Included Work Areas

- Remote networking and access notes under `docs/remote_communication/`.
- ZED RTSP/WebRTC streaming helpers and Tailscale notes under `scripts/zed_rtsp/`.
- Slack robot-status and stream-control prototypes under `scripts/slack/` and
  `ros2_ws/src/scoutmini_tasks/`.
- The isolated YOLO people-detection/social-perception package under
  `ros2_ws/src/scoutmini_social_perception/`.

## Branch Policy

Use `rohan/work` as a personal integration branch and admin handoff branch. Do
not treat it as one final review branch. When one direction is ready, cut a
smaller PR branch from this branch or rebuild the focused changes from
`origin/main`.

Suggested future PR slices:

- ZED remote streaming and Tailscale documentation.
- Slack status/command gateway.
- YOLO people detection and social navigation perception scaffold.

## Deliberately Excluded

- Slack tokens, local env files, generated logs, bags, model weights, and build
  artifacts.
- Local route/waypoint experiments and unrelated Nav2 edits.
- Submodule dirty state unless intentionally committed in a future PR.
