# Scout Remote Communication Docs

This directory bundles the planning and feasibility notes for the Scout remote
communication work.

## Files

- `remote_access_tradeoffs.md`: options, tradeoffs, architecture, and default
  recommendation for remote access, camera streaming, Slack, Zenoh, and the
  later perception/social-navigation track.
- `slack_zenoh_message_contract.md`: first message contract for human-facing
  Slack commands and low-rate robot/backend/robot Zenoh messages.
- `slack_pr_summary.md`: PR-scoped summary, validation, and review notes for
  the Slack/WebRTC communication slice.
- `../../scripts/slack/start_slack_bot.sh`: manual launcher that sources the
  ignored local Slack env file and starts the Slack command bot.
- `../../scripts/slack/control_zed_stream.sh`: Slack-safe ZED stream control
  supervisor for status/start/stop/diagnostics.

## Current Recommendation

Use the currently working network plus Tailscale as the private access layer,
WebRTC for the ZED camera viewer, Slack for human-facing status and camera
stream control, and Zenoh later for low-rate structured robot/backend
messaging. Eduroam remains a future uplink option after credentials/certificate
access is sorted out.
