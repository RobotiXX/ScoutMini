# ScoutMini Agent Conventions

These conventions apply when making changes in this repository.

## ROS Package Ownership

- Put domain-specific robot functionality in a dedicated ROS 2 package under
  `ros2_ws/src`.
- Keep `scoutmini_tasks` focused on robot task execution. Do not add unrelated
  integrations such as Slack, streaming, dashboards, or remote access there.
- Put runtime launch files, package config, and package-owned scripts in the
  owning ROS package.
- Keep manual test helpers and local-only tooling under the owning package's
  `test` or tooling area.

## Generated Files and Local State

- Prefer repo-wide ignore rules in the root `.gitignore`.
- Do not add nested `.gitignore` files only to ignore logs, rosbags,
  diagnostics, downloaded binaries, or other generated output.
- Keep credentials and local secrets under `.local/`; never commit tokens.

## Docs and PR Hygiene

- Do not commit PR summaries, review notes, or temporary handoff notes that
  belong in the GitHub PR conversation.
- Keep only durable operator or developer documentation in the repo.
- Use placeholders such as `<robot_ip>` or `<robot_ip_or_tailscale_ip>` in docs
  and scripts instead of lab-specific addresses, unless the value is explicitly
  labeled as historical diagnostic output.

## External Dependencies

- External ROS runtime dependencies used by this workspace should live as
  submodules under `ros2_ws/src` unless there is a clear reason otherwise.
- Follow the existing submodule URL style: `git@github.com:RobotiXX/<repo>.git`.

## PR Feedback Workflow

- Inspect unresolved GitHub review threads before editing.
- Keep fixes on the PR branch they belong to; do not mix unrelated PRs in one
  worktree.
- Address review comments with scoped code or docs changes, and call out any
  comment that needs a GitHub reply rather than a code change.
