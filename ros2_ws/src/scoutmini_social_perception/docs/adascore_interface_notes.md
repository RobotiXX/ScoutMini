# AdaSCoRe Interface Notes

This file records the current integration contract found from inspecting the AdaSCoRe `humble` branch outside this repository in `/tmp/adascore_inspect`.

## Confirmed interface

- AdaSCoRe depends on HuNavSim and `wg-perception/people`.
- The social force model imports `people_msgs.msg.People`.
- AdaSCoRe's Python SFM helper subscribes directly to `/people`.
- AdaSCoRe Nav2 social-window configs also expose a `sensor_interface.people_topic` parameter:
  - simulation config: `people`
  - real config: `/vicon/people`
- The `people_msgs/People` message contains a header and an array of `people_msgs/Person`.
- The `people_msgs/Person` message contains:
  - `name`
  - `geometry_msgs/Point position`
  - `geometry_msgs/Point velocity`
  - `float64 reliability`
  - `string[] tagnames`
  - `string[] tags`

AdaSCoRe's SFM code reads `position.x`, `position.y`, and `position.z` as x, y, yaw. It reads `velocity.x`, `velocity.y`, and `velocity.z` as linear x, linear y, angular z.

## ScoutMini adapter policy

The adapter stays disabled by default and defaults to `output_message_type: json_debug` so this package remains buildable on the robot without `people_msgs` installed.

For real AdaSCoRe operation, switch the adapter to:

```yaml
adascore_people_adapter:
  ros__parameters:
    enabled: true
    output_message_type: people_msgs
    adascore_people_topic: /people
    adascore_frame_id: map
    require_frame_match: true
```

Do this only after installing the AdaSCoRe dependency stack, including `wg-perception/people` on its `ros2` branch.

## Frame requirement

The current perception pipeline projects detections into a configured output frame. AdaSCoRe's social-force code compares person x/y/yaw against the robot pose used by the controller, so the people message must be in the same global/controller frame, normally `map`.

Do not publish base-link-relative projected coordinates to `/people` for AdaSCoRe. The adapter will drop `people_msgs` output when `require_frame_match` is true and the incoming frame does not match `adascore_frame_id`.

The package includes `people_frame_transform`, which converts `/people/projected` into `/people/projected_map` with TF. The AdaSCoRe adapter launch includes that node and feeds the transformed topic into the adapter.

## Dry run

Use the dry-run launch to verify the ScoutMini-side data path without camera input, YOLO, `people_msgs`, AdaSCoRe, or robot motion:

```bash
ros2 launch scoutmini_social_perception adascore_dry_run.launch.py
```

The launch publishes fake projected people in `adascore_dry_base_link`, starts an identity static transform to `adascore_dry_map`, transforms the people onto `/people/projected_map`, and mirrors the adapter output as JSON on `/adascore/dry_run/people`. The isolated dry-run frames avoid collisions with a live robot TF tree.

To test the exact AdaSCoRe topic after confirming no real AdaSCoRe controller is consuming it:

```bash
ros2 launch scoutmini_social_perception adascore_dry_run.launch.py adascore_people_topic:=/people
```

To intentionally test the live robot TF path, add `target_frame:=map source_frame:=base_link`.

## Remaining integration work

- Install or vendor AdaSCoRe dependencies in a deliberate integration phase, not inside this package.
- Validate TF availability from the projection frame to `map` on live robot bags and live robot startup.
- Confirm whether the active controller consumes the hard-coded `/people` subscription, the Nav2 `sensor_interface.people_topic`, or both.
- Validate with Nav2 paused before allowing social-window output to command the robot.
