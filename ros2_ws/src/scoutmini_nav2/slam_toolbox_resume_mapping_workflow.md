# SLAM Toolbox: resume mapping later and extend the same floor

This note is for **ROS 2 + slam_toolbox** when you already mapped part of a floor, saved it, and want to come back later to map the rest.

## Main idea

With `slam_toolbox`, save two outputs:

1. Occupancy map for Nav2/RViz
   - `*.yaml`
   - `*.pgm` (or image file referenced by yaml)
2. Serialized pose graph for multi-session resume
   - `*.posegraph`

If you only keep `yaml/pgm`, you can localize on that map later, but you cannot cleanly continue the original SLAM graph.

---

## Important warning

If your goal is to **extend the map**, do **not** run slam_toolbox in pure `mode: localization` for that session.

Localization mode is designed to load a serialized map and localize within it using a rolling buffer, where old temporary scans are removed and the underlying map is not permanently updated.

Use normal mapping / continued mapping instead.

---

## Session 1: map the first half and save everything

Start ScoutMini mapping launch (this launch already includes slam_toolbox online async, and by default uses `mapper_params_online_async.yaml`):

```bash
ros2 launch scoutmini_nav2 mapping_2d.launch.py \
  port_name:=can0
```

Drive and map the first part of the floor.

### 1) Save occupancy map (yaml + pgm)

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
"{name: {data: 'src/scoutmini_nav2/maps/floor1_partial'}}"
```

Expected files:

- `src/scoutmini_nav2/maps/floor1_partial.yaml`
- `src/scoutmini_nav2/maps/floor1_partial.pgm`

### 2) Save serialized pose graph

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
"{filename: 'src/scoutmini_nav2/maps/floor1_partial.posegraph'}"
```

---

## Session 2: come back later and continue mapping

Important: use mapping mode again (not pure localization mode) when you want to extend the map.

### Option A (recommended): set map_file_name in a resume params file

1. Copy params file:

```bash
cp src/scoutmini_nav2/config/mapper_params_online_async.yaml \
  src/scoutmini_nav2/config/mapper_params_resume_floor1.yaml
```

2. Edit the new file and set one of the following:

- Start near original dock/start:

```yaml
map_file_name: src/scoutmini_nav2/maps/floor1_partial.posegraph
map_start_at_dock: true
```

- Start at known pose in old map:

```yaml
map_file_name: src/scoutmini_nav2/maps/floor1_partial.posegraph
map_start_pose: [x, y, yaw]
```

3. Launch mapping with resume params:

```bash
ros2 launch scoutmini_nav2 mapping_2d.launch.py \
  port_name:=can0 \
  slam_params_file:=src/scoutmini_nav2/config/mapper_params_resume_floor1.yaml
```

### Option B: deserialize pose graph after startup

1. Launch mapping as usual:

```bash
ros2 launch scoutmini_nav2 mapping_2d.launch.py \
  port_name:=can0
```

2. Load old pose graph by service call.

Start near old dock/start (`match_type: 1`):

```bash
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{
  filename: 'src/scoutmini_nav2/maps/floor1_partial.posegraph',
  match_type: 1,
  initial_pose: {
    position: {x: 0.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

Start at known map pose (`match_type: 2`):

```bash
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{
  filename: 'src/scoutmini_nav2/maps/floor1_partial.posegraph',
  match_type: 2,
  initial_pose: {
    position: {x: 12.3, y: -4.8, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}
  }
}"
```

Then drive through already-mapped overlap first, and only after stable alignment continue to unknown area.

---

## Final save after full floor is mapped

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
"{name: {data: 'src/scoutmini_nav2/maps/floor1_full'}}"

ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
"{filename: 'src/scoutmini_nav2/maps/floor1_full.posegraph'}"
```

---

## Use saved map in your 2D simulation launch

Once you have `floor1_full.yaml`, you can run the RViz-only simulation flow in this repo:

```bash
ros2 launch scoutmini_nav2 simulation_2d.launch.py \
  map:=src/scoutmini_nav2/maps/floor1_full.yaml
```

---

## Verify services on this machine

```bash
ros2 service list | grep slam_toolbox
ros2 service type /slam_toolbox/save_map
ros2 service type /slam_toolbox/serialize_map
ros2 service type /slam_toolbox/deserialize_map
```

Inspect service request fields if needed:

```bash
ros2 interface show slam_toolbox/srv/SaveMap
ros2 interface show slam_toolbox/srv/SerializePoseGraph
ros2 interface show slam_toolbox/srv/DeserializePoseGraph
```

---

## Official references

1. slam_toolbox README
   - https://github.com/SteveMacenski/slam_toolbox

2. Key README points relevant to this workflow
   - Continuing to refine / remap / continue mapping a saved serialized pose graph
   - Multi-session mapping to map half an area at a time
   - Localization mode uses a rolling buffer and does not permanently change the underlying map
   - Exposed services: `save_map`, `serialize_map`, `deserialize_map`

3. Raw service definitions
   - SaveMap.srv
     - https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/ros2/srv/SaveMap.srv
   - SerializePoseGraph.srv
     - https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/ros2/srv/SerializePoseGraph.srv
   - DeserializePoseGraph.srv
     - https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/ros2/srv/DeserializePoseGraph.srv

---

## One-line summary

In this ScoutMini workspace: save `yaml+pgm` for Nav2 use, save `posegraph` for resume, and load that pose graph in a later mapping session to extend the same floor.
