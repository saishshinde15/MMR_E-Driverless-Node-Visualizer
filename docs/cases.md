# MMR ROS Graph — Operational Cases & ROS 2 Behaviors

Reference guide for common ROS 2 Humble and `mmr_ros_graph` behaviors encountered during testing and track/bench usage.

---

## 1. Live Parameter Updates Are Not Saved to YAML or Launch Files on Disk

### What Happens
When you edit a parameter in the **Selection** sidebar (checkbox, slider + **Set**, or text/numeric input + **Set**), `mmr_ros_graph` calls the target node's `<node_name>/set_parameters` (`rcl_interfaces/srv/SetParameters`) service. The parameter updates **immediately in the running node's memory (RAM)**, **not** in any `.yaml` or `.launch.py` file on disk. Restarting the node reverts to its startup defaults.

### Why ROS 2 Works This Way
1. **Nodes do not track their source config file**: A node may get parameter values from compiled C++/Python defaults, inline dictionaries inside a launch file, CLI `--ros-args -p ...`, or multiple overlaid YAML files. ROS 2 (`ros2 param set`, `rqt_reconfigure`, and `mmr_ros_graph`) only sends `SetParameters` requests over DDS to live memory.
2. **Launch files are executable code**: `.launch.py` files often compute parameters dynamically, use launch arguments, and contain formatting/comments that cannot be safely rewritten in place by a viewer.
3. **Track/bench safety**: During tuning or fault injection (such as `enabled: false` or `simulate_dropouts: true`), silently modifying tracked config files in Git risks leaving temporary overrides active on the next boot.

### Also Note: Dynamic Reaction Depends on the Target Node
- Nodes that read parameters on every loop or register `add_on_set_parameters_callback` react immediately in real time (for example, changing `publish_rate_hz` on `/sensors/lidar_driver` in `mmr_race_sim`).
- External nodes that only read a parameter once during `__init__` will store the new value in memory (unless rejected), but will not change runtime behavior until restarted or re-read.
- Parameters declared with `read_only=True` stay locked in the UI; invalid values rejected by a node callback show the node's `reason` string.

### How to Persist Tuned Values When You Want to Keep Them
Dump the node's live parameters to a standard ROS 2 YAML file once tuning is finished:

```bash
ros2 param dump /control/mpc_controller > mpc_params.yaml
```

Then reload that YAML file on future runs:

```bash
ros2 run <package> <executable> --ros-args --params-file mpc_params.yaml
```

Or pass `parameters=['path/to/mpc_params.yaml']` in your launch file's `Node(...)` definition.

---

## 2. QoS Mismatches ("Silent Wires") and Discovery `UNKNOWN` Fields

### What Happens
Two nodes can appear attached to the same topic name and message type, yet zero messages flow between them. `mmr_ros_graph` runs Humble's `rclpy.qos.qos_check_compatible` on every publisher/subscriber pair and marks incompatible wires dashed red (or dashed amber for warnings), also listing them in the **Health** tab.

Common incompatible pairs:
- **Reliability**: `BEST_EFFORT` publisher → `RELIABLE` subscriber (fails).
- **Durability**: `VOLATILE` publisher → `TRANSIENT_LOCAL` subscriber (fails).

### Why `history` and `depth` Show `UNKNOWN / 0`
In ROS 2 Humble (Fast-DDS), SPDP/SEDP discovery packets propagate `reliability`, `durability`, `liveliness`, `deadline`, and `lifespan`, **not** `history` or queue `depth`. Seeing `"history": "UNKNOWN", "depth": 0` in snapshots is normal DDS discovery behavior.

---

## 3. Duplicate Node Names & Stale ROS 2 Daemon Entries

### What Happens
If two copies of the same node run without distinct names or namespaces (for example, starting `talker` twice), ROS 2 logs `WARNING: Be aware that there are nodes in the graph that share an exact name`. `mmr_ros_graph` counts occurrences per `(namespace, name)` and displays a `duplicate_node` warning badge (`x2`, `x3`, …) plus an entry in **Health**.

### What to Do
- Stop duplicate background instances or launch each copy with unique names/namespaces (`--ros-args -r __node:=... -r __ns:=...`).
- If `ros2 node list` continues showing nodes you already closed, reset the CLI discovery daemon:
  ```bash
  ros2 daemon stop
  ```

---

## 4. Port Fallback & Orphan Prevention on macOS VM Setup

### What Happens
- **Port fallback**: `live_graph` tries `127.0.0.1:8765` first and automatically binds an available ephemeral port if `8765` is already held by another running `live_graph` session. Always check the printed URL in the terminal.
- **macOS host vs. container (`ros-shell`)**: Run `python3 live_graph` in the **Mac terminal**, not inside `ros-shell`. The Mac server opens the browser locally and spawns one `live_graph worker` inside `ros2_humble` over stdio JSON-lines.
- **Clean teardown**: Press `Ctrl-C` in the Mac terminal to close `stdin` to the container worker, causing `/mmr_ros_graph_<pid>` to exit cleanly without leaving orphan ROS nodes.

---

## 5. On-Demand Execution, `ROS_DOMAIN_ID`, and Early Startup Placeholders

- **On-demand, project-independent**: `mmr_ros_graph` does not run as a permanent background daemon and is not tied to any single package folder. Launch it once in a terminal while inspecting any active ROS 2 nodes in the same domain, and stop it with `Ctrl-C` when done.
- **Network isolation on Mac**: By default, `ros2-mac` sets `ROS_DOMAIN_ID=0` and `ROS_LOCALHOST_ONLY=1` inside the VM. Override those environment variables before launching when connecting to a real car or remote robot on the network.
- **Unknown DDS startup placeholders**: During the first ~100–300 ms of discovery, Fast-DDS can briefly emit `_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_` before participant metadata arrives; `is_user_node()` filters these placeholders while the reader warms up.

---

## 6. Heavy Topic Monitoring (`Hz` / Echo)

- Topic message rate (`Hz`), bandwidth (`KB/s`), and payload previews only subscribe while **Monitor Live** is active on the selected topic/wire.
- Large binary blobs (`> 64` bytes) and long numeric arrays (`> 64` items, e.g., high-resolution `LaserScan` or `PointCloud2`) are automatically truncated in the UI JSON preview (`... (+N more)`) and throttled to ~5 Hz so high-rate sensor streams do not freeze the browser.
