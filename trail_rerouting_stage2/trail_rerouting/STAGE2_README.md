# Stage 2 — simulated bidirectional inspection

This adds a **new executable** to the original `trail_rerouting` package. It does not modify the original `rerouting_planner.cpp` or any teammate packages.

- `bidirectional_inspection MAP.yaml START_JUNCTION` plans coverage and a return to the start.
- `bidirectional_inspection MAP.yaml START_JUNCTION BLOCKED_TRAIL_ID FRACTION` simulates an obstacle at a fraction of the trail's distance from its `start_junction`, e.g. `2 0.5` for halfway along Trail 2.
- The planner will **never transit through a blocked trail**. Instead it plans an approach from each reachable end, stops short of the obstacle, returns to that end's junction, and routes around the loop.
- `INSPECT BLOCKED SIDE` is a *planning instruction*, not proof of physical inspection or obstacle safety. A real robot must stop before a measured obstacle using sensor data and maintain clearance. The simulator does not check the Nav2 map, physical collision geometry, or safe approach distance.
- Blocked trails are reported as blocked, **not fully inspected**. `MISSION END` is the planned return to the starting junction, not proof that a ROS robot reached it.
- The planner uses greedy task selection and Dijkstra over unblocked edges; it does not optimise global inspection travel distance.
- The terminal test uses only one simulated blockage per invocation. Runtime obstacle detection and Nav2 integration are future steps.
- The updated YAML has Trails 1–6 and an incomplete endpoint marker Trail 7. The latter is skipped, and Junction 1 is used as the return point when the mission starts at Junction 1.

## Install/update (Ubuntu)
Copy this `trail_rerouting` directory over your existing package directory **after backing up your changes**. Build from workspace root:

```bash
cd ~/Trailblazer
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select trail_rerouting
source install/setup.bash
ros2 run trail_rerouting bidirectional_inspection ~/Trailblazer/trail_rerouting_starter/trail_rerouting/config/junction_nodes.yaml 1 2 0.5
```

Use your actual YAML path if different.
