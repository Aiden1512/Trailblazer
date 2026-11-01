# Trailblazer standalone rerouting prototype

This is a graph-level **offline planner**, not a ROS node and not a Nav2 controller. It does not move Husky. It reads the existing junction/trail YAML, preserves parallel trails by ID, computes Euclidean polyline trail lengths, and generates a greedy inspection traversal. Blocked trail IDs are excluded from routing. Unreachable/blocked trails are reported rather than marked inspected.

## Install into your existing workspace

Place the `trail_rerouting` folder alongside your existing ROS packages (for example `~/Trailblazer/trail_rerouting/` if that is where `trail_decision_making/` is located).

```bash
sudo apt install libyaml-cpp-dev
cd ~/Trailblazer
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select trail_rerouting
source install/setup.bash
ros2 run trail_rerouting rerouting_planner ~/Trailblazer/trail_rerouting/config/junction_nodes.yaml 1
```

Simulate blocked Trail 2:

```bash
ros2 run trail_rerouting rerouting_planner ~/Trailblazer/trail_rerouting/config/junction_nodes.yaml 1 2
```

Multiple blocked trails:

```bash
ros2 run trail_rerouting rerouting_planner ~/Trailblazer/trail_rerouting/config/junction_nodes.yaml 1 2 5
```

If your packages live in `~/Trailblazer/src/`, adjust the folder path accordingly. You can also compile this standalone file directly with `g++ -std=c++17 src/rerouting_planner.cpp -lyaml-cpp -o rerouting_planner`.

## Important limitations

- This planner assumes bidirectional trails and plans inspection of each usable edge once. Verify whether that is consistent with the mission requirements.
- A `BLOCKED` trail is treated as fully impassable. Partial inspections, obstacle coordinates, and approach-from-opposite-end logic are **not yet implemented**.
- The output is a planned sequence only. Do not mark trails `INSPECTED` on the live robot until Nav2 confirms successful traversal.
- Does not subscribe to obstacle flags or control Nav2. Integration must avoid two competing Nav2 goal publishers.
- Trail 8 has a null endpoint and no waypoints, so it is reported as incomplete and excluded.
- Greedy coverage is not guaranteed globally shortest; Dijkstra finds shortest repositioning paths at each step.
