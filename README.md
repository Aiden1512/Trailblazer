# Trailblazer

Trailblazer is a ROS 2 Humble simulation for manually recording a Husky robot's trail network and then conducting autonomous trail inspections. It integrates Gazebo, SLAM Toolbox, Nav2, LiDAR obstacle detection, YOLO-based obstacle classification, and a custom operator GUI.

This guide demonstrates one representative scenario: **map the trail manually, start autonomous inspection, and introduce an obstacle to observe the system response**.


## 1. Build the workspace
> Install dependencies using the project installation instructions before building. 

```bash
cd /path/to/Trailblazer
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. Launch the complete system

If Gazebo has OpenGL rendering issues (which is often the case when using WSL2), set these variables **in the launch terminal**:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
```

Launch the trail world, including the mapping, navigation, perception, and GUI components:

```bash
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py \slam:=true nav2:=true rviz:=true \world:=trail_world \spawn_x:=6.5 spawn_y:=3 spawn_yaw:=3.14159
```

**Expected windows and output:**
| Window | Output |
| --- | --- |
| **Gazebo** | Forest world with brown trails, grass, a pond, obstacles, and the Husky robot. |
| **RViz** | Robot, laser observations, occupancy map, and navigation visualisations. |
| **Trailblazer GUI** | Start screen offering a new simulation or continuation of saved progress. |
| **Launch terminal** | ROS node startup logs; keep this terminal running. |

Allow the simulation and Nav2 to initialise before interacting with the GUI.

## 3. Phase 1 — Manual mapping

### Start recording

1. In the GUI, select **Start a New Simulation**.
2. Click **Start Recording** before moving the robot. The button should change to **Recording in Progress**.
3. Click inside the GUI's map area to give it keyboard focus.



### Drive the robot around the trails

| Key | Action |
| --- | --- |
| **W** | Move forward |
| **S** | Move backward |
| **A** | Rotate left |
| **D** | Rotate right |
| **Space** | Stop |

**Important:** Releasing a movement key does **not** automatically stop the robot. Press **Space** to stop.

Drive along the brown trails and through intersections to record the network. 

While mapping:

- RViz displays the occupancy grid produced by SLAM Toolbox.
- The GUI draws the travelled route and shows recorded junctions and nodes.
- `path_recorder` records and saves junctions and intermediate nodes.
- `water_guard` limits commands that would drive the robot into the pond.


### Finish mapping

1. Press **Space** to stop the Husky.
2. Confirm the recorded junctions and trail connections appear in the GUI.
3. Select **Finish Mapping → Phase 2**.

The GUI saves its recorded data for **Continue Where You Left Off**. To start over instead, choose **Reset Run**, then select **Start Recording** again.

## 4. Phase 2 — Autonomous inspection

### Start the mission

In the Phase 2 GUI, click **Start Autonomous Inspection**.

The GUI should display the recorded trail network, robot position, mission progress and inspection status while the robot navigates between trail locations.


### Introduce an obstacle

Use the GUI's obstacle placement controls to place a **rock or fallen tree on a trail ahead of the Husky**.
To do this press **Place Obstacle**, then choose the obstacle type byt pressing either fallen tree or rock. Then press on the map where tou want the obstacle. Then press the place obstacle button again.

Allow the robot to approach it.

Expected behaviour:

1. The LiDAR-based `ground_filter` identifies the obstacle and checks whether it is on the trail and not part of the known scenery.
2. The YOLO node processes RGB camera frames and publishes an annotated image and selected object class.
3. The mission manager reacts to `/husky1/obstacle_stop` and coordinates the navigation response.
4. The GUI displays the obstacle snapshot/classification and updated inspection status.

The robot will continue the autonomous inspection until all trails have been explored. Once the mission is complete, the user can start another inspection, add new obstacles or remove existing obstacles through the GUI to test different scenarios and observe how the system responds.


## 5. Common issues

| Issue | Troubleshooting |
| --- | --- |
| Gazebo renders incorrectly | Set `LIBGL_ALWAYS_SOFTWARE=1` and `MESA_GL_VERSION_OVERRIDE=3.3` before launching. |
| GUI does not respond to W/A/S/D | Click the GUI map to restore keyboard focus. Press **Space** to stop the robot. |
| No path or junctions appear | Check **Start Recording** was pressed and that `path_recorder` started successfully in the unified launch. |
| Autonomous inspection does not begin | Wait for Nav2 and localisation to initialise; check logs and verify a trail graph is available. |
| The system is slow | Gazebo, RViz, Nav2 and CPU-based YOLO can compete for resources. Close unnecessary programs or launch without RViz if appropriate. |


