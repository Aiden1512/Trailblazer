# Trailblazer

Trailblazer provides a GUI for manually mapping a Husky's trail and then running an autonomous inspection with ROS 2 Humble, Gazebo, Nav2, LiDAR obstacle detection, and YOLO-based obstacle snapshots.

## Build the workspace

Build once before running:

```bash
cd /Path/to/Trailblazer
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

In **each new ROS terminal**, run:

```bash
source /opt/ros/humble/setup.bash
cd /Path/to/Trailblazer
source install/setup.bash
```

## Phase 1: Manual mapping using the GUI

Run the following in order, using separate terminals for processes that must remain running.

### 1. Launch the Husky simulation

If Gazebo has OpenGL rendering issues, run these commands in the simulation terminal first:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
```

Launch the **trail world** with SLAM, Nav2, and RViz:

```bash
ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py slam:=true nav2:=true rviz:=true world:=trail_world spawn_x:=6.5 spawn_y:=4 spawn_yaw:=3.14159
```

### 2. Launch the Trailblazer GUI

In a separate terminal:

```bash
ros2 run trailblazer_gui main.py
```

Select **Start a New Simulation** to open Phase 1.

### 3. Start the path recorder

In another terminal:

```bash
ros2 run husky_mapping path_recorder
```


### 4. Press Start Recording in the GUI

Click **Start Recording** before mapping the trail.

- The button changes to **Recording in Progress** and becomes disabled.
- The GUI begins drawing and storing the yellow travelled trail.
- The junction recorder begins recording junctions and waypoints.
- Movement before pressing **Start Recording** is not included in the GUI recording.

### 5. Teleoperate using the GUI

Click the GUI map to give it keyboard focus, then use:

| Key | Action |
| --- | --- |
| **W** | Forward |
| **S** | Backward |
| **A** | Rotate left |
| **D** | Rotate right |
| **Space** | Stop |

**Releasing a movement key does not stop the robot.** Press **Space** or another movement key to change its command.

The yellow trail and recorded junctions/waypoints will appear on the GUI as the robot moves.

> **Water guard note:** The GUI sends teleop commands to `/husky1/cmd_vel_teleop`

### 6. Reset if you need to start over

Click **Reset Run** to:

- Clear the GUI trail, junctions, and waypoints.
- Request a reset of the junction recorder graph.
- Re-enable **Start Recording**.

Press **Start Recording** again before remapping.
=

### 8. Finish mapping

Press **Space** to stop the robot, then click **Finish Mapping → Phase 2**.

The GUI's finish workflow saves its trail/node data to `saved_map.json` for **Continue Where You Left Off**. 

## Phase 2: Autonomous inspection and obstacle detection

Use a separate terminal for each process and source ROS and the workspace as described above.

### 1. Launch or keep running the simulation

If continuing directly from Phase 1, **do not launch a second simulation**.

For a fresh simulation in the trail world:

```bash
ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py slam:=true nav2:=true rviz:=true world:=trail_world
```

Alternatively, for the obstacle world:

```bash
ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py slam:=true nav2:=true rviz:=true world:=obstacle_world spawn_x:=6.5 spawn_y:=4 spawn_yaw:=3.14159
```

### 2. Launch or keep running the GUI

If it is not already running:

```bash
ros2 run trailblazer_gui main.py
```

Choose **Continue Where You Left Off** to load a previously saved trail, or move into Phase 2 directly from Phase 1.

### 3. Start YOLO for obstacle classification and snapshots

YOLO must run for the annotated obstacle snapshot and selected obstacle class to be available.

From the repository directory:

```bash
/usr/bin/python3 image_detection/yolo_ros_node.py
```

If `python3` already resolves to the correct Linux Python environment, you can use:

```bash
python3 image_detection/yolo_ros_node.py
```

The environment needs `ultralytics`, `cv2`, ROS 2 Python packages, and `cv_bridge`.

The YOLO node subscribes to `/husky1/camera/image` and publishes annotated frames to `/husky1/yolo_image`. With the obstacle-name changes in place, it also publishes the selected class to `/husky1/front_obstacle`.

### 4. LiDAR obstacle detection

`ground_filter` is started by the simulation launch file, so there is nothing to run here. Don't start a
second copy with `ros2 run`: both would publish `/husky1/obstacle_stop`. To check it is running:

```bash
ros2 node list | grep ground_filter
```

The LiDAR finds obstacles and the camera checks whether each one is on the (brown) trail; obstacles on the
grass beside the trail are ignored for stopping (Nav2 still avoids them). Set `trail_check:=false` on the
node to go back to LiDAR only. To see what it checks:
`ros2 run rqt_image_view rqt_image_view /husky1/trail_check_debug` (green = trail, magenta = not trail).

The detector publishes:

- `/husky1/obstacle_detected`: obstacle warning.
- `/husky1/obstacle_stop`: obstacle within the stop range.
- `/husky1/obstacle_image`: a captured YOLO-annotated snapshot when a new stop event activates.

The Phase 2 GUI shows the snapshot and the detected obstacle name. The label should update only when a new snapshot is received.

### 5. Start the mission manager

```bash
ros2 run trail_decision_making mission_manager_node --ros-args -r __ns:=/husky1 --params-file ~/Trailblazer/trail_decision_making/config/mission_manager.yaml -r /tf:=/husky1/tf -r /tf_static:=/husky1/tf_static
```

The mission manager waits for the GUI to start the autonomous inspection.

### 6. Start autonomous inspection in the GUI

Wait for the simulation, Nav2, and localisation to be ready. In Phase 2, click **Start Autonomous Inspection**.

This calls `/husky1/start_mission`. The GUI then shows mission progress, obstacle warnings, captured snapshots, and inspection messages.

### Optional: Trigger the mission manually

To bypass the GUI button:

```bash
ros2 service call /husky1/start_mission std_srvs/srv/Trigger
```

