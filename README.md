# Trailblazer

## Run the autonomous inspection

Build the workspace once:

```bash
cd ~/Trailblazer
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## PHASE 1: Manual mapping and object-detection workflow (run in order)

If you are doing the manual map-creation and detection workflow, run the commands below in this order, one command per terminal unless noted otherwise. These are the steps for setting up the simulation, teleoperating the robot, saving a map, and launching the YOLO-based detection node.

1. Set the OpenGL compatibility variables for Gazebo / Ignition rendering:

   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   export MESA_GL_VERSION_OVERRIDE=3.3
   ```

   These fix rendering issues on systems without full hardware OpenGL support.

2. Launch the Husky simulation in the trail world without Nav2, and open RViz:

   ```bash
   ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py slam:=true nav2:=false rviz:=true world:=trail_world
   ```

   This starts the simulated robot and the local SLAM / visualization stack for manual driving and mapping.

3. Start keyboard teleoperation for the Husky:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/husky1/cmd_vel
   ```

   This lets you drive the robot manually through the environment while collecting the map.

4. Start the path recorder to log the robot trajectory:

   ```bash
   ros2 run husky_mapping path_recorder
   ```

   This records the path travelled so the map can be reconstructed or saved later.

5. Save the generated map to disk:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/FILEPATH/FILENAME --ros-args -r map:=/husky1/map
   ```

   This writes the current occupancy map to a file for later use in autonomous navigation.

6. Start the Husky WASD teleop node:

   ```bash
   ros2 run husky_mapping husky_wasd_node
   ```

   This runs a direct keyboard-driving node for the Husky when you want a simple manual drive interface.

7. Run the repository's YOLO ROS detection node:

   ```bash
   python ~/41068_ws/src/Trailblazer/image_detection/yolo_ros_node.py
   ```

   This launches the object-detection pipeline using the YOLO model in this project.

8. If using the alternate YOLOE environment, activate it and run the alternate node:

   ```bash
   source yoloe_env/bin/activate
   python ~/yoloe_test/yolo_ros_node.py
   ```

   These commands switch to a separate YOLOE environment for the alternate detection script.


## PHASE 2: Autonomous inspection phase
Start each command below in a **separate terminal**. In each terminal, first run:

```bash
source /opt/ros/humble/setup.bash
cd ~/Trailblazer
source install/setup.bash
```

1. Start the Husky simulation, SLAM, Nav2, and RViz:

   There may be an issue on the first run of opening the simulation, to fix run these two: 

    ```bash
    export MESA_GL_VERSION_OVERRIDE=3.3
    export LIBGL_ALWAYS_SOFTWARE=1
    ```

   To start the simulation with no obstacles:

   ```bash
   ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py slam:=true nav2:=true rviz:=true world:=trail_world
   ```
   or for obstacle world: 

    ```bash
   ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py slam:=true nav2:=true rviz:=true world:=obstacle_world spawn_x:=6.5 spawn_y:=4 spawn_yaw:=3.14159
   ```
2. Start the GUI:

   ```bash
   ros2 run trailblazer_gui main.py
   ```

3. Start LiDAR obstacle detection:

   ```bash
   ros2 run trail_object_detection ground_filter
   ```

   The camera water guard (stops Phase 1 teleop driving into the blue pond) is
   started by the launch file. To run it on its own instead:

   ```bash
   ros2 run trail_object_detection water_guard
   ```

   For Phase 1, drive with teleop publishing to `/husky1/cmd_vel_teleop` (not
   `/husky1/cmd_vel`) so its commands pass through the guard:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/husky1/cmd_vel_teleop
   ```
   or
   ```bash
   ros2 run husky_mapping husky_wasd_node --ros-args -r /husky1/cmd_vel:=/husky1/cmd_vel_teleop
   ```

   The guard slows forward driving when blue ground is within 1.8 m ahead and
   blocks it inside 0.9 m; reversing and turning always work. It also blocks
   forward driving if no camera frames arrive. Phase 2 (Nav2) publishes to
   `/husky1/cmd_vel` directly and isn't affected. To see what it detects, view
   `/husky1/water_debug` in RViz or `rqt_image_view` (water in the robot's path
   is painted red).

4. Start the mission manager:

   ```bash
   ros2 run trail_decision_making mission_manager_node --ros-args -r __ns:=/husky1 --params-file ~/Trailblazer/trail_decision_making/config/mission_manager.yaml -r /tf:=/husky1/tf -r /tf_static:=/husky1/tf_static
   ```

5. To override the GUI button and automatically start the mission:

   ```bash
   ros2 service call /husky1/start_mission std_srvs/srv/Trigger
   ```

In the GUI, map a trail in **Phase 1** or continue with a saved map, then go to
**Phase 2 - Autonomous Inspection**. Press **Start Autonomous Inspection** to
start the inspection; the button sends the start request to the mission manager.
Wait for the simulation, Nav2, and localisation to be ready before starting.
