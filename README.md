# Trailblazer

## Run the autonomous inspection

Build the workspace once:

```bash
cd ~/Trailblazer
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

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
   ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py slam:=true nav2:=true rviz:=true world:=obstacle_world
   ```
2. Start the GUI:

   ```bash
   ros2 run trailblazer_gui main.py
   ```

3. Start LiDAR obstacle detection:

   ```bash
   ros2 run trail_object_detection ground_filter
   ```

   Start the camera water guard (stops Phase 1 teleop driving into the blue pond):

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
