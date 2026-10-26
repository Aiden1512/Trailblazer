# Trailblazer
helpful commands

export MESA_GL_VERSION_OVERRIDE=3.3
export LIBGL_ALWAYS_SOFTWARE=1

# GENERAL:
cd ~/Trailblazer/
colcon build --symlink-install
source install/setup.bash
source /opt/ros/humble/setup.bash


ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/husky1/cmd_vel


# PACKAGES:
ros2 run trailblazer_gui main.py

ros2 run trail_object_detection ground_filter --ros-args --log-level debug

ros2 launch 41068_ignition_bringup   41068_ignition_husky.launch.py   slam:=true nav2:=true rviz:=true world:=trail_world

ros2 run trail_decision_making mission_manager_node --ros-args   -r __ns:=/husky1   --params-file ~/Trailblazer/trail_decision_making/config/mission_manager.yaml   -r /tf:=/husky1/tf   -r /tf_static:=/husky1/tf_static