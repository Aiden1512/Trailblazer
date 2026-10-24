"""
The robot's actual position, taken from the simulation.

The Husky's odometry is bridged from Ignition Gazebo into ROS 2 as
    /husky1/odometry   (nav_msgs/msg/Odometry, ~20 Hz)
(see config/gazebo_bridge_husky1.yaml in 41068_ignition_bringup).

The Husky spawns at the world origin (launch file: x=0, y=0), so its
odometry is the same as its world position, and the world origin is the
centre of the GUI map. RobotPoseFeed converts metres (ROS) to centimetres
(the GUI's map units) and keeps the latest value for the GUI to read. It runs
in a background thread; the GUI polls get_pose() so no Tk calls are made from
the ROS thread.
"""
import math
import os
import threading
import time

import ros_context

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    ROS_AVAILABLE = True
    _ROS_IMPORT_ERROR = ""
except Exception as exc:                  # normally ImportError; never let it crash the GUI
    ROS_AVAILABLE = False
    _ROS_IMPORT_ERROR = str(exc)

DEFAULT_TOPIC = "/husky1/odometry"
STALE_AFTER_S = 2.0                       # odometry is ~20 Hz, so 2 s of silence = problem

# Where the world origin sits relative to the odometry origin, in metres.
# (0, 0) because the Husky spawns at the world origin; change if you ever
# spawn it elsewhere.
ODOM_TO_WORLD_M = (0.0, 0.0)

# Colours for status text, shared by every widget that shows feed status.
STATE_COLORS = {
    "live": "#2e7d32",
    "waiting": "#b26a00",
    "stale": "#b26a00",
    "error": "#c0392b",
    "unavailable": "#c0392b",
}


class RobotPoseFeed:
    """Subscribes to the robot's odometry and keeps the latest position."""

    def __init__(self, topic=DEFAULT_TOPIC):
        self.topic = topic
        self._lock = threading.Lock()
        self._pose = None                 # (x_cm, y_cm, yaw_rad)
        self._last_time = None
        self._startup_error = ""

        self._stop_event = threading.Event()
        self._thread = None
        self._node = None
        self._executor = None
        self._holds_context = False

    # ---------------- lifecycle ----------------
    def start(self):
        """Start listening. Returns True if the subscriber is running."""
        if not ROS_AVAILABLE:
            self._startup_error = (
                "ROS 2 not found, so the robot position can't be read from the simulation. "
                "Source ROS in this terminal first: source /opt/ros/humble/setup.bash "
                f"(and your workspace). Import error: {_ROS_IMPORT_ERROR}")
            return False
        try:
            ros_context.acquire()
            self._holds_context = True
            self._node = Node(f"trailblazer_gui_pose_{os.getpid()}")
            self._node.create_subscription(Odometry, self.topic, self._on_odom,
                                           qos_profile_sensor_data)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
        except Exception as exc:
            self._startup_error = f"Could not start the ROS position subscriber: {exc}"
            self._cleanup_ros()
            return False

        self._thread = threading.Thread(target=self._spin, name="robot-pose-feed", daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._cleanup_ros()

    def _cleanup_ros(self):
        try:
            if self._executor is not None:
                self._executor.shutdown()
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        if self._holds_context:
            ros_context.release()
            self._holds_context = False
        self._executor = None
        self._node = None

    def _spin(self):
        while not self._stop_event.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:
                break

    # ---------------- ROS callback (runs in the ROS thread) ----------------
    def _on_odom(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x_cm = (pos.x + ODOM_TO_WORLD_M[0]) * 100.0
        y_cm = (pos.y + ODOM_TO_WORLD_M[1]) * 100.0
        with self._lock:
            self._pose = (x_cm, y_cm, yaw)
            self._last_time = time.monotonic()

    # ---------------- GUI-facing API ----------------
    def get_pose(self):
        """Latest (x_cm, y_cm, yaw_rad) in map coordinates (origin at the map
        centre, +x right, +y up), or None if nothing has been received yet."""
        with self._lock:
            return self._pose

    def status(self):
        """Return (state, text); state is one of
        'live', 'waiting', 'stale', 'unavailable'."""
        if self._startup_error:
            return "unavailable", self._startup_error
        with self._lock:
            last = self._last_time
        if last is None:
            return "waiting", f"Waiting for the robot's position on {self.topic} ... is the simulation running?"
        age = time.monotonic() - last
        if age > STALE_AFTER_S:
            return "stale", f"No position update for {age:.0f} s on {self.topic}"
        return "live", "Receiving robot position from the simulation"