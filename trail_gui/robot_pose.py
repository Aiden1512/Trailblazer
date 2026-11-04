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
import json


import ros_context

try:
    import rclpy
    import tf2_ros
    from rclpy.time import Time
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from geometry_msgs.msg import Twist
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
        self._pose = None
        self._map_pose = None
        self._graph = None
        self._graph_version = 0                 # (x_cm, y_cm, yaw_rad)
        self._last_time = None
        self._startup_error = ""

        self._stop_event = threading.Event()
        self._thread = None
        self._node = None
        self._executor = None
        self._holds_context = False
        self.tf_buffer = None
        self.tf_listener = None
        self._cmd_vel_pub = None

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
            self._node = Node(f"trailblazer_gui_pose_{os.getpid()}",
                cli_args=["--ros-args", "-r", "/tf:=/husky1/tf", "-r", "/tf_static:=/husky1/tf_static"],
                use_global_arguments=False
            )
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self._node, spin_thread=False)
            self._node.create_subscription(Odometry, self.topic, self._on_odom,
                                           qos_profile_sensor_data)
            self._map_pose_sub = self._node.create_subscription( PoseWithCovarianceStamped, "/husky1/pose", self._on_map_pose, qos_profile_sensor_data)
            self._graph_sub = self._node.create_subscription(String, "/husky1/trail_graph", self._on_graph, 10)
            self._cmd_vel_pub = self._node.create_publisher(Twist, "/husky1/cmd_vel_teleop", 10)
            self._reset_client = self._node.create_client(Trigger, "/husky1/reset_trail_graph")
            self._start_client = self._node.create_client(Trigger, "/husky1/start_trail_recording")
            self._stop_client = self._node.create_client(Trigger, "/husky1/stop_trail_recording")
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
        self.tf_listener = None
        self.tf_buffer = None

    def _spin(self):
        while not self._stop_event.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:
                break

    # ---------------- ROS callbacks (runs in the ROS thread) ----------------
    def _on_odom(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x_cm = (pos.x + ODOM_TO_WORLD_M[0]) * 100.0
        y_cm = (pos.y + ODOM_TO_WORLD_M[1]) * 100.0
        with self._lock:
            self._pose = (x_cm, y_cm, yaw)
            self._last_time = time.monotonic()

    def _on_map_pose(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2( 2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._map_pose = (pos.x, pos.y, yaw)

    def _on_graph(self, msg):
        try:
            graph = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        with self._lock:
            self._graph = graph
            self._graph_version += 1

    def reset_graph(self):
        if self._node is None or not self._reset_client.service_is_ready():
            return False

        self._reset_client.call_async(Trigger.Request())
        return True

    def start_recording(self):
        if self._node is None or not self._start_client.service_is_ready():
            return False

        self._start_client.call_async(Trigger.Request())
        return True

    def stop_recording(self):
        if self._node is None or not self._stop_client.service_is_ready():
            return False

        self._stop_client.call_async(Trigger.Request())
        return True

    def send_velocity(self, linear=0.0, angular=0.0):
        if self._cmd_vel_pub is None:
            return

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_vel_pub.publish(msg)

    # ---------------- GUI-facing API ----------------
    def get_pose(self):
        """Latest (x_cm, y_cm, yaw_rad) in map coordinates (origin at the map
        centre, +x right, +y up), or None if nothing has been received yet."""
        with self._lock:
            return self._pose
            
    def get_graph(self):
        with self._lock:
            return self._graph_version, self._graph

    
    def map_to_display(self, x, y):
        """Convert husky1_map coordinates to the GUI's odometry coordinates."""

        with self._lock:
            map_pose = self._map_pose
            odom_pose = self._pose

        if map_pose is None or odom_pose is None:
            return None

        mx, my, map_yaw = map_pose
        # /100 because GUI odometry is stored in centimetres
        ox = odom_pose[0] / 100.0
        oy = odom_pose[1] / 100.0
        odom_yaw = odom_pose[2]

        angle = odom_yaw - map_yaw  # Difference in robot orientation between coordinate systems
        dx = x - mx        # Position relative to the robot in map coordinates
        dy = y - my
        cos_a = math.cos(angle)   # Rotate into the GUI odom coord system
        sin_a = math.sin(angle)
        display_x = ox + dx * cos_a - dy * sin_a
        display_y = oy + dx * sin_a + dy * cos_a

        return display_x, display_y


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