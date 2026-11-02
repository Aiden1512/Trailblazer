"""
What the real robot is doing: obstacle detection and mission progress, via ROS 2.

    /husky1/obstacle_detected   std_msgs/Bool    warn  - obstacle in the warn range   (ground_filter, every lidar frame)
    /husky1/obstacle_stop       std_msgs/Bool    stop  - obstacle in the stop range   (ground_filter, every lidar frame)
    /husky1/mission_status      std_msgs/String  one-line mission progress         (mission_manager, latched)
    /husky1/mission_progress    std_msgs/Float32 mission completion, 0-100 %       (mission_manager, latched)
    /husky1/blocked_points      std_msgs/String  JSON list of where trails are blocked (mission_manager, latched)
    /husky1/water_detected      std_msgs/Bool    water (blue ground) in slow range (water_guard, every camera frame)
    /husky1/water_stop          std_msgs/Bool    forward teleop blocked by water   (water_guard, every camera frame)
    /husky1/start_mission       std_srvs/Trigger starts the inspection             (mission_manager holds until called)

MissionStatusFeed listens in a background thread and keeps the latest values;
the GUI polls get() and status() so no Tk calls are made from the ROS thread.
request_start() asks mission_manager to start; its answer is queued in
start_results as (ok, message) for the GUI to log.
"""
import json
import os
import queue
import threading
import time

import ros_context

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import Bool, Float32, String
    from std_srvs.srv import Trigger
    ROS_AVAILABLE = True
    _ROS_IMPORT_ERROR = ""
except Exception as exc:                  # normally ImportError; never let it crash the GUI
    ROS_AVAILABLE = False
    _ROS_IMPORT_ERROR = str(exc)

WARN_TOPIC = "/husky1/obstacle_detected"
STOP_TOPIC = "/husky1/obstacle_stop"
MISSION_TOPIC = "/husky1/mission_status"
PROGRESS_TOPIC = "/husky1/mission_progress"
BLOCKED_TOPIC = "/husky1/blocked_points"
START_SERVICE = "/husky1/start_mission"
WATER_WARN_TOPIC = "/husky1/water_detected"
WATER_STOP_TOPIC = "/husky1/water_stop"
FRONT_OBSTACLE_TOPIC = "/husky1/front_obstacle"
STALE_AFTER_S = 2.0                       # the detector publishes every lidar frame (~10 Hz)
WATER_STALE_AFTER_S = 3.0                 # water_guard publishes every camera frame (~3 Hz)


class MissionStatusFeed:
    """Subscribes to the obstacle detector and mission manager and keeps the latest state."""

    def __init__(self):
        self._lock = threading.Lock()
        self._warn = False
        self._stop = False
        self._mission_text = None
        self._mission_progress = None
        self._blocked_points = []
        self._last_detector_time = None
        self._water_warn = False
        self._water_stop = False
        self._last_water_time = None
        self._front_obstacle = "None"
        self._last_front_obstacle_time = None
        self._startup_error = ""
        self._start_client = None
        self.start_results = queue.Queue()

        self._stop_event = threading.Event()
        self._thread = None
        self._node = None
        self._executor = None
        self._holds_context = False

    # ---------------- lifecycle ----------------
    def start(self):
        """Start listening. Returns True if the subscribers are running."""
        if not ROS_AVAILABLE:
            self._startup_error = (
                "ROS 2 not found, so obstacle and mission status can't be read. "
                "Source ROS in this terminal first: source /opt/ros/humble/setup.bash "
                f"(and your workspace). Import error: {_ROS_IMPORT_ERROR}")
            return False
        try:
            ros_context.acquire()
            self._holds_context = True
            self._node = Node(f"trailblazer_gui_status_{os.getpid()}")
            self._node.create_subscription(Bool, WARN_TOPIC, self._on_warn, 10)
            self._node.create_subscription(Bool, STOP_TOPIC, self._on_stop, 10)
            self._node.create_subscription(Bool, WATER_WARN_TOPIC, self._on_water_warn, 10)
            self._node.create_subscription(Bool, WATER_STOP_TOPIC, self._on_water_stop, 10)
            self._node.create_subscription(String, FRONT_OBSTACLE_TOPIC, self._on_front_obstacle, 10)
            # mission_manager latches its last status, so we get it even if it was sent before we started.
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._node.create_subscription(String, MISSION_TOPIC, self._on_mission, latched)
            self._node.create_subscription(Float32, PROGRESS_TOPIC, self._on_progress, latched)
            self._node.create_subscription(String, BLOCKED_TOPIC, self._on_blocked, latched)
            self._start_client = self._node.create_client(Trigger, START_SERVICE)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
        except Exception as exc:
            self._startup_error = f"Could not start the ROS status subscribers: {exc}"
            self._cleanup_ros()
            return False

        self._thread = threading.Thread(target=self._spin, name="mission-status-feed", daemon=True)
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
        self._start_client = None

    def _spin(self):
        while not self._stop_event.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:
                break

    # ---------------- ROS callbacks (run in the ROS thread) ----------------
    def _on_warn(self, msg):
        with self._lock:
            self._warn = msg.data
            self._last_detector_time = time.monotonic()

    def _on_stop(self, msg):
        with self._lock:
            self._stop = msg.data
            self._last_detector_time = time.monotonic()

    def _on_water_warn(self, msg):
        with self._lock:
            self._water_warn = msg.data
            self._last_water_time = time.monotonic()

    def _on_water_stop(self, msg):
        with self._lock:
            self._water_stop = msg.data
            self._last_water_time = time.monotonic()

    def _on_mission(self, msg):
        with self._lock:
            self._mission_text = msg.data

    def _on_progress(self, msg):
        with self._lock:
            self._mission_progress = msg.data

    def _on_blocked(self, msg):
        try:
            points = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        # map frame metres -> GUI world centimetres (the map frame is lined up with the world)
        blocked = [(p["x"] * 100.0, p["y"] * 100.0, p.get("edge"), p.get("from", "")) for p in points]
        with self._lock:
            self._blocked_points = blocked

    def _on_front_obstacle(self, msg):
        with self._lock:
            self._front_obstacle = msg.data
            self._last_front_obstacle_time = time.monotonic()

    def _on_start_reply(self, future):
        try:
            reply = future.result()
        except Exception as exc:
            self.start_results.put((False, f"Start request failed: {exc}"))
            return
        self.start_results.put((reply.success, reply.message))

    # ---------------- commands ----------------
    def request_start(self):
        """Ask mission_manager to start the inspection. Doesn't block; the
        answer (or why it couldn't be asked) arrives in start_results."""
        if self._start_client is None:
            self.start_results.put((False, "Can't start the mission: ROS isn't connected. " + self._startup_error))
            return
        if not self._start_client.service_is_ready():
            self.start_results.put((False, f"Can't start the mission: mission_manager isn't running "
                                           f"(nothing offering {START_SERVICE})."))
            return
        self._start_client.call_async(Trigger.Request()).add_done_callback(self._on_start_reply)

    # ---------------- GUI-facing API ----------------
    def get(self):
        """Return (obstacle_level, mission_text). obstacle_level is 'stop',
        'warn', 'clear', or None while the detector isn't publishing;
        mission_text is None until mission_manager has said something."""
        with self._lock:
            last = self._last_detector_time
            warn, stop, text = self._warn, self._stop, self._mission_text
        if last is None or time.monotonic() - last > STALE_AFTER_S:
            return None, text
        return ("stop" if stop else "warn" if warn else "clear"), text

    def get_progress(self):
        """Return the mission completion percentage (0-100), or None until
        mission_manager has started a mission."""
        with self._lock:
            return self._mission_progress

    def get_blocked_points(self):
        """Return [(x_cm, y_cm, edge, from_junction), ...]: where mission_manager found trails blocked."""
        with self._lock:
            return list(self._blocked_points)

    def get_water(self):
        """Return 'stop', 'warn', 'clear', or None while water_guard isn't
        publishing (not running, or it has no camera frames)."""
        with self._lock:
            last = self._last_water_time
            warn, stop = self._water_warn, self._water_stop
        if last is None or time.monotonic() - last > WATER_STALE_AFTER_S:
            return None
        return "stop" if stop else "warn" if warn else "clear"
    
    def get_front_obstacle(self):
        with self._lock:
            name = self._front_obstacle
            last = self._last_front_obstacle_time

        if last is None or time.monotonic() - last > 2.0:
            return "Unknown"

        return name if name != "None" else "Unknown"

    def status(self):
        """Return (state, text) for the detector connection; state is one of
        'live', 'waiting', 'stale', 'unavailable'."""
        if self._startup_error:
            return "unavailable", self._startup_error
        with self._lock:
            last = self._last_detector_time
        if last is None:
            return "waiting", f"Obstacle detector not running (nothing on {STOP_TOPIC})"
        age = time.monotonic() - last
        if age > STALE_AFTER_S:
            return "stale", f"No obstacle detector update for {age:.0f} s on {STOP_TOPIC}"
        return "live", "Obstacle detector running"
