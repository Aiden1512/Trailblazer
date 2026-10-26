"""
What the real robot is doing: obstacle detection and mission progress, via ROS 2.

    /husky1/obstacle_detected   std_msgs/Bool    warn  - obstacle in the warn range   (ground_filter, every lidar frame)
    /husky1/obstacle_stop       std_msgs/Bool    stop  - obstacle in the stop range   (ground_filter, every lidar frame)
    /husky1/mission_status      std_msgs/String  one-line mission progress         (mission_manager, latched)
    /husky1/start_mission       std_srvs/Trigger starts the inspection             (mission_manager holds until called)

MissionStatusFeed listens in a background thread and keeps the latest values;
the GUI polls get() and status() so no Tk calls are made from the ROS thread.
request_start() asks mission_manager to start; its answer is queued in
start_results as (ok, message) for the GUI to log.
"""
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
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    ROS_AVAILABLE = True
    _ROS_IMPORT_ERROR = ""
except Exception as exc:                  # normally ImportError; never let it crash the GUI
    ROS_AVAILABLE = False
    _ROS_IMPORT_ERROR = str(exc)

WARN_TOPIC = "/husky1/obstacle_detected"
STOP_TOPIC = "/husky1/obstacle_stop"
MISSION_TOPIC = "/husky1/mission_status"
START_SERVICE = "/husky1/start_mission"
STALE_AFTER_S = 2.0                       # the detector publishes every lidar frame (~10 Hz)


class MissionStatusFeed:
    """Subscribes to the obstacle detector and mission manager and keeps the latest state."""

    def __init__(self):
        self._lock = threading.Lock()
        self._warn = False
        self._stop = False
        self._mission_text = None
        self._last_detector_time = None
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
            # mission_manager latches its last status, so we get it even if it was sent before we started.
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._node.create_subscription(String, MISSION_TOPIC, self._on_mission, latched)
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

    def _on_mission(self, msg):
        with self._lock:
            self._mission_text = msg.data

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
