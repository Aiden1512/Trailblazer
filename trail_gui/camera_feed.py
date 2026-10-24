"""
Live camera feed from the Ignition Gazebo simulation, via ROS 2.

How the feed gets here
----------------------
The 41068_ignition_bringup package already bridges the Husky's RGB camera
into ROS 2 (see config/gazebo_bridge_husky1.yaml):

    Gazebo  /model/husky1/camera/image  -->  ROS 2  /husky1/camera/image
                                              (sensor_msgs/msg/Image, 720x480, ~3 Hz)

CameraFeed subscribes to that topic in a background thread, converts each
sensor_msgs/Image into a PIL image (no cv_bridge / OpenCV needed), and keeps
only the newest frame. The Tk GUI polls for it from its own thread via
get_frame(), so no Tk calls are ever made from the ROS thread.

If ROS 2 isn't available (rclpy can't be imported) the GUI still runs; the
camera panel just reports why there's no signal.

Before running the GUI you need ROS sourced in the same terminal, e.g.
    source /opt/ros/humble/setup.bash
    source ~/41068_ws/install/setup.bash
"""
import os
import threading
import time
from collections import deque

try:
    import numpy as np
except ImportError:                       # pragma: no cover - reported via status()
    np = None

try:
    from PIL import Image
except ImportError:                       # pragma: no cover - reported via status()
    Image = None

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image as RosImage
    ROS_AVAILABLE = True
    _ROS_IMPORT_ERROR = ""
except Exception as exc:                  # normally ImportError; never let it crash the GUI
    ROS_AVAILABLE = False
    _ROS_IMPORT_ERROR = str(exc)

import ros_context

DEFAULT_TOPIC = "/husky1/camera/image"
STALE_AFTER_S = 3.0                       # camera runs ~3 Hz, so 3 s of silence = problem

# encoding -> (channels, colour order)
_ENCODINGS = {
    "rgb8": (3, "rgb"),
    "bgr8": (3, "bgr"),
    "rgba8": (4, "rgb"),
    "bgra8": (4, "bgr"),
    "mono8": (1, "mono"),
    "8uc1": (1, "mono"),
}


def ros_image_to_pil(msg):
    """Convert a sensor_msgs/Image (anything with height/width/step/encoding/
    data) into an RGB PIL image. Raises ValueError for unsupported data."""
    encoding = msg.encoding.lower()
    if encoding not in _ENCODINGS:
        raise ValueError(f'unsupported image encoding "{msg.encoding}"')
    channels, order = _ENCODINGS[encoding]

    data = np.frombuffer(msg.data, dtype=np.uint8)
    expected = msg.height * msg.step
    if data.size < expected:
        raise ValueError(f"image data too short ({data.size} bytes, expected {expected})")

    rows = data[:expected].reshape(msg.height, msg.step)
    pixels = rows[:, :msg.width * channels]          # drop any row padding

    if channels == 1:
        return Image.fromarray(np.ascontiguousarray(pixels)).convert("RGB")

    pixels = pixels.reshape(msg.height, msg.width, channels)
    rgb = pixels[..., :3]                            # ignore alpha if present
    if order == "bgr":
        rgb = rgb[..., ::-1]
    return Image.fromarray(np.ascontiguousarray(rgb))


class CameraFeed:
    """Subscribes to a ROS 2 camera topic and keeps the latest frame."""

    def __init__(self, topic=DEFAULT_TOPIC):
        self.topic = topic
        self._lock = threading.Lock()
        self._frame = None
        self._frame_id = 0
        self._frame_times = deque(maxlen=10)
        self._last_frame_time = None
        self._last_error = ""
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
                "ROS 2 not found, so there is no camera feed. Source ROS in this "
                "terminal first: source /opt/ros/humble/setup.bash "
                f"(and your workspace). Import error: {_ROS_IMPORT_ERROR}")
            return False
        if np is None or Image is None:
            self._startup_error = ("numpy and Pillow are required for the camera feed "
                                   "(sudo apt install python3-numpy python3-pil python3-pil.imagetk)")
            return False

        try:
            ros_context.acquire()
            self._holds_context = True
            self._node = Node(f"trailblazer_gui_camera_{os.getpid()}")
            # Sensor-data QoS (best effort) can connect to both best-effort and
            # reliable publishers, so it works however the bridge is configured.
            self._node.create_subscription(RosImage, self.topic, self._on_image,
                                           qos_profile_sensor_data)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
        except Exception as exc:
            self._startup_error = f"Could not start the ROS camera subscriber: {exc}"
            self._cleanup_ros()
            return False

        self._thread = threading.Thread(target=self._spin, name="camera-feed", daemon=True)
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
                break          # context was shut down underneath us

    # ---------------- ROS callback (runs in the camera thread) ----------------
    def _on_image(self, msg):
        try:
            frame = ros_image_to_pil(msg)
        except ValueError as exc:
            self._last_error = str(exc)
            return
        now = time.monotonic()
        with self._lock:
            self._frame = frame
            self._frame_id += 1
            self._frame_times.append(now)
            self._last_frame_time = now
        self._last_error = ""

    # ---------------- GUI-facing API ----------------
    def get_frame(self):
        """Return (frame_id, PIL.Image or None). frame_id changes whenever a
        new frame has arrived, so the GUI can skip redundant redraws."""
        with self._lock:
            return self._frame_id, self._frame

    def status(self):
        """Return (state, text); state is one of
        'live', 'waiting', 'stale', 'error', 'unavailable'."""
        if self._startup_error:
            return "unavailable", self._startup_error
        with self._lock:
            last = self._last_frame_time
            times = list(self._frame_times)
            frame = self._frame
        if last is None:
            if self._last_error:
                return "error", f"Camera frames can't be shown: {self._last_error}"
            return "waiting", f"Waiting for frames on {self.topic} ... is the simulation running?"
        age = time.monotonic() - last
        if age > STALE_AFTER_S:
            return "stale", f"No new frames for {age:.0f} s on {self.topic}"
        fps = 0.0
        if len(times) >= 2 and times[-1] > times[0]:
            fps = (len(times) - 1) / (times[-1] - times[0])
        return "live", f"LIVE  {frame.width}x{frame.height}  {fps:.1f} fps"