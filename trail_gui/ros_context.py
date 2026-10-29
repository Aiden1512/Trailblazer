"""
Shared rclpy start-up/shut-down.

Several parts of the GUI talk to ROS 2 (robot position, camera, ...). Each
one calls acquire() when it starts and release() when it stops; rclpy is
initialised by the first and shut down only when the last one has let go, so
they can come and go independently without pulling ROS out from under each
other.
"""
import threading

try:
    import rclpy
except Exception:                # ROS not sourced - callers check their own *_AVAILABLE flag first
    rclpy = None

_lock = threading.Lock()
_users = 0
_we_initialised = False


def acquire():
    global _users, _we_initialised
    with _lock:
        if not rclpy.ok():
            rclpy.init()
            _we_initialised = True
        _users += 1


def release():
    global _users, _we_initialised
    with _lock:
        _users = max(0, _users - 1)
        if _users == 0 and _we_initialised and rclpy.ok():
            rclpy.shutdown()
            _we_initialised = False