import json
import threading
from std_msgs.msg import String


class TrailGraphFeed:
    def __init__(self, ros_node):
        self._lock = threading.Lock()
        self._graph = None

        self.subscription = ros_node.create_subscription(String, "/husky1/trail_graph", self._callback, 10)

    def _callback(self, msg):
        try:
            graph = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        with self._lock:
            self._graph = graph

    def get_graph(self):
        with self._lock:
            return self._graph