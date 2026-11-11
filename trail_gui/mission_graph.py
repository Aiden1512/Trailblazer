"""
The trail network that mission_manager inspects, for the Phase 2 map.

mission_manager publishes the graph it loaded (with each trail's live status) on
/husky1/mission_graph (see mission_status.py). Until that arrives - e.g. before
mission_manager is running - the GUI reads the same YAML file itself, found
through waypoints_file in trail_decision_making/config/mission_manager.yaml
(or the TRAILBLAZER_TRAIL_FILE environment variable).

Both give the same structure, positions in metres in the map frame:
    {"file": path,
     "junctions": [{"node": 0, "junction": 1, "x": .., "y": ..}, ...],   # junction -1 = trail dead end
     "trails": [{"edge": 0, "trail": 1, "from": 0, "to": 1, "status": "unvisited",
                 "far_side": -1, "waypoints": [[x, y], ...], "inaccessible": [[x, y], ...]}, ...]}
load_trail_file() mirrors TrailGraph::loadFromYaml (trail_graph.cpp), so edge numbers match.
"""
import math
import os

import yaml

_HERE = os.path.dirname(os.path.realpath(__file__))   # realpath: the launch file starts us via a symlink in install/
_REPO_ROOT = os.path.dirname(_HERE)
MISSION_PARAMS = os.path.join(_REPO_ROOT, "trail_decision_making", "config", "mission_manager.yaml")
NODE_MERGE_TOLERANCE = 0.5   # metres, kNodeMergeTolerance in trail_graph.cpp


def find_trail_file():
    """Path of the trail file mission_manager is configured to visit, or None."""
    env = os.environ.get("TRAILBLAZER_TRAIL_FILE")
    if env:
        return env
    try:
        with open(MISSION_PARAMS) as f:
            params = yaml.safe_load(f)
        for node in params.values():
            path = node.get("ros__parameters", {}).get("waypoints_file")
            if path:
                # Relative paths in mission_manager.yaml are relative to the repo root (as in the launch file)
                return os.path.join(_REPO_ROOT, path)
    except (OSError, yaml.YAMLError, AttributeError):
        pass
    return None


def load_trail_file(path):
    """Read a trail file the way mission_manager does. Raises OSError / ValueError."""
    with open(path) as f:
        try:
            root = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            raise ValueError(f"trail file isn't valid YAML ({exc})")
    if not isinstance(root, dict) or not isinstance(root.get("trails"), list):
        raise ValueError("trail file has no 'trails' list")

    nodes = []                   # {"node", "junction", "x", "y"}

    def get_or_create(x, y):
        for n in nodes:
            if math.hypot(n["x"] - x, n["y"] - y) < NODE_MERGE_TOLERANCE:
                return n["node"]
        nodes.append({"node": len(nodes), "junction": -1, "x": x, "y": y})
        return len(nodes) - 1

    junction_nodes = {}
    for j in root.get("junctions") or []:
        node = get_or_create(float(j["x"]), float(j["y"]))
        nodes[node]["junction"] = int(j["id"])
        junction_nodes[int(j["id"])] = node

    trails = []
    for t in root["trails"]:
        wps = [[float(w["x"]), float(w["y"])] for w in (t.get("waypoints") or [])]
        start = junction_nodes.get(t.get("start_junction"))
        end = junction_nodes.get(t.get("end_junction"))
        if start is not None:
            wps.insert(0, [nodes[start]["x"], nodes[start]["y"]])
        if end is not None:
            wps.append([nodes[end]["x"], nodes[end]["y"]])
        if len(wps) < 2:
            continue                      # same rule as the C++ loader: not a usable edge
        trails.append({
            "edge": len(trails),
            "trail": int(t["id"]) if t.get("id") is not None else -1,
            "from": start if start is not None else get_or_create(*wps[0]),
            "to": end if end is not None else get_or_create(*wps[-1]),
            "status": "unvisited",
            "far_side": -1,
            "waypoints": wps,
            "inaccessible": [],
        })
    if not trails:
        raise ValueError("trail file has no usable trails")
    return {"file": path, "junctions": nodes, "trails": trails}


def trail_color(trail):
    """Map colour for a trail's status."""
    if trail["status"] == "inspected":
        return "#2e8b32"                                  # green
    if trail["status"] == "blocked":
        return "#e08e00" if trail.get("far_side", -1) >= 0 else "#c0392b"   # orange: far side still to check
    return "#111111"                                      # black: not inspected yet


def cut_off_trails(graph):
    """Trails blocked at both ends, with a stretch between the two obstacles the robot can't reach."""
    return [t for t in graph["trails"] if len(t.get("inaccessible") or []) >= 2]


def summary(graph):
    """(inspected, blocked, still to do) trail counts."""
    trails = graph["trails"]
    inspected = sum(t["status"] == "inspected" for t in trails)
    blocked = sum(t["status"] == "blocked" for t in trails)
    return inspected, blocked, len(trails) - inspected - blocked
