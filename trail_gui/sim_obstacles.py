"""
Places (and removes) obstacles in the running Ignition Gazebo simulation.

When the operator places an obstacle on the Phase 2 map, SimObstacleSpawner
asks the simulator to create a real model at the same spot, using Gazebo's
`/world/<world>/create` service (and `/world/<world>/remove` to take them
away again). It does this by calling the `ign service` command line tool
(`gz service` on newer Gazebo), the same toolchain the simulation is launched
with, so nothing extra needs installing.

Map coordinates are centimetres with the origin at the map centre; the sim's
world origin is the same point and its units are metres, so
    world_metres = map_cm / 100.

Calls take a second or two, so they run in background threads. Outcomes are
queued in `results` as (ok, message) for the GUI to log.
"""
import itertools
import os
import queue
import re
import shutil
import subprocess
import threading
import time

OBSTACLE_WIDTH_M = 0.6          # footprint of the box
OBSTACLE_HEIGHT_M = 1.0         # tall enough for the lidar to see it
CALL_TIMEOUT_MS = 5000

_SDF = ("<?xml version='1.0'?>"
        "<sdf version='1.8'><model name='{name}'><static>true</static>"
        "<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>"
        "<link name='link'>"
        "<collision name='collision'><geometry><box><size>{w} {w} {h}</size></box></geometry></collision>"
        "<visual name='visual'><geometry><box><size>{w} {w} {h}</size></box></geometry>"
        "<material><ambient>0.9 0.3 0.1 1</ambient><diffuse>0.9 0.3 0.1 1</diffuse></material></visual>"
        "</link></model></sdf>")


def build_obstacle_sdf(name, x_m, y_m):
    return _SDF.format(name=name, x=x_m, y=y_m, z=OBSTACLE_HEIGHT_M / 2,
                       w=OBSTACLE_WIDTH_M, h=OBSTACLE_HEIGHT_M)


def _pb_string(text):
    """Escape text for a protobuf text-format string literal."""
    return text.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")


class SimObstacleSpawner:
    def __init__(self, world=None):
        # Leave world unset to auto-detect it from the running simulation.
        self.world = world or os.environ.get("TRAILBLAZER_WORLD") or None
        self.results = queue.Queue()
        self._names = []
        self._lock = threading.Lock()
        self._counter = itertools.count(1)
        self._session = int(time.time())

    # ---------------- public API (non-blocking) ----------------
    def spawn(self, x_cm, y_cm):
        """Create an obstacle at map position (x_cm, y_cm). Returns its name."""
        name = f"tb_obstacle_{self._session}_{next(self._counter)}"
        threading.Thread(target=self._spawn_job, args=(name, x_cm / 100.0, y_cm / 100.0),
                         daemon=True).start()
        return name

    def remove_all(self):
        """Remove every obstacle this GUI has created from the simulation."""
        with self._lock:
            names, self._names = self._names, []
        if names:
            threading.Thread(target=self._remove_job, args=(names,), daemon=True).start()

    # ---------------- workers ----------------
    def _spawn_job(self, name, x_m, y_m):
        cli = self._find_cli()
        if cli is None:
            return self._fail("Couldn't place the obstacle in the simulation: Gazebo's command line tool "
                              "('ign' or 'gz') wasn't found - source your Ignition / ROS environment "
                              "before starting the GUI.")
        tool, prefix = cli
        world = self._find_world(tool)
        if world is None:
            return self._fail("Couldn't place the obstacle in the simulation: no running Gazebo world "
                              "found - is the simulation running?")

        sdf = build_obstacle_sdf(name, x_m, y_m)
        request = f'sdf: "{_pb_string(sdf)}" name: "{name}"'
        ok, detail = self._call(tool, f"/world/{world}/create", f"{prefix}.msgs.EntityFactory", request)
        if ok:
            with self._lock:
                self._names.append(name)
            self.results.put((True, f"Spawned in the simulation at ({x_m:.2f} m, {y_m:.2f} m)."))
        else:
            self._fail(f"Couldn't place the obstacle in the simulation: it wasn't accepted ({detail})")

    def _remove_job(self, names):
        cli = self._find_cli()
        world = self._find_world(cli[0]) if cli else None
        if cli is None or world is None:
            return self._fail("Couldn't remove the obstacles from the simulation (is it still running?)")
        tool, prefix = cli
        removed = 0
        for name in names:
            ok, _ = self._call(tool, f"/world/{world}/remove", f"{prefix}.msgs.Entity",
                               f'name: "{name}" type: MODEL')
            removed += ok
        self.results.put((removed == len(names),
                          f"Removed {removed} of {len(names)} obstacle(s) from the simulation."))

    # ---------------- helpers ----------------
    def _fail(self, message):
        self.results.put((False, message))

    @staticmethod
    def _find_cli():
        for tool, prefix in (("ign", "ignition"), ("gz", "gz")):
            if shutil.which(tool):
                return tool, prefix
        return None

    def _find_world(self, tool):
        if self.world:
            return self.world
        try:
            out = subprocess.run([tool, "service", "-l"], capture_output=True, text=True, timeout=10).stdout
        except (subprocess.SubprocessError, OSError):
            return None
        match = re.search(r"/world/([^/\s]+)/create", out)
        if match:
            self.world = match.group(1)
        return self.world

    @staticmethod
    def _call(tool, service, reqtype, request):
        cmd = [tool, "service", "-s", service,
               "--reqtype", reqtype, "--reptype", reqtype.rsplit(".", 1)[0] + ".Boolean",
               "--timeout", str(CALL_TIMEOUT_MS), "--req", request]
        try:
            done = subprocess.run(cmd, capture_output=True, text=True, timeout=CALL_TIMEOUT_MS / 1000 + 5)
        except subprocess.TimeoutExpired:
            return False, "timed out"
        except OSError as exc:
            return False, str(exc)
        out = (done.stdout or "").lower()
        if "data: true" in out:
            return True, ""
        return False, (done.stderr or done.stdout or "no reply").strip().splitlines()[0][:120]