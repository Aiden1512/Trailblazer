"""
Phase 1 - Teleop Mapping tab.

The robot is driven inside the simulation (Ignition Gazebo); this tab does
not move it. Instead the map follows the robot's ACTUAL position, read from
the simulation's odometry (see robot_pose.py), and records the trail as it
goes:

  * a waypoint node is dropped every MIN_NODE_SPACING cm travelled;
  * where the path turns sharply, that node becomes a junction;
  * the operator can also mark a junction by hand with SPACE / the button.

Coordinates are real-world centimetres with the origin (0, 0) at the centre
of the 15 m x 15 m map, +x to the right and +y up. (The W / A / S / D legend
is kept as a reminder of the drive controls, but the GUI no longer reacts to
those keys.)
"""
import math
import tkinter as tk
from tkinter import ttk, messagebox

import state as state_module
from camera_panel import CameraPanel
from map_canvas import MapCanvas
from robot_pose import STATE_COLORS
from state import TrailState

TICK_MS = 30                    # GUI refresh, ~33 fps
TRAIL_STEP_CM = 2.0             # record a trail point each time the robot has moved this far
MIN_NODE_SPACING = 55           # cm of travel between auto-dropped waypoint nodes
TURN_ANGLE_DEG = 40             # a bend sharper than this turns the corner node into a junction
JUNCTION_MERGE_CM = 5           # a manual junction this close to the last node just upgrades it


class Phase1Frame(ttk.Frame):
    def __init__(self, parent, state: TrailState, on_finish_mapping, on_back,
                 pose_feed=None, on_reset=None, camera_feed=None):
        super().__init__(parent)
        self.state = state
        self.on_finish_mapping = on_finish_mapping
        self.on_back = on_back
        self.pose_feed = pose_feed
        self.on_reset = on_reset
        self.camera_feed = camera_feed

        self.running = False
        self._tick_job = None
        self.robot_x = self.robot_y = None       # None until the sim reports a position
        self._started = False                    # True once the start node has been dropped
        self._last_recorded = None
        self.distance_since_last_node = 0.0
        self._dirty = True                       # trail/nodes need redrawing

        self._build_layout()

    # ---------------- layout ----------------
    def _build_layout(self):
        main = ttk.Frame(self)
        main.pack(fill="both", expand=True, padx=10, pady=10)

        left = ttk.Frame(main)
        left.pack(side="left", fill="both", expand=True)

        ttk.Label(left, text="Phase 1 - Teleop Mapping", font=("Arial", 14, "bold")).pack(anchor="w")
        # wraplength keeps this no wider than the map, so it can't squeeze the side panel
        ttk.Label(left, text="15 m x 15 m trail area. Drive the robot in the simulation and the map follows it; "
                              "click the map, then press SPACE to mark a junction.",
                  wraplength=760, justify="left").pack(anchor="w", pady=(0, 6))

        self.canvas = MapCanvas(left)   # loads the bundled floor texture automatically
        self.canvas.pack()
        self.canvas.bind("<space>", lambda e: self._drop_junction())
        self.canvas.bind("<Button-1>", lambda e: self.canvas.focus_set())

        button_opts = dict(font=("Arial", 11, "bold"), padx=14, pady=6,
                           relief="raised", bd=2, cursor="hand2")

        controls = ttk.Frame(left)
        controls.pack(fill="x", pady=8)
        tk.Button(controls, text="Reset Run", command=self._reset_run,
                  **button_opts).pack(side="left")
        tk.Button(controls, text="Drop Junction (Space)", command=self._drop_junction,
                  **button_opts).pack(side="left", padx=6)
        tk.Button(controls, text="Finish Mapping -> Phase 2", command=self._finish_mapping,
                  **button_opts).pack(side="right")

        right = ttk.Frame(main)
        right.pack(side="left", fill="y", padx=(14, 0))

        tk.Button(right, text="< Back to Start", command=self.on_back, cursor="hand2",
                  font=("Arial", 10, "bold"), padx=10, pady=3, relief="raised", bd=2).pack(anchor="w", pady=(0, 12))

        ttk.Label(right, text="Controls", font=("Arial", 11, "bold")).pack(anchor="w")
        ttk.Label(right, justify="left",
                  text="W - forward\nA - left\nS - backward\nD - right\nSPACE - mark junction").pack(anchor="w", pady=(0, 12))

        ttk.Label(right, text="Status", font=("Arial", 11, "bold")).pack(anchor="w")
        self.sim_var = tk.StringVar()
        self.pos_var = tk.StringVar()
        self.node_var = tk.StringVar()
        self.sim_label = ttk.Label(right, textvariable=self.sim_var, wraplength=250, justify="left")
        self.sim_label.pack(anchor="w")
        ttk.Label(right, textvariable=self.pos_var).pack(anchor="w")
        ttk.Label(right, textvariable=self.node_var).pack(anchor="w", pady=(0, 12))

        if self.camera_feed is not None:
            CameraPanel(right, self.camera_feed).pack(anchor="w", pady=(0, 12))

        ttk.Label(right, text="Legend", font=("Arial", 11, "bold")).pack(anchor="w")
        ttk.Label(right, justify="left",
                  text="Dashed yellow = travelled trail\nBlue dot = junction node\n"
                       "Light dot = waypoint node\nDark circle = robot (live from simulation)").pack(anchor="w")

    # ---------------- run lifecycle ----------------
    def begin(self):
        """Start a fresh mapping run (called when the operator picks 'Start a new simulation')."""
        self._reset_run()

    def halt(self):
        """Stop following the robot (leaving this tab for good)."""
        self.running = False
        if self._tick_job is not None:
            self.after_cancel(self._tick_job)
            self._tick_job = None

    def _reset_run(self):
        if self.on_reset is not None:
            self.on_reset()                      # e.g. take leftover obstacles out of the simulation
        self.state.reset()
        self._started = False
        self._last_recorded = None
        self.distance_since_last_node = 0.0
        self._dirty = True
        self.canvas.draw_trail([])
        self.canvas.draw_nodes([])
        self.running = True
        self.canvas.focus_set()
        # Cancel any loop that's already running so pressing "Reset Run" can't stack up extra loops.
        if self._tick_job is not None:
            self.after_cancel(self._tick_job)
        self._tick_job = self.after(TICK_MS, self._tick)

    def _finish_mapping(self):
        if len(self.state.nodes) < 2:
            messagebox.showwarning("Not enough trail",
                                   "Drive the robot around in the simulation a bit before finishing mapping.")
            return
        self.halt()
        try:
            state_module.save_map(self.state)    # so 'Continue where you left off' has something to load
        except OSError as exc:
            messagebox.showwarning("Couldn't save the map",
                                   f"The trail couldn't be saved for next time:\n{exc}")
        self.on_finish_mapping()

    def focus_canvas(self):
        self.canvas.focus_set()

    # ---------------- following the robot ----------------
    def _tick(self):
        if not self.running:
            return
        pose = self.pose_feed.get_pose() if self.pose_feed is not None else None
        if pose is not None:
            self._follow(pose[0], pose[1])
        self._refresh_status()
        self._redraw()
        self._tick_job = self.after(TICK_MS, self._tick)

    def _follow(self, x, y):
        """Update the robot's position from the simulation and record the trail."""
        self.robot_x, self.robot_y = x, y

        if not self._started:                    # first position: this is where the trail starts
            self.state.add_node(x, y, kind="junction")
            self.state.trail_points.append((x, y))
            self._last_recorded = (x, y)
            self._started = True
            self._dirty = True
            return

        step = math.hypot(x - self._last_recorded[0], y - self._last_recorded[1])
        if step < TRAIL_STEP_CM:
            return                               # not moved (or just sensor jitter)
        self.state.trail_points.append((x, y))
        self._last_recorded = (x, y)
        self.distance_since_last_node += step
        self._dirty = True

        if self.distance_since_last_node >= MIN_NODE_SPACING:
            nodes = self.state.nodes
            if len(nodes) >= 2 and self._bend_degrees(nodes[-2], nodes[-1], (x, y)) > TURN_ANGLE_DEG:
                nodes[-1].kind = "junction"      # the path turned at the last node
            self.state.add_node(x, y, kind="waypoint")
            self.distance_since_last_node = 0.0

    @staticmethod
    def _bend_degrees(a, b, c):
        """How sharply the path a -> b -> c turns at b, in degrees (0 = straight on)."""
        abx, aby = b.x - a.x, b.y - a.y
        bcx, bcy = c[0] - b.x, c[1] - b.y
        if math.hypot(abx, aby) < 10 or math.hypot(bcx, bcy) < 10:
            return 0.0
        return abs(math.degrees(math.atan2(abx * bcy - aby * bcx, abx * bcx + aby * bcy)))

    def _drop_junction(self):
        """Mark a junction at the robot's current position."""
        if not self.running or not self._started:
            return
        last = self.state.nodes[-1]
        if math.hypot(self.robot_x - last.x, self.robot_y - last.y) < JUNCTION_MERGE_CM:
            last.kind = "junction"
        else:
            self.state.add_node(self.robot_x, self.robot_y, kind="junction")
            self.distance_since_last_node = 0.0
        self._dirty = True

    # ---------------- drawing / status ----------------
    def _redraw(self):
        if self._dirty:
            self.canvas.draw_trail(self.state.trail_points)
            self.canvas.draw_nodes(self.state.nodes)
            self._dirty = False
        if self.robot_x is not None:
            self.canvas.draw_robot(self.robot_x, self.robot_y)

    def _refresh_status(self):
        if self.pose_feed is not None:
            feed_state, text = self.pose_feed.status()
            self.sim_var.set("Simulation: " + text)
            self.sim_label.configure(foreground=STATE_COLORS.get(feed_state, "black"))
        if self.robot_x is None:
            self.pos_var.set("Position: waiting for the simulation")
        else:
            self.pos_var.set(f"Position: ({int(self.robot_x)}, {int(self.robot_y)}) cm")
        self.node_var.set(f"Nodes recorded: {len(self.state.nodes)}")