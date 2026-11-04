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
from map_canvas import MapCanvas
from robot_pose import STATE_COLORS
from state import TrailState, Node


TICK_MS = 30                    # GUI refresh, ~33 fps
TRAIL_STEP_CM = 2.0             # record a trail point each time the robot has moved this far
MIN_NODE_SPACING = 55           # cm of travel between auto-dropped waypoint nodes
TURN_ANGLE_DEG = 40             # a bend sharper than this turns the corner node into a junction
JUNCTION_MERGE_CM = 5           # a manual junction this close to the last node just upgrades it

# water_guard (trail_object_detection) watches the camera for blue ground and
# blocks forward teleop near it; this tab only reports what it is doing.
WATER_TEXT = {
    "stop": ("Water: TOO CLOSE - forward blocked, reverse or turn away", "#c0392b"),
    "warn": ("Water: ahead - forward speed reduced", "#b26a00"),
    "clear": ("Water: none ahead", "#2e7d32"),
    None: ("Water guard: not running (or no camera)", "#b26a00"),
}


class Phase1Frame(ttk.Frame):
    def __init__(self, parent, state: TrailState, on_finish_mapping, on_back, pose_feed=None, on_reset=None, on_open_camera=None, status_feed=None):
        super().__init__(parent)
        self.state = state
        self.on_finish_mapping = on_finish_mapping
        self.on_back = on_back
        self.pose_feed = pose_feed
        self.on_reset = on_reset
        self.on_open_camera = on_open_camera
        self.status_feed = status_feed

        self.running = False
        self.recording = False
        self._tick_job = None
        self.robot_x = self.robot_y = None       # None until the sim reports a position
        self._started = False                    # True once the start node has been dropped
        self._last_recorded = None
        self.distance_since_last_node = 0.0
        self._dirty = True                       # trail/nodes need redrawing
        self._last_graph_version = -1
        self._display_nodes = []
        self._display_cache = {}

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
                              "and automatically records junctions and waypoints",
                  wraplength=760, justify="left").pack(anchor="w", pady=(0, 6))

        self.canvas = MapCanvas(left)   # loads the bundled floor texture automatically
        self.canvas.pack()
        self.canvas.bind("<Button-1>", lambda e: self.canvas.focus_set())
        self.canvas.bind("<KeyPress>", self._on_key_press)

        button_opts = dict(font=("Arial", 11, "bold"), padx=14, pady=6, relief="raised", bd=2, cursor="hand2")
        controls = ttk.Frame(left)
        controls.pack(fill="x", pady=8)
        tk.Button(controls, text="Reset Run", command=self._reset_run,
                  **button_opts).pack(side="left")
        
        self.record_button = tk.Button(controls, text="Start Recording", command=self._toggle_recording, **button_opts)
        self.record_button.pack(side="left", padx=8)
        self._record_button_default_bg = self.record_button.cget("bg")


        tk.Button(controls, text="Finish Mapping -> Phase 2", command=self._finish_mapping,
                  **button_opts).pack(side="right")

        right = ttk.Frame(main)
        right.pack(side="left", fill="y", padx=(14, 0))

        tk.Button(right, text="< Back to Start", command=self.on_back, cursor="hand2",
                  font=("Arial", 10, "bold"), padx=10, pady=3, relief="raised", bd=2).pack(anchor="w", pady=(0, 6))
        if self.on_open_camera is not None:
            tk.Button(right, text="View Live Camera", command=self.on_open_camera, cursor="hand2",
                      font=("Arial", 10, "bold"), padx=10, pady=3, relief="raised", bd=2).pack(anchor="w", pady=(0, 12))

        ttk.Label(right, text="Controls", font=("Arial", 11, "bold")).pack(anchor="w")
        ttk.Label(right, justify="left",
                  text="W - forward\nA - left\nS - backward\nD - right\nSPACE - stop").pack(anchor="w", pady=(0, 12))

        ttk.Label(right, text="Status", font=("Arial", 11, "bold")).pack(anchor="w")
        self.sim_var = tk.StringVar()
        self.pos_var = tk.StringVar()
        self.node_var = tk.StringVar()
        self.sim_label = ttk.Label(right, textvariable=self.sim_var, wraplength=250, justify="left")
        self.sim_label.pack(anchor="w")
        ttk.Label(right, textvariable=self.pos_var).pack(anchor="w")
        ttk.Label(right, textvariable=self.node_var).pack(anchor="w")
        self.water_var = tk.StringVar()
        self.water_label = ttk.Label(right, textvariable=self.water_var, wraplength=250, justify="left")
        if self.status_feed is not None:
            self.water_label.pack(anchor="w")
        ttk.Frame(right).pack(pady=(0, 12))      # gap before the next section

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
        if self.pose_feed is not None:
            self.pose_feed.send_velocity()

    def _reset_run(self):
        if self.pose_feed is not None:
            self.pose_feed.reset_graph()
        self._last_graph_version = -1                     # e.g. take leftover obstacles out of the simulation
        self.state.reset()
        self.recording = False
        self._show_recording()
        self._started = False
        self._display_cache.clear()
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

    def _toggle_recording(self):
        if self.recording:
            self._stop_recording()
            return
        if self.pose_feed is None or not self.pose_feed.start_recording():
            messagebox.showwarning("Recording unavailable", "Could not contact the junction recorder.")
            return
        self.recording = True
        self._show_recording()

    def _stop_recording(self):
        """Stop the junction recorder adding to the trail (it keeps what it has). Pressing Start Recording
        again carries on the same trail from wherever the robot is then."""
        if not self.recording:
            return
        if self.pose_feed is None or not self.pose_feed.stop_recording():
            messagebox.showwarning("Recording unavailable", "Could not contact the junction recorder to stop it.")
            return
        self.recording = False
        self._last_recorded = None
        self._show_recording()

    def _show_recording(self):
        if self.recording:
            self.record_button.config(text="Stop Recording", bg="#c0392b", fg="white",
                                      activebackground="#962d22", activeforeground="white")
        else:
            self.record_button.config(text="Start Recording", bg=self._record_button_default_bg, fg="black",
                                      activebackground=self._record_button_default_bg, activeforeground="black")

    def _finish_mapping(self):
        if len(self.state.nodes) < 2:
            messagebox.showwarning("Not enough trail",
                                   "Drive the robot around in the simulation a bit before finishing mapping.")
            return
        self._stop_recording()                   # otherwise the recorder carries on through Phase 2
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
        self._update_graph()
        self._refresh_status()
        self._redraw()
        self._tick_job = self.after(TICK_MS, self._tick)

    def _follow(self, x, y):
        """Update the robot's position from the simulation and record the trail."""
        self.robot_x, self.robot_y = x, y
        if not self.recording:
            return

        if not self._started:
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
        self._dirty = True
        
    @staticmethod
    def _bend_degrees(a, b, c):
        """How sharply the path a -> b -> c turns at b, in degrees (0 = straight on)."""
        abx, aby = b.x - a.x, b.y - a.y
        bcx, bcy = c[0] - b.x, c[1] - b.y
        if math.hypot(abx, aby) < 10 or math.hypot(bcx, bcy) < 10:
            return 0.0
        return abs(math.degrees(math.atan2(abx * bcy - aby * bcx, abx * bcx + aby * bcy)))
    
    def _update_graph(self):
        if self.pose_feed is None:
            return

        version, graph = self.pose_feed.get_graph()

        if graph is None or version == self._last_graph_version:
            return

        nodes = []
        display_nodes = []
        

        for junction in graph.get("junctions", []):
            node = Node(junction["x"] * 100, junction["y"] * 100, kind="junction")
            node.id = junction["id"]
            node.order = junction["id"]
            nodes.append(node)

            key = ("junction", junction["id"])
            if key not in self._display_cache:
                pos = self.pose_feed.map_to_display(junction["x"], junction["y"])
                if pos is None:
                    return
                self._display_cache[key] = pos
            pos = self._display_cache[key]   
            display_node = Node(pos[0] * 100, pos[1] * 100, kind="junction")
            display_node.id = junction["id"]
            display_node.order = junction["id"]
            display_nodes.append(display_node)


        for trail in graph.get("trails", []):
            for index, waypoint in enumerate(trail.get("waypoints", [])):
                nodes.append(Node( waypoint["x"] * 100, waypoint["y"] * 100, kind="waypoint" ))
                key = ("waypoint", trail["id"], index)
                if key not in self._display_cache:
                    pos = self.pose_feed.map_to_display(waypoint["x"], waypoint["y"])
                    if pos is None:
                        return
                    self._display_cache[key] = pos
                pos = self._display_cache[key] 
                display_nodes.append(Node(pos[0] * 100, pos[1] * 100, kind="waypoint"))      
        
        self.state.nodes = nodes
        self._display_nodes = display_nodes
        self.state.display_nodes = display_nodes
        self._last_graph_version = version
        self._dirty = True

    # ---------------- drawing / status ----------------
    def _redraw(self):
        if self._dirty:
            self.canvas.draw_trail(self.state.trail_points)
            self.canvas.draw_nodes(self._display_nodes)
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
        if self.status_feed is not None:
            text, colour = WATER_TEXT[self.status_feed.get_water()]
            self.water_var.set(text)
            self.water_label.configure(foreground=colour)

    # ---------------- teleop ----------------
    def _on_key_press(self, event):
        if self.pose_feed is None:
            return

        key = event.keysym.lower()

        if key == "w":
            self.pose_feed.send_velocity(linear=0.5)
        elif key == "s":
            self.pose_feed.send_velocity(linear=-0.5)
        elif key == "a":
            self.pose_feed.send_velocity(angular=0.8)
        elif key == "d":
            self.pose_feed.send_velocity(angular=-0.8)
        elif key == "space":
            self.pose_feed.send_velocity()
        else:
            return

        return "break"
