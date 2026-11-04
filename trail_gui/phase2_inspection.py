"""
Phase 2 - Autonomous Inspection tab.

The map shows the trail network the real mission inspects: the same trail file
mission_manager visits (junctions J1.. and trails T1.. as numbered in that file,
each drawn along its recorded waypoints; see mission_graph.py). mission_manager
publishes the graph it loaded with each trail's live status, so the colours follow
the real inspection: black = not inspected yet, green = inspected, orange =
blocked with its far side still to check, red = blocked. Red dots mark obstacles the
robot found on a trail, whether it was blocked or went around them. Before
mission_manager is running, the GUI reads the
trail file itself so the map is there straight away.

The dark circle on the map is the robot's ACTUAL position, read live from the
simulation.

Placing an obstacle on the map also places a real obstacle in the simulation
(see sim_obstacles.py): a rock or a fallen tree, picked with the "Rock" /
"Fallen Tree" buttons, so the camera's YOLO detector can be shown naming both.
"Clear Obstacles" removes them again.

"Start Autonomous Inspection" starts the REAL mission: mission_manager holds on
launch until the GUI calls /husky1/start_mission (see mission_status.py), and its
replies and progress appear in the log. Once it is running, "E-STOP" stops the
robot where it is and holds the mission, "Resume" carries on from there, and
"Return Home" skips the trails still to inspect and drives home along the trails.

The obstacle mode button chooses what mission_manager does when an obstacle
blocks a trail: "Backtrack" (the default) marks the trail blocked and goes back
to the previous junction; "Go Around" has Nav2 replan around the obstacle and
carry on along the trail, backtracking only if Nav2 can't get past. It can be
changed at any time and applies from the next obstacle.

The Mission section shows mission_manager's latest status line and a bar of its
exploration percentage (/husky1/mission_progress). The Robot section shows the
obstacle detector's warn/stop flags, the robot's live position and the camera
snapshot taken when an obstacle stop activates (see mission_status.py and
camera_feed.py). The map key lives in a pop-out window ("Map Legend",
legend_window.py) to keep the panel short.
"""
import tkinter as tk
from tkinter import ttk

from camera_panel import CameraPanel
from legend_window import LegendWindow
from map_canvas import MapCanvas
from robot_pose import STATE_COLORS
from sim_obstacles import DEFAULT_OBSTACLE, OBSTACLE_TYPES
import mission_graph
from state import TrailState

POLL_MS = 100              # how often the real robot position / simulation messages are refreshed

# Real obstacle detector state (mission_status.py) -> banner text/colour and log line.
OBSTACLE_BANNER = {
    "clear": ("CLEAR - no obstacle ahead", "#2e7d32"),
    "warn": ("WARNING - obstacle ahead", "#e08e00"),
    "stop": ("STOPPED - obstacle in stop range", "#c0392b"),
}
OBSTACLE_LOG = {
    "clear": "Robot: path ahead clear.",
    "warn": "Robot: obstacle detected ahead.",
    "stop": "Robot: stopped - obstacle in stop range.",
}


class Phase2Frame(ttk.Frame):
    def __init__(self, parent, state: TrailState, on_back, pose_feed=None, spawner=None, on_open_camera=None,
                 status_feed=None, snapshot_feed=None):
        super().__init__(parent)
        self.state = state
        self.on_back = on_back
        self.pose_feed = pose_feed
        self.spawner = spawner
        self.on_open_camera = on_open_camera
        self.status_feed = status_feed
        self.snapshot_feed = snapshot_feed
        self._last_obstacle_level = None
        self._last_mission_text = None
        self._last_snapshot_id = 0
        self._blocked_points = []
        self.placing_obstacles = False
        self.go_around = False      # obstacle mode: False = backtrack to the junction (default), True = Nav2 goes round
        self._graph = None         # the mission's trail network (mission_graph.py)
        self._legend_window = None  # opened on demand, see _open_legend()

        self._build_layout()
        self._poll()

    # ---------------- layout ----------------
    def _build_layout(self):
        main = ttk.Frame(self)
        main.pack(fill="both", expand=True, padx=10, pady=10)

        left = ttk.Frame(main)
        left.pack(side="left", fill="both", expand=True)

        ttk.Label(left, text="Phase 2 - Autonomous Inspection", font=("Arial", 14, "bold")).pack(anchor="w")
        ttk.Label(left, text="15 m x 15 m trail area. Review the mapped trail, seed storm obstacles, "
                              "then run the inspection.", wraplength=760, justify="left").pack(anchor="w", pady=(0, 6))

        self.canvas = MapCanvas(left)
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self._on_canvas_click)

        controls = ttk.Frame(left)
        controls.pack(fill="x", pady=8)
        button_opts = dict(font=("Arial", 11, "bold"), padx=14, pady=6,
                           relief="raised", bd=2, cursor="hand2")
        self.obstacle_btn = tk.Button(controls, text="Place Obstacles (click map)",
                                       command=self._toggle_obstacle_mode, **button_opts)
        self.obstacle_btn.pack(side="left")
        self._obstacle_btn_default_bg = self.obstacle_btn.cget("bg")
        # Which kind the next clicks place (a key of sim_obstacles.OBSTACLE_TYPES)
        self.obstacle_kind = tk.StringVar(value=DEFAULT_OBSTACLE)
        for kind, spec in OBSTACLE_TYPES.items():
            tk.Radiobutton(controls, text=spec["label"], variable=self.obstacle_kind, value=kind,
                           indicatoron=False, selectcolor="#f0c27b",
                           **dict(button_opts, padx=10)).pack(side="left", padx=(6, 0))
        tk.Button(controls, text="Clear Obstacles", command=self.clear_obstacles,
                  **button_opts).pack(side="left", padx=6)
        self.start_btn = tk.Button(controls, text="Start Autonomous Inspection",
                                    command=self._start_inspection, **button_opts)
        self.start_btn.pack(side="right")

        mission_controls = ttk.Frame(left)
        mission_controls.pack(fill="x")
        tk.Button(mission_controls, text="E-STOP", command=self._estop, bg="#c0392b", fg="white",
                  activebackground="#962d22", activeforeground="white",
                  **dict(button_opts, font=("Arial", 13, "bold"), padx=24)).pack(side="left")
        tk.Button(mission_controls, text="Resume", command=self._resume, bg="#2e7d32", fg="white",
                  activebackground="#1b5e20", activeforeground="white", **button_opts).pack(side="left", padx=6)
        tk.Button(mission_controls, text="Return Home", command=self._return_home, bg="#1565c0", fg="white",
                  activebackground="#0d47a1", activeforeground="white", **button_opts).pack(side="left")
        self.obstacle_mode_btn = tk.Button(mission_controls, command=self._toggle_obstacle_mode_setting,
                                           fg="white", activeforeground="white", **button_opts)
        self.obstacle_mode_btn.pack(side="right")
        self._show_obstacle_mode()
        # Which trails the next Start inspects; the label follows the graph mission_manager publishes
        self.trail_source_btn = tk.Button(mission_controls, command=self._toggle_trail_source,
                                          fg="white", activeforeground="white", **button_opts)
        self.trail_source_btn.pack(side="right", padx=6)
        self._show_trail_source()

        right = ttk.Frame(main)
        right.pack(side="left", fill="y", padx=(14, 0))

        small_btn = dict(cursor="hand2", font=("Arial", 10, "bold"), padx=8, pady=3, relief="raised", bd=2)
        nav = ttk.Frame(right)
        nav.pack(anchor="w", fill="x", pady=(0, 10))
        tk.Button(nav, text="< Back", command=self.on_back, **small_btn).pack(side="left")
        if self.on_open_camera is not None:
            tk.Button(nav, text="Live Camera", command=self.on_open_camera, **small_btn).pack(side="left", padx=6)
        tk.Button(nav, text="Map Legend", command=self._open_legend, **small_btn).pack(side="left")

        # Mission: what mission_manager is doing and how much of the network it has explored
        ttk.Label(right, text="Mission", font=("Arial", 11, "bold")).pack(anchor="w")
        self.mission_var = tk.StringVar(value="Ready - review the map, place obstacles, then start.")
        ttk.Label(right, textvariable=self.mission_var, wraplength=300,
                  justify="left").pack(anchor="w", pady=(0, 4))
        bar_row = ttk.Frame(right)
        bar_row.pack(anchor="w", fill="x")
        ttk.Style(self).configure("Exploration.Horizontal.TProgressbar", background="#2e8b32",
                                  troughcolor="#dcdcdc", thickness=16)
        self.exploration_bar = ttk.Progressbar(bar_row, style="Exploration.Horizontal.TProgressbar",
                                               maximum=100, length=240)
        self.exploration_bar.pack(side="left")
        self.exploration_var = tk.StringVar(value="0%")
        ttk.Label(bar_row, textvariable=self.exploration_var, width=5,
                  font=("Arial", 10, "bold")).pack(side="left", padx=(6, 0))
        self.progress_var = tk.StringVar(value="No trail network loaded")
        ttk.Label(right, textvariable=self.progress_var, wraplength=300, foreground="#555555",
                  font=("Arial", 9)).pack(anchor="w", pady=(2, 12))

        # Robot: simulation link, position, obstacle detector and the last obstacle it photographed
        ttk.Label(right, text="Robot", font=("Arial", 11, "bold")).pack(anchor="w")
        self.obstacle_banner = tk.Label(right, font=("Arial", 11, "bold"), fg="white", anchor="w",
                                        padx=8, pady=4, wraplength=300, justify="left")
        self.obstacle_banner.pack(anchor="w", fill="x")
        self._show_obstacle_level(None, "Obstacle detector not connected")
        self.sim_var = tk.StringVar()
        self.pos_var = tk.StringVar(value="Position: waiting for the simulation")
        self.sim_label = ttk.Label(right, textvariable=self.sim_var, wraplength=300, justify="left",
                                   font=("Arial", 9))
        self.sim_label.pack(anchor="w", pady=(2, 0))
        ttk.Label(right, textvariable=self.pos_var, font=("Arial", 9)).pack(anchor="w")
        self.obstacle_name_var = tk.StringVar(value="Detected obstacle: None")
        ttk.Label(right, textvariable=self.obstacle_name_var, font=("Arial", 10, "bold"),
                  wraplength=300).pack(anchor="w", pady=(4, 4))

        if self.snapshot_feed is not None:
            CameraPanel(right, self.snapshot_feed, title="Obstacle Snapshot",
                        placeholder_text="NO SNAPSHOT YET").pack(anchor="w", pady=(0, 10))

        ttk.Label(right, text="Inspection Log", font=("Arial", 11, "bold")).pack(anchor="w")
        log_frame = ttk.Frame(right)
        log_frame.pack(fill="both", expand=True)
        self.log_box = tk.Listbox(log_frame, height=8, width=34)
        self.log_box.pack(side="left", fill="both", expand=True)
        scroll = ttk.Scrollbar(log_frame, command=self.log_box.yview)
        scroll.pack(side="right", fill="y")
        self.log_box.config(yscrollcommand=scroll.set)

    # ---------------- called when this tab becomes active ----------------
    def refresh_from_state(self):
        """Show the mission's trail network: as published by mission_manager if it is running, otherwise
        read from the same trail file it is configured to visit."""
        self.log_box.delete(0, tk.END)
        graph = self.status_feed.get_graph() if self.status_feed is not None else None
        if graph is not None:
            self._log(f"Trail network from mission_manager ({graph['file']}).")
        else:
            path = mission_graph.find_trail_file()
            try:
                if path is None:
                    raise ValueError("no waypoints_file in trail_decision_making/config/mission_manager.yaml")
                graph = mission_graph.load_trail_file(path)
                self._log(f"Trail network from {path}.")
            except (OSError, ValueError) as exc:
                self._log(f"Can't show the trail network: {exc}")
        self._set_graph(graph)
        if self._last_mission_text is None:
            self.mission_var.set("Ready - review the map, place obstacles, then start.")

    def _set_graph(self, graph):
        self._graph = graph
        self._show_trail_source()
        if graph is not None:
            inspected, blocked, todo = mission_graph.summary(graph)
            junctions = sum(1 for n in graph["junctions"] if n["junction"] >= 0)
            cut_off = len(mission_graph.cut_off_trails(graph))
            self.progress_var.set(f"{len(graph['trails'])} trails, {junctions} junctions - {inspected} inspected, "
                                  f"{blocked} blocked" + (f" ({cut_off} at both ends)" if cut_off else "") +
                                  f", {todo} to go")
        else:
            self.progress_var.set("No trail network loaded")
        self._redraw()

    # ---------------- obstacle placement ----------------
    def _toggle_obstacle_mode(self):
        self.placing_obstacles = not self.placing_obstacles
        if self.placing_obstacles:
            self.obstacle_btn.config(relief="sunken", bg="#c0392b", fg="white",
                                      text="Placing Obstacles... (click map)")
        else:
            self.obstacle_btn.config(relief="raised", bg=self._obstacle_btn_default_bg, fg="black",
                                      text="Place Obstacles (click map)")

    def _on_canvas_click(self, event):
        if not self.placing_obstacles:
            return
        wx, wy = self.canvas.screen_to_world(event.x, event.y)
        kind = self.obstacle_kind.get()
        self.state.obstacles.append((wx, wy, kind))
        self._log(f"{OBSTACLE_TYPES[kind]['label']} placed at ({wx:.0f} cm, {wy:.0f} cm)")
        if self.spawner is not None:
            self.spawner.spawn(wx, wy, kind)     # the outcome is logged by _poll when it arrives
        self._redraw()

    def clear_obstacles(self):
        """Remove all obstacles from the map and from the simulation."""
        had = bool(self.state.obstacles)
        self.state.obstacles.clear()
        if self.spawner is not None:
            self.spawner.remove_all()
        if had:
            self._log("Obstacles cleared.")
        self._redraw()

    def abort(self):
        """Stop any inspection in progress (leaving this tab for good)."""
        if self.placing_obstacles:
            self._toggle_obstacle_mode()

    # ---------------- inspection run ----------------
    def _start_inspection(self):
        # mission_manager holds on launch until asked; its progress then comes back on the map colours,
        # the Robot Status section and the log.
        if self.placing_obstacles:
            self._toggle_obstacle_mode()         # leave obstacle mode (and reset the button's look)
        if self.status_feed is None:
            self._log("Can't start: not connected to ROS.")
            return
        self._log("Asking mission_manager to start the real inspection...")
        self.status_feed.request_obstacle_mode(self.go_around)   # in case mission_manager restarted since it was set
        self.status_feed.request_start()       # the reply is logged by _poll_robot_status
        self.mission_var.set("Autonomous inspection requested...")

    # ---------------- mission controls ----------------
    # mission_manager decides whether each makes sense right now (e.g. Resume when not stopped); its reply is
    # logged by _poll_robot_status.
    def _estop(self):
        if self.status_feed is None:
            self._log("Can't e-stop: not connected to ROS.")
            return
        self._log("E-STOP pressed: stopping the robot...")
        self.status_feed.request_pause()
        self.mission_var.set("E-STOP requested...")

    def _resume(self):
        if self.status_feed is None:
            self._log("Can't resume: not connected to ROS.")
            return
        self._log("Asking mission_manager to resume...")
        self.status_feed.request_resume()
        self.mission_var.set("Resume requested...")

    def _return_home(self):
        if self.status_feed is None:
            self._log("Can't return home: not connected to ROS.")
            return
        self._log("Asking mission_manager to skip the remaining trails and return home...")
        self.status_feed.request_return_home()
        self.mission_var.set("Return home requested...")

    def _toggle_obstacle_mode_setting(self):
        self.go_around = not self.go_around
        self._show_obstacle_mode()
        if self.status_feed is None:
            self._log("Obstacle mode will be sent when connected to ROS.")
            return
        self._log("Asking mission_manager to " + ("go around obstacles with Nav2..." if self.go_around
                                                  else "backtrack from obstacles to the junction..."))
        self.status_feed.request_obstacle_mode(self.go_around)

    def _show_obstacle_mode(self):
        if self.go_around:
            self.obstacle_mode_btn.config(text="Obstacles: Go Around (Nav2)", bg="#6a1b9a",
                                          activebackground="#4a148c")
        else:
            self.obstacle_mode_btn.config(text="Obstacles: Backtrack", bg="#546e7a",
                                          activebackground="#37474f")

    def _toggle_trail_source(self):
        if self.status_feed is None:
            self._log("Can't change the trails: not connected to ROS.")
            return
        phase1 = not self._phase1_trails()
        self._log("Asking mission_manager to use the " + ("trails Phase 1 recorded..." if phase1
                                                          else "pre-recorded trails..."))
        self.status_feed.request_trail_source(phase1)   # the new graph comes back on mission_graph

    def _phase1_trails(self):
        return bool(self._graph and self._graph.get("phase1"))

    def _show_trail_source(self):
        if self._phase1_trails():
            self.trail_source_btn.config(text="Trails: Phase 1 Mapped", bg="#ef6c00", activebackground="#e65100")
        else:
            self.trail_source_btn.config(text="Trails: Pre-recorded", bg="#00796b", activebackground="#004d40")

    # ---------------- drawing / logging ----------------
    def _redraw(self):
        self.canvas.draw_mission_graph(self._graph)
        self.canvas.draw_obstacles(self.state.obstacles)
        self.canvas.draw_blocked_points(self._blocked_points)
        self._draw_real_robot()

    def _draw_real_robot(self):
        pose = self.pose_feed.get_pose() if self.pose_feed is not None else None
        if pose is None:
            self.canvas.delete("robot")
        else:
            self.canvas.draw_robot(pose[0], pose[1])        # drawn last, so it sits on top

    def _poll(self):
        """Keep the real robot dot and the simulation status current, and log
        what the simulation said about obstacle requests."""
        while self.spawner is not None and not self.spawner.results.empty():
            _ok, message = self.spawner.results.get_nowait()
            self._log(message)
        self._poll_robot_status()
        if self.winfo_ismapped():
            pose = self.pose_feed.get_pose() if self.pose_feed is not None else None
            if self.pose_feed is not None:
                feed_state, text = self.pose_feed.status()
                self.sim_var.set("Simulation: " + text)
                self.sim_label.configure(foreground=STATE_COLORS.get(feed_state, "black"))
            self.pos_var.set("Position: waiting for the simulation" if pose is None
                             else f"Position: ({int(pose[0])}, {int(pose[1])}) cm")
            self._draw_real_robot()
        try:
            self.after(POLL_MS, self._poll)
        except tk.TclError:
            pass

    def _poll_robot_status(self):
        """Show what the real robot's obstacle detector and mission manager are
        reporting, and log every change (even while this tab is hidden)."""
        if self.status_feed is not None:
            while not self.status_feed.command_results.empty():
                ok, message = self.status_feed.command_results.get_nowait()
                self._log(("Robot: " if ok else "Not done: ") + message)
                if self._last_mission_text is not None:     # replace the "...requested" placeholder
                    self.mission_var.set(self._last_mission_text)
            level, mission_text = self.status_feed.get()
            if level != self._last_obstacle_level:
                if level in OBSTACLE_LOG:
                    self._log(OBSTACLE_LOG[level])
                self._last_obstacle_level = level
            self._show_obstacle_level(level, self.status_feed.status()[1])
            if mission_text is not None and mission_text != self._last_mission_text:
                self.mission_var.set(mission_text)
                self._log("Mission: " + mission_text)
                self._last_mission_text = mission_text
            progress = self.status_feed.get_progress()
            if progress is not None:
                self._show_exploration(progress)
            graph = self.status_feed.get_graph()
            if graph is not None and graph != self._graph:
                known = {t["edge"] for t in mission_graph.cut_off_trails(self._graph)} if self._graph else set()
                for trail in mission_graph.cut_off_trails(graph):
                    if trail["edge"] not in known:
                        self._log(f"Trail {trail['trail']} blocked at both ends: the stretch between the two "
                                  f"obstacles can't be checked.")
                self._set_graph(graph)           # trail statuses changed (or mission_manager just started)
            blocked = self.status_feed.get_blocked_points()
            if blocked != self._blocked_points:
                logged = {p[:5] for p in self._blocked_points}
                for x, y, edge, origin, avoided, _bounds_gap in blocked:
                    if (x, y, edge, origin, avoided) in logged:
                        continue                 # already logged (or only now flagged as cutting off a trail)
                    if avoided:
                        self._log(f"Obstacle on trail {edge} at ({x / 100:.1f}, {y / 100:.1f}) m, "
                                  f"found coming from {origin}: going around it.")
                    else:
                        self._log(f"Trail {edge} blocked at ({x / 100:.1f}, {y / 100:.1f}) m, "
                                  f"found coming from {origin}.")
                self._blocked_points = blocked
                self.canvas.draw_blocked_points(blocked)
        if self.snapshot_feed is not None:
            frame_id, _frame = self.snapshot_feed.get_frame()
            if frame_id != self._last_snapshot_id:
                self._log("Obstacle snapshot captured from the robot's camera.")
                self._last_snapshot_id = frame_id
                if self.status_feed is not None:
                    name = self.status_feed.get_front_obstacle()
                    self.obstacle_name_var.set(f"Detected obstacle: {name}")

    def _show_exploration(self, percent):
        """mission_manager's exploration percentage: inspected trails count fully, blocked ones half until
        their far side has been checked too (TrailGraph::explorationPercentage)."""
        self.exploration_bar["value"] = percent
        self.exploration_var.set(f"{percent:.0f}%")

    def _open_legend(self):
        """Show the map legend window, or bring it to the front if it's already open."""
        if self._legend_window is not None:
            self._legend_window.deiconify()
            self._legend_window.lift()
            return
        self._legend_window = LegendWindow(self, on_closed=self._legend_closed)

    def _legend_closed(self):
        self._legend_window = None

    def _show_obstacle_level(self, level, detector_text):
        text, color = OBSTACLE_BANNER.get(level, (detector_text, "#757575"))
        self.obstacle_banner.configure(text=text, bg=color)

    def _log(self, text):
        self.log_box.insert(tk.END, text)
        self.log_box.see(tk.END)