"""
Phase 2 - Autonomous Inspection tab.

Replays the trail network recorded in Phase 1 as a graph of nodes joined,
in creation order, by edges (black = unvisited). The operator can seed the
map with simulated storm obstacles, then start the autonomous inspection:
the rover travels node to node, turning each edge green once cleared, or
red (and backtracking) if an obstacle blocks it.

The dark circle on the map is the robot's ACTUAL position, read live from the
simulation. The orange circle is the inspection playback: until the real robot
is sent along the route by the navigation stack, the inspection is shown as a
playback on the map (it does not drive the robot).

Placing an obstacle on the map also places a real obstacle in the simulation
(see sim_obstacles.py); "Clear Obstacles" removes them again.

Swap-out point for the real system: `_arrive_at_edge_end` currently checks
a simple point-to-line distance against manually placed obstacle markers.
Once real perception / obstacle-detection is available, replace that check
with the incoming detection result and keep the edge-coloring / backtrack
/ logging logic unchanged.
"""
import tkinter as tk
from tkinter import ttk

from camera_panel import CameraPanel
from map_canvas import MapCanvas
from robot_pose import STATE_COLORS
from state import TrailState

TICK_MS = 25
POLL_MS = 100              # how often the real robot position / simulation messages are refreshed
TRAVEL_SPEED = 5           # cm per tick
OBSTACLE_CLEARANCE = 18    # cm; an edge counts as blocked if an obstacle is this close to it


class Phase2Frame(ttk.Frame):
    def __init__(self, parent, state: TrailState, on_back, pose_feed=None, spawner=None, camera_feed=None):
        super().__init__(parent)
        self.state = state
        self.on_back = on_back
        self.pose_feed = pose_feed
        self.spawner = spawner
        self.camera_feed = camera_feed
        self.mission_visible = False
        self.placing_obstacles = False
        self.running = False
        self.mission_x = self.mission_y = 0
        self.edge_index = 0
        self.target = None
        self.backtracking = False

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
        tk.Button(controls, text="Clear Obstacles", command=self.clear_obstacles,
                  **button_opts).pack(side="left", padx=6)
        self.start_btn = tk.Button(controls, text="Start Autonomous Inspection",
                                    command=self._start_inspection, **button_opts)
        self.start_btn.pack(side="right")

        right = ttk.Frame(main)
        right.pack(side="left", fill="y", padx=(14, 0))

        tk.Button(right, text="< Back to Start", command=self.on_back, cursor="hand2",
                  font=("Arial", 10, "bold"), padx=10, pady=3, relief="raised", bd=2).pack(anchor="w", pady=(0, 12))

        ttk.Label(right, text="Status", font=("Arial", 11, "bold")).pack(anchor="w")
        self.sim_var = tk.StringVar()
        self.pos_var = tk.StringVar(value="Robot position: waiting for the simulation")
        self.sim_label = ttk.Label(right, textvariable=self.sim_var, wraplength=250, justify="left")
        self.sim_label.pack(anchor="w")
        ttk.Label(right, textvariable=self.pos_var, wraplength=250).pack(anchor="w", pady=(0, 6))
        self.phase_var = tk.StringVar(value="Idle")
        self.route_var = tk.StringVar(value="-")
        self.progress_var = tk.StringVar(value="Exploration: 0%")
        ttk.Label(right, textvariable=self.phase_var, wraplength=230).pack(anchor="w")
        ttk.Label(right, textvariable=self.route_var, wraplength=230).pack(anchor="w")
        ttk.Label(right, textvariable=self.progress_var).pack(anchor="w", pady=(0, 10))

        ttk.Label(right, text="Legend", font=("Arial", 11, "bold")).pack(anchor="w")
        ttk.Label(right, justify="left",
                  text="Black = unvisited route\nGreen = clear / passed\n"
                       "Red = blocked by obstacle\nRed X = obstacle (also in the sim)\n"
                       "Dark circle = robot (live from simulation)\n"
                       "Orange circle = inspection playback").pack(anchor="w", pady=(0, 10))

        if self.camera_feed is not None:
            CameraPanel(right, self.camera_feed).pack(anchor="w", pady=(0, 10))

        ttk.Label(right, text="Inspection Log", font=("Arial", 11, "bold")).pack(anchor="w")
        log_frame = ttk.Frame(right)
        log_frame.pack(fill="both", expand=True)
        self.log_box = tk.Listbox(log_frame, height=8, width=34)
        self.log_box.pack(side="left", fill="both", expand=True)
        scroll = ttk.Scrollbar(log_frame, command=self.log_box.yview)
        scroll.pack(side="right", fill="y")
        self.log_box.config(yscrollcommand=scroll.set)

    # ---------------- called when this tab becomes active after Phase 1 ----------------
    def refresh_from_state(self):
        self.state.build_edges_from_nodes()
        for edge in self.state.edges:
            edge.status = "unvisited"
        self.edge_index = 0
        self.running = False
        self.backtracking = False
        self.mission_visible = False
        if self.state.nodes:
            self.mission_x, self.mission_y = self.state.nodes[0].pos()
        self.log_box.delete(0, tk.END)
        self.phase_var.set("Ready - review the map, place obstacles, then start.")
        self.route_var.set("-")
        self.progress_var.set("Exploration: 0%")
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
        if not self.placing_obstacles or self.running:
            return
        wx, wy = self.canvas.screen_to_world(event.x, event.y)
        self.state.obstacles.append((wx, wy))
        self._log(f"Obstacle placed at ({wx:.0f} cm, {wy:.0f} cm)")
        if self.spawner is not None:
            self.spawner.spawn(wx, wy)           # the outcome is logged by _poll when it arrives
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
        self.running = False
        if self.placing_obstacles:
            self._toggle_obstacle_mode()

    # ---------------- inspection run ----------------
    def _start_inspection(self):
        if not self.state.edges:
            self._log("No trail network to inspect - map a route in Phase 1 first.")
            return
        if self.placing_obstacles:
            self._toggle_obstacle_mode()         # leave obstacle mode (and reset the button's look)
        self.running = True
        self.mission_visible = True
        self.edge_index = 0
        self.mission_x, self.mission_y = self.state.nodes[0].pos()
        self.phase_var.set("Mission: Autonomous inspection in progress")
        self._log("Inspection started.")
        self._begin_next_edge()

    def _begin_next_edge(self):
        if self.edge_index >= len(self.state.edges):
            self.running = False
            self.phase_var.set("Mission complete - inspection finished.")
            self._log("All routes inspected.")
            self._redraw()
            return
        edge = self.state.edges[self.edge_index]
        self.route_var.set(f"Inspecting node {edge.node_a.order} -> node {edge.node_b.order}")
        self.backtracking = False
        self.target = edge.node_b.pos()
        self.after(TICK_MS, self._travel_tick)

    def _travel_tick(self):
        if not self.running:
            return
        tx, ty = self.target
        dx, dy = tx - self.mission_x, ty - self.mission_y
        dist = (dx ** 2 + dy ** 2) ** 0.5
        if dist <= TRAVEL_SPEED:
            self.mission_x, self.mission_y = tx, ty
            self._redraw()
            if self.backtracking:
                self.edge_index += 1
                self.after(200, self._begin_next_edge)
            else:
                self._arrive_at_edge_end()
            return
        self.mission_x += dx / dist * TRAVEL_SPEED
        self.mission_y += dy / dist * TRAVEL_SPEED
        self._redraw()
        self.after(TICK_MS, self._travel_tick)

    def _arrive_at_edge_end(self):
        edge = self.state.edges[self.edge_index]
        blocked = any(edge.distance_to_point(ox, oy) <= OBSTACLE_CLEARANCE
                       for (ox, oy) in self.state.obstacles)
        if blocked:
            edge.status = "blocked"
            self._log(f"Node {edge.node_a.order} -> {edge.node_b.order}: obstacle detected, "
                      f"marked blocked. Backtracking to node {edge.node_a.order}.")
            self.backtracking = True
            self.target = edge.node_a.pos()
            self.after(200, self._travel_tick)
        else:
            edge.status = "clear"
            self._log(f"Node {edge.node_a.order} -> {edge.node_b.order}: clear, route accessible.")
            self.edge_index += 1
            self.after(150, self._begin_next_edge)
        self.progress_var.set(f"Exploration: {self.state.exploration_percentage():.0f}%")
        self._redraw()

    # ---------------- drawing / logging ----------------
    def _redraw(self):
        self.canvas.draw_edges(self.state.edges)
        self.canvas.draw_nodes(self.state.nodes)
        self.canvas.draw_obstacles(self.state.obstacles)
        if self.mission_visible:
            self.canvas.draw_robot(self.mission_x, self.mission_y, tag="mission", color="#ff9800")
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
        if self.winfo_ismapped():
            pose = self.pose_feed.get_pose() if self.pose_feed is not None else None
            if self.pose_feed is not None:
                feed_state, text = self.pose_feed.status()
                self.sim_var.set("Simulation: " + text)
                self.sim_label.configure(foreground=STATE_COLORS.get(feed_state, "black"))
            self.pos_var.set("Robot position: waiting for the simulation" if pose is None
                             else f"Robot position: ({int(pose[0])}, {int(pose[1])}) cm")
            self._draw_real_robot()
        try:
            self.after(POLL_MS, self._poll)
        except tk.TclError:
            pass

    def _log(self, text):
        self.log_box.insert(tk.END, text)
        self.log_box.see(tk.END)