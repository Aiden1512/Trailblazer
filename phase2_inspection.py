"""
Phase 2 - Autonomous Inspection tab.

Replays the trail network recorded in Phase 1 as a graph of nodes joined,
in creation order, by edges (black = unvisited). The operator can seed the
map with simulated storm obstacles, then start the autonomous inspection:
the rover travels node to node, turning each edge green once cleared, or
red (and backtracking) if an obstacle blocks it.

Swap-out point for the real system: `_arrive_at_edge_end` currently checks
a simple point-to-line distance against manually placed obstacle markers.
Once real perception / obstacle-detection is available, replace that check
with the incoming detection result and keep the edge-coloring / backtrack
/ logging logic unchanged.
"""
import tkinter as tk
from tkinter import ttk

from map_canvas import MapCanvas
from state import TrailState

TICK_MS = 25
TRAVEL_SPEED = 5
OBSTACLE_CLEARANCE = 18   # px; an edge counts as blocked if an obstacle is this close to it


class Phase2Frame(ttk.Frame):
    def __init__(self, parent, state: TrailState):
        super().__init__(parent)
        self.state = state
        self.placing_obstacles = False
        self.running = False
        self.robot_x = self.robot_y = 0
        self.edge_index = 0
        self.target = None
        self.backtracking = False

        self._build_layout()

    # ---------------- layout ----------------
    def _build_layout(self):
        main = ttk.Frame(self)
        main.pack(fill="both", expand=True, padx=10, pady=10)

        left = ttk.Frame(main)
        left.pack(side="left", fill="both", expand=True)

        ttk.Label(left, text="Phase 2 - Autonomous Inspection", font=("Arial", 14, "bold")).pack(anchor="w")
        ttk.Label(left, text="Review the mapped trail, seed storm obstacles, then run the inspection.").pack(anchor="w", pady=(0, 6))

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
        tk.Button(controls, text="Clear Obstacles", command=self._clear_obstacles,
                  **button_opts).pack(side="left", padx=6)
        self.start_btn = tk.Button(controls, text="Start Autonomous Inspection",
                                    command=self._start_inspection, **button_opts)
        self.start_btn.pack(side="right")

        right = ttk.Frame(main, width=250)
        right.pack(side="left", fill="y", padx=(14, 0))

        ttk.Label(right, text="Status", font=("Arial", 11, "bold")).pack(anchor="w")
        self.phase_var = tk.StringVar(value="Idle")
        self.route_var = tk.StringVar(value="-")
        self.progress_var = tk.StringVar(value="Exploration: 0%")
        ttk.Label(right, textvariable=self.phase_var, wraplength=230).pack(anchor="w")
        ttk.Label(right, textvariable=self.route_var, wraplength=230).pack(anchor="w")
        ttk.Label(right, textvariable=self.progress_var).pack(anchor="w", pady=(0, 10))

        ttk.Label(right, text="Legend", font=("Arial", 11, "bold")).pack(anchor="w")
        ttk.Label(right, justify="left",
                  text="Black = unvisited route\nGreen = clear / passed\n"
                       "Red = blocked by obstacle\nRed X = detected obstacle").pack(anchor="w", pady=(0, 10))

        ttk.Label(right, text="Inspection Log", font=("Arial", 11, "bold")).pack(anchor="w")
        log_frame = ttk.Frame(right)
        log_frame.pack(fill="both", expand=True)
        self.log_box = tk.Listbox(log_frame, height=18, width=34)
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
        if self.state.nodes:
            self.robot_x, self.robot_y = self.state.nodes[0].pos()
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
        self.state.obstacles.append((event.x, event.y))
        self._log(f"Obstacle placed at ({event.x}, {event.y})")
        self._redraw()

    def _clear_obstacles(self):
        self.state.obstacles.clear()
        self._redraw()

    # ---------------- inspection run ----------------
    def _start_inspection(self):
        if not self.state.edges:
            self._log("No trail network to inspect - map a route in Phase 1 first.")
            return
        self.placing_obstacles = False
        self.running = True
        self.edge_index = 0
        self.robot_x, self.robot_y = self.state.nodes[0].pos()
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
        dx, dy = tx - self.robot_x, ty - self.robot_y
        dist = (dx ** 2 + dy ** 2) ** 0.5
        if dist <= TRAVEL_SPEED:
            self.robot_x, self.robot_y = tx, ty
            self._redraw()
            if self.backtracking:
                self.edge_index += 1
                self.after(200, self._begin_next_edge)
            else:
                self._arrive_at_edge_end()
            return
        self.robot_x += dx / dist * TRAVEL_SPEED
        self.robot_y += dy / dist * TRAVEL_SPEED
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
        self.canvas.draw_robot(self.robot_x, self.robot_y, color="#2a2a2a")

    def _log(self, text):
        self.log_box.insert(tk.END, text)
        self.log_box.see(tk.END)