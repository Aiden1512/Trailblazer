"""
Phase 1 - Teleop Mapping tab.

The ranger drives the simulated skid-steer rover around the trail map with
W / A / S / D (forward / left / backward / right). As the rover moves,
waypoint nodes are dropped automatically along the trail, and the operator
can drop an explicit "trail junction" node at any time with SPACE.

Swap-out point for the real system: `_move_robot` currently moves the robot
directly from key state. Once real teleop + SLAM odometry is available,
replace the body of `_move_robot` with the incoming pose from that topic
and keep the node-dropping logic below it unchanged.
"""
import tkinter as tk
from tkinter import ttk, messagebox

from map_canvas import MapCanvas
from state import TrailState

SPEED = 4                      # pixels per tick
TICK_MS = 30                    # ~33 fps
MIN_NODE_SPACING = 55           # px between auto-dropped waypoint nodes
DIRECTION_KEYS = {"w": (0, -1), "s": (0, 1), "a": (-1, 0), "d": (1, 0)}


class Phase1Frame(ttk.Frame):
    def __init__(self, parent, state: TrailState, on_finish_mapping):
        super().__init__(parent)
        self.state = state
        self.on_finish_mapping = on_finish_mapping

        self.pressed_keys = set()
        self.last_direction = None
        self.robot_x, self.robot_y = 60, 60
        self.distance_since_last_node = 0.0
        self.running = False

        self._build_layout()
        self._reset_run()

    # ---------------- layout ----------------
    def _build_layout(self):
        main = ttk.Frame(self)
        main.pack(fill="both", expand=True, padx=10, pady=10)

        left = ttk.Frame(main)
        left.pack(side="left", fill="both", expand=True)

        ttk.Label(left, text="Phase 1 - Teleop Mapping", font=("Arial", 14, "bold")).pack(anchor="w")
        ttk.Label(left, text="Drive the rover to map the trail network. Click the map to focus keyboard input.").pack(anchor="w", pady=(0, 6))

        self.canvas = MapCanvas(left)
        self.canvas.pack()
        self.canvas.bind("<KeyPress>", self._on_key_press)
        self.canvas.bind("<KeyRelease>", self._on_key_release)
        self.canvas.bind("<Button-1>", lambda e: self.canvas.focus_set())

        controls = ttk.Frame(left)
        controls.pack(fill="x", pady=8)
        button_opts = dict(font=("Arial", 11, "bold"), padx=14, pady=6,
                           relief="raised", bd=2, cursor="hand2")
        tk.Button(controls, text="Reset Run", command=self._reset_run,
                  **button_opts).pack(side="left")
        tk.Button(controls, text="Drop Junction (Space)", command=self._drop_junction,
                  **button_opts).pack(side="left", padx=6)
        tk.Button(controls, text="Finish Mapping -> Phase 2", command=self._finish_mapping,
                  **button_opts).pack(side="right")

        right = ttk.Frame(main, width=220)
        right.pack(side="left", fill="y", padx=(14, 0))

        ttk.Label(right, text="Controls", font=("Arial", 11, "bold")).pack(anchor="w")
        ttk.Label(right, justify="left",
                  text="W - forward\nA - left\nS - backward\nD - right\nSPACE - mark junction").pack(anchor="w", pady=(0, 12))

        ttk.Label(right, text="Status", font=("Arial", 11, "bold")).pack(anchor="w")
        self.pos_var = tk.StringVar()
        self.node_var = tk.StringVar()
        ttk.Label(right, textvariable=self.pos_var).pack(anchor="w")
        ttk.Label(right, textvariable=self.node_var).pack(anchor="w", pady=(0, 12))

        ttk.Label(right, text="Legend", font=("Arial", 11, "bold")).pack(anchor="w")
        ttk.Label(right, justify="left",
                  text="Dashed grey = travelled trail\nBlue dot = junction node\n"
                       "Light dot = waypoint node\nDark circle = rover").pack(anchor="w")

    # ---------------- run lifecycle ----------------
    def _reset_run(self):
        self.state.reset()
        self.robot_x, self.robot_y = 60, 60
        self.distance_since_last_node = 0.0
        self.last_direction = None
        self.state.add_node(self.robot_x, self.robot_y, kind="junction")  # ranger station / start
        self._redraw()
        self.running = True
        self.canvas.focus_set()
        self.after(TICK_MS, self._tick)

    def _finish_mapping(self):
        if len(self.state.nodes) < 2:
            messagebox.showwarning("Not enough trail", "Drive around a bit before finishing mapping.")
            return
        self.running = False
        self.on_finish_mapping()

    def focus_canvas(self):
        self.canvas.focus_set()

    # ---------------- input handling ----------------
    def _on_key_press(self, event):
        key = event.keysym.lower()
        if key in DIRECTION_KEYS:
            self.pressed_keys.add(key)
        elif key == "space":
            self._drop_junction()

    def _on_key_release(self, event):
        self.pressed_keys.discard(event.keysym.lower())

    # ---------------- simulation loop ----------------
    def _tick(self):
        if not self.running:
            return
        self._move_robot()
        self._redraw()
        self.after(TICK_MS, self._tick)

    def _move_robot(self):
        dx = dy = 0
        for key in self.pressed_keys:
            vx, vy = DIRECTION_KEYS[key]
            dx += vx
            dy += vy
        if dx == 0 and dy == 0:
            return

        norm = (dx ** 2 + dy ** 2) ** 0.5
        dx, dy = (dx / norm) * SPEED, (dy / norm) * SPEED
        new_x, new_y = self.canvas.clamp(self.robot_x + dx, self.robot_y + dy)
        step = ((new_x - self.robot_x) ** 2 + (new_y - self.robot_y) ** 2) ** 0.5
        self.robot_x, self.robot_y = new_x, new_y
        self.state.trail_points.append((self.robot_x, self.robot_y))
        self.distance_since_last_node += step

        direction = (round(dx, 2), round(dy, 2))
        changed_direction = self.last_direction is not None and direction != self.last_direction
        self.last_direction = direction

        if changed_direction and self.distance_since_last_node > 20:
            self._auto_drop_node(kind="junction")
        elif self.distance_since_last_node >= MIN_NODE_SPACING:
            self._auto_drop_node(kind="waypoint")

    def _auto_drop_node(self, kind):
        self.state.add_node(self.robot_x, self.robot_y, kind=kind)
        self.distance_since_last_node = 0.0

    def _drop_junction(self):
        if not self.running:
            return
        self._auto_drop_node(kind="junction")
        self._redraw()

    # ---------------- drawing ----------------
    def _redraw(self):
        self.canvas.draw_trail(self.state.trail_points)
        self.canvas.draw_nodes(self.state.nodes)
        self.canvas.draw_robot(self.robot_x, self.robot_y)
        self.pos_var.set(f"Position: ({int(self.robot_x)}, {int(self.robot_y)})")
        self.node_var.set(f"Nodes recorded: {len(self.state.nodes)}")