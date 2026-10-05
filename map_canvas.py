"""
Shared canvas drawing helpers for the trail map, used by both GUI phases
so Phase 1 and Phase 2 look and behave consistently.
"""
import tkinter as tk

MAP_BG = "#e7f0e2"          # pale "national park" green
GRID_COLOR = "#cddbc4"
TRAIL_COLOR = "#8d8d8d"
NODE_RADIUS = 7
ROBOT_RADIUS = 9


class MapCanvas(tk.Canvas):
    """A canvas pre-configured to look like a simple top-down trail map."""

    def __init__(self, parent, width=760, height=560, **kwargs):
        super().__init__(parent, width=width, height=height, bg=MAP_BG,
                          highlightthickness=1, highlightbackground="#9fae95", **kwargs)
        self.map_width = width
        self.map_height = height
        self._draw_grid()

    def _draw_grid(self, spacing=40):
        for x in range(0, self.map_width, spacing):
            self.create_line(x, 0, x, self.map_height, fill=GRID_COLOR)
        for y in range(0, self.map_height, spacing):
            self.create_line(0, y, self.map_width, y, fill=GRID_COLOR)

    def clamp(self, x, y, margin=14):
        x = max(margin, min(self.map_width - margin, x))
        y = max(margin, min(self.map_height - margin, y))
        return x, y

    def draw_trail(self, points, tag="trail"):
        self.delete(tag)
        if len(points) >= 2:
            flat = [c for p in points for c in p]
            self.create_line(*flat, fill=TRAIL_COLOR, width=2, dash=(4, 2), tag=tag)

    def draw_nodes(self, nodes, tag="nodes", numbered=True):
        self.delete(tag)
        for node in nodes:
            fill = "#3f6fb5" if node.kind == "junction" else "#6c9bd1"
            self.create_oval(node.x - NODE_RADIUS, node.y - NODE_RADIUS,
                              node.x + NODE_RADIUS, node.y + NODE_RADIUS,
                              fill=fill, outline="#1c3a5e", width=2, tag=tag)
            if numbered:
                self.create_text(node.x, node.y - NODE_RADIUS - 10, text=str(node.order),
                                  font=("Arial", 8, "bold"), fill="#1c3a5e", tag=tag)

    def draw_edges(self, edges, tag="edges"):
        self.delete(tag)
        for edge in edges:
            self.create_line(edge.node_a.x, edge.node_a.y, edge.node_b.x, edge.node_b.y,
                              fill=edge.color(), width=3, tag=tag)

    def draw_obstacles(self, obstacles, tag="obstacles"):
        self.delete(tag)
        r = 8
        for (x, y) in obstacles:
            self.create_line(x - r, y - r, x + r, y + r, fill="#8b0000", width=3, tag=tag)
            self.create_line(x - r, y + r, x + r, y - r, fill="#8b0000", width=3, tag=tag)

    def draw_robot(self, x, y, tag="robot", color="#222222"):
        self.delete(tag)
        self.create_oval(x - ROBOT_RADIUS, y - ROBOT_RADIUS, x + ROBOT_RADIUS, y + ROBOT_RADIUS,
                          fill=color, outline="white", width=2, tag=tag)
