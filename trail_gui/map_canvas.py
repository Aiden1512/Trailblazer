"""
Shared canvas drawing helpers for the trail map, used by both GUI phases.

The map represents a real 15 m x 15 m survey area with the ORIGIN (0, 0) AT
THE CENTRE. Internally every position (robot, nodes, obstacles) is stored in
real-world CENTIMETRES: x runs -750 .. +750 (left to right) and y runs
-750 .. +750 (bottom to top, like a ROS map frame), so distances, speeds and
clearances have a physical meaning and line up with the simulation, where the
Husky spawns at (0, 0). The canvas widget itself is kept at a smaller, screen
friendly pixel size, and a scale factor converts world cm -> screen px for
drawing (and back again for mouse clicks), so the map stays usable on any
monitor regardless of the 1500x1500 logical size.
"""
import os
import tkinter as tk

try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

MAP_SIZE_CM = 1500            # 15 m x 15 m survey area, in centimetres
CANVAS_PX = 760                # on-screen canvas size in pixels (square)

# Bundled floor texture, shown automatically behind the grid/trail/nodes on
# both Phase 1 and Phase 2. Kept next to this file so it travels with the
# package; swap the file to change the map art, no code changes needed.
BACKGROUND_IMAGE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Floor_Texture.png")

MAP_BG = "#e7f0e2"            # fallback colour if no background image is loaded
GRID_COLOR = "#ffffff"          # white reads clearly over both the grass and the lake
AXIS_COLOR = "#ffffff"          # X/Y axes through the origin (solid, thicker than the grid)
TRAIL_COLOR = "#fff176"         # bright yellow dashes stay visible over any terrain
NODE_RADIUS = 6                 # screen pixels - marker size, not world-scaled
ROBOT_RADIUS = 8


class MapCanvas(tk.Canvas):
    """Maps a world_size_cm x world_size_cm real-world area onto a
    display_px x display_px square of screen pixels."""

    def __init__(self, parent, world_size_cm=MAP_SIZE_CM, display_px=CANVAS_PX, **kwargs):
        super().__init__(parent, width=display_px, height=display_px, bg=MAP_BG,
                          highlightthickness=1, highlightbackground="#9fae95", **kwargs)
        self.world_size = world_size_cm
        self.display_px = display_px
        self.scale = display_px / world_size_cm
        self._bg_image = None     # keep a reference so Tk doesn't garbage-collect it
        self._draw_grid()
        self._load_default_background()

    def _load_default_background(self):
        """Auto-load the bundled floor texture. The GUI still runs without
        it (falling back to the plain MAP_BG colour), but we always print
        exactly why it's missing so this isn't a silent, confusing blank
        map."""
        if not PIL_AVAILABLE:
            print("[map_canvas] Background image disabled: Pillow isn't installed. "
                  "Run: pip install pillow --break-system-packages "
                  "(or: sudo apt install python3-pil python3-pil.imagetk)")
            return
        if not os.path.exists(BACKGROUND_IMAGE_PATH):
            print(f"[map_canvas] Background image disabled: Floor_Texture.png not found at "
                  f"{BACKGROUND_IMAGE_PATH} - make sure it's in the same folder as map_canvas.py.")
            return
        try:
            self.set_background_image(BACKGROUND_IMAGE_PATH)
        except Exception as exc:
            print(f"[map_canvas] Could not load background image: {exc}")

    # ---------- world (cm) <-> screen (px) ----------
    def world_to_screen(self, x, y):
        """World cm (origin at centre, +y up) -> canvas pixels (origin top-left, +y down)."""
        half = self.world_size / 2
        return (x + half) * self.scale, (half - y) * self.scale

    def screen_to_world(self, x, y):
        half = self.world_size / 2
        return x / self.scale - half, half - y / self.scale

    def clamp(self, x, y, margin_cm=14):
        """Clamp a world-space (cm) position to stay inside the map."""
        limit = self.world_size / 2 - margin_cm
        return max(-limit, min(limit, x)), max(-limit, min(limit, y))

    # ---------- background image ----------
    def set_background_image(self, path):
        """Load an image file and stretch it to cover the whole 15x15 m
        map area. Requires Pillow (pip install pillow). Call any time;
        the image is always kept beneath the grid and everything else."""
        if not PIL_AVAILABLE:
            raise RuntimeError("Pillow is required for background images. Install it with: pip install pillow")
        image = Image.open(path).convert("RGB")
        image = image.resize((self.display_px, self.display_px), Image.LANCZOS)
        self._bg_image = ImageTk.PhotoImage(image)
        self.delete("background")
        self.create_image(0, 0, anchor="nw", image=self._bg_image, tag="background")
        self.tag_lower("background")

    def clear_background_image(self):
        self._bg_image = None
        self.delete("background")

    # ---------- grid ----------
    def _draw_grid(self, spacing_cm=100):
        """Grid lines every spacing_cm (default 1 m) of world distance, lined up
        with the origin, plus highlighted X/Y axes crossing at (0, 0)."""
        self.delete("grid")
        half = self.world_size / 2
        value = -((half // spacing_cm) * spacing_cm)       # first multiple inside the map
        while value <= half:
            sx, _ = self.world_to_screen(value, 0)
            _, sy = self.world_to_screen(0, value)
            if value != 0:
                self.create_line(sx, 0, sx, self.display_px, fill=GRID_COLOR, stipple="gray50", tag="grid")
                self.create_line(0, sy, self.display_px, sy, fill=GRID_COLOR, stipple="gray50", tag="grid")
            value += spacing_cm

        ox, oy = self.world_to_screen(0, 0)
        self.create_line(ox, 0, ox, self.display_px, fill=AXIS_COLOR, width=2, tag="grid")
        self.create_line(0, oy, self.display_px, oy, fill=AXIS_COLOR, width=2, tag="grid")
        label_font = ("Arial", 9, "bold")
        self.create_text(ox + 6, oy + 6, text="(0, 0)", anchor="nw", fill=AXIS_COLOR,
                         font=label_font, tag="grid")
        self.create_text(self.display_px - 6, oy - 6, text="+X", anchor="se", fill=AXIS_COLOR,
                         font=label_font, tag="grid")
        self.create_text(ox + 6, 6, text="+Y", anchor="nw", fill=AXIS_COLOR,
                         font=label_font, tag="grid")
        self.tag_lower("grid")

    def draw_trail(self, points, tag="trail"):
        self.delete(tag)
        if len(points) >= 2:
            flat = []
            for (x, y) in points:
                sx, sy = self.world_to_screen(x, y)
                flat.extend([sx, sy])
            self.create_line(*flat, fill=TRAIL_COLOR, width=2, dash=(4, 2), tag=tag)

    def draw_nodes(self, nodes, tag="nodes", numbered=True):
        self.delete(tag)
        for node in nodes:
            sx, sy = self.world_to_screen(node.x, node.y)
            fill = "#3f6fb5" if node.kind == "junction" else "#6c9bd1"
            self.create_oval(sx - NODE_RADIUS, sy - NODE_RADIUS,
                              sx + NODE_RADIUS, sy + NODE_RADIUS,
                              fill=fill, outline="#1c3a5e", width=2, tag=tag)
            if numbered:
                self.create_text(sx, sy - NODE_RADIUS - 10, text=str(node.order),
                                  font=("Arial", 8, "bold"), fill="#1c3a5e", tag=tag)

    def draw_edges(self, edges, tag="edges"):
        self.delete(tag)
        for edge in edges:
            ax, ay = self.world_to_screen(edge.node_a.x, edge.node_a.y)
            bx, by = self.world_to_screen(edge.node_b.x, edge.node_b.y)
            self.create_line(ax, ay, bx, by, fill=edge.color(), width=3, tag=tag)

    def draw_obstacles(self, obstacles, tag="obstacles"):
        self.delete(tag)
        r = 8
        for (x, y) in obstacles:
            sx, sy = self.world_to_screen(x, y)
            self.create_line(sx - r, sy - r, sx + r, sy + r, fill="#8b0000", width=3, tag=tag)
            self.create_line(sx - r, sy + r, sx + r, sy - r, fill="#8b0000", width=3, tag=tag)

    def draw_robot(self, x, y, tag="robot", color="#222222"):
        self.delete(tag)
        sx, sy = self.world_to_screen(x, y)
        self.create_oval(sx - ROBOT_RADIUS, sy - ROBOT_RADIUS, sx + ROBOT_RADIUS, sy + ROBOT_RADIUS,
                          fill=color, outline="white", width=2, tag=tag)