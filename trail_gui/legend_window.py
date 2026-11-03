"""
Pop-out map legend for the Phase 2 tab, opened from its "Map Legend" button.

Each row draws a small sample of the mark exactly as map_canvas.py draws it on
the map (same colours and shapes), so the key matches what the ranger sees.
"""
import tkinter as tk

SWATCH_W, SWATCH_H = 44, 22


def _line(c, color, dash=None, halo=False):
    y = SWATCH_H / 2
    if halo:
        c.create_line(6, y, SWATCH_W - 6, y, fill="white", width=8, capstyle="round")
    c.create_line(6, y, SWATCH_W - 6, y, fill=color, width=6 if dash else 4, capstyle="round", dash=dash)


def _junction(c):
    x, y, r = SWATCH_W / 2, SWATCH_H / 2, 6
    c.create_oval(x - r, y - r, x + r, y + r, fill="#3f6fb5", outline="#1c3a5e", width=2)


def _dead_end(c):
    x, y, r = SWATCH_W / 2, SWATCH_H / 2, 4
    c.create_oval(x - r, y - r, x + r, y + r, fill="#9e9e9e", outline="#424242")


def _red_dot(c, ring=False):
    x, y, r = SWATCH_W / 2, SWATCH_H / 2, 6
    if ring:
        c.create_oval(x - r - 3, y - r - 3, x + r + 3, y + r + 3, outline="#7b1fa2", width=3)
    c.create_oval(x - r, y - r, x + r, y + r, fill="#e53935", outline="#7f0000", width=2)


def _placed(c):
    x, y, r = SWATCH_W / 2, SWATCH_H / 2, 7
    c.create_line(x - r, y - r, x + r, y + r, fill="#8b0000", width=3)
    c.create_line(x - r, y + r, x + r, y - r, fill="#8b0000", width=3)


def _robot(c):
    x, y, r = SWATCH_W / 2, SWATCH_H / 2, 8
    c.create_oval(x - r, y - r, x + r, y + r, fill="#222222", outline="white", width=2)


SECTIONS = [
    ("Trails", [
        (lambda c: _line(c, "#111111"), "Not inspected yet"),
        (lambda c: _line(c, "#2e8b32"), "Inspected"),
        (lambda c: _line(c, "#e08e00"), "Blocked - other side still to check"),
        (lambda c: _line(c, "#c0392b"), "Blocked"),
        (lambda c: _line(c, "#7b1fa2", dash=(6, 4), halo=True), "Blocked at both ends - can't be checked"),
    ]),
    ("Network", [
        (_junction, "Junction Jn (Tn = trail, as in the trail file)"),
        (_dead_end, "Trail dead end"),
    ]),
    ("Obstacles & robot", [
        (_red_dot, "Obstacle found on a trail (blocked, or gone around)"),
        (lambda c: _red_dot(c, ring=True), "One of the two obstacles cutting off a trail"),
        (_placed, "Obstacle you placed (also in the sim)"),
        (_robot, "Robot (live from simulation)"),
    ]),
]


class LegendWindow(tk.Toplevel):
    def __init__(self, parent, on_closed=None):
        super().__init__(parent)
        self.on_closed = on_closed
        self.title("TrailBlazer - Map Legend")
        self.resizable(False, False)
        self.transient(parent.winfo_toplevel())    # stays above the main window
        self.protocol("WM_DELETE_WINDOW", self.close)

        body = tk.Frame(self, padx=12, pady=10)
        body.pack(fill="both", expand=True)
        for title, rows in SECTIONS:
            tk.Label(body, text=title, font=("Arial", 10, "bold")).pack(anchor="w", pady=(6, 2))
            for draw, text in rows:
                row = tk.Frame(body)
                row.pack(anchor="w", fill="x")
                # light grey behind the swatch so the white robot outline / halo are visible
                swatch = tk.Canvas(row, width=SWATCH_W, height=SWATCH_H, bg="#d9e3d3", highlightthickness=0)
                swatch.pack(side="left", pady=1)
                draw(swatch)
                tk.Label(row, text=text, font=("Arial", 10)).pack(side="left", padx=(8, 0))

        tk.Button(self, text="Close", command=self.close, cursor="hand2",
                  font=("Arial", 10, "bold"), padx=10, pady=3, relief="raised",
                  bd=2).pack(anchor="e", padx=10, pady=(0, 10))

    def close(self):
        self.destroy()
        if self.on_closed is not None:
            self.on_closed()
