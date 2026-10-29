"""
Start screen - the first page shown when the GUI opens.

  Continue where you left off  -> loads the saved trail map (nodes and
                                  junctions) and goes straight to Phase 2.
  Start a new simulation       -> Phase 1 (map the trail by driving the robot
                                  in the simulation), then Phase 2.
"""
import tkinter as tk
from tkinter import ttk

import state as state_module
from robot_pose import STATE_COLORS

POLL_MS = 500


class StartScreen(ttk.Frame):
    def __init__(self, parent, on_new, on_continue, pose_feed=None):
        super().__init__(parent)
        self.pose_feed = pose_feed

        centre = ttk.Frame(self)
        centre.place(relx=0.5, rely=0.42, anchor="center")

        ttk.Label(centre, text="TrailBlazer", font=("Arial", 30, "bold")).pack()
        ttk.Label(centre, text="Autonomous trail inspection - operator console",
                  font=("Arial", 12)).pack(pady=(2, 34))

        button_opts = dict(font=("Arial", 16, "bold"), width=28, padx=20, pady=16,
                           relief="raised", bd=3, cursor="hand2")

        tk.Button(centre, text="Continue where you left off", command=on_continue,
                  **button_opts).pack()
        self.saved_var = tk.StringVar()
        ttk.Label(centre, textvariable=self.saved_var, foreground="#555555").pack(pady=(6, 26))

        tk.Button(centre, text="Start a new simulation", command=on_new,
                  **button_opts).pack()
        ttk.Label(centre, text="Map the trail first (Phase 1), then run the inspection (Phase 2)",
                  foreground="#555555").pack(pady=(6, 0))

        self.sim_label = ttk.Label(self, text="", wraplength=700, justify="center")
        self.sim_label.place(relx=0.5, rely=0.94, anchor="center")

        self.refresh()
        self._poll()

    def refresh(self):
        """Update the saved-map line (called each time this screen is shown)."""
        count = state_module.saved_map_node_count()
        if count is None:
            self.saved_var.set("No saved map yet - finish a mapping run first")
        else:
            self.saved_var.set(f"Saved map found: {count} nodes - opens Phase 2 (Autonomous Inspection)")
        self._update_sim_status()

    def _update_sim_status(self):
        if self.pose_feed is not None:
            feed_state, text = self.pose_feed.status()
            self.sim_label.configure(text="Simulation: " + text, foreground=STATE_COLORS.get(feed_state, "black"))

    def _poll(self):
        if self.winfo_ismapped():
            self._update_sim_status()
        try:
            self.after(POLL_MS, self._poll)
        except tk.TclError:
            pass