#!/usr/bin/env python3
"""
TrailBlazer - Operator GUI (MVP)
=================================

Opens on a start screen with two options:

  Start a new simulation
      Phase 1 - Teleop Mapping: the robot is driven in the simulation; the
      map follows its ACTUAL position (from /husky1/odometry) and records
      waypoint and junction nodes along the trail. Finishing the mapping
      saves the trail and moves on to Phase 2.

  Continue where you left off
      Loads the saved trail map (saved_map.json) and goes straight to Phase 2.

  Phase 2 - Autonomous Inspection (both options)
      The recorded nodes are joined, in the order they were created, by black
      route lines. Obstacles placed on the map are also created in the
      simulation. The inspection turns each route green once cleared, or red
      if an obstacle blocks it.

Needs ROS 2 sourced in the terminal (to read the robot's position) and the
simulation running (ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py).
Without them the GUI still opens and says what's missing.

Phase 2 also shows the real robot's status: obstacle warn/stop from the
obstacle detector, the mission manager's progress, and the obstacle snapshot
from /husky1/obstacle_image.

The live camera panel is on hold: set TRAILBLAZER_CAMERA=1 to switch it back on.

Run with:
    python3 main.py
"""
import os
import tkinter as tk
from tkinter import ttk, messagebox

import state as state_module
from camera_feed import CameraFeed, SNAPSHOT_TOPIC
from mission_status import MissionStatusFeed
from phase1_teleop import Phase1Frame
from phase2_inspection import Phase2Frame
from robot_pose import RobotPoseFeed, DEFAULT_TOPIC as DEFAULT_ODOM_TOPIC
from sim_obstacles import SimObstacleSpawner
from start_screen import StartScreen
from state import TrailState

SHOW_CAMERA = os.environ.get("TRAILBLAZER_CAMERA", "0") == "1"      # camera feed is on hold

PHASE1_TITLE = "Phase 1 - Teleop Mapping"
PHASE2_TITLE = "Phase 2 - Autonomous Inspection"


class TrailBlazerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("TrailBlazer - Operator GUI")
        # The map canvas is a 760x760 square (the 15 m x 15 m trail area), so the
        # window needs real height. Resizable with a safe minimum.
        self.root.geometry("1140x1020")
        self.root.minsize(1100, 860)
        self.root.resizable(True, True)

        # 'clam' renders consistently across Linux desktop themes.
        ttk.Style(root).theme_use("clam")

        self.state = TrailState()

        # ROS-facing parts. Each one copes with ROS being absent.
        self.pose_feed = RobotPoseFeed(os.environ.get("TRAILBLAZER_ODOM_TOPIC", DEFAULT_ODOM_TOPIC))
        self.pose_feed.start()
        self.camera_feed = None
        if SHOW_CAMERA:
            from camera_feed import DEFAULT_TOPIC as DEFAULT_CAMERA_TOPIC
            self.camera_feed = CameraFeed(os.environ.get("TRAILBLAZER_CAMERA_TOPIC", DEFAULT_CAMERA_TOPIC))
            self.camera_feed.start()
        self.status_feed = MissionStatusFeed()
        self.status_feed.start()
        self.snapshot_feed = CameraFeed(os.environ.get("TRAILBLAZER_SNAPSHOT_TOPIC", SNAPSHOT_TOPIC),
                                        snapshot=True)
        self.snapshot_feed.start()
        self.obstacle_spawner = SimObstacleSpawner()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        try:
            self.container = ttk.Frame(root)
            self.container.pack(fill="both", expand=True)

            self.start_screen = StartScreen(self.container, on_new=self.start_new_simulation,
                                            on_continue=self.continue_saved_map, pose_feed=self.pose_feed)
            self.notebook = ttk.Notebook(self.container)

            self.phase2 = Phase2Frame(self.notebook, self.state, on_back=self.show_start,
                                      pose_feed=self.pose_feed, spawner=self.obstacle_spawner,
                                      camera_feed=self.camera_feed, status_feed=self.status_feed,
                                      snapshot_feed=self.snapshot_feed)
            self.phase1 = Phase1Frame(self.notebook, self.state, on_finish_mapping=self._go_to_phase2,
                                      on_back=self.show_start, pose_feed=self.pose_feed,
                                      on_reset=self.phase2.clear_obstacles, camera_feed=self.camera_feed)
        except Exception:
            self._stop_feeds()       # don't leave ROS threads running if startup fails
            raise

        self.notebook.bind("<<NotebookTabChanged>>", self._on_tab_changed)
        self.show_start()

    # ---------------- screens ----------------
    def show_start(self):
        self.phase1.halt()
        self.phase2.abort()
        self.notebook.pack_forget()
        self.start_screen.refresh()
        self.start_screen.pack(fill="both", expand=True)

    def _show_tabs(self, *tabs):
        """Show the notebook containing just these (frame, title) tabs."""
        for tab in self.notebook.tabs():
            self.notebook.forget(tab)
        for frame, title in tabs:
            self.notebook.add(frame, text=title)
        self.start_screen.pack_forget()
        self.notebook.pack(fill="both", expand=True)

    def start_new_simulation(self):
        """Phase 1 (map the trail) then Phase 2."""
        self._show_tabs((self.phase1, PHASE1_TITLE), (self.phase2, PHASE2_TITLE))
        self.phase1.begin()                      # starts a fresh run (and clears old obstacles)
        self.phase2.refresh_from_state()         # empty map until mapping is finished
        self.notebook.select(self.phase1)
        self.phase1.focus_canvas()

    def continue_saved_map(self):
        """Load the saved trail map and go straight to Phase 2."""
        try:
            state_module.load_map(self.state)
        except FileNotFoundError:
            messagebox.showinfo("No saved map yet",
                                "There's no saved trail map to continue from yet.\n\n"
                                "Start a new simulation and press 'Finish Mapping' to save one.")
            return
        except (ValueError, OSError) as exc:
            messagebox.showerror("Couldn't load the saved map", str(exc))
            return
        self.phase1.halt()
        self.phase2.clear_obstacles()            # take any leftovers out of the simulation
        self._show_tabs((self.phase2, PHASE2_TITLE))
        self.phase2.refresh_from_state()

    def _go_to_phase2(self):
        self.phase2.refresh_from_state()
        self.notebook.select(self.phase2)

    def _on_tab_changed(self, _event):
        if self.notebook.tabs() and self.notebook.select() == str(self.phase1):
            self.phase1.focus_canvas()

    # ---------------- shutdown ----------------
    def _stop_feeds(self):
        self.pose_feed.stop()
        self.status_feed.stop()
        self.snapshot_feed.stop()
        if self.camera_feed is not None:
            self.camera_feed.stop()

    def on_close(self):
        self._stop_feeds()               # shuts the ROS subscribers down cleanly
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = TrailBlazerApp(root)
    root.mainloop()