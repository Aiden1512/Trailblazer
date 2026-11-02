"""
Pop-out live camera window, opened on demand from the "View Live Camera"
button on either phase tab.

The camera subscriber only runs while this window is open: opening it starts a
fresh CameraFeed and closing it stops the feed again, so the robot's camera
stream isn't pulled into the GUI when the ranger is only watching the map.
"""
import tkinter as tk

from camera_feed import CameraFeed
from camera_panel import CameraPanel

VIEW_W, VIEW_H = 720, 480           # the Husky camera's native resolution


class CameraWindow(tk.Toplevel):
    def __init__(self, parent, topic, on_closed=None):
        super().__init__(parent)
        self.on_closed = on_closed
        self.title("TrailBlazer - Live Camera")
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.close)

        self.feed = CameraFeed(topic)
        self.feed.start()               # failures are shown by the panel's status line

        CameraPanel(self, self.feed, width=VIEW_W, height=VIEW_H,
                    title="Live Camera - " + topic).pack(padx=10, pady=10)
        tk.Button(self, text="Close", command=self.close, cursor="hand2",
                  font=("Arial", 10, "bold"), padx=10, pady=3, relief="raised",
                  bd=2).pack(anchor="e", padx=10, pady=(0, 10))

    def close(self):
        self.feed.stop()
        self.destroy()
        if self.on_closed is not None:
            self.on_closed()
