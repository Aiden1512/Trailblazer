"""
Camera panel widget: shows the newest frame from a CameraFeed, letterboxed
into a fixed-size box, with a status line underneath (LIVE / waiting / no
signal). Used on both the Phase 1 and Phase 2 tabs; they share one feed.
"""
import tkinter as tk
from tkinter import ttk

from PIL import Image, ImageDraw, ImageFont, ImageOps, ImageTk

UPDATE_MS = 100                     # GUI poll rate; the sim camera is only ~3 Hz
BOX_COLOR = (17, 17, 17)
STATUS_COLORS = {
    "live": "#2e7d32",
    "waiting": "#b26a00",
    "stale": "#b26a00",
    "error": "#c0392b",
    "unavailable": "#c0392b",
}


def _load_font(size=22):
    """Best available font. Older Pillow versions (e.g. Ubuntu 22.04's
    python3-pil) can't size the built-in font, so try a real TrueType font first."""
    for name in ("DejaVuSans-Bold.ttf", "DejaVuSans.ttf", "Arial.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    try:
        return ImageFont.load_default(size=size)
    except TypeError:               # old Pillow: no size argument
        return ImageFont.load_default()


def _make_placeholder(width, height, text="NO SIGNAL"):
    """Dark box with 'NO SIGNAL'. Purely cosmetic, so it must never be able to
    crash the GUI: any font problem just leaves a plain dark box."""
    image = Image.new("RGB", (width, height), BOX_COLOR)
    try:
        font = _load_font()
        # getmask().size works for bitmap and TrueType fonts on every Pillow
        # version (textbbox/textsize each fail on some versions).
        text_w, text_h = font.getmask(text).size
        ImageDraw.Draw(image).text(((width - text_w) // 2, (height - text_h) // 2),
                                   text, fill=(150, 150, 150), font=font)
    except Exception:
        pass
    return image


class CameraPanel(ttk.Frame):
    def __init__(self, parent, feed, width=300, height=200, title="Live Camera",
                 placeholder_text="NO SIGNAL"):
        super().__init__(parent)
        self.feed = feed
        self.box_w = width
        self.box_h = height
        self._shown_id = None
        self._after_id = None

        ttk.Label(self, text=title, font=("Arial", 11, "bold")).pack(anchor="w")

        self._placeholder = ImageTk.PhotoImage(_make_placeholder(width, height, placeholder_text))
        self._photo = self._placeholder
        self.image_label = tk.Label(self, image=self._placeholder, bg="#111111",
                                    bd=1, relief="sunken")
        self.image_label.pack(anchor="w", pady=(2, 2))

        self.status_label = ttk.Label(self, text="", wraplength=width, justify="left")
        self.status_label.pack(anchor="w")

        self._refresh()

    def _fit_to_box(self, frame):
        fitted = ImageOps.contain(frame, (self.box_w, self.box_h))
        box = Image.new("RGB", (self.box_w, self.box_h), BOX_COLOR)
        box.paste(fitted, ((self.box_w - fitted.width) // 2, (self.box_h - fitted.height) // 2))
        return box

    def _refresh(self):
        # Skip the work while this tab isn't visible; the frame id check below
        # makes sure we catch up the moment it is shown again.
        if self.winfo_ismapped():
            state, text = self.feed.status()
            self.status_label.configure(text=("\u25cf " if state == "live" else "\u25cb ") + text,
                                        foreground=STATUS_COLORS.get(state, "black"))
            frame_id, frame = self.feed.get_frame()
            if frame is not None and frame_id != self._shown_id:
                self._photo = ImageTk.PhotoImage(self._fit_to_box(frame))
                self.image_label.configure(image=self._photo)
                self._shown_id = frame_id
        self._after_id = self.after(UPDATE_MS, self._refresh)

    def destroy(self):
        if self._after_id is not None:
            self.after_cancel(self._after_id)
            self._after_id = None
        super().destroy()