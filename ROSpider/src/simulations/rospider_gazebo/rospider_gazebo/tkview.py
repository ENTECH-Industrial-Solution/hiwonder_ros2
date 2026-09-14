"""Small Tk pieces shared by the tuning windows.

Tk, like OpenCV's highgui, must own the main thread; the node's rclpy.spin
runs on a daemon thread and the window polls shared state with after(). The
helpers here are the three things every tuner needs: a camera frame as a
PhotoImage, a slider you can also type into, and a mainloop that ends when
ROS does.
"""

import tkinter as tk
from tkinter import ttk

import cv2
import rclpy


def photo_from_bgr(frame, max_width=640):
    """A tk.PhotoImage of a BGR frame, no PIL needed.

    Goes through PPM: Tk reads binary PPM natively, and cv2.imencode writes
    the channels in file order (RGB) from a BGR array, so no swap is needed.
    The caller must keep a reference to the returned image or Tk drops it.
    """
    height, width = frame.shape[:2]
    if width > max_width:
        frame = cv2.resize(frame, (max_width, int(height * max_width / width)))
    ok, ppm = cv2.imencode('.ppm', frame)
    if not ok:
        raise RuntimeError('cv2.imencode(.ppm) failed')
    return tk.PhotoImage(data=ppm.tobytes())


class LabeledScale(ttk.Frame):
    """A slider and an entry bound to one Tk variable.

    The slider is for exploring; the entry is for typing the exact value a
    tuning session ends on. `command` fires after either changes the value.
    """

    def __init__(self, parent, text, variable, from_, to, resolution,
                 command):
        super().__init__(parent)
        self.variable = variable
        self.command = command
        self.columnconfigure(0, weight=1)
        self.scale = tk.Scale(self, label=text, variable=variable,
                              from_=from_, to=to, resolution=resolution,
                              orient='horizontal', showvalue=False,
                              command=lambda _v: command())
        self.scale.grid(row=0, column=0, sticky='ew')
        self.entry = ttk.Entry(self, textvariable=variable, width=8)
        self.entry.grid(row=0, column=1, padx=(4, 0), sticky='s')
        self.entry.bind('<Return>', self._typed)
        self.entry.bind('<FocusOut>', self._typed)

    def _typed(self, _event=None):
        """Call command() when the entry is edited.

        Does not validate the entry itself; the reader of all widgets
        (_tuner_apply's call to read_widgets()) catches exceptions and
        reports them in one place.
        """
        self.command()


def mainloop_until_shutdown(root):
    """root.mainloop(), ended early when rclpy shuts down (Ctrl-C in the
    terminal, or ros2 launch tearing the process down)."""
    def poll():
        if not rclpy.ok():
            root.destroy()
            return
        root.after(200, poll)
    root.after(200, poll)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
