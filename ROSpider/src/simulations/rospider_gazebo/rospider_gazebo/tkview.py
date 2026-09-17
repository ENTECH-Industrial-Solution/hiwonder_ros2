"""Small Tk pieces shared by the tuning windows.

Tk, like OpenCV's highgui, must own the main thread; the node's rclpy.spin
runs on a daemon thread and the window polls shared state with after(). The
helpers here are what the tuners need: a camera frame as a PhotoImage, a
slider you can also type into, a slider drawn the way Hiwonder's LAB_Tool
draws them, and a mainloop that ends when ROS does.
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


#: LAB_Tool's palette: orange controls on the default grey window, thin
#: grey boxes round each group of controls, a red frame round each area.
ORANGE = '#f0a030'
TRACK = '#606060'
KNOB = '#d0d0d0'
GREY_BUTTON = '#a8a8a8'
BOX = '#b8b8b8'
RED_FRAME = '#ff0000'


def flat_button(parent, text, width, height, bg=ORANGE, command=None,
                **kwargs):
    """A borderless coloured button with white text, `width` x `height`
    in pixels.

    tk.Button sizes itself in characters unless it shows an image, so a
    1x1 transparent image is attached (compound='center') to make the size
    a pixel one, as LAB_Tool's square -/+ and its row of equal-width
    buttons need. The image is kept alive on the button.
    """
    pixel = tk.PhotoImage(width=1, height=1)
    button = tk.Button(parent, text=text, image=pixel, compound='center',
                       width=width, height=height, bg=bg, fg='white',
                       activebackground=bg, activeforeground='white',
                       disabledforeground='#e8e8e8', relief='flat', bd=0,
                       highlightthickness=0, padx=0, pady=0,
                       command=command, **kwargs)
    button.pixel = pixel
    return button


def box(parent, **kwargs):
    """A tk.Frame with LAB_Tool's thin grey outline."""
    return tk.Frame(parent, highlightbackground=BOX, highlightthickness=1,
                    **kwargs)


class HiwonderScale(tk.Frame):
    """A slider drawn like the ones in Hiwonder's LAB_Tool.

    [-] a dark track with an orange fill up to a grey knob [+] and the
    value as a plain number on the right. The -/+ buttons step by one;
    clicking or dragging on the track sets the value directly. `command`
    fires after every change.
    """

    #: Knob radius; the track is inset by it so the knob never leaves the
    #: canvas.
    RADIUS = 8

    def __init__(self, parent, variable, from_=0, to=255, command=None,
                 length=120):
        super().__init__(parent)
        self.variable = variable
        self.from_, self.to = from_, to
        self.command = command
        self.length = length
        self.minus = flat_button(self, '-', 17, 17, font=('TkDefaultFont', 11, 'bold'),
                                 command=lambda: self._step(-1))
        self.minus.pack(side='left')
        self.canvas = tk.Canvas(self, width=length, height=2 * self.RADIUS + 2,
                                highlightthickness=0, bg=self.cget('bg'))
        self.canvas.pack(side='left', padx=4)
        self.plus = flat_button(self, '+', 17, 17, font=('TkDefaultFont', 11, 'bold'),
                                command=lambda: self._step(1))
        self.plus.pack(side='left')
        tk.Label(self, textvariable=variable, width=3, anchor='w', pady=0,
                 font=('TkDefaultFont', 11)).pack(side='left', padx=(8, 0))
        self.canvas.bind('<Button-1>', self._pointed)
        self.canvas.bind('<B1-Motion>', self._pointed)
        variable.trace_add('write', lambda *_a: self._draw())
        self._draw()

    def _set(self, value):
        value = max(self.from_, min(self.to, int(round(value))))
        if value != self.variable.get():
            self.variable.set(value)
            if self.command:
                self.command()

    def _step(self, delta):
        self._set(self.variable.get() + delta)

    def _pointed(self, event):
        r = self.RADIUS
        span = self.length - 2 * r
        self._set(self.from_ + (event.x - r) / span * (self.to - self.from_))

    def _draw(self):
        canvas = self.canvas
        canvas.delete('all')
        r = self.RADIUS
        y = r + 1
        span = self.length - 2 * r
        try:
            value = self.variable.get()
        except tk.TclError:
            value = self.from_
        x = r + span * (value - self.from_) / (self.to - self.from_)
        canvas.create_line(r, y, self.length - r, y, fill=TRACK, width=7)
        if x > r:
            canvas.create_line(r, y, x, y, fill=ORANGE, width=7)
        canvas.create_oval(x - r, y - r, x + r, y + r, fill=KNOB,
                           outline='#f0f0f0', width=2)


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
