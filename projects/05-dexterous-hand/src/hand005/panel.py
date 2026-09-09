"""Main-thread Tk controls: absolute targets, no held-key/repeat dependence."""
import tkinter as tk
from tkinter import ttk
import numpy as np
from .core import OPEN, GRASP, PINCH


class Panel:
    def __init__(self, ranges):
        self.root = tk.Tk()
        self.root.title('005 Hand Controls - Allegro | SIMULATION FIXTURE')
        self.root.geometry('650x760')
        self.closed = self.paused = False
        self.request = None
        self.buttons = {}
        self.scales = []
        self.target = np.r_[OPEN, 0., 0.]
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        ttk.Label(self.root, text='005 | FOUR-FINGER HAND LAB', font=('', 16, 'bold')).pack(pady=6)
        ttk.Label(self.root, text='16 finger joints + 2-axis wrist fixture. NOT a whole humanoid.').pack()
        ttk.Label(self.root, text='Sliders set persistent joint targets. No need to hold keys.').pack()
        row = ttk.Frame(self.root)
        row.pack(fill='x', padx=8, pady=8)
        for text, command in [('1 Open', lambda: self.preset(OPEN)), ('2 Power pose', lambda: self.preset(GRASP)),
                              ('3 Pinch pose*', lambda: self.preset(PINCH)),
                              ('Run grasp demo', lambda: self.action('demo'))]:
            button = ttk.Button(row, text=text, command=command)
            button.pack(side='left', expand=True, fill='x')
            self.buttons[text] = button
        for finger, title in enumerate(['Index', 'Middle', 'Ring', 'Thumb']):
            frame = ttk.LabelFrame(self.root, text=title)
            frame.pack(fill='x', padx=10, pady=2)
            for joint in range(4):
                i = finger * 4 + joint
                ttk.Label(frame, text=f'J{joint} (rad)', width=10).grid(row=joint, column=0)
                var = tk.DoubleVar(value=float(self.target[i]))
                scale = ttk.Scale(frame, from_=ranges[i, 0], to=ranges[i, 1], variable=var,
                                  command=lambda value, index=i: self.slider(index, value))
                scale.grid(row=joint, column=1, sticky='ew')
                frame.columnconfigure(1, weight=1)
                self.scales.append(var)
        wrist = ttk.LabelFrame(self.root, text='Actuated fixture — meters, not a robot arm')
        wrist.pack(fill='x', padx=10, pady=4)
        for i, name, end in [(16, 'Y transfer', .20), (17, 'Z lift', .12)]:
            ttk.Label(wrist, text=name, width=12).pack(side='left')
            var = tk.DoubleVar(value=0.)
            ttk.Scale(wrist, from_=0., to=end, variable=var,
                      command=lambda value, index=i: self.slider(index, value)).pack(side='left', expand=True, fill='x')
            self.scales.append(var)
        row = ttk.Frame(self.root)
        row.pack(fill='x', padx=10, pady=5)
        for title, command in [('Pause / resume', self.pause), ('Cancel demo / hold', lambda: self.action('hold')),
                               ('Quit', self.close)]:
            ttk.Button(row, text=title, command=command).pack(side='left', expand=True, fill='x')
        self.status = tk.StringVar(value='Ready — open hand. Run demo starts a measured trial.')
        ttk.Label(self.root, textvariable=self.status, justify='left').pack(padx=10, pady=5, anchor='w')
        ttk.Label(self.root, text='* Pinch is a motion preset, not a validated grasp.\nManual targets persist after focus loss. This is simulation-only.').pack()
        for key, pose in [('1', OPEN), ('2', GRASP), ('3', PINCH)]:
            self.root.bind(key, lambda event, p=pose: self.preset(p))
        self.root.bind('<space>', lambda event: self.pause())
        self.root.bind('c', lambda event: self.action('hold'))
        self.root.update()

    def slider(self, index, value):
        self.target[index] = float(value)
        self.request = 'manual'

    def preset(self, pose):
        self.target[:16] = pose
        for var, value in zip(self.scales[:16], pose):
            var.set(float(value))
        self.request = 'manual'

    def action(self, name):
        self.request = name

    def pause(self):
        self.paused = not self.paused

    def close(self):
        self.closed = True

    def pump(self):
        self.root.update()

    def display(self, now, phase, obs):
        f = obs['finger_forces']
        p = obs['object']
        self.status.set(f'{"PAUSED | " if self.paused else ""}{phase} | sim {now:.1f} s\n'
                        f'Object: x={p[0]:.3f}, y={p[1]:.3f}, z={p[2]:.3f} m\n'
                        f'Contact normal force: I={f["ff"]:.2f} M={f["mf"]:.2f} '
                        f'R={f["rf"]:.2f} T={f["th"]:.2f} N\n'
                        'Demo requires source position. Restart the program for a fresh trial.')

    def destroy(self):
        self.root.destroy()
