#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped, WrenchStamped
# from franka_msgs import FrankaState
# from rv_msgs.msg import ManipulatorState
from std_msgs.msg import Float64, String, Bool
from numpy import sign, array, zeros, sqrt, sum
import tkinter as tk


class TestInterface:

    def __init__(self):
        self.window = tk.Tk()
        self.window.mainloop()

    def create_window(self):
        self.window = 1


if __name__ == '__main__':

    TestInterface()

    """window = tk.Tk()

    frame_a = tk.Frame()
    frame_b = tk.Frame()

    label_a = tk.Label(master=frame_a, text="I'm in Frame A")
    label_a.pack()

    label_b = tk.Label(master=frame_b, text="I'm in Frame B")
    label_b.pack()

    frame_a.pack()
    frame_b.pack()

    window.mainloop()"""


