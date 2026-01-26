import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import TextBox
import numpy as np
import benURDFV2


import numpy as np
import time
import math
import pygame
import serial
import benURDFV2
from collections import deque

from pykinect import nui
from pykinect.nui import JointId, TransformSmoothParameters

#=================VARIABLES =================

window_width = 800
window_height = 600

# ================= SERIAL =================
ser = serial.Serial("COM5", 115200, timeout=0.05)
time.sleep(2)
print("Serial connected")

# ================= CONSTANTS =================
SEND_INTERVAL = 0.1       # seconds (10 Hz)
DEADBAND = 3              # degrees

last_angles = {}
last_send_time = 0



def send_all_servos(angles):
    """Send all servo commands in a single serial message"""
    # Check which servos need updating
    to_send = {}
    for servo_id, angle in angles.items():
        last = last_angles.get(servo_id)
        
        if last is None or abs(angle - last) >= DEADBAND:
            to_send[servo_id] = angle
            last_angles[servo_id] = angle
    
    # If nothing changed, don't send
    if not to_send:
        return
    
    # Send all commands in ONE message: "13,pitch:14,roll:8,elbow\n"
    try:
        msg = ":".join(["%d,%d" % (sid, ang) for sid, ang in sorted(to_send.items())])
        msg += "\n"
        ser.write(msg.encode("utf-8"))
    except Exception as e:
        print("Serial write error:", e)


now = time.time()
ben = benURDFV2.Ben()

# ...existing code...
target_position = [0.1, -0.1, -0.1]
ik_sol = ben.ik_target(target_position,ben.left_arm)
angles_deg = np.degrees(ik_sol)

angles = {
    13: angles_deg[2],
    14: angles_deg[1],
    8: angles_deg[3]
}

send_all_servos(angles)
last_send_time = now

