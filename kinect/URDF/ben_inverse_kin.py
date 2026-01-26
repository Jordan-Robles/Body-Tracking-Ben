import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import TextBox
import numpy as np
import benURDFV2

ben = benURDFV2.Ben()
target = np.array([0.1, 0.05, 0.15])

target_frame = np.eye(4)
target_frame[:3, 3] = target
angles = ben.left_arm.inverse_kinematics(target_frame, initial_position=[0]*len(ben.left_arm.links))

print(np.degrees(angles))