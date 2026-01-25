import os
os.environ['TCL_LIBRARY'] = r'C:\Python27\tcl\tcl8.5'
os.environ['TK_LIBRARY'] = r'C:\Python27\tcl\tk8.5'

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

# =============================
# Create the kinematic chain
# =============================

arm = Chain(name="3dof_arm", links=[
    OriginLink(),
    # Base (fixed)
    URDFLink(
        name="base",
        translation_vector=[0, 0, 0],
        orientation=[0, 0, 0],
        rotation=[0,0,1],
        bounds=(-np.pi, np.pi)
    ),

    # Joint 1: Base rotation (yaw)
    URDFLink(
        name="joint_1",
        translation_vector=[0, 0, 0.1],
        orientation=[0, 0, 0],
        rotation=[0, 0, 1],   # Z axis
        bounds=(-np.pi, np.pi)
    ),

    # Joint 2: Shoulder pitch
    URDFLink(
        name="joint_2",
        translation_vector=[0.15, 0, 0],
        orientation=[0, 0, 0],
        rotation=[0, 1, 0],   # Y axis
        bounds=(-np.pi/2, np.pi/2)
    ),

    # Joint 3: Elbow pitch
    URDFLink(
        name="joint_3",
        translation_vector=[0.1, 0, 0],
        orientation=[0, 0, 0],
        rotation=[0, 1, 0],   # Y axis
        bounds=(-np.pi/2, np.pi/2)
    )
])

# =============================
# Inverse Kinematics target
# =============================

target_position = [0.2, 0.05, 0.15]
target_frame = np.eye(4)
target_frame[:3, 3] = target_position

ik_solution = arm.inverse_kinematics(
    target_frame,
    initial_position=[0, 0, 0, 0, 0]  # base + 3 joints
)

print("Joint angles (rad):")
print(ik_solution)

print("Joint angles (deg):")
print(np.degrees(ik_solution))

# =============================
# Plot the robot
# =============================
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


# Draw the robot
arm.plot(ik_solution, ax, target=target_position)


# Set equal aspect ratio
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(-0.3,0.3)
ax.set_ylim(-0.3,0.3)
ax.set_zlim(0,0.4)
ax.set_title("3-DOF Robot Arm Visualization")
plt.show()