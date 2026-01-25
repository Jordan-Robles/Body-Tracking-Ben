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

class Ben():
    def __init__(self):
        self.torso, self.left_arm, self.right_arm = self.benURDF()
    def benURDF(self):
        torso = Chain(name="toseo", links=[
            OriginLink(),
            URDFLink(
                name="torso_base",
                translation_vector=[0,0,0],
                orientation=[0,0,0],
                rotation=[0,0,0],
                bounds=(-np.pi, np.pi)
            )
        ]
                    
                    
                    )

        left_arm = Chain(name="left_arm", links=[
            OriginLink(),
            # left_shoulder (fixed to torso)
            URDFLink(
                name="left_shoulder_pitch",
                translation_vector=[0, -0.1, 0],
                orientation=[0, 0, 0],
                rotation=[0,0,1],
                bounds=(-np.pi, np.pi)
            ),

            #left shoulder (yaw)
            URDFLink(
                name="left_shoulder_yaw",
                translation_vector=[0, 0.1, 0],
                orientation=[0, 0, 0],
                rotation=[0, 0, 1],   # Z axis
                bounds=(-np.pi, np.pi)
            ),

            #Left elbow pitch
            URDFLink(
                name="elbow",
                translation_vector=[0, 0.1, 0],
                orientation=[0, 0, 0],
                rotation=[0, 1, 0],   # Y axis
                bounds=(-np.pi/2, np.pi/2)
            ),


        ])

        right_arm = Chain(name="right_arm", links=[
            OriginLink(),
            # left_shoulder (fixed to torso)
            URDFLink(
                name="right_shoulder_pitch",
                translation_vector=[0, 0, 0],
                orientation=[0, 0, 0],
                rotation=[0,0,1],
                bounds=(-np.pi, np.pi)
            ),

            #left shoulder (yaw)
            URDFLink(
                name="right_shoulder_yaw",
                translation_vector=[0, 0, 0.1],
                orientation=[0, 0, 0],
                rotation=[0, 0, 1],   # Z axis
                bounds=(-np.pi, np.pi)
            ),

            #Right elbow pitch
            URDFLink(
                name="elbow",
                translation_vector=[0.15, 0, 0],
                orientation=[0, 0, 0],
                rotation=[0, 1, 0],   # Y axis
                bounds=(-np.pi/2, np.pi/2)
            ),


        ])
        return torso, left_arm, right_arm
    
    def ik_target(self, target_position, arm):
        target_frame = np.eye(4)
        target_frame[:3, 3] = target_position

        num_links = len(arm.links)
        inital_position = [0] * num_links

        ik_solution = arm.inverse_kinematics(
            target_frame,
            initial_position=inital_position
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

if __name__ == "__main__":
    ben = Ben()
    target_position = [0.2, 0.05, 0.15]
    ben.ik_target(target_position, ben.left_arm)