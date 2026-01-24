import pykinect
from pykinect import nui
from pykinect.nui import JointId
import numpy as np

#most of the code was found from some resoucres from 2010

# Define the callback function FIRST
def skeleton_frame_ready(skeleton_frame):
    skeletons = skeleton_frame.SkeletonData
    
    for index, data in enumerate(skeletons):
        # Check if skeleton is being tracked
        if data.eTrackingState == nui.SkeletonTrackingState.TRACKED:
            
            # Get head position
            head_position = data.SkeletonPositions[JointId.Head]
            print("Head Position: X={}, Y={}, Z={}".format(head_position.x, head_position.y, head_position.z))
            
            # Handle left arm joints
            LEFT_ARM = (
                JointId.ShoulderCenter, 
                JointId.ShoulderLeft, 
                JointId.ElbowLeft, 
                JointId.WristLeft, 
                JointId.HandLeft
            )
            
            left_arm_positions = [data.SkeletonPositions[joint] for joint in LEFT_ARM]
            
            # Print left arm positions
            for i, joint_id in enumerate(LEFT_ARM):
                pos = left_arm_positions[i]
                print("{}: X={}, Y={}, Z={}".format(joint_id, pos.x, pos.y, pos.z))

# NOW create the kinect instance
kinect = nui.Runtime()

# Enable the skeleton engine
kinect.skeleton_engine.enabled = True

# Skeleton frame ready event handling (function is now defined)
kinect.skeleton_frame_ready += skeleton_frame_ready

# Start the kinect
# Note: nui.Runtime() usually starts automatically, but you might need this
try:
    kinect.skeleton_engine.enabled = True
except:
    pass

# Keep the program running
print("Tracking started. Press Enter to stop...")
input()