import numpy as np
import time
import math
import pygame
import serial
import benURDFV2
from collections import deque

from pykinect import nui
from pykinect.nui import JointId, TransformSmoothParameters

#14 = shoulder pitch
#13 = arm raise
#  

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

ben = benURDFV2.Ben()

#================= SKELETON DRAWING =================
def draw_skeleton(window, skeleton, width, height):
    """Draw simple visualization of left arm"""
    if not skeleton:
        return
    
    # Get left shoulder and elbow positions
    shoulder_left = skeleton.SkeletonPositions[JointId.ShoulderLeft]
    elbow_left = skeleton.SkeletonPositions[JointId.ElbowLeft]
    wrist_left = skeleton.SkeletonPositions[JointId.WristLeft]

    shoulder_right = skeleton.SkeletonPositions[JointId.ShoulderRight]
    elbow_right  = skeleton.SkeletonPositions[JointId.ElbowRight]
    wrist_right  = skeleton.SkeletonPositions[JointId.WristRight]
    
    shoulder_left_x = int((shoulder_left.x + 1) * width / 2)
    shoulder_left_y = int((-shoulder_left.y + 1) * height / 2)
    
    elbow_left_x = int((elbow_left.x + 1) * width / 2)
    elbow_left_y = int((-elbow_left.y + 1) * height / 2)

    wrist_left_x = int((wrist_left.x + 1) * width / 2)
    wrist_left_y = int((-wrist_left.y + 1) * height / 2) 


    shoulder_right_x = int((shoulder_right.x + 1) * width / 2)
    shoulder_right_y = int((-shoulder_right.y + 1) * height / 2)
    
    elbow_right_x = int((elbow_right.x + 1) * width / 2)
    elbow_right_y = int((-elbow_right.y + 1) * height / 2)

    wrist_right_x = int((wrist_right.x + 1) * width / 2)
    wrist_right_y = int((-wrist_right.y + 1) * height / 2)
    

    #left side
    pygame.draw.line(window, (0, 255, 0), (shoulder_left_x, shoulder_left_y), (elbow_left_x, elbow_left_y), 3) # Draw line from shoulder to elbow
    pygame.draw.line(window, (0, 255, 0), (elbow_left_x, elbow_left_y), (wrist_left_x, wrist_left_y), 3) # Draw line from elbow to wrist
    pygame.draw.circle(window, (0, 255, 0), (shoulder_left_x, shoulder_left_y), 5)# Draw circle at shoulder
    pygame.draw.circle(window, (0, 255, 0), (elbow_left_x, elbow_left_y), 5)# Draw Circle at elbow
    pygame.draw.circle(window, (255, 0, 0), (wrist_left_x, wrist_left_y), 5)# Draw Circle at Wrist

    pygame.draw.line(window, (0, 255, 0), (shoulder_right_x, shoulder_right_y), (elbow_right_x, elbow_right_y), 3) # Draw line from shoulder to elbow
    pygame.draw.line(window, (0, 255, 0), (elbow_right_x, elbow_right_y), (wrist_right_x, wrist_right_y), 3) # Draw line from elbow to wrist
    pygame.draw.circle(window, (0, 255, 0), (shoulder_right_x, shoulder_right_y), 5)# Draw circle at shoulder
    pygame.draw.circle(window, (0, 255, 0), (elbow_right_x, elbow_right_y), 5)# Draw Circle at elbow
    pygame.draw.circle(window, (255, 0, 0), (wrist_right_x, wrist_right_y), 5)# Draw Circle at Wrist

    

# ================= MAIN =================
def run():
    #pygame
    pygame.init()
    pygame_window = pygame.display.set_mode((window_height, window_width))
    pygame.display.set_caption("Kinect Visual")
    clock = pygame.time.Clock()

    #kinect 
    kinect = nui.Runtime()
    kinect.skeleton_engine.enabled = True


    # Enable smoothing with default parameters
    try:
        kinect.skeleton_engine.smooth_parameters = nui.SkeletonEngine.TransformSmoothParameters
    except:
        pass  # Smoothing not available or already enabled
    

    tracker = KinectTracking()
    kinect.skeleton_frame_ready += tracker.skeleton_frame_ready


    print("Kinect started")
    
    pygame_running = True
    try:
        while pygame_running:
            for event in pygame.event.get():
                if event.type==pygame.KEYDOWN:
                    if event.key==pygame.K_ESCAPE:
                        pygame_running = False

            pygame_window.fill((0,0,0)) #fill the background BLACK
            
            # Display pitch value
            right_target_string = np.array(tracker.right_target)
 

            font = pygame.font.Font(None, 36)
            #end_eff_right_x = font.render("right_end: %d" % right_target_string, True, (255, 255, 255))
            #pygame_window.blit(end_eff_right_x, (10, 70))




            if tracker.current_skeleton:
                draw_skeleton(pygame_window, tracker.current_skeleton, window_width, window_height)

            pygame.display.flip()#update display

            clock.tick(30) #fps = 30

    except KeyboardInterrupt:
        print("Shutting down...")
        kinect.close()
        ser.close()
        pygame.quit()

# ================= TRACKER =================
class KinectTracking(object):

    def __init__(self):
        self.last_send_time = 0
        self.last_angles = {}
        self.current_skeleton = None #store for visualization
        self.pitch = 0  # Store pitch for display
        self.elbow_angle = 0  # Store elbow angle for display
        self.roll = 0
        self.right_target = 0
        self.inital_left_ik = [0, 0, 0, 0, 0]
        self.inital_right_ik = [0, 0, 0, 0, 0]
    
    

    def skeleton_frame_ready(self, skeleton_frame):
        now = time.time()
        if now - self.last_send_time < SEND_INTERVAL:
            return

        skeletons = skeleton_frame.SkeletonData
        tracked = None

        for s in skeletons:
            if s and s.eTrackingState == nui.SkeletonTrackingState.TRACKED:
                tracked = s
                break

        if not tracked:
            self.current_skeleton =None
            return
        
        self.current_skeleton = tracked

        # ----------- JOINTS -----------
        left_end_effector = tracked.SkeletonPositions[JointId.WristLeft]
        left_origin = tracked.SkeletonPositions[JointId.ShoulderLeft]

        right_end_effector = tracked.SkeletonPositions[JointId.WristRight]
        right_origin = tracked.SkeletonPositions[JointId.ShoulderRight]

        mapped_left_end_eff = [
            (left_end_effector.x - left_origin.x),
            (left_end_effector.y - left_origin.y),
            (left_end_effector.z - left_origin.z)
        ]

        mapped_right_end_eff = [
            (right_end_effector.x - right_origin.x),
            (right_end_effector.y - right_origin.y),
            (right_end_effector.z - right_origin.z)
        ]

        self.left_target = np.array([-mapped_left_end_eff[2], mapped_left_end_eff[0], mapped_left_end_eff[1]])
        
        target_frame = np.eye(4)
        target_frame[:3, 3] = self.left_target
        left_ik_sol = ben.left_arm.inverse_kinematics(target_frame, initial_position=self.inital_left_ik)
        self.inital_left_ik = left_ik_sol
        left_angles_deg = np.degrees(left_ik_sol)



        self.right_target = np.array([mapped_right_end_eff[2], mapped_right_end_eff[0], mapped_right_end_eff[1]])
        target_frame_right = np.eye(4)
        target_frame_right[:3, 3] = self.right_target
        right_ik_sol = ben.right_arm.inverse_kinematics(target_frame_right, initial_position=self.inital_right_ik)
        self.inital_right_ik = right_ik_sol
        right_angles_deg = np.degrees(right_ik_sol)

        # Build single message with all servo commands
        angles = {
            #left
            13: left_angles_deg[2],
            14: left_angles_deg[1],
            8: left_angles_deg[3],

            #right
            16: right_angles_deg[1],
            17: right_angles_deg[2],
            18: right_angles_deg[3]
        }

        self.send_all_servos(angles)
        self.last_send_time = now

    def send_all_servos(self, angles):
        """Send all servo commands in a single serial message"""
        # Check which servos need updating
        to_send = {}
        for servo_id, angle in angles.items():
            last = self.last_angles.get(servo_id)
            
            if last is None or abs(angle - last) >= DEADBAND:
                to_send[servo_id] = angle
                self.last_angles[servo_id] = angle
        
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


# ================= RUN =================
if __name__ == "__main__":
    run()
