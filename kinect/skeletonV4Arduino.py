import numpy as np
import time
import math
import pygame
import serial
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


#================= SKELETON DRAWING =================
def draw_skeleton(window, skeleton, width, height):
    """Draw simple visualization of left arm"""
    if not skeleton:
        return
    
    # Get left shoulder and elbow positions
    shoulder = skeleton.SkeletonPositions[JointId.ShoulderLeft]
    elbow = skeleton.SkeletonPositions[JointId.ElbowLeft]
    wrist = skeleton.SkeletonPositions[JointId.WristLeft]
    
    # Convert to screen coordinates
    shoulder_x = int((shoulder.x + 1) * width / 2)
    shoulder_y = int((-shoulder.y + 1) * height / 2)
    
    elbow_x = int((elbow.x + 1) * width / 2)
    elbow_y = int((-elbow.y + 1) * height / 2)

    wrist_x = int((wrist.x + 1) * width / 2)
    wrist_y = int((-wrist.y + 1) * height / 2) 
    
    #grab distance info
    y_shoulder_to_elbow_px = abs(elbow_y - shoulder_y)
    x_shoulder_to_elbow_px = abs(elbow_x - shoulder_x)

    #====skeleton===
    # Draw line from shoulder to elbow
    pygame.draw.line(window, (0, 255, 0), (shoulder_x, shoulder_y), (elbow_x, elbow_y), 3)
    # Draw line from elbow to wrist
    pygame.draw.line(window, (0, 255, 0), (elbow_x, elbow_y), (wrist_x, wrist_y), 3)
    # Draw circle at shoulder
    pygame.draw.circle(window, (0, 255, 0), (shoulder_x, shoulder_y), 5)
    # Draw Circle at elbow
    pygame.draw.circle(window, (0, 255, 0), (elbow_x, elbow_y), 5)
    # Draw Circle at Wrist
    pygame.draw.circle(window, (0, 255, 0), (wrist_x, wrist_y), 5)

    #===x any y lines for shoulder to elbow===
    pygame.draw.line(window, (255, 255, 0), (shoulder_x, shoulder_y), (elbow_x, shoulder_y), 3) #capturing the x pixel distance from shoudler to elbow
    pygame.draw.line(window, (255, 255, 0), (elbow_x, elbow_y), (elbow_x, shoulder_y), 3) #capturing the y pixel distance from shoudler to elbow

    #===x and y lines for elbow to wrist===
    pygame.draw.line(window, (255, 255, 0), (elbow_x, elbow_y), (wrist_x, elbow_y), 3) #capturing the x pixel distance from shoudler to elbow
    pygame.draw.line(window, (255, 255, 0), (wrist_x, wrist_y), (wrist_x, elbow_y), 3) #capturing the y pixel distance from shoudler to elbow

    #display info
    font = pygame.font.Font(None, 36)
    text_y = font.render("Y Distance: %d px" % y_shoulder_to_elbow_px, True, (255,255,255))
    text_x = font.render("X Distance: %d px" % x_shoulder_to_elbow_px, True, (255,255,255))
    

    window.blit(text_y, (10,10))
    window.blit(text_x, (10,50))

    



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
            font = pygame.font.Font(None, 36)
            roll_text = font.render("roll: %d" % tracker.roll, True, (255, 255, 255))
            pitch_text =font.render("pitch: %d" % tracker.pitch, True, (255, 255, 255))
            pygame_window.blit(roll_text, (10, 70))
            pygame_window.blit(pitch_text, (10, 90))



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
        self.previous_roll_angle =0
        self.previous_elbow_angle = 0  # Track previous elbow angle
    
    

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

        #left arm
        shoulder = tracked.SkeletonPositions[JointId.ShoulderLeft]
        elbow    = tracked.SkeletonPositions[JointId.ElbowLeft]
        wrist    = tracked.SkeletonPositions[JointId.WristLeft]

        #right arm 
        #..... will finish after i figure out joint mapping

        #shoudler to elbow
        x_shoulder_to_elbow = elbow.x - shoulder.x
        y_shoulder_to_elbow = elbow.y - shoulder.y
        z_shoulder_to_elbow = elbow.z - shoulder.z

        #elbbow to wrist
        x_elbow_to_wrist = wrist.x - elbow.x
        y_elbow_to_wrist = wrist.y - elbow.y
        z_elbow_to_wrist = wrist.z - elbow.z
    




        # -----------Roll -----------
        #positive = front
        
        roll_raw = math.degrees(math.atan(z_shoulder_to_elbow/math.sqrt(x_shoulder_to_elbow**2 + y_shoulder_to_elbow**2)))
        if y_shoulder_to_elbow >0:
            roll = round((roll_raw + 170))
        elif y_shoulder_to_elbow <0:
            roll = round(-roll_raw) 
        elif y_shoulder_to_elbow: 
            
        self.roll = roll




        # ----------- SHOULDER -----------
        if z_shoulder_to_elbow < 10 and z_shoulder_to_elbow > -10:
            pitch_raw = math.degrees(math.atan(y_shoulder_to_elbow/x_shoulder_to_elbow))
            pitch = round(abs(pitch_raw - 90))
            self.pitch = pitch  # Store for display
        else: #this is new
            pitch_raw = math.degrees(math.atan(x_shoulder_to_elbow/z_shoulder_to_elbow))
            self.pitch = pitch  # Store for display



        # ----------- ELBOW -----------
        elbow_raw = math.degrees(math.atan(y_elbow_to_wrist/x_elbow_to_wrist))
        current_elbow_angle = round(elbow_raw-pitch_raw)
        self.elbow_angle = current_elbow_angle

        #here we handle the sudden cahnge from negative to positve
        if y_elbow_to_wrist < 0:
            if current_elbow_angle > 0:
                elbow_angle = current_elbow_angle
            else:
                elbow_angle = -current_elbow_angle
        elif y_elbow_to_wrist > 0:
            if current_elbow_angle < 0:
                elbow_angle = current_elbow_angle
            else:
                elbow_angle = -current_elbow_angle
        


        # Build single message with all servo commands
        angles = {
            13: pitch,
            14: roll,
            8: elbow_angle
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
