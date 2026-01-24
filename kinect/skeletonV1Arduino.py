import numpy as np
import time
import math
import serial
from collections import deque

from pykinect import nui
from pykinect.nui import JointId, TransformSmoothParameters

#14 = shoulder pitch
#13 = arm raise
#  

# ================= SERIAL =================
ser = serial.Serial("COM5", 115200, timeout=0.05)
time.sleep(2)
print("Serial connected")


# ================= CONSTANTS =================
SEND_INTERVAL = 0.1       # seconds (10 Hz)
DEADBAND = 3              # degrees


# ================= VECTOR HELPERS =================
def vec(a, b):
    return np.array([b.x - a.x, b.y - a.y, b.z - a.z])


def angle_between(v1, v2):
    dot = np.dot(v1, v2)
    mag = np.linalg.norm(v1) * np.linalg.norm(v2)
    if mag == 0:
        return 0
    return math.degrees(math.acos(np.clip(dot / mag, -1.0, 1.0)))


# ================= MAIN =================
def run():
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

    try:
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Shutting down...")
        kinect.close()
        ser.close()


# ================= TRACKER =================
class KinectTracking(object):

    def __init__(self):
        self.last_send_time = 0
        self.last_angles = {}

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
            return

        # ----------- JOINTS -----------

        #left arm
        shoulder = tracked.SkeletonPositions[JointId.ShoulderLeft]
        elbow    = tracked.SkeletonPositions[JointId.ElbowLeft]
        wrist    = tracked.SkeletonPositions[JointId.WristLeft]

        #right arm 



        # ----------- SHOULDER -----------
        upper = vec(shoulder, elbow)

        pitch = int(math.degrees(math.atan2(upper[1], upper[2])))
        roll  = int(math.degrees(math.atan2(upper[0], upper[2])))

        # ----------- ELBOW -----------
        fore = vec(elbow, wrist)

        elbow_angle = angle_between(upper, fore)  # 180 = straight
        elbow_angle = int((180 - elbow_angle) * (60.0 / 120.0))

        # Build single message with all servo commands
        angles = {
            13: pitch,
            14: -roll,
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
