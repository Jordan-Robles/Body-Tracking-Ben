# -*- coding: utf-8 -*-
import numpy as np
import time
import math
import serial
import pygame
import sys
import threading

from pykinect import nui
from pykinect.nui import JointId


# ================= SERIAL =================
ser = serial.Serial("COM5", 115200, timeout=0)
time.sleep(2)
print("Serial connected")


# ================= CONSTANTS =================
SEND_INTERVAL = 0.1
DEADBAND = 3

WINDOW_W, WINDOW_H = 900, 600
SCALE = 300
CENTER = (WINDOW_W // 2, WINDOW_H // 2 + 100)


# ================= VECTOR =================
def vec(a, b):
    return np.array([b.x - a.x, b.y - a.y, b.z - a.z])


def angle_between(v1, v2):
    dot = np.dot(v1, v2)
    mag = np.linalg.norm(v1) * np.linalg.norm(v2)
    if mag == 0:
        return 0
    return math.degrees(math.acos(np.clip(dot / mag, -1.0, 1.0)))


def project(joint):
    x = int(CENTER[0] + joint.x * SCALE)
    y = int(CENTER[1] - joint.y * SCALE)
    return x, y


# ================= THREAD-SAFE TRACKER =================
class KinectTracking(object):

    def __init__(self):
        self.lock = threading.Lock()

        self.raw_joints = None
        self.raw_coords = {}

        self.last_send_time = 0
        self.last_angles = {}

        self.joints_2d = {}
        self.angles = {}

    # CALLBACK: COPY DATA ONLY
    def skeleton_frame_ready(self, skeleton_frame):
        for s in skeleton_frame.SkeletonData:
            if s and s.eTrackingState == nui.SkeletonTrackingState.TRACKED:
                with self.lock:
                    self.raw_joints = {
                        "Shoulder": s.SkeletonPositions[JointId.ShoulderLeft],
                        "Elbow":    s.SkeletonPositions[JointId.ElbowLeft],
                        "Wrist":    s.SkeletonPositions[JointId.WristLeft]
                    }

                    # Store raw XYZ coordinates
                    self.raw_coords = {}
                    for name, j in self.raw_joints.iteritems():
                        self.raw_coords[name] = (j.x, j.y, j.z)
                break


# ================= MAIN =================
def run():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Kinect Raw Joint Coordinates")
    font = pygame.font.SysFont("consolas", 18)
    clock = pygame.time.Clock()

    kinect = nui.Runtime()
    kinect.skeleton_engine.enabled = True

    tracker = KinectTracking()
    kinect.skeleton_frame_ready += tracker.skeleton_frame_ready

    print("Kinect started")

    running = True
    while running:
        clock.tick(60)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((20, 20, 20))

        # ---- COPY DATA SAFELY ----
        joints = None
        coords = None
        with tracker.lock:
            if tracker.raw_joints:
                joints = tracker.raw_joints.copy()
                coords = tracker.raw_coords.copy()

        if joints:
            shoulder = joints["Shoulder"]
            elbow = joints["Elbow"]
            wrist = joints["Wrist"]

            # Projection
            tracker.joints_2d = {
                "Shoulder": project(shoulder),
                "Elbow": project(elbow),
                "Wrist": project(wrist)
            }

            # Angles
            upper = vec(shoulder, elbow)
            fore = vec(elbow, wrist)

            pitch = int(math.degrees(math.atan2(upper[1], upper[2])))
            roll  = int(math.degrees(math.atan2(upper[0], upper[2])))

            elbow_angle = angle_between(upper, fore)
            elbow_angle = int((180 - elbow_angle) * (60.0 / 120.0))

            tracker.angles = {
                "Shoulder Pitch": pitch,
                "Shoulder Roll": -roll,
                "Elbow": elbow_angle
            }

            # ---- SERIAL SEND ----
            now = time.time()
            if now - tracker.last_send_time > SEND_INTERVAL:
                angles = {13: pitch, 14: -roll, 8: elbow_angle}

                to_send = {}
                for sid, ang in angles.iteritems():
                    ang = int(ang)
                    last = tracker.last_angles.get(sid)
                    if last is None or abs(ang - last) >= DEADBAND:
                        to_send[sid] = ang
                        tracker.last_angles[sid] = ang

                if to_send:
                    msg = ":".join("%d,%d" % (sid, ang)
                                   for sid, ang in sorted(to_send.iteritems())) + "\n"
                    try:
                        ser.write(msg.encode("utf-8"))
                    except:
                        pass

                tracker.last_send_time = now

        # ---- DRAW SKELETON ----
        j = tracker.joints_2d
        if len(j) == 3:
            pygame.draw.line(screen, (0, 255, 0), j["Shoulder"], j["Elbow"], 4)
            pygame.draw.line(screen, (0, 255, 0), j["Elbow"], j["Wrist"], 4)

            for p in j.values():
                pygame.draw.circle(screen, (255, 0, 0), p, 6)

        # ---- DRAW RAW COORDINATES ----
        y = 20
        title = font.render("Raw Joint Coordinates (meters)", True, (0, 200, 255))
        screen.blit(title, (20, y))
        y += 30

        if coords:
            for name, (x, yv, z) in coords.iteritems():
                txt = "%s  X: %.3f  Y: %.3f  Z: %.3f" % (name, x, yv, z)
                surf = font.render(txt, True, (255, 255, 255))
                screen.blit(surf, (20, y))
                y += 25

        # ---- DRAW ANGLES ----
        y += 10
        for name, val in tracker.angles.iteritems():
            txt = font.render("%s: %d deg" % (name, val), True, (255, 200, 200))
            screen.blit(txt, (20, y))
            y += 25

        pygame.display.flip()

    # ---- CLEANUP ----
    kinect.close()
    ser.close()
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    run()
