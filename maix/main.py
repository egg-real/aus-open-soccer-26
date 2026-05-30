import cv2
from math import tan, pow, pi, cos, sin, atan2
import numpy as np
import time

from camera import Camera, UART

fieldW, fieldL = 1820, 2430
fieldRW, fieldRL = int(fieldW / 2), int(fieldL / 2) 
HFOV = 81
screenToWorldCartesian = None
screenToWorldPolar = None

#screenToWorldPolar[screenx, screeny] -> (distance, angle)
#screenToWorldCartesian[worldx, worldy] -> (x, y)

# with open("screen2world.txt",'r') as file:
#     s2wc, s2wp = file.read().strip().split(sep = "\ndelimiter\n")
#     screenToWorldCartesian = np.array(eval(s2wc))
#     screenToWorldPolar = np.array(eval(s2wp))
cameraZ = 158 
cameraAOD = pi/6
div = 374.67 
def getAngle(xPixel, yPixel):
    z = ((179.5 - yPixel) / div) * cos(cameraAOD) - sin(cameraAOD)
    y = cos(cameraAOD) + ((179.5 - yPixel) / div) * sin(cameraAOD)
    x = (xPixel-319.5) / div
    pos = np.array([x,y,z])
    pos /= np.linalg.norm(pos)
    return atan2(*(pos * cameraZ / -pos[2])[:2]) * (180/pi)

def getBoundingPolygon(mask):
    ct, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    if len(ct) == 0:
        return []
    return max(ct,key=cv2.contourArea)


paths = ["origin","-500 -300","800 -1000","100 900"]
colourSpecs = {
    # colour : ( HSV , HSV Weighting , lowerbound , upperBound)
    "org" : np.array(([8,200,200],[18,256,256])),
    "blu" : np.array(([100,255,255],[100,255,255])),
    "ylw" : np.array(([19,200,180],[25,255,255])),
    "grn" : np.array(([40,65,75],[70,155,180])),
    "blk" : np.array(((0,0,0),(180,20,20)))
}

cap = Camera(640, 360, debug=True, show=False)
uart = UART()

running = True

hsvFrame = None
k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))

prev_angle = 0

try:
    while running:
        ret, frame = cap.read(prev_angle)
        if not ret:
            continue
        hsvFrame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        ballMask = cv2.inRange(hsvFrame,*colourSpecs["org"])
        ct = getBoundingPolygon(ballMask)
        if len(ct) > 0:
            (x,y), r = cv2.minEnclosingCircle(ct[0])
            angle = getAngle(x, y)
            prev_angle = angle
            print("angle:", angle)
            uart.send_packet(1, angle)
        else:
            print("no balls")
except KeyboardInterrupt:
    running = False
finally:
    time.sleep(1)
    if uart.uart.is_open():
        uart.uart.close()
    time.sleep(2)