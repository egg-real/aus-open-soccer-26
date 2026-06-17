from maix import camera, display, image, nn, app, comm, time
from math import pi, atan2, hypot, cos, sin
import numpy as np

model_path = "model.mud"
detector = nn.YOLOv5(model=model_path)

IMG_WIDTH = 640
IMG_HEIGHT = 360
HFOV = 81

cameraZ, cameraY = 158, 37.227 
cameraAOD = pi/6
div = 374.67 
def getPolarPosition(xPixel, yPixel):
    z = ((179.5 - yPixel) / div) * cos(cameraAOD) - sin(cameraAOD)
    y = cos(cameraAOD) + ((179.5 - yPixel) / div) * sin(cameraAOD)
    x = (xPixel-319.5) / div
    pos = np.array([x,y,z])
    pos /= np.linalg.norm(pos)
    worldPos = (pos * cameraZ / -pos[2])[:2]
    worldPos[1] += cameraY
    return atan2(*worldPos) * (180/pi), hypot(*worldPos)

DO_DISP = True

cam = camera.Camera(IMG_WIDTH, IMG_HEIGHT, detector.input_format())
dis = display.Display()

while not app.need_exit():
    # msg = p.get_msg()

    img = cam.read()
    objs = detector.detect(img, conf_th = 0.5, iou_th = 0.45)

    for obj in objs:
        angle, dist = getPolarPosition(obj.x + (obj.w/2), obj.y + (obj.h/2))
        if DO_DISP:
            img.draw_rect(obj.x, obj.y, obj.w, obj.h, color = image.COLOR_RED)
            msg = f'{detector.labels[obj.class_id]}: {obj.score:.2f}'
            img.draw_string(obj.x, obj.y, msg, color = image.COLOR_RED)
            print(angle, dist, time.fps())
            # print(time.fps())
    if DO_DISP:
        dis.show(img)

