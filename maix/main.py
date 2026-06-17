from maix import camera as maix_camera, display, image, nn, app, time
from math import pi, atan2, hypot, cos, sin
import numpy as np
from camera import UART

model_path = "model.mud"
detector = nn.YOLOv5(model=model_path)

IMG_WIDTH = 640
IMG_HEIGHT = 360
HFOV = 81
MM_PER_CM = 10

BALL_LABEL = "Ball"
YELLOW_GOAL_LABEL = "Yellow Goal"
BLUE_GOAL_LABEL = "Blue Goal"

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

def to_cm(dist):
    return int(round(dist / MM_PER_CM))

DO_DISP = False

cam = maix_camera.Camera(IMG_WIDTH, IMG_HEIGHT, detector.input_format())
uart = UART()
if DO_DISP:
    dis = display.Display()
else:
    dis = None

while not app.need_exit():
    # msg = p.get_msg()

    try:
        img = cam.read()
    except RuntimeError:
        uart.send_packet(cam_ok=False)
        continue

    objs = detector.detect(img, conf_th = 0.5, iou_th = 0.45)
    ball = None
    goal = None

    for obj in objs:
        angle, dist = getPolarPosition(obj.x + (obj.w/2), obj.y + (obj.h/2))
        label = detector.labels[obj.class_id]

        if label == BALL_LABEL:
            if ball is None or dist < ball[1]:
                ball = (angle, dist)
        elif label == YELLOW_GOAL_LABEL or label == BLUE_GOAL_LABEL:
            if goal is None or dist < goal[1]:
                goal = (angle, dist, label == YELLOW_GOAL_LABEL)

        if DO_DISP:
            img.draw_rect(obj.x, obj.y, obj.w, obj.h, color = image.COLOR_RED)
            msg = f'{label}: {obj.score:.2f}'
            img.draw_string(obj.x, obj.y, msg, color = image.COLOR_RED)

    uart.send_packet(
        see_ball=ball is not None,
        ball_dir=ball[0] if ball else 0,
        ball_dist=to_cm(ball[1]) if ball else 0,
        see_goal=goal is not None,
        yellow_goal=goal[2] if goal else False,
        goal_dir=goal[0] if goal else 0,
        goal_dist=to_cm(goal[1]) if goal else 0,
        wall_dir=0,
        wall_dist=0,
        cam_ok=True,
    )

    if DO_DISP:
        print(time.fps())
        dis.show(img)

