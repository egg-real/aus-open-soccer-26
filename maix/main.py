from maix import camera as maix_camera, display, image, nn, app, time
import numpy as np
from camera import UART, CMD_STOP, CMD_DETECT, CMD_DEBUG, CMD_TRAINING

model_path = "model.mud"
detector = nn.YOLO26(model=model_path)
screenToWorldPolar = np.load("screen-2-world-polar.npy")

IMG_WIDTH = 640
IMG_HEIGHT = 360
MM_PER_CM = 10

BALL_LABEL = "Ball"
YELLOW_GOAL_LABEL = "Yellow Goal"
BLUE_GOAL_LABEL = "Blue Goal"

def getPolarPosition(xPixel, yPixel):
    xPixel = max(0, min(IMG_WIDTH - 1, int(round(xPixel))))
    yPixel = max(0, min(IMG_HEIGHT - 1, int(round(yPixel))))
    angle, dist = screenToWorldPolar[xPixel][yPixel]
    if not np.isfinite(dist):
        return None
    return angle, dist

def to_cm(dist):
    if not np.isfinite(dist):
        return 0
    return int(round(dist / MM_PER_CM))

DO_DISP = not True
# Set True on-device to skip waiting for a pi detect command and log FPS.
BENCH_FPS = True

# Quality (0-100) for frames streamed in DEBUG mode. Lower keeps the JPEG
# small enough to push over the 115200 baud UART at a usable frame rate.
DEBUG_JPEG_QUALITY = 60
TRAINING_JPEG_QUALITY = 100

# Operating modes, switched by commands from the pi (see camera.UART).
MODE_STOPPED = 0   # idle, send nothing
MODE_DETECT = 1    # stream detection packets (existing behaviour)
MODE_DEBUG = 2     # broadcast compressed JPEG frames for web streaming/debugging
MODE_TRAINING = 3  # broadcast full-quality JPEG frames for training capture

# Map an incoming command byte to the mode it selects.
_COMMAND_MODES = {
    CMD_STOP: MODE_STOPPED,
    CMD_DETECT: MODE_DETECT,
    CMD_DEBUG: MODE_DEBUG,
    CMD_TRAINING: MODE_TRAINING,
}

cam = maix_camera.Camera(IMG_WIDTH, IMG_HEIGHT, detector.input_format())
uart = UART()
if DO_DISP:
    dis = display.Display()
else:
    dis = None

mode = MODE_DETECT if BENCH_FPS else MODE_STOPPED
debug_jpeg_quality = DEBUG_JPEG_QUALITY

while not app.need_exit():
    # Check whether the pi has asked us to start/stop or switch modes.
    command_frame = uart.read_command()
    if command_frame is not None:
        command, payload = command_frame
        if command in _COMMAND_MODES:
            mode = _COMMAND_MODES[command]
            if command == CMD_DEBUG and payload:
                debug_jpeg_quality = max(1, min(100, payload[0]))

    # Idle: don't touch the camera, just keep listening for commands.
    if mode == MODE_STOPPED:
        time.sleep_ms(10)
        continue

    try:
        img = cam.read()
    except RuntimeError:
        if mode == MODE_DETECT:
            uart.send_packet(cam_ok=False)
        continue

    # Debug/training modes: ship the same frame the model would see as JPEG and
    # skip detection entirely so the UART can send images as fast as possible.
    if mode == MODE_DEBUG or mode == MODE_TRAINING:
        quality = TRAINING_JPEG_QUALITY if mode == MODE_TRAINING else debug_jpeg_quality
        jpg = img.to_jpeg(quality)
        uart.send_image(jpg.to_bytes())
        del jpg
        if DO_DISP:
            print(time.fps())
            dis.show(img)
        continue

    objs = detector.detect(img, conf_th = 0.5, iou_th = 0.45)
    ball = None
    goal = None

    for obj in objs:
        polar = getPolarPosition(obj.x + (obj.w/2), obj.y + (obj.h/2))
        if polar is None:
            continue
        angle, dist = polar
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
        cam_ok=True,
    )
    if BENCH_FPS:
        fps = time.fps()
        print("fps: %.1f" % fps)
    if DO_DISP:
        dis.show(img)
