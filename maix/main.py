from maix import camera as maix_camera, display, image, nn, app, time
from math import pi, atan2, hypot, cos, sin, radians, degrees
import numpy as np
from camera import UART, CMD_STOP, CMD_DETECT, CMD_DEBUG, CMD_TRAINING

model_path = "model.mud"
detector = nn.YOLOv5(model=model_path)
screenToWorldPolar = np.load("screen-2-world-polar.npy")

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
    xPixel = max(0, min(IMG_WIDTH - 1, int(round(xPixel))))
    yPixel = max(0, min(IMG_HEIGHT - 1, int(round(yPixel))))
    return screenToWorldPolar[xPixel][yPixel][0], screenToWorldPolar[xPixel][yPixel][1]

def to_cm(dist):
    return int(round(dist / MM_PER_CM))

# ----- Line detection (white field lines) ----- #
# Thresholds in LAB (L range 0-100 for RGB888).
WHITE = [[80, 100, -10, 10, -10, 10]]
GREEN = [[0, 80, -120, -10, 0, 30]]
MASK_WHITE = [[50, 100, -128, 127, -128, 127]]  # white pixels of a binary image

GREEN_DILATE = 2        # px to dilate green so it bridges over thin lines
LINE_MIN_AREA = 60      # drop specks
# A line is thin in one dimension and long in the other. Reject blobs that are
# chunky in both dimensions, they're some other white object.
LINE_MIN_ASPECT = 3.0   # major-axis length must be >= LINE_MIN_ASPECT * minor-axis
LINE_MAX_THICKNESS = 40 # px; reject blobs whose short side is fatter than this
LINE_MIN_LENGTH = 40    # px; reject stubs that are too short to trust as lines


def get_ground_xy(x_pixel, y_pixel):
    """Convert a screen pixel to ground coords (mm) in the bot frame.

    Returns (world_x, world_y) where world_x is lateral (right +) and world_y
    is forward, or None when the pixel is outside the lookup table.
    """
    xi = int(round(x_pixel))
    yi = int(round(y_pixel))
    if xi < 0 or xi >= IMG_WIDTH or yi < 0 or yi >= IMG_HEIGHT:
        return None

    angle, distance = screenToWorldPolar[xi][yi]
    angle_rad = radians(float(angle))
    distance = float(distance)
    world_x = distance * sin(angle_rad)
    world_y = distance * cos(angle_rad)
    return world_x, world_y


def _axis_length(axis_line):
    x1, y1, x2, y2 = axis_line
    return hypot(x2 - x1, y2 - y1)


def is_line_shaped(blob):
    """True when the blob is thin in one dimension and long in the other."""
    if blob.area() < LINE_MIN_AREA:
        return False

    major = _axis_length(blob.major_axis_line())
    minor = _axis_length(blob.minor_axis_line())

    if major < LINE_MIN_LENGTH:
        return False
    if minor > LINE_MAX_THICKNESS:
        return False
    if minor <= 0 or major / minor < LINE_MIN_ASPECT:
        return False
    return True


def closest_point_on_blob(blob):
    """Closest point of the line to the camera, as (angle_deg, distance_mm).

    The camera sits at the ground-frame origin, so the closest point on the
    line is the foot of the perpendicular dropped from the origin onto the
    line. We unproject the major-axis endpoints to the ground, then project the
    origin onto that segment (clamped, so when the foot falls beyond an end of
    the visible line the nearer endpoint is used instead).
    Returns None if an endpoint can't be unprojected.
    """
    x1, y1, x2, y2 = blob.major_axis_line()
    p1 = get_ground_xy(x1, y1)
    p2 = get_ground_xy(x2, y2)
    if p1 is None or p2 is None:
        return None

    ax, ay = p1
    bx, by = p2
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-6:
        fx, fy = ax, ay
    else:
        t = -(ax * dx + ay * dy) / seg_len_sq
        t = max(0.0, min(1.0, t))
        fx, fy = ax + dx * t, ay + dy * t

    distance = hypot(fx, fy)
    angle = degrees(atan2(fx, fy))
    return angle, distance


def find_closest_line(img):
    """Scan the frame for white-on-green lines, return the closest as
    (angle_deg, distance_mm), or None when no line is found."""
    # Isolate white-on-green pixels so off-field white can't trigger a line.
    green = img.binary(GREEN, copy=True)
    green.dilate(GREEN_DILATE)
    line_px = img.binary(WHITE, copy=True)
    line_px.b_and(green)

    blobs = line_px.find_blobs(MASK_WHITE, pixels_threshold=LINE_MIN_AREA,
                               area_threshold=LINE_MIN_AREA, merge=True)

    closest = None  # (angle_deg, distance_mm)
    for blob in blobs:
        if not is_line_shaped(blob):
            continue
        result = closest_point_on_blob(blob)
        if result is None:
            continue
        if closest is None or result[1] < closest[1]:
            closest = result

        if DO_DISP:
            x1, y1, x2, y2 = blob.major_axis_line()
            img.draw_line(x1, y1, x2, y2, image.COLOR_RED, 2)

    return closest

DO_DISP = False

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

mode = MODE_STOPPED
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

    # line = find_closest_line(img)  # (angle_deg, distance_mm) or None
    line = None

    uart.send_packet(
        see_ball=ball is not None,
        ball_dir=ball[0] if ball else 0,
        ball_dist=to_cm(ball[1]) if ball else 0,
        see_goal=goal is not None,
        yellow_goal=goal[2] if goal else False,
        goal_dir=goal[0] if goal else 0,
        goal_dist=to_cm(goal[1]) if goal else 0,
        see_line=line is not None,
        line_dir=line[0] if line else 0,
        line_dist=to_cm(line[1]) if line else 0,
        cam_ok=True,
    )
    if DO_DISP:
        print(time.fps())
        dis.show(img)
