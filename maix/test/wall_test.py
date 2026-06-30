from maix import app, camera, display, image
import math
import numpy as np
import time

IMG_WIDTH = 640
IMG_HEIGHT = 360

_NPY_PATH = "screen-2-world-polar.npy"
screenToWorldPolar = np.load(_NPY_PATH)

LINE_COUNT = 5
SAMPLE_COUNT = 16
SAMPLE_MARGIN = 20
BLACK_THRESHOLD = 400
GREEN_RATIO_THRESHOLD = 35


cam = camera.Camera(IMG_WIDTH, IMG_HEIGHT, image.Format.FMT_RGB888)
disp = display.Display()


def get_polar(x_pixel, y_pixel):
    xi = max(0, min(IMG_WIDTH - 1, int(round(x_pixel))))
    yi = max(0, min(IMG_HEIGHT - 1, int(round(y_pixel))))
    angle, distance = screenToWorldPolar[xi][yi]
    return float(angle), float(distance)


def get_ground_xy(x_pixel, y_pixel):
    angle, distance = get_polar(x_pixel, y_pixel)
    angle_rad = math.radians(angle)
    return distance * math.sin(angle_rad), distance * math.cos(angle_rad)


def is_black(pixel):
    r, g, b = pixel
    total = r + g + b
    if total > BLACK_THRESHOLD:
        return False
    if total == 0:
        return True
    return (100 * g) / total <= GREEN_RATIO_THRESHOLD


def scan_points(img):
    black_samples = []
    column_points = []
    for x in sample_xs:
        lowest = None
        for y in sample_ys:
            if is_black(img.get_pixel(x, y, rgbtuple=True)):
                black_samples.append((x, y))
                lowest = (x, y)
        if lowest is not None:
            column_points.append(lowest)

    closest = None
    closest_xy = None
    for x, y in black_samples:
        angle, distance = get_polar(x, y)
        if closest is None or distance < closest[1]:
            closest = (angle, distance)
            closest_xy = (x, y)

    return column_points, closest, closest_xy, set(black_samples)


def wall_angle_deg(column_points):
    ground_points = [get_ground_xy(x, y) for x, y in column_points]
    if len(ground_points) < 2:
        return None

    pts = np.array(ground_points, dtype=float)
    pts -= pts.mean(axis=0)
    if np.allclose(pts, 0):
        return None

    _, _, vh = np.linalg.svd(pts)
    dx, dy = vh[0]
    return math.degrees(math.atan2(dx, dy))


def sample_positions(length, count):
    if count <= 1:
        return [length // 2]

    span = length - (SAMPLE_MARGIN * 2)
    step = span / (count - 1)
    return [int(SAMPLE_MARGIN + (step * i)) for i in range(count)]


sample_xs = sample_positions(IMG_WIDTH, LINE_COUNT)
sample_ys = sample_positions(IMG_HEIGHT, SAMPLE_COUNT)

last_time = time.time()
frame_count = 0
fps = 0

while not app.need_exit():
    img = cam.read()

    column_points, closest, closest_xy, black_samples = scan_points(img)
    wall_angle = wall_angle_deg(column_points)

    for x in sample_xs:
        img.draw_line(x, 0, x, IMG_HEIGHT - 1, image.COLOR_BLUE, 1)

    for x in sample_xs:
        for y in sample_ys:
            colour = image.COLOR_WHITE if (x, y) in black_samples else image.COLOR_BLACK
            img.draw_circle(x, y, 3, colour, 1)

    for x, y in column_points:
        img.draw_circle(x, y, 5, image.COLOR_GREEN, 2)

    if closest_xy is not None:
        img.draw_circle(closest_xy[0], closest_xy[1], 9, image.COLOR_YELLOW, 3)

    if wall_angle is not None:
        angle_text = "wall: %.1f deg (%d cols)" % (wall_angle, len(column_points))
    else:
        angle_text = "wall: unknown (%d cols)" % len(column_points)

    if closest is not None:
        closest_text = "closest: %.1f deg, %.0f mm" % closest
    else:
        closest_text = "closest: none"

    img.draw_string(0, 0, angle_text, image.COLOR_BLUE)
    img.draw_string(0, 16, closest_text, image.COLOR_BLUE)
    img.draw_string(0, 32, "FPS: %.1f" % fps, image.COLOR_YELLOW)

    frame_count += 1
    now = time.time()
    elapsed = now - last_time
    if elapsed >= 1.0:
        fps = frame_count / elapsed
        last_time = now
        frame_count = 0

    if wall_angle is not None:
        print("wall angle: %.1f deg (%d columns)" % (wall_angle, len(column_points)))
    else:
        print("wall angle: unknown (%d columns)" % len(column_points))

    if closest is not None:
        print("closest: angle=%.1f deg, distance=%.0f mm" % closest)
    else:
        print("closest: none")

    disp.show(img)
