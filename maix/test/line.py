from maix import camera, display, image
from math import pi, atan2, hypot, cos, sin, radians

# ----- Camera geometry (matches maix/main.py, calibrated for 640x360) ----- #
IMG_WIDTH = 640
IMG_HEIGHT = 360
cameraZ, cameraY = 158, 37.227
cameraAOD = pi / 6
div = 374.67
CX = (IMG_WIDTH - 1) / 2
CY = (IMG_HEIGHT - 1) / 2


def get_polar_position(x_pixel, y_pixel):
    """Unproject a floor pixel to (angle_deg, distance) in the bot frame."""
    z = ((CY - y_pixel) / div) * cos(cameraAOD) - sin(cameraAOD)
    y = cos(cameraAOD) + ((CY - y_pixel) / div) * sin(cameraAOD)
    x = (x_pixel - CX) / div
    norm = hypot(x, hypot(y, z))
    x, y, z = x / norm, y / norm, z / norm
    if z >= 0:
        # Ray points at/above horizon, never meets the ground.
        return None
    world_x = x * cameraZ / -z
    world_y = y * cameraZ / -z + cameraY
    return atan2(world_x, world_y) * (180 / pi), hypot(world_x, world_y)


# ----- Thresholds (LAB, L range 0-100 for RGB888) ----- #
WHITE = [[80, 100, -10, 10, -10, 10]]
GREEN = [[0, 80, -120, -10, 0, 30]]
MASK_WHITE = [[50, 100, -128, 127, -128, 127]]  # white pixels of a binary image

# ----- Tuning knobs ----- #
GREEN_DILATE = 2        # px to dilate green over thin lines (object rejection #1)
MIN_AREA = 60           # drop specks
MAX_AREA = 6000         # drop big filled blobs
MIN_ELONGATION = 0.6    # keep elongated stripes (object rejection #2)
MAX_DENSITY = 0.5       # keep thin (low fill) blobs
# Robot-body guard: ignore blobs whose centroid is in this bottom-centre box.
BODY_X = (IMG_WIDTH // 2 - 90, IMG_WIDTH // 2 + 90)
BODY_Y = IMG_HEIGHT - 70

cam = camera.Camera(IMG_WIDTH, IMG_HEIGHT)
disp = display.Display()


def is_line_blob(blob):
    if not (MIN_AREA <= blob.area() <= MAX_AREA):
        return False
    if blob.elongation() < MIN_ELONGATION:
        return False
    if blob.density() > MAX_DENSITY:
        return False
    cx, cy = blob.cx(), blob.cy()
    if cy > BODY_Y and BODY_X[0] < cx < BODY_X[1]:
        return False
    return True


def line_equation(x1, y1, x2, y2):
    """Two ground points -> (m, c) for y=mx+c, with m=254 vertical sentinel."""
    if abs(x2 - x1) < 1e-3:
        return 254, x1  # vertical: c is the x-intercept
    m = (y2 - y1) / (x2 - x1)
    c = y1 - m * x1
    return m, c


while True:
    img = cam.read()

    # Stage A: isolate white-on-green pixels (reject off-field white).
    green = img.binary(GREEN, copy=True)
    green.dilate(GREEN_DILATE)
    line_px = img.binary(WHITE, copy=True)
    line_px.b_and(green)

    # Stage B: keep only thin elongated stripes (reject filled white).
    blobs = line_px.find_blobs(MASK_WHITE, pixels_threshold=MIN_AREA,
                               area_threshold=MIN_AREA, merge=True)

    kept = 0
    for blob in blobs:
        if not is_line_blob(blob):
            continue
        kept += 1

        x1, y1, x2, y2 = blob.major_axis_line()

        # Stage C: unproject endpoints to the ground, fit y=mx+c.
        p1 = get_polar_position(x1, y1)
        p2 = get_polar_position(x2, y2)

        img.draw_line(x1, y1, x2, y2, image.COLOR_RED, 2)
        img.draw_rect(blob.x(), blob.y(), blob.w(), blob.h(), image.COLOR_GREEN, 1)

        if p1 is None or p2 is None:
            continue
        gx1, gy1 = p1[1] * sin(radians(p1[0])), p1[1] * cos(radians(p1[0]))
        gx2, gy2 = p2[1] * sin(radians(p2[0])), p2[1] * cos(radians(p2[0]))
        m, c = line_equation(gx1, gy1, gx2, gy2)
        img.draw_string(blob.x(), blob.y() - 12,
                        f"m={m:.1f} c={c:.0f}", image.COLOR_YELLOW)

    img.draw_string(0, 0, f"lines: {kept}/{len(blobs)}", image.COLOR_BLUE)
    disp.show(img)
