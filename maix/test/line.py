from maix import camera, display, image
import numpy as np
import math
import os

IMG_WIDTH = 640
IMG_HEIGHT = 360

# Screen->world lookup, shape (640, 360, 2): [x][y] -> [angle_deg, distance_mm].
_NPY_PATH = os.path.join(os.path.dirname(__file__), "..", "screen-2-world-polar.npy")
screenToWorldPolar = np.load(_NPY_PATH)


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
    angle_rad = math.radians(float(angle))
    distance = float(distance)
    world_x = distance * math.sin(angle_rad)
    world_y = distance * math.cos(angle_rad)
    return world_x, world_y


# ----- Thresholds (LAB, L range 0-100 for RGB888) ----- #
WHITE = [[80, 100, -10, 10, -10, 10]]
GREEN = [[0, 80, -120, -10, 0, 30]]
MASK_WHITE = [[50, 100, -128, 127, -128, 127]]  # white pixels of a binary image

# ----- Tuning knobs ----- #
GREEN_DILATE = 2        # px to dilate green so it bridges over thin lines
MIN_AREA = 60           # drop specks
# A line is thin in one dimension and long in the other. Anything that is
# chunky in both dimensions is some other white object, so reject it.
MIN_ASPECT = 3.0        # major-axis length must be >= MIN_ASPECT * minor-axis
MAX_THICKNESS = 40      # px; reject blobs whose short side is fatter than this
MIN_LENGTH = 40         # px; reject stubs that are too short to trust as lines

cam = camera.Camera(IMG_WIDTH, IMG_HEIGHT)
disp = display.Display()


def _axis_length(axis_line):
    x1, y1, x2, y2 = axis_line
    return math.hypot(x2 - x1, y2 - y1)


def is_line_shaped(blob):
    """True when the blob is thin in one dimension and long in the other."""
    if blob.area() < MIN_AREA:
        return False

    major = _axis_length(blob.major_axis_line())
    minor = _axis_length(blob.minor_axis_line())

    if major < MIN_LENGTH:
        return False
    if minor > MAX_THICKNESS:
        return False
    if minor <= 0 or major / minor < MIN_ASPECT:
        return False
    return True


def closest_point_on_blob(blob):
    """Closest point of the line to the camera, as (angle_deg, distance_mm).

    The camera sits at the ground-frame origin, so the closest point on the
    line is the foot of the perpendicular dropped from the origin onto the
    line. That perpendicular is, by definition, at right angles to the line
    itself. We unproject the major-axis endpoints to the ground, then project
    the origin onto that segment (clamped, so when the foot falls beyond an end
    of the visible line the nearer endpoint is used instead).
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
        # Degenerate: endpoints unprojected to the same spot.
        fx, fy = ax, ay
    else:
        # Param of the origin's projection onto the infinite line, clamped to
        # the visible segment.
        t = -(ax * dx + ay * dy) / seg_len_sq
        t = max(0.0, min(1.0, t))
        fx, fy = ax + dx * t, ay + dy * t

    distance = math.hypot(fx, fy)
    angle = math.degrees(math.atan2(fx, fy))
    return angle, distance


while True:
    img = cam.read()

    # Isolate white-on-green pixels so off-field white can't trigger a line.
    green = img.binary(GREEN, copy=True)
    green.dilate(GREEN_DILATE)
    line_px = img.binary(WHITE, copy=True)
    line_px.b_and(green)

    blobs = line_px.find_blobs(MASK_WHITE, pixels_threshold=MIN_AREA,
                               area_threshold=MIN_AREA, merge=True)

    closest = None  # (angle_deg, distance_mm)
    for blob in blobs:
        if not is_line_shaped(blob):
            # Not line shaped: likely another white object, ignore it.
            continue

        result = closest_point_on_blob(blob)
        if result is None:
            continue
        if closest is None or result[1] < closest[1]:
            closest = result

        x1, y1, x2, y2 = blob.major_axis_line()
        img.draw_line(x1, y1, x2, y2, image.COLOR_RED, 2)

    if closest is None:
        print("no line")
        img.draw_string(0, 0, "no line", image.COLOR_BLUE)
    else:
        angle, distance = closest
        print("closest line: %.0f mm @ %.0f deg" % (distance, angle))
        img.draw_string(0, 0, "closest: %.0f mm @ %.0f deg" % (distance, angle),
                        image.COLOR_BLUE)

    disp.show(img)
