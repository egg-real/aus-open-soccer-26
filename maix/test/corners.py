from maix import camera, display, image
import numpy as np
import math
import os

# Screen->world lookup, shape (640, 360, 2): [x][y] -> [angle_deg, distance_mm].
# Matches getPolarPosition in maix/main.py. The .npy lives in maix/, one level
# up from this test script, so resolve it relative to this file.
_NPY_PATH = os.path.join(os.path.dirname(__file__), "..", "screen-2-world-polar.npy")
screenToWorldPolar = np.load(_NPY_PATH)

IMG_WIDTH = 640
IMG_HEIGHT = 360


def get_polar(x_pixel, y_pixel):
    """Look up (angle_deg, distance_mm) for a screen pixel, or None if off-screen."""
    xi = int(round(x_pixel))
    yi = int(round(y_pixel))
    if xi < 0 or xi >= IMG_WIDTH or yi < 0 or yi >= IMG_HEIGHT:
        return None
    angle, distance = screenToWorldPolar[xi][yi]
    return float(angle), float(distance)


# ----- Thresholds (LAB, L range 0-100 for RGB888) ----- #
WHITE = [[80, 100, -10, 10, -10, 10]]
GREEN = [[0, 80, -120, -10, 0, 30]]
MASK_WHITE = [[50, 100, -128, 127, -128, 127]]  # white pixels of a binary image

# ----- Line tuning knobs (same rules as line.py) ----- #
GREEN_DILATE = 2        # px to dilate green so it bridges over thin lines
MIN_AREA = 60           # drop specks
MIN_ASPECT = 3.0        # major-axis length must be >= MIN_ASPECT * minor-axis
MAX_THICKNESS = 40      # px; reject blobs whose short side is fatter than this
MIN_LENGTH = 40         # px; reject stubs that are too short to trust as lines

# ----- Corner tuning knobs ----- #
PERP_TOL = 20.0         # deg; two lines count as a corner within this of 90 deg
MAX_JOIN_GAP = 30       # px; intersection must lie this close to BOTH segments
DEDUP_PX = 25           # px; merge corners found this close together

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


def perpendicularity_error(seg_a, seg_b):
    """How far (deg) the angle between two segments is from a right angle."""
    ax1, ay1, ax2, ay2 = seg_a
    bx1, by1, bx2, by2 = seg_b
    ang_a = math.degrees(math.atan2(ay2 - ay1, ax2 - ax1))
    ang_b = math.degrees(math.atan2(by2 - by1, bx2 - bx1))
    # Lines are undirected, so fold the difference into [0, 180).
    diff = abs(ang_a - ang_b) % 180.0
    return abs(diff - 90.0)


def intersect(seg_a, seg_b):
    """Intersection of the two infinite lines through the segments, or None."""
    x1, y1, x2, y2 = seg_a
    x3, y3, x4, y4 = seg_b
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 1e-6:
        return None  # parallel
    pre_a = x1 * y2 - y1 * x2
    pre_b = x3 * y4 - y3 * x4
    px = (pre_a * (x3 - x4) - (x1 - x2) * pre_b) / den
    py = (pre_a * (y3 - y4) - (y1 - y2) * pre_b) / den
    return px, py


def point_seg_dist(px, py, seg):
    """Shortest distance from a point to a segment."""
    x1, y1, x2, y2 = seg
    dx, dy = x2 - x1, y2 - y1
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-6:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    fx, fy = x1 + dx * t, y1 + dy * t
    return math.hypot(px - fx, py - fy)


while True:
    img = cam.read()

    # Isolate white-on-green pixels so off-field white can't trigger a line.
    green = img.binary(GREEN, copy=True)
    green.dilate(GREEN_DILATE)
    line_px = img.binary(WHITE, copy=True)
    line_px.b_and(green)

    blobs = line_px.find_blobs(MASK_WHITE, pixels_threshold=MIN_AREA,
                               area_threshold=MIN_AREA, merge=True)

    # Keep only line-shaped blobs and remember their major axes.
    segs = []
    for blob in blobs:
        if not is_line_shaped(blob):
            # Not line shaped: likely another white object, ignore it.
            continue
        seg = blob.major_axis_line()
        segs.append(seg)
        img.draw_line(seg[0], seg[1], seg[2], seg[3], image.COLOR_RED, 2)

    # A corner is where two lines meet perpendicularly: roughly a right angle
    # between them AND their intersection actually sits on both segments.
    corners = []  # (px, py, angle_deg, distance_mm)
    for i in range(len(segs)):
        for j in range(i + 1, len(segs)):
            if perpendicularity_error(segs[i], segs[j]) > PERP_TOL:
                continue
            pt = intersect(segs[i], segs[j])
            if pt is None:
                continue
            px, py = pt
            if point_seg_dist(px, py, segs[i]) > MAX_JOIN_GAP:
                continue
            if point_seg_dist(px, py, segs[j]) > MAX_JOIN_GAP:
                continue
            polar = get_polar(px, py)
            if polar is None:
                continue

            # Skip corners that duplicate one we already found nearby.
            if any(math.hypot(px - cx, py - cy) < DEDUP_PX
                   for cx, cy, _, _ in corners):
                continue
            corners.append((px, py, polar[0], polar[1]))

    if not corners:
        print("no corners")
        img.draw_string(0, 0, "no corners", image.COLOR_BLUE)
    else:
        parts = []
        for px, py, angle, distance in corners:
            parts.append("%.0fmm@%.0fdeg" % (distance, angle))
            img.draw_circle(int(px), int(py), 6, image.COLOR_YELLOW, 2)
            img.draw_string(int(px) + 8, int(py) - 8,
                            "%.0fmm %.0fdeg" % (distance, angle),
                            image.COLOR_YELLOW)
        print("corners: " + ", ".join(parts))
        img.draw_string(0, 0, "corners: %d" % len(corners), image.COLOR_BLUE)

    disp.show(img)
