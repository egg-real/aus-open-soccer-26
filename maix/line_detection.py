# Stashed line detection from main.py — regression-based white line finder.
# Re-import with: from line_detection import find_closest_line

# Same regression thresholds as maix/test/line.py (LAB, L range 0-100).
LINE_THRESHOLDS = [[90, 100, -10, 10, -10, 10]]
LINE_AREA_THRESHOLD = 100


def regression_theta_to_dir(theta):
    """Convert Maix regression theta to a signed bearing (see maix/test/line.py)."""
    if theta > 90:
        return 270 - theta
    return 90 - theta


def find_closest_line(img, get_polar_position, img_width, img_height, do_disp=False):
    """Return the nearest white line as (angle_deg, distance_mm), or None."""
    from maix import image

    lines = img.get_regression(LINE_THRESHOLDS, area_threshold=LINE_AREA_THRESHOLD)
    if not lines:
        return None

    closest = None
    cx = img_width / 2
    cy = img_height / 2

    for line in lines:
        x1, y1, x2, y2 = line.x1(), line.y1(), line.x2(), line.y2()
        dx = x2 - x1
        dy = y2 - y1
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-6:
            px, py = x1, y1
        else:
            t = ((cx - x1) * dx + (cy - y1) * dy) / seg_len_sq
            t = max(0.0, min(1.0, t))
            px = x1 + t * dx
            py = y1 + t * dy

        polar = get_polar_position(px, py)
        if polar is None:
            continue
        angle, dist = polar
        if closest is None or dist < closest[1]:
            closest = (angle, dist)

        if do_disp:
            img.draw_line(x1, y1, x2, y2, image.COLOR_GREEN, 2)
            theta = regression_theta_to_dir(line.theta())
            rho = line.rho()
            img.draw_string(0, 0, "theta: %d, rho: %d" % (theta, rho), image.COLOR_BLUE)

    return closest
