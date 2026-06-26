from maix import app, camera, display, image


IMG_WIDTH = 640
IMG_HEIGHT = 360
CENTER_X = IMG_WIDTH // 2

SAMPLE_COUNT = 16
SAMPLE_MARGIN = 20
BLACK_THRESHOLD = 80


cam = camera.Camera(IMG_WIDTH, IMG_HEIGHT, image.Format.FMT_RGB888)
disp = display.Display()


def is_black(pixel):
    r, g, b = pixel
    return r <= BLACK_THRESHOLD and g <= BLACK_THRESHOLD and b <= BLACK_THRESHOLD


def sample_y_positions():
    if SAMPLE_COUNT <= 1:
        return [IMG_HEIGHT // 2]

    span = IMG_HEIGHT - (SAMPLE_MARGIN * 2)
    step = span / (SAMPLE_COUNT - 1)
    return [int(SAMPLE_MARGIN + (step * i)) for i in range(SAMPLE_COUNT)]


sample_points = [(CENTER_X, y) for y in sample_y_positions()]


while not app.need_exit():
    img = cam.read()

    black_count = 0
    for x, y in sample_points:
        pixel = img.get_pixel(x, y, rgbtuple=True)
        black = is_black(pixel)
        black_count += 1 if black else 0
        print("point (%d, %d): RGB %s black=%s" % (x, y, pixel, black))

        colour = image.COLOR_GREEN if black else image.COLOR_RED
        img.draw_circle(x, y, 4, colour, 2)

    img.draw_line(CENTER_X, 0, CENTER_X, IMG_HEIGHT - 1, image.COLOR_BLUE, 1)
    img.draw_string(
        0,
        0,
        "black: %d/%d threshold: <=%d" % (black_count, SAMPLE_COUNT, BLACK_THRESHOLD),
        image.COLOR_BLUE,
    )

    disp.show(img)
