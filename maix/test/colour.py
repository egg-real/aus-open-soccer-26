from maix import camera, display, image

IMG_WIDTH = 640
IMG_HEIGHT = 360
CENTER_X = IMG_WIDTH // 2
CENTER_Y = IMG_HEIGHT // 2
CROSSHAIR_SIZE = 16

cam = camera.Camera(IMG_WIDTH, IMG_HEIGHT, image.Format.FMT_RGB888)
disp = display.Display()


while True:
    img = cam.read()
    pixel = img.get_pixel(CENTER_X, CENTER_Y, rgbtuple=True)

    img.draw_line(
        CENTER_X - CROSSHAIR_SIZE,
        CENTER_Y,
        CENTER_X + CROSSHAIR_SIZE,
        CENTER_Y,
        image.COLOR_RED,
        2,
    )
    img.draw_line(
        CENTER_X,
        CENTER_Y - CROSSHAIR_SIZE,
        CENTER_X,
        CENTER_Y + CROSSHAIR_SIZE,
        image.COLOR_RED,
        2,
    )
    img.draw_string(0, 0, f"center RGB: {pixel}", image.COLOR_BLUE)

    disp.show(img)
