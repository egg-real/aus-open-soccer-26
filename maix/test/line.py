from maix import camera, display, image

cam = camera.Camera(320, 240)
disp = display.Display()

# thresholds = [[0, 80, 40, 80, 10, 80]] # red
# thresholds = [[0, 80, -120, -10, 0, 30]] # green
# thresholds = [[0, 80, 30, 100, -120, -60]] # blue
thresholds = [[80, 100, -10, 10, -10, 10]] # white

while 1:
    img = cam.read()

    blobs = img.find_blobs(thresholds, pixels_threshold = 100, area_threshold = 100, merge = False)
    for blob in blobs:
        img.draw_rect(blob.x(), blob.y(), blob.w(), blob.h(), image.COLOR_GREEN, 2)
        img.draw_cross(blob.cx(), blob.cy(), image.COLOR_BLUE, 8, 2)

    img.draw_string(0, 0, "white lines: " + str(len(blobs)), image.COLOR_BLUE)

    disp.show(img)
