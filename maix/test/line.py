from maix import camera, display, image, time

cam = camera.Camera(320, 240, image.Format.FMT_GRAYSCALE)
disp = display.Display()

# thresholds = [[0, 80, 40, 80, 10, 80]] # red
thresholds = [[80, 100]] # green
# thresholds = [[0, 80, 30, 100, -120, -60]] # blue

while 1:
    img = cam.read()

    lines = img.get_regression(thresholds, area_threshold = 100)
    for a in lines:
        img.draw_line(a.x1(), a.y1(), a.x2(), a.y2(), image.COLOR_GREEN, 2)
        theta = a.theta()
        rho = a.rho()
        if theta > 90:
            theta = 270 - theta
        else:
            theta = 90 - theta
        img.draw_string(0, 0, "theta: " + str(theta) + ", rho: " + str(rho), image.COLOR_BLUE)

    disp.show(img)
    fps = time.fps()
    print("fps: %.1f" % fps)
