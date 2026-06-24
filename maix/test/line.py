from maix import camera, display, image

cam = camera.Camera(80, 60, image.Format.FMT_GRAYSCALE)
disp = display.Display()

# thresholds = [[0, 80, 40, 80, 10, 80]] # red
# thresholds = [[0, 80, -120, -10, 0, 30]] # green
# thresholds = [[0, 80, 30, 100, -120, -60]] # blue
thresholds = [[70, 100]]

while True:
    img = cam.read()

    lines = img.find_lines(thresholds, area_threshold = 20)
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