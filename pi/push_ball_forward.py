import math
from camera import Cameras
from lib.drive import Drive

cams = Cameras()
cams.start_streaming()
drive = Drive()

while True:
    cams.process()
    ball_dir = cams.get_ball_dir()
    ball_dist = cams.get_ball_dist()
    if ball_dir is not None and ball_dist is not None:
        print(f"Ball direction: {ball_dir}, distance: {ball_dist}")
        dx = ball_dist * math.cos(math.radians(ball_dir))
        dy = ball_dist * math.sin(math.radians(ball_dir))
        direction = math.degrees(math.atan2(dy, dx))

        offset = 0
        speed = 0.5
        if ball_dist < 400:
            if -10 < direction < 10:
                direction = 0
                speed = 0.8
            elif 0 < direction < 180:
                offset = 80
            else:
                offset = -80
        elif ball_dist > 500:
            speed = 800
        direction += offset
        direction = direction % 360
        print(f"Direction: {direction}, Speed: {speed}")
        drive.move(direction, speed)

    else:
        drive.move(0, 0, 0)
        print("Ball not found")