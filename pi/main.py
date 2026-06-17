from camera import Cameras

cams = Cameras(["/dev/ttyAMA0"], naive=False)

while 1:
    ball_dir = cams.process()
    print(ball_dir)