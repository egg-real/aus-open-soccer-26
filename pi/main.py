from camera import Cameras

cams = Cameras(["/dev/ttyAMA0"], naive=True)

while True:
    ball_dir = cams.process()
    print(ball_dir)