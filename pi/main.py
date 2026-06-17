from camera import Cameras

cams = Cameras(["/dev/ttyAMA0"])

while True:
    cams.process()
    ball_dir = cams.get_ball_dir()
    ball_dist = cams.get_ball_dist()
    if ball_dir is not None and ball_dist is not None:
        print(ball_dir, ball_dist)
    else:
        print("Ball not found")
