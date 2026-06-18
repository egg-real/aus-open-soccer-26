import time

from lib.dribbler import Dribbler
from lib.drive import Drive


DRIBBLER_SPEED = 1.0
MOVE_SPEED = 0.2
MOVE_DURATION_SECONDS = 2


if __name__ == "__main__":
    dribbler = Dribbler()
    drive = Drive()

    dribbler.set_speed(DRIBBLER_SPEED)

    try:
        drive.move(0, MOVE_SPEED)
        time.sleep(MOVE_DURATION_SECONDS)

        drive.move(180, MOVE_SPEED)
        time.sleep(MOVE_DURATION_SECONDS)
    except KeyboardInterrupt:
        pass
    finally:
        dribbler.set_speed(0)
        drive.stop()
