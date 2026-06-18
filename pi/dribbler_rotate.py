import time

from lib.dribbler import Dribbler
from lib.drive import Drive


DRIBBLER_SPEED = 1.0
CLOCKWISE_DEGREES_PER_SECOND = -90.0
UPDATE_INTERVAL_SECONDS = 0.05


if __name__ == "__main__":
    dribbler = Dribbler()
    drive = Drive()

    dribbler.set_speed(DRIBBLER_SPEED)

    target_rotation = 0.0
    try:
        while True:
            target_rotation += CLOCKWISE_DEGREES_PER_SECOND * UPDATE_INTERVAL_SECONDS
            drive.move(0, 0, target_rotation)
            time.sleep(UPDATE_INTERVAL_SECONDS)
    except KeyboardInterrupt:
        pass
    finally:
        dribbler.set_speed(0)
        drive.stop()
