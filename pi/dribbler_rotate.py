import time

from lib.dribbler import Dribbler
from lib.drive import Drive
from lib.kicker import Kicker

from board import D27


DRIBBLER_SPEED = 1.0
UPDATE_INTERVAL_SECONDS = 0.05


if __name__ == "__main__":
    dribbler = Dribbler()
    drive = Drive()
    kicker = Kicker(D27, 0.02)

    dribbler.set_speed(DRIBBLER_SPEED)

    time.sleep(1)

    target_rotation = 0.0
    try:
        drive.move(0, 0, 180, True)
        time.sleep(3)
        dribbler.set_speed(0)
        kicker.kick()
        # time.sleep(UPDATE_INTERVAL_SECONDS)
    finally:
        dribbler.set_speed(0)
        drive.stop()
        kicker.deinit()
