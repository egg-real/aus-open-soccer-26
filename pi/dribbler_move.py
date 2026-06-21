import time

from lib.dribbler import Dribbler
from lib.drive import Drive
from lib.break_beam import BreakBeam
import board

DRIBBLER_SPEED = 1.0
MOVE_SPEED = 0.2


if __name__ == "__main__":
    dribbler = Dribbler()
    drive = Drive()
    break_beam = BreakBeam(board.D17)

    dribbler.set_speed(DRIBBLER_SPEED)

    try:
        drive.move(0, MOVE_SPEED)
        while True:
            if break_beam.read():
                drive.move(180, MOVE_SPEED)
            else:
                drive.move(0, MOVE_SPEED)
    except KeyboardInterrupt:
        pass
    finally:
        dribbler.set_speed(0)
        drive.stop()
