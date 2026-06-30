import time

import board

from lib.kicker import Kicker
from lib.dribbler import Dribbler
from lib.break_beam import BreakBeam
from lib.drive import Drive

DRIBBLER_SPEED = 1.0
UPDATE_INTERVAL_SECONDS = 0.02
SPIN_TARGET_DEG = 180.0
# Nominal strafe speed (m/s) for tidal-lock spin rate while in possession.
NOMINAL_ORBIT_STRAFE_MS = 0.3

SOLENOID_PIN = board.D27
BREAK_BEAM_PIN = board.D17
PULSE_S = 0.02


if __name__ == "__main__":
    dribbler = Dribbler()
    drive = Drive()
    kicker = Kicker(SOLENOID_PIN, PULSE_S)
    break_beam = BreakBeam(BREAK_BEAM_PIN)

    dribbler.set_speed(DRIBBLER_SPEED)
    target_yaw = 180.0

    try:
        while drive.yaw < target_yaw:
            have_ball = break_beam.read()
            drive.move(0, 0, target_yaw, possession=have_ball)
            time.sleep(UPDATE_INTERVAL_SECONDS)

        drive.stop()
        dribbler.set_speed(0)
        kicker.kick()
        time.sleep(PULSE_S + 0.1)
    except KeyboardInterrupt:
        pass
    finally:
        dribbler.set_speed(0)
        drive.stop()
        kicker.deinit()
