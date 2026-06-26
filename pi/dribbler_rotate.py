import math
import time

from lib.dribbler import Dribbler
from lib.drive import Drive, wrap_angle

DRIBBLER_SPEED = 1.0
STRAFE_SPEED = 0.4
ORBIT_SIGN = 1  # +1 strafe right, -1 strafe left
ORBIT_RADIUS_CM = 9.0  # distance from bot centre to ball
UPDATE_INTERVAL_SECONDS = 0.02

STRAFE_DIR = 90 * ORBIT_SIGN
RAD_TO_DEG = 180.0 / math.pi


def get_orbit_yaw_rate_deg(strafe_speed_ms, orbit_radius_cm):
    """Yaw rate (deg/s) to stay tidally locked while orbiting at strafe_speed_ms."""
    orbit_radius_m = orbit_radius_cm / 100.0
    if orbit_radius_m <= 0:
        return 0.0
    # Positive strafe right -> turn left (negative yaw).
    return -(strafe_speed_ms / orbit_radius_m) * RAD_TO_DEG


if __name__ == "__main__":
    dribbler = Dribbler()
    drive = Drive()

    dribbler.set_speed(DRIBBLER_SPEED)
    target_yaw = 0.0

    try:
        while True:
            strafe_speed_ms = drive.get_speed_in_direction(STRAFE_DIR)
            omega_deg = get_orbit_yaw_rate_deg(strafe_speed_ms, ORBIT_RADIUS_CM)
            target_yaw = wrap_angle(target_yaw + omega_deg * UPDATE_INTERVAL_SECONDS)

            drive.move(STRAFE_DIR, STRAFE_SPEED, target_yaw)
            time.sleep(UPDATE_INTERVAL_SECONDS)
    except KeyboardInterrupt:
        pass
    finally:
        dribbler.set_speed(0)
        drive.stop()
