import math
import time

from lib.drive import Drive
from lib.imu import IMU
from lib.localisation import Localisation
from lib.tof import ToF


TARGET_TOLERANCE_MM = 10
MAX_SPEED = 0.6
SLOW_RADIUS_MM = 300
LOOP_DELAY_SECONDS = 0.02


def wrap_angle(theta):
    return (theta + 180) % 360 - 180


imu = IMU()
drive = Drive(imu)
tofs = (ToF(0x50), ToF(0x51), ToF(0x52), ToF(0x53))
localisation = Localisation(imu, drive, tofs)

try:
    while True:
        current_position = localisation.get_position()
        print(f"Current position: {current_position}")
finally:
    drive.stop()
