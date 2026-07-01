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
        target_x = float(input("What x position to move to? "))
        target_y = float(input("What y position to move to? "))
        current_position = localisation.get_position()
        print(f"Current position: {current_position}")
        while True:
            dx = target_x - current_position[0]
            dy = target_y - current_position[1]
            distance = math.hypot(dx, dy)
            if distance <= TARGET_TOLERANCE_MM:
                break

            field_direction = math.degrees(math.atan2(dx, dy))
            move_direction = wrap_angle(field_direction - drive.yaw)
            speed = min(MAX_SPEED, distance / SLOW_RADIUS_MM * MAX_SPEED)
            drive.move(move_direction, speed, drive.yaw)

            time.sleep(LOOP_DELAY_SECONDS)
            current_position = localisation.get_position()
        drive.stop()
finally:
    drive.stop()
