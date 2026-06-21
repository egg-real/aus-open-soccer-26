import time
import math
import threading
from lib.motor import Motor
import lib.config as config
from lib.imu import IMU
from math import radians, sin

SMOOTHING_TIME = 0.30
YAW_CORRECT_THRESHOLD = 3.0
YAW_CORRECT_SPEED = 0.1
POSSESSION_YAW_CORRECT_SPEED = 0.05

def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))

def wrap_angle(theta):
    """Input an angle in degrees. Returns same angle but in [-180, 180)."""
    return (theta + 180) % 360 - 180

def capture_startup_yaw(imu, sample_count=25, sample_interval=0.02):
    """Average a short burst of IMU samples so startup yaw is not just the first reading."""
    print("Stabilizing IMU yaw reference...")
    sin_sum = 0.0
    cos_sum = 0.0
    samples = 0
    while samples < sample_count:
        yaw = imu.get_yaw()
        if yaw is not None:
            yaw_rad = math.radians(yaw)
            sin_sum += math.sin(yaw_rad)
            cos_sum += math.cos(yaw_rad)
            samples += 1
        time.sleep(sample_interval)
    return math.degrees(math.atan2(sin_sum, cos_sum))

class Drive:
    def __init__(self, motors: list[str]=["ne", "se", "sw", "nw"]):
        motors_config = config.get_value("motors")
        self.motors = {}
        self.current_direction = 0
        self.current_speed = 0
        self.target_direction = 0
        self.target_speed = 0
        self.target_rotation = 0
        self.target_yaw_correct_speed = YAW_CORRECT_SPEED
        self.target_lock = threading.Lock()
        self.last_update_time = time.monotonic()
        self.imu = IMU()
        self.initial_yaw = capture_startup_yaw(self.imu)
        self.yaw = 0
        for motor_direction in motors:
            motor_config = motors_config.get(motor_direction)
            try:
                self.motors[motor_direction] = Motor(motor_config["address"],
                                            elec_angle_offset=motor_config["elec_angle_offset"],
                                            sin_cos_centre=motor_config["sin_cos_centre"])
            except Exception as e:
                print(f"Error initializing motor at address {motor_config['address']}: {e}")

        self.drive_thread = threading.Thread(target=self._drive_loop, daemon=True)
        self.drive_thread.start()

    def move(self, angle, speed=0.5, rotation=0, possession=False): # rotation is the desired yaw value
        with self.target_lock:
            self.target_direction = angle
            self.target_speed = speed
            self.target_rotation = wrap_angle(rotation)
            if possession:
                self.target_yaw_correct_speed = POSSESSION_YAW_CORRECT_SPEED
            else:
                self.target_yaw_correct_speed = YAW_CORRECT_SPEED

    def _drive_loop(self):
        motors_config = config.get_value("motors")
        while True:
            self._update_current_velocity()
            yaw_correction = self._get_yaw_correction()

            for motor_direction, motor in self.motors.items():
                motor_config = motors_config.get(motor_direction)
                motor_angle = self.current_direction - motor_config["angle_off"]
                motor_speed = self.current_speed * -sin(radians(motor_angle))
                motor_speed = clamp(motor_speed - yaw_correction, -1.0, 1.0)
                motor.set_speed(motor_speed)

    def _get_yaw_correction(self):
        raw_yaw = self.imu.get_yaw()
        if raw_yaw is None:
            return 0

        self.yaw = wrap_angle(self.initial_yaw - raw_yaw)
        with self.target_lock:
            target_rotation = self.target_rotation
            yaw_correct_speed = self.target_yaw_correct_speed

        yaw_error = wrap_angle(target_rotation - self.yaw)
        if abs(yaw_error) <= YAW_CORRECT_THRESHOLD:
            return 0

        return clamp(
            (yaw_error / 60.0) * yaw_correct_speed,
            -yaw_correct_speed,
            yaw_correct_speed,
        )

    def _update_current_velocity(self):
        with self.target_lock:
            target_direction = self.target_direction
            target_speed = self.target_speed

        dx = math.cos(math.radians(self.current_direction)) * self.current_speed
        dy = math.sin(math.radians(self.current_direction)) * self.current_speed

        target_dx = math.cos(math.radians(target_direction)) * target_speed
        target_dy = math.sin(math.radians(target_direction)) * target_speed

        now = time.monotonic()
        dt = now - self.last_update_time
        self.last_update_time = now

        alpha = min(dt / SMOOTHING_TIME, 1.0)
        new_dx = dx + (target_dx - dx) * alpha
        new_dy = dy + (target_dy - dy) * alpha

        self.current_direction = math.degrees(math.atan2(new_dy, new_dx))
        self.current_speed = math.hypot(new_dx, new_dy)

    def stop(self):
        with self.target_lock:
            self.target_direction = 0
            self.target_speed = 0
            self.target_rotation = self.yaw

        self.current_direction = 0
        self.current_speed = 0

        for motor in self.motors.values():
            motor.set_speed(0)