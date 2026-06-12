import time
import math
import threading
from lib.motor import Motor
import lib.config as config
from math import radians, sin

SMOOTHING_TIME = 0.30

class Drive:
    def __init__(self, motors: list[str]=["ne", "se", "sw", "nw"]):
        motors_config = config.get_value("motors")
        self.motors = {}
        self.current_direction = 0
        self.current_speed = 0
        self.target_direction = 0
        self.target_speed = 0
        self.target_lock = threading.Lock()
        self.last_update_time = time.monotonic()
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

    def move(self, angle, speed=0.5):
        with self.target_lock:
            self.target_direction = angle
            self.target_speed = speed

    def _drive_loop(self):
        motors_config = config.get_value("motors")
        while True:
            self._update_current_velocity()

            for motor_direction, motor in self.motors.items():
                motor_config = motors_config.get(motor_direction)
                motor_angle = self.current_direction - motor_config["angle_off"]
                motor_speed = self.current_speed * -sin(radians(motor_angle))
                motor.set_speed(motor_speed)

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

        self.current_direction = 0
        self.current_speed = 0

        for motor in self.motors.values():
            motor.set_speed(0)