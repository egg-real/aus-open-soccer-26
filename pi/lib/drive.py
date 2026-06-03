from lib.motor import Motor
import lib.config as config
from math import radians, sin

class Drive:
    def __init__(self, motors: list[str]=["ne", "se", "sw", "nw"]):
        motors_config = config.get_value("motors")
        self.motors = {}
        for motor_direction in motors:
            motor_config = motors_config.get(motor_direction)
            try:
                self.motors[motor_direction] = Motor(motor_config["address"],
                                            elec_angle_offset=motor_config["elec_angle_offset"],
                                            sin_cos_centre=motor_config["sin_cos_centre"])
            except Exception as e:
                print(f"Error initializing motor at address {motor_config["address"]}: {e}")

    def move(self, angle, speed=0.5):
        motors_config = config.get_value("motors")
        for motor_direction, motor in self.motors.items():
            motor_config = motors_config.get(motor_direction)
            motor_angle = angle - motor_config["angle_off"]
            motor_speed = speed * -sin(radians(motor_angle))
            motor.set_speed(motor_speed)

    def stop(self):
        self.move(0, 0)