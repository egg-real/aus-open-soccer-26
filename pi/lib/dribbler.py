from lib.motor import Motor
import lib.config as config

class Dribbler:
    def __init__(self):
        dribbler_config = config.get_value("motors").get("dribbler")
        self.motor = Motor(dribbler_config["address"],
                            elec_angle_offset=dribbler_config["elec_angle_offset"],
                            sin_cos_centre=dribbler_config["sin_cos_centre"])

    def set_speed(self, speed):
        self.motor.set_speed(speed)