import digitalio
from lib.config import Config

class Switch:
    switch = None
    def __init__(self, pin, config:Config=Config()):
        self.on_when_high = config.get_value("")
        self.switch = digitalio.DigitalInOut(pin)
        self.switch.direction = digitalio.Direction.INPUT
        self.switch.pull = digitalio.Pull.UP

    def read(self):
        return not self.switch.value