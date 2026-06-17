import digitalio

class CommModule:
    output = None
    def __init__(self, output_pin: digitalio.DigitalInOut):
        self.output = digitalio.DigitalInOut(output_pin)
        self.output.direction = digitalio.Direction.INPUT
        self.output.pull = digitalio.Pull.DOWN
    
    def read(self) -> bool:
        return self.output.value