from maix import camera as _camera, uart as _uart, pinmap as _pinmap
import time


class Camera(_camera.Camera):
    def __init__(self, w:int, h:int, port="/dev/ttyS3"):
        super().__init__(w, h)
        # CHECK THAT PINS ARE MAPPED BEFORE PORT IS INITIALISED
        _pinmap.set_pin_function("P19", "UART3_TX")
        _pinmap.set_pin_function("P20", "UART3_RX")

        self.uart = _uart.UART(port, 115200)

        print("\n\n!!!PORT!!!: " + self.uart.get_port() + "\n\n")

    def _write(self, msg=0b0100010):
        if not self.uart.is_open():
            return False
        
        print("[debug] wrote " + bin(msg))
        self.uart.write(msg)
        time.sleep(0.005)
        
        return True
    
    def send_packet(self, has_ball:bool=1, ball_dir:int=128):
        packet = 0b0
        
        sign_bit = bool((ball_dir/abs(ball_dir) + 1) / 2)
        num = min(abs(ball_dir), 127)
        packet = ((packet + sign_bit) << 7) + num

        # packet = (packet << 1) + has_ball

        self._write(packet)


        
if __name__ == "__main__":
    cam = Camera(360, 540)
    frame = cam.read()