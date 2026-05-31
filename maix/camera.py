from maix import camera as _camera, uart as _uart, pinmap as _pinmap, image as _image, display as _display, time as _time
from math import sin, cos, pi, copysign


class Camera():
    def __init__(self, w:int, h:int, debug=False, show=False):
        self.cam = _camera.Camera(w, h)
        self._debug = debug
        self._show = show

        self._w = w
        self._h = h

        if debug:
            self.disp = _display.Display()

    def read(self, prev_angle=None):
        try:
            img = self.cam.read()
            frame = _image.image2cv(img)
            
            if self._show:
                if prev_angle is not None:
                    a = -(prev_angle - 90) * (pi/180)
                    length = 50
                    cx = int(self._w / 2)
                    cy = int(self._h - 1)
                    end_x = int(cx + cos(a) * length)
                    end_y = int(cy - sin(a) * length)

                    img = img.draw_line(cx, cy, end_x, end_y, _image.COLOR_ORANGE, 2)
                self.disp.show(img)

            if self._debug:
                print(f"FPS: {_time.fps()}")

            del img
            return True, frame
        except RuntimeError:
            return False, None


class UART():
    def __init__(self, port="/dev/ttyS3"):
        # CHECK THAT PINS ARE MAPPED BEFORE PORT IS INITIALISED
        _pinmap.set_pin_function("P19", "UART3_TX")
        _pinmap.set_pin_function("P20", "UART3_RX")

        self.uart = _uart.UART(port, 115200)

        print("\n\n!!!PORT!!!: " + self.uart.get_port() + "\n\n")
    
    def _write(self, msg=0b00000000.to_bytes(1)):
        if not self.uart.is_open():
            return False
        
        print("[debug] wrote " + bin(int.from_bytes(msg)))
        self.uart.write(msg)
        _time.sleep_ms(5)
        
        return True
    
    def send_packet(self, has_ball:bool=True, ball_dir:int=128):
        packet = 0b0
        
        sign_bit = bool((copysign(1, ball_dir) + 1) / 2) # 0 for neg, 1 for pos
        num = min(abs(int(ball_dir)), 127)
        packet = ((packet + sign_bit) << 7) | num

        # packet = (packet << 1) + has_ball

        self._write(packet.to_bytes((packet.bit_length() + 7) // 8))