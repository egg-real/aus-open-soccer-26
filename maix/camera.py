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
    
    def _write(self, msg):
        if not self.uart.is_open():
            return False
        # print("[debug] wrote " + bin(int.from_bytes(msg, "big")))
        self.uart.write(msg)    
        _time.sleep_ms(5)
        
        return True
    
    @staticmethod
    def _packsigned(num):
        sign_bit = bool((copysign(1, num) + 1) / 2)
        # 0xff is reserved as the packet start flag, so avoid it in data bytes.
        num = min(abs(int(num)), 126 if sign_bit else 127)
        return ((sign_bit) << 7) | num
    
    def send_packet(self, see_ball:bool=False, ball_dir:int=0, ball_dist:int=0,
                    see_goal:bool=False, yellow_goal:bool=False, goal_dir:int=0, goal_dist:int=0,
                    wall_dir:int=0, wall_dist:int=0,
                    cam_ok:bool=True):

        info_byte = 0x08 * see_ball + 0x04 * see_goal + 0x02 * yellow_goal + 0x01 * cam_ok

        packet = ((0xff << 56)
                    | (info_byte << 48) 
                    | (self._packsigned(ball_dir) << 40)
                    | (min(ball_dist, 127) << 32)
                    | (self._packsigned(wall_dir) << 24)
                    | (min(wall_dist, 127) << 16)
                    | (self._packsigned(goal_dir) << 8)
                    | (min(goal_dist, 127))
                   )

        self._write(packet.to_bytes(8, "big"))