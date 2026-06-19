from maix import camera as _camera, uart as _uart, pinmap as _pinmap, image as _image, display as _display, time as _time
from math import sin, cos, pi, copysign


# ---- Pi -> Maix control protocol ----
# The pi sends a 2-byte command frame: [CMD_FRAME_MARKER, command].
# Keep these values in sync with pi/camera.py.
CMD_FRAME_MARKER = 0xAA
CMD_STOP = 0x00     # stop streaming, go idle
CMD_DETECT = 0x01   # stream detection packets (the existing behaviour)
CMD_DEBUG = 0x02    # broadcast JPEG frames for web streaming/debugging
CMD_TRAINING = 0x03 # broadcast full-quality JPEG frames for training capture

# ---- Maix -> Pi image framing ----
# An image frame is: IMG_MAGIC + 4-byte big-endian length + JPEG payload.
# This magic is chosen so it doesn't clash with the 0xff-delimited
# detection packets. Keep in sync with pi/camera.py.
IMG_MAGIC = b"\xab\xcd\xef\x01"


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

        # Holds bytes received from the pi that don't yet form a full command.
        self._cmd_buf = bytearray()

        print("\n\n!!!PORT!!!: " + self.uart.get_port() + "\n\n")
    
    def _write(self, msg):
        if not self.uart.is_open():
            return False
        # print("[debug] wrote " + bin(int.from_bytes(msg, "big")))
        self.uart.write(msg)    
        _time.sleep_ms(5)
        
        return True

    def read_command(self):
        """Poll the UART for a control command sent by the pi.

        Non-blocking: reads whatever is in the receive buffer, parses any
        ``[CMD_FRAME_MARKER, command]`` frames it finds, and returns the most
        recent command byte. Returns ``None`` if no complete command arrived.
        """
        if not self.uart.is_open():
            return None

        data = self.uart.read()  # read(-1, 0): return immediately with buffer
        if data:
            self._cmd_buf.extend(data)

        command = None
        while True:
            marker = self._cmd_buf.find(CMD_FRAME_MARKER)
            if marker == -1:
                # No marker at all, nothing worth keeping.
                self._cmd_buf.clear()
                break
            if marker + 1 >= len(self._cmd_buf):
                # Marker is the last byte; keep it and wait for the command byte.
                del self._cmd_buf[:marker]
                break
            command = self._cmd_buf[marker + 1]
            del self._cmd_buf[:marker + 2]

        return command

    def send_image(self, jpeg_bytes):
        """Send a single JPEG frame to the pi."""
        if not self.uart.is_open():
            return False

        header = IMG_MAGIC + len(jpeg_bytes).to_bytes(4, "big")
        self.uart.write(header)
        self.uart.write(jpeg_bytes)
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