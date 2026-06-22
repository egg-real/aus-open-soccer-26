from maix import camera as _camera, uart as _uart, pinmap as _pinmap, image as _image, display as _display, time as _time
from math import sin, cos, pi, copysign


# ---- Pi -> Maix control protocol ----
# The pi sends [CMD_FRAME_MARKER, command]. Debug adds a quality byte:
# [CMD_FRAME_MARKER, CMD_DEBUG, quality].
# Keep these values in sync with pi/camera.py.
CMD_FRAME_MARKER = 0xAA
CMD_STOP = 0x00     # stop streaming, go idle
CMD_DETECT = 0x01   # stream detection packets (the existing behaviour)
CMD_DEBUG = 0x02    # broadcast JPEG frames for web streaming/debugging
CMD_TRAINING = 0x03 # broadcast full-quality JPEG frames for training capture
UART_BAUDRATE = 115200
UART_IMAGE_CHUNK_BYTES = 512

# ---- Maix -> Pi image framing ----
# An image frame is: IMG_MAGIC + 4-byte big-endian length + JPEG payload.
# This magic is chosen so it doesn't clash with the 0xff-delimited
# detection packets. Keep in sync with pi/camera.py.
IMG_MAGIC = b"\xab\xcd\xef\x01"
JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"


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

        self.uart = _uart.UART(port, UART_BAUDRATE)

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
        command frames it finds, and returns the most recent
        ``(command, payload)`` tuple. Returns ``None`` if no complete command
        arrived.
        """
        if not self.uart.is_open():
            return None

        data = self.uart.read()  # read(-1, 0): return immediately with buffer
        if data:
            self._cmd_buf.extend(data)

        command_frame = None
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
            payload_len = 1 if command == CMD_DEBUG else 0
            frame_len = 2 + payload_len
            if marker + frame_len > len(self._cmd_buf):
                # Keep the complete marker/command prefix until the payload arrives.
                del self._cmd_buf[:marker]
                break
            payload_start = marker + 2
            payload = bytes(self._cmd_buf[payload_start:payload_start + payload_len])
            command_frame = (command, payload)
            del self._cmd_buf[:marker + frame_len]

        return command_frame

    def send_image(self, jpeg_bytes):
        """Send a single JPEG frame to the pi."""
        if not self.uart.is_open():
            return False

        jpeg_bytes = self._normalize_jpeg(jpeg_bytes)
        if jpeg_bytes is None:
            return False

        header = IMG_MAGIC + len(jpeg_bytes).to_bytes(4, "big")
        return self._write_image_bytes(header + jpeg_bytes)

    def _write_image_bytes(self, data):
        for start in range(0, len(data), UART_IMAGE_CHUNK_BYTES):
            chunk = data[start:start + UART_IMAGE_CHUNK_BYTES]
            self.uart.write(chunk)
            # 8N1 UART uses about 10 line bits per byte. Pace each chunk so
            # large JPEGs do not overflow small serial transmit buffers.
            chunk_time_ms = ((len(chunk) * 10 * 1000) // UART_BAUDRATE) + 1
            _time.sleep_ms(max(1, int(chunk_time_ms)))
        return True

    @staticmethod
    def _normalize_jpeg(jpeg_bytes):
        if not jpeg_bytes.startswith(JPEG_SOI):
            return None
        end = jpeg_bytes.find(JPEG_EOI)
        if end < 0:
            return None
        return jpeg_bytes[:end + len(JPEG_EOI)]
    
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