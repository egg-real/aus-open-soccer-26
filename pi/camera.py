import threading
import queue
import atexit
import serial
import time
from pathlib import Path


# ---- Pi -> Maix control protocol ----
# Sent to a camera as [CMD_FRAME_MARKER, command]. Debug includes a quality byte:
# [CMD_FRAME_MARKER, CMD_DEBUG, quality].
# Keep these values in sync with maix/camera.py.
CMD_FRAME_MARKER = 0xAA
CMD_STOP = 0x00     # stop streaming, go idle
CMD_DETECT = 0x01   # stream detection packets (the existing behaviour)
CMD_DEBUG = 0x02    # broadcast JPEG frames for web streaming/debugging
CMD_TRAINING = 0x03 # broadcast full-quality JPEG frames for training capture

# ---- Maix -> Pi image framing ----
# An image frame is: IMG_MAGIC + 4-byte big-endian length + JPEG payload.
# Keep in sync with maix/camera.py.
IMG_MAGIC = b"\xab\xcd\xef\x01"
JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"
MAX_IMAGE_BYTES = 1_000_000
UART_BAUDRATE = 115200

# Local mode tracking so the listener threads know how to parse the stream.
MODE_DETECT = CMD_DETECT
MODE_DEBUG = CMD_DEBUG
MODE_TRAINING = CMD_TRAINING
MODE_STOPPED = CMD_STOP

DEFAULT_CAMERA_PORTS = (
    "/dev/ttyAMA0",
    "/dev/ttyAMA1",
    "/dev/ttyAMA2",
    "/dev/ttyAMA3",
)


class Cameras():
    def __init__(self, ports=None, naive=False):
        """
        Ports should be defined where cameras NESW correspond to indexes 0123

        `naive` mode runs on 1 camera that only feeds the direction of the ball.
        """

        self.prev_ball_dir = 0
        ports = list(DEFAULT_CAMERA_PORTS if ports is None else ports)

        self._threads = []
        self._naive = naive
        self.running = True

        self.ball_dir = None
        self.ball_dist = None
        self.yellow_goal_dir = None
        self.yellow_goal_dist = None
        self.blue_goal_dir = None
        self.blue_goal_dist = None
        self.lines = []

        self._lock = threading.Lock()
        self._data = [None] * len(ports)

        # JPEG frames received per camera while in DEBUG/TRAINING mode.
        self._frames = [None] * len(ports)
        self._frame_status = [
            {
                "state": "idle",
                "last_error": None,
                "frames": 0,
                "last_size": 0,
            }
            for _ in ports
        ]
        # Open serial handles, so we can also write commands back to the cameras.
        self._serials = [None] * len(ports)
        # Desired streaming mode per camera. Cameras start idle and only stream
        # after the pi explicitly asks for a mode.
        self._modes = [MODE_STOPPED] * len(ports)
        self._mode_payloads = [b""] * len(ports)
        # Where TRAINING mode saves JPEGs received from each camera.
        self._training_dir = Path("training_images")
        self._deinited = False

        for i, port in enumerate(ports):
            # i: N = 0, E = 1, S = 2, W = 3
            thread = threading.Thread(target=self._listen_port, args=(port, i), daemon=True)
            thread.start()
            self._threads.append(thread)

        atexit.register(self.deinit)

    # ----- Commands to the cameras ----- #

    def send_command(self, command, cam_index=None, payload=b""):
        """Send a control command to one camera (cam_index) or all of them."""
        frame = bytes([CMD_FRAME_MARKER, command]) + payload
        targets = range(len(self._serials)) if cam_index is None else [cam_index]
        for i in targets:
            self._modes[i] = command
            self._mode_payloads[i] = payload
            if command == MODE_DEBUG or command == MODE_TRAINING:
                with self._lock:
                    self._frames[i] = None
                    self._frame_status[i] = {
                        "state": "waiting for image frame",
                        "last_error": None,
                        "frames": 0,
                        "last_size": 0,
                    }
            port = self._serials[i]
            if port is not None and port.is_open:
                port.write(frame)

    def start_streaming(self, cam_index=None):
        """Tell the camera(s) to stream detection packets."""
        self.send_command(CMD_DETECT, cam_index)

    def stop_streaming(self, cam_index=None):
        """Tell the camera(s) to stop streaming and go idle."""
        self.send_command(CMD_STOP, cam_index)

    def start_debug_stream(self, cam_index=None, quality=60):
        """Tell the camera(s) to broadcast JPEG frames for web streaming/debugging."""
        quality = min(max(int(quality), 1), 100)
        self.send_command(CMD_DEBUG, cam_index, bytes([quality]))

    def start_raw_stream(self, cam_index=None, quality=60):
        """Backward-compatible alias for start_debug_stream()."""
        self.start_debug_stream(cam_index, quality)

    def start_training_capture(self, save_dir="training_images", cam_index=None):
        """Tell the camera(s) to broadcast and save full-quality training frames."""
        self._training_dir = Path(save_dir)
        self._training_dir.mkdir(parents=True, exist_ok=True)
        targets = range(len(self._serials)) if cam_index is None else [cam_index]
        for i in targets:
            (self._training_dir / f"cam{i}").mkdir(parents=True, exist_ok=True)
        self.send_command(CMD_TRAINING, cam_index)

    def deinit(self):
        """Stop all camera streams and close serial ports."""
        if getattr(self, "_deinited", True):
            return
        self._deinited = True

        self.stop_streaming()
        self.running = False

        for port in self._serials:
            if port is None:
                continue
            try:
                if port.is_open:
                    port.flush()
                    port.close()
            except serial.SerialException:
                pass

    def __del__(self):
        self.deinit()

    def get_frame(self, cam_index=0):
        """Return the most recent JPEG frame (bytes) from a camera, or None."""
        with self._lock:
            return self._frames[cam_index]

    def get_frame_status(self, cam_index=0):
        """Return image-stream receive status for debugging."""
        with self._lock:
            return self._frame_status[cam_index].copy()

    @property
    def camera_count(self):
        """Number of configured camera ports."""
        return len(self._serials)

    def get_ball_dir(self):
        return self.ball_dir

    def get_ball_dist(self):
        return self.ball_dist

    def get_yellow_goal_dir(self):
        return self.yellow_goal_dir
    def get_yellow_goal_dist(self):
        return self.yellow_goal_dist

    def get_blue_goal_dir(self):
        return self.blue_goal_dir

    def get_blue_goal_dist(self):
        return self.blue_goal_dist

    def get_lines(self):
        return self.lines

    @staticmethod
    def _unpacksigned(byte:int):
        return ((byte & 0x80 > 0) * 2 - 1) * (byte & 0x7f)

    def _listen_port(self, port_name:str, cam_index:int):
        print(f"Opening port {port_name}")
        port = serial.Serial(port_name, baudrate=UART_BAUDRATE, timeout=0.1)
        self._serials[cam_index] = port

        while not port.is_open:
            continue
        self.send_command(self._modes[cam_index], cam_index, self._mode_payloads[cam_index])
        if self._naive:
            while self.running:
                res = port.read(1)
                if not res:
                    continue
                with self._lock:
                    self._data[cam_index] = res[0]
            return

        body = bytearray()
        synced = False
        while self.running:
            mode = self._modes[cam_index]
            if mode == MODE_STOPPED:
                synced = False
                body.clear()
                time.sleep(0.01)
                continue

            # In DEBUG/TRAINING mode the camera sends framed JPEG images instead of
            # the 0xff-delimited detection packets.
            if mode == MODE_DEBUG or mode == MODE_TRAINING:
                synced = False
                body.clear()
                frame = self._read_image_frame(port, cam_index)
                if frame is not None:
                    with self._lock:
                        self._frames[cam_index] = frame
                        self._frame_status[cam_index]["state"] = "received JPEG frame"
                        self._frame_status[cam_index]["frames"] += 1
                        self._frame_status[cam_index]["last_size"] = len(frame)
                    if self._modes[cam_index] == MODE_TRAINING:
                        self._save_training_frame(cam_index, frame)
                continue

            data = port.read(1)
            if not data:
                continue
            byte = data[0]
            if not synced:
                # Re-sync to a packet boundary (also used after leaving image modes).
                if byte == 0xff:
                    synced = True
                continue
            if byte == 0xff:
                if len(body) > 0:
                    # print(bytes(body))
                    with self._lock:
                        self._data[cam_index] = bytes(body)
                    body.clear()
                continue
            body.append(byte)

    def _read_image_frame(self, port, cam_index):
        """Read one IMG_MAGIC-framed JPEG payload from the port.

        Returns the JPEG bytes, or None if the mode changed / framing broke.
        """
        matched = 0
        while self.running and (self._modes[cam_index] == MODE_DEBUG or self._modes[cam_index] == MODE_TRAINING):
            data = port.read(1)
            if not data:
                self._set_frame_state(cam_index, "waiting for image magic")
                return None
            byte = data[0]
            if byte == IMG_MAGIC[matched]:
                matched += 1
                if matched == len(IMG_MAGIC):
                    break
            else:
                # Restart matching, allowing this byte to begin a new magic.
                matched = 1 if byte == IMG_MAGIC[0] else 0

        if matched != len(IMG_MAGIC):
            return None

        length_bytes = port.read(4)
        if len(length_bytes) < 4:
            self._set_frame_error(
                cam_index,
                f"incomplete image length header: {len(length_bytes)}/4 byte(s)",
            )
            return None
        length = int.from_bytes(length_bytes, "big")
        if length <= 0 or length > MAX_IMAGE_BYTES:
            self._set_frame_error(cam_index, f"invalid image length: {length} byte(s)")
            return None

        data = bytearray()
        payload_deadline = time.monotonic() + self._image_read_timeout(length)
        while len(data) < length and self.running:
            chunk = port.read(min(4096, length - len(data)))
            if not chunk:
                if time.monotonic() >= payload_deadline:
                    break
                self._set_frame_state(
                    cam_index,
                    f"receiving image payload: {len(data)}/{length} byte(s)",
                )
                continue
            data.extend(chunk)

        if len(data) != length:
            self._set_frame_error(
                cam_index,
                f"incomplete image payload: {len(data)}/{length} byte(s)",
            )
            return None
        frame = bytes(data)
        frame = self._normalize_jpeg_frame(frame)
        if frame is None:
            self._set_frame_error(
                cam_index,
                (
                    f"invalid JPEG payload: length={len(data)}, "
                    f"start={bytes(data[:2]).hex()}, end={bytes(data[-2:]).hex()}"
                ),
            )
            return None
        return frame

    @staticmethod
    def _normalize_jpeg_frame(frame):
        if not frame.startswith(JPEG_SOI):
            return None
        end = frame.find(JPEG_EOI)
        if end < 0:
            return None
        return frame[:end + len(JPEG_EOI)]

    @staticmethod
    def _image_read_timeout(length):
        # 8N1 UART sends roughly 10 line bits per byte; add margin for scheduling.
        return max(1.0, (length * 10 / UART_BAUDRATE) + 1.0)

    def _set_frame_state(self, cam_index, state):
        with self._lock:
            self._frame_status[cam_index]["state"] = state
            if state.startswith("receiving image payload"):
                self._frame_status[cam_index]["last_error"] = None

    def _set_frame_error(self, cam_index, error):
        with self._lock:
            self._frame_status[cam_index]["state"] = "rejected image frame"
            self._frame_status[cam_index]["last_error"] = error

    def _save_training_frame(self, cam_index, frame):
        camera_dir = self._training_dir / f"cam{cam_index}"
        camera_dir.mkdir(parents=True, exist_ok=True)
        filename = camera_dir / f"cam{cam_index}_{time.time_ns()}.jpg"
        with open(filename, "wb") as f:
            f.write(frame)

    @staticmethod
    def _process_block(block):
        """
        Returns variables processed from a block of data
        
        ---

        see_ball
            bool: can the ball be seen
        ball_dir
            int: angle of the ball relative to centre of the camera
        ball_dist
            int: approx distance to ball in cm

        see_goal
            bool: can either goal be seen
        goal_dir
            int: angle to th centre of the goal relative to centre of the camera
        goal_dist
            int: approx distance to the goal in cm
        goal_yellow
            bool: if the goal is yellow or not (False = blue)

        wall_dir
            int: angle between tangent of goal to centre of the camera
        wall_dist
            int: approx distance to the goal in cm
        
        cam_ok
            bool: if the camera is running ok (False may suggest some camera error that needs to be addressed)
        """
        cam_ok = block[0] & 0x01 > 0
        see_yellow = block[0] & 0x02 > 0
        see_goal = block[0] & 0x04 > 0
        see_ball = block[0] & 0x08 > 0

        ball_dir = Cameras._unpacksigned(block[1])
        ball_dist = block[2]

        wall_dir = Cameras._unpacksigned(block[3])
        wall_dist = block[4]

        goal_dir = Cameras._unpacksigned(block[5])
        goal_dist = block[6]

        lines = []
        for i in range(7, len(block) - 1, 2):
            if block[i] == 254:
                lines.append((254, Cameras._unpacksigned(block[i + 1])))
                continue

            lines.append((
                Cameras._unpacksigned(block[i]),
                Cameras._unpacksigned(block[i + 1]),
            ))

        filtered_lines = []
        min_separation = 10

        for new_line in lines:
            add_line = True
            for existing_line in filtered_lines:
                if abs(new_line[0] - existing_line[0]) < min_separation and abs(new_line[1] - existing_line[1]) < min_separation:
                    add_line = False
                    break
            if add_line:
                filtered_lines.append(new_line)

        lines = filtered_lines

        return see_ball, see_goal, see_yellow, cam_ok, ball_dir, ball_dist, wall_dir, wall_dist, goal_dir, goal_dist, lines

    def process(self):

        # Process new data in queue
        data = []
        with self._lock:
            data = self._data.copy()
        # print(data)
        # Naive:
        if self._naive:
            ball_dir = self._unpacksigned(data[0])
            return ball_dir

        ball_spotted = False
        yellow_goal_spotted = False
        blue_goal_spotted = False
        lines = []

        for i in range(len(data)):
            if data[i] is None or len(data[i]) < 7:
                continue
            block = data[i]
            see_ball, see_goal, see_yellow, cam_ok, ball_dir, ball_dist, wall_dir, wall_dist, goal_dir, goal_dist, block_lines = self._process_block(block)
            if not cam_ok:
                print(f"CAMERA {i} NOT OK")
                continue
            lines.extend(block_lines)
            if see_ball:
                self.ball_dir = ball_dir + i * 90
                self.ball_dist = ball_dist
                ball_spotted = True
            if see_goal:
                if see_yellow:
                    yellow_goal_spotted = True
                    self.yellow_goal_dir = goal_dir + i * 90
                    self.yellow_goal_dist = goal_dist
                else:
                    blue_goal_spotted = True
                    self.blue_goal_dir = goal_dir + i * 90
                    self.blue_goal_dist = goal_dist

        if not ball_spotted:
            self.ball_dir = None
            self.ball_dist = None
        if not yellow_goal_spotted:
            self.yellow_goal_dir = None
            self.yellow_goal_dist = None
        if not blue_goal_spotted:
            self.blue_goal_dir = None
            self.blue_goal_dist = None
        self.lines = lines
