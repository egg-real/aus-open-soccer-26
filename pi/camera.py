import threading
import queue
import serial

class Cameras():
    def __init__(self, ports=[
        "/dev/ttyAMA0",
        "/dev/ttyAMA1",
        "/dev/ttyAMA2",
        "/dev/ttyAMA3"
        ], naive=False):
        """
        Ports should be defined where cameras NESW correspond to indexes 0123

        `naive` mode runs on 1 camera that only feeds the direction of the ball.
        """

        self.prev_ball_dir = 0

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

        for i, port in enumerate(ports):
            # i: N = 0, E = 1, S = 2, W = 3
            thread = threading.Thread(target=self._listen_port, args=(port, i), daemon=True)
            thread.start()
            self._threads.append(thread)

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
        port = serial.Serial(port_name, baudrate=115200)

        while not port.is_open:
            continue
        if self._naive:
            while self.running:
                res = port.read(1)
                with self._lock:
                    self._data[cam_index] = res[0]
            return

        while port.read(1)[0] != 0xff:
            continue

        body = bytearray()
        while self.running:
            byte = port.read(1)[0]
            if byte == 0xff:
                if len(body) > 0:
                    # print(bytes(body))
                    with self._lock:
                        self._data[cam_index] = bytes(body)
                    body.clear()
                continue
            body.append(byte)

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
