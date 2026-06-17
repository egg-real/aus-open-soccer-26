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

        self._lock = threading.Lock()
        self._data = [0] * len(ports)

        for i, port in enumerate(ports):
            # i: N = 0, E = 1, S = 2, W = 3
            thread = threading.Thread(target=self._listen_port, args=(port, i), daemon=True)
            thread.start()
            self._threads.append(thread)

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

        while self.running:
            res = port.read(8)
            body = res[1:]
            if res[0] != 0xff:
                # read must have shifted
                # find start flag block in rest of res
                for i, block in enumerate(res[1:], 1):
                    if block == 0xff:
                        # read off rest
                        rest = port.read(i)
                        body = res[i:] + rest
                        break

            with self._lock:
                self._data[cam_index] = body

    @staticmethod
    def _proccess_block(block):
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

        see_ball = block[0] & 0x01 > 0
        see_goal = block[0] & 0x02 > 0
        goal_yellow = block[0] & 0x03 > 0
        cam_ok = block[0] & 0x04 > 0

        ball_dir = Cameras._unpacksigned(block[1])
        ball_dist = block[2]

        wall_dir = Cameras._unpacksigned(block[3])
        wall_dist = block[4]

        goal_dir = Cameras._unpacksigned(block[5])
        goal_dist = block[6]

        return (see_ball, ball_dir, ball_dist,
                see_goal, goal_dir, goal_dist, goal_yellow,
                wall_dir, wall_dist,
                cam_ok)
    
    def process(self):
            
        # Process new data in queue
        data = []
        with self._lock:
            data = self._data[:]
        
        # Naive:
        if self._naive:
            ball_dir = self._unpacksigned(data[0])
            return ball_dir
        
        # Take all data
        ball_locations = []
        ball_dists = []

        ygoal_dirs = []
        ygoal_dists = []

        bgoal_dirs = []
        bgoal_dists = []

        wall_dirs = []
        wall_dists = []

        for i, block in enumerate(data):
            see_ball, ball_dir, ball_dist,\
                see_goal, goal_dir, goal_dist, goal_yellow,\
                wall_dir, wall_dist, cam_ok = self._proccess_block(block)
            if not cam_ok:
                print(f"[WARNING] maix{"nesw"[i]} not ok")
            if see_ball:
                ball_locations.append((ball_dir + 90*i) % 360)
                ball_dists.append(ball_dist)
            if see_goal:
                if goal_yellow:
                    ygoal_dirs.append((goal_dir + 90*i) % 360)
                    ygoal_dists.append(goal_dist)
                else:
                    bgoal_dirs.append((goal_dir + 90*i) % 360)
                    bgoal_dists.append(goal_dist)

        # Pick the best data
        