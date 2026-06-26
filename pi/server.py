from lib.drive import Drive
from lib.interface import WSServer
import numpy as np

# Drive

d = Drive()

# Interface
def move_handler(websocket, data):
        x = data.get("x", 0)
        y = data.get("y", 0)
        rot = data.get("rotation", 0)

        move_dir = np.arctan2(x, y) * 180 / np.pi
        move_mag = min(np.hypot(x, y), 0.5)
        move_rot = rot

        d.move(move_dir, move_mag, move_rot)

interface = WSServer()
interface.add_handler("move", move_handler)

interface.run()
while True:
    pass

# Camera

# cap = picamera2.Picamera2()

# cap_cfg = cap.create_video_configuration()
# cap_cfg["raw"]["format"] = "SRGGB10_CSI2P"
# cap_cfg["raw"]["size"] = (2304, 1296)

# cap_cfg["main"]["format"] = "RGB888"
# cap_cfg["main"]["size"] = (640, 640)    
# print(cap_cfg)
# cap.configure(cap_cfg)
# cap.start()

# cap.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 200})

# while True:
#     frame = cap.capture_array("main")
#     frame = cv2.flip(frame, 1)

#     hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

#     orangeMask = cv2.inRange(hsv_frame, (10, 100, 100), (30, 255, 255))
#     ball = cv2.findContours(orangeMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
#     if len(ball) > 0:
#         ball = max(ball, key=cv2.contourArea)
#         (x, y), radius = cv2.minEnclosingCircle(ball)
#         if radius > 5:
#             # cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
#             # cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
#             pass

#     interface.send_frame(frame)
