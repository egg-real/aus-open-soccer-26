from camera import Camera
import time

cam = Camera(720, 480)
running = True

start_time = time.time()

# cam.send_packet(1, -120)
cam.send_packet(0, -127)

time.sleep(2)
if cam.uart.is_open():
    cam.uart.close()
print("exiting...")
time.sleep(3)
