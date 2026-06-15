from maix import video, time, image, camera, display, app
import os

width = 1280
height = 720
cam = camera.Camera(width, height, image.Format.FMT_YVU420SP)
disp = display.Display()

i = 0

# here
base_path = "/root/video_output/output_{}.mp4"
index = 0
while True:
    file_path = base_path.format(index)
    if not os.path.exists(file_path):
        break
    index += 1

e = video.Encoder(file_path, width, height)

record_ms = 10 * 1000
start_ms = time.ticks_ms()
while not app.need_exit():
    img = cam.read()
    e.encode(img)
    # disp.show(img)

    if time.ticks_ms() - start_ms > record_ms:
        app.set_exit_flag(True)