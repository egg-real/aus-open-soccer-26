from lib.drive import Drive
from time import sleep

d = Drive()

while True:
    direction = 0
    speed = 0.1
    d.move(0, 0.1)