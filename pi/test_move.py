from lib.drive import Drive
from time import sleep

d = Drive()

d.move(0, 0.1)
sleep(5)
d.move(0, 0)