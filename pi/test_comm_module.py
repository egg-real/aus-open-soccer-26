import lib.comm_module
import digitalio
import board

comm_module = lib.comm_module.CommModule(digitalio.DigitalInOut(board.D18))

while True:
    print(comm_module.read())