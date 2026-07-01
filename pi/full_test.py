import time
from lib.break_beam import BreakBeam
from lib.dribbler import Dribbler
import test_all_uart
import test_solenoid
from lib.tof import ToF
import board

print("Starting UART test")
test_all_uart.main()

print("Starting ToF test")
tofs = [ToF(0x50), ToF(0x51), ToF(0x52), ToF(0x53)]
for tof in tofs:
    print(tof._address, "reading", tof.read())

print("Starting capturing test.")
test_dribbler = input("Test dribbler? (y/n) ")
if test_dribbler.lower() == "y":
    dribbler = Dribbler()
    dribbler.set_speed(1.0)

start_time = time.monotonic()
break_beam = BreakBeam(board.D17)

while time.monotonic() < start_time + 10:
    print("Break Beam is", break_beam.read())

test_kicker = input("Test kicker? (y/n) ")
if test_kicker.lower() == "y":
    test_solenoid.main()

test_move = input("Test move? (y/n) ")
if test_move.lower() == "y":
    import test_move

# TODO: Test communication