import sys
import time

from lib.drive import Drive, wrap_angle
from lib.tof import ToF


TOF_ADDRESSES = [0x50, 0x51, 0x52, 0x53]
ROTATION_DEG_PER_SECOND = 20.0
PRINT_INTERVAL_SECONDS = 0.05


def parse_addresses(args):
    if not args:
        return TOF_ADDRESSES
    if len(args) != 4:
        raise ValueError("Expected exactly 4 ToF I2C addresses.")
    return [int(address, 0) for address in args]


def format_distance(distance):
    if distance is None:
        return "----"
    return f"{distance:4d}"


if __name__ == "__main__":
    addresses = parse_addresses(sys.argv[1:])
    sensors = [ToF(address=address) for address in addresses]
    drive = Drive()

    target_yaw = 0.0
    last_update = time.monotonic()

    try:
        print(
            "Rotating slowly. ToF addresses: "
            + ", ".join(f"0x{address:02x}" for address in addresses)
        )
        while True:
            now = time.monotonic()
            dt = now - last_update
            last_update = now

            target_yaw = wrap_angle(target_yaw + ROTATION_DEG_PER_SECOND * dt)
            drive.move(0, 0, target_yaw)

            distances = [sensor.read() for sensor in sensors]
            readings = "  ".join(
                f"tof{i + 1}@0x{addresses[i]:02x}: {format_distance(distance)}"
                for i, distance in enumerate(distances)
            )
            print(f"\r{readings}", end="", flush=True)

            time.sleep(PRINT_INTERVAL_SECONDS)
    except KeyboardInterrupt:
        print()
    finally:
        drive.stop()
        for sensor in sensors:
            sensor.close()
