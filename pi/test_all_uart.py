import time

import serial

from lib.camera import CMD_DETECT, CMD_FRAME_MARKER, CMD_STOP, DEFAULT_CAMERA_PORTS


PORTS = list(DEFAULT_CAMERA_PORTS)
BAUDRATE = 115200
READ_TIMEOUT = 0.2
CHECK_SECONDS = 5
BLOCK_LENGTH = 7
FRAME_MARKER = 0xFF


def send_command(port, command):
    port.write(bytes([CMD_FRAME_MARKER, command]))


def read_camera_block(port):
    body = bytearray()

    while True:
        data = port.read(1)
        if not data:
            return None

        byte = data[0]
        if byte != FRAME_MARKER:
            continue

        break

    while True:
        data = port.read(1)
        if not data:
            return None

        byte = data[0]
        if byte == FRAME_MARKER:
            if body:
                return bytes(body)
            continue

        body.append(byte)
        if len(body) == BLOCK_LENGTH:
            return bytes(body)


def check_port(port_name):
    deadline = time.monotonic() + CHECK_SECONDS

    try:
        with serial.Serial(port_name, baudrate=BAUDRATE, timeout=READ_TIMEOUT) as port:
            send_command(port, CMD_DETECT)
            try:
                while time.monotonic() < deadline:
                    block = read_camera_block(port)
                    if block is None:
                        continue

                    if len(block) != BLOCK_LENGTH:
                        continue

                    cam_ok = block[0] & 0x01 > 0
                    return cam_ok, block
            finally:
                send_command(port, CMD_STOP)
    except serial.SerialException as error:
        return False, error

    return False, None


def main():
    all_ok = True

    for index, port_name in enumerate(PORTS):
        cam_ok, result = check_port(port_name)
        if cam_ok:
            print(f"UART {index} ({port_name}): signal found, cam_ok true")
            continue

        all_ok = False
        if isinstance(result, Exception):
            print(f"UART {index} ({port_name}): failed to open/read: {result}")
        elif result is None:
            print(f"UART {index} ({port_name}): no valid signal found")
        else:
            print(f"UART {index} ({port_name}): signal found, cam_ok false")

    if not all_ok:
        raise SystemExit(1)

    print("All UART cameras have signal and cam_ok true")


if __name__ == "__main__":
    main()
