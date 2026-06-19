import serial

from camera import CMD_DETECT, CMD_FRAME_MARKER, CMD_STOP


PORT = "/dev/ttyAMA0"
BAUDRATE = 115200


def send_command(uart, command):
    uart.write(bytes([CMD_FRAME_MARKER, command]))


def main():
    with serial.Serial(PORT, baudrate=BAUDRATE, timeout=0.2) as uart:
        send_command(uart, CMD_DETECT)
        print(f"Reading detection bytes from {PORT}. Press Ctrl-C to stop.")

        try:
            while True:
                data = uart.read(1)
                if not data:
                    continue
                print(f"{data[0]:08b}")
        finally:
            send_command(uart, CMD_STOP)


if __name__ == "__main__":
    main()
