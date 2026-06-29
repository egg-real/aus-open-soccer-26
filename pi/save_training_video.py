#!/usr/bin/env python3
"""Save a continuous training capture as JPEG frames.

The camera protocol sends individual JPEG images, not a video container. This
script keeps every camera in training mode and writes each received frame into
per-camera folders inside the output folder.
"""

import argparse
import time
from pathlib import Path

from lib.camera import Cameras


def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture full-quality training JPEGs from every Maix camera."
    )
    parser.add_argument(
        "-o",
        "--output",
        default="training_images",
        help="folder to save per-camera JPEG frame folders into (default: %(default)s)",
    )
    parser.add_argument(
        "--ports",
        nargs="+",
        help="camera UART ports in camera index order (default: ttyAMA0-ttyAMA3)",
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        help="optional capture duration in seconds; omit to run until Ctrl-C",
    )
    parser.add_argument(
        "--status-interval",
        type=float,
        default=5.0,
        help="seconds between saved-frame count updates (default: %(default)s)",
    )
    args = parser.parse_args()

    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be greater than zero")
    if args.status_interval <= 0:
        parser.error("--status-interval must be greater than zero")

    return args


def count_saved_frames(output_dir):
    return sum(1 for path in output_dir.rglob("*.jpg") if path.is_file())


def main():
    args = parse_args()
    output_dir = Path(args.output)
    cams = Cameras(ports=args.ports)

    try:
        cams.start_training_capture(save_dir=output_dir)
        print(
            f"Saving training JPEG frames from {cams.camera_count} camera(s) "
            f"to {output_dir.resolve()}"
        )
        print("Press Ctrl-C to stop and return cameras to idle.")

        start = time.monotonic()
        next_status = start
        while True:
            now = time.monotonic()
            if args.duration is not None and now - start >= args.duration:
                break
            if now >= next_status:
                print(f"Saved {count_saved_frames(output_dir)} JPEG frame(s)")
                next_status = now + args.status_interval
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping training capture...")
    finally:
        cams.deinit()


if __name__ == "__main__":
    main()
