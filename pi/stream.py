#!/usr/bin/env python3
"""Stream live debug JPEGs from the Maix cameras over HTTP."""

import argparse
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from lib.camera import Cameras


BOUNDARY = "frame"


class CameraStreamServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def parse_args():
    parser = argparse.ArgumentParser(
        description="Serve live debug JPEG streams from every Maix camera."
    )
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="HTTP bind address (default: %(default)s)",
    )
    parser.add_argument(
        "-p",
        "--port",
        type=int,
        default=8000,
        help="HTTP port (default: %(default)s)",
    )
    parser.add_argument(
        "--ports",
        nargs="+",
        help="camera UART ports in camera index order (default: ttyAMA0-ttyAMA3)",
    )
    parser.add_argument(
        "-q",
        "--quality",
        type=int,
        default=60,
        choices=range(1, 101),
        metavar="1-100",
        help="debug JPEG quality sent to the cameras (default: %(default)s)",
    )
    return parser.parse_args()


def make_handler(cams, stop_event):
    class StreamHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/":
                self._serve_index()
                return

            parts = [part for part in path.split("/") if part]
            if len(parts) == 2 and parts[0] == "stream" and parts[1].endswith(".mjpg"):
                self._serve_stream(parts[1][:-5])
                return
            if len(parts) == 2 and parts[0] == "latest" and parts[1].endswith(".jpg"):
                self._serve_latest(parts[1][:-4])
                return

            self.send_error(HTTPStatus.NOT_FOUND, "Not found")

        def _camera_index(self, value):
            try:
                cam_index = int(value)
            except ValueError:
                self.send_error(HTTPStatus.BAD_REQUEST, "Camera index must be an integer")
                return None

            if cam_index < 0 or cam_index >= cams.camera_count:
                self.send_error(HTTPStatus.NOT_FOUND, "Camera index out of range")
                return None
            return cam_index

        def _serve_index(self):
            streams = "\n".join(
                f"""
                <section>
                  <h2>Camera {i}</h2>
                  <p>
                    <a href="/stream/{i}.mjpg">MJPEG stream</a>
                    · <a href="/latest/{i}.jpg">latest JPEG</a>
                  </p>
                  <img src="/stream/{i}.mjpg" alt="Camera {i} stream">
                </section>
                """
                for i in range(cams.camera_count)
            )
            body = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Camera Debug Streams</title>
  <style>
    body {{ font-family: sans-serif; margin: 2rem; background: #111; color: #eee; }}
    section {{ margin-bottom: 2rem; }}
    img {{ max-width: 100%; border: 1px solid #555; background: #222; }}
    a {{ color: #8cc8ff; }}
  </style>
</head>
<body>
  <h1>Camera Debug Streams</h1>
  <p>Cameras are in debug JPEG mode. Images appear once a frame arrives over UART.</p>
  {streams}
</body>
</html>
"""
            data = body.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _serve_latest(self, cam_value):
            cam_index = self._camera_index(cam_value)
            if cam_index is None:
                return

            frame = cams.get_frame(cam_index)
            if frame is None:
                status = cams.get_frame_status(cam_index)
                message = (
                    "No frame received from this camera yet.\n"
                    f"State: {status['state']}\n"
                )
                if status["last_error"]:
                    message += f"Last error: {status['last_error']}\n"
                self.send_response(HTTPStatus.SERVICE_UNAVAILABLE)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Retry-After", "1")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(message.encode("utf-8"))
                return

            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(frame)

        def _serve_stream(self, cam_value):
            cam_index = self._camera_index(cam_value)
            if cam_index is None:
                return

            self.send_response(HTTPStatus.OK)
            self.send_header(
                "Content-Type",
                f"multipart/x-mixed-replace; boundary={BOUNDARY}",
            )
            self.send_header("Cache-Control", "no-store")
            self.end_headers()

            last_frame = None
            while not stop_event.is_set():
                frame = cams.get_frame(cam_index)
                if frame is None or frame == last_frame:
                    time.sleep(0.05)
                    continue

                try:
                    self.wfile.write(f"--{BOUNDARY}\r\n".encode("ascii"))
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    break

                last_frame = frame

        def log_message(self, fmt, *args):
            print(f"{self.address_string()} - {fmt % args}")

    return StreamHandler


def main():
    args = parse_args()
    cams = Cameras(ports=args.ports)
    stop_event = threading.Event()
    server = CameraStreamServer(
        (args.host, args.port),
        make_handler(cams, stop_event),
    )

    try:
        cams.start_debug_stream(quality=args.quality)
        print(f"Streaming {cams.camera_count} camera(s) at http://{args.host}:{args.port}/")
        print(f"Debug JPEG quality: {args.quality}")
        print("Press Ctrl-C to stop and return cameras to idle.")
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping debug stream...")
    finally:
        stop_event.set()
        server.server_close()
        cams.deinit()


if __name__ == "__main__":
    main()
