"""Small socket-based robot communication helper."""

import json
import socket
import threading
import time
from typing import Any, Callable


DEFAULT_BROKER = "localhost"
DEFAULT_PORT = 8765
DEFAULT_TOPIC = "soccer/pi/messages"
DEFAULT_REPLY_TOPIC = "soccer/pi/replies"

MessageCallback = Callable[[dict[str, Any], str], None]
ReplyCallback = Callable[[dict[str, Any], str], None]


class Communication:
    def __init__(
        self,
        host: bool | str = True,
        hostname: str | None = None,
        port: int = DEFAULT_PORT,
        topic: str = DEFAULT_TOPIC,
        reply_topic: str = DEFAULT_REPLY_TOPIC,
        client_id: str | None = None,
        broker: str | None = None,
        **_ignored: Any,
    ) -> None:
        self.host = host
        self.hostname = hostname or broker or DEFAULT_BROKER
        self.port = port
        self.topic = topic
        self.reply_topic = reply_topic
        self.client_id = client_id or f"pi-{socket.gethostname()}"

        self.on_message: MessageCallback | None = None
        self.on_reply: ReplyCallback | None = None

        self._stop_event = threading.Event()
        self._connected_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None
        self._listener: socket.socket | None = None
        self._send_lock = threading.Lock()

    @staticmethod
    def decode_payload(payload: bytes) -> dict[str, Any]:
        text = payload.decode("utf-8", errors="replace")
        try:
            decoded = json.loads(text)
        except json.JSONDecodeError:
            return {"message": text}

        if isinstance(decoded, dict):
            return decoded
        return {"message": decoded}

    def build_payload(self, message: Any, reply_topic: str | None = None, kind: str = "message") -> dict[str, Any]:
        return {
            "kind": kind,
            "sender": self.client_id,
            "message": message,
            "reply_topic": reply_topic or self.reply_topic,
            "sent_at": time.time(),
        }

    def _encode(self, payload: dict[str, Any]) -> bytes:
        return json.dumps(payload, separators=(",", ":")).encode("utf-8") + b"\n"

    def _close_socket(self, sock: socket.socket | None) -> None:
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def _send_payload(self, payload: dict[str, Any]) -> bool:
        sock = self._socket
        if sock is None:
            return False

        data = self._encode(payload)
        with self._send_lock:
            try:
                sock.sendall(data)
            except OSError:
                return False
        return True

    def _handle_packet(self, payload: dict[str, Any]) -> None:
        topic = payload.get("reply_topic") or payload.get("topic") or self.topic
        if payload.get("kind") == "reply":
            if self.on_reply is not None:
                self.on_reply(payload, topic)
            return

        if self.on_message is not None:
            self.on_message(payload, topic)

    def _recv_loop(self, sock: socket.socket) -> None:
        buffer = bytearray()
        sock.settimeout(0.5)

        while not self._stop_event.is_set():
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break

            if not chunk:
                break

            buffer.extend(chunk)
            while True:
                newline = buffer.find(b"\n")
                if newline < 0:
                    break

                line = bytes(buffer[:newline])
                del buffer[:newline + 1]
                if not line:
                    continue

                self._handle_packet(self.decode_payload(line))

    def _serve(self) -> None:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener = listener
        try:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(("0.0.0.0", self.port))
            listener.listen(1)
            listener.settimeout(0.5)
            print(f"Listening on 0.0.0.0:{self.port}")

            while not self._stop_event.is_set():
                try:
                    conn, address = listener.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break

                print(f"Connected from {address[0]}:{address[1]}")
                self._socket = conn
                self._connected_event.set()
                try:
                    self._recv_loop(conn)
                finally:
                    self._connected_event.clear()
                    self._close_socket(conn)
                    if self._socket is conn:
                        self._socket = None
        finally:
            self._close_socket(listener)
            self._listener = None

    def _connect_once(self, hostname: str) -> socket.socket | None:
        try:
            address_info = socket.getaddrinfo(hostname, self.port, socket.AF_UNSPEC, socket.SOCK_STREAM)
        except OSError:
            return None

        for family, socktype, proto, _, address in address_info:
            sock = None
            try:
                sock = socket.socket(family, socktype, proto)
                sock.settimeout(1.0)
                sock.connect(address)
                sock.settimeout(None)
                return sock
            except OSError:
                self._close_socket(sock)
                continue

        return None

    def _search_and_connect(self) -> None:
        hostname = self.hostname
        print(f"Searching for {hostname}:{self.port}")

        while not self._stop_event.is_set():
            sock = self._connect_once(hostname)
            if sock is None:
                time.sleep(1.0)
                continue

            print(f"Connected to {hostname}:{self.port}")
            self._socket = sock
            self._connected_event.set()
            try:
                self._recv_loop(sock)
            finally:
                self._connected_event.clear()
                self._close_socket(sock)
                if self._socket is sock:
                    self._socket = None

            if not self._stop_event.is_set():
                time.sleep(1.0)

    def connect(self) -> None:
        if self._thread is None:
            self.start()
        self._connected_event.wait(timeout=5.0)

    def loop_forever(self) -> None:
        self.start()
        try:
            while not self._stop_event.is_set():
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.stop()

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return

        self._stop_event.clear()
        self._connected_event.clear()

        if self.host is True:
            target = self._serve
        else:
            target = self._search_and_connect

        self._thread = threading.Thread(target=target, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._connected_event.clear()
        self._close_socket(self._socket)
        self._close_socket(self._listener)
        self._socket = None
        self._listener = None

        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)

    def send(self, message: Any, topic: str | None = None, reply_topic: str | None = None) -> bool:
        payload = self.build_payload(message, reply_topic, kind="message")
        if topic is not None:
            payload["topic"] = topic
        return self._send_payload(payload)

    def send_and_wait(self, message: Any, topic: str | None = None, reply_topic: str | None = None) -> bool:
        self._connected_event.wait(timeout=5.0)
        return self.send(message, topic, reply_topic)

    def reply(self, message: Any, topic: str | None = None) -> bool:
        payload = self.build_payload(message, topic, kind="reply")
        if topic is not None:
            payload["topic"] = topic
        return self._send_payload(payload)