import json
import socket
import threading
import time
from typing import Any, Callable


DEFAULT_BROKER = "255.255.255.255"
DEFAULT_PORT = 1883
DEFAULT_TOPIC = "soccer/pi/messages"
DEFAULT_REPLY_TOPIC = "soccer/pi/replies"
DEFAULT_BIND_HOST = "0.0.0.0"
DEFAULT_HEARTBEAT_INTERVAL_S = 0.25
DEFAULT_PEER_TIMEOUT_S = 1.0
DEFAULT_SOCKET_TIMEOUT_S = 0.02

MessageCallback = Callable[[dict[str, Any], str], None]
ReplyCallback = Callable[[dict[str, Any], str], None]


class Communication:
    def __init__(
        self,
        broker: str = DEFAULT_BROKER,
        port: int = DEFAULT_PORT,
        topic: str = DEFAULT_TOPIC,
        reply_topic: str = DEFAULT_REPLY_TOPIC,
        client_id: str | None = None,
        bind_host: str = DEFAULT_BIND_HOST,
        bind_port: int | None = None,
        peer_port: int | None = None,
        heartbeat_interval_s: float = DEFAULT_HEARTBEAT_INTERVAL_S,
        peer_timeout_s: float = DEFAULT_PEER_TIMEOUT_S,
    ) -> None:
        self.broker = broker
        self.port = port
        self.topic = topic
        self.reply_topic = reply_topic
        self.client_id = client_id or f"pi-{socket.gethostname()}"
        self.bind_host = bind_host
        self.bind_port = bind_port or port
        self.peer_port = peer_port or port
        self.heartbeat_interval_s = heartbeat_interval_s
        self.peer_timeout_s = peer_timeout_s

        self.on_message: MessageCallback | None = None
        self.on_reply: ReplyCallback | None = None

        self.peer_alive = False
        self.last_seen: float | None = None

        self._socket: socket.socket | None = None
        self._peer_addr: tuple[str, int] | None = None
        self._session_id = f"{self.client_id}-{time.time_ns()}"
        self._seq = 0
        self._last_peer_seq: dict[tuple[str, str], int] = {}
        self._running = threading.Event()
        self._lock = threading.Lock()
        self._receive_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None

    def _open_socket(self) -> None:
        if self._socket is not None:
            return

        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        udp_socket.settimeout(DEFAULT_SOCKET_TIMEOUT_S)
        udp_socket.bind((self.bind_host, self.bind_port))
        self._socket = udp_socket

    def _target_addr(self) -> tuple[str, int]:
        with self._lock:
            if self._peer_addr is not None:
                return self._peer_addr
        return (self.broker, self.peer_port)

    def _next_seq(self) -> int:
        with self._lock:
            self._seq += 1
            return self._seq

    def _encode(self, payload: dict[str, Any]) -> bytes:
        return json.dumps(payload, separators=(",", ":")).encode("utf-8")

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

    def build_payload(self, message: Any, reply_topic: str | None = None, topic: str | None = None) -> dict[str, Any]:
        return {
            "sender": self.client_id,
            "session_id": self._session_id,
            "message": message,
            "topic": topic or self.topic,
            "reply_topic": reply_topic or self.reply_topic,
            "seq": self._next_seq(),
            "sent_at": time.time(),
        }

    def connect(self) -> None:
        self._open_socket()

    def start(self) -> None:
        self.connect()
        if self._running.is_set():
            return

        self._running.set()
        self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._receive_thread.start()
        self._heartbeat_thread.start()

    def stop(self) -> None:
        self._running.clear()

        for worker in (self._receive_thread, self._heartbeat_thread):
            if worker is not None and worker.is_alive():
                worker.join(timeout=0.5)

        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None

        self._receive_thread = None
        self._heartbeat_thread = None
        self.peer_alive = False

    def loop_forever(self) -> None:
        self.start()
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            self.stop()

    def send(self, message: Any, topic: str | None = None, reply_topic: str | None = None) -> None:
        payload = self.build_payload(message, reply_topic, topic)
        self._send_payload(payload)

    def send_and_wait(self, message: Any, topic: str | None = None, reply_topic: str | None = None) -> None:
        self.send(message, topic, reply_topic)

    def reply(self, message: Any, topic: str | None = None) -> None:
        payload = {
            "sender": self.client_id,
            "session_id": self._session_id,
            "message": message,
            "topic": topic or self.reply_topic,
            "seq": self._next_seq(),
            "received_at": time.time(),
        }
        self._send_payload(payload)

    def _send_payload(self, payload: dict[str, Any], broadcast: bool = False) -> None:
        if self._socket is None:
            self.connect()
        if self._socket is None:
            return

        target = (self.broker, self.peer_port) if broadcast else self._target_addr()
        try:
            self._socket.sendto(self._encode(payload), target)
        except OSError:
            pass

    def _heartbeat_loop(self) -> None:
        while self._running.is_set():
            self._send_heartbeat()
            self._update_peer_alive()
            time.sleep(self.heartbeat_interval_s)

    def _send_heartbeat(self) -> None:
        payload = {
            "sender": self.client_id,
            "session_id": self._session_id,
            "message": {"type": "heartbeat"},
            "topic": self.topic,
            "seq": self._next_seq(),
            "sent_at": time.time(),
            "heartbeat": True,
        }
        self._send_payload(payload, broadcast=True)

    def _update_peer_alive(self) -> None:
        if self.last_seen is None:
            self.peer_alive = False
            return
        self.peer_alive = time.monotonic() - self.last_seen <= self.peer_timeout_s

    def _receive_loop(self) -> None:
        while self._running.is_set():
            if self._socket is None:
                time.sleep(DEFAULT_SOCKET_TIMEOUT_S)
                continue

            try:
                packet, addr = self._socket.recvfrom(65535)
            except socket.timeout:
                self._update_peer_alive()
                continue
            except OSError:
                if self._running.is_set():
                    time.sleep(DEFAULT_SOCKET_TIMEOUT_S)
                continue

            payload = self.decode_payload(packet)
            if not self._should_accept(payload, addr):
                continue

            self._learn_peer(addr)
            self.last_seen = time.monotonic()
            self.peer_alive = True

            if payload.get("heartbeat"):
                continue

            topic = str(payload.get("topic", self.topic))
            callback = self.on_reply if topic == self.reply_topic else self.on_message
            if callback is not None:
                try:
                    callback(payload, topic)
                except Exception:
                    pass

    def _should_accept(self, payload: dict[str, Any], addr: tuple[str, int]) -> bool:
        sender = payload.get("sender")
        if not isinstance(sender, str) or sender == self.client_id:
            return False

        seq = payload.get("seq")
        session_id = payload.get("session_id")
        if isinstance(seq, int):
            if not isinstance(session_id, str):
                session_id = ""
            peer_key = (sender, session_id)
            previous_seq = self._last_peer_seq.get(peer_key, 0)
            if seq <= previous_seq:
                return False
            self._last_peer_seq[peer_key] = seq

        return True

    def _learn_peer(self, addr: tuple[str, int]) -> None:
        with self._lock:
            self._peer_addr = addr
