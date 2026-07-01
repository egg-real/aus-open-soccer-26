import json
import socket
import time
from typing import Any, Callable

try:
    import paho.mqtt.client as mqtt  # type: ignore[import-not-found]
except ImportError as error:
    raise SystemExit("Install MQTT support with: python -m pip install paho-mqtt") from error


DEFAULT_BROKER = "10.42.0.1"
DEFAULT_PORT = 1883
DEFAULT_TOPIC = "soccer/pi/messages"
DEFAULT_REPLY_TOPIC = "soccer/pi/replies"

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
    ) -> None:
        self.broker = broker
        self.port = port
        self.topic = topic
        self.reply_topic = reply_topic
        self.client_id = client_id or f"pi-{socket.gethostname()}"
        self.on_message: MessageCallback | None = None
        self.on_reply: ReplyCallback | None = None

        self.client = mqtt.Client(client_id=self.client_id)
        self.client.on_connect = self._handle_connect
        self.client.on_message = self._handle_message

    def _handle_connect(self, client, _userdata, _flags, reason_code, _properties=None) -> None:
        if int(reason_code) != 0:
            print(f"Failed to connect to MQTT broker: {reason_code}")
            return

        client.subscribe(self.topic)
        if self.reply_topic != self.topic:
            client.subscribe(self.reply_topic)
        print(f"Connected to {self.broker}:{self.port}")

    def _handle_message(self, _client, _userdata, message) -> None:
        payload = self.decode_payload(message.payload)
        if message.topic == self.reply_topic:
            if self.on_reply is not None:
                self.on_reply(payload, message.topic)
            return

        if self.on_message is not None:
            self.on_message(payload, message.topic)

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

    def build_payload(self, message: Any, reply_topic: str | None = None) -> dict[str, Any]:
        return {
            "sender": self.client_id,
            "message": message,
            "reply_topic": reply_topic or self.reply_topic,
            "sent_at": time.time(),
        }

    def connect(self) -> None:
        self.client.connect(self.broker, self.port, keepalive=60)

    def loop_forever(self) -> None:
        self.connect()
        self.client.loop_forever()

    def start(self) -> None:
        self.connect()
        self.client.loop_start()

    def stop(self) -> None:
        self.client.loop_stop()
        self.client.disconnect()

    def send(self, message: Any, topic: str | None = None, reply_topic: str | None = None) -> None:
        payload = self.build_payload(message, reply_topic)
        self.client.publish(topic or self.topic, json.dumps(payload), qos=1)

    def send_and_wait(self, message: Any, topic: str | None = None, reply_topic: str | None = None) -> None:
        payload = self.build_payload(message, reply_topic)
        self.client.publish(topic or self.topic, json.dumps(payload), qos=1).wait_for_publish()

    def reply(self, message: Any, topic: str | None = None) -> None:
        payload = {
            "sender": self.client_id,
            "message": message,
            "received_at": time.time(),
        }
        self.client.publish(topic or self.reply_topic, json.dumps(payload), qos=1)
