import argparse
import json
import socket
import time

try:
    import paho.mqtt.client as mqtt  # type: ignore[import-not-found]
except ImportError as error:
    raise SystemExit("Install MQTT support with: python -m pip install paho-mqtt") from error


DEFAULT_BROKER = "localhost"
DEFAULT_PORT = 1883
DEFAULT_TOPIC = "soccer/pi/messages"
DEFAULT_REPLY_TOPIC = "soccer/pi/replies"


def build_client(client_id):
    return mqtt.Client(client_id=client_id)


def decode_payload(payload):
    text = payload.decode("utf-8", errors="replace")
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return {"message": text}


def main():
    parser = argparse.ArgumentParser(description="Receive MQTT messages from another Raspberry Pi.")
    parser.add_argument("--broker", default=DEFAULT_BROKER, help="MQTT broker hostname or IP address")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="MQTT broker port")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="Topic to listen on")
    parser.add_argument("--reply-topic", default=DEFAULT_REPLY_TOPIC, help="Default topic used for replies")
    parser.add_argument("--client-id", default=f"pi-server-{socket.gethostname()}", help="MQTT client id")
    args = parser.parse_args()

    client = build_client(args.client_id)

    def on_connect(client, _userdata, _flags, reason_code, _properties=None):
        if int(reason_code) != 0:
            print(f"Failed to connect to MQTT broker: {reason_code}")
            return

        client.subscribe(args.topic)
        print(f"Connected to {args.broker}:{args.port}; listening on {args.topic}")

    def on_message(client, _userdata, message):
        payload = decode_payload(message.payload)
        sender = payload.get("sender", "unknown")
        body = payload.get("message", payload)
        reply_topic = payload.get("reply_topic") or args.reply_topic

        print(f"[{time.strftime('%H:%M:%S')}] {sender} -> {message.topic}: {body}")

        reply = {
            "sender": args.client_id,
            "message": f"received: {body}",
            "received_at": time.time(),
        }
        client.publish(reply_topic, json.dumps(reply), qos=1)

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.broker, args.port, keepalive=60)
    client.loop_forever()


if __name__ == "__main__":
    main()
