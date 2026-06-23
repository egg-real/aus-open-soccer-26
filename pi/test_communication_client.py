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


def build_payload(sender, message, reply_topic):
    return {
        "sender": sender,
        "message": message,
        "reply_topic": reply_topic,
        "sent_at": time.time(),
    }


def main():
    parser = argparse.ArgumentParser(description="Send MQTT messages to another Raspberry Pi.")
    parser.add_argument("message", nargs="?", help="Message to send. If omitted, messages are read from stdin.")
    parser.add_argument("--broker", default=DEFAULT_BROKER, help="MQTT broker hostname or IP address")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="MQTT broker port")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="Topic to publish messages to")
    parser.add_argument("--reply-topic", default=DEFAULT_REPLY_TOPIC, help="Topic to listen on for replies")
    parser.add_argument("--client-id", default=f"pi-client-{socket.gethostname()}", help="MQTT client id")
    args = parser.parse_args()

    client = mqtt.Client(client_id=args.client_id)

    def on_connect(client, _userdata, _flags, reason_code, _properties=None):
        if int(reason_code) != 0:
            print(f"Failed to connect to MQTT broker: {reason_code}")
            return

        client.subscribe(args.reply_topic)
        print(f"Connected to {args.broker}:{args.port}; publishing to {args.topic}")

    def on_message(_client, _userdata, message):
        payload = message.payload.decode("utf-8", errors="replace")
        print(f"reply on {message.topic}: {payload}")

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.broker, args.port, keepalive=60)
    client.loop_start()

    try:
        if args.message is not None:
            payload = build_payload(args.client_id, args.message, args.reply_topic)
            client.publish(args.topic, json.dumps(payload), qos=1).wait_for_publish()
            time.sleep(1)
            return

        print("Type messages to publish. Press Ctrl+C to exit.")
        while True:
            message = input("> ").strip()
            if not message:
                continue

            payload = build_payload(args.client_id, message, args.reply_topic)
            client.publish(args.topic, json.dumps(payload), qos=1)
    except KeyboardInterrupt:
        print()
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
