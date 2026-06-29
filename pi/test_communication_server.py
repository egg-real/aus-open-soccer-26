import argparse
import socket
import time

from lib.communication import Communication, DEFAULT_BROKER, DEFAULT_PORT, DEFAULT_REPLY_TOPIC, DEFAULT_TOPIC


def main():
    parser = argparse.ArgumentParser(description="Receive MQTT messages from another Raspberry Pi.")
    parser.add_argument("--broker", default=DEFAULT_BROKER, help="MQTT broker hostname or IP address")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="MQTT broker port")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="Topic to listen on")
    parser.add_argument("--reply-topic", default=DEFAULT_REPLY_TOPIC, help="Default topic used for replies")
    parser.add_argument("--client-id", default=f"pi-server-{socket.gethostname()}", help="MQTT client id")
    args = parser.parse_args()

    communication = Communication(
        broker=args.broker,
        port=args.port,
        topic=args.topic,
        reply_topic=args.reply_topic,
        client_id=args.client_id,
    )

    def on_message(payload, topic):
        sender = payload.get("sender", "unknown")
        body = payload.get("message", payload)
        reply_topic = payload.get("reply_topic") or args.reply_topic

        print(f"[{time.strftime('%H:%M:%S')}] {sender} -> {topic}: {body}")
        communication.reply(f"received: {body}", reply_topic)

    communication.on_message = on_message
    print(f"Listening on {args.topic}")
    communication.loop_forever()


if __name__ == "__main__":
    main()
