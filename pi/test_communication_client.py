import argparse
import socket
import time

from lib.communication import Communication, DEFAULT_BROKER, DEFAULT_PORT, DEFAULT_REPLY_TOPIC, DEFAULT_TOPIC


def main():
    parser = argparse.ArgumentParser(description="Send MQTT messages to another Raspberry Pi.")
    parser.add_argument("message", nargs="?", help="Message to send. If omitted, messages are read from stdin.")
    parser.add_argument("--broker", default=DEFAULT_BROKER, help="MQTT broker hostname or IP address")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="MQTT broker port")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="Topic to publish messages to")
    parser.add_argument("--reply-topic", default=DEFAULT_REPLY_TOPIC, help="Topic to listen on for replies")
    parser.add_argument("--client-id", default=f"pi-client-{socket.gethostname()}", help="MQTT client id")
    args = parser.parse_args()

    communication = Communication(
        broker=args.broker,
        port=args.port,
        topic=args.topic,
        reply_topic=args.reply_topic,
        client_id=args.client_id,
    )

    def on_reply(payload, topic):
        print(f"reply on {topic}: {payload}")

    communication.on_reply = on_reply
    communication.start()
    print(f"Publishing to {args.topic}")

    try:
        if args.message is not None:
            communication.send_and_wait(args.message)
            time.sleep(1)
            return

        print("Type messages to publish. Press Ctrl+C to exit.")
        while True:
            message = input("> ").strip()
            if not message:
                continue

            communication.send(message)
    except KeyboardInterrupt:
        print()
    finally:
        communication.stop()


if __name__ == "__main__":
    main()
