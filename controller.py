import lcm
import time
from heartbeat_t import heartbeat_t
from message_t import message_t

lc = lcm.LCM()

log_file = "log.txt"

def log_message(msg):
    with open(log_file, "a") as f:
        f.write(f"Timestamp: {msg.timestamp}, ID: {msg.ID}, Name: {msg.name}, DetectedObject: {msg.detectedObject}, Mode: {msg.mode}\n")

def message_handler(channel, data):
    msg = message_t.decode(data)
    print(f"Received message on channel {channel}: {msg.name}, DetectedObject: {msg.detectedObject}")

    log_message(msg)

    local_time = int(time.time() * 1000000)
    print(f"Local time on message reception: {local_time}")

def main():
    lc.subscribe("MESSAGE", message_handler)

    while True:
        lc.handle()

if __name__ == "__main__":
    main()
