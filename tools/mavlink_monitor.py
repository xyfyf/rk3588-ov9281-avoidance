"""Receive-only UDP monitor. Does not arm, set parameters, or command the FC."""
import argparse
import json
import socket
import time
from pymavlink.dialects.v20 import common as mav


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=14550)
    parser.add_argument("--duration", type=float, default=30)
    args = parser.parse_args()
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind((args.host, args.port))
    receiver.settimeout(0.5)
    decoder = mav.MAVLink(None)
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.duration:
            try:
                packet, _ = receiver.recvfrom(4096)
            except socket.timeout:
                continue
            for message in decoder.parse_buffer(packet) or []:
                if message.get_type() == "OBSTACLE_DISTANCE":
                    step = message.increment_f or message.increment
                    def nearest(angle):
                        index = int(round(((angle - message.angle_offset) % 360) / step)) % 72 if step else 0
                        return message.distances[index]
                    print(json.dumps({"sysid": message.get_srcSystem(), "compid": message.get_srcComponent(),
                                      "frame": message.frame, "increment_f": message.increment_f,
                                      "angle_offset": message.angle_offset,
                                      "front_cm": nearest(0), "right_cm": nearest(90),
                                      "back_cm": nearest(180), "left_cm": nearest(270),
                                      "valid_sectors": sum(d != 65535 for d in message.distances),
                                      "capture_time_usec": message.time_usec}), flush=True)
    finally:
        receiver.close()


if __name__ == "__main__":
    main()
