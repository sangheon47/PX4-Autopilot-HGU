#!/usr/bin/env python3

import argparse
import csv
import datetime as dt
import os
import struct
import sys
import time
from pathlib import Path
from typing import Optional


REPO_ROOT = Path(__file__).resolve().parents[1]
PYMAVLINK_ROOT = REPO_ROOT / "src/modules/mavlink/mavlink"

if str(PYMAVLINK_ROOT) not in sys.path:
    sys.path.insert(0, str(PYMAVLINK_ROOT))

os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "common")

try:
    from pymavlink import mavutil
except ImportError as exc:
    raise SystemExit(
        "pymavlink import failed. Use the repo-local submodule or install pymavlink."
    ) from exc


RT_CONTROL_TUNNEL_PAYLOAD_TYPE = 32768
PAYLOAD = struct.Struct("<QddIIIIIII3f3f6f4H2H")

CSV_FIELDS = [
    "timestamp_us",
    "cycle",
    "ideal_start_s",
    "actual_start_s",
    "start_jitter_s",
    "exec_us",
    "input_us",
    "control_us",
    "output_us",
    "slack_us",
    "missed_cycles",
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "motor_1",
    "motor_2",
    "motor_3",
    "motor_4",
    "motor_5",
    "motor_6",
    "servo_1_pwm_us",
    "servo_2_pwm_us",
    "servo_3_pwm_us",
    "servo_4_pwm_us",
    "bldc_1_dshot",
    "bldc_2_dshot",
]


def default_output_path() -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(f"rt_telem_{stamp}.csv")


def build_endpoint(args: argparse.Namespace) -> str:
    if args.mode == "udp":
        return f"udpin:{args.udp_host}:{args.udp_port}"

    if args.mode == "tcp":
        return f"tcp:{args.tcp_host}:{args.tcp_port}"

    return args.serial_device


def open_link(args: argparse.Namespace):
    endpoint = build_endpoint(args)
    kwargs = {
        "dialect": "common",
        "autoreconnect": True,
        "source_system": 255,
        "source_component": 190,
    }

    if args.mode == "serial":
        kwargs["baud"] = args.serial_baud

    link = mavutil.mavlink_connection(endpoint, **kwargs)
    print(f"connected {endpoint}", file=sys.stderr)
    return link


def payload_to_row(values):
    return {
        "timestamp_us": values[0],
        "cycle": values[3],
        "ideal_start_s": values[1],
        "actual_start_s": values[2],
        "start_jitter_s": values[2] - values[1],
        "exec_us": values[4],
        "input_us": values[5],
        "control_us": values[6],
        "output_us": values[7],
        "slack_us": values[8],
        "missed_cycles": values[9],
        "accel_x": values[10],
        "accel_y": values[11],
        "accel_z": values[12],
        "gyro_x": values[13],
        "gyro_y": values[14],
        "gyro_z": values[15],
        "motor_1": values[16],
        "motor_2": values[17],
        "motor_3": values[18],
        "motor_4": values[19],
        "motor_5": values[20],
        "motor_6": values[21],
        "servo_1_pwm_us": values[22],
        "servo_2_pwm_us": values[23],
        "servo_3_pwm_us": values[24],
        "servo_4_pwm_us": values[25],
        "bldc_1_dshot": values[26],
        "bldc_2_dshot": values[27],
    }


def extract_payload(msg) -> Optional[bytes]:
    if msg.get_type() != "TUNNEL":
        return None

    if int(msg.payload_type) != RT_CONTROL_TUNNEL_PAYLOAD_TYPE:
        return None

    payload = bytes(msg.payload[: msg.payload_length])

    if len(payload) != PAYLOAD.size:
        return None

    return payload


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Receive rt_control telemetry over MAVLink TUNNEL and write CSV."
    )
    parser.add_argument("--mode", choices=["udp", "tcp", "serial"], default="udp")
    parser.add_argument("--output", type=Path, default=default_output_path())
    parser.add_argument("--udp-host", default="0.0.0.0")
    parser.add_argument("--udp-port", type=int, default=14550)
    parser.add_argument("--tcp-host", default="192.168.2.1")
    parser.add_argument("--tcp-port", type=int, default=14550)
    parser.add_argument("--serial-device", default="/dev/ttyUSB0")
    parser.add_argument("--serial-baud", type=int, default=921600)
    parser.add_argument("--flush-every", type=int, default=200)
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    link = open_link(args)
    rows_written = 0
    last_report = time.monotonic()

    with args.output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()

        while True:
            msg = link.recv_match(blocking=True, timeout=1.0)

            if msg is None:
                continue

            if msg.get_type() == "BAD_DATA":
                continue

            payload = extract_payload(msg)

            if payload is None:
                continue

            row = payload_to_row(PAYLOAD.unpack(payload))
            writer.writerow(row)
            rows_written += 1

            if rows_written % max(args.flush_every, 1) == 0:
                f.flush()

            now = time.monotonic()

            if now - last_report >= 1.0:
                print(
                    f"rows={rows_written} last_cycle={row['cycle']} last_exec_us={row['exec_us']}",
                    file=sys.stderr,
                )
                last_report = now

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
