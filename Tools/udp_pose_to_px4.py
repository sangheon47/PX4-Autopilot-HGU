#!/usr/bin/env python3

import argparse
import csv
import math
import os
import select
import socket
import struct
import sys
import time
from pathlib import Path
from typing import Optional, Sequence


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


POSE_PACKET = struct.Struct("<6f")
RT_CONTROL_TUNNEL_PAYLOAD_TYPE = 32768
RT_PAYLOAD = struct.Struct("<dIIIIII3f3f4f4H6fIIB7x")
RT_CSV_FIELDS = [
    "cycle",
    "actual_start_s",
    "input_us",
    "control_us",
    "output_us",
    "exec_us",
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
    "pwm_1",
    "pwm_2",
    "pwm_3",
    "pwm_4",
    "opti_x",
    "opti_y",
    "opti_z",
    "opti_roll",
    "opti_pitch",
    "opti_yaw",
    "opti_seq",
    "opti_age_us",
    "opti_valid",
]


def default_log_output_path() -> Path:
    return REPO_ROOT / "logs" / "rt_telem_test3.csv"


def resolve_serial_device(serial_device: str) -> str:
    if serial_device != "auto":
        return serial_device

    candidates = sorted(Path("/dev").glob("ttyACM*")) + sorted(Path("/dev").glob("ttyUSB*"))

    if not candidates:
        raise SystemExit(
            "serial auto-detect found no /dev/ttyACM* or /dev/ttyUSB* device. "
            "Reconnect PX4 USB or pass --serial-device explicitly."
        )

    selected = str(candidates[0])
    print(f"serial auto-detect selected {selected}", file=sys.stderr)

    if len(candidates) > 1:
        print(
            f"serial auto-detect picked {selected} from {[str(path) for path in candidates]}",
            file=sys.stderr,
        )

    return selected


def build_mavlink_endpoint(args: argparse.Namespace) -> str:
    if args.mode == "udp":
        return f"udpout:{args.udp_host}:{args.udp_port}"

    if args.mode == "tcp":
        return f"tcp:{args.tcp_host}:{args.tcp_port}"

    args.serial_device = resolve_serial_device(args.serial_device)
    return args.serial_device


def open_mavlink_link(args: argparse.Namespace):
    endpoint = build_mavlink_endpoint(args)
    kwargs = {
        "dialect": "common",
        "autoreconnect": True,
        "source_system": args.source_system,
        "source_component": args.source_component,
    }

    if args.mode == "serial":
        kwargs["baud"] = args.serial_baud

    link = mavutil.mavlink_connection(endpoint, **kwargs)

    if args.mode == "serial" and hasattr(link, "port"):
        try:
            link.port.write_timeout = 0.2
        except Exception:
            pass

    print(f"mavlink connected {endpoint}", file=sys.stderr)
    return link


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Sequence[float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qw, qx, qy, qz)


def convert_input_pose(
    x: float, y: float, z: float, roll: float, pitch: float, yaw: float, args: argparse.Namespace
) -> tuple[float, float, float, float, float, float]:
    # The default assumes the incoming UDP packet is already in the exact PX4 frame you want to use.
    # If your sender still uses raw OptiTrack coordinates, change this function once on the PC side.
    if args.angle_unit == "deg":
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

    if args.ignore_attitude:
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

    return x, y, z, roll, pitch, yaw


def extract_rt_payload(msg) -> Optional[bytes]:
    if msg.get_type() != "TUNNEL":
        return None

    if int(msg.payload_type) != RT_CONTROL_TUNNEL_PAYLOAD_TYPE:
        return None

    payload = bytes(msg.payload[: msg.payload_length])

    if len(payload) != RT_PAYLOAD.size:
        return None

    return payload


def rt_payload_to_row(values: Sequence[float]) -> dict[str, float]:
    return {
        "cycle": values[1],
        "actual_start_s": values[0],
        "input_us": values[3],
        "control_us": values[4],
        "output_us": values[5],
        "exec_us": values[2],
        "missed_cycles": values[6],
        "accel_x": values[7],
        "accel_y": values[8],
        "accel_z": values[9],
        "gyro_x": values[10],
        "gyro_y": values[11],
        "gyro_z": values[12],
        "motor_1": values[13],
        "motor_2": values[14],
        "motor_3": values[15],
        "motor_4": values[16],
        "pwm_1": values[17],
        "pwm_2": values[18],
        "pwm_3": values[19],
        "pwm_4": values[20],
        "opti_x": values[21],
        "opti_y": values[22],
        "opti_z": values[23],
        "opti_roll": values[24],
        "opti_pitch": values[25],
        "opti_yaw": values[26],
        "opti_seq": values[27],
        "opti_age_us": values[28],
        "opti_valid": values[29],
    }


def maybe_send_heartbeat(link, now: float, last_heartbeat: float) -> float:
    if now - last_heartbeat < 1.0:
        return last_heartbeat

    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )
    return now


def send_odometry(link, args: argparse.Namespace, sample: tuple[float, float, float, float, float, float]):
    x, y, z, roll, pitch, yaw = sample
    q = quaternion_from_rpy(roll, pitch, yaw)
    nan = float("nan")
    pose_covariance = [0.0] * 21
    velocity_covariance = [0.0] * 21

    link.mav.odometry_send(
        time.time_ns() // 1000,
        mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
        mavutil.mavlink.MAV_FRAME_BODY_FRD,
        x,
        y,
        z,
        q,
        nan,
        nan,
        nan,
        nan,
        nan,
        nan,
        pose_covariance,
        velocity_covariance,
        0,
        args.estimator_type,
        args.quality,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Receive UDP <6f> pose packets and forward them to PX4 as MAVLink ODOMETRY."
    )
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=38030)
    parser.add_argument("--angle-unit", choices=["deg", "rad"], default="rad")
    parser.add_argument("--mode", choices=["udp", "tcp", "serial"], default="udp")
    parser.add_argument("--udp-host", default="192.168.2.1")
    parser.add_argument("--udp-port", type=int, default=14550)
    parser.add_argument("--tcp-host", default="192.168.2.1")
    parser.add_argument("--tcp-port", type=int, default=14550)
    parser.add_argument("--serial-device", default="auto")
    parser.add_argument("--serial-baud", type=int, default=921600)
    parser.add_argument("--source-system", type=int, default=200)
    parser.add_argument(
        "--source-component",
        type=int,
        default=mavutil.mavlink.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY,
    )
    parser.add_argument(
        "--estimator-type",
        type=int,
        default=mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,
    )
    parser.add_argument("--quality", type=int, default=100)
    parser.add_argument("--odometry-rate", type=float, default=50.0)
    parser.add_argument("--ignore-attitude", action="store_true")
    parser.add_argument("--send-heartbeat", action="store_true")
    parser.add_argument("--log-output", type=Path, default=default_log_output_path())
    parser.add_argument("--flush-every", type=int, default=200)
    args = parser.parse_args()

    link = open_mavlink_link(args)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind((args.listen_host, args.listen_port))
    sock.setblocking(False)
    print(f"udp pose input bound {args.listen_host}:{args.listen_port}", file=sys.stderr)

    log_file = None
    csv_writer = None

    if args.log_output is not None:
        args.log_output.parent.mkdir(parents=True, exist_ok=True)
        log_file = args.log_output.open("w", newline="")
        csv_writer = csv.DictWriter(log_file, fieldnames=RT_CSV_FIELDS)
        csv_writer.writeheader()
    rows_written = 0
    last_heartbeat = 0.0
    last_report = time.monotonic()
    packets_received = 0
    odom_sent = 0
    odom_send_failures = 0
    odom_rate_limited = 0
    last_pose = None
    last_pose_addr = None
    last_odometry_send = 0.0
    odometry_period = 1.0 / args.odometry_rate if args.odometry_rate > 0.0 else 0.0

    try:
        while True:
            now = time.monotonic()

            if args.send_heartbeat:
                last_heartbeat = maybe_send_heartbeat(link, now, last_heartbeat)

            readable, _, _ = select.select([sock], [], [], 1.0)

            if readable:
                drained_packets = 0

                while drained_packets < 256:
                    try:
                        data, addr = sock.recvfrom(2048)
                    except BlockingIOError:
                        break

                    drained_packets += 1

                    if len(data) != POSE_PACKET.size:
                        print(f"ignored udp packet len={len(data)} from={addr}", file=sys.stderr)
                        continue

                    raw_pose = POSE_PACKET.unpack(data)
                    sample = convert_input_pose(*raw_pose, args=args)
                    packets_received += 1
                    last_pose = sample
                    last_pose_addr = addr

                    if packets_received == 1:
                        print(
                            f"first udp pose from={addr} "
                            f"pose=({sample[0]:.3f}, {sample[1]:.3f}, {sample[2]:.3f}, "
                            f"{sample[3]:.3f}, {sample[4]:.3f}, {sample[5]:.3f})",
                            file=sys.stderr,
                        )

                    now = time.monotonic()
                    send_now = odometry_period <= 0.0 or now >= (last_odometry_send + odometry_period)

                    if send_now:
                        try:
                            send_odometry(link, args, sample)
                            odom_sent += 1
                            last_odometry_send = now
                        except Exception as exc:
                            odom_send_failures += 1
                            print(f"odometry forward failed: {exc}", file=sys.stderr)
                    else:
                        odom_rate_limited += 1

            for _ in range(50):
                msg = link.recv_match(blocking=False)

                if msg is None:
                    break

                if msg.get_type() == "BAD_DATA":
                    continue

                if csv_writer is None:
                    continue

                payload = extract_rt_payload(msg)

                if payload is None:
                    continue

                row = rt_payload_to_row(RT_PAYLOAD.unpack(payload))
                csv_writer.writerow(row)
                rows_written += 1

                if rows_written % max(args.flush_every, 1) == 0:
                    log_file.flush()

            now = time.monotonic()

            if now - last_report >= 1.0:
                if last_pose is None:
                    pose_text = "last_pose=none"
                else:
                    pose_text = (
                        f"last_pose=({last_pose[0]:.3f}, {last_pose[1]:.3f}, {last_pose[2]:.3f}, "
                        f"{last_pose[3]:.3f}, {last_pose[4]:.3f}, {last_pose[5]:.3f})"
                    )

                print(
                    f"udp_packets={packets_received} odom_sent={odom_sent} "
                    f"odom_send_failures={odom_send_failures} odom_rate_limited={odom_rate_limited} "
                    f"rows={rows_written} "
                    f"last_addr={last_pose_addr} {pose_text}",
                    file=sys.stderr,
                )
                last_report = now

    finally:
        if log_file is not None:
            log_file.flush()
            log_file.close()
        sock.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
