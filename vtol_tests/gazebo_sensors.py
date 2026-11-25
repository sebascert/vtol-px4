#!/usr/bin/env python3
"""Verify custom Stallion sensors reach PX4 and EKF estimates agree with truth."""

import json
import hashlib
import math
import os
from pathlib import Path
import pty
import re
import select
import signal
import socket
import subprocess
import tempfile
import time

CONTROLLER = Path(__file__).resolve().parents[1]
REPO = CONTROLLER.parent
OUTPUT = CONTROLLER / "build/gazebo-sensors"


def plain(data):
    text = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", data.decode(errors="replace"))
    return "\n".join(line.rstrip("\r").rsplit("\r", 1)[-1] for line in text.split("\n"))


def scalar(text, key):
    match = re.search(r"^\s+" + re.escape(key) + r":\s*(\S+)", text, re.M)
    if not match:
        raise ValueError(f"Missing {key}: {text}")
    return float(match[1])


def vector(text, key):
    values = re.search(r"\b" + key + r":\s*\[([^]]+)\]", text)
    if not values:
        raise ValueError(f"Missing vector {key}")
    return [float(value) for value in values[1].split(",")]


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    for port in (14580, 18570):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("0.0.0.0", port))
    env = dict(
        os.environ, GZ_PARTITION=f"vtol-sensors-{os.getpid()}", GZ_IP="127.0.0.1"
    )
    processes, console = [], bytearray()
    result = {
        "passed": False,
        "scope": "Custom aircraft sensors and PX4 EKF only; disarmed, no allocator/output mapping",
        "vehicle_config_sha256": hashlib.sha256(
            (Path.home() / ".local/share/vtol26/config/vehicle.json").read_bytes()
        ).hexdigest(),
    }
    master, slave = pty.openpty()

    def stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)

    with (
        tempfile.TemporaryDirectory(prefix="run-", dir=OUTPUT) as run,
        (OUTPUT / "gazebo.log").open("w") as gz_log,
        (OUTPUT / "px4.log").open("wb") as px4_log,
    ):

        def pump(seconds=0.1):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                if select.select(
                    [master], [], [], min(0.1, max(0, end - time.monotonic()))
                )[0]:
                    try:
                        data = os.read(master, 65536)
                    except OSError:
                        data = b""
                    console.extend(data)
                    px4_log.write(data)
                    px4_log.flush()
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError("Gazebo/PX4 exited early; inspect logs")

        def topic(name):
            mark = len(console)
            os.write(master, f"listener {name}\n".encode())
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                pump()
                text = plain(console[mark:])
                if (
                    "TOPIC: " + name in text
                    and "\npxh>" in text.split("TOPIC: " + name, 1)[1]
                ):
                    return text
                if "never published" in text:
                    return text
            raise TimeoutError(name)

        try:
            processes.append(
                subprocess.Popen(
                    [str(REPO / "simulation/run.sh"), "-s", "--seed", "42"],
                    cwd=REPO,
                    env=env,
                    stdout=gz_log,
                    stderr=gz_log,
                    start_new_session=True,
                )
            )
            processes.append(
                subprocess.Popen(
                    [str(CONTROLLER / "run-gazebo-sensors.sh"), "-w", run],
                    cwd=REPO,
                    env=env,
                    stdin=slave,
                    stdout=slave,
                    stderr=slave,
                    start_new_session=True,
                )
            )
            os.close(slave)
            deadline = time.monotonic() + 75
            while "Startup script returned successfully" not in plain(console):
                pump()
                if time.monotonic() > deadline:
                    raise TimeoutError("PX4 startup")
            # Wait for genuine EKF local position, not ground-truth substitution.
            while time.monotonic() < deadline:
                pump(1)
                local = topic("vehicle_local_position")
                if all(
                    re.search(r"\b" + key + r": True\b", local)
                    for key in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")
                ):
                    break
            else:
                raise TimeoutError("EKF local position validity")
            pump(5)
            names = (
                "sensor_combined",
                "sensor_mag",
                "sensor_baro",
                "sensor_gps",
                "vehicle_attitude",
                "vehicle_local_position",
                "vehicle_global_position",
                "vehicle_attitude_groundtruth",
                "vehicle_local_position_groundtruth",
                "vehicle_global_position_groundtruth",
                "vehicle_status",
            )
            first = {name: topic(name) for name in names}
            pump(2)
            second = {name: topic(name) for name in names}
            for name in names:
                (OUTPUT / f"{name}.txt").write_text(
                    first[name] + "\nSECOND SAMPLE\n" + second[name]
                )
                assert scalar(second[name], "timestamp") > scalar(
                    first[name], "timestamp"
                ), name
            accel = vector(second["sensor_combined"], "accelerometer_m_s2")
            assert abs(math.sqrt(sum(x * x for x in accel)) - 9.81) < 0.3, accel
            assert 80000 < scalar(second["sensor_baro"], "pressure") < 110000
            assert scalar(second["sensor_gps"], "fix_type") >= 3
            assert (
                abs(scalar(second["sensor_gps"], "latitude_deg") - 47.3979710577)
                < 0.0001
            )
            q, truth = (
                vector(second[name], "q")
                for name in ("vehicle_attitude", "vehicle_attitude_groundtruth")
            )
            dot = abs(sum(a * b for a, b in zip(q, truth))) / math.sqrt(
                sum(x * x for x in q) * sum(x * x for x in truth)
            )
            angle = 2 * math.acos(min(1, dot))
            assert angle < 0.05, angle
            estimated, actual = (
                second["vehicle_local_position"],
                second["vehicle_local_position_groundtruth"],
            )
            assert all(
                re.search(r"\b" + key + r": True\b", estimated)
                for key in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")
            )
            # gz_bridge local truth uses world position with an initial-GPS reference;
            # its global truth provides the consistent geographic position/altitude.
            global_est, global_truth = (
                second["vehicle_global_position"],
                second["vehicle_global_position_groundtruth"],
            )
            north = (
                math.radians(scalar(global_est, "lat") - scalar(global_truth, "lat"))
                * 6371000
            )
            east = (
                math.radians(scalar(global_est, "lon") - scalar(global_truth, "lon"))
                * 6371000
                * math.cos(math.radians(47.397971))
            )
            horizontal_error = math.hypot(north, east)
            assert horizontal_error < 0.5, horizontal_error
            altitude_error = abs(
                scalar(global_est, "alt") - scalar(global_truth, "alt")
            )
            assert altitude_error < 0.5, altitude_error
            velocity_error = math.sqrt(
                sum(
                    (scalar(estimated, k) - scalar(actual, k)) ** 2
                    for k in ("vx", "vy", "vz")
                )
            )
            assert velocity_error < 0.3, velocity_error
            assert scalar(second["vehicle_status"], "arming_state") == 1  # DISARMED
            for module in ("gz_bridge", "sensor_mag_sim", "ekf2"):
                mark = len(console)
                os.write(master, f"{module} status\n".encode())
                pump(0.3)
                (OUTPUT / f"{module}-status.txt").write_text(plain(console[mark:]))
            result.update(
                passed=True,
                advancing_topics=list(names),
                attitude_error_rad=angle,
                horizontal_error_m=horizontal_error,
                altitude_error_m=altitude_error,
                velocity_error_m_s=velocity_error,
                imu_rate_hz=1e6 / scalar(second["sensor_combined"], "gyro_integral_dt"),
            )
            print(json.dumps(result, indent=2))
        except BaseException as error:
            result["error"] = str(error)
            raise
        finally:
            (OUTPUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            for p in reversed(processes):
                if p.poll() is None:
                    os.killpg(p.pid, signal.SIGTERM)
                    try:
                        p.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(p.pid, signal.SIGKILL)
                        p.wait()
            os.close(master)


if __name__ == "__main__":
    main()
