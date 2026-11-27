#!/usr/bin/env python3
"""Run the project SITL/X500 and allocator; save proof of diagnostic reception."""

import argparse
from contextlib import ExitStack
import hashlib
import json
import math
import os
from pathlib import Path
import pty
import re
import select
import signal
import socket
import subprocess
import time

from pymavlink import mavutil

CONTROLLER = Path(__file__).resolve().parents[1]
REPO = CONTROLLER.parent
SITL = CONTROLLER / "build/sitl"
OUTPUT = CONTROLLER / "build/gazebo-communication"


def plain(data):
    text = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", data.decode(errors="replace"))
    return "\n".join(line.rstrip("\r").rsplit("\r", 1)[-1] for line in text.split("\n"))


def scalar(text, key):
    match = re.search(r"^    " + key + r":\s*(\S+)", text, re.M)
    if not match:
        raise ValueError(f"Missing PX4 field {key}")
    return float(match[1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    vehicle = Path.home() / ".local/share/vtol26/config/vehicle.json"
    allocator = REPO / "companion/allocator/build/vtol_allocator"
    if not allocator.is_file() or not (SITL / "bin/px4").is_file():
        parser.error("Build the allocator and controller sitl-native targets first")
    if json.loads(vehicle.read_text()) != json.loads(
        (SITL / "vehicle.json").read_text()
    ):
        parser.error(
            "Rebuild sitl-native with the selected vehicle configuration first"
        )
    # Avoid colliding with another PX4 instance or an existing serial bridge.
    for port in (14540, 14580, 18570):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("0.0.0.0", port))
    OUTPUT.mkdir(parents=True, exist_ok=True)
    link = OUTPUT / "allocator.pty"
    link.unlink(missing_ok=True)
    env = dict(
        os.environ,
        HEADLESS="1",
        GZ_DISTRO="harmonic",
        GZ_IP="127.0.0.1",
        GIT_SUBMODULES_ARE_EVIL="1",
        GZ_PARTITION=f"vtol26-allocator-test-{os.getpid()}",
    )
    for key in (
        "PX4_SYS_AUTOSTART",
        "PX4_SIMULATOR",
        "PX4_SIM_MODEL",
        "PX4_GZ_STANDALONE",
    ):
        env.pop(key, None)
    processes = []
    console = bytearray()
    master, slave = pty.openpty()
    heartbeat = mavutil.mavlink_connection(
        "udpout:127.0.0.1:18570", source_system=255, source_component=190
    )
    next_heartbeat = 0
    result = {
        "passed": False,
        "scope": "Project PX4 SITL + stock X500 + diagnostic allocator; no PSO actuation",
        "vehicle_config": str(vehicle),
        "vehicle_snapshot_sha256": hashlib.sha256(
            (SITL / "vehicle.json").read_bytes()
        ).hexdigest(),
    }

    def stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)

    def terminate(process):
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()

    with ExitStack() as files:
        px4_log = files.enter_context((OUTPUT / "px4.log").open("wb"))
        output = files.enter_context((OUTPUT / "allocator.jsonl").open("w"))
        errors = files.enter_context((OUTPUT / "allocator.stderr").open("w"))
        bridge_log = files.enter_context((OUTPUT / "bridge.log").open("w"))

        def pump(duration=0.1):
            nonlocal next_heartbeat
            end = time.monotonic() + duration
            while time.monotonic() < end:
                now = time.monotonic()
                if now >= next_heartbeat:
                    heartbeat.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0,
                        0,
                        mavutil.mavlink.MAV_STATE_ACTIVE,
                    )
                    next_heartbeat = now + 1
                if select.select([master], [], [], min(0.1, max(0, end - now)))[0]:
                    try:
                        data = os.read(master, 65536)
                    except OSError:
                        data = b""
                    if data:
                        console.extend(data)
                        px4_log.write(data)
                        px4_log.flush()

        def wait_for(predicate, seconds, label):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                pump()
                if predicate():
                    return
                if processes and processes[0].poll() is not None:
                    raise RuntimeError(f"PX4 launcher exited while waiting for {label}")
            raise TimeoutError(label)

        def command(value):
            mark = len(console)
            os.write(master, (value + "\n").encode())
            return mark

        def topic(name):
            mark = command("listener " + name)

            def complete():
                text = plain(console[mark:])
                return (
                    "TOPIC: " + name in text
                    and "\npxh>" in text.split("TOPIC: " + name, 1)[1]
                )

            wait_for(complete, 8, name)
            return plain(console[mark:])

        def allocations():
            lines = (OUTPUT / "allocator.jsonl").read_text().splitlines()
            # The writer may currently be appending the last line.
            return [json.loads(line) for line in lines if line.endswith("}")]

        try:
            processes.append(
                subprocess.Popen(
                    [
                        "cmake",
                        "--build",
                        str(SITL),
                        "--target",
                        "gz_x500",
                        "--parallel",
                        "2",
                    ],
                    cwd=REPO,
                    env=env,
                    stdin=slave,
                    stdout=slave,
                    stderr=slave,
                    start_new_session=True,
                )
            )
            os.close(slave)
            wait_for(
                lambda: "Ready for takeoff!" in plain(console),
                90,
                "Gazebo and estimator readiness",
            )
            first_sensor = scalar(topic("sensor_combined"), "timestamp")
            bridge = subprocess.Popen(
                [
                    "socat",
                    f"PTY,link={link},rawer,echo=0",
                    "UDP4:127.0.0.1:14580,bind=127.0.0.1:14540",
                ],
                stdout=bridge_log,
                stderr=bridge_log,
                start_new_session=True,
            )
            processes.append(bridge)
            wait_for(link.exists, 5, "serial bridge")
            processes.append(
                subprocess.Popen(
                    [
                        str(allocator),
                        "--port",
                        str(link),
                        "--baud",
                        "115200",
                        "--pso-config",
                        str(REPO / "companion/allocator/src/pso/config.json"),
                    ],
                    stdout=output,
                    stderr=errors,
                    start_new_session=True,
                )
            )
            command("commander takeoff")
            wait_for(
                lambda: (
                    len(allocations()) >= 3
                    and any(
                        row["state_position_ned"][2] < -1.5 for row in allocations()
                    )
                ),
                45,
                "airborne allocator output",
            )
            samples = []
            for _ in range(2):
                text = topic("pso_allocation")
                values = [
                    float(value)
                    for value in re.search(r"actuators:\s*\[([^]]+)\]", text)[1].split(
                        ","
                    )
                ]
                sample = {
                    "timestamp": scalar(text, "timestamp"),
                    "actuators": values,
                    "fitness": scalar(text, "fitness"),
                    "source_system": scalar(text, "source_system"),
                    "source_component": scalar(text, "source_component"),
                }
                pump(0.2)
                matching = [
                    row
                    for row in allocations()
                    if len(values) == 9
                    and all(
                        math.isclose(a, b, rel_tol=1e-6, abs_tol=0.002)
                        for a, b in zip(values, row["actuators"])
                    )
                    and math.isclose(
                        sample["fitness"], row["fitness"], rel_tol=1e-6, abs_tol=0.002
                    )
                ]
                if (
                    not matching
                    or sample["source_system"] != 245
                    or sample["source_component"] != 191
                ):
                    raise AssertionError(
                        "PX4 allocation does not match the companion output"
                    )
                sample["allocator_output"] = matching[-1]
                samples.append(sample)
                pump(1.2)
            if samples[1]["timestamp"] <= samples[0]["timestamp"]:
                raise AssertionError("PX4 allocation topic did not advance")
            second_sensor = scalar(topic("sensor_combined"), "timestamp")
            if second_sensor <= first_sensor:
                raise AssertionError("PX4 simulated sensors did not advance")
            position = topic("vehicle_local_position")
            result.update(
                samples=samples,
                sensor_timestamps_us=[first_sensor, second_sensor],
                airborne_ned_z_m=scalar(position, "z"),
            )
            command("commander land")
            wait_for(
                lambda: "Disarmed by landing" in plain(console),
                35,
                "landing and automatic disarm",
            )
            topic("vehicle_land_detected")
            result.update(
                passed=True,
                allocation_count=len(allocations()),
                landing_and_disarm=True,
            )
            # Close the serial client before shutdown closes PX4's UDP endpoint.
            for process in reversed(processes[1:]):
                terminate(process)
            command("shutdown")
            pump(2)
        except (Exception, KeyboardInterrupt) as error:
            result["error"] = str(error) or type(error).__name__
        finally:
            for process in reversed(processes):
                terminate(process)
            heartbeat.close()
            os.close(master)
            link.unlink(missing_ok=True)
            (OUTPUT / "px4.txt").write_text(plain(console))
            (OUTPUT / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
