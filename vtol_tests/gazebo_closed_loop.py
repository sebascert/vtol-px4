#!/usr/bin/env python3
"""Run the real pose-target allocator -> PX4 -> custom Gazebo plant loop."""

import argparse
from contextlib import ExitStack
import hashlib
import json
import math
import os
from pathlib import Path
import pty
import runpy
import select
import signal
import socket
import statistics
import subprocess
import time
import xml.etree.ElementTree as ET
from pymavlink import mavutil
from gazebo_sensors import plain, scalar, vector

CONTROLLER = Path(__file__).resolve().parents[1]
REPO = CONTROLLER.parent
OUTPUT = CONTROLLER / "build/gazebo-closed-loop"


def run(vehicle, loss, OUTPUT):
    config = json.loads(vehicle.read_text())
    for port in (14540, 14550, 14580, 18570):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", port))
    OUTPUT.mkdir(parents=True, exist_ok=True)
    prepared = OUTPUT / "world"
    subprocess.run(
        [
            REPO / "simulation/build/vtol_prepare",
            vehicle,
            REPO / "simulation",
            prepared,
        ],
        check=True,
    )
    # A real ground start, not a kinematic hold or injected lift force.
    world = ET.parse(prepared / "pvtol.sdf")
    radius = float(
        ET.parse(prepared / "models/stallion_vtol/model.sdf").findtext(
            "model/link/collision/geometry/sphere/radius"
        )
    )
    pose = world.find("world/include/pose")
    values = pose.text.split()
    values[2] = str(radius)
    pose.text = " ".join(values)
    world.write(prepared / "ground-start.sdf", encoding="utf-8", xml_declaration=True)
    hil_config = runpy.run_path(str(REPO / "companion/allocator/prepare_hil.py"))[
        "configuration"
    ]
    tuning = hil_config(
        config,
        json.loads((REPO / "companion/allocator/src/pso/config.json").read_text()),
    )
    (OUTPUT / "pso.json").write_text(json.dumps(tuning, indent=2) + "\n")
    env = dict(
        os.environ,
        GZ_PARTITION=f"vtol-loop-{os.getpid()}-{loss}",
        GZ_IP="127.0.0.1",
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(REPO / "simulation/build"),
        GZ_SIM_RESOURCE_PATH=str(prepared / "models"),
    )
    processes = []
    console = bytearray()
    samples = []
    latest = {}
    goal = None
    references = True
    phase = "initialization"
    next_reference = 0
    next_heartbeat = 0
    descent = 0
    result = {
        "passed": False,
        "scope": "Real companion PSO, PX4 gated outputs, custom dynamic Gazebo plant; no physical hardware",
        "loss_case": loss,
        "vehicle_sha256": hashlib.sha256(
            (CONTROLLER / "build/sitl/vehicle.json").read_bytes()
        ).hexdigest(),
    }
    (OUTPUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    master, slave = pty.openpty()
    connection = mavutil.mavlink_connection(
        "udpin:127.0.0.1:14550", source_system=255, source_component=190
    )

    def terminate_signal(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate_signal)
    with ExitStack() as files:
        gzlog = files.enter_context((OUTPUT / "gazebo.log").open("w"))
        pxlog = files.enter_context((OUTPUT / "px4.log").open("wb"))
        allocatorlog = files.enter_context((OUTPUT / "allocator.jsonl").open("w"))
        allocatorerr = files.enter_context((OUTPUT / "allocator.stderr").open("w"))
        bridgefile = files.enter_context((OUTPUT / "serial.log").open("w"))
        trace = files.enter_context((OUTPUT / "state.jsonl").open("w"))

        def pump(seconds=0.1):
            nonlocal next_reference, next_heartbeat
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                now = time.monotonic()
                while (message := connection.recv_match(blocking=False)) is not None:
                    if message.get_srcSystem() != 1:
                        continue
                    latest[message.get_type()] = message
                    if message.get_type() == "LOCAL_POSITION_NED":
                        row = {
                            "wall_time": now,
                            "boot_ms": message.time_boot_ms,
                            "phase": phase,
                            "position": [message.x, message.y, message.z],
                            "velocity": [message.vx, message.vy, message.vz],
                            "target": goal,
                        }
                        samples.append(row)
                        trace.write(json.dumps(row) + "\n")
                        trace.flush()
                if now >= next_heartbeat:
                    connection.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0,
                        0,
                        0,
                    )
                    next_heartbeat = now + 1
                if references and goal is not None and now >= next_reference:
                    connection.mav.set_position_target_local_ned_send(
                        0,
                        1,
                        1,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        2520 if descent else 2552,
                        *goal,
                        0,
                        0,
                        descent,
                        0,
                        0,
                        0,
                        0,
                        0,
                    )
                    next_reference = now + 0.02
                if select.select([master], [], [], 0.005)[0]:
                    try:
                        data = os.read(master, 65536)
                    except OSError:
                        data = b""
                    console.extend(data)
                    pxlog.write(data)
                    pxlog.flush()
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError("A process exited; inspect logs")

        def wait(predicate, seconds, label):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                pump()
                if predicate():
                    return
            raise TimeoutError(label)

        def command(value):
            mark = len(console)
            os.write(master, (value + "\n").encode())
            return mark

        def topic(name):
            mark = command("listener " + name)
            wait(
                lambda: (
                    "TOPIC: " + name in plain(console[mark:])
                    and "\npxh>" in plain(console[mark:]).split("TOPIC: " + name, 1)[1]
                ),
                8,
                name,
            )
            text = plain(console[mark:])
            (OUTPUT / (phase + "-" + name + ".txt")).write_text(text)
            return text

        def outputs():
            return topic("pso_actuator_outputs")

        def is_active(text):
            return "active: True" in text

        try:
            processes.append(
                subprocess.Popen(
                    [
                        "gz",
                        "sim",
                        "-r",
                        "-s",
                        "--seed",
                        "42",
                        str(prepared / "ground-start.sdf"),
                    ],
                    env=env,
                    stdout=gzlog,
                    stderr=gzlog,
                    start_new_session=True,
                )
            )
            run = OUTPUT / f"rootfs-{os.getpid()}"
            run.mkdir(exist_ok=True)
            processes.append(
                subprocess.Popen(
                    [str(CONTROLLER / "run-gazebo-pso.sh"), "-w", str(run)],
                    cwd=REPO,
                    env=env,
                    stdin=slave,
                    stdout=slave,
                    stderr=slave,
                    start_new_session=True,
                )
            )
            os.close(slave)
            wait(
                lambda: "Startup script returned successfully" in plain(console),
                75,
                "PX4 startup",
            )
            wait(lambda: "LOCAL_POSITION_NED" in latest, 30, "position telemetry")
            pump(12)
            local = latest["LOCAL_POSITION_NED"]
            goal = [local.x, local.y, local.z]
            initial_truth = topic("vehicle_local_position_groundtruth")
            phase = "prearm"
            pump(1)
            command("commander mode offboard")
            pump(1)
            assert scalar(topic("vehicle_status"), "nav_state") == 14, (
                "Offboard rejected"
            )
            link = OUTPUT / "allocator.pty"
            link.unlink(missing_ok=True)
            processes.append(
                subprocess.Popen(
                    [
                        "socat",
                        f"PTY,link={link},rawer,echo=0",
                        "UDP4:127.0.0.1:14580,bind=127.0.0.1:14540",
                    ],
                    stdout=bridgefile,
                    stderr=bridgefile,
                    start_new_session=True,
                )
            )
            wait(link.exists, 5, "serial bridge")
            allocator = subprocess.Popen(
                [
                    str(REPO / "companion/allocator/build/vtol_allocator"),
                    "--port",
                    str(link),
                    "--baud",
                    "115200",
                    "--pso-config",
                    str(OUTPUT / "pso.json"),
                    "--loop-ms",
                    "2",
                    "--pso-period-ms",
                    "20",
                    "--pso-max-particles",
                    "96",
                    "--pso-max-iterations",
                    "15",
                    "--pso-external-cycles",
                    "2",
                ],
                stdout=allocatorlog,
                stderr=allocatorerr,
                start_new_session=True,
            )
            processes.append(allocator)
            pump(1)
            assert not is_active(outputs()), "Outputs active while disarmed"
            phase = "ground_arm_cycle"
            command("commander arm")
            pump(0.2)
            assert scalar(topic("vehicle_status"), "arming_state") == 2
            command("commander disarm")
            pump(0.3)
            assert scalar(topic("vehicle_status"), "arming_state") == 1
            assert not is_active(outputs()), "Disarm did not inhibit outputs"
            phase = "climb"
            initial_z = latest["LOCAL_POSITION_NED"].z
            goal = [goal[0], goal[1], initial_z - 0.3]
            command("commander arm")
            pump(1)
            assert scalar(topic("vehicle_status"), "arming_state") == 2, (
                "Arming rejected"
            )
            wait(lambda: is_active(outputs()), 8, "first gated allocation")
            pump(10)
            rows = [r for r in samples if r["phase"] == "climb"]
            final = [r for r in rows if r["wall_time"] > rows[-1]["wall_time"] - 2]
            error = math.sqrt(
                sum((r["position"][2] - goal[2]) ** 2 for r in final) / len(final)
            )
            horizontal_error = math.sqrt(
                sum(
                    (r["position"][0] - goal[0]) ** 2
                    + (r["position"][1] - goal[1]) ** 2
                    for r in final
                )
                / len(final)
            )
            rise = initial_z - min(r["position"][2] for r in rows)
            result.update(
                vertical_tracking_rmse_m=error,
                horizontal_tracking_rmse_m=horizontal_error,
                maximum_rise_m=rise,
            )
            topic("pso_allocation")
            applied = outputs()
            topic("actuator_motors")
            topic("actuator_servos")
            assert is_active(applied), "Consumer stopped during climb"
            assert rise > 0.15 and error < 0.15 and horizontal_error < 0.2, (
                "Tracking",
                rise,
                error,
                horizontal_error,
            )
            truth = topic("vehicle_local_position_groundtruth")
            actual_rise = scalar(initial_truth, "z") - scalar(truth, "z")
            assert actual_rise > 0.15, ("No physical rise", actual_rise)
            result["gazebo_rise_m"] = actual_rise
            result["applied_source_age_ms"] = (
                scalar(applied, "timestamp") - scalar(applied, "timestamp_sample")
            ) / 1000
            assert result["applied_source_age_ms"] < 250
            phase = "lateral"
            goal = [goal[0] + 0.2, goal[1], goal[2]]
            pump(8)
            rows = [r for r in samples if r["phase"] == "lateral"]
            final = [r for r in rows if r["wall_time"] > rows[-1]["wall_time"] - 2]
            error = math.sqrt(
                sum(
                    sum((p - t) ** 2 for p, t in zip(r["position"], goal))
                    for r in final
                )
                / len(final)
            )
            result["lateral_step_pose_rmse_m"] = error
            assert error < 0.15, ("Lateral pose tracking", error)
            assert is_active(outputs()), "Consumer stopped during lateral step"
            truth = topic("vehicle_local_position_groundtruth")
            displacement = [
                scalar(truth, axis) - scalar(initial_truth, axis) for axis in ("x", "y")
            ]
            result["gazebo_horizontal_displacement_ned_m"] = displacement
            assert 0.05 < displacement[0] < 0.35 and abs(displacement[1]) < 0.2, (
                "Physical lateral response",
                displacement,
            )
            allocator_rows = [
                json.loads(l)
                for l in (OUTPUT / "allocator.jsonl").read_text().splitlines()
                if l.endswith("}")
            ]
            result["allocations"] = len(allocator_rows)
            result["max_solver_ms"] = max(
                r["solve_duration_ms"] for r in allocator_rows
            )
            result["mean_solver_ms"] = statistics.mean(
                r["solve_duration_ms"] for r in allocator_rows
            )
            result["allocation_rate_hz"] = (
                (len(allocator_rows) - 1)
                * 1000
                / (
                    allocator_rows[-1]["source_state_boot_ms"]
                    - allocator_rows[0]["source_state_boot_ms"]
                )
            )
            result["predicted_converged_allocations"] = sum(
                r["position_converged"] for r in allocator_rows
            )
            assert result["predicted_converged_allocations"] > 0
            assert result["max_solver_ms"] < 250
            phase = loss + "_loss"
            if loss == "allocator":
                os.killpg(allocator.pid, signal.SIGSTOP)
            else:
                references = False
            pump(0.4)
            safe = outputs()
            assert not is_active(safe) and scalar(safe, "reason") == 4, (
                loss + "-loss gate"
            )
            controls = vector(safe, "controls")
            assert controls[:3] == [0, 0, 0]
            if loss == "allocator":
                os.killpg(allocator.pid, signal.SIGCONT)
            else:
                references = True
            pump(0.4)
            assert not is_active(outputs()), "Fault latch unexpectedly resumed outputs"
            phase = "landing_after_loss"
            # PX4's land detector requires a descent request in position-control mode.
            descent = 0.5
            goal = [goal[0], goal[1], initial_z]
            wait(
                lambda: (
                    "landed: True"
                    in [
                        line.strip()
                        for line in topic("vehicle_land_detected").splitlines()
                    ]
                ),
                15,
                "landing after " + loss + " loss",
            )
            phase = "disarm"
            command("commander disarm")
            pump(0.5)
            assert (
                not is_active(outputs())
                and scalar(topic("vehicle_status"), "arming_state") == 1
            )
            assert scalar(outputs(), "reason") == 1, "Disarm did not clear fault"
            result["passed"] = True
            result["checks"] = [
                "disarmed outputs",
                "offboard arming",
                "live PSO actuation",
                "vertical and lateral target steps",
                loss + " timeout",
                "fault latch",
                "disarm",
                "ground arm/disarm/rearm cycle",
                "fault cleared by disarm",
            ]
        except BaseException as error:
            result["error"] = repr(error)
            raise
        finally:
            (OUTPUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            for p in reversed(processes):
                if p.poll() is None:
                    try:
                        os.killpg(p.pid, signal.SIGCONT)
                        os.killpg(p.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    try:
                        p.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(p.pid, signal.SIGKILL)
                        p.wait()
            connection.close()
            os.close(master)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    vehicle = Path.home() / ".local/share/vtol26/config/vehicle.json"
    if json.loads(vehicle.read_text()) != json.loads(
        (CONTROLLER / "build/sitl/vehicle.json").read_text()
    ):
        parser.error("Rebuild SITL with this vehicle configuration first")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    summary = {"passed": False, "runs": {}}
    (OUTPUT / "result.json").write_text(json.dumps(summary, indent=2) + "\n")
    try:
        # Each loss starts from a fresh world, not a vehicle damaged by the other stop.
        for loss in ("allocator", "target"):
            summary["runs"][loss] = run(vehicle, loss, OUTPUT / loss)
        summary["passed"] = True
    finally:
        (OUTPUT / "result.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
