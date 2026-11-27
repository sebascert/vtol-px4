"""Run the real pose-target allocator -> PX4 -> custom Gazebo plant loop."""

import argparse
import fcntl
import hashlib
import json
import math
import os
import pty
import runpy
import select
import signal
import statistics
import subprocess
import tempfile
import time
from contextlib import ExitStack
from pathlib import Path

from gazebo_sensors import plain, scalar, vector
from pymavlink import mavutil

CONTROLLER = Path(__file__).resolve().parents[1]
REPO = CONTROLLER.parent
OUTPUT = CONTROLLER / "build/gazebo-closed-loop"


def run(vehicle, loss, OUTPUT):
    config = json.loads(vehicle.read_text())
    OUTPUT.mkdir(parents=True, exist_ok=True)
    bridge_output = OUTPUT / "world"
    bridge_status = bridge_output / "status.json"
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
    )
    processes = []
    console = bytearray()
    samples = []
    latest = {}
    goal = None
    references = loss != "waypoints"
    phase = "initialization"
    next_reference = 0
    next_heartbeat = 0
    descent = 0
    result = {
        "passed": False,
        "scope": "Desktop PX4 profile 22001, Python HIL bridge, companion PSO, Gazebo plant",
        **({"scenario": "waypoints"} if loss == "waypoints" else {"loss_case": loss}),
        "vehicle_sha256": hashlib.sha256(
            (CONTROLLER / "build/sitl-hil/vehicle.json").read_bytes()
        ).hexdigest(),
    }
    (OUTPUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")

    def terminate_signal(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate_signal)
    with ExitStack() as files:
        master, slave_fd = pty.openpty()
        files.callback(os.close, master)
        slave = files.enter_context(os.fdopen(slave_fd, "r+b", buffering=0))
        connection = mavutil.mavlink_connection(
            "udpin:127.0.0.1:0", source_system=255, source_component=190
        )
        files.callback(connection.close)
        hil_log = files.enter_context((OUTPUT / "hil.log").open("w"))
        pty_dir = Path(
            files.enter_context(tempfile.TemporaryDirectory(prefix="vtol-sil-"))
        )
        pxlog = files.enter_context((OUTPUT / "px4.log").open("wb"))
        allocatorlog = files.enter_context((OUTPUT / "allocator.jsonl").open("w"))
        allocatorerr = files.enter_context((OUTPUT / "allocator.stderr").open("w"))
        bridgefile = files.enter_context((OUTPUT / "serial.log").open("w"))
        trace = files.enter_context((OUTPUT / "state.jsonl").open("w"))

        def pump(seconds=0.1):
            nonlocal next_reference, next_heartbeat, goal
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                now = time.monotonic()
                while (message := connection.recv_match(blocking=False)) is not None:
                    if message.get_srcSystem() != 1:
                        continue
                    latest[message.get_type()] = message
                    if (
                        loss == "waypoints"
                        and message.get_type() == "POSITION_TARGET_LOCAL_NED"
                    ):
                        goal = [message.x, message.y, message.z]
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

        def serial_pair(name):
            ends = [pty_dir / f"{name}-{i}" for i in range(2)]
            processes.append(
                subprocess.Popen(
                    ["socat", *(f"PTY,link={end},rawer,echo=0" for end in ends)],
                    stdout=bridgefile,
                    stderr=bridgefile,
                    start_new_session=True,
                )
            )
            wait(lambda: all(end.exists() for end in ends), 5, f"{name} PTY pair")
            return [end.resolve(strict=True) for end in ends]

        def ground_truth():
            # Query Gazebo directly while pump() keeps Offboard setpoints flowing.
            with (OUTPUT / f"{phase}-gazebo-pose.json").open("w+") as evidence:
                query = subprocess.Popen(
                    [
                        "gz",
                        "topic",
                        "-t",
                        "/world/pvtol_world/pose/info",
                        "-e",
                        "-n",
                        "1",
                        "--json-output",
                    ],
                    env=env,
                    stdout=evidence,
                    stderr=hil_log,
                )
                try:
                    wait(lambda: query.poll() is not None, 8, "Gazebo ground truth")
                    if query.returncode:
                        raise RuntimeError("Gazebo pose query failed; inspect hil.log")
                finally:
                    if query.poll() is None:
                        query.kill()
                    query.wait()
                evidence.seek(0)
                poses = json.load(evidence)["pose"]
            pos = next(
                p["position"] for p in poses if p.get("name") == "controlled_drone"
            )
            # Gazebo world ENU to the NED frame used by the tracking assertions.
            return pos.get("y", 0), pos.get("x", 0), -pos.get("z", 0)

        def bridge_ready():
            if not bridge_status.exists():
                return False
            status = json.loads(bridge_status.read_text())
            return status["ready"] and status["imu"] > 0 and status["gps"] > 0

        try:
            usb_bridge, usb_px4 = serial_pair("usb")
            link, companion_px4 = serial_pair("companion")
            run = Path(tempfile.mkdtemp(prefix="rootfs-", dir=OUTPUT))
            processes.append(
                subprocess.Popen(
                    [
                        str(CONTROLLER / "run-gazebo-pso.sh"),
                        "--usb",
                        str(usb_px4),
                        "--companion",
                        str(companion_px4),
                        "-w",
                        str(run),
                    ],
                    cwd=REPO,
                    env=env,
                    stdin=slave,
                    stdout=slave,
                    stderr=slave,
                    start_new_session=True,
                )
            )
            slave.close()
            wait(
                lambda: "Startup script returned successfully" in plain(console),
                75,
                "PX4 startup",
            )
            bridge_status.unlink(missing_ok=True)
            processes.append(
                subprocess.Popen(
                    [
                        str(REPO / "simulation/run-hil.sh"),
                        "--port",
                        str(usb_bridge),
                        "--headless",
                        *(["--waypoints"] if loss == "waypoints" else []),
                        "--seed",
                        "42",
                        "--gcs",
                        f"127.0.0.1:{connection.port.getsockname()[1]}",
                        "--output-dir",
                        str(bridge_output),
                    ],
                    env=env,
                    stdout=hil_log,
                    stderr=hil_log,
                    start_new_session=True,
                )
            )
            wait(bridge_ready, 90, "Python HIL bridge readiness")
            wait(lambda: "LOCAL_POSITION_NED" in latest, 30, "position telemetry")
            pump(15)
            position = topic("vehicle_local_position")
            for flag in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid"):
                assert f"{flag}: True" in position, ("EKF not ready", flag)
            assert scalar(topic("vehicle_status"), "hil_state") == 1
            local = latest["LOCAL_POSITION_NED"]
            goal = [local.x, local.y, local.z]
            initial_truth = ground_truth()
            phase = "prearm"
            pump(1)
            command("commander mode offboard")
            pump(1)
            assert scalar(topic("vehicle_status"), "nav_state") == 14, (
                "Offboard rejected"
            )
            allocator = subprocess.Popen(
                [
                    str(REPO / "companion/allocator/build/vtol_allocator"),
                    "--port",
                    str(link),
                    "--baud",
                    "230400",
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
            if loss == "waypoints":
                from waypoint_demo import verify_flight

                def set_phase(value):
                    nonlocal phase
                    phase = value

                result.update(verify_flight(OUTPUT, pump, command, topic, set_phase))
                return result
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
            truth = ground_truth()
            actual_rise = initial_truth[2] - truth[2]
            assert actual_rise > 0.15, ("No physical rise", actual_rise)
            result["gazebo_rise_m"] = actual_rise
            result["applied_source_age_ms"] = (
                scalar(applied, "timestamp") - scalar(applied, "timestamp_sample")
            ) / 1000
            assert result["applied_source_age_ms"] < 250
            status = json.loads(bridge_status.read_text())
            assert status["ready"] and status["controls"] > 0, (
                "Missing HIL actuator traffic"
            )
            result["bridge_status"] = status
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
            truth = ground_truth()
            displacement = [truth[i] - initial_truth[i] for i in (0, 1)]
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
            for p in reversed(processes):
                if p.poll() is None:
                    try:
                        os.killpg(p.pid, signal.SIGCONT)
                        os.killpg(p.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    try:
                        # The HIL launcher gives its Gazebo child five seconds to exit.
                        p.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(p.pid, signal.SIGKILL)
                        p.wait()
            (OUTPUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--waypoint-demo",
        action="store_true",
        help="record a measured desktop waypoint flight",
    )
    args = parser.parse_args()
    vehicle = Path.home() / ".local/share/vtol26/config/vehicle.json"
    snapshot = CONTROLLER / "build/sitl-hil/vehicle.json"
    if not snapshot.exists():
        parser.error("Build desktop HIL PX4 first: ./controller/build.sh sitl")
    if json.loads(vehicle.read_text()) != json.loads(snapshot.read_text()):
        parser.error("Rebuild SITL with this vehicle configuration first")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    with (OUTPUT / ".run.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("Another closed-loop run is using this output directory")
        if args.waypoint_demo:
            directory = Path(
                tempfile.mkdtemp(prefix="waypoint-flight-", dir=CONTROLLER / "build")
            )
            print(f"Waypoint artifacts: {directory}", flush=True)
            print(
                json.dumps(run(vehicle, "waypoints", directory), indent=2), flush=True
            )
            subprocess.run(
                [
                    REPO / "simulation/build/venv/bin/python3",
                    REPO / "simulation/render_waypoints.py",
                    directory,
                ],
                check=True,
            )
            return
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
