"""Exercise the Pixhawk HIL waypoint procedure with desktop PX4 as the operator."""

import json
import shutil
import time

from gazebo_sensors import scalar


def verify_flight(output, pump, command, topic, set_phase):
    world = output / "world"
    status_file = world / "waypoints.json"
    assert json.loads(status_file.read_text())["phase"] == "on pad"
    command("commander arm")
    checked = 0
    disarm_requested = False
    deadline = time.monotonic() + 180
    while time.monotonic() < deadline:
        pump(0.1)
        status = json.loads(status_file.read_text())
        phase = status["phase"]
        if status["error"]:
            raise RuntimeError(status["error"])
        for arrival in status["arrivals"][checked:]:
            set_phase(arrival["phase"])
            if "active: True" not in topic("pso_actuator_outputs"):
                raise RuntimeError(f"Allocator inactive at {arrival['phase']}")
            checked += 1
            print(f"Reached waypoint {arrival['phase']}", flush=True)
        set_phase(phase)
        if phase == "landed" and not disarm_requested:
            assert "landed: True" in {
                line.strip() for line in topic("vehicle_land_detected").splitlines()
            }
            command("commander disarm")
            disarm_requested = True
        if phase == "disarmed":
            assert checked == 5 and status["passed"]
            assert scalar(topic("vehicle_status"), "arming_state") == 1
            assert scalar(topic("pso_actuator_outputs"), "reason") == 1
            # Keep the confirmed disarmed state visible at the end of the replay.
            pump(0.6)
            status["end_monotonic_s"] = time.monotonic()
            shutil.copyfile(world / "waypoint-trace.csv", output / "waypoint-trace.csv")
            return status
    raise TimeoutError(f"Waypoint flight stopped progressing at {phase}")
