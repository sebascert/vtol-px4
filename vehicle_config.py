#!/usr/bin/env python3
"""Validate an explicitly selected vehicle file and stage a build-local snapshot."""
import argparse
import hashlib
import json
import math
from pathlib import Path

from jsonschema import Draft202012Validator, ValidationError

SCHEMA = Path(__file__).with_name("vehicle.schema.json")


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON field: {key}")
        result[key] = value
    return result


def reject_constant(value):
    raise ValueError(f"Nonfinite JSON number: {value}")


def finite_numbers(value, path="vehicle"):
    if isinstance(value, dict):
        for key, child in value.items():
            finite_numbers(child, f"{path}.{key}")
    elif isinstance(value, list):
        for i, child in enumerate(value):
            finite_numbers(child, f"{path}[{i}]")
    elif isinstance(value, (int, float)):
        try:
            finite = math.isfinite(value)
        except OverflowError:
            finite = False
        if not finite:
            raise ValueError(f"{path}: number must be finite and representable")


def validate(vehicle):
    finite_numbers(vehicle)
    schema = json.loads(SCHEMA.read_text())
    Draft202012Validator(schema).validate(vehicle)

    inertia = vehicle["rigid_body"]["inertia_kg_m2"]
    if any(value <= 0 for value in inertia):
        raise ValueError("rigid_body.inertia_kg_m2: moments must be positive")
    for i, value in enumerate(inertia):
        if value > sum(inertia[j] for j in range(3) if j != i):
            raise ValueError("rigid_body.inertia_kg_m2: violates the inertia triangle inequality")

    coefficients = vehicle["propulsion"]["k_t_n_per_command"]
    if any(value <= 0 for value in coefficients):
        raise ValueError("propulsion.k_t_n_per_command: coefficients must be positive")

    channels = set()
    for i, actuator in enumerate(vehicle["actuators"]):
        name = actuator["name"]
        lower, upper = actuator["min"], actuator["max"]
        if lower >= upper:
            raise ValueError(f"{name}: min must be below max")
        for key in ("neutral", "disarmed"):
            if not lower <= actuator[key] <= upper:
                raise ValueError(f"{name}.{key}: outside actuator limits")
        if i < 3 and (lower != 0 or actuator["disarmed"] != 0):
            raise ValueError(f"{name}: non-reversible motors require zero minimum and disarmed commands")
        if i in (3, 4) and not lower <= 0 <= upper:
            raise ValueError(f"{name}: tilt limits must permit vertical rotors (0 rad)")
        channel = actuator["hil_channel"]
        if channel in channels:
            raise ValueError(f"Duplicate HIL channel: {channel}")
        channels.add(channel)

    capacity = sum(coefficients[i] * vehicle["actuators"][i]["max"] for i in range(3))
    weight = vehicle["rigid_body"]["mass_kg"] * vehicle["environment"]["gravity_m_s2"]
    if not math.isfinite(capacity) or not math.isfinite(weight):
        raise ValueError("Derived thrust and weight must be finite")
    if capacity <= weight:
        raise ValueError("Motor thrust capacity must exceed weight for the reference hover envelope")
    return vehicle


def load(path):
    with Path(path).open() as source:
        vehicle = json.load(source, object_pairs_hook=unique_object, parse_constant=reject_constant)
    return validate(vehicle)


def stage(vehicle, output_dir):
    data = (json.dumps(vehicle, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
    digest = hashlib.sha256(data).hexdigest()
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "vehicle.json").write_bytes(data)
    (output_dir / "vehicle.sha256").write_text(f"{digest}  vehicle.json\n")
    return digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path, help="explicit path to the vehicle JSON")
    parser.add_argument("--output-dir", type=Path, help="write the validated snapshot and SHA-256 here")
    args = parser.parse_args()
    try:
        vehicle = load(args.config)
        digest = stage(vehicle, args.output_dir) if args.output_dir else None
    except ValidationError as error:
        path = ".".join(map(str, error.absolute_path)) or "vehicle"
        parser.exit(1, f"Invalid vehicle configuration at {path}: {error.message}\n")
    except (OSError, ValueError) as error:
        parser.exit(1, f"Invalid vehicle configuration: {error}\n")
    print(f"Validated {args.config.resolve()} ({vehicle['metadata']['calibration_status']})")
    if digest:
        print(f"Vehicle snapshot: {args.output_dir / 'vehicle.json'}; SHA-256: {digest}")


if __name__ == "__main__":
    main()
