"""Check build target selection and failure handling without installing tools."""

import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class BuildTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=ROOT / "build")
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)
        self.project = self.work / "controller"
        self.project.mkdir()
        shutil.copy2(ROOT / "build.sh", self.project / "build.sh")
        self.env = dict(os.environ, HOME=str(self.work))
        subprocess.run(
            ["bash", ROOT.parent / "config/install.sh"],
            env=self.env,
            check=True,
            capture_output=True,
        )
        self.vehicle = self.work / ".local/share/vtol26/config/vehicle.json"
        data = json.loads((ROOT.parent / "config/vehicle.json").read_text())
        data["metadata"]["revision"] = "selected-by-user"
        self.vehicle.write_text(json.dumps(data))
        self.bin = self.work / "bin"
        self.bin.mkdir()
        self.log = self.work / "commands.log"
        self.env = dict(
            self.env,
            PATH=f"{self.bin}:{os.environ['PATH']}",
            BUILD_TEST_LOG=str(self.log),
            BUILD_TEST_BIN=str(self.bin),
            BUILD_TEST_PYTHON=sys.executable,
            BUILD_TEST_PIP_FAIL="0",
            BUILD_TEST_CONFIG_FAIL="0",
        )
        commands = {
            "python3": """#!/usr/bin/env bash
printf 'python %s\\n' "$*" >> "$BUILD_TEST_LOG"
if [[ $1 == -m && $2 == venv ]]; then
    mkdir -p "$3/bin"
    cp "$BUILD_TEST_BIN/python3" "$3/bin/python3"
elif [[ $1 == -m && $2 == pip ]]; then
    exit "$BUILD_TEST_PIP_FAIL"
else
    exec "$BUILD_TEST_PYTHON" "$@"
fi
""",
            "cmake": """#!/usr/bin/env bash
printf 'cmake %s\\n' "$*" >> "$BUILD_TEST_LOG"
printf 'GZ_DISTRO=%s\\n' "${GZ_DISTRO:-}" >> "$BUILD_TEST_LOG"
if [[ $1 == -S ]]; then exit "$BUILD_TEST_CONFIG_FAIL"; fi
""",
        }
        for name, contents in commands.items():
            path = self.bin / name
            path.write_text(contents)
            path.chmod(0o755)

    def run_build(self, *args):
        return subprocess.run(
            ["bash", str(self.project / "build.sh"), *args],
            cwd=self.work,
            env=self.env,
            capture_output=True,
            text=True,
            check=False,
        )

    def test_targets_and_project_local_paths(self):
        for args, target, directory in [
            (("sitl",), "px4_sitl_hil", "sitl-hil"),
            (("sitl-native",), "px4_sitl_vtol", "sitl"),
            (("firmware",), "px4_fmu-v6c_default", "firmware"),
        ]:
            with self.subTest(target=target):
                self.log.write_text("")
                result = self.run_build(*args, "-DEXAMPLE=ON")
                self.assertEqual(result.returncode, 0, result.stderr)
                log = self.log.read_text()
                self.assertIn(f"-r {self.project}/requirements.txt", log)
                self.assertIn(f"-DCONFIG={target}", log)
                self.assertIn(f"--build {self.project}/build/{directory}", log)
                self.assertIn("-DEXAMPLE=ON", log)
                if directory == "sitl":
                    self.assertIn("GZ_DISTRO=harmonic", log)
                snapshot = self.project / f"build/{directory}/vehicle.json"
                self.assertEqual(
                    json.loads(snapshot.read_text())["metadata"]["revision"],
                    "selected-by-user",
                )
                self.assertTrue(snapshot.with_suffix(".sha256").is_file())
                self.assertFalse((self.work / "build").exists())

    def test_invalid_target_does_not_install(self):
        self.assertEqual(self.run_build("invalid").returncode, 2)
        self.assertFalse(self.log.exists())

    def test_explicit_target_is_required(self):
        for args in (
            (),
            ("--vehicle-config", str(self.vehicle)),
            ("--vehicle-config", str(self.vehicle), "sitl"),
        ):
            with self.subTest(args=args):
                result = self.run_build(*args)
                self.assertEqual(result.returncode, 2)
                self.assertIn("<sitl|sitl-native|firmware>", result.stderr)
                self.assertFalse(self.log.exists())

    def test_help_does_not_install(self):
        for args in (("--help",), ("-h",), ("sitl", "--help"), ("firmware", "-h")):
            with self.subTest(args=args):
                result = self.run_build(*args)
                self.assertEqual(result.returncode, 0)
                self.assertIn("Usage:", result.stdout)
                self.assertFalse(self.log.exists())

    def test_dependency_failure_prevents_configuration(self):
        self.env["BUILD_TEST_PIP_FAIL"] = "1"
        self.assertNotEqual(self.run_build("sitl").returncode, 0)
        self.assertNotIn("cmake", self.log.read_text())

    def test_configuration_failure_prevents_compilation(self):
        self.env["BUILD_TEST_CONFIG_FAIL"] = "1"
        self.assertNotEqual(self.run_build("sitl").returncode, 0)
        self.assertNotIn("cmake --build", self.log.read_text())

    def test_vehicle_path_override_is_rejected(self):
        for option in ("--vehicle-config", "--vehicle-config=other.json"):
            result = self.run_build("sitl", option, "other.json")
            self.assertEqual(result.returncode, 2)
            self.assertIn("~/.local/share/vtol26/config/vehicle.json", result.stderr)
            self.assertFalse(self.log.exists())

    def test_missing_installed_vehicle_prevents_cmake(self):
        self.vehicle.unlink()
        result = self.run_build("sitl")
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn("cmake", self.log.read_text())

    def test_invalid_vehicle_prevents_cmake(self):
        self.vehicle.write_text('{"schema_version": 99}')
        result = self.run_build("sitl")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Invalid vehicle configuration", result.stderr)
        self.assertNotIn("cmake", self.log.read_text())
        self.assertFalse((self.project / "build/sitl-hil/vehicle.json").exists())


if __name__ == "__main__":
    unittest.main()
