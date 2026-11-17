"""Check build target selection and failure handling without installing tools."""
import os
import shutil
import subprocess
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
        self.bin = self.work / "bin"
        self.bin.mkdir()
        self.log = self.work / "commands.log"
        self.env = dict(os.environ, PATH=f"{self.bin}:{os.environ['PATH']}",
                        BUILD_TEST_LOG=str(self.log), BUILD_TEST_BIN=str(self.bin),
                        BUILD_TEST_PIP_FAIL="0", BUILD_TEST_CONFIG_FAIL="0")
        commands = {
            "python3": '''#!/usr/bin/env bash
printf 'python %s\\n' "$*" >> "$BUILD_TEST_LOG"
if [[ $1 == -m && $2 == venv ]]; then
    mkdir -p "$3/bin"
    cp "$BUILD_TEST_BIN/python3" "$3/bin/python3"
else
    exit "$BUILD_TEST_PIP_FAIL"
fi
''',
            "cmake": '''#!/usr/bin/env bash
printf 'cmake %s\\n' "$*" >> "$BUILD_TEST_LOG"
if [[ $1 == -S ]]; then exit "$BUILD_TEST_CONFIG_FAIL"; fi
''',
        }
        for name, contents in commands.items():
            path = self.bin / name
            path.write_text(contents)
            path.chmod(0o755)

    def run_build(self, *args):
        return subprocess.run(["bash", str(self.project / "build.sh"), *args],
                              cwd=self.work, env=self.env, capture_output=True, text=True)

    def test_targets_and_project_local_paths(self):
        for args, target, directory in [((), "px4_sitl_vtol", "sitl"),
                                         (("firmware",), "px4_fmu-v6c_default", "firmware")]:
            with self.subTest(target=target):
                self.log.write_text("")
                self.assertEqual(self.run_build(*args).returncode, 0)
                log = self.log.read_text()
                self.assertIn(f"-r {self.project}/requirements.txt", log)
                self.assertIn(f"-DCONFIG={target}", log)
                self.assertIn(f"--build {self.project}/build/{directory}", log)
                self.assertFalse((self.work / "build").exists())

    def test_invalid_target_does_not_install(self):
        self.assertEqual(self.run_build("invalid").returncode, 2)
        self.assertFalse(self.log.exists())

    def test_dependency_failure_prevents_configuration(self):
        self.env["BUILD_TEST_PIP_FAIL"] = "1"
        self.assertNotEqual(self.run_build().returncode, 0)
        self.assertNotIn("cmake", self.log.read_text())

    def test_configuration_failure_prevents_compilation(self):
        self.env["BUILD_TEST_CONFIG_FAIL"] = "1"
        self.assertNotEqual(self.run_build().returncode, 0)
        self.assertNotIn("cmake --build", self.log.read_text())


if __name__ == "__main__":
    unittest.main()
