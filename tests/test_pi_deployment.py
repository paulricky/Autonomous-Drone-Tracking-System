"""Deployment argument/failure checks with mocked Docker; no devices opened."""
import json
import os
import platform
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class PiDeploymentTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        source = Path(__file__).resolve().parents[1]
        for relative in ("scripts/pi_deploy.sh", "scripts/pi_diagnostics.sh",
                         "src/robotics_bringup/config/hardware.yaml"):
            target = self.root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source / relative, target)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.log = self.root / "docker.jsonl"
        self.env = dict(os.environ, PATH=str(self.bin) + os.pathsep + os.environ["PATH"],
                        ROBOTICS_DATA_DIR=str(self.root / "data"),
                        MOCK_DOCKER_LOG=str(self.log), MOCK_ARCH="aarch64", MOCK_DAEMON="arm64")
        self.executable("uname", '#!/bin/sh\ncase "$1" in -s) echo "${MOCK_OS:-Linux}";; -m) echo "$MOCK_ARCH";; *) echo Linux "$MOCK_ARCH";; esac\n')
        self.executable("git", '#!/bin/sh\nif [ "$1" = rev-parse ]; then echo fixture-revision; fi\n')
        self.executable("docker", '''#!/usr/bin/env python3
import json, os, sys
a = sys.argv[1:]
with open(os.environ["MOCK_DOCKER_LOG"], "a") as f: f.write(json.dumps(a) + "\\n")
if a[0] == "info": print(os.environ["MOCK_DAEMON"])
elif a[:2] == ["image", "inspect"]: print("sha256:current")
elif a[:2] == ["container", "inspect"]:
    if not os.environ.get("MOCK_EXISTS"): sys.exit(1)
    print(os.environ.get("MOCK_EXISTING_IMAGE", "sha256:current"))
elif a[0] == "build":
    print("mock build evidence")
    sys.exit(int(os.environ.get("MOCK_BUILD_STATUS", "0")))
''')

    def executable(self, name, text):
        path = self.bin / name
        path.write_text(text)
        path.chmod(0o755)

    def run_script(self, *args):
        return subprocess.run(["bash", str(self.root / "scripts/pi_deploy.sh"), *args],
                              env=self.env, cwd="/tmp", text=True, capture_output=True, timeout=20)

    def calls(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def test_arm_build_platform_and_persistent_evidence(self):
        result = self.run_script("build")
        self.assertEqual(result.returncode, 0, result.stderr)
        build = next(call for call in self.calls() if call[0] == "build")
        self.assertEqual(build[build.index("--platform") + 1], "linux/arm64")
        logs = list((self.root / "data/logs").glob("build_*.log"))
        self.assertEqual(len(logs), 1)
        self.assertIn("mock build evidence", logs[0].read_text())

    def test_mismatched_architecture_never_builds(self):
        self.env["MOCK_DAEMON"] = "amd64"
        self.assertNotEqual(self.run_script("build").returncode, 0)
        self.assertFalse(any(call[0] == "build" for call in self.calls()))

    def test_failed_build_preserves_log_and_failure(self):
        self.env["MOCK_BUILD_STATUS"] = "7"
        self.assertEqual(self.run_script("build").returncode, 7)
        self.assertTrue(list((self.root / "data/logs").glob("build_*.log")))
        self.assertFalse((self.root / "data/build-info.txt").exists())

    def test_start_preserves_config_uses_init_readonly_mounts_and_no_devices(self):
        self.assertEqual(self.run_script("start").returncode, 0)
        config = self.root / "data/config/hardware.yaml"
        config.write_text("operator configuration retained\n")
        self.assertEqual(self.run_script("start").returncode, 0)
        self.assertEqual(config.read_text(), "operator configuration retained\n")
        run = next(call for call in self.calls() if call[0] == "run")
        self.assertIn("--init", run)
        self.assertNotIn("--device", run)
        self.assertIn(f"type=bind,src={self.root}/data/config,dst=/var/lib/robotics/config,readonly", run)

    def test_stale_container_image_cannot_silently_start(self):
        self.env.update(MOCK_EXISTS="1", MOCK_EXISTING_IMAGE="sha256:old")
        result = self.run_script("start")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("older image", result.stderr)
        self.assertFalse(any(call[0] in ("run", "start") for call in self.calls()))

    def test_physical_commands_reject_macos_before_device_access(self):
        self.env["MOCK_OS"] = "Darwin"
        for command in ("camera", "capture", "telemetry", "sensors", "diagnose"):
            result = self.run_script(command)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("Linux Pi host", result.stderr)
        self.assertFalse(self.calls())

    def test_preflight_is_bounded_read_only_service_query(self):
        self.env["ROBOTICS_QUERY_CONTAINER"] = "robotics-bench"
        self.assertEqual(self.run_script("preflight").returncode, 0)
        call = self.calls()[0]
        self.assertEqual(call[:2], ["exec", "robotics-bench"])
        self.assertIn("timeout 10s ros2 service call /preflight/check", call[-1])

    @unittest.skipUnless(platform.system() == "Linux" and os.geteuid() == 0,
                         "requires an isolated root Linux test container")
    def test_hardware_device_mappings_readonly_calibration_and_safe_launch_args(self):
        # Only fake character-device identities in an isolated /dev; mocked
        # Docker records arguments and never opens these devices.
        for variable, directory, suffix, target in (
                ("LEFT_CAMERA_DEVICE", "/dev/v4l/by-id", "left", "/dev/null"),
                ("RIGHT_CAMERA_DEVICE", "/dev/v4l/by-id", "right", "/dev/zero"),
                ("FLIGHT_CONTROLLER_DEVICE", "/dev/serial/by-id", "controller", "/dev/null")):
            path = Path(directory) / (self.root.name + "-" + suffix)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.symlink_to(target)
            self.addCleanup(path.unlink)
            self.env[variable] = str(path)
        result = self.run_script("sensors")
        self.assertEqual(result.returncode, 0, result.stderr)
        run = next(call for call in self.calls() if call[0] == "run")
        self.assertIn("--init", run)
        self.assertNotIn("--privileged", run)
        for variable in ("LEFT_CAMERA_DEVICE", "RIGHT_CAMERA_DEVICE", "FLIGHT_CONTROLLER_DEVICE"):
            device = self.env[variable]
            self.assertIn(device + ":" + device, run)
        self.assertIn(f"type=bind,src={self.root}/data/calibration,dst=/var/lib/robotics/calibration,readonly", run)
        self.assertEqual(run[-3:], ["bench_launch", "sensor_diagnostics.launch.py", "config:=/var/lib/robotics/config/hardware.yaml"])
        self.assertIn('exec ros2 launch robotics_bringup "$@"', run[-4])

    @unittest.skipUnless(platform.system() == "Linux", "Ubuntu diagnostic utilities required")
    def test_diagnostics_keep_collecting_after_usb_video_failures(self):
        self.executable("v4l2-ctl", "#!/bin/sh\nexit 6\n")
        self.executable("lsusb", "#!/bin/sh\nexit 7\n")
        result = self.run_script("diagnose")
        self.assertEqual(result.returncode, 0, result.stderr)
        report = Path(result.stdout.strip())
        self.assertIn("exited 6", (report / "cameras.txt").read_text())
        self.assertIn("exited 7", (report / "usb.txt").read_text())
        self.assertTrue((report / "bench_ros_graph.txt").is_file())


if __name__ == "__main__":
    unittest.main()
