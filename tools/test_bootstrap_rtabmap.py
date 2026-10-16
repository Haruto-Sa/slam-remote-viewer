"""Read-only failure-path checks: these tests never download/build dependencies."""
import os
from pathlib import Path
import platform
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).with_name("bootstrap-rtabmap-macos.sh").resolve()
APPLE_SILICON = platform.system() == "Darwin" and subprocess.run(
    ["/usr/sbin/sysctl", "-n", "hw.optional.arm64"],
    text=True, capture_output=True, timeout=5).stdout.strip() == "1"


class BootstrapTests(unittest.TestCase):
    def run_script(self, *arguments, **environment):
        command = ["bash", str(SCRIPT), *arguments]
        if APPLE_SILICON:
            command = ["/usr/bin/arch", "-arm64", "/bin/bash", str(SCRIPT), *arguments]
        return subprocess.run(command, env={**os.environ, **environment},
                              text=True, capture_output=True, timeout=15)

    def test_help_does_not_require_conda(self):
        result = self.run_script("--help", SLAM_CONDA_EXE="/missing/conda")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Downloads dependencies/source", result.stdout)

    def test_unknown_option_is_rejected(self):
        result = self.run_script("--unknown")
        self.assertEqual(result.returncode, 2)
        self.assertIn("unexpected arguments", result.stderr)

    @unittest.skipUnless(APPLE_SILICON, "Apple Silicon bootstrap preflight")
    def test_invalid_jobs_and_unsafe_roots_fail_before_installing(self):
        for jobs in ("0", "-1", "33", "99999999999999999999999", "4x"):
            result = self.run_script(SLAM_BUILD_JOBS=jobs)
            self.assertIn("SLAM_BUILD_JOBS must be", result.stderr)
            self.assertNotEqual(result.returncode, 0)
        for root in ("/", "/private/tmp", "/tmp/../Users", "/tmp/test;directory"):
            result = self.run_script(SLAM_RTAB_WORK_ROOT=root, SLAM_BUILD_JOBS="1")
            self.assertIn("work root", result.stderr)
            self.assertNotEqual(result.returncode, 0)

    @unittest.skipUnless(APPLE_SILICON, "Apple Silicon bootstrap preflight")
    def test_missing_conda_does_not_create_environment(self):
        with tempfile.TemporaryDirectory(dir="/tmp", prefix="rtab-preflight-") as temporary:
            root = Path(temporary)
            result = self.run_script(SLAM_RTAB_WORK_ROOT=str(root),
                                     SLAM_CONDA_EXE="/missing/conda", SLAM_BUILD_JOBS="1")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("conda executable not found", result.stderr)
            self.assertEqual(list(root.iterdir()), [])

    @unittest.skipUnless(APPLE_SILICON, "Apple Silicon bootstrap preflight")
    def test_existing_non_repository_is_not_overwritten(self):
        with tempfile.TemporaryDirectory(dir="/tmp", prefix="rtab-preflight-") as temporary:
            root = Path(temporary)
            source = root / "rtabmap"
            source.mkdir()
            marker = source / "user-data.txt"
            marker.write_text("keep me")
            result = self.run_script(SLAM_RTAB_WORK_ROOT=str(root),
                                     SLAM_CONDA_EXE="/usr/bin/true", SLAM_BUILD_JOBS="1")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("not a Git checkout", result.stderr)
            self.assertEqual(marker.read_text(), "keep me")
            self.assertFalse((root / "env").exists())

    @unittest.skipUnless(APPLE_SILICON, "Apple Silicon bootstrap preflight")
    def test_symlink_cannot_redirect_dependency_install(self):
        with tempfile.TemporaryDirectory(dir="/tmp", prefix="rtab-preflight-") as temporary:
            root = Path(temporary)
            target = root / "other-environment"
            target.mkdir()
            (root / "env").symlink_to(target)
            result = self.run_script(SLAM_RTAB_WORK_ROOT=str(root),
                                     SLAM_CONDA_EXE="/usr/bin/true", SLAM_BUILD_JOBS="1")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("must not be a symlink", result.stderr)
            self.assertEqual(list(target.iterdir()), [])
            self.assertFalse((root / "rtabmap").exists())


if __name__ == "__main__":
    unittest.main()
