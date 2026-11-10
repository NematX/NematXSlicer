"""Check that accidental seam raytracing stops the native test process.

The hidden Catch2 probes enter the plugin and native visibility paths without
permission. Exit code 71 and a named diagnostic prove rejection happened at the
guard, rather than in a later geometry failure. Empty native samples also test
the explicit permission tag and the disabled path without casting any rays.

    VisibilityGuardTests
        |-- run_probe(): start one bounded plugins_tests process
        |-- assert_blocked(): check the guard's exit code and diagnostic
        `-- test_permission_does_not_leak(): run authorized then forbidden cases
"""

import argparse
import os
from pathlib import Path
import subprocess
import sys
import unittest


EXECUTABLE = Path(__file__).resolve().parents[2] / "build/tests/plugins/Debug/plugins_tests.exe"


class VisibilityGuardTests(unittest.TestCase):
    """Run isolated probes so the guard can terminate a process unconditionally."""

    def run_probe(self, selection, section):
        """Return the completed bounded process, keeping stdout and stderr for diagnosis."""
        # Probes intentionally stop before their end record; never append to a caller's timing stream.
        environment = os.environ.copy()
        environment.pop("SLIC3R_TEST_TIMINGS", None)
        return subprocess.run(
            [str(EXECUTABLE), selection, "--order", "decl", "-c", section],
            cwd=EXECUTABLE.parent, env=environment, capture_output=True, timeout=30,
        )

    def assert_blocked(self, result, path):
        """Distinguish an intentional guard exit from a crash or ordinary Catch failure."""
        output = result.stdout + result.stderr
        self.assertEqual(result.returncode, 71, output.decode("utf-8", "replace"))
        self.assertIn(b"Forbidden seam visibility calculation", output)
        self.assertIn(path.encode(), output)
        self.assertIn(b"Seam visibility guard probe", output)

    def test_plugin_path_rejects_unmarked_visibility(self):
        """Exercise an applied regional override through the real Cost session factory."""
        self.assert_blocked(self.run_probe("Seam visibility guard probe", "plugin"), "plugin")

    def test_native_path_rejects_unmarked_visibility(self):
        """Enter the retained native raycasting routine with empty input, before any rays."""
        self.assert_blocked(self.run_probe("Seam visibility guard probe", "native"), "native")

    def test_model_path_rejects_unmarked_visibility(self):
        """Direct model construction checks permission before preparing geometry."""
        self.assert_blocked(self.run_probe("Seam visibility guard probe", "model"), "plugin")

    def test_worker_cannot_bypass_permission(self):
        """The policy belongs to the process, not just Catch2's main thread."""
        self.assert_blocked(self.run_probe("Seam visibility guard probe", "worker"), "plugin")

    def test_disabled_native_path_needs_no_permission(self):
        """A disabled native visibility calculation remains usable by ordinary tests."""
        result = self.run_probe("Seam visibility guard probe", "native disabled")
        self.assertEqual(result.returncode, 0, (result.stdout + result.stderr).decode("utf-8", "replace"))

    def test_marked_native_path_has_permission(self):
        """An explicitly authorized case completes normally instead of bypassing a failure."""
        result = self.run_probe("Seam visibility authorized probe", "native")
        self.assertEqual(result.returncode, 0, (result.stdout + result.stderr).decode("utf-8", "replace"))
        self.assertIn(b"Authorized seam visibility probe completed", result.stdout)

    def test_permission_does_not_leak(self):
        """Catch2's declaration order runs the marked probe before the forbidden one."""
        result = self.run_probe("Seam visibility authorized probe,Seam visibility guard probe", "native")
        self.assertIn(b"Authorized seam visibility probe completed", result.stdout)
        self.assert_blocked(result, "native")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("executable", type=Path, nargs="?", default=EXECUTABLE)
    arguments, unittest_arguments = parser.parse_known_args()
    EXECUTABLE = arguments.executable.resolve()
    unittest.main(argv=[sys.argv[0], *unittest_arguments])
