"""Check exporter-child trace isolation and the local runner's strict TSV reader.

Each exporter parent runs in a fresh native process with its own timing file.
Exactly one start/end pair must survive there, while the native parent still
checks its child's exit status. Synthetic logs separately exercise rejection
of mixed processes, child names, duplicates, partial pairs and failed cases.
The runner is local build tooling; --runner selects its existing implementation.
CTest runs the exporter integration class without depending on that local file.

    ExporterChildTimingTests
        `-- test_parent_only_trace(): list and run each parent, inspect its raw TSV
    TimingReaderTests
        |-- read(): give an owned temporary TSV to the actual runner reader
        `-- test_*(): assert valid records or rejection, without repairing logs
"""

import argparse
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[2]
EXECUTABLE = REPO / "build/tests/plugins/Debug/plugins_tests.exe"
RUNNER = REPO / "build/run_plugins_tests.py"
PARENTS = (
    "An unavailable default exporter is never replaced by another provider",
    "Bambu rejects an active PNG provider without image capability",
)


class ExporterChildTimingTests(unittest.TestCase):
    """Keep child assertions in their native process, never in the parent's trace."""

    def test_parent_only_trace(self):
        """Require one paired record from the actual parent PID for both probes."""
        for name in PARENTS:
            with self.subTest(name=name), tempfile.TemporaryDirectory(prefix="exporter-timings-") as directory:
                timings = Path(directory) / "parent.tsv"
                environment = os.environ.copy()
                environment["SLIC3R_TEST_TIMINGS"] = str(timings)
                environment.pop("SLIC3R_TEST_SIMPLE_EXPORTER_DISABLED", None)
                environment.pop("SLIC3R_TEST_BAMBU_TEXT_ONLY_PNG", None)
                listed = subprocess.run(
                    [str(EXECUTABLE), name, "--list-test-names-only"], cwd=EXECUTABLE.parent.parent,
                    env=environment, capture_output=True, text=True, timeout=60,
                )
                self.assertEqual(listed.stdout.splitlines(), [name], listed.stderr)
                self.assertFalse(timings.exists(), "Listing must not execute the parent or its child")
                with subprocess.Popen(
                    [str(EXECUTABLE), name, "-r", "compact"], cwd=EXECUTABLE.parent.parent,
                    env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                ) as process:
                    try:
                        output, _ = process.communicate(timeout=360)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        output, _ = process.communicate()
                        self.fail(f"Parent timed out: {output.decode('utf-8', 'replace')}")
                    self.assertEqual(process.returncode, 0, output.decode("utf-8", "replace"))
                    records = [line.split("\t") for line in timings.read_text(encoding="utf-8").splitlines()]
                    self.assertEqual(len(records), 2, records)
                    for record in records:
                        self.assertEqual(len(record), 10, records)
                        self.assertEqual(record[1], name)
                        self.assertEqual(int(record[9]), process.pid)
                        self.assertEqual(int(record[7]), 0)
                    self.assertEqual([record[0] for record in records], ["started", "ended"])
                    self.assertEqual([record[5] for record in records], ["started", "passed"])
                    self.assertGreater(int(records[1][6]), 0)


class TimingReaderTests(unittest.TestCase):
    """Exercise the runner itself; no alternative permissive parser is used."""

    @classmethod
    def setUpClass(cls):
        """Load the selected local runner without executing its command line."""
        spec = importlib.util.spec_from_file_location("plugins_test_runner", RUNNER)
        cls.runner = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.runner)

    def read(self, lines, names=("parent",)):
        """Write exact records and return the real reader's result or exception."""
        with tempfile.TemporaryDirectory(prefix="timing-reader-") as directory:
            path = Path(directory) / "timings.tsv"
            path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            return self.runner.read_timings(path, names)

    @staticmethod
    def pair(name="parent", pid=123):
        """A successful pair with explicit identity and assertion counts."""
        return [f"started\t{name}\ttest.cpp\t10\t0\tstarted\t0\t0\t0\t{pid}",
                f"ended\t{name}\ttest.cpp\t10\t25\tpassed\t2\t0\t0\t{pid}"]

    def test_valid_pair(self):
        """Only one measured duration is returned for the expected parent."""
        self.assertEqual(self.read(self.pair()), {"parent": 0.025})

    def test_mixed_processes(self):
        """Even a paired expected name from another PID must be rejected."""
        with self.assertRaisesRegex(ValueError, "mixes processes"):
            self.read(self.pair() + self.pair("second", 456), ("parent", "second"))

    def test_unexpected_child_name(self):
        """A foreign name is invalid even if it shares the parent's PID."""
        with self.assertRaisesRegex(ValueError, "invalid timing record"):
            self.read(self.pair() + self.pair("child"))

    def test_duplicate_pair(self):
        """Repeated successful records cannot inflate measured coverage."""
        with self.assertRaisesRegex(ValueError, "duplicate"):
            self.read(self.pair() + self.pair())

    def test_incomplete_pair(self):
        """A missing end cannot be accepted as a successful measured run."""
        with self.assertRaisesRegex(ValueError, "incomplete"):
            self.read(self.pair()[:1])

    def test_end_without_start(self):
        """An end alone is not a completed test."""
        with self.assertRaisesRegex(ValueError, "unpaired"):
            self.read(self.pair()[1:])

    def test_failed_case(self):
        """A failure must not update duration weights."""
        lines = self.pair()
        lines[1] = lines[1].replace("\tpassed\t2\t0\t", "\tfailed\t1\t1\t")
        with self.assertRaisesRegex(ValueError, "unsuccessful"):
            self.read(lines)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("--runner", type=Path, default=RUNNER)
    arguments, remaining = parser.parse_known_args()
    EXECUTABLE = arguments.executable.resolve()
    RUNNER = arguments.runner.resolve()
    unittest.main(argv=[sys.argv[0], *remaining])
