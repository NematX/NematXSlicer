"""Run the host-side same-Print fixture with this build's real speed editor.

Requires a host plugins_tests executable containing [external-grid-replay].
The fixture supplies marked paths; this isolates the speed editor and checkpoint
behavior, not pattern generation or GUI event handling. Full-package activation
and pattern generation are checked separately by smoke_export.py.
"""
import argparse
import os
from pathlib import Path
import subprocess
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--host-tests", type=Path, required=True)
parser.add_argument("--build", type=Path, default=Path(__file__).resolve().parents[3] / ".build")
args = parser.parse_args()
env = os.environ.copy()
env["SLIC3R_TEST_EXTERNAL_PLUGIN_REPOSITORY"] = str(args.build.resolve() / "test-data/infill_gridvarspeed/plugins")
env["SLIC3R_TEST_EXTERNAL_PLUGIN_IDS"] = "nematx_gridvarspeed_speed"
result = subprocess.run([str(args.host_tests.resolve()), "[external-grid-replay]", "--reporter", "compact"],
                        env=env, capture_output=True, text=True, errors="replace")
text = result.stdout + result.stderr
print(text)
if result.returncode or "No test cases matched" in text or "Passed 1 test case" not in text:
    sys.exit(result.returncode or 1)
