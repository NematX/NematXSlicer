"""Run the real external NematX firmware and Reader pressure export tests.

The host binary must contain [nematx-pressure]. No missing selection is accepted.
The optional native feature packages publish their definitions at startup; the
first lexical case scopes their execution before the focused pressure cases.
Temporary files stay under the qualification run, or build/temp/b5b otherwise.
    module body -> subprocess.run: load both providers and run the bounded native gate
"""
import argparse
import os
from pathlib import Path
import re
import subprocess
import sys
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--host-tests", type=Path, required=True)
parser.add_argument("--build", type=Path, default=Path(__file__).resolve().parents[3] / "build")
parser.add_argument("--selector", default="[nematx-pressure]")
parser.add_argument("--expected-cases", type=int, default=13)
parser.add_argument("--timeout", type=float, default=120)
parser.add_argument("--feature-fixtures", action="store_true", help="Initialize the native GridVarSpeed/CornerStretch packages")
args = parser.parse_args()
env = os.environ.copy()
env["SLIC3R_TEST_EXTERNAL_PLUGIN_REPOSITORY"] = str(args.build.resolve() / "test-data/nematx_firmware/plugins")
env["SLIC3R_TEST_EXTERNAL_PLUGIN_IDS"] = "nematx,gcode.reader.nematx"
order = []
if args.feature_fixtures:
    env["SLIC3R_TEST_EXTERNAL_PLUGIN_IDS"] += ",gridvarspeed,nematx_gridvarspeed_speed,nematx_gridvarspeed_recipe,nematx_corner_stretch"
    env["SLIC3R_PRESSURE_FEATURE_FIXTURES"] = "1"
    # The CLI/feature case runs first and disables the initialized feature
    # providers outside its own scopes, before any focused pressure case runs.
    order = ["--order", "lex"]
scratch = (Path(env["SLIC3R_PRESSURE_QUALIFICATION_OUTPUT"]) / "scratch"
           if "SLIC3R_PRESSURE_QUALIFICATION_OUTPUT" in env else args.build.resolve() / "temp/b5b/scratch")
scratch.mkdir(parents=True, exist_ok=True)
env["TEMP"] = env["TMP"] = str(scratch)
try:
    started = time.monotonic()
    result = subprocess.run([str(args.host_tests.resolve()), args.selector, "--reporter", "compact", "--durations", "yes"] + order,
                            env=env, capture_output=True, text=True, errors="replace", timeout=args.timeout)
except subprocess.TimeoutExpired as error:
    print(f"NematX pressure native gate exceeded {args.timeout:g} seconds", file=sys.stderr)
    print(error.stdout or "")
    print(error.stderr or "")
    sys.exit(1)
text = result.stdout + result.stderr
print(text)
print(f"NematX pressure native gate: {time.monotonic() - started:.3f} seconds")
passed = re.search(r"Passed (?:all )?(\d+) test case", text)
if result.returncode or "No test cases matched" in text or not passed or int(passed[1]) != args.expected_cases:
    sys.exit(result.returncode or 1)
