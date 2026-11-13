# Released under the GNU Affero General Public License, version 3 or later.

"""Check the versioned annotation codec on actual standalone rewriter output.

The executable prints a small sample, captured in memory with normalized Windows
newlines. Exact schema checks reject missing, duplicate and nonfinite fields.
Literal spatial/E values and analytical phase boundaries are independent oracles;
no export, temporary file, consumer adapter or runtime provider is involved.

Call tree:
    main: run the executable, compare literal output and exercise invalid records.
        decode: require one marker, exact ordered fields and numeric constraints.
"""

import math
import subprocess
import sys


FIELDS = (
    "v", "tool", "phase", "v0_mm_s", "v1_mm_s", "e0_mm_s", "e1_mm_s",
    "dt_s", "k_s", "de_nom_mm", "de_pressure_mm",
)


def decode(line):
    """Return typed fields from one exact v1 record or raise ValueError."""
    if line.count(";NX_PRESSURE ") != 1:
        raise ValueError("one marker required per group")
    code, marker = line.split(";NX_PRESSURE ")
    pairs = marker.split()
    if len(pairs) != len(FIELDS):
        raise ValueError("wrong field count")
    values = {}
    for expected, pair in zip(FIELDS, pairs):
        key, separator, value = pair.partition("=")
        if not separator or key != expected or key in values:
            raise ValueError("unknown, duplicate or unordered field")
        values[key] = value
    if values["v"] != "1" or not values["tool"].isascii() or not values["tool"].isdigit():
        raise ValueError("invalid version/tool")
    values["v"] = 1
    values["tool"] = int(values["tool"])
    if values["tool"] > 2**32 - 1 or values["phase"] not in {"accel", "steady", "decel", "merged"}:
        raise ValueError("invalid tool/phase")
    for key in FIELDS[3:]:
        values[key] = float(values[key])
        if not math.isfinite(values[key]):
            raise ValueError("nonfinite value")
    if values["dt_s"] <= 0 or values["k_s"] < 0:
        raise ValueError("invalid duration/coefficient")
    return code.rstrip(), values


def main():
    """Check literal groups, summed times and rejection cases; fail nonzero."""
    if len(sys.argv) != 2:
        raise ValueError("test executable required")
    process = subprocess.run(
        [sys.argv[1], "--annotation-fixture"], capture_output=True, text=True,
        timeout=30, check=False,
    )
    if process.returncode != 0:
        raise RuntimeError(f"rewriter exited {process.returncode}: {process.stderr}")
    if process.stderr:
        raise ValueError(f"unexpected fixture diagnostics: {process.stderr}")
    lines = process.stdout.splitlines()
    if len(lines) != 4:
        raise ValueError("three actual phases and one independently merged source required")
    expected = (
        ("G1 X2 E.4", "accel", 0, 2, 0, .2, 2, .2, .2),
        ("G1 X8 E.6", "steady", 2, 2, .20000000000000004, .20000000000000004, 3, .6000000000000001, 0),
        ("G1 X10 E0", "decel", 2, 0, .19999999999999996, 0, 2, .19999999999999996, -.2),
        ("G1 X1 E.1", "merged", 0, 0, 0, 0, 1.01, .1, 0),
    )
    checks = 0
    elapsed = 0
    for line, oracle in zip(lines, expected):
        code, values = decode(line)
        actual = (code, values["phase"], *(values[key] for key in (
            "v0_mm_s", "v1_mm_s", "e0_mm_s", "e1_mm_s", "dt_s", "de_nom_mm", "de_pressure_mm")))
        if actual != oracle or values["tool"] != 0 or values["v"] != 1 or values["k_s"] != .1:
            raise ValueError(f"literal group mismatch: {actual}")
        checks += 12
        if values["phase"] != "merged":
            elapsed += values["dt_s"]
            if elapsed != {"accel": 2, "steady": 5, "decel": 7}[values["phase"]]:
                raise ValueError("phase boundary times must be 2, 5 and 7 seconds")
            checks += 1
    negatives = (
        lines[0].replace("v=1", "v=2"),
        lines[0].replace("tool=0", "tool=-1"),
        lines[0].replace("tool=0", "tool=0.0"),
        lines[0].replace("phase=accel", "phase=other"),
        lines[0].replace(" dt_s=2", ""),
        lines[0].replace(" dt_s=2", " dt_s=0"),
        lines[0].replace(" k_s=0.1", " k_s=-0.1"),
        lines[0].replace(" v0_mm_s=0", " v0_mm_s=nan"),
        lines[0].replace(" e0_mm_s=0", " e0_mm_s=inf"),
        lines[0] + " tool=0",
        lines[0].replace(" v1_mm_s=2", " v0_mm_s=2"),
        lines[0] + " ;NX_PRESSURE v=1",
        "G1 X1 E0",
    )
    for invalid in negatives:
        try:
            decode(invalid)
        except ValueError:
            checks += 1
        else:
            raise ValueError(f"invalid record accepted: {invalid}")
    print(f"pressure rewrite annotations: {len(expected) + len(negatives)} cases, {checks} checks, 0 failures")


if __name__ == "__main__":
    main()
