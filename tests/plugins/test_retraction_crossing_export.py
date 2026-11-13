"""Bounded production-host regression for perimeter-based retraction filtering.

    main -> write_slab -> export: private profile, activation and G-code
         -> compare retraction counts with the interior filter and exceptions

The gate preserves every host failure and checks deposited XY geometry separately
from E-only retraction events. It uses a small generated STL, not a prebuilt plan.
"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time

BASE = {
    "layer_height": "0.2", "first_layer_height": "0.2", "seam_visibility": "0",
    "gcode_flavor": "marlin2", "gcode_comments": "1",
    "use_relative_e_distances": "1", "use_firmware_retraction": "0",
    "skirts": "0", "brim_width": "0", "perimeters": "1", "fill_density": "20%",
    "top_solid_layers": "0", "bottom_solid_layers": "0", "fill_pattern": "rectilinear",
    "infill_connection": "notconnected", "infill_connection_solid": "notconnected",
    "infill_connection_top": "notconnected", "infill_connection_bottom": "notconnected",
    "only_retract_when_crossing_perimeters": "1",
    "enforce_retract_first_layer": "0", "retract_before_travel": "0.1", "retract_length": "2",
    "retract_layer_change": "0", "wipe": "0", "retract_lift": "0",
    "fan_below_layer_time": "0", "slowdown_below_layer_time": "0",
}
WORD = re.compile(r"([A-Z])([+-]?(?:\d+(?:\.\d*)?|\.\d+))")


def write_slab(path):
    """Write a watertight 8 x 8 x 1.2 mm slab with six printed layers."""
    vertices = [(x, y, z) for z in (0., 1.2) for y in (0., 8.) for x in (0., 8.)]
    faces = [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4),
             (2, 6, 7), (2, 7, 3), (0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5)]
    lines = ["solid slab"]
    for face in faces:
        lines += ["facet normal 0 0 0", "outer loop"]
        lines += ["vertex " + " ".join(map(str, vertices[i])) for i in face]
        lines += ["endloop", "endfacet"]
    path.write_text("\n".join(lines + ["endsolid slab"]), encoding="ascii")


def read_deposition(path):
    """Count relative-E retractions and canonicalize positive planar deposits."""
    position = {"X": 0., "Y": 0., "Z": 0.}
    retractions = 0
    deposits = []
    for line in path.read_text(encoding="utf-8").splitlines():
        code = line.split(";", 1)[0].strip()
        if not code or code.split()[0] not in ("G0", "G1"):
            continue
        words = {k: float(v) for k, v in WORD.findall(code)}
        before = position.copy()
        position.update({k: words[k] for k in position if k in words})
        extrusion = words.get("E", 0.)
        retractions += extrusion < 0.
        if extrusion > 0. and any(position[k] != before[k] for k in ("X", "Y")):
            ends = sorted(tuple(round(p[k], 4) for k in ("X", "Y", "Z")) for p in (before, position))
            deposits.append((ends, round(extrusion, 5)))
    return retractions, sorted(deposits)


def main():
    """Export linear variations and retain an explicit pass/fail report."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=Path, required=True)
    parser.add_argument("--defaults", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--runtime-path", type=Path, action="append", default=[])
    args = parser.parse_args()
    root = args.output.resolve() / str(time.time_ns())
    root.mkdir(parents=True)
    environment = dict(os.environ)
    environment["PATH"] = os.pathsep.join([str(p) for p in args.runtime_path] + [environment.get("PATH", "")])
    results = []
    passed = False

    def export(name, changes):
        """Use the real production registry without isolating other NematX providers."""
        case = root / name
        (case / "data/plugins").mkdir(parents=True)
        (case / "data/plugins/activated.ini").write_bytes(args.defaults.read_bytes())
        profile = case / "profile.ini"
        profile.write_text("".join(f"{k} = {v}\n" for k, v in (BASE | changes).items()), encoding="utf-8")
        write_slab(case / "slab.stl")
        with (case / "host.log").open("wb") as log:
            run = subprocess.run([str(args.host.resolve()), "--datadir", str(case / "data"), "--threads", "2",
                "--load", str(profile), "--export-gcode", "--output", str(case / "output.gcode"),
                str(case / "slab.stl")], stdout=log, stderr=subprocess.STDOUT, env=environment, timeout=120)
        if run.returncode:
            results.append({"name": name, "status": "failed", "exit": run.returncode})
            raise AssertionError(f"{name}: see {case / 'host.log'}")
        log = (case / "host.log").read_text(encoding="utf-8", errors="replace")
        assert not re.search(r"\[error\]|Unknown option|unknown option", log), log
        retracts, deposits = read_deposition(case / "output.gcode")
        assert deposits, "No deposited geometry"
        results.append({"name": name, "status": "passed", "retractions": retracts, "deposits": len(deposits)})
        return retracts, deposits

    try:
        unrestricted = export("unrestricted", {"only_retract_when_crossing_perimeters": "0"})
        filtered = export("filtered", {})
        assert filtered[0] < unrestricted[0], "Interior filtering did not suppress retractions"
        assert filtered[1] == unrestricted[1], "Filtering changed deposited geometry or volume"
        first = export("first-layer-exemption", {"enforce_retract_first_layer": "1"})
        assert first[0] > filtered[0], "First-layer exception had no effect"
        assert first[1] == filtered[1]
        high = export("minimum-distance", {"retract_before_travel": "100"})
        assert high[0] <= filtered[0]
        zero = export("zero-length", {"retract_length": "0"})
        assert zero[0] == 0
        assert zero[1] == filtered[1]
        export("avoiding", {"avoid_crossing_perimeters": "1", "avoid_crossing_not_first_layer": "0"})
        export("wipe", {"wipe": "1"})
        passed = True
    finally:
        (root / "report.json").write_text(json.dumps({"status": "passed" if passed else "failed",
            "cases": results}, indent=2), encoding="utf-8")
        print(root)


if __name__ == "__main__":
    main()
