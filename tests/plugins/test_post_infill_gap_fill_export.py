"""Linear STL-to-G-code gate for the built-in infill residual pass.

    main -> run_all -> export: private profile and runtime, exports in parallel
         -> read_moves: compare deposited geometry, not comments or travel
         -> activation, persistence, sparse preservation and input boundaries

Each export owns its datadir, so all run in parallel except the reloaded one,
which needs the settings saved by the enabled export. The catalogue must match
the plugin's declared inputs. Diagnostics and measured results stay in the
requested output directory, outside versioned source files.
"""
from __future__ import annotations

import argparse
import hashlib
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time

BASE = {
    "layer_height": "0.2", "first_layer_height": "0.2", "seam_visibility": "0",
    "gcode_comments": "1", "gcode_flavor": "marlin2",
    "skirts": "0", "brim_width": "0", "perimeters": "1", "fill_density": "20%",
    "top_solid_layers": "2", "bottom_solid_layers": "2", "support_material": "0",
    "fill_pattern": "rectilinear", "solid_fill_pattern": "concentric", "top_fill_pattern": "concentric",
    "bottom_fill_pattern": "concentric",
    "infill_filled_top": "1", "infill_filled_bottom": "1", "infill_filled_solid": "1",
    "gap_fill_enabled": "1", "gap_fill_min_area": "0", "gap_fill_min_length": "0",
    "gap_fill_min_width": "0.1", "gap_fill_max_width": "0.8", "gap_fill_extension": "0",
    "enforce_full_fill_volume": "0", "fan_below_layer_time": "0", "slowdown_below_layer_time": "0",
}
DOMAINS = {
    "infill_filled_top": ["0"], "infill_filled_solid": ["0"], "infill_filled_bottom": ["0"],
    "gap_fill_enabled": ["0"], "gap_fill_min_area": ["0.1", "100%"],
    "gap_fill_min_width": ["0", "200%"], "gap_fill_max_width": ["0", "0.1"],
    "gap_fill_min_length": ["0.5", "200%"], "gap_fill_extension": ["0.1", "50%"],
    "resolution": ["0.005", "0.05"],
    "nozzle_diameter": ["0.5", "0.6"],
}
WORD = re.compile(r"([A-Z])([+-]?(?:\d+(?:\.\d*)?|\.\d+))")


def write_slab(path: Path) -> None:
    """Write a small closed slab with top, bottom and internal solid layers."""
    vertices = [(x, y, z) for z in (0., 1.6) for y in (0., 8.) for x in (0., 12.)]
    faces = [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4),
             (2, 6, 7), (2, 7, 3), (0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5)]
    lines = ["solid slab"]
    for face in faces:
        lines += ["facet normal 0 0 0", "outer loop"]
        lines += ["vertex " + " ".join(map(str, vertices[index])) for index in face]
        lines += ["endloop", "endfacet"]
    path.write_text("\n".join(lines + ["endsolid slab"]), encoding="ascii")


def read_moves(path: Path) -> list:
    """Canonicalize positive-E planar moves; ignore orientation and travels."""
    position = dict(X=0., Y=0., Z=0., E=0.)
    relative_e = False
    relative_xyz = False
    role = ""
    moves = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith(";TYPE:"):
            role = line[6:].strip()
        code = line.split(";", 1)[0].strip()
        words = dict((key, float(value)) for key, value in WORD.findall(code))
        command = code.split(" ", 1)[0]
        if command == "M82": relative_e = False
        if command == "M83": relative_e = True
        if command == "G90": relative_xyz = False
        if command == "G91": relative_xyz = True
        if command == "G92":
            position.update({key: value for key, value in words.items() if key in position})
        if command not in ("G0", "G1"):
            continue
        before = position.copy()
        for key in position:
            if key in words:
                relative = relative_e if key == "E" else relative_xyz
                position[key] = words[key] + (position[key] if relative else 0.)
        delta_e = position["E"] - before["E"]
        if delta_e > 0. and any(position[k] != before[k] for k in ("X", "Y")):
            ends = sorted(tuple(round(p[k], 4) for k in ("X", "Y", "Z")) for p in (before, position))
            moves.append((role, ends, round(delta_e, 5)))
    return sorted(moves)


def sparse_geometry(moves: list) -> list:
    """Compare collinear coverage with multiplicity, independently of segmentation."""
    lines = defaultdict(lambda: defaultdict(int))
    for role, ends, _ in moves:
        if role != "Internal infill":
            continue
        (x0, y0, z), (x1, y1, _) = [[round(v * 10000) for v in p] for p in ends]
        dx, dy = x1 - x0, y1 - y0
        divisor = math.gcd(abs(dx), abs(dy))
        if not divisor:
            continue
        ux, uy = dx // divisor, dy // divisor
        if ux < 0 or (ux == 0 and uy < 0): ux, uy = -ux, -uy
        key = z, ux, uy, ux * y0 - uy * x0
        first, last = sorted((ux * x0 + uy * y0, ux * x1 + uy * y1))
        lines[key][first] += 1
        lines[key][last] -= 1
    result = []
    for key, events in lines.items():
        previous, depth, intervals = None, 0, []
        for position, change in sorted(events.items()):
            if previous is not None and position > previous and depth:
                if intervals and intervals[-1][1:] == (previous, depth):
                    intervals[-1] = intervals[-1][0], position, depth
                else:
                    intervals.append((previous, position, depth))
            depth += change
            previous = position
        result.extend(key + interval for interval in intervals)
    return sorted(result)


def main() -> None:
    """Run the bounded gate and retain a report even if an assertion fails."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=Path, required=True)
    parser.add_argument("--defaults", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--runtime-path", type=Path, action="append", default=[])
    args = parser.parse_args()
    root = args.output.resolve() / str(time.time_ns())
    root.mkdir(parents=True)
    source = Path(__file__).parents[2] / "src/libslic3r/Plugins/Infill/PostInfillGapFill.cpp"
    declared = set(re.findall(r'\{"([^\"]+)", RAW_CO_', source.read_text(encoding="utf-8")))
    if declared != set(DOMAINS):
        raise AssertionError(f"Domain catalogue mismatch: {declared ^ set(DOMAINS)}")
    environment = dict(os.environ)
    environment["PATH"] = os.pathsep.join([str(p) for p in args.runtime_path] + [environment.get("PATH", "")])
    results = []
    started = time.monotonic()
    passed = False

    def export(name, changes=None, provider=True, reload=None):
        """Run the production host with private activation and retain its log."""
        case = root / name
        (case / "data/plugins").mkdir(parents=True)
        activation = args.defaults.read_text(encoding="utf-8")
        if not provider:
            activation, count = re.subn(r"(?m)^infill\.post_process\.gap_fill\s*=\s*1\s*$",
                                       "infill.post_process.gap_fill = 0", activation)
            if count != 1:
                raise AssertionError("Cannot disable built-in gap fill")
        (case / "data/plugins/activated.ini").write_text(activation, encoding="utf-8")
        options = BASE | (changes or {})
        profile = case / "profile.ini"
        profile.write_text("".join(f"{k} = {v}\n" for k, v in options.items()), encoding="utf-8")
        write_slab(case / "slab.stl")
        command = [str(args.host.resolve()), "--datadir", str(case / "data"), "--threads", "2",
                   "--load", str(reload or profile), "--save", str(case / "saved.ini"),
                   "--export-gcode", "--output", str(case / "output.gcode"), str(case / "slab.stl")]
        with (case / "host.log").open("wb") as log:
            run = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=environment,
                                 timeout=min(120., max(1., 240. - (time.monotonic() - started))))
        log = (case / "host.log").read_text(encoding="utf-8", errors="replace")
        if run.returncode or re.search(r"\[error\]|Unknown option|unknown option|initialization failed", log):
            results.append({"name": name, "changes": changes or {}, "status": "failed",
                            "exit_code": run.returncode, "log": str(case / "host.log")})
            raise AssertionError(f"{name}: host exit {run.returncode}; see {case / 'host.log'}")
        moves = read_moves(case / "output.gcode")
        if not moves:
            raise AssertionError(f"{name}: empty deposition")
        result = {"name": name, "changes": changes or {}, "status": "passed", "moves": len(moves),
                  "gaps": sum(m[0] == "Gap fill" for m in moves)}
        if name == "enabled":
            result["moves_sha256"] = hashlib.sha256(
                json.dumps(moves, ensure_ascii=True, separators=(",", ":"), allow_nan=False).encode("ascii")).hexdigest()
        results.append(result)
        return moves

    def run_all(cases):
        """Export {name: keyword arguments} in parallel; return the moves by name."""
        # Host startup, single-threaded, dominates a debug export: one worker per logical processor.
        with ThreadPoolExecutor(max_workers=os.cpu_count() or 2) as pool:
            futures = {name: pool.submit(export, name, **options) for name, options in cases.items()}
            return {name: future.result() for name, future in futures.items()}

    try:
        off = dict.fromkeys(("infill_filled_top", "infill_filled_solid", "infill_filled_bottom"), "0")
        patterns = {pattern: {"changes": {"solid_fill_pattern": pattern, "top_fill_pattern": pattern,
                                          "bottom_fill_pattern": pattern, "fill_pattern": pattern}}
                    for pattern in ("rectilinear", "alignedrectilinear", "monotonic")}
        catalogue = {f"{key}-{index}": {"changes": {key: value}}
                     for key, values in DOMAINS.items() for index, value in enumerate(values)}
        first = run_all({
            "disabled": {"changes": off},
            "provider-disabled": {"provider": False},
            "enabled": {},
            "sparse-off": {"changes": off | {"fill_density": "20%"}},
            "sparse-on": {"changes": {"fill_density": "20%"}},
            "volume-corrected": {"changes": {"enforce_full_fill_volume": "1"}},
            "without-perimeters": {"changes": {"perimeters": "0"}},
        } | patterns | catalogue)
        disabled, enabled = first["disabled"], first["enabled"]
        assert first["provider-disabled"] == disabled
        assert enabled != disabled, "Fixture must create additional deposition"
        assert any(m[0] == "Gap fill" for m in enabled), "No exported gap-fill paths"
        # Reloading needs the settings saved by the enabled export.
        assert export("reloaded", reload=root / "enabled/saved.ini") == enabled
        sparse_off, sparse_on = first["sparse-off"], first["sparse-on"]
        assert sparse_geometry(sparse_off), "Fixture must exercise sparse deposition"
        assert sparse_geometry(sparse_off) == sparse_geometry(sparse_on), "Sparse infill changed"
        sparse_e = lambda moves: sum(m[2] for m in moves if m[0] == "Internal infill")
        assert abs(sparse_e(sparse_off) - sparse_e(sparse_on)) < .0005, "Sparse extrusion quantity changed"
        passed = True
    finally:
        (root / "report.json").write_text(json.dumps({"status": "passed" if passed else "failed",
            "cases": results}, indent=2), encoding="utf-8")
        print(root)


if __name__ == "__main__":
    main()
