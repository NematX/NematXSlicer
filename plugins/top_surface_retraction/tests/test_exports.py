"""Linear production export gate for the top-surface retraction policy.

    main -> export: private activation/profile, generated STL, real host process
         -> parse: per-height retractions and canonical positive XY deposits
         -> compare: top-only effect, thresholds, disabled length, routing/wipe

Every run keeps its input, saved settings, G-code, diagnostics and JSON report.
With --package-dir, copy a built standalone package into the private data dir;
otherwise exercise the embedded provider. Other production plugins stay active.
"""
import argparse
import configparser
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

PLUGIN = "nematx_top_surface_retraction"
BASE = {
    "layer_height": "0.2", "first_layer_height": "0.2", "seam_visibility": "0",
    "gcode_flavor": "marlin2", "gcode_comments": "1",
    "use_relative_e_distances": "1", "use_firmware_retraction": "0",
    "skirts": "0", "brim_width": "0", "perimeters": "1", "fill_density": "20%",
    "top_solid_layers": "1", "bottom_solid_layers": "0", "fill_pattern": "rectilinear",
    "top_fill_pattern": "rectilinear", "bottom_fill_pattern": "rectilinear",
    "infill_connection": "notconnected", "infill_connection_solid": "notconnected",
    "infill_connection_top": "notconnected", "infill_connection_bottom": "notconnected",
    "enforce_retract_top_surface": "1", "only_retract_when_crossing_perimeters": "1",
    "enforce_retract_first_layer": "0", "retract_before_travel": "0.1", "retract_length": "2",
    "retract_layer_change": "0", "wipe": "0", "retract_lift": "0",
    "fan_below_layer_time": "0", "slowdown_below_layer_time": "0",
}
DOMAINS = {"enforce_retract_top_surface": ["0", "1"]}
WORD = re.compile(r"([A-Z])([+-]?(?:\d+(?:\.\d*)?|\.\d+))")


def slab(path):
    """Write a watertight six-layer 8 mm square STL with a flat exposed roof."""
    vertices = [(x, y, z) for z in (0., 1.2) for y in (0., 8.) for x in (0., 8.)]
    faces = [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4),
             (2, 6, 7), (2, 7, 3), (0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5)]
    lines = ["solid slab"]
    for face in faces:
        lines += ["facet normal 0 0 0", "outer loop"]
        lines += ["vertex " + " ".join(map(str, vertices[i])) for i in face]
        lines += ["endloop", "endfacet"]
    path.write_text("\n".join(lines + ["endsolid slab"]), encoding="ascii")


def parse(path):
    """Read relative E events by layer and compare only deposited geometry/volume."""
    position = {"X": 0., "Y": 0., "Z": 0.}
    counts, deposits = {}, []
    for line in path.read_text(encoding="utf-8").splitlines():
        code = line.split(";", 1)[0].strip()
        if not code or code.split()[0] not in ("G0", "G1"):
            continue
        words = {k: float(v) for k, v in WORD.findall(code)}
        before = position.copy()
        position.update({k: words[k] for k in position if k in words})
        e = words.get("E", 0.)
        if e < 0.:
            z = round(position["Z"], 4)
            counts[z] = counts.get(z, 0) + 1
        if e > 0. and any(position[k] != before[k] for k in ("X", "Y")):
            ends = sorted(tuple(round(p[k], 4) for k in ("X", "Y", "Z")) for p in (before, position))
            deposits.append((ends, round(e, 5)))
    assert deposits, "No deposited geometry"
    return counts, sorted(deposits)


def main():
    """Qualify the setting domain and interactions without a Cartesian matrix."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=Path, required=True)
    parser.add_argument("--defaults", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--runtime-path", type=Path, action="append", default=[])
    parser.add_argument("--package-dir", type=Path)
    args = parser.parse_args()
    root = args.output.resolve() / str(time.time_ns())
    root.mkdir(parents=True)
    env = dict(os.environ)
    # The loader runs before CLI loglevel parsing, so enable its diagnostics early.
    env["SLIC3R_LOGLEVEL"] = "5"
    env["PATH"] = os.pathsep.join([str(p.resolve()) for p in args.runtime_path] + [env.get("PATH", "")])
    results, exercised = [], set()
    passed = False

    def export(name, changes=None, active=True, saved=None):
        """Run with production defaults and reject diagnostics or ignored options."""
        changes = changes or {}
        case = root / name
        data = case / "data"
        (data / "plugins").mkdir(parents=True)
        activation = configparser.ConfigParser(interpolation=None)
        activation.read(args.defaults, encoding="utf-8")
        activation.set("activated", PLUGIN, str(int(active)))
        if args.package_dir:
            destination = data / "plugins/nematx-top-surface-retraction"
            destination.mkdir()
            for filename in ("plugin.dll", "description.ini", "version.ini"):
                shutil.copyfile(args.package_dir / filename, destination / filename)
            metadata = configparser.ConfigParser(interpolation=None)
            metadata.read(destination / "version.ini", encoding="utf-8")
            activation.set("installed", "nematx-top-surface-retraction", metadata["plugin"]["package_version"])
            activation.set("installed", "nematx-top-surface-retraction.slicer_version", metadata["plugin"]["slicer_version"])
            activation.set("plugin_packages", PLUGIN, "nematx-top-surface-retraction")
        with (data / "plugins/activated.ini").open("w", encoding="utf-8") as stream:
            activation.write(stream)
        slab(case / "slab.stl")
        settings = BASE | changes
        if saved:
            profile = saved
        else:
            profile = case / "profile.ini"
            profile.write_text("".join(f"{k} = {v}\n" for k, v in settings.items()), encoding="utf-8")
        command = [str(args.host.resolve()), "--loglevel", "5", "--datadir", str(data), "--threads", "1", "--load", str(profile),
            "--save", str(case / "saved.ini"), "--export-gcode", "--output", str(case / "output.gcode"),
            str(case / "slab.stl")]
        with (case / "host.log").open("wb") as log:
            run = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=env, timeout=120)
        assert run.returncode == 0, f"{name}: {case / 'host.log'} (exit {run.returncode})"
        log = (case / "host.log").read_text(encoding="utf-8", errors="replace")
        assert not re.search(r"\[error\]|Unknown option|unknown option|incompatible|Failed to load", log), log
        if args.package_dir:
            assert re.search(r"Loaded plugin DLL '[^']*nematx-top-surface-retraction[^']*plugin.dll'", log), "Standalone DLL load was not observed"
        counts, deposits = parse(case / "output.gcode")
        if active:
            saved_settings = (case / "saved.ini").read_text(encoding="utf-8")
            assert f"enforce_retract_top_surface = {settings['enforce_retract_top_surface']}" in saved_settings
            exercised.add(settings["enforce_retract_top_surface"])
        results.append({"name": name, "retractions": counts, "deposits": len(deposits)})
        return counts, deposits

    try:
        enabled = export("enabled")
        disabled = export("disabled", {"enforce_retract_top_surface": "0"})
        assert enabled[0].get(1.2, 0) > disabled[0].get(1.2, 0), "No top-layer effect"
        assert {z: n for z, n in enabled[0].items() if z < 1.2} == {z: n for z, n in disabled[0].items() if z < 1.2}
        assert enabled[1] == disabled[1], "The policy changed deposited geometry or volume"
        inactive = export("inactive", active=False)
        assert inactive == disabled, "Plugin deactivation retained a top policy"
        high = export("minimum-distance", {"retract_before_travel": "100"})
        assert sum(high[0].values()) < sum(enabled[0].values()), "Distance threshold bypassed"
        zero = export("zero-length", {"retract_length": "0"})
        assert not zero[0] and zero[1] == enabled[1]
        export("avoiding", {"avoid_crossing_perimeters": "1", "avoid_crossing_not_first_layer": "0"})
        export("wipe", {"wipe": "1"})
        unrestricted = export("unrestricted", {"only_retract_when_crossing_perimeters": "0"})
        unrestricted_off = export("unrestricted-off", {"only_retract_when_crossing_perimeters": "0", "enforce_retract_top_surface": "0"})
        assert unrestricted == unrestricted_off, "Policy acted without the perimeter filter"
        reloaded = export("reloaded", saved=root / "enabled/saved.ini")
        assert reloaded == enabled, "Save/reload changed the policy result"
        assert exercised == set(DOMAINS["enforce_retract_top_surface"]), "Missing setting domain"
        passed = True
    finally:
        (root / "report.json").write_text(json.dumps({"status": "passed" if passed else "failed",
            "mode": "package" if args.package_dir else "builtin", "cases": results}, indent=2), encoding="utf-8")
        print(root)


if __name__ == "__main__":
    main()
