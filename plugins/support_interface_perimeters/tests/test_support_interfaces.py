"""Linear STL-to-G-code gate for support interface perimeter providers.

Every declared input has a variation domain, checked against the C++ key lists.
Cases change one input from an enabled baseline, not a Cartesian product. Other
comparisons cover zero-count parity, explicit legacy selection, sparse paths and
standalone versus built-in delivery. Each run owns its datadir and keeps logs.

    main -> Gate.export -> write_roof / host CLI -> read_moves
         -> checked_domains -> linear cases -> report.json
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
import os
import re
import shutil
import subprocess
import time
from pathlib import Path

COUNT = "support_material_interface_perimeters"
IDS = ("nematx.support.bottom_contact", "nematx.support.bottom_interface",
       "nematx.support.top_contact", "nematx.support.top_interface")
ROLES = ("support_toolpath_bottom_contact", "support_toolpath_bottom_interface",
         "support_toolpath_top_contact", "support_toolpath_top_interface")
LEGACY = ("support.toolpath.legacy_bottom_contact", "support.toolpath.legacy_bottom",
          "support.toolpath.legacy_top_contact", "support.toolpath.legacy_top")
BASE = {
    "seam_visibility": "0", "layer_height": "0.2", "first_layer_height": "0.2",
    "gcode_comments": "1", "gcode_firmware_plugin": "gcode.firmware.marlin2",
    "skirts": "0", "brim_width": "0", "perimeters": "1", "fill_density": "15%",
    "top_solid_layers": "1", "bottom_solid_layers": "1", "support_material": "1",
    "support_material_style": "grid", "support_material_buildplate_only": "0",
    "support_material_threshold": "0", "support_material_interface_layers": "3",
    "support_material_bottom_interface_layers": "3", "support_material_interface_spacing": "0",
    "support_material_extruder": "1", "support_material_interface_extruder": "1",
    "support_material_contact_distance_type": "plane", "support_material_contact_distance": "0.2",
    "support_material_bottom_contact_distance": "0.2", "dont_support_bridges": "0",
    "support_material_top_interface_pattern": "concentric", "support_material_bottom_interface_pattern": "concentric",
    "fan_below_layer_time": "0", "slowdown_below_layer_time": "0", "infill_overlap": "25%", COUNT: "2",
}
# Domains include inherited inputs of the reused support pipeline. Tool choices
# stay valid on this one-tool printer; focused host flow tests cover two tools.
DOMAIN_TEXT = """
support_material = 0
support_material_style = snug|organic|filled
support_material_buildplate_only = 1
support_material_enforce_layers = 1
support_material_threshold = 45
support_material_synchronize_layers = 1
support_material_contact_distance_type = none
support_material_contact_distance = 0.1
support_material_bottom_contact_distance = 0.1
support_material_xy_spacing = 0|75%
support_material_layer_height = 0.2
support_material_interface_layer_height = 0.2
support_material_interface_layers = 0|1
support_material_bottom_interface_layers = 0|1
support_material_interface_spacing = 0.2
support_material_spacing = 1
support_material_angle = 30
support_material_closing_radius = 0
support_material_bottom_interface_expansion = 0.2
support_material_extruder = 0
support_material_interface_extruder = 0
support_material_extrusion_width = 0.5
first_layer_height = 0.25
layer_height = 0.25
nozzle_diameter = 0.5
min_layer_height = 0.05
max_layer_height = 0.3
z_step = 0.001
resolution_internal = 0.02
extrusion_width = 0.5
extrusion_spacing = 0.45
first_layer_extrusion_width = 0.6
first_layer_extrusion_spacing = 0.55
first_layer_infill_extrusion_width = 0.6
first_layer_infill_extrusion_spacing = 0.55
external_perimeter_extrusion_width = 0.45
external_perimeter_extrusion_spacing = 0.4
external_perimeter_extrusion_change_odd_layers = 0.4|90%
external_perimeter_overlap = 50%
perimeter_extrusion_width = 0.45
perimeter_extrusion_spacing = 0.4
perimeter_extruder = 1
infill_extruder = 1
solid_infill_extruder = 1
perimeters = 2
fill_density = 30%
top_solid_layers = 0
bottom_solid_layers = 0
solid_infill_every_layers = 2
infill_extrusion_width = 0.45
infill_extrusion_spacing = 0.4
infill_extrusion_change_odd_layers = 0.4|90%
bridge_type = flow
bridge_flow_ratio = 80%
filament_max_overlap = 80%
raft_first_layer_expansion = 0
support_material_pattern = honeycomb
support_material_with_sheath = 1
support_material_top_interface_pattern = rectilinear|auto|rectiwithperimeter
support_material_bottom_interface_pattern = rectilinear|auto
support_material_interface_contact_loops = 1
support_material_angle_height = 1
support_material_interface_angle = 30
support_material_interface_angle_increment = 90
raft_first_layer_density = 80%
gap_fill_min_area = 0
gap_fill_min_width = 0
gap_fill_max_width = 1
gap_fill_min_length = 0
gap_fill_extension = 0
fill_smooth_width = 1
fill_smooth_distribution = 50%
support_material_interface_perimeters = 0|1|3
infill_overlap = 0%|50%|0.1
"""
WORD = re.compile(r"([A-Z])([+-]?(?:\d+(?:\.\d*)?|\.\d+))")
# Filled support accepts only soluble, non-raft configurations. This named group
# varies those coupled inputs together rather than treating a refused profile as
# an algorithm failure or silently skipping it.
COUPLED_DOMAINS = {("support_material_style", "filled"): {"support_material_contact_distance_type": "none", "raft_layers": "0"}}


def checked_domains(host_source: Path) -> dict[str, list[str]]:
    """Reject missing, duplicate and stale domains against both shared lists and wrapper declarations."""
    domains = {}
    for line in DOMAIN_TEXT.strip().splitlines():
        key, values = (part.strip() for part in line.split("=", 1))
        if key in domains:
            raise AssertionError(f"duplicate domain: {key}")
        domains[key] = values.split("|")
    shared = (host_source / "src/libslic3r/Plugins/Support/Common/SupportSettings.cpp").read_text(encoding="utf-8")
    wrapper = (Path(__file__).parents[1] / "src/SupportInterfacePerimeters.cpp").read_text(encoding="utf-8")
    keys = set(re.findall(r'\{"([^"]+)", RAW_CO_', shared + wrapper)) | {COUNT}
    if keys != set(domains):
        raise AssertionError(f"domain coverage mismatch: missing={keys - domains.keys()}, stale={domains.keys() - keys}")
    return domains


def write_roof(path: Path) -> None:
    """Small roof on a post and low platform, producing support above and below contacts."""
    faces = ((3, 2, 1), (1, 0, 3), (4, 5, 6), (6, 7, 4), (0, 1, 5), (5, 4, 0),
             (1, 2, 6), (6, 5, 1), (2, 3, 7), (7, 6, 2), (3, 0, 4), (4, 7, 3))
    lines = ["solid support_roof"]
    for x0, y0, z0, x1, y1, z1 in ((0, 0, 0, 1.5, 4, 2), (0, 0, 2, 5, 4, 2.4), (0, 0, 0, 5, 4, 0.2)):
        vertices = [(x, y, z) for z in (z0, z1) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
        for face in faces:
            lines.append(" facet normal 0 0 0\n  outer loop")
            lines.extend("   vertex {} {} {}".format(*vertices[index]) for index in face)
            lines.append("  endloop\n endfacet")
    path.write_text("\n".join(lines + ["endsolid support_roof"]) + "\n", encoding="ascii")


def read_moves(text: str) -> list[tuple]:
    """Read absolute XY and absolute/relative E, retaining deposited straight moves and role labels."""
    position = {"X": 0., "Y": 0., "Z": 0., "E": 0.}
    role, relative, moves = "", False, []
    for line in text.splitlines():
        if line.startswith(";TYPE:"):
            role = line.split(":", 1)[1].strip().lower()
        command = line.split(";", 1)[0].strip()
        if not command:
            continue
        fields = dict((key, float(value)) for key, value in WORD.findall(command))
        opcode = command.split()[0]
        if opcode in ("M82", "M83"):
            relative = opcode == "M83"
        elif opcode == "G92":
            for axis in position:
                if axis in fields:
                    position[axis] = fields[axis]
        elif opcode in ("G0", "G1"):
            end = dict(position)
            end.update((axis, fields[axis]) for axis in ("X", "Y", "Z") if axis in fields)
            delta_e = fields.get("E", 0.) if relative else fields.get("E", position["E"]) - position["E"]
            end["E"] = position["E"] + delta_e
            distance = math.hypot(end["X"] - position["X"], end["Y"] - position["Y"])
            if delta_e > 1e-8 and distance > 1e-7:
                values = (position["X"], position["Y"], end["X"], end["Y"], end["Z"], delta_e)
                if not all(math.isfinite(value) for value in values):
                    raise AssertionError("non-finite deposited move")
                moves.append((role,) + tuple(round(value, 6) for value in values))
            position = end
    if not moves:
        raise AssertionError("no readable deposited G-code moves")
    return moves


def sparse_geometry(moves: list[tuple]) -> list[tuple]:
    """Compare exact collinear coverage, including multiplicity, not later travel-induced segment breaks."""
    lines = defaultdict(lambda: defaultdict(int))
    for role, x0, y0, x1, y1, z, _ in moves:
        if "support" in role and "interface" not in role:
            x0, y0, x1, y1 = (round(value * 1_000_000) for value in (x0, y0, x1, y1))
            dx, dy = x1 - x0, y1 - y0
            divisor = math.gcd(abs(dx), abs(dy))
            if divisor == 0:
                raise AssertionError("deposited sparse segment vanished at the reader's precision")
            ux, uy = dx // divisor, dy // divisor
            if ux < 0 or (ux == 0 and uy < 0):
                ux, uy = -ux, -uy
            key = z, ux, uy, ux * y0 - uy * x0
            first, last = sorted((ux * x0 + uy * y0, ux * x1 + uy * y1))
            lines[key][first] += 1
            lines[key][last] -= 1
    result = []
    for key, events in lines.items():
        depth, previous, intervals = 0, None, []
        for position, change in sorted(events.items()):
            if previous is not None and position > previous and depth:
                if intervals and intervals[-1][1] == previous and intervals[-1][2] == depth:
                    intervals[-1] = intervals[-1][0], position, depth
                else:
                    intervals.append((previous, position, depth))
            depth += change
            previous = position
        result.extend(key + interval for interval in intervals)
    return sorted(result)


class Gate:
    """Bounded host invocations with isolated configuration and complete verification artifacts."""

    def __init__(self, args):
        self.args = args
        self.root = args.output.resolve() / str(time.time_ns())
        self.root.mkdir(parents=True, exist_ok=False)
        self.started = time.monotonic()
        self.results = []
        self.environment = dict(os.environ)
        self.environment["PATH"] = os.pathsep.join(args.runtime_path + [str(args.host_executable.parent), self.environment.get("PATH", "")])

    def export(self, name: str, changes: dict[str, str], standalone: bool = False, save: bool = False,
               source_profile: Path | None = None) -> list[tuple]:
        """Export once, rejecting process failures and plugin diagnostics, and retain the saved configuration."""
        case = self.root / name
        case.mkdir()
        datadir = case / "data"
        (datadir / "plugins").mkdir(parents=True)
        shutil.copy2(self.args.default_activated, datadir / "plugins/activated.ini")
        if standalone:
            prepared = self.args.build_dir / "test-data/support_interface_perimeters"
            shutil.copy2(prepared / "plugins/activated.ini", datadir / "plugins/activated.ini")
            shutil.copytree(prepared / "plugins/nematx-support-interface-perimeters", datadir / "plugins/nematx-support-interface-perimeters")
        model, profile, output = case / "roof.stl", case / "profile.ini", case / "output.gcode"
        write_roof(model)
        options = BASE | changes
        profile.write_text("\n".join(f"{key} = {value}" for key, value in options.items()) + "\n", encoding="utf-8")
        command = [str(self.args.host_executable), "--datadir", str(datadir), "--threads", "2", "--load", str(source_profile or profile)]
        if save:
            command += ["--save", str(case / "saved.ini")]
        command += ["--export-gcode", "--output", str(output), str(model)]
        remaining = self.args.budget - (time.monotonic() - self.started)
        if remaining <= 0:
            raise TimeoutError("E2E gate exhausted its total time budget")
        started = time.monotonic()
        log_path = case / "host.log"
        with log_path.open("wb") as log_file:
            run = subprocess.run(command, stdout=log_file, stderr=subprocess.STDOUT,
                                 timeout=min(120, remaining), env=self.environment)
        log = log_path.read_text(encoding="utf-8", errors="replace")
        if run.returncode != 0 or not output.is_file():
            raise AssertionError(f"{name}: host failed ({run.returncode}); see {case / 'host.log'}")
        if re.search(r"\[error\]|initialization failed|Cannot define|Unknown option|unknown option", log):
            raise AssertionError(f"{name}: host diagnostic; see {case / 'host.log'}")
        moves = read_moves(output.read_text(encoding="utf-8"))
        self.results.append({"case": name, "changes": changes, "standalone": standalone,
                             "seconds": time.monotonic() - started, "deposited_moves": len(moves)})
        return moves


def main() -> None:
    """Run parity and delivery comparisons before spending the remaining budget on the linear catalogue."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--host-executable", type=Path, required=True)
    parser.add_argument("--host-source", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path, required=True)
    parser.add_argument("--default-activated", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--runtime-path", action="append", default=[])
    parser.add_argument("--budget", type=float, default=600.)
    args = parser.parse_args()
    domains = checked_domains(args.host_source)
    gate = Gate(args)
    raft_contact = {}
    try:
        zero = gate.export("zero", {COUNT: "0"}, save=True)
        defaults = dict(line.split(" = ", 1) for line in (gate.root / "zero/saved.ini").read_text(encoding="utf-8").splitlines() if " = " in line)
        if defaults.get(COUNT) != "0":
            raise AssertionError("perimeter option was not retained in the saved configuration")
        selectors = {role: next(key for key in defaults if key.startswith("exclusive_group_") and role in key) for role in ROLES}
        legacy = gate.export("legacy", {COUNT: "3"} | {selectors[role]: provider for role, provider in zip(ROLES, LEGACY)})
        if zero != legacy:
            raise AssertionError("zero-count paths differ from explicit legacy paths")
        baseline = gate.export("enabled", {}, save=True)
        if baseline == zero:
            raise AssertionError("automatic providers did not apply the enabled feature")
        if not any("support" in move[0] and "interface" in move[0] for move in baseline):
            raise AssertionError("fixture did not produce support interface moves")
        baseline_sparse, zero_sparse = sparse_geometry(baseline), sparse_geometry(zero)
        if not zero_sparse:
            raise AssertionError("fixture did not exercise sparse support deposition")
        if baseline_sparse != zero_sparse:
            raise AssertionError("interface perimeters changed sparse support geometry")
        sparse_e = lambda moves: sum(move[-1] for move in moves if "support" in move[0] and "interface" not in move[0])
        if not math.isclose(sparse_e(baseline), sparse_e(zero), rel_tol=1e-7, abs_tol=1e-5):
            raise AssertionError("interface perimeters changed sparse support deposited volume")
        packaged = gate.export("standalone", {}, standalone=True)
        if packaged != baseline:
            raise AssertionError("standalone and built-in paths differ")
        reloaded = gate.export("reloaded", {}, source_profile=gate.root / "enabled/saved.ini")
        if reloaded != baseline:
            raise AssertionError("repeated export is not deterministic")
        raft_zero = gate.export("raft-zero", {"raft_layers": "3", COUNT: "0"})
        raft_enabled = gate.export("raft-enabled", {"raft_layers": "3"})
        model_first_z = min(move[5] for move in raft_enabled if "support" not in move[0])
        raft_interfaces = lambda moves: [move for move in moves if "support" in move[0] and "interface" in move[0]
                                        and move[5] < model_first_z - 1e-5]
        zero_contact, enabled_contact = raft_interfaces(raft_zero), raft_interfaces(raft_enabled)
        contact_emitted = bool(zero_contact or enabled_contact)
        # Some host generators do not publish raft contacts. Record that missing path instead of claiming raft coverage.
        raft_contact = {"requested_layers": 3, "model_first_z": model_first_z,
                        "dense_moves_below_model": len(enabled_contact),
                        "status": "verified" if contact_emitted else "not_emitted_by_host"}
        if contact_emitted and (not enabled_contact or enabled_contact == zero_contact):
            raise AssertionError("interface perimeters did not affect the emitted raft contact")
        for key, values in domains.items():
            for index, value in enumerate(values):
                changes = {key: value} | COUPLED_DOMAINS.get((key, value), {})
                gate.export(f"domain-{key}-{index}", changes)
    finally:
        (gate.root / "report.json").write_text(json.dumps({"tag": "plugin-e2e", "domains": domains,
            "raft_contact": raft_contact,
            "coupled_domains": [{"setting": key, "value": value, "changes": changes} for (key, value), changes in COUPLED_DOMAINS.items()],
            "results": gate.results, "seconds": time.monotonic() - gate.started}, indent=2), encoding="utf-8")
    print(f"Support interface gate passed: {len(gate.results)} exports, {len(domains)} input domains; {gate.root}")


if __name__ == "__main__":
    main()
