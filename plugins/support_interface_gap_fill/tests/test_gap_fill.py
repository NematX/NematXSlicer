"""Bounded standalone STL-to-G-code gate for support interface gap fill.

Reuse the perimeter fixture and reader without changing that package.

    main -> export: private profile, staged package, roof fixture, host CLI
         -> compare disabled / enabled / reloaded and linear setting variations
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

HERE = Path(__file__).resolve()
spec = importlib.util.spec_from_file_location("perimeter_fixture", HERE.parents[2] / "support_interface_perimeters/tests/test_support_interfaces.py")
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
OPTION = "support_material_interface_gap_fill"


def main() -> None:
    """Check actual deposited paths, persistence and declared input domains."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=Path, required=True)
    parser.add_argument("--host-source", type=Path, required=True)
    parser.add_argument("--prepared", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--embedded-defaults", type=Path)
    parser.add_argument("--compare-report", type=Path)
    parser.add_argument("--runtime-path", action="append", default=[])
    parser.add_argument("--budget", type=float, default=840.)
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args()
    root = args.output.resolve() / str(time.time_ns())
    root.mkdir(parents=True)
    results = []
    passed = False
    enabled_digest = None
    started = time.monotonic()
    environment = dict(os.environ)
    environment["PATH"] = os.pathsep.join(args.runtime_path + [environment.get("PATH", "")])
    base = fixture.BASE | {fixture.COUNT: "0", OPTION: "1", "gap_fill_min_area": "0"}

    def export(name, changes, source=None, plugin=True):
        """Run once with a private datadir and retain the full diagnostics."""
        case = root / name
        case.mkdir()
        if args.embedded_defaults:
            (case / "data/plugins").mkdir(parents=True)
            shutil.copy2(args.embedded_defaults, case / "data/plugins/activated.ini")
        else:
            shutil.copytree(args.prepared, case / "data")
        activation = case / "data/plugins/activated.ini"
        text = activation.read_text(encoding="utf-8")
        if not plugin:
            text, count = re.subn(r"(?m)^nematx\.support\.interface_gap_fill\s*=\s*1\s*$",
                                 "nematx.support.interface_gap_fill = 0", text)
            if count != 1:
                raise AssertionError("Cannot disable the gap-fill provider for baseline comparison")
        activation.write_text(text, encoding="utf-8")
        profile = case / "profile.ini"
        options = base | changes
        if not plugin:
            options.pop(OPTION)
        profile.write_text("".join(f"{key} = {value}\n" for key, value in options.items()), encoding="utf-8")
        model = case / "roof.stl"
        fixture.write_roof(model)
        output = case / "output.gcode"
        command = [str(args.host), "--datadir", str(case / "data"), "--threads", "2", "--load", str(source or profile),
                   "--save", str(case / "saved.ini"), "--export-gcode", "--output", str(output), str(model)]
        remaining = args.budget - (time.monotonic() - started)
        if remaining <= 0:
            raise TimeoutError("Gap-fill gate exhausted its time budget")
        with (case / "host.log").open("wb") as log:
            case_started = time.monotonic()
            run = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=min(120, remaining), env=environment)
        log = (case / "host.log").read_text(encoding="utf-8", errors="replace")
        if run.returncode or not output.is_file() or re.search(r"\[error\]|initialization failed|Unknown option|unknown option|unknown toolpath provider", log):
            raise AssertionError(f"{name}: host failed; see {case / 'host.log'}")
        moves = fixture.read_moves(output.read_text(encoding="utf-8"))
        results.append({"case": name, "changes": changes, "moves": len(moves),
                        "seconds": time.monotonic() - case_started})
        return moves

    try:
        disabled = export("disabled", {OPTION: "0"})
        if export("provider-disabled", {}, plugin=False) != disabled:
            raise AssertionError("Disabled option differs from disabled provider")
        enabled = export("enabled", {})
        enabled_digest = hashlib.sha256(json.dumps(enabled, separators=(",", ":")).encode()).hexdigest()
        if args.compare_report:
            reference = json.loads(args.compare_report.read_text(encoding="utf-8"))
            if reference.get("status") != "passed" or reference.get("enabled_sha256") != enabled_digest:
                raise AssertionError("Embedded and standalone deposited paths differ")
        if enabled == disabled:
            raise AssertionError("Fixture did not produce a gap-fill effect")
        if fixture.sparse_geometry(disabled) != fixture.sparse_geometry(enabled):
            raise AssertionError("Gap fill changed sparse deposition")
        saved = root / "enabled/saved.ini"
        if f"{OPTION} = 1" not in saved.read_text(encoding="utf-8"):
            raise AssertionError("Gap-fill option was not retained")
        if export("reloaded", {}, saved) != enabled:
            raise AssertionError("Reloaded export differs")
        defaults = dict(line.split(" = ", 1) for line in saved.read_text(encoding="utf-8").splitlines() if " = " in line)
        selectors = {role: next(key for key in defaults if key.startswith("exclusive_group_") and role in key)
                     for role in fixture.ROLES}
        legacy = {selectors[role]: provider for role, provider in zip(fixture.ROLES, fixture.LEGACY)}
        if export("legacy", legacy) != enabled:
            raise AssertionError("Legacy and zero-perimeter composition differ")
        for perimeter_count in (1, 3):
            on = export(f"perimeters-{perimeter_count}-on", {fixture.COUNT: str(perimeter_count)})
            off = export(f"perimeters-{perimeter_count}-off", {fixture.COUNT: str(perimeter_count), OPTION: "0"})
            if fixture.sparse_geometry(on) != fixture.sparse_geometry(off):
                raise AssertionError("Gap fill with perimeters changed sparse deposition")
        if export("spaced-enabled", {"support_material_interface_spacing": "0.2"}) != export(
                "spaced-disabled", {OPTION: "0", "support_material_interface_spacing": "0.2"}):
            raise AssertionError("Spaced interface was modified")
        if not args.smoke:
            domains = dict(line.strip().split(" = ", 1) for line in fixture.DOMAIN_TEXT.strip().splitlines())
            domains.pop(fixture.COUNT)
            domains.pop("infill_overlap")
            domains[OPTION] = "0"
            domains["raft_layers"] = "0|3"
            shared = (args.host_source / "src/libslic3r/Plugins/Support/Common/SupportSettings.cpp").read_text(encoding="utf-8")
            wrapper = (HERE.parents[1] / "src/SupportInterfaceGapFill.cpp").read_text(encoding="utf-8")
            expected = set(re.findall(r'\{"([^"]+)", RAW_CO_', shared + wrapper)) | {OPTION}
            if set(domains) != expected:
                raise AssertionError(f"Input domains differ: {set(domains) ^ expected}")
            for key, values in domains.items():
                for index, value in enumerate(values.split("|")):
                    coupled = fixture.COUPLED_DOMAINS.get((key, value), {})
                    export(f"{key}-{index}", {key: value} | coupled)
        passed = True
    finally:
        (root / "report.json").write_text(json.dumps({"status": "passed" if passed else "failed",
            "embedded": bool(args.embedded_defaults),
            "enabled_sha256": enabled_digest, "cases": results}, indent=2), encoding="utf-8")
        print(root)


if __name__ == "__main__":
    main()
