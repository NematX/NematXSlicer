"""Qualify real pressure exports within one 90-second sequential deadline.

The normal DLL and embedded firmware export the same tiny STL in fresh host
processes. The native gate reads those files and exports the GridVarSpeed,
CornerStretch and cooling pairs, catalogue, STL matrix and Print replay.
No export survives as an input to a later run. A native-host DLL package is a
build prerequisite, not an export cache. Each run retains logs and a JSON report.
    main -> stage isolated datadirs -> CLI exports -> native gate -> report
         -> annotations: validate the versioned phase fields
         -> verify_outputs: require the exact exports and record their content
         -> verify_reader_reports: require named, finite, quantum-bounded results
         -> verify_native_profiles: match the fresh CLI base and explicit feature settings
         -> check_package: verify normal/native imports and matching ZIP manifests
    imports -> inspect PE import names without loading a second host DLL
"""
import argparse
import configparser
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import time
from zipfile import ZipFile

DEADLINE_SECONDS = 90


def imports(path):
    """Return the actual PE import DLL names from a Windows plugin file."""
    data = path.read_bytes()
    pe = struct.unpack_from("<I", data, 0x3c)[0]
    assert data[pe:pe + 4] == b"PE\0\0", path
    count = struct.unpack_from("<H", data, pe + 6)[0]
    optional_size = struct.unpack_from("<H", data, pe + 20)[0]
    optional = pe + 24
    directory = optional + (112 if struct.unpack_from("<H", data, optional)[0] == 0x20b else 96)
    sections = []
    for index in range(count):
        row = optional + optional_size + index * 40
        size, rva, raw_size, offset = struct.unpack_from("<IIII", data, row + 8)
        sections.append((rva, max(size, raw_size), offset))
    # PE addresses in directory records are virtual addresses, not file offsets.
    def offset(rva):
        """Map one section-backed virtual address to its file offset."""
        for start, size, raw in sections:
            if start <= rva < start + size:
                return raw + rva - start
        raise AssertionError(f"Unmapped PE address {rva} in {path}")
    record = offset(struct.unpack_from("<I", data, directory + 8)[0])
    names = []
    while any(data[record:record + 20]):
        name = offset(struct.unpack_from("<I", data, record + 12)[0])
        names.append(data[name:data.index(b"\0", name)].decode("ascii"))
        record += 20
    return names


def write_box(path):
    """Write a closed 4 by 4 by 0.4 mm ASCII STL with twelve triangles."""
    vertices = [(x, y, z) for z in (0, .4) for y in (0, 4) for x in (0, 4)]
    faces = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4),
             (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    rows = ["solid box"]
    for a, b, c, d in faces:
        for triangle in ((a, b, c), (a, c, d)):
            rows.append("facet normal 0 0 0\nouter loop")
            rows.extend("vertex %g %g %g" % vertices[i] for i in triangle)
            rows.append("endloop\nendfacet")
    path.write_text("\n".join(rows + ["endsolid box"]) + "\n", encoding="ascii")


def annotations(path):
    """Read versioned numeric phase fields, without interpreting G-code commands."""
    result = []
    required = {"v", "tool", "phase", "v0_mm_s", "v1_mm_s", "e0_mm_s", "e1_mm_s",
                "dt_s", "k_s", "de_nom_mm", "de_pressure_mm"}
    for row in path.read_text(encoding="utf-8").splitlines():
        if ";NX_PRESSURE " not in row:
            continue
        fields = dict(word.split("=", 1) for word in row.split(";NX_PRESSURE ", 1)[1].split())
        assert fields.keys() == required, path
        assert fields["v"] == "1" and fields["phase"] in ("accel", "steady", "decel", "merged"), path
        values = {key: float(value) for key, value in fields.items() if key != "phase"}
        assert all(math.isfinite(value) for value in values.values()), path
        assert values["dt_s"] > 0, path
        result.append(dict(values, phase=fields["phase"]))
    return result


def check_package(build, normal, native, package_id):
    """Check one real native-host fixture against its normal staged package and ZIP."""
    assert (native / "plugin.dll").is_file(), (
        f"Missing native-host fixture {package_id}. Prepare pressure-native-host as documented in README.")
    result = {"normal_imports": imports(normal / "plugin.dll"), "native_imports": imports(native / "plugin.dll")}
    assert "slic3r.dll" in [name.lower() for name in result["normal_imports"]], package_id
    assert "plugins_tests.exe" in [name.lower() for name in result["native_imports"]], package_id
    assert "slic3r.dll" not in [name.lower() for name in result["native_imports"]], package_id
    result["dll_sha256"] = {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in
                            (("normal", normal / "plugin.dll"), ("native", native / "plugin.dll"))}
    manifest = configparser.ConfigParser()
    manifest.read(normal / "version.ini", encoding="utf-8-sig")
    native_manifest = configparser.ConfigParser()
    native_manifest.read(native / "version.ini", encoding="utf-8-sig")
    for section in ("abi", "plugin"):
        assert dict(native_manifest[section]) == dict(manifest[section]), (package_id, "native fixture", section)
        result[section] = dict(manifest[section])
    result["native_manifest_matches"] = True
    normal_dll = (normal / "plugin.dll").read_bytes()
    result["matching_archives"] = []
    for archive in (build / "package/windows-x86_64").glob(f"*/{package_id}_*.zip"):
        with ZipFile(archive) as package:
            for name in package.namelist():
                if name.endswith("plugin.dll") and package.read(name) == normal_dll:
                    zipped_manifest = configparser.ConfigParser()
                    sibling = name.rsplit("/", 1)[0] + "/version.ini" if "/" in name else "version.ini"
                    zipped_manifest.read_string(package.read(sibling).decode("utf-8-sig"))
                    for section in ("abi", "plugin"):
                        assert dict(zipped_manifest[section]) == dict(manifest[section]), (str(archive), section)
                    result["matching_archives"].append(str(archive))
    assert result["matching_archives"], f"No ZIP contains the staged normal DLL for {package_id}"
    return result


def verify_outputs(run):
    """Require every expected export and record its actual content by case."""
    domains = ("machine_limits_usage", "machine_max_acceleration_x", "machine_max_acceleration_y",
               "machine_max_acceleration_z", "machine_max_acceleration_e", "machine_max_acceleration_extruding",
               "machine_max_feedrate_x", "machine_max_feedrate_y", "machine_max_feedrate_z", "machine_max_feedrate_e",
               "gcode_precision_xyz", "gcode_precision_e")
    expected = {"catalogue": {"domain-neutral-baseline": False}, "replay": {}, "two_tools": {}, "cli": {}, "native_features": {}}
    expected["catalogue"].update(("domain-" + key, True) for key in domains)
    expected["catalogue"].update(("domain-neutral-" + key, False) for key in ("gcode_precision_xyz", "gcode_precision_e"))
    for mode in ("rel", "abs"):
        expected["replay"][mode + "-neutral"] = False
        for index in range(5):
            expected["replay"][f"{mode}-edit-{index}"] = index in (0, 2)
            if index < 4:
                expected["replay"][f"{mode}-fresh-{index}"] = index in (0, 2)
        expected["replay"][mode + "-repeat-active"] = True
        expected["replay"][mode + "-repeat-neutral"] = False
        expected["replay"][mode + "-active-compressibility"] = True
    expected["two_tools"] = {"two-tools-neutral": False, "two-tools-active": True}
    for pair in ("standalone", "embedded", "marlin"):
        for state in (("active",) if pair == "marlin" else ("neutral", "active")):
            expected["cli"][f"cli-{pair}-{state}"] = state == "active" and pair != "marlin"
    for pair in ("grid", "corner", "cooling"):
        for state in ("neutral", "active"):
            expected["native_features"][f"native-{pair}-{state}"] = state == "active"
    names = {name + ".gcode" for case in expected.values() for name in case}
    actual = {path.name for path in run.glob("*.gcode")}
    assert actual == names, {"missing": sorted(names - actual), "unexpected": sorted(actual - names)}
    assert len(actual) == 54, len(actual)
    assert not list(run.glob("*.gcode.tmp")), "Unpublished temporary exports remain"
    result = {}
    for case, files in expected.items():
        result[case] = []
        for name, annotated in sorted(files.items()):
            path = run / (name + ".gcode")
            content = path.read_bytes()
            text = content.decode("utf-8")
            # The header depends on the exporter (the test host writes none), so it is not checked.
            assert content, name
            assert "NX_INTERNAL" not in text, name
            # The console exporter writes the statistics for every firmware; the test host writes none.
            if name.startswith("cli-"):
                assert "; total layers count = 2\n" in text, name
            phases = annotations(path)
            assert bool(phases) == annotated, name
            result[case].append({"name": path.name, "bytes": len(content), "phases": len(phases),
                                 "sha256": hashlib.sha256(content).hexdigest()})
    return result


def verify_reader_reports(run, exports):
    """Require every named Reader comparison and its finite, quantum-bounded measurements."""
    names = {entry["name"][7:-6] for entry in exports["catalogue"]
             if not entry["name"].startswith("domain-neutral-")}
    names.update(f"{mode}-{index}" for mode in ("abs", "rel") for index in range(3))
    names.update(("two-tools", "standalone", "embedded", "grid", "corner", "cooling"))
    assert len(names) == 24, names
    paths = {path.stem[7:]: path for path in run.glob("reader-*.json")}
    assert paths.keys() == names, {"missing": sorted(names - paths.keys()), "unexpected": sorted(paths.keys() - names)}
    result = []
    for name in sorted(names):
        values = json.loads(paths[name].read_text(encoding="utf-8"))
        assert values["name"] == name, paths[name]
        for key, value in values.items():
            if key != "name":
                assert isinstance(value, (int, float)) and math.isfinite(value), (name, key, value)
        # Each comparison contains two printed layers. The two-tool case has
        # printing on both tools in each layer, independently of path splitting.
        assert values["cycles_checked"] >= (4 if name == "two-tools" else 2), name
        assert values["e_quantum"] > 0 and values["xyz_quantum"] > 0, name
        assert 0 <= values["max_cycle_error_mm"] <= values["e_quantum"] + 1e-6, name
        assert 0 <= values["max_geometry_error_mm"] <= 2 * values["xyz_quantum"], name
        result.append(values)
    return result


def verify_native_profiles(run):
    """Match each native pair to the supplied fresh CLI base and its explicit overrides."""
    supplied = dict(row.split(" = ", 1) for row in (run / "cli-embedded-neutral.ini").read_text(encoding="utf-8").splitlines() if " = " in row)
    reference = dict(row.split(" = ", 1) for row in (run / "cli-embedded-neutral-effective.ini").read_text(encoding="utf-8").splitlines() if " = " in row)
    # The legacy cooling flag is not serialized; its slowdown and fan settings
    # are checked through the actual saved keys instead. step_gcode_plugin has a
    # single choice, the plan writer, and is not saved either.
    assert supplied.keys() - reference.keys() <= {"cooling", "step_gcode_plugin"}
    result = []
    for pair in ("grid", "corner", "cooling"):
        overrides = {"grid": dict(fill_pattern="gridvarspeed", top_fill_pattern="gridvarspeed", top_solid_layers="1",
                                  fill_gridvarspeed_width_diff="50%", fill_gridvarspeed_zhop="0", top_solid_infill_speed="30"),
                     "corner": dict(stretch_corners="1", stretch_corners_arc="0"),
                     "cooling": dict(slowdown_below_layer_time="20", min_print_speed="3")}[pair]
        for state in ("neutral", "active"):
            name = f"native-{pair}-{state}"
            effective = dict(row.split(" = ", 1) for row in (run / (name + "-effective.ini")).read_text(encoding="utf-8").splitlines() if " = " in row)
            expected = {key: reference[key] for key in supplied if key in reference}
            expected.update(overrides, extruder_pressure_factor="0.02" if state == "active" else "0",
                            split_extrusion_acceleration="1" if state == "active" else "0")
            for key, value in expected.items():
                assert effective[key] == value, (name, key, effective.get(key), value)
            phases = annotations(run / (name + ".gcode"))
            result.append({"name": name, "mode": "native", "phases": len(phases), "effective": expected})
    return result


def main():
    """Run all current qualification cases and retain both success and failure evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", type=Path, required=True)
    parser.add_argument("--host-tests", type=Path, required=True)
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--native-fixture", type=Path, required=True)
    args = parser.parse_args()
    build = args.build.resolve()
    run = build / "temp/pressure-exports" / ("run-" + str(time.time_ns()))
    run.mkdir(parents=True)
    started = time.monotonic()
    report = {"success": False, "commands": [], "scenarios": [], "deadline_seconds": DEADLINE_SECONDS, "run": str(run),
              "expected_qualification_exports": 54,
              "export_count_scope": "Retained qualification files; the thirteen focused native cases export separately."}
    environment = os.environ.copy()
    environment["PATH"] = os.pathsep.join((str(args.host.resolve().parent),
                                           str(args.host_tests.resolve().parent), environment.get("PATH", "")))
    (run / "scratch").mkdir()
    environment["TEMP"] = environment["TMP"] = str(run / "scratch")
    environment["SLIC3R_PRESSURE_QUALIFICATION_OUTPUT"] = str(run)
    environment["SLIC3R_LOGLEVEL"] = "4"

    def command(name, argv):
        """Run one child within the shared deadline and retain all diagnostics."""
        remaining = DEADLINE_SECONDS - (time.monotonic() - started)
        if remaining <= 0:
            raise AssertionError(f"{DEADLINE_SECONDS}-second deadline expired before {name}")
        before = time.monotonic()
        try:
            result = subprocess.run([str(arg) for arg in argv], cwd=run, env=environment,
                                    capture_output=True, timeout=remaining)
        except subprocess.TimeoutExpired as error:
            (run / (name + ".log")).write_bytes((error.stdout or b"") + (error.stderr or b""))
            report["commands"].append({"name": name, "argv": list(map(str, argv)), "timeout": True,
                                       "seconds": time.monotonic() - before})
            raise AssertionError(f"{name}: shared {DEADLINE_SECONDS}-second deadline exceeded") from error
        log = result.stdout + result.stderr
        (run / (name + ".log")).write_bytes(log)
        report["commands"].append({"name": name, "argv": list(map(str, argv)),
                                   "exit": result.returncode, "seconds": time.monotonic() - before})
        if result.returncode:
            raise AssertionError(f"{name}: exit {result.returncode}; see {run / (name + '.log')}")
        return log.decode("utf-8", errors="replace")

    try:
        normal = build / "test-data/nematx_firmware/plugins/nematx-firmware"
        host_cache = (args.host.resolve().parents[2] / "CMakeCache.txt").read_text(encoding="utf-8")
        source = Path(__file__).resolve().parents[3]
        # The host may store this path unnormalized, such as ".../cmake/../../NematXSlicer_plugins".
        embedded = re.search(r"^NEMATX_PLUGINS_SOURCE_DIR:PATH=(.+)$", host_cache, re.M)
        assert embedded and Path(embedded[1].strip()).resolve() == source, embedded
        assert "SLIC3R_NEMATX_BUILTIN_PLUGINS:BOOL=ON" in host_cache
        report["builtin_source"] = str(source)
        native_root = args.native_fixture.resolve() / "test-data/nematx_firmware/plugins"
        report["packages"] = {}
        for package_id, folder in (("nematx-firmware", "nematx_firmware"),
                                   ("nematx-infill-gridvarspeed", "infill_gridvarspeed"),
                                   ("nematx-corner-stretch", "corner_stretch")):
            report["packages"][package_id] = check_package(build,
                build / "test-data" / folder / "plugins" / package_id, native_root / package_id, package_id)
        firmware = report["packages"]["nematx-firmware"]
        for key in ("normal_imports", "native_imports", "dll_sha256", "abi", "native_manifest_matches", "matching_archives"):
            report[key] = firmware[key]
        assert report["abi"]["slic3r_gcode_reader.h"] == "2.3"
        assert report["abi"]["slic3r_gcode_firmware.h"] == "6.1"
        activation = build / "test-data/nematx_firmware/plugins/activated.ini"
        for mode in ("standalone", "embedded"):
            plugins = run / mode / "plugins"
            plugins.mkdir(parents=True)
            shutil.copy2(activation, plugins / "activated.ini")
            if mode == "standalone":
                shutil.copytree(normal, plugins / "nematx-firmware")
            assert (plugins / "nematx-firmware").exists() == (mode == "standalone")
        active = configparser.ConfigParser()
        active.read(activation, encoding="utf-8-sig")
        for provider in ("nematx", "gcode.reader.nematx"):
            assert active["activated"][provider] == "1", provider
        report["activated_ids"] = ["nematx", "gcode.reader.nematx"]
        model = run / "cli-box.stl"
        write_box(model)
        base = dict(
            step_gcode_plugin="gcode.printing_plan_file_writer", gcode_flavor="nematx",
            seam_visibility="0", layer_height="0.2", first_layer_height="0.2", perimeters="1",
            skirts="0", brim_width="0", fill_density="20%", top_solid_layers="0", bottom_solid_layers="0",
            top_solid_min_thickness="0", bottom_solid_min_thickness="0", solid_infill_below_area="0",
            fill_pattern="rectilinear", cooling="0", use_relative_e_distances="1", use_volumetric_e="0",
            slowdown_below_layer_time="0", fan_below_layer_time="0",
            machine_limits_usage="ignore", gcode_precision_xyz="3", gcode_precision_e="5",
            gcode_no_comment="0", gcode_line_number="0", filament_compressibility_factor="0",
            retract_length="0.8", wipe="1", retract_before_travel="0.1", retract_layer_change="1",
            wipe_only_crossing="0", wipe_speed="40",
            infill_speed="30", perimeter_speed="20", external_perimeter_speed="20", first_layer_speed="20",
            first_layer_infill_speed="20", first_layer_min_speed="0", seam_position="rear",
            start_gcode='G71\\nG90\\nG1 G90 X0 Y0 Z0 G91 A0 F6000\\n{"A[SET_POSITION POS=0]"}\\n',
            end_gcode=";Q_VOLUME={extruded_volume[0]}\\n;Q_RETRACT={e_retracted[0]}\\n")
        for pair in ("standalone", "embedded", "marlin"):
            for state in (("active",) if pair == "marlin" else ("neutral", "active")):
                settings = dict(base)
                settings.update(extruder_pressure_factor="0.02" if state == "active" else "0",
                                split_extrusion_acceleration="1" if state == "active" else "0")
                if pair == "marlin":
                    settings.update(gcode_flavor="marlin2", start_gcode="G28\\nG92 E0")
                name = f"cli-{pair}-{state}"
                profile = run / (name + ".ini")
                profile.write_text("\n".join(f"{key} = {value}" for key, value in settings.items()) + "\n", encoding="utf-8")
                output = run / (name + ".gcode")
                mode = "standalone" if pair == "standalone" else "embedded"
                log = command(name, [args.host.resolve(), "--datadir", run / mode, "--loglevel", "4",
                                     "--threads", "1",
                                     "--load", profile, "--save", run / (name + "-effective.ini"),
                                     "--export-gcode", "--output", output, model])
                assert not re.search(r"unknown option|will be disabled|\[error\]|fallback", log, re.I), name
                plugin_time = re.search(r"Loaded the entire plugin library in (\d+) ms", log)
                assert plugin_time, (name, "missing plugin startup duration")
                report["commands"][-1]["plugin_library_seconds"] = int(plugin_time[1]) / 1000
                report["commands"][-1]["other_seconds"] = report["commands"][-1]["seconds"] - int(plugin_time[1]) / 1000
                for provider in ("nematx", "gcode.reader.nematx"):
                    assert f"Activated plugin '{provider}'." in log, (name, provider)
                loaded_normal = str(run / "standalone/plugins/nematx-firmware/plugin.dll")
                assert (loaded_normal in log) == (mode == "standalone"), name
                text = output.read_text(encoding="utf-8")
                effective = dict(row.split(" = ", 1) for row in
                                 (run / (name + "-effective.ini")).read_text(encoding="utf-8").splitlines()
                                 if " = " in row)
                # The plan writer is the only G-code writer, so the host saves no step_gcode_plugin choice.
                checked = ("gcode_flavor", "extruder_pressure_factor",
                           "split_extrusion_acceleration", "filament_compressibility_factor")
                for key in checked:
                    assert effective[key] == settings[key], (name, key, effective[key], settings[key])
                assert ("NX_PRESSURE" in text) == (state == "active" and pair != "marlin"), name
                assert "NX_INTERNAL" not in text, name
                if pair != "marlin":
                    assert "G71\n" in text and "; total layers count = 2" in text, name
                phases = annotations(output)
                report["scenarios"].append({"name": name, "mode": mode, "phases": len(phases),
                                            "effective": {key: effective[key] for key in checked},
                                            "max_abs_pressure_mm": max((abs(p["de_pressure_mm"]) for p in phases), default=0)})
        standalone = annotations(run / "cli-standalone-active.gcode")
        embedded = annotations(run / "cli-embedded-active.gcode")
        assert len(standalone) == len(embedded) > 0
        for left, right in zip(standalone, embedded):
            for key in left:
                assert left[key] == right[key] if key == "phase" else math.isclose(left[key], right[key], rel_tol=1e-12, abs_tol=1e-12)
        native_timeout = DEADLINE_SECONDS - (time.monotonic() - started) - 2
        assert native_timeout > 0, f"No native time remains in the shared {DEADLINE_SECONDS}-second deadline"
        command("native", [sys.executable, Path(__file__).with_name("check_host_pressure.py"),
                           "--host-tests", args.host_tests.resolve(), "--build", args.native_fixture.resolve(),
                           "--selector", "[nematx-pressure],[nematx-pressure-qualification]", "--expected-cases", "17",
                           "--timeout", str(native_timeout), "--feature-fixtures"])
        native_result = (run / "native.log").read_text(encoding="utf-8")
        counts = re.search(r"Passed (?:all )?(\d+) test cases? with (\d+) assertions?", native_result)
        assert counts, "Missing native case and assertion counts"
        report["native_cases"], report["native_assertions"] = map(int, counts.groups())
        first_case = re.search(r"^[\d.]+ s: (.+)$", native_result, re.M)
        assert first_case and first_case[1] == "NematX CLI standalone embedded and feature pairs pass the real Reader"
        report["scenarios"].extend(verify_native_profiles(run))
        report["native_phase_seconds"] = {}
        for row in native_result.splitlines():
            if row.startswith("PRESSURE_TIMING "):
                for key, value in re.findall(r"(\w+_s)=([\d.e+-]+)", row):
                    report["native_phase_seconds"][key] = report["native_phase_seconds"].get(key, 0) + float(value)
        report["verified_exports"] = verify_outputs(run)
        assert sum(map(len, report["verified_exports"].values())) == report["expected_qualification_exports"]
        report["reader_comparisons"] = verify_reader_reports(run, report["verified_exports"])
        assert time.monotonic() - started < DEADLINE_SECONDS, f"{DEADLINE_SECONDS}-second deadline expired while verifying outputs"
        report["success"] = True
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        report["seconds"] = time.monotonic() - started
        report["exports"] = len(list(run.glob("*.gcode")))
        (run / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
