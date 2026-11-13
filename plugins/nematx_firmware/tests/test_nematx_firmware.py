"""End-to-end checks for the NematX firmware package.

Each case exports a model through the host executable with
gcode_flavor = nematx, then checks the G-code text.
"""

from __future__ import annotations

import argparse
import configparser
import json
import math
import os
import re
import shutil
import subprocess
import sys
import unittest
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tests" / "common"))
from test_outputs import finish_run, new_run, remove_data  # noqa: E402


PROJECT = Path(__file__).resolve().parents[3]
CUBE = PROJECT / "tests" / "second_layer_flow" / "fixtures" / "cube-5mm.stl"
HOST_BUILD: Path
HOST_EXECUTABLE: Path
PLUGIN_DATADIR: Path
OUTPUT_ROOT: Path
RUNTIME_PATH: list[str] = []
LEGACY_EXECUTABLE: Path | None = None

# Settings common to every case: NematX firmware, simple and fast prints.
BASE_PROFILE = (
    "gcode_flavor = nematx",
    # Seam visibility is very slow with a Debug host and irrelevant here.
    "seam_visibility = 0",
    "layer_height = 0.2",
    "first_layer_height = 0.2",
    "temperature = 210",
    "first_layer_temperature = 215",
    "bed_temperature = 60",
    "first_layer_bed_temperature = 60",
    "start_gcode = G28",
    "end_gcode = M84",
)


# Per-extruder printer settings (the host's FFF_TOOL_EXTRUDER keys). The command
# line, unlike the GUI, does not resize them to the extruder count, and a Debug
# host checks that each one has a value per extruder.
EXTRUDER_KEYS = (
    "deretract_speed", "extruder_clearance", "extruder_colour", "extruder_extrusion_multiplier_speed",
    "extruder_fan_offset", "extruder_offset", "extruder_temperature_offset", "max_layer_height",
    "min_layer_height", "retract_before_travel", "retract_before_wipe", "retract_layer_change",
    "retract_length", "retract_length_toolchange", "retract_lift", "retract_lift_above",
    "retract_lift_before_travel", "retract_lift_below", "retract_lift_first_layer", "retract_lift_top",
    "retract_restart_extra", "retract_restart_extra_toolchange", "retract_restart_toolchange_on_perimeter",
    "retract_restart_wipe_toolchange", "retract_speed", "seam_gap", "seam_gap_external",
    "temperature_heat_speed", "tool_name", "travel_lift_before_obstacle", "travel_ramping_lift",
    "travel_slope", "wipe", "wipe_extra_perimeter", "wipe_inside_depth", "wipe_inside_end",
    "wipe_inside_start", "wipe_lift", "wipe_lift_length", "wipe_min", "wipe_only_crossing",
    "wipe_return", "wipe_speed",
)


def host_defaults() -> dict[str, str]:
    """Default value of every setting, as saved by the host (key = serialized value)."""
    case_dir = OUTPUT_ROOT / "defaults"
    case_dir.mkdir(parents=True, exist_ok=True)
    (case_dir / "profile.ini").write_text(BASE_PROFILE[0] + "\n", encoding="utf-8")
    saved = case_dir / "defaults.ini"
    environment = dict(os.environ)
    environment["PATH"] = os.pathsep.join(RUNTIME_PATH + [environment.get("PATH", "")])
    subprocess.run([str(HOST_EXECUTABLE), "--datadir", str(PLUGIN_DATADIR), "--load", str(case_dir / "profile.ini"),
                    "--save", str(saved)], cwd=HOST_BUILD, capture_output=True, timeout=600, env=environment,
                   check=True)
    defaults = {}
    for line in saved.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition(" = ")
        if separator:
            defaults[key] = value
    return defaults


def doubled_extruder_options(defaults: dict[str, str]) -> tuple[str, ...]:
    """Give each per-extruder setting its default value for two extruders."""
    options = []
    for key in EXTRUDER_KEYS:
        value = defaults[key]
        # String vectors are separated by ';', the other vectors by ','.
        separator = ";" if value.startswith('"') else ","
        options.append(f"{key} = {value}{separator}{value}")
    return tuple(options)


def write_3mf(path: Path, boxes: list[tuple[float, float, float, float]], extruders: list[int] | None = None) -> None:
    """Write boxes (x, y, width, height 2 mm) as volumes of one object."""
    vertices: list[str] = []
    triangles: list[str] = []
    faces = (
        (3, 2, 1), (1, 0, 3), (4, 5, 6), (6, 7, 4), (0, 1, 5), (5, 4, 0),
        (1, 2, 6), (6, 5, 1), (2, 3, 7), (7, 6, 2), (3, 0, 4), (4, 7, 3),
    )
    volumes = []
    for x0, y0, width, depth in boxes:
        base = len(vertices)
        first_triangle = len(triangles)
        for x, y, z in ((0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)):
            vertices.append(f'<vertex x="{x0 + x * width:g}" y="{y0 + y * depth:g}" z="{z * 2:g}" />')
        for face in faces:
            triangles.append(f'<triangle v1="{base + face[0]}" v2="{base + face[1]}" v3="{base + face[2]}" />')
        volumes.append((first_triangle, len(triangles) - 1))
    write_mesh_3mf(path, vertices, triangles, volumes, extruders)


def write_mesh_3mf(path, vertices, triangles, volumes, extruders) -> None:
    model = (
        '<?xml version="1.0" encoding="UTF-8"?><model unit="millimeter" xml:lang="en-US" '
        'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"><resources>'
        '<object id="1" type="model"><mesh><vertices>' + "".join(vertices) + "</vertices><triangles>"
        + "".join(triangles) + '</triangles></mesh></object></resources><build><item objectid="1" /></build></model>'
    )
    content_types = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml" />'
        '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml" />'
        '<Override PartName="/Metadata/Slic3r_PE_model.config" ContentType="application/xml" /></Types>'
    )
    relationships = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
        'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" /></Relationships>'
    )
    with ZipFile(path, "w", ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml", content_types)
        archive.writestr("_rels/.rels", relationships)
        archive.writestr("3D/3dmodel.model", model)
        if extruders:
            config = '<?xml version="1.0" encoding="UTF-8"?><config><object id="1">'
            for index, ((first, last), extruder) in enumerate(zip(volumes, extruders)):
                config += (f'<volume firstid="{first}" lastid="{last}">'
                           f'<metadata type="volume" key="name" value="part-{index}" />'
                           '<metadata type="volume" key="volume_type" value="ModelPart" />'
                           f'<metadata type="volume" key="extruder" value="{extruder}" /></volume>')
            config += "</object></config>"
            archive.writestr("Metadata/Slic3r_PE_model.config", config)


def write_gear_3mf(path: Path, teeth: int, radius: float, depth: float) -> None:
    """Write a 1 mm high gear: an outline with many points that simplification keeps."""
    outline = []
    for index in range(2 * teeth):
        angle = math.pi * index / teeth
        r = radius if index % 2 == 0 else radius - depth
        outline.append((r * math.cos(angle), r * math.sin(angle)))
    vertices = [f'<vertex x="{x:.4f}" y="{y:.4f}" z="{z:g}" />' for z in (0, 1) for x, y in outline]
    count = len(outline)
    vertices += ['<vertex x="0" y="0" z="0" />', '<vertex x="0" y="0" z="1" />']
    bottom, top = 2 * count, 2 * count + 1
    triangles = []
    for i in range(count):
        j = (i + 1) % count
        # Side quad, bottom and top fans around the centre (the gear is star-shaped).
        triangles += [f'<triangle v1="{i}" v2="{j}" v3="{count + j}" />',
                      f'<triangle v1="{i}" v2="{count + j}" v3="{count + i}" />',
                      f'<triangle v1="{bottom}" v2="{j}" v3="{i}" />',
                      f'<triangle v1="{top}" v2="{count + i}" v3="{count + j}" />']
    write_mesh_3mf(path, vertices, triangles, [(0, len(triangles) - 1)], None)


def metadata_records(path: Path) -> tuple[list[dict], list[dict], list[str]]:
    """Object and plater JSON records of a G-code file, and their raw lines.

    A record may continue on following "; " comment lines: its JSON text is the
    concatenation of the lines without that prefix.
    """
    objects: list[dict] = []
    platers: list[dict] = []
    raw: list[str] = []
    pending: tuple[list[dict], str] | None = None
    for line in path.read_text(encoding="utf-8").splitlines():
        if pending is None:
            for prefix, records in (("; object:", objects), ("; plater:", platers)):
                if line.startswith(prefix):
                    pending = (records, line[len(prefix):])
                    break
            else:
                continue
        elif line.startswith("; "):
            pending = (pending[0], pending[1] + line[2:])
        else:
            raise AssertionError(f"metadata record interrupted by {line!r}")
        raw.append(line)
        try:
            record = json.loads(pending[1])
        except json.JSONDecodeError:
            continue
        pending[0].append(record)
        pending = None
    if pending is not None:
        raise AssertionError("unfinished metadata record at the end of the file")
    return objects, platers, raw


def profile_text(options: tuple[str, ...]) -> str:
    """Write "key = value" options; a later option replaces an earlier one with the same key."""
    merged: dict[str, str] = {}
    for option in options:
        key = option.split("=", 1)[0].strip()
        merged[key] = option
    return "\n".join(merged.values()) + "\n"


def run_export(name: str, options: tuple[str, ...] = (), model: Path = CUBE,
               arguments: tuple[str, ...] = (), *, datadir: Path | None = None) -> tuple[Path, str, int]:
    case_dir = OUTPUT_ROOT / name
    case_dir.mkdir(parents=True, exist_ok=True)
    profile = case_dir / "profile.ini"
    output = case_dir / "result.gcode"
    profile.write_text(profile_text(BASE_PROFILE + options), encoding="utf-8")
    output.unlink(missing_ok=True)
    environment = dict(os.environ)
    environment["PATH"] = os.pathsep.join(RUNTIME_PATH + [environment.get("PATH", "")])
    # The datadir must be a native absolute path: the Debug host checks it.
    command = [str(HOST_EXECUTABLE), "--datadir", str(datadir or PLUGIN_DATADIR), "--export-gcode",
               "--load", str(profile), "--output", str(output), *arguments, str(model)]
    result = subprocess.run(command, cwd=HOST_BUILD, capture_output=True, timeout=1800, env=environment)
    log = (result.stdout + result.stderr).decode("utf-8", errors="replace")
    (case_dir / "host.log").write_text(log, encoding="utf-8")
    return output, log, result.returncode


def code_lines(path: Path) -> list[str]:
    """Commands of a G-code file, without comments and blank lines."""
    lines = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        code = raw.split(";", 1)[0].strip()
        if code:
            lines.append(code)
    return lines


def command(line: str) -> str:
    return re.split(r"[ =]", line, maxsplit=1)[0]


class FirmwareCalibrationOwnership(unittest.TestCase):
    """Fresh processes must define calibration even with no built-in provider."""

    def test_cold_start_firmware_calibration(self) -> None:
        roles = (
            "default_fan_speed", "perimeter_fan_speed", "external_perimeter_fan_speed",
            "infill_fan_speed", "solid_infill_fan_speed", "top_fan_speed", "gap_fill_fan_speed",
            "bridge_fan_speed", "internal_bridge_fan_speed", "overhangs_fan_speed",
            "support_material_fan_speed", "support_material_interface_fan_speed",
        )
        builtins = ("marlin", "marlin2", "prusa", "reprapfirmware", "sprinter", "klipper")
        nematx = "nematx"
        # Each run gets a new datadir: an earlier successful process must not
        # leave a saved activation/configuration that conceals missing ownership.
        root = new_run(OUTPUT_ROOT, "cold-calibration-")
        cases = ((builtins[0], False), (builtins[-1], False), (nematx, False), (nematx, True))
        passed = []
        for firmware, mixed in cases:
            with self.subTest(firmware=firmware, mixed=mixed):
                case = root / (firmware + ("-mixed" if mixed else "-only"))
                plugins = case / "datadir" / "plugins"
                plugins.mkdir(parents=True)
                shutil.copytree(PLUGIN_DATADIR / "plugins" / "nematx-firmware", plugins / "nematx-firmware")
                activation = configparser.RawConfigParser()
                activation.optionxform = str
                activation.read(PLUGIN_DATADIR / "plugins" / "activated.ini", encoding="utf-8-sig")
                for provider in builtins + (nematx,):
                    activation.set("activated", provider, "1" if mixed or provider == firmware else "0")
                with (plugins / "activated.ini").open("w", encoding="utf-8") as stream:
                    activation.write(stream)

                for request in (20, 0):
                    options = (
                        f"gcode_flavor = {firmware}", "fan_printer_min_speed = 35",
                        "extruder_fan_offset = 0%", "disable_fan_first_layers = 0",
                        "full_fan_speed_layer = 0", "fan_below_layer_time = 0",
                        "slowdown_below_layer_time = 0", "fan_speedup_time = 0", "fan_kickstart = 0",
                        "seam_position = aligned",
                    ) + tuple(f"{role} = {request}" for role in roles)
                    name = str((case / f"request-{request}").relative_to(OUTPUT_ROOT))
                    output, log, code = run_export(name, options, datadir=case / "datadir")
                    self.assertEqual(code, 0, f"{name}: see host.log")
                    self.assertTrue(output.is_file(), name)
                    self.assertNotIn("no active plugin", log.lower(), name)
                    for provider in builtins + (nematx,):
                        self.assertNotIn(f"Disabled plugin '{provider}'", log, name)
                    fans = [line for line in code_lines(output) if re.match(r"M(?:106|107|126)\b", line)]
                    expected = ("M106=89.25" if request else "M106=0") if firmware == nematx else (
                        "M106 S89" if request else "M107")
                    self.assertEqual(fans, [expected], name)
                    if firmware == nematx:
                        self.assertIn("G71", code_lines(output), "NematX, not a fallback dialect")

                    # A second fresh process verifies profile retention and the
                    # actual selected provider, not merely the requested input.
                    saved = output.parent / "saved.ini"
                    environment = dict(os.environ)
                    environment["PATH"] = os.pathsep.join(RUNTIME_PATH + [environment.get("PATH", "")])
                    result = subprocess.run(
                        [str(HOST_EXECUTABLE), "--datadir", str(case / "datadir"),
                         "--load", str(output.parent / "profile.ini"), "--save", str(saved)],
                        cwd=HOST_BUILD, capture_output=True, timeout=600, env=environment)
                    (output.parent / "save.log").write_bytes(result.stdout + result.stderr)
                    self.assertEqual(result.returncode, 0, name)
                    values = dict(line.split(" = ", 1) for line in
                                  saved.read_text(encoding="utf-8").splitlines() if " = " in line)
                    self.assertEqual(values["fan_printer_min_speed"], "35")
                    self.assertEqual(values["gcode_flavor"], firmware)
                # Reached only when every check of this case passed.
                remove_data(case / "datadir")
                passed.append(case)
        finish_run(root, len(passed) == len(cases))


class NematXFirmwareIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        plugin = PLUGIN_DATADIR / "plugins" / "nematx-firmware" / ("plugin.dll" if sys.platform == "win32" else "plugin.so")
        for required in (HOST_EXECUTABLE, CUBE, plugin):
            if not required.is_file():
                raise FileNotFoundError(f"Missing build or staged package input: {required}")

        two_parts = OUTPUT_ROOT / "two-parts.3mf"
        OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
        write_3mf(two_parts, [(0, 0, 10, 10), (20, 0, 10, 10)], extruders=[1, 2])
        # 300 teeth: the object and plater outlines are too long for one line.
        # Shallow teeth: deeper ones are thin walls, and a Debug host asserts in
        # its medial axis on them.
        gear = OUTPUT_ROOT / "gear.3mf"
        write_gear_3mf(gear, teeth=300, radius=40, depth=0.3)

        two_tools = doubled_extruder_options(host_defaults()) + ('nozzle_diameter = 0.4,0.4', 'extruder_axis = "A";"B"', "temperature = 210,200",
                     "first_layer_temperature = 215,205", "filament_diameter = 1.75,1.75", "wipe_tower = 0",
                     "disable_fan_from_unused_extruder = 1", "fan_always_on = 1", "min_fan_speed = 40")
        cases = {
            "relative": (("use_relative_e_distances = 1",), CUBE),
            "absolute": (("use_relative_e_distances = 0",), CUBE),
            "two-tools": (("use_relative_e_distances = 1",) + two_tools, two_parts),
            "numbered": (("use_relative_e_distances = 1", "gcode_line_number = 1", "gcode_no_comment = 1"), CUBE),
            "gears": (("use_relative_e_distances = 1",), gear, ("--duplicate", "2")),
        }
        cls.outputs: dict[str, Path] = {}
        cls.logs: dict[str, str] = {}
        for name, (options, model, *arguments) in cases.items():
            output, log, code = run_export(name, options, model, *arguments)
            if code != 0 or not output.is_file() or output.stat().st_size == 0:
                raise AssertionError(f"{name} export failed (exit {code}); see {OUTPUT_ROOT / name / 'host.log'}")
            cls.outputs[name] = output
            cls.logs[name] = log

    def assert_no_marlin_commands(self, lines: list[str], name: str) -> None:
        for line in lines:
            self.assertNotRegex(line, r"^(M82|M83|M107|M204|G92|G0|T\d+)\b", f"{name}: Marlin command {line!r}")
            self.assertNotRegex(line, r"^G[123] .*\bE-?[\d.]", f"{name}: E axis in {line!r}")

    def test_relative_extrusion_dialect(self) -> None:
        lines = code_lines(self.outputs["relative"])
        self.assert_no_marlin_commands(lines, "relative")
        self.assertLess(lines.index("G71"), 3, "G71 at startup")
        self.assertIn("G90", lines[:3])
        # Temperatures with '=' and separate bare waits.
        self.assertIn("M140=60", lines)
        self.assertEqual(lines[lines.index("M140=60") + 1], "M190")
        self.assertIn("M104=215", lines)
        self.assertEqual(lines[lines.index("M104=215") + 1], "M109")
        self.assertTrue(any(line.startswith("M106=") for line in lines), "first-tool fan uses M106=")
        # Every extrusion is relative inside an absolute move.
        moves = [line for line in lines if line.startswith("G1 ")]
        extrusions = [line for line in moves if re.search(r"\bA-?[\d.]", line)]
        self.assertGreater(len(extrusions), 100)
        for line in extrusions:
            self.assertRegex(line, r"^G1( G90 [XYZ][^G]*)? G91 A-?[\d.]+( F[\d.]+)?$", line)
        for line in moves:
            if re.search(r"\b[XYZ]-?[\d.]", line):
                self.assertTrue(line.startswith("G1 G90 "), line)

    def test_absolute_extrusion_dialect(self) -> None:
        lines = code_lines(self.outputs["absolute"])
        self.assert_no_marlin_commands(lines, "absolute")
        moves = [line for line in lines if line.startswith("G1 ")]
        self.assertFalse([line for line in moves if " G90" in line or " G91" in line], "no inline modes")
        positions = [float(m.group(1)) for line in moves if (m := re.search(r"\bA(-?[\d.]+)", line))]
        self.assertGreater(len(positions), 100)
        self.assertGreater(max(positions), 10.0, "absolute A position grows")
        for line in lines:
            if "SET_POSITION" in line:
                self.assertEqual(line, "A[SET_POSITION POS=0]")

    def test_two_extruders_use_their_axis_heater_and_fan_codes(self) -> None:
        lines = code_lines(self.outputs["two-tools"])
        self.assert_no_marlin_commands(lines, "two-tools")
        self.assertTrue(any(re.search(r" G91 A[\d.]", line) for line in lines), "first extruder on A")
        self.assertTrue(any(re.search(r" G91 B[\d.]", line) for line in lines), "second extruder on B")
        self.assertIn("M124=205", lines, "second heater uses M124=")
        self.assertIn("M129", lines, "second heater wait uses M129")
        self.assertTrue(any(line.startswith("M126=") for line in lines), "second fan uses M126=")
        # disable_fan_from_unused_extruder: at a tool change, the fan of the
        # extruder left is switched off (unless it is already off, as on the
        # first layer). Collect the commands between the two extruders' moves.
        switches: dict[str, list[list[str]]] = {"A": [], "B": []}
        previous_axis, previous_index = None, 0
        for index, line in enumerate(lines):
            match = re.search(r" ([AB])-?[\d.]", line)
            if not match:
                continue
            if previous_axis and match.group(1) != previous_axis:
                switches[match.group(1)].append(lines[previous_index:index])
            previous_axis, previous_index = match.group(1), index
        self.assertTrue(switches["A"] and switches["B"], "both extruders take turns")
        self.assertTrue(any("M106=0" in between for between in switches["B"]), "A's fan off when B starts")
        self.assertTrue(any("M126=0" in between for between in switches["A"]), "B's fan off when A starts")

    def test_line_numbers_without_comments(self) -> None:
        text = self.outputs["numbered"].read_text(encoding="utf-8").splitlines()
        # The session writes the G-code; the exporter's header and statistics
        # around it stay comments without line numbers.
        body = [index for index, line in enumerate(text) if line.strip() and not line.startswith(";")]
        self.assertGreater(len(body), 100)
        self.assertEqual(body, list(range(body[0], body[-1] + 1)), "the G-code is one block")
        for count, index in enumerate(body):
            line = text[index]
            self.assertTrue(line.startswith(f"N{(count + 1) * 10} "), f"line {index}: {line!r}")
            self.assertNotIn(";", line, f"comment kept: {line!r}")
        self.assertIn("; has_support = 0", text[body[-1] + 1:], "statistics after the G-code")

    def test_metadata(self) -> None:
        """The controller reads the layer count and has_support once, in the statistics."""
        lines = self.outputs["relative"].read_text(encoding="utf-8").splitlines()
        counts = [line for line in lines if line.startswith("; total layers count = ")]
        self.assertEqual(len(counts), 1)
        self.assertEqual(int(counts[0].rsplit("=", 1)[1]), sum(1 for line in lines if line == ";LAYER_CHANGE"))
        self.assertEqual([line for line in lines if line.startswith("; has_support = ")], ["; has_support = 0"])

    def test_object_metadata(self) -> None:
        """The cube is described for the controller before the start G-code."""
        path = self.outputs["relative"]
        objects, platers, raw = metadata_records(path)
        self.assertEqual(len(objects), 1)
        self.assertEqual(len(platers), 1)
        lines = path.read_text(encoding="utf-8").splitlines()
        self.assertLess(lines.index(raw[-1]), lines.index("G28"), "metadata before the start G-code")
        name = next(line for line in lines if line.startswith("; printing object ")).removeprefix("; printing object ")
        cube = objects[0]
        self.assertEqual(cube["name"], name, "same name as the object labels")
        self.assertEqual((cube["id"], cube["object_id"], cube["copy"]), ("0", 0, 0))
        self.assertEqual(cube["object_center"], [100, 100, 0])
        self.assertEqual(cube["boundingbox_center"], [100, 100, 2.5])
        self.assertEqual(cube["boundingbox_size"], [5, 5, 5])
        self.assertEqual(cube["scale"], [1, 1, 1])
        corners = [[97.5, 97.5], [102.5, 97.5], [102.5, 102.5], [97.5, 102.5]]
        self.assertEqual(sorted(cube["outline"]), sorted(corners))
        plater = platers[0]
        self.assertEqual(plater["object_center"], [100, 100, 0])
        self.assertEqual(plater["boundingbox_center"], [100, 100, 2.5])
        self.assertEqual(plater["boundingbox_size"], [5, 5, 5])
        self.assertEqual(plater["outline"], corners, "convex hull, counter-clockwise")

    def test_long_object_metadata_is_cut(self) -> None:
        """Two gear instances: long outlines continue on new lines under 4096 characters."""
        objects, platers, raw = metadata_records(self.outputs["gears"])
        self.assertEqual([(o["id"], o["object_id"], o["copy"]) for o in objects], [("0", 0, 0), ("1", 1, 1)])
        self.assertNotEqual(objects[0]["name"], objects[1]["name"])
        for gear in objects:
            self.assertGreaterEqual(len(gear["outline"]), 400, "gear teeth kept by the simplification")
            self.assertEqual(gear["boundingbox_size"][2], 1)
        self.assertEqual(len(platers), 1)
        self.assertGreaterEqual(len(platers[0]["outline"]), 300, "plater hull through the tooth tips")
        self.assertGreater(len(raw), 3 * 2, "object and plater lines are cut")
        for line in raw:
            self.assertLessEqual(len(line), 4096, "line longer than the controller accepts")
            self.assertLessEqual(len(line), 4000 + 30)

    def test_rejected_configurations(self) -> None:
        for name, options, message in (
            ("firmware-retraction", ("use_firmware_retraction = 1",), "firmware retraction"),
            ("three-extruders", ("nozzle_diameter = 0.4,0.4,0.4", "filament_diameter = 1.75,1.75,1.75"),
             "at most 2 extruders"),
            ("bad-axis", ('extruder_axis = "X"',), "extruder_axis"),
        ):
            output, log, code = run_export(name, options)
            self.assertNotEqual(code, 0, f"{name} must be rejected; see {OUTPUT_ROOT / name / 'host.log'}")
            self.assertFalse(output.exists(), f"{name} must not produce G-code")
            self.assertIn(message, log, f"{name}: expected {message!r}; see {OUTPUT_ROOT / name / 'host.log'}")

    def test_same_commands_as_the_legacy_slicer(self) -> None:
        """Compare the command words with a legacy export (gcode_flavor = nematx) of the same cube."""
        if LEGACY_EXECUTABLE is None:
            self.skipTest("pass --legacy-executable to compare with the legacy slicer")
        case_dir = OUTPUT_ROOT / "legacy"
        case_dir.mkdir(parents=True, exist_ok=True)
        profile = case_dir / "profile.ini"
        output = case_dir / "result.gcode"
        legacy_profile = ("gcode_flavor = nematx",) + BASE_PROFILE[1:] + ("use_relative_e_distances = 1",)
        profile.write_text(profile_text(legacy_profile), encoding="utf-8")
        output.unlink(missing_ok=True)
        result = subprocess.run([str(LEGACY_EXECUTABLE), "--datadir", str(case_dir / "datadir"), "--export-gcode",
                                 "--load", str(profile), "--output", str(output), str(CUBE)],
                                capture_output=True, timeout=600)
        self.assertEqual(result.returncode, 0, "legacy export failed")
        legacy = {command(line) for line in code_lines(output)}
        current = {command(line) for line in code_lines(self.outputs["relative"])}
        self.assertEqual(current - legacy, set(), "commands the legacy slicer never wrote")
        self.assertEqual(legacy - current, set(), "legacy commands missing from the NematX firmware")
        # Same object and plater descriptions. The legacy plater line starts
        # with a stray comma ("plater:{,"), removed by the NematX firmware; the
        # outline may start at another corner.
        legacy_text = output.read_text(encoding="utf-8").replace("; plater:{,", "; plater:{")
        (output.parent / "fixed.gcode").write_text(legacy_text, encoding="utf-8")
        legacy_objects, legacy_platers, _ = metadata_records(output.parent / "fixed.gcode")
        objects, platers, _ = metadata_records(self.outputs["relative"])
        for record in legacy_objects + objects:
            record["outline"] = sorted(record["outline"])
        self.assertEqual(objects, legacy_objects)
        self.assertEqual(platers, legacy_platers)

    def test_reader_reports_no_unknown_command(self) -> None:
        for name, log in self.logs.items():
            self.assertNotRegex(log.lower(), r"unknown (g-code )?command", f"{name}: reader diagnostic in log")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-dir", type=Path, required=True)
    parser.add_argument("--host-executable", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path, default=PROJECT / ".build")
    parser.add_argument("--plugin-datadir", type=Path)
    parser.add_argument("--runtime-path", action="append", default=[],
                        help="directory added to PATH for the host's DLL dependencies")
    parser.add_argument("--legacy-executable", type=Path,
                        help="legacy NematXSlicer console executable, for the command comparison")
    args, unittest_args = parser.parse_known_args()
    LEGACY_EXECUTABLE = args.legacy_executable.resolve() if args.legacy_executable else None

    build_dir = args.build_dir.resolve()
    HOST_BUILD = args.host_dir.resolve()
    HOST_EXECUTABLE = args.host_executable.resolve()
    PLUGIN_DATADIR = (args.plugin_datadir or build_dir / "test-data" / "nematx_firmware").resolve()
    OUTPUT_ROOT = build_dir / "test-results" / "nematx-firmware"
    RUNTIME_PATH = [str(Path(path).resolve()) for path in args.runtime_path]
    unittest.main(argv=[sys.argv[0], *unittest_args], verbosity=2)
