"""End-to-end checks for the first-layer fan program of the legacy slicer.

Each case exports 10 x 10 x 2 mm boxes through the host executable and reads
the fan percentage (M106 S is 0-255) during the first perimeter move of each
layer and tool. With --legacy-executable, the cases using the legacy settings are also
exported by the legacy slicer and must give the same values.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import unittest
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

PROJECT = Path(__file__).resolve().parents[3]
HOST_EXECUTABLE: Path
LEGACY_EXECUTABLE: Path | None = None
PLUGIN_DATADIR: Path
OUTPUT_ROOT: Path
RUNTIME_PATH: list[str] = []

BASE_PROFILE = (
    # Seam visibility is very slow with a Debug host and irrelevant here.
    "seam_visibility = 0",
    "layer_height = 0.2",
    "first_layer_height = 0.2",
    "gcode_comments = 1",
    "skirts = 0",
    "brim_width = 0",
    "perimeters = 1",
    "fill_density = 0%",
    "top_solid_layers = 0",
    "bottom_solid_layers = 0",
    # Only the first-layer program changes the 60% of every role.
    "min_fan_speed = 0",
    "max_fan_speed = 100",
    "default_fan_speed = 60",
    "fan_below_layer_time = 0",
    "slowdown_below_layer_time = 0",
)
WORD = re.compile(r"([A-Z])([+-]?(?:\d+(?:\.\d*)?|\.\d+))")


def write_cube(path: Path) -> None:
    """ASCII STL cube of 10 x 10 x 2 mm."""
    corners = [(x, y, z) for z in (0, 2) for y in (0, 10) for x in (0, 10)]
    quads = ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5))
    lines = ["solid cube"]
    for a, b, c, d in quads:
        for triangle in ((a, b, c), (a, c, d)):
            lines.append(" facet normal 0 0 0\n  outer loop")
            lines += ["   vertex {} {} {}".format(*corners[index]) for index in triangle]
            lines.append("  endloop\n endfacet")
    lines.append("endsolid cube")
    path.write_text("\n".join(lines) + "\n")

# Per-extruder printer settings: the command line does not resize them to the
# extruder count, so a second extruder repeats the host default of each one.
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


def two_extruder_options() -> tuple[str, ...]:
    case_dir = OUTPUT_ROOT / "defaults-saved"
    case_dir.mkdir(parents=True, exist_ok=True)
    (case_dir / "profile.ini").write_text(BASE_PROFILE[0] + "\n", encoding="utf-8")
    saved = case_dir / "defaults.ini"
    environment = dict(os.environ)
    environment["PATH"] = os.pathsep.join(RUNTIME_PATH + [environment.get("PATH", "")])
    subprocess.run([str(HOST_EXECUTABLE), "--datadir", str(PLUGIN_DATADIR), "--load", str(case_dir / "profile.ini"),
                    "--save", str(saved)], capture_output=True, timeout=600, env=environment, check=True)
    defaults = dict(line.split(" = ", 1) for line in saved.read_text(encoding="utf-8").splitlines() if " = " in line)
    options = []
    for key in EXTRUDER_KEYS:
        # String vectors are separated by ';', the other vectors by ','.
        separator = ";" if defaults[key].startswith('"') else ","
        options.append(f"{key} = {defaults[key]}{separator}{defaults[key]}")
    return tuple(options) + ("nozzle_diameter = 0.4,0.4", "filament_diameter = 1.75,1.75", "temperature = 210,210",
                             "first_layer_temperature = 210,210", "wipe_tower = 0")


def write_two_parts(path: Path) -> None:
    """Two 10 x 10 x 2 mm boxes, 10 mm apart, printed by extruders 1 and 2."""
    faces = ((3, 2, 1), (1, 0, 3), (4, 5, 6), (6, 7, 4), (0, 1, 5), (5, 4, 0),
             (1, 2, 6), (6, 5, 1), (2, 3, 7), (7, 6, 2), (3, 0, 4), (4, 7, 3))
    vertices, triangles, config = [], [], ""
    for index, x0 in enumerate((0, 20)):
        base, first = len(vertices), len(triangles)
        vertices += [(x0 + x, y, z) for z in (0, 2) for x, y in ((0, 0), (10, 0), (10, 10), (0, 10))]
        triangles += [tuple(base + corner for corner in face) for face in faces]
        config += (f'<volume firstid="{first}" lastid="{len(triangles) - 1}">'
                   f'<metadata type="volume" key="name" value="part-{index}" />'
                   '<metadata type="volume" key="volume_type" value="ModelPart" />'
                   f'<metadata type="volume" key="extruder" value="{index + 1}" /></volume>')
    model = ('<?xml version="1.0" encoding="UTF-8"?><model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"><resources>'
             '<object id="1" type="model"><mesh><vertices>'
             + "".join(f'<vertex x="{x:g}" y="{y:g}" z="{z:g}" />' for x, y, z in vertices) + "</vertices><triangles>"
             + "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}" />' for a, b, c in triangles)
             + '</triangles></mesh></object></resources><build><item objectid="1" /></build></model>')
    with ZipFile(path, "w", ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml",
                         '<?xml version="1.0" encoding="UTF-8"?>'
                         '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
                         '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml" />'
                         '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml" />'
                         '<Override PartName="/Metadata/Slic3r_PE_model.config" ContentType="application/xml" /></Types>')
        archive.writestr("_rels/.rels",
                         '<?xml version="1.0" encoding="UTF-8"?>'
                         '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
                         '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
                         'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" /></Relationships>')
        archive.writestr("3D/3dmodel.model", model)
        archive.writestr("Metadata/Slic3r_PE_model.config",
                         f'<?xml version="1.0" encoding="UTF-8"?><config><object id="1">{config}</object></config>')


def perimeter_fans(path: Path) -> dict[tuple[int, int], int]:
    """Fan percentage during the first perimeter extrusion of each (tool, layer)."""
    fan = 0
    role = ""
    layer = -1
    tool = 0
    out: dict[tuple[int, int], int] = {}
    e = 0.0
    absolute_e = True
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if raw.startswith(";LAYER_CHANGE"):
            layer += 1
        if raw.startswith(";TYPE:"):
            role = raw[6:].strip()
        code = raw.split(";", 1)[0].strip().upper()
        if not code:
            continue
        command = code.split(" ", 1)[0]
        fields = {key: float(value) for key, value in WORD.findall(code[len(command):])}
        if re.fullmatch(r"T\d+", command):
            tool = int(command[1:])
        elif command == "M106":
            fan = round(fields.get("S", 255.0) * 100 / 255)
        elif command == "M107":
            fan = 0
        elif command == "M82":
            absolute_e = True
        elif command == "M83":
            absolute_e = False
        elif command == "G92" and "E" in fields:
            e = fields["E"]
        elif command in ("G1", "G2", "G3") and "E" in fields and ("X" in fields or "Y" in fields):
            de = fields["E"] - e if absolute_e else fields["E"]
            e = fields["E"] if absolute_e else e + fields["E"]
            if de > 0 and "perimeter" in role.lower():
                out.setdefault((tool, layer), fan)
        elif command in ("G1", "G2", "G3") and "E" in fields:
            e = fields["E"] if absolute_e else e + fields["E"]
    return out


def layer_fans(fans: dict[tuple[int, int], int], tool: int = 0) -> list[int]:
    """The fans of one tool, layer by layer."""
    return [fan for (fan_tool, _), fan in sorted(fans.items(), key=lambda item: item[0][1]) if fan_tool == tool]


def export(executable: Path, name: str, options: tuple[str, ...], model: Path, datadir: Path,
           host: bool = True) -> dict[tuple[int, int], int]:
    case_dir = OUTPUT_ROOT / name
    case_dir.mkdir(parents=True, exist_ok=True)
    profile = case_dir / "profile.ini"
    output = case_dir / "result.gcode"
    merged = {option.split("=", 1)[0].strip(): option for option in BASE_PROFILE + options}
    profile.write_text("\n".join(merged.values()) + "\n", encoding="utf-8")
    output.unlink(missing_ok=True)
    datadir.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ)
    environment["PATH"] = os.pathsep.join(RUNTIME_PATH + [environment.get("PATH", "")])
    # The datadir must be a native absolute path: the Debug host checks it.
    result = subprocess.run([str(executable), "--datadir", str(datadir), "--export-gcode", "--load", str(profile),
                             "--output", str(output), str(model)],
                            capture_output=True, timeout=1800, env=environment)
    log = (result.stdout + result.stderr).decode("utf-8", errors="replace")
    (case_dir / "host.log").write_text(log, encoding="utf-8")
    if result.returncode != 0 or not output.is_file():
        raise AssertionError(f"{name}: export failed (exit {result.returncode}); see {case_dir / 'host.log'}")
    if host:
        for message in ("will be disabled", "unknown option", "[error]"):
            if message in log.lower():
                raise AssertionError(f"{name}: {message!r} in {case_dir / 'host.log'}")
    return perimeter_fans(output)


class FanFirstLayerProgram(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        plugin = PLUGIN_DATADIR / "plugins" / "nematx-fan-first-layer-program" / (
            "plugin.dll" if sys.platform == "win32" else "plugin.so")
        for required in (HOST_EXECUTABLE, plugin):
            if not required.is_file():
                raise FileNotFoundError(f"Missing build or staged package input: {required}")
        OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
        cls.model = OUTPUT_ROOT / "cube.stl"
        write_cube(cls.model)

    def check_program(self, name: str, options: tuple[str, ...], expected: list[int]) -> None:
        fans = layer_fans(export(HOST_EXECUTABLE, name, options, self.model, PLUGIN_DATADIR))
        self.assertEqual(fans[:len(expected)], expected, f"{name}: fan per layer {fans}")
        if LEGACY_EXECUTABLE is not None:
            legacy_dir = OUTPUT_ROOT / f"{name}-legacy"
            legacy = layer_fans(export(LEGACY_EXECUTABLE, legacy_dir.name, options, self.model, legacy_dir / "datadir",
                                       host=False))
            self.assertEqual(legacy[:len(expected)], expected, f"{name}: legacy fan per layer {legacy}")

    def test_defaults_keep_the_host_program(self) -> None:
        # E = 0 and G = -1: the host ramps from disable_fan_first_layers to full_fan_speed_layer.
        fans = layer_fans(export(HOST_EXECUTABLE, "defaults", ("disable_fan_first_layers = 1", "full_fan_speed_layer = 4"),
                                 self.model, PLUGIN_DATADIR))
        self.assertEqual(fans[:5], [0, 20, 40, 60, 60], fans)

    def test_enabled_then_disabled_layers_with_gradient(self) -> None:
        self.check_program("enabled", ("enable_fan_first_layers = 1", "disable_fan_first_layers = 3",
                                       "fan_speed_layer_gradient = 1"), [60, 30, 0, 0, 30, 60, 60])

    def test_gradient_longer_than_the_disabled_layers(self) -> None:
        self.check_program("overlap", ("enable_fan_first_layers = 1", "disable_fan_first_layers = 2",
                                       "fan_speed_layer_gradient = 3"), [60, 30, 0, 15, 30, 45, 60, 60])

    def test_explicit_legacy_defaults(self) -> None:
        self.check_program("legacy-defaults", ("enable_fan_first_layers = 0", "disable_fan_first_layers = 1",
                                               "fan_speed_layer_gradient = 4"), [0, 12, 24, 36, 48, 60, 60])

    def test_enabled_layers_use_a_four_layer_gradient_by_default(self) -> None:
        fans = layer_fans(export(HOST_EXECUTABLE, "sentinel", ("enable_fan_first_layers = 1", "disable_fan_first_layers = 1"),
                                 self.model, PLUGIN_DATADIR))
        self.assertEqual(fans[:8], [60, 0, 12, 24, 36, 48, 60, 60], fans)

    def test_each_extruder_uses_its_own_program(self) -> None:
        model = OUTPUT_ROOT / "two-parts.3mf"
        write_two_parts(model)
        fans = export(HOST_EXECUTABLE, "two-extruders", two_extruder_options() + (
            "default_fan_speed = 60,60", "max_fan_speed = 100,100", "fan_below_layer_time = 0,0",
            "slowdown_below_layer_time = 0,0", "enable_fan_first_layers = 0,1", "disable_fan_first_layers = 1,3",
            "fan_speed_layer_gradient = 4,1"),
            model, PLUGIN_DATADIR)
        self.assertEqual(layer_fans(fans, 0)[:7], [0, 12, 24, 36, 48, 60, 60], fans)
        self.assertEqual(layer_fans(fans, 1)[:7], [60, 30, 0, 0, 30, 60, 60], fans)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-executable", type=Path, required=True)
    parser.add_argument("--legacy-executable", type=Path, help="compare the legacy cases with the legacy slicer")
    parser.add_argument("--build-dir", type=Path, default=PROJECT / ".build")
    parser.add_argument("--runtime-path", action="append", default=[],
                        help="directory added to PATH for the host's DLL dependencies")
    args, unittest_args = parser.parse_known_args()
    build_dir = args.build_dir.resolve()
    HOST_EXECUTABLE = args.host_executable.resolve()
    LEGACY_EXECUTABLE = args.legacy_executable.resolve() if args.legacy_executable else None
    PLUGIN_DATADIR = build_dir / "test-data" / "fan_first_layer_program"
    OUTPUT_ROOT = build_dir / "test-results" / "fan-first-layer-program"
    RUNTIME_PATH = [str(Path(path).resolve()) for path in args.runtime_path]
    unittest.main(argv=[sys.argv[0], *unittest_args], verbosity=2)
