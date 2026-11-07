"""End-to-end checks for the smoothed temperature changes.

Each case exports 10 x 10 x 4.5 mm boxes (15 layers of 0.3 mm) through the host
executable and reads the nozzle setpoint in effect during the extrusions of each
layer, tool and object. Height ranges, modifiers and object settings are written
in a 3MF project. The expected ramps follow the rule of TemperatureRamp.hpp,
computed here independently: a request restarts the ramp from the value printed
on the previous layer, the request layer takes the first step, the target is
reached on the N-th layer, and values are rounded to the nearest degree, halves
towards the target.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import unittest
from dataclasses import dataclass, field
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

PROJECT = Path(__file__).resolve().parents[3]
HOST_EXECUTABLE: Path
PLUGIN_DATADIR: Path
OUTPUT_ROOT: Path
RUNTIME_PATH: list[str] = []

LAYERS = 15
BASE_PROFILE = (
    # Seam visibility is very slow with a Debug host and irrelevant here.
    "seam_visibility = 0",
    "layer_height = 0.3",
    "first_layer_height = 0.3",
    "gcode_comments = 1",
    "gcode_label_objects = octoprint",
    "skirts = 0",
    "brim_width = 0",
    "perimeters = 1",
    "fill_density = 0%",
    "top_solid_layers = 0",
    "bottom_solid_layers = 0",
    "wipe_tower = 0",
    "temperature = 220",
    "first_layer_temperature = 220",
)
WORD = re.compile(r"([A-Z])([+-]?(?:\d+(?:\.\d*)?|\.\d+))")

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


def ramp(generals: list[int], layers: int | list[int]) -> list[int]:
    """Values printed on each layer of one instance and tool, from its general temperature per layer."""
    out = []
    state: list[int] | None = None  # from, target, N, layers counted since the request
    for layer, general in enumerate(generals):
        n = layers[layer] if isinstance(layers, list) else layers
        if state is None:
            state = [general, general, 0, 0]
        elif state[1] != general:
            state = [ramp_value(state), general, n, 1]
        elif state[3] < state[2]:
            state[3] += 1
        out.append(ramp_value(state))
    return out


def ramp_value(state: list[int]) -> int:
    start, target, n, done = state
    if done >= n:
        return target
    step = (target - start) * done
    rounded = (abs(step) * 2 + n) // (2 * n)
    return start + (rounded if step >= 0 else -rounded)


@dataclass
class Volume:
    """Box from (x0, y0, z0) to (x1, y1, z1) in mm; a modifier when settings are given."""
    box: tuple[float, float, float, float, float, float]
    modifier: bool = False
    config: dict[str, str] = field(default_factory=dict)


@dataclass
class Object:
    name: str
    x0: float = 0.0
    config: dict[str, str] = field(default_factory=dict)
    # (min_z, max_z, settings) in mm.
    ranges: list[tuple[float, float, dict[str, str]]] = field(default_factory=list)
    modifiers: list[Volume] = field(default_factory=list)


def write_project(path: Path, objects: list[Object]) -> None:
    """3MF project with one 10 x 10 x 4.5 mm box per object, its settings, height ranges and modifiers."""
    faces = ((3, 2, 1), (1, 0, 3), (4, 5, 6), (6, 7, 4), (0, 1, 5), (5, 4, 0),
             (1, 2, 6), (6, 5, 1), (2, 3, 7), (7, 6, 2), (3, 0, 4), (4, 7, 3))
    meshes, configs, ranges = "", "", ""
    for object_id, obj in enumerate(objects, 1):
        vertices: list[tuple[float, float, float]] = []
        triangles: list[tuple[int, int, int]] = []
        volumes = ""
        for volume in [Volume((obj.x0, 0, 0, obj.x0 + 10, 10, 4.5))] + obj.modifiers:
            x0, y0, z0, x1, y1, z1 = volume.box
            base, first = len(vertices), len(triangles)
            vertices += [(x, y, z) for z in (z0, z1) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
            triangles += [tuple(base + corner for corner in face) for face in faces]
            kind = "ParameterModifier" if volume.modifier else "ModelPart"
            volumes += (f'<volume firstid="{first}" lastid="{len(triangles) - 1}">'
                        f'<metadata type="volume" key="volume_type" value="{kind}" />'
                        + "".join(f'<metadata type="volume" key="{key}" value="{value}" />'
                                  for key, value in volume.config.items()) + "</volume>")
        meshes += (f'<object id="{object_id}" type="model"><mesh><vertices>'
                   + "".join(f'<vertex x="{x:g}" y="{y:g}" z="{z:g}" />' for x, y, z in vertices)
                   + "</vertices><triangles>"
                   + "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}" />' for a, b, c in triangles)
                   + "</triangles></mesh></object>")
        configs += (f'<object id="{object_id}" instances_count="1">'
                    f'<metadata type="object" key="name" value="{obj.name}" />'
                    + "".join(f'<metadata type="object" key="{key}" value="{value}" />'
                              for key, value in obj.config.items()) + volumes + "</object>")
        if obj.ranges:
            ranges += f'<object id="{object_id}">' + "".join(
                f'<range min_z="{low:g}" max_z="{high:g}"><option opt_key="layer_height">0.3</option>'
                + "".join(f'<option opt_key="{key}">{value}</option>' for key, value in settings.items())
                + "</range>" for low, high, settings in obj.ranges) + "</object>"
    model = ('<?xml version="1.0" encoding="UTF-8"?><model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"><resources>' + meshes
             + "</resources><build>"
             + "".join(f'<item objectid="{object_id}" />' for object_id in range(1, len(objects) + 1))
             + "</build></model>")
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
                         f'<?xml version="1.0" encoding="UTF-8"?><config>{configs}</config>')
        if ranges:
            archive.writestr("Metadata/Slic3r_layer_config_ranges.xml",
                             f'<?xml version="1.0" encoding="utf-8"?><objects>{ranges}</objects>')


@dataclass
class Result:
    # (layer, tool, object name) -> distinct setpoints in effect during its extrusions, in order of use.
    setpoints: dict[tuple[int, int, str], list[int]]
    waits: int
    gcode: list[str]

    def layers(self, tool: int = 0, name: str = "box") -> list[int]:
        """Setpoint of the first extrusion of each layer for one tool and object; layers without it are skipped."""
        return [values[0] for (_, value_tool, value_name), values in sorted(self.setpoints.items())
                if value_tool == tool and value_name == name]

    def distinct(self, tool: int = 0, name: str = "box") -> list[list[int]]:
        """Sorted distinct setpoints of each layer for one tool and object."""
        return [sorted(values) for (_, value_tool, value_name), values in sorted(self.setpoints.items())
                if value_tool == tool and value_name == name]


def read_gcode(path: Path) -> Result:
    """Follow setpoints, tools, objects and extrusions in Marlin G-code."""
    setpoint: dict[int, int] = {}
    tool = 0
    layer = -1
    name = ""
    e = 0.0
    absolute_e = True
    waits = 0
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    out: dict[tuple[int, int, str], list[int]] = {}
    for raw in lines:
        if raw.startswith(";LAYER_CHANGE"):
            layer += 1
        elif raw.startswith("; printing object "):
            name = raw[len("; printing object "):].split(" id:", 1)[0].strip()
        elif raw.startswith("; stop printing object"):
            name = ""
        code = raw.split(";", 1)[0].strip().upper()
        if not code:
            continue
        command = code.split(" ", 1)[0]
        fields = {key: float(value) for key, value in WORD.findall(code[len(command):])}
        extruded: int | None = None
        if command in ("M104", "M109") and "S" in fields:
            setpoint[int(fields.get("T", tool))] = int(fields["S"])
            waits += command == "M109"
        elif re.fullmatch(r"T\d+", command):
            tool = int(command[1:])
        elif command == "M82":
            absolute_e = True
        elif command == "M83":
            absolute_e = False
        elif command == "G92" and "E" in fields:
            e = fields["E"]
        elif command in ("G1", "G2", "G3") and "E" in fields:
            de = fields["E"] - e if absolute_e else fields["E"]
            e = fields["E"] if absolute_e else e + fields["E"]
            if de > 0 and ("X" in fields or "Y" in fields):
                extruded = tool
        if extruded is not None and layer >= 0 and name:
            values = out.setdefault((layer, extruded, name), [])
            if setpoint.get(extruded) not in values:
                values.append(setpoint.get(extruded))
    return Result(out, waits, lines)


def environment() -> dict[str, str]:
    env = dict(os.environ)
    env["PATH"] = os.pathsep.join(RUNTIME_PATH + [env.get("PATH", "")])
    return env


def saved_defaults() -> dict[str, str]:
    case_dir = OUTPUT_ROOT / "defaults-saved"
    case_dir.mkdir(parents=True, exist_ok=True)
    (case_dir / "profile.ini").write_text(BASE_PROFILE[0] + "\n", encoding="utf-8")
    saved = case_dir / "defaults.ini"
    subprocess.run([str(HOST_EXECUTABLE), "--datadir", str(PLUGIN_DATADIR), "--load", str(case_dir / "profile.ini"),
                    "--save", str(saved)], capture_output=True, timeout=900, env=environment(), check=True)
    return dict(line.split(" = ", 1) for line in saved.read_text(encoding="utf-8").splitlines() if " = " in line)


def two_extruder_options() -> tuple[str, ...]:
    defaults = saved_defaults()
    options = []
    for key in EXTRUDER_KEYS:
        # String vectors are separated by ';', the other vectors by ','.
        separator = ";" if defaults[key].startswith('"') else ","
        options.append(f"{key} = {defaults[key]}{separator}{defaults[key]}")
    return tuple(options) + ("nozzle_diameter = 0.4,0.4", "filament_diameter = 1.75,1.75")


def export(name: str, options: tuple[str, ...], objects: list[Object]) -> Result:
    case_dir = OUTPUT_ROOT / name
    case_dir.mkdir(parents=True, exist_ok=True)
    profile = case_dir / "profile.ini"
    model = case_dir / "model.3mf"
    output = case_dir / "result.gcode"
    merged = {option.split("=", 1)[0].strip(): option for option in BASE_PROFILE + options}
    profile.write_text("\n".join(merged.values()) + "\n", encoding="utf-8")
    write_project(model, objects)
    output.unlink(missing_ok=True)
    # The datadir must be a native absolute path: the Debug host checks it.
    result = subprocess.run([str(HOST_EXECUTABLE), "--datadir", str(PLUGIN_DATADIR), "--export-gcode", "--load",
                             str(profile), "--output", str(output), str(model)],
                            capture_output=True, timeout=900, env=environment())
    log = (result.stdout + result.stderr).decode("utf-8", errors="replace")
    (case_dir / "host.log").write_text(log, encoding="utf-8")
    if result.returncode != 0 or not output.is_file():
        raise AssertionError(f"{name}: export failed (exit {result.returncode}); see {case_dir / 'host.log'}")
    # The host reports the priorities of its built-in firmware group on every run; that report is not about this plugin.
    checked = "\n".join(line for line in log.lower().splitlines()
                        if "exclusive group 'gcode_firmware_plugin' has incompatible execution priorities" not in line)
    for message in ("will be disabled", "unknown option", "[error]"):
        if message in checked:
            raise AssertionError(f"{name}: {message!r} in {case_dir / 'host.log'}")
    return read_gcode(output)


def without_comments(result: Result) -> list[str]:
    return [line.split(";", 1)[0].rstrip() for line in result.gcode if line.split(";", 1)[0].strip()]


# Height ranges of 1.2 mm (4 layers) from layer 4 on: 210, 200, then 190 from layer 12.
RANGES = [(1.2, 2.4, {"print_temperature": "210"}), (2.4, 3.6, {"print_temperature": "200"}),
          (3.6, 4.5, {"print_temperature": "190"})]
RANGE_GENERALS = [205] + [220] * 3 + [210] * 4 + [200] * 4 + [190] * 3


class SmoothTemperature(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        plugin = PLUGIN_DATADIR / "plugins" / "nematx-smooth-temperature" / (
            "plugin.dll" if sys.platform == "win32" else "plugin.so")
        for required in (HOST_EXECUTABLE, plugin):
            if not required.is_file():
                raise FileNotFoundError(f"Missing build or staged package input: {required}")
        OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)

    def check_ranges(self, name: str, layers: int) -> Result:
        result = export(name, ("first_layer_temperature = 205", f"print_temperature_smooth_change = {layers}"),
                        [Object("box", ranges=RANGES)])
        self.assertEqual(result.layers(), ramp(RANGE_GENERALS, layers), f"{name}: setpoint per layer")
        return result

    def test_setting_is_defined_and_selected_by_default(self) -> None:
        defaults = saved_defaults()
        self.assertEqual(defaults.get("print_temperature_smooth_change"), "0")
        selectors = [value for key, value in defaults.items()
                     if key.startswith("exclusive_group_") and "layer_extrusion_edit_temperature" in key]
        self.assertEqual(selectors, ["nematx_smooth_temperature"], "layer temperature provider selected by default")

    def test_example_of_the_specification(self) -> None:
        # N = 2: first layer 230, then a height range at 210 on the second layer, then 180.
        result = export("example", ("temperature = 180", "first_layer_temperature = 230",
                                    "print_temperature_smooth_change = 2"),
                        [Object("box", ranges=[(0.3, 0.6, {"print_temperature": "210"})])])
        self.assertEqual(result.layers()[:5], [230, 220, 200, 180, 180])

    def test_height_ranges(self) -> None:
        result = self.check_ranges("ranges-3", 3)
        self.assertEqual(result.layers(), [205, 210, 215, 220, 217, 213, 210, 210, 207, 203, 200, 200, 197, 193, 190])

    def test_other_ramp_lengths(self) -> None:
        for layers in (2, 50):
            self.check_ranges(f"ranges-{layers}", layers)

    def test_setting_read_with_the_general_temperature(self) -> None:
        # Only the change into the 200 range carries N = 3; the other changes apply at once.
        ranges = [(low, high, dict(settings)) for low, high, settings in RANGES]
        ranges[1][2]["print_temperature_smooth_change"] = "3"
        result = export("one-range", ("first_layer_temperature = 205",), [Object("box", ranges=ranges)])
        self.assertEqual(result.layers(), [205] + [220] * 3 + [210] * 4 + [207, 203, 200, 200] + [190] * 3)

    def test_zero_and_one_layers_match_the_host_provider(self) -> None:
        objects = [Object("box", ranges=RANGES)]
        options = ("first_layer_temperature = 205",)
        selector = next(key for key in saved_defaults()
                        if key.startswith("exclusive_group_") and "layer_extrusion_edit_temperature" in key)
        host = export("host-provider", options + (f"{selector} = layer_extrusion_edit.temperature.default",
                                                  "print_temperature_smooth_change = 3"), objects)
        self.assertEqual(host.layers(), RANGE_GENERALS, "the host provider ignores the setting")
        for layers in (0, 1):
            smooth = export(f"ranges-{layers}", options + (f"print_temperature_smooth_change = {layers}",), objects)
            self.assertEqual(without_comments(smooth), without_comments(host), f"N = {layers}")

    def test_partial_modifier_is_an_override(self) -> None:
        # Two perimeters keep the modifier's paths apart: regions differing only by temperature share their perimeters.
        modifier = Volume((-1, -1, 1.2, 5, 11, 2.4), True, {"print_temperature": "250", "perimeters": "2"})
        result = export("partial-modifier", ("first_layer_temperature = 205", "print_temperature_smooth_change = 3"),
                        [Object("box", ranges=RANGES, modifiers=[modifier])])
        expected = ramp(RANGE_GENERALS, 3)
        self.assertEqual(result.distinct(), [sorted({value, 250}) if 4 <= layer < 8 else [value]
                                             for layer, value in enumerate(expected)])

    def test_modifier_covering_the_section_is_general(self) -> None:
        modifier = Volume((-1, -1, 1.2, 11, 11, 2.4), True, {"print_temperature": "210"})
        result = export("covering-modifier", ("print_temperature_smooth_change = 2",),
                        [Object("box", modifiers=[modifier])])
        self.assertEqual(result.layers(), ramp([220] * 4 + [210] * 4 + [220] * 7, 2))

    def test_instances_have_their_own_ramps(self) -> None:
        result = export("two-objects", ("print_temperature_smooth_change = 2",),
                        [Object("a", 0, ranges=[(1.2, 4.5, {"print_temperature": "230"})]), Object("b", 40)])
        self.assertEqual(result.layers(name="a"), ramp([220] * 4 + [230] * 11, 2))
        self.assertEqual(result.layers(name="b"), [220] * LAYERS)

    def test_objects_printed_one_by_one_start_at_their_first_layer(self) -> None:
        result = export("sequential", ("complete_objects = 1", "first_layer_temperature = 205",
                                       "print_temperature_smooth_change = 3"),
                        [Object("a", 0), Object("b", 80)])
        for name in ("a", "b"):
            self.assertEqual(result.layers(name=name), ramp([205] + [220] * 14, 3), name)

    def test_two_extruders(self) -> None:
        # Extruder 2 prints layers 4 to 7; extruder 1 ramps to 240 from layer 2 and pauses meanwhile.
        modifier = Volume((-1, -1, 1.2, 11, 11, 2.4), True, {"extruder": "2"})
        objects = [Object("box", ranges=[(0.6, 4.5, {"print_temperature": "240"})], modifiers=[modifier])]
        options = two_extruder_options() + ("temperature = 220,200", "first_layer_temperature = 220,200")
        host = export("two-extruders-0", options + ("print_temperature_smooth_change = 0",), objects)
        smooth = export("two-extruders-4", options + ("print_temperature_smooth_change = 4",), objects)
        self.assertEqual(smooth.layers(0), [220, 220, 225, 230, 235, 240, 240, 240, 240, 240, 240])
        self.assertEqual(smooth.layers(1), [240] * 4)
        self.assertEqual(smooth.waits, host.waits, "the ramp adds no wait")
        ooze = export("two-extruders-ooze", options + ("print_temperature_smooth_change = 4", "ooze_prevention = 1",
                                                       "standby_temperature_delta = -30"), objects)
        self.assertEqual(ooze.layers(0), smooth.layers(0), "standby only suspends the ramp")
        self.assertIn("M104 S190 T0", ooze.gcode, "extruder 1 goes to standby when extruder 2 starts")
        self.assertIn("M109 S235 T0", ooze.gcode, "the returning extruder waits for its next ramp value")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-executable", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path, default=PROJECT / ".build")
    parser.add_argument("--runtime-path", action="append", default=[],
                        help="directory added to PATH for the host's DLL dependencies")
    args, unittest_args = parser.parse_known_args()
    build_dir = args.build_dir.resolve()
    HOST_EXECUTABLE = args.host_executable.resolve()
    PLUGIN_DATADIR = build_dir / "test-data" / "smooth_temperature"
    OUTPUT_ROOT = build_dir / "test-results" / "smooth-temperature"
    RUNTIME_PATH = [str(Path(path).resolve()) for path in args.runtime_path]
    unittest.main(argv=[sys.argv[0], *unittest_args], verbosity=2)
