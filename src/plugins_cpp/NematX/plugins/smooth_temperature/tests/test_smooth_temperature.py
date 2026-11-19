"""Fast package E2E gate: micro-boxes, real 3MF instances and nine host exports.

The common scene combines ranges, partial/full modifiers, independent objects,
copies, a synthetic skirt, policy changes and two tools/standby. Separate small
scenes compare the host with N=0/1 and SEMM and exercise sequential copies.
Single-tool scenes retain smoothing with SEMM checked and one or two configured tools.
Expected sequences are written explicitly; the C++ tests cover the large
arithmetic matrix without slicing it. The first export also saves the default
catalogue for all scenes; the eight others then run in parallel, each with its own
datadir copy, before the tests read their cached results. All share one budget.
A passed export drops its datadir; a passed run is removed, a failed one is kept.

    main -> prefetch -> immediate_result, then *_result in parallel
                     -> export -> write_project / HostRunner.run -> parse_gcode
         -> SmoothTemperature -> cached *_result -> Result.series
         -> timings.json (including command and fixture/parser times)
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import functools
import json
import os
import shutil
import sys
import time
import unittest
from dataclasses import dataclass, field
from pathlib import Path
from xml.sax.saxutils import escape, quoteattr
from zipfile import ZIP_DEFLATED, ZipFile

from gcode_reader import Result, parse_gcode
from host_runner import HostRunner

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tests" / "common"))
from test_outputs import finish_run, new_run, remove_data  # noqa: E402

PROJECT = Path(__file__).resolve().parents[3]
HOST_EXECUTABLE: Path
PLUGIN_DATADIR: Path
OUTPUT_ROOT: Path
RUNNER: HostRunner
STAGES: list[dict] = []

LAYERS = 10
BASE_PROFILE = (
    "seam_visibility = 0",  # Visibility calculations do not affect temperature decisions.
    "layer_height = 0.3", "first_layer_height = 0.3", "gcode_comments = 1",
    "gcode_label_objects = octoprint", "skirts = 0", "brim_width = 0", "perimeters = 1",
    "fill_density = 0%", "top_solid_layers = 0", "bottom_solid_layers = 0", "wipe_tower = 0",
    "temperature = 220", "first_layer_temperature = 220", "gcode_flavor = marlin2",
)
# The CLI does not resize per-extruder printer vectors. Repeat the actual defaults,
# not a second hard-coded printer profile that would drift from the host.
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


@dataclass
class Volume:
    """Axis-aligned local box; a parameter modifier when modifier is true."""
    box: tuple[float, float, float, float, float, float]
    modifier: bool = False
    config: dict[str, str] = field(default_factory=dict)


@dataclass
class Object:
    """One 4 mm wide mesh with optional copies sharing the same 3MF object id."""
    name: str
    x0: float = 0.0
    config: dict[str, str] = field(default_factory=dict)
    ranges: list[tuple[float, float, dict[str, str]]] = field(default_factory=list)
    modifiers: list[Volume] = field(default_factory=list)
    height: float = 3.0
    copies: tuple[float, ...] = (0.0,)


def write_project(path: Path, objects: list[Object]) -> None:
    """Write tiny meshes once and translate build items to create real copies."""
    faces = ((3, 2, 1), (1, 0, 3), (4, 5, 6), (6, 7, 4), (0, 1, 5), (5, 4, 0),
             (1, 2, 6), (6, 5, 1), (2, 3, 7), (7, 6, 2), (3, 0, 4), (4, 7, 3))
    meshes, configs, ranges, build = "", "", "", ""
    for object_id, obj in enumerate(objects, 1):
        vertices, triangles, volumes = [], [], ""
        for volume in [Volume((0, 0, 0, 4, 4, obj.height))] + obj.modifiers:
            x0, y0, z0, x1, y1, z1 = volume.box
            base, first = len(vertices), len(triangles)
            vertices += [(x, y, z) for z in (z0, z1) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
            triangles += [tuple(base + corner for corner in face) for face in faces]
            kind = "ParameterModifier" if volume.modifier else "ModelPart"
            volumes += (f'<volume firstid="{first}" lastid="{len(triangles) - 1}">'
                        f'<metadata type="volume" key="volume_type" value="{kind}" />'
                        + "".join(f'<metadata type="volume" key={quoteattr(key)} value={quoteattr(value)} />'
                                  for key, value in volume.config.items()) + "</volume>")
        meshes += (f'<object id="{object_id}" type="model"><mesh><vertices>'
                   + "".join(f'<vertex x="{x:g}" y="{y:g}" z="{z:g}" />' for x, y, z in vertices)
                   + "</vertices><triangles>"
                   + "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}" />' for a, b, c in triangles)
                   + "</triangles></mesh></object>")
        configs += (f'<object id="{object_id}" instances_count="{len(obj.copies)}">'
                    f'<metadata type="object" key="name" value={quoteattr(obj.name)} />'
                    + "".join(f'<metadata type="object" key={quoteattr(key)} value={quoteattr(value)} />'
                              for key, value in obj.config.items()) + volumes + "</object>")
        if obj.ranges:
            ranges += f'<object id="{object_id}">' + "".join(
                f'<range min_z="{low:g}" max_z="{high:g}"><option opt_key="layer_height">0.3</option>'
                + "".join(f'<option opt_key={quoteattr(key)}>{escape(value)}</option>' for key, value in settings.items())
                + "</range>" for low, high, settings in obj.ranges) + "</object>"
        build += "".join(f'<item objectid="{object_id}" transform="1 0 0 0 1 0 0 0 1 {obj.x0 + offset:g} 0 0" />'
                         for offset in obj.copies)
    model = ('<?xml version="1.0" encoding="UTF-8"?><model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
             '<metadata name="slic3rpe:Version3mf">1</metadata><resources>' + meshes
             + "</resources><build>" + build + "</build></model>")
    with ZipFile(path, "w", ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml", '<?xml version="1.0" encoding="UTF-8"?>'
                         '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
                         '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml" />'
                         '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml" />'
                         '<Override PartName="/Metadata/SuperSlicer_model.config" ContentType="application/xml" /></Types>')
        archive.writestr("_rels/.rels", '<?xml version="1.0" encoding="UTF-8"?>'
                         '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
                         '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
                         'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" /></Relationships>')
        archive.writestr("3D/3dmodel.model", model)
        archive.writestr("Metadata/SuperSlicer_model.config", f'<?xml version="1.0" encoding="UTF-8"?><config>{configs}</config>')
        if ranges:
            archive.writestr("Metadata/SuperSlicer_layer_config_ranges.xml",
                             f'<?xml version="1.0" encoding="utf-8"?><objects>{ranges}</objects>')


@functools.lru_cache(maxsize=1)
def immediate_result() -> Result:
    """Export N=0/1 and save the default catalogue in the same host invocation."""
    return export("immediate", (), parity_scene(), save_defaults=True)


def saved_defaults() -> dict[str, str]:
    """Read the catalogue saved by the first export, without a separate host launch."""
    immediate_result()
    saved = OUTPUT_ROOT / "immediate" / "defaults.ini"
    return dict(line.split(" = ", 1) for line in saved.read_text(encoding="utf-8").splitlines() if " = " in line)


def selector() -> str:
    """Find the real temperature group setting instead of hard-coding its generated key."""
    return next(key for key in saved_defaults()
                if key.startswith("exclusive_group_") and "layer_extrusion_edit_temperature" in key)


def two_extruder_options() -> tuple[str, ...]:
    """Extend the printer's real default vectors to two nozzles."""
    defaults = saved_defaults()
    options = []
    for key in EXTRUDER_KEYS:
        separator = ";" if defaults[key].startswith('"') else ","
        options.append(f"{key} = {defaults[key]}{separator}{defaults[key]}")
    return tuple(options) + ("nozzle_diameter = 0.4,0.4", "filament_diameter = 1.75,1.75")


def export(name: str, options: tuple[str, ...], objects: list[Object], save_defaults: bool = False) -> Result:
    """Export one scene and reject host errors, stale files and unreadable heater state."""
    start = time.monotonic()
    RUNNER.remaining()
    case = OUTPUT_ROOT / name
    case.mkdir()
    profile, model, output = case / "profile.ini", case / "model.3mf", case / "result.gcode"
    merged = {option.split("=", 1)[0].strip(): option for option in BASE_PROFILE + options}
    profile.write_text("\n".join(merged.values()) + "\n", encoding="utf-8")
    write_project(model, objects)
    # Exports run in parallel: each owns a copy of the template datadir, which the host fills at startup.
    datadir = case / "datadir"
    shutil.copytree(PLUGIN_DATADIR, datadir)
    prepared = time.monotonic()
    # These handfuls of rectangular paths do not benefit from more host threads.
    actions = ["--save", str(case / "defaults.ini")] if save_defaults else []
    RUNNER.run([str(HOST_EXECUTABLE), "--threads", "1", "--dont-arrange", "--datadir", str(datadir), *actions, "--export-gcode", "--load", str(profile),
                "--output", str(output), str(model)], case / "host.log")
    parsing = time.monotonic()
    log = (case / "host.log").read_text(encoding="utf-8", errors="replace")
    # This known firmware-group diagnostic is unrelated to the temperature provider.
    # Keep the complete log, and do not ignore any other error or disabled plugin.
    checked = "\n".join(line for line in log.lower().splitlines()
                        if "exclusive group 'gcode_flavor' has incompatible execution priorities" not in line)
    for message in ("will be disabled", "unknown option", "[error]"):
        if message in checked:
            raise AssertionError(f"{name}: {message!r}; see {case / 'host.log'}")
    result = parse_gcode(output.read_text(encoding="utf-8").splitlines())
    if not result.setpoints:
        raise AssertionError(f"{name}: no object extrusion was read")
    STAGES.append({"case": name, "fixture_seconds": prepared - start, "parse_seconds": time.monotonic() - parsing})
    RUNNER.remaining()
    remove_data(datadir)
    return result


def common_scene() -> list[Object]:
    """One scene exercises several rules while the provider and N vary independently."""
    config = {"print_first_layer_temperature": "205"}
    ranges = [(0.6, 1.2, {"print_temperature": "210"}), (1.2, 1.8, {"print_temperature": "230"}),
              (1.8, 3.0, {"print_temperature": "200"})]
    return [Object("partial", 8, config, ranges,
                   [Volume((-1, -1, 0.6, 2, 5, 1.2), True, {"print_temperature": "250", "perimeters": "2"})]),
            Object("cover", 16, config, modifiers=[Volume((-1, -1, 0.6, 5, 5, 1.2), True,
                                                                        {"print_temperature": "210"})], height=2.4),
            Object("copies", 24, config, ranges, copies=(0, 8))]


def parity_scene() -> list[Object]:
    """Compare both immediate lengths on a deterministic single-object range scene."""
    ranges = [(low, high, dict(settings, print_temperature_smooth_change=str(n)))
              for (low, high, settings), n in zip(common_scene()[0].ranges, (0, 1, 0))]
    return [Object("one", config={"print_first_layer_temperature": "205", "print_temperature_smooth_change": "1"},
                   ranges=ranges)]


def tool_scene() -> list[Object]:
    """T0 leaves this instance on layers 2–5 while T1 prints it."""
    return [Object("tools", 80, ranges=[(0.3, 3, {"print_temperature": "240"})],
                   modifiers=[Volume((-1, -1, 0.6, 5, 5, 1.8), True, {"extruder": "2"})])]


@functools.lru_cache(maxsize=1)
def main_result() -> Result:
    """One host export serves the independent modifier, policy and tool assertions."""
    objects = common_scene() + [
        Object("example", 40, {"print_first_layer_temperature": "230", "print_temperature": "180",
                               "print_temperature_smooth_change": "2"}, [(0.3, 0.6, {"print_temperature": "210"})], height=1.5),
        Object("cancel", 48, {"print_temperature": "200"},
               [(0.3, 0.9, {"print_temperature": "230", "print_temperature_smooth_change": "8"}),
                (0.9, 3, {"print_temperature": "230", "print_temperature_smooth_change": "0"})], height=1.5),
        Object("range_n", 56, {"print_temperature_smooth_change": "0"},
               [(0.6, 1.8, {"print_temperature": "200", "print_temperature_smooth_change": "3"})], height=2.4),
        Object("long", 64, {"print_temperature_smooth_change": "50"}, [(0.3, 3, {"print_temperature": "230"})], height=1.8),
    ]
    immediate = common_scene()[0]
    immediate.name, immediate.x0 = "immediate_partial", 72
    immediate.height = 2.4
    immediate.config = dict(immediate.config, print_temperature_smooth_change="0")
    objects += [immediate] + tool_scene()
    # The tool scenario has N=4; the other objects keep their own policies or N=3.
    objects[-1].config["print_temperature_smooth_change"] = "4"
    options = two_extruder_options() + ("temperature = 220,200", "first_layer_temperature = 220,200",
                                       "ooze_prevention = 1", "standby_temperature_delta = -30",
                                       "skirts = 1", "skirt_height = 4", "print_temperature_smooth_change = 3")
    return export("smooth", options, objects)


@functools.lru_cache(maxsize=1)
def host_result() -> Result:
    """The host provider on the parity scene, to compare with N=0 and N=1."""
    return export("host", (f"{selector()} = layer_extrusion_edit.temperature.default",
                           "print_temperature_smooth_change = 3"), parity_scene())


@functools.lru_cache(maxsize=1)
def sequential_result() -> Result:
    """Two copies of one mesh printed one after the other."""
    return export("sequential", ("complete_objects = 1", "print_temperature_smooth_change = 3"),
                  [Object("same", config={"print_first_layer_temperature": "205"}, height=1.5, copies=(0, 80))])


@functools.lru_cache(maxsize=1)
def tools_base_result() -> Result:
    """The host provider on the tool scene, with a companion object and standby."""
    options = two_extruder_options() + ("temperature = 220,200", "first_layer_temperature = 220,200",
                                       "ooze_prevention = 1", "standby_temperature_delta = -30")
    return export("tools-base", options + ("print_temperature_smooth_change = 4",
                                           f"{selector()} = layer_extrusion_edit.temperature.default"),
                  tool_scene() + [Object("companion")])


def semm_options() -> tuple[str, ...]:
    """A real two-material SEMM plan, without an expensive wipe tower."""
    return two_extruder_options() + ("single_extruder_multi_material = 1", "print_temperature_smooth_change = 4")


@functools.lru_cache(maxsize=1)
def semm_base_result() -> Result:
    """The host provider on the tool scene in SEMM mode."""
    return export("semm-base", semm_options() + (f"{selector()} = layer_extrusion_edit.temperature.default",),
                  tool_scene())


@functools.lru_cache(maxsize=1)
def semm_smooth_result() -> Result:
    """The smoothing provider on the tool scene in SEMM mode."""
    return export("semm-smooth", semm_options(), tool_scene())


@functools.lru_cache(maxsize=2)
def single_tool_semm_result(configured_tools: int) -> Result:
    """Keep a real ramp when the checked SEMM setting has only one visited tool."""
    options = two_extruder_options() if configured_tools == 2 else ()
    options += ("single_extruder_multi_material = 1", "print_temperature_smooth_change = 3")
    return export(f"single-tool-semm-{configured_tools}", options,
                  [Object("single", config={"print_first_layer_temperature": "205"}, height=1.5)])


def prefetch() -> None:
    """Run the first export, whose saved catalogue the others read, then the others in parallel."""
    immediate_result()
    exports = (main_result, host_result, sequential_result, tools_base_result, semm_base_result, semm_smooth_result,
               functools.partial(single_tool_semm_result, 1), functools.partial(single_tool_semm_result, 2))
    with ThreadPoolExecutor(max_workers=len(exports)) as pool:
        for future in [pool.submit(result) for result in exports]:
            future.result()


class SmoothTemperature(unittest.TestCase):
    """Check real extrusions, not merely the presence of temperature commands."""

    def expect(self, result: Result, name: str, values: list[int], tool: int = 0, copy: int = 0,
               positions: list[int] | None = None) -> None:
        """Require every expected printed layer and a single setpoint throughout it."""
        self.assertEqual(result.series(name, len(values), tool, copy, positions), [[value] for value in values], (name, tool, copy))

    def test_01_defaults(self) -> None:
        """The package setting and default exclusive-group selection are exposed."""
        self.assertEqual(saved_defaults().get("print_temperature_smooth_change"), "0")
        self.assertEqual(saved_defaults()[selector()], "nematx_smooth_temperature")

    def test_02_common_scene(self) -> None:
        """Compare complete host commands and read modifier/copy/skirt setpoints."""
        host = host_result()
        self.expect(host, "one", [205, 220, 210, 210, 230, 230, 200, 200, 200, 200])
        immediate = immediate_result()
        self.assertEqual(immediate.commands(), host.commands(), "N=0 and N=1: complete host G-code")
        smooth = main_result()
        expected = [205, 210, 210, 210, 217, 223, 215, 208, 200, 200]
        self.expect(smooth, "copies", expected, copy=0)
        self.expect(smooth, "copies", expected, copy=1)
        self.expect(smooth, "cover", [205, 210, 210, 210, 213, 217, 220, 220])
        self.assertEqual([sorted(set(values)) for values in smooth.series("partial", LAYERS)],
                         [sorted({value, 250}) if 2 <= layer < 4 else [value] for layer, value in enumerate(expected)])
        immediate_values = [205, 220, 210, 210, 230, 230, 200, 200]
        self.assertEqual([sorted(set(values)) for values in smooth.series("immediate_partial", len(immediate_values))],
                         [sorted({value, 250}) if 2 <= layer < 4 else [value] for layer, value in enumerate(immediate_values)])
        self.assertTrue(any(kind.lower() in ("skirt", "skirt/brim") for _, kind, _ in smooth.synthetic), "a real synthetic skirt was printed")
        self.assertEqual(smooth.synthetic_heaters, [], "the synthetic skirt adds no heater request")

    def test_03_policy_changes(self) -> None:
        """Restart, cancel at unchanged target, per-range N and an unfinished long ramp."""
        result = main_result()
        self.expect(result, "example", [230, 220, 200, 180, 180])
        self.expect(result, "cancel", [200, 204, 208, 230, 230])
        self.expect(result, "range_n", [220, 220, 213, 207, 200, 200, 220, 220])
        self.expect(result, "long", [220, 220, 220, 221, 221, 221])

    def test_04_sequential_copies(self) -> None:
        """Two copies of one mesh start immediately, even after another copy's ramp."""
        result = sequential_result()
        for copy in (0, 1):
            self.expect(result, "same", [205, 210, 215, 220, 220], copy=copy)

    def test_05_tools_and_semm(self) -> None:
        """Standby is immediate; an absent tool resumes its ramp with the base wait."""
        host = tools_base_result()
        self.expect(host, "tools", [220, 240, 240, 240, 240, 240], positions=[0, 1, 6, 7, 8, 9])
        smooth = main_result()
        self.expect(smooth, "tools", [220, 225, 230, 235, 240, 240], positions=[0, 1, 6, 7, 8, 9])
        self.expect(smooth, "tools", [240] * 4, tool=1, positions=[2, 3, 4, 5])
        self.assertEqual(smooth.wait_sites(), host.wait_sites(), "same wait locations as the base, not merely the same count")
        self.assertIn("M104 S190 T0", smooth.commands(), "inactive T0 keeps the base standby temperature")
        for layer, tool, temperature in smooth.waits:
            if layer < 0:
                continue
            first = next(values[0] for key, values in smooth.setpoints.items() if key[:2] == (layer, tool))
            self.assertEqual(temperature, first, "a returning tool waits for the first root's checked ramp value")
        semm_base, semm_smooth = semm_base_result(), semm_smooth_result()
        self.assertEqual(semm_smooth.commands(), semm_base.commands(), "SEMM leaves all base decisions unchanged")
        self.expect(semm_smooth, "tools", [220, 240, 240, 240, 240, 240], positions=[0, 1, 6, 7, 8, 9])
        self.expect(semm_smooth, "tools", [240] * 4, tool=1, positions=[2, 3, 4, 5])


    def test_06_single_used_tool_keeps_smoothing_with_semm_checked(self) -> None:
        """Configured tool slots do not disable smoothing unless two tools are visited."""
        for configured_tools in (1, 2):
            with self.subTest(configured_tools=configured_tools):
                result = single_tool_semm_result(configured_tools)
                self.expect(result, "single", [205, 210, 215, 220, 220])
                self.assertEqual({key[1] for key in result.setpoints}, {0})


def main() -> int:
    """Run in an isolated datadir and retain a timing report even when a test fails."""
    global HOST_EXECUTABLE, PLUGIN_DATADIR, OUTPUT_ROOT, RUNNER
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-executable", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path, default=PROJECT / ".build")
    parser.add_argument("--runtime-path", action="append", default=[])
    parser.add_argument("--builtin", action="store_true", help="qualify the embedded provider without a standalone package")
    parser.add_argument("--budget-seconds", type=float, default=90)
    args, unittest_args = parser.parse_known_args()
    HOST_EXECUTABLE = args.host_executable.resolve()
    build = args.build_dir.resolve()
    env = dict(os.environ)
    runtime = [str(HOST_EXECUTABLE.parent)] + [str(Path(path).resolve()) for path in args.runtime_path]
    key = "PATH" if os.name == "nt" else "DYLD_LIBRARY_PATH" if sys.platform == "darwin" else "LD_LIBRARY_PATH"
    env[key] = os.pathsep.join(runtime + [env.get(key, "")])
    RUNNER = HostRunner(args.budget_seconds, env)
    # The two variants share the results directory under different prefixes.
    OUTPUT_ROOT = new_run(build / "test-results" / "smooth-temperature", "builtin-" if args.builtin else "run-")
    PLUGIN_DATADIR = OUTPUT_ROOT / "datadir"
    PLUGIN_DATADIR.mkdir()
    if not args.builtin:
        staged = build / "test-data" / "smooth_temperature" / "plugins"
        plugin = staged / "nematx-smooth-temperature" / ("plugin.dll" if os.name == "nt" else "plugin.so")
        if not plugin.is_file():
            raise FileNotFoundError(f"Build and stage the standalone package first: {plugin}")
        shutil.copytree(staged, PLUGIN_DATADIR / "plugins")
    print(f"SmoothTemperature logs: {OUTPUT_ROOT}", flush=True)
    succeeded = False
    try:
        prefetch()
        program = unittest.main(argv=[sys.argv[0], *unittest_args], verbosity=2, exit=False)
        RUNNER.remaining()
        succeeded = program.result.wasSuccessful()
        return 0 if succeeded else 1
    finally:
        report = {"success": succeeded, "seconds": time.monotonic() - RUNNER.started, "budget_seconds": args.budget_seconds,
                  "exports": len(STAGES), "calls": RUNNER.calls, "stages": STAGES, "builtin": args.builtin}
        (OUTPUT_ROOT / "timings.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        finish_run(OUTPUT_ROOT, succeeded)
        print(f"SmoothTemperature total: {report['seconds']:.3f}s; host calls: {len(RUNNER.calls)}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
