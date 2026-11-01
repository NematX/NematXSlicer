"""End-to-end checks for the fan anticipation and kickstart (fan_speedup_time, fan_speedup_overhangs, fan_kickstart).

Each case exports a generated model through the host executable. The G-code is read
back with the time model of the plugin: a move with a displacement lasts its 3D
length over its feed rate, anything else (retractions, fan commands) lasts nothing.
The fan percentage (M106 S is 0-255) of every command is then compared with the
start of the extrusions it serves.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import shutil
import subprocess
import sys
import unittest
from dataclasses import dataclass
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

PROJECT = Path(__file__).resolve().parents[3]
HOST_EXECUTABLE: Path
BUILD_DIR: Path
PLUGIN_DATADIR: Path
HOST_DATADIR: Path
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
    "perimeters = 2",
    "fill_density = 20%",
    "top_solid_layers = 0",
    "bottom_solid_layers = 0",
    # Automatic requests only: 20% on the perimeters, 80% on the infill, with no
    # change from the cooling policies or the first layers.
    "min_fan_speed = 0",
    "default_fan_speed = 20",
    "bridge_fan_speed = 20",
    "perimeter_fan_speed = 20",
    "external_perimeter_fan_speed = 20",
    "infill_fan_speed = 80",
    "solid_infill_fan_speed = 80",
    "top_fan_speed = 80",
    "fan_below_layer_time = 0",
    "slowdown_below_layer_time = 0",
    "disable_fan_first_layers = 0",
    "full_fan_speed_layer = 0",
    # Disabled by default: each case enables what it checks.
    "fan_speedup_time = 0",
    "fan_speedup_overhangs = 0",
    "fan_kickstart = 0",
)
WORD = re.compile(r"([A-Z])([+-]?(?:\d+(?:\.\d*)?|\.\d+))")
DEFAULT_FEED = 1800.0  # mm/min, only used before the first F word
# The plugin places a change at a precision of 0.1 s: it never cuts a path in pieces shorter than that,
# and a change that would is moved to the nearest boundary. Times of the G-code are compared with it.
PLACEMENT_TOLERANCE = 0.1


@dataclass
class Move:
    layer: int
    role: str
    start_time: float
    end_time: float
    deposited_e: float
    start: tuple[float, float, float]
    end: tuple[float, float, float]
    feed: float  # mm/min


@dataclass
class FanCommand:
    layer: int
    time: float  # modeled time of the command
    percent: int
    line: int  # index in the file, to compare the order with the layer markers


@dataclass
class Gcode:
    text: str
    moves: list[Move]
    fans: list[FanCommand]
    layer_starts: list[float]  # modeled time of each layer marker
    layer_lines: list[int]  # index in the file of each layer marker
    tool_changes: list[tuple[int, int, float]]  # (index in the file, tool, modeled time) of each Tn command

    def extrusions(self, layer: int | None = None, role: str = "") -> list[Move]:
        return [move for move in self.moves if move.deposited_e > 0 and (layer is None or move.layer == layer)
                and role.lower() in move.role.lower()]

    def layer_fans(self, layer: int) -> list[FanCommand]:
        return [fan for fan in self.fans if fan.layer == layer]

    def layer_count(self) -> int:
        return 1 + max(move.layer for move in self.moves)


def box(x0: float, y0: float, z0: float, x1: float, y1: float, z1: float):
    vertices = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = ((3, 2, 1), (1, 0, 3), (4, 5, 6), (6, 7, 4), (0, 1, 5), (5, 4, 0),
             (1, 2, 6), (6, 5, 1), (2, 3, 7), (7, 6, 2), (3, 0, 4), (4, 7, 3))
    return vertices, faces


def write_3mf(path: Path, boxes: list[tuple]) -> None:
    """One object made of box volumes."""
    vertices, triangles, ranges = [], [], []
    for bounds in boxes:
        box_vertices, faces = box(*bounds)
        first = len(triangles)
        triangles += [tuple(len(vertices) + index for index in face) for face in faces]
        vertices += box_vertices
        ranges.append((first, len(triangles) - 1))
    model = ('<?xml version="1.0" encoding="UTF-8"?><model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"><resources>'
             '<object id="1" type="model"><mesh><vertices>'
             + "".join(f'<vertex x="{x:g}" y="{y:g}" z="{z:g}" />' for x, y, z in vertices)
             + "</vertices><triangles>"
             + "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}" />' for a, b, c in triangles)
             + '</triangles></mesh></object></resources><build><item objectid="1" /></build></model>')
    metadata = lambda kind, key, value: f'<metadata type="{kind}" key="{key}" value="{value}" />'
    config = '<?xml version="1.0" encoding="UTF-8"?><config><object id="1">'
    for index, (first, last) in enumerate(ranges):
        config += (f'<volume firstid="{first}" lastid="{last}">' + metadata("volume", "name", f"part-{index}")
                   + metadata("volume", "volume_type", "ModelPart") + "</volume>")
    config += "</object></config>"
    content_types = ('<?xml version="1.0" encoding="UTF-8"?>'
                     '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
                     '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml" />'
                     '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml" />'
                     '<Override PartName="/Metadata/Slic3r_PE_model.config" ContentType="application/xml" /></Types>')
    relationships = ('<?xml version="1.0" encoding="UTF-8"?>'
                     '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
                     '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
                     'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" /></Relationships>')
    with ZipFile(path, "w", ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml", content_types)
        archive.writestr("_rels/.rels", relationships)
        archive.writestr("3D/3dmodel.model", model)
        archive.writestr("Metadata/Slic3r_PE_model.config", config)


def datadir_without_speedup() -> Path:
    """A copy of the host datadir in which nematx_fan_speedup is explicitly deactivated.

    A host can embed this plugin and activate it by default, so the host datadir alone may apply the fan
    timing options; an explicit 0 in [activated] leaves the fan_temporal_plugin group without a provider."""
    datadir = OUTPUT_ROOT / "datadir-without-speedup"
    shutil.rmtree(datadir, ignore_errors=True)
    shutil.copytree(HOST_DATADIR, datadir)
    activation = datadir / "plugins" / "activated.ini"
    text = activation.read_text(encoding="utf-8")
    text, replaced = re.subn(r"^nematx_fan_speedup = \S+$", "nematx_fan_speedup = 0", text, flags=re.MULTILINE)
    if not replaced:
        text = text.replace("[activated]\n", "[activated]\nnematx_fan_speedup = 0\n", 1)
    if "nematx_fan_speedup = 0" not in text:
        raise RuntimeError(f"No [activated] section in {activation}")
    activation.write_text(text, encoding="utf-8")
    return datadir


def host_command(case_dir: Path, options: tuple[str, ...], model: Path, datadir: Path) -> tuple[int, str, Path]:
    """Exports `model` with the options; returns the exit code, the host log and the G-code path."""
    case_dir.mkdir(parents=True, exist_ok=True)
    profile = case_dir / "profile.ini"
    output = case_dir / "result.gcode"
    # A later option replaces an earlier one with the same key.
    merged = {option.split("=", 1)[0].strip(): option for option in BASE_PROFILE + options}
    profile.write_text("\n".join(merged.values()) + "\n", encoding="utf-8")
    output.unlink(missing_ok=True)
    environment = dict(os.environ)
    environment["PATH"] = os.pathsep.join(RUNTIME_PATH + [environment.get("PATH", "")])
    # The datadir must be a native absolute path: the Debug host checks it.
    result = subprocess.run([str(HOST_EXECUTABLE), "--datadir", str(datadir), "--export-gcode", "--load", str(profile),
                             "--output", str(output), str(model)],
                            capture_output=True, timeout=1800, env=environment)
    log = (result.stdout + result.stderr).decode("utf-8", errors="replace")
    (case_dir / "host.log").write_text(log, encoding="utf-8")
    return result.returncode, log, output


def run_export(name: str, options: tuple[str, ...], model: Path, datadir: Path | None = None) -> Gcode:
    case_dir = OUTPUT_ROOT / name
    code, log, output = host_command(case_dir, options, model, datadir or PLUGIN_DATADIR)
    if code != 0 or not output.is_file():
        raise AssertionError(f"{name}: export failed (exit {code}); see {case_dir / 'host.log'}")
    for message in ("will be disabled", "unknown option", "[error]", "fan timing:"):
        if message in log.lower():
            raise AssertionError(f"{name}: {message!r} in {case_dir / 'host.log'}")
    return read_gcode(output)


def read_gcode(path: Path) -> Gcode:
    """Moves and fan commands of the file, with their modeled time (see the module docstring)."""
    text = path.read_text(encoding="utf-8", errors="replace")
    position = [0.0, 0.0, 0.0]
    feed = DEFAULT_FEED
    absolute_e = True
    e = 0.0
    time = 0.0
    layer = -1
    role = ""
    moves: list[Move] = []
    fans: list[FanCommand] = []
    layer_starts: list[float] = []
    layer_lines: list[int] = []
    tool_changes: list[tuple[int, int, float]] = []
    for line_index, raw in enumerate(text.splitlines()):
        if raw.startswith(";LAYER_CHANGE"):
            layer += 1
            layer_starts.append(time)
            layer_lines.append(line_index)
        if raw.startswith(";TYPE:"):
            role = raw[6:].strip()
        code = raw.split(";", 1)[0].strip().upper()
        if not code:
            continue
        command = code.split(" ", 1)[0]
        fields = {key: float(value) for key, value in WORD.findall(code[len(command):])}
        if re.fullmatch(r"T\d+", command):
            tool_changes.append((line_index, int(command[1:]), time))
        elif command == "M106":
            fans.append(FanCommand(layer, time, round(fields.get("S", 255.0) * 100 / 255), line_index))
        elif command == "M107":
            fans.append(FanCommand(layer, time, 0, line_index))
        elif command == "M82":
            absolute_e = True
        elif command == "M83":
            absolute_e = False
        elif command == "G92" and "E" in fields:
            e = fields["E"]
        elif command in ("G0", "G1"):
            feed = fields.get("F", feed)
            target = [fields.get("X", position[0]), fields.get("Y", position[1]), fields.get("Z", position[2])]
            deposited = 0.0
            if "E" in fields:
                deposited = fields["E"] - e if absolute_e else fields["E"]
                e = fields["E"] if absolute_e else e + fields["E"]
            distance = math.dist(position, target)
            start = time
            if distance > 0:
                time += distance / (feed / 60.0)
                if layer >= 0:
                    moves.append(Move(layer, role, start, time, deposited, tuple(position), tuple(target), feed))
            position = target
        elif command in ("G2", "G3"):
            raise AssertionError("arc moves are not modeled: disable arc fitting")
    return Gcode(text, moves, fans, layer_starts, layer_lines, tool_changes)


def rises(result: Gcode) -> list[FanCommand]:
    """The commands that raise a fan speed already running. The first command of a print is not one:
    its entry state is unknown and nothing is moved or kicked before it."""
    out: list[FanCommand] = []
    previous = 0
    for fan in result.fans:
        if previous > 0 and fan.percent > previous:
            out.append(fan)
        previous = fan.percent
    return out


def kicks(result: Gcode) -> list[tuple[FanCommand, FanCommand]]:
    """Each 100% command and the command that follows it in the same layer."""
    out = []
    for index, fan in enumerate(result.fans):
        if fan.percent == 100 and index + 1 < len(result.fans) and result.fans[index + 1].layer == fan.layer:
            out.append((fan, result.fans[index + 1]))
    return out


def layer_end(result: Gcode, layer: int) -> float:
    """Modeled time at the end of the last move of a layer."""
    return max(move.end_time for move in result.moves if move.layer == layer)


@dataclass
class ToolGroup:
    index: int
    start_time: float  # modeled time of the layer marker or tool change that opens the group
    tool: int
    starts_with_tool_change: bool  # the tool was selected at the start of the group
    end_time: float  # modeled time of the next layer marker or tool change
    next_is_tool_change: bool
    fans: list[FanCommand]


def tool_groups(result: Gcode) -> list[ToolGroup]:
    """The stretches of the file printed by one tool: from a layer marker or a tool change to the next one."""
    events = sorted([(line, False, -1, result.layer_starts[number]) for number, line in enumerate(result.layer_lines)]
                    + [(line, True, tool, when) for line, tool, when in result.tool_changes])
    groups: list[ToolGroup] = []
    tool = -1
    for index, (line, selects_tool, selected, when) in enumerate(events):
        tool = selected if selects_tool else tool
        following = events[index + 1] if index + 1 < len(events) else None
        end_line = following[0] if following else len(result.text.splitlines())
        groups.append(ToolGroup(index, when, tool, selects_tool, following[3] if following else result.moves[-1].end_time,
                                bool(following and following[1]), [fan for fan in result.fans if line < fan.line < end_line]))
    return groups

def cut_moves(base: Gcode, new: Gcode) -> list[Move]:
    """Checks that `new` follows the path of `base` move by move and returns the moves of `base` that were cut.

    Every move of the base ends at the same point in the new file, possibly cut in several moves. The
    points added by the cuts lie on the base move, in the direction of travel, and the pieces keep its
    feed rate, its length and its extruded length (the G-code has three decimals, hence the tolerances).
    """
    tolerance = 2e-3
    index = 0
    cut: list[Move] = []
    for number, move in enumerate(base.moves):
        pieces: list[Move] = []
        while index < len(new.moves):
            pieces.append(new.moves[index])
            index += 1
            if pieces[-1].end == move.end:
                break
        if not pieces or pieces[-1].end != move.end:
            raise AssertionError(f"move {number} of the base ({move.start} -> {move.end}) has no end in the new file")
        length = math.dist(move.start, move.end)
        current = move.start
        along = 0.0
        for piece in pieces:
            if math.dist(piece.start, current) > tolerance:
                raise AssertionError(f"move {number}: the piece starting at {piece.start} does not follow {current}")
            if length > 0:
                position = sum((piece.end[k] - move.start[k]) * (move.end[k] - move.start[k]) for k in range(3)) / length**2
                foot = tuple(move.start[k] + position * (move.end[k] - move.start[k]) for k in range(3))
                if math.dist(piece.end, foot) > tolerance or position < along - 1e-6:
                    raise AssertionError(f"move {number}: the point {piece.end} is not on the base move, in order")
                along = position
            if piece.feed != move.feed or piece.layer != move.layer or piece.role != move.role:
                raise AssertionError(f"move {number}: a piece changes the feed rate, the layer or the role")
            current = piece.end
        if abs(sum(math.dist(piece.start, piece.end) for piece in pieces) - length) > tolerance * len(pieces):
            raise AssertionError(f"move {number}: the pieces do not add up to the length of the base move")
        if abs(sum(piece.deposited_e for piece in pieces) - move.deposited_e) > 1e-4 * len(pieces):
            raise AssertionError(f"move {number}: the pieces do not extrude the same length as the base move")
        if len(pieces) > 1:
            cut.append(move)
    if index != len(new.moves):
        raise AssertionError(f"{len(new.moves) - index} moves were added after the end of the base")
    return cut


# The helpers below (EXTRUDER_KEYS, two_extruder_options, write_two_parts) are copied from
# plugins/fan_first_layer_program/tests/test_fan_first_layer_program.py: the packages stay independent.
#
# Per-extruder printer settings: the command line does not resize them to the extruder count, so a
# second extruder repeats the host default of each one.
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
# The fan settings of the two extruders: every role of the first asks for 80% and every role of the second
# for 40%, and the first-layer program of the host raises them layer by layer from 0% over the first six
# layers. Each tool group of those layers is therefore one request, whatever the order of the roles: the group
# of the tool that goes on from the previous layer raises the fan, and so does the group of the first tool
# when the second one has run (the fan is shared).
TWO_EXTRUDER_FANS = (
    "default_fan_speed = 80,40", "bridge_fan_speed = 80,40", "perimeter_fan_speed = 80,40",
    "external_perimeter_fan_speed = 80,40", "infill_fan_speed = 80,40", "solid_infill_fan_speed = 80,40",
    "top_fan_speed = 80,40", "min_fan_speed = 0,0", "fan_below_layer_time = 0,0", "slowdown_below_layer_time = 0,0",
    "disable_fan_first_layers = 1,1", "full_fan_speed_layer = 6,6",
)


def two_extruder_options() -> tuple[str, ...]:
    """The settings of a printer with two extruders: the host defaults of the per-extruder settings, twice."""
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


class FanSpeedup(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        plugin = PLUGIN_DATADIR / "plugins" / "nematx-fan-speedup" / ("plugin.dll" if sys.platform == "win32" else "plugin.so")
        for required in (HOST_EXECUTABLE, plugin, HOST_DATADIR / "plugins" / "activated.ini"):
            if not required.is_file():
                raise FileNotFoundError(f"Missing build or staged package input: {required}")
        OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
        # Long enough paths for the kicks of a few seconds to end inside a layer.
        cls.cube = OUTPUT_ROOT / "cube.3mf"
        write_3mf(cls.cube, [(0, 0, 0, 30, 30, 1)])
        # A plate over a pillar that supports half of it: the other half has overhang perimeters.
        cls.table = OUTPUT_ROOT / "table.3mf"
        write_3mf(cls.table, [(0, 0, 0, 20, 10, 1), (0, 0, 1, 20, 20, 1.4)])
        cls.baseline = run_export("baseline", (), cls.cube)

    def test_rise_is_anticipated_by_the_delay(self) -> None:
        result = run_export("anticipation", ("fan_speedup_time = 1",), self.cube)
        plain, moved = rises(self.baseline), rises(result)
        self.assertEqual(len(plain), len(moved))
        long_enough = 0
        for before, after in zip(plain, moved):
            # The command moves one second earlier, or to the start of its layer if that comes first.
            room = before.time - self.baseline.layer_starts[before.layer]
            self.assertEqual(before.layer, after.layer)
            self.assertAlmostEqual(before.time - after.time, min(1.0, room), delta=PLACEMENT_TOLERANCE,
                                   msg=f"layer {before.layer}")
            long_enough += room > 1.05
        self.assertGreaterEqual(long_enough, 2, "no rise with room for the whole delay")

    def test_anticipation_never_crosses_the_start_of_a_layer(self) -> None:
        # A delay longer than the print: every rise goes back as far as its layer allows, no further.
        result = run_export("anticipation-long", ("fan_speedup_time = 100",), self.cube)
        rise_layers = {fan.layer for fan in rises(self.baseline)}
        self.assertGreaterEqual(len(rise_layers), 2)
        for layer in range(result.layer_count()):
            fans = [(fan.percent, fan.time) for fan in result.layer_fans(layer)]
            first_move = min(move.start_time for move in result.moves if move.layer == layer)
            if layer in rise_layers:
                # The rise starts with the layer, together with the first path; the lower requests that were
                # before it are erased. Nothing is commanded if the fan already runs at that speed.
                earlier = [fan.percent for fan in result.fans if fan.layer < layer]
                running = earlier[-1] if earlier else 0
                self.assertEqual(fans, [] if running == 80 else [(80, first_move)], f"layer {layer}, fan running at {running}%")
            else:
                self.assertEqual(fans, [(fan.percent, fan.time) for fan in self.baseline.layer_fans(layer)],
                                 f"layer {layer}")

    def test_kick_is_proportional_to_the_rise(self) -> None:
        for infill, name in ((80, "full"), (50, "half")):
            options = (f"infill_fan_speed = {infill}", f"solid_infill_fan_speed = {infill}", f"top_fan_speed = {infill}",
                       "fan_kickstart = 2")
            plain = rises(run_export(f"plain-{name}", options[:3], self.cube))
            result = run_export(f"kick-{name}", options, self.cube)
            pairs = kicks(result)
            self.assertEqual(len(pairs), len(plain))
            for (kick, target), before in zip(pairs, plain):
                self.assertEqual(target.percent, infill)
                # 2 s for a rise of 100 points, in proportion to the rise from 20%, from the start of the rise.
                self.assertAlmostEqual(target.time - kick.time, 2.0 * (infill - 20) / 100, delta=PLACEMENT_TOLERANCE)
                self.assertAlmostEqual(kick.time, before.time, delta=PLACEMENT_TOLERANCE)
            # The kick cuts the paths without changing them.
            self.assertTrue(cut_moves(self.baseline, result), f"nothing was cut ({name})")

    def test_kick_follows_the_anticipation(self) -> None:
        result = run_export("kick-anticipated", ("fan_speedup_time = 1", "fan_kickstart = 2"), self.cube)
        pairs = kicks(result)
        plain = rises(self.baseline)
        self.assertEqual(len(pairs), len(plain))
        for (kick, target), before in zip(pairs, plain):
            room = before.time - self.baseline.layer_starts[before.layer]
            # The kick starts where the rise was moved to and lasts 2 s * 60 / 100.
            self.assertAlmostEqual(before.time - kick.time, min(1.0, room), delta=PLACEMENT_TOLERANCE)
            self.assertAlmostEqual(target.time - kick.time, 1.2, delta=PLACEMENT_TOLERANCE)

    def test_rises_of_ten_points_or_less_are_not_kicked_without_anticipation(self) -> None:
        for infill, kicked in ((30, False), (31, True)):
            options = (f"infill_fan_speed = {infill}", f"solid_infill_fan_speed = {infill}", f"top_fan_speed = {infill}",
                       "fan_kickstart = 2")
            result = run_export(f"threshold-{infill}", options, self.cube)
            self.assertEqual(bool(kicks(result)), kicked, f"rise to {infill}%")
        # With the anticipation, any rise is kicked. The 0.1 s of a rise of 5 points is as short as the
        # tolerance: the kick lasts between nothing and two tolerances.
        options = ("infill_fan_speed = 25", "solid_infill_fan_speed = 25", "top_fan_speed = 25", "fan_kickstart = 2",
                   "fan_speedup_time = 1")
        pairs = kicks(run_export("threshold-anticipated", options, self.cube))
        self.assertTrue(pairs)
        for kick, target in pairs:
            self.assertAlmostEqual(target.time - kick.time, 0.1, delta=PLACEMENT_TOLERANCE)

    def test_a_kick_is_restored_at_the_end_of_the_layer(self) -> None:
        # 60 s for the rise from 20% to 80%: the kick is still running when the layer ends.
        result = run_export("kick-restored", ("fan_kickstart = 100",), self.cube)
        pairs = kicks(result)
        self.assertEqual(len(pairs), len(rises(self.baseline)))
        for kick, target in pairs:
            self.assertEqual(target.percent, 80)
            self.assertEqual(result.layer_fans(kick.layer)[-1], target)
            self.assertAlmostEqual(target.time, layer_end(result, kick.layer), delta=0.01)

    def test_kicks_and_entry_state_across_tools(self) -> None:
        model = OUTPUT_ROOT / "two-parts.3mf"
        write_two_parts(model)
        options = two_extruder_options() + TWO_EXTRUDER_FANS
        baseline = run_export("tools-baseline", options, model)
        plain = tool_groups(baseline)
        printed = baseline.extrusions()
        kicked = tool_groups(run_export("tools-kick", options + ("fan_speedup_time = 1", "fan_kickstart = 2"), model))
        long_kick = tool_groups(run_export("tools-long-kick", options + ("fan_kickstart = 100",), model))
        self.assertGreaterEqual(sum(group.starts_with_tool_change for group in plain), 4)
        for groups in (kicked, long_kick):
            self.assertEqual([(g.tool, g.starts_with_tool_change) for g in groups], [(g.tool, g.starts_with_tool_change) for g in plain])

        # The percentage last commanded before each group (the fan is shared by the tools and starts off).
        running = 0
        before: list[int] = []
        for group in plain:
            before.append(running)
            for fan in group.fans:
                running = fan.percent
        # The first group of the plan is the first one that prints: the entry state of the print is unknown there.
        first_printing_group = next(group.index for group in plain if any(group.start_time <= move.start_time < group.end_time for move in printed))

        # Every group that follows a tool change, and the first group of the print, start from an unknown state:
        # their requests are neither anticipated nor kicked. A tool that goes on from one layer to the next
        # knows its state: its rise is kicked, from the start of the group.
        unknown_rises = kicked_groups = 0
        for group, base, state in zip(kicked, plain, before):
            if group.starts_with_tool_change or group.index == first_printing_group:
                unknown_rises += bool(base.fans) and base.fans[0].percent > state
                self.assertEqual([(f.percent, round(f.time, 2)) for f in group.fans], [(f.percent, round(f.time, 2)) for f in base.fans],
                                 f"group {group.index}")
            elif base.fans and base.fans[0].percent > state:
                kicked_groups += 1
                self.assertEqual([fan.percent for fan in group.fans][:2], [100, base.fans[0].percent], f"group {group.index}")
        self.assertGreater(unknown_rises, 0, "no rise after a tool change")
        self.assertGreater(kicked_groups, 0, "no tool goes on from one layer to the next")

        # A kick still running at the end of its tool group is restored in the window that follows the last
        # path, before the next tool is selected, with the target of the group.
        restored_before_a_tool = 0
        for group, base in zip(long_kick, plain):
            percents = [fan.percent for fan in group.fans]
            if group.starts_with_tool_change:
                self.assertNotIn(100, percents, f"group {group.index} follows a tool change")
            if 100 not in percents:
                continue
            self.assertNotEqual(group.fans[-1].percent, 100, f"group {group.index}")
            self.assertEqual(group.fans[-1].percent, base.fans[-1].percent, f"group {group.index}")
            self.assertAlmostEqual(group.fans[-1].time, group.end_time, delta=1e-6, msg=f"group {group.index}")
            restored_before_a_tool += group.next_is_tool_change
        self.assertGreater(restored_before_a_tool, 0, "no kick runs into a tool change")

        # The window between the last path of a group and the next tool change holds nothing but the restoration of
        # a kick that is still running, with the value of the target; no kick and its return are added there.
        def window(group: ToolGroup) -> list[int]:
            return [fan.percent for fan in group.fans if fan.time >= group.end_time - 1e-6]

        for groups, name in ((kicked, "short kick"), (long_kick, "long kick")):
            for group, base in zip(groups, plain):
                if group.next_is_tool_change:
                    # Either the commands of the plain export, or the single restoration of a kick still running.
                    allowed = [window(base)] + ([[base.fans[-1].percent]] if base.fans else [])
                    self.assertIn(window(group), allowed, f"{name}, group {group.index}")

    def test_overhang_filter(self) -> None:
        # One overhang stretch on the far side of the plate, then the infill: 20% -> 90% -> 20% -> 80%.
        options = ("perimeters = 1", "overhangs_fan_speed = 90")
        timed = options + ("fan_speedup_time = 0.5",)
        none = run_export("overhangs-baseline", options, self.table)
        every = run_export("overhangs-off", timed + ("fan_speedup_overhangs = 0",), self.table)
        only = run_export("overhangs-on", timed + ("fan_speedup_overhangs = 1",), self.table)
        layer = next(index for index in range(none.layer_count()) if none.extrusions(index, "overhang"))

        def time_of(result: Gcode, percent: int) -> float:
            return next(fan.time for fan in result.layer_fans(layer) if fan.percent == percent)

        room = time_of(none, 90) - none.layer_starts[layer]
        # The rise to the overhang speed moves earlier, as far as the start of the layer allows.
        self.assertAlmostEqual(time_of(none, 90) - time_of(only, 90), min(0.5, room), delta=PLACEMENT_TOLERANCE)
        self.assertAlmostEqual(time_of(none, 90) - time_of(every, 90), min(0.5, room), delta=PLACEMENT_TOLERANCE)
        # The rise to the infill speed only moves when every role is anticipated; the period
        # of the overhang speed that precedes it limits the move.
        self.assertAlmostEqual(time_of(none, 80), time_of(only, 80), delta=0.01)
        self.assertGreater(time_of(none, 80) - time_of(every, 80), 0.05)
        # With one perimeter, the paths of the role External perimeter are the loop of shell 0, which carries the
        # seam marker. Some of them are cut, and the cuts do not repeat the marker.
        self.assertIn("External perimeter", {move.role for move in cut_moves(none, every)})
        seams = [line for line in none.text.splitlines() if line.startswith(";SEAM:")]
        self.assertTrue(seams)
        for result in (every, only):
            self.assertEqual([line for line in result.text.splitlines() if line.startswith(";SEAM:")], seams)

    def test_extruded_volume_and_geometry_are_kept(self) -> None:
        result = run_export("conservation", ("fan_speedup_time = 1", "fan_kickstart = 2"), self.cube)
        self.assertAlmostEqual(sum(move.deposited_e for move in result.moves),
                               sum(move.deposited_e for move in self.baseline.moves), delta=1e-3)
        # Every point added by a cut is on the move it cuts, in order, with the same feed rate, length and E.
        self.assertTrue(cut_moves(self.baseline, result), "nothing was cut")
        self.assertAlmostEqual(result.moves[-1].end_time, self.baseline.moves[-1].end_time, delta=1e-3,
                               msg="the modeled time changed")

    def test_disabled_options_change_nothing(self) -> None:
        reference = run_export("host-identity", (), self.cube, HOST_DATADIR)
        self.assertEqual(self.baseline.text, reference.text)

    def test_nonzero_options_are_refused_without_the_package(self) -> None:
        datadir = datadir_without_speedup()
        for option in ("fan_speedup_time = 1", "fan_kickstart = 2"):
            code, log, _ = host_command(OUTPUT_ROOT / f"refused-{option.split()[0]}", (option,), self.cube, datadir)
            self.assertNotEqual(code, 0, option)
            self.assertIn(option.split()[0], log)
            self.assertIn("fan_temporal_plugin", log)

    def test_exports_are_deterministic(self) -> None:
        options = ("fan_speedup_time = 1", "fan_kickstart = 2")
        first = run_export("determinism-1", options, self.cube)
        second = run_export("determinism-2", options, self.cube)
        self.assertEqual(first.text, second.text)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-executable", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path, default=PROJECT / ".build")
    parser.add_argument("--runtime-path", action="append", default=[],
                        help="directory added to PATH for the host's DLL dependencies")
    args, unittest_args = parser.parse_known_args()
    BUILD_DIR = args.build_dir.resolve()
    HOST_EXECUTABLE = args.host_executable.resolve()
    PLUGIN_DATADIR = BUILD_DIR / "test-data" / "fan_speedup"
    HOST_DATADIR = BUILD_DIR / "test-data" / "host"
    OUTPUT_ROOT = BUILD_DIR / "test-results" / "fan-speedup"
    RUNTIME_PATH = [str(Path(path).resolve()) for path in args.runtime_path]
    unittest.main(argv=[sys.argv[0], *unittest_args], verbosity=2)