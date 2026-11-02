"""End-to-end check of the whole fan chain in one export.

The three fan packages (nematx-regional-fan, nematx-fan-first-layer-program, nematx-fan-speedup)
and the fan policies of the host (standard, short-layer cooling, first layers, dynamic overhangs) work on
one generated model, and the fan commands of the G-code are read back with the time model of the
speedup plugin. The export is compared with the same export without the speedup package, which only
cuts paths, moves the instants of the fan commands and inserts the 100% pulses of the kicks (an
Override keeps its value except during those pulses). Instants are compared with the 0.1 s placement
tolerance of the plugin.
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
from functools import cached_property
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

PROJECT = Path(__file__).resolve().parents[2]
HOST_EXECUTABLE: Path
BUILD_DIR: Path
OUTPUT_ROOT: Path
RUNTIME_PATH: list[str] = []

# Packages of the chain, with the plugin ids they register (the datadirs are prepared by the build).
REGIONAL = ("regional_fan", "nematx-regional-fan", "nematx_regional_fan")
PROGRAM = ("fan_first_layer_program", "nematx-fan-first-layer-program", "nematx_fan_first_layer_program")
SPEEDUP = ("fan_speedup", "nematx-fan-speedup", "nematx_fan_speedup")

OVERRIDE = 85  # print_fan_speed of the Override region
DELAY = 1.0  # fan_speedup_time, s
KICKSTART = 1.6  # fan_kickstart, s for a rise of 100 points
# The Override rise of the layers 2 and 3 (fan off before) is a rise of OVERRIDE points, kicked for
# KICKSTART * OVERRIDE / 100 = 1.36 s, from the start of the layer at the earliest. The Override region of those
# layers lasts about 1.68 s from the layer start: with 2.0 s (a kick of 1.7 s) the region ended during the kick and
# the fall cut it before its target, as the speedup is meant to do; 1.6 s leaves 0.3 s of margin.
MAXIMUM = 80  # max_fan_speed: the fan of the short layers

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
    # Automatic requests: 30% on the perimeters, 60% on the infill, a dynamic ramp on the overhangs.
    "default_fan_speed = 30",
    "perimeter_fan_speed = 30",
    "external_perimeter_fan_speed = 30",
    "infill_fan_speed = 60",
    "solid_infill_fan_speed = 60",
    "top_fan_speed = 60",
    "bridge_fan_speed = 30",
    "overhangs_dynamic_fan_speed = 0:5:1:0x100:25x80:50x60:75x40:100x20",
    "min_fan_speed = 0",
    f"max_fan_speed = {MAXIMUM}",
    "fan_speedup_time = 0",
    "fan_speedup_overhangs = 0",
    "fan_kickstart = 0",
)
WORD = re.compile(r"([A-Z])([+-]?(?:\d+(?:\.\d*)?|\.\d+))")
DEFAULT_FEED = 1800.0  # mm/min, only used before the first F word
PLACEMENT_TOLERANCE = 0.1  # s, the placement precision of the speedup plugin
RAMP_STEP = 0.05  # s, the longest gap between the steps of one ramp of the dynamic overhang fan


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
    fan: int  # fan percentage in force when the move starts


@dataclass
class FanCommand:
    layer: int
    time: float  # modeled time of the command
    percent: int


@dataclass
class Gcode:
    text: str
    moves: list[Move]
    fans: list[FanCommand]
    layer_starts: list[float]

    def extrusions(self, layer: int | None = None, region: str = "") -> list[Move]:
        """The deposits of a layer, optionally of one of the two tables of the model (see write_model)."""
        return [move for move in self.moves if move.deposited_e > 0 and (layer is None or move.layer == layer)
                and (not region or self.region(move) == region)]

    @cached_property
    def boundary(self) -> float:
        # The tables span 60 mm of x, the Override one starting 40 mm after the other: the host centres the
        # object on the bed, so the boundary is found from the extent of the print.
        return min(min(m.start[0], m.end[0]) for m in self.moves if m.deposited_e > 0) + 35

    def region(self, move: Move) -> str:
        return "override" if (move.start[0] + move.end[0]) / 2 > self.boundary else "automatic"

    def layer_count(self) -> int:
        return len(self.layer_starts)

    def layer_end(self, layer: int) -> float:
        return max(move.end_time for move in self.moves if move.layer == layer)


def box(x0: float, y0: float, z0: float, x1: float, y1: float, z1: float):
    vertices = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = ((3, 2, 1), (1, 0, 3), (4, 5, 6), (6, 7, 4), (0, 1, 5), (5, 4, 0),
             (1, 2, 6), (6, 5, 1), (2, 3, 7), (7, 6, 2), (3, 0, 4), (4, 7, 3))
    return vertices, faces


def write_3mf(path: Path, volumes: list[tuple[tuple, dict[str, str]]]) -> None:
    """One object made of box volumes, each with its own settings.

    Adapted from plugins/regional_fan/tests/test_regional_fan.py: the packages stay independent."""
    vertices, triangles, ranges = [], [], []
    for bounds, _ in volumes:
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
    for index, ((first, last), (_, settings)) in enumerate(zip(ranges, volumes)):
        config += (f'<volume firstid="{first}" lastid="{last}">' + metadata("volume", "name", f"part-{index}")
                   + metadata("volume", "volume_type", "ModelPart")
                   + "".join(metadata("volume", key, value) for key, value in settings.items()) + "</volume>")
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


def write_model(path: Path) -> None:
    """Two tables of one object, 10 mm apart: the left one (x < 30) follows the automatic fan settings,
    the right one (x >= 40) has the Override print_fan_speed.

    Each table has a base, a pillar of 10 x 10 mm (the short layers) and a plate 20 x 20 mm over the
    pillar, 0.6 mm thick, that overhangs on two sides (perimeters in overhang, bridge infill).
    The automatic table starts with a slab of 30 x 30 mm, 1.2 mm thick (the first layers are long)."""
    override = {"print_fan_speed": f"{OVERRIDE}%"}
    write_3mf(path, [
        ((0, 0, 0, 30, 30, 1.2), {}),
        ((0, 0, 1.2, 10, 10, 2.4), {}),
        ((0, 0, 2.4, 20, 20, 3.0), {}),
        ((40, 0, 0, 50, 10, 2.4), override),
        ((40, 0, 2.4, 60, 20, 3.0), override),
    ])


def prepare_datadir(name: str, packages: tuple[tuple[str, str, str], ...]) -> Path:
    """One datadir with the staged packages, built from the datadirs prepared by the build.

    The activation file is completed like the around-holes check of plugins/regional_fan/tests does."""
    first, *others = packages
    datadir = OUTPUT_ROOT / name
    shutil.rmtree(datadir, ignore_errors=True)
    shutil.copytree(BUILD_DIR / "test-data" / first[0], datadir)
    activation = datadir / "plugins" / "activated.ini"
    for directory, package, plugin in others:
        staged = BUILD_DIR / "test-data" / directory / "plugins"
        shutil.copytree(staged / package, datadir / "plugins" / package)
        installed = dict(re.findall(rf"^{re.escape(package)}(\S*) = (\S+)$",
                                    (staged / "activated.ini").read_text(encoding="utf-8"), re.MULTILINE))
        subprocess.run([sys.executable, str(PROJECT / "cmake" / "prepare_activated_ini.py"), "--defaults", str(activation),
                        "--output", str(activation), "--plugin-id", plugin, "--package-id", package,
                        "--plugin-version", installed[""], "--slicer-version", installed[".slicer_version"]], check=True)
    return datadir


def run_export(name: str, options: tuple[str, ...], model: Path, datadir: Path) -> Gcode:
    case_dir = OUTPUT_ROOT / name
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
    if result.returncode != 0 or not output.is_file():
        raise AssertionError(f"{name}: export failed (exit {result.returncode}); see {case_dir / 'host.log'}")
    for message in ("will be disabled", "unknown option", "[error]", "fan timing:"):
        if message in log.lower():
            raise AssertionError(f"{name}: {message!r} in {case_dir / 'host.log'}")
    return read_gcode(output)


def read_gcode(path: Path) -> Gcode:
    """Moves and fan commands of the file, with their modeled time.

    Adapted from plugins/fan_speedup/tests/test_fan_speedup.py: a move with a displacement lasts its 3D
    length over its feed rate, anything else (retractions, fan commands) lasts nothing."""
    text = path.read_text(encoding="utf-8", errors="replace")
    position = [0.0, 0.0, 0.0]
    feed = DEFAULT_FEED
    absolute_e = True
    e = 0.0
    time = 0.0
    fan = 0
    layer = -1
    role = ""
    moves: list[Move] = []
    fans: list[FanCommand] = []
    layer_starts: list[float] = []
    for raw in text.splitlines():
        if raw.startswith(";LAYER_CHANGE"):
            layer += 1
            layer_starts.append(time)
        if raw.startswith(";TYPE:"):
            role = raw[6:].strip()
        code = raw.split(";", 1)[0].strip().upper()
        if not code:
            continue
        command = code.split(" ", 1)[0]
        fields = {key: float(value) for key, value in WORD.findall(code[len(command):])}
        if command == "M106":
            fan = round(fields.get("S", 255.0) * 100 / 255)
            fans.append(FanCommand(layer, time, fan))
        elif command == "M107":
            fan = 0
            fans.append(FanCommand(layer, time, 0))
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
                    moves.append(Move(layer, role, start, time, deposited, tuple(position), tuple(target), feed, fan))
            position = target
        elif command in ("G2", "G3"):
            raise AssertionError("arc moves are not modeled: disable arc fitting")
    return Gcode(text, moves, fans, layer_starts)


def cut_moves(base: Gcode, new: Gcode) -> list[tuple[Move, list[Move]]]:
    """Checks that `new` follows the path of `base` move by move and returns each move of `base` with its pieces.

    Adapted from plugins/fan_speedup/tests/test_fan_speedup.py. Every move of the base ends at the same
    point in the new file, possibly cut in several moves. The points added by the cuts lie on the base
    move, in the direction of travel, and the pieces keep its feed rate, its length and its extruded length
    (the G-code has three decimals, hence the tolerances)."""
    tolerance = 2e-3
    index = 0
    out: list[tuple[Move, list[Move]]] = []
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
        out.append((move, pieces))
    if index != len(new.moves):
        raise AssertionError(f"{len(new.moves) - index} moves were added after the end of the base")
    return out


def effective(result: Gcode) -> list[FanCommand]:
    """The fan changes of a file: of several commands at the same instant (a kick return followed by the
    next request) only the last one is in force during a path."""
    out: list[FanCommand] = []
    for fan in result.fans:
        if out and abs(out[-1].time - fan.time) < 1e-6:
            out[-1] = fan
        else:
            out.append(fan)
    return out


@dataclass
class Rise:
    layer: int
    time: float  # first instant of the rise
    last: float  # last instant of the rise (the steps of a ramp)
    percent: int  # final value


def rises(result: Gcode) -> list[Rise]:
    """The rises of a fan speed already running, each at its first instant and with its final value: the steps
    of a ramp (the dynamic overhang fan changes by steps a few milliseconds apart) are one rise. The first
    value of a print is not one: its entry state is unknown, it is neither compared nor moved by the speedup."""
    changes = effective(result)
    out: list[Rise] = []
    last_step = None  # the instant of the previous step, when the previous change was a rise
    for previous, fan in zip(changes, changes[1:]):
        if fan.percent <= previous.percent:
            last_step = None
        elif last_step is not None and fan.time - last_step <= RAMP_STEP:
            out[-1] = Rise(out[-1].layer, out[-1].time, fan.time, fan.percent)
            last_step = fan.time
        else:
            out.append(Rise(fan.layer, fan.time, fan.time, fan.percent))
            last_step = fan.time
    return out


@dataclass
class Full:
    start: FanCommand  # the 100% command
    end: FanCommand  # the command that follows it in its layer: the target of a kick, or the end of the period
    before: int  # percentage running before the 100%


def full_periods(result: Gcode) -> list[Full]:
    """Each 100% (S255) command followed, in its layer, by a lower value."""
    return [Full(fan, result.fans[index + 1], result.fans[index - 1].percent)
            for index, fan in enumerate(result.fans)
            if fan.percent == 100 and index > 0 and index + 1 < len(result.fans)
            and result.fans[index + 1].layer == fan.layer and result.fans[index + 1].percent < 100]


def split_full_periods(chain: Gcode, reference: Gcode) -> tuple[list[Full], list[Full]]:
    """The 100% periods of the chain that the fan asks for itself (the dynamic overhang fan, here), and the kicks.

    A natural period is a reference period, possibly moved earlier by up to the delay, or the union of
    several of them that the anticipation joined (the lower requests between them are erased, so their gap
    is at most the delay): it starts at most the delay before the first one and ends where the last one ends.
    Every other 100% is a kick."""
    natural_periods = full_periods(reference)
    natural: list[Full] = []
    kicks: list[Full] = []
    tolerance = PLACEMENT_TOLERANCE
    for period in full_periods(chain):
        candidates = [p for p in natural_periods if p.start.layer == period.start.layer]
        joined = False
        for first, head in enumerate(candidates):
            if not head.start.time - DELAY - tolerance <= period.start.time <= head.start.time + tolerance:
                continue
            for last in range(first, len(candidates)):
                if last > first and candidates[last].start.time - candidates[last - 1].end.time > DELAY + tolerance:
                    break
                if abs(candidates[last].end.time - period.end.time) <= tolerance:
                    joined = True
        (natural if joined else kicks).append(period)
    # Every natural period of the reference is covered by one of the chain.
    for period in natural_periods:
        if not any(own.start.time <= period.start.time + tolerance and own.end.time >= period.end.time - tolerance
                   for own in natural):
            raise AssertionError(f"the 100% of the reference at {period.start.time:.2f} s is lost")
    return natural, kicks


class FanChain(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        required = [HOST_EXECUTABLE]
        for directory, package, _ in (REGIONAL, PROGRAM, SPEEDUP):
            required.append(BUILD_DIR / "test-data" / directory / "plugins" / package /
                            ("plugin.dll" if sys.platform == "win32" else "plugin.so"))
        for path in required:
            if not path.is_file():
                raise FileNotFoundError(f"Missing build or staged package input: {path}")
        OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
        cls.model = OUTPUT_ROOT / "tables.3mf"
        write_model(cls.model)
        cls.without = prepare_datadir("datadir-without-speedup", (REGIONAL, PROGRAM))
        cls.datadir = prepare_datadir("datadir-chain", (REGIONAL, PROGRAM, SPEEDUP))
        # Fan reference of the layer-time cooling. The host first slows the short layers down to the slowdown
        # time (6 s), then the fan policy reads that slowed duration: a layer slowed to exactly 6 s gets the
        # interpolation factor (8 - 6) / (8 - 6) = 1 between slowdown and fan times, that is max_fan_speed,
        # while a layer of 8 s or more (the fan time) keeps its automatic value. Equal times would leave a
        # slowed layer at the limit, uncooled.
        cls.options = ("enable_fan_first_layers = 1", "disable_fan_first_layers = 3", "fan_speed_layer_gradient = 1",
                       "fan_below_layer_time = 8", "slowdown_below_layer_time = 6")
        cls.timed = cls.options + (f"fan_speedup_time = {DELAY}", f"fan_kickstart = {KICKSTART}")
        cls.reference = run_export("reference", cls.options, cls.model, cls.without)
        cls.chain = run_export("chain", cls.timed, cls.model, cls.datadir)
        cls.natural, cls.kicks = split_full_periods(cls.chain, cls.reference)

    # ----- The export without nematx-fan-speedup: what the policies and the two other packages ask for.

    def test_reference_layers_are_what_the_model_needs(self) -> None:
        # The 10 x 10 mm pillars (layers 6..11) last about 3 s unslowed; the host slows them down to the
        # slowdown time (6 s), measured at 6.0 s in the G-code: the bounds 5.5 and 7 s leave room for the
        # time model's rounding. The others (the slab and the plates, 8.7 s and more as measured) stay above the
        # fan time (8 s) with a margin, so that they are neither slowed nor cooled.
        starts = self.reference.layer_starts
        last = self.reference.layer_count() - 1
        for layer in range(1, last + 1):  # layer 0 includes the start G-code
            duration = (starts[layer + 1] if layer < last else self.reference.layer_end(layer)) - starts[layer]
            if 6 <= layer <= 11:
                self.assertLess(duration, 7.0, f"layer {layer} is not slowed to the slowdown time")
                self.assertGreater(duration, 5.5, f"layer {layer} is not slowed to the slowdown time")
            else:
                self.assertGreater(duration, 8.5, f"layer {layer} is not long")
        self.assertEqual(self.reference.layer_count(), 15)

    def test_reference_override_region_keeps_its_value(self) -> None:
        # The first layers (off on layers 2 and 3), the short layers and the dynamic overhang fan never
        # change an Override.
        for layer in range(self.reference.layer_count()):
            values = {move.fan for move in self.reference.extrusions(layer, "override")}
            self.assertEqual(values, {OVERRIDE}, f"layer {layer}")
        self.assertTrue(any("overhang" in move.role.lower() for move in self.reference.extrusions(12, "override")))

    def test_reference_first_layer_program_applies_outside_the_override(self) -> None:
        # Normal for one layer, ramped by 1/2 on one layer, off for three layers, ramped back: the host
        # program of the legacy settings E = 1, D = 3, G = 1 gives 1, 1/2, 0, 0, 1/2, 1 times the
        # automatic value: 30% on the perimeters and 60% on the infill (the layers are not short).
        for layer, factor in enumerate((1, 0.5, 0, 0, 0.5, 1)):
            moves = self.reference.extrusions(layer, "automatic")
            self.assertTrue(moves, f"layer {layer}")
            for move in moves:
                expected = round((30 if "perimeter" in move.role.lower() else 60) * factor)
                self.assertEqual(move.fan, expected, f"layer {layer}, {move.role}")

    def test_reference_short_layers_are_cooled_outside_the_override(self) -> None:
        # The short layers 6..11 go to max_fan_speed on every role that can increase; the long ones
        # after the first layers keep the automatic values (bridges and overhangs aside).
        for layer in range(6, 12):
            self.assertEqual({move.fan for move in self.reference.extrusions(layer, "automatic")}, {MAXIMUM}, f"layer {layer}")
        for layer in (13, 14):
            for move in self.reference.extrusions(layer, "automatic"):
                self.assertEqual(move.fan, 30 if "perimeter" in move.role.lower() else 60, f"layer {layer}, {move.role}")

    def test_reference_dynamic_overhang_fan_applies_outside_the_override(self) -> None:
        overhangs = [move.fan for move in self.reference.extrusions(12, "automatic") if "overhang" in move.role.lower()]
        self.assertTrue(overhangs)
        # The graph goes from 20% (no overhang) to 100% (full overhang) and never lowers the 30% perimeters.
        self.assertGreaterEqual(max(overhangs), 95)
        self.assertGreaterEqual(min(overhangs), 30)

    # ----- The chain with the anticipation and the kickstart.

    def test_override_region_is_not_changed_by_the_speedup(self) -> None:
        # An Override keeps its value: only the 100% of a kick, written in its mode, differs, and a path at
        # 100% lies entirely inside the window of a real kick (from its start to the return of its target).
        # The 100% periods that the dynamic overhang fan asks for itself (self.natural) are no window.
        tolerance = PLACEMENT_TOLERANCE
        kicked = 0
        for move in self.chain.extrusions(region="override"):
            if move.fan == OVERRIDE:
                continue
            self.assertEqual(move.fan, 100, f"layer {move.layer}, {move.role}")
            self.assertTrue(any(kick.start.time - tolerance <= move.start_time and move.end_time <= kick.end.time + tolerance
                                for kick in self.kicks),
                            f"layer {move.layer}: a 100% path of the Override region outside a kick")
            kicked += 1
        self.assertGreater(kicked, 0, "no kick reaches the Override region")

    def test_override_region_may_start_up_to_the_delay_earlier(self) -> None:
        plain = {}
        for fan in rises(self.reference):
            if fan.percent == OVERRIDE:
                plain.setdefault(fan.layer, fan)
        moved = {}
        entry = effective(self.chain)[0].time  # the first value of the print is not a rise
        for index, fan in enumerate(self.chain.fans):
            if fan.time > entry and fan.percent == OVERRIDE and fan.layer not in moved:
                # The rise starts with its kick, when there is one.
                kick = self.chain.fans[index - 1]
                moved[fan.layer] = kick if kick.percent == 100 and kick.layer == fan.layer else fan
        self.assertEqual(set(plain), set(moved))
        earliest = 0.0
        for layer, fan in plain.items():
            advance = fan.time - moved[layer].time
            self.assertGreaterEqual(advance, -PLACEMENT_TOLERANCE, f"layer {layer}: the Override starts later")
            self.assertLessEqual(advance, DELAY + PLACEMENT_TOLERANCE, f"layer {layer}: more than the delay")
            self.assertGreaterEqual(moved[layer].time, self.chain.layer_starts[layer] - PLACEMENT_TOLERANCE)
            earliest = max(earliest, advance)
        self.assertGreater(earliest, DELAY - PLACEMENT_TOLERANCE, "no Override rise moved by the whole delay")

    def test_policies_still_apply_outside_the_override(self) -> None:
        # Path by path, the automatic region has the fan of the reference (first-layer program, short-layer
        # cooling, dynamic overhangs), except where the speedup acts: a path takes the target of the first
        # rise of the reference that follows its base move, if that rise was moved up to the delay earlier
        # over the path (the lower requests between them are erased), or the 100% of a real kick.
        pieces = cut_moves(self.reference, self.chain)
        reference_rises = rises(self.reference)
        tolerance = PLACEMENT_TOLERANCE
        automatic = [(move, parts) for move, parts in pieces if move.deposited_e > 0
                     and self.reference.region(move) == "automatic"]
        self.assertTrue(automatic)
        changed = 0
        for move, parts in automatic:
            for piece in parts:
                if piece.fan == move.fan:
                    continue
                changed += 1
                following = next((fan for fan in reference_rises if fan.last >= move.end_time - 1e-6), None)
                advanced = (following is not None and following.percent == piece.fan
                            and following.time - DELAY - tolerance <= piece.start_time < following.time + tolerance)
                kicked = piece.fan == 100 and any(kick.start.time - tolerance <= piece.start_time <= kick.end.time + tolerance
                                                  for kick in self.kicks)
                self.assertTrue(advanced or kicked, f"layer {piece.layer}, {piece.role}: {move.fan}% became {piece.fan}%")
        self.assertGreater(changed, 0, "the speedup changed nothing outside the Override")

    def test_a_kick_is_followed_by_its_target(self) -> None:
        # The 100% periods that the dynamic overhang fan asks for itself (layer 12 only) are told from the
        # kicks by the reference: they are its 100% periods, possibly moved earlier and joined.
        self.assertEqual({period.start.layer for period in full_periods(self.reference)}, {12})
        self.assertTrue(self.natural, "the 100% of the dynamic overhang fan is lost")
        self.assertGreaterEqual(len(self.kicks), 5)
        self.assertIn(12, {kick.start.layer for kick in self.kicks}, "no kick on the layer with the overhangs")
        for kick in self.kicks:
            self.assertGreater(kick.end.percent, kick.before, f"layer {kick.start.layer}: a kick for a fall")
            # fan_kickstart seconds for a rise of 100 points, in proportion to the rise from the fan running before.
            self.assertAlmostEqual(kick.end.time - kick.start.time, KICKSTART * (kick.end.percent - kick.before) / 100,
                                   delta=PLACEMENT_TOLERANCE,
                                   msg=f"layer {kick.start.layer}: {kick.before}% -> {kick.end.percent}%")
        # S255 is what the file holds for these 100%.
        self.assertIn("M106 S255", self.chain.text)

    def test_extruded_volume_layers_and_geometry_are_kept(self) -> None:
        self.assertEqual(self.chain.layer_count(), self.reference.layer_count())
        self.assertAlmostEqual(sum(move.deposited_e for move in self.chain.moves),
                               sum(move.deposited_e for move in self.reference.moves), delta=1e-3)
        # Only cuts: every point added lies on the move it cuts, in order, with the same feed rate, length and E.
        pieces = cut_moves(self.reference, self.chain)
        self.assertTrue(any(len(parts) > 1 for _, parts in pieces), "nothing was cut")
        self.assertAlmostEqual(self.chain.moves[-1].end_time, self.reference.moves[-1].end_time, delta=1e-3,
                               msg="the modeled time changed")
        for layer in range(self.chain.layer_count()):
            self.assertAlmostEqual(self.chain.layer_starts[layer], self.reference.layer_starts[layer], delta=1e-3)

    def test_exports_are_deterministic(self) -> None:
        again = run_export("chain-again", self.timed, self.model, self.datadir)
        self.assertEqual(again.text, self.chain.text)
        self.assertEqual(run_export("reference-again", self.options, self.model, self.without).text,
                         self.reference.text)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-executable", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path, default=PROJECT / ".build")
    parser.add_argument("--runtime-path", action="append", default=[],
                        help="directory added to PATH for the host's DLL dependencies")
    args, unittest_args = parser.parse_known_args()
    BUILD_DIR = args.build_dir.resolve()
    HOST_EXECUTABLE = args.host_executable.resolve()
    OUTPUT_ROOT = BUILD_DIR / "test-results" / "fan-chain"
    RUNTIME_PATH = [str(Path(path).resolve()) for path in args.runtime_path]
    unittest.main(argv=[sys.argv[0], *unittest_args], verbosity=2)
