"""End-to-end checks for the regional fan speed overrides.

Each case exports a generated model through the host executable and reads the
fan percentage active during every extrusion move (M106 S is 0-255).
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
    # Automatic requests: 40% everywhere, without time or first-layer changes.
    "min_fan_speed = 0",
    "default_fan_speed = 40",
    "bridge_fan_speed = 40",
    "fan_below_layer_time = 0",
    "slowdown_below_layer_time = 0",
    "disable_fan_first_layers = 0",
    "full_fan_speed_layer = 0",
)
WORD = re.compile(r"([A-Z])([+-]?(?:\d+(?:\.\d*)?|\.\d+))")


@dataclass
class Move:
    layer: int
    role: str
    start: tuple[float, float]
    end: tuple[float, float]
    deposited_e: float
    fan: float

    @property
    def middle(self) -> tuple[float, float]:
        return ((self.start[0] + self.end[0]) / 2, (self.start[1] + self.end[1]) / 2)


def box(x0: float, y0: float, z0: float, x1: float, y1: float, z1: float):
    vertices = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = ((3, 2, 1), (1, 0, 3), (4, 5, 6), (6, 7, 4), (0, 1, 5), (5, 4, 0),
             (1, 2, 6), (6, 5, 1), (2, 3, 7), (7, 6, 2), (3, 0, 4), (4, 7, 3))
    return vertices, faces


def write_3mf(path: Path, volumes: list[tuple[tuple, dict[str, str]]], object_config: dict[str, str] | None = None) -> None:
    """One object made of box volumes, each with its own settings."""
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
    config = ('<?xml version="1.0" encoding="UTF-8"?><config><object id="1">'
              + "".join(metadata("object", key, value) for key, value in (object_config or {}).items()))
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


def write_plate_with_hole(path: Path) -> None:
    """ASCII STL: 30 mm square plate, 1 mm thick, centred 10 mm square hole."""
    outer = [(0, 0), (30, 0), (30, 30), (0, 30)]
    hole = [(10, 10), (10, 20), (20, 20), (20, 10)]
    facets = []

    def quad(a, b, c, d):
        facets.extend(((a, b, c), (a, c, d)))

    for z, flip in ((1, False), (0, True)):
        for i in range(4):
            o0, o1 = outer[i], outer[(i + 1) % 4]
            h0, h1 = hole[(4 - i) % 4], hole[(3 - i) % 4]
            points = [(*o0, z), (*o1, z), (*h1, z), (*h0, z)]
            quad(*(reversed(points) if flip else points))
    for ring in (outer, hole):
        for i in range(4):
            a, b = ring[i], ring[(i + 1) % 4]
            quad((*a, 0), (*b, 0), (*b, 1), (*a, 1))
    lines = ["solid plate"]
    for triangle in facets:
        lines.append(" facet normal 0 0 0\n  outer loop")
        lines += [f"   vertex {x} {y} {z}" for x, y, z in triangle]
        lines.append("  endloop\n endfacet")
    lines.append("endsolid plate")
    path.write_text("\n".join(lines) + "\n")


def run_export(name: str, options: tuple[str, ...], model: Path, datadir: Path | None = None,
               arguments: tuple[str, ...] = ()) -> list[Move]:
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
    result = subprocess.run([str(HOST_EXECUTABLE), "--datadir", str(datadir or PLUGIN_DATADIR), "--export-gcode",
                             "--load", str(profile), *arguments, "--output", str(output), str(model)],
                            capture_output=True, timeout=1800, env=environment)
    log = (result.stdout + result.stderr).decode("utf-8", errors="replace")
    (case_dir / "host.log").write_text(log, encoding="utf-8")
    if result.returncode != 0 or not output.is_file():
        raise AssertionError(f"{name}: export failed (exit {result.returncode}); see {case_dir / 'host.log'}")
    for message in ("will be disabled", "unknown option", "[error]"):
        if message in log.lower():
            raise AssertionError(f"{name}: {message!r} in {case_dir / 'host.log'}")
    return moves(output)


def moves(path: Path) -> list[Move]:
    """Every XY move after the start G-code, with the fan value active during it."""
    x = y = e = 0.0
    fan = 0.0
    absolute_e = True
    role = ""
    layer = -1
    out: list[Move] = []
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
        if command == "M106":
            fan = round(fields.get("S", 255.0) * 100 / 255)
        elif command == "M107":
            fan = 0.0
        elif command == "M82":
            absolute_e = True
        elif command == "M83":
            absolute_e = False
        elif command == "G92" and "E" in fields:
            e = fields["E"]
        elif command in ("G0", "G1", "G2", "G3"):
            nx, ny = fields.get("X", x), fields.get("Y", y)
            de = 0.0
            if "E" in fields:
                de = fields["E"] - e if absolute_e else fields["E"]
                e = fields["E"] if absolute_e else e + fields["E"]
            if layer >= 0 and (nx, ny) != (x, y):
                out.append(Move(layer, role, (x, y), (nx, ny), de, fan))
            x, y = nx, ny
    return out


def deposits(all_moves: list[Move]) -> list[Move]:
    return [move for move in all_moves if move.deposited_e > 0]


class RegionalFan(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        plugin = PLUGIN_DATADIR / "plugins" / "nematx-regional-fan" / ("plugin.dll" if sys.platform == "win32" else "plugin.so")
        for required in (HOST_EXECUTABLE, plugin):
            if not required.is_file():
                raise FileNotFoundError(f"Missing build or staged package input: {required}")
        OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
        cls.cube = OUTPUT_ROOT / "cube.3mf"
        write_3mf(cls.cube, [((0, 0, 0, 10, 10, 1), {})])

    def assert_fans(self, selected: list[Move], expected: float, what: str) -> None:
        self.assertTrue(selected, f"no {what}")
        values = sorted({move.fan for move in selected})
        self.assertEqual(values, [expected], f"{what}: fan values {values}")

    def test_override_is_protected_from_cooling_and_first_layers(self) -> None:
        # Host defaults stop the fan on the first layer and raise it on short layers.
        result = deposits(run_export("protected", (
            "print_fan_speed = 70%", "skirts = 1", "disable_fan_first_layers = 1",
            "fan_below_layer_time = 60", "full_fan_speed_layer = 4"), self.cube))
        self.assert_fans([move for move in result if move.layer == 0], 70, "first-layer deposits")
        self.assert_fans(result, 70, "deposits")
        self.assertTrue(any("skirt" in move.role.lower() for move in result), "the skirt is not printed")

    def test_role_overrides_win_over_the_general_override(self) -> None:
        # A 10 mm plate on a 2 mm pillar: bridge infill and overhang perimeters under the plate.
        model = OUTPUT_ROOT / "table.3mf"
        write_3mf(model, [((4, 0, 0, 6, 10, 2), {}), ((0, 0, 2, 10, 10, 3), {})])
        result = deposits(run_export("roles", (
            "print_fan_speed = 70%", "print_perimeters_fan_speed = 45%", "print_solid_infill_fan_speed = 55%",
            "print_bridge_fan_speed = 0%", "fill_density = 20%"), model))
        # Overhang perimeters are not perimeters for the legacy rule; an enabled 0% stops the fan.
        for role, fan in {"External perimeter": 45, "Overhang perimeter": 70, "Bridge infill": 0, "Solid infill": 55,
                          "Top solid infill": 55, "Internal infill": 70}.items():
            self.assert_fans([move for move in result if move.role == role], fan, role)
    def test_paths_are_split_at_region_boundaries_of_every_instance(self) -> None:
        model = OUTPUT_ROOT / "two-regions.3mf"
        write_3mf(model, [((0, 0, 0, 10, 10, 1), {}), ((10, 0, 0, 20, 10, 1), {"print_fan_speed": "70%"})])
        result = deposits(run_export("boundary", ("fill_density = 20%", "top_solid_layers = 0",
                                                  "bottom_solid_layers = 0"), model, arguments=("--duplicate", "2")))
        # The two instances are 20 x 10 mm: separate them by the widest gap.
        xs = sorted(x for move in result for x in (move.start[0], move.end[0]))
        gap = max(range(1, len(xs)), key=lambda index: xs[index] - xs[index - 1])
        if xs[gap] - xs[gap - 1] > 2:
            instances = [[move for move in result if (move.middle[0] < xs[gap]) == left] for left in (True, False)]
        else:
            ys = sorted(y for move in result for y in (move.start[1], move.end[1]))
            gap = max(range(1, len(ys)), key=lambda index: ys[index] - ys[index - 1])
            instances = [[move for move in result if (move.middle[1] < ys[gap]) == low] for low in (True, False)]
        for instance in instances:
            boundary = (min(min(m.start[0], m.end[0]) for m in instance) + max(max(m.start[0], m.end[0]) for m in instance)) / 2
            crossing = [move for move in instance if (move.start[0] - boundary) * (move.end[0] - boundary) < -1e-6]
            self.assertFalse(crossing, f"moves crossing the region boundary x={boundary:g}: {crossing[:3]}")
            self.assert_fans([move for move in instance if move.middle[0] < boundary], 40, "left region deposits")
            self.assert_fans([move for move in instance if move.middle[0] > boundary], 70, "right region deposits")
            self.assertTrue(any(min(abs(move.start[0] - boundary), abs(move.end[0] - boundary)) < 0.03
                                and "infill" in move.role.lower() for move in instance), "no infill cut at the boundary")

    def test_brim_uses_the_print_values_rather_than_the_region(self) -> None:
        # The brim belongs to the print-level auxiliary object: it keeps the
        # automatic 40% instead of the object (60%) or volume (50%) override.
        model = OUTPUT_ROOT / "cube-with-overrides.3mf"
        write_3mf(model, [((0, 0, 0, 10, 10, 1), {"print_fan_speed": "50%"})], {"print_fan_speed": "60%"})
        result = deposits(run_export("brim", ("brim_width = 3",), model))
        brim = [move for move in result if "skirt" in move.role.lower() or "brim" in move.role.lower()]
        self.assert_fans(brim, 40, "brim")
        self.assert_fans([move for move in result if move not in brim], 50, "object deposits")
    def test_around_holes_connectors_keep_the_fan_state(self) -> None:
        aroundholes = BUILD_DIR / "test-data" / "infill_aroundholes" / "plugins"
        if not (aroundholes / "nematx-infill-aroundholes").is_dir():
            raise FileNotFoundError(f"Missing staged package: {aroundholes / 'nematx-infill-aroundholes'}")
        # One datadir with both packages activated.
        datadir = OUTPUT_ROOT / "aroundholes-datadir"
        shutil.rmtree(datadir, ignore_errors=True)
        shutil.copytree(PLUGIN_DATADIR, datadir)
        shutil.copytree(aroundholes / "nematx-infill-aroundholes", datadir / "plugins" / "nematx-infill-aroundholes")
        installed = dict(re.findall(r"^nematx-infill-aroundholes(\S*) = (\S+)$",
                                    (aroundholes / "activated.ini").read_text(encoding="utf-8"), re.MULTILINE))
        activation = datadir / "plugins" / "activated.ini"
        subprocess.run([sys.executable, str(PROJECT / "cmake" / "prepare_activated_ini.py"), "--defaults", str(activation),
                        "--output", str(activation), "--plugin-id", "rectilineararoundholes",
                        "--package-id", "nematx-infill-aroundholes", "--plugin-version", installed[""],
                        "--slicer-version", installed[".slicer_version"]], check=True)
        model = OUTPUT_ROOT / "plate.stl"
        write_plate_with_hole(model)
        result = run_export("aroundholes", (
            "print_fan_speed = 70%", "perimeters = 1", "top_solid_layers = 0", "bottom_solid_layers = 0",
            "fill_density = 30%", "fill_pattern = rectilineararoundholes", "fill_rectilinearholes_travel_flow_ratio = 0%"),
            model, datadir)
        # The first layer uses the solid pattern, without connectors, and the top layer is solid infill: only
        # the layers that deposit internal infill are checked. A move is of a layer by its position in the
        # file, and a travel keeps the role of the last ";TYPE:" before it, so only deposits count as infill
        # (the layer change travel carries the role of the previous layer's last path).
        checked = 0
        for layer in sorted({move.layer for move in result} - {0}):
            infill = [index for index, move in enumerate(result)
                      if move.layer == layer and move.role == "Internal infill" and move.deposited_e > 0]
            if not infill:
                continue
            checked += 1
            span = result[infill[0]:infill[-1] + 1]
            connectors = [move for move in span if move.deposited_e == 0 and move.role == "Internal infill"]
            self.assertTrue(connectors, f"no zero-flow connector on layer {layer}")
            self.assert_fans(span, 70, f"infill and connectors of layer {layer}")
        self.assertGreaterEqual(checked, 2, "too few layers with internal infill")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-executable", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path, default=PROJECT / ".build")
    parser.add_argument("--runtime-path", action="append", default=[],
                        help="directory added to PATH for the host's DLL dependencies")
    args, unittest_args = parser.parse_known_args()
    BUILD_DIR = args.build_dir.resolve()
    HOST_EXECUTABLE = args.host_executable.resolve()
    PLUGIN_DATADIR = BUILD_DIR / "test-data" / "regional_fan"
    OUTPUT_ROOT = BUILD_DIR / "test-results" / "regional-fan"
    RUNTIME_PATH = [str(Path(path).resolve()) for path in args.runtime_path]
    unittest.main(argv=[sys.argv[0], *unittest_args], verbosity=2)
