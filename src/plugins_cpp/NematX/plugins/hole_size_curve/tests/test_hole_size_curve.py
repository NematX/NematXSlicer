"""End-to-end checks for the hole compensation by diameter.

Each case exports two square plates through the host executable, one with a
3 mm round hole and one with a 10 mm round hole, then measures the external
perimeter loop of each hole on a middle layer. The loop follows the hole wall at
half an extrusion width, so the change of its width between two exports is the
change of the hole diameter.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import subprocess
import sys
import unittest
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

HOST_EXECUTABLE: Path
PLUGIN_DATADIR: Path
OUTPUT_ROOT: Path
RUNTIME_PATH: list[str] = []

BASE_PROFILE = (
    # Seam visibility is very slow with a Debug host and irrelevant here.
    "seam_visibility = 0",
    "layer_height = 0.2",
    "first_layer_height = 0.2",
    "perimeters = 1",
    "fill_density = 0%",
    "top_solid_layers = 0",
    "bottom_solid_layers = 0",
    "skirts = 0",
    "brim_width = 0",
    "gcode_comments = 1",
    "hole_size_compensation = 0",
    "xy_inner_size_compensation = 0",
)

# Hole diameter (mm) -> compensation (mm): -0.1 up to 5 mm, -0.3 from 6 mm.
LINEAR_CURVE = "0:4:1:0x-0.1:5x-0.1:6x-0.3:20x-0.3"
# Square curve: the value of the point at or before the diameter.
SQUARE_CURVE = "0:3:0:0x-0.1:5x-0.2:20x-0.2"


def plate_with_round_hole(size: float, radius: float, x0: float, points: int = 64):
    """Vertices and triangles of a 2 mm square plate with a centred round hole.

    The plate cap is built from quads between the hole ring and the same ring
    projected on the square, so no triangulation library is needed. With a
    multiple of 8 points, the projection hits the square corners.
    """
    half = size / 2
    centre = (x0 + half, half)
    inner, outer = [], []
    for index in range(points):
        angle = 2 * math.pi * index / points
        c, s = math.cos(angle), math.sin(angle)
        inner.append((centre[0] + radius * c, centre[1] + radius * s))
        scale = half / max(abs(c), abs(s))
        outer.append((centre[0] + scale * c, centre[1] + scale * s))
    vertices = []
    for z in (0, 2):
        vertices += [(x, y, z) for x, y in inner] + [(x, y, z) for x, y in outer]
    n = points
    inner_bottom, outer_bottom, inner_top, outer_top = 0, n, 2 * n, 3 * n
    triangles = []
    for i in range(n):
        j = (i + 1) % n
        # Bottom (facing down) and top (facing up) caps.
        triangles += [(inner_bottom + i, outer_bottom + j, outer_bottom + i),
                      (inner_bottom + i, inner_bottom + j, outer_bottom + j),
                      (inner_top + i, outer_top + i, outer_top + j),
                      (inner_top + i, outer_top + j, inner_top + j)]
        # Outer wall (facing out) and hole wall (facing the hole axis).
        triangles += [(outer_bottom + i, outer_bottom + j, outer_top + j),
                      (outer_bottom + i, outer_top + j, outer_top + i),
                      (inner_bottom + i, inner_top + j, inner_bottom + j),
                      (inner_bottom + i, inner_top + i, inner_top + j)]
    return vertices, triangles


def write_plates_3mf(path: Path) -> None:
    objects = []
    for object_id, (size, radius, x0) in enumerate(((12, 1.5, 0), (20, 5, 20)), start=1):
        vertices, triangles = plate_with_round_hole(size, radius, x0)
        objects.append(
            f'<object id="{object_id}" type="model"><mesh><vertices>'
            + "".join(f'<vertex x="{x:.5f}" y="{y:.5f}" z="{z}" />' for x, y, z in vertices)
            + "</vertices><triangles>"
            + "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}" />' for a, b, c in triangles)
            + "</triangles></mesh></object>")
    model = ('<?xml version="1.0" encoding="UTF-8"?><model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"><resources>'
             + "".join(objects) + '</resources><build><item objectid="1" /><item objectid="2" /></build></model>')
    content_types = ('<?xml version="1.0" encoding="UTF-8"?>'
                     '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
                     '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml" />'
                     '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml" />'
                     '</Types>')
    relationships = ('<?xml version="1.0" encoding="UTF-8"?>'
                     '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
                     '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
                     'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" /></Relationships>')
    with ZipFile(path, "w", ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml", content_types)
        archive.writestr("_rels/.rels", relationships)
        archive.writestr("3D/3dmodel.model", model)


def run_export(name: str, options: tuple[str, ...], model: Path) -> Path:
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
    result = subprocess.run([str(HOST_EXECUTABLE), "--datadir", str(PLUGIN_DATADIR), "--export-gcode",
                             "--load", str(profile), "--output", str(output), str(model)],
                            capture_output=True, timeout=1800, env=environment)
    log = (result.stdout + result.stderr).decode("utf-8", errors="replace")
    (case_dir / "host.log").write_text(log, encoding="utf-8")
    if result.returncode != 0 or not output.is_file():
        raise AssertionError(f"{name}: export failed (exit {result.returncode}); see {case_dir / 'host.log'}")
    for message in ("will be disabled", "unknown option", "[error]"):
        if message in log.lower():
            raise AssertionError(f"{name}: {message!r} in {case_dir / 'host.log'}")
    return output


def hole_loop_widths(path: Path, layer: int = 4) -> list[float]:
    """Widths (X extent) of the hole loops on one layer, smallest first.

    Loops are runs of consecutive extruding moves of the external perimeter;
    the hole loops are those inside the outer loop of their plate.
    """
    x = y = e = 0.0
    absolute_e = True
    role = ""
    layer_index = -1
    loops: list[list[tuple[float, float]]] = []
    current: list[tuple[float, float]] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        if raw.startswith(";LAYER_CHANGE"):
            layer_index += 1
        if raw.startswith(";TYPE:"):
            role = raw[6:].strip()
        code = raw.split(";", 1)[0].strip()
        words = dict(re.findall(r"([A-Z])(-?[\d.]+)", code))
        command = code.split(" ", 1)[0] if code else ""
        if command == "M82":
            absolute_e = True
        elif command == "M83":
            absolute_e = False
        elif command == "G92" and "E" in words:
            e = float(words["E"])
        elif command in ("G0", "G1"):
            nx, ny = float(words.get("X", x)), float(words.get("Y", y))
            extruded = ("E" in words) and ((float(words["E"]) - e) if absolute_e else float(words["E"])) > 0
            if "E" in words and absolute_e:
                e = float(words["E"])
            if extruded and role == "External perimeter" and layer_index == layer:
                if not current:
                    current.append((x, y))
                current.append((nx, ny))
            elif current:
                loops.append(current)
                current = []
            x, y = nx, ny
    if current:
        loops.append(current)
    boxes = []
    for loop in loops:
        xs, ys = [p[0] for p in loop], [p[1] for p in loop]
        boxes.append((min(xs), min(ys), max(xs), max(ys)))
    # A hole loop lies inside the outer loop of its plate.
    def inside(box, other):
        return other[0] < box[0] and other[1] < box[1] and box[2] < other[2] and box[3] < other[3]
    return sorted(box[2] - box[0] for box in boxes if any(inside(box, other) for other in boxes))


class HoleSizeCurve(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        plugin = PLUGIN_DATADIR / "plugins" / "nematx-hole-size-curve" / ("plugin.dll" if sys.platform == "win32" else "plugin.so")
        for required in (HOST_EXECUTABLE, plugin):
            if not required.is_file():
                raise FileNotFoundError(f"Missing build or staged package input: {required}")
        OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
        cls.model = OUTPUT_ROOT / "plates.3mf"
        write_plates_3mf(cls.model)
        cls.baseline = hole_loop_widths(run_export("baseline", (), cls.model))
        if len(cls.baseline) != 2:
            raise AssertionError(f"expected two hole loops, found {cls.baseline}")

    def assert_diameter_changes(self, widths: list[float], expected: tuple[float, float]) -> None:
        self.assertEqual(len(widths), 2, widths)
        for width, baseline, change in zip(widths, self.baseline, expected):
            self.assertAlmostEqual(width - baseline, change, delta=0.01,
                                   msg=f"widths {widths}, baseline {self.baseline}")

    def test_baseline_holes(self) -> None:
        # Loop width = hole diameter + one external perimeter width.
        self.assertLess(self.baseline[0], 4.5)
        self.assertGreater(self.baseline[1], 10)

    def test_linear_curve_by_diameter(self) -> None:
        widths = hole_loop_widths(run_export("linear", (f"hole_size_compensations_curve = {LINEAR_CURVE}",), self.model))
        # -0.1 mm on the 3 mm hole, -0.3 mm on the 10 mm hole: diameters +0.2 and +0.6.
        self.assert_diameter_changes(widths, (0.2, 0.6))

    def test_square_curve(self) -> None:
        widths = hole_loop_widths(run_export("square", (f"hole_size_compensations_curve = {SQUARE_CURVE}",), self.model))
        self.assert_diameter_changes(widths, (0.2, 0.4))

    def test_curve_adds_to_hole_and_inner_compensations(self) -> None:
        widths = hole_loop_widths(run_export("cumulative", (
            f"hole_size_compensations_curve = {LINEAR_CURVE}",
            "hole_size_compensation = -0.05",
            "xy_inner_size_compensation = -0.02"), self.model))
        # Each value moves the wall: diameters +2 x (0.1 + 0.05 + 0.02) and +2 x (0.3 + 0.05 + 0.02).
        self.assert_diameter_changes(widths, (0.34, 0.74))

    def test_empty_curve_changes_nothing(self) -> None:
        widths = hole_loop_widths(run_export("empty", ("hole_size_compensations_curve = 0:0:1",), self.model))
        self.assert_diameter_changes(widths, (0.0, 0.0))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-executable", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path, default=Path(__file__).resolve().parents[3] / ".build")
    parser.add_argument("--runtime-path", action="append", default=[],
                        help="directory added to PATH for the host's DLL dependencies")
    args, unittest_args = parser.parse_known_args()
    build_dir = args.build_dir.resolve()
    HOST_EXECUTABLE = args.host_executable.resolve()
    PLUGIN_DATADIR = build_dir / "test-data" / "hole_size_curve"
    OUTPUT_ROOT = build_dir / "test-results" / "hole-size-curve"
    RUNTIME_PATH = [str(Path(path).resolve()) for path in args.runtime_path]
    unittest.main(argv=[sys.argv[0], *unittest_args], verbosity=2)
