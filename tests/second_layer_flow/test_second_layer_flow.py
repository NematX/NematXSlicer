"""End-to-end checks for the external second-layer flow plugin."""

from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
import unittest
from dataclasses import dataclass, field
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile


PROJECT = Path(__file__).resolve().parents[2]
TEST_DIRECTORY = Path(__file__).resolve().parent
HOST_BUILD: Path
HOST_EXECUTABLE: Path
HOST_DATADIR: Path
SECOND_LAYER_FLOW_DATADIR: Path
MODEL = TEST_DIRECTORY / "fixtures" / "cube-5mm.stl"
OUTPUT_ROOT: Path
PLUGIN_EXTENSION = {"win32": ".dll", "darwin": ".dylib"}.get(sys.platform, ".so")
PLUGIN_FILENAME = f"plugin{PLUGIN_EXTENSION}"
NUMBER = re.compile(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?")
WORD = re.compile(r"([A-Z])(" + NUMBER.pattern + r")")


@dataclass
class LayerMetrics:
    deposited_e: float = 0.0
    path_length: float = 0.0
    feed_rates: list[float] = field(default_factory=list)

    @property
    def e_per_mm(self) -> float:
        return self.deposited_e / self.path_length if self.path_length else 0.0

    @property
    def average_feed(self) -> float:
        return sum(self.feed_rates) / len(self.feed_rates) if self.feed_rates else 0.0


@dataclass
class ExtrusionMove:
    layer: int
    extrusion_type: str
    start_x: float
    start_y: float
    end_x: float
    end_y: float
    deposited_e: float
    path_length: float

    @property
    def e_per_mm(self) -> float:
        return self.deposited_e / self.path_length if self.path_length else 0.0


def make_two_region_3mf(
    path: Path,
    with_region_config: bool,
    separated_islands: bool = False,
) -> None:
    """Create two 10 x 10 mm regions, optionally as disconnected islands."""
    vertices: list[str] = []
    triangles: list[str] = []
    box_faces = (
        (3, 2, 1), (1, 0, 3), (4, 5, 6), (6, 7, 4),
        (0, 1, 5), (5, 4, 0), (1, 2, 6), (6, 5, 1),
        (2, 3, 7), (7, 6, 2), (3, 0, 4), (4, 7, 3),
    )
    offsets = (0.0, 20.0) if separated_islands else (0.0, 10.0)
    for x_offset in offsets:
        base = len(vertices)
        for x, y, z in (
            (0, 0, 0), (10, 0, 0), (10, 10, 0), (0, 10, 0),
            (0, 0, 2), (10, 0, 2), (10, 10, 2), (0, 10, 2),
        ):
            vertices.append(
                f'<vertex x="{x + x_offset:g}" y="{y:g}" z="{z:g}" />'
            )
        for face in box_faces:
            triangles.append(
                f'<triangle v1="{base + face[0]}" v2="{base + face[1]}" '
                f'v3="{base + face[2]}" />'
            )

    model = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<model unit="millimeter" xml:lang="en-US" '
        'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
        '<resources><object id="1" type="model"><mesh><vertices>'
        + "".join(vertices)
        + "</vertices><triangles>"
        + "".join(triangles)
        + '</triangles></mesh></object></resources><build><item objectid="1" />'
        '</build></model>'
    )
    content_types = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
        '<Default Extension="rels" '
        'ContentType="application/vnd.openxmlformats-package.relationships+xml" />'
        '<Default Extension="model" '
        'ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml" />'
        '<Override PartName="/Metadata/Slic3r_PE_model.config" '
        'ContentType="application/xml" /></Types>'
    )
    relationships = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
        'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" />'
        '</Relationships>'
    )

    with ZipFile(path, "w", ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml", content_types)
        archive.writestr("_rels/.rels", relationships)
        archive.writestr("3D/3dmodel.model", model)
        if with_region_config:
            model_config = (
                '<?xml version="1.0" encoding="UTF-8"?>'
                '<config><object id="1">'
                '<volume firstid="0" lastid="11">'
                '<metadata type="volume" key="name" value="region-100" />'
                '<metadata type="volume" key="volume_type" value="ModelPart" />'
                '<metadata type="volume" key="second_layer_flow_ratio" value="100%" />'
                '</volume><volume firstid="12" lastid="23">'
                '<metadata type="volume" key="name" value="region-150" />'
                '<metadata type="volume" key="volume_type" value="ModelPart" />'
                '<metadata type="volume" key="second_layer_flow_ratio" value="150%" />'
                '</volume></object></config>'
            )
            archive.writestr("Metadata/Slic3r_PE_model.config", model_config)


def analyze_gcode(path: Path) -> list[LayerMetrics]:
    layers: list[LayerMetrics] = []
    coordinates = {"X": 0.0, "Y": 0.0, "Z": 0.0}
    absolute_xyz = True
    absolute_e = True
    current_e = 0.0
    feed_rate = 0.0
    current_layer: LayerMetrics | None = None

    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.split(";", 1)[0].strip().upper()
        if raw_line.startswith(";LAYER_CHANGE"):
            current_layer = LayerMetrics()
            layers.append(current_layer)
            continue
        if not line:
            continue
        fields = {letter: float(value) for letter, value in WORD.findall(line)}
        if line.startswith("G90"):
            absolute_xyz = True
            continue
        if line.startswith("G91"):
            absolute_xyz = False
            continue
        if line.startswith("M82"):
            absolute_e = True
            continue
        if line.startswith("M83"):
            absolute_e = False
            continue
        if line.startswith("G92"):
            for axis in coordinates:
                if axis in fields:
                    coordinates[axis] = fields[axis]
            if "E" in fields:
                current_e = fields["E"]
            continue
        if not line.startswith(("G0 ", "G1 ", "G2 ", "G3 ")):
            continue

        start = coordinates.copy()
        for axis in coordinates:
            if axis in fields:
                coordinates[axis] = fields[axis] if absolute_xyz else coordinates[axis] + fields[axis]
        if "F" in fields:
            feed_rate = fields["F"]

        distance = math.sqrt(sum((coordinates[axis] - start[axis]) ** 2 for axis in coordinates))
        if line.startswith(("G2 ", "G3 ")) and distance > 0 and ("I" in fields or "J" in fields):
            center_x = start["X"] + fields.get("I", 0.0)
            center_y = start["Y"] + fields.get("J", 0.0)
            radius = math.hypot(start["X"] - center_x, start["Y"] - center_y)
            start_angle = math.atan2(start["Y"] - center_y, start["X"] - center_x)
            end_angle = math.atan2(coordinates["Y"] - center_y, coordinates["X"] - center_x)
            sweep = (start_angle - end_angle) % (2 * math.pi) if line.startswith("G2 ") else (end_angle - start_angle) % (2 * math.pi)
            arc_length = radius * sweep
            distance = math.hypot(arc_length, coordinates["Z"] - start["Z"])

        if "E" in fields:
            delta_e = fields["E"] - current_e if absolute_e else fields["E"]
            current_e = fields["E"] if absolute_e else current_e + fields["E"]
            if current_layer is not None and delta_e > 0 and distance > 1e-8:
                current_layer.deposited_e += delta_e
                current_layer.path_length += distance
                if feed_rate > 0:
                    current_layer.feed_rates.append(feed_rate)

    return layers


def extrusion_moves(path: Path) -> list[ExtrusionMove]:
    moves: list[ExtrusionMove] = []
    coordinates = {"X": 0.0, "Y": 0.0, "Z": 0.0}
    absolute_xyz = True
    absolute_e = True
    current_e = 0.0
    current_type = ""
    current_layer = -1

    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if raw_line.startswith(";LAYER_CHANGE"):
            current_layer += 1
            continue
        if raw_line.startswith(";TYPE:"):
            current_type = raw_line.partition(":")[2].strip()
            continue
        line = raw_line.split(";", 1)[0].strip().upper()
        if not line:
            continue
        fields = {letter: float(value) for letter, value in WORD.findall(line)}
        if line.startswith("G90"):
            absolute_xyz = True
            continue
        if line.startswith("G91"):
            absolute_xyz = False
            continue
        if line.startswith("M82"):
            absolute_e = True
            continue
        if line.startswith("M83"):
            absolute_e = False
            continue
        if line.startswith("G92"):
            for axis in coordinates:
                if axis in fields:
                    coordinates[axis] = fields[axis]
            if "E" in fields:
                current_e = fields["E"]
            continue
        if not line.startswith(("G0 ", "G1 ", "G2 ", "G3 ")):
            continue

        start = coordinates.copy()
        for axis in coordinates:
            if axis in fields:
                coordinates[axis] = fields[axis] if absolute_xyz else coordinates[axis] + fields[axis]
        distance = math.sqrt(sum((coordinates[axis] - start[axis]) ** 2 for axis in coordinates))
        if "E" not in fields:
            continue
        delta_e = fields["E"] - current_e if absolute_e else fields["E"]
        current_e = fields["E"] if absolute_e else current_e + fields["E"]
        if current_layer >= 0 and delta_e > 0 and distance > 1e-8:
            moves.append(
                ExtrusionMove(
                    current_layer, current_type, start["X"], start["Y"],
                    coordinates["X"], coordinates["Y"], delta_e, distance,
                )
            )
    return moves


def profile_text(
    ratio: str | None,
    max_volumetric_speed: float,
    extra_options: tuple[str, ...] = (),
) -> str:
    lines = [
        "seam_visibility = 0",
        "layer_height = 0.2",
        "first_layer_height = 0.2",
        "cooling = 0",
        f"max_volumetric_speed = {max_volumetric_speed:g}",
    ]
    if ratio is not None:
        lines.append(f"second_layer_flow_ratio = {ratio}")
    lines.extend(extra_options)
    return "\n".join(lines) + "\n"


def run_export(
    name: str,
    ratio: str | None,
    plugin_enabled: bool,
    max_volumetric_speed: float = 0.0,
    model: Path = MODEL,
    extra_options: tuple[str, ...] = (),
) -> tuple[Path, str, int]:
    case_dir = OUTPUT_ROOT / name
    case_dir.mkdir(parents=True, exist_ok=True)
    profile = case_dir / "profile.ini"
    output = case_dir / "result.gcode"
    log_path = case_dir / "host.log"
    profile.write_text(profile_text(ratio, max_volumetric_speed, extra_options), encoding="utf-8")
    output.unlink(missing_ok=True)

    command = [
        str(HOST_EXECUTABLE),
        "--datadir",
        str(SECOND_LAYER_FLOW_DATADIR if plugin_enabled else HOST_DATADIR),
        "--export-gcode",
        "--load",
        str(profile),
        "--output",
        str(output),
        str(model),
    ]
    result = subprocess.run(command, cwd=HOST_BUILD, capture_output=True, timeout=180)
    log = (result.stdout + result.stderr).decode("utf-8", errors="replace")
    log_path.write_text(log, encoding="utf-8")
    return output, log, result.returncode


class SecondLayerFlowIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        for required in (
            HOST_EXECUTABLE,
            MODEL,
            SECOND_LAYER_FLOW_DATADIR / "plugins" / "nematx-extrusion" / PLUGIN_FILENAME,
        ):
            if not required.is_file():
                raise FileNotFoundError(f"Missing build or staged package input: {required}")

        cls.outputs: dict[str, Path] = {}
        cls.logs: dict[str, str] = {}
        for name, ratio, enabled in (
            ("baseline", None, False),
            ("neutral-100", "100%", True),
            ("positive-1", "1%", True),
            ("reduced-80", "80%", True),
            ("raised-125", "125%", True),
        ):
            output, log, return_code = run_export(name, ratio, enabled)
            if return_code != 0 or not output.is_file() or output.stat().st_size == 0:
                raise AssertionError(f"{name} export failed (exit {return_code}); see {OUTPUT_ROOT / name / 'host.log'}")
            cls.outputs[name] = output
            cls.logs[name] = log

        regional_profile = ("bottom_solid_layers = 0", "top_solid_layers = 0", "fill_density = 20%")
        for name, with_plugin, separated in (
            ("two-region-baseline", False, False),
            ("two-region-100-150", True, False),
            ("separate-islands-baseline", False, True),
            ("separate-islands-100-150", True, True),
        ):
            regional_model = OUTPUT_ROOT / f"{name}.3mf"
            make_two_region_3mf(
                regional_model,
                with_region_config=with_plugin,
                separated_islands=separated,
            )
            output, log, return_code = run_export(
                name,
                None,
                with_plugin,
                model=regional_model,
                extra_options=regional_profile,
            )
            if return_code != 0 or not output.is_file() or output.stat().st_size == 0:
                raise AssertionError(f"{name} export failed (exit {return_code}); see {OUTPUT_ROOT / name / 'host.log'}")
            cls.outputs[name] = output
            cls.logs[name] = log

        for name, ratio, enabled in (
            ("global-cap-baseline", None, False),
            ("global-cap-80", "80%", True),
            ("global-cap-125", "125%", True),
        ):
            output, log, return_code = run_export(name, ratio, enabled, max_volumetric_speed=2.0)
            if return_code != 0 or not output.is_file() or output.stat().st_size == 0:
                raise AssertionError(f"{name} export failed (exit {return_code}); see {OUTPUT_ROOT / name / 'host.log'}")
            cls.outputs[name] = output
            cls.logs[name] = log

        cls.metrics = {name: analyze_gcode(path) for name, path in cls.outputs.items()}

    def test_cube_has_enough_layers_and_flow_changes_only_on_the_second_layer(self) -> None:
        baseline = self.metrics["baseline"]
        self.assertGreaterEqual(len(baseline), 4)
        self.assertEqual(len(baseline), len(self.metrics["reduced-80"]))

        for case, expected in (
            ("neutral-100", 1.0),
            ("positive-1", 0.01),
            ("reduced-80", 0.8),
            ("raised-125", 1.25),
        ):
            result = self.metrics[case]
            self.assertEqual(len(baseline), len(result))
            for layer_index, (initial, changed) in enumerate(zip(baseline, result)):
                if initial.path_length <= 0:
                    continue
                ratio = changed.e_per_mm / initial.e_per_mm
                target = expected if layer_index == 1 else 1.0
                self.assertAlmostEqual(
                    ratio,
                    target,
                    delta=max(1e-5, expected * 0.025),
                    msg=f"{case}, LAYER_CHANGE index {layer_index}: expected {target}, got {ratio}",
                )

    def test_fixed_speed_and_cooling_output_stay_stable(self) -> None:
        baseline = self.metrics["baseline"]
        for case in ("neutral-100", "positive-1", "reduced-80", "raised-125"):
            result = self.metrics[case]
            for layer_index, (initial, changed) in enumerate(zip(baseline, result)):
                if initial.average_feed <= 0:
                    continue
                self.assertAlmostEqual(
                    changed.average_feed / initial.average_feed,
                    1.0,
                    delta=0.025,
                    msg=f"{case}, fixed-speed layer {layer_index}",
                )

    def test_global_volumetric_cap_uses_the_modified_volume(self) -> None:
        baseline = self.metrics["global-cap-baseline"][1]
        for case, ratio in (("global-cap-80", 0.8), ("global-cap-125", 1.25)):
            changed = self.metrics[case][1]
            self.assertAlmostEqual(changed.e_per_mm / baseline.e_per_mm, ratio, delta=0.025)
            self.assertGreater(baseline.average_feed, 0)
            self.assertAlmostEqual(
                changed.average_feed / baseline.average_feed,
                1.0 / ratio,
                delta=0.08,
                msg=f"{case}: the native 2 mm3/s ceiling should divide by the changed flow",
            )

    def test_regional_flow_splits_at_the_boundary_and_only_scales_the_150_percent_region(self) -> None:
        baseline = extrusion_moves(self.outputs["two-region-baseline"])
        regional = extrusion_moves(self.outputs["two-region-100-150"])
        baseline_fill = [move for move in baseline if move.layer == 1 and "infill" in move.extrusion_type.lower()]
        regional_fill = [move for move in regional if move.layer == 1 and "infill" in move.extrusion_type.lower()]
        self.assertTrue(baseline_fill, "The baseline model must contain second-layer infill.")
        self.assertTrue(regional_fill, "The regional model must contain second-layer infill.")

        x_positions = [
            x
            for move in regional
            if move.layer == 1
            for x in (move.start_x, move.end_x)
        ]
        boundary_x = (min(x_positions) + max(x_positions)) / 2.0
        crosses_boundary = lambda move: (move.start_x - boundary_x) * (move.end_x - boundary_x) < -1e-6
        self.assertTrue(
            any(crosses_boundary(move) for move in baseline_fill),
            "The single-region reference should contain infill moves that cross the region boundary.",
        )
        self.assertFalse(
            any(crosses_boundary(move) for move in regional_fill),
            "Regional infill moves must be cut where they meet the 100%/150% boundary.",
        )

        def weighted_e_per_mm(moves: list[ExtrusionMove]) -> float:
            return sum(move.deposited_e for move in moves) / sum(move.path_length for move in moves)

        baseline_e_per_mm = weighted_e_per_mm(baseline_fill)
        for side, expected in (("100%", 1.0), ("150%", 1.5)):
            is_left = side == "100%"
            side_moves = [
                move
                for move in regional_fill
                if (((move.start_x + move.end_x) / 2.0 < boundary_x) == is_left)
            ]
            self.assertTrue(side_moves, f"No second-layer infill was found in the {side} region.")
            self.assertAlmostEqual(
                weighted_e_per_mm(side_moves) / baseline_e_per_mm,
                expected,
                delta=0.035,
                msg=f"The {side} region must use {expected:g}x the baseline flow.",
            )

        boundary_endpoints = [
            move
            for move in regional_fill
            if abs(move.start_x - boundary_x) < 0.03 or abs(move.end_x - boundary_x) < 0.03
        ]
        self.assertGreaterEqual(
            len(boundary_endpoints),
            2,
            "Split infill fragments should terminate at the shared region boundary.",
        )

        def boundary_y(move: ExtrusionMove) -> float:
            return move.start_y if abs(move.start_x - boundary_x) < 0.03 else move.end_y

        left_boundary_y = [
            boundary_y(move)
            for move in regional_fill
            if (move.start_x + move.end_x) / 2.0 < boundary_x
            and (abs(move.start_x - boundary_x) < 0.03 or abs(move.end_x - boundary_x) < 0.03)
        ]
        right_boundary_y = [
            boundary_y(move)
            for move in regional_fill
            if (move.start_x + move.end_x) / 2.0 > boundary_x
            and (abs(move.start_x - boundary_x) < 0.03 or abs(move.end_x - boundary_x) < 0.03)
        ]
        self.assertTrue(
            any(abs(left_y - right_y) < 0.03 for left_y in left_boundary_y for right_y in right_boundary_y),
            "A crossing infill line should produce matching 100% and 150% fragments at the boundary.",
        )

    def test_separate_islands_use_their_region_ratio(self) -> None:
        baseline = [
            move for move in extrusion_moves(self.outputs["separate-islands-baseline"])
            if move.layer == 1 and "infill" in move.extrusion_type.lower()
        ]
        regional = [
            move for move in extrusion_moves(self.outputs["separate-islands-100-150"])
            if move.layer == 1 and "infill" in move.extrusion_type.lower()
        ]
        self.assertTrue(baseline, "The reference must contain second-layer infill in both islands.")
        self.assertTrue(regional, "The regional model must contain second-layer infill in both islands.")

        x_positions = sorted(x for move in regional for x in (move.start_x, move.end_x))
        gaps = [(right - left, left, right) for left, right in zip(x_positions, x_positions[1:])]
        largest_gap, left_edge, right_edge = max(gaps)
        self.assertGreater(largest_gap, 5.0, "The fixture must contain two disconnected islands.")
        island_boundary = (left_edge + right_edge) / 2.0

        def component_flow(moves: list[ExtrusionMove], left: bool) -> float:
            component = [
                move for move in moves
                if (((move.start_x + move.end_x) / 2.0 < island_boundary) == left)
            ]
            self.assertTrue(component, "Expected infill in each disconnected island.")
            return sum(move.deposited_e for move in component) / sum(move.path_length for move in component)

        baseline_left = component_flow(baseline, True)
        baseline_right = component_flow(baseline, False)
        self.assertAlmostEqual(component_flow(regional, True) / baseline_left, 1.0, delta=0.035)
        self.assertAlmostEqual(component_flow(regional, False) / baseline_right, 1.5, delta=0.035)

    def test_zero_ratio_is_rejected_without_a_successful_output(self) -> None:
        output, log, return_code = run_export("invalid-zero", "0%", True)
        self.assertFalse(output.exists(), "0% must not produce a G-code result")
        self.assertNotEqual(
            return_code,
            0,
            f"A 0% profile must be rejected; see {OUTPUT_ROOT / 'invalid-zero' / 'host.log'}",
        )
        self.assertIn(
            "Value out of range: second_layer_flow_ratio",
            log,
            f"Expected the host range diagnostic, exit {return_code}; "
            f"see {OUTPUT_ROOT / 'invalid-zero' / 'host.log'}",
        )

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-dir", type=Path, required=True)
    parser.add_argument("--host-executable", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path, default=PROJECT / ".build")
    parser.add_argument("--host-datadir", type=Path)
    parser.add_argument("--plugin-datadir", type=Path)
    args, unittest_args = parser.parse_known_args()

    build_dir = args.build_dir.resolve()
    HOST_BUILD = args.host_dir.resolve()
    HOST_EXECUTABLE = args.host_executable.resolve()
    HOST_DATADIR = (args.host_datadir or build_dir / "test-data" / "host").resolve()
    SECOND_LAYER_FLOW_DATADIR = (
        args.plugin_datadir or build_dir / "test-data" / "second_layer_flow"
    ).resolve()
    OUTPUT_ROOT = build_dir / "test-results" / "second-layer-flow"
    unittest.main(argv=[sys.argv[0], *unittest_args], verbosity=2)
