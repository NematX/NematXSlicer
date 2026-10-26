"""Real-host smoke test for the rectilinear-around-holes pattern.

Builds nothing: run it after building the nematx_infill_aroundholes target, which
stages the package in an isolated datadir. A 30 x 30 x 1 mm plate with a 10 mm
square hole is sliced with the pattern, with 1% and 0% bypass flow.
"""
import argparse
import math
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WORD = re.compile(r"([A-Z])([+-]?(?:\d+(?:\.\d*)?|\.\d+))")


def write_plate(path: Path) -> None:
    """ASCII STL: 30 mm square plate, 1 mm thick, centred 10 mm square hole."""
    outer = [(0, 0), (30, 0), (30, 30), (0, 30)]
    hole = [(10, 10), (10, 20), (20, 20), (20, 10)]
    facets = []

    def quad(a, b, c, d):
        facets.append((a, b, c))
        facets.append((a, c, d))

    # Top and bottom faces as four trapezoids around the hole.
    for z, flip in ((1, False), (0, True)):
        for i in range(4):
            o0, o1 = outer[i], outer[(i + 1) % 4]
            h0, h1 = hole[(4 - i) % 4], hole[(3 - i) % 4]
            pts = [(*o0, z), (*o1, z), (*h1, z), (*h0, z)]
            quad(*(reversed(pts) if flip else pts))
    for ring in (outer, hole):
        for i in range(4):
            a, b = ring[i], ring[(i + 1) % 4]
            quad((*a, 0), (*b, 0), (*b, 1), (*a, 1))
    lines = ["solid plate"]
    for tri in facets:
        lines.append(" facet normal 0 0 0\n  outer loop")
        lines += [f"   vertex {x} {y} {z}" for x, y, z in tri]
        lines.append("  endloop\n endfacet")
    lines.append("endsolid plate")
    path.write_text("\n".join(lines) + "\n")


def moves(gcode: Path):
    """Yield (feedrate mm/min, length mm, E delta, has E) for G0/G1 moves after the first layer."""
    x = y = e = f = 0.0
    absolute_e = True
    layer = -1
    for raw in gcode.read_text(errors="replace").splitlines():
        if raw.startswith(";LAYER_CHANGE"):
            layer += 1
        line = raw.split(";", 1)[0].strip().upper()
        if line.startswith("M83"):
            absolute_e = False
        elif line.startswith("M82"):
            absolute_e = True
        elif line.startswith("G92"):
            fields = dict(WORD.findall(line))
            if "E" in fields:
                e = float(fields["E"])
        elif line.startswith(("G1 ", "G0 ")):
            fields = {k: float(v) for k, v in WORD.findall(line)}
            nx, ny = fields.get("X", x), fields.get("Y", y)
            f = fields.get("F", f)
            de = 0.0
            if "E" in fields:
                de = fields["E"] - e if absolute_e else fields["E"]
                e = fields["E"] if absolute_e else e + fields["E"]
            if layer >= 1:  # first-layer speed limits may change the bypass speed
                yield f, math.hypot(nx - x, ny - y), de, "E" in fields
            x, y = nx, ny


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=Path, required=True)
    parser.add_argument("--build", type=Path, default=ROOT / ".build")
    args = parser.parse_args()
    datadir = args.build.resolve() / "test-data" / "infill_aroundholes"
    out = args.build.resolve() / "test-output" / "infill_aroundholes"
    out.mkdir(parents=True, exist_ok=True)
    model = out / "plate.stl"
    write_plate(model)
    travel_speed = 150.0

    for case, ratio in (("flow1", "1%"), ("flow0", "0%")):
        profile = out / f"{case}.ini"
        profile.write_text("\n".join([
            "layer_height = 0.2", "first_layer_height = 0.2", "perimeters = 1",
            "top_solid_layers = 0", "bottom_solid_layers = 0", "fill_density = 30%",
            "fill_pattern = rectilineararoundholes", "skirts = 0", "brim_width = 0",
            # Keep ordinary extrusion speeds distinct from the 55.5 mm/s bypass,
            # and disable automatic cooling slowdown so F identifies it reliably.
            "infill_speed = 25", "perimeter_speed = 20", "external_perimeter_speed = 20", "cooling = 0",
            f"travel_speed = {travel_speed:g}", "fill_rectilinearholes_travel_speed = 37%",
            f"fill_rectilinearholes_travel_flow_ratio = {ratio}", "seam_visibility = 0",
        ]) + "\n")
        gcode = out / f"{case}.gcode"
        gcode.unlink(missing_ok=True)
        result = subprocess.run([str(args.host.resolve()), "--datadir", str(datadir), "--export-gcode",
                                 "--load", str(profile), "--output", str(gcode), str(model)],
                                capture_output=True, timeout=180, cwd=args.host.resolve().parent)
        log = (result.stdout + result.stderr).decode("utf-8", errors="replace")
        (out / f"{case}.log").write_text(log, encoding="utf-8")
        if result.returncode != 0 or not gcode.exists():
            raise AssertionError(f"{case}: export failed\n{log[-4000:]}")
        text = gcode.read_text(errors="replace")
        if "rectilineararoundholes" in log and "fallback" in log.lower():
            raise AssertionError(f"{case}: pattern not active\n{log[-4000:]}")
        bypass_feed = travel_speed * 0.37 * 60.0
        sequence = list(moves(gcode))
        bypass = [(length, de) for feed, length, de, has_e in sequence
                  if abs(feed - bypass_feed) < 1.0 and length > 0.0]
        # A bypass joins two lines without leaving the material: no retraction
        # right before or right after it.
        for i, (feed, length, de, has_e) in enumerate(sequence):
            if abs(feed - bypass_feed) < 1.0 and length > 0.0:
                for j in (i - 1, i + 1):
                    if 0 <= j < len(sequence):
                        assert sequence[j][2] >= 0.0, f"{case}: retraction next to a bypass move"
        assert bypass, f"{case}: no move at the bypass speed ({bypass_feed:g} mm/min)"
        assert all(de >= -1e-9 for _, de in bypass), f"{case}: bypass retracts while moving"
        printed = [(length, de) for feed, length, de, has_e in moves(gcode)
                   if has_e and de > 0 and length > 0 and abs(feed - bypass_feed) >= 1.0]
        assert printed, f"{case}: no ordinary extrusion moves"
        line_e_per_mm = sum(de for _, de in printed) / sum(length for length, _ in printed)
        bypass_e_per_mm = sum(de for _, de in bypass) / sum(length for length, _ in bypass)
        if ratio == "0%":
            assert all(abs(de) < 1e-9 for _, de in bypass), f"{case}: bypass extrudes at 0%"
        else:
            assert 0 < bypass_e_per_mm < 0.05 * line_e_per_mm, f"{case}: bypass E/mm {bypass_e_per_mm}"
        print(f"{case}: {len(bypass)} bypass moves, {bypass_e_per_mm:.6f} E/mm "
              f"(lines {line_e_per_mm:.5f}); {len(text)} bytes")
    print("infill_aroundholes_smoke: passed")


if __name__ == "__main__":
    main()
