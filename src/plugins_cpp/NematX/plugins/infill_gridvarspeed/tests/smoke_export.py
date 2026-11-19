"""Targeted real-host exports; builds nothing and writes only inside --build.

Use the matching staged package and provide runtime DLL directories on PATH.
Checks sparse/top gridvarspeed, flow endpoints, speed caps, Z sign and no retractions
between contiguous modulated infill fragments. Not a machine-collision proof.
"""
import argparse
import math
from pathlib import Path
import re
import statistics
import subprocess

ROOT = Path(__file__).resolve().parents[3]
WORD = re.compile(r"([A-Z])([+-]?(?:\d+(?:\.\d*)?|\.\d+))")


def configuration_error(log):
    return bool(re.search(r"unknown option|will be disabled|\[error\]|\bplugin\b[^\r\n]*\bfallback\b|\bfallback\b[^\r\n]*\bplugin\b",
                          log, re.IGNORECASE))


def write_box(path):
    vertices = [(x, y, z) for z in (0, 1) for y in (0, 30) for x in (0, 30)]
    faces = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4),
             (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    text = ["solid box"]
    for a, b, c, d in faces:
        for tri in ((a, b, c), (a, c, d)):
            text.append(" facet normal 0 0 0\n  outer loop")
            text.extend("   vertex %g %g %g" % vertices[i] for i in tri)
            text.append("  endloop\n endfacet")
    path.write_text("\n".join(text + ["endsolid box"]) + "\n")


def moves(path):
    x = y = z = e = f = 0.
    absolute_e = True
    absolute_xyz = True
    role = ""
    layer = -1
    for raw in path.read_text(errors="replace").splitlines():
        if raw.startswith(";LAYER_CHANGE"):
            layer += 1
        if raw.startswith(";TYPE:"):
            role = raw[6:].strip().lower()
        code = raw.split(";", 1)[0].strip().upper()
        fields = {k: float(v) for k, v in WORD.findall(code)}
        op = code.split(" ", 1)[0]
        if op == "M83":
            absolute_e = False
        elif op == "M82":
            absolute_e = True
        elif op == "G90":
            absolute_xyz = True
        elif op == "G91":
            absolute_xyz = False
        elif op == "G92":
            x, y, z, e = (fields.get("X", x), fields.get("Y", y), fields.get("Z", z), fields.get("E", e))
        elif op in ("G0", "G1"):
            nx, ny, nz = [(fields.get(k, old) if absolute_xyz else old + fields.get(k, 0.))
                          for k, old in (("X", x), ("Y", y), ("Z", z))]
            de = fields.get("E", e) - e if absolute_e else fields.get("E", 0.)
            if "E" in fields:
                e = fields["E"] if absolute_e else e + fields["E"]
            f = fields.get("F", f)
            yield dict(layer=layer, role=role, speed=f / 60., length=math.hypot(nx-x, ny-y),
                       de=de, start=(x, y, z), end=(nx, ny, nz))
            x, y, z = nx, ny, nz


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=Path, required=True)
    parser.add_argument("--build", type=Path, default=ROOT / ".build")
    parser.add_argument("--case", help="Run only one named case")
    parser.add_argument("--check-existing", action="store_true", help="Only validate previously exported files")
    args = parser.parse_args()
    build = args.build.resolve()
    out = build / "test-output/infill_gridvarspeed"
    out.mkdir(parents=True, exist_ok=True)
    model = out / "box.stl"
    write_box(model)
    # name, top only, ratio%, dz, infill_first, maximum mm/s, role speed
    cases = [("sparse50", False, 50, "0", False, 200, 30),
             ("sparse0", False, 0, "0", False, 200, 30),
             ("sparse100", False, 100, "0", False, 200, 30),
             ("speedcap", False, 50, "0", False, 45, 30),
             ("maxspeed", False, 50, "0", False, 80, 0),
             ("autospeed", False, 50, "0", False, 200, 0),
             ("zclamp", False, 50, "0.3", False, 200, 30),
             ("zup", False, 50, "25%", False, 200, 30),
             ("zdown", False, 50, "0.05", True, 200, 30),
             ("top50", True, 50, "0", False, 200, 30)]
    if args.case and args.case not in {case[0] for case in cases}:
        parser.error(f"Unknown case: {args.case}")
    for name, top, ratio, dz, first, maximum, speed in cases:
        if args.case and args.case != name:
            continue
        profile = out / (name + ".ini")
        settings = ["layer_height = 0.2", "first_layer_height = 0.2", "perimeters = 1",
                    "top_solid_layers = " + ("1" if top else "0"), "bottom_solid_layers = 0",
                    "fill_density = 20%", "fill_pattern = " + ("rectilinear" if top else "gridvarspeed"),
                    "top_fill_pattern = " + ("gridvarspeed" if top else "rectilinear"),
                    "skirts = 0", "brim_width = 0", "cooling = 0",
                    f"infill_speed = {speed}", f"top_solid_infill_speed = {speed}",
                    "perimeter_speed = 20", "external_perimeter_speed = 20", "travel_speed = 150",
                    f"max_print_speed = {maximum}", "max_volumetric_speed = 0",
                    "filament_max_speed = 0", "filament_max_volumetric_speed = 0",
                    "first_layer_speed = 20", "first_layer_infill_speed = 20", "first_layer_min_speed = 0",
                    "seam_visibility = 0", "gcode_comments = 1", "gcode_label_objects = 0",
                    "infill_first = " + ("1" if first else "0"), "retract_lift = 0.2",
                     f"fill_gridvarspeed_width_diff = {ratio}%", f"fill_gridvarspeed_zhop = {dz}"]
        if name == "autospeed":
            settings += ["max_volumetric_speed = 20", "autospeed_min_thin_flow = 1",
                         "solid_infill_speed = 0",
                         "filament_diameter = 1.75", "extrusion_multiplier = 1", "print_extrusion_multiplier = 100%"]
        # Profiles reject duplicate keys; case overrides replace defaults.
        settings = dict(line.split(" = ", 1) for line in settings)
        profile.write_text("\n".join(f"{key} = {value}" for key, value in settings.items()) + "\n")
        gcode = out / (name + ".gcode")
        if not args.check_existing:
            gcode.unlink(missing_ok=True)
            result = subprocess.run([str(args.host.resolve()), "--datadir", str(build / "test-data/infill_gridvarspeed"),
                                      "--loglevel", "2", "--export-gcode", "--load", str(profile), "--output", str(gcode), str(model)],
                                    cwd=out, capture_output=True, timeout=240)
            log = (result.stdout + result.stderr).decode("utf-8", errors="replace")
            (out / (name + ".log")).write_text(log, encoding="utf-8")
            if result.returncode or not gcode.exists():
                raise AssertionError(f"{name}: export failed ({result.returncode})\n{log[-6000:]}")
        else:
            log = (out / (name + ".log")).read_text(encoding="utf-8")
        if configuration_error(log):
            raise AssertionError(f"{name}: plugin or configuration error\n{log[:3000]}\n{log[-3000:]}")
        seq = list(moves(gcode))
        printed = [m for m in seq if m["layer"] >= 1 and "infill" in m["role"] and
                   ("top" in m["role"] if top else m["role"] == "internal infill") and
                   m["de"] > 0 and m["length"] > 0.001]
        assert printed, f"{name}: no identified gridvarspeed extrusion"
        feeds = {round(m["speed"], 2) for m in printed}
        if name == "autospeed":
            # Prescribed volumetric target = 1 mm3/s, below every physical cap (200 mm/s print
            # and 20 mm3/s volumetric). q is the nominal flow per mm of path in mm3/mm
            # (`nominal_flow` below: E/mm times the filament cross-section). Autospeed resolves
            # the baseline from the reduced flow q*f as target/(q*f), then the plugin applies
            # 1/f again: expected = target/(q*f^2), i.e. 4x the nominal speed at f=0.5 and
            # 16/9 (about 1.8x) at f=0.75, all under the caps.
            # Infer the flow factors from E/mm, independently of feedrates.
            samples = [m for m in printed if m["length"] > 0.1]
            emax = max(m["de"] / m["length"] for m in samples)
            buckets = {factor: [] for factor in (1., 0.75, 0.5)}
            for m in samples:
                factor = min(buckets, key=lambda v: abs(m["de"] / m["length"] / emax - v))
                buckets[factor].append(m)
            nominal_e = statistics.median(m["de"] / m["length"] for m in buckets[1.])
            nominal_flow = nominal_e * math.pi * (1.75 / 2.)**2
            for factor, samples in buckets.items():
                assert samples, f"{name}: missing factor {factor}"
                expected = 1. / (nominal_flow * factor**2)
                actual = statistics.median(m["speed"] for m in samples)
                assert expected < maximum and abs(actual / expected - 1.) < 0.03, (factor, actual, expected)
        assert max(feeds) <= maximum + 0.02, f"{name}: exceeds speed cap {feeds}"
        if ratio in (0, 100):
            assert feeds == {30.}, f"{name}: endpoints must be uniform {feeds}"
        elif speed:
            assert any(abs(v - min(60, maximum)) < 0.02 for v in feeds), f"{name}: no reduced-flow speed {feeds}"
            assert 30. in feeds and 40. in feeds, f"{name}: missing normal/transition speeds {feeds}"
            by_speed = {}
            for m in printed:
                if m["length"] > 0.05:
                    by_speed.setdefault(round(m["speed"], 2), []).append(m["de"] / m["length"])
            nominal = statistics.median(by_speed[30.])
            for feed, factor in ((40., 0.75), (float(min(60, maximum)), 0.5)):
                actual = statistics.median(by_speed[feed]) / nominal
                assert abs(actual - factor) < 0.025, f"{name}: wrong E/mm factor at {feed}: {actual}"
        # A retract/unretract pair must not be introduced BETWEEN contiguous
        # modulated extrusion moves. XY/Z travels and separate chains are allowed.
        previous = None
        retracted = False
        moved = False
        for m in seq:
            if m["de"] < -1e-6:
                retracted = True
            if m["length"] > 1e-6 or abs(m["end"][2] - m["start"][2]) > 1e-6:
                if m["de"] <= 0:
                    moved = True
            if m["de"] > 0 and m["length"] > 0.001:
                if previous and not moved and retracted and "infill" in m["role"] and "infill" in previous["role"]:
                    contiguous = all(abs(a-b) < 1e-5 for a, b in zip(previous["end"], m["start"]))
                    modulated = any(abs(v-40.) < 0.02 or abs(v-min(60, maximum)) < 0.02
                                    for v in (previous["speed"], m["speed"]))
                    assert not (contiguous and modulated), f"{name}: retraction inside contiguous modulated path"
                previous, retracted, moved = m, False, False
        if name in ("zup", "zdown", "zclamp"):
            offset = 0.1 if name == "zclamp" else 0.05 if name == "zup" else -0.05
            if name == "zclamp":
                assert log.count("GridVarSpeed: requested Z offset") == 1 and "limited to" in log, "missing or repeated clamp warning"
            zmods = {round(m["end"][2] - 0.2 * (m["layer"]+1), 3) for m in printed}
            assert 0. in zmods and offset in zmods, f"{name}: wrong pass Z offsets {zmods}"
            for layer in sorted({m["layer"] for m in printed}):
                ordered = []
                for m in printed:
                    if m["layer"] != layer:
                        continue
                    assert abs(m["start"][2] - m["end"][2]) < 1e-6, f"{name}: diagonal extrusion between Z levels"
                    level = round(m["end"][2] - 0.2 * (layer+1), 3)
                    if not ordered or ordered[-1] != level:
                        ordered.append(level)
                expected = [offset, 0.] if name == "zdown" else [0., offset]
                assert ordered == expected, f"{name}: passes reordered in layer {layer}: {ordered}"
        print(f"{name}: {len(printed)} infill moves, speeds {sorted(feeds)}")
    print("gridvarspeed export smoke passed")


if __name__ == "__main__":
    main()
