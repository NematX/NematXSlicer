"""Bounded native-package exports and a strict oracle for perimeter starts.

Four real host exports compare a deterministic neutral scene with the host,
then exercise height-limited volumes, old names, true copies, rotated instances
and a linear matrix of all three declared cost settings in one small scene.
The oracle reads actual XY extrusion starts, not external-only seam comments.
Each run owns its data, profiles, G-code, logs and timing report.

    main -> export -> write_project -> bounded host process
                   -> read_starts -> per-layer internal/external assertions
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from xml.sax.saxutils import quoteattr
from zipfile import ZIP_DEFLATED, ZipFile

FIELDS = re.compile(r"([A-Z])([+-]?(?:\d+(?:\.\d*)?|\.\d+))")
LABEL = re.compile(r"; printing object (.*?) id:(\d+) copy (\d+)$")
TYPE = "SeamPositionInternalCenterZ"
# The host-side metadata check requires exactly these declared setting domains.
DOMAINS = {"seam_angle_cost": ("0", "60", "1000"), "seam_travel_cost": ("0", "100", "1000"),
           "seam_visibility": ("0", "1")}
BASE = {"layer_height": "0.3", "first_layer_height": "0.3", "perimeters": "2", "fill_density": "0%",
        "top_solid_layers": "0", "bottom_solid_layers": "0", "skirts": "0", "brim_width": "0",
        "seam_visibility": "0", "seam_angle_cost": "0", "seam_travel_cost": "0",
        "seam_gap": "0", "seam_gap_external": "0", "gcode_label_objects": "octoprint", "gcode_comments": "1",
        "gcode_firmware_plugin": "gcode.firmware.marlin2", "only_one_perimeter_first_layer": "0", "only_one_perimeter_top": "0"}


def read_starts(lines: list[str]) -> dict[tuple[str, int, int, str], list[tuple[float, float]]]:
    """Track real positive-E XY paths with reset/retraction and absolute/relative E."""
    result: dict[tuple[str, int, int, str], list[tuple[float, float]]] = {}
    x = y = e = 0.0
    known_xy = False
    absolute_e, printing = True, False
    layer, instance, role = -1, None, ""
    for raw in lines:
        if raw.startswith(";LAYER_CHANGE"):
            layer += 1
            printing = False
        elif match := LABEL.fullmatch(raw):
            instance = (match[1], int(match[3]))
            printing = False
        elif raw.startswith("; printing object "):
            raise AssertionError(f"Unreadable object label: {raw}")
        elif raw.startswith("; stop printing object"):
            instance = None
            printing = False
        elif raw.startswith(";TYPE:"):
            role = raw[len(";TYPE:"):].strip()
        code = raw.split(";", 1)[0].strip().upper()
        if not code:
            continue
        command = code.split(" ", 1)[0]
        fields = {key: float(value) for key, value in FIELDS.findall(code[len(command):])}
        if command in ("M82", "M83"):
            absolute_e = command == "M82"
        elif command == "G92":
            e = fields.get("E", e)
        elif command in ("G0", "G1", "G2", "G3"):
            xy = "X" in fields or "Y" in fields
            delta = (fields["E"] - e if absolute_e else fields["E"]) if "E" in fields else 0
            positive = command != "G0" and delta > 0 and xy
            if positive and not printing and instance and role in ("Perimeter", "External perimeter"):
                if not known_xy or layer < 0:
                    raise AssertionError("Perimeter extrusion lacks a known XY start/layer")
                name, copy = instance
                result.setdefault((name, copy, layer, role), []).append((x, y))
            if xy:
                x, y = fields.get("X", x), fields.get("Y", y)
                known_xy = True
            if "E" in fields:
                e = fields["E"] if absolute_e else e + fields["E"]
            printing = positive
    if not result:
        raise AssertionError("No perimeter starts read from export")
    return result


def series(starts, name, role, copy=0, layers=5):
    """Require precisely one real loop on every expected printed layer."""
    keys = sorted(key for key in starts if key[0:2] == (name, copy) and key[3] == role)
    assert [key[2] for key in keys] == list(range(layers)), (name, copy, role, keys)
    values = [starts[key] for key in keys]
    assert all(len(value) == 1 for value in values), (name, role, values)
    return [value[0] for value in values]


def write_project(path: Path, objects: list[dict]) -> None:
    """Write five-layer boxes plus seam-only meshes and true transformed build copies."""
    faces = ((3, 2, 1), (1, 0, 3), (4, 5, 6), (6, 7, 4), (0, 1, 5), (5, 4, 0),
             (1, 2, 6), (6, 5, 1), (2, 3, 7), (7, 6, 2), (3, 0, 4), (4, 7, 3))
    resources, configs, build = [], [], []
    for index, obj in enumerate(objects, 1):
        vertices, triangles, volumes = [], [], []
        boxes = [((0, 0, 0, 4, 4, obj.get('height', 1.5)), "ModelPart")]
        if obj.get("attractor", True):
            low, high = obj.get('zrange', (0.35, 1.05))
            boxes.append(((5, 1.5, low, 6, 2.5, high), obj.get("type", TYPE)))
        for box, kind in boxes:
            x0, y0, z0, x1, y1, z1 = box
            base, first = len(vertices), len(triangles)
            vertices += [(x, y, z) for z in (z0, z1) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
            triangles += [tuple(base + corner for corner in face) for face in faces]
            volumes.append(f'<volume firstid="{first}" lastid="{len(triangles)-1}">'
                           f'<metadata type="volume" key="volume_type" value="{kind}" /></volume>')
        resources.append(f'<object id="{index}" type="model"><mesh><vertices>' +
                         "".join(f'<vertex x="{x}" y="{y}" z="{z}" />' for x, y, z in vertices) +
                         '</vertices><triangles>' +
                         "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}" />' for a, b, c in triangles) +
                         '</triangles></mesh></object>')
        configs.append(f'<object id="{index}"><metadata type="object" key="name" value={quoteattr(obj["name"])} />' +
                       "".join(f'<metadata type="object" key={quoteattr(key)} value={quoteattr(value)} />'
                               for key, value in obj.get("config", {}).items()) + "".join(volumes) + '</object>')
        for offset in obj.get("copies", (0,)):
            matrix = "0 1 0 -1 0 0 0 0 1" if obj.get("rotate") else "1 0 0 0 1 0 0 0 1"
            build.append(f'<item objectid="{index}" transform="{matrix} {obj["x"] + offset} 10 0" />')
    model = ('<?xml version="1.0"?><model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
             '<metadata name="slic3rpe:Version3mf">1</metadata><resources>' + "".join(resources) +
             '</resources><build>' + "".join(build) + '</build></model>')
    with ZipFile(path, "w", ZIP_DEFLATED) as archive:
        archive.writestr('[Content_Types].xml', '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
                         '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml" />'
                         '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml" /></Types>')
        archive.writestr('_rels/.rels', '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
                         '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
                         'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" /></Relationships>')
        archive.writestr('3D/3dmodel.model', model)
        archive.writestr('Metadata/SuperSlicer_model.config', '<config>' + "".join(configs) + '</config>')


def write_cube_stl(path: Path) -> None:
    """Feed a real binary STL through the unmodified neutral slicing pipeline."""
    vertices = [(x, y, z) for z in (0, 1.5) for x, y in ((0, 0), (4, 0), (4, 4), (0, 4))]
    faces = ((3, 2, 1), (1, 0, 3), (4, 5, 6), (6, 7, 4), (0, 1, 5), (5, 4, 0),
             (1, 2, 6), (6, 5, 1), (2, 3, 7), (7, 6, 2), (3, 0, 4), (4, 7, 3))
    content = b'NematX seam neutral fixture'.ljust(80, b'\0') + struct.pack('<I', len(faces))
    for face in faces:
        values = [0., 0., 0.] + [coordinate for corner in face for coordinate in vertices[corner]]
        content += struct.pack('<12fH', *values, 0)
    path.write_bytes(content)


def main() -> int:
    """Run the complete export matrix under one deadline; keep all diagnostics on failure."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host-executable', type=Path, required=True)
    parser.add_argument('--build-dir', type=Path, required=True)
    parser.add_argument('--runtime-path', action='append', default=[])
    parser.add_argument('--builtin', action='store_true')
    parser.add_argument('--budget-seconds', type=float, default=45)
    args = parser.parse_args()
    started = time.monotonic()
    results = args.build_dir / 'test-results/seam'
    results.mkdir(parents=True, exist_ok=True)
    root = Path(tempfile.mkdtemp(prefix='run-', dir=results))
    data = root / 'datadir'
    data.mkdir()
    if not args.builtin:
        staged = args.build_dir / 'test-data/seam/plugins'
        library = 'plugin.dll' if os.name == 'nt' else 'plugin.dylib' if sys.platform == 'darwin' else 'plugin.so'
        assert (staged / 'nematx-seam' / library).is_file(), 'Missing standalone library; do not qualify the embedded fallback by accident'
        assert (staged / 'nematx-seam/version.ini').is_file(), 'Missing standalone ABI manifest'
        manifest = (staged / 'nematx-seam/version.ini').read_text(encoding='utf-8')
        required_volume = re.search(r'^slic3r_volume\.h\s*=\s*(\d+)\.(\d+)\s*$', manifest, re.MULTILINE)
        assert required_volume and int(required_volume[1]) == 2 and int(required_volume[2]) >= 1, 'Manifest omits the Volume 2.1 contract used by the decorator'
        shutil.copytree(staged, data / 'plugins')
    env = dict(os.environ)
    key = 'PATH' if os.name == 'nt' else 'DYLD_LIBRARY_PATH' if sys.platform == 'darwin' else 'LD_LIBRARY_PATH'
    env[key] = os.pathsep.join([str(args.host_executable.parent), *args.runtime_path, env.get(key, '')])
    calls = []
    success = False
    print(f'Internal seam export artifacts: {root}', flush=True)

    def export(name, objects, selector=None, save=False):
        """Export a fresh scene, kill descendants on timeout, then parse strict output."""
        case = root / name
        case.mkdir()
        settings = dict(BASE)
        if selector:
            settings[selector[0]] = selector[1]
        (case / 'profile.ini').write_text(''.join(f'{k} = {v}\n' for k, v in settings.items()), encoding='utf-8')
        model = case / 'model.3mf'
        if len(objects) == 1 and not objects[0].get('attractor', True):
            model = case / 'neutral.stl'
            write_cube_stl(model)
        else:
            write_project(model, objects)
        actions = ['--save', str(case / 'saved.ini')] if save else []
        # The Windows console wrapper also probes a windowed hardware OpenGL
        # context. Exports need no graphics context; bypass that unrelated probe.
        renderer = ['--sw-renderer'] if os.name == 'nt' else []
        command = [str(args.host_executable), *renderer, '--threads', '1', '--dont-arrange', '--datadir', str(data), *actions,
                   '--export-gcode', '--load', str(case / 'profile.ini'), '--output', str(case / 'result.gcode'), str(model)]
        remaining = args.budget_seconds - (time.monotonic() - started)
        assert remaining > 0, 'Global internal seam export budget exhausted'
        start = time.monotonic()
        with (case / 'host.log').open('wb') as log:
            options = {'creationflags': subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == 'nt' else {'start_new_session': True}
            process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, **options)
            try:
                process.wait(timeout=min(18 if len(objects) > 1 else 12, remaining))
            except BaseException:
                if os.name == 'nt':
                    subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'], stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL, timeout=3, check=False)
                else:
                    os.killpg(process.pid, signal.SIGKILL)
                if process.poll() is None:
                    process.kill()
                process.wait(timeout=3)
                raise
            finally:
                calls.append({'case': name, 'seconds': time.monotonic()-start, 'exit': process.returncode})
        assert process.returncode == 0, f'Host failed: {case / "host.log"}'
        log = (case / 'host.log').read_text(encoding='utf-8', errors='replace').lower()
        checked = '\n'.join(line for line in log.splitlines()
                            if "exclusive group 'gcode_firmware_plugin' has incompatible execution priorities" not in line)
        assert not any(word in checked for word in ('[error]', 'will be disabled', 'unknown option')), case / 'host.log'
        lines = (case / 'result.gcode').read_text(encoding='utf-8').splitlines()
        return lines, read_starts(lines)

    try:
        neutral = [{'name': 'neutral', 'x': 10, 'attractor': False}]
        plugin_lines, plugin = export('neutral-plugin', neutral, save=True)
        saved = dict(line.split(' = ', 1) for line in (root / 'neutral-plugin/saved.ini').read_text(encoding='utf-8').splitlines() if ' = ' in line)
        selector = next(key for key in saved if key.startswith('exclusive_group_') and 'seam_placer_plugin' in key)
        assert saved[selector] == 'nematx_seam', 'NematX seam provider is not preferred'
        assert all(key in saved for key in DOMAINS), 'A declared seam cost setting is absent from the catalogue'
        host_lines, host = export('neutral-host', neutral, (selector, 'seam_placer.cost'))
        commands = lambda lines: [line.split(';', 1)[0].strip() for line in lines if line.split(';', 1)[0].strip()]
        assert commands(plugin_lines) == commands(host_lines), 'No-attractor full-command host parity failed'
        series(plugin, 'neutral_stl', 'Perimeter')

        scene = [{'name': 'canonical', 'x': 10}, {'name': 'legacy', 'x': 20, 'type': 'SeamPositionInsideCenter'},
                 {'name': 'copies', 'x': 30, 'copies': (0, 8)}, {'name': 'rotated', 'x': 50, 'rotate': True}]
        for key, values in DOMAINS.items():
            for value in values[1:]:
                obj = {'name': f'{key}_{value}', 'x': 60 + 8 * (len(scene)-4), 'config': {key: value}}
                if key == 'seam_visibility':
                    # The host traces its full 30,000-sample visibility model per
                    # external loop. One actual layer covers enabled visibility;
                    # repeating that same setup on five layers adds no attractor rule.
                    obj.update(height=0.3, zrange=(0, 0.5), layers=1)
                scene.append(obj)
        _, active = export('active-plugin', scene)
        _, base = export('active-host', scene, (selector, 'seam_placer.cost'))
        for obj in scene:
            name = obj['name']
            for copy, offset in enumerate(obj.get('copies', (0,))):
                layers = obj.get('layers', 5)
                internal = series(active, name, 'Perimeter', copy, layers)
                baseline = series(base, name, 'Perimeter', copy, layers)
                active_layers = (0,) if layers == 1 else (1, 2)
                for layer, ((x, y), original) in enumerate(zip(internal, baseline)):
                    if layer in active_layers:
                        if obj.get('rotate'):
                            assert abs(x - (obj['x']-2)) < 0.02 and y > 13, (name, layer, x, y)
                        else:
                            assert x > obj['x'] + offset + 3 and abs(y-12) < 0.02, (name, layer, x, y)
                    elif obj.get('config', {}).get('seam_travel_cost', '0') == '0':
                        assert (x, y) == original, (name, 'inactive height changed host seam', layer, (x, y), original)
                assert any(internal[layer] != baseline[layer] for layer in active_layers), (name, 'attractor had no effect')
                # Zero travel weight makes the external host decision independent
                # of the changed approach from the preceding internal loop.
                if obj.get('config', {}).get('seam_travel_cost', '0') == '0':
                    assert series(active, name, 'External perimeter', copy, layers) == series(base, name, 'External perimeter', copy, layers), name
        assert time.monotonic() - started < args.budget_seconds, 'Global export budget exceeded'
        success = True
        return 0
    finally:
        report = {'success': success, 'seconds': time.monotonic()-started, 'calls': calls, 'builtin': args.builtin,
                  'domains': DOMAINS, 'budget_seconds': args.budget_seconds}
        (root / 'timings.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(f'Internal seam exports: {report["seconds"]:.3f}s; calls: {len(calls)}; success: {success}', flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
