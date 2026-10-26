"""Focused real-host smoke test; builds nothing and uses only an isolated datadir."""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests' / 'second_layer_flow'))
from test_second_layer_flow import analyze_gcode

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=Path, required=True)
    parser.add_argument('--build', type=Path, default=ROOT / '.build')
    args = parser.parse_args()
    build = args.build.resolve()
    datadir = build / 'test-data' / 'corner_stretch'
    out = build / 'test-output' / 'corner_stretch'
    out.mkdir(parents=True, exist_ok=True)
    totals = {}
    lengths = {}
    for case, enabled, arcs, native in [('off', 0, 0, 0), ('lines', 1, 0, 0),
                                       ('arcs', 1, 1, 1), ('tessellated', 1, 1, 0)]:
        config = out / f'{case}.ini'
        config.write_text('\n'.join([
            'seam_visibility = 0', 'layer_height = 0.2', 'first_layer_height = 0.2',
            'cooling = 0', 'skirts = 0', 'brim_width = 0',
            'arc_fitting = emit_center' if native else 'arc_fitting = disabled',
            f'stretch_corners = {enabled}', f'stretch_corners_arc = {arcs}',
        ]) + '\n')
        output = out / f'{case}.gcode'
        result = subprocess.run([
            str(args.host.resolve()), '--datadir', str(datadir), '--export-gcode',
            '--load', str(config), '--output', str(output),
            str(ROOT / 'tests/second_layer_flow/fixtures/cube-5mm.stl'),
        ], capture_output=True, timeout=120, cwd=args.host.resolve().parent)
        log = (result.stdout + result.stderr).decode('utf-8', errors='replace')
        (out / f'{case}.log').write_text(log, encoding='utf-8')
        if result.returncode or not output.exists():
            raise AssertionError(f'{case} export failed: {log[-4000:]}')
        if 'disabled' in log.lower() and 'nematx_corner_stretch' in log:
            raise AssertionError(f'Check plugin activation: {log[-4000:]}')
        metrics = analyze_gcode(output)
        totals[case] = sum(layer.deposited_e for layer in metrics)
        lengths[case] = sum(layer.path_length for layer in metrics)
        assert totals[case] > 0, f'{case}: no extrusion'
    for case in ('lines', 'arcs', 'tessellated'):
        assert abs(totals[case] / totals['off'] - 1) < .002, totals
        assert lengths[case] > lengths['off'] + .1, f'{case}: no observable deformation: {lengths}'
    assert any(line.startswith(('G2 ', 'G3 ')) for line in (out/'arcs.gcode').read_text().splitlines()), 'No native arcs'
    assert not any(line.startswith(('G2 ', 'G3 ')) for line in (out/'tessellated.gcode').read_text().splitlines()), 'Arcs emitted while disabled'
    print('corner_stretch_export: passed', totals, lengths)

if __name__ == '__main__':
    main()
