"""Host-independent checks of the real-export fixture and XYZE/F parser."""
from collections import Counter
from pathlib import Path
import tempfile
import unittest

from smoke_export import configuration_error, moves, write_box


class SmokeParserTests(unittest.TestCase):
    def test_configuration_errors_not_output_paths(self):
        self.assertFalse(configuration_error("Slicing result exported to C:/plugins/autospeed_fallback.gcode"))
        for message in ("Plugin gridvarspeed will be disabled", "Unknown option: x",
                        "[error] invalid recipe", "Plugin selection fallback to rectilinear"):
            self.assertTrue(configuration_error(message), message)

    def test_closed_oriented_box(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "box.stl"
            write_box(path)
            vertices = [tuple(map(float, line.split()[1:])) for line in path.read_text().splitlines()
                        if line.strip().startswith("vertex ")]
        edges = Counter()
        volume6 = 0.
        for i in range(0, len(vertices), 3):
            a, b, c = vertices[i:i+3]
            edges.update(((a, b), (b, c), (c, a)))
            volume6 += (a[0]*(b[1]*c[2]-b[2]*c[1]) + a[1]*(b[2]*c[0]-b[0]*c[2])
                        + a[2]*(b[0]*c[1]-b[1]*c[0]))
        self.assertEqual(len(vertices), 36)
        self.assertAlmostEqual(volume6 / 6., 900.)
        self.assertTrue(all(count == 1 and edges[(b, a)] == 1 for (a, b), count in edges.items()))

    def test_absolute_relative_extrusion_and_xyz(self):
        text = """G90
M82
G92 X0 Y0 Z0 E0
;LAYER_CHANGE
;TYPE:Internal infill
G1 X3 Y4 Z0.2 E1 F1800
G1 X6 Y8 E2 F2400
G92 E0
M83
G1 X9 Y12 E0.5
G1 E-0.25
G91
G1 X3 Y4 Z0.05 E0.5 F3600
G90
M82
G92 E4
G1 X15 Y20 E5
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.gcode"
            path.write_text(text)
            parsed = list(moves(path))
        self.assertEqual([m["de"] for m in parsed], [1., 1., 0.5, -0.25, 0.5, 1.])
        self.assertEqual([m["speed"] for m in parsed], [30., 40., 40., 40., 60., 60.])
        self.assertEqual([m["length"] for m in parsed], [5., 5., 5., 0., 5., 5.])
        self.assertEqual(parsed[-1]["end"], (15., 20., 0.25))
        self.assertTrue(all(m["role"] == "internal infill" and m["layer"] == 0 for m in parsed))


if __name__ == "__main__":
    unittest.main()
