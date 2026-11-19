"""Fast negative and extrusion-state checks of the native export oracle.

    unittest -> read_starts / series

Synthetic commands cover both E modes, resets, retractions, arcs, real copy
labels and missing outputs. The oracle must fail on a partial scene instead of
accepting whichever layers happened to survive an export.
"""
import unittest
from test_exports import read_starts, series


class Reader(unittest.TestCase):
    def test_extrusion_modes(self):
        """Retractions and pure-E recovery are not path starts; retain arc starts."""
        result = read_starts(['M82', ';LAYER_CHANGE', '; printing object a id:0 copy 0', ';TYPE:Perimeter',
                              'G0 X1 Y2', 'G92 E10', 'G1 E9', 'G1 E10', 'G1 X2 E11',
                              'G1 X3 E12', 'G1 E11', 'G0 X4 Y5', 'M83', 'G1 E1', 'G2 X5 E.5'])
        self.assertEqual(result[('a', 0, 0, 'Perimeter')], [(1, 2), (4, 5)])

    def test_copy_ids(self):
        """Do not merge two instances that share the same source object."""
        result = read_starts(['M83', ';LAYER_CHANGE', ';TYPE:Perimeter', '; printing object a id:0 copy 0',
                              'G0 X1 Y2', 'G1 X2 E1', '; printing object a id:0 copy 1', 'G0 X8 Y2', 'G1 X9 E1'])
        self.assertEqual(result[('a', 0, 0, 'Perimeter')], [(1, 2)])
        self.assertEqual(result[('a', 1, 0, 'Perimeter')], [(8, 2)])

    def test_incomplete_exports_fail(self):
        """Empty files, malformed labels and missing layers are visible failures."""
        with self.assertRaises(AssertionError):
            read_starts([])
        with self.assertRaises(AssertionError):
            read_starts(['; printing object a'])
        with self.assertRaises(AssertionError):
            series({('a', 0, 0, 'Perimeter'): [(1, 2)]}, 'a', 'Perimeter')
        with self.assertRaises(AssertionError):
            read_starts(['M83', ';LAYER_CHANGE', '; printing object a id:0 copy 0', ';TYPE:Perimeter', 'G1 X2 E1'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
