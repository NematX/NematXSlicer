"""Check the E2E gate's G-code reader and source-catalogue failure visibility.

These small examples distinguish resets, relative extrusion and travel moves so
the feature's export comparisons cannot pass on an empty or misread output.
    unittest -> read_moves / sparse_geometry / checked_domains
"""
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from test_support_interfaces import checked_domains, read_moves, sparse_geometry


class ReaderTests(unittest.TestCase):
    """Small explicit traces are independent of the slicer's exporter."""

    def test_absolute_reset_and_roles(self):
        """An E reset changes the origin, not the deposited amount or role."""
        moves = read_moves("M82\nG1 X1 Y1 Z0.2\n;TYPE:Support material\nG1 X3 E1\n"
                           "G92 E0\n;TYPE:Support material interface\nG1 Y3 E0.5\nG1 X4\n")
        self.assertEqual(moves, [("support material", 1., 1., 3., 1., 0.2, 1.),
                                 ("support material interface", 3., 1., 3., 3., 0.2, 0.5)])
        self.assertEqual(sparse_geometry(moves), [(0.2, 1, 0, 1_000_000, 1_000_000, 3_000_000, 1)])

    def test_collinear_splits_keep_coverage_and_multiplicity(self):
        """Travel phases may split a segment, but extra or missing deposition must still fail equality."""
        whole = read_moves("M83\n;TYPE:Support material\nG1 Z0.2\nG1 X4 E0.4\n")
        split = read_moves("M83\n;TYPE:Support material\nG1 Z0.2\nG1 X1 E0.1\nG1 X3 E0.2\nG1 X4 E0.1\n")
        self.assertEqual(sparse_geometry(whole), sparse_geometry(split))
        self.assertNotEqual(sparse_geometry(whole), sparse_geometry(split + split))
        self.assertNotEqual(sparse_geometry(whole), sparse_geometry(split[:-1]))

    def test_relative_retraction_and_travel(self):
        """Retraction and pure travel are not counted as deposited geometry."""
        moves = read_moves("M83\nG1 Z0.4\n;TYPE:Support material interface\nG1 X2 E0.2\n"
                           "G1 E-1\nG1 X4\nG1 Y2 E0.3\n")
        self.assertEqual([move[-1] for move in moves], [0.2, 0.3])
        self.assertEqual(len(moves), 2)

    def test_empty_output_fails(self):
        """A missing parser match cannot masquerade as equality of two exports."""
        with self.assertRaisesRegex(AssertionError, "no readable"):
            read_moves("G1 X1\nG1 E-1\n")

    def test_new_declared_setting_requires_domain(self):
        """A new C++ dependency without a variation domain must fail visibly."""
        with TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "src/libslic3r/Plugins/Support/Common/SupportSettings.cpp"
            source.parent.mkdir(parents=True)
            source.write_text('{"uncovered_option", RAW_CO_INT, RAW_CONTAINER_TYPE_OBJECT, RAW_PRESET_TYPE_FFF_PRINT}', encoding="utf-8")
            with self.assertRaisesRegex(AssertionError, "uncovered_option"):
                checked_domains(root)


if __name__ == "__main__":
    unittest.main()
