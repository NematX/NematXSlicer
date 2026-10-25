"""Ownership and dispatch tests for Python geometry and Clipper helpers."""

import copy
import ctypes
import gc
import importlib
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
RUNTIME = ROOT / "src/libslic3r/Api/plugin/python"
sys.dont_write_bytecode = True
sys.path.insert(0, str(RUNTIME))


def load_python_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def address(handle):
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    return int(handle or 0)


class GeometryClipperHost:
    """Small host that records calls and ownership releases."""

    def __init__(self):
        self.next_handle = 100
        self.calls = []
        self.freed = []
        self.fail_next = False
        self.polyline_sizes = {21: 2, 32: 2}
        self.polyline_collection_sizes = {51: 3, 52: 0}

    def _result(self, name, *arguments):
        self.calls.append((name, tuple(address(arg) for arg in arguments)))
        if self.fail_next:
            self.fail_next = False
            return 0
        self.next_handle += 1
        return self.next_handle

    def polygon_convex_hull(self, storage, polygon):
        return self._result("convex_hull", storage, polygon)

    def expolygons_to_polylines(self, storage, src):
        return self._result("expolygons_to_polylines", storage, src)

    def clipper_diff_polyline_expolygons(self, storage, subject, clip):
        return self._result("polyline_diff", storage, subject, clip)

    def clipper_intersection_polyline_expolygons(self, storage, subject, clip):
        return self._result("polyline_intersection", storage, subject, clip)

    def clipper_split(self, storage, subject, clip, inside, outside):
        self.calls.append(("split", tuple(address(arg) for arg in (storage, subject, clip))))
        inside._obj.value = self.next_handle + 1
        outside._obj.value = self.next_handle + 2
        self.next_handle += 2
        return 1

    def clipper_split_polyline_expolygons(self, storage, subject, clip, inside, outside):
        self.calls.append(("polyline_split", tuple(address(arg) for arg in (storage, subject, clip))))
        if self.fail_next:
            self.fail_next = False
            return 0
        inside._obj.value = self.next_handle + 1
        outside._obj.value = self.next_handle + 2
        self.next_handle += 2
        return 1

    def clipper_diff_polylines_expolygons(self, storage, subject, clip):
        return self._result("polylines_diff", storage, subject, clip)

    def clipper_intersection_polylines_expolygons(self, storage, subject, clip):
        return self._result("polylines_intersection", storage, subject, clip)

    def clipper_clip_expolygons_with_subject_bbox(self, storage, src, bbox):
        self.calls.append(("clip_expolygons_bbox", (address(storage), address(src)), bbox))
        if self.fail_next:
            self.fail_next = False
            return 0
        self.next_handle += 1
        return self.next_handle

    def polyline_as_multipoint_const(self, polyline):
        return address(polyline)

    def multipoint_size(self, multipoint):
        return self.polyline_sizes.get(address(multipoint), 0)

    def storage_new_polylines(self, storage):
        self.next_handle += 1
        self.polyline_collection_sizes[self.next_handle] = 0
        return self.next_handle

    def polylines_size(self, polylines):
        return self.polyline_collection_sizes[address(polylines)]

    def storage_free(self, storage, handle):
        self.freed.append((address(storage), address(handle)))
        return 1


class PythonGeometryClipperViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "geometry_clipper_test_api_generator",
            RUNTIME / "generate_slic3r_api.py",
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.geometry = importlib.import_module("slic3r_geometry_views")
        cls.clipper = importlib.import_module("slic3r_clipper_views")
        cls.api_module = importlib.import_module("slic3r_api")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def make_api(self):
        api = self.api_module.Slic3rAPI.__new__(self.api_module.Slic3rAPI)
        api.host = GeometryClipperHost()
        return api

    def test_geometry_conversions_return_storage_owned_results(self):
        api = self.make_api()
        polygon = self.geometry.Polygon(api, 11)
        expolygons = self.geometry.ExPolygonCollection(api, 12)

        hull = api.convex_hull(7, polygon)
        polylines = api.expolygons_to_polylines(7, expolygons)

        self.assertIsInstance(hull, self.geometry.StoredPolygon)
        self.assertIsInstance(polylines, self.geometry.StoredPolylineCollection)
        self.assertEqual(hull.storage(), 7)
        self.assertEqual(polylines.storage(), 7)
        self.assertEqual(polygon.handle(), 11)
        self.assertEqual(expolygons.handle(), 12)
        hull_handle = hull.handle()
        polylines_handle = polylines.handle()
        self.assertTrue(hull.free_from_storage())
        self.assertTrue(polylines.free_from_storage())
        self.assertFalse(hull.free_from_storage())
        self.assertEqual(api.host.freed, [(7, hull_handle), (7, polylines_handle)])

    def test_storage_owned_geometry_rejects_shallow_and_deep_copy(self):
        api = self.make_api()
        owner_types = (
            self.geometry.StoredPolygon,
            self.geometry.StoredPolyline,
            self.geometry.StoredPolygonCollection,
            self.geometry.StoredPolylineCollection,
            self.geometry.StoredExPolygon,
            self.geometry.StoredExPolygonCollection,
        )
        owners = [
            owner_type.adopt_owned(api, 7, 300 + idx)
            for idx, owner_type in enumerate(owner_types)
        ]

        for owner in owners:
            with self.subTest(owner=type(owner).__name__):
                for copier in (copy.copy, copy.deepcopy):
                    with self.assertRaisesRegex(
                        TypeError, rf"{type(owner).__name__}.*unique native resource"
                    ):
                        copier(owner)
                gc.collect()
                self.assertTrue(owner)
                self.assertNotIn((7, owner.handle()), api.host.freed)

        handles = [owner.handle() for owner in owners]
        for owner in owners:
            self.assertTrue(owner.free_from_storage())
        self.assertEqual(api.host.freed, [(7, handle) for handle in handles])
        with self.assertRaisesRegex(TypeError, "StoredPolygon.*unique native resource"):
            copy.copy(owners[0])

    def test_borrowed_geometry_and_python_distancer_remain_copyable(self):
        api = self.make_api()
        polygon = self.geometry.Polygon(api, 501)
        polygon_copy = copy.copy(polygon)
        distancer = self.geometry.LineDistancer()
        distancer_copy = copy.deepcopy(distancer)

        self.assertIsNot(polygon_copy, polygon)
        self.assertEqual(polygon_copy.handle(), polygon.handle())
        self.assertIsNot(distancer_copy, distancer)
        self.assertTrue(distancer_copy.empty())
        self.assertEqual(api.host.freed, [])

    def test_clipper_operand_has_unique_ownership(self):
        api = self.make_api()
        operand = self.clipper.ClipperOperand.adopt_owned(api, 8, 601)

        for copier in (copy.copy, copy.deepcopy):
            with self.assertRaisesRegex(TypeError, "ClipperOperand.*unique native resource"):
                copier(operand)
        self.assertEqual(api.host.freed, [])
        self.assertTrue(operand)
        self.assertTrue(operand.free_from_storage())
        self.assertEqual(api.host.freed, [(8, 601)])

    def test_polyline_clipping_and_bbox_helpers_own_their_results(self):
        api = self.make_api()
        subject = self.geometry.Polyline(api, 21)
        clip = self.geometry.ExPolygonCollection(api, 22)
        bbox = self.generated.CBoundingBox()
        context = api.clipper(9)

        outside = context.diff_polyline_expolygons(subject, clip)
        inside = context.intersection_polyline_expolygons(subject, clip)
        bounded = context.clip_expolygons_with_subject_bbox(clip, bbox)

        self.assertIsInstance(outside, self.geometry.StoredPolylineCollection)
        self.assertIsInstance(inside, self.geometry.StoredPolylineCollection)
        self.assertIsInstance(bounded, self.geometry.StoredExPolygonCollection)
        self.assertEqual([call[0] for call in api.host.calls], [
            "polyline_diff",
            "polyline_intersection",
            "clip_expolygons_bbox",
        ])
        self.assertIs(api.host.calls[-1][2], bbox)
        for result in (outside, inside, bounded):
            self.assertEqual(result.storage(), 9)
            self.assertTrue(result.free_from_storage())

    def test_polyline_split_returns_two_storage_owned_results(self):
        api = self.make_api()
        subject = self.geometry.Polyline(api, 21)
        clip = self.geometry.ExPolygonCollection(api, 22)

        inside, outside = api.clipper(9).split_polyline_expolygons(subject, clip)

        self.assertEqual(api.host.calls, [("polyline_split", (9, 21, 22))])
        for result in (inside, outside):
            self.assertIsInstance(result, self.geometry.StoredPolylineCollection)
            self.assertEqual(result.storage(), 9)
        self.assertNotEqual(inside.handle(), outside.handle())
        handles = [inside.handle(), outside.handle()]
        self.assertTrue(inside.free_from_storage())
        self.assertTrue(outside.free_from_storage())
        self.assertEqual(api.host.freed, [(9, handle) for handle in handles])

        api.host.fail_next = True
        with self.assertRaises(ValueError):
            api.clipper_split_polyline_expolygons(9, subject, clip)

    def test_operand_split_returns_two_owned_operands(self):
        api = self.make_api()
        subject = self.clipper.ClipperOperand.adopt_owned(api, 9, 41)
        clip = self.clipper.ClipperOperand.adopt_owned(api, 9, 42)

        inside, outside = api.clipper(9).split(subject, clip)

        self.assertEqual(api.host.calls, [("split", (9, 41, 42))])
        for result in (inside, outside):
            self.assertIsInstance(result, self.clipper.ClipperOperand)
            self.assertEqual(result.storage(), 9)
        self.assertNotEqual(inside.handle(), outside.handle())

    def test_helpers_reject_invalid_storage_api_and_native_results(self):
        api = self.make_api()
        other_api = self.make_api()
        polygon = self.geometry.Polygon(api, 31)
        subject = self.geometry.Polyline(api, 32)
        foreign_clip = self.geometry.ExPolygonCollection(other_api, 33)

        with self.assertRaises(ValueError):
            self.geometry.convex_hull(0, polygon)
        with self.assertRaises(ValueError):
            self.clipper.clipper_diff_polyline_expolygons(
                5, subject, foreign_clip
            )
        with self.assertRaises(ValueError):
            api.clipper(5).diff_polyline_expolygons(subject, foreign_clip)
        with self.assertRaises(ValueError):
            api.convex_hull(5, self.geometry.Polygon(other_api, 34))

        api.host.fail_next = True
        with self.assertRaises(ValueError):
            self.geometry.convex_hull(5, polygon)

    def test_batch_clipping_dispatches_once_and_owns_results(self):
        for operation in ("diff", "intersection"):
            for facade in ("function", "api", "context"):
                with self.subTest(operation=operation, facade=facade):
                    api = self.make_api()
                    subject = self.geometry.PolylineCollection(api, 51)
                    clip = self.geometry.ExPolygonCollection(api, 22)
                    name = f"clipper_{operation}_polylines_expolygons"
                    if facade == "function":
                        result = getattr(self.clipper, name)(9, subject, clip)
                    elif facade == "api":
                        result = getattr(api, name)(9, subject, clip)
                    else:
                        result = getattr(api.clipper(9), f"{operation}_polylines_expolygons")(subject, clip)
                    self.assertIsInstance(result, self.geometry.StoredPolylineCollection)
                    self.assertEqual(result.storage(), 9)
                    self.assertEqual(api.host.calls, [(f"polylines_{operation}", (9, 51, 22))])
                    handle = result.handle()
                    self.assertTrue(result.free_from_storage())
                    self.assertFalse(result.free_from_storage())
                    self.assertEqual(api.host.freed, [(9, handle)])
                    self.assertEqual(subject.handle(), 51)
                    self.assertEqual(clip.handle(), 22)

    def test_batch_clipping_short_circuits_empty_subjects(self):
        for operation in ("diff", "intersection"):
            with self.subTest(operation=operation):
                api = self.make_api()
                subject = self.geometry.PolylineCollection(api, 52)
                clip = self.geometry.ExPolygonCollection(api, 22)
                with getattr(api, f"clipper_{operation}_polylines_expolygons")(9, subject, clip) as result:
                    self.assertTrue(result.empty())
                self.assertEqual(api.host.calls, [])
                self.assertEqual(len(api.host.freed), 1)

    def test_batch_clipping_rejects_invalid_storage_api_and_native_results(self):
        for operation in ("diff", "intersection"):
            with self.subTest(operation=operation):
                api = self.make_api()
                other_api = self.make_api()
                subject = self.geometry.PolylineCollection(api, 51)
                clip = self.geometry.ExPolygonCollection(api, 22)
                foreign_clip = self.geometry.ExPolygonCollection(other_api, 22)
                foreign_subject = self.geometry.PolylineCollection(other_api, 51)
                name = f"clipper_{operation}_polylines_expolygons"
                function = getattr(self.clipper, name)
                with self.assertRaises(ValueError):
                    function(0, subject, clip)
                with self.assertRaises(ValueError):
                    function(9, subject, foreign_clip)
                with self.assertRaises(ValueError):
                    getattr(api, name)(9, foreign_subject, clip)
                with self.assertRaises(ValueError):
                    getattr(api.clipper(9), f"{operation}_polylines_expolygons")(subject, foreign_clip)
                with self.assertRaises(ValueError):
                    getattr(api.clipper(9), f"{operation}_polylines_expolygons")(foreign_subject, foreign_clip)
                self.assertEqual(api.host.calls, [])
                api.host.fail_next = True
                with self.assertRaises(ValueError):
                    function(9, subject, clip)
                self.assertEqual(api.host.freed, [])

    def test_generated_bindings_include_batch_clipping_signatures(self):
        signatures = {name: (result, arguments) for name, result, arguments in self.generated.C_FUNCTION_SIGNATURES}
        for operation in ("diff", "intersection"):
            name = f"clipper_{operation}_polylines_expolygons"
            with self.subTest(name=name):
                self.assertIn(name, signatures)
                self.assertEqual(signatures[name], (ctypes.c_void_p, [ctypes.c_void_p] * 3))
                self.assertIn(name, self.clipper.__all__)
                self.assertTrue(callable(getattr(self.api_module, name)))
        self.assertEqual(self.generated.SLIC3R_PLUGIN_API_CLIPPER_MINOR, 3)

    def test_collection_cleanups_and_polyline_offset_forward_to_the_host(self):
        api = self.make_api()
        host = api.host
        calls = []
        for name in ("polygons_ensure_valid", "polygons_smooth_outward", "polylines_ensure_valid"):
            setattr(host, name, lambda handle, value, name=name: calls.append((name, address(handle), int(value))))
        for name in ("polygons_remove_sticks", "polygons_remove_degenerate", "polylines_remove_degenerate"):
            setattr(host, name, lambda handle, name=name: calls.append((name, address(handle))) or 1)
        host.clipper_offset_polylines = lambda storage, polylines, delta, join, miter, end: (
            calls.append(("offset", address(storage), address(polylines), delta, int(join), miter, int(end))) or 701
        )
        polygons = self.geometry.StoredPolygonCollection.adopt_owned(api, 9, 61)
        polylines = self.geometry.StoredPolylineCollection.adopt_owned(api, 9, 62)

        polygons.ensure_valid(15)
        self.assertTrue(polygons.remove_sticks())
        self.assertTrue(polygons.remove_degenerate())
        polygons.smooth_outward(300)
        polylines.ensure_valid(16)
        self.assertTrue(polylines.remove_degenerate())
        operand = self.clipper.clipper_offset_polylines(9, polylines, 250.0)
        self.assertIsInstance(operand, self.clipper.ClipperOperand)
        self.assertEqual(operand.handle(), 701)
        self.assertEqual(calls, [
            ("polygons_ensure_valid", 61, 15),
            ("polygons_remove_sticks", 61),
            ("polygons_remove_degenerate", 61),
            ("polygons_smooth_outward", 61, 300),
            ("polylines_ensure_valid", 62, 16),
            ("polylines_remove_degenerate", 62),
            ("offset", 9, 62, 250.0, self.generated.CLIPPER_JOIN_SQUARE, 0.0, self.generated.CLIPPER_END_OPEN_BUTT),
        ])
        with self.assertRaises(ValueError):
            self.clipper.clipper_offset_polylines(9, polylines, 0.0)
        self.assertIn("clipper_offset_polylines", self.clipper.__all__)
        self.assertEqual(self.generated.SLIC3R_PLUGIN_API_GEOMETRY_MINOR, 1)
        signatures = {name: (result, arguments) for name, result, arguments in self.generated.C_FUNCTION_SIGNATURES}
        self.assertEqual(signatures["clipper_offset_polylines"],
                         (ctypes.c_void_p, [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double,
                                            ctypes.c_int32, ctypes.c_double, ctypes.c_int32]))
        for name in ("polygons_ensure_valid", "polygons_remove_sticks", "polygons_remove_degenerate",
                     "polygons_smooth_outward", "polylines_ensure_valid", "polylines_remove_degenerate"):
            self.assertIn(name, signatures)

    def test_polyline_difference_short_circuits_an_empty_subject(self):
        api = self.make_api()
        subject = self.geometry.Polyline(api, 41)
        clip = self.geometry.ExPolygonCollection(api, 42)

        result = self.clipper.clipper_diff_polyline_expolygons(6, subject, clip)

        self.assertIsInstance(result, self.geometry.StoredPolylineCollection)
        self.assertTrue(result.empty())
        self.assertFalse(any(call[0] == "polyline_diff" for call in api.host.calls))
        self.assertTrue(result.free_from_storage())


if __name__ == "__main__":
    unittest.main()
