"""Behavior and ownership tests for Python mesh, volume and painting views."""

import ctypes
import importlib
import importlib.util
import math
from pathlib import Path
import random
import sys
import tempfile
import unittest
from types import SimpleNamespace


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


class VolumeHost:
    """Small host that records mesh slicing and painting projection calls."""

    def __init__(self, generated):
        self.generated = generated
        self.next_handle = 100
        self.mesh_triangle_counts = {11: 0, 12: 4}
        self.volume_types = {}
        self.painting = set()
        self.slice_calls = []
        # Status returned by triangle_mesh_slice_to_polygons(): 1 done, 0 cancelled, -1 invalid, -2 internal.
        self.slice_status = 1
        self.projection_calls = []
        self.freed = []
        # Polygon collections allocated and not freed yet.
        self.live_polygons = set()

    def triangle_mesh_triangle_count(self, mesh):
        return self.mesh_triangle_counts[address(mesh)]

    def storage_new_polygons(self, storage):
        self.next_handle += 1
        self.live_polygons.add(self.next_handle)
        return self.next_handle

    def storage_new_expolygons(self, storage):
        self.next_handle += 1
        return self.next_handle

    def _record_slice(self, name, mesh, params, z_values, output, count):
        self.slice_calls.append(
            {
                "name": name,
                "mesh": address(mesh),
                "params": params,
                "z": [float(z_values[idx]) for idx in range(int(count))],
                "output": [address(output[idx]) for idx in range(int(count))],
            }
        )

    def triangle_mesh_slice_to_polygons(self, mesh, params, z_values, output, count, context):
        self._record_slice("polygons", mesh, params, z_values, output, count)
        self.slice_calls[-1]["context"] = ctypes.addressof(context._obj) if context is not None else None
        return self.slice_status

    def volume_get_type(self, volume):
        return self.volume_types[address(volume)]

    def volume_has_painting(self, volume, key):
        return (address(volume), bytes(key)) in self.painting

    def object_count_layer(self, obj):
        return 2 if address(obj) == 51 else 0

    def object_project_painting_to_polygons(
        self, obj, paint_key, painting_value, output, layer_count
    ):
        self.projection_calls.append(
            (
                address(obj),
                bytes(paint_key),
                int(painting_value),
                [address(output[idx]) for idx in range(int(layer_count))],
            )
        )

    def storage_free(self, storage, handle):
        self.freed.append((address(storage), address(handle)))
        self.live_polygons.discard(address(handle))
        return 1


class SlicingPipelineHost(VolumeHost):
    """
    Host with in-memory polygons and expolygons: it records the calls of the post-processing of the
    slices (ensure_valid, collinear points, union, offsets, largest contour) in order, and gives the
    raw rings, the union result and the signed area of each contour from the test.
    """

    def __init__(self, generated):
        super().__init__(generated)
        self.calls = []
        # Raw rings of each layer, as lists of (x, y); set by the test before the call.
        self.raw_layers = []
        # Areas of the expolygons returned by the union, set by the test.
        self.union_areas = []
        self.rings = {}              # polygon handle -> points
        self.polygon_lists = {}      # polygon collection handle -> polygon handles
        self.expolygon_lists = {}    # expolygon collection handle -> expolygon handles
        self.areas = {}              # expolygon handle (also its contour handle) -> signed area

    def _new(self):
        self.next_handle += 1
        return self.next_handle

    def storage_new_expolygon(self, storage):
        handle = self._new()
        self.areas[handle] = 0.0
        return handle

    def triangle_mesh_slice_to_polygons(self, mesh, params, z_values, output, count, context):
        status = super().triangle_mesh_slice_to_polygons(mesh, params, z_values, output, count, context)
        self.calls.append(("raw_slice", int(params._obj.mode), int(params._obj.mode_below)))
        for layer in range(int(count)):
            polygons = []
            for ring in self.raw_layers[layer]:
                handle = self._new()
                self.rings[handle] = list(ring)
                polygons.append(handle)
            self.polygon_lists[address(output[layer])] = polygons
        return status

    def polygons_ensure_valid(self, polygons, resolution):
        self.calls.append(("polygons_ensure_valid", int(resolution)))

    def polygons_size(self, polygons):
        return len(self.polygon_lists[address(polygons)])

    def polygons_at(self, polygons, idx):
        return self.polygon_lists[address(polygons)][int(idx)]

    def polygon_as_multipoint(self, polygon):
        return address(polygon)

    def multipoint_view_const(self, multipoint):
        points = self.rings[address(multipoint)]
        array = [self.generated.CPoint(x, y) for x, y in points]
        return SimpleNamespace(size=len(points), array=array)

    def multipoint_clear(self, multipoint):
        self.rings[address(multipoint)] = []

    def multipoint_insert_array(self, multipoint, idx, array, count):
        self.rings[address(multipoint)][int(idx):int(idx)] = [
            (int(array[item].x), int(array[item].y)) for item in range(int(count))
        ]

    def _expolygons_result(self, name, *arguments):
        self.calls.append((name, *arguments))
        handle = self._new()
        self.expolygon_lists[handle] = []
        return handle

    def clipper_union_ex(self, storage, polygons, fill_rule):
        handle = self._expolygons_result("union_ex", int(fill_rule))
        for area in self.union_areas:
            expolygon = self._new()
            self.areas[expolygon] = area
            self.expolygon_lists[handle].append(expolygon)
        return handle

    def _copy_expolygons(self, handle, source):
        self.expolygon_lists[handle] = list(self.expolygon_lists[address(source)])
        return handle

    def clipper_offset_ex(self, storage, src, delta, join_type, miter_limit):
        handle = self._expolygons_result("offset_ex", float(delta), int(join_type), float(miter_limit))
        return self._copy_expolygons(handle, src)

    def clipper_offset2_ex(self, storage, src, delta1, delta2, join_type, miter_limit):
        handle = self._expolygons_result(
            "offset2_ex", float(delta1), float(delta2), int(join_type), float(miter_limit)
        )
        return self._copy_expolygons(handle, src)

    def expolygons_ensure_valid(self, expolygons, resolution):
        self.calls.append(("expolygons_ensure_valid", int(resolution)))

    def expolygons_size(self, expolygons):
        return len(self.expolygon_lists[address(expolygons)])

    def expolygons_at_const(self, expolygons, idx):
        return self.expolygon_lists[address(expolygons)][int(idx)]

    def expolygons_at(self, expolygons, idx):
        return self.expolygon_lists[address(expolygons)][int(idx)]

    def expolygon_contour_const(self, expolygon):
        return address(expolygon)

    def polygon_area(self, polygon):
        return self.areas[address(polygon)]

    def expolygon_move(self, destination, source):
        self.areas[address(destination)] = self.areas[address(source)]

    def expolygons_erase(self, expolygons, begin, size):
        del self.expolygon_lists[address(expolygons)][int(begin):int(begin) + int(size)]

    def expolygons_clear(self, expolygons):
        self.expolygon_lists[address(expolygons)] = []

    def expolygons_insert_move(self, expolygons, idx, expolygon):
        self.expolygon_lists[address(expolygons)].insert(int(idx), address(expolygon))


def native_squared_distance(a, b, point):
    """Line::distance_to_squared_abp (Line.cpp) written with 2D vectors, as the native code does."""
    v = (float(b[0] - a[0]), float(b[1] - a[1]))
    va = (float(point[0] - a[0]), float(point[1] - a[1]))
    l2 = v[0] * v[0] + v[1] * v[1]
    if l2 == 0.0:
        return va[0] * va[0] + va[1] * va[1]
    t = va[0] * v[0] + va[1] * v[1]
    if t <= 0.0:
        return va[0] * va[0] + va[1] * va[1]
    if t >= l2:
        d = (float(point[0] - b[0]), float(point[1] - b[1]))
        return d[0] * d[0] + d[1] * d[1]
    w = ((t / l2) * v[0], (t / l2) * v[1])
    return (w[0] - va[0]) * (w[0] - va[0]) + (w[1] - va[1]) * (w[1] - va[1])


def native_remove_collinear(points, max_offset):
    """Transcription of Polygon::remove_collinear (Polygon.cpp), with a list erase in place of the vector one."""
    pts = list(points)
    if len(pts) < 3:
        return pts
    min_dist_sq = float(max_offset) * max_offset
    while len(pts) > 2 and native_squared_distance(pts[-1], pts[1], pts[0]) < min_dist_sq:
        del pts[0]
    idx = 1
    while idx < len(pts) - 1:
        if native_squared_distance(pts[idx - 1], pts[idx + 1], pts[idx]) < min_dist_sq:
            del pts[idx]
        else:
            idx += 1
    while len(pts) > 2 and native_squared_distance(pts[-2], pts[0], pts[-1]) < min_dist_sq:
        del pts[-1]
    return pts


class PythonVolumeViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "volume_test_api_generator", RUNTIME / "generate_slic3r_api.py"
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.views = importlib.import_module("slic3r_datatree_views")
        cls.geometry = importlib.import_module("slic3r_geometry_views")
        cls.api_module = importlib.import_module("slic3r_api")
        cls.run_context_module = importlib.import_module("slic3r_run_context")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def make_api(self):
        api = self.api_module.Slic3rAPI.__new__(self.api_module.Slic3rAPI)
        api.host = VolumeHost(self.generated)
        return api

    def test_mesh_empty_and_slicing_ownership(self):
        api = self.make_pipeline_api()
        api.host.raw_layers = [[], []]
        self.assertTrue(self.views.TriangleMesh(api, 11).empty())
        mesh = self.views.TriangleMesh(api, 12)
        self.assertFalse(mesh.empty())
        transform = self.generated.CMatrix4d()
        params = self.generated.CMeshSlicingParams()

        polygons = mesh.slice_to_polygons(7, transform, [0.2, 0.4])
        expolygons = mesh.slice_to_expolygons(7, transform, [0.3], params)

        self.assertEqual(len(polygons), 2)
        self.assertEqual(len(expolygons), 1)
        self.assertTrue(all(
            isinstance(item, self.geometry.StoredPolygonCollection)
            for item in polygons
        ))
        self.assertIsInstance(expolygons[0], self.geometry.StoredExPolygonCollection)
        # The ExPolygons are made from the raw rings of the host: the post-processing runs in Python.
        self.assertEqual([call["name"] for call in api.host.slice_calls], [
            "polygons",
            "polygons",
        ])
        self.assertEqual(api.host.slice_calls[0]["z"], [
            ctypes.c_float(0.2).value,
            ctypes.c_float(0.4).value,
        ])
        for result in [*polygons, *expolygons]:
            self.assertEqual(result.storage(), 7)
            self.assertTrue(result.free_from_storage())

    def test_mesh_slicing_maps_the_host_status_and_passes_the_context(self):
        api = self.make_api()
        mesh = self.views.TriangleMesh(api, 12)
        transform = self.generated.CMatrix4d()
        context = self.generated.PluginRunContext()
        address_of_context = ctypes.addressof(context)

        mesh.slice_to_polygons(7, transform, [0.2], None, address_of_context)[0].free_from_storage()
        mesh.slice_to_polygons(7, transform, [0.2])[0].free_from_storage()
        self.assertEqual([call["context"] for call in api.host.slice_calls], [address_of_context, None])

        expected_errors = {0: self.api_module.PluginCancelled, -1: ValueError, -2: RuntimeError}
        for status, error in expected_errors.items():
            api.host.slice_status = status
            api.host.freed.clear()
            with self.subTest(status=status), self.assertRaises(error):
                mesh.slice_to_polygons(7, transform, [0.2, 0.4], None, address_of_context)
            # The two collections allocated for the call are released before the exception leaves.
            self.assertEqual(len(api.host.freed), 2)
            self.assertEqual(api.host.live_polygons, set())

        # The three forms of context reach the host as the same structure; no layer still asks the host.
        api.host.slice_status = 1
        api.host.slice_calls.clear()
        for form in (address_of_context, self.run_context_module.PluginRunContextView(api, context), context):
            mesh.slice_to_polygons(7, transform, [0.2], None, form)[0].free_from_storage()
        self.assertEqual({call["context"] for call in api.host.slice_calls}, {address_of_context})
        api.host.slice_status = 0
        for slicer in (mesh.slice_to_polygons, mesh.slice_to_expolygons):
            with self.subTest(slicer=slicer.__name__), self.assertRaises(self.api_module.PluginCancelled):
                slicer(7, transform, [], None, address_of_context)

    def test_mesh_slicing_with_a_bad_argument_leaves_nothing_in_the_storage(self):
        api = self.make_pipeline_api()
        api.host.raw_layers = [[], []]
        mesh = self.views.TriangleMesh(api, 12)
        transform = self.generated.CMatrix4d()

        # Neither a context of an unknown type nor a transform of an unknown type reaches the host.
        with self.assertRaises((TypeError, ValueError)):
            mesh.slice_to_polygons(7, transform, [0.2, 0.4], None, object())
        with self.assertRaises((TypeError, ValueError)):
            mesh.slice_to_expolygons(7, transform, [0.2, 0.4], None, object())
        with self.assertRaises(TypeError):
            mesh.slice_to_polygons(7, "not a matrix", [0.2, 0.4])
        self.assertEqual(api.host.slice_calls, [])
        self.assertEqual(api.host.live_polygons, set())

    def test_expolygon_slicing_checks_the_context_at_every_layer(self):
        api = self.make_pipeline_api()
        api.host.raw_layers = [[], [], []]
        mesh = self.views.TriangleMesh(api, 12)
        transform = self.generated.CMatrix4d()
        checks = []
        # The host is a fake: only the checks of the Python post-processing reach this callback.
        for cancel_after in range(0, 4):
            checks.clear()
            context = self.generated.PluginRunContext()
            callback = self.generated.PLUGIN_IS_CANCELLED(
                lambda _host: checks.append(None) or (1 if len(checks) > cancel_after else 0)
            )
            context.is_cancelled = callback
            with self.subTest(cancel_after=cancel_after), self.assertRaises(self.api_module.PluginCancelled):
                mesh.slice_to_expolygons(7, transform, [0.1, 0.2, 0.3], None, context)
            # It stops at the first check that answers yes: no more work after it.
            self.assertEqual(len(checks), cancel_after + 1)
            self.assertEqual(api.host.live_polygons, set())

    def test_mesh_slicing_rejects_invalid_height_order_and_values(self):
        api = self.make_api()
        mesh = self.views.TriangleMesh(api, 12)
        transform = self.generated.CMatrix4d()

        for heights in ([0.4, 0.2], [0.2, math.nan], [math.inf]):
            with self.subTest(heights=heights), self.assertRaises(ValueError):
                mesh.slice_to_polygons(7, transform, heights)
        # An empty list of heights still goes through the host, which checks the cancellation first.
        self.assertEqual(mesh.slice_to_expolygons(7, transform, []), [])
        self.assertEqual([call["z"] for call in api.host.slice_calls], [[]])

    def test_volume_role_and_painting_predicates(self):
        api = self.make_api()
        role_checks = {
            self.generated.RAW_VOLUME_TYPE_MODEL_PART: "is_model_part",
            self.generated.RAW_VOLUME_TYPE_NEGATIVE_VOLUME: "is_negative",
            self.generated.RAW_VOLUME_TYPE_PARAMETER_MODIFIER: "is_modifier",
            self.generated.RAW_VOLUME_TYPE_SUPPORT_ENFORCER: "is_support_enforcer",
            self.generated.RAW_VOLUME_TYPE_SUPPORT_BLOCKER: "is_support_blocker",
            self.generated.RAW_VOLUME_TYPE_SEAM_POSITION_CENTER: "is_seam_position",
            self.generated.RAW_VOLUME_TYPE_SEAM_POSITION_CENTER_Z: "is_seam_position",
            self.generated.RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE_CENTER: "is_seam_position",
            self.generated.RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE: "is_seam_position",
            self.generated.RAW_VOLUME_TYPE_BRIM_PATCH: "is_brim",
            self.generated.RAW_VOLUME_TYPE_BRIM_NEGATIVE: "is_brim",
            self.generated.RAW_VOLUME_TYPE_SUPPORT_COLUMN: "is_support_column",
            self.generated.RAW_VOLUME_TYPE_NO_SUPPORT_ZONE: "is_no_support_zone",
            self.generated.RAW_VOLUME_TYPE_SUPPORT_INTERFACE_COLUMN: "is_support_interface_column",
        }
        self.assertEqual(self.generated.RAW_VOLUME_TYPE_SUPPORT_COLUMN, 11)
        self.assertEqual(self.generated.RAW_VOLUME_TYPE_NO_SUPPORT_ZONE, 12)
        self.assertEqual(self.generated.RAW_VOLUME_TYPE_SUPPORT_INTERFACE_COLUMN, 13)
        for handle, (volume_type, predicate) in enumerate(role_checks.items(), 20):
            api.host.volume_types[handle] = volume_type
            volume = self.views.Volume(api, handle)
            with self.subTest(volume_type=volume_type):
                self.assertTrue(getattr(volume, predicate)())
                # Demand modifiers and geometry modifiers stay two disjoint families.
                self.assertEqual(volume.is_support_modifier(),
                                 predicate in {"is_support_enforcer", "is_support_blocker"})
                self.assertEqual(volume.is_support_geometry_modifier(),
                                 predicate in {"is_support_column", "is_no_support_zone", "is_support_interface_column"})

        painted_handle = 45
        api.host.volume_types[painted_handle] = self.generated.RAW_VOLUME_TYPE_MODEL_PART
        api.host.painting.update({
            (painted_handle, self.generated.RAW_FACET_PAINTING_FDM_SUPPORT.encode()),
            (painted_handle, self.generated.RAW_FACET_PAINTING_SEAM.encode()),
            (painted_handle, self.generated.RAW_FACET_PAINTING_MMU_SEGMENTATION.encode()),
        })
        volume = self.views.Volume(api, painted_handle)
        self.assertTrue(volume.is_fdm_support_painted())
        self.assertTrue(volume.is_seam_painted())
        self.assertTrue(volume.is_mm_painted())

    def test_generic_painting_projection_returns_owned_layer_results(self):
        api = self.make_api()
        obj = self.views.Object(api, 51)

        projected = api.project_painting_to_polygons(
            8,
            obj,
            self.generated.RAW_FACET_PAINTING_SEAM,
            self.generated.RAW_FACET_PAINTING_ENFORCER,
        )

        self.assertEqual(len(projected), 2)
        self.assertEqual(api.host.projection_calls[0][:3], (
            51,
            self.generated.RAW_FACET_PAINTING_SEAM.encode(),
            self.generated.RAW_FACET_PAINTING_ENFORCER,
        ))
        self.assertEqual(
            api.host.projection_calls[0][3],
            [item.handle() for item in projected],
        )
        for result in projected:
            self.assertEqual(result.storage(), 8)
            self.assertTrue(result.free_from_storage())

        empty_object = self.views.Object(api, 52)
        self.assertEqual(api.project_painting_to_polygons(
            8,
            empty_object,
            self.generated.RAW_FACET_PAINTING_SEAM,
            self.generated.RAW_FACET_PAINTING_ENFORCER,
        ), [])
        with self.assertRaises(ValueError):
            api.project_painting_to_polygons(
                8,
                self.views.Object(self.make_api(), 51),
                self.generated.RAW_FACET_PAINTING_SEAM,
                self.generated.RAW_FACET_PAINTING_ENFORCER,
            )

    def make_pipeline_api(self):
        api = self.api_module.Slic3rAPI.__new__(self.api_module.Slic3rAPI)
        api.host = SlicingPipelineHost(self.generated)
        return api

    def slice_pipeline(self, api, params, layers=1):
        mesh = self.views.TriangleMesh(api, 12)
        return mesh.slice_to_expolygons(7, params.transform, [0.1 * (idx + 1) for idx in range(layers)], params)

    def test_expolygon_slicing_replays_the_native_post_processing(self):
        api = self.make_pipeline_api()
        host = api.host
        generated = self.generated
        params = generated.CMeshSlicingParams()
        params.mode = generated.RAW_MESH_SLICING_MODE_EVEN_ODD
        params.mode_below = generated.RAW_MESH_SLICING_MODE_POSITIVE_LARGEST_CONTOUR
        params.slicing_mode_normal_below_layer = 1
        params.closing_radius = 0.3
        params.extra_offset = 0.1
        params.model_resolution = 0.0000019   # 1 scaled unit once truncated (2 once rounded)
        params.resolution = 0.0125
        host.raw_layers = [[[(0, 0), (10, 0), (20, 0), (20, 20)]], [[(0, 0), (10, 0), (20, 0), (20, 20)]]]
        host.union_areas = [10.0, 30.0, 20.0, 30.0]

        result = self.slice_pipeline(api, params, layers=2)

        scale = generated.UNSCALING_FACTOR
        closing = int(float(ctypes.c_float(0.3).value) * scale)
        extra = int(float(ctypes.c_float(0.1).value) * scale)
        resolution = int(0.0125 * scale)
        mitre = generated.CLIPPER_JOIN_MITER
        offsets = [
            ("offset2_ex", float(closing), -float(closing - extra), mitre, 3.0),
            ("offset_ex", 10.0, mitre, 3.0),
        ]
        valid = [("expolygons_ensure_valid", resolution), ("expolygons_ensure_valid", resolution)]
        # Layer 0 is below slicing_mode_normal_below_layer: PositiveLargestContour for it (fill positive, largest contour only).
        # Layer 1 is EvenOdd. The raw slicing never sees PositiveLargestContour.
        expected = [
            ("raw_slice", generated.RAW_MESH_SLICING_MODE_EVEN_ODD, generated.RAW_MESH_SLICING_MODE_POSITIVE),
            ("polygons_ensure_valid", generated.SCALED_EPSILON),
            ("union_ex", generated.CLIPPER_FILL_POSITIVE),
            offsets[0], offsets[1], valid[0], valid[1],
            ("polygons_ensure_valid", generated.SCALED_EPSILON),
            ("union_ex", generated.CLIPPER_FILL_EVENODD),
            offsets[0], offsets[1], valid[0], valid[1],
        ]
        self.assertEqual(host.calls, expected)
        # Both rings lose the collinear point (10, 0) at the model resolution of 1 unit (1.9 truncates to 1).
        self.assertEqual([ring for ring in host.rings.values()], [[(0, 0), (20, 0), (20, 20)]] * 2)
        self.assertEqual(len(result), 2)
        self.assertEqual(len(host.expolygon_lists[result[0].handle()]), 1)
        self.assertEqual(host.areas[host.expolygon_lists[result[0].handle()][0]], 30.0)
        self.assertEqual(len(host.expolygon_lists[result[1].handle()]), 4)
        for item in result:
            self.assertTrue(item.free_from_storage())

    def test_expolygon_slicing_without_cleaning_distance_skips_offsets(self):
        api = self.make_pipeline_api()
        host = api.host
        params = self.generated.CMeshSlicingParams()
        host.raw_layers = [[]]
        host.union_areas = []

        result = self.slice_pipeline(api, params)

        self.assertEqual(host.calls, [
            ("raw_slice", 0, 0),
            ("polygons_ensure_valid", self.generated.SCALED_EPSILON),
            ("union_ex", self.generated.CLIPPER_FILL_NONZERO),
            ("expolygons_ensure_valid", self.generated.SCALED_EPSILON),
            ("expolygons_ensure_valid", 0),
        ])
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].storage(), 7)
        self.assertTrue(result[0].free_from_storage())

    def test_expolygon_slicing_offset_branches_follow_the_truncated_integers(self):
        generated = self.generated
        mitre = generated.CLIPPER_JOIN_MITER
        cases = [
            # (closing, extra) in millimeters -> expected offsets in scaled units
            (0.0000019, 0.0, [("offset2_ex", 1.0, -1.0, mitre, 3.0)]),   # 1.9 truncates to 1, never 2
            (0.0, 0.0000019, [("offset_ex", 1.0, mitre, 3.0)]),          # the extra offset alone
            (0.0, -0.0000019, [("offset_ex", -1.0, mitre, 3.0)]),        # a negative extra offset shrinks
            (0.0000009, 0.0, []),                                          # below one unit: nothing to do
            (0.0000019, 0.0000013, [("offset_ex", 1.0, mitre, 3.0)]),    # equal after truncation: grow only
        ]
        for closing, extra, expected in cases:
            with self.subTest(closing=closing, extra=extra):
                api = self.make_pipeline_api()
                params = generated.CMeshSlicingParams()
                params.closing_radius = closing
                params.extra_offset = extra
                api.host.raw_layers = [[]]
                result = self.slice_pipeline(api, params)
                offsets = [call for call in api.host.calls if call[0] in ("offset_ex", "offset2_ex")]
                self.assertEqual(offsets, expected)
                for item in result:
                    item.free_from_storage()

    def test_remove_collinear_points_on_the_case_where_a_removal_hides_the_next_one(self):
        ring = [(0, 0), (1000, 99), (1100, -9), (2000, 0), (0, 1000)]
        # The native function deletes (1100,-9) and keeps (1000,99).
        self.assertEqual(self.views._remove_collinear_points(ring, 100), [(0, 0), (1000, 99), (2000, 0), (0, 1000)])
        self.assertEqual(native_remove_collinear(ring, 100), [(0, 0), (1000, 99), (2000, 0), (0, 1000)])

    def test_remove_collinear_points_matches_the_native_algorithm(self):
        listed = [
            [], [(0, 0)], [(0, 0), (10, 10)], [(0, 0), (10, 0), (20, 0)],
            [(0, 0), (10, 0), (20, 0), (30, 0), (40, 0)],
            [(0, 0), (10, 0), (20, 0), (30, 0), (30, 40)],
            [(0, 0), (0, 40), (10, 40), (20, 40), (30, 40)],
            [(0, 0), (5, 0), (10, 0), (10, 10), (10, 20), (0, 20), (0, 10)],
            [(0, 0), (0, 0), (10, 0), (10, 0), (10, 10), (0, 10)],
            [(0, 0), (0, 0), (0, 0), (0, 0)],
            [(0, 0), (100, 0), (50, 1), (200, 0), (0, 100)],
            [(0, 0), (100, 0), (-50, 1), (200, 0), (0, 100)],
            [(0, 0), (10, 0), (0, 0), (10, 0), (0, 0)],
        ]
        generator = random.Random(20261019)
        rings = list(listed)
        for _unused in range(3000):
            count = generator.randint(0, 14)
            rings.append([(generator.randint(-60, 60), generator.randint(-60, 60)) for _item in range(count)])
            walk = (0, 0)
            ring = []
            for _item in range(count):
                walk = (walk[0] + generator.randint(-2, 2) * 20 + generator.randint(-1, 1),
                        walk[1] + generator.randint(-2, 2) * 20 + generator.randint(-1, 1))
                ring.append(walk)
            rings.append(ring)
        removed = 0
        for ring in rings:
            for offset in (0, 1, 2, 5, 40, 300):
                kept = self.views._remove_collinear_points(ring, offset)
                self.assertEqual(kept, native_remove_collinear(ring, offset), (ring, offset))
                removed += len(ring) - len(kept)
        self.assertGreater(removed, 1000)

    def test_slicing_pipeline_rewrites_only_the_rings_that_lose_a_point(self):
        api = self.make_pipeline_api()
        host = api.host
        params = self.generated.CMeshSlicingParams()
        params.model_resolution = 0.000001 * 100
        host.raw_layers = [[
            [(0, 0), (1000, 99), (1100, -9), (2000, 0), (0, 1000)],
            [(0, 0), (1000, 0), (1000, 1000), (0, 1000)],
        ]]
        rewritten = []
        original = host.multipoint_clear
        host.multipoint_clear = lambda multipoint: (rewritten.append(address(multipoint)), original(multipoint))

        result = self.slice_pipeline(api, params)

        first, second = host.polygon_lists[next(iter(host.polygon_lists))]
        self.assertEqual(host.rings[first], [(0, 0), (1000, 99), (2000, 0), (0, 1000)])
        self.assertEqual(host.rings[second], [(0, 0), (1000, 0), (1000, 1000), (0, 1000)])
        self.assertEqual(rewritten, [first])
        for item in result:
            item.free_from_storage()


if __name__ == "__main__":
    unittest.main()
