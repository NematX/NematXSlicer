"""Test Python Surface ownership, angle access and finalization callback transport.

The fake host invalidates element addresses on structural changes and records
collection ownership. Bridge-angle tests use its local metadata; finalization
tests use generated ctypes callbacks with synthetic addresses, not a live slicer.

Selected direct calls from the finalization transport test:

    test_finalize_payload_preserves_both_callbacks_and_passes_the_batch()
        |-- FinalizeSlicingContext.from_finalize_slicing_run_context()
        |-- FinalizeSlicingContext.requested_support_areas()
        `-- FinalizeSlicingContext.replace_infill_buckets()

The wrappers invoke typed ctypes callbacks stored in the synthetic payload. Its
request lambda returns a fake address; the replace callback records the borrowed
batch synchronously and returns success without validating native geometry.
"""

import copy
import ctypes
import importlib
import importlib.util
import math
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


class ReallocatingSurfaceHost:
    """Tiny ABI host that invalidates every element handle on append."""

    def __init__(self):
        self._next_handle = 100
        self.collections = {}
        self.surfaces = {}
        self.properties = {}
        self.geometry_point_counts = {}
        self.freed = []

    @staticmethod
    def _int(handle):
        if isinstance(handle, ctypes.c_void_p):
            return int(handle.value or 0)
        return int(handle)

    def _new_handle(self):
        self._next_handle += 1
        return self._next_handle

    def storage_new_surface_collection(self, _storage):
        # Retain the owner id so the angle test can reject a foreign storage capability.
        handle = self._new_handle()
        self.collections[handle] = {"items": [], "handles": [], "storage": self._int(_storage)}
        return handle

    def surface_collection_size(self, collection):
        return len(self.collections[self._int(collection)]["items"])

    def _rebuild_element_handles(self, collection):
        record = self.collections[self._int(collection)]
        for old_handle in record["handles"]:
            self.surfaces.pop(old_handle, None)
        record["handles"] = []
        for item in record["items"]:
            handle = self._new_handle()
            property_handle = self._new_handle()
            record["handles"].append(handle)
            self.surfaces[handle] = item
            self.properties[property_handle] = item["properties"]
            item["property_handle"] = property_handle

    def surface_collection_append_expolygon_copy(
        self, collection, area, surface_type
    ):
        source_area = self._int(area)
        copied_area = self._new_handle()
        self.geometry_point_counts[copied_area] = self.geometry_point_counts.get(
            source_area, 4
        )
        record = self.collections[self._int(collection)]
        record["items"].append(
            {
                "area": copied_area,
                "type": int(surface_type),
                "properties": {},
            }
        )
        self._rebuild_element_handles(collection)

    def surface_collection_append_expolygon_move(
        self, collection, area, surface_type
    ):
        source_area = self._int(area)
        self.surface_collection_append_expolygon_copy(
            collection, area, surface_type
        )
        self.geometry_point_counts[source_area] = 0

    def surface_collection_at(self, collection, idx):
        return self.collections[self._int(collection)]["handles"][int(idx)]

    def surface_collection_at_mutable(self, collection, idx):
        return self.surface_collection_at(collection, idx)

    def _surface(self, handle):
        address = self._int(handle)
        if address not in self.surfaces:
            raise RuntimeError("stale Surface handle used after reallocation")
        return self.surfaces[address]

    def surface_get_expolygon(self, surface):
        return self._surface(surface)["area"]

    def surface_get_type(self, surface):
        return self._surface(surface)["type"]

    def surface_get_bridge_angle(self, surface):
        """Read fake metadata without changing element handles; absent means -1."""
        return self._surface(surface).get("bridge_angle", -1.0)

    def surface_collection_set_bridge_angle(self, storage, collection, index, angle):
        """Model ownership and modulo-pi writes for the tests' nonnegative indices."""
        record = self.collections[self._int(collection)]
        if record["storage"] != self._int(storage) or index >= len(record["items"]) or not math.isfinite(angle) or angle < 0:
            return 0
        record["items"][index]["bridge_angle"] = angle % math.pi
        return 1

    def surface_collection_set_type(self, storage, collection, index, surface_type):
        """Reclassify owned fake metadata while leaving geometry/properties intact."""
        record = self.collections[self._int(collection)]
        position, density = surface_type & 15, surface_type & 112
        if (record["storage"] != self._int(storage) or index >= len(record["items"])
                or not position or position & (position - 1) or not density or density & (density - 1)
                or surface_type & ~(15 | 112 | 128 | 256 | 512 | 2048)):
            return 0
        record["items"][index]["type"] = surface_type
        return 1

    def surface_get_properties(self, surface):
        return self._surface(surface)["property_handle"]

    def surface_collection_get_properties_mutable(self, collection, idx):
        return self.surface_get_properties(self.surface_collection_at_mutable(collection, idx))

    def expolygon_contour_const(self, expolygon):
        return self._int(expolygon)

    def polygon_as_multipoint_const(self, polygon):
        return self._int(polygon)

    def multipoint_size(self, multipoint):
        return self.geometry_point_counts.get(self._int(multipoint), 4)

    def plugin_property_copy_all(self, destination, source):
        dst = self.properties[self._int(destination)]
        src_address = self._int(source)
        if src_address:
            dst.clear()
            dst.update(self.properties[src_address])

    def storage_free(self, _storage, handle):
        address = self._int(handle)
        self.freed.append((self._int(_storage), address))
        record = self.collections.pop(address, None)
        if record is not None:
            for surface_handle in record["handles"]:
                self.surfaces.pop(surface_handle, None)
        return 1


class PythonSurfaceViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "surface_test_api_generator", RUNTIME / "generate_slic3r_api.py"
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = (
            Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        )
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.geometry = importlib.import_module("slic3r_geometry_views")
        cls.surfaces = importlib.import_module("slic3r_surface_views")
        cls.datatree = importlib.import_module("slic3r_datatree_views")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def test_append_like_snapshots_template_before_destination_reallocation(self):
        host = ReallocatingSurfaceHost()
        api = type("Api", (), {"host": host})()
        collection = self.surfaces.StoredSurfaceCollection(api, 1)
        collection.append(self.geometry.ExPolygon(api, 900), 17)
        template = collection.front()
        host._surface(template.c_handle())["properties"][42] = b"metadata"

        collection.append_like(self.geometry.ExPolygon(api, 901), template)

        self.assertEqual(collection.size(), 2)
        appended = host._surface(collection.back().c_handle())
        self.assertEqual(appended["type"], 17)
        self.assertEqual(appended["properties"], {42: b"metadata"})
        collection.free_from_storage()

    def test_bridge_angle_is_canonical_and_only_scratch_exposes_the_setter(self):
        """Check undefined/zero angles, normalization and the owned-only write API."""
        host = ReallocatingSurfaceHost()
        api = type("Api", (), {"host": host})()
        collection = self.surfaces.StoredSurfaceCollection(api, 1)
        collection.append(self.geometry.ExPolygon(api, 900), 17)
        surface = collection.front()
        self.assertEqual(surface.bridge_angle(), -1.0)
        self.assertFalse(hasattr(surface, "set_bridge_angle"))
        self.assertTrue(collection.set_bridge_angle(0, 0.0))
        self.assertEqual(surface.bridge_angle(), 0.0)
        self.assertTrue(collection.set_bridge_angle(0, math.pi))
        self.assertEqual(surface.bridge_angle(), 0.0)
        self.assertFalse(collection.set_bridge_angle(0, -1.0))
        self.assertFalse(collection.set_bridge_angle(0, math.nan))
        collection.free_from_storage()

    def test_owned_type_edit_preserves_geometry_angle_and_properties(self):
        """The wrapper forwards the storage capability, index and complete type bits."""
        host = ReallocatingSurfaceHost()
        api = type("Api", (), {"host": host})()
        collection = self.surfaces.StoredSurfaceCollection(api, 1)
        collection.append(self.geometry.ExPolygon(api, 900), 20)
        surface = collection.front()
        host._surface(surface.c_handle())["properties"][42] = b"metadata"
        collection.set_bridge_angle(0, 0.0)
        area = host._surface(surface.c_handle())["area"]
        self.assertFalse(hasattr(surface, "set_type"))
        self.assertTrue(collection.set_type(0, 20 | 128))
        self.assertEqual(surface.type(), 20 | 128)
        self.assertEqual(surface.bridge_angle(), 0.0)
        self.assertEqual(host._surface(surface.c_handle())["area"], area)
        self.assertEqual(host._surface(surface.c_handle())["properties"], {42: b"metadata"})
        self.assertFalse(collection.set_type(0, 21))
        self.assertFalse(collection.set_type(1, 20))
        self.assertFalse(host.surface_collection_set_type(2, collection.c_handle(), 0, 20))
        collection.free_from_storage()

    def test_finalize_payload_preserves_both_callbacks_and_passes_the_batch(self):
        """Check generated callback fields and synchronous argument transport without a host."""
        import slic3r_api_generated as generated
        from steps.execution import FinalizeSlicingContext

        fields = dict(generated.RunCtxFinalizeSlicing._fields_)
        self.assertIn("get_requested_support_areas", fields)
        calls = []

        def replace(_context, storage, entries, count):
            """Record the borrowed batch immediately; return success without geometry checks."""
            calls.append((storage, [(entries[index].destination, entries[index].prepared) for index in range(count)]))
            return 1

        request = fields["get_requested_support_areas"](lambda _island: 700)
        batch = fields["replace_infill_buckets"](replace)
        payload = generated.RunCtxFinalizeSlicing()
        payload.print = 10
        payload.object = 20
        payload.support_demand_computed = 1
        payload.support_material_generated = 0
        payload.get_requested_support_areas = request
        payload.replace_infill_buckets = batch
        common = generated.PluginRunContext()
        common.step = generated.STEP_FINALIZE_SLICING
        common.data = ctypes.addressof(payload)
        api = type("Api", (), {"host": ReallocatingSurfaceHost()})()
        context = FinalizeSlicingContext.from_finalize_slicing_run_context(api, ctypes.addressof(common))
        self.assertIsNotNone(context)
        self.assertTrue(context.support_demand_computed())
        self.assertFalse(context.support_material_generated())
        self.assertEqual(context.requested_support_areas(self.datatree.LayerIsland(api, 30)).address, 700)
        self.assertTrue(context.replace_infill_buckets(40, [(50, 60), (51, 61)]))
        self.assertEqual(calls, [(40, [(50, 60), (51, 61)])])

    def test_support_surface_type_builder_and_predicate(self):
        """Support membership composes with density, position and bridge modifiers."""
        from slic3r_api import surface_type_is_support

        views = self.surfaces
        base = views.srf_type().internal().sparse()
        support = base.support()
        self.assertEqual(support.raw(), views.RAW_SURFACE_TYPE_POS_INTERNAL |
                         views.RAW_SURFACE_TYPE_DENS_SPARSE | views.RAW_SURFACE_TYPE_MOD_SUPPORT)
        self.assertEqual(views.RAW_SURFACE_TYPE_MOD_SUPPORT, 1 << 9)
        self.assertTrue(surface_type_is_support(support.raw()))
        self.assertFalse(surface_type_is_support(base.raw()))
        combined = support.bridge().overbridge().raw()
        mask = (views.RAW_SURFACE_TYPE_MOD_SUPPORT | views.RAW_SURFACE_TYPE_MOD_BRIDGE |
                views.RAW_SURFACE_TYPE_MOD_OVERBRIDGE)
        self.assertEqual(combined & mask, mask)

    def test_contact_surface_type_builder_and_predicate(self):
        """The contact modifier marks the areas of a support contact layer and composes with support, position and density."""
        from slic3r_api import surface_type_is_contact, surface_type_is_support

        views = self.surfaces
        contact = views.srf_type().top().solid().support().contact()
        self.assertEqual(contact.raw(), views.RAW_SURFACE_TYPE_POS_TOP | views.RAW_SURFACE_TYPE_DENS_SOLID |
                         views.RAW_SURFACE_TYPE_MOD_SUPPORT | views.RAW_SURFACE_TYPE_MOD_CONTACT)
        self.assertEqual(views.RAW_SURFACE_TYPE_MOD_CONTACT, 1 << 11)
        self.assertTrue(surface_type_is_contact(contact.raw()))
        self.assertTrue(surface_type_is_support(contact.raw()))
        self.assertFalse(surface_type_is_contact(views.srf_type().top().solid().support().raw()))

    def test_append_like_move_uses_the_same_stable_snapshot(self):
        host = ReallocatingSurfaceHost()
        api = type("Api", (), {"host": host})()
        collection = self.surfaces.StoredSurfaceCollection(api, 1)
        collection.append(self.geometry.ExPolygon(api, 910), 23)
        template = collection.front()
        host._surface(template.c_handle())["properties"][7] = b"moved"
        area = self.geometry.StoredExPolygon.adopt_owned(api, 1, 911)

        collection.append_like_move(area, template)

        self.assertEqual(collection.size(), 2)
        appended = host._surface(collection.back().c_handle())
        self.assertEqual(appended["type"], 23)
        self.assertEqual(appended["properties"], {7: b"moved"})
        self.assertTrue(area)
        self.assertTrue(area.contour().empty())
        area.free_from_storage()
        collection.free_from_storage()

    def test_stored_surface_wrappers_have_unique_ownership(self):
        host = ReallocatingSurfaceHost()
        api = type("Api", (), {"host": host})()
        collection = self.surfaces.StoredSurfaceCollection(api, 1)
        area = self.geometry.StoredExPolygon.adopt_owned(api, 1, 920)
        surface = self.surfaces.StoredSurface(area, 17)

        for owner in (collection, surface):
            for copier in (copy.copy, copy.deepcopy):
                with self.subTest(owner=type(owner).__name__, copier=copier.__name__):
                    with self.assertRaisesRegex(
                        TypeError, rf"{type(owner).__name__}.*unique native resource"
                    ):
                        copier(owner)
        self.assertEqual(host.freed, [])
        self.assertTrue(collection.free_from_storage())
        self.assertTrue(surface.free_from_storage())
        self.assertEqual(len(host.freed), 2)

    def test_source_properties_are_read_only_and_scratch_requires_a_capability(self):
        class Payload(ctypes.Structure):
            _fields_ = [("value", ctypes.c_uint32)]
            property_type = 700

        payload = Payload(42)
        host = ReallocatingSurfaceHost()
        api = type("Api", (), {"host": host, "orchestrator": ctypes.c_void_p(1)})()
        host.plugin_property_data_size = lambda *_args: ctypes.sizeof(Payload)
        host.plugin_property_data = lambda *_args: ctypes.addressof(payload)
        host.plugin_property_data_mutable = lambda *_args: ctypes.addressof(payload)
        host.plugin_property_get_or_add_data_mutable = lambda *_args: ctypes.addressof(payload)
        scratch = self.surfaces.StoredSurfaceCollection(api, 1)
        scratch.append(self.geometry.ExPolygon(api, 900), 17)
        source = scratch.front()
        for forbidden in ("get_or_add", "get_mutable", "clear", "remove", "copy_from", "data_mutable_address"):
            self.assertFalse(hasattr(source.properties(), forbidden), forbidden)
        self.assertFalse(hasattr(source, "mutable_properties"))
        self.assertEqual(source.properties().get(Payload).value, 42)
        source.properties().get(Payload).value = 99
        self.assertEqual(payload.value, 42)
        with self.assertRaises(TypeError):
            self.surfaces.MutableSurface(api, source.c_handle())
        scratch.mutable_at(0).mutable_properties().get_or_add(Payload).value = 73
        self.assertEqual(source.properties().get(Payload).value, 73)
        scratch.free_from_storage()

    def test_node_properties_separate_read_and_mutable_getters(self):
        host = ReallocatingSurfaceHost()
        api = type("Api", (), {"host": host})()
        for name, prefix in (("Object", "object"), ("Layer", "layer"),
                             ("LayerRegion", "layer_region"), ("LayerIsland", "layer_island"),
                             ("LayerRegionIsland", "layer_region_island")):
            with self.subTest(node=name):
                setattr(host, prefix + "_get_properties", lambda *_args: 100)
                setattr(host, prefix + "_get_properties_mutable", lambda *_args: 200)
                node = getattr(self.datatree, name)(api, 1)
                self.assertIsInstance(node.properties(), self.datatree.ConstPluginProperties)
                self.assertNotIsInstance(node.properties(), self.datatree.PluginProperties)
                self.assertEqual(node.properties().address, 100)
                self.assertIsInstance(node.mutable_properties(), self.datatree.PluginProperties)
                self.assertEqual(node.mutable_properties().address, 200)

    def test_refused_capabilities_propagate_without_native_null_dereferences(self):
        host = ReallocatingSurfaceHost()
        api = type("Api", (), {"host": host, "orchestrator": ctypes.c_void_p(1)})()
        host.object_get_properties_mutable = lambda *_args: None
        host.object_add_auxiliary_layer = lambda *_args: None
        host.plugin_property_data_size = lambda *_args: 0
        host.plugin_property_get_or_add_data_mutable = lambda *_args: None
        obj = self.datatree.Object(api, 1)
        properties = obj.mutable_properties()
        self.assertFalse(properties.valid())
        self.assertIsNone(properties.get_mutable(self.datatree.LayerSupportProperty))
        with self.assertRaises(RuntimeError):
            self.datatree.LayerSupportProperty.key.get_or_add(properties)
        with self.assertRaises(RuntimeError):
            properties.get_or_add(self.datatree.LayerSupportProperty)
        writable = self.datatree.PluginProperties(api, 200)
        with self.assertRaisesRegex(RuntimeError, "native property creation"):
            writable.get_or_add(self.datatree.LayerSupportProperty)
        with self.assertRaises(ValueError):
            obj.add_auxiliary_layer(1, 1, 1)

    def test_refused_source_geometry_and_config_views_do_not_wrap_null_handles(self):
        host = ReallocatingSurfaceHost()
        api = type("Api", (), {"host": host})()
        host.layer_island_get_slice_mutable = lambda *_args: None
        host.layer_island_get_region_island_mutable = lambda *_args: None
        host.layer_region_island_get_mutable_extrusion = lambda *_args: None
        host.object_get_config_mutable = lambda *_args: None
        host.print_get_config_mutable = lambda *_args: None
        island = self.datatree.MutableLayerIsland(api, 1)
        with self.assertRaises(ValueError):
            island.slice_mutable()
        with self.assertRaises(ValueError):
            island.region_island_mutable(0)
        self.assertIsNone(self.datatree.MutableLayerRegionIsland(api, 1).extrusion_mutable(0))
        with self.assertRaises(ValueError):
            self.datatree.MutableObject(api, 1).config_mutable()
        with self.assertRaises(ValueError):
            self.datatree.MutablePrint(api, 1).config_mutable()


if __name__ == "__main__":
    unittest.main()
