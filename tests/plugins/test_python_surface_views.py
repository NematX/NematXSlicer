"""Behavior tests for the high-level Python Surface wrappers."""

import copy
import ctypes
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
        handle = self._new_handle()
        self.collections[handle] = {"items": [], "handles": []}
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
