"""Behavior tests for the small public Python data-tree accessors."""

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


def address(handle):
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    return int(handle)


class DataTreeHost:
    def __init__(self, generated):
        self.generated = generated
        self.flow_calls = []
        self.bridge_calls = []
        self.region_island_calls = []
        self.curled = [
            generated.CCurledLine(
                generated.CPoint(10, 20), generated.CPoint(30, 40), 0.5
            ),
            generated.CCurledLine(
                generated.CPoint(50, 60), generated.CPoint(70, 80), 1.5
            ),
        ]
        self.freed = []

    def storage_new_config(self, storage):
        self.config_storage = address(storage)
        return 801

    def storage_free(self, storage, handle):
        self.freed.append((address(storage), address(handle)))
        return 1

    def layer_region_get_flow(self, region, role):
        self.flow_calls.append((address(region), int(role)))
        return self.generated.CFlow()

    def layer_region_get_bridging_flow(self, region, role):
        self.bridge_calls.append((address(region), int(role)))
        return self.generated.CFlow()

    def layer_region_island_count_region(self, region_island):
        return 2

    def layer_region_island_get_region(self, region_island, idx):
        return 200 + int(idx)

    def layer_island_get_or_create_region_island(
        self, island, _regions, region_count, extruder_id
    ):
        self.region_island_calls.append(
            (address(island), int(region_count), int(extruder_id))
        )
        return 701

    def layer_get_height(self, layer):
        return 40

    def layer_get_print_z(self, layer):
        return 140

    def layer_get_object(self, layer):
        return 300

    def layer_count_curled_line(self, layer):
        return len(self.curled)

    def layer_get_curled_line(self, layer, idx):
        return self.curled[int(idx)]

    def object_count_instance(self, obj):
        return 2

    def object_get_instance_shift(self, obj, idx):
        shifts = ((1000, 2000), (3000, 4000))
        return self.generated.CPoint(*shifts[int(idx)])


class PythonDataTreeViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "datatree_test_api_generator", RUNTIME / "generate_slic3r_api.py"
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = (
            Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        )
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.views = importlib.import_module("slic3r_datatree_views")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def make_api(self):
        return type("Api", (), {"host": DataTreeHost(self.generated)})()

    def test_region_registration_and_immutable_public_surface(self):
        api = self.make_api()
        calls = []
        def register(printer, config):
            calls.append((address(printer), address(config)))
            return 901
        api.host.print_register_print_region_config = register
        region = self.views.Print(api, 11).register_print_region_config(self.views.Config(api, 21))
        self.assertEqual(region.address, 901)
        self.assertEqual(calls, [(11, 21)])
        self.assertFalse(hasattr(self.views, "MutablePrintRegion"))
        self.assertFalse(hasattr(self.views.MutableObject, "print_region_mutable"))
        self.assertTrue(hasattr(self.views.MutablePrint, "config_mutable"))
        self.assertTrue(hasattr(self.views.MutableObject, "config_mutable"))
        api.host.print_register_print_region_config = lambda *args: None
        with self.assertRaises(ValueError):
            self.views.Print(api, 11).register_print_region_config(self.views.Config(api, 21))

    def test_layer_region_flows_and_region_island_iteration(self):
        api = self.make_api()
        layer_region = self.views.LayerRegion(api, 20)

        self.assertIsInstance(
            layer_region.flow(7), self.generated.CFlow
        )
        self.assertIsInstance(layer_region.bridging_flow(8), self.generated.CFlow)
        self.assertEqual(api.host.flow_calls, [(20, 7)])
        self.assertEqual(api.host.bridge_calls, [(20, 8)])

        region_island = self.views.LayerRegionIsland(api, 30)
        self.assertEqual(region_island.region_count(), 2)
        self.assertEqual(
            [region.address for region in region_island.regions()], [200, 201]
        )
        with self.assertRaises(IndexError):
            region_island.region(-1)
        with self.assertRaises(IndexError):
            region_island.region(2)

    def test_region_island_extruder_boundaries_reject_before_ctypes_narrowing(self):
        api = self.make_api()
        island = self.views.LayerIsland(api, 30)
        host = api.host

        for extruder_id in (self.generated.EXTRUDER_ID_UNKNOWN, 0, 0x7FFF):
            region_island = island.get_or_create_full_region_island(extruder_id)
            self.assertEqual(region_island.address, 701)
            self.assertEqual(host.region_island_calls[-1], (30, 0, extruder_id))

        call_count = len(host.region_island_calls)
        for invalid in (-2, 0x8000):
            with self.assertRaises(ValueError):
                island.get_or_create_full_region_island(invalid)
        with self.assertRaises(TypeError):
            island.get_or_create_full_region_island(1.5)
        self.assertEqual(len(host.region_island_calls), call_count)

    def test_auxiliary_fill_surface_binding_moves_clears_and_reports_rejection(self):
        """Forward temporary surfaces or None to the auxiliary mutator and preserve its success result."""
        api = self.make_api()
        calls = []
        collections = {801: ["support"]}
        destination = []

        def publish(region, surfaces):
            """Model native move semantics for auxiliary region 30 and reject object region 31."""
            source = address(surfaces) if surfaces is not None else 0
            calls.append((address(region), source))
            if address(region) != 30:
                return 0
            destination[:] = collections[source] if source else []
            if source:
                collections[source].clear()
            return 1

        api.host.layer_region_island_set_fill_surfaces = publish
        api.host.storage_new_surface_collection = lambda storage: 801
        surfaces = self.views.StoredSurfaceCollection(api, 11)
        region = self.views.LayerRegionIsland(api, 30)
        self.assertTrue(region.set_fill_surfaces(surfaces))
        self.assertEqual(destination, ["support"])
        self.assertEqual(collections[801], [])
        self.assertTrue(region.set_fill_surfaces())
        self.assertEqual(destination, [])
        collections[801].append("interface")
        self.assertFalse(self.views.LayerRegionIsland(api, 31).set_fill_surfaces(surfaces))
        self.assertEqual(collections[801], ["interface"])
        self.assertEqual(calls, [(30, 801), (30, 0), (31, 801)])
        surfaces.free_from_storage()

    def test_stored_config_has_unique_ownership(self):
        api = self.make_api()
        config = self.views.StoredConfig(api, 11)

        for copier in (copy.copy, copy.deepcopy):
            with self.assertRaisesRegex(TypeError, "StoredConfig.*unique native resource"):
                copier(config)
        self.assertEqual(api.host.freed, [])
        self.assertEqual(config.address, 801)
        self.assertTrue(config.close())
        self.assertEqual(api.host.freed, [(11, 801)])

    def test_layer_owner_bottom_and_curled_line_snapshots(self):
        api = self.make_api()
        layer = self.views.Layer(api, 40)

        self.assertEqual(layer.bottom_z(), 100)
        self.assertEqual(layer.object().address, 300)
        self.assertEqual(layer.curled_line_count(), 2)
        lines = layer.curled_lines()
        self.assertEqual(
            [(line.a.x, line.b.y) for line in lines], [(10, 40), (50, 80)]
        )
        with self.assertRaises(IndexError):
            layer.curled_line(2)

    def test_object_instance_shifts_are_scaled_value_snapshots(self):
        api = self.make_api()
        obj = self.views.Object(api, 50)

        self.assertEqual(obj.instance_count(), 2)
        shift = obj.instance_shift(1)
        self.assertEqual((shift.x, shift.y), (3000, 4000))
        with self.assertRaises(IndexError):
            obj.instance_shift(-1)
        with self.assertRaises(IndexError):
            obj.instance_shift(2)

    def test_object_support_and_region_flows_forward_their_arguments(self):
        api = self.make_api()
        calls = []

        def support_flow(obj, role, flow_height, print_height, flags):
            calls.append(("support", address(obj), int(role), flow_height, print_height, int(flags)))
            return self.generated.CFlow(width=440)

        def region_flow(obj, region_idx, role, height, layer_id):
            calls.append(("region", address(obj), int(region_idx), int(role), height, int(layer_id)))
            return self.generated.CFlow(width=450)

        api.host.object_get_support_flow = support_flow
        api.host.object_get_region_flow = region_flow
        obj = self.views.Object(api, 50)
        flags = self.generated.OBJECT_SUPPORT_FLOW_BOTTOM_CONTACT | self.generated.OBJECT_SUPPORT_FLOW_BRIDGE
        self.assertEqual(obj.support_flow(7, 0.2, 0.15, flags).width, 440)
        self.assertEqual(obj.support_flow(7, 0.2).width, 440)
        self.assertEqual(obj.region_flow(1, 9, 0.2, 2).width, 450)
        self.assertEqual(calls, [
            ("support", 50, 7, 0.2, 0.15, flags),
            ("support", 50, 7, 0.2, 0.0, 0),
            ("region", 50, 1, 9, 0.2, 2),
        ])

        def support_flow_mm(obj, role, flow_height, print_height, flags):
            calls.append(("support_mm", address(obj), int(role), flow_height, print_height, int(flags)))
            return self.generated.CFlowMm(spacing=0.44)

        def region_flow_mm(obj, region_idx, role, height, layer_id):
            calls.append(("region_mm", address(obj), int(region_idx), int(role), height, int(layer_id)))
            return self.generated.CFlowMm(width=0.45)

        api.host.object_get_support_flow_mm = support_flow_mm
        api.host.object_get_region_flow_mm = region_flow_mm
        calls.clear()
        self.assertEqual(obj.support_flow_mm(7, 0.2, 0.15, flags).spacing, 0.44)
        self.assertEqual(obj.region_flow_mm(1, 9, 0.2, 2).width, 0.45)
        self.assertEqual(calls, [
            ("support_mm", 50, 7, 0.2, 0.15, flags),
            ("region_mm", 50, 1, 9, 0.2, 2),
        ])
        self.assertEqual(
            (self.generated.OBJECT_SUPPORT_FLOW_FIRST_LAYER, self.generated.OBJECT_SUPPORT_FLOW_BOTTOM_CONTACT,
             self.generated.OBJECT_SUPPORT_FLOW_BRIDGE),
            (1, 2, 4),
        )


if __name__ == "__main__":
    unittest.main()
