"""Behavior tests for Python extrusion properties and regional setting helpers."""

import copy
import ctypes
import importlib
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock


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


class CustomPayload(ctypes.Structure):
    _fields_ = [("value", ctypes.c_uint32)]


class PropertyHost:
    def __init__(self):
        self.registration = None
        self.payloads = {}
        self.generic_payloads = {}
        self.children = {1: [2], 2: []}
        self.freed = []
        self.polyline_set = None

    def storage_free(self, storage, handle):
        self.freed.append((address(storage), address(handle)))
        return 1

    def extrusion_polyline_set_points(self, entity, points, count):
        self.polyline_set = (
            address(entity),
            ctypes.cast(points, ctypes.c_void_p).value,
            int(count),
        )
        return 1

    def orchestrator_register_property(self, orchestrator, name, byte_count, alignment):
        self.registration = (
            address(orchestrator),
            bytes(name),
            int(byte_count),
            int(alignment),
        )
        return 700

    def extrusion_is_loop(self, entity):
        return address(entity) == 1

    def extrusion_child_count(self, entity):
        return len(self.children.get(address(entity), []))

    def extrusion_child(self, entity, idx):
        return self.children[address(entity)][int(idx)]

    def extrusion_child_mutable(self, entity, idx):
        return self.extrusion_child(entity, idx)

    def extrusion_property_count(self, entity):
        return len(self.payloads.get(address(entity), {}))

    def extrusion_property_type_at(self, entity, idx):
        return list(self.payloads.get(address(entity), {}))[int(idx)]

    def extrusion_property_has(self, entity, property_type):
        return int(property_type) in self.payloads.get(address(entity), {})

    def extrusion_property_data(self, entity, property_type):
        payload = self.payloads.get(address(entity), {}).get(int(property_type))
        return 0 if payload is None else ctypes.addressof(payload)

    def extrusion_property_data_mutable(self, entity, property_type):
        return self.extrusion_property_data(entity, property_type)

    def extrusion_property_get_or_add_data_mutable(self, orchestrator, entity, property_type):
        payloads = self.payloads.setdefault(address(entity), {})
        payload = payloads.setdefault(int(property_type), CustomPayload())
        return ctypes.addressof(payload)

    def extrusion_property_remove(self, entity, property_type):
        return self.payloads.get(address(entity), {}).pop(int(property_type), None) is not None

    def plugin_property_data_size(self, properties, property_type):
        payload = self.generic_payloads.get(address(properties), {}).get(int(property_type))
        return 0 if payload is None else ctypes.sizeof(payload)

    def plugin_property_data(self, properties, property_type):
        payload = self.generic_payloads.get(address(properties), {}).get(int(property_type))
        return 0 if payload is None else ctypes.addressof(payload)

    def plugin_property_data_mutable(self, properties, property_type):
        return self.plugin_property_data(properties, property_type)

    def plugin_property_get_or_add_data_mutable(self, orchestrator, properties, property_type):
        payloads = self.generic_payloads.setdefault(address(properties), {})
        payload = payloads.setdefault(int(property_type), CustomPayload())
        return ctypes.addressof(payload)

    def plugin_property_has(self, properties, property_type):
        return int(property_type) in self.generic_payloads.get(address(properties), {})

    def plugin_property_remove(self, properties, property_type):
        return self.generic_payloads.get(address(properties), {}).pop(int(property_type), None) is not None


class FakeValue:
    def __init__(self, value):
        self.value = value

    def __eq__(self, other):
        return isinstance(other, FakeValue) and self.value == other.value


class FakePolylineView:
    def __init__(self, view):
        self._view = view

    def view(self):
        return self._view


class FakeArea:
    def __init__(self, contains):
        self._contains = contains

    def contains(self, _point):
        return self._contains


class FakeRegion:
    def __init__(self, value, contains):
        self.value = value
        self._areas = [FakeArea(contains)]

    def print_region(self):
        return self

    def config(self):
        return self

    def slices(self):
        return self._areas


class FakeRegionIsland:
    def __init__(self, regions):
        self._regions = regions

    def region_count(self):
        return len(self._regions)

    def regions(self):
        return iter(self._regions)


class PythonExtrusionRegionViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "extrusion_region_test_api_generator", RUNTIME / "generate_slic3r_api.py"
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.properties = importlib.import_module("slic3r_property_key")
        cls.extrusions = importlib.import_module("slic3r_extrusion_views")
        cls.datatree = importlib.import_module("slic3r_datatree_views")
        cls.regions = importlib.import_module("slic3r_region_settings_views")
        cls.run_context = importlib.import_module("slic3r_run_context")
        cls.perimeter = importlib.import_module("steps.perimeter")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def make_api(self):
        return type(
            "Api",
            (),
            {"host": PropertyHost(), "orchestrator": ctypes.c_void_p(77)},
        )()

    def test_loop_uses_complete_native_predicate(self):
        api = self.make_api()
        self.assertTrue(self.extrusions.ExtrusionEntity(api, 1).is_loop())
        self.assertFalse(self.extrusions.ExtrusionEntity(api, 2).is_loop())

    def test_stored_extrusion_entity_has_unique_ownership(self):
        api = self.make_api()
        entity = self.extrusions.StoredExtrusionEntity.adopt_owned(api, 12, 901)

        for copier in (copy.copy, copy.deepcopy):
            with self.assertRaisesRegex(
                TypeError, "StoredExtrusionEntity.*unique native resource"
            ):
                copier(entity)
        self.assertEqual(api.host.freed, [])
        self.assertEqual(entity.address, 901)
        self.assertTrue(entity.free_from_storage())
        self.assertEqual(api.host.freed, [(12, 901)])

    def test_set_polyline_borrows_the_source_point_array(self):
        api = self.make_api()
        points = (self.generated.CPoint * 3)(
            self.generated.CPoint(0, 0),
            self.generated.CPoint(1000000, 5),
            self.generated.CPoint(2000000, 0),
        )
        view = self.generated.MultipointConstView(points, 3, 0)
        entity = self.extrusions.MutableExtrusionEntity(api, 42)

        self.assertTrue(entity.set_polyline(FakePolylineView(view)))

        # The wrapper must forward the borrowed array, not a rebuilt copy.
        self.assertEqual(api.host.polyline_set, (42, ctypes.addressof(points), 3))
    def test_linear_points_use_the_two_call_size_query(self):
        api = self.make_api()
        linear = [(0, 0), (500, 120), (1000, 0)]
        calls = []

        def copy_linear_points(entity, dst, capacity):
            calls.append((address(entity), dst is None, int(capacity)))
            if dst is not None:
                for idx, (x, y) in enumerate(linear):
                    dst[idx] = self.generated.CPoint(x, y)
            return len(linear)

        api.host.extrusion_polyline_copy_linear_points = copy_linear_points
        points = self.extrusions.ExtrusionEntity(api, 7).linear_points()
        self.assertEqual([(point.x, point.y) for point in points], linear)
        self.assertEqual(calls, [(7, True, 0), (7, False, 3)])

    def test_dynamic_property_key_is_typed_and_orchestrator_bound(self):
        api = self.make_api()
        key = self.properties.PluginPropertyKey.register_dynamic(
            api, "example.custom", CustomPayload
        )
        self.assertEqual(api.host.registration, (77, b"example.custom", 4, 4))

        entity = self.extrusions.MutableExtrusionEntity(api, 1)
        mutable = key.get_or_add(entity)
        mutable.value = 42
        copied = key.get(entity)
        self.assertEqual(copied.value, 42)
        copied.value = 9
        self.assertEqual(key.get(entity).value, 42)

        other_api = self.make_api()
        other_api.host.payloads[1] = {700: CustomPayload(99)}
        foreign_entity = self.extrusions.MutableExtrusionEntity(other_api, 1)
        for access in (
            lambda: foreign_entity.has_property(key),
            lambda: foreign_entity.property_data_address(key),
            lambda: foreign_entity.property(key),
            lambda: foreign_entity.property_data_mutable_address(key),
            lambda: foreign_entity.property_mutable(key),
            lambda: foreign_entity.get_or_add_property_data_mutable_address(key),
            lambda: foreign_entity.remove_property(key),
            lambda: foreign_entity.store_property_data_aligned(
                key, other_api.host.payloads[1][700], "value", b"x"
            ),
        ):
            with self.subTest(access=access):
                with self.assertRaises(ValueError):
                    access()
        self.assertIn(700, other_api.host.payloads[1])
        self.assertTrue(foreign_entity.remove_property(700))

        properties = self.datatree.PluginProperties(api, 80)
        key.get_or_add(properties).value = 73
        self.assertTrue(key.has(properties))
        self.assertEqual(key.get(properties).value, 73)
        self.assertTrue(key.remove(properties))

        other_api.host.generic_payloads[80] = {700: CustomPayload(88)}
        foreign_properties = self.datatree.PluginProperties(other_api, 80)
        for access in (
            lambda: foreign_properties.has(key),
            lambda: foreign_properties.data_size(key),
            lambda: foreign_properties.data_address(key),
            lambda: foreign_properties.data_mutable_address(key),
            lambda: foreign_properties.get(key),
            lambda: foreign_properties.get_mutable(key),
            lambda: foreign_properties.get_or_add(key),
            lambda: foreign_properties.remove(key),
        ):
            with self.subTest(access=access):
                with self.assertRaises(ValueError):
                    access()
        self.assertIn(700, other_api.host.generic_payloads[80])
        self.assertTrue(foreign_properties.remove(700))

    def test_public_visitor_tracks_depth_and_inherited_typed_property(self):
        api = self.make_api()
        key = self.properties.PluginPropertyKey.register_dynamic(
            api, "example.inherited", CustomPayload
        )
        key.get_or_add(self.extrusions.MutableExtrusionEntity(api, 1)).value = 12

        class Visitor(self.extrusions.ExtrusionTreeConstVisitor):
            def __init__(self):
                super().__init__()
                self.seen = []

            def visit_leaf(self, entity):
                self.seen.append((self.depth(), self.current_property(key).value))

        visitor = Visitor()
        self.assertIsInstance(visitor, self.extrusions.ExtrusionTreeVisitorBase)
        visitor.traverse(self.extrusions.ExtrusionEntity(api, 1))
        self.assertEqual(visitor.seen, [(1, 12)])

    def test_layer_adhesion_helpers_support_modern_and_legacy_markers(self):
        class Properties:
            def __init__(self, adhesion=None, legacy=False):
                self.adhesion = adhesion
                self.legacy = legacy

            def get(self, key):
                if key.property_type == self.datatree.LayerAdhesionProperty.property_type:
                    return self.adhesion
                return None

            def has(self, key):
                return self.legacy and (
                    key.property_type == self.datatree.LayerBrimProperty.property_type
                )

        class Layer:
            def __init__(self, properties):
                self._properties = properties

            def properties(self):
                return self._properties

        properties_type = Properties
        properties_type.datatree = self.datatree
        adhesion = self.datatree.LayerAdhesionProperty(
            self.generated.RAW_LAYER_ADHESION_KIND_SKIRT,
            self.generated.RAW_LAYER_ADHESION_FLAG_FIRST_LAYER_ONLY,
        )
        layer = Layer(Properties(adhesion))
        self.assertTrue(self.datatree.layer_is_skirt_first_layer_only(layer))
        self.assertFalse(self.datatree.layer_is_normal_skirt(layer))
        self.assertFalse(self.datatree.layer_is_brim(layer))
        self.assertTrue(self.datatree.layer_is_brim(Layer(Properties(legacy=True))))

    def test_point_lookup_covers_uniform_outside_found_and_ambiguous(self):
        def create_value(_cls, _default, actual, _keys):
            return FakeValue(actual.value)

        lookup = self.regions.RegionSettings.lookup_at_point
        with mock.patch.object(
            self.regions.RegionSettingsValue,
            "create",
            classmethod(create_value),
        ):
            uniform = lookup(
                FakeRegionIsland([FakeRegion(1, False), FakeRegion(1, False)]),
                ["speed"],
                (0, 0),
            )
            self.assertTrue(uniform.found())
            self.assertTrue(uniform.uniform)

            outside = lookup(
                FakeRegionIsland([FakeRegion(1, False), FakeRegion(2, False)]),
                ["speed"],
                (0, 0),
            )
            self.assertEqual(outside.status, self.regions.RegionSettingsPointStatus.Outside)

            found = lookup(
                FakeRegionIsland([FakeRegion(1, True), FakeRegion(2, False)]),
                ["speed"],
                (0, 0),
            )
            self.assertEqual(found.value.value, 1)

            ambiguous = lookup(
                FakeRegionIsland([FakeRegion(1, True), FakeRegion(2, True)]),
                ["speed"],
                (0, 0),
            )
            self.assertEqual(
                ambiguous.status,
                self.regions.RegionSettingsPointStatus.Ambiguous,
            )

    def test_perimeter_shortcuts_use_current_island_and_internal_flow(self):
        api = type("Api", (), {})()
        generation = self.perimeter.PerimeterGenerationContextView.__new__(
            self.perimeter.PerimeterGenerationContextView
        )
        generation.api = api
        flow_calls = []

        class Region:
            def flow(self, role):
                flow_calls.append(role)
                return "flow"

        class Island:
            def region_count(self):
                return 1

            def region(self, idx):
                self.last_index = idx
                return Region()

        island = Island()
        generation.perimeter_generation_plugin_storage = lambda: 55
        generation.perimeter_generation_island = lambda: island

        with mock.patch.object(self.perimeter, "RegionSettings") as settings_cls:
            self.assertIs(
                generation.region_settings([["speed"]]),
                settings_cls.return_value,
            )
            settings_cls.assert_called_once_with(api, 55, island, [["speed"]])

        self.assertEqual(generation.perimeter_flow(), "flow")
        self.assertEqual(
            flow_calls,
            [self.generated.RAW_EXTRUSION_ROLE_INTERNAL_PERIMETER],
        )

    def test_perimeter_callback_defers_plugin_cancellation_until_native_return(self):
        errors = []
        native_events = []
        generation = self.generated.PerimeterGenerationContext()
        node = self.generated.PerimeterNode()

        @self.generated.PLUGIN_REPORT
        def report_error(_host_context, message):
            errors.append(message.decode("utf-8"))

        @self.generated.PERIMETER_RUN_REGION_GROUP
        def run_region_group(
            _payload,
            _regions,
            _region_count,
            _root_area,
            generator_state,
            generate_node,
        ):
            result = generate_node(
                generator_state,
                ctypes.pointer(generation),
                ctypes.pointer(node),
                ctypes.c_void_p(31),
                ctypes.c_void_p(32),
            )
            native_events.append(("callback returned", int(result)))
            return 73

        common = self.generated.PluginRunContext()
        common.report_error = report_error
        payload = self.generated.RunCtxGeneratePerimeter()
        payload.run_region_group = run_region_group
        context = self.perimeter.PerimeterContext(
            types.SimpleNamespace(host=types.SimpleNamespace()),
            common,
            ctypes.pointer(payload),
        )

        def cancel_provider(_state, _generation, _node, _inner, _inner_fill):
            raise self.run_context.PluginCancelled("provider cancelled")

        with self.assertRaisesRegex(
            self.run_context.PluginCancelled, "provider cancelled"
        ):
            context.run_region_group([], None, object(), cancel_provider)

        self.assertEqual(native_events, [("callback returned", 0)])
        self.assertEqual(errors, [])
        self.assertEqual(context._states, {})
        self.assertEqual(context._callbacks, [])

    def test_perimeter_callback_propagates_native_cancellation_after_return(self):
        cancelled = False
        native_events = []
        generation = self.generated.PerimeterGenerationContext()
        node = self.generated.PerimeterNode()

        @self.generated.PLUGIN_IS_CANCELLED
        def is_cancelled(_host_context):
            return int(cancelled)

        @self.generated.PERIMETER_RUN_REGION_GROUP
        def run_region_group(
            _payload,
            _regions,
            _region_count,
            _root_area,
            generator_state,
            generate_node,
        ):
            nonlocal cancelled
            result = generate_node(
                generator_state,
                ctypes.pointer(generation),
                ctypes.pointer(node),
                ctypes.c_void_p(31),
                ctypes.c_void_p(32),
            )
            native_events.append(("callback returned", int(result)))
            cancelled = True
            return 0

        common = self.generated.PluginRunContext()
        common.host_context = ctypes.c_void_p(1)
        common.is_cancelled = is_cancelled
        payload = self.generated.RunCtxGeneratePerimeter()
        payload.run_region_group = run_region_group
        context = self.perimeter.PerimeterContext(
            types.SimpleNamespace(host=types.SimpleNamespace()),
            common,
            ctypes.pointer(payload),
        )

        with self.assertRaises(self.run_context.PluginCancelled):
            context.run_region_group([], None, object(), lambda *_args: True)

        self.assertEqual(native_events, [("callback returned", 1)])
        self.assertEqual(context._states, {})
        self.assertEqual(context._callbacks, [])


if __name__ == "__main__":
    unittest.main()
