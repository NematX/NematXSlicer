"""Check that Python configuration-definition views expose immutable copied values.

Controlled definitions supply labels, enum entries and GUI metadata, including
file-path fields. Tests read them through the public wrappers and compare values
and container behavior without creating GUI widgets or changing the host schema.
The fixture owns the C-facing data; returned Python values must not depend on
temporary buffers that the fixture can replace after a query.

Representative direct calls (branches are not a timeline):

    test_orchestrator_and_api_facades_return_the_same_values()
        |-- self.make_api()
        |-- self.orchestrator_views.OrchestratorView()
        |-- orchestrator.option_keys()
        |-- orchestrator.option_definition()
        |-- self.api_module.Slic3rAPI.__new__()
        `-- public_api.option_keys()
"""

import ctypes
from dataclasses import FrozenInstanceError
import importlib
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
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


class ConfigDefinitionHost:
    """Model the native two-call catalogue and one replaceable TLS snapshot."""

    def __init__(self, generated):
        self.generated = generated
        self.keys = [
            (b"alpha", generated.RAW_CO_INT),
            (b"point3", generated.RAW_CO_POINT3),
            (b"zeta", generated.RAW_CO_STRING),
        ]
        self.snapshot_number = 0
        self.backing = []

    def orchestrator_get_option_keys(self, orchestrator, type_filter, output, capacity):
        if not getattr(orchestrator, "value", orchestrator):
            return 0
        selected = [
            entry
            for entry in self.keys
            if int(type_filter) == self.generated.RAW_CO_NONE
            or entry[1] == int(type_filter)
        ]
        if output:
            for index, (key, option_type) in enumerate(selected[: int(capacity)]):
                output[index].key = key
                output[index].type = option_type
        return len(selected)

    def _strings(self, values):
        encoded = [value.encode("utf-8") for value in values]
        array = (ctypes.c_char_p * len(encoded))(*encoded)
        self.backing.extend([encoded, array])
        return self.generated.ConstStrings(
            ctypes.cast(array, ctypes.POINTER(ctypes.c_void_p)), len(encoded)
        )

    def orchestrator_get_option_def(self, orchestrator, key, output):
        raw = output._obj
        ctypes.memset(ctypes.addressof(raw), 0, ctypes.sizeof(raw))
        if not getattr(orchestrator, "value", orchestrator) or key != b"alpha":
            return 0

        self.snapshot_number += 1
        self.backing = []
        suffix = str(self.snapshot_number)
        raw.opt_key = b"alpha"
        raw.type = self.generated.RAW_CO_FLOAT
        raw.category = self.generated.RAW_OPTION_CATEGORY_ADVANCED
        raw.level = self.generated.RAW_OPTION_LEVEL_EXPERT
        raw.container_type = self.generated.RAW_CONTAINER_TYPE_PROJECT
        raw.option_preset_type = self.generated.RAW_PRESET_TYPE_FFF_PRINT
        raw.gui_type = self.generated.RAW_GUI_TYPE_F_ENUM_OPEN
        raw.printer_technology = self.generated.RAW_PT_FFF
        raw.invalidates_step = self.generated.STEP_INFILL
        raw.is_optional = 1
        raw.gui_flags = b"serialized"
        raw.label = f"Label {suffix}".encode("utf-8")
        raw.full_label = b"Full label"
        raw.tooltip = b"Tooltip"
        raw.translation_domain = b"Tests"
        raw.sidetext = b"mm"
        raw.cli = b"alpha=f"
        raw.ratio_over = b"nozzle_diameter"
        raw.multiline = 1
        raw.full_width = 1
        raw.is_code = 1
        raw.is_vector_extruder = 1
        raw.readonly = 1
        raw.can_phony = 1
        raw.can_be_disabled = 1
        raw.height = 7
        raw.width = 19
        raw.label_width = 23
        raw.aligned_label_left = 1
        raw.sidetext_width = 11
        raw.min_value = -2.5
        raw.max_value = 8.5
        raw.has_min = 1
        raw.has_max = 1
        raw.max_literal_value = 4.5
        raw.max_literal_is_percent = 1
        raw.has_max_literal = 1
        raw.precision = 4
        raw.mode = self.generated.RAW_CONFIG_OPTION_MODE_EXPERT
        raw.flags = 0x1020304050607080
        raw.aliases = self._strings(["old_alpha", f"old_alpha_{suffix}"])
        raw.shortcut = self._strings(["perimeter_speed"])
        raw.depends_on = self._strings(["layer_height#0"])
        raw.default_serialized_value = b"1.25"

        pairs = (self.generated.KeyValueStringPair * 2)(
            self.generated.KeyValueStringPair(b"0.5", b"Half"),
            self.generated.KeyValueStringPair(b"1.25", b"Nominal"),
        )
        self.backing.append(pairs)
        raw.enum_def.value_label_pairs = self.generated.KeyValueStringPairArray(pairs, 2)

        allowed_types = (ctypes.c_int * 2)(
            self.generated.RAW_GRAPH_TYPE_LINEAR,
            self.generated.RAW_GRAPH_TYPE_SPLINE,
        )
        reset_points = (self.generated.CVec2d * 2)(
            self.generated.CVec2d(0.0, 10.0),
            self.generated.CVec2d(100.0, 20.0),
        )
        enforced_values = (self.generated.CVec2d * 1)(
            self.generated.CVec2d(0.0, 10.0),
        )
        graph_settings = self.generated.RawGraphSettings()
        graph_settings.title = b"Graph title"
        graph_settings.description = b"Graph description"
        graph_settings.x_label = b"Position"
        graph_settings.y_label = b"Value"
        graph_settings.min_x = 0.0
        graph_settings.max_x = 100.0
        graph_settings.step_x = 1.0
        graph_settings.min_y = 0.0
        graph_settings.max_y = 50.0
        graph_settings.step_y = 0.5
        graph_settings.allowed_types = allowed_types
        graph_settings.allowed_type_count = 2
        graph_settings.reset_values = self.generated.RawGraphData(
            0,
            2,
            self.generated.RAW_GRAPH_TYPE_LINEAR,
            reset_points,
            2,
        )
        graph_settings.enforced_values = enforced_values
        graph_settings.enforced_value_count = 1
        graph_settings_pointer = ctypes.pointer(graph_settings)
        self.backing.extend(
            [allowed_types, reset_points, enforced_values, graph_settings, graph_settings_pointer]
        )
        raw.graph_settings = graph_settings_pointer
        return 1


class PythonConfigDefinitionViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "config_definition_test_api_generator", RUNTIME / "generate_slic3r_api.py"
        )
        cls.generator = generator
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.views = importlib.import_module("slic3r_config_def_views")
        cls.orchestrator_views = importlib.import_module("slic3r_orchestrator_views")
        cls.api_module = importlib.import_module("slic3r_api")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def make_api(self):
        host = ConfigDefinitionHost(self.generated)
        api = types.SimpleNamespace(host=host, orchestrator=ctypes.c_void_p(10))
        return api, host

    def test_catalogue_is_ordered_filtered_and_handles_capacity_growth(self):
        api, host = self.make_api()
        keys = self.views.option_keys(api)
        self.assertEqual([entry.key for entry in keys], ["alpha", "point3", "zeta"])
        self.assertEqual(
            self.views.option_keys(api, self.generated.RAW_CO_POINT3),
            (self.views.ConfigOptionKey("point3", self.generated.RAW_CO_POINT3),),
        )

        original = host.orchestrator_get_option_keys
        calls = 0

        def growing(orchestrator, type_filter, output, capacity):
            nonlocal calls
            calls += 1
            if calls == 2:
                host.keys.insert(1, (b"beta", self.generated.RAW_CO_BOOL))
            return original(orchestrator, type_filter, output, capacity)

        host.orchestrator_get_option_keys = growing
        self.assertEqual(
            [entry.key for entry in self.views.option_keys(api)],
            ["alpha", "beta", "point3", "zeta"],
        )

    def test_definition_is_a_complete_detached_immutable_value(self):
        api, host = self.make_api()
        first = self.views.option_definition(api, "alpha")
        self.assertIsNotNone(first)
        self.assertEqual(first.label, "Label 1")
        self.assertEqual(first.aliases, ("old_alpha", "old_alpha_1"))
        self.assertEqual(first.enum_entries[1].label, "Nominal")
        self.assertEqual(first.min_value, -2.5)
        self.assertEqual(first.max_value, 8.5)
        self.assertEqual(first.max_literal_value, 4.5)
        self.assertEqual(first.flags, 0x1020304050607080)
        self.assertEqual(first.graph_settings.title, "Graph title")
        self.assertEqual(
            first.graph_settings.allowed_types,
            (
                self.generated.RAW_GRAPH_TYPE_LINEAR,
                self.generated.RAW_GRAPH_TYPE_SPLINE,
            ),
        )
        self.assertEqual(first.graph_settings.reset_values.points[1], (100.0, 20.0))
        self.assertEqual(first.graph_settings.enforced_values, ((0.0, 10.0),))

        second = self.views.option_definition(api, "alpha")
        self.assertEqual(second.label, "Label 2")
        self.assertEqual(first.label, "Label 1")
        self.assertEqual(first.aliases, ("old_alpha", "old_alpha_1"))
        self.assertEqual(first.graph_settings.title, "Graph title")
        with self.assertRaises(FrozenInstanceError):
            first.label = "changed"
        self.assertIsNone(self.views.option_definition(api, "missing"))

    def test_gui_types_keep_their_numeric_values_and_the_path_types_come_last(self):
        """Check that the generated constants match the C++ enumeration and that the path types were appended."""
        self.assertEqual(
            (self.generated.RAW_GUI_TYPE_UNDEFINED, self.generated.RAW_GUI_TYPE_F_ENUM_OPEN,
             self.generated.RAW_GUI_TYPE_SELECT_CLOSE, self.generated.RAW_GUI_TYPE_SAVE_PATH,
             self.generated.RAW_GUI_TYPE_OPEN_PATH),
            (0, 2, 7, 8, 9),
        )

    def test_natures_copy_stable_numeric_values_from_raw(self):
        """Check literal withdrawal constants and copied named or unnamed combinations."""
        self.assertEqual(
            (self.generated.RAW_CONFIG_NATURE_NORMAL, self.generated.RAW_CONFIG_NATURE_LOCAL,
             self.generated.RAW_CONFIG_NATURE_BOOKKEEPING, self.generated.RAW_CONFIG_NATURE_PRESET_META,
             self.generated.RAW_CONFIG_NATURE_VOLATILE),
            (0, 6, 8, 4, 15),
        )
        raw = self.generated.RawConfigOptionDef()
        self.assertEqual(raw.nature, 0)
        for expected in (0, 6, 8, 4, 15, 10):
            with self.subTest(nature=expected):
                raw.nature = expected
                definition = self.views.ConfigOptionDefinition.from_raw(raw)
                raw.nature = 0
                self.assertEqual(definition.nature, expected)

    def test_create_option_def_passes_nature_to_raw(self):
        """Check withdrawal combinations reach the borrowed registration record."""
        class CreationHost:
            def orchestrator_create_option_def(self, orchestrator, definition):
                self.nature = definition._obj.nature
                return 0

        host = CreationHost()
        api = self.api_module.Slic3rAPI.__new__(self.api_module.Slic3rAPI)
        api.host = host
        api.orchestrator = ctypes.c_void_p(10)
        for expected in (0, 6, 8, 4, 15, 10):
            with self.subTest(nature=expected):
                api.create_option_def(opt_key="plugin.nature", nature=expected)
                self.assertEqual(host.nature, expected)

    def test_nullable_strings_distinguish_empty_from_absent(self):
        raw = self.generated.RawConfigOptionDef()
        raw.opt_key = b"empty"
        raw.label = b""
        raw.default_serialized_value = b""

        empty = self.views.ConfigOptionDefinition.from_raw(raw)
        self.assertEqual(empty.label, "")
        self.assertEqual(empty.default_serialized_value, "")

        raw.label = None
        raw.default_serialized_value = None
        absent = self.views.ConfigOptionDefinition.from_raw(raw)
        self.assertIsNone(absent.label)
        self.assertIsNone(absent.default_serialized_value)

    def test_orchestrator_and_api_facades_return_the_same_values(self):
        api, _host = self.make_api()
        orchestrator = self.orchestrator_views.OrchestratorView(api)
        self.assertEqual(orchestrator.option_keys()[0].key, "alpha")
        self.assertEqual(orchestrator.option_definition("alpha").label, "Label 1")

        public_api = self.api_module.Slic3rAPI.__new__(self.api_module.Slic3rAPI)
        public_api.host = api.host
        public_api.orchestrator = api.orchestrator
        self.assertEqual(public_api.option_keys()[1].key, "point3")
        self.assertEqual(public_api.option_definition("alpha").label, "Label 2")

    def test_create_option_def_passes_complete_graph_settings(self):
        generated = self.generated

        class CreationHost:
            def __init__(self):
                self.captured = None

            def orchestrator_create_option_def(self, orchestrator, definition):
                raw = definition._obj
                graph = raw.graph_settings.contents
                self.captured = (
                    raw.opt_key,
                    graph.title,
                    tuple(graph.allowed_types[index] for index in range(graph.allowed_type_count)),
                    tuple(
                        (graph.reset_values.points[index].x, graph.reset_values.points[index].y)
                        for index in range(graph.reset_values.point_count)
                    ),
                )
                return generated.OPTION_DEF_ERROR_OK

        allowed_types = (ctypes.c_int * 2)(
            generated.RAW_GRAPH_TYPE_LINEAR,
            generated.RAW_GRAPH_TYPE_SPLINE,
        )
        reset_points = (generated.CVec2d * 2)(
            generated.CVec2d(0.0, 1.0),
            generated.CVec2d(10.0, 2.0),
        )
        graph = generated.RawGraphSettings()
        graph.title = b"Creation graph"
        graph.allowed_types = allowed_types
        graph.allowed_type_count = 2
        graph.reset_values = generated.RawGraphData(
            0, 2, generated.RAW_GRAPH_TYPE_LINEAR, reset_points, 2
        )

        host = CreationHost()
        api = self.api_module.Slic3rAPI.__new__(self.api_module.Slic3rAPI)
        api.host = host
        api.orchestrator = ctypes.c_void_p(10)
        api.create_option_def(
            opt_key="plugin.graph",
            type=generated.RAW_CO_GRAPH,
            container_type=generated.RAW_CONTAINER_TYPE_PROJECT,
            option_preset_type=generated.RAW_PRESET_TYPE_FFF_PRINT,
            printer_technology=generated.RAW_PT_FFF,
            default_serialized_value="0:2:1:0x1:10x2",
            graph_settings=graph,
        )

        self.assertEqual(
            host.captured,
            (
                b"plugin.graph",
                b"Creation graph",
                (generated.RAW_GRAPH_TYPE_LINEAR, generated.RAW_GRAPH_TYPE_SPLINE),
                ((0.0, 1.0), (10.0, 2.0)),
            ),
        )

    def test_wrappers_declare_the_config_definition_contract(self):
        attribute = "__slic3r_abi_headers__"
        self.assertIn("slic3r_config_def.h", getattr(self.views.ConfigOptionDefinition, attribute))
        self.assertIn("slic3r_config_def.h", getattr(self.views.option_keys, attribute))
        self.assertIn(
            "slic3r_config_def.h",
            getattr(self.orchestrator_views.OrchestratorView.option_definition, attribute),
        )
        self.assertIn("slic3r_config_def.h", getattr(self.api_module.Slic3rAPI.option_keys, attribute))

    def test_host_binding_reports_every_missing_export_immediately(self):
        class BoundFunction:
            pass

        class PartialHost:
            present = BoundFunction()

        api = self.api_module.Slic3rAPI.__new__(self.api_module.Slic3rAPI)
        api.host = PartialHost()
        original = self.api_module.C_FUNCTION_SIGNATURES
        self.api_module.C_FUNCTION_SIGNATURES = [
            ("first_missing", None, []),
            ("present", None, []),
            ("second_missing", None, []),
        ]
        try:
            with self.assertRaisesRegex(
                RuntimeError, "first_missing, second_missing"
            ):
                api._bind_host_functions()
        finally:
            self.api_module.C_FUNCTION_SIGNATURES = original

    def test_removed_definition_handler_is_not_kept_as_an_opaque_type(self):
        self.assertNotIn("print_config_def_handler", self.generator.OPAQUE_TYPES)


if __name__ == "__main__":
    unittest.main()
