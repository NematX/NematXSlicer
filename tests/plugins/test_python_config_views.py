"""Behavior tests for the high-level Python configuration views."""

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
    return int(handle or 0)


class ConfigHost:
    def __init__(self, generated):
        self.generated = generated
        self.options = {
            11: {"kind": "bool", "values": [True], "enabled": [True]},
            12: {"kind": "ints", "values": [10, 20], "enabled": [True, False]},
            13: {"kind": "float", "values": [2.5], "enabled": [False]},
            14: {"kind": "string", "values": ["hello"], "enabled": [True]},
            15: {"kind": "strings", "values": ["left", "right"], "enabled": [True, True]},
            16: {"kind": "points", "values": [1.5, 2.5, 3.5, 4.5], "size": 2, "enabled": [True, False]},
            17: {"kind": "graph", "values": [71], "enabled": [True]},
            18: {
                "kind": "floats_or_percents",
                "values": [
                    generated.CFloatOrPercent(50.0, 1),
                    generated.CFloatOrPercent(4.0, 0),
                ],
                "enabled": [True, False],
            },
            19: {"kind": "ints", "values": [], "enabled": []},
            20: {"kind": "point", "values": [1.5, 2.5], "size": 1, "enabled": [True]},
        }
        self.keys = {
            b"enabled_bool": 11,
            b"vector_int": 12,
            b"disabled_float": 13,
            b"string": 14,
            b"strings": 15,
            b"points": 16,
            b"graph": 17,
            b"float_or_percent": 18,
            b"empty": 19,
            b"point": 20,
        }

    def _option(self, handle):
        return self.options[address(handle)]

    def config_get(self, config, key):
        return self.keys.get(bytes(key), 0)

    def config_option_size(self, option):
        record = self._option(option)
        return record.get("size", len(record["values"]))

    def config_option_type_get(self, option):
        record = self._option(option)
        return {
            "bool": self.generated.SLIC3R_CONFIG_OPTION_BOOL,
            "ints": self.generated.SLIC3R_CONFIG_OPTION_INTS,
            "float": self.generated.SLIC3R_CONFIG_OPTION_FLOAT,
            "string": self.generated.SLIC3R_CONFIG_OPTION_STRING,
            "strings": self.generated.SLIC3R_CONFIG_OPTION_STRINGS,
            "point": self.generated.SLIC3R_CONFIG_OPTION_POINT,
            "points": self.generated.SLIC3R_CONFIG_OPTION_POINTS,
            "graph": self.generated.SLIC3R_CONFIG_OPTION_GRAPH,
            "floats_or_percents": self.generated.SLIC3R_CONFIG_OPTION_FLOATS_OR_PERCENTS,
        }[record["kind"]]

    def config_option_is_vector(self, option):
        return bool(
            self.config_option_type_get(option)
            & self.generated.SLIC3R_CONFIG_OPTION_VECTOR_TYPE
        )

    def config_option_is_enabled(self, option, idx):
        enabled = self._option(option)["enabled"]
        return 0 <= int(idx) < len(enabled) and enabled[int(idx)]

    def config_option_get_bool(self, option, idx):
        return int(bool(self._option(option)["values"][int(idx)]))

    def config_option_get_int(self, option, idx):
        return int(self._option(option)["values"][int(idx)])

    def config_option_get_float(self, option, idx):
        return float(self._option(option)["values"][int(idx)])

    def config_option_get_float_or_percent(self, option, idx):
        return self._option(option)["values"][int(idx)]

    def config_option_get_string(self, option, idx, output, capacity):
        record = self._option(option)
        values = record["values"]
        selected_idx = 0 if record["kind"] == "string" else int(idx)
        if selected_idx >= len(values):
            return 0
        encoded = values[selected_idx].encode("utf-8")
        if output is not None and capacity:
            ctypes.memmove(output, encoded + b"\0", min(len(encoded) + 1, capacity))
        return len(encoded)

    def config_option_get_graph(self, option, idx):
        record = self._option(option)
        if record["kind"] != "graph":
            return 0
        return record["values"][0]

    def config_option_serialize(self, option, output, capacity):
        record = self._option(option)
        encoded = (
            b"0:3:1:1.25x2.5:3.75x4.5:8x9.25"
            if record["kind"] == "graph"
            else b""
        )
        if output is not None and capacity:
            ctypes.memmove(output, encoded + b"\0", min(len(encoded) + 1, capacity))
        return len(encoded)

    def config_option_vector_cast_mutable(self, option):
        return option if self.config_option_is_vector(option) else 0

    def config_option_vector_deserialize_at(self, option, idx, text):
        record = self._option(option)
        index = int(idx)
        if index < 0 or index >= len(record["values"]):
            return 0
        try:
            value = int(bytes(text))
        except ValueError:
            return 0
        record["values"][index] = value
        return 1

    def c_float_or_percent_get_effective_value(self, value, ratio):
        typed = ctypes.cast(
            value, ctypes.POINTER(self.generated.CFloatOrPercent)
        ).contents
        return float(ratio) * typed.value / 100.0 if typed.percent else typed.value

    def config_get_computed_value(self, config, key, extruder_id, output):
        if bytes(key) != b"computed" or int(extruder_id) != 2:
            return 0
        ctypes.cast(output, ctypes.POINTER(ctypes.c_double)).contents.value = 9.25
        return 1

    def graph_data_size(self, graph):
        return 3 if address(graph) == 71 else 0

    def graph_data_get_point(self, graph, index, output):
        points = ((1.25, 2.5), (3.75, 4.5), (8.0, 9.25))
        if address(graph) != 71 or int(index) >= len(points):
            return 0
        point = ctypes.cast(
            output, ctypes.POINTER(self.generated.CVec2d)
        ).contents
        point.x, point.y = points[int(index)]
        return 1

    def graph_data_type(self, graph):
        if address(graph) != 71:
            return self.generated.RAW_GRAPH_TYPE_NONE
        return self.generated.RAW_GRAPH_TYPE_LINEAR

    def graph_interpolate(self, graph, x):
        return float(x) + 1.0

    def graph_inverse_interpolate(self, graph, y):
        return float(y) - 1.0

    def graph_validate(self, graph):
        return address(graph) == 71

class PythonConfigViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "config_test_api_generator", RUNTIME / "generate_slic3r_api.py"
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.views = importlib.import_module("slic3r_datatree_views")
        cls.api_module = importlib.import_module("slic3r_api")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def make_config(self):
        api = type("Api", (), {})()
        api.host = ConfigHost(self.generated)
        return self.views.Config(api, 1)

    def test_option_serialization_failure_is_not_an_empty_value(self):
        """Check RuntimeError for the 0xFFFFFFFF failure result or changed lengths, and preserve valid empty text."""
        for results in ([0xFFFFFFFF], [5, 0xFFFFFFFF]):
            with self.subTest(results=results):
                config = self.make_config()
                responses = iter(results)
                config.api.host.config_option_serialize = lambda *args: next(responses)
                option = self.views.ConfigOption(config.api, 14)
                with self.assertRaises(RuntimeError) as raised:
                    option.serialize()
                self.assertEqual(str(raised.exception), "The configuration option could not be serialized")
        for results in ([5, 6], [0, 1]):
            with self.subTest(results=results):
                config = self.make_config()
                responses = iter(results)
                config.api.host.config_option_serialize = lambda *args: next(responses)
                option = self.views.ConfigOption(config.api, 14)
                with self.assertRaises(RuntimeError) as raised:
                    option.serialize()
                self.assertEqual(str(raised.exception), "The configuration option changed while it was being serialized")
        config = self.make_config()
        config.api.host.config_option_serialize = lambda *args: 0
        self.assertEqual(self.views.ConfigOption(config.api, 14).serialize(), "")

    def test_vector_item_serialization_failure_is_not_an_empty_value(self):
        """Check RuntimeError for the 0xFFFFFFFF failure result or changed item lengths, and preserve valid empty text."""
        for results in ([0xFFFFFFFF], [5, 0xFFFFFFFF]):
            with self.subTest(results=results):
                config = self.make_config()
                responses = iter(results)
                config.api.host.config_option_vector_cast = lambda option: option
                config.api.host.config_option_vector_serialize_at = lambda *args: next(responses)
                option = self.views.ConfigOption(config.api, 12)
                with self.assertRaises(RuntimeError) as raised:
                    option.vector_serialize_at(0)
                self.assertEqual(str(raised.exception), "The configuration option could not be serialized")
        for results in ([5, 6], [0, 1]):
            with self.subTest(results=results):
                config = self.make_config()
                responses = iter(results)
                config.api.host.config_option_vector_cast = lambda option: option
                config.api.host.config_option_vector_serialize_at = lambda *args: next(responses)
                option = self.views.ConfigOption(config.api, 12)
                with self.assertRaises(RuntimeError) as raised:
                    option.vector_serialize_at(0)
                self.assertEqual(str(raised.exception), "The configuration option changed while it was being serialized")
        config = self.make_config()
        config.api.host.config_option_vector_cast = lambda option: option
        config.api.host.config_option_vector_serialize_at = lambda *args: 0
        self.assertEqual(self.views.ConfigOption(config.api, 12).vector_serialize_at(0), "")

    def test_typed_string_point_and_graph_accessors(self):
        config = self.make_config()

        self.assertEqual(config.option("string").get_string(), "hello")
        self.assertEqual(config.option("string").get_string(42), "hello")
        self.assertEqual(
            config.option("points").get_point(1),
            self.views.ConfigPoint(3.5, 4.5),
        )
        with self.assertRaises(IndexError):
            config.option("points").get_point(-1)
        with self.assertRaises(IndexError):
            config.option("points").get_point(2)

        graph = config.option("graph").graph()
        self.assertIsNotNone(config.option("graph").graph(42))
        self.assertIsNotNone(graph)
        self.assertEqual(graph.size(), 3)
        self.assertEqual(graph.point(1), self.views.ConfigPoint(3.75, 4.5))
        self.assertEqual(
            graph.points(),
            (
                self.views.ConfigPoint(1.25, 2.5),
                self.views.ConfigPoint(3.75, 4.5),
                self.views.ConfigPoint(8.0, 9.25),
            ),
        )
        self.assertEqual(graph.type(), self.generated.RAW_GRAPH_TYPE_LINEAR)
        with self.assertRaises(IndexError):
            graph.point(-1)
        with self.assertRaises(IndexError):
            graph.point(3)
        self.assertEqual(graph.interpolate(2.0), 3.0)
        self.assertEqual(graph.inverse_interpolate(3.0), 2.0)
        self.assertTrue(graph.validate())
        self.assertEqual(
            config.option("graph").serialize(),
            "0:3:1:1.25x2.5:3.75x4.5:8x9.25",
        )

    def test_vector_deserialize_at_replaces_one_existing_item(self):
        config = self.make_config()
        mutable = self.views.MutableConfigOption(config.api, 12)

        self.assertTrue(mutable.vector_deserialize_at(1, "73"))
        self.assertEqual(config.option("vector_int").get_int(1), 73)
        self.assertFalse(mutable.vector_deserialize_at(2, "19"))
        self.assertFalse(mutable.vector_deserialize_at(0, "invalid"))
        self.assertEqual(config.option("vector_int").get_int(0), 10)

    def test_scalar_helpers_filter_missing_disabled_and_empty_options(self):
        config = self.make_config()

        self.assertTrue(config.has("enabled_bool"))
        self.assertFalse(config.has("missing"))
        self.assertTrue(config.bool_or_default("enabled_bool", False))
        self.assertEqual(config.float_or_default("disabled_float", 8.0), 8.0)
        self.assertEqual(config.int_or_default("empty", 7), 7)
        self.assertEqual(config.string_or_default("missing", "fallback"), "fallback")
        self.assertEqual(
            config.point_or_default("missing", self.views.ConfigPoint(8.0, 9.0)),
            self.views.ConfigPoint(8.0, 9.0),
        )

        self.assertEqual(config.float("disabled_float", default=6.0), 6.0)
        self.assertEqual(config.int("vector_int", 3, 5), 5)
        self.assertEqual(config.float("point", 1, -1.0), 2.5)
        self.assertEqual(config.float("point", 2, -1.0), -1.0)
        self.assertEqual(config.float("points", 1, -1.0), 2.5)
        self.assertEqual(config.float("points", 2, -1.0), -1.0)
        self.assertEqual(config.float("points", 3, -1.0), -1.0)
        self.assertEqual(config.float("points", 4, -1.0), -1.0)

    def test_vector_helpers_filter_indices_and_disabled_items(self):
        config = self.make_config()

        self.assertEqual(config.vector_int_or_default("vector_int", 0, 99), 10)
        self.assertEqual(config.vector_int_or_default("vector_int", 1, 99), 99)
        self.assertEqual(config.vector_int_or_default("vector_int", -1, 99), 99)
        self.assertEqual(config.vector_enum_or_default("vector_int", 0, 99), 10)
        self.assertEqual(config.vector_string_or_default("strings", 1, "x"), "right")
        self.assertEqual(
            config.vector_point_or_default(
                "points", 0, self.views.ConfigPoint(8.0, 9.0)
            ),
            self.views.ConfigPoint(1.5, 2.5),
        )

        fallback = self.generated.CFloatOrPercent(12.0, 0)
        selected = config.vector_float_or_percent_or_default(
            "float_or_percent", 0, fallback
        )
        self.assertEqual((selected.value, selected.percent), (50.0, 1))
        disabled = config.vector_float_or_percent_or_default(
            "float_or_percent", 1, fallback
        )
        self.assertEqual((disabled.value, disabled.percent), (12.0, 0))
        self.assertIsNot(disabled, fallback)
        self.assertEqual(
            config.vector_effective_float_or_percent_or_default(
                "float_or_percent", 0, 20.0, -1.0
            ),
            10.0,
        )
        self.assertEqual(config.percent_or_default("disabled_float", 3.0), 3.0)

    def test_computed_and_graph_fallbacks(self):
        config = self.make_config()
        graph = config.graph_or_default("graph", None)

        self.assertIsNotNone(graph)
        self.assertEqual(config.computed_float_or_default("computed", 2, -1.0), 9.25)
        self.assertEqual(config.computed_float_or_default("computed", 1, -1.0), -1.0)
        self.assertIs(config.graph_or_default("missing", graph), graph)
        self.assertIs(config.vector_graph_or_default("graph", 3, graph), graph)
        self.assertIsNone(config.graph_or_default("enabled_bool", graph))

    def test_legacy_api_shortcuts_use_the_same_enabled_item_rules(self):
        api = self.api_module.Slic3rAPI.__new__(self.api_module.Slic3rAPI)
        api.host = ConfigHost(self.generated)

        self.assertTrue(api.config_bool(1, "enabled_bool"))
        self.assertEqual(api.config_float(1, "disabled_float"), 0.0)
        self.assertEqual(api.config_int(1, "vector_int", 0), 10)
        self.assertEqual(api.config_int(1, "vector_int", 1), 0)
        self.assertEqual(api.config_int(1, "vector_int", 5), 0)
        self.assertEqual(api.config_float(1, "point", 1), 2.5)
        self.assertEqual(api.config_float(1, "point", 2), 0.0)
        self.assertEqual(api.config_float(1, "points", 1), 2.5)
        self.assertEqual(api.config_float(1, "points", 2), 0.0)
        self.assertEqual(api.config_float(1, "points", 3), 0.0)
        self.assertEqual(api.config_float(1, "points", 4), 0.0)
        self.assertEqual(
            api.config_float_or_percent_effective(
                1, "float_or_percent", 20.0, 0
            ),
            10.0,
        )


if __name__ == "__main__":
    unittest.main()
