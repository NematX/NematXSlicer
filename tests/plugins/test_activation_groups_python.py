"""Check the Python solidarity wrapper against the generated C call contract.

The tests generate slic3r_api_generated.py into a temporary directory from the
current C headers. Native registry semantics are covered by C++ tests.
"""

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


class RecordingHost:
    def __init__(self, result=1):
        self.result = result
        self.calls = []

    def orchestrator_register_activation_group(self, orchestrator, group_id, members):
        self.calls.append((orchestrator.value, group_id, [ctypes.string_at(members.items[i]) for i in range(members.size)]))
        return self.result


class ActivationGroupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "activation_groups_test_api_generator", RUNTIME / "generate_slic3r_api.py"
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.api_module = importlib.import_module("slic3r_api")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def make_api(self, result=1):
        api = object.__new__(self.api_module.Slic3rAPI)
        api.orchestrator = ctypes.c_void_p(123)
        api.host = RecordingHost(result)
        return api

    def test_generated_signature(self):
        signature = next(s for s in self.generated.C_FUNCTION_SIGNATURES if s[0] == "orchestrator_register_activation_group")
        self.assertEqual(signature[1], ctypes.c_int32)
        self.assertEqual(signature[2][-1], self.generated.ConstStrings)

    def test_strings_and_generator_remain_alive_during_call(self):
        api = self.make_api()
        self.assertIsNone(api.register_activation_group("solid", (name for name in ["a", "b"])))
        self.assertEqual(api.host.calls, [(123, b"solid", [b"a", b"b"])])

    def test_repeated_declarations_are_forwarded_not_overwritten(self):
        api = self.make_api()
        api.register_activation_group("solid", ["a", "b"])
        api.register_activation_group("solid", ["b", "c"])
        self.assertEqual(len(api.host.calls), 2)
        self.assertEqual(api.host.calls[1][2], [b"b", b"c"])

    def test_host_rejection_raises(self):
        api = self.make_api(0)
        with self.assertRaisesRegex(ValueError, "solid"):
            api.register_activation_group("solid", ["a", "a"])


if __name__ == "__main__":
    unittest.main()
