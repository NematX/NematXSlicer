"""Offline ABI packaging checks, independent of the platform DLL loader."""
import importlib.util
import json
from pathlib import Path
import sys
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT / "src/plugins_cpp"))
sys.path.insert(0, str(ROOT / "src/libslic3r/Api/plugin/python"))
import write_plugin_abi_manifest as manifest
from slic3r_abi import ABI_HEADERS_ATTRIBUTE, abi_requires_headers


def load_python_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class AbiPackagingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator_path = ROOT / "src/libslic3r/Api/plugin/python/generate_slic3r_api.py"
        analyzer_path = ROOT / "src/libslic3r/Api/plugin/python/generate_python_plugin_abi.py"
        cls.generator = load_python_module("abi_generator", generator_path)
        cls.analyzer = load_python_module("python_plugin_abi_generator", analyzer_path)

    def test_mesh_query_plain_types_callbacks_and_diagnostics_are_generated(self):
        import ctypes
        namespace = {}
        exec(self.generator.generate(), namespace)
        cancel = namespace["MeshQueryCancel"]
        self.assertEqual([field[0] for field in cancel._fields_],
                         ["context", "is_cancelled"])
        self.assertEqual(ctypes.sizeof(cancel), 2 * ctypes.sizeof(ctypes.c_void_p))
        callback_type = cancel._fields_[1][1]
        callback = callback_type(lambda context: 1)
        self.assertEqual(cancel(None, callback).is_cancelled(None), 1)
        self.assertEqual(ctypes.sizeof(namespace["MeshQueryDiagnostic"]), 512)
        self.assertEqual(ctypes.sizeof(namespace["MeshQueryRay"]), 64)
        catalog = self.generator.generate_abi_catalog()
        self.assertIn("slic3r_mesh_queries.h", catalog)
        self.assertIn("slic3r_geometry_types.h", catalog)

    def test_python_abi_decorator_preserves_and_composes_targets(self):
        def binding():
            return 42

        original = binding
        binding = abi_requires_headers(" first.h ", "first.h")(binding)
        binding = abi_requires_headers("steps/second.h")(binding)
        self.assertIs(binding, original)
        self.assertEqual(binding(), 42)
        self.assertEqual(getattr(binding, ABI_HEADERS_ATTRIBUTE), ("first.h", "steps/second.h"))

        @abi_requires_headers("class.h")
        class Binding:
            @abi_requires_headers("method.h")
            def method(self):
                return 7

        self.assertEqual(getattr(Binding, ABI_HEADERS_ATTRIBUTE), ("class.h",))
        self.assertEqual(getattr(Binding.method, ABI_HEADERS_ATTRIBUTE), ("method.h",))
        self.assertEqual(Binding().method(), 7)
        with self.assertRaises(ValueError):
            abi_requires_headers()
        with self.assertRaises(ValueError):
            abi_requires_headers("")

    def test_clear_callback_is_not_exposed(self):
        namespace = {}
        exec(self.generator.generate(), namespace)
        self.assertNotIn("PLUGIN_CLEAR", namespace)
        self.assertIn("PLUGIN_RUN", namespace)
        self.assertEqual(self.generator.parse_callbacks(
            "typedef int32_t (*plugin_clear_fn)(void *plugin_ctx, "
            "const print_handle *print, storage_handle *plugin_storage);"), [])
        catalog = json.loads(self.generator.generate_abi_catalog())
        self.assertNotIn("PLUGIN_CLEAR", catalog["python_symbols"])

    def test_error_id_contract(self):
        namespace = {}
        exec(self.generator.generate(), namespace)
        self.assertEqual(namespace["PLUGIN_ERROR_OK"], 0)
        self.assertEqual(namespace["PLUGIN_ERROR_GENERAL"], 1)
        catalog = json.loads(self.generator.generate_abi_catalog())
        for symbol in ("PLUGIN_ERROR_OK", "PLUGIN_ERROR_GENERAL"):
            self.assertEqual(catalog["python_symbols"][symbol], ["slic3r_plugin_types.h"])
        self.assertEqual(catalog["headers"]["slic3r_plugin_types.h"]["version"], [3, 1])

    def test_union_and_failed_generation_never_publish_partial_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binary, base, abi, output = (root / name for name in ("plugin.dll", "base.ini", "abi.ini", "version.ini"))
            binary.write_bytes(manifest.BEGIN + b"slic3r_plugin_types.h=1.0\nslic3r_extrusion_entity.h=0.0\n" + manifest.END)
            base.write_text("[plugin]\npackage_version=1.0.0\nslicer_version=999.0.0\n", encoding="utf-8")
            abi.write_text("[abi]\nslic3r_extrusion_entity.h=1.0\n", encoding="utf-8")
            command = [sys.executable, manifest.__file__, "--base", str(base), "--binary", str(binary),
                       "--abi", str(abi), "--output", str(output)]
            subprocess.run(command, check=True, capture_output=True)
            self.assertEqual(manifest.read_ini_abi(output), {
                "slic3r_plugin_types.h": (1, 0), "slic3r_extrusion_entity.h": (1, 0)})
            previous = output.read_bytes()
            binary.write_bytes(b"missing metadata")
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)
            self.assertEqual(output.read_bytes(), previous)

    def test_record_validation(self):
        good = manifest.BEGIN + b"slic3r_plugin_types.h=1u.0u\n" + manifest.END
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "plugin.dll"
            binary.write_bytes(b"not an executable" + good)
            self.assertEqual(manifest.read_binary_abi(binary), {"slic3r_plugin_types.h": (1, 0)})
            for bad in (b"", good + good, good[:-1], manifest.BEGIN + b"a.h=1.0\na.h=1.0\n" + manifest.END):
                binary.write_bytes(bad)
                with self.assertRaises(ValueError):
                    manifest.read_binary_abi(binary)

    def test_python_header_coverage(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "abi.ini"
            output.write_text(self.generator.generate_abi_manifest(), encoding="utf-8")
            versions = manifest.read_ini_abi(output)
            self.assertIn("slic3r_plugin_types.h", versions)
            for header in self.generator.header_paths():
                if re.search(r"^#define SLIC3R_PLUGIN_API_\w+_MAJOR\s+\d+", header.read_text(encoding="utf-8"), re.MULTILINE):
                    self.assertIn(header.relative_to(self.generator.HEADER_ROOT).as_posix(), versions)

            first = self.generator.generate_abi_catalog()
            self.assertEqual(first, self.generator.generate_abi_catalog())
            catalog = json.loads(first)
            self.assertEqual(catalog["headers"]["steps/slic3r_step_support.h"]["version"], [1, 3])
            self.assertEqual(catalog["headers"]["steps/slic3r_step_support_toolpath.h"]["version"], [1, 3])
            self.assertIn("steps/slic3r_step_support_toolpath.h", catalog["python_symbols"]["RunCtxSupportToolpath"])
            self.assertIn("slic3r_extrusion_entity.h", catalog["c_functions"]["extrusion_is_loop"])
            self.assertIn("slic3r_geometry.h", catalog["python_symbols"]["CPoint"])
            self.assertIn("slic3r_plugin_types.h", catalog["python_symbols"]["PLUGIN_SETUP"])
            self.assertIn("slic3r_slicing_step.h", catalog["python_symbols"]["STEP_ORDERING"])
            self.assertIn(
                "steps/slic3r_step_seam_placer.h",
                catalog["python_symbols"]["RawSeamPlacerInstance"],
            )
            self.assertIn("slic3r_gcode_firmware.h", catalog["headers"])
            self.assertIn("steps/slic3r_step_gcode_firmware.h", catalog["headers"])
            self.assertNotIn(
                "slic3r_gcode_thumbnail.h",
                catalog["headers"]["steps/slic3r_step_gcode.h"]["dependencies"],
            )
            self.assertEqual(
                catalog["headers"]["slic3r_extrusion_entity.h"]["dependencies"],
                sorted(catalog["headers"]["slic3r_extrusion_entity.h"]["dependencies"]))

    def test_high_level_helpers_declare_their_native_contracts(self):
        runtime = ROOT / "src/libslic3r/Api/plugin/python"
        with tempfile.TemporaryDirectory() as directory:
            catalog, _catalog_path = self._write_catalog(Path(directory))
            bindings = self.analyzer.build_binding_index(runtime, catalog)

            for method in (
                "path_count",
                "bounding_box",
            ):
                with self.subTest(method=method):
                    self.assertIn("slic3r_clipper.h", bindings.methods[method])

            for method in (
                "empty",
                "slice_to_polygons",
                "slice_to_expolygons",
                "is_model_part",
                "is_negative",
                "is_modifier",
                "is_support_enforcer",
                "is_support_blocker",
                "is_support_modifier",
                "is_seam_position",
                "is_brim",
                "is_fdm_support_painted",
                "is_seam_painted",
                "is_mm_painted",
            ):
                with self.subTest(method=method):
                    self.assertIn("slic3r_volume.h", bindings.methods[method])

            # The ExPolygon slicing runs its post-processing through the Clipper and geometry API.
            for headers in (
                bindings.methods["slice_to_expolygons"],
                bindings.class_methods[("SlicingContext", "slice_volume_to_expolygons")],
            ):
                for header in ("slic3r_volume.h", "slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h"):
                    self.assertIn(header, headers)

            self.assertIn(
                "slic3r_volume.h",
                bindings.symbols["project_painting_to_polygons"],
            )
            self.assertIn(
                "slic3r_gcode_firmware.h",
                bindings.symbols["GCodeFirmwareSession"],
            )
            self.assertIn(
                "steps/slic3r_step_gcode_firmware.h",
                bindings.symbols["GCodeFirmwareContext"],
            )
            self.assertIn(
                "slic3r_gcode_script.h",
                bindings.symbols["GCodeScriptProcessorView"],
            )
            self.assertIn(
                "slic3r_gcode_firmware.h",
                bindings.methods["gcode_firmware"],
            )

            for surface_type in (
                "MutableSurface",
                "StoredSurface",
                "StoredSurfaceCollection",
            ):
                with self.subTest(surface_type=surface_type):
                    self.assertIn(
                        "slic3r_data_tree.h",
                        bindings.symbols[surface_type],
                    )

            for surface_method in (
                "has_any_flag",
                "has_all_flags",
                "copy_properties_from",
                "append_like",
                "append_like_move",
                "mutable_at",
                "back_mutable",
            ):
                with self.subTest(surface_method=surface_method):
                    self.assertIn(
                        "slic3r_data_tree.h",
                        bindings.methods[surface_method],
                    )

            for data_tree_method in (
                "bridging_flow",
                "bottom_z",
                "copy_from",
                "curled_line",
                "curled_line_count",
                "curled_lines",
                "flow",
                "instance_count",
                "instance_shift",
                "object",
                "region",
                "region_count",
                "regions",
            ):
                with self.subTest(data_tree_method=data_tree_method):
                    self.assertIn(
                        "slic3r_data_tree.h",
                        bindings.methods[data_tree_method],
                    )

            for config_symbol in ("ConfigPoint", "GraphData"):
                with self.subTest(config_symbol=config_symbol):
                    self.assertIn(
                        "slic3r_config_option.h",
                        bindings.symbols[config_symbol],
                    )

            for config_method in (
                "bool_or_default",
                "computed_float_or_default",
                "effective_float_or_percent_or_default",
                "float_or_default",
                "float_or_percent_or_default",
                "get_point",
                "get_string",
                "graph",
                "graph_or_default",
                "int_or_default",
                "point_or_default",
                "string_or_default",
                "vector_bool_or_default",
                "vector_effective_float_or_percent_or_default",
                "vector_float_or_default",
                "vector_float_or_percent_or_default",
                "vector_graph_or_default",
                "vector_int_or_default",
                "vector_point_or_default",
                "vector_string_or_default",
            ):
                with self.subTest(config_method=config_method):
                    self.assertIn(
                        "slic3r_config_option.h",
                        bindings.methods[config_method],
                    )

            self.assertIn("slic3r_geometry.h", bindings.symbols["LineDistancer"])
            for geometry_helper in (
                "convex_hull",
                "expolygons_to_polylines",
            ):
                with self.subTest(geometry_helper=geometry_helper):
                    self.assertIn(
                        "slic3r_geometry.h",
                        bindings.symbols[geometry_helper],
                    )

            for clipper_helper in (
                "clipper_clip_expolygons_with_subject_bbox",
                "clipper_diff_polyline_expolygons",
                "clipper_intersection_polyline_expolygons",
                "clipper_split_polyline_expolygons",
            ):
                with self.subTest(clipper_helper=clipper_helper):
                    self.assertIn(
                        "slic3r_clipper.h",
                        bindings.symbols[clipper_helper],
                    )

            for visitor in (
                "ExtrusionTreeVisitorBase",
                "ExtrusionTreeConstVisitor",
                "ExtrusionTreeVisitor",
            ):
                with self.subTest(visitor=visitor):
                    self.assertIn(
                        "slic3r_extrusion_entity.h",
                        bindings.symbols[visitor],
                    )
                    self.assertIn(
                        "slic3r_extrusion_property.h",
                        bindings.symbols[visitor],
                    )

            self.assertIn(
                "slic3r_orchestrator.h",
                bindings.symbols["PluginPropertyKey"],
            )
            for layer_helper in (
                "layer_adhesion_property",
                "layer_has_adhesion_kind",
                "layer_is_brim",
                "layer_is_normal_skirt",
                "layer_is_skirt_first_layer_only",
            ):
                with self.subTest(layer_helper=layer_helper):
                    self.assertIn(
                        "slic3r_data_tree.h",
                        bindings.symbols[layer_helper],
                    )

            for region_method in ("lookup_at_point",):
                with self.subTest(region_method=region_method):
                    self.assertIn(
                        "slic3r_data_tree.h",
                        bindings.methods[region_method],
                    )

            for perimeter_method in ("region_settings", "perimeter_flow"):
                with self.subTest(perimeter_method=perimeter_method):
                    self.assertIn(
                        "steps/slic3r_step_perimeter.h",
                        bindings.methods[perimeter_method],
                    )

            for orchestrator_symbol in ("OrchestratorView", "PluginView"):
                with self.subTest(orchestrator_symbol=orchestrator_symbol):
                    self.assertIn(
                        "slic3r_orchestrator.h",
                        bindings.symbols[orchestrator_symbol],
                    )
            self.assertIn(
                "slic3r_bridge_detector.h",
                bindings.symbols["BridgeDetector"],
            )
            self.assertIn(
                "slic3r_orchestrator.h",
                bindings.methods["create_bridge_detector"],
            )
            self.assertIn(
                "slic3r_bridge_detector.h",
                bindings.methods["create_bridge_detector"],
            )

            execution_methods = {
                "pre_perimeter_context": "steps/slic3r_step_pre_perimeter.h",
                "pre_infill_context": "steps/slic3r_step_pre_infill.h",
                "infill_grouping_context": "steps/slic3r_step_infill_group.h",
                "infill_context": "steps/slic3r_step_infill.h",
                "support_demand_context": "steps/slic3r_step_support_demand.h",
                "support_toolpath_context": "steps/slic3r_step_support_toolpath.h",
                "skirt_brim_context": "steps/slic3r_step_skirt_brim.h",
                "ordering_context": "steps/slic3r_step_ordering.h",
                "gcode_context": "steps/slic3r_step_gcode.h",
                "layer_extrusion_edit_context": "steps/slic3r_step_layer_extrusion_edit.h",
            }
            for method, header in execution_methods.items():
                with self.subTest(execution_method=method):
                    self.assertIn(header, bindings.methods[method])
            self.assertIn("steps/slic3r_step_support_toolpath.h", bindings.symbols["SupportToolpathContext"])

            self.assertIn(
                "slic3r_plugin_run_context.h",
                bindings.symbols["PluginProgress"],
            )
            parallel_headers = bindings.symbols["parallel_for_storage_with_progress"]
            self.assertIn("slic3r_utils.h", parallel_headers)
            self.assertIn("slic3r_plugin_run_context.h", parallel_headers)

            auxiliary_headers = bindings.symbols[
                "build_auxiliary_layer_regions_from_subject"
            ]
            for header in (
                "slic3r_clipper.h",
                "slic3r_config.h",
                "slic3r_config_option.h",
                "slic3r_data_tree.h",
                "slic3r_geometry.h",
                "slic3r_volume.h",
            ):
                with self.subTest(auxiliary_header=header):
                    self.assertIn(header, auxiliary_headers)

            print_helper_headers = {
                "slic3r_config.h",
                "slic3r_config_option.h",
                "slic3r_data_tree.h",
            }
            for helper_name in (
                "object_extruders",
                "collect_object_printing_extruders",
                "collect_print_extruders_for_skirt",
                "get_object_first_layer_height",
                "get_min_first_layer_height",
                "e_per_mm",
            ):
                with self.subTest(print_helper=helper_name):
                    self.assertTrue(
                        print_helper_headers.issubset(bindings.symbols[helper_name])
                    )
                    self.assertTrue(
                        print_helper_headers.issubset(bindings.methods[helper_name])
                    )
            self.assertIn("slic3r_def.h", bindings.symbols["check_z_step"])
            self.assertIn("slic3r_def.h", bindings.methods["check_z_step"])
            self.assertNotIn("slic3r_print_helpers.h", catalog.headers)

            seam_header = "steps/slic3r_step_seam_placer.h"
            for seam_symbol in (
                "SeamPlacer",
                "SeamPlacerContext",
                "SeamPlacerSession",
                "make_seam_placer_instance",
            ):
                with self.subTest(seam_symbol=seam_symbol):
                    self.assertIn(seam_header, bindings.symbols[seam_symbol])
            self.assertIn(seam_header, bindings.methods["seam_placer_context"])
            self.assertIn(seam_header, bindings.methods["seam_placer_plan"])
            self.assertIn(seam_header, bindings.methods["publish_seam_placer"])
            self.assertIn(seam_header, bindings.methods["place_seam"])

    def test_auxiliary_fill_surface_mutator_abi(self):
        """The auxiliary mutator (data tree 4.1), the object flows (4.2) and their millimeter variants (4.3) declare the data-tree header."""
        with tempfile.TemporaryDirectory() as directory:
            catalog, _catalog_path = self._write_catalog(Path(directory))
            self.assertEqual(catalog.headers["slic3r_data_tree.h"]["version"], [4, 7])
            self.assertIn("slic3r_data_tree.h", catalog.c_functions["layer_region_island_set_fill_surfaces"])
            for function in ("object_get_support_flow", "object_get_region_flow",
                             "object_get_support_flow_mm", "object_get_region_flow_mm"):
                self.assertIn("slic3r_data_tree.h", catalog.c_functions[function])
            bindings = self.analyzer.build_binding_index(ROOT / "src/libslic3r/Api/plugin/python", catalog)
            for method in ("set_fill_surfaces", "support_flow", "region_flow", "support_flow_mm", "region_flow_mm"):
                self.assertIn("slic3r_data_tree.h", bindings.methods[method])

    def _write_catalog(self, root):
        catalog_path = root / "catalog.json"
        catalog_path.write_text(self.generator.generate_abi_catalog(), encoding="utf-8")
        return self.analyzer.AbiCatalog.load(catalog_path), catalog_path

    def test_python_plugin_analysis_combines_raw_symbols_wrappers_and_local_imports(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime = root / "runtime"
            runtime.mkdir()
            catalog, _catalog_path = self._write_catalog(root)
            runtime.joinpath("slic3r_api.py").write_text(
                "from slic3r_abi import abi_requires_headers\n"
                "@abi_requires_headers('slic3r_config.h')\n"
                "def wrapped(): pass\n"
                "class First:\n"
                " @abi_requires_headers('slic3r_geometry.h')\n"
                " def touch(self): pass\n"
                "class Second:\n"
                " @abi_requires_headers('slic3r_extrusion_entity.h')\n"
                " def touch(self): pass\n",
                encoding="utf-8")
            runtime.joinpath("slic3r_geometry_views.py").write_text(
                "from slic3r_abi import abi_requires_headers\n"
                "@abi_requires_headers('slic3r_geometry.h')\n"
                "class GeometryView: pass\n",
                encoding="utf-8")
            runtime.joinpath("slic3r_abi.py").write_text("def abi_requires_headers(*headers): pass\n", encoding="utf-8")
            root.joinpath("helper.py").write_text(
                "from slic3r_api import CPoint\n"
                "def use_type(value: CPoint): return value\n", encoding="utf-8")
            root.joinpath("relative_helper.py").write_text(
                "from slic3r_api import STEP_ORDERING\n"
                "def ordering_step(): return STEP_ORDERING\n", encoding="utf-8")
            entry = root / "plugin.py"
            entry.write_text(
                "from slic3r_api import wrapped as wrapped_alias\n"
                "from slic3r_geometry_views import GeometryView\n"
                "from helper import use_type\n"
                "from .relative_helper import ordering_step\n"
                "def register_plugin(api):\n"
                " wrapped_alias()\n"
                " GeometryView()\n"
                " api.host.extrusion_is_loop(0)\n"
                " unknown.touch()\n",
                encoding="utf-8")

            bindings = self.analyzer.build_binding_index(runtime, catalog)
            requirements = self.analyzer.analyze_plugin(entry, root, catalog, bindings)
            self.assertTrue({
                "slic3r_plugin_types.h",
                "slic3r_config.h",
                "slic3r_geometry.h",
                "slic3r_extrusion_entity.h",
                "slic3r_slicing_step.h",
            }.issubset(requirements))

    def test_common_context_methods_require_only_the_common_run_context(self):
        runtime = ROOT / "src/libslic3r/Api/plugin/python"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            for method in (
                "plugin_storage",
                "is_cancelled",
                "report_warning",
                "report_error",
                "report_progress",
            ):
                with self.subTest(method=method):
                    self.assertEqual(
                        bindings.methods[method],
                        {"slic3r_plugin_run_context.h"},
                    )

    def test_python_firmware_imports_only_the_firmware_service_contracts(self):
        runtime = ROOT / "src/libslic3r/Api/plugin/python"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            entry = root / "plugin.py"
            entry.write_text(
                "from slic3r_api import GCodeFirmwareContext, GCodeFirmwareSession\n"
                "def create():\n"
                " return GCodeFirmwareContext, GCodeFirmwareSession\n",
                encoding="utf-8",
            )
            requirements = self.analyzer.analyze_plugin(
                entry, root, catalog, bindings
            )
            self.assertIn("slic3r_plugin_types.h", requirements)
            self.assertIn("slic3r_gcode_firmware.h", requirements)
            self.assertIn("steps/slic3r_step_gcode_firmware.h", requirements)
            self.assertNotIn("steps/slic3r_step_seam_placer.h", requirements)
            self.assertNotIn("slic3r_bridge_detector.h", requirements)

    def test_typed_non_firmware_methods_do_not_require_firmware_contracts(self):
        runtime = ROOT / "src/libslic3r/Api/plugin/python"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            entry = root / "plugin.py"
            entry.write_text(
                "from slic3r_api import ConfigOption, Print, PrintingPlan\n"
                "def inspect(print_view: Print, option: ConfigOption, plan: PrintingPlan):\n"
                " print_view.config()\n"
                " option.get_int()\n"
                " plan.valid()\n",
                encoding="utf-8",
            )

            requirements = self.analyzer.analyze_plugin(
                entry, root, catalog, bindings
            )

            self.assertIn("slic3r_config_option.h", requirements)
            self.assertIn("slic3r_printing_plan.h", requirements)
            self.assertNotIn("slic3r_gcode_script.h", requirements)
            self.assertNotIn("slic3r_gcode_firmware.h", requirements)
            self.assertNotIn("steps/slic3r_step_gcode_firmware.h", requirements)

    def test_printing_plan_iterator_items_keep_their_receiver_type(self):
        runtime = ROOT / "src/libslic3r/Api/plugin/python"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            entry = root / "plugin.py"
            entry.write_text(
                "from slic3r_api import PrintingPlan\n"
                "def inspect(plan: PrintingPlan):\n"
                " for group in plan.groups():\n"
                "  group.valid()\n"
                " groups = plan.groups()\n"
                " for assigned_group in groups:\n"
                "  assigned_group.valid()\n"
                " for direct_group in plan:\n"
                "  direct_group.valid()\n",
                encoding="utf-8",
            )

            requirements = self.analyzer.analyze_plugin(
                entry, root, catalog, bindings
            )

            self.assertIn("slic3r_printing_plan.h", requirements)
            self.assertNotIn("slic3r_gcode_script.h", requirements)
            self.assertNotIn("slic3r_gcode_firmware.h", requirements)
            self.assertNotIn("steps/slic3r_step_gcode_firmware.h", requirements)

    def test_printing_plan_comprehensions_do_not_require_firmware_contracts(self):
        runtime = ROOT / "src/libslic3r/Api/plugin/python"
        expressions = {
            "list": "[group.valid() for group in plan.groups()]",
            "set": "{group.valid() for group in plan.groups()}",
            "dict": "{group.address: group.valid() for group in plan.groups()}",
            "generator": "(group.valid() for group in plan.groups())",
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            entry = root / "plugin.py"
            for kind, expression in expressions.items():
                with self.subTest(comprehension=kind):
                    entry.write_text(
                        "from slic3r_api import PrintingPlan\n"
                        "def inspect(plan: PrintingPlan):\n"
                        f" return {expression}\n",
                        encoding="utf-8",
                    )
                    requirements = self.analyzer.analyze_plugin(
                        entry, root, catalog, bindings
                    )
                    self.assertIn("slic3r_printing_plan.h", requirements)
                    unexpected = requirements.intersection({
                        "slic3r_gcode_script.h",
                        "slic3r_gcode_firmware.h",
                        "steps/slic3r_step_gcode_firmware.h",
                    })
                    self.assertFalse(unexpected, f"unrelated contracts: {sorted(unexpected)}")

    def test_possibly_empty_loop_preserves_unknown_receiver_requirements(self):
        runtime = ROOT / "src/libslic3r/Api/plugin/python"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            entry = root / "plugin.py"
            entry.write_text(
                "from slic3r_api import PrintingPlan\n"
                "def inspect(plan: PrintingPlan, fallback):\n"
                " value = fallback\n"
                " for value in plan.groups():\n"
                "  pass\n"
                " return value.valid()\n",
                encoding="utf-8",
            )
            requirements = self.analyzer.analyze_plugin(
                entry, root, catalog, bindings
            )

            # An empty plan leaves value equal to the unknown fallback, which
            # may be a firmware or script view. Retain those possible contracts.
            self.assertIn("slic3r_printing_plan.h", requirements)
            missing = {
                "slic3r_gcode_firmware.h",
                "slic3r_gcode_script.h",
            }.difference(requirements)
            self.assertFalse(missing, f"fallback contracts omitted: {sorted(missing)}")

    def test_unknown_reassignment_discards_the_previous_receiver_type(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime = root / "runtime"
            runtime.mkdir()
            runtime.joinpath("slic3r_api.py").write_text(
                "from slic3r_abi import abi_requires_headers\n"
                "@abi_requires_headers('slic3r_geometry.h')\n"
                "class First:\n"
                " def touch(self): pass\n"
                "@abi_requires_headers('slic3r_extrusion_entity.h')\n"
                "class Second:\n"
                " def touch(self): pass\n",
                encoding="utf-8",
            )
            runtime.joinpath("slic3r_abi.py").write_text(
                "def abi_requires_headers(*headers): pass\n", encoding="utf-8"
            )
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            entry = root / "plugin.py"
            entry.write_text(
                "from slic3r_api import First\n"
                "def inspect(value: First, other):\n"
                " value = other\n"
                " value.touch()\n",
                encoding="utf-8",
            )

            requirements = self.analyzer.analyze_plugin(
                entry, root, catalog, bindings
            )

            self.assertIn("slic3r_geometry.h", requirements)
            self.assertIn("slic3r_extrusion_entity.h", requirements)

    def test_conditional_branches_merge_receiver_types_conservatively(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime = root / "runtime"
            runtime.mkdir()
            runtime.joinpath("slic3r_api.py").write_text(
                "from slic3r_abi import abi_requires_headers\n"
                "@abi_requires_headers('slic3r_geometry.h')\n"
                "class First:\n"
                " def touch(self): pass\n"
                "@abi_requires_headers('slic3r_extrusion_entity.h')\n"
                "class Second:\n"
                " def touch(self): pass\n",
                encoding="utf-8",
            )
            runtime.joinpath("slic3r_abi.py").write_text(
                "def abi_requires_headers(*headers): pass\n", encoding="utf-8"
            )
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            entry = root / "plugin.py"
            cases = {
                "unknown_if_branch": (
                    "def inspect(value: First, unknown, condition):\n"
                    " if condition:\n"
                    "  value = unknown\n"
                    " else:\n"
                    "  value = First()\n"
                    " value.touch()\n",
                    {"slic3r_geometry.h", "slic3r_extrusion_entity.h"},
                ),
                "two_known_branches": (
                    "def inspect(value: First, condition):\n"
                    " if condition:\n"
                    "  value = First()\n"
                    " else:\n"
                    "  value = Second()\n"
                    " value.touch()\n",
                    {"slic3r_geometry.h", "slic3r_extrusion_entity.h"},
                ),
                "missing_else": (
                    "def inspect(value: First, unknown, condition):\n"
                    " if condition:\n"
                    "  value = unknown\n"
                    " value.touch()\n",
                    {"slic3r_geometry.h", "slic3r_extrusion_entity.h"},
                ),
                "nested_elif": (
                    "def inspect(value: First, unknown, first, second):\n"
                    " if first:\n"
                    "  if second:\n"
                    "   value = Second()\n"
                    "  else:\n"
                    "   value = First()\n"
                    " elif second:\n"
                    "  value = unknown\n"
                    " else:\n"
                    "  value = First()\n"
                    " value.touch()\n",
                    {"slic3r_geometry.h", "slic3r_extrusion_entity.h"},
                ),
                "unchanged_receiver": (
                    "def inspect(value: First, unknown, condition):\n"
                    " if condition:\n"
                    "  other = unknown\n"
                    " else:\n"
                    "  other = unknown\n"
                    " value.touch()\n",
                    {"slic3r_geometry.h"},
                ),
            }
            for name, (source, expected) in cases.items():
                with self.subTest(branches=name):
                    entry.write_text(
                        "from slic3r_api import First, Second\n" + source,
                        encoding="utf-8",
                    )
                    requirements = self.analyzer.analyze_plugin(
                        entry, root, catalog, bindings
                    )
                    branch_contracts = requirements.intersection({
                        "slic3r_geometry.h",
                        "slic3r_extrusion_entity.h",
                    })
                    self.assertEqual(branch_contracts, expected)

    def test_conditional_branches_merge_iterable_item_types(self):
        runtime = ROOT / "src/libslic3r/Api/plugin/python"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            entry = root / "plugin.py"
            entry.write_text(
                "from slic3r_api import PrintingPlan\n"
                "def inspect(plan: PrintingPlan, unknown, condition):\n"
                " if condition:\n"
                "  groups = plan.groups()\n"
                " else:\n"
                "  groups = unknown\n"
                " for group in groups:\n"
                "  group.valid()\n",
                encoding="utf-8",
            )

            requirements = self.analyzer.analyze_plugin(
                entry, root, catalog, bindings
            )

            self.assertIn("slic3r_printing_plan.h", requirements)
            self.assertIn("slic3r_gcode_script.h", requirements)
            self.assertIn("slic3r_gcode_firmware.h", requirements)

    def test_ambiguous_dunder_init_call_is_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime = root / "runtime"
            runtime.mkdir()
            runtime.joinpath("slic3r_api.py").write_text("", encoding="utf-8")
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            self.assertNotIn("__init__", bindings.methods)
            entry = root / "plugin.py"
            entry.write_text(
                "class Local:\n"
                " def __init__(self): pass\n"
                "def run(value):\n"
                " value.__init__()\n",
                encoding="utf-8",
            )
            self.assertEqual(
                self.analyzer.analyze_plugin(entry, root, catalog, bindings),
                {"slic3r_plugin_types.h"},
            )

    def test_python_plugin_dynamic_access_requires_an_explicit_override(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime = root / "runtime"
            runtime.mkdir()
            runtime.joinpath("slic3r_api.py").write_text("", encoding="utf-8")
            runtime.joinpath("slic3r_abi.py").write_text("def abi_requires_headers(*headers): pass\n", encoding="utf-8")
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            entry = root / "plugin.py"
            entry.write_text("def run(api, name):\n getattr(api.host, name)()\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, r"plugin\.py:2:2: dynamic plugin API access"):
                self.analyzer.analyze_plugin(entry, root, catalog, bindings)

            entry.write_text(
                "from slic3r_abi import abi_requires_headers\n"
                "@abi_requires_headers('slic3r_geometry.h')\n"
                "def run(api, name):\n getattr(api.host, name)()\n",
                encoding="utf-8")
            requirements = self.analyzer.analyze_plugin(entry, root, catalog, bindings)
            self.assertIn("slic3r_geometry.h", requirements)

    def test_failed_python_analysis_does_not_replace_a_valid_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime = root / "runtime"
            runtime.mkdir()
            runtime.joinpath("slic3r_api.py").write_text("", encoding="utf-8")
            runtime.joinpath("slic3r_abi.py").write_text("def abi_requires_headers(*headers): pass\n", encoding="utf-8")
            _catalog, catalog_path = self._write_catalog(root)
            entry = root / "plugin.py"
            entry.write_text("def run(api, name):\n getattr(api.host, name)()\n", encoding="utf-8")
            output = root / "abi.ini"
            output.write_text("previous valid manifest\n", encoding="utf-8")
            command = [
                sys.executable,
                str(ROOT / "src/libslic3r/Api/plugin/python/generate_python_plugin_abi.py"),
                "--entry", str(entry),
                "--source-root", str(root),
                "--runtime-root", str(runtime),
                "--catalog", str(catalog_path),
                "--output", str(output),
            ]
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertRegex(result.stderr, r"plugin\.py:2:2: dynamic plugin API access")
            self.assertEqual(output.read_text(encoding="utf-8"), "previous valid manifest\n")

    def test_python_plugin_rejects_star_import_and_unknown_decorator_header(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime = root / "runtime"
            runtime.mkdir()
            runtime.joinpath("slic3r_api.py").write_text("", encoding="utf-8")
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            entry = root / "plugin.py"
            entry.write_text("from slic3r_api import *\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "wildcard imports"):
                self.analyzer.analyze_plugin(entry, root, catalog, bindings)
            entry.write_text(
                "from slic3r_abi import abi_requires_headers\n"
                "@abi_requires_headers('missing.h')\n"
                "def run(): pass\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unknown C ABI header 'missing.h'"):
                self.analyzer.analyze_plugin(entry, root, catalog, bindings)
            entry.write_text("from slic3r_api import missing_binding\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unknown Python plugin API symbol 'missing_binding'"):
                self.analyzer.analyze_plugin(entry, root, catalog, bindings)

            runtime.joinpath("slic3r_api.py").write_text(
                "class Binding:\n"
                " def method_only(self): pass\n",
                encoding="utf-8",
            )
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            entry.write_text("from slic3r_api import method_only\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unknown Python plugin API symbol 'method_only'"):
                self.analyzer.analyze_plugin(entry, root, catalog, bindings)

    def test_repository_python_plugins_generate_targeted_manifests(self):
        runtime = ROOT / "src/libslic3r/Api/plugin/python"
        plugins = ROOT / "src/plugins_python"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            full_count = len(catalog.headers)
            entries = [
                (plugins / "examples/example_python_plugin.py", None),
                (plugins / "polyholes/polyholes_low_level.py", "steps/slic3r_step_post_slicing.h"),
                (plugins / "polyholes/polyholes_high_level.py", "steps/slic3r_step_post_slicing.h"),
                (plugins / "perimeter/simple_perimeter_generator.py", "steps/slic3r_step_perimeter.h"),
                (plugins / "perimeter/hairy_object.py", "steps/slic3r_step_post_perimeter.h"),
                (
                    plugins / "surface/initial_typed_surface_builder.py",
                    "steps/slic3r_step_surface_generation.h",
                ),
            ]
            for entry, expected_step in entries:
                with self.subTest(entry=entry.name):
                    requirements = self.analyzer.analyze_plugin(entry, plugins, catalog, bindings)
                    self.assertIn("slic3r_plugin_types.h", requirements)
                    self.assertLess(len(requirements), full_count)
                    if expected_step is not None:
                        self.assertIn(expected_step, requirements)

            perimeter_requirements = self.analyzer.analyze_plugin(
                plugins / "perimeter/simple_perimeter_generator.py",
                plugins,
                catalog,
                bindings,
            )
            self.assertIn("steps/slic3r_step_perimeter.h", perimeter_requirements)
            self.assertTrue({
                "steps/slic3r_step_layer_height.h",
                "steps/slic3r_step_post_infill.h",
                "steps/slic3r_step_post_perimeter.h",
                "steps/slic3r_step_post_slicing.h",
                "steps/slic3r_step_slicing.h",
                "steps/slic3r_step_surface_generation.h",
            }.isdisjoint(perimeter_requirements))


if __name__ == "__main__":
    unittest.main()
