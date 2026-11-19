"""Check NematX embedding in a checkout with no sibling plugin repository.

Each fixture copies the real build adapter and gives it inert sources. CMake
configuration exercises default source selection, explicit overrides and the
registration and activation it generates, without building the modules.
"""
import argparse
from pathlib import Path
import shutil
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[2]
OUT = None
CMAKE = None
SOURCES = (
    "src/SecondLayerFlow.cpp", "src/CornerStretch.cpp",
    "plugins/fan_first_layer_program/src/FanFirstLayerProgram.cpp",
    "plugins/fan_speedup/src/FanSpeedup.cpp", "plugins/hole_size_curve/src/HoleSizeCurve.cpp",
    "plugins/infill_aroundholes/src/AroundHolesPattern.cpp",
    "plugins/infill_gridvarspeed/src/GridVarSpeedPattern.cpp",
    "plugins/infill_gridvarspeed/src/GridVarSpeedProcess.cpp",
    "plugins/infill_gridvarspeed/src/rectilinear/RectilinearFills.cpp",
    "plugins/nematx_firmware/src/NematXFirmwarePlugin.cpp",
    "plugins/nematx_firmware/src/NematXFirmwareSession.cpp",
    "plugins/nematx_firmware/src/NematXGCodeReader.cpp",
    *(f"plugins/pressure/src/Pressure{part}.cpp" for part in (
        "Classification", "Controls", "Extrusion", "Flow", "Geometry", "Kinematics",
        "Nominal", "Planner", "Reader", "Rewrite", "Source", "Tokens")),
    "plugins/regional_fan/src/RegionalFan.cpp",
    "plugins/wipetower/src/WipeTowerManager.cpp", "plugins/wipetower/src/WipeTowerLayout.cpp",
    "plugins/wipetower/src/TowerFootprints.cpp", "plugins/wipetower/src/FilamentSections.cpp",
    "plugins/wipetower/src/rectilinear/RectilinearFills.cpp",
    "plugins/wipetower/src/rectilinear/PluginEdgeGrid.cpp",
    "plugins/smooth_temperature/src/SmoothTemperature.cpp",
    "plugins/support_interface_perimeters/src/SupportInterfacePerimeters.cpp",
    "plugins/support_interface_perimeters/src/InterfacePerimeters.cpp",
    "plugins/seam/src/InternalSeam.cpp", "plugins/seam/src/InternalSeamPlugin.cpp",
    "plugins/top_surface_retraction/src/TopSurfaceRetraction.cpp",
)
IDS = (
    "nematx_second_layer_flow", "nematx_corner_stretch", "nematx_fan_first_layer_program",
    "nematx_fan_speedup", "nematx_hole_size_curve", "rectilineararoundholes",
    "gridvarspeed", "nematx_gridvarspeed_speed", "nematx_gridvarspeed_recipe",
    "nematx", "gcode.reader.nematx", "nematx_regional_fan", "nematx_wipetower",
    "nematx_smooth_temperature", "nematx.support.bottom_contact", "nematx.support.bottom_interface",
    "nematx.support.top_contact", "nematx.support.top_interface", "nematx_seam",
    "nematx_top_surface_retraction",
)


class BuiltinPluginBuildTests(unittest.TestCase):
    def configure(self, name, brand="NematXSlicer", enabled=None, missing=False, override=False):
        """Build a minimal checkout layout and configure it without compiling."""
        source = OUT / name / "source"
        binary = OUT / name / "binary"
        plugins = source / ("override" if override else "src/plugins_cpp/NematX")
        source.mkdir(parents=True, exist_ok=True)
        adapter = source / "cmake/NematXBuiltinPlugins.cmake"
        adapter.parent.mkdir(exist_ok=True)
        shutil.copyfile(ROOT / "cmake/NematXBuiltinPlugins.cmake", adapter)
        (source / "empty.cpp").write_text("// Inert configure-only fixture.\n", encoding="utf-8")
        if not missing:
            for relative in ("CMakeLists.txt", *SOURCES):
                path = plugins / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("// Inert configure-only fixture.\n", encoding="utf-8")
        script = f'''cmake_minimum_required(VERSION 3.16)
project(libslic3r LANGUAGES CXX)
set(SLIC3R_APP_KEY "{brand}")
add_library(libslic3r STATIC empty.cpp)
add_library(boost_headeronly INTERFACE)
include("${{CMAKE_CURRENT_SOURCE_DIR}}/cmake/NematXBuiltinPlugins.cmake")
get_property(activations GLOBAL PROPERTY SLIC3R_NEMATX_DEFAULT_ACTIVATIONS)
file(WRITE "${{CMAKE_BINARY_DIR}}/activations.txt" "${{activations}}")
get_property(targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
file(WRITE "${{CMAKE_BINARY_DIR}}/targets.txt" "${{targets}}")
foreach(target IN LISTS targets)
    if(target MATCHES "^nematx_builtin_")
        get_target_property(definitions ${{target}} COMPILE_DEFINITIONS)
        file(WRITE "${{CMAKE_BINARY_DIR}}/${{target}}-definitions.txt" "${{definitions}}")
        get_target_property(sources ${{target}} SOURCES)
        file(WRITE "${{CMAKE_BINARY_DIR}}/${{target}}-sources.txt" "${{sources}}")
    endif()
endforeach()
'''
        (source / "CMakeLists.txt").write_text(script, encoding="utf-8")
        command = [CMAKE, "-S", str(source), "-B", str(binary)]
        if override:
            command.append("-DNEMATX_PLUGINS_SOURCE_DIR:PATH=" + str(plugins))
        if enabled is not None:
            command.append("-DSLIC3R_NEMATX_BUILTIN_PLUGINS:BOOL=" + ("ON" if enabled else "OFF"))
        result = subprocess.run(command, capture_output=True, text=True, timeout=900)
        (OUT / name / "configure.log").write_text(result.stdout + result.stderr, encoding="utf-8")
        return binary, result

    def test_nematx_embeds_all_modules_by_default(self):
        """Use only the tracked layout and check every source, symbol and plugin ID."""
        binary, result = self.configure("nematx-default")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        targets = (binary / "targets.txt").read_text().split(";")
        modules = [target for target in targets if target.startswith("nematx_builtin_")]
        self.assertEqual(len(modules), 14)
        activated = (binary / "activations.txt").read_text().splitlines()
        self.assertEqual(set(activated), {plugin_id + " = 1" for plugin_id in IDS})
        registry = (binary / "NematXBuiltinPlugins.cpp").read_text()
        source_root = binary.parent / "source/src/plugins_cpp/NematX"
        sources = []
        for plugin_id in IDS:
            self.assertIn(f'host.get_plugin("{plugin_id}") == nullptr', registry)
        for target in modules:
            name = target.removeprefix("nematx_builtin_")
            sources.extend(Path(path).relative_to(source_root).as_posix()
                           for path in (binary / (target + "-sources.txt")).read_text().split(";"))
            definitions = (binary / (target + "-definitions.txt")).read_text().split(";")
            self.assertIn("register_plugin=nematx_builtin_register_" + name, definitions)
            self.assertIn("slic3r_plugin_abi_version=nematx_builtin_abi_" + name, definitions)
            self.assertIn("slic3r_plugin_abi_manifest=nematx_builtin_manifest_" + name, definitions)
            self.assertNotIn("SLIC3R_PLUGIN_EXPORTS", definitions)
        self.assertCountEqual(sources, SOURCES)

    def test_optional_embedding_off_needs_no_plugin_sources(self):
        """An explicit opt-out works even on a checkout with no module sources."""
        binary, result = self.configure("explicit-off", enabled=False, missing=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual((binary / "activations.txt").read_text(), "")
        self.assertNotIn("nematx_builtin_", (binary / "targets.txt").read_text())

    def test_other_brands_do_not_enable_embedding(self):
        """The source directory alone does not enable NematX plugins for other brands."""
        binary, result = self.configure("upstream-default", brand="SuperSlicer")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual((binary / "activations.txt").read_text(), "")

    def test_missing_sources_fail_visibly(self):
        """A NematX build must not succeed without its requested modules."""
        for name, enabled, override in (("missing-default", None, False), ("missing-override", True, True)):
            with self.subTest(name=name):
                _, result = self.configure(name, enabled=enabled, missing=True, override=override)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("NematX plugin sources not found", result.stdout + result.stderr)

    def test_explicit_source_override(self):
        """An explicit source tree still supports standalone plugin development."""
        binary, result = self.configure("source-override", override=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        source = Path((binary / "nematx_builtin_corner_stretch-sources.txt").read_text())
        self.assertEqual(source, binary.parent / "source/override/src/CornerStretch.cpp")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--cmake", required=True)
    args, remaining = parser.parse_known_args()
    OUT = args.output_dir.resolve()
    CMAKE = args.cmake
    unittest.main(argv=[__file__, *remaining])
