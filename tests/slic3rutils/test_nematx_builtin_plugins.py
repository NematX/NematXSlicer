"""Configure-only checks of optional NematX embedding, with inert source fixtures."""
import argparse
from pathlib import Path
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
    "plugins/regional_fan/src/RegionalFan.cpp",
    "plugins/wipetower/src/WipeTowerManager.cpp", "plugins/wipetower/src/WipeTowerLayout.cpp",
    "plugins/wipetower/src/TowerFootprints.cpp", "plugins/wipetower/src/FilamentSections.cpp",
    "plugins/wipetower/src/rectilinear/RectilinearFills.cpp",
    "plugins/wipetower/src/rectilinear/RectilinearEdgeGrid.cpp",
    "plugins/smooth_temperature/src/SmoothTemperature.cpp",
    "plugins/support_interface_perimeters/src/SupportInterfacePerimeters.cpp",
    "plugins/support_interface_perimeters/src/InterfacePerimeters.cpp",
    "plugins/seam/src/InternalSeam.cpp", "plugins/seam/src/InternalSeamPlugin.cpp",
)
IDS = (
    "nematx_second_layer_flow", "nematx_corner_stretch", "nematx_fan_first_layer_program",
    "nematx_fan_speedup", "nematx_hole_size_curve", "rectilineararoundholes",
    "gridvarspeed", "nematx_gridvarspeed_speed", "nematx_gridvarspeed_recipe",
    "nematx", "gcode.reader.nematx", "nematx_regional_fan", "nematx_wipetower",
    "nematx_smooth_temperature", "nematx.support.bottom_contact", "nematx.support.bottom_interface",
    "nematx.support.top_contact", "nematx.support.top_interface", "nematx_seam",
)


class BuiltinPluginBuildTests(unittest.TestCase):
    def configure(self, name, brand="NematXSlicer", enabled=None, missing=False):
        source = OUT / name / "source"
        binary = OUT / name / "binary"
        external = source / "external"
        source.mkdir(parents=True, exist_ok=True)
        (source / "empty.cpp").write_text("// Inert configure-only fixture.\n", encoding="utf-8")
        if not missing:
            for relative in ("CMakeLists.txt", *SOURCES):
                path = external / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("// Inert configure-only fixture.\n", encoding="utf-8")
        script = f'''cmake_minimum_required(VERSION 3.16)
project(libslic3r LANGUAGES CXX)
set(SLIC3R_APP_KEY "{brand}")
add_library(libslic3r STATIC empty.cpp)
add_library(boost_headeronly INTERFACE)
include("{(ROOT / 'cmake/NematXBuiltinPlugins.cmake').as_posix()}")
get_property(activations GLOBAL PROPERTY SLIC3R_NEMATX_DEFAULT_ACTIVATIONS)
file(WRITE "${{CMAKE_BINARY_DIR}}/activations.txt" "${{activations}}")
get_property(targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
file(WRITE "${{CMAKE_BINARY_DIR}}/targets.txt" "${{targets}}")
foreach(target IN LISTS targets)
    if(target MATCHES "^nematx_builtin_")
        get_target_property(definitions ${{target}} COMPILE_DEFINITIONS)
        file(WRITE "${{CMAKE_BINARY_DIR}}/${{target}}-definitions.txt" "${{definitions}}")
    endif()
endforeach()
'''
        (source / "CMakeLists.txt").write_text(script, encoding="utf-8")
        command = [CMAKE, "-S", str(source), "-B", str(binary),
                   "-DNEMATX_PLUGINS_SOURCE_DIR:PATH=" + str(external)]
        if enabled is not None:
            command.append("-DSLIC3R_NEMATX_BUILTIN_PLUGINS:BOOL=" + ("ON" if enabled else "OFF"))
        result = subprocess.run(command, capture_output=True, text=True, timeout=900)
        (OUT / name / "configure.log").write_text(result.stdout + result.stderr, encoding="utf-8")
        return binary, result

    def test_nematx_embeds_all_modules_by_default(self):
        binary, result = self.configure("nematx-default")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        targets = (binary / "targets.txt").read_text().split(";")
        modules = [target for target in targets if target.startswith("nematx_builtin_")]
        self.assertEqual(len(modules), 13)
        activated = (binary / "activations.txt").read_text().splitlines()
        self.assertEqual(set(activated), {plugin_id + " = 1" for plugin_id in IDS})
        registry = (binary / "NematXBuiltinPlugins.cpp").read_text()
        for plugin_id in IDS:
            self.assertIn(f'host.get_plugin("{plugin_id}") == nullptr', registry)
        for target in modules:
            name = target.removeprefix("nematx_builtin_")
            definitions = (binary / (target + "-definitions.txt")).read_text().split(";")
            self.assertIn("register_plugin=nematx_builtin_register_" + name, definitions)
            self.assertIn("slic3r_plugin_abi_version=nematx_builtin_abi_" + name, definitions)
            self.assertIn("slic3r_plugin_abi_manifest=nematx_builtin_manifest_" + name, definitions)
            self.assertNotIn("SLIC3R_PLUGIN_EXPORTS", definitions)

    def test_optional_embedding_off_needs_no_external_checkout(self):
        binary, result = self.configure("explicit-off", enabled=False, missing=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual((binary / "activations.txt").read_text(), "")
        self.assertNotIn("nematx_builtin_", (binary / "targets.txt").read_text())

    def test_other_brands_do_not_enable_embedding(self):
        binary, result = self.configure("upstream-default", brand="SuperSlicer")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual((binary / "activations.txt").read_text(), "")

    def test_requested_missing_sources_fail_visibly(self):
        _, result = self.configure("missing-requested", enabled=True, missing=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("NematX plugin sources not found", result.stdout + result.stderr)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--cmake", required=True)
    args, remaining = parser.parse_known_args()
    OUT = args.output_dir.resolve()
    CMAKE = args.cmake
    unittest.main(argv=[__file__, *remaining])
