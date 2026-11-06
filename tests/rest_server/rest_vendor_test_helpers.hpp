// Small vendor trees shared by catalog, real TCP, launch and deployment tests.
// Abstract parents exercise the same system-bundle flattening as the GUI.
#pragma once

#include <filesystem>
#include <fstream>
#include <string>

namespace RestServerTest {

// Copy the checked-in Basic bundle, optionally adding an independent vendor
// whose two printers and conditions make compatibility changes observable.
inline void write_vendor_profiles(const std::filesystem::path &root, bool with_test = true)
{
    std::filesystem::create_directories(root / "vendor");
    std::filesystem::copy_file(std::filesystem::path(SLIC3R_REST_SOURCE_DIRECTORY) /
        "tests/data/rest_profiles/Basic.ini", root / "vendor/Basic.ini",
        std::filesystem::copy_options::overwrite_existing);
    if (!with_test)
        return;
    std::ofstream(root / "vendor/Test.ini") << R"ini([vendor]
name = TestVendor
config_version = 0.1.0

[printer_model:Test]
name = Test
variants = 0.4;0.25

[printer:*base*]
printer_model = Test
default_filament_profile = Test PLA @TEST
bed_shape = 0x0,200x0,200x200,0x200
gcode_flavor = marlin

[printer:Test 0.4]
inherits = *base*
printer_variant = 0.4
nozzle_diameter = 0.4
default_print_profile = Test print @TEST

[printer:Test 0.25]
inherits = *base*
printer_variant = 0.25
nozzle_diameter = 0.25
default_print_profile = Test fine .25N

[print:*base*]
layer_height = 0.2
perimeters = 3

[print:Test print @TEST]
inherits = *base*
compatible_printers_condition = nozzle_diameter[0]==0.4
test_future_key = 1

[print:Test fine .25N]
inherits = *base*
layer_height = 0.1
compatible_printers_condition = nozzle_diameter[0]==0.25

[filament:Test PLA @TEST]
filament_diameter = 1.75
temperature = 205
compatible_prints_condition = layer_height==0.2
)ini";
}

} // namespace RestServerTest
