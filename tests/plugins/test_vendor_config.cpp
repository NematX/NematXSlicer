#include <catch2/catch.hpp>

#include <string>
#include <unordered_map>
#include <utility>

#include "plugin_test_helpers.hpp"
#include "libslic3r/Config/PrintConfig.hpp"

/*
Verify historical setting destinations and serialized values during ordinary loading.
Runtime initialization supplies registered options before conversion. Each independent
source map is then converted and checked for its destination and serialized value.
The maps are test-owned; these checks do not change schema definitions or plugin selections.

Ordinary conversion test path:
  TEST_CASE - check 21 source/value pairs independently, without schema filtering.
    ensure_plugin_test_runtime_initialized - initialize the shared option/plugin runtime.
    PrintConfigDef::handle_legacy_map - convert the source map before destination/value assertions.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized()
        `-- PrintConfigDef::handle_legacy_map()
*/

using namespace Slic3r;

// One source at a time exposes the exact destination and converted text, without schema filtering.
TEST_CASE("Fixed legacy branches preserve ordinary serialized results", "[plugins][config][vendor-config-names]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const struct LegacyCase {
        const char *source;
        const char *input;
        const char *destination;
        const char *output;
    } cases[] = {
        {"octoprint_host", "printer.local", "print_host", "printer.local"},
        {"octoprint_cafile", "cert.pem", "printhost_cafile", "cert.pem"},
        {"octoprint_apikey", "test-token", "printhost_apikey", "test-token"},
        {"elefant_foot_compensation", "0.25", "first_layer_size_compensation", "-0.25"},
        {"elefant_foot_compensation", "0", "first_layer_size_compensation", "0"},
        {"elefant_foot_compensation", "-0.25", "first_layer_size_compensation", "-0.25"},
        {"elefant_foot_min_width", "0.3", "elephant_foot_min_width", "0.3"},
        {"z_steps_per_mm", "4", "z_step", "0.25"},
        {"z_steps_per_mm", "0", "z_step", "0"},
        {"z_steps_per_mm", "-4", "z_step", "-4"},
        {"infill_not_connected", "1", "infill_connection", "notconnected"},
        {"infill_not_connected", "0", "infill_connection", "connected"},
        {"preset_name", "sentinel", "preset_names", "sentinel"},
        {"print_machine_envelope", "1", "machine_limits_usage", "emit_to_gcode"},
        {"print_machine_envelope", "0", "machine_limits_usage", "time_estimate_only"},
        {"retract_lift_not_last_layer", "1", "retract_lift_top", "Not on top"},
        {"retract_lift_not_last_layer", "0", "retract_lift_top", "All surfaces"},
        {"support_material_solid_first_layer", "1", "raft_first_layer_density", "100"},
        {"support_material_solid_first_layer", "0", "raft_first_layer_density", "100"},
        {"external_perimeters_vase", "1", "seam_slope_type", "all"},
        {"external_perimeters_vase", "0", "seam_slope_type", "none"},
    };
    for (const LegacyCase &entry : cases) {
        CAPTURE(entry.source, entry.input);
        std::unordered_map<t_config_option_key, std::pair<t_config_option_key, std::string>> values;
        values[entry.source] = {entry.source, entry.input};
        PrintConfigDef::handle_legacy_map(values, false);
        REQUIRE(values.size() == 1);
        REQUIRE(values.count(entry.source) == 1);
        CHECK(values.at(entry.source).first == entry.destination);
        CHECK(values.at(entry.source).second == entry.output);
    }
}
