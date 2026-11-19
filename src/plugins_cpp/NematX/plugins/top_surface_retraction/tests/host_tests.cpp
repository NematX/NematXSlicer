///|/ Copyright (c) 2026 NematX. AGPL-3.0-or-later.
#include <filesystem>
#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tbb/global_control.h>
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Plugins/PluginLoader.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/SLA/SLAPrintConfig.hpp"
#include "libslic3r/Utils.hpp"

/* Production replay and provenance gate for the top-surface policy.
   Unlike isolated scope fixtures, these prints use the complete registry and
   real plan checkpoints. They compare a changed live Print against a fresh
   Print, first on two true instances and then with supports and multiple tools.

       main -> initialize production runtime and check the setting catalogue
            -> replay: process on -> apply off -> fresh off -> apply on
                -> snapshot: paired retractions by height, deposited volume

   Exports belong to the supplied data directory. Failures throw
   in every build; no assertion is disabled in Release builds. */
namespace {
using namespace Slic3r;
constexpr const char *setting = "enforce_retract_top_surface";
size_t checks = 0;

// Fail visibly and count each accepted condition.
void check(bool condition, const std::string &message);
// Initialize the actual plugins before configuration caches become immutable.
void initialize(const std::filesystem::path &data, const std::filesystem::path &defaults,
                const std::filesystem::path &resources);
// Return a small connected-infill-free production profile.
DynamicPrintConfig configuration();
// Count semantic events recursively, resolving inherited physical attributes.
void count(const ExtrusionEntity &entity, raw_extrusion_role inherited, size_t &retracts, size_t &supports);

struct Snapshot {
    std::map<coord_t, size_t> retracts;
    double volume = 0.;
    size_t supports = 0;
    std::set<extruder_index_t> tools;
};
// Inspect final semantic retractions and physical deposits independently.
Snapshot snapshot(const Print &print);
// Reapply a setting in the same Print and compare against both fresh configurations.
void replay(Model &model, DynamicPrintConfig config, bool require_support, bool require_tools,
            const std::filesystem::path &output);

void check(bool condition, const std::string &message)
{
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

void initialize(const std::filesystem::path &data, const std::filesystem::path &defaults,
                const std::filesystem::path &resources)
{
    std::filesystem::create_directories(data / "plugins");
    std::filesystem::copy_file(defaults, data / "plugins/activated.ini", std::filesystem::copy_options::overwrite_existing);
    set_data_dir(data.string()); set_resources_dir(resources.string()); set_logging_level(0);
    PrintConfigDef::instance_mutable().init_common_params();
    init_fff_params(PrintConfigDef::instance_mutable()); init_sla_params(PrintConfigDef::instance_mutable());
    load_plugins();
    initialize_fff_print_config_cache(); initialize_sla_print_config_cache();
    PrintConfigDef::instance_mutable().finalize();
    const Plugin *plugin = Orchestrator::instance().get_plugin("nematx_top_surface_retraction");
    check(plugin != nullptr && Orchestrator::instance().is_plugin_active(plugin->get_id()), "missing top-surface provider");
    std::set<std::string> used;
    for (const Plugin::UsedConfigKey &key : plugin->get_used_config_keys()) used.insert(key.key);
    check(used == std::set<std::string>{setting}, "declared setting lacks an export variation domain");
    std::set<std::string> defined;
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys()) defined.insert(key.key);
    check(defined == used, "defined setting lacks an export variation domain");
    check(DynamicPrintConfig::full_print_config().option(setting)->serialize() == "1", "wrong historical default");
}

DynamicPrintConfig configuration()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{setting, "1"}, {"only_retract_when_crossing_perimeters", "1"},
        {"enforce_retract_first_layer", "0"}, {"retract_before_travel", "0.1"}, {"retract_length", "2"},
        {"retract_layer_change", "0"}, {"seam_visibility", "0"}, {"layer_height", "0.2"},
        {"first_layer_height", "0.2"}, {"skirts", "0"}, {"brim_width", "0"}, {"perimeters", "1"},
        {"fill_density", "20%"}, {"top_solid_layers", "1"}, {"bottom_solid_layers", "0"},
        {"top_fill_pattern", "rectilinear"}, {"bottom_fill_pattern", "rectilinear"},
        {"infill_connection", "notconnected"}, {"infill_connection_solid", "notconnected"},
        {"infill_connection_top", "notconnected"}, {"infill_connection_bottom", "notconnected"},
        {"gcode_flavor", "marlin2"}});
    return config;
}

void count(const ExtrusionEntity &entity, raw_extrusion_role role, size_t &retracts, size_t &supports)
{
    if (const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>()) role = raw_extrusion_role(attributes->role);
    if (entity.child_count() == 0) {
        const ExtrusionPropertyExtrusionAxis *axis = entity.get_property<ExtrusionPropertyExtrusionAxis>();
        retracts += axis != nullptr && axis->operation == C_EXTRUSION_AXIS_OPERATION_RETRACT_TO;
        supports += RAW_EXTRUSION_ROLE_IS_SUPPORT(role);
    }
    for (size_t idx = 0; idx < entity.child_count(); ++idx) count(entity.child(idx), role, retracts, supports);
}

Snapshot snapshot(const Print &print)
{
    Snapshot result;
    check(print.printing_plan() != nullptr, "no printing plan");
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers) {
            size_t &retracts = result.retracts[layer.print_z];
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups) {
                result.tools.insert(tool.extruder_id);
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                    count(*extrusion.root, RAW_EXTRUSION_ROLE_NONE, retracts, result.supports);
                    result.volume += extrusion.root->total_volume();
                }
            }
        }
    return result;
}

void replay(Model &model, DynamicPrintConfig config, bool require_support, bool require_tools,
            const std::filesystem::path &output)
{
    std::filesystem::create_directories(output);
    Print live;
    for (ModelObject &object : model.objects()) live.auto_assign_extruders(&object);
    live.apply(model, config); live.set_status_silent(); live.process();
    Orchestrator::instance().export_gcode(live, (output / "on.gcode").string(), nullptr);
    const Snapshot on = snapshot(live);
    check(on.volume > 0., "empty deposits");
    check(!require_support || on.supports > 0, "fixture has no support paths");
    check(!require_tools || on.tools.size() == 2, "fixture does not use two tools");
    config.set_deserialize_strict({{setting, "0"}});
    live.apply(model, config); live.process();
    Orchestrator::instance().export_gcode(live, (output / "off.gcode").string(), nullptr);
    const Snapshot off = snapshot(live);
    Print fresh;
    fresh.apply(model, config); fresh.set_status_silent(); fresh.process();
    Orchestrator::instance().export_gcode(fresh, (output / "fresh-off.gcode").string(), nullptr);
    const Snapshot fresh_off = snapshot(fresh);
    check(off.retracts == fresh_off.retracts, "off replay differs from fresh Print");
    check(std::abs(off.volume - fresh_off.volume) < 1e-6, "off replay changed deposits");
    check(std::abs(on.volume - off.volume) < 1e-6, "top policy changed physical volume");
    check(on.retracts != off.retracts, "fixture shows no top-surface effect");
    config.set_deserialize_strict({{setting, "1"}});
    live.apply(model, config); live.process();
    Orchestrator::instance().export_gcode(live, (output / "again.gcode").string(), nullptr);
    const Snapshot again = snapshot(live);
    check(again.retracts == on.retracts, "on replay retained stale off decisions");
    check(std::abs(again.volume - on.volume) < 1e-6, "on replay changed deposits");
}
}

// Run bounded small scenes with the production plugin registry; preserve failures.
int main(int argc, char **argv)
{
    try {
        if (argc != 4) throw std::runtime_error("Expected data directory, defaults and resources.");
        tbb::global_control workers(tbb::global_control::max_allowed_parallelism, 2);
        initialize(argv[1], argv[2], argv[3]);
        Model copies;
        ModelObject *cube = copies.add_object("copies", "copies.stl", make_cube(8, 8, 1.2));
        cube->add_instance()->set_offset(Vec3d(10, 10, 0));
        cube->add_instance()->set_offset(Vec3d(30, 10, 0));
        replay(copies, configuration(), false, false, std::filesystem::path(argv[1]) / "copies");

        Model supported;
        ModelObject *overhang = supported.add_object("overhang", "overhang.stl", make_cube(3, 3, 1.2));
        TriangleMesh roof = make_cube(8, 8, .4f); roof.translate(-2.5f, -2.5f, 1.2f);
        overhang->add_volume(std::move(roof)); overhang->add_instance();
        DynamicPrintConfig support = configuration();
        support.set_deserialize_strict({{"support_material", "1"}, {"dont_support_bridges", "0"}});
        replay(supported, support, true, false, std::filesystem::path(argv[1]) / "support");

        Model tools;
        for (int idx = 0; idx < 2; ++idx) {
            ModelObject *object = tools.add_object("tool", "tool.stl", make_cube(8, 8, 1.2));
            object->config.set_deserialize_strict("extruder", idx == 0 ? "1" : "2");
            object->add_instance()->set_offset(Vec3d(10 + idx * 20, 10, 0));
        }
        DynamicPrintConfig multi = configuration();
        multi.set_num_extruders(2);
        multi.set_deserialize_strict({{"nozzle_diameter", "0.4,0.4"}, {"filament_diameter", "1.75,1.75"},
            {"retract_length", "2,2"}, {"retract_before_travel", "0.1,0.1"}, {"retract_layer_change", "0,0"}});
        replay(tools, multi, false, true, std::filesystem::path(argv[1]) / "tools");
        std::cout << checks << " checks passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
