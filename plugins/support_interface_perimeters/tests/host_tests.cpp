///|/ Released under the GNU Affero General Public License, version 3 or later.
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#ifdef _WIN32
#include <crtdbg.h>
#endif

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Plugins/PluginLoader.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/SLA/SLAPrintConfig.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Utils.hpp"

/* Native integration checks use the normal plugin loader in a separate process
   so the production providers and their settings do not alter the shared core
   test runtime. Complete support trees are compared after changing the count
   on an already processed Print and against a fresh Print. Two objects then
   exchange their own counts, detecting cached cross-object state.

       main -> initialize: load the embedded providers and finalize settings
            -> replay: compare each recomputed tree with a fresh print
            -> objects: swap per-object settings and compare independent trees
       signature -> tree_signature: include geometry, ordering flags and flow */

namespace {
using namespace Slic3r;
namespace api = slic3r_api;
constexpr const char *k_count = "support_material_interface_perimeters";

// Report a failed comparison with a nonzero process exit, also in Release builds.
void require(bool condition, const char *message);
// Register production providers before configuration caches become immutable.
void initialize(const std::filesystem::path &data, const std::filesystem::path &defaults, const std::filesystem::path &resources);
// Build a roof and platform like the support fixtures, with separate model objects for object-local settings.
void scene(Model &model, size_t count);
// Return a small valid configuration with dense interfaces and deterministic, inexpensive model paths.
DynamicPrintConfig configuration(int count);
// Validate and process a model without exporting G-code.
void process(Print &print, Model &model, const DynamicPrintConfig &config);
// Include direct attributes and each node's flags and points, not only total extrusion length.
void tree_signature(std::ostream &out, const api::ExtrusionEntity &entity);
// Snapshot one object's complete support trees, or its sparse trees alone.
std::string signature(const Print &print, size_t object_index, bool sparse_only = false);
// Compare replayed counts against fresh prints and restore the original zero-count tree.
void replay();
// Exercise distinct counts on two objects, then exchange them on the same Print.
void objects();
// Keep interface and sparse paths in their separately selected extruders.
void tools();
// Export neutral and enabled interfaces.
void export_smoke(const std::filesystem::path &directory);

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

void initialize(const std::filesystem::path &data, const std::filesystem::path &defaults, const std::filesystem::path &resources)
{
    std::filesystem::create_directories(data / "plugins");
    std::filesystem::copy_file(defaults, data / "plugins/activated.ini", std::filesystem::copy_options::overwrite_existing);
    set_data_dir(data.string());
    set_resources_dir(resources.string());
    set_logging_level(0);
    PrintConfigDef::instance_mutable().init_common_params();
    init_fff_params(PrintConfigDef::instance_mutable());
    init_sla_params(PrintConfigDef::instance_mutable());
    load_plugins();
    for (const char *id : {"nematx.support.bottom_contact", "nematx.support.bottom_interface", "nematx.support.top_contact", "nematx.support.top_interface"}) {
        if (!Orchestrator::instance().is_plugin_active(id)) {
            std::ostringstream message;
            message << "Required provider is inactive: " << id;
            for (const Orchestrator::PluginMessage &diagnostic : Orchestrator::instance().consume_plugin_messages())
                message << '\n' << diagnostic.plugin_id << ": " << diagnostic.message;
            throw std::runtime_error(message.str());
        }
    }
    initialize_fff_print_config_cache();
    initialize_sla_print_config_cache();
    PrintConfigDef::instance_mutable().finalize();
    require(PrintConfigDef::instance().get(k_count) != nullptr, "perimeter setting not registered");
    require(PrintConfigDef::instance().get(k_count)->default_value->serialize() == "0", "perimeter default is not zero");
    require(DynamicPrintConfig::full_print_config().option("gcode_firmware_plugin")->serialize() == "gcode.firmware.marlin2",
        "the NematX firmware metadata changed the default firmware");
    const std::string layout = Orchestrator::instance().merged_ui_layout("print.ui",
        "page:Support material\ngroup:Options for support material and raft\nsetting:support_material_interface_contact_loops\n");
    require(layout.find(k_count) != std::string::npos, "interface perimeter setting is missing from the support layout");
}

void scene(Model &model, size_t count)
{
    for (size_t index = 0; index < count; ++index) {
        TriangleMesh mesh = make_cube(3., 8., 4.);
        TriangleMesh roof = make_cube(12., 8., 1.);
        roof.translate(0., 0., 4.);
        mesh.merge(roof);
        mesh.merge(make_cube(12., 8., 0.6));
        ModelObject *object = model.add_object();
        object->name = "support-roof";
        object->add_volume(std::move(mesh));
        object->add_instance()->set_offset(Vec3d(20. * index, 0., 0.));
    }
    model.center_instances_around_point({100., 100.});
}

DynamicPrintConfig configuration(int count)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{k_count, std::to_string(count)}, {"seam_visibility", "0"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"skirts", "0"}, {"brim_width", "0"},
        {"support_material", "1"}, {"support_material_style", "grid"},
        {"support_material_contact_distance_type", "plane"}, {"support_material_contact_distance", "0.2"},
        {"support_material_bottom_contact_distance", "0.2"}, {"support_material_interface_layers", "3"},
        {"support_material_bottom_interface_layers", "3"}, {"support_material_interface_spacing", "0"},
        {"support_material_extruder", "1"}, {"support_material_interface_extruder", "1"},
        {"support_material_top_interface_pattern", "concentric"}, {"dont_support_bridges", "0"},
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"}});
    return config;
}

void process(Print &print, Model &model, const DynamicPrintConfig &config)
{
    for (ModelObject &object : model.objects()) {
        object.ensure_on_bed();
        print.auto_assign_extruders(&object);
    }
    print.apply(model, config);
    const std::pair<PrintBase::PrintValidationError, std::string> validation = print.validate();
    if (validation.first != PrintBase::pveNone) throw std::runtime_error("Invalid integration configuration: " + validation.second);
    print.set_status_silent();
    print.process();
}

void tree_signature(std::ostream &out, const api::ExtrusionEntity &entity)
{
    out << '(' << entity.flags();
    if (const api::EPropertyAttributes *attributes = entity.get(api::EPropertyAttributes::key)) {
        const c_extrusion_property_attributes &raw = *attributes;
        out << ':' << attributes->extrusion_role() << ':' << attributes->mm3_per_mm() << ':' << raw.width << ':' << raw.height;
    }
    if (entity.has_polyline())
        for (const c_point point : entity.points()) out << '[' << point.x << ',' << point.y << ']';
    for (uint32_t index = 0; index < entity.child_count(); ++index) tree_signature(out, entity.child(index));
    out << ')';
}

std::string signature(const Print &print, size_t object_index, bool sparse_only)
{
    std::ostringstream out;
    out << std::setprecision(17);
    size_t trees = 0;
    for (const Layer &layer : print.object(object_index).auxiliary_layers()) {
        out << '@' << layer.scaled_print_z() << ':' << layer.scaled_height();
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const ExtrusionRole role : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE}) {
                    if ((sparse_only && role != LayerRegionIsland::SUPPORT) || !region.has_extrusion(role)) continue;
                    ++trees;
                    const ExtrusionEntity &tree = region.extrusion(role);
                    tree_signature(out, api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(&tree)));
                }
    }
    require(trees > 0, "integration fixture produced no support paths");
    return out.str();
}

void replay()
{
    Model model;
    scene(model, 1);
    Print replayed;
    process(replayed, model, configuration(0));
    const std::string original = signature(replayed, 0);
    const std::string sparse = signature(replayed, 0, true);
    for (const int count : {1, 2, 3, 0}) {
        const DynamicPrintConfig config = configuration(count);
        process(replayed, model, config);
        Print fresh;
        process(fresh, model, config);
        require(signature(replayed, 0) == signature(fresh, 0), "re-slice differs from a fresh Print");
        require(signature(replayed, 0, true) == sparse, "interface setting changed sparse trees");
        if ((signature(replayed, 0) == original) != (count == 0)) {
            const ConfigOption *effective = replayed.object(0).config().option(k_count);
            const Plugin *selected = Steps::selected_active_plugin_from_group(Orchestrator::instance(), STEP_SUPPORT_TOOLPATH,
                "support_toolpath_top_interface", &replayed.object(0).config());
            const std::string key = Steps::plugin_exclusive_group_option_key(Orchestrator::instance(), STEP_SUPPORT_TOOLPATH, "support_toolpath_top_interface");
            const ConfigOption *selector = replayed.object(0).config().option(key);
            throw std::runtime_error("Count was ignored or zero did not restore original paths: requested=" + std::to_string(count) +
                ", effective=" + (effective ? effective->serialize() : "<missing>") + ", provider=" + (selected ? selected->get_id() : "<none>") +
                ", selector=" + (selector ? selector->serialize() : "<missing>"));
        }
    }
}

void objects()
{
    Model model;
    scene(model, 2);
    model.objects()[0].config.set_deserialize_strict(k_count, "1");
    model.objects()[1].config.set_deserialize_strict(k_count, "2");
    Print print;
    const DynamicPrintConfig config = configuration(0);
    process(print, model, config);
    const std::string first = signature(print, 0), second = signature(print, 1);
    require(first != second, "per-object counts were ignored");
    model.objects()[0].config.set_deserialize_strict(k_count, "2");
    model.objects()[1].config.set_deserialize_strict(k_count, "1");
    process(print, model, config);
    require(signature(print, 0) == second && signature(print, 1) == first, "per-object state leaked across replay");
}

void tools()
{
    Model model;
    scene(model, 1);
    DynamicPrintConfig config = configuration(2);
    config.set_deserialize_strict({{"nozzle_diameter", "0.4,0.4"}, {"min_layer_height", "0.05,0.05"},
        {"max_layer_height", "0.3,0.3"}, {"support_material_interface_extruder", "2"}});
    Print print;
    process(print, model, config);
    size_t interfaces = 0, sparse = 0;
    for (const Layer &layer : print.object(0).auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands()) {
                const api::LayerRegionIsland view(reinterpret_cast<const layer_region_island_handle *>(&region));
                if (region.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE)) {
                    require(view.extruder_id() == 1, "interface paths used the sparse tool");
                    ++interfaces;
                }
                if (region.has_extrusion(LayerRegionIsland::SUPPORT)) {
                    require(view.extruder_id() == 0, "sparse paths used the interface tool");
                    ++sparse;
                }
            }
    require(interfaces > 0 && sparse > 0, "two-tool fixture did not exercise both buckets");
}

void export_smoke(const std::filesystem::path &directory)
{
    for (const int count : {0, 2}) {
        Model model;
        scene(model, 1);
        Print print;
        process(print, model, configuration(count));
        print.set_status_default();
        const std::filesystem::path output = directory / ("native-" + std::to_string(count) + ".gcode");
        Orchestrator::instance().export_gcode(print, output.string(), nullptr, nullptr);
        require(std::filesystem::exists(output) && std::filesystem::file_size(output) > 0, "native export produced no G-code");
    }
}
}

// Paths are explicit arguments so the test never reads or mutates the user's normal datadir.
int main(int argc, char **argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::setvbuf(stderr, nullptr, _IONBF, 0);
#ifdef _WIN32
    _set_error_mode(_OUT_TO_STDERR);
    _set_abort_behavior(0, _CALL_REPORTFAULT);
#endif
    try {
        require(argc == 4 || (argc == 5 && std::string(argv[4]) == "--export-only"), "expected isolated datadir, activation defaults and source resources");
        initialize(argv[1], argv[2], argv[3]);
        if (argc == 5) {
            export_smoke(argv[1]);
            return 0;
        }
        replay();
        objects();
        tools();
        std::cout << "Support interface replay and per-object checks passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
