///|/ Released under the GNU Affero General Public License, version 3 or later.
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#ifdef _WIN32
#include <crtdbg.h>
#endif

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
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

/* Native checks run the embedded provider with production configuration and
   lifecycle handling. Unlike separate CLI exports, replaying the same Print
   detects stale paths after invalidation and object-local state leaks.

       main -> initialize: register production providers and immutable settings
            -> replay: compare toggles and thresholds against fresh prints
            -> objects: exchange settings on two identical model objects
            -> tools: retain separate base and interface extruders
            -> loops: leave removable top-contact loops and their trees intact
       signature -> write_tree: record node order, flags, geometry and flow

   Each process uses an explicit private datadir and contains failures in main.
   These tests stop before G-code export. */
namespace {
using namespace Slic3r;
namespace api = slic3r_api;
constexpr const char *k_option = "support_material_interface_gap_fill";
// Fail visibly with a useful message in every build configuration.
void require(bool, const char *);
// Load only the explicit test profile and initialize the production registry.
void initialize(const std::filesystem::path &, const std::filesystem::path &, const std::filesystem::path &);
// Build repeated roof/post/platform objects with the same local geometry.
void scene(Model &, size_t);
// Return a small dense-support profile with no initial interface perimeters.
DynamicPrintConfig configuration(bool);
// Apply object settings and run the normal slicing lifecycle on a Print.
void process(Print &, Model &, const DynamicPrintConfig &);
// Include every node's shape, direct flow and ordering flags in the signature.
void write_tree(std::ostream &, const api::ExtrusionEntity &);
// Capture support paths, tool assignments and layer dimensions for one object.
std::string signature(const Print &, size_t, bool sparse = false);
// Replay option, threshold, selector and perimeter changes against fresh Prints.
void replay();
// Detect state shared between concurrent object callbacks or subsequent runs.
void objects();
// Keep all interface roots on the selected interface tool, including additions.
void tools();
// Contacts with removable loops must retain exactly their old extrusion trees.
void loops();

void require(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
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
    require(Orchestrator::instance().is_plugin_active("nematx.support.interface_gap_fill"), "Embedded gap-fill provider is inactive");
    initialize_fff_print_config_cache();
    initialize_sla_print_config_cache();
    PrintConfigDef::instance_mutable().finalize();
    const ConfigOptionDef *option = PrintConfigDef::instance().get(k_option);
    require(option && option->default_value->serialize() == "0", "Gap-fill default is not false");
    const std::string layout = Orchestrator::instance().merged_ui_layout("print.ui",
        "page:Support material\ngroup:Options for support material and raft\nsetting:support_material_interface_spacing\n");
    require(layout.find(k_option) != std::string::npos, "Gap-fill option is absent from the support layout");
}

void scene(Model &model, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        TriangleMesh mesh = make_cube(3., 8., 4.);
        TriangleMesh roof = make_cube(12., 8., 1.);
        roof.translate(0., 0., 4.);
        mesh.merge(roof);
        mesh.merge(make_cube(12., 8., 0.6));
        ModelObject *object = model.add_object();
        object->name = "gap-fill-roof";
        object->add_volume(std::move(mesh));
        object->add_instance()->set_offset(Vec3d(20. * i, 0., 0.));
    }
    model.center_instances_around_point({100., 100.});
}

DynamicPrintConfig configuration(bool enabled)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{k_option, enabled ? "1" : "0"}, {"gap_fill_min_area", "0"},
        {"support_material_interface_perimeters", "0"}, {"seam_visibility", "0"},
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
    if (validation.first != PrintBase::pveNone) throw std::runtime_error(validation.second);
    print.set_status_silent();
    print.process();
}

void write_tree(std::ostream &out, const api::ExtrusionEntity &entity)
{
    out << '(' << entity.flags();
    if (const api::EPropertyAttributes *attributes = entity.get(api::EPropertyAttributes::key)) {
        const c_extrusion_property_attributes &raw = *attributes;
        out << ':' << raw.role << ':' << raw.mm3_per_mm << ':' << raw.width << ':' << raw.height;
    }
    if (entity.has_polyline())
        for (const c_point point : entity.points()) out << '[' << point.x << ',' << point.y << ']';
    for (uint32_t i = 0; i < entity.child_count(); ++i) write_tree(out, entity.child(i));
    out << ')';
}

std::string signature(const Print &print, size_t object_index, bool sparse)
{
    std::ostringstream out;
    out << std::setprecision(17);
    size_t trees = 0;
    for (const Layer &layer : print.object(object_index).auxiliary_layers()) {
        out << '@' << layer.scaled_print_z() << ':' << layer.scaled_height();
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const ExtrusionRole role : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE}) {
                    if ((sparse && role != LayerRegionIsland::SUPPORT) || !region.has_extrusion(role)) continue;
                    ++trees;
                    out << '#' << region.extruder_id();
                    const ExtrusionEntity &tree = region.extrusion(role);
                    write_tree(out, api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(&tree)));
                }
    }
    require(trees > 0, "Fixture produced no support paths");
    return out.str();
}

void replay()
{
    Model model;
    scene(model, 1);
    Print replayed;
    process(replayed, model, configuration(false));
    const std::string original = signature(replayed, 0), sparse = signature(replayed, 0, true);
    std::vector<DynamicPrintConfig> cases;
    cases.push_back(configuration(true));
    for (const std::pair<const char *, const char *> change : {
        std::pair<const char *, const char *>{"gap_fill_min_area", "1000"},
        {"gap_fill_min_width", "0.8"}, {"gap_fill_max_width", "0.1"},
        {"gap_fill_min_length", "100"}, {"gap_fill_extension", "0.01"},
        {"support_material_interface_perimeters", "1"}, {"support_material_interface_perimeters", "3"}}) {
        DynamicPrintConfig config = configuration(true);
        config.set_deserialize_strict(change.first, change.second);
        cases.push_back(std::move(config));
    }
    cases.push_back(configuration(false));
    for (size_t i = 0; i < cases.size(); ++i) {
        process(replayed, model, cases[i]);
        Print fresh;
        process(fresh, model, cases[i]);
        if (signature(replayed, 0) != signature(fresh, 0))
            throw std::runtime_error("Same-Print replay differs from a fresh Print at case " + std::to_string(i));
        require(signature(replayed, 0, true) == sparse, "Gap-fill configuration changed sparse paths");
        if (i == 0) require(signature(replayed, 0) != original, "Enabled gap fill had no effect");
    }
    require(signature(replayed, 0) == original, "Disabling gap fill did not restore the old trees");
}

void objects()
{
    Model model;
    scene(model, 2);
    model.objects()[0].config.set_deserialize_strict(k_option, "1");
    model.objects()[1].config.set_deserialize_strict(k_option, "0");
    Print print;
    const DynamicPrintConfig config = configuration(false);
    process(print, model, config);
    const std::string first = signature(print, 0), second = signature(print, 1);
    require(first != second, "Per-object gap-fill option was ignored");
    model.objects()[0].config.set_deserialize_strict(k_option, "0");
    model.objects()[1].config.set_deserialize_strict(k_option, "1");
    process(print, model, config);
    require(signature(print, 0) == second && signature(print, 1) == first, "Per-object gap-fill state leaked");
}

void tools()
{
    Model model;
    scene(model, 1);
    DynamicPrintConfig config = configuration(true);
    config.set_deserialize_strict({{"nozzle_diameter", "0.4,0.4"}, {"min_layer_height", "0.05,0.05"},
        {"max_layer_height", "0.3,0.3"}, {"support_material_interface_extruder", "2"}});
    Print print;
    process(print, model, config);
    size_t interfaces = 0, sparse = 0;
    for (const Layer &layer : print.object(0).auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands()) {
                if (region.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE)) {
                    require(region.extruder_id() == 1, "Interface used the base tool");
                    ++interfaces;
                }
                if (region.has_extrusion(LayerRegionIsland::SUPPORT)) {
                    require(region.extruder_id() == 0, "Base used the interface tool");
                    ++sparse;
                }
            }
    require(interfaces && sparse, "Two-tool fixture did not exercise both buckets");
}

void loops()
{
    Model model;
    scene(model, 1);
    DynamicPrintConfig off = configuration(false), on = configuration(true);
    off.set_deserialize_strict("support_material_interface_contact_loops", "1");
    on.set_deserialize_strict("support_material_interface_contact_loops", "1");
    Print old_print, new_print;
    process(old_print, model, off);
    process(new_print, model, on);
    // Compare layers whose only dense surfaces are the excluded top contacts.
    const auto contact_trees = [](const Print &print) {
        std::map<std::pair<coord_t, coord_t>, std::string> result;
        for (const Layer &layer : print.object(0).auxiliary_layers()) {
            bool contact = false, other_dense = false;
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region : island.regions_islands()) {
                    const layer_region_island_handle *handle = reinterpret_cast<const layer_region_island_handle *>(&region);
                    for (uint32_t s = 0; s < layer_region_island_count_fill_surface(handle); ++s) {
                        const raw_surface_type type = surface_get_type(layer_region_island_get_fill_surface(handle, s));
                        const raw_surface_type mask = raw_surface_type(RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_DENS_SOLID);
                        if ((type & mask) != mask) continue;
                        const bool top_contact = (type & RAW_SURFACE_TYPE_POS_TOP) && api::surface_type_is_contact(type);
                        contact = contact || top_contact;
                        other_dense = other_dense || !top_contact;
                    }
                }
            if (!contact || other_dense) continue;
            std::ostringstream tree;
            tree << std::setprecision(17);
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region : island.regions_islands())
                    for (const ExtrusionRole role : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE})
                        if (region.has_extrusion(role)) {
                            tree << '#' << region.extruder_id();
                            write_tree(tree, api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(&region.extrusion(role))));
                        }
            result[{layer.scaled_print_z(), layer.scaled_height()}] = tree.str();
        }
        return result;
    };
    const std::map<std::pair<coord_t, coord_t>, std::string> before = contact_trees(old_print);
    require(!before.empty(), "Loop fixture did not expose an excluded top contact");
    require(before == contact_trees(new_print), "Gap fill changed a top contact with removable loops");
}
}

// No normal user profile is read; failures return nonzero without an assertion dialog.
int main(int argc, char **argv)
{
#ifdef _WIN32
    _set_error_mode(_OUT_TO_STDERR);
    _set_abort_behavior(0, _CALL_REPORTFAULT);
#endif
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    try {
        require(argc == 4, "Expected isolated datadir, activation defaults and source resources");
        initialize(argv[1], argv[2], argv[3]);
        replay();
        objects();
        tools();
        loops();
        std::cout << "Gap-fill replay, object scope and tool checks passed\n";
        return 0;
    } catch (const std::exception &failure) {
        std::cerr << failure.what() << '\n';
        return 1;
    }
}
