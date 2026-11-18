///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include <catch2/catch.hpp>
#include <algorithm>
#include <array>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include "plugin_test_helpers.hpp"
#include "plugin_e2e_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

/* Exercise support gap fill through the shared production registry and normal
   Print invalidation. Identical local roofs make object scope measurable without
   comparing translated machine coordinates. Tree signatures include direct flow,
   geometry and ordering, so replay cannot silently keep stale additions.
   An alternate selectable top provider publishes no paths; layers containing
   only its dense interface surfaces isolate provider-independent completion.

       TEST_CASE -> scene / configuration -> process: normal slicing lifecycle
           signature -> write_tree: compare all support nodes or sparse buckets
           contact_trees -> write_tree: compare only excluded top-contact layers
           interface_trees -> write_tree: isolate the alternate provider's pass
       E2E TEST_CASE -> linear_cases: require every declared key to vary
           STL round trip -> process -> export: measure actual deposition

   Prints and models own native data; SDK views are borrowed only during reads.
   Shared runtime initialization precedes configuration creation. */
namespace {
using namespace Slic3r;
namespace api = slic3r_api;
namespace E2E = Slic3r::Test::PluginE2E;
namespace GC = Slic3r::Test::GCode;
constexpr const char *k_option = "support_material_interface_gap_fill";
// Build identical local roof/post/platform objects at separate instance positions.
void scene(Model &, size_t);
// Return a dense-support profile using the shared runtime's defined option.
DynamicPrintConfig configuration(bool);
// Apply settings and run normal validation and processing on an owned Print.
void process(Print &, Model &, const DynamicPrintConfig &);
// Record support paths, tools and layer dimensions; require a nonempty result.
std::string signature(const Print &, size_t, bool sparse = false);
// Capture only layers whose dense surfaces are removable top contacts.
std::map<std::pair<coord_t, coord_t>, std::string> contact_trees(const Print &);
// Record interface buckets on layers with only non-contact top dense surfaces; require such layers.
std::map<std::pair<coord_t, coord_t>, std::string> interface_trees(const Print &);
// Serialize direct flow, order flags and points recursively into the supplied stream.
void write_tree(std::ostream &, const api::ExtrusionEntity &);
// Supply explicit independent or named coupled domains for every provider input.
std::vector<E2E::Domain> domains();
}

// Verify registration precedes immutable option caches and contributes the support layout.
TEST_CASE("Support gap fill defines its disabled object option and layout", "[plugins][support][gap-fill][support-gap-fill]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    REQUIRE(Orchestrator::instance().is_plugin_active("support.toolpath.interface_gap_fill"));
    const ConfigOptionDef *option = PrintConfigDef::instance().get(k_option);
    REQUIRE(option != nullptr);
    REQUIRE(option->default_value->serialize() == "0");
    const std::string layout = Orchestrator::instance().merged_ui_layout("print.ui",
        "page:Support material\ngroup:Options for support material and raft\nsetting:support_material_interface_spacing\n");
    REQUIRE(layout.find(k_option) != std::string::npos);
}

// Traverse enabled thresholds successively on one Print, comparing each state with fresh output.
TEST_CASE("Support gap fill replay equals fresh prints and preserves sparse paths", "[plugins][support][gap-fill][support-gap-fill]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Model model;
    scene(model, 1);
    Print replayed;
    process(replayed, model, configuration(false));
    const std::string original = signature(replayed, 0), sparse = signature(replayed, 0, true);
    std::vector<DynamicPrintConfig> cases{configuration(true)};
    for (const std::pair<const char *, const char *> change : {
        std::pair<const char *, const char *>{"gap_fill_min_area", "1000"},
        {"gap_fill_min_width", "0.8"}, {"gap_fill_max_width", "0.1"},
        {"gap_fill_min_length", "100"}, {"gap_fill_extension", "0.01"}}) {
        DynamicPrintConfig changed = configuration(true);
        changed.set_deserialize_strict(change.first, change.second);
        cases.push_back(std::move(changed));
    }
    cases.push_back(configuration(false));
    for (size_t i = 0; i < cases.size(); ++i) {
        INFO("Sequential replay case " << i);
        process(replayed, model, cases[i]);
        Print fresh;
        process(fresh, model, cases[i]);
        REQUIRE(signature(replayed, 0) == signature(fresh, 0));
        REQUIRE(signature(replayed, 0, true) == sparse);
        if (i == 0)
            REQUIRE(signature(replayed, 0) != original);
    }
    REQUIRE(signature(replayed, 0) == original);
}

// Select an empty alternate top provider and verify completion of its published dense surfaces.
TEST_CASE("Support gap fill completes dense interfaces from another selected provider", "[plugins][support][gap-fill][support-gap-fill]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    const char *id = "test.support_toolpath.alternate";
    REQUIRE(host.get_plugin(id) != nullptr);
    const bool was_active = host.is_plugin_active(id);
    struct RestoreActivation {
        Orchestrator &host;
        const char *id;
        bool active;
        // Restore the borrowed runtime's activation even if an assertion aborts the case.
        ~RestoreActivation() { host.set_plugin_active(id, active); }
    } restore{host, id, was_active};
    REQUIRE(host.set_plugin_active(id, true));
    const std::string selector = Steps::plugin_exclusive_group_option_key(host,
        STEP_SUPPORT_TOOLPATH, "support_toolpath_top_interface");
    DynamicPrintConfig off = configuration(false), on = configuration(true);
    off.set_deserialize_strict(selector, id);
    on.set_deserialize_strict(selector, id);
    const Plugin *selected = Steps::selected_active_plugin_from_group(host, STEP_SUPPORT_TOOLPATH,
        "support_toolpath_top_interface", &on);
    REQUIRE(selected != nullptr);
    REQUIRE(selected->get_id() == id);
    Model model;
    scene(model, 1);
    Print old_print, new_print;
    process(old_print, model, off);
    process(new_print, model, on);
    REQUIRE(interface_trees(old_print) != interface_trees(new_print));
    REQUIRE(signature(old_print, 0, true) == signature(new_print, 0, true));
}

// Swap object overrides on the same Print and require their local outputs to swap exactly.
TEST_CASE("Support gap fill stays object scoped across a settings swap", "[plugins][support][gap-fill][support-gap-fill]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Model model;
    scene(model, 2);
    model.objects()[0].config.set_deserialize_strict(k_option, "1");
    model.objects()[1].config.set_deserialize_strict(k_option, "0");
    Print print;
    const DynamicPrintConfig config = configuration(false);
    process(print, model, config);
    const std::string first = signature(print, 0), second = signature(print, 1);
    REQUIRE(first != second);
    model.objects()[0].config.set_deserialize_strict(k_option, "0");
    model.objects()[1].config.set_deserialize_strict(k_option, "1");
    process(print, model, config);
    REQUIRE(signature(print, 0) == second);
    REQUIRE(signature(print, 1) == first);
}

// Require both tool buckets to exist with the selected interface and base tool ids.
TEST_CASE("Support gap fill keeps base and interface tools separate", "[plugins][support][gap-fill][support-gap-fill]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
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
                    REQUIRE(region.extruder_id() == 1);
                    ++interfaces;
                }
                if (region.has_extrusion(LayerRegionIsland::SUPPORT)) {
                    REQUIRE(region.extruder_id() == 0);
                    ++sparse;
                }
            }
    REQUIRE(interfaces > 0);
    REQUIRE(sparse > 0);
}

// Compare excluded contact layers and deliberately spaced interfaces without altering their trees.
TEST_CASE("Support gap fill leaves removable loops and spaced interfaces unchanged", "[plugins][support][gap-fill][support-gap-fill]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Model model;
    scene(model, 1);
    DynamicPrintConfig off = configuration(false), on = configuration(true);
    SECTION("removable top-contact loops") {
        off.set_deserialize_strict("support_material_interface_contact_loops", "1");
        on.set_deserialize_strict("support_material_interface_contact_loops", "1");
        Print old_print, new_print;
        process(old_print, model, off);
        process(new_print, model, on);
        const std::map<std::pair<coord_t, coord_t>, std::string> before = contact_trees(old_print);
        REQUIRE(!before.empty());
        REQUIRE(before == contact_trees(new_print));
    }
    SECTION("nonzero interface spacing") {
        off.set_deserialize_strict("support_material_interface_spacing", "0.2");
        on.set_deserialize_strict("support_material_interface_spacing", "0.2");
        Print old_print, new_print;
        process(old_print, model, off);
        process(new_print, model, on);
        for (const Print *print : {&old_print, &new_print}) {
            size_t interfaces = 0;
            for (const Layer &layer : print->object(0).auxiliary_layers())
                for (const LayerSliceIsland &island : layer.islands())
                    for (const LayerRegionIsland &region : island.regions_islands())
                        if (region.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE))
                            ++interfaces;
            REQUIRE(interfaces > 0);
        }
        REQUIRE(signature(old_print, 0) == signature(new_print, 0));
    }
}

// Export each declared-input variation, measuring activation and sparse deposition on imported STL.
TEST_CASE("Support gap fill exports a complete linear STL catalogue", "[plugins][support][plugin-e2e][support-gap-fill-e2e][support-gap-fill]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig baseline = configuration(true);
    baseline.set_deserialize_strict({{"step_gcode_plugin", "gcode.printing_plan_file_writer"},
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"},
        {"gcode_comments", "1"}, {"use_relative_e_distances", "0"},
        {"filament_diameter", "1.75,1.75"},
        {"output_filename_format", "support-gap-fill.gcode"}});
    const std::vector<E2E::Case> cases = E2E::linear_cases(Orchestrator::instance(),
        {"support.toolpath.interface_gap_fill"}, baseline, domains());
    Model source;
    scene(source, 1);
    boost::filesystem::path stl = GC::temporary_gcode_path();
    stl.replace_extension(".stl");
    TriangleMesh mesh;
    try {
        TriangleMesh original = source.objects()[0].volumes.front()->mesh();
        REQUIRE(original.write_binary(stl.string().c_str()));
        REQUIRE(mesh.ReadSTLFile(stl.string().c_str()));
        boost::filesystem::remove(stl);
    } catch (...) {
        boost::system::error_code ignored;
        boost::filesystem::remove(stl, ignored);
        throw;
    }
    std::string enabled, disabled, sparse_enabled, sparse_disabled;
    std::vector<std::array<float, 6>> sparse_enabled_moves, sparse_disabled_moves;
    // Bound each generated run to two independent exports without dropping domains.
    const size_t group = GENERATE_COPY(Catch::Generators::range(size_t(0), (cases.size() + 1) / 2));
    for (size_t index = group * 2; index < std::min(cases.size(), (group + 1) * 2); ++index) {
        const E2E::Case &test = cases[index];
        INFO(test.name);
        const DynamicPrintConfig config = E2E::apply_case(baseline, test);
        Model model;
        ModelObject *object = model.add_object();
        object->add_volume(mesh);
        object->add_instance();
        model.center_instances_around_point({100., 100.});
        Print print;
        process(print, model, config);
        const boost::filesystem::path path = GC::temporary_gcode_path();
        std::string code;
        try {
            code = GC::read_text_file(Orchestrator::instance().export_gcode_cli(print, path.string()));
            GC::remove_output_pair(path);
        } catch (...) {
            GC::remove_output_pair(path);
            throw;
        }
        REQUIRE(code.find("G1 ") != std::string::npos);
        // Compare positive deposition, ignoring travel, feed rates and absolute E offsets.
        std::ostringstream deposition;
        std::string role;
        std::vector<std::array<float, 6>> sparse_moves;
        GCodeReader reader;
        reader.apply_config(config);
        reader.parse_buffer(code, [&](GCodeReader &state, const GCodeReader::GCodeLine &line) {
            if (line.comment().rfind("TYPE:", 0) == 0) role = line.comment().substr(5);
            if (!line.extruding(state) || line.dist_XY(state) <= 0.) return;
            std::array<float, 3> first{state.x(), state.y(), state.z()};
            std::array<float, 3> last{line.new_X(state), line.new_Y(state), line.new_Z(state)};
            if (last < first) std::swap(first, last);
            deposition << role << ':' << first[0] << ',' << first[1] << ',' << first[2]
                << ':' << last[0] << ',' << last[1] << ',' << last[2] << '\n';
            if (role == "Support material") sparse_moves.push_back({first[0], first[1], first[2], last[0], last[1], last[2]});
        });
        std::sort(sparse_moves.begin(), sparse_moves.end());
        REQUIRE(!deposition.str().empty());
        if (test.name == "default") {
            enabled = deposition.str();
            sparse_enabled = signature(print, 0, true);
            sparse_enabled_moves = sparse_moves;
        } else if (test.name == "gap fill disabled") {
            disabled = deposition.str();
            sparse_disabled = signature(print, 0, true);
            sparse_disabled_moves = sparse_moves;
        }
    }
    if (group == 0) {
        REQUIRE(!enabled.empty());
        REQUIRE(!disabled.empty());
        REQUIRE(enabled != disabled);
        REQUIRE(sparse_enabled == sparse_disabled);
        REQUIRE(!sparse_enabled_moves.empty());
        REQUIRE(sparse_enabled_moves == sparse_disabled_moves);
    }
}

namespace {
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
    config.set_deserialize_strict({{k_option, enabled ? "1" : "0"}, {"gap_fill_min_area", "0"}, {"seam_visibility", "0"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"skirts", "0"}, {"brim_width", "0"},
        {"support_material", "1"}, {"support_material_style", "grid"},
        {"support_material_contact_distance_type", "plane"}, {"support_material_contact_distance", "0.2"},
        {"support_material_bottom_contact_distance", "0.2"}, {"support_material_interface_layers", "3"},
        {"support_material_bottom_interface_layers", "3"}, {"support_material_interface_spacing", "0"},
        {"support_material_extruder", "1"}, {"support_material_interface_extruder", "1"},
        {"support_material_top_interface_pattern", "concentric"}, {"dont_support_bridges", "0"},
        {"gcode_flavor", "marlin2"}});
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
    REQUIRE(validation.first == PrintBase::pveNone);
    print.set_status_silent();
    print.process();
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
                    write_tree(out, api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(&region.extrusion(role))));
                }
    }
    REQUIRE(trees > 0);
    return out.str();
}

std::map<std::pair<coord_t, coord_t>, std::string> contact_trees(const Print &print)
{
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
}

std::map<std::pair<coord_t, coord_t>, std::string> interface_trees(const Print &print)
{
    std::map<std::pair<coord_t, coord_t>, std::string> result;
    for (const Layer &layer : print.object(0).auxiliary_layers()) {
        bool top_interface = false, other_dense = false;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands()) {
                const layer_region_island_handle *handle = reinterpret_cast<const layer_region_island_handle *>(&region);
                for (uint32_t s = 0; s < layer_region_island_count_fill_surface(handle); ++s) {
                    const raw_surface_type type = surface_get_type(layer_region_island_get_fill_surface(handle, s));
                    const raw_surface_type mask = raw_surface_type(RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_DENS_SOLID);
                    if ((type & mask) != mask) continue;
                    const bool selected_pass = (type & RAW_SURFACE_TYPE_POS_TOP) && !api::surface_type_is_contact(type);
                    top_interface = top_interface || selected_pass;
                    other_dense = other_dense || !selected_pass;
                }
            }
        if (!top_interface || other_dense) continue;
        std::ostringstream tree;
        tree << std::setprecision(17);
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                if (region.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE)) {
                    tree << '#' << region.extruder_id();
                    write_tree(tree, api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(
                        &region.extrusion(LayerRegionIsland::SUPPORT_INTERFACE))));
                }
        result[{layer.scaled_print_z(), layer.scaled_height()}] = tree.str();
    }
    REQUIRE(!result.empty());
    return result;
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

std::vector<E2E::Domain> domains()
{
    return {
        {{k_option}, {{"gap fill disabled", {{k_option, "0"}}}}},
        {{"support_material"}, {{"support disabled", {{"support_material", "0"}}}}},
        {{"raft_layers"}, {{"raft", {{"raft_layers", "2"}}}}},
        {{"support_material_style"}, {{"snug", {{"support_material_style", "snug"}}}}},
        {{"support_material_buildplate_only"}, {{"bed only", {{"support_material_buildplate_only", "1"}}}}},
        {{"support_material_enforce_layers"}, {{"enforced", {{"support_material_enforce_layers", "2"}}}}},
        {{"support_material_threshold"}, {{"threshold", {{"support_material_threshold", "60"}}}}},
        {{"support_material_synchronize_layers"}, {{"synchronized", {{"support_material_synchronize_layers", "1"}}}}},
        {{"support_material_contact_distance_type", "support_material_contact_distance", "support_material_bottom_contact_distance"},
            {{"coupled soluble contacts", {{"support_material_contact_distance_type", "none"},
                {"support_material_contact_distance", "0"}, {"support_material_bottom_contact_distance", "0"}}}}},
        {{"support_material_xy_spacing"}, {{"xy clearance", {{"support_material_xy_spacing", "0.4"}}}}},
        {{"support_material_contact_xy_spacing", "support_material_xy_spacing"},
            {{"coupled contact clearance", {{"support_material_contact_xy_spacing", "0.1"}, {"support_material_xy_spacing", "1"}}}}},
        {{"support_material_contact_cone_growth", "support_material_contact_xy_spacing", "support_material_xy_spacing"},
            {{"coupled contact cone growth", {{"support_material_contact_cone_growth", "25%"},
                {"support_material_contact_xy_spacing", "0.1"}, {"support_material_xy_spacing", "1"}}}}},
        {{"support_material_layer_height", "support_material_interface_layer_height", "layer_height", "first_layer_height"},
            {{"coupled layer heights", {{"support_material_layer_height", "0.15"}, {"support_material_interface_layer_height", "0.15"},
                {"layer_height", "0.15"}, {"first_layer_height", "0.15"}}}}},
        {{"support_material_interface_layers"}, {{"no top interfaces", {{"support_material_interface_layers", "0"}}}}},
        {{"support_material_bottom_interface_layers"}, {{"no bottom interfaces", {{"support_material_bottom_interface_layers", "0"}}}}},
        {{"support_material_interface_spacing"}, {{"spaced interface", {{"support_material_interface_spacing", "0.2"}}}}},
        {{"support_material_spacing"}, {{"base spacing", {{"support_material_spacing", "1.5"}}}}},
        {{"support_material_angle"}, {{"base angle", {{"support_material_angle", "30"}}}}},
        {{"support_material_closing_radius"}, {{"closing radius", {{"support_material_closing_radius", "1"}}}}},
        {{"support_material_bottom_interface_expansion"}, {{"bottom expansion", {{"support_material_bottom_interface_expansion", "0.3"}}}}},
        {{"support_material_extruder", "support_material_interface_extruder", "perimeter_extruder", "infill_extruder", "solid_infill_extruder",
          "nozzle_diameter", "min_layer_height", "max_layer_height"},
            {{"coupled tool assignment", {{"support_material_extruder", "2"}, {"support_material_interface_extruder", "2"},
                {"perimeter_extruder", "2"}, {"infill_extruder", "2"}, {"solid_infill_extruder", "2"},
                {"nozzle_diameter", "0.4,0.5"}, {"min_layer_height", "0.04,0.04"}, {"max_layer_height", "0.25,0.25"}}}}},
        {{"support_material_extrusion_width"}, {{"support width", {{"support_material_extrusion_width", "0.5"}}}}},
        {{"z_step"}, {{"z step", {{"z_step", "0.01"}}}}},
        {{"resolution_internal"}, {{"internal resolution", {{"resolution_internal", "0.03"}}}}},
        {{"extrusion_width", "extrusion_spacing"}, {{"coupled default section", {{"extrusion_width", "0.48"}, {"extrusion_spacing", "0.43"}}}}},
        {{"first_layer_extrusion_width", "first_layer_extrusion_spacing"},
            {{"coupled first section", {{"first_layer_extrusion_width", "0.48"}, {"first_layer_extrusion_spacing", "0.43"}}}}},
        {{"first_layer_infill_extrusion_width", "first_layer_infill_extrusion_spacing"},
            {{"coupled first infill section", {{"first_layer_infill_extrusion_width", "0.48"}, {"first_layer_infill_extrusion_spacing", "0.43"}}}}},
        {{"external_perimeter_extrusion_width", "external_perimeter_extrusion_spacing"},
            {{"coupled outer section", {{"external_perimeter_extrusion_width", "0.48"}, {"external_perimeter_extrusion_spacing", "0.43"}}}}},
        {{"external_perimeter_extrusion_change_odd_layers"}, {{"odd outer section", {{"external_perimeter_extrusion_change_odd_layers", "110%"}}}}},
        {{"external_perimeter_overlap"}, {{"outer overlap", {{"external_perimeter_overlap", "50%"}}}}},
        {{"perimeter_extrusion_width", "perimeter_extrusion_spacing"},
            {{"coupled inner section", {{"perimeter_extrusion_width", "0.48"}, {"perimeter_extrusion_spacing", "0.43"}}}}},
        {{"perimeters"}, {{"single wall", {{"perimeters", "1"}}}}},
        {{"fill_density"}, {{"sparse object", {{"fill_density", "10%"}}}}},
        {{"top_solid_layers"}, {{"top layers", {{"top_solid_layers", "2"}}}}},
        {{"bottom_solid_layers"}, {{"bottom layers", {{"bottom_solid_layers", "2"}}}}},
        {{"solid_infill_every_layers"}, {{"periodic solid", {{"solid_infill_every_layers", "3"}}}}},
        {{"infill_extrusion_width", "infill_extrusion_spacing"},
            {{"coupled infill section", {{"infill_extrusion_width", "0.48"}, {"infill_extrusion_spacing", "0.43"}}}}},
        {{"infill_extrusion_change_odd_layers"}, {{"odd infill section", {{"infill_extrusion_change_odd_layers", "110%"}}}}},
        {{"bridge_type"}, {{"bridge section", {{"bridge_type", "flow"}}}}},
        {{"bridge_flow_ratio"}, {{"bridge flow", {{"bridge_flow_ratio", "90%"}}}}},
        {{"filament_max_overlap"}, {{"filament overlap", {{"filament_max_overlap", "80%"}}}}},
        {{"raft_first_layer_expansion"}, {{"raft expansion", {{"raft_first_layer_expansion", "4"}}}}},
        {{"support_material_pattern"}, {{"base grid", {{"support_material_pattern", "rectilinear-grid"}}}}},
        {{"support_material_with_sheath"}, {{"base sheath", {{"support_material_with_sheath", "0"}}}}},
        {{"support_material_top_interface_pattern"}, {{"top lines", {{"support_material_top_interface_pattern", "rectilinear"}}}}},
        {{"support_material_bottom_interface_pattern"}, {{"bottom lines", {{"support_material_bottom_interface_pattern", "rectilinear"}}}}},
        {{"support_material_interface_contact_loops"}, {{"contact loops", {{"support_material_interface_contact_loops", "1"}}}}},
        {{"support_material_angle_height"}, {{"base angle height", {{"support_material_angle_height", "1"}}}}},
        {{"support_material_interface_angle"}, {{"interface angle", {{"support_material_interface_angle", "30"}}}}},
        {{"support_material_interface_angle_increment"}, {{"interface rotation", {{"support_material_interface_angle_increment", "30"}}}}},
        {{"raft_first_layer_density"}, {{"raft density", {{"raft_first_layer_density", "80%"}}}}},
        {{"gap_fill_min_area"}, {{"gap area", {{"gap_fill_min_area", "1000"}}}}},
        {{"gap_fill_min_width"}, {{"gap minimum", {{"gap_fill_min_width", "0.8"}}}}},
        {{"gap_fill_max_width"}, {{"gap maximum", {{"gap_fill_max_width", "0.1"}}}}},
        {{"gap_fill_min_length"}, {{"gap length", {{"gap_fill_min_length", "100"}}}}},
        {{"gap_fill_extension"}, {{"gap extension", {{"gap_fill_extension", "0.01"}}}}},
        {{"fill_smooth_width"}, {{"smooth width", {{"fill_smooth_width", "60%"}}}}},
        {{"fill_smooth_distribution"}, {{"smooth distribution", {{"fill_smooth_distribution", "50%"}}}}}
    };
}
}
