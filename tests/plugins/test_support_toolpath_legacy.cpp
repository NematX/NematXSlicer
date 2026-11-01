#include "support_grid_test_helpers.hpp"
#include "probe_pattern_plugin.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Plugins/Support/Toolpath/LegacySupportPaths.hpp"
#include "libslic3r/Plugins/Support/Common/SupportCloseZ.hpp"
#include "libslic3r/Plugins/Support/Common/SupportCounters.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

/* Small roof-and-platform objects exercise the three built-in support roles.
   The slicing prefix leaves support editable, then the normal dispatcher fills
   the typed surfaces. Leaf checks resolve inherited flow attributes, so both
   native paths and generic API wrappers are tested in their published form.
   A failing pattern checks that a legacy algorithm preserves its sole error message.
   Two cases drive the internal helpers directly: synthetic slabs for height
    modulation and plain arguments for the filler pattern choice. A variable-height grid/tree case runs the full support
    toolpath dispatcher, reports modulation counters and checks published heights and height-scaled flow. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
namespace ToolpathInternal = slic3r_api::Support::Toolpath::Internal;
namespace {
// Scope the active providers like the infill pattern tests, restoring the saved plugin pointers afterward.
class ScopedActivePlugins
{
public:
    // Save the process-owned providers before the caller changes activation; allocation failure leaves activation intact.
    explicit ScopedActivePlugins() : m_host(Orchestrator::instance())
    {
        for (Plugin *plugin : m_host.active_plugins())
            m_previous.push_back(plugin);
    }
    // Restore activation without transferring ownership of the process-lifetime plugin instances.
    ~ScopedActivePlugins()
    {
        m_host.clear_active_plugins();
        for (Plugin *plugin : m_previous)
            m_host.set_plugin_active(plugin, true);
    }
private:
    Orchestrator &m_host;
    std::vector<Plugin *> m_previous;
};

struct PathCounts { size_t base = 0, interface = 0, loops = 0, bridges = 0; double first_width = 0; };
// Check leaf role, positive flow and bucket placement and count loops and bridge flows.
void check_paths(const ExtrusionEntity &entity, ExtrusionRole bucket, PathCounts &counts, bool first, const ExtrusionAttributes *attributes = nullptr);

// Collect the polyline-bearing leaves of one extrusion tree in traversal order.
void collect_leaves(const ExtrusionEntity &entity, std::vector<const ExtrusionEntity *> &leaves);

// Sum the length of the leaves whose flow matches the circular bridging section of their height.
double bridge_length(const ExtrusionEntity &tree);
// Sum the bridging-flow interface path length published across one object's auxiliary layers.
double published_bridge_length(const PrintObject &object);

// Resolve nearest attributes and inspect every published leaf; missing flow or a role in the wrong bucket fails the test while counts accumulate owned scalar values.
void check_paths(const ExtrusionEntity &entity, ExtrusionRole bucket, PathCounts &counts, bool first, const ExtrusionAttributes *attributes)
{
    if (const ExtrusionAttributes *local = entity.get_property<ExtrusionAttributes>()) attributes = local;
    if (entity.is_loop()) ++counts.loops;
    if (entity.has_polyline()) {
        REQUIRE(attributes != nullptr);
        CHECK(attributes->width > 0);
        CHECK(attributes->height > 0);
        CHECK(attributes->mm3_per_mm > 0);
        if (bucket == LayerRegionIsland::SUPPORT) {
            CHECK(attributes->role == ExtrusionRole::SupportMaterial);
            ++counts.base;
        } else {
            CHECK(attributes->role == ExtrusionRole::SupportMaterialInterface);
            ++counts.interface;
        }
        if (std::abs(attributes->mm3_per_mm - M_PI * attributes->height * attributes->height / 4.) < 1e-6) ++counts.bridges;
        if (first) counts.first_width = attributes->width;
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) check_paths(*child, bucket, counts, first, attributes);
}

// Recurse into collections and append every polyline-bearing leaf; the returned pointers stay owned by the inspected tree.
void collect_leaves(const ExtrusionEntity &entity, std::vector<const ExtrusionEntity *> &leaves)
{
    if (entity.has_polyline()) {
        leaves.push_back(&entity);
        return;
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) collect_leaves(*child, leaves);
}

// Compare each leaf flow against the area of a circle of its own height and add the matching path lengths in millimetres.
double bridge_length(const ExtrusionEntity &tree)
{
    std::vector<const ExtrusionEntity *> leaves;
    collect_leaves(tree, leaves);
    double total = 0;
    for (const ExtrusionEntity *leaf : leaves) {
        const ExtrusionAttributes *attributes = leaf->get_property<ExtrusionAttributes>();
        REQUIRE(attributes != nullptr);
        if (std::abs(attributes->mm3_per_mm - M_PI * attributes->height * attributes->height / 4.) >= 1e-6) continue;
        for (const ArcPolyline &polyline : leaf->as_polylines()) total += unscaled(polyline.length());
    }
    return total;
}

// Walk the owned auxiliary layers and sum the published interface trees without changing them.
double published_bridge_length(const PrintObject &object)
{
    double total = 0;
    for (const Layer &layer : object.auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                if (region.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE))
                    total += bridge_length(region.extrusion(LayerRegionIsland::SUPPORT_INTERFACE));
    return total;
}

// Deactivate the branch provider for the scope of a test; the destructor restores its activation.
struct ScopedBranchProviderOff {
    ScopedBranchProviderOff() { REQUIRE(Orchestrator::instance().set_plugin_active("support.toolpath.branch_sheath", false)); }
    ~ScopedBranchProviderOff() { Orchestrator::instance().set_plugin_active("support.toolpath.branch_sheath", true); }
};
}

// Check that the built-in support toolpaths (not the native generator) print the interface lines of a support layer at the angle
// interface_angle + interface_id * increment, for every style. Lines at 0 degrees run along X and lines at 90 degrees along Y: the
// base settings test below prints its contacts at the base angle 0 along X and fails on any vertical path. The first layer is left out:
// its flange uses its own angle.
TEST_CASE("Built-in support toolpaths turn interfaces by the angle increment per interface id", "[plugins][support-toolpath-legacy][support-toolpath]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    // Grid and snug with an increment of 90, then snug with no increment.
    const int case_index = GENERATE(0, 1, 2);
    const char *style = case_index == 0 ? "grid" : "snug";
    const bool no_increment = case_index == 2;
    DynamicPrintConfig config = SG::config(style, false, 3);
    config.set_deserialize_strict({{"support_material_interface_angle", "90"},
                                   {"support_material_interface_angle_increment", no_increment ? "0" : "90"}});
    Print print;
    Model model;
    Test::init_print({SG::slab(true)}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    SG::toolpaths(print);
    // Count of the layers seen for each parity of the interface id.
    size_t layers_of_parity[2] = {0, 0};
    for (const Layer &layer : print.object(0).auxiliary_layers()) {
        if (layer.scaled_print_z() - layer.scaled_height() <= 0) continue;
        const LayerSupportProperty *property = layer.get_property<LayerSupportProperty>();
        REQUIRE(property != nullptr);
        double length_along[2] = {0, 0};
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands()) {
                if (!region.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE)) continue;
                std::vector<const ExtrusionEntity *> leaves;
                collect_leaves(region.extrusion(LayerRegionIsland::SUPPORT_INTERFACE), leaves);
                for (const ExtrusionEntity *leaf : leaves)
                    for (const ArcPolyline &line : leaf->as_polylines())
                        for (size_t index = 1; index < line.size(); ++index) {
                            const Point delta = line.get_point(index) - line.get_point(index - 1);
                            length_along[0] += unscaled(double(std::abs(delta.x())));
                            length_along[1] += unscaled(double(std::abs(delta.y())));
                        }
            }
        const double total = length_along[0] + length_along[1];
        if (total == 0) continue;
        // With an increment of 90 an even id prints at 90 or 270 degrees (along Y) and an odd id at 180 or 360 degrees (along X);
        // with no increment every id prints at 90 degrees (along Y). The links between neighbouring lines are short:
        // the lines own nearly all the length.
        const size_t parity = property->interface_id & 1;
        const size_t expected_axis = no_increment || parity == 0 ? 1 : 0;
        ++layers_of_parity[parity];
        CHECK(length_along[expected_axis] > 0.9 * total);
    }
    if (no_increment) {
        REQUIRE(layers_of_parity[0] + layers_of_parity[1] > 0);
    } else {
        REQUIRE(layers_of_parity[0] > 0);
        REQUIRE(layers_of_parity[1] > 0);
    }
}

// Check registration, activation and dispatch ordering of the production algorithms: sparse, branch, bottom, top.
TEST_CASE("Native support toolpath providers are active in role order", "[plugins][support-toolpath-legacy]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::vector<Plugin *> providers = Orchestrator::instance().get_active_plugins_for_step(STEP_SUPPORT_TOOLPATH);
    REQUIRE(providers.size() == 4);
    const char *ids[] = {"support.toolpath.legacy_sparse", "support.toolpath.branch_sheath", "support.toolpath.legacy_bottom",
                         "support.toolpath.legacy_top"};
    const int32_t priorities[] = {0, 50, 100, 200};
    for (size_t index = 0; index < 4; ++index) {
        CHECK(providers[index]->get_id() == ids[index]);
        CHECK(providers[index]->get_priority() == priorities[index]);
    }
}

// Run real surfaces through every role and check first-layer and bridge flows on a roof above a platform; a role with no interface layer prints its contact as base, without a bridge flow.
TEST_CASE("Native support paths carry roles buckets and positive flow", "[plugins][support-toolpath-legacy]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool soluble = GENERATE(false, true);
    const int interfaces = GENERATE(0, 3);
    Print print;
    Model model;
    DynamicPrintConfig config = SG::config("grid", soluble, interfaces);
    Test::init_print({SG::slab(true)}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    SG::toolpaths(print);
    PathCounts counts;
    for (const Layer &layer : print.object(0).auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const ExtrusionRole bucket : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE})
                    if (region.has_extrusion(bucket)) check_paths(region.extrusion(bucket), bucket, counts, layer.scaled_print_z() - layer.scaled_height() <= 0);
    CHECK(counts.base > 0);
    if (interfaces > 0) CHECK(counts.interface > 0);
    else CHECK(counts.interface == 0);
    // A non-soluble bottom contact keeps its bridging flow, with or without interface layers.
    if (!soluble) CHECK(counts.bridges > 0);
}

// Compare enabled contact loops with the same roof without loops; path geometry must change without removing either support role.
TEST_CASE("Native top contacts generate removable loops", "[plugins][support-toolpath-legacy]")
{
    double lengths[2] = {};
    for (int enabled = 0; enabled < 2; ++enabled) {
        DynamicPrintConfig config = SG::config();
        config.set_deserialize_strict({{"support_material_interface_contact_loops", enabled ? "1" : "0"}});
        Print print;
        Model model;
        Test::init_print({SG::slab()}, print, model, config);
        SG::prefix(print);
        SG::family(print);
        SG::toolpaths(print);
        PathCounts counts;
        for (const Layer &layer : print.object(0).auxiliary_layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region : island.regions_islands())
                    if (region.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE)) {
                        const ExtrusionEntity &paths = region.extrusion(LayerRegionIsland::SUPPORT_INTERFACE);
                        check_paths(paths, LayerRegionIsland::SUPPORT_INTERFACE, counts, false);
                        for (const ArcPolyline &line : paths.as_polylines()) lengths[enabled] += line.length();
                    }
        REQUIRE(counts.interface > 0);
    }
    CHECK(std::abs(lengths[0] - lengths[1]) > scale_i(0.1));
}

// Verify the support flange uses the first-layer flow instead of the ordinary support flow.
TEST_CASE("Native sparse first layer uses the support flange flow", "[plugins][support-toolpath-legacy]")
{
    DynamicPrintConfig config = SG::config();
    config.set_deserialize_strict({{"first_layer_extrusion_width", "0.7"}, {"raft_first_layer_density", "80%"}});
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    SG::toolpaths(print);
    const Layer &first = print.object(0).auxiliary_layers().front();
    PathCounts counts;
    for (const LayerSliceIsland &island : first.islands())
        for (const LayerRegionIsland &region : island.regions_islands())
            if (region.has_extrusion(LayerRegionIsland::SUPPORT)) check_paths(region.extrusion(LayerRegionIsland::SUPPORT), LayerRegionIsland::SUPPORT, counts, true);
    CHECK(counts.base > 0);
    CHECK(counts.loops > 0);
    CHECK(counts.first_width == Approx(0.7));
}

// Run the real legacy sparse flange through pattern_service_fill; the pattern's error must remain the sole diagnostic.
TEST_CASE("Legacy support pattern failure preserves only the pattern error", "[plugins][support-toolpath-legacy]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    namespace Probe = Test::ProbePatternPlugin;
    Orchestrator &host = Orchestrator::instance();
    // Restore probe activation and failure controls even when a required assertion aborts the test.
    struct ResetProbe {
        Orchestrator &host;
        bool active;
        // Clear all shared probe controls before preparation; no probe run is active at test entry or exit.
        explicit ResetProbe(Orchestrator &orchestrator) : host(orchestrator), active(host.is_plugin_active(Probe::probe_pattern_id))
        {
            reset();
        }
        // Reset counters and failure, cancellation and rendezvous controls only while no probe run is active.
        void reset()
        {
            Probe::pattern_setup_count = 0;
            Probe::pattern_active_runs = 0;
            Probe::pattern_max_active_runs = 0;
            Probe::pattern_rendezvous = false;
            Probe::pattern_fail_setup = false;
            Probe::pattern_fail_run = 0;
            Probe::pattern_wrote_before_failure = false;
            Probe::pattern_cancel_print = nullptr;
            Probe::rendezvous_timed_out = false;
            host.reset_plugin_cancel();
            (void) host.consume_plugin_messages();
        }
        // Release the test's failure mode and stop request without retaining any print-owned data.
        ~ResetProbe()
        {
            reset();
            host.set_plugin_active(Probe::probe_pattern_id, active);
        }
    };
    ResetProbe reset{host};
    DynamicPrintConfig config = SG::config();
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    // The configured rectilinear flange resolves to the sole active pattern, with a fallback warning, not an error.
    const ScopedActivePlugins active;
    host.clear_active_plugins();
    for (const char *id : {"support.toolpath.legacy_sparse", "support.toolpath.branch_sheath",
                           "support.toolpath.legacy_bottom", "support.toolpath.legacy_top", Probe::probe_pattern_id})
        REQUIRE(host.set_plugin_active(id, true));
    Probe::pattern_fail_run = 1;
    (void) host.consume_plugin_messages();
    std::string failure;
    try {
        SG::toolpaths(print);
    } catch (const std::exception &error) {
        failure = error.what();
    }
    const std::vector<Orchestrator::PluginMessage> messages = host.consume_plugin_messages();
    CAPTURE(failure);
    CHECK(Probe::pattern_wrote_before_failure);
    CHECK(failure.find("STEP_SUPPORT_TOOLPATH stopped during ") != std::string::npos);
    CHECK(failure.find("of plugin 'support.toolpath.legacy_sparse' after plugin 'test.pattern.probe' reported an error.") != std::string::npos);
    size_t errors = 0;
    for (const Orchestrator::PluginMessage &message : messages) {
        INFO(message.plugin_id << ": " << message.message);
        CHECK(message.message.find("could not fill a perimeter-wrapped area") == std::string::npos);
        if (message.level != Orchestrator::PluginMessageLevel::Error) continue;
        ++errors;
        CHECK(message.plugin_id == Probe::probe_pattern_id);
        CHECK(message.message == "probe pattern run failure");
    }
    CHECK(errors == 1);
}

// Collect the leaves of a plugin-API extrusion tree in traversal order.
void collect_api_leaves(const slic3r_api::ExtrusionEntity &entity, std::vector<slic3r_api::ExtrusionEntity> &leaves)
{
    if (entity.has_polyline()) {
        leaves.push_back(entity);
        return;
    }
    for (uint32_t index = 0; index < entity.child_count(); ++index)
        collect_api_leaves(entity.child(index), leaves);
}

// Length in millimetres of the polyline of one plugin-API leaf.
double api_length(const slic3r_api::ExtrusionEntity &leaf)
{
    const std::vector<c_point> points = leaf.points();
    double length = 0;
    for (size_t index = 1; index < points.size(); ++index)
        length += std::hypot(double(points[index].x - points[index - 1].x), double(points[index].y - points[index - 1].y));
    return unscaled(length);
}

// Drive the modulation helper with synthetic slabs: a path crossing an overlapping footprint is
// split and its flow scaled by the remaining height, and an empty overlap list leaves it untouched.
TEST_CASE("Native support path modulation splits and thins overlapping paths", "[plugins][support-toolpath-legacy]")
{
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const coord_t layer_z = scale_to_layer_coord(1.);
    const std::vector<c_point> line{c_point{0, 0}, c_point{scale_i(10.), 0}};
    slic3r_api::EPropertyAttributes attributes;
    attributes.extrusion_role(int32_t(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL)).mm3_per_mm(0.1).width(0.4f).height(0.2f);
    // One slab 0.1 mm below the layer top, covering the middle of the path.
    const std::vector<c_point> square{c_point{scale_i(4.), -scale_i(1.)}, c_point{scale_i(6.), -scale_i(1.)},
                                      c_point{scale_i(6.), scale_i(1.)}, c_point{scale_i(4.), scale_i(1.)}};
    slic3r_api::StoredPolygonCollection footprint(handle);
    slic3r_api::StoredPolygon polygon(handle);
    polygon.insert_array(0, square.data(), uint32_t(square.size()));
    footprint.push_back(polygon);
    std::vector<ToolpathInternal::Overlap> overlapping;
    overlapping.push_back(ToolpathInternal::Overlap{scale_to_layer_coord(0.9), std::move(footprint)});
    slic3r_api::StoredExtrusionEntity split(handle);
    slic3r_api::StoredExtrusionEntity split_path(handle, line);
    split_path.set_flags(uint32_t(RAW_EXTRUSION_FLAG_REVERSIBLE));
    split_path.get_or_add(slic3r_api::EPropertyAttributes::key) = attributes;
    split.append_child_move(split_path.mutable_view());
    ToolpathInternal::modulate(handle, split.mutable_view(), layer_z, overlapping, SCALED_EPSILON);
    // The path is cut at the overlap boundary: one thinned middle path between two unchanged parts.
    std::vector<slic3r_api::ExtrusionEntity> leaves;
    collect_api_leaves(split.readonly(), leaves);
    REQUIRE(leaves.size() == 3);
    double thinned_length = 0, thinned_height = 0, thinned_mm3 = 0;
    std::vector<double> heights;
    for (const slic3r_api::ExtrusionEntity &leaf : leaves) {
        const slic3r_api::EPropertyAttributes *leaf_attributes = leaf.get(slic3r_api::EPropertyAttributes::key);
        REQUIRE(leaf_attributes != nullptr);
        heights.push_back(leaf_attributes->c_extrusion_property_attributes::height);
        CHECK(!leaf.reversible());
        if (leaf_attributes->c_extrusion_property_attributes::height < 0.2f) {
            thinned_length = api_length(leaf);
            thinned_height = leaf_attributes->c_extrusion_property_attributes::height;
            thinned_mm3 = leaf_attributes->mm3_per_mm();
        }
    }
    CHECK(thinned_height == Approx(0.1));
    CHECK(thinned_mm3 == Approx(0.1 * 0.1 / 0.2));
    CHECK(thinned_length == Approx(2.4)); // The 2 mm footprint plus half the flow width on both sides.
    // The fragments form one group in the source order. Like the native multipath, neither the group nor its
    // fragments may be reversed or sorted, so the printing direction is kept; the first child insertion would make
    // the group sortable, which is why the flags are cleared after the children.
    const slic3r_api::ExtrusionEntity modulated = split.readonly().child(0);
    REQUIRE(modulated.child_count() == 3);
    CHECK(!modulated.reversible());
    CHECK(!modulated.sortable());
    CHECK(modulated.continuous());
    REQUIRE(heights.size() == 3);
    CHECK(heights[0] == Approx(0.2));
    CHECK(heights[1] == Approx(0.1));
    CHECK(heights[2] == Approx(0.2));
    // Each fragment starts exactly where the previous one ends; the chain starts at the source start and ends at its end.
    for (size_t index = 0; index + 1 < leaves.size(); ++index) {
        const std::vector<c_point> ending = leaves[index].points();
        const std::vector<c_point> starting = leaves[index + 1].points();
        CHECK(ending.back().x == starting.front().x);
        CHECK(ending.back().y == starting.front().y);
    }
    const std::vector<c_point> first_points = leaves.front().points();
    const std::vector<c_point> last_points = leaves.back().points();
    CHECK(first_points.front().x == line.front().x);
    CHECK(first_points.front().y == line.front().y);
    CHECK(unscaled(std::hypot(double(last_points.back().x - line.back().x), double(last_points.back().y - line.back().y))) < 1e-3);
    // Without overlapping slabs the helper must leave the path untouched.
    slic3r_api::StoredExtrusionEntity untouched(handle);
    slic3r_api::StoredExtrusionEntity untouched_path(handle, line);
    untouched_path.set_flags(uint32_t(RAW_EXTRUSION_FLAG_REVERSIBLE));
    untouched_path.get_or_add(slic3r_api::EPropertyAttributes::key) = attributes;
    untouched.append_child_move(untouched_path.mutable_view());
    ToolpathInternal::modulate(handle, untouched.mutable_view(), layer_z, {}, SCALED_EPSILON);
    leaves.clear();
    collect_api_leaves(untouched.readonly(), leaves);
    REQUIRE(leaves.size() == 1);
    CHECK(leaves.front().reversible());
    const slic3r_api::EPropertyAttributes *leaf_attributes = leaves.front().get(slic3r_api::EPropertyAttributes::key);
    REQUIRE(leaf_attributes != nullptr);
    CHECK(leaf_attributes->c_extrusion_property_attributes::height == 0.2f);
    CHECK(leaf_attributes->mm3_per_mm() == Approx(0.1));
}

// Two paths in one pass, one footprint touching only the first: only that path is rebuilt (split, thinned, non-reversible);
// the other keeps its object, flags, flow and place among its siblings.
TEST_CASE("Native support path modulation keeps the paths no footprint touches", "[plugins][support-toolpath-legacy]")
{
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const coord_t layer_z = scale_to_layer_coord(1.);
    const std::vector<c_point> cut_line{c_point{0, 0}, c_point{scale_i(10.), 0}};
    // At y = 5 mm, 3.8 mm outside the footprint (y in [-1, 1] mm) grown by half the 0.4 mm width.
    const std::vector<c_point> far_line{c_point{0, scale_i(5.)}, c_point{scale_i(10.), scale_i(5.)}};
    slic3r_api::EPropertyAttributes attributes;
    attributes.extrusion_role(int32_t(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL)).mm3_per_mm(0.1).width(0.4f).height(0.2f);
    const std::vector<c_point> square{c_point{scale_i(4.), -scale_i(1.)}, c_point{scale_i(6.), -scale_i(1.)},
                                      c_point{scale_i(6.), scale_i(1.)}, c_point{scale_i(4.), scale_i(1.)}};
    slic3r_api::StoredPolygonCollection footprint(handle);
    slic3r_api::StoredPolygon polygon(handle);
    polygon.insert_array(0, square.data(), uint32_t(square.size()));
    footprint.push_back(polygon);
    std::vector<ToolpathInternal::Overlap> overlapping;
    overlapping.push_back(ToolpathInternal::Overlap{scale_to_layer_coord(0.9), std::move(footprint)});
    slic3r_api::StoredExtrusionEntity pass(handle);
    for (const std::vector<c_point> &line : {cut_line, far_line}) {
        slic3r_api::StoredExtrusionEntity path(handle, line);
        path.set_flags(uint32_t(RAW_EXTRUSION_FLAG_REVERSIBLE));
        path.get_or_add(slic3r_api::EPropertyAttributes::key) = attributes;
        pass.append_child_move(path.mutable_view());
    }
    // What the second path is before the modulation: its object, flags and role must be the same afterwards.
    REQUIRE(pass.readonly().child_count() == 2);
    const extrusion_entity_handle *second_handle = pass.readonly().child(1).handle();
    const uint32_t second_flags = pass.readonly().child(1).flags();
    const slic3r_api::EPropertyAttributes *second_before = pass.readonly().child(1).get(slic3r_api::EPropertyAttributes::key);
    REQUIRE(second_before != nullptr);
    const auto second_role = second_before->c_extrusion_property_attributes::role;
    ToolpathInternal::modulate(handle, pass.mutable_view(), layer_z, overlapping, SCALED_EPSILON);
    REQUIRE(pass.readonly().child_count() == 2);
    // The first path is split into three parts, the middle one thinned, and neither it nor its parts can be reversed.
    const slic3r_api::ExtrusionEntity first = pass.readonly().child(0);
    REQUIRE(!first.has_polyline());
    REQUIRE(first.child_count() == 3);
    CHECK(!first.reversible());
    CHECK(!first.sortable());
    std::vector<slic3r_api::ExtrusionEntity> leaves;
    collect_api_leaves(first, leaves);
    REQUIRE(leaves.size() == 3);
    for (const slic3r_api::ExtrusionEntity &leaf : leaves)
        CHECK(!leaf.reversible());
    const slic3r_api::EPropertyAttributes *middle = leaves[1].get(slic3r_api::EPropertyAttributes::key);
    REQUIRE(middle != nullptr);
    CHECK(middle->c_extrusion_property_attributes::height == Approx(0.1));
    CHECK(middle->mm3_per_mm() == Approx(0.1 * 0.1 / 0.2));
    // The second path is the same leaf as before: one polyline, still reversible, same points, height and flow, still second.
    const slic3r_api::ExtrusionEntity second = pass.readonly().child(1);
    REQUIRE(second.has_polyline());
    CHECK(second.handle() == second_handle);
    CHECK(second.flags() == second_flags);
    CHECK(second.reversible());
    const std::vector<c_point> second_points = second.points();
    REQUIRE(second_points.size() == far_line.size());
    for (size_t index = 0; index < far_line.size(); ++index) {
        CHECK(second_points[index].x == far_line[index].x);
        CHECK(second_points[index].y == far_line[index].y);
    }
    const slic3r_api::EPropertyAttributes *second_attributes = second.get(slic3r_api::EPropertyAttributes::key);
    REQUIRE(second_attributes != nullptr);
    CHECK(second_attributes->c_extrusion_property_attributes::role == second_role);
    CHECK(second_attributes->c_extrusion_property_attributes::height == 0.2f);
    CHECK(second_attributes->mm3_per_mm() == Approx(0.1));
}

// With a Z step of 0.04 mm and layers of 0.04 mm, the contact match window (SCALED_EPSILON + 2 * the close-Z limit, about one step) holds the
// neighbouring object layers too: the match keeps the layer nearest to the expected Z, the lower one on a tie, and none when the window is empty.
TEST_CASE("Contact match keeps the nearest object layer", "[plugins][support-toolpath-legacy][support-close-z-step]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("grid", false, 0);
    config.set_deserialize_strict({{"z_step", "0.04"}, {"layer_height", "0.04"}, {"first_layer_height", "0.04"},
                                   {"min_layer_height", "0.04,0.04"}, {"max_layer_height", "0.04,0.04"}});
    Print print;
    Model model;
    Test::init_print({make_cube(10., 10., 1.)}, print, model, config);
    SG::prefix(print);
    const PrintObject &host_object = print.object(0);
    const slic3r_api::Object object = SG::object_view(host_object);
    const uint32_t count = object.layer_count();
    REQUIRE(count > 12);
    const uint32_t k = 10;
    const coord_t step = scale_to_layer_coord(0.04);
    REQUIRE(object.layer(k + 1).bottom_z() - object.layer(k).bottom_z() == step);
    REQUIRE(object.layer(k).print_z() - object.layer(k).bottom_z() == step);
    const coord_t tolerance = SCALED_EPSILON + 2 * slic3r_api::Support::Common::close_z_tolerance_limit(0.04);
    REQUIRE(tolerance > step);
    // Both neighbours lie in the window, yet the layer on the expected Z wins, whichever key is compared.
    CHECK(ToolpathInternal::matched_layer(object, object.layer(k).bottom_z(), tolerance, true) == k);
    CHECK(ToolpathInternal::matched_layer(object, object.layer(k).bottom_z() + 3000, tolerance, true) == k);
    CHECK(ToolpathInternal::matched_layer(object, object.layer(k).bottom_z() - 3000, tolerance, true) == k);
    CHECK(ToolpathInternal::matched_layer(object, object.layer(k).print_z(), tolerance, false) == k);
    CHECK(ToolpathInternal::matched_layer(object, object.layer(k).print_z() - 3000, tolerance, false) == k);
    // Exactly between two layers: the lower one.
    CHECK(ToolpathInternal::matched_layer(object, object.layer(k).bottom_z() + step / 2, tolerance, true) == k);
    CHECK(ToolpathInternal::matched_layer(object, object.layer(k).bottom_z() + step / 2 + 100, tolerance, true) == k + 1);
    // Nothing in the window.
    CHECK(ToolpathInternal::matched_layer(object, object.layer(count - 1).print_z() + 10 * step, tolerance, false) == count);
}

// Generates grid/tree supports on variable object heights and runs the registered toolpath providers. Print/model own
// inputs and published paths; scoped counter enablement is restored on failure. Counts are diagnostic, while every leaf must
// have positive geometry/flow, fit its slab height and scale nominal mm3_per_mm consistently with its height.
// variant: "soluble" (no Z gap), "plane" (non-soluble, contact distance type plane) or "filament" (non-soluble, filament distance).
// Non-soluble contacts keep a Z gap, so support layers of different heights can overlap in Z, which is what triggers the
// legacy modulation; soluble ones never overlap.
void check_variable_height_toolpaths(const char *style, const char *variant)
{
    namespace Counters = slic3r_api::Support::Common::Counters;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool soluble = strcmp(variant, "soluble") == 0;
    CAPTURE(style, variant);
    // Soluble contacts exclude bridging flow, leaving the ordinary role flow as the nominal reference for non-flange paths.
    DynamicPrintConfig config = SG::config(style, soluble, 3);
    config.set_deserialize_strict({{"support_material_synchronize_layers", "0"}});
    if (strcmp(variant, "filament") == 0)
        config.set_deserialize_strict({{"support_material_contact_distance_type", "filament"}});
    TriangleMesh mesh = make_cube(4., 8., 6.);
    TriangleMesh roof = make_cube(12., 8., 1.);
    roof.translate(-4., 0., 6.);
    mesh.merge(roof);
    const std::vector<coordf_t> profile{0., 0.1, 2., 0.1, 2.01, 0.3, 4., 0.3, 4.01, 0.15, 7., 0.15};
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, std::move(mesh), profile, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() > 2);
    bool variable = false;
    for (size_t id = 1; id < object.layer_count(); ++id)
        variable = variable || object.layer(id).scaled_height() != object.layer(0).scaled_height();
    REQUIRE(variable);
    SG::family(print);
    REQUIRE_FALSE(object.auxiliary_layers().empty());
    const slic3r_api::Support::Common::SupportParams params = SG::params(print, object);
    REQUIRE(params.soluble == soluble);
    const slic3r_api::Object object_view = SG::object_view(object);
    struct RestoreCounters {
        bool previous;
        // Restores process-owned counter enablement after synchronous toolpath generation, including assertion/exception exits.
        ~RestoreCounters() { Counters::set_enabled_for_tests(previous); }
    } restore{Counters::set_enabled_for_tests(true)};
    Counters::reset();
    SG::toolpaths(print);
    const unsigned long long modulation_layers = Counters::totals.modulation_layers.load(std::memory_order_relaxed);
    const unsigned long long height_changes = Counters::totals.height_changes.load(std::memory_order_relaxed);
    const unsigned long long overlap_pairs = Counters::totals.overlap_pairs.load(std::memory_order_relaxed);
    INFO("style=" << style << " variant=" << variant << " modulation_layers=" << modulation_layers << " height_changes=" << height_changes << " overlap_pairs=" << overlap_pairs);
    std::printf("support-toolpath-variable-height style=%s variant=%s modulation_layers=%llu height_changes=%llu overlap_pairs=%llu\n",
        style, variant, modulation_layers, height_changes, overlap_pairs);
    size_t leaves = 0, thinner = 0, layers_with_paths = 0;
    // Resolves inherited attributes in borrowed trees, checks each leaf against its original role/slab flow and accumulates
    // scalar counts without changing paths; nominal is a non-bridging flow and geometry/generation failures reach Catch.
    const auto inspect = [&](const auto &self, const ExtrusionEntity &entity, const ExtrusionAttributes *inherited,
                             const c_flow &nominal, double layer_height, bool bridging_flow) -> void {
        const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
        if (!attributes) attributes = inherited;
        if (entity.has_polyline()) {
            REQUIRE(attributes != nullptr);
            REQUIRE(std::isfinite(attributes->height));
            REQUIRE(attributes->height > 0.);
            CHECK(double(attributes->height) <= layer_height + 1e-6);
            CHECK(std::isfinite(attributes->width));
            CHECK(attributes->width > 0.);
            REQUIRE(std::isfinite(attributes->mm3_per_mm));
            REQUIRE(attributes->mm3_per_mm > 0.);
            const float nominal_height = float(unscaled(nominal.height));
            REQUIRE(nominal_height > 0.f);
            // A non-soluble bottom contact prints with the bridging flow, whose width and height are not those of the
            // ordinary interface role: the comparison to the role flow only holds without it.
            if (!bridging_flow) {
                CHECK(attributes->width == Approx(unscaled(nominal.width)).margin(1e-6));
                CHECK(attributes->mm3_per_mm == Approx(nominal.mm3_per_mm * (attributes->height / nominal_height)).epsilon(1e-5));
            }
            const std::vector<ArcPolyline> lines = entity.as_polylines();
            REQUIRE_FALSE(lines.empty());
            for (const ArcPolyline &line : lines) {
                CHECK(line.size() >= 2);
                CHECK(line.length() > 0);
            }
            ++leaves;
            thinner += double(attributes->height) < layer_height - 1e-6;
        }
        for (const ExtrusionEntityUPtr &child : entity.children())
            self(self, *child, attributes, nominal, layer_height, bridging_flow);
    };
    for (const Layer &layer : object.auxiliary_layers()) {
        const size_t before = leaves;
        const double height = layer.unscaled_height();
        // Only the interface bucket of a layer holding a bottom contact surface can carry the bridging flow (non-soluble only).
        bool bottom_contact = false;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces())
                    bottom_contact = bottom_contact || (surface.surface_type & stPosBottom) != 0;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const ExtrusionRole bucket : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE}) {
                    if (!region.has_extrusion(bucket)) continue;
                    const raw_extrusion_role role = bucket == LayerRegionIsland::SUPPORT ?
                        RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL : RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
                    const c_flow nominal = layer.scaled_bottom_z() <= 0 ? params.first_flow :
                        object_view.support_flow(role, params.layer_height, height);
                    inspect(inspect, region.extrusion(bucket), nullptr, nominal, height,
                            !soluble && bucket == LayerRegionIsland::SUPPORT_INTERFACE && bottom_contact);
                }
        layers_with_paths += leaves > before;
    }
    REQUIRE(leaves > 0);
    REQUIRE(layers_with_paths > 0);
    INFO("thinner_leaves=" << thinner << " leaves=" << leaves << " layers_with_paths=" << layers_with_paths);
    std::printf("support-toolpath-variable-height style=%s variant=%s thinner_leaves=%zu leaves=%zu layers_with_paths=%zu\n",
        style, variant, thinner, leaves, layers_with_paths);
}

// Soluble contacts: no Z gap, so no support layer overlaps another (the legacy modulation does not fire).
TEST_CASE("Variable layer height support toolpaths report legacy modulation", "[plugins][support-toolpath-legacy][support-toolpath-variable-height]")
{
    check_variable_height_toolpaths(GENERATE("grid", "tree"), "soluble");
}

// Non-soluble contacts with a Z gap (plane and filament distance types): support layers of different heights can overlap in Z.
TEST_CASE("Variable layer height support toolpaths with a Z gap report legacy modulation", "[plugins][support-toolpath-legacy][support-toolpath-variable-height]")
{
    const char *style = GENERATE("grid", "tree");
    const char *variant = GENERATE("plane", "filament");
    check_variable_height_toolpaths(style, variant);
}

// With no interface layers and separate support extruders the contact cannot merge into the base, so
// it stays on its role's surface and that role prints it with the base settings and its own filler.
TEST_CASE("Native contacts print with base settings without interface layers", "[plugins][support-toolpath-legacy]")
{
    DynamicPrintConfig config = SG::config("grid", false, 0, false);
    config.set_deserialize_strict({{"support_material_angle", "0"}, {"support_material_interface_angle", "90"}});
    Print print;
    Model model;
    Test::init_print({SG::slab(true)}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    SG::toolpaths(print);
    PathCounts counts;
    size_t contact_layers = 0, wrong_tool = 0, sparse_contact_layers = 0;
    bool vertical = false;
    for (const Layer &layer : print.object(0).auxiliary_layers()) {
        // Collect the dense surfaces of this layer; the sparse base must not share them.
        ExPolygons dense_area;
        bool sparse_surface = false;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces()) {
                    const SurfaceType type = surface.surface_type;
                    if (type == (stModSupport | stPosTop | stDensSolid) ||
                        type == (stModSupport | stPosBottom | stDensSolid) ||
                        type == (stModSupport | stPosBottom | stDensSolid | stModBridge))
                        dense_area.push_back(surface.expolygon);
                    else if (type == (stModSupport | stPosInternal | stDensSparse)) sparse_surface = true;
                }
        if (dense_area.empty()) continue;
        ++contact_layers;
        if (sparse_surface) ++sparse_contact_layers;
        // The role writes its own contact into the support bucket with the support tool.
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands()) {
                if (!region.has_extrusion(LayerRegionIsland::SUPPORT)) continue;
                const ExtrusionEntity &paths = region.extrusion(LayerRegionIsland::SUPPORT);
                check_paths(paths, LayerRegionIsland::SUPPORT, counts, false);
                if (region.extruder_id() != 0) ++wrong_tool;
                for (const ArcPolyline &polyline : paths.as_polylines())
                    vertical = vertical || std::abs(polyline.back().y() - polyline.front().y()) >
                        std::abs(polyline.back().x() - polyline.front().x());
                // The paths stay inside the surface that carries them.
                const Polylines lines = to_polylines(paths.as_polylines());
                const Polygons allowed = offset(union_ex(dense_area), scale_d(0.01));
                double kept = 0, total = 0;
                for (const Polyline &line : intersection_pl(lines, allowed)) kept += unscaled(line.length());
                for (const Polyline &line : lines) total += unscaled(line.length());
                CHECK(kept == Approx(total));
            }
    }
    CHECK(contact_layers > 0);
    CHECK(counts.base > 0);
    CHECK(counts.interface == 0);
    CHECK(wrong_tool == 0);
    CHECK(sparse_contact_layers == 0);
    CHECK_FALSE(vertical);
}

// The filler choice of a contact printed with the base settings is decided by surface_pattern and by
// the family's typing rule (see "Grid family types a mergeable contact as base"); the cases here
// cover the filler patterns and the base-settings path through real surfaces.
// FillSupportBase rejects full-infill and bridging parameters, so a bridging contact that is not
// printed as base falls back to rectilinear; a base-printed part uses the base pattern where the
// support regions could merge and the intermediate rectilinear filler otherwise.
TEST_CASE("Native support pattern keeps bridging contacts away from the sparse pattern", "[plugins][support-toolpath-legacy]")
{
    CHECK(ToolpathInternal::surface_pattern(false, true, true, false, "supportbase", "honeycomb") == "rectilinear");
    CHECK(ToolpathInternal::surface_pattern(false, true, false, false, "supportbase", "honeycomb") == "supportbase");
    CHECK(ToolpathInternal::surface_pattern(false, false, false, false, "concentric", "honeycomb") == "rectilinear");
    CHECK(ToolpathInternal::surface_pattern(true, true, false, true, "concentric", "honeycomb") == "honeycomb");
    CHECK(ToolpathInternal::surface_pattern(true, true, false, false, "concentric", "honeycomb") == "rectilinear");
}

// An automatic bottom pattern resolving to the sparse pattern must not reach the bridging bottom
// contact: the run completes and the contact keeps the paths of an explicit rectilinear pattern.
TEST_CASE("Native bridging bottom contacts avoid the sparse pattern", "[plugins][support-toolpath-legacy]")
{
    // Slice the platform slab with the automatic or the explicit rectilinear bottom pattern and
    // measure the bridging-flow interface paths the family publishes.
    const auto slice_and_measure = [](const char *bottom_pattern) {
        DynamicPrintConfig config = SG::config("grid", false, 3);
        config.set_deserialize_strict({{"support_material_top_interface_pattern", "auto"},
                                       {"support_material_bottom_interface_pattern", bottom_pattern},
                                       {"support_material_interface_spacing", "2"}});
        Print print;
        Model model;
        Test::init_print({SG::slab(true)}, print, model, config);
        SG::prefix(print);
        SG::family(print);
        SG::toolpaths(print);
        return published_bridge_length(print.object(0));
    };
    const double automatic = slice_and_measure("auto");
    const double rectilinear = slice_and_measure("rectilinear");
    CHECK(automatic > 0);
    CHECK(automatic == Approx(rectilinear));
}

// One island holds a plain sparse surface on x in [0, 4] mm and a branch surface on x in [6, 10] mm. The branch role
// owns the branch surface, so the legacy sparse role must fill only the plain one. The branch provider is kept off, so
// every published path comes from the legacy sparse role.
TEST_CASE("Legacy sparse toolpath leaves branch surfaces to the branch role", "[plugins][support-toolpath-legacy]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ScopedBranchProviderOff branch_off;
    DynamicPrintConfig config = SG::config("grid", false, 0, false);
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    SG::prefix(print);
    PrintObject &object = print.object(0);
    const ExPolygon plain(Polygon({Point(0, 0), Point(scale_i(4.), 0),
                                   Point(scale_i(4.), scale_i(4.)), Point(0, scale_i(4.))}));
    const ExPolygon branch(Polygon({Point(scale_i(6.), 0), Point(scale_i(10.), 0),
                                    Point(scale_i(10.), scale_i(4.)), Point(scale_i(6.), scale_i(4.))}));
    const ExPolygons subject {ExPolygon(Polygon({Point(0, 0), Point(scale_i(10.), 0),
                                                 Point(scale_i(10.), scale_i(4.)), Point(0, scale_i(4.))}))};
    PluginStorage storage;
    const slic3r_api::AuxiliaryLayerBuildResult built = slic3r_api::build_auxiliary_layer_regions_from_subject(
        reinterpret_cast<storage_handle *>(&storage), slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::Object(reinterpret_cast<const object_handle *>(&object)),
        slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&subject)),
        scale_i(0.2), scale_i(0.2), scale_i(0.1));
    REQUIRE(built.created);
    layer_handle *layer = object_get_auxiliary_layer(reinterpret_cast<const object_handle *>(&object), 0);
    REQUIRE(plugin_property_get_or_add_data_mutable(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()), layer_get_properties_mutable(layer),
        PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) != nullptr);
    layer_region_island_handle *region = layer_island_get_or_create_region_island(
        layer_get_island_mutable(layer, 0), nullptr, 0, 0);
    REQUIRE(region != nullptr);
    slic3r_api::StoredSurfaceCollection surfaces(reinterpret_cast<storage_handle *>(&storage));
    surfaces.append(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&plain)),
                    slic3r_api::srf_type().internal().sparse().support().raw());
    surfaces.append(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&branch)),
                    slic3r_api::srf_type().internal().sparse().support().branch().raw());
    REQUIRE(slic3r_api::LayerRegionIsland(region).set_fill_surfaces(&surfaces));

    SG::toolpaths(print);

    size_t paths = 0;
    for (const Layer &support_layer : object.auxiliary_layers())
        for (const LayerSliceIsland &island : support_layer.islands())
            for (const LayerRegionIsland &region_island : island.regions_islands()) {
                if (!region_island.has_extrusion(LayerRegionIsland::SUPPORT)) continue;
                for (const ArcPolyline &polyline : region_island.extrusion(LayerRegionIsland::SUPPORT).as_polylines()) {
                    ++paths;
                    CHECK(polyline.front().x() <= scale_i(5.));
                    CHECK(polyline.back().x() <= scale_i(5.));
                }
            }
    CHECK(paths > 0);
}
