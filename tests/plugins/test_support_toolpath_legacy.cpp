#include "support_grid_test_helpers.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Plugins/Support/Toolpath/LegacySupportPaths.hpp"

/* Small roof-and-platform objects exercise the three built-in support roles.
   The slicing prefix leaves support editable, then the normal dispatcher fills
   the typed surfaces. Leaf checks resolve inherited flow attributes, so both
   native paths and generic API wrappers are tested in their published form.
   Two cases drive the internal helpers directly: synthetic slabs for height
   modulation and plain arguments for the filler pattern choice. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
namespace ToolpathInternal = slic3r_api::Support::Toolpath::Internal;
namespace {
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
}

// Check registration, activation and dispatch ordering of the production algorithms.
TEST_CASE("Native support toolpath providers are active in role order", "[plugins][support-toolpath-legacy]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::vector<Plugin *> providers = Orchestrator::instance().get_active_plugins_for_step(STEP_SUPPORT_TOOLPATH);
    REQUIRE(providers.size() == 3);
    const char *ids[] = {"support.toolpath.legacy_sparse", "support.toolpath.legacy_bottom", "support.toolpath.legacy_top"};
    for (size_t index = 0; index < 3; ++index) {
        CHECK(providers[index]->get_id() == ids[index]);
        CHECK(providers[index]->get_priority() == int(index * 100));
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

// Drive the modulation helper with synthetic slabs: a path crossing an overlapping footprint is
// split and its flow scaled by the remaining height, and an empty overlap list leaves it untouched.
TEST_CASE("Native support path modulation splits and thins overlapping paths", "[plugins][support-toolpath-legacy]")
{
    const Flow flow = Flow::new_from_config_width(frSupportMaterial, ConfigOptionFloatOrPercent(0.4, false),
        ConfigOptionFloatOrPercent(0.4, false), 0.4f, 0.2f, 0.f);
    const coord_t layer_z = scale_to_layer_coord(1.);
    Polyline line;
    line.append(Point(0, 0));
    line.append(Point(scale_i(10.), 0));
    // One slab 0.1 mm below the layer top, covering the middle of the path.
    Polygon square;
    square.points = {Point(scale_i(4.), -scale_i(1.)), Point(scale_i(6.), -scale_i(1.)),
                     Point(scale_i(6.), scale_i(1.)), Point(scale_i(4.), scale_i(1.))};
    const std::vector<ToolpathInternal::Overlap> overlapping{{scale_to_layer_coord(0.9), Polygons{square}}};
    ExtrusionEntityCollection split;
    ExtrusionPath split_path(ExtrusionAttributes(ExtrusionRole::SupportMaterial, flow), nullptr);
    split_path.set_polyline(line);
    split.append(std::move(split_path));
    ToolpathInternal::modulate(split, layer_z, overlapping, SCALED_EPSILON);
    // A split source path keeps its fragments in one nested collection, so the leaves are gathered.
    std::vector<const ExtrusionEntity *> leaves;
    collect_leaves(split, leaves);
    // The path is cut at the overlap boundary: one thinned middle path between two unchanged parts.
    REQUIRE(leaves.size() == 3);
    double thinned_length = 0, thinned_height = 0, thinned_mm3 = 0;
    for (const ExtrusionEntity *leaf : leaves) {
        const ExtrusionAttributes *attributes = leaf->get_property<ExtrusionAttributes>();
        REQUIRE(attributes != nullptr);
        double length = 0;
        for (const ArcPolyline &polyline : leaf->as_polylines()) length += unscaled(polyline.length());
        if (attributes->height < flow.height()) {
            thinned_length = length;
            thinned_height = attributes->height;
            thinned_mm3 = attributes->mm3_per_mm;
        }
    }
    CHECK(thinned_height == Approx(0.1));
    CHECK(thinned_mm3 == Approx(flow.mm3_per_mm() * 0.1 / 0.2));
    CHECK(thinned_length == Approx(2.4)); // The 2 mm footprint plus half the flow width on both sides.
    // Without overlapping slabs the helper must leave the path untouched.
    ExtrusionEntityCollection untouched;
    ExtrusionPath untouched_path(ExtrusionAttributes(ExtrusionRole::SupportMaterial, flow), nullptr);
    untouched_path.set_polyline(std::move(line));
    untouched.append(std::move(untouched_path));
    ToolpathInternal::modulate(untouched, layer_z, {}, SCALED_EPSILON);
    leaves.clear();
    collect_leaves(untouched, leaves);
    REQUIRE(leaves.size() == 1);
    const ExtrusionAttributes *attributes = leaves.front()->get_property<ExtrusionAttributes>();
    REQUIRE(attributes != nullptr);
    CHECK(attributes->height == flow.height());
    CHECK(attributes->mm3_per_mm == Approx(flow.mm3_per_mm()));
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
    CHECK(ToolpathInternal::surface_pattern(false, true, true, false, ipSupportBase, ipHoneycomb) == ipRectilinear);
    CHECK(ToolpathInternal::surface_pattern(false, true, false, false, ipSupportBase, ipHoneycomb) == ipSupportBase);
    CHECK(ToolpathInternal::surface_pattern(false, false, false, false, ipConcentric, ipHoneycomb) == ipRectilinear);
    CHECK(ToolpathInternal::surface_pattern(true, true, false, true, ipConcentric, ipHoneycomb) == ipHoneycomb);
    CHECK(ToolpathInternal::surface_pattern(true, true, false, false, ipConcentric, ipHoneycomb) == ipRectilinear);
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
