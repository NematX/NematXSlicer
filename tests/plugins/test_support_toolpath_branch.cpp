#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "support_grid_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "libslic3r/Api/host/GCodeReaderService.hpp"
#include "libslic3r/Plugins/Support/Common/SupportPaths.hpp"
#include "libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"
#include "libslic3r/Plugins/Support/Toolpath/BranchIslandClass.hpp"

/* Tree branch sheaths, run through the support toolpath step on synthetic auxiliary layers. Each layer is built from a
   subject area and holds typed surfaces; the step then runs the active providers, and the helpers read the published
   region islands back. The first-layer case compares the branch flange with the legacy base flange of the same surface,
   and the selector case runs both algorithms over the same sparse surfaces, including an inactive-provider fallback.
   The thin islands print their centreline instead
    of a sheath or a stationary deposit when elongated (BranchIslandClass.hpp); their cases check order, flow, volume,
    and the islands that fall back to a deposit. A full tree print checks deposit G-code and the reader's support volume. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
namespace {
// Activation ids of the two producers that compete for the support surfaces.
const char *const k_branch_id = "support.toolpath.branch_sheath";
const char *const k_sparse_id = "support.toolpath.legacy_sparse";

// Return owned grid settings with an explicit sheath sparse selector; no print state changes and configuration failures propagate.
DynamicPrintConfig branch_config();

// A support surface with its surface type, as published on an auxiliary layer.
struct TypedArea {
    ExPolygon area;
    raw_surface_type type;
};

// Sparse support surface of the first region island.
raw_surface_type base_type() { return slic3r_api::srf_type().internal().sparse().support().raw(); }

DynamicPrintConfig branch_config()
{
    DynamicPrintConfig config = SG::config();
    config.set_deserialize_strict("exclusive_group_1350_support_toolpath_sparse_plugin", k_branch_id);
    return config;
}

// Deactivate a provider for the scope of a test; the destructor activates it again.
class ScopedProviderOff {
public:
    explicit ScopedProviderOff(const char *id) : m_id(id) { REQUIRE(Orchestrator::instance().set_plugin_active(m_id, false)); }
    ~ScopedProviderOff() { Orchestrator::instance().set_plugin_active(m_id, true); }

private:
    const char *m_id;
};

// A square with corners (x0, y0) and (x1, y1), in millimeters and counter-clockwise.
ExPolygon square(double x0, double y0, double x1, double y1)
{
    return ExPolygon(Polygon({Point(scale_i(x0), scale_i(y0)), Point(scale_i(x1), scale_i(y0)),
                              Point(scale_i(x1), scale_i(y1)), Point(scale_i(x0), scale_i(y1))}));
}

// A circle of the given radius around (x, y), in millimeters and counter-clockwise.
ExPolygon circle(double radius, double x, double y)
{
    Polygon polygon;
    for (size_t index = 0; index < 64; ++index) {
        const double angle = 2. * PI * double(index) / 64.;
        polygon.points.push_back(Point::new_scale(x + radius * std::cos(angle), y + radius * std::sin(angle)));
    }
    return ExPolygon(polygon);
}

// Slice the test cube with the given settings, leaving support editable for the synthetic layers.
void slice(Print &print, DynamicPrintConfig config)
{
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    SG::prefix(print);
}

// Publish one support layer at print_z that holds the typed areas inside one island of subject.
void publish_layer(PrintObject &object, PluginStorage &storage, coord_t print_z, const ExPolygon &subject,
                   const std::vector<TypedArea> &areas)
{
    const Print &print = *object.print();
    const ExPolygons subjects {subject};
    const slic3r_api::AuxiliaryLayerBuildResult built = slic3r_api::build_auxiliary_layer_regions_from_subject(
        reinterpret_cast<storage_handle *>(&storage), slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::Object(reinterpret_cast<const object_handle *>(&object)),
        slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&subjects)),
        scale_i(0.2), print_z, print_z - scale_i(0.1));
    REQUIRE(built.created);
    layer_handle *layer = object_get_auxiliary_layer(reinterpret_cast<const object_handle *>(&object), 0);
    REQUIRE(plugin_property_get_or_add_data_mutable(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
                                                    layer_get_properties_mutable(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) != nullptr);
    layer_region_island_handle *region = layer_island_get_or_create_region_island(layer_get_island_mutable(layer, 0), nullptr, 0, 0);
    REQUIRE(region != nullptr);
    slic3r_api::StoredSurfaceCollection surfaces(reinterpret_cast<storage_handle *>(&storage));
    for (const TypedArea &typed : areas)
        surfaces.append(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&typed.area)), typed.type);
    REQUIRE(slic3r_api::LayerRegionIsland(region).set_fill_surfaces(&surfaces));
}

// Publish the typed areas on a layer of a sliced print, then run the support toolpath step.
void run_layer(Print &print, PluginStorage &storage, coord_t print_z, const ExPolygon &subject, const std::vector<TypedArea> &areas)
{
    publish_layer(print.object(0), storage, print_z, subject, areas);
    SG::toolpaths(print);
}

// The first region island of the first auxiliary layer that holds support paths, or nullptr when none does.
const LayerRegionIsland *published_region(PrintObject &object)
{
    for (const LayerSliceIsland &island : object.auxiliary_layers().front().islands())
        for (const LayerRegionIsland &region : island.regions_islands())
            if (region.has_extrusion(LayerRegionIsland::SUPPORT))
                return &region;
    return nullptr;
}

// Visit every polyline-bearing leaf of an extrusion tree, in traversal order.
void for_each_leaf(const ExtrusionEntity &entity, const std::function<void(const ExtrusionEntity &)> &visit)
{
    if (entity.has_polyline())
        visit(entity);
    for (const ExtrusionEntityUPtr &child : entity.children())
        for_each_leaf(*child, visit);
}

// Count the closed loops of an extrusion tree.
size_t count_loops(const ExtrusionEntity &entity)
{
    size_t loops = entity.is_loop() ? 1 : 0;
    for (const ExtrusionEntityUPtr &child : entity.children())
        loops += count_loops(*child);
    return loops;
}

// Total length in millimeters of the polylines of an extrusion tree.
double total_length(const ExtrusionEntity &entity)
{
    double total = 0.;
    for (const ArcPolyline &line : entity.as_polylines())
        total += unscaled(line.length());
    return total;
}

// Distance in millimeters from a point to the center (x, y).
double distance_to(const Point &point, double x, double y)
{
    return std::hypot(unscaled(point.x()) - x, unscaled(point.y()) - y);
}

// Width of the bounding box of a polyline in millimeters.
double x_extent(const ArcPolyline &line)
{
    coord_t low = std::numeric_limits<coord_t>::max(), high = std::numeric_limits<coord_t>::min();
    for (size_t index = 0; index < line.size(); ++index) {
        low = std::min(low, line.get_point(index).x());
        high = std::max(high, line.get_point(index).x());
    }
    return unscaled(high - low);
}

// Polylines of the auxiliary layers whose points all lie left of x = 5 mm (plain), or all right of it (branch).
struct Sides {
    size_t plain = 0, branch = 0, mixed = 0;
};
// Count the published polylines of the first auxiliary layer on each side of x = 5 mm; a polyline on neither side is mixed.
Sides count_sides(PrintObject &object)
{
    Sides sides;
    for (const LayerSliceIsland &island : object.auxiliary_layers().front().islands())
        for (const LayerRegionIsland &region : island.regions_islands()) {
            if (!region.has_extrusion(LayerRegionIsland::SUPPORT))
                continue;
            for (const ArcPolyline &line : region.extrusion(LayerRegionIsland::SUPPORT).as_polylines()) {
                bool all_left = true, all_right = true;
                for (size_t index = 0; index < line.size(); ++index) {
                    all_left = all_left && line.get_point(index).x() <= scale_i(5.);
                    all_right = all_right && line.get_point(index).x() >= scale_i(5.);
                }
                if (all_left)
                    ++sides.plain;
                else if (all_right)
                    ++sides.branch;
                else
                    ++sides.mixed;
            }
        }
    return sides;
}

// A polygon from its vertices in millimeters, counter-clockwise.
ExPolygon polygon_of(const std::vector<std::pair<double, double>> &vertices)
{
    Polygon polygon;
    for (const std::pair<double, double> &vertex : vertices)
        polygon.points.push_back(Point::new_scale(vertex.first, vertex.second));
    return ExPolygon(polygon);
}

// The shape translated by (x, y) millimeters.
ExPolygon moved(ExPolygon shape, double x, double y)
{
    shape.translate(scale_d(x), scale_d(y));
    return shape;
}

// True when every point of every polyline of the entity lies in the box (x0, y0)-(x1, y1) in millimeters, within a micrometer.
bool lies_in(const ExtrusionEntity &entity, double x0, double y0, double x1, double y1)
{
    for (const ArcPolyline &line : entity.as_polylines())
        for (size_t index = 0; index < line.size(); ++index) {
            const double x = unscaled(line.get_point(index).x()), y = unscaled(line.get_point(index).y());
            if (x < x0 - 1e-3 || x > x1 + 1e-3 || y < y0 - 1e-3 || y > y1 + 1e-3)
                return false;
        }
    return true;
}

// The extrusion width of the support flow that the branch layers use, in millimeters.
double branch_width(const Print &print) { return unscaled(SG::params(print, print.object(0)).base_flow.width); }

// The children of the support bucket of the first published region island, or none when the layer printed nothing.
std::vector<const ExtrusionEntity *> bucket_children(PrintObject &object)
{
    std::vector<const ExtrusionEntity *> children;
    if (const LayerRegionIsland *region = published_region(object))
        for (const ExtrusionEntityUPtr &child : region->extrusion(LayerRegionIsland::SUPPORT).children())
            children.push_back(child.get());
    return children;
}

// Classify a host polygon with the extrusion width of the print, as the step does for the fused island.
slic3r_api::Support::Toolpath::BranchIslandMeasure classify(const Print &print, const ExPolygon &shape)
{
    PluginStorage storage;
    return slic3r_api::Support::Toolpath::classify_island(
        reinterpret_cast<storage_handle *>(&storage), slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&shape)),
        scale_i(branch_width(print)));
}
} // namespace

// A small round branch is one sheath path: it is not reversible, starts on an anchor inside the circle, turns
// counter-clockwise like the loop it leads into, and its length is the sheath circumference plus the anchor.
TEST_CASE("Branch sheath of a small round branch is one anchored non-reversible path", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice(print, branch_config());
    const double radius = 1.2;
    PluginStorage storage;
    run_layer(print, storage, scale_i(0.6), square(0., 0., 10., 10.), {TypedArea{circle(radius, 5., 5.), base_type()}});

    const LayerRegionIsland *region = published_region(print.object(0));
    REQUIRE(region != nullptr);
    const ExtrusionEntity &bucket = region->extrusion(LayerRegionIsland::SUPPORT);
    REQUIRE(bucket.children().size() == 1);
    const ExtrusionEntity &sheath = *bucket.children().front();
    CHECK(sheath.has_polyline());
    CHECK_FALSE(sheath.can_reverse());
    REQUIRE(sheath.as_polylines().size() == 1);
    const ArcPolyline line = sheath.as_polylines().front();

    const c_flow flow = SG::params(print, print.object(0)).base_flow;
    const double width = unscaled(flow.width), spacing = unscaled(flow.spacing);
    CHECK(distance_to(line.front(), 5., 5.) < radius - width);
    CHECK(distance_to(line.front(), 5., 5.) > 0.);

    // The sheath runs counter-clockwise around the branch, the anchor included.
    for (size_t index = 1; index < line.size(); ++index) {
        const Point &previous = line.get_point(index - 1), &current = line.get_point(index);
        const double ax = unscaled(previous.x()) - 5., ay = unscaled(previous.y()) - 5.;
        const double bx = unscaled(current.x()) - 5., by = unscaled(current.y()) - 5.;
        CHECK(ax * by - ay * bx >= 0.);
    }

    const double sheath_circle = 2. * PI * (radius - 0.5 * width);
    const double length = unscaled(line.length());
    CHECK(length > sheath_circle + 4. * spacing);
    CHECK(length < sheath_circle + 6.5 * spacing);
}

// The support flow and its role: the sparse support paths carry the base support material flow at the layer height, in the
// support bucket of the base-tool region island.
TEST_CASE("Branch sheath paths carry the support material flow and the base tool", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice(print, branch_config());
    PluginStorage storage;
    run_layer(print, storage, scale_i(0.6), square(0., 0., 10., 10.), {TypedArea{circle(1.2, 5., 5.), base_type()}});

    const LayerRegionIsland *region = published_region(print.object(0));
    REQUIRE(region != nullptr);
    CHECK(region->extruder_id() == 0);
    CHECK_FALSE(region->has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE));
    const c_flow flow = SG::params(print, print.object(0)).base_flow;
    size_t leaves = 0;
    for_each_leaf(region->extrusion(LayerRegionIsland::SUPPORT), [&](const ExtrusionEntity &leaf) {
        const ExtrusionAttributes *attributes = leaf.get_property<ExtrusionAttributes>();
        REQUIRE(attributes != nullptr);
        ++leaves;
        CHECK(attributes->role == ExtrusionRole::SupportMaterial);
        CHECK(double(attributes->width) == Approx(unscaled(flow.width)).epsilon(1e-6));
        CHECK(double(attributes->height) == Approx(unscaled(flow.height)).epsilon(1e-6));
        CHECK(attributes->mm3_per_mm == Approx(flow.mm3_per_mm).epsilon(1e-6));
    });
    CHECK(leaves == 1);
}

// A branch surface larger than the double-wall circle gets its outer perimeter and an inner sheath, grouped so that
// neither can be sorted or reversed. The inner sheath prints first, as the legacy group does.
TEST_CASE("Large branch surfaces print the inner sheath before the outer perimeter", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice(print, branch_config());
    const double radius = 2.5;
    PluginStorage storage;
    run_layer(print, storage, scale_i(0.6), square(0., 0., 10., 10.), {TypedArea{circle(radius, 5., 5.), base_type()}});

    const LayerRegionIsland *region = published_region(print.object(0));
    REQUIRE(region != nullptr);
    const ExtrusionEntity &bucket = region->extrusion(LayerRegionIsland::SUPPORT);
    REQUIRE(bucket.children().size() == 1);
    const ExtrusionEntity &group = *bucket.children().front();
    CHECK_FALSE(group.can_reverse());
    CHECK_FALSE(group.can_sort());
    REQUIRE(group.children().size() == 2);
    const double width = unscaled(SG::params(print, print.object(0)).base_flow.width);

    const ExtrusionEntity &inner = *group.children()[0], &outer = *group.children()[1];
    REQUIRE(outer.as_polylines().size() == 1);
    const ArcPolyline perimeter = outer.as_polylines().front();
    for (size_t index = 0; index < perimeter.size(); ++index)
        CHECK(distance_to(perimeter.get_point(index), 5., 5.) == Approx(radius - 0.5 * width).margin(0.02));
    REQUIRE(inner.as_polylines().size() == 1);
    const ArcPolyline sheath = inner.as_polylines().front();
    CHECK(distance_to(sheath.front(), 5., 5.) < radius - width);
    CHECK(distance_to(sheath.back(), 5., 5.) == Approx(radius - 1.5 * width).margin(0.02));
}

// A branch surface with a hole gets one sheath per contour and per hole when the double wall is off.
TEST_CASE("Branch surface with a hole gets one sheath per contour and hole", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = branch_config();
    config.set_deserialize_strict({{"support_tree_branch_diameter_double_wall", "0"}});
    Print print;
    slice(print, config);
    ExPolygon holed = square(2., 2., 8., 8.);
    Polygon hole({Point(scale_i(4.), scale_i(4.)), Point(scale_i(4.), scale_i(6.)), Point(scale_i(6.), scale_i(6.)),
                  Point(scale_i(6.), scale_i(4.))});
    hole.make_clockwise();
    holed.holes.push_back(hole);
    PluginStorage storage;
    run_layer(print, storage, scale_i(0.6), square(0., 0., 10., 10.), {TypedArea{holed, base_type()}});

    const LayerRegionIsland *region = published_region(print.object(0));
    REQUIRE(region != nullptr);
    const ExtrusionEntity &bucket = region->extrusion(LayerRegionIsland::SUPPORT);
    REQUIRE(bucket.children().size() == 2);
    std::vector<double> extents;
    for (const ExtrusionEntityUPtr &child : bucket.children()) {
        CHECK(child->has_polyline());
        CHECK_FALSE(child->can_reverse());
        REQUIRE(child->as_polylines().size() == 1);
        extents.push_back(x_extent(child->as_polylines().front()));
    }
    // The contour sheath spans the outline, the hole sheath spans the hole alone.
    CHECK(*std::max_element(extents.begin(), extents.end()) > 4.5);
    CHECK(*std::min_element(extents.begin(), extents.end()) < 4.5);
}

// The outline runs along x = 0 and the 120-degree apex of the hole points at it. The legacy closing moves the outline in by
// half a width; the mitered candidate tip at the apex then lies 1.08 w from the closed outline but 1.15 w from the closed
// hole, so a nearest-contour rule tags it as outline. The loop starts at the hole's first vertex, 1 w down the lower side,
// and the anchor walks up that side to the apex: a tip tagged as outline cuts the anchor short of six spacings.
TEST_CASE("Hole sheath anchor follows its own offset past a miter tip near the outline", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = branch_config();
    config.set_deserialize_strict({{"support_tree_branch_diameter_double_wall", "0"}});
    Print print;
    slice(print, config);
    const c_flow flow = SG::params(print, print.object(0)).base_flow;
    const double w = unscaled(flow.width), spacing = unscaled(flow.spacing);
    const double cos60 = std::cos(PI / 3.), sin60 = std::sin(PI / 3.);
    const double apex_x = 3.31 * w, apex_y = 7. * w;
    // The first vertex lies 1 w down the lower side; it is bent 0.005 w outward because Clipper drops collinear vertices.
    const Point first(scale_i(apex_x + cos60 * w - 0.005 * w * sin60), scale_i(apex_y - sin60 * w - 0.005 * w * cos60));
    Polygon hole({first, Point(scale_i(apex_x), scale_i(apex_y)), Point(scale_i(apex_x + 3. * w * cos60), scale_i(apex_y + 3. * w * sin60)),
                  Point(scale_i(apex_x + 3. * w * cos60), scale_i(apex_y - 3. * w * sin60))});
    hole.make_clockwise();
    ExPolygon surface = square(0., 0., 10. * w, 14. * w);
    surface.holes.push_back(hole);
    PluginStorage storage;
    run_layer(print, storage, scale_i(0.6), square(0., 0., 10. * w, 14. * w), {TypedArea{surface, base_type()}});

    const LayerRegionIsland *region = published_region(print.object(0));
    REQUIRE(region != nullptr);
    const ExtrusionEntity &bucket = region->extrusion(LayerRegionIsland::SUPPORT);
    REQUIRE(bucket.children().size() == 2);
    // The sheath loops follow the legacy closing of the surface: the hole loop is the closed hole of that closing.
    const ExPolygons closed = closing_ex(to_polygons(surface), double(SCALED_EPSILON), double(SCALED_EPSILON) + 0.5 * double(flow.width));
    REQUIRE(closed.size() == 1);
    REQUIRE(closed.front().holes.size() == 1);
    const double loop = unscaled(closed.front().holes.front().length()) - 0.15 * spacing;
    const ArcPolyline sheath = bucket.children()[1]->as_polylines().front();
    const double anchor = unscaled(sheath.length()) - loop;
    CAPTURE(anchor, spacing);
    CHECK(anchor == Approx(6. * spacing).margin(0.25 * spacing));
}

// On the first layer the branch surface prints the legacy base flange: the same loops and paths, with the same length,
// as a plain sparse surface of the same shape, and every path at the first-layer flow.
TEST_CASE("First-layer branch surface prints the legacy base flange", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExPolygon shape = square(2., 2., 8., 8.);
    double lengths[2] = {0., 0.};
    size_t loops[2] = {0, 0};
    for (const bool branch : {true, false}) {
        Print print;
        slice(print, branch ? branch_config() : SG::config());
        PluginStorage storage;
        run_layer(print, storage, scale_i(0.2), square(0., 0., 10., 10.),
                  {TypedArea{shape, base_type()}});
        const LayerRegionIsland *region = published_region(print.object(0));
        REQUIRE(region != nullptr);
        const ExtrusionEntity &bucket = region->extrusion(LayerRegionIsland::SUPPORT);
        const double first_width = unscaled(SG::params(print, print.object(0)).first_flow.width);
        for_each_leaf(bucket, [&](const ExtrusionEntity &leaf) {
            const ExtrusionAttributes *attributes = leaf.get_property<ExtrusionAttributes>();
            REQUIRE(attributes != nullptr);
            CHECK(double(attributes->width) == Approx(first_width).epsilon(1e-6));
        });
        lengths[branch ? 0 : 1] = total_length(bucket);
        loops[branch ? 0 : 1] = count_loops(bucket);
    }
    CHECK(loops[0] > 0);
    CHECK(lengths[0] > 0.);
    CHECK(loops[0] == loops[1]);
    CHECK(lengths[0] == Approx(lengths[1]).epsilon(1e-6));
}

// Both sparse areas use the selected algorithm, including fallback from an inactive explicit provider.
TEST_CASE("The sparse selector chooses which algorithm prints every sparse surface of an object", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto run = [](bool sheath, bool disable_sheath) {
        Print print;
        slice(print, sheath ? branch_config() : SG::config());
        std::unique_ptr<ScopedProviderOff> off;
        if (disable_sheath)
            off = std::make_unique<ScopedProviderOff>(k_branch_id);
        CHECK(Steps::selected_active_plugin_from_group(Orchestrator::instance(), STEP_SUPPORT_TOOLPATH,
            "support_toolpath_sparse", &print.object(0).config())->get_id() ==
            (sheath && !disable_sheath ? k_branch_id : k_sparse_id));
        PluginStorage storage;
        run_layer(print, storage, scale_i(0.6), square(0., 0., 10., 10.),
                  {TypedArea{square(0.5, 0.5, 4.5, 9.5), base_type()}, TypedArea{circle(1.5, 7.5, 5.), base_type()}});
        const Sides sides = count_sides(print.object(0));
        CHECK(sides.plain > 0);
        CHECK(sides.branch > 0);
        CHECK(sides.mixed == 0);
        double circle_length = 0.;
        const LayerRegionIsland *region = published_region(print.object(0));
        REQUIRE(region != nullptr);
        for (const ArcPolyline &line : region->extrusion(LayerRegionIsland::SUPPORT).as_polylines())
            if (line.front().x() > scale_i(5.))
                circle_length += unscaled(line.length());
        return circle_length;
    };
    const double sheath_length = run(true, false);
    const double legacy_length = run(false, false);
    CHECK(sheath_length != Approx(legacy_length).epsilon(1e-6));
    CHECK(run(true, true) == Approx(legacy_length).epsilon(1e-6));
}

// The sheath provider shares sparse execution priority but keeps a later fallback rank and declares its settings.
TEST_CASE("Branch sheath provider has its role group, priority and declared settings", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Plugin *provider = Orchestrator::instance().get_plugin(k_branch_id);
    REQUIRE(provider != nullptr);
    CHECK(provider->get_step() == STEP_SUPPORT_TOOLPATH);
    CHECK(provider->get_exclusive_group() == "support_toolpath_sparse");
    CHECK(provider->get_priority() == 0);
    CHECK(provider->get_exclusive_group_priority() == 10);
    const Plugin *legacy = Orchestrator::instance().get_plugin(k_sparse_id);
    REQUIRE(legacy != nullptr);
    CHECK(legacy->get_priority() == 0);
    CHECK(legacy->get_exclusive_group_priority() == 0);
    const auto declares = [provider](const std::string &key) {
        return std::any_of(provider->get_used_config_keys().begin(), provider->get_used_config_keys().end(),
                           [&key](const Plugin::UsedConfigKey &used) { return used.key == key; });
    };
    CHECK(declares("support_tree_branch_diameter_double_wall"));
    CHECK(declares("raft_first_layer_density"));
    CHECK(declares("support_material_angle"));
    CHECK(provider->get_used_config_keys().size() ==
          slic3r_api::Support::Common::support_geometry_keys().size + slic3r_api::Support::Common::support_path_keys().size + 1);
}


namespace {
// Shapes sized in extrusion widths w that the thin test leaves to the sheath, laid out in the cells of a grid. They stress the
// opening and the closing: islands just wider than the opening (1.5 w), a square with a sharp wedge cut from its top edge (a
// reflex vertex whose miter wedge is truncated by the miter limit), and a triangle with a sharp tip (a convex vertex).
std::vector<std::pair<std::string, ExPolygon>> sheath_shapes(double w)
{
    std::vector<std::pair<std::string, ExPolygon>> shapes;
    shapes.emplace_back("disc", circle(2.4 * w, 0., 0.));
    shapes.emplace_back("rectangle 2:1", square(-2.4 * w, -1.2 * w, 2.4 * w, 1.2 * w));
    shapes.emplace_back("disc just over the opening", circle(0.8 * w, 0., 0.));
    shapes.emplace_back("square just over the opening", square(-0.8 * w, -0.8 * w, 0.8 * w, 0.8 * w));
    for (const double degrees : {8., 15., 25., 40., 60.}) {
        const double half = 5. * w * std::tan(0.5 * degrees * PI / 180.);
        shapes.emplace_back("notch " + std::to_string(int(degrees)) + " degrees",
                            polygon_of({{-3.5 * w, -3.5 * w}, {3.5 * w, -3.5 * w}, {3.5 * w, 3.5 * w}, {half, 3.5 * w}, {0., -1.5 * w},
                                        {-half, 3.5 * w}, {-3.5 * w, 3.5 * w}}));
    }
    for (const double degrees : {20., 30.}) {
        const double half = 9.5 * w * std::tan(0.5 * degrees * PI / 180.);
        shapes.emplace_back("tip " + std::to_string(int(degrees)) + " degrees",
                            polygon_of({{-4.75 * w, 0.}, {4.75 * w, -half}, {4.75 * w, half}}));
    }
    return shapes;
}

// The index of the one child that lies in the box (x0, y0)-(x1, y1) in millimeters; the test fails when none or several do.
size_t child_in(const std::vector<const ExtrusionEntity *> &children, double x0, double y0, double x1, double y1)
{
    std::vector<size_t> found;
    for (size_t index = 0; index < children.size(); ++index)
        if (lies_in(*children[index], x0, y0, x1, y1))
            found.push_back(index);
    REQUIRE(found.size() == 1);
    return found.front();
}

// The nozzle diameter of SG::config() in millimeters.
constexpr double k_nozzle = 0.4;

// The width and the flow of every line that the centreline prints, the number of stationary deposits it falls back to when it has
// no line, and the layer height.
struct PrintedLines {
    std::vector<std::pair<double, double>> lines;
    size_t deposits = 0;
    double height = 0.;
};

// What the centreline prints for a 4 mm strip alone on a branch layer, whose extrusion width (its width plus the rounded ends of
// a line) is the given one in millimeters. xy_spacing is the support_material_xy_spacing value, or nullptr for the default. The
// strip is thin, elongated and narrower than the support width, so the closing leaves no sheath piece: any line is the centreline.
PrintedLines printed_lines(double extrusion_width, const char *xy_spacing)
{
    DynamicPrintConfig config = branch_config();
    if (xy_spacing != nullptr)
        config.set_deserialize_strict({{"support_material_xy_spacing", xy_spacing}});
    Print print;
    slice(print, config);
    const c_flow flow = SG::params(print, print.object(0)).base_flow;
    const double rounded_ends = unscaled(flow.height) * (1. - 0.25 * PI) * double(flow.spacing_ratio);
    const ExPolygon strip = square(3., 5., 7., 5. + extrusion_width - rounded_ends);
    CHECK(classify(print, strip).kind == slic3r_api::Support::Toolpath::BranchIslandClass::Centreline);
    CHECK(closing_ex(to_polygons(strip), double(SCALED_EPSILON), double(SCALED_EPSILON) + 0.5 * double(flow.width)).empty());
    PluginStorage storage;
    run_layer(print, storage, scale_i(0.6), square(0., 0., 10., 10.), {TypedArea{strip, base_type()}});
    PrintedLines result;
    result.height = unscaled(flow.height);
    for (const ExtrusionEntity *child : bucket_children(print.object(0)))
        for_each_leaf(*child, [&](const ExtrusionEntity &leaf) {
            if (slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(&leaf)).is_deposit()) {
                ++result.deposits;
                return;
            }
            const ExtrusionAttributes *attributes = leaf.get_property<ExtrusionAttributes>();
            REQUIRE(attributes != nullptr);
            result.lines.emplace_back(double(attributes->width), attributes->mm3_per_mm);
        });
    return result;
}
} // namespace

// An island whose thin part is under half of its area keeps its sheath: the closing of the layer leaves a piece in it. Each
// shape is alone in its cell, and a cell must hold a printed entity. Twice the nozzle as extrusion width runs the same shapes
// with a wider opening.
TEST_CASE("Every island left to the sheath keeps a sheath", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const char *extrusion_width : {"0", "0.8"}) {
        DYNAMIC_SECTION("support_material_extrusion_width=" << extrusion_width) {
            DynamicPrintConfig config = branch_config();
            config.set_deserialize_strict({{"support_material_extrusion_width", extrusion_width}});
            Print print;
            slice(print, config);
            const double w = branch_width(print), pitch = 10. * w;
            const std::vector<std::pair<std::string, ExPolygon>> shapes = sheath_shapes(w);
            const size_t columns = 4, rows = (shapes.size() + columns - 1) / columns;
            std::vector<TypedArea> areas;
            for (size_t index = 0; index < shapes.size(); ++index)
                areas.push_back(TypedArea{moved(shapes[index].second, (double(index % columns) + 0.5) * pitch, (double(index / columns) + 0.5) * pitch),
                                          base_type()});
            PluginStorage storage;
            run_layer(print, storage, scale_i(0.6), square(0., 0., double(columns) * pitch, double(rows) * pitch), areas);

            const std::vector<const ExtrusionEntity *> children = bucket_children(print.object(0));
            for (size_t index = 0; index < shapes.size(); ++index) {
                CAPTURE(shapes[index].first, w);
                CHECK(classify(print, areas[index].area).kind == slic3r_api::Support::Toolpath::BranchIslandClass::Sheath);
                const double x = double(index % columns) * pitch, y = double(index / columns) * pitch;
                const size_t printed = size_t(std::count_if(children.begin(), children.end(), [&](const ExtrusionEntity *child) {
                    return lies_in(*child, x, y, x + pitch, y + pitch);
                }));
                CHECK(printed >= 1);
            }
        }
    }
}

// Two discs joined by a neck narrower than the sheath offset: the thin part is a small share of the island, so the sheath
// prints, and the closing of the layer cuts the neck. Each disc gets its sheath and nothing crosses the neck.
TEST_CASE("A dumbbell with a narrow neck prints two sheaths and nothing in the neck", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice(print, branch_config());
    const double w = branch_width(print);
    const ExPolygons dumbbell = union_ex(ExPolygons{circle(2. * w, 5. - 3.5 * w, 5.), circle(2. * w, 5. + 3.5 * w, 5.),
                                                    square(5. - 4. * w, 5. - 0.45 * w, 5. + 4. * w, 5. + 0.45 * w)});
    REQUIRE(dumbbell.size() == 1);
    CHECK(classify(print, dumbbell.front()).kind == slic3r_api::Support::Toolpath::BranchIslandClass::Sheath);
    PluginStorage storage;
    run_layer(print, storage, scale_i(0.6), square(0., 0., 10., 10.), {TypedArea{dumbbell.front(), base_type()}});

    const std::vector<const ExtrusionEntity *> children = bucket_children(print.object(0));
    REQUIRE(children.size() == 2);
    const size_t left = lies_in(*children[0], 5. - 5.5 * w, 5. - 2. * w, 5. - 1.5 * w, 5. + 2. * w) ? 0 : 1;
    CHECK(lies_in(*children[left], 5. - 5.5 * w, 5. - 2. * w, 5. - 1.5 * w, 5. + 2. * w));
    CHECK(lies_in(*children[1 - left], 5. + 1.5 * w, 5. - 2. * w, 5. + 5.5 * w, 5. + 2. * w));
}

// A thin island is replaced by one centreline entity. The entity takes the place of the first sheath piece of its island;
// an island whose pieces all vanished under the closing (a strip under one width wide) appends the entity after the others.
// The wide discs are sheaths. The test locates the strip entity by geometry and pins its rank against the two discs, which lie
// on a diagonal: with a piece, the strip ranks between them, which holds for the orders of the closing result that are monotone
// in x or in y; without a piece, it ranks after both.
TEST_CASE("A thin island takes the place of its sheath piece or comes after the sheaths", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const bool vanishes : {false, true}) {
        DYNAMIC_SECTION("strip " << (vanishes ? "without" : "with") << " a sheath piece") {
            Print print;
            slice(print, branch_config());
            const c_flow flow = SG::params(print, print.object(0)).base_flow;
            const double w = unscaled(flow.width), thickness = (vanishes ? 0.9 : 1.3) * w;
            const ExPolygon first = circle(1., 2., 2.), strip = square(4., 5.7, 8., 5.7 + thickness), last = circle(1., 10., 10.);
            CHECK(classify(print, strip).kind == slic3r_api::Support::Toolpath::BranchIslandClass::Centreline);
            PluginStorage storage;
            run_layer(print, storage, scale_i(0.6), square(0., 0., 12., 12.),
                      {TypedArea{first, base_type()}, TypedArea{strip, base_type()}, TypedArea{last, base_type()}});

            // The pieces that the closing leaves: the entity replaces one of them, or is added when the strip has none.
            const ExPolygons pieces = closing_ex(to_polygons(ExPolygons{first, strip, last}), double(SCALED_EPSILON),
                                                 double(SCALED_EPSILON) + 0.5 * double(flow.width));
            CHECK(pieces.size() == (vanishes ? 2u : 3u));
            const std::vector<const ExtrusionEntity *> children = bucket_children(print.object(0));
            CHECK(children.size() == pieces.size() + (vanishes ? 1u : 0u));
            REQUIRE(children.size() == 3);
            const size_t rank_first = child_in(children, 1., 1., 3., 3.), rank_last = child_in(children, 9., 9., 11., 11.);
            const size_t rank_strip = child_in(children, 4., 5.7, 8., 5.7 + thickness);
            if (vanishes)
                CHECK((rank_strip > rank_first && rank_strip > rank_last));
            else
                CHECK(((rank_first < rank_strip && rank_strip < rank_last) || (rank_last < rank_strip && rank_strip < rank_first)));

            // The centreline entity is reversible and holds line paths with the support material attributes, as wide as the
            // strip plus the rounded ends, where a sheath keeps the extrusion width.
            const double rounded_ends = unscaled(flow.height) * (1. - 0.25 * PI) * double(flow.spacing_ratio);
            CHECK(children[rank_strip]->can_reverse());
            size_t lines = 0;
            for_each_leaf(*children[rank_strip], [&](const ExtrusionEntity &leaf) {
                const ExtrusionAttributes *attributes = leaf.get_property<ExtrusionAttributes>();
                REQUIRE(attributes != nullptr);
                CHECK(attributes->role == ExtrusionRole::SupportMaterial);
                CHECK(double(attributes->width) == Approx(thickness + rounded_ends).margin(0.001));
                ++lines;
            });
            CHECK(lines >= 1);
        }
    }
}

// A thin elongated island that the closing cuts into two pieces: a bar 1.3 w wide whose middle is a neck 0.9 w wide. The sides
// keep a piece, the neck does not. The island prints one reversible entity at the rank of its first piece, none for the second,
// and the line runs over the whole bar. The discs on both sides lie on a diagonal, as in the case above.
TEST_CASE("A thin island cut in two pieces by the closing prints one entity", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice(print, branch_config());
    const c_flow flow = SG::params(print, print.object(0)).base_flow;
    const double w = unscaled(flow.width), side = 0.65 * w, neck = 0.45 * w, y = 6.;
    const ExPolygon first = circle(1., 2., 2.), last = circle(1., 10., 10.);
    const ExPolygon bar = polygon_of({{3., y - side}, {5.5, y - side}, {5.5, y - neck}, {6.5, y - neck}, {6.5, y - side}, {9., y - side},
                                      {9., y + side}, {6.5, y + side}, {6.5, y + neck}, {5.5, y + neck}, {5.5, y + side}, {3., y + side}});
    CHECK(classify(print, bar).kind == slic3r_api::Support::Toolpath::BranchIslandClass::Centreline);
    const double closing_distance = double(SCALED_EPSILON) + 0.5 * double(flow.width);
    CHECK(closing_ex(to_polygons(bar), double(SCALED_EPSILON), closing_distance).size() == 2);
    PluginStorage storage;
    run_layer(print, storage, scale_i(0.6), square(0., 0., 12., 12.),
              {TypedArea{first, base_type()}, TypedArea{bar, base_type()}, TypedArea{last, base_type()}});

    const ExPolygons pieces = closing_ex(to_polygons(ExPolygons{first, bar, last}), double(SCALED_EPSILON), closing_distance);
    CHECK(pieces.size() == 4);
    const std::vector<const ExtrusionEntity *> children = bucket_children(print.object(0));
    REQUIRE(children.size() == 3);
    const size_t rank_first = child_in(children, 1., 1., 3., 3.), rank_last = child_in(children, 9., 9., 11., 11.);
    const size_t rank_bar = child_in(children, 3., y - side, 9., y + side);
    CHECK(((rank_first < rank_bar && rank_bar < rank_last) || (rank_last < rank_bar && rank_bar < rank_first)));
    CHECK(children[rank_bar]->can_reverse());
    CHECK(total_length(*children[rank_bar]) > 5.5);
}

// A compact thin disc prints one non-reversible stationary deposit, with geometric volume and the support flow attributes.
TEST_CASE("A blob island prints one deposit at its contour centroid", "[plugins][support-toolpath][support-toolpath-branch][support-branch-blob][deposit]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice(print, branch_config());
    const double w = branch_width(print);
    const ExPolygon dot = circle(0.7 * w, 5., 5.);
    CHECK(classify(print, dot).kind == slic3r_api::Support::Toolpath::BranchIslandClass::Blob);
    PluginStorage storage;
    run_layer(print, storage, scale_i(0.6), square(0., 0., 10., 10.), {TypedArea{dot, base_type()}});

    const std::vector<const ExtrusionEntity *> children = bucket_children(print.object(0));
    REQUIRE(children.size() == 1);
    CHECK(children.front()->has_polyline());
    CHECK(slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(children.front())).is_deposit());
    REQUIRE(children.front()->polyline_ref().size() == 1);
    CHECK(distance_to(children.front()->polyline_ref().front(), 5., 5.) < 1e-5);
    CHECK_FALSE(children.front()->can_reverse());
    CHECK_FALSE(children.front()->can_sort());
    const ExtrusionAttributes *attributes = children.front()->get_property<ExtrusionAttributes>();
    REQUIRE(attributes != nullptr);
    CHECK(double(attributes->width) == Approx(w).epsilon(1e-6));
    const c_flow flow = SG::params(print, print.object(0)).base_flow;
    CHECK(attributes->role == ExtrusionRole::SupportMaterial);
    CHECK(attributes->mm3_per_mm == Approx(flow.mm3_per_mm));
    CHECK(double(attributes->height) == Approx(unscaled(flow.height)).epsilon(1e-6));
    const ExtrusionPropertySpecialCommand *command = children.front()->get_property<ExtrusionPropertySpecialCommand>();
    REQUIRE(command != nullptr);
    // Fusing and resolution validation define the producer's island, so its area need not equal the input disc's area.
    const slic3r_api::ClipperContext clipper(reinterpret_cast<storage_handle *>(&storage));
    slic3r_api::StoredExPolygonCollection fused = slic3r_api::clipper_offset2(
        clipper(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&dot))),
        double(SCALED_EPSILON), -double(SCALED_EPSILON)).to_expolygon_collection();
    fused.ensure_valid(SG::params(print, print.object(0)).resolution);
    REQUIRE(fused.size() == 1);
    CHECK(command->extra_data == Approx(unscaled(unscaled(fused.at(0).area())) * unscaled(flow.height)));
}

// The direct builder subtracts holes from volume. The contour centroid lies inside a hole here, so the island does not contain it and
// the deposit sits on the hole boundary closest to it.
TEST_CASE("A support deposit uses net area and moves off a centroid inside a hole", "[plugins][support-branch-blob][deposit]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice(print, branch_config());
    const c_flow flow = SG::params(print, print.object(0)).base_flow;
    ExPolygon island = square(4., 4., 6., 6.);
    Polygon hole = square(4.6, 4.8, 5.4, 5.3).contour;
    hole.make_clockwise();
    island.holes.push_back(hole);
    PluginStorage storage;
    slic3r_api::StoredExtrusionEntity parent(reinterpret_cast<storage_handle *>(&storage));
    slic3r_api::Support::Common::append_deposit(reinterpret_cast<storage_handle *>(&storage), parent.mutable_view(),
        slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&island)), flow);
    REQUIRE(parent.readonly().child_count() == 1);
    const slic3r_api::ExtrusionEntity deposit = parent.readonly().child(0);
    CHECK(deposit.is_deposit());
    REQUIRE(deposit.point_count() == 1);
    // The hole edges are 0.4, 0.4, 0.2 and 0.3 mm from the centroid (5, 5): the closest is y = 4.8.
    CHECK(deposit.point(0).x == scale_i(5.));
    CHECK(deposit.point(0).y == scale_i(4.8));
    CHECK(deposit.get(slic3r_api::EPropertySpecialCommand::key)->extra_data ==
        Approx((4. - 0.4) * unscaled(flow.height)));
}

// A crescent (a C-shaped island) has its contour centroid in the hollow: the deposit sits on the boundary, at the point closest to
// the centroid, and its volume stays the net area times the layer height.
TEST_CASE("A support deposit of a crescent sits on the boundary closest to the centroid", "[plugins][support-branch-blob][deposit]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice(print, branch_config());
    const c_flow flow = SG::params(print, print.object(0)).base_flow;
    const ExPolygons crescents = diff_ex(Polygons{circle(2., 5., 5.).contour}, Polygons{circle(1.8, 5.5, 5.).contour});
    REQUIRE(crescents.size() == 1);
    const ExPolygon &island = crescents.front();
    REQUIRE(island.holes.empty());
    const Point centroid = island.contour.centroid();
    REQUIRE_FALSE(island.contains(centroid));
    PluginStorage storage;
    slic3r_api::StoredExtrusionEntity parent(reinterpret_cast<storage_handle *>(&storage));
    slic3r_api::Support::Common::append_deposit(reinterpret_cast<storage_handle *>(&storage), parent.mutable_view(),
        slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&island)), flow);
    REQUIRE(parent.readonly().child_count() == 1);
    const slic3r_api::ExtrusionEntity deposit = parent.readonly().child(0);
    REQUIRE(deposit.point_count() == 1);
    const Point point(deposit.point(0).x, deposit.point(0).y);
    double closest = std::numeric_limits<double>::max(), on_boundary = std::numeric_limits<double>::max();
    for (const Line &line : island.contour.lines()) {
        closest = std::min(closest, line.distance_to(centroid));
        on_boundary = std::min(on_boundary, line.distance_to(point));
    }
    CAPTURE(closest, on_boundary, (point - centroid).cast<double>().norm());
    // The point lies on the boundary and is as close to the centroid as the boundary gets (a few units of rounding).
    CHECK(on_boundary <= 3.);
    CHECK((point - centroid).cast<double>().norm() <= closest + 3.);
    CHECK(deposit.get(slic3r_api::EPropertySpecialCommand::key)->extra_data == Approx(unscaled(unscaled(island.area())) * unscaled(flow.height)));
}
// Blob replacement preserves closed-piece order; a vanished blob appends after the sheaths instead of disappearing.
TEST_CASE("Blob islands replace their first piece or append after sheaths", "[plugins][support-toolpath-branch][support-branch-blob]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const bool vanishes : {false, true}) {
        DYNAMIC_SECTION("blob " << (vanishes ? "without" : "with") << " a sheath piece") {
            Print print;
            slice(print, branch_config());
            const double w = branch_width(print);
            const ExPolygon first = circle(1., 2., 2.), dot = circle((vanishes ? 0.45 : 0.7) * w, 6., 6.), last = circle(1., 10., 10.);
            CHECK(classify(print, dot).kind == slic3r_api::Support::Toolpath::BranchIslandClass::Blob);
            PluginStorage storage;
            run_layer(print, storage, scale_i(0.6), square(0., 0., 12., 12.),
                {TypedArea{first, base_type()}, TypedArea{dot, base_type()}, TypedArea{last, base_type()}});
            const std::vector<const ExtrusionEntity *> children = bucket_children(print.object(0));
            REQUIRE(children.size() == 3);
            const size_t a = child_in(children, 1., 1., 3., 3.), b = child_in(children, 9., 9., 11., 11.);
            const size_t t = child_in(children, 5., 5., 7., 7.);
            CHECK(slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(children[t])).is_deposit());
            if (vanishes)
                CHECK((t > a && t > b));
            else
                CHECK(((a < t && t < b) || (b < t && t < a)));
        }
    }
}

// An organic tree print exports stationary support moves after approach travel. With retraction disabled, positive E-only
// support moves are exactly deposits; their sum and the reader's whole support volume match the published geometry.
// Organic slabs use sparse support and select sheath through their style preference. The 0.8 mm width and equal
// tip diameter leave compact thin sections at the sampled layer heights with the default branch diameter.
TEST_CASE("Tree tips export deposits with conserved support volume", "[plugins][support-branch-blob][deposit][gcode]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("organic", false, 0, true);
    config.set_deserialize_strict({{"support_tree_tip_diameter", "0.8"}, {"support_material_extrusion_width", "0.8"},
        {"seam_visibility", "0"}, {"step_gcode_plugin", "gcode.printing_plan_file_writer"},
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"}, {"machine_limits_usage", "ignore"},
        {"use_relative_e_distances", "1"}, {"retract_length", "0,0"}, {"retract_restart_extra", "0,0"},
        {"wipe", "0,0"}, {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1.3,1.3"}, {"gcode_precision_e", "5"}});
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    const boost::filesystem::path path = Test::GCode::temporary_gcode_path();
    std::string code;
    print.process();
    // Measure the published support trees before export and separately accumulate the explicit deposit commands.
    double volume = 0., deposit_volume = 0.;
    size_t deposits = 0;
    for (const Layer &layer : print.object(0).auxiliary_layers()) {
        for (const std::pair<const std::pair<extruder_index_t, uint16_t>, SG::PathMeasure> &item : SG::measure_layer(layer))
            if (item.first.second == uint16_t(ExtrusionRole::SupportMaterial))
                volume += item.second.volume;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                if (region.has_extrusion(LayerRegionIsland::SUPPORT))
                    for_each_leaf(region.extrusion(LayerRegionIsland::SUPPORT), [&](const ExtrusionEntity &leaf) {
                        if (!slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(&leaf)).is_deposit()) return;
                        CHECK(layer.scaled_bottom_z() > 0);
                        deposit_volume += leaf.get_property<ExtrusionPropertySpecialCommand>()->extra_data;
                        ++deposits;
                    });
    }
    CAPTURE(volume, print.object(0).auxiliary_layers().size());
    REQUIRE(deposits > 0);
    try {
        Orchestrator::instance().export_gcode(print, path.string(), nullptr, nullptr);
        code = Test::GCode::read_text_file(path);
    } catch (...) {
        Test::GCode::remove_output_pair(path);
        throw;
    }
    Test::GCode::remove_output_pair(path);
    // Track the last XY approach and the active role; comments and machine-setting commands do not interrupt that approach.
    double emitted_e = 0.;
    size_t emitted_deposits = 0;
    bool approached = false;
    std::string type, line;
    std::istringstream stream(code);
    while (std::getline(stream, line)) {
        if (line.rfind(";TYPE:", 0) == 0) type = line.substr(6);
        const std::string command = line.substr(0, line.find(';'));
        if (command.rfind("G0 ", 0) != 0 && command.rfind("G1 ", 0) != 0) continue;
        const bool xy = command.find(" X") != std::string::npos || command.find(" Y") != std::string::npos;
        const size_t e = command.find(" E");
        if (xy) approached = e == std::string::npos;
        if (type == "Support material" && !xy && e != std::string::npos && std::stod(command.substr(e + 2)) > 0.) {
            CHECK(approached);
            CHECK(command.rfind("G1 ", 0) == 0);
            emitted_e += std::stod(command.substr(e + 2));
            ++emitted_deposits;
            approached = false;
        }
    }
    CHECK(emitted_deposits == deposits);
    const double filament_area = 0.25 * PI * 1.75 * 1.75;
    CHECK(emitted_e == Approx(deposit_volume * 1.3 / filament_area).margin(double(deposits) * 1e-5));
    // Read both moving paths and stationary deposits back through the public reader, including the extrusion multiplier.
    GCodeReaderService reader(Orchestrator::instance(), &print);
    const GCodeProcessorResult result = reader.read_for_viewer("gcode.reader.marlin2", code);
    CHECK(result.print_statistics.volumes_per_role_per_extruder.at(GCodeExtrusionRole::SupportMaterial).at(0) ==
        Approx(volume * 1.3).margin(0.01));
}

// The centreline of a thin strip: variable-width paths with the support attributes, inside the strip, along its length. A strip
// wider than the nozzle is printed as wide as it is, up to 1.5 widths of island and the rounded ends of the line.
TEST_CASE("The centreline of a thin strip runs inside it with the width of the strip", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice(print, branch_config());
    const c_flow flow = SG::params(print, print.object(0)).base_flow;
    const double w = unscaled(flow.width), height = unscaled(flow.height), nozzle = unscaled(flow.nozzle_diameter);
    const double rounded_ends = height * (1. - 0.25 * PI) * double(flow.spacing_ratio);
    const double strip_width = 0.5;
    PluginStorage storage;
    run_layer(print, storage, scale_i(0.6), square(0., 0., 10., 10.), {TypedArea{square(3., 5., 7., 5. + strip_width), base_type()}});

    const std::vector<const ExtrusionEntity *> children = bucket_children(print.object(0));
    REQUIRE(children.size() == 1);
    CHECK(lies_in(*children.front(), 3., 5., 7., 5. + strip_width));
    CHECK(total_length(*children.front()) > 3.);
    size_t lines = 0;
    for_each_leaf(*children.front(), [&](const ExtrusionEntity &leaf) {
        const ExtrusionAttributes *attributes = leaf.get_property<ExtrusionAttributes>();
        REQUIRE(attributes != nullptr);
        ++lines;
        CHECK(attributes->role == ExtrusionRole::SupportMaterial);
        CHECK(double(attributes->height) == Approx(height).epsilon(1e-6));
        // The strip is uniform: the line is as wide as the strip plus the rounded ends.
        CHECK(double(attributes->width) == Approx(strip_width + rounded_ends).margin(0.001));
        CHECK(double(attributes->width) >= nozzle - 1e-3);
        CHECK(double(attributes->width) <= slic3r_api::Support::Toolpath::centreline_max_width_over_width * w + rounded_ends + 1e-3);
        CHECK(attributes->mm3_per_mm == Approx(height * (double(attributes->width) - height * (1. - 0.25 * PI))).epsilon(1e-3));
    });
    CHECK(lines >= 1);
}

// A part whose extrusion width is below the nozzle by less than the widening limit prints as wide as the nozzle, with the flow
// of that width. Parts below the limit, or whose widening exceeds twice the support XY distance, have no line and print one stationary
// deposit instead. The strips are narrower than the support width, so no sheath could print in their place.
TEST_CASE("A strip within the widening limit below the nozzle prints as wide as the nozzle", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const double widening = slic3r_api::Support::Toolpath::centreline_max_widening;
    const PrintedLines printed = printed_lines(k_nozzle / (1. + 0.6 * widening), nullptr);
    REQUIRE(printed.lines.size() >= 1);
    for (const std::pair<double, double> &line : printed.lines) {
        CHECK(line.first == Approx(k_nozzle).margin(0.001));
        CHECK(line.second == Approx(printed.height * (line.first - printed.height * (1. - 0.25 * PI))).epsilon(1e-3));
    }
}

TEST_CASE("A strip more than the widening limit below the nozzle prints a deposit instead of a line", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const PrintedLines printed = printed_lines(k_nozzle / (1. + 1.5 * slic3r_api::Support::Toolpath::centreline_max_widening), nullptr);
    CHECK(printed.lines.empty());
    CHECK(printed.deposits == 1);
}

// With an XY distance of 0.01 mm the widening may not exceed 0.02 mm: the strip of the first case, 6 % under the nozzle, prints
// a deposit and no line, and a strip 0.01 mm under it, whose half widening is 0.005 mm, prints as wide as the nozzle.
TEST_CASE("The support XY distance bounds the widening of a strip", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const PrintedLines too_wide = printed_lines(k_nozzle / (1. + 0.6 * slic3r_api::Support::Toolpath::centreline_max_widening), "0.01");
    CHECK(too_wide.lines.empty());
    CHECK(too_wide.deposits == 1);
    const PrintedLines allowed = printed_lines(k_nozzle - 0.01, "0.01");
    REQUIRE(allowed.lines.size() >= 1);
    for (const std::pair<double, double> &line : allowed.lines)
        CHECK(line.first == Approx(k_nozzle).margin(0.001));
}

// Run a synthetic layer with a thin strip and a wider head; the local print and storage own its pipeline state. Check that the
// resulting variable-width centreline stays in the island and that every line is within the nozzle-to-maximum range. Growth
// precedes extrusion-width conversion; no anchor-skipped point in this shape stays below the nozzle (measured minimum 0.399999 mm).
TEST_CASE("A wide head on a thin strip gets lines between the nozzle and the maximum width", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice(print, branch_config());
    const c_flow flow = SG::params(print, print.object(0)).base_flow;
    const double w = unscaled(flow.width), height = unscaled(flow.height), nozzle = unscaled(flow.nozzle_diameter);
    const double rounded_ends = height * (1. - 0.25 * PI) * double(flow.spacing_ratio);
    const ExPolygons island = union_ex(ExPolygons{square(1., 5., 9., 5.5), square(9., 4.45, 10.6, 6.05)});
    REQUIRE(island.size() == 1);
    CHECK(classify(print, island.front()).kind == slic3r_api::Support::Toolpath::BranchIslandClass::Centreline);
    PluginStorage storage;
    run_layer(print, storage, scale_i(0.6), square(0., 0., 12., 12.), {TypedArea{island.front(), base_type()}});

    const std::vector<const ExtrusionEntity *> children = bucket_children(print.object(0));
    REQUIRE(children.size() == 1);
    CHECK(lies_in(*children.front(), 1., 4.45, 10.6, 6.05));
    CHECK(total_length(*children.front()) > 6.);
    for_each_leaf(*children.front(), [&](const ExtrusionEntity &leaf) {
        const ExtrusionAttributes *attributes = leaf.get_property<ExtrusionAttributes>();
        REQUIRE(attributes != nullptr);
        CHECK(double(attributes->width) >= nozzle - 1e-6);
        CHECK(double(attributes->width) <= slic3r_api::Support::Toolpath::centreline_max_width_over_width * w + rounded_ends + 1e-3);
    });
}
