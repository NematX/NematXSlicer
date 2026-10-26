#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "support_grid_test_helpers.hpp"
#include "libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"

/* Tree branch sheaths, run through the support toolpath step on synthetic auxiliary layers. Each layer is built from a
   subject area and holds typed surfaces; the step then runs the active providers, and the helpers read the published
   region islands back. The first-layer case compares the branch flange with the legacy base flange of the same surface,
   and the provider case deactivates one of the two producers at a time. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
namespace {
// Activation ids of the two producers that compete for the support surfaces.
const char *const k_branch_id = "support.toolpath.branch_sheath";
const char *const k_sparse_id = "support.toolpath.legacy_sparse";

// A support surface with its surface type, as published on an auxiliary layer.
struct TypedArea {
    ExPolygon area;
    raw_surface_type type;
};

// Plain sparse support surface and branch surface of the first region island.
raw_surface_type base_type() { return slic3r_api::srf_type().internal().sparse().support().raw(); }
raw_surface_type branch_type() { return slic3r_api::srf_type().internal().sparse().support().branch().raw(); }

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
} // namespace

// A small round branch is one sheath path: it is not reversible, starts on an anchor inside the circle, turns
// counter-clockwise like the loop it leads into, and its length is the sheath circumference plus the anchor.
TEST_CASE("Branch sheath of a small round branch is one anchored non-reversible path", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice(print, SG::config());
    const double radius = 1.2;
    PluginStorage storage;
    run_layer(print, storage, scale_i(0.6), square(0., 0., 10., 10.), {TypedArea{circle(radius, 5., 5.), branch_type()}});

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

// The support flow and its role: the branch paths carry the base support material flow at the layer height, in the
// support bucket of the base-tool region island.
TEST_CASE("Branch sheath paths carry the support material flow and the base tool", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice(print, SG::config());
    PluginStorage storage;
    run_layer(print, storage, scale_i(0.6), square(0., 0., 10., 10.), {TypedArea{circle(1.2, 5., 5.), branch_type()}});

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
    slice(print, SG::config());
    const double radius = 2.5;
    PluginStorage storage;
    run_layer(print, storage, scale_i(0.6), square(0., 0., 10., 10.), {TypedArea{circle(radius, 5., 5.), branch_type()}});

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
    DynamicPrintConfig config = SG::config();
    config.set_deserialize_strict({{"support_tree_branch_diameter_double_wall", "0"}});
    Print print;
    slice(print, config);
    ExPolygon holed = square(2., 2., 8., 8.);
    Polygon hole({Point(scale_i(4.), scale_i(4.)), Point(scale_i(4.), scale_i(6.)), Point(scale_i(6.), scale_i(6.)),
                  Point(scale_i(6.), scale_i(4.))});
    hole.make_clockwise();
    holed.holes.push_back(hole);
    PluginStorage storage;
    run_layer(print, storage, scale_i(0.6), square(0., 0., 10., 10.), {TypedArea{holed, branch_type()}});

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
    DynamicPrintConfig config = SG::config();
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
    run_layer(print, storage, scale_i(0.6), square(0., 0., 10. * w, 14. * w), {TypedArea{surface, branch_type()}});

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
        slice(print, SG::config());
        PluginStorage storage;
        run_layer(print, storage, scale_i(0.2), square(0., 0., 10., 10.),
                  {TypedArea{shape, branch ? branch_type() : base_type()}});
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

// A plain sparse surface and a branch surface on one layer: each algorithm prints its own surface only. Turning one
// algorithm off removes exactly the surface it owned, which shows that the other one never printed it.
TEST_CASE("Plain and branch surfaces on one layer print through their own algorithms only", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto run = [](const char *disabled) {
        Print print;
        slice(print, SG::config());
        std::unique_ptr<ScopedProviderOff> off;
        if (*disabled != '\0')
            off = std::make_unique<ScopedProviderOff>(disabled);
        PluginStorage storage;
        run_layer(print, storage, scale_i(0.6), square(0., 0., 10., 10.),
                  {TypedArea{square(0.5, 0.5, 4.5, 9.5), base_type()}, TypedArea{circle(1.5, 7.5, 5.), branch_type()}});
        return count_sides(print.object(0));
    };
    const Sides both = run("");
    CHECK(both.plain > 0);
    CHECK(both.branch > 0);
    CHECK(both.mixed == 0);
    const Sides without_branch = run(k_branch_id);
    CHECK(without_branch.plain > 0);
    CHECK(without_branch.branch == 0);
    const Sides without_sparse = run(k_sparse_id);
    CHECK(without_sparse.plain == 0);
    CHECK(without_sparse.branch > 0);
}

// The branch provider is an active support toolpath algorithm of its own role group, between the sparse base and the
// interfaces, and it declares the settings it reads: the shared support lists and the double-wall branch diameter.
TEST_CASE("Branch sheath provider has its role group, priority and declared settings", "[plugins][support-toolpath][support-toolpath-branch]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Plugin *provider = Orchestrator::instance().get_plugin(k_branch_id);
    REQUIRE(provider != nullptr);
    CHECK(provider->get_step() == STEP_SUPPORT_TOOLPATH);
    CHECK(provider->get_exclusive_group() == "support_toolpath_branch");
    CHECK(provider->get_priority() == 50);
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
