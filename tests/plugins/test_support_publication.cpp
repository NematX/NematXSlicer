#include <memory>

#include "support_grid_test_helpers.hpp"
#include "libslic3r/Plugins/Support/Common/SupportPublication.hpp"
#include "libslic3r/Plugins/Support/Common/SupportCounters.hpp"
#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"

/* Publication of hand-typed slabs, without a support family. Each test fills a GeneratedSupport with slabs of chosen
   types, publishes it through Common::publish and reads the auxiliary layers back. A branch slab is published as a
   branch surface of the base extruder and a base slab as a plain sparse surface. Every area keeps one owner: an
   interface keeps the area it shares with a branch, and a branch keeps the area it shares with a base slab. The last
   test runs the support toolpath step and checks that the branch sheath prints the branch surface. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
using slic3r_api::Support::Common::GeneratedSupport;
using slic3r_api::Support::Common::GeneratorLayer;
using slic3r_api::Support::Common::GeneratorLayers;
using slic3r_api::Support::Common::LayerType;

namespace {
// Provider id of the branch sheath, the algorithm that prints branch surfaces.
const char *const k_branch_id = "support.toolpath.branch_sheath";
// Surface types of the three published roles.
const SurfaceType branch_type = stModSupport | stModBranch | stPosInternal | stDensSparse;
const SurfaceType sparse_type = stModSupport | stPosInternal | stDensSparse;
const SurfaceType top_type = stModSupport | stPosTop | stDensSolid;
const SurfaceType bottom_type = stModSupport | stPosBottom | stDensSolid;
// SG::config() prints the base support on extruder index 0 and the interfaces on extruder index 1.
const extruder_index_t base_extruder = 0;
const extruder_index_t interface_extruder = 1;

// A rectangle with corners (x0, y0) and (x1, y1), in millimeters and counter-clockwise.
Polygon rect_polygon(double x0, double y0, double x1, double y1)
{
    return Polygon{Point(scale_i(x0), scale_i(y0)), Point(scale_i(x1), scale_i(y0)),
                   Point(scale_i(x1), scale_i(y1)), Point(scale_i(x0), scale_i(y1))};
}

// Append a slab of the given type over the area, at print Z 0.6 mm and 0.2 mm thick, and list it under its role.
void add_slab(GeneratedSupport &support, GeneratorLayers &role, LayerType type, const Polygon &area)
{
    support.slabs.emplace_back(support.storage);
    GeneratorLayer &slab = support.slabs.back();
    slab.type = type;
    slab.print_z = scale_i(0.6);
    slab.height = scale_i(0.2);
    slab.bottom_z = slab.print_z - slab.height;
    const Polygons polygons{area};
    polygons_copy(slab.polygons.mutable_handle(), reinterpret_cast<const polygon_collection_handle *>(&polygons));
    role.push_back(&slab);
}

// Slice a 20 mm cube with the grid settings, which leaves the auxiliary layers free to publish.
void slice_cube(Print &print)
{
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, SG::config());
    SG::prefix(print);
}

// Derives one parameter set and publishes borrowed slabs on the sliced print's first object; the object owns published layers, and publication/allocation failures propagate.
void publish_support(Print &print, GeneratedSupport &support)
{
    slic3r_api::Support::Common::publish(nullptr, SG::object_view(print.object(0)), SG::params(print, print.object(0)), support,
                                         reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
}

// The fill surfaces of one type printed on one extruder, over every auxiliary layer of the object.
ExPolygons surfaces_of(const PrintObject &object, SurfaceType type, extruder_index_t extruder)
{
    ExPolygons out;
    for (const Layer &layer : object.auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands()) {
                if (region.extruder_id() != extruder)
                    continue;
                for (const Surface &surface : region.fill_surfaces())
                    if (surface.surface_type == type)
                        out.push_back(surface.expolygon);
            }
    return out;
}

// True when both sets of areas cover the same points, whatever way their polygons are split.
bool same_area(const ExPolygons &a, const ExPolygons &b)
{
    return diff_ex(a, b).empty() && diff_ex(b, a).empty();
}

// Published support polylines counted by side of x = 9 mm: the branch square lies left of it, the base square right of it.
struct Sides {
    size_t branch = 0, base = 0, mixed = 0;
};

// Count the support polylines of the auxiliary layers that lie entirely on one side of x = 9 mm; the others are mixed.
Sides count_sides(const PrintObject &object)
{
    Sides sides;
    for (const Layer &layer : object.auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands()) {
                if (!region.has_extrusion(LayerRegionIsland::SUPPORT))
                    continue;
                for (const ArcPolyline &line : region.extrusion(LayerRegionIsland::SUPPORT).as_polylines()) {
                    bool left = true, right = true;
                    for (size_t index = 0; index < line.size(); ++index) {
                        left = left && line.get_point(index).x() <= scale_i(9.);
                        right = right && line.get_point(index).x() >= scale_i(9.);
                    }
                    if (left)
                        ++sides.branch;
                    else if (right)
                        ++sides.base;
                    else
                        ++sides.mixed;
                }
            }
    return sides;
}

// Deactivate a support toolpath provider for the scope of a test; the destructor activates it again.
class ScopedProviderOff {
public:
    explicit ScopedProviderOff(const char *id) : m_id(id) { REQUIRE(Orchestrator::instance().set_plugin_active(m_id, false)); }
    ~ScopedProviderOff() { Orchestrator::instance().set_plugin_active(m_id, true); }

private:
    const char *m_id;
};
} // namespace

// Publishes two separated role islands plus an empty slab with counters enabled between joined runs; verifies every
// per-island union, including empty-input unions and normalization, while scoped restoration preserves enablement on failure.
TEST_CASE("Publication counts role normalization unions", "[plugins][support-publication][support-counters-check]")
{
    namespace Counters = slic3r_api::Support::Common::Counters;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice_cube(print);
    PluginStorage storage;
    GeneratedSupport support{reinterpret_cast<storage_handle *>(&storage)};
    add_slab(support, support.interfaces, LayerType::TopInterface, rect_polygon(2., 2., 6., 6.));
    add_slab(support, support.base, LayerType::Base, rect_polygon(12., 2., 16., 6.));
    add_slab(support, support.base, LayerType::Branch, rect_polygon(2., 2., 6., 6.));
    support.slabs.back().polygons.clear();
    struct RestoreCounters {
        bool previous;
        // Restores the borrowed process enable state after joined publication, including assertion/exception exits, without failure.
        ~RestoreCounters() { Counters::set_enabled_for_tests(previous); }
    } restore{Counters::set_enabled_for_tests(true)};
    Counters::reset();
    publish_support(print, support);
    REQUIRE(print.object(0).auxiliary_layers().size() == 1);
    CHECK(Counters::totals.groups.load(std::memory_order_relaxed) == 1);
    CHECK(Counters::totals.islands.load(std::memory_order_relaxed) == 2);
    CHECK(Counters::totals.slabs.load(std::memory_order_relaxed) == 3);
    CHECK(Counters::totals.intersections.load(std::memory_order_relaxed) == 6);
    CHECK(Counters::totals.differences.load(std::memory_order_relaxed) == 0);
    // Each island makes three role-combination unions and two calls for its one nonempty role: union plus normalization.
    CHECK(Counters::totals.unions.load(std::memory_order_relaxed) == 2 * (3 + 2));
}

// Publishes an exact near-closing slab ring with and without a remote ring, exercising filtered and borrowed operands through
// the public publisher. Compares full-operand intersections and their published normalization point-for-point; storage owns
// temporary geometry, the sliced print owns surfaces, and geometry/allocation/publication failures fail the test.
TEST_CASE("Publication filtering preserves near-closing ring vertices", "[plugins][support-publication][support-publication-filter]")
{
    namespace Common = slic3r_api::Support::Common;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const bool remote : {false, true}) {
        CAPTURE(remote);
        Print print;
        slice_cube(print);
        PluginStorage storage;
        storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
        GeneratedSupport support{handle};
        Polygon near;
        near.points = {Point(0, 0), Point(1000000, 0), Point(1000000, 1000000), Point(0, 10)};
        add_slab(support, support.interfaces, LayerType::TopInterface, near);
        Polygons rings{near};
        if (remote) rings.push_back(rect_polygon(12., 2., 16., 6.));
        polygons_copy(support.slabs.back().polygons.mutable_handle(), reinterpret_cast<const polygon_collection_handle *>(&rings));
        REQUIRE(support.slabs.back().polygons.at(0).size() == 4);
        publish_support(print, support);
        const PrintObject &object = print.object(0);
        REQUIRE(object.auxiliary_layers().size() == 1);
        const ExPolygons actual = surfaces_of(object, top_type, interface_extruder);
        REQUIRE(actual.size() == (remote ? 2 : 1));
        const slic3r_api::ClipperContext clipper(handle);
        const slic3r_api::PolygonCollection full(reinterpret_cast<const polygon_collection_handle *>(&rings));
        const Polygons local_rings{near};
        const slic3r_api::PolygonCollection local(reinterpret_cast<const polygon_collection_handle *>(&local_rings));
        ExPolygons expected;
        size_t near_islands = 0;
        for (const LayerSliceIsland &island : object.auxiliary_layers().front().islands()) {
            const slic3r_api::ExPolygon slice(reinterpret_cast<const expolygon_handle *>(&island.get_slice()));
            const slic3r_api::StoredPolygonCollection clipped = slic3r_api::clipper_intersection(clipper(full), clipper(slice)).to_polygon_collection();
            // The island near the origin cannot touch the remote rectangle: removing it must preserve every output point.
            if (slice.contour().bounding_box().max.x < scale_i(2.)) {
                ++near_islands;
                REQUIRE_FALSE(clipped.empty());
                const slic3r_api::StoredPolygonCollection local_clip = slic3r_api::clipper_intersection(clipper(local), clipper(slice)).to_polygon_collection();
                CHECK(SG::native(clipped) == SG::native(local_clip));
                slic3r_api::StoredPolygonCollection moved_copy(handle);
                moved_copy.push_back(local.at(0));
                REQUIRE(moved_copy.at(0).size() == 3);
                const slic3r_api::StoredPolygonCollection changed_clip = slic3r_api::clipper_intersection(clipper(moved_copy), clipper(slice)).to_polygon_collection();
                CHECK(SG::native(clipped) != SG::native(changed_clip));
            }
            const slic3r_api::StoredExPolygonCollection normalized = Common::native_expolygons(slic3r_api::clipper_union(clipper(clipped)));
            for (const slic3r_api::ExPolygon polygon : normalized)
                expected.push_back(*reinterpret_cast<const ExPolygon *>(polygon.handle()));
        }
        REQUIRE(near_islands == 1);
        REQUIRE(actual.size() == expected.size());
        for (size_t index = 0; index < expected.size(); ++index) {
            CHECK(actual[index].contour.points == expected[index].contour.points);
            REQUIRE(actual[index].holes.size() == expected[index].holes.size());
            for (size_t hole = 0; hole < expected[index].holes.size(); ++hole)
                CHECK(actual[index].holes[hole].points == expected[index].holes[hole].points);
        }
    }
}

// A slab without polygons publishes nothing: no auxiliary layer, island or extrusion is created for it.
TEST_CASE("Empty slab publishes nothing and prints nothing", "[plugins][support-publication]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice_cube(print);
    PluginStorage storage;
    GeneratedSupport support{reinterpret_cast<storage_handle *>(&storage)};
    add_slab(support, support.base, LayerType::Base, rect_polygon(2., 2., 6., 6.));
    support.slabs.back().polygons.clear();
    REQUIRE(support.slabs.back().polygons.empty());
    publish_support(print, support);
    REQUIRE_NOTHROW(SG::toolpaths(print));
    CHECK(print.object(0).auxiliary_layers().empty());
}

// A branch slab prints as a branch surface of the base extruder, without the area that the interface above owns.
TEST_CASE("Branch slab publishes as a branch surface and yields the area an interface covers", "[plugins][support-publication]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice_cube(print);
    PluginStorage storage;
    GeneratedSupport support{reinterpret_cast<storage_handle *>(&storage)};
    add_slab(support, support.base, LayerType::Branch, rect_polygon(2., 2., 6., 6.));
    add_slab(support, support.interfaces, LayerType::TopInterface, rect_polygon(4., 2., 8., 6.));
    add_slab(support, support.base, LayerType::Base, rect_polygon(12., 2., 16., 6.));
    publish_support(print, support);

    const PrintObject &object = print.object(0);
    REQUIRE(object.auxiliary_layers().size() == 1);
    CHECK(same_area(surfaces_of(object, branch_type, base_extruder), ExPolygons{ExPolygon(rect_polygon(2., 2., 4., 6.))}));
    CHECK(same_area(surfaces_of(object, sparse_type, base_extruder), ExPolygons{ExPolygon(rect_polygon(12., 2., 16., 6.))}));
    CHECK(same_area(surfaces_of(object, top_type, interface_extruder), ExPolygons{ExPolygon(rect_polygon(4., 2., 8., 6.))}));
}

// The areas of contact slabs carry the contact bit on both sides, a bridging lower contact carries the bridge and contact bits, a contact keeps the
// area it shares with an interface, and the base, branch and interface areas carry no contact bit.
TEST_CASE("Contact slabs publish their areas with the contact bit", "[plugins][support-publication]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice_cube(print);
    PluginStorage storage;
    GeneratedSupport support{reinterpret_cast<storage_handle *>(&storage)};
    add_slab(support, support.top, LayerType::TopContact, rect_polygon(2., 2., 6., 6.));
    add_slab(support, support.interfaces, LayerType::TopInterface, rect_polygon(4., 2., 8., 6.));
    add_slab(support, support.bottom, LayerType::BottomContact, rect_polygon(2., 12., 6., 16.));
    add_slab(support, support.interfaces, LayerType::BottomInterface, rect_polygon(4., 12., 8., 16.));
    add_slab(support, support.bottom, LayerType::BottomContact, rect_polygon(2., 22., 6., 26.));
    support.slabs.back().bridging = true;
    add_slab(support, support.base, LayerType::Base, rect_polygon(12., 2., 16., 6.));
    add_slab(support, support.base, LayerType::Branch, rect_polygon(12., 12., 16., 16.));
    publish_support(print, support);

    const PrintObject &object = print.object(0);
    REQUIRE(object.auxiliary_layers().size() == 1);
    CHECK(same_area(surfaces_of(object, SurfaceType(top_type | stModContact), interface_extruder), ExPolygons{ExPolygon(rect_polygon(2., 2., 6., 6.))}));
    CHECK(same_area(surfaces_of(object, top_type, interface_extruder), ExPolygons{ExPolygon(rect_polygon(6., 2., 8., 6.))}));
    CHECK(same_area(surfaces_of(object, SurfaceType(bottom_type | stModContact), interface_extruder), ExPolygons{ExPolygon(rect_polygon(2., 12., 6., 16.))}));
    CHECK(same_area(surfaces_of(object, bottom_type, interface_extruder), ExPolygons{ExPolygon(rect_polygon(6., 12., 8., 16.))}));
    CHECK(same_area(surfaces_of(object, SurfaceType(bottom_type | stModBridge | stModContact), interface_extruder),
                    ExPolygons{ExPolygon(rect_polygon(2., 22., 6., 26.))}));
    CHECK(same_area(surfaces_of(object, sparse_type, base_extruder), ExPolygons{ExPolygon(rect_polygon(12., 2., 16., 6.))}));
    CHECK(same_area(surfaces_of(object, branch_type, base_extruder), ExPolygons{ExPolygon(rect_polygon(12., 12., 16., 16.))}));
    // The base and branch areas, and the dense areas of a side that are not a contact slab, carry no contact bit.
    for (const Layer &layer : object.auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces()) {
                    const SurfaceType type = surface.surface_type;
                    CAPTURE(surfaceType_to_string(type));
                    if ((type & stPosInternal) != 0 && (type & stDensSparse) != 0)
                        CHECK((type & stModContact) == 0);
                    if (type == top_type || type == bottom_type || type == (bottom_type | stModBridge))
                        CHECK((type & stModContact) == 0);
                }
}

// Without interface layers on a side the remaining dense areas are its contacts: a contact that cannot merge into the base keeps its contact bit,
// while a mergeable contact is published as sparse base without it.
TEST_CASE("Contact bit survives a side without interface layers and drops when the contact merges into the base", "[plugins][support-publication]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool same_tool = GENERATE(false, true);
    CAPTURE(same_tool);
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, SG::config("grid", false, 0, same_tool));
    SG::prefix(print);
    PluginStorage storage;
    GeneratedSupport support{reinterpret_cast<storage_handle *>(&storage)};
    add_slab(support, support.top, LayerType::TopContact, rect_polygon(2., 2., 6., 6.));
    publish_support(print, support);

    const PrintObject &object = print.object(0);
    REQUIRE(object.auxiliary_layers().size() == 1);
    const ExPolygons area{ExPolygon(rect_polygon(2., 2., 6., 6.))};
    // Separate support extruders: the contact is dense and keeps the bit.
    // One support extruder with no base slab: the contact merges into the base.
    const bool merged = same_tool;
    CHECK(same_area(surfaces_of(object, merged ? sparse_type : SurfaceType(top_type | stModContact), merged ? base_extruder : interface_extruder), area));
    CHECK(surfaces_of(object, merged ? SurfaceType(top_type | stModContact) : sparse_type, merged ? interface_extruder : base_extruder).empty());
    for (const Layer &layer : object.auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces())
                    if (merged)
                        CHECK((surface.surface_type & stModContact) == 0);
                    else
                        CHECK((surface.surface_type & stModContact) != 0);
}

// A base slab yields the area it shares with a branch slab, so that area is printed once.
TEST_CASE("Base slab yields the area it shares with a branch slab", "[plugins][support-publication]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    slice_cube(print);
    PluginStorage storage;
    GeneratedSupport support{reinterpret_cast<storage_handle *>(&storage)};
    add_slab(support, support.base, LayerType::Branch, rect_polygon(2., 2., 6., 6.));
    add_slab(support, support.base, LayerType::Base, rect_polygon(4., 2., 10., 6.));
    publish_support(print, support);

    const PrintObject &object = print.object(0);
    CHECK(same_area(surfaces_of(object, branch_type, base_extruder), ExPolygons{ExPolygon(rect_polygon(2., 2., 6., 6.))}));
    CHECK(same_area(surfaces_of(object, sparse_type, base_extruder), ExPolygons{ExPolygon(rect_polygon(6., 2., 10., 6.))}));
}

// The branch surface is printed by the branch sheath when the support toolpath step runs: with that provider off it gets no
// extrusion, while the base square keeps its sparse extrusion.
TEST_CASE("Published branch surface is printed by the branch sheath", "[plugins][support-publication]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto print_layer = [](bool branch_provider) {
        Print print;
        slice_cube(print);
        std::unique_ptr<ScopedProviderOff> off;
        if (!branch_provider)
            off = std::make_unique<ScopedProviderOff>(k_branch_id);
        PluginStorage storage;
        GeneratedSupport support{reinterpret_cast<storage_handle *>(&storage)};
        add_slab(support, support.base, LayerType::Branch, rect_polygon(2., 2., 6., 6.));
        add_slab(support, support.base, LayerType::Base, rect_polygon(12., 2., 16., 6.));
        publish_support(print, support);
        SG::toolpaths(print);
        return count_sides(print.object(0));
    };
    const Sides both = print_layer(true);
    CHECK(both.branch > 0);
    CHECK(both.base > 0);
    CHECK(both.mixed == 0);
    const Sides without_branch = print_layer(false);
    CHECK(without_branch.branch == 0);
    CHECK(without_branch.base > 0);
}
