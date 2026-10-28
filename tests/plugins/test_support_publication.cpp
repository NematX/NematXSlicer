#include <memory>

#include "support_grid_test_helpers.hpp"
#include "libslic3r/Plugins/Support/Common/SupportPublication.hpp"

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
