#include <catch2/catch.hpp>

#include <algorithm>
#include <vector>

#include "support_grid_test_helpers.hpp"
#include "libslic3r/Plugins/Support/Common/SupportInterfaces.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"

/* The cones of the contacts (Common::merge_contact_cones()), checked on synthetic slabs without slicing a model. A 0.2-mm stack
   of base slabs lies under a top contact or over a bottom contact; each scene merges the cones without trimming and reads the
   slabs. A cone shrinks by one step per slab, stops when it vanishes or when the support it meets covers it, and grows from the
   support it joins. Configured-step cases derive the step through SupportParams, then measure both directions on the same stack. */

using namespace Slic3r;
namespace SC = slic3r_api::Support::Common;
namespace SG = Slic3r::Test::SupportGrid;

namespace {
// The rectangle [x0, x1] x [0, y1] (mm).
slic3r_api::StoredPolygonCollection rectangle(storage_handle *storage, double x0, double x1, double y1)
{
    const std::vector<c_point> corners{{scale_i(x0), 0}, {scale_i(x1), 0}, {scale_i(x1), scale_i(y1)}, {scale_i(x0), scale_i(y1)}};
    slic3r_api::StoredPolygon polygon(storage);
    polygon.insert_array(0, corners.data(), uint32_t(corners.size()));
    slic3r_api::StoredPolygonCollection result(storage);
    result.push_back_move(std::move(polygon));
    return result;
}

// A square of the given side (mm) whose lower left corner is at (x, 0) mm.
slic3r_api::StoredPolygonCollection square(storage_handle *storage, double x, double side)
{
    return rectangle(storage, x, x + side, side);
}

// Empty base slabs of 0.2 mm from bottom (mm), count of them.
struct Scene {
    PluginStorage storage;
    SC::GeneratedSupport support{SG::handle(storage)};
    Scene(double bottom, size_t count)
    {
        // The geometry of the slabs goes through the plugin API table of the host.
        Test::Plugins::ensure_plugin_test_runtime_initialized();
        for (size_t i = 0; i < count; ++i) {
            const coord_t top = scale_to_layer_coord(bottom + 0.2 * double(i + 1));
            SC::GeneratorLayer &slab = SC::add_layer(support, SC::LayerType::Base, top, top - scale_to_layer_coord(0.2),
                                                     scale_to_layer_coord(0.2));
            support.base.push_back(&slab);
        }
    }
    SC::GeneratorLayer &contact(SC::LayerType type, double top, slic3r_api::StoredPolygonCollection &&polygons)
    {
        SC::GeneratorLayer &slab = SC::add_layer(support, type, scale_to_layer_coord(top), scale_to_layer_coord(top - 0.2),
                                                 scale_to_layer_coord(0.2));
        slab.polygons = std::move(polygons);
        (type == SC::LayerType::TopContact ? support.top : support.bottom).push_back(&slab);
        return slab;
    }
    // Merge the cones without trimming them, then return the area (mm2) of every slab, bottom first.
    std::vector<double> areas(coord_t step)
    {
        SC::merge_contact_cones(step, SCALED_EPSILON, support, support.base,
            [this](size_t, const slic3r_api::StoredPolygonCollection &cone) { return cone.readonly().clone(support.storage); });
        std::vector<double> result;
        for (const SC::GeneratorLayer *slab : support.base)
            result.push_back(unscaled(unscaled(area(union_ex(SG::native(slab->polygons))))));
        return result;
    }
};

// The area of a side-mm square shrunk depth times by 0.5 mm on each side.
double shrunk(double side, int depth)
{
    const double s = side - 2. * 0.5 * depth;
    return s > 0 ? s * s : 0.;
}
} // namespace

TEST_CASE("Contact cones shrink slab by slab below a top contact until they vanish", "[plugins][support-contact-xy]")
{
    Scene scene(0., 12); // slabs topping at 0.2 ... 2.4
    scene.contact(SC::LayerType::TopContact, 2.6, square(scene.support.storage, 0., 10.));
    const std::vector<double> areas = scene.areas(scale_i(0.5));
    // A 10-mm square vanishes at the tenth step: the slabs 1 to 9 below the contact get it, the others nothing.
    for (int depth = 1; depth <= 12; ++depth) {
        CAPTURE(depth);
        CHECK(areas[size_t(12 - depth)] == Approx(shrunk(10., depth)));
    }
}

TEST_CASE("Contact cones stop where the support covers them", "[plugins][support-contact-xy]")
{
    Scene scene(0., 5);
    for (SC::GeneratorLayer *slab : scene.support.base)
        slab->polygons = square(scene.support.storage, 0., 10.);
    scene.contact(SC::LayerType::TopContact, 1.2, square(scene.support.storage, 0., 10.));
    for (const double area : scene.areas(scale_i(0.5)))
        CHECK(area == Approx(100.));
}

TEST_CASE("Contact cones join the support they meet and follow it", "[plugins][support-contact-xy]")
{
    // The support keeps 3 mm from a wall at x = 0; the contact reaches the wall. Each slab gets the cone 0.5 mm further from the
    // wall, grown from the union with the support above, until the support covers it.
    Scene scene(0., 8);
    for (SC::GeneratorLayer *slab : scene.support.base)
        slab->polygons = rectangle(scene.support.storage, 3., 10., 10.);
    scene.contact(SC::LayerType::TopContact, 1.8, square(scene.support.storage, 0., 10.));
    scene.areas(scale_i(0.5));
    for (int depth = 1; depth <= 8; ++depth) {
        CAPTURE(depth);
        const BoundingBox box = get_extents(SG::native(scene.support.base[size_t(8 - depth)]->polygons));
        CHECK(unscaled(box.min.x()) == Approx(std::min(3., 0.5 * depth)));
    }
}

TEST_CASE("Contact cones shrink upwards above a bottom contact", "[plugins][support-contact-xy]")
{
    Scene scene(0.2, 3); // slabs topping at 0.4, 0.6 and 0.8
    scene.contact(SC::LayerType::BottomContact, 0.2, square(scene.support.storage, 0., 10.));
    const std::vector<double> areas = scene.areas(scale_i(0.5));
    CHECK(areas[0] == Approx(shrunk(10., 1)));
    CHECK(areas[1] == Approx(shrunk(10., 2)));
    CHECK(areas[2] == Approx(shrunk(10., 3)));
}

TEST_CASE("Contact cones do nothing without a step", "[plugins][support-contact-xy]")
{
    Scene scene(0., 3);
    scene.contact(SC::LayerType::TopContact, 0.8, square(scene.support.storage, 0., 10.));
    for (const double area : scene.areas(0))
        CHECK(area == 0.);
}

// Changing the configured distance changes the footprint at every depth and the last slab reached by the cone.
TEST_CASE("Configured cone growth controls top and bottom contact footprints", "[plugins][support-contact-xy]")
{
    const bool above = GENERATE(false, true);
    for (const char *growth : {"25%", "67%", "100%", "0.268", "0"}) {
        CAPTURE(above, growth);
        DynamicPrintConfig config = SG::config();
        config.set_deserialize_strict({{"support_material_xy_spacing", "1.5"}, {"support_material_contact_xy_spacing", "0.1"},
            {"support_material_contact_cone_growth", growth}});
        Print print;
        Model model;
        Test::init_print({SG::slab()}, print, model, config);
        const SC::SupportParams params = SG::params(print, print.object(0));
        Scene scene(above ? 0.2 : 0., 10);
        scene.contact(above ? SC::LayerType::BottomContact : SC::LayerType::TopContact, above ? 0.2 : 2.2,
                      square(scene.support.storage, 0., 10.));
        const std::vector<double> areas = scene.areas(params.contact_cone_step);
        REQUIRE(areas.size() == 10);
        for (size_t i = 0; i < areas.size(); ++i) {
            const size_t depth = above ? i + 1 : areas.size() - i;
            const double side = params.contact_cone_step == 0 ? 0. :
                std::max(0., 10. - 2. * unscaled(params.contact_cone_step) * double(depth));
            CHECK(areas[i] == Approx(side * side).margin(1e-6));
        }
    }
}
