#include <catch2/catch.hpp>

#include <vector>

#include "support_grid_test_helpers.hpp"
#include "libslic3r/Plugins/Support/Common/SupportInterfaces.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"

/* The cones of the contacts (Common::contact_cones()), checked on synthetic slabs without slicing a model. A 0.2-mm stack of
   base slabs lies under a top contact or over a bottom contact; each scene reads the area of the cone each slab receives. The
   depth counts only the slabs whose column meets the contact, and the walk stops at the depth limit or when the cone vanishes. */

using namespace Slic3r;
namespace SC = slic3r_api::Support::Common;
namespace SG = Slic3r::Test::SupportGrid;

namespace {
// A square of the given side (mm) whose lower left corner is at (x, 0) mm.
slic3r_api::StoredPolygonCollection square(storage_handle *storage, double x, double side)
{
    const std::vector<c_point> corners{{scale_i(x), 0}, {scale_i(x + side), 0}, {scale_i(x + side), scale_i(side)},
                                       {scale_i(x), scale_i(side)}};
    slic3r_api::StoredPolygon polygon(storage);
    polygon.insert_array(0, corners.data(), uint32_t(corners.size()));
    slic3r_api::StoredPolygonCollection result(storage);
    result.push_back_move(std::move(polygon));
    return result;
}

// Base slabs of 0.2 mm from bottom (mm), count of them, each with a 10-mm square column at the origin.
struct Scene {
    PluginStorage storage;
    SC::GeneratedSupport support{SG::handle(storage)};
    std::vector<slic3r_api::StoredPolygonCollection> columns;
    Scene(double bottom, size_t count)
    {
        // The geometry of the slabs goes through the plugin API table of the host.
        Test::Plugins::ensure_plugin_test_runtime_initialized();
        for (size_t i = 0; i < count; ++i) {
            const coord_t top = scale_to_layer_coord(bottom + 0.2 * double(i + 1));
            SC::GeneratorLayer &slab = SC::add_layer(support, SC::LayerType::Base, top, top - scale_to_layer_coord(0.2),
                                                     scale_to_layer_coord(0.2));
            support.base.push_back(&slab);
            columns.push_back(square(support.storage, 0., 10.));
        }
    }
    // The area (mm2) of the cone of every slab, bottom first.
    std::vector<double> areas(coord_t step, size_t max_depth)
    {
        const std::vector<slic3r_api::StoredPolygonCollection> cones = SC::contact_cones(step, max_depth, support, support.base, columns);
        std::vector<double> result;
        for (const slic3r_api::StoredPolygonCollection &cone : cones)
            result.push_back(unscaled(unscaled(area(SG::native(cone)))));
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

TEST_CASE("Contact cones shrink by one step per slab below a top contact", "[plugins][support-contact-xy]")
{
    Scene scene(0.8, 5); // slabs topping at 1.0, 1.2, 1.4, 1.6 and 1.8
    SC::GeneratorLayer &contact = SC::add_layer(scene.support, SC::LayerType::TopContact, scale_to_layer_coord(2.),
                                                scale_to_layer_coord(1.8), scale_to_layer_coord(0.2));
    contact.polygons = square(scene.support.storage, 0., 10.);
    scene.support.top.push_back(&contact);
    const coord_t step = scale_i(0.5);

    SECTION("the depth limit ends the cone") {
        const std::vector<double> areas = scene.areas(step, 3);
        CHECK(areas[4] == Approx(shrunk(10., 1)));
        CHECK(areas[3] == Approx(shrunk(10., 2)));
        CHECK(areas[2] == Approx(shrunk(10., 3)));
        CHECK(areas[1] == 0.);
        CHECK(areas[0] == 0.);
    }
    SECTION("a slab of another column is not counted") {
        scene.columns[3] = square(scene.support.storage, 20., 5.);
        const std::vector<double> areas = scene.areas(step, 3);
        CHECK(areas[4] == Approx(shrunk(10., 1)));
        CHECK(areas[3] == 0.);
        CHECK(areas[2] == Approx(shrunk(10., 2)));
        CHECK(areas[1] == Approx(shrunk(10., 3)));
        CHECK(areas[0] == 0.);
    }
    SECTION("a vanishing cone ends the walk") {
        contact.polygons = square(scene.support.storage, 0., 1.5);
        const std::vector<double> areas = scene.areas(step, 3);
        CHECK(areas[4] == Approx(shrunk(1.5, 1)));
        CHECK(areas[3] == 0.);
        CHECK(areas[2] == 0.);
    }
    SECTION("outside contact mode there is no cone") {
        for (const double area : scene.areas(step, 0))
            CHECK(area == 0.);
    }
}

TEST_CASE("Contact cones shrink upwards above a bottom contact", "[plugins][support-contact-xy]")
{
    Scene scene(0.2, 3); // slabs topping at 0.4, 0.6 and 0.8
    SC::GeneratorLayer &contact = SC::add_layer(scene.support, SC::LayerType::BottomContact, scale_to_layer_coord(0.2), 0,
                                                scale_to_layer_coord(0.2));
    contact.polygons = square(scene.support.storage, 0., 10.);
    scene.support.bottom.push_back(&contact);
    const std::vector<double> areas = scene.areas(scale_i(0.5), 2);
    CHECK(areas[0] == Approx(shrunk(10., 1)));
    CHECK(areas[1] == Approx(shrunk(10., 2)));
    CHECK(areas[2] == 0.);
}

TEST_CASE("Contact cones of neighbouring contacts are united", "[plugins][support-contact-xy]")
{
    Scene scene(0.8, 1);
    scene.columns[0] = square(scene.support.storage, 0., 30.);
    for (const double x : {0., 5.}) {
        SC::GeneratorLayer &contact = SC::add_layer(scene.support, SC::LayerType::TopContact, scale_to_layer_coord(1.2),
                                                    scale_to_layer_coord(1.), scale_to_layer_coord(0.2));
        contact.polygons = square(scene.support.storage, x, 10.);
        scene.support.top.push_back(&contact);
    }
    // Two 9-mm squares shifted by 5 mm overlap on a 4 x 9 mm band.
    const std::vector<double> areas = scene.areas(scale_i(0.5), 1);
    CHECK(areas[0] == Approx(2. * 81. - 36.));
}
