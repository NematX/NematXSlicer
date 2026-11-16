/* No-support zones in the built-in support families.

   A no-support zone is avoided by the support like the object: every family adds the zone to the object outline of its
   clearance, and publication removes the zone from every slab once more. These cases slice the overhanging slab of
   support_grid_test_helpers.hpp, add zone volumes to its object and run the production support step, then read the
   published support surfaces of every auxiliary layer.

       zone case
       |-- zone_print(): slab scene, zone volumes, SG::prefix() and SG::family()
       |-- zones_under(): native zone slices of the object layers that overlap an auxiliary layer
       |-- support_area() / zone_overlap(): published support surfaces, alone and inside the zones
       `-- SG::layer_surface_areas(): bottom contacts and interfaces, which must not appear on a zone

   The zone slices of the oracle come from the native PrintObject::slice_support_volumes(), not from the plugin helper
   under test. */
#include "support_grid_test_helpers.hpp"

#include <string>
#include <vector>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"

namespace {
using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;

// Whether the style is routed by the tree family.
bool is_tree_style(const std::string &style)
{
    return style == "tree" || style == "organic";
}

// A box with its minimum corner at (x, y, z), in the coordinates of the slab mesh.
TriangleMesh zone_box(double x, double y, double z, double sx, double sy, double sz)
{
    TriangleMesh box = make_cube(sx, sy, sz);
    box.translate(x, y, z);
    return box;
}

// Slices the slab with the given zone volumes and runs the support family of the style; the print keeps the published layers.
void zone_print(Print &print, Model &model, const char *style, const std::vector<TriangleMesh> &zones)
{
    const bool filled = std::string(style) == "filled";
    DynamicPrintConfig config = SG::config(style, filled, filled ? 0 : 2);
    Test::init_print({SG::slab(false)}, print, model, config);
    if (!zones.empty()) {
        for (ModelObject &model_object : model.objects())
            for (const TriangleMesh &zone : zones)
                model_object.add_volume(TriangleMesh(zone), ModelVolumeType::NO_SUPPORT_ZONE, false);
        print.apply(model, config);
        print.validate();
        print.set_status_silent();
    }
    SG::prefix(print);
    SG::family(print);
}

// The union of the native zone slices of the object layers whose Z span overlaps the printable interval of the layer.
ExPolygons zones_under(const PrintObject &object, const std::vector<ExPolygons> &zone_slices, const Layer &layer)
{
    ExPolygons zones;
    const coord_t top = layer.scaled_print_z();
    const coord_t lower = top - layer.scaled_height();
    for (size_t idx = 0; idx < object.layer_count() && idx < zone_slices.size(); ++idx) {
        const Layer &object_layer = object.layer(idx);
        if (object_layer.scaled_print_z() > lower && object_layer.scaled_print_z() - object_layer.scaled_height() < top)
            append(zones, zone_slices[idx]);
    }
    return union_ex(zones);
}

// The published support surfaces of a layer, every role together.
ExPolygons support_surfaces(const Layer &layer)
{
    ExPolygons surfaces;
    for (const LayerSliceIsland &island : layer.islands())
        for (const LayerRegionIsland &region : island.regions_islands())
            for (const Surface &surface : region.fill_surfaces())
                if ((surface.surface_type & stModSupport) != 0)
                    surfaces.push_back(surface.expolygon);
    return surfaces;
}

// The area of the expolygons, in mm2.
double area_mm2(const ExPolygons &expolygons)
{
    double area = 0;
    for (const ExPolygon &expolygon : expolygons)
        area += unscaled(unscaled(expolygon.area()));
    return area;
}

// The auxiliary layers of the object, in their order.
std::vector<const Layer *> auxiliary_layers(const PrintObject &object)
{
    std::vector<const Layer *> layers;
    for (const Layer &layer : object.auxiliary_layers())
        layers.push_back(&layer);
    return layers;
}

// The published support area of every auxiliary layer, in mm2.
double support_area(const PrintObject &object)
{
    double area = 0;
    for (const Layer &layer : object.auxiliary_layers())
        area += area_mm2(support_surfaces(layer));
    return area;
}

// The published support area inside the native zone slices, summed over the auxiliary layers, in mm2.
double zone_overlap(const PrintObject &object)
{
    const std::vector<ExPolygons> zone_slices = object.slice_support_volumes(ModelVolumeType::NO_SUPPORT_ZONE);
    double area = 0;
    for (const Layer &layer : object.auxiliary_layers()) {
        const ExPolygons zones = zones_under(object, zone_slices, layer);
        if (!zones.empty())
            area += area_mm2(intersection_ex(support_surfaces(layer), zones));
    }
    return area;
}
} // namespace

// The slab's overhang spans x = 4 to 12. A zone across the middle of the overhang, through the roof, removes the support under it:
// contacts, interfaces and sparse areas alike, while the support on both sides of the zone remains.
TEST_CASE("No-support zones remove every support role in each built-in family", "[plugins][support-volumes][no-support-zone]")
{
    const char *style = GENERATE("grid", "snug", "tree", "organic", "filled");
    CAPTURE(style);
    Print reference_print;
    Model reference_model;
    zone_print(reference_print, reference_model, style, {});
    const double reference = support_area(reference_print.object(0));
    REQUIRE(reference > 1.);

    Print print;
    Model model;
    zone_print(print, model, style, {zone_box(7., -1., 0., 2., 10., 6.)});
    const PrintObject &object = print.object(0);
    const double area = support_area(object);
    const double overlap = zone_overlap(object);
    CAPTURE(reference, area, overlap);
    CHECK(overlap < 1e-4);
    // Grid, snug and filled support only lose the zone; trees route around it, which may add branch area elsewhere.
    if (!is_tree_style(style))
        CHECK(area < reference);
    // The zone covers a quarter of the overhang width at most; the support beside it stays.
    CHECK(area > 0.25 * reference);
}

// Support never rests on a zone: a zone halfway between the bed and the overhang creates no bottom contact or interface on its top,
// in any family; the slab scene's support otherwise stands on the bed and has none either.
TEST_CASE("Support does not rest on a no-support zone", "[plugins][support-volumes][no-support-zone]")
{
    const char *style = GENERATE("grid", "snug", "tree", "organic", "filled");
    CAPTURE(style);
    Print print;
    Model model;
    zone_print(print, model, style, {zone_box(6., -1., 1., 4., 10., 1.)});
    double bottom = 0;
    for (const Layer &layer : print.object(0).auxiliary_layers()) {
        const SG::LayerSurfaceAreas areas = SG::layer_surface_areas(layer);
        bottom += areas.bottom_contact + areas.bottom_interface;
    }
    CHECK(bottom == Approx(0.).margin(1e-6));
    CHECK(zone_overlap(print.object(0)) < 1e-4);
}

// A zone away from the support keeps the support of grid, snug and filled support unchanged: the clearance and publication paths that
// a zone activates leave the slabs it does not reach as they are. Trees route on collision areas simplified after their union with the
// zones, so a distant zone may move the branches slightly; their support area stays within a few percent.
TEST_CASE("A distant no-support zone leaves the support unchanged", "[plugins][support-volumes][no-support-zone]")
{
    const char *style = GENERATE("grid", "snug", "tree", "organic", "filled");
    CAPTURE(style);
    Print reference_print;
    Model reference_model;
    zone_print(reference_print, reference_model, style, {});
    Print print;
    Model model;
    zone_print(print, model, style, {zone_box(30., 30., 0., 2., 2., 6.)});
    if (is_tree_style(style)) {
        const double reference = support_area(reference_print.object(0));
        CHECK(support_area(print.object(0)) == Approx(reference).epsilon(0.03));
        return;
    }
    const std::vector<const Layer *> expected_layers = auxiliary_layers(reference_print.object(0));
    const std::vector<const Layer *> layers = auxiliary_layers(print.object(0));
    REQUIRE(layers.size() == expected_layers.size());
    for (size_t idx = 0; idx < layers.size(); ++idx) {
        CAPTURE(idx, layers[idx]->unscaled_print_z());
        CHECK(layers[idx]->scaled_print_z() == expected_layers[idx]->scaled_print_z());
        const ExPolygons actual = support_surfaces(*layers[idx]);
        const ExPolygons wanted = support_surfaces(*expected_layers[idx]);
        const double difference = area_mm2(diff_ex(actual, wanted)) + area_mm2(diff_ex(wanted, actual));
        CHECK(difference < 1e-6);
    }
}
