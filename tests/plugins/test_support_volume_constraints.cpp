/* No-support zones and support columns in the built-in support families.

   A no-support zone is avoided by the support like the object: every family adds the zone to the object outline of its
   clearance, and publication removes the zone from every slab once more. A support column becomes sparse support over its own
   height, whether or not a surface asks for support. These cases slice the overhanging slab of support_grid_test_helpers.hpp
   (zones) or a plain cube (columns), add the helper volumes to its object and run the production support step, then read the
   published support surfaces of every auxiliary layer.

       zone or column case
       |-- zone_print() / column_print(): scene, helper volumes, SG::prefix() and SG::family()
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

// Slices a plain 10 x 10 x 5 mm cube, which asks for no support, with the given columns of the given type, the given sparse support
// columns and the given zones, and runs the support family of the style; support_material off keeps the columns but disables the
// support. overrides holds setting keys and values in turn.
void column_print(Print &print, Model &model, const char *style, const std::vector<TriangleMesh> &columns, bool support = true,
                  ModelVolumeType type = ModelVolumeType::SUPPORT_COLUMN, const std::vector<TriangleMesh> &sparse_columns = {},
                  const std::vector<TriangleMesh> &zones = {}, const t_config_option_keys &overrides = {})
{
    const bool filled = std::string(style) == "filled";
    DynamicPrintConfig config = SG::config(style, filled, filled ? 0 : 2);
    config.set_deserialize_strict({{"support_material", support ? "1" : "0"}});
    for (size_t idx = 0; idx + 1 < overrides.size(); idx += 2)
        config.set_deserialize_strict(overrides[idx], overrides[idx + 1]);
    Test::init_print({make_cube(10., 10., 5.)}, print, model, config);
    for (ModelObject &model_object : model.objects())
        for (const TriangleMesh &column : columns)
            model_object.add_volume(TriangleMesh(column), type, false);
    for (ModelObject &model_object : model.objects())
        for (const TriangleMesh &column : sparse_columns)
            model_object.add_volume(TriangleMesh(column), ModelVolumeType::SUPPORT_COLUMN, false);
    for (ModelObject &model_object : model.objects())
        for (const TriangleMesh &zone : zones)
            model_object.add_volume(TriangleMesh(zone), ModelVolumeType::NO_SUPPORT_ZONE, false);
    print.apply(model, config);
    print.validate();
    print.set_status_silent();
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

// A column beside a cube that asks for no support becomes sparse support over its own height only: its 2 x 2 mm section on every
// layer between z = 1 and 4, nothing below it (the column is not extended to the bed) and nothing above it.
TEST_CASE("A support column alone becomes support over its own height in each built-in family", "[plugins][support-volumes][support-column]")
{
    const char *style = GENERATE("grid", "snug", "tree", "organic", "filled");
    CAPTURE(style);
    Print print;
    Model model;
    column_print(print, model, style, {zone_box(14., 2., 1., 2., 2., 3.)});
    const PrintObject &object = print.object(0);
    const std::vector<ExPolygons> columns = object.slice_support_volumes(ModelVolumeType::SUPPORT_COLUMN);
    REQUIRE(columns.size() > 10);
    const ExPolygons footprint = columns[10];
    REQUIRE(area_mm2(footprint) == Approx(4.).margin(0.05));
    size_t carrying = 0;
    for (const Layer *layer : auxiliary_layers(object)) {
        const double middle = layer->unscaled_print_z() - 0.5 * layer->unscaled_height();
        const double area = area_mm2(support_surfaces(*layer));
        CAPTURE(layer->unscaled_print_z(), layer->unscaled_height(), area);
        if (middle < 1. || middle > 4.) {
            CHECK(area < 1e-6);
        } else {
            // All the support of the layer is the column section.
            CHECK(area == Approx(4.).margin(0.1));
            CHECK(area_mm2(intersection_ex(support_surfaces(*layer), footprint)) == Approx(area).margin(1e-3));
            ++carrying;
        }
    }
    // Three millimeters of 0.25 mm column layers: the default support layer height is the maximum, 0.3 mm, kept 0.05 mm below it.
    CHECK(carrying >= 12);
}

// The column keeps the object clearance: a column through the cube prints only outside the cube grown by the XY gap.
TEST_CASE("A support column keeps clear of the object", "[plugins][support-volumes][support-column]")
{
    const char *style = GENERATE("grid", "tree", "filled");
    CAPTURE(style);
    Print print;
    Model model;
    // x = 8 to 14 crosses the cube's side at x = 10.
    column_print(print, model, style, {zone_box(8., 2., 1., 6., 2., 3.)});
    const PrintObject &object = print.object(0);
    double printed = 0;
    for (const Layer *layer : auxiliary_layers(object)) {
        const ExPolygons support = support_surfaces(*layer);
        printed += area_mm2(support);
        for (size_t idx = 0; idx < object.layer_count(); ++idx) {
            const Layer &object_layer = object.layer(idx);
            if (object_layer.scaled_print_z() > layer->scaled_print_z() - layer->scaled_height() &&
                object_layer.scaled_print_z() - object_layer.scaled_height() < layer->scaled_print_z())
                CHECK(area_mm2(intersection_ex(support, object_layer.lslices())) < 1e-4);
        }
    }
    // The part outside the cube, 4 x 2 mm less the XY gap, is printed.
    CHECK(printed > 1.);
}

// A support interface column prints its section as top interface over its own height, and where it overlaps a sparse column the
// area is printed once, by the interface: the sparse support keeps only the rest of its own section.
TEST_CASE("A support interface column becomes interface support and wins over a sparse column", "[plugins][support-volumes][support-column]")
{
    const char *style = GENERATE("grid", "snug", "tree", "organic");
    CAPTURE(style);
    Print print;
    Model model;
    // Interface column x = 14 to 16; sparse column x = 15 to 17, sharing x = 15 to 16 with it.
    column_print(print, model, style, {zone_box(14., 2., 1., 2., 2., 3.)}, true, ModelVolumeType::SUPPORT_INTERFACE_COLUMN,
                 {zone_box(15., 2., 1., 2., 2., 3.)});
    size_t carrying = 0;
    for (const Layer *layer : auxiliary_layers(print.object(0))) {
        const double middle = layer->unscaled_print_z() - 0.5 * layer->unscaled_height();
        if (middle < 1.2 || middle > 3.8)
            continue;
        const SG::LayerSurfaceAreas areas = SG::layer_surface_areas(*layer);
        CAPTURE(layer->unscaled_print_z(), areas.top_interface, areas.top_contact, areas.sparse);
        CHECK(areas.top_interface + areas.top_contact == Approx(4.).margin(0.1));
        CHECK(areas.sparse == Approx(2.).margin(0.1));
        ++carrying;
    }
    CHECK(carrying >= 10);
}

// On the bed layer a column is widened like the support foot, by raft_first_layer_expansion; the layers above keep its section.
TEST_CASE("A support column standing on the bed gets the widened support foot", "[plugins][support-volumes][support-column]")
{
    const char *style = GENERATE("grid", "tree");
    CAPTURE(style);
    Print print;
    Model model;
    column_print(print, model, style, {zone_box(16., 2., 0., 2., 2., 3.)}, true, ModelVolumeType::SUPPORT_COLUMN, {}, {},
                 {"raft_first_layer_expansion", "2"});
    const std::vector<const Layer *> layers = auxiliary_layers(print.object(0));
    REQUIRE(layers.size() > 2);
    // A 2 x 2 mm section grown by 2 mm on every side covers about 6 x 6 mm less its rounded corners.
    CHECK(area_mm2(support_surfaces(*layers.front())) > 25.);
    CHECK(area_mm2(support_surfaces(*layers[2])) == Approx(4.).margin(0.1));
}

// A column that the clearance removes entirely, inside the object or inside a no-support zone, prints nothing at all.
TEST_CASE("A support column inside the object or a no-support zone prints nothing", "[plugins][support-volumes][support-column]")
{
    const char *style = GENERATE("grid", "tree", "filled");
    CAPTURE(style);
    {
        Print print;
        Model model;
        column_print(print, model, style, {zone_box(4., 4., 1., 2., 2., 3.)});
        CHECK(support_area(print.object(0)) < 1e-6);
    }
    {
        Print print;
        Model model;
        column_print(print, model, style, {zone_box(14., 2., 1., 2., 2., 3.)}, true, ModelVolumeType::SUPPORT_COLUMN, {},
                     {zone_box(13., 1., 0., 4., 4., 5.)});
        CHECK(support_area(print.object(0)) < 1e-6);
    }
}

// A column that rises into the slab's roof (bottom at z = 4) ends one contact distance below it, like the top contact of the
// automatic support, and not on the object layer grid: 0.15 mm under 0.2 mm layers puts its top at z = 3.85. Automatic support is
// off, so all the support under the roof is the column.
TEST_CASE("A support column ends one contact distance under the object above it", "[plugins][support-volumes][support-column]")
{
    const char *style = GENERATE("grid", "tree");
    CAPTURE(style);
    DynamicPrintConfig config = SG::config(style, false, 0);
    config.set_deserialize_strict({{"support_material_auto", "0"}, {"support_material_contact_distance", "0.15"}});
    Print print;
    Model model;
    Test::init_print({SG::slab(false)}, print, model, config);
    for (ModelObject &model_object : model.objects())
        model_object.add_volume(zone_box(6., 2., 0., 2., 2., 4.5), ModelVolumeType::SUPPORT_COLUMN, false);
    print.apply(model, config);
    print.validate();
    print.set_status_silent();
    SG::prefix(print);
    SG::family(print);
    const PrintObject &object = print.object(0);
    const std::vector<ExPolygons> columns = object.slice_support_volumes(ModelVolumeType::SUPPORT_COLUMN);
    REQUIRE(columns.size() > 10);
    const ExPolygons footprint = columns[10];
    double top = 0;
    for (const Layer *layer : auxiliary_layers(object))
        if (area_mm2(intersection_ex(support_surfaces(*layer), footprint)) > 1.)
            top = std::max(top, layer->unscaled_print_z());
    CHECK(top == Approx(3.85).margin(1e-6));
}

// A support enforcer volume around the slab's overhang, or support enforcer painting on the roof's underside (facets 12 and 13,
// after the post's 12), with automatic support off, asks for support on the same surface as the automatic demand: the roof's
// underside. The support layers must then be the same, whatever the support layer height.
TEST_CASE("Support enforcers over an overhang give the support layers of the automatic demand", "[plugins][support-volumes][support-demand]")
{
    const char *style = GENERATE("grid", "tree");
    const char *height = GENERATE("0", "0.3", "0.22", "0.13");
    CAPTURE(style, height);
    enum class Demand { automatic, volume, painting };
    const auto layer_list = [&](Demand demand) {
        DynamicPrintConfig config = SG::config(style, false, 2);
        config.set_deserialize_strict({{"support_material_layer_height", height},
                                       {"support_material_auto", demand == Demand::automatic ? "1" : "0"}});
        Print print;
        Model model;
        Test::init_print({SG::slab(false)}, print, model, config);
        if (demand != Demand::automatic) {
            for (ModelObject &model_object : model.objects()) {
                if (demand == Demand::volume) {
                    model_object.add_volume(zone_box(3., -1., 0., 10., 10., 6.), ModelVolumeType::SUPPORT_ENFORCER, false);
                } else {
                    // "4" is the encoded enforcer state of a painted facet. reset() renews the painting's timestamp, so that
                    // Print::apply() copies it into the print, already applied once.
                    ModelVolume &part = *model_object.volumes.front();
                    part.supported_facets.reset();
                    part.supported_facets.reserve(2);
                    for (int facet : {12, 13})
                        part.supported_facets.set_triangle_from_string(facet, "4");
                    part.supported_facets.shrink_to_fit();
                }
            }
            print.apply(model, config);
            print.validate();
            print.set_status_silent();
        }
        SG::prefix(print);
        SG::family(print);
        // "print_z/height" in mm, readable in a failure report.
        std::vector<std::string> result;
        for (const Layer *layer : auxiliary_layers(print.object(0)))
            result.push_back(std::to_string(layer->unscaled_print_z()) + "/" + std::to_string(layer->unscaled_height()));
        return result;
    };
    const std::vector<std::string> automatic = layer_list(Demand::automatic);
    REQUIRE_FALSE(automatic.empty());
    CHECK(layer_list(Demand::volume) == automatic);
    CHECK(layer_list(Demand::painting) == automatic);
}

// The heights of the column layers that carry the given footprint, from the bottom.
std::vector<double> column_heights(const PrintObject &object, const ExPolygons &footprint)
{
    std::vector<double> heights;
    for (const Layer *layer : auxiliary_layers(object))
        if (area_mm2(intersection_ex(support_surfaces(*layer), footprint)) > 1.)
            heights.push_back(layer->unscaled_height());
    return heights;
}

// A column alone, from z = 1 to 6 beside a 5 mm cube of 0.2 mm layers, follows the support layer height, above the cube too:
// 0.25 mm asked, or 0 for the maximum (0.3 mm) kept one minimum height (0.05 mm) below it. Synchronized support layers follow the
// object layers instead, and their top height above the object.
TEST_CASE("Support columns alone follow the support layer height unless the layers are synchronized", "[plugins][support-volumes][support-column]")
{
    const char *height = GENERATE("0.25", "0");
    const bool synchronized = GENERATE(false, true);
    CAPTURE(height, synchronized);
    Print print;
    Model model;
    column_print(print, model, "grid", {zone_box(14., 2., 1., 2., 2., 5.)}, true, ModelVolumeType::SUPPORT_COLUMN, {}, {},
                 {"support_material_layer_height", height, "support_material_synchronize_layers", synchronized ? "1" : "0"});
    const PrintObject &object = print.object(0);
    const std::vector<ExPolygons> columns = object.slice_support_volumes(ModelVolumeType::SUPPORT_COLUMN);
    REQUIRE(columns.size() > 10);
    const std::vector<double> heights = column_heights(object, columns[10]);
    CHECK(heights.size() == (synchronized ? 25 : 20));
    for (double layer_height : heights)
        CHECK(layer_height == Approx(synchronized ? 0.2 : 0.25).margin(1e-6));
}

// Under the slab's roof (bottom at z = 4) with a 0.18 mm contact distance, the column top lies at 3.82 mm. On 0.2 mm column
// layers the last 0.02 mm cannot be printed alone: the 3.6 to 3.8 mm layer gives up the column area under the roof, and one
// 0.22 mm cap prints it up to 3.82 mm. No column layer is thinner than the minimum height.
TEST_CASE("A support column merges a rest too thin to print into one thicker top layer", "[plugins][support-volumes][support-column]")
{
    DynamicPrintConfig config = SG::config("grid", false, 0);
    config.set_deserialize_strict({{"support_material_auto", "0"}, {"support_material_contact_distance", "0.18"},
                                   {"support_material_layer_height", "0.2"}});
    Print print;
    Model model;
    Test::init_print({SG::slab(false)}, print, model, config);
    for (ModelObject &model_object : model.objects())
        model_object.add_volume(zone_box(6., 2., 0., 2., 2., 4.5), ModelVolumeType::SUPPORT_COLUMN, false);
    print.apply(model, config);
    print.validate();
    print.set_status_silent();
    SG::prefix(print);
    SG::family(print);
    const PrintObject &object = print.object(0);
    const std::vector<ExPolygons> columns = object.slice_support_volumes(ModelVolumeType::SUPPORT_COLUMN);
    REQUIRE(columns.size() > 10);
    const ExPolygons footprint = columns[10];
    const Layer *top = nullptr;
    for (const Layer *layer : auxiliary_layers(object))
        if (area_mm2(intersection_ex(support_surfaces(*layer), footprint)) > 1.) {
            CHECK(layer->unscaled_height() >= 0.05 - 1e-6);
            if (top == nullptr || layer->scaled_print_z() > top->scaled_print_z())
                top = layer;
        }
    REQUIRE(top != nullptr);
    CHECK(top->unscaled_print_z() == Approx(3.82).margin(1e-6));
    CHECK(top->unscaled_height() == Approx(0.22).margin(1e-6));
}

// Without support_material the columns are kept in the project but print nothing.
TEST_CASE("Support columns print nothing when support is disabled", "[plugins][support-volumes][support-column]")
{
    Print print;
    Model model;
    column_print(print, model, "grid", {zone_box(14., 2., 1., 2., 2., 3.)}, false);
    CHECK(print.object(0).auxiliary_layers().empty());
}
