#include <algorithm>

#include "support_grid_test_helpers.hpp"
#include "libslic3r/GCode/ToolOrdering.hpp"

/* Support layers and tool ordering.
   ToolOrdering asks support_layer_role() which support buckets a support layer
   uses, then adds the base and/or interface extruder to that layer's tools.
   These tests slice a post carrying a high roof and a low shelf: below the
   shelf, the roof support is still base while the shelf support is already
   interface, so some support layers hold both buckets. The object itself is
   printed with the interface extruder, so the base extruder can only reach
   those layers through the support role. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;

namespace {
// Slice the roof-and-shelf object and generate its grid support paths with
// the base on extruder 1 and the interface and object on extruder 2.
void prepare_two_level_support(Print &print, Model &model);

// Return the role expected from the support buckets that hold extrusions.
ExtrusionRole expected_support_role(const Layer &layer);

void prepare_two_level_support(Print &print, Model &model)
{
    // The shelf leaves the post on the side opposite to the roof, so the two
    // supported areas do not overlap but share the same support layers.
    TriangleMesh mesh = SG::slab();
    TriangleMesh shelf = make_cube(6., 8., 0.6);
    shelf.translate(-6., 0., 2.);
    mesh.merge(shelf);
    DynamicPrintConfig config = SG::config("grid", false, 3);
    config.set_deserialize_strict({{"perimeter_extruder", "2"}, {"infill_extruder", "2"},
        {"solid_infill_extruder", "2"}});
    Test::init_print({mesh}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    SG::toolpaths(print);
}

ExtrusionRole expected_support_role(const Layer &layer)
{
    bool has_base = false;
    bool has_interface = false;
    for (const LayerSliceIsland &island : layer.islands())
        for (const LayerRegionIsland &region : island.regions_islands()) {
            has_base = has_base || region.has_extrusion(LayerRegionIsland::SUPPORT);
            has_interface = has_interface || region.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE);
        }
    if (has_base && has_interface)
        return ExtrusionRole::Mixed;
    if (has_base)
        return ExtrusionRole::SupportMaterial;
    return has_interface ? ExtrusionRole::SupportMaterialInterface : ExtrusionRole::None;
}
}

// Every kind of support layer maps to its own role; a layer with both buckets
// must be Mixed, and an object layer without support buckets is None.
TEST_CASE("Support layer role distinguishes base interface and mixed layers", "[plugins][tool-ordering]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_two_level_support(print, model);
    size_t base_layers = 0, interface_layers = 0, mixed_layers = 0;
    for (const Layer &layer : print.object(0).auxiliary_layers()) {
        if (layer.get_property<LayerSupportProperty>() == nullptr || !layer.has_extrusions())
            continue;
        const ExtrusionRole expected = expected_support_role(layer);
        CAPTURE(unscaled(layer.scaled_print_z()));
        CHECK(support_layer_role(layer) == expected);
        base_layers += expected == ExtrusionRole::SupportMaterial;
        interface_layers += expected == ExtrusionRole::SupportMaterialInterface;
        mixed_layers += expected == ExtrusionRole::Mixed;
    }
    CHECK(base_layers > 0);
    CHECK(interface_layers > 0);
    REQUIRE(mixed_layers > 0);
    REQUIRE(print.object(0).layer_count() > 0);
    CHECK(support_layer_role(print.object(0).layer(0)) == ExtrusionRole::None);
}

// On a layer holding both support buckets, the tool list must contain the base
// extruder (index 0) and the interface extruder (index 1); LayerTools stores
// zero-based extruder indices once the tools are ordered.
TEST_CASE("Tool ordering keeps both support extruders on mixed support layers", "[plugins][tool-ordering]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_two_level_support(print, model);
    const ToolOrdering ordering(print.object(0), EXTRUDER_ID_UNKNOWN);
    size_t mixed_layers = 0;
    for (const Layer &layer : print.object(0).auxiliary_layers()) {
        if (layer.get_property<LayerSupportProperty>() == nullptr || !layer.has_extrusions() ||
            expected_support_role(layer) != ExtrusionRole::Mixed)
            continue;
        ++mixed_layers;
        CAPTURE(unscaled(layer.scaled_print_z()));
        const LayerTools *tools = ordering.tools_for_layer(layer.scaled_print_z());
        REQUIRE(tools != nullptr);
        CHECK(tools->has_support);
        CHECK(std::count(tools->extruders.begin(), tools->extruders.end(), extruder_index_t(0)) == 1);
        CHECK(std::count(tools->extruders.begin(), tools->extruders.end(), extruder_index_t(1)) == 1);
    }
    REQUIRE(mixed_layers > 0);
}
