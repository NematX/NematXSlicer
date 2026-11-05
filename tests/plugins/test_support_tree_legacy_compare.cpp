#include "support_grid_test_helpers.hpp"
#include "support_tree_test_helpers.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Support/TreeSupport.hpp"
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

/* Loose comparison of the tree and organic families with the legacy tree generator, as a safety net. The families differ on
   purpose (exact contact distances, grid-like interfaces, support layers chosen from the branch slopes), so no value is
   expected to match. Each case measures the roof under every overhang, the extruded volume, the islands printed on the bed
   and the support printed inside the object, then checks them against bounds that allow for those differences. Where the
   legacy tree generation breaks, the case keeps the plugin-side checks only. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
using SG::measure_layer;
using SG::prepare_legacy_inputs;

namespace {
// Generate plugin support and legacy support where a reference exists, then check existing bounds; floor scenarios omit the comparison table.
// Prints own their generated layers; generation and report I/O failures remain visible to the test.
void compare(int model_id, const char *style, int interfaces, int floor_scenario = 0);

// Bounds set from the measured values in the table (SLIC3R_TREE_COMPARE_TABLE). With interface layers, the roofs of both
// families cover 94 to 100 percent of the demand and of each other: the contacts sit on different grids, so the bound leaves
// room for their edges. The share of the legacy roof that ours covers is bounded by it with and without interface layers
// (without them ours covers 99 percent of the legacy tips).
constexpr double roof_cover_min = 0.75;
// Without interface layers the roofs are the branches under the overhang: the legacy tips cover 16 to 19 percent of the demand,
// ours 11 to 30 percent (the tree sections widen down from the tips, and the organic meshes are tilted). Each side stays between
// a twentieth and two fifths of the demand, and the two sides agree within a factor of two.
constexpr double tip_cover_min = 0.05, tip_cover_max = 0.4;
constexpr double tip_ratio_min = 0.5, tip_ratio_max = 2.;
// Extruded volume (measured 0.85 to 1.13): the branch paths and sparse bases differ in spacing and sheath.
constexpr double volume_ratio_min = 0.5, volume_ratio_max = 2.;
// Islands on the bed (measured 0.8 to 1.25): the trees are spaced differently, so their counts agree to a factor of two.
constexpr double islands_ratio_max = 2.;
// Support inside the object: the clearance keeps it out, so only rounding can remain.
constexpr double inside_max_mm2 = 0.01;

// The three comparison objects (slab, T, shelf) are built by the shared helper, as the baseline dump uses them too.
using Slic3r::Test::SupportTree::comparison_mesh;

// The legacy tree generation breaks on the tree style for every model except the slab with interface layers: an assertion
// at TreeSupport.cpp:3090 on the T and the shelf, and no message on the slab without interface layers.
bool has_legacy_reference(int model_id, const char *style, int interfaces)
{
    return strcmp(style, "tree") != 0 || (model_id == 0 && interfaces > 0);
}

double mm2(const ExPolygons &polygons) { return unscaled(unscaled(area(polygons))); }

// The footprint of the layers that print within (low, high]. The legacy layers carry no typed surfaces, so both prints are
// measured by footprint; the branch sections below a roof lie inside its projection.
ExPolygons roof(const PrintObject &object, coord_t low, coord_t high)
{
    ExPolygons result;
    for (const Layer &layer : object.auxiliary_layers())
        if (layer.scaled_print_z() > low && layer.scaled_print_z() <= high)
            result = union_ex(result, layer.lslices());
    return result;
}

// What one print publishes: its layer count, the islands on the bed, the extruded volume, and the support footprint printed
// inside the object, which must stay empty.
struct Printed {
    size_t layers = 0, first_islands = 0;
    double volume = 0, inside = 0;
};

Printed summarize(const PrintObject &object)
{
    Printed result;
    result.layers = object.auxiliary_layers().size();
    ExPolygons first;
    for (const Layer &layer : object.auxiliary_layers()) {
        for (const auto &item : measure_layer(layer)) result.volume += item.second.volume;
        if (layer.scaled_bottom_z() <= 0) first = union_ex(first, layer.lslices());
        for (const Layer &object_layer : object.layers()) {
            const coord_t overlap = std::min(layer.scaled_print_z(), object_layer.scaled_print_z()) -
                std::max(layer.scaled_bottom_z(), object_layer.scaled_bottom_z());
            if (overlap > scale_to_layer_coord(0.01))
                result.inside += mm2(intersection_ex(layer.lslices(), object_layer.lslices()));
        }
    }
    result.first_islands = first.size();
    return result;
}

// One overhang: the bottom Z of the object layer where it starts, the area that needs a roof (mm²), and the roof area of each
// print under it, with the part shared by both roofs (zero when the legacy has no reference).
struct Band {
    coord_t z;
    double demand, plugin, legacy, shared;
};

// The table lets a human compare the measured numbers; nothing is written unless the variable is set.
void record(const std::string &line)
{
    const char *path = std::getenv("SLIC3R_TREE_COMPARE_TABLE");
    if (path == nullptr || path[0] == 0)
        return;
    std::ofstream out(path, std::ios::app);
    INFO("comparison table " << path);
    REQUIRE(out.is_open());
    out << line << "\n";
    out.flush();
    REQUIRE(out.good());
}

void compare(int model_id, const char *style, int interfaces, int floor_scenario)
{
    CAPTURE(model_id, style, interfaces);
    const TriangleMesh mesh = comparison_mesh(model_id);
    DynamicPrintConfig config = SG::config(style, floor_scenario == 2, interfaces, floor_scenario == 2);
    // Near-vertical branches land on the lower lobe instead of routing around it to the bed.
    if (floor_scenario != 0)
        config.set_deserialize_strict({{"support_tree_angle", "10"}, {"support_tree_angle_slow", "5"}});
    // A 0.1 mm interface limit splits body slabs inside the 0.2 mm bottom contact, giving modulation an interior body top.
    if (floor_scenario == 1)
        config.set_deserialize_strict({{"support_material_interface_layer_height", "0.1"}});
    if (floor_scenario == 2)
        config.set_deserialize_strict({{"support_material_bottom_interface_layers", "0"}});
    Print plugin_print, legacy_print;
    Model plugin_model, legacy_model;
    Test::init_print({mesh}, plugin_print, plugin_model, config);
    SG::prefix(plugin_print);
    SG::family(plugin_print);
    std::vector<SG::LayerSurfaceAreas> published;
    if (std::getenv("SLIC3R_SUPPORT_LAYER_TABLE") != nullptr)
        for (const Layer &layer : plugin_print.object(0).auxiliary_layers()) published.push_back(SG::layer_surface_areas(layer));
    std::map<std::pair<coord_t, coord_t>, ExPolygons> sparse_floors;
    if (floor_scenario == 1) {
        bool bottom_contact = false, bottom_interface = false, body_in_contact_window = false;
        const PrintObject &object = plugin_print.object(0);
        for (const Layer &layer : object.auxiliary_layers()) {
            const SG::LayerSurfaceAreas areas = SG::layer_surface_areas(layer);
            bottom_interface = bottom_interface || areas.bottom_interface > 0.;
            if (areas.bottom_contact <= 0.) continue;
            bottom_contact = true;
            // Match the open interval used by toolpath modulation; unknown sparse modifiers do not exclude branch bodies.
            for (const Layer &body : object.auxiliary_layers())
                if (body.scaled_print_z() > layer.scaled_bottom_z() && body.scaled_print_z() < layer.scaled_print_z())
                    body_in_contact_window = body_in_contact_window || SG::layer_surface_areas(body).sparse > 0.;
        }
        REQUIRE(bottom_contact);
        REQUIRE(bottom_interface);
        REQUIRE(body_in_contact_window);
    }
    if (floor_scenario == 2) {
        // The shelf's lower lobe ends at 3 mm; its exposed top excludes the post that continues upward.
        ExPolygons lobe_top;
        const PrintObject &object = plugin_print.object(0);
        for (const Layer &layer : object.layers())
            if (layer.scaled_print_z() == scale_to_layer_coord(3.) && layer.upper_layer != nullptr)
                lobe_top = diff_ex(layer.lslices(), layer.upper_layer->lslices());
        REQUIRE_FALSE(lobe_top.empty());
        for (const Layer &layer : object.auxiliary_layers()) {
            if (layer.scaled_bottom_z() != scale_to_layer_coord(3.)) continue;
            ExPolygons sparse_area;
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region : island.regions_islands())
                    for (const Surface &surface : region.fill_surfaces()) {
                        const SurfaceType sparse = stModSupport | stPosInternal | stDensSparse;
                        if ((surface.surface_type & sparse) == sparse && (surface.surface_type & stModContact) == 0)
                            sparse_area = union_ex(sparse_area, intersection_ex(ExPolygons{surface.expolygon}, lobe_top));
                    }
            if (mm2(sparse_area) > 0.) sparse_floors[{layer.scaled_print_z(), layer.scaled_height()}] = std::move(sparse_area);
        }
        REQUIRE_FALSE(sparse_floors.empty());
    }
    SG::toolpaths(plugin_print);
    const PrintObject &plugin_object = plugin_print.object(0);
    for (const auto &[floor, area] : sparse_floors) {
        bool support_paths = false;
        for (const Layer &layer : plugin_object.auxiliary_layers()) {
            if (layer.scaled_print_z() != floor.first || layer.scaled_height() != floor.second) continue;
            // Intersect path centre lines with the saved floor footprint, not unrelated support elsewhere on this layer.
            ExtrusionPaths paths;
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region : island.regions_islands())
                    if (region.has_extrusion(LayerRegionIsland::SUPPORT))
                        SG::legacy_paths(region.extrusion(LayerRegionIsland::SUPPORT), paths);
            for (const ExtrusionPath &path : paths)
                if (path.role() == ExtrusionRole::SupportMaterial)
                    for (const Polyline &inside : intersection_pl(path.polyline_ref().to_polyline(), area))
                        support_paths = support_paths || inside.length() > 0.;
        }
        REQUIRE(support_paths);
    }

    const PrintObject *legacy_object = nullptr;
    if (has_legacy_reference(model_id, style, interfaces)) {
        Test::init_print({mesh}, legacy_print, legacy_model, config);
        SG::prefix(legacy_print);
        REQUIRE(legacy_print.object(0).auxiliary_layers().empty());
        prepare_legacy_inputs(legacy_print.object(0));
        fff_tree_support_generate(legacy_print.object(0));
        legacy_object = &legacy_print.object(0);
    }

    // Overhangs start where an object layer covers more than the layer below it, outside the clearance of that layer.
    // The roof of an overhang lies one gap below it, then its interface layers, so the band covers both.
    const auto params = SG::params(plugin_print, plugin_object);
    const coord_t depth = scale_to_layer_coord(0.6 + 0.2 * interfaces);
    std::vector<Band> bands;
    for (size_t i = 1; i < plugin_object.layer_count(); ++i) {
        const Layer &layer = plugin_object.layer(i);
        const ExPolygons demand = diff_ex(layer.lslices(), offset_ex(plugin_object.layer(i - 1).lslices(), double(params.gap_xy)));
        if (mm2(demand) < 0.01)
            continue;
        const coord_t bottom = layer.scaled_bottom_z();
        const ExPolygons plugin_roof = intersection_ex(roof(plugin_object, bottom - depth, bottom), demand);
        const ExPolygons legacy_roof = legacy_object != nullptr ? intersection_ex(roof(*legacy_object, bottom - depth, bottom), demand) : ExPolygons{};
        bands.push_back(Band{bottom, mm2(demand), mm2(plugin_roof), mm2(legacy_roof), mm2(intersection_ex(plugin_roof, legacy_roof))});
    }
    REQUIRE_FALSE(bands.empty());

    const Printed plugin = summarize(plugin_object);
    REQUIRE(plugin.layers > 0);
    Printed legacy;
    if (legacy_object != nullptr) {
        legacy = summarize(*legacy_object);
        REQUIRE(legacy.layers > 0);
    }

    std::ostringstream line;
    line << std::fixed << std::setprecision(3) << "model=" << model_id << " style=" << style << " interfaces=" << interfaces
         << " plugin_layers=" << plugin.layers << " plugin_first_islands=" << plugin.first_islands << " plugin_volume_mm3=" << plugin.volume
         << " plugin_inside_mm2=" << plugin.inside;
    if (legacy_object != nullptr)
        line << " legacy_layers=" << legacy.layers << " legacy_first_islands=" << legacy.first_islands << " legacy_volume_mm3=" << legacy.volume
             << " legacy_inside_mm2=" << legacy.inside << " volume_ratio=" << plugin.volume / legacy.volume;
    else
        line << " legacy=none";
    for (const Band &band : bands) {
        line << " band_z=" << unscaled(double(band.z)) << " demand_mm2=" << band.demand << " plugin_roof_mm2=" << band.plugin
             << " plugin_demand=" << band.plugin / band.demand;
        if (legacy_object != nullptr)
            line << " legacy_roof_mm2=" << band.legacy << " legacy_demand=" << band.legacy / band.demand << " shared_mm2=" << band.shared
                 << " plugin_covers_legacy=" << band.shared / band.legacy << " legacy_covers_plugin=" << band.shared / band.plugin;
    }
    if (floor_scenario == 0) record(line.str());

    for (const Band &band : bands) {
        CAPTURE(band.z, band.demand, band.plugin, band.legacy, band.shared);
        REQUIRE(band.plugin > 0.);
        if (interfaces > 0) {
            CHECK(band.plugin / band.demand >= roof_cover_min);
        } else {
            CHECK(band.plugin / band.demand >= tip_cover_min);
            CHECK(band.plugin / band.demand <= tip_cover_max);
        }
        if (legacy_object == nullptr)
            continue;
        REQUIRE(band.legacy > 0.);
        CHECK(band.shared / band.legacy >= roof_cover_min);
        if (interfaces > 0) {
            CHECK(band.legacy / band.demand >= roof_cover_min);
            CHECK(band.shared / band.plugin >= roof_cover_min);
        } else {
            CHECK(band.legacy / band.demand >= tip_cover_min);
            CHECK(band.legacy / band.demand <= tip_cover_max);
            CHECK(band.plugin / band.legacy >= tip_ratio_min);
            CHECK(band.plugin / band.legacy <= tip_ratio_max);
        }
    }
    REQUIRE(plugin.first_islands > 0);
    CHECK(plugin.inside <= inside_max_mm2);
    if (std::getenv("SLIC3R_SUPPORT_LAYER_TABLE") != nullptr) {
        std::ostringstream section;
        section << "tree model=" << model_id << " style=" << style << " interfaces=" << interfaces
                << " scenario=" << floor_scenario << " object=0";
        // A missing native reference has no further checks, so its plugin report is complete here.
        if (legacy_object == nullptr)
            SG::record_layer_report(section.str() + " plugin", plugin_object, published, "");
    }
    if (legacy_object == nullptr)
        return;
    REQUIRE(legacy.first_islands > 0);
    REQUIRE(plugin.volume > 0.);
    REQUIRE(legacy.volume > 0.);
    CHECK(plugin.volume / legacy.volume >= volume_ratio_min);
    CHECK(plugin.volume / legacy.volume <= volume_ratio_max);
    CHECK(double(plugin.first_islands) / double(legacy.first_islands) <= islands_ratio_max);
    CHECK(double(legacy.first_islands) / double(plugin.first_islands) <= islands_ratio_max);
    CHECK(legacy.inside <= inside_max_mm2);
    if (std::getenv("SLIC3R_SUPPORT_LAYER_TABLE") != nullptr) {
        std::ostringstream section;
        section << "tree model=" << model_id << " style=" << style << " interfaces=" << interfaces
                << " scenario=" << floor_scenario << " object=0";
        SG::record_layer_report(section.str() + " plugin", plugin_object, published, "");
        SG::record_layer_report(section.str() + " native", *legacy_object,
            std::vector<SG::LayerSurfaceAreas>(legacy_object->auxiliary_layers().size()), "", false);
    }
}

void run_comparison(int model_id)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const char *style : {"organic", "tree"})
        for (const int interfaces : {0, 3}) {
            DYNAMIC_SECTION("style=" << style << " interfaces=" << interfaces) {
                compare(model_id, style, interfaces);
            }
        }
}
}

// Compare a roof over the bed for both styles; the legacy tree has no reference without interface layers.
TEST_CASE("Tree legacy comparison: overhanging slab", "[plugins][support-tree-compare][legacy-compare][slow]") { run_comparison(0); }
// Compare a T whose bar overhangs both sides of its stem; the legacy tree has no reference for its tree style.
TEST_CASE("Tree legacy comparison: T bridge", "[plugins][support-tree-compare][legacy-compare][slow]") { run_comparison(1); }
// Compare a shelf whose overhang lands partly on a lower lobe; the legacy tree has no reference for its tree style.
TEST_CASE("Tree legacy comparison: shelf over a lower lobe", "[plugins][support-tree-compare][legacy-compare][slow]") { run_comparison(2); }

// Compare organic floors above the shelf's lower lobe with separate interface tools.
TEST_CASE("Organic floor legacy comparison: bottom interfaces on a shelf", "[plugins][support-tree-floors][legacy-compare][slow]")
{
    compare(2, "organic", 3, 1);
}

// Compare organic floors merged into the base with one tool and no bottom interfaces.
TEST_CASE("Organic floor legacy comparison: same-tool shelf without bottom interfaces", "[plugins][support-tree-floors][legacy-compare][slow]")
{
    compare(2, "organic", 3, 2);
}
