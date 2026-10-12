#include "support_grid_test_helpers.hpp"
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

// Three simple objects without raft: an overhanging slab above the bed, a T whose bar overhangs both sides of its stem, and a
// shelf whose overhang lands partly on a lower lobe of the object, where branches stand on the object.
TriangleMesh comparison_mesh(int model_id)
{
    if (model_id == 0) return SG::slab();
    if (model_id == 1) {
        TriangleMesh stem = make_cube(4., 4., 6.);
        stem.translate(6., 2., 0.);
        TriangleMesh bar = make_cube(16., 4., 2.);
        bar.translate(0., 2., 6.);
        stem.merge(bar);
        return stem;
    }
    TriangleMesh post = make_cube(4., 8., 8.);
    TriangleMesh shelf = make_cube(20., 8., 1.);
    shelf.translate(0., 0., 8.);
    TriangleMesh lobe = make_cube(6., 8., 3.);
    lobe.translate(10., 0., 0.);
    post.merge(shelf);
    post.merge(lobe);
    return post;
}

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

// Generate the plugin support of one object and, where it has a reference, the legacy support, then check them.
void compare(int model_id, const char *style, int interfaces)
{
    CAPTURE(model_id, style, interfaces);
    const TriangleMesh mesh = comparison_mesh(model_id);
    DynamicPrintConfig config = SG::config(style, false, interfaces, false);
    Print plugin_print, legacy_print;
    Model plugin_model, legacy_model;
    Test::init_print({mesh}, plugin_print, plugin_model, config);
    SG::prefix(plugin_print);
    SG::family(plugin_print);
    SG::toolpaths(plugin_print);
    const PrintObject &plugin_object = plugin_print.object(0);

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
    record(line.str());

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
