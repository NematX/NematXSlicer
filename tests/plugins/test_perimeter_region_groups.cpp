#include <catch2/catch.hpp>

#include "perimeter_test_helpers.hpp"

#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionProperty.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

/*
These tests check how the perimeter generators group the regions of one layer
island. Regions that disagree on a perimeter setting must not share a perimeter
tree: the generator gives each setting value its own region group, the host
publishes one LayerRegionIsland per group, and each group uses the extruder and
perimeter count of its own regions. Regions that agree stay one group, whose
output equals the output of the same island with a single region.

The island is a 20 x 20 mm square; one modifier region covers its right half,
or a central band that cuts the default region into two pieces of one group.

    TEST_CASE
    |-- run_perimeter_region_islands_case(): one capture per published region island
    |-- group_summaries(): extruder, region count, x extent and deepest shell per group
    `-- printed_leaves(): exact points and flow of a perimeter tree
*/

namespace {
using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

// What a test needs to know about one region group, independent of publication order.
struct GroupSummary
{
    extruder_index_t extruder_id = EXTRUDER_ID_UNKNOWN;
    size_t region_count = 0;
    coord_t min_x = std::numeric_limits<coord_t>::max();
    coord_t max_x = std::numeric_limits<coord_t>::lowest();
    int16_t max_perimeter_idx = -1;

    bool operator==(const GroupSummary &rhs) const
    {
        return extruder_id == rhs.extruder_id && region_count == rhs.region_count && min_x == rhs.min_x &&
               max_x == rhs.max_x && max_perimeter_idx == rhs.max_perimeter_idx;
    }
};

// One printable leaf of a perimeter tree, compared exactly.
struct PrintedLeaf
{
    Points points;
    float width = 0.f;
    double mm3_per_mm = 0.;

    bool operator==(const PrintedLeaf &rhs) const
    {
        return points == rhs.points && width == rhs.width && mm3_per_mm == rhs.mm3_per_mm;
    }
};

// The generators that split islands into region groups, with the Python simple one when Python plugins are built.
std::vector<const char *> generators();
// Append the printable leaves of entity in tree order.
void collect_printed_leaves(const ExtrusionEntity &entity, std::vector<PrintedLeaf> &out);
// Summarize each captured group, sorted from left to right.
std::vector<GroupSummary> group_summaries(const std::vector<PerimeterRegionIslandCapture> &captures);
// Return the printable leaves of every captured group, in capture order.
std::vector<PrintedLeaf> printed_leaves(const std::vector<PerimeterRegionIslandCapture> &captures);
// Run generator on the square island, with one override region on its right half when settings is not empty.
std::vector<PerimeterRegionIslandCapture> run_half_override(const char *generator, const DynamicPrintConfig &config,
    const std::vector<std::pair<std::string, std::string>> &settings);

std::vector<const char *> generators()
{
    std::vector<const char *> out{CLASSIC_PERIMETER_GENERATOR, SIMPLE_PERIMETER_GENERATOR, ARACHNE_PERIMETER_GENERATOR};
#ifdef SLIC3R_TEST_PYTHON_PLUGINS
    out.push_back(PYTHON_SIMPLE_PERIMETER_GENERATOR);
#endif
    return out;
}

void collect_printed_leaves(const ExtrusionEntity &entity, std::vector<PrintedLeaf> &out)
{
    if (entity.is_leaf()) {
        if (!entity.has_polyline())
            return;
        PrintedLeaf leaf;
        entity.collect_points(leaf.points);
        if (const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>()) {
            leaf.width = attributes->width;
            leaf.mm3_per_mm = attributes->mm3_per_mm;
        }
        out.push_back(std::move(leaf));
        return;
    }
    for (size_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        collect_printed_leaves(entity.child(child_idx), out);
}

std::vector<GroupSummary> group_summaries(const std::vector<PerimeterRegionIslandCapture> &captures)
{
    std::vector<GroupSummary> out;
    for (const PerimeterRegionIslandCapture &capture : captures) {
        GroupSummary summary;
        summary.extruder_id = capture.extruder_id;
        summary.region_count = capture.region_count;
        std::vector<PrintedLeaf> leaves;
        collect_printed_leaves(capture.perimeters, leaves);
        for (const PrintedLeaf &leaf : leaves)
            for (const Point &point : leaf.points) {
                summary.min_x = std::min(summary.min_x, point.x());
                summary.max_x = std::max(summary.max_x, point.x());
            }
        for (const PerimeterLoopPayload &loop : perimeter_loop_payloads(capture.perimeters))
            summary.max_perimeter_idx = std::max(summary.max_perimeter_idx, loop.perimeter_idx);
        out.push_back(summary);
    }
    std::sort(out.begin(), out.end(), [](const GroupSummary &a, const GroupSummary &b) { return a.min_x < b.min_x; });
    return out;
}

std::vector<PrintedLeaf> printed_leaves(const std::vector<PerimeterRegionIslandCapture> &captures)
{
    std::vector<PrintedLeaf> out;
    for (const PerimeterRegionIslandCapture &capture : captures)
        collect_printed_leaves(capture.perimeters, out);
    return out;
}

std::vector<PerimeterRegionIslandCapture> run_half_override(const char *generator, const DynamicPrintConfig &config,
    const std::vector<std::pair<std::string, std::string>> &settings)
{
    std::vector<PerimeterRegionOverride> overrides;
    if (!settings.empty())
        overrides.push_back({rectangle_expolygon(0., -10., 10., 10.), settings});
    return run_perimeter_region_islands_case(config, {generator}, rectangle_expolygon(-10., -10., 10., 10.), 1,
                                             overrides);
}

} // namespace

TEST_CASE("Perimeter generators give each region group its own settings",
          "[plugins][perimeter][perimeter-region-groups]")
{
    for (const char *generator : generators()) {
        INFO(generator);

        // The extruder is chosen per region group, so each half must get its
        // own group and keep the same extruder on every run.
        const DynamicPrintConfig two_tools = perimeter_config({{"perimeters", "2"}, {"nozzle_diameter", "0.4,0.4"}});
        const std::vector<GroupSummary> tools =
            group_summaries(run_half_override(generator, two_tools, {{"perimeter_extruder", "2"}}));
        REQUIRE(tools.size() == 2);
        CHECK(tools[0].extruder_id == 0);
        CHECK(tools[1].extruder_id == 1);
        CHECK(tools[0].region_count == 1);
        CHECK(tools[1].region_count == 1);
        CHECK(tools[0].max_x < 0);
        CHECK(tools[1].min_x > 0);
        CHECK(group_summaries(run_half_override(generator, two_tools, {{"perimeter_extruder", "2"}})) == tools);

        // Each half gets the perimeter count of its own region.
        const std::vector<GroupSummary> counts =
            group_summaries(run_half_override(generator, perimeter_config({{"perimeters", "1"}}), {{"perimeters", "3"}}));
        REQUIRE(counts.size() == 2);
        CHECK(counts[0].max_perimeter_idx == 0);
        CHECK(counts[1].max_perimeter_idx == 2);
        CHECK(counts[0].max_x < 0);
        CHECK(counts[1].min_x > 0);

        // Regions with equal settings stay one group rooted at the whole island.
        const DynamicPrintConfig config = perimeter_config({{"perimeters", "2"}});
        const std::vector<PerimeterRegionIslandCapture> single = run_half_override(generator, config, {});
        const std::vector<PerimeterRegionIslandCapture> agreeing = run_half_override(generator, config, {{"perimeters", "2"}});
        REQUIRE(single.size() == 1);
        REQUIRE(agreeing.size() == 1);
        CHECK(single.front().region_count == 1);
        CHECK(agreeing.front().region_count == 2);
        CHECK(printed_leaves(agreeing) == printed_leaves(single));
    }
}

TEST_CASE("A region group cut into several pieces keeps the perimeters of every piece",
          "[plugins][perimeter][perimeter-region-groups]")
{
    // A central band with another extruder cuts the default region in two
    // pieces. Both pieces belong to one region group and one region island,
    // which must hold the perimeters of the left and of the right piece.
    const DynamicPrintConfig config = perimeter_config({{"perimeters", "2"}, {"nozzle_diameter", "0.4,0.4"}});
    const std::vector<PerimeterRegionOverride> band{
        {rectangle_expolygon(-3., -10., 3., 10.), {{"perimeter_extruder", "2"}}}};
    for (const char *generator : generators()) {
        INFO(generator);
        const std::vector<GroupSummary> groups = group_summaries(run_perimeter_region_islands_case(
            config, {generator}, rectangle_expolygon(-10., -10., 10., 10.), 1, band));
        REQUIRE(groups.size() == 2);
        const GroupSummary &outer = groups[0].extruder_id == 0 ? groups[0] : groups[1];
        const GroupSummary &inner = groups[0].extruder_id == 0 ? groups[1] : groups[0];
        CHECK(outer.extruder_id == 0);
        CHECK(inner.extruder_id == 1);
        CHECK(outer.min_x < scale_i(-3.));
        CHECK(outer.max_x > scale_i(3.));
        CHECK(inner.min_x > scale_i(-3.));
        CHECK(inner.max_x < scale_i(3.));
    }
}

TEST_CASE("Perimeter generators keep regions with different perimeter flows in separate groups",
          "[plugins][perimeter][perimeter-region-groups]")
{
    // A group takes the flows of its first region, and the flows read the
    // spacings, odd-layer changes and overlaps of both perimeter roles, so a
    // region differing only by one of them must get its own group.
    const DynamicPrintConfig config = perimeter_config({{"perimeters", "2"}});
    for (const char *generator : generators()) {
        for (const std::pair<std::string, std::string> &setting :
             std::vector<std::pair<std::string, std::string>>{{"external_perimeter_extrusion_spacing", "0.5"},
                                                              {"external_perimeter_extrusion_change_odd_layers", "0.5"},
                                                              {"external_perimeter_overlap", "60%"},
                                                              {"perimeter_extrusion_spacing", "0.5"},
                                                              {"perimeter_extrusion_change_odd_layers", "0.5"},
                                                              {"perimeter_overlap", "60%"}}) {
            INFO(generator << " " << setting.first);
            const std::vector<GroupSummary> groups = group_summaries(run_half_override(generator, config, {setting}));
            REQUIRE(groups.size() == 2);
            CHECK(groups[0].region_count == 1);
            CHECK(groups[1].region_count == 1);
            CHECK(groups[0].max_x < 0);
            CHECK(groups[1].min_x > 0);
        }
    }
}

TEST_CASE("Arachne keeps regions with different wall settings in separate groups",
          "[plugins][perimeter][perimeter-region-groups][arachne]")
{
    // WallToolPaths reads its wall settings from the group's first region, so
    // a region differing only by one of them must get its own group.
    const DynamicPrintConfig config = perimeter_config({{"perimeters", "2"}});
    for (const std::pair<std::string, std::string> &setting :
         std::vector<std::pair<std::string, std::string>>{{"wall_distribution_count", "3"},
                                                          {"min_bead_width", "50%"},
                                                          {"perimeter_direction", "cw_ccw"}}) {
        INFO(setting.first);
        const std::vector<GroupSummary> groups =
            group_summaries(run_half_override(ARACHNE_PERIMETER_GENERATOR, config, {setting}));
        REQUIRE(groups.size() == 2);
        CHECK(groups[0].region_count == 1);
        CHECK(groups[1].region_count == 1);
        CHECK(groups[0].max_x < 0);
        CHECK(groups[1].min_x > 0);
    }
}
