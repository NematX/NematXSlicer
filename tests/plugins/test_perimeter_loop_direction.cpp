#include <catch2/catch.hpp>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionGeometryUnits.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

/*
These tests check the direction of the closed perimeter loops, which the
perimeter.post_process.loop_direction plugin sets after every generator:
perimeter_direction per contour and hole, perimeter_reverse on odd layers, and
overhangs_reverse for the overhanging loops of a steep overhang on odd layers.

    TEST_CASE
    |-- run a generator, then the post-process plugins, on a small island
    |   `-- loop_orientations(): direction, kind and permissions of each loop
    `-- export a cube with a hole to G-code
        `-- gcode_loops(): direction of each printed perimeter loop, per layer
*/

namespace {
using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

const char *const k_directions[] = {"ccw_cw", "ccw_ccw", "cw_ccw", "cw_cw"};

// One closed perimeter loop of a published perimeter tree.
struct LoopOrientation
{
    bool hole = false;
    bool ccw = false;
    bool overhang = false;
    bool reversible = false;
};

// One perimeter loop printed in a G-code.
struct GCodeLoop
{
    size_t layer = 0;
    bool hole = false;
    bool ccw = false;
};

// The generators that can publish loops, with the Python simple one when Python plugins are built.
std::vector<const char *> generators();
// The wanted direction of a loop: perimeter_direction, flipped when the loop is reversed.
bool want_ccw(const std::string &direction, bool hole, bool reversed);
// Whether entity or one of its descendants has an overhang perimeter role.
bool has_overhang_path(const ExtrusionEntity &entity);
// Collect each loop of the tree: the first entity with the LOOP perimeter flag whose points close.
void collect_loops(const ExtrusionEntity &entity, std::vector<LoopOrientation> &out);
std::vector<LoopOrientation> loop_orientations(const ExtrusionEntity &root);
// Read the perimeter loops of a G-code with relative extrusion; a loop smaller than 15 x 15 mm is the hole's.
std::vector<GCodeLoop> gcode_loops(const std::string &gcode);
// Island with a hole far from the overhang, for the overhang reversal: the overhang is the part right of x = 5.
ExPolygon overhang_target();

std::vector<const char *> generators()
{
    std::vector<const char *> out{CLASSIC_PERIMETER_GENERATOR, SIMPLE_PERIMETER_GENERATOR, ARACHNE_PERIMETER_GENERATOR};
#ifdef SLIC3R_TEST_PYTHON_PLUGINS
    out.push_back(PYTHON_SIMPLE_PERIMETER_GENERATOR);
#endif
    return out;
}

bool want_ccw(const std::string &direction, const bool hole, const bool reversed)
{
    const bool ccw = hole ? (direction == "cw_ccw" || direction == "ccw_ccw") :
                            (direction == "ccw_cw" || direction == "ccw_ccw");
    return ccw != reversed;
}

bool has_overhang_path(const ExtrusionEntity &entity)
{
    if (const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>())
        if (attributes->extrusion_role().is_perimeter() && attributes->extrusion_role().is_bridge())
            return true;
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        if (has_overhang_path(entity.child(idx)))
            return true;
    return false;
}

void collect_loops(const ExtrusionEntity &entity, std::vector<LoopOrientation> &out)
{
    const ExtrusionPropertyLoopRole *loop = entity.get_property<ExtrusionPropertyLoopRole>();
    Points points;
    entity.collect_points(points);
    if (loop != nullptr && (loop->perimeter_role() & elrDefault) != 0 && points.size() > 2 &&
        points.front() == points.back()) {
        out.push_back({(loop->perimeter_role() & elrHole) != 0, extrusion_geometry_area(entity) > 0.,
                       has_overhang_path(entity), entity.can_reverse()});
        return;
    }
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        collect_loops(entity.child(idx), out);
}

std::vector<LoopOrientation> loop_orientations(const ExtrusionEntity &root)
{
    std::vector<LoopOrientation> out;
    collect_loops(root, out);
    return out;
}

std::vector<GCodeLoop> gcode_loops(const std::string &gcode)
{
    std::vector<GCodeLoop> out;
    std::vector<Vec2d> points;
    std::string type, line;
    size_t layer = 0;
    bool started = false;
    double x = 0., y = 0.;
    const auto flush = [&]() {
        if (points.size() > 3 && (type == "External perimeter" || type == "Perimeter")) {
            double twice_area = 0.;
            for (size_t idx = 0; idx < points.size(); ++idx) {
                const Vec2d &a = points[idx];
                const Vec2d &b = points[(idx + 1) % points.size()];
                twice_area += a.x() * b.y() - b.x() * a.y();
            }
            // Smaller pieces are not loops: gap fill or a loop cut by a type change.
            if (std::abs(twice_area) > 2. * 20.)
                out.push_back({layer, std::abs(twice_area) < 2. * 225., twice_area > 0.});
        }
        points.clear();
    };
    std::istringstream input(gcode);
    while (std::getline(input, line)) {
        if (line.rfind(";LAYER_CHANGE", 0) == 0) {
            flush();
            if (started)
                ++layer;
            started = true;
            continue;
        }
        if (line.rfind(";TYPE:", 0) == 0) {
            flush();
            type = line.substr(6, line.find_last_not_of(" \t\r") - 5);
            continue;
        }
        std::istringstream words(line.substr(0, line.find(';')));
        std::string word;
        words >> word;
        if (word != "G1" && word != "G0")
            continue;
        double nx = x, ny = y, e = 0.;
        bool moves = false;
        while (words >> word) {
            if (word.size() < 2)
                continue;
            const double value = std::stod(word.substr(1));
            if (word[0] == 'X') {
                nx = value;
                moves = true;
            } else if (word[0] == 'Y') {
                ny = value;
                moves = true;
            } else if (word[0] == 'E') {
                e = value;
            }
        }
        if (moves && e > 0.) {
            if (points.empty())
                points.emplace_back(x, y);
            points.emplace_back(nx, ny);
        } else if (moves) {
            flush();
        }
        x = nx;
        y = ny;
    }
    flush();
    return out;
}

ExPolygon overhang_target()
{
    ExPolygon out = rectangle_expolygon(-10., -5., 10., 5.);
    out.holes.push_back(Polygon({
        Point(scale_i(-8.), scale_i(-2.)),
        Point(scale_i(-8.), scale_i(2.)),
        Point(scale_i(-4.), scale_i(2.)),
        Point(scale_i(-4.), scale_i(-2.))
    }));
    return out;
}

} // namespace

TEST_CASE("Loop direction orients every perimeter loop by contour, hole and odd-layer reversal",
          "[plugins][perimeter][loop-direction]")
{
    const char *direction = GENERATE(from_range(std::begin(k_directions), std::end(k_directions)));
    const bool reverse = GENERATE(false, true);
    const size_t layer = GENERATE(size_t(1), size_t(2));
    for (const char *generator : generators()) {
        CAPTURE(generator, direction, reverse, layer);
        const DynamicPrintConfig config = perimeter_config({{"perimeters", "2"},
                                                            {"perimeter_direction", direction},
                                                            {"perimeter_reverse", reverse ? "1" : "0"}});
        const PerimeterRunCapture capture = run_perimeter_and_post_case(config, {generator}, {LOOP_DIRECTION},
                                                                        rectangle_with_hole_expolygon(), layer);
        const std::vector<LoopOrientation> loops = loop_orientations(capture.external_perimeters);
        size_t holes = 0;
        for (const LoopOrientation &loop : loops) {
            CAPTURE(loop.hole);
            holes += loop.hole ? 1 : 0;
            CHECK(loop.ccw == want_ccw(direction, loop.hole, reverse && layer % 2 == 1));
            CHECK_FALSE(loop.reversible);
        }
        CHECK(holes >= 2);
        CHECK(loops.size() - holes >= 2);
    }
}

TEST_CASE("Loop direction reverses the overhanging loops of a steep overhang on odd layers",
          "[plugins][perimeter][loop-direction]")
{
    // The island overhangs the layer below by 5 mm on its right side. On an odd
    // layer, the loops with an overhang path run backwards, the loops around the
    // supported hole do not; perimeter_reverse reverses every loop instead, and a
    // threshold wider than twice the overhang turns the reversal off.
    const bool reverse = GENERATE(false, true);
    const size_t layer = GENERATE(size_t(1), size_t(2));
    const bool wide_threshold = GENERATE(false, true);
    for (const char *generator : generators()) {
        CAPTURE(generator, reverse, layer, wide_threshold);
        const DynamicPrintConfig config = perimeter_config({{"overhangs", "1"},
                                                            {"overhangs_flow_ratio", "100"},
                                                            {"overhangs_type", "nozzle"},
                                                            {"overhangs_width", "0.2"},
                                                            {"overhangs_width_speed", "0.2"},
                                                            {"overhangs_reverse", "1"},
                                                            {"overhangs_reverse_threshold", wide_threshold ? "11" : "250%"},
                                                            {"perimeter_reverse", reverse ? "1" : "0"},
                                                            {"perimeters", "2"}});
        const PerimeterRunCapture capture = run_perimeter_and_post_case_with_lower_area(
            config, {generator}, {DETECT_OVERHANG, LOOP_DIRECTION}, overhang_target(),
            rectangle_expolygon(-10., -5., 5., 5.), layer);
        const std::vector<LoopOrientation> loops = loop_orientations(capture.external_perimeters);
        size_t overhanging = 0;
        for (const LoopOrientation &loop : loops) {
            CAPTURE(loop.hole, loop.overhang);
            overhanging += loop.overhang ? 1 : 0;
            const bool odd = layer % 2 == 1;
            const bool reversed = odd && (reverse || (loop.overhang && !wide_threshold));
            CHECK(loop.ccw == want_ccw("ccw_cw", loop.hole, reversed));
        }
        CHECK(overhanging > 0);
        CHECK(loops.size() > overhanging);
    }
}

TEST_CASE("Exported perimeter loops keep the direction of the loop direction settings",
          "[plugins][perimeter][loop-direction][plugin-e2e]")
{
    // Ordering may still choose each seam, but no later step may reverse a loop.
    const char *direction = GENERATE(from_range(std::begin(k_directions), std::end(k_directions)));
    const bool reverse = GENERATE(false, true);
    for (const char *generator : generators()) {
        CAPTURE(generator, direction, reverse);
        DynamicPrintConfig config = perimeter_config({{"gcode_exporter_plugin", "gcode.exporter.simple"},
                                                      {"step_perimeter_plugin", generator},
                                                      {"layer_height", "0.2"},
                                                      {"first_layer_height", "0.2"},
                                                      {"perimeters", "2"},
                                                      {"perimeter_direction", direction},
                                                      {"perimeter_reverse", reverse ? "1" : "0"},
                                                      {"fill_density", "0%"},
                                                      {"top_solid_layers", "0"},
                                                      {"bottom_solid_layers", "0"},
                                                      {"top_solid_min_thickness", "0"},
                                                      {"bottom_solid_min_thickness", "0"},
                                                      {"skirts", "0"},
                                                      {"use_relative_e_distances", "1"}});
        Print print;
        Model model;
        Test::init_print({Test::TestMesh::cube_with_hole}, print, model, config);
        const std::vector<GCodeLoop> loops = gcode_loops(Test::gcode(print));
        size_t holes = 0;
        for (const GCodeLoop &loop : loops) {
            CAPTURE(loop.layer, loop.hole);
            holes += loop.hole ? 1 : 0;
            CHECK(loop.ccw == want_ccw(direction, loop.hole, reverse && loop.layer % 2 == 1));
        }
        // Two contour and two hole loops on each of the 50 layers.
        CHECK(holes >= 50);
        CHECK(loops.size() - holes >= 50);
    }
}

TEST_CASE("Loop direction owns perimeter_reverse and its activation rule", "[plugins][perimeter][loop-direction][gui-rules]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    Plugin *plugin = orchestrator.get_plugin(LOOP_DIRECTION);
    REQUIRE(plugin != nullptr);

    std::vector<std::string> defined_keys;
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys())
        defined_keys.push_back(key.key);
    CHECK(defined_keys == std::vector<std::string>{"perimeter_reverse"});

    const ConfigOptionDef *def = PrintConfigDef::instance().get("perimeter_reverse");
    REQUIRE(def != nullptr);
    CHECK(def->container_type == ConfigOptionContainerType::Region);
    CHECK(def->invalidates_step == STEP_PERIMETER);
    CHECK(PrintConfig::defaults().option("perimeter_reverse") == nullptr);
    CHECK(PrintRegionConfig::defaults().option("perimeter_reverse") != nullptr);

    // Every perimeter generator now reverses through the plugin, so the plugin rule alone decides.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const auto enabled = [&orchestrator, &config]() {
        for (const Orchestrator::GuiRuleApplication &application :
             orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, -1))
            if (application.target_key == "perimeter_reverse")
                return application.enabled;
        return false;
    };

    config.set_deserialize("step_perimeter_plugin", "perimeter.generator.simple");
    config.option<ConfigOptionInt>("perimeters")->value = 0;
    CHECK_FALSE(enabled());
    config.option<ConfigOptionInt>("perimeters")->value = 2;
    CHECK(enabled());
}
