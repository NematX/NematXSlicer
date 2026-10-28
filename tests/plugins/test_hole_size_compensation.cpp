#include <catch2/catch.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/Api/plugin/cpp/HoleGeometry.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Plugins/HoleSizeCompensation.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "libslic3r/TriangleMesh.hpp"

/*
These tests exercise convex-hole geometry, regional ownership and host
post-slicing publication. Small plates include round and concave voids, shared
walls and islands inside holes. Geometry stays owned by each fixture's storage
until copied results have been checked.

    TEST_CASE
    |-- run(): call compensate_holes() with one setting per region
    |-- hole_box(): measure the final merged void independently
    `-- StepPostSlicing::run_step(): check composition with inner XY offsets
*/

namespace {

using namespace Slic3r;
using PolygonType = Slic3r::Polygon;
using slic3r_api::StoredExPolygonCollection;
using slic3r_api::HoleSizeCompensationPlugin::detail::RegionHoleSettings;
using slic3r_api::HoleSizeCompensationPlugin::detail::compensate_holes;

struct TestStorage
{
    PluginStorage native;
    storage_handle *get() { return reinterpret_cast<storage_handle *>(&native); }
};

coord_t mm(double value) { return scale_i(value); }

PolygonType rectangle(double min_x, double min_y, double max_x, double max_y)
{
    return PolygonType{ Point(mm(min_x), mm(min_y)), Point(mm(max_x), mm(min_y)),
                        Point(mm(max_x), mm(max_y)), Point(mm(min_x), mm(max_y)) };
}

// A regular polygon approximating a circle, clockwise like an ExPolygon hole.
PolygonType round_hole(double center_x, double center_y, double radius, int points = 64)
{
    PolygonType hole;
    for (int idx = 0; idx < points; ++idx) {
        const double angle = -2. * PI * idx / points;
        hole.points.emplace_back(mm(center_x + radius * std::cos(angle)), mm(center_y + radius * std::sin(angle)));
    }
    return hole;
}

PolygonType clockwise(PolygonType polygon)
{
    polygon.make_clockwise();
    return polygon;
}

// 20 x 20 mm plate with one hole.
ExPolygon plate_with(const PolygonType &hole)
{
    return ExPolygon(rectangle(0., 0., 20., 20.), clockwise(hole));
}

slic3r_api::ExPolygonCollection sdk_view(const ExPolygons &expolygons)
{
    return slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&expolygons));
}

const ExPolygons &native_result(const StoredExPolygonCollection &geometry)
{
    return *reinterpret_cast<const ExPolygons *>(geometry.handle());
}

RegionHoleSettings settings(double delta_mm, double threshold_mm2 = 100.)
{
    RegionHoleSettings result;
    result.delta = mm(delta_mm);
    result.threshold_area = scale_d(scale_d(threshold_mm2));
    return result;
}

// Run the geometry core on regions given as native slices.
std::vector<ExPolygons> run(TestStorage &storage, const std::vector<ExPolygons> &regions,
                            const std::vector<RegionHoleSettings> &region_settings, bool *changed = nullptr)
{
    std::vector<slic3r_api::ExPolygonCollection> views;
    for (const ExPolygons &region : regions)
        views.push_back(sdk_view(region));
    bool modified = false;
    const std::vector<StoredExPolygonCollection> result = compensate_holes(storage.get(), region_settings, views, modified);
    if (changed != nullptr)
        *changed = modified;
    std::vector<ExPolygons> out;
    for (const StoredExPolygonCollection &region : result)
        out.push_back(native_result(region));
    return out;
}

// Bounding box of the only hole of the merged regions.
BoundingBox hole_box(const std::vector<ExPolygons> &regions)
{
    ExPolygons all;
    for (const ExPolygons &region : regions)
        append(all, region);
    const ExPolygons merged = union_ex(all);
    REQUIRE(merged.size() == 1);
    REQUIRE(merged.front().holes.size() == 1);
    return get_extents(merged.front().holes.front());
}

double width(const BoundingBox &box) { return unscaled(box.size().x()); }

double area_of(const ExPolygons &areas)
{
    double result = 0.;
    for (const ExPolygon &area : areas)
        result += area.area();
    return result;
}

std::vector<c_point> sdk_points(const PolygonType &polygon)
{
    std::vector<c_point> points;
    for (const Point &point : polygon.points)
        points.push_back(c_point{ point.x(), point.y() });
    return points;
}

} // namespace

// ---- HoleGeometry.hpp -------------------------------------------------------

TEST_CASE("Hole geometry detects convex holes with the legacy tolerance", "[plugins][hole-size-compensation]")
{
    using slic3r_api::is_convex_hole;
    CHECK(is_convex_hole(sdk_points(clockwise(rectangle(8., 8., 12., 12.)))));
    CHECK(is_convex_hole(sdk_points(round_hole(10., 10., 2.5))));
    // An L-shaped cut-out has one concave corner.
    const PolygonType l_shape = clockwise(PolygonType{ Point(mm(6.), mm(6.)), Point(mm(14.), mm(6.)), Point(mm(14.), mm(9.)),
                                                       Point(mm(9.), mm(9.)), Point(mm(9.), mm(14.)), Point(mm(6.), mm(14.)) });
    CHECK_FALSE(is_convex_hole(sdk_points(l_shape)));
    // The middle of the bottom edge pushed into the hole: the corner there is
    // PI + 2 x bend. A 0.06 rad excess is tolerated, 0.2 rad is not.
    for (const auto &[bend, convex] : { std::pair<double, bool>{ 0.03, true }, std::pair<double, bool>{ 0.1, false } }) {
        const PolygonType square{ Point(0, 0), Point(0, mm(10.)), Point(mm(10.), mm(10.)), Point(mm(10.), 0),
                                  Point(mm(5.), mm(5. * std::tan(bend))) };
        REQUIRE(square.is_clockwise());
        CAPTURE(bend);
        CHECK(is_convex_hole(sdk_points(square)) == convex);
    }
    CHECK_FALSE(is_convex_hole(std::vector<c_point>{ { 0, 0 }, { 1, 1 } }));
}

TEST_CASE("Hole threshold factor ramps from full to none between T and 4T", "[plugins][hole-size-compensation]")
{
    using slic3r_api::hole_threshold_factor;
    CHECK(hole_threshold_factor(50., 0.) == 1.);
    CHECK(hole_threshold_factor(50., 100.) == 1.);
    CHECK(hole_threshold_factor(100., 100.) == 1.);
    CHECK(hole_threshold_factor(250., 100.) == Approx(0.5));
    CHECK(hole_threshold_factor(400., 100.) == 0.);
    CHECK(hole_threshold_factor(1000., 100.) == 0.);
}

TEST_CASE("Hole diameter estimate of a round hole", "[plugins][hole-size-compensation]")
{
    CHECK(unscaled(slic3r_api::approximate_hole_diameter(sdk_points(round_hole(30., -4., 2.5)))) == Approx(5.).margin(1e-6));
    CHECK(slic3r_api::approximate_hole_diameter(std::vector<c_point>{}) == 0.);
}

// ---- Geometry core ----------------------------------------------------------------

TEST_CASE("Hole compensation moves convex hole walls only", "[plugins][hole-size-compensation]")
{
    TestStorage storage;
    bool changed = false;

    SECTION("A round hole is enlarged by a negative value") {
        const auto out = run(storage, { { plate_with(round_hole(10., 10., 2.5)) } }, { settings(-0.1) }, &changed);
        CHECK(changed);
        CHECK(width(hole_box(out)) == Approx(5.2).margin(0.002));
    }
    SECTION("A square hole is shrunk by a positive value") {
        const auto out = run(storage, { { plate_with(rectangle(8., 8., 12., 12.)) } }, { settings(0.2) }, &changed);
        CHECK(changed);
        CHECK(width(hole_box(out)) == Approx(3.6).margin(0.00001));
    }
    SECTION("An L-shaped hole is not a bore and is not changed") {
        const PolygonType l_shape{ Point(mm(6.), mm(6.)), Point(mm(14.), mm(6.)), Point(mm(14.), mm(9.)),
                                   Point(mm(9.), mm(9.)), Point(mm(9.), mm(14.)), Point(mm(6.), mm(14.)) };
        const ExPolygons source{ plate_with(l_shape) };
        const auto out = run(storage, { source }, { settings(-0.3) }, &changed);
        CHECK_FALSE(changed);
        CHECK(area_of(out.front()) == Approx(area_of(source)));
    }
    SECTION("The threshold reduces the value for larger holes") {
        // Hole area: about 19.6 mm².
        const ExPolygons source{ plate_with(round_hole(10., 10., 2.5)) };
        CHECK_FALSE((run(storage, { source }, { settings(-0.1, 1.) }, &changed), changed)); // area > 4T
        const double factor = (40. - std::abs(round_hole(10., 10., 2.5).area()) / scale_d(scale_d(1.))) / 30.;
        const auto ramp = run(storage, { source }, { settings(-0.1, 10.) });
        CHECK(width(hole_box(ramp)) == Approx(5. + 0.2 * factor).margin(0.002));
        const auto unlimited = run(storage, { source }, { settings(-0.1, 0.) });
        CHECK(width(hole_box(unlimited)) == Approx(5.2).margin(0.002));
    }
    SECTION("A zero value changes nothing") {
        CHECK_FALSE((run(storage, { { plate_with(round_hole(10., 10., 2.5)) } }, { settings(0.) }, &changed), changed));
    }
}

TEST_CASE("Hole compensation moves each region's part of a shared hole wall", "[plugins][hole-size-compensation]")
{
    TestStorage storage;
    const ExPolygons plate{ plate_with(rectangle(8., 7., 12., 13.)) };
    const ExPolygons left = intersection_ex(plate, ExPolygons{ ExPolygon(rectangle(-1., -1., 10., 21.)) });
    const ExPolygons right = diff_ex(plate, left);

    SECTION("Enlarging: each region removes only its own material") {
        const auto out = run(storage, { left, right }, { settings(-0.2), settings(0.) });
        const BoundingBox box = hole_box(out);
        CHECK(unscaled(box.min.x()) == Approx(7.8).margin(0.00001));
        CHECK(unscaled(box.max.x()) == Approx(12.).margin(0.00001));
        CHECK(area_of(out[1]) == Approx(area_of(right)));
        CHECK(intersection_ex(out[0], out[1]).empty());
    }
    SECTION("Shrinking: the new material goes to the region forming that part of the wall") {
        const auto out = run(storage, { left, right }, { settings(0.2), settings(0.) });
        const BoundingBox box = hole_box(out);
        CHECK(unscaled(box.min.x()) == Approx(8.2).margin(0.00001));
        CHECK(unscaled(box.max.x()) == Approx(12.).margin(0.00001));
        CHECK(area_of(out[1]) == Approx(area_of(right)));
        CHECK(area_of(out[0]) > area_of(left));
        CHECK(intersection_ex(out[0], out[1]).empty());
    }
    SECTION("A region that does not reach the hole is not changed") {
        // Split the plate far from the hole: only the region around it acts.
        const ExPolygons far_left = intersection_ex(plate, ExPolygons{ ExPolygon(rectangle(-1., -1., 3., 21.)) });
        const ExPolygons rest = diff_ex(plate, far_left);
        const auto out = run(storage, { far_left, rest }, { settings(-0.5), settings(0.) });
        CHECK(area_of(out[0]) == Approx(area_of(far_left)));
        CHECK(width(hole_box(out)) == Approx(4.).margin(0.00001));
    }
}

TEST_CASE("Hole compensation leaves an island inside the hole untouched", "[plugins][hole-size-compensation]")
{
    TestStorage storage;
    const ExPolygon island(rectangle(9.5, 9.5, 10.5, 10.5));
    const ExPolygons source{ plate_with(rectangle(7., 7., 13., 13.)), island };
    for (double delta : { -0.3, 0.6 }) {
        CAPTURE(delta);
        const auto out = run(storage, { source }, { settings(delta) });
        // The island is kept whole, and new material does not cover it twice.
        CHECK(diff_ex(ExPolygons{ island }, out.front()).empty());
        const ExPolygons inside = intersection_ex(out.front(), ExPolygons{ ExPolygon(rectangle(8., 8., 12., 12.)) });
        CHECK(area_of(inside) == Approx(area_of(ExPolygons{ island })));
    }
}

// ---- Settings and pipeline ---------------------------------------------------------

TEST_CASE("Hole compensation owns its regional settings and runs before XY compensation",
          "[plugins][hole-size-compensation][config]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *plugin = orchestrator.get_plugin("slicing.hole_size_compensation");
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_step() == STEP_POST_SLICING);
    Plugin *xy = orchestrator.get_plugin("slicing.xy_size_compensation");
    REQUIRE(xy != nullptr);
    CHECK(plugin->get_priority() < xy->get_priority());

    std::vector<std::string> defined_keys;
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys())
        defined_keys.push_back(key.key);
    CHECK(defined_keys.size() == 2);
    for (const auto &[key, default_value, sidetext] : { std::tuple<const char *, const char *, const char *>{ "hole_size_compensation", "0", "mm" },
                                                        std::tuple<const char *, const char *, const char *>{ "hole_size_threshold", "100", "mm²" } }) {
        CAPTURE(key);
        CHECK(std::find(defined_keys.begin(), defined_keys.end(), key) != defined_keys.end());
        const ConfigOptionDef *def = PrintConfigDef::instance().get(key);
        REQUIRE(def != nullptr);
        CHECK(def->type == coFloat);
        CHECK(def->container_type == ConfigOptionContainerType::Region);
        CHECK(def->option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
        CHECK(def->invalidates_step == STEP_POST_SLICING);
        CHECK(def->sidetext == sidetext);
        CHECK(def->default_value->serialize() == default_value);
        CHECK(PrintRegionConfig::defaults().option(key) != nullptr);
        CHECK(PrintObjectConfig::defaults().option(key) == nullptr);
    }
}

namespace {

// Restrict the pipeline tests to slicing and the two compensations.
class ScopedHolePlugins
{
public:
    ScopedHolePlugins() : orchestrator(Orchestrator::instance())
    {
        for (Plugin *plugin : orchestrator.active_plugins())
            previous.push_back(plugin);
        orchestrator.clear_active_plugins();
        for (const char *id : { "standard_layer_height_generator", "slice_volume",
                               "slicing.hole_size_compensation", "slicing.xy_size_compensation" })
            REQUIRE(orchestrator.set_plugin_active(id, true));
    }
    ~ScopedHolePlugins()
    {
        orchestrator.clear_active_plugins();
        for (Plugin *plugin : previous)
            orchestrator.set_plugin_active(plugin, true);
    }
private:
    Orchestrator &orchestrator;
    std::vector<Plugin *> previous;
};

DynamicPrintConfig hole_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height", "1" }, { "first_layer_height", "1" },
        { "xy_size_compensation", "0" }, { "xy_inner_size_compensation", "0" },
        { "first_layer_size_compensation", "0" }, { "first_layer_size_compensation_no_collapse", "0" },
        { "hole_size_compensation", "0" }, { "hole_size_threshold", "100" },
        { "slice_closing_radius", "0" }, { "raft_layers", "0" }
    });
    return config;
}

// Slice two touching 10 x 20 mm boxes (one region each), then replace their
// raw slices on layer 0 with the two halves of a plate with a square hole.
struct TwoRegionPlate
{
    Print print;
    Model model;
    std::vector<std::size_t> regions;

    TwoRegionPlate(const DynamicPrintConfig &config, const char *left_value, const char *right_value)
    {
        ModelObject *object = model.add_object();
        for (int index = 0; index < 2; ++index) {
            TriangleMesh mesh = make_cube(10., 20., 2.);
            mesh.translate(10. * index, 0., 0.);
            ModelVolume *volume = object->add_volume(std::move(mesh));
            volume->config.set_deserialize_strict("hole_size_compensation", index == 0 ? left_value : right_value);
        }
        object->add_instance();
        print.auto_assign_extruders(object);
        print.apply(model, config);
        Orchestrator &orchestrator = Orchestrator::instance();
        Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
        Steps::StepSlicing::run_step(orchestrator, print);

        Layer &layer = print.object(0).layer(0);
        for (std::size_t idx = 0; idx < layer.region_count(); ++idx)
            if (!layer.region(idx).get_raw_slices().empty())
                regions.push_back(idx);
        // Volumes with equal settings share one region.
        REQUIRE(regions.size() == (std::string(left_value) == right_value ? 1 : 2));
        // Put the region of the left volume first, whatever the host's order.
        const auto value_of = [&layer](std::size_t idx) {
            return layer.region(idx).region().config().opt_float("hole_size_compensation");
        };
        if (regions.size() == 2 && value_of(regions[0]) != std::stod(left_value))
            std::swap(regions[0], regions[1]);
        // The volumes may have been centred on the bed: build the plate where they are.
        const BoundingBox bounds = get_extents(layer.lslices());
        const double x0 = unscaled(bounds.min.x()), y0 = unscaled(bounds.min.y());
        const ExPolygons plate{ ExPolygon(rectangle(x0, y0, x0 + 20., y0 + 20.),
                                          clockwise(rectangle(x0 + 8., y0 + 7., x0 + 12., y0 + 13.))) };
        const ExPolygons left = intersection_ex(plate, ExPolygons{ ExPolygon(rectangle(x0 - 1., y0 - 1., x0 + 10., y0 + 21.)) });
        if (regions.size() == 1) {
            ApiInternal::LayerRegionAccess::slices_mutable(layer.region(regions[0])) = plate;
        } else {
            ApiInternal::LayerRegionAccess::slices_mutable(layer.region(regions[0])) = left;
            ApiInternal::LayerRegionAccess::slices_mutable(layer.region(regions[1])) = diff_ex(plate, left);
        }
        ApiInternal::LayerAccess::recompute_slices_from_layer_regions(layer);
    }

    void run_post_slicing()
    {
        Steps::StepPostSlicing::run_step(Orchestrator::instance(), print);
        std::string error;
        const bool valid = Steps::StepPostSlicing::validate_post(print, error);
        INFO(error);
        CHECK(valid);
    }

    BoundingBox hole() const
    {
        const Layer &layer = print.object(0).layer(0);
        REQUIRE(layer.lslices().size() == 1);
        REQUIRE(layer.lslices().front().holes.size() == 1);
        return get_extents(layer.lslices().front().holes.front());
    }
    double left_edge() const { return unscaled(hole().min.x() - get_extents(print.object(0).layer(0).lslices()).min.x()); }
    double right_edge() const { return unscaled(hole().max.x() - get_extents(print.object(0).layer(0).lslices()).min.x()); }
};

} // namespace

TEST_CASE("Hole compensation follows the region settings in the post-slicing step",
          "[plugins][hole-size-compensation][integration]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedHolePlugins active;

    SECTION("Each region moves its part of the wall") {
        TwoRegionPlate plate(hole_config(), "-0.2", "0");
        plate.run_post_slicing();
        CHECK(plate.left_edge() == Approx(7.8).margin(0.00001));
        CHECK(plate.right_edge() == Approx(12.).margin(0.00001));
    }
    SECTION("Inner XY compensation adds to the hole compensation") {
        DynamicPrintConfig config = hole_config();
        config.set_deserialize_strict("xy_inner_size_compensation", "-0.1");
        TwoRegionPlate plate(config, "-0.2", "-0.2");
        plate.run_post_slicing();
        CHECK(plate.left_edge() == Approx(7.7).margin(0.00001));
        CHECK(plate.right_edge() == Approx(12.3).margin(0.00001));
    }
    SECTION("The threshold of the region limits large holes") {
        DynamicPrintConfig config = hole_config();
        config.set_deserialize_strict("hole_size_threshold", "5"); // hole: 24 mm² > 4 x 5
        TwoRegionPlate plate(config, "-0.2", "-0.2");
        plate.run_post_slicing();
        CHECK(plate.left_edge() == Approx(8.).margin(0.00001));
        CHECK(plate.right_edge() == Approx(12.).margin(0.00001));
    }
}
