#include <catch2/catch.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"
#include "libslic3r/Plugins/Infill/Rectilinear/RectilinearFills.hpp"

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Fill/FillRectilinear.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Surface.hpp"

/*
Rectilinear patterns port
=========================

The ten rectilinear patterns (rectilinear, aligned rectilinear, monotonic,
monotonic lines, grid, triangles, stars, cubic, support base and scattered
rectilinear) are a verbatim port of FillRectilinear.cpp. These tests run the
core and the ported implementations on the same surfaces, angle and density,
and require the same set of extruded segments, the same number of paths and the
same total length.

The comparison is a multiset of segments: path ordering belongs to the later
STEP_ORDERING stage, so the general comparison does not require a path order.
Monotonic patterns additionally compare every point in every path in order.
*/

namespace rect_work = slic3r_api::PluginCommon::polygon_work;
namespace rect_fills = slic3r_api::Infill::RectilinearPatternsPlugin::fills;

namespace {

Slic3r::ExPolygon make_core_surface(const std::string &name)
{
    if (name == "square") {
        return Slic3r::ExPolygon(Slic3r::Polygon({ Slic3r::Point(0, 0), Slic3r::Point(40000000, 0),
                                                   Slic3r::Point(40000000, 30000000), Slic3r::Point(0, 30000000) }));
    }
    if (name == "hole") {
        Slic3r::ExPolygon out(Slic3r::Polygon({ Slic3r::Point(-20000000, -15000000), Slic3r::Point(20000000, -15000000),
                                                Slic3r::Point(20000000, 15000000), Slic3r::Point(-20000000, 15000000) }));
        out.holes.push_back(Slic3r::Polygon({ Slic3r::Point(-6000000, -4000000), Slic3r::Point(-6000000, 4000000),
                                              Slic3r::Point(6000000, 4000000), Slic3r::Point(6000000, -4000000) }));
        return out;
    }
    if (name == "headstick") {
        // The closing edge retraces the first edge, so the only stick sits
        // around the first vertex: remove_sticks() drops (0, 0) and keeps a 3
        // point triangle. The multi-sweep path offsets the source with a zero
        // rotation angle, the only way to reach the "reuse the API source" fast
        // path, which must stay closed once that point is gone.
        return Slic3r::ExPolygon(Slic3r::Polygon({ Slic3r::Point(0, 0), Slic3r::Point(20000000, 0),
                                                   Slic3r::Point(20000000, 20000000), Slic3r::Point(10000000, 0) }));
    }
    if (name == "dirtyholes") {
        // The outer has an interior spike and the hole has a head-only stick,
        // all points distinct so the source stays assert valid: both cleanups
        // must apply, the hole used to be skipped when the outer contour was
        // cleaned.
        Slic3r::ExPolygon out(Slic3r::Polygon({ Slic3r::Point(0, 0), Slic3r::Point(40000000, 0),
                                                Slic3r::Point(20000000, 0), Slic3r::Point(20000000, 30000000),
                                                Slic3r::Point(0, 30000000) }));
        out.holes.push_back(Slic3r::Polygon({ Slic3r::Point(4000000, 6000000), Slic3r::Point(10000000, 6000000),
                                              Slic3r::Point(16000000, 24000000), Slic3r::Point(16000000, 6000000) }));
        return out;
    }
    // L shape, concave.
    return Slic3r::ExPolygon(Slic3r::Polygon({ Slic3r::Point(0, 0), Slic3r::Point(30000000, 0),
                                               Slic3r::Point(30000000, 10000000), Slic3r::Point(10000000, 10000000),
                                               Slic3r::Point(10000000, 25000000), Slic3r::Point(0, 25000000) }));
}

using Segment = std::array<coord_t, 4>;

Segment make_segment(const Slic3r::Point &a, const Slic3r::Point &b)
{
    if (b.x() < a.x() || (b.x() == a.x() && b.y() < a.y()))
        return { b.x(), b.y(), a.x(), a.y() };
    return { a.x(), a.y(), b.x(), b.y() };
}

Segment make_segment(const rect_work::Point &a, const rect_work::Point &b)
{
    if (b.x() < a.x() || (b.x() == a.x() && b.y() < a.y()))
        return { b.x(), b.y(), a.x(), a.y() };
    return { a.x(), a.y(), b.x(), b.y() };
}

std::vector<Segment> core_segments(const Slic3r::Polylines &polylines, double *out_length)
{
    std::vector<Segment> out;
    *out_length = 0.;
    for (const Slic3r::Polyline &polyline : polylines) {
        for (size_t idx = 1; idx < polyline.points.size(); ++idx)
            out.push_back(make_segment(polyline.points[idx - 1], polyline.points[idx]));
        *out_length += polyline.length();
    }
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<Segment> ported_segments(const rect_work::Polylines &polylines, double *out_length)
{
    std::vector<Segment> out;
    *out_length = 0.;
    for (const rect_work::Polyline &polyline : polylines) {
        for (size_t idx = 1; idx < polyline.points.size(); ++idx)
            out.push_back(make_segment(polyline.points[idx - 1], polyline.points[idx]));
        *out_length += polyline.length();
    }
    std::sort(out.begin(), out.end());
    return out;
}

void require_same_lines(const Slic3r::Polylines &core, const rect_work::Polylines &ported, const std::string &info)
{
    INFO(info);
    double core_length = 0.;
    double ported_length = 0.;
    const std::vector<Segment> core_segs = core_segments(core, &core_length);
    const std::vector<Segment> ported_segs = ported_segments(ported, &ported_length);

    REQUIRE(core.size() == ported.size());
    REQUIRE(core_segs.size() == ported_segs.size());
    CHECK(core_length == Approx(ported_length).margin(1.));
    for (size_t idx = 0; idx < std::min(core_segs.size(), ported_segs.size()); ++idx) {
        INFO("segment " << idx);
        CHECK(core_segs[idx] == ported_segs[idx]);
    }
}

// Monotonic parity includes direction because STEP_ORDERING must not reverse it.
void require_same_directed_lines(const Slic3r::Polylines &core,
                                 const rect_work::Polylines &ported,
                                 const std::string &info)
{
    INFO(info);
    REQUIRE(core.size() == ported.size());
    for (size_t polyline_idx = 0; polyline_idx < core.size(); ++polyline_idx) {
        INFO("polyline " << polyline_idx);
        REQUIRE(core[polyline_idx].points.size() == ported[polyline_idx].points.size());
        for (size_t point_idx = 0; point_idx < core[polyline_idx].points.size(); ++point_idx) {
            CHECK(core[polyline_idx].points[point_idx].x() == ported[polyline_idx].points[point_idx].x());
            CHECK(core[polyline_idx].points[point_idx].y() == ported[polyline_idx].points[point_idx].y());
        }
    }
}

struct PatternCase
{
    const char *label;
    Slic3r::InfillPattern pattern;
    bool monotonic;
};

std::unique_ptr<rect_fills::FillRectilinear> make_ported_pattern(const PatternCase &pattern_case)
{
    switch (pattern_case.pattern) {
    case Slic3r::ipRectilinear:           return std::make_unique<rect_fills::FillRectilinear>();
    case Slic3r::ipAlignedRectilinear:    return std::make_unique<rect_fills::FillAlignedRectilinear>();
    case Slic3r::ipMonotonic:             return std::make_unique<rect_fills::FillMonotonic>();
    case Slic3r::ipGrid:                  return std::make_unique<rect_fills::FillGrid>();
    case Slic3r::ipTriangles:             return std::make_unique<rect_fills::FillTriangles>();
    case Slic3r::ipStars:                 return std::make_unique<rect_fills::FillStars>();
    case Slic3r::ipCubic:                 return std::make_unique<rect_fills::FillCubic>();
    case Slic3r::ipSupportBase:           return std::make_unique<rect_fills::FillSupportBase>();
    case Slic3r::ipScatteredRectilinear:  return std::make_unique<rect_fills::FillScatteredRectilinear>();
    default:                              return nullptr;
    }
}

} // namespace

TEST_CASE("Rectilinear fill port matches the core line geometry", "[plugins][infill][rectilinear]")
{
    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);

    const std::vector<PatternCase> pattern_cases = {
        { "rectilinear", Slic3r::ipRectilinear, false },
        { "alignedrectilinear", Slic3r::ipAlignedRectilinear, false },
        { "monotonic", Slic3r::ipMonotonic, true },
        { "grid", Slic3r::ipGrid, false },
        { "triangles", Slic3r::ipTriangles, false },
        { "stars", Slic3r::ipStars, false },
        { "cubic", Slic3r::ipCubic, false },
        { "supportbase", Slic3r::ipSupportBase, false },
        { "scatteredrectilinear", Slic3r::ipScatteredRectilinear, false },
    };
    const std::vector<std::string> surfaces = { "square", "hole", "lshape", "headstick", "dirtyholes" };
    const std::vector<coord_t> spacings = { scale_i(0.4), scale_i(0.8) };
    // The multi-sweep patterns (grid, triangles, stars, cubic) only support
    // sparse infill: the core asserts !params.full_infill() in that path.
    const std::vector<float> sparse_densities = { 0.15f, 0.5f };
    const std::vector<float> all_densities = { 0.15f, 1.0f };
    const std::vector<float> angles = { 0.f, float(M_PI / 6.), float(M_PI / 4.) };

    for (const PatternCase &pattern_case : pattern_cases) {
        const bool multi_sweep = pattern_case.pattern == Slic3r::ipGrid || pattern_case.pattern == Slic3r::ipTriangles ||
                                 pattern_case.pattern == Slic3r::ipStars || pattern_case.pattern == Slic3r::ipCubic;
        const bool support_base = pattern_case.pattern == Slic3r::ipSupportBase;
        const std::vector<float> &densities = multi_sweep || support_base ? sparse_densities : all_densities;
        for (const std::string &surface_name : surfaces) {
            for (const coord_t spacing : spacings) {
                for (const float density : densities) {
                    for (const float angle : angles) {
                        const std::string info = std::string(pattern_case.label) + " " + surface_name +
                            " spacing " + std::to_string(spacing) + " density " + std::to_string(density) +
                            " angle " + std::to_string(angle);

                        const Slic3r::ExPolygon core_expolygon = make_core_surface(surface_name);
                        const Slic3r::BoundingBox core_bbox = Slic3r::get_extents(core_expolygon);
                        const slic3r_api::ExPolygon ported_expolygon(
                            reinterpret_cast<const expolygon_handle *>(&core_expolygon));

                        Slic3r::FillParams core_params;
                        core_params.density = density;
                        core_params.dont_adjust = false;
                        core_params.monotonic = pattern_case.monotonic;
                        if (support_base) {
                            core_params.anchor_length = 2.f;
                            core_params.anchor_length_max = 5.f;
                        }

                        std::unique_ptr<Slic3r::Fill> core_fill(Slic3r::Fill::new_from_type(pattern_case.pattern));
                        REQUIRE(core_fill != nullptr);
                        core_fill->set_bounding_box(core_bbox);
                        core_fill->layer_id = 3;
                        core_fill->z = 0.2;
                        core_fill->angle = angle;
                        core_fill->can_angle_cross = (pattern_case.pattern == Slic3r::ipRectilinear);
                        core_fill->init_spacing(spacing, core_params);
                        Slic3r::Surface core_surface(Slic3r::stPosInternal | Slic3r::stDensSparse, core_expolygon);
                        const Slic3r::Polylines core_lines = core_fill->fill_surface(&core_surface, core_params);

                        raw_infill_pattern_params ported_params = {};
                        ported_params.density = density;
                        ported_params.infill_flags = pattern_case.monotonic ? C_INFILL_FLAG_MONOTONIC : 0;
                        ported_params.connection = C_INFILL_CONNECTION_CONNECTED;
                        ported_params.bridge_angle = -1.;
                        ported_params.fill_resolution = scale_i(0.0125);
                        ported_params.anchor_length = 1000.f;
                        ported_params.anchor_length_max = 1000.f;
                        if (support_base) {
                            ported_params.anchor_length = 2.f;
                            ported_params.anchor_length_max = 5.f;
                        }

                        std::unique_ptr<rect_fills::FillRectilinear> ported_fill = make_ported_pattern(pattern_case);
                        REQUIRE(ported_fill != nullptr);
                        ported_fill->plugin_storage = storage;
                        rect_work::BoundingBox ported_bbox;
                        ported_bbox.min = rect_work::Point(core_bbox.min.x(), core_bbox.min.y());
                        ported_bbox.max = rect_work::Point(core_bbox.max.x(), core_bbox.max.y());
                        ported_bbox.defined = true;
                        ported_fill->bounding_box = ported_bbox;
                        ported_fill->layer_id = 3;
                        ported_fill->z = 0.2;
                        ported_fill->angle = angle;
                        ported_fill->can_angle_cross = (pattern_case.pattern == Slic3r::ipRectilinear);
                        ported_fill->init_spacing(spacing, ported_params);
                        const rect_work::Polylines ported_lines = ported_fill->fill_surface(ported_expolygon, ported_params);

                        require_same_lines(core_lines, ported_lines, info);
                        if (pattern_case.monotonic)
                            require_same_directed_lines(core_lines, ported_lines, info);
                    }
                }
            }
        }
    }
}

TEST_CASE("Monotonic extrusion paths publish non-reversible flags",
          "[plugins][infill][rectilinear]")
{
    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const std::vector<c_point> points = { c_point{ 0, 0 }, c_point{ scale_i(10.), 0 } };
    raw_infill_pattern_params params = {};
    params.infill_flags = C_INFILL_FLAG_MONOTONIC;

    slic3r_api::StoredExtrusionEntity group(storage);
    slic3r_api::Infill::PatternHelpers::append_polyline(group.mutable_view(), storage, points, params);
    group.disable_sort();
    group.disable_reverse();

    REQUIRE(group.child_count() == 1);
    CHECK((group.flags() & RAW_EXTRUSION_FLAG_REVERSIBLE) == 0);
    CHECK_FALSE(group.child(0).reversible());
    CHECK(group.child(0).front().x == points.front().x);
    CHECK(group.child(0).back().x == points.back().x);
}
