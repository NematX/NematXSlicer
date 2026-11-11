///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include <catch2/catch.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <utility>
#include "libslic3r/Api/plugin/cpp/ParallelFor.hpp"
#include "libslic3r/Plugins/Common/ResidualGapFill.hpp"
#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"
#include "libslic3r/Plugins/Support/Toolpath/LegacySupportPaths.hpp"

/* Analytic SDK checks for residual geometry shared by infill and support.
   Rectangles supply known widths independently of production geometry. Inherited
   attributes exercise snapshots and publication without a slicing pipeline.

       TEST_CASE -> check: host storage encloses every stored view
           rectangle -> known printable slot
           collect_paths / coverage -> inherited dimensions and spacing
           generate -> widths, area filter, roles and cancellation
           ordered_boundary / append_additions -> old properties stay private
           modulate -> new leaf height and volume change exactly once
       TEST_CASE -> resolve_limits: record setting keys and dimensional bases

   The callback captures exceptions before returning through the C ABI. */
namespace {
using namespace slic3r_api;
using namespace slic3r_api::GapFill;

// Build one counter-clockwise rectangle in millimeters in borrowed storage.
StoredExPolygon rectangle(storage_handle *, double width, double length);
// Run geometry assertions in borrowed host storage and capture any escaping exception.
void check(uint32_t, storage_handle *, void *);
} // namespace

// Keep host storage alive throughout the checks and rethrow contained callback failures.
TEST_CASE("Residual gap fill preserves geometry, flow and ordered publication", "[plugins][gap-fill][gap-fill-core]")
{
    std::exception_ptr error;
    slic3r_parallel_for_storage(0, 1, &error, check);
    if (error)
        std::rethrow_exception(error);
}

// Record every setting lookup and verify bases, returned thresholds and scaled resolution.
TEST_CASE("Residual gap fill resolves each threshold against its dimensional base", "[plugins][gap-fill][gap-fill-core]")
{
    std::vector<std::pair<std::string, double>> calls;
    const slic3r_api::GapFill::EffectiveValue value = [&calls](const char *key, double base) {
        calls.emplace_back(key, base);
        return double(calls.size());
    };
    const double width = 0.5;
    const coord_t resolution = scale_i(0.0125);
    const slic3r_api::GapFill::Limits limits = slic3r_api::GapFill::resolve_limits(value, width, resolution);
    const std::vector<std::pair<std::string, double>> expected{
        {"gap_fill_min_width", width}, {"gap_fill_max_width", width},
        {"gap_fill_min_area", width * width}, {"gap_fill_min_length", width},
        {"gap_fill_extension", width}};
    REQUIRE(calls == expected);
    REQUIRE(limits.min_width == 1.);
    REQUIRE(limits.max_width == 2.);
    REQUIRE(limits.min_area == 3.);
    REQUIRE(limits.min_length == 4.);
    REQUIRE(limits.extension == 5.);
    REQUIRE(limits.resolution == resolution);
}

namespace {
void check(uint32_t, storage_handle *storage, void *error)
{
    try {
        c_flow flow{};
        flow.width = scale_i(0.45); flow.height = scale_i(0.2);
        flow.nozzle_diameter = scale_i(0.4); flow.spacing = scale_i(0.40708);
        flow.spacing_ratio = 1.f; flow.mm3_per_mm = 0.081416;
        StoredExtrusionEntity original(storage);
        original.get_or_add(EPropertyAttributes::key).width(0.6f).height(0.2f)
            .mm3_per_mm(0.1).extrusion_role(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE);
        original.get_or_add(EPropertyZOffset::key).set(scale_i(0.07));
        const std::vector<c_point> points{{0, 0}, {scale_i(10.), 0}};
        original.emplace_child().set_points(points);
        std::vector<Path> snapshots;
        collect_paths(original.readonly(), snapshots);
        REQUIRE(snapshots.size() == 1);
        REQUIRE(std::abs(snapshots[0].attributes.mm3_per_mm() - 0.1) < 1e-12);
        StoredPolygonCollection useful = coverage(storage, snapshots, flow.spacing_ratio, Footprint::Spacing);
        StoredPolygonCollection physical = coverage(storage, snapshots, flow.spacing_ratio, Footprint::Physical);
        double useful_area = 0., physical_area = 0.;
        for (const Polygon polygon : useful) useful_area += polygon.area();
        for (const Polygon polygon : physical) physical_area += polygon.area();
        REQUIRE(useful_area > 0.);
        REQUIRE(physical_area > useful_area);

        Limits limits;
        limits.resolution = scale_i(0.0125);
        StoredExPolygon slot = rectangle(storage, 0.3, 8.);
        StoredExPolygonCollection domain(storage, slot);
        StoredExtrusionEntity generated = generate(storage, domain, domain, flow, limits,
            RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE);
        REQUIRE(!generated.readonly().empty());
        std::vector<Path> added;
        collect_paths(generated.readonly(), added);
        REQUIRE(!added.empty());
        for (const Path &path : added) {
            const c_extrusion_property_attributes &a = path.attributes;
            REQUIRE(std::abs(a.height - 0.2f) < 1e-6);
            REQUIRE(path.attributes.extrusion_role() == RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE);
        }
        StoredExtrusionEntity infill = generate(storage, domain, domain, flow, limits, RAW_EXTRUSION_ROLE_GAP_FILL);
        std::vector<Path> infill_paths;
        collect_paths(infill.readonly(), infill_paths);
        REQUIRE(!infill_paths.empty());
        for (const Path &path : infill_paths)
            REQUIRE(path.attributes.extrusion_role() == RAW_EXTRUSION_ROLE_GAP_FILL);

        limits.min_area = 100.;
        StoredExtrusionEntity area_filtered = generate(storage, domain, domain, flow, limits, RAW_EXTRUSION_ROLE_GAP_FILL);
        REQUIRE(area_filtered.readonly().empty());
        // Cancellation must remain visible even when every component fails the area filter.
        plugin_run_context run{};
        run.is_cancelled = [](plugin_host_context *) -> int { return 1; };
        REQUIRE_THROWS(generate(storage, domain, domain, flow, limits, RAW_EXTRUSION_ROLE_GAP_FILL, &run));
        limits.min_area = 0.;
        limits.min_width = 0.4;
        StoredExtrusionEntity width_filtered = generate(storage, domain, domain, flow, limits, RAW_EXTRUSION_ROLE_GAP_FILL);
        REQUIRE(width_filtered.readonly().empty());
        limits.min_width = 0.;
        limits.max_width = 0.1;
        StoredExtrusionEntity wide_filtered = generate(storage, domain, domain, flow, limits, RAW_EXTRUSION_ROLE_GAP_FILL);
        std::vector<Path> wide_paths;
        collect_paths(wide_filtered.readonly(), wide_paths);
        REQUIRE(!wide_paths.empty());
        for (const Path &path : wide_paths) {
            const c_extrusion_property_attributes &a = path.attributes;
            const double useful_width = a.width - a.height * (1. - 3.14159265358979323846 / 4.) * flow.spacing_ratio;
            REQUIRE(useful_width <= limits.max_width + EPSILON);
            for (const c_point p : path.points)
                REQUIRE((unscaled(p.y) <= 0.3 || unscaled(p.y) >= 7.7));
        }
        limits.max_width = 0.;
        flow.is_bridge = 1;
        flow.width = flow.height = scale_i(0.4);
        StoredExtrusionEntity bridge_filtered = generate(storage, domain, domain, flow, limits,
            RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE);
        REQUIRE(bridge_filtered.readonly().empty());

        MutableExtrusionEntity queue = ordered_boundary(original.mutable_view());
        REQUIRE(queue.valid());
        REQUIRE(!original.readonly().sortable());
        REQUIRE(!original.readonly().reversible());
        REQUIRE(original.readonly().get(EPropertyAttributes::key) == nullptr);
        REQUIRE(original.readonly().get(EPropertyZOffset::key) == nullptr);
        REQUIRE(queue.readonly().get(EPropertyZOffset::key) == nullptr);
        const EPropertyZOffset *old_offset = original.readonly().child(0).get(EPropertyZOffset::key);
        REQUIRE(old_offset != nullptr);
        REQUIRE(old_offset->get() == scale_i(0.07));
        append_additions(queue, generated);
        std::vector<Path> final_paths;
        collect_paths(original.readonly(), final_paths);
        REQUIRE(final_paths.size() == snapshots.size() + added.size());
        REQUIRE(final_paths[0].points.size() == snapshots[0].points.size());
        REQUIRE(std::equal(final_paths[0].points.begin(), final_paths[0].points.end(), snapshots[0].points.begin(),
            [](c_point a, c_point b) { return a.x == b.x && a.y == b.y; }));

        // A lower slab must scale only a new leaf's height and volume, once.
        StoredExtrusionEntity modulated(storage);
        MutableExtrusionEntity leaf = modulated.emplace_child();
        leaf.set_points(std::vector<c_point>{{scale_i(0.15), scale_i(1.)}, {scale_i(0.15), scale_i(7.)}});
        leaf.get_or_add(EPropertyAttributes::key).width(0.4f).height(0.2f).mm3_per_mm(0.08)
            .extrusion_role(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE);
        std::vector<Support::Toolpath::Internal::Overlap> slabs;
        slabs.push_back({scale_to_layer_coord(0.2), Support::Common::expolygon_rings(storage, slot)});
        Support::Toolpath::Internal::modulate(storage, modulated.mutable_view(), scale_to_layer_coord(0.3), slabs, limits.resolution);
        std::vector<Path> fragments;
        collect_paths(modulated.readonly(), fragments);
        REQUIRE(!fragments.empty());
        for (const Path &fragment : fragments) {
            const c_extrusion_property_attributes &a = fragment.attributes;
            REQUIRE(std::abs(a.width - 0.4f) < 1e-6);
            REQUIRE(std::abs(a.height - 0.1f) < 1e-6);
            REQUIRE(std::abs(a.mm3_per_mm - 0.04) < 1e-7);
        }
    } catch (...) { *static_cast<std::exception_ptr *>(error) = std::current_exception(); }
}
StoredExPolygon rectangle(storage_handle *storage, double width, double length)
{
    StoredExPolygon result(storage);
    for (const c_point p : {c_point{0, 0}, c_point{scale_i(width), 0},
        c_point{scale_i(width), scale_i(length)}, c_point{0, scale_i(length)}})
        multipoint_push_back(polygon_as_multipoint(expolygon_contour(result.mutable_handle())), p);
    return result;
}
} // namespace
