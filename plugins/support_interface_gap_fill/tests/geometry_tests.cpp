///|/ Released under the GNU Affero General Public License, version 3 or later.
#include "InterfaceGapFill.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"
#include "libslic3r/Plugins/Support/Toolpath/LegacySupportPaths.hpp"
#ifdef _WIN32
#include <crtdbg.h>
#endif

/* Analytic SDK checks for support residual geometry. Rectangles supply known
   widths independently of the production booleans. An inherited flow exercises
   snapshots and ordered insertion without depending on the slicing pipeline.

       main -> check (host-owned storage callback)
           collect_paths / coverage: inherited effective dimensions
           generate: printable, narrow, area-filtered and bridge residuals

   The callback captures exceptions instead of letting them cross the C ABI. */
namespace {
using namespace nematx::interface_gaps;
// Fail in Release too; a failed predicate is never just a debug assertion.
void require(bool condition, const char *message);
// Build one counter-clockwise rectangle in millimeter coordinates.
StoredExPolygon rectangle(storage_handle *, double width, double length);
// Run analytic checks in borrowed storage and capture any exception.
void check(uint32_t, storage_handle *, void *);

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

StoredExPolygon rectangle(storage_handle *storage, double width, double length)
{
    StoredExPolygon result(storage);
    for (const c_point p : {c_point{0, 0}, c_point{scale_i(width), 0},
        c_point{scale_i(width), scale_i(length)}, c_point{0, scale_i(length)}})
        multipoint_push_back(polygon_as_multipoint(expolygon_contour(result.mutable_handle())), p);
    return result;
}

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
        require(snapshots.size() == 1, "Inherited path was lost");
        require(std::abs(snapshots[0].attributes.mm3_per_mm() - 0.1) < 1e-12, "Inherited volume changed");
        StoredPolygonCollection useful = coverage(storage, snapshots, flow);
        StoredPolygonCollection physical = coverage(storage, snapshots, flow, true);
        double useful_area = 0., physical_area = 0.;
        for (const Polygon polygon : useful) useful_area += polygon.area();
        for (const Polygon polygon : physical) physical_area += polygon.area();
        require(useful_area > 0. && physical_area > useful_area, "Coverage did not distinguish spacing from width");

        Limits limits;
        limits.resolution = scale_i(0.0125);
        StoredExPolygon slot = rectangle(storage, 0.3, 8.);
        StoredExPolygonCollection domain(storage, slot);
        StoredExtrusionEntity generated = generate(storage, domain, domain, flow, limits);
        require(!generated.readonly().empty(), "Printable slot produced no gap fill");
        std::vector<Path> added;
        collect_paths(generated.readonly(), added);
        require(!added.empty(), "Gap fill has no printable leaves");
        for (const Path &path : added) {
            const c_extrusion_property_attributes &a = path.attributes;
            require(std::abs(a.height - 0.2f) < 1e-6, "SDK changed nominal height");
            require(path.attributes.extrusion_role() == RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE, "Wrong gap fill role");
        }
        limits.min_area = 100.;
        StoredExtrusionEntity area_filtered = generate(storage, domain, domain, flow, limits);
        require(area_filtered.readonly().empty(), "Area threshold ignored");
        limits.min_area = 0.;
        limits.min_width = 0.4;
        StoredExtrusionEntity width_filtered = generate(storage, domain, domain, flow, limits);
        require(width_filtered.readonly().empty(), "Minimum width ignored");
        limits.min_width = 0.;
        limits.max_width = 0.1;
        StoredExtrusionEntity wide_filtered = generate(storage, domain, domain, flow, limits);
        std::vector<Path> wide_paths;
        collect_paths(wide_filtered.readonly(), wide_paths);
        require(!wide_paths.empty(), "Width-limit fixture did not exercise retained narrow tips");
        for (const Path &path : wide_paths) {
            const c_extrusion_property_attributes &a = path.attributes;
            const double useful_width = a.width - a.height * (1. - 3.14159265358979323846 / 4.) * flow.spacing_ratio;
            require(useful_width <= limits.max_width + EPSILON, "Maximum useful width was exceeded");
            for (const c_point p : path.points)
                require(unscaled(p.y) <= 0.3 || unscaled(p.y) >= 7.7, "Width filter filled the wide central span");
        }
        limits.max_width = 0.;
        flow.is_bridge = 1;
        flow.width = flow.height = scale_i(0.4);
        StoredExtrusionEntity bridge_filtered = generate(storage, domain, domain, flow, limits);
        require(bridge_filtered.readonly().empty(), "Narrow bridge diameter was reduced");

        MutableExtrusionEntity queue = original.emplace_ordered_leaf(OrderedLeafPosition::After,
            ExistingPropertyPlacement::MoveWithExistingContent);
        require(queue.valid(), "Ordered insertion rejected");
        require(!original.readonly().sortable() && !original.readonly().reversible(), "Outer boundary can reorder paths");
        require(original.readonly().get(EPropertyAttributes::key) == nullptr, "Old attributes leaked onto the new boundary");
        require(original.readonly().get(EPropertyZOffset::key) == nullptr && queue.readonly().get(EPropertyZOffset::key) == nullptr,
            "Old private property leaked onto new gap fills");
        const EPropertyZOffset *old_offset = original.readonly().child(0).get(EPropertyZOffset::key);
        require(old_offset && old_offset->get() == scale_i(0.07), "Old private property was lost during insertion");
        require(queue.append_child_move(generated.mutable_view()) != EXTRUSION_INDEX_INVALID, "Cannot append generated tree");
        std::vector<Path> final_paths;
        collect_paths(original.readonly(), final_paths);
        require(final_paths.size() == snapshots.size() + added.size(), "Publication lost or duplicated paths");
        require(final_paths[0].points.size() == snapshots[0].points.size() &&
            std::equal(final_paths[0].points.begin(), final_paths[0].points.end(), snapshots[0].points.begin(),
                [](c_point a, c_point b) { return a.x == b.x && a.y == b.y; }), "Old path changed during insertion");

        // A lower slab must scale only a new leaf's height and volume, once.
        StoredExtrusionEntity modulated(storage);
        MutableExtrusionEntity leaf = modulated.emplace_child();
        leaf.set_points(std::vector<c_point>{{scale_i(0.15), scale_i(1.)}, {scale_i(0.15), scale_i(7.)}});
        leaf.get_or_add(EPropertyAttributes::key).width(0.4f).height(0.2f).mm3_per_mm(0.08)
            .extrusion_role(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE);
        std::vector<slic3r_api::Support::Toolpath::Internal::Overlap> slabs;
        slabs.push_back({scale_to_layer_coord(0.2), slic3r_api::Support::Common::expolygon_rings(storage, slot)});
        slic3r_api::Support::Toolpath::Internal::modulate(storage, modulated.mutable_view(),
            scale_to_layer_coord(0.3), slabs, limits.resolution);
        std::vector<Path> fragments;
        collect_paths(modulated.readonly(), fragments);
        require(!fragments.empty(), "Modulation lost the new path");
        for (const Path &fragment : fragments) {
            const c_extrusion_property_attributes &a = fragment.attributes;
            require(std::abs(a.width - 0.4f) < 1e-6, "Modulation changed width");
            require(std::abs(a.height - 0.1f) < 1e-6, "Modulation used the wrong slab height");
            require(std::abs(a.mm3_per_mm - 0.04) < 1e-7, "Modulation did not scale volume exactly once");
        }
    } catch (...) { *static_cast<std::exception_ptr *>(error) = std::current_exception(); }
}
}

// Host storage lifetime encloses every check; report a contained failure as nonzero.
int main()
{
#ifdef _WIN32
    _set_error_mode(_OUT_TO_STDERR);
    _set_abort_behavior(0, _CALL_REPORTFAULT);
#endif
    std::exception_ptr error;
    slic3r_parallel_for_storage(0, 1, &error, check);
    try {
        if (error) std::rethrow_exception(error);
        std::cout << "Support interface gap fill geometry checks passed\n";
        return 0;
    } catch (const std::exception &failure) {
        std::cerr << failure.what() << '\n';
        return 1;
    }
}
