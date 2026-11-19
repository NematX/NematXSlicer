///|/ Released under the GNU Affero General Public License, version 3 or later.
#include "InterfacePerimeters.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>
#ifdef _WIN32
#include <crtdbg.h>
#endif

#include "libslic3r/Plugins/Support/Common/SupportPaths.hpp"

/* Public-SDK geometry checks use analytic rectangles rather than the production
   offset routine as their oracle. A probe pattern records its received area and
   supplies distinct attributes and an inherited Z property; the wrapper must
   preserve both. Thin and split areas exercise termination and branch ordering.

       main -> storage_callback: contain exceptions at the C callback boundary
                 test_rectangles / test_split / test_thin / test_failure
                   fill_interface -> probe -> inspect complete output tree */

namespace {
using namespace slic3r_api;
using nematx::support_interfaces::Settings;
using nematx::support_interfaces::fill_interface;
namespace Common = slic3r_api::Support::Common;

// Throw a visible test failure rather than relying on assertions disabled in Release.
void require(bool condition, const char *message);
// Construct millimeter coordinates and a storage-owned counter-clockwise rectangle.
c_point point(double x, double y);
StoredExPolygon rectangle(storage_handle *storage, double x0, double y0, double x1, double y1);
// Analytic bounds of one ring, expressed in millimeters.
std::vector<double> bounds(const std::vector<c_point> &points);
// Read contour leaves and inherited marker properties without changing the tree.
void inspect(const ExtrusionEntity &entity, std::vector<std::vector<c_point>> &loops, size_t &patterns, bool marker = false);
// Compare expected centers and inner areas on a rectangle, optionally with one rectangular hole.
void test_rectangles(storage_handle *storage);
// Check independent branches when a narrow neck disappears during the first offset.
void test_split(storage_handle *storage);
// Stop when the current shell cannot contain a centerline, without invoking the pattern.
void test_thin(storage_handle *storage);
// A failing inner pattern must leave the caller's destination untouched.
void test_failure(storage_handle *storage);
// Catch every exception before returning to the host's C parallel-for callback.
void storage_callback(uint32_t, storage_handle *storage, void *result);

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

c_point point(double x, double y) { return {scale_i(x), scale_i(y)}; }

StoredExPolygon rectangle(storage_handle *storage, double x0, double y0, double x1, double y1)
{
    StoredExPolygon area(storage);
    for (const c_point p : {point(x0, y0), point(x1, y0), point(x1, y1), point(x0, y1)})
        multipoint_push_back(polygon_as_multipoint(expolygon_contour(area.mutable_handle())), p);
    return area;
}

std::vector<double> bounds(const std::vector<c_point> &points)
{
    std::vector<double> box{1e6, 1e6, -1e6, -1e6};
    for (const c_point p : points) {
        box[0] = std::min(box[0], unscaled(p.x));
        box[1] = std::min(box[1], unscaled(p.y));
        box[2] = std::max(box[2], unscaled(p.x));
        box[3] = std::max(box[3], unscaled(p.y));
    }
    return box;
}

void inspect(const ExtrusionEntity &entity, std::vector<std::vector<c_point>> &loops, size_t &patterns, bool marker)
{
    if (const EPropertyZOffset *property = entity.get(EPropertyZOffset::key)) {
        require(property->get() == scale_i(0.07), "inner pattern Z property changed");
        marker = true;
    }
    if (entity.has_polyline()) {
        const EPropertyAttributes *attributes = entity.get(EPropertyAttributes::key);
        require(attributes != nullptr, "leaf attributes missing");
        if (marker) {
            require(std::abs(attributes->mm3_per_mm() - 0.123) < 1e-9, "inner pattern flow was overwritten");
            const c_extrusion_property_attributes &raw = *attributes;
            require(std::abs(raw.width - 0.7f) < 1e-6 && std::abs(raw.height - 0.25f) < 1e-6,
                "inner pattern dimensions were overwritten");
            ++patterns;
        } else {
            require(std::abs(attributes->mm3_per_mm() - 0.18) < 1e-9, "perimeter flow changed");
            require(attributes->extrusion_role() == RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE, "perimeter role changed");
            const std::vector<c_point> points = entity.points();
            require(points.size() >= 4 && points.front().x == points.back().x && points.front().y == points.back().y, "open contour");
            loops.push_back(points);
        }
    }
    for (uint32_t index = 0; index < entity.child_count(); ++index)
        inspect(entity.child(index), loops, patterns, marker);
}

void test_rectangles(storage_handle *storage)
{
    for (const bool hole : {false, true}) for (const unsigned count : {0u, 1u, 2u, 3u}) for (const float overlap : {0.f, 0.25f, 0.5f}) {
        StoredExPolygon area = rectangle(storage, 0., 0., 20., 20.);
        if (hole) {
            area.holes_resize(1);
            for (const c_point p : {point(8., 8.), point(8., 12.), point(12., 12.), point(12., 8.)})
                multipoint_push_back(polygon_as_multipoint(expolygon_hole_at(area.mutable_handle(), 0)), p);
        }
        Settings settings;
        settings.count = count;
        settings.spacing = 1.;
        settings.encroachment = overlap;
        settings.role = RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
        settings.flow.width = scale_i(1.);
        settings.flow.height = scale_i(0.2);
        settings.flow.mm3_per_mm = 0.18;
        size_t calls = 0;
        const Common::InnerFill inner = [&](const surface_handle *surface, MutableExtrusionEntity output) {
            ++calls;
            const ExPolygon inset = Surface(surface).expolygon();
            const std::vector<double> box = bounds(inset.contour().points());
            const double expected = count == 0 ? 0. : double(count) - overlap;
            require(std::abs(box[0] - expected) < 1e-5 && std::abs(box[2] - (20. - expected)) < 1e-5, "wrong terminal inner domain");
            if (hole) {
                require(inset.hole_size() == 1, "inner hole lost");
                const std::vector<double> hole_box = bounds(inset.hole(0).points());
                require(std::abs(hole_box[0] - (8. - expected)) < 1e-5, "wrong terminal hole domain");
            }
            output.get_or_add(EPropertyZOffset::key).set(scale_i(0.07));
            StoredExtrusionEntity path(storage, std::vector<c_point>{point(9., 4.), point(11., 4.)});
            path.get_or_add(EPropertyAttributes::key).mm3_per_mm(0.123).width(0.7f).height(0.25f);
            output.append_child_move(path.mutable_view());
            output.set_flags(0u);
        };
        StoredExtrusionEntity output(storage);
        fill_interface(storage, Surface(area.readonly(), RAW_SURFACE_TYPE_DENS_SOLID), settings, inner, output.mutable_view());
        std::vector<std::vector<c_point>> loops;
        size_t patterns = 0;
        inspect(output.readonly(), loops, patterns);
        require(loops.size() == count * (hole ? 2u : 1u), "wrong number of perimeter levels");
        require(calls == 1 && patterns == 1, "inner pattern missing or duplicated");
        for (unsigned level = 0; level < count; ++level) {
            const std::vector<double> outer = bounds(loops[level * (hole ? 2 : 1)]);
            require(std::abs(outer[0] - (0.5 + level)) < 1e-5, "wrong outer centerline");
            const std::vector<c_point> &ring = loops[level * (hole ? 2 : 1)];
            double signed_area = 0.;
            for (size_t index = 1; index < ring.size(); ++index)
                signed_area += double(ring[index - 1].x) * ring[index].y - double(ring[index].x) * ring[index - 1].y;
            require(signed_area > 0., "outer contour orientation changed");
            if (hole) {
                const std::vector<double> inner_box = bounds(loops[level * 2 + 1]);
                require(std::abs(inner_box[0] - (7.5 - level)) < 1e-5, "wrong hole centerline");
                const std::vector<c_point> &hole_ring = loops[level * 2 + 1];
                double hole_signed_area = 0.;
                for (size_t index = 1; index < hole_ring.size(); ++index)
                    hole_signed_area += double(hole_ring[index - 1].x) * hole_ring[index].y - double(hole_ring[index].x) * hole_ring[index - 1].y;
                require(hole_signed_area < 0., "hole orientation changed");
            }
        }
    }
}

void test_split(storage_handle *storage)
{
    StoredExPolygon area(storage);
    for (const c_point p : {point(0, 0), point(8, 0), point(8, 3.9), point(12, 3.9), point(12, 0), point(20, 0),
                           point(20, 8), point(12, 8), point(12, 4.1), point(8, 4.1), point(8, 8), point(0, 8)})
        multipoint_push_back(polygon_as_multipoint(expolygon_contour(area.mutable_handle())), p);
    Settings settings;
    settings.count = 2;
    settings.spacing = 1.;
    settings.role = RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
    settings.flow.mm3_per_mm = 0.18;
    size_t calls = 0;
    StoredExtrusionEntity output(storage);
    fill_interface(storage, Surface(area.readonly(), RAW_SURFACE_TYPE_DENS_SOLID), settings,
        [&](const surface_handle *, MutableExtrusionEntity) { ++calls; }, output.mutable_view());
    std::vector<std::vector<c_point>> loops;
    size_t patterns = 0;
    inspect(output.readonly(), loops, patterns);
    require(loops.size() == 4 && calls == 2, "split islands lost contours or inner fill");
    const ExtrusionEntity branch = output.readonly().child(0).child(0);
    require(!branch.sortable(), "contours and inner fill can be reordered");
}

void test_thin(storage_handle *storage)
{
    // The quarter-spacing regularization first erodes by 0.75 on each side;
    // growing it back cannot restore an erased shell. Further levels consume two spacings of width.
    const std::vector<std::pair<double, size_t>> cases{{0.2, 0}, {1.2, 0}, {1.49, 0}, {1.51, 1}, {2.2, 1}, {3.49, 1}, {3.51, 2}};
    for (const std::pair<double, size_t> &test : cases) {
        const double width = test.first;
        StoredExPolygon area = rectangle(storage, 0., 0., width, 10.);
        Settings settings;
        settings.count = 3;
        settings.spacing = 1.;
        settings.flow.mm3_per_mm = 0.18;
        settings.role = RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
        size_t calls = 0;
        StoredExtrusionEntity output(storage);
        fill_interface(storage, Surface(area.readonly(), RAW_SURFACE_TYPE_DENS_SOLID), settings,
            [&](const surface_handle *, MutableExtrusionEntity) { ++calls; }, output.mutable_view());
        std::vector<std::vector<c_point>> loops;
        size_t patterns = 0;
        inspect(output.readonly(), loops, patterns);
        require(loops.size() == test.second, "thin branch did not stop at available contours");
        require(calls == 0, "unreachable inner pattern was called");
    }
}

void test_failure(storage_handle *storage)
{
    StoredExPolygon area = rectangle(storage, 0., 0., 10., 10.);
    Settings settings;
    settings.count = 1;
    settings.spacing = 1.;
    StoredExtrusionEntity output(storage);
    bool failed = false;
    try {
        fill_interface(storage, Surface(area.readonly(), RAW_SURFACE_TYPE_DENS_SOLID), settings,
            [](const surface_handle *, MutableExtrusionEntity) { throw std::runtime_error("probe failure"); }, output.mutable_view());
    } catch (const std::runtime_error &error) { failed = std::string(error.what()) == "probe failure"; }
    require(failed && output.empty(), "pattern error was hidden or partial paths were published");
}

void storage_callback(uint32_t, storage_handle *storage, void *result)
{
    try {
        test_rectangles(storage);
        test_split(storage);
        test_thin(storage);
        test_failure(storage);
    } catch (...) { *static_cast<std::exception_ptr *>(result) = std::current_exception(); }
}
}

// Run the SDK checks with host-owned temporary storage and report failures as a nonzero exit.
int main()
{
#ifdef _WIN32
    _set_error_mode(_OUT_TO_STDERR);
    _set_abort_behavior(0, _CALL_REPORTFAULT);
#endif
    try {
        std::exception_ptr error;
        slic3r_parallel_for_storage(0, 1, &error, storage_callback);
        if (error) std::rethrow_exception(error);
        std::cout << "Support interface geometry checks passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
