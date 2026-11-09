///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include "ExtrusionGeometryUnits.hpp"

#include <algorithm>
#include <cassert>
#include <optional>
#include <utility>

#include "ExtrusionEntity.hpp"
#include "PointUtils.hpp"

/* Project stored extrusion geometry for read-only collision, travel, support and
   preview analyses. Attribute inheritance is restored between siblings. The
   outer continuous unit keeps its identity; supported runs split at missing
   attributes, point-only paths and incompatible XY/Z joins. Commands without
   stored geometry add no points and no implicit legacy Nop position is invented.

   The source owns all borrowed nodes, paths and attributes and must remain
   unchanged while these views are used. Polyline copies preserve arcs and local
   Z samples. Preview meshes own tessellated XY lines with widths and heights;
   their requested XY translation is applied exactly once. No routine consumes
   permissions or edits properties. Allocation failures propagate to the caller.

       project_extrusion_geometry()
           |-- collect_extrusion_geometry_units()
           |   `-- append_geometry_units() [resolve attributes per source branch]
           |-- extrusion_geometry_runs() [retain supported exact joins only]
           `-- tessellate paths into separate destination meshes
       extrusion_geometry_polyline() [copy one supported run, retaining arc/Z data]
       extrusion_geometry_area()
           `-- accumulate_contour_area() [sum stored segments, never add a closing edge]
*/

namespace Slic3r {
namespace {

// Sum stored paths into one accumulator without filtering attributes or joining paths.
void accumulate_contour_area(const ExtrusionEntity &entity, double &area);

// Keep the outer continuous unit while restoring attributes naturally between siblings.
void append_geometry_units(const ExtrusionEntity &entity, const ExtrusionAttributes *inherited,
                           ExtrusionGeometryUnit *active, std::vector<ExtrusionGeometryUnit> &out);

} // namespace

double extrusion_geometry_area(const ExtrusionEntity &root)
{
    double area = 0.;
    accumulate_contour_area(root, area);
    return area * 0.5;
}

std::vector<ExtrusionGeometryMesh> project_extrusion_geometry(
    const ExtrusionEntity &root, const Point &shift,
    const std::map<GCodeExtrusionRole, size_t> &destinations)
{
    std::vector<ExtrusionGeometryMesh> meshes;
    for (const ExtrusionGeometryUnit &unit : collect_extrusion_geometry_units(root)) {
        for (const ExtrusionGeometryRun &run : extrusion_geometry_runs(unit)) {
            ExtrusionGeometryMesh mesh;
            // Finish one destination without joining across unsupported geometry or run boundaries.
            const auto flush = [&]() {
                if (!mesh.lines.empty()) {
                    mesh.closed = mesh.lines.front().a == mesh.lines.back().b;
                    meshes.emplace_back(std::move(mesh));
                    mesh = ExtrusionGeometryMesh();
                }
            };
            for (const ExtrusionGeometryLeaf &leaf : run.leaves) {
                const std::map<GCodeExtrusionRole, size_t>::const_iterator found =
                    destinations.find(extrusion_role_to_gcode_extrusion_role(leaf.attributes->extrusion_role()));
                const size_t destination = found == destinations.end() ? 0 : found->second;
                if (destination != mesh.destination)
                    flush();
                mesh.destination = destination;
                Polyline polyline = leaf.polyline->to_polyline();
                polyline.remove_duplicate_points();
                if (polyline.points.size() < 2) {
                    flush();
                    continue;
                }
                polyline.translate(shift);
                const Lines lines = polyline.lines();
                mesh.lines.insert(mesh.lines.end(), lines.begin(), lines.end());
                mesh.widths.insert(mesh.widths.end(), lines.size(), leaf.attributes->width);
                mesh.heights.insert(mesh.heights.end(), lines.size(), leaf.attributes->height);
            }
            flush();
        }
    }
    return meshes;
}

std::vector<ExtrusionGeometryUnit> collect_extrusion_geometry_units(const ExtrusionEntity &root)
{
    std::vector<ExtrusionGeometryUnit> units;
    append_geometry_units(root, nullptr, nullptr, units);
    units.erase(std::remove_if(units.begin(), units.end(), [](const ExtrusionGeometryUnit &unit) {
        return unit.leaves.empty();
    }), units.end());
    return units;
}

std::vector<ExtrusionGeometryRun> extrusion_geometry_runs(const ExtrusionGeometryUnit &unit)
{
    std::vector<ExtrusionGeometryRun> runs;
    bool separated = true;
    for (const ExtrusionGeometryLeaf &leaf : unit.leaves) {
        if (leaf.attributes == nullptr || leaf.polyline->size() < 2) {
            separated = true;
            continue;
        }
        if (!separated) {
            const ArcPolyline &previous = *runs.back().leaves.back().polyline;
            const coord_t previous_z = previous.has_z_offset() ? previous.z_offset(previous.size() - 1) : 0;
            const coord_t current_z = leaf.polyline->has_z_offset() ? leaf.polyline->z_offset(0) : 0;
            separated = previous.back() != leaf.polyline->front() || previous_z != current_z;
        }
        if (separated)
            runs.push_back({unit.root, {}});
        runs.back().leaves.push_back(leaf);
        separated = false;
    }
    return runs;
}

ExtrusionRole extrusion_geometry_role(const ExtrusionGeometryRun &run)
{
    std::optional<ExtrusionRole> role;
    bool travel = false;
    for (const ExtrusionGeometryLeaf &leaf : run.leaves) {
        assert(leaf.attributes != nullptr);
        const ExtrusionRole current = leaf.attributes->extrusion_role();
        if (current == ExtrusionRole::Travel) {
            travel = true;
            continue;
        }
        if (!role)
            role = current;
        else if (*role != current)
            return ExtrusionRole::Mixed;
    }
    return role ? *role : travel ? ExtrusionRole::Travel : ExtrusionRole::None;
}

ArcPolyline extrusion_geometry_polyline(const ExtrusionGeometryRun &run)
{
    ArcPolyline result;
    for (const ExtrusionGeometryLeaf &leaf : run.leaves) {
        assert(leaf.attributes != nullptr && leaf.polyline->size() >= 2);
        ArcPolyline copy = *leaf.polyline;
        result.append(std::move(copy));
    }
    return result;
}

namespace {

void accumulate_contour_area(const ExtrusionEntity &entity, double &area)
{
    if (const ArcPolyline *path = entity.polyline_or_null()) {
        assert(path->size() >= 2);
        if (path->size() < 2)
            return;
        if (path->has_arc()) {
            const Polyline polyline = path->to_polyline();
            Point previous = polyline.front();
            for (size_t idx = 1; idx < polyline.size(); ++idx) {
                const Point &current = polyline[idx];
                area += cross2(previous.cast<double>(), current.cast<double>());
                previous = current;
            }
        } else {
            Point previous = path->front();
            for (size_t idx = 1; idx < path->size(); ++idx) {
                const Point &current = path->get_point(idx);
                area += cross2(previous.cast<double>(), current.cast<double>());
                previous = current;
            }
        }
        return;
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        if (child)
            accumulate_contour_area(*child, area);
}

void append_geometry_units(const ExtrusionEntity &entity, const ExtrusionAttributes *inherited,
                           ExtrusionGeometryUnit *active, std::vector<ExtrusionGeometryUnit> &out)
{
    const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
    if (attributes == nullptr)
        attributes = inherited;
    if (const ArcPolyline *polyline = entity.polyline_or_null()) {
        if (polyline->empty())
            return;
        if (active == nullptr) {
            out.push_back({&entity, {}});
            active = &out.back();
        }
        active->leaves.push_back({&entity, polyline, attributes});
        return;
    }
    if (active == nullptr && entity.ExtrusionEntity::is_continuous()) {
        out.push_back({&entity, {}});
        active = &out.back();
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        if (child)
            append_geometry_units(*child, attributes, active, out);
}

} // namespace
} // namespace Slic3r
