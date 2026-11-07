///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef slic3r_ExtrusionGeometryUnits_hpp_
#define slic3r_ExtrusionGeometryUnits_hpp_

#include <cstddef>
#include <map>
#include <vector>

#include "ExtrusionRole.hpp"
#include "Line.hpp"

/* Supply stored geometry to collision, travel, support and preview readers.
   Collect outer continuous units, then split each into supported runs:
       for (const ExtrusionGeometryUnit &unit : collect_extrusion_geometry_units(root))
           for (const ExtrusionGeometryRun &run : extrusion_geometry_runs(unit))
               ArcPolyline geometry = extrusion_geometry_polyline(run);
   Units retain the source identity used by obstacle marking. Runs contain paths
   with effective attributes and exact compatible XY/Z endpoints; unsupported
   paths remain barriers rather than borrowing a sibling's role. These are
   geometry queries, not command execution or permission-changing operations.
   Borrowed paths and attributes require an unchanged live source tree throughout
   use. Polyline copies and preview meshes instead own their geometry; the preview
   projection applies one XY shift and carries widths, heights and destination
   indices to the GUI. Allocation failures propagate without changing the source.
*/

namespace Slic3r {

class ArcPolyline;
class ExtrusionEntity;
struct ExtrusionAttributes;

// References remain valid while the source tree, paths and properties stay unchanged.
// A missing attribute is retained so consumers can stop rather than borrow a sibling's role.
struct ExtrusionGeometryLeaf
{
    const ExtrusionEntity *entity;
    const ArcPolyline *polyline;
    const ExtrusionAttributes *attributes;
};

// One outer continuous, unsortable subtree, or one local path, with no invented joins.
struct ExtrusionGeometryUnit
{
    const ExtrusionEntity *root;
    std::vector<ExtrusionGeometryLeaf> leaves;
};

// Segment-bearing paths with attributes and exact compatible XY/Z joins.
// Several separate runs may belong to the same unit and share its identity.
struct ExtrusionGeometryRun
{
    const ExtrusionEntity *unit_root;
    std::vector<ExtrusionGeometryLeaf> leaves;
};

// Owned CPU inputs for one preview mesh; destination indexes the caller's geometry list.
struct ExtrusionGeometryMesh
{
    Lines lines;
    std::vector<double> widths;
    std::vector<double> heights;
    size_t destination = 0;
    bool closed = false;
};

// Sum a contour's signed XY area in scaled coordinates and stored segment order.
// Paths must contain at least two points. Attributes are irrelevant, arcs use
// their current tessellation, and no last-to-first edge or repair is added.
double extrusion_geometry_area(const ExtrusionEntity &root);

// Prepare preview lines with inherited widths/heights and one XY translation.
// Missing roles use destination zero; destination changes and run barriers split meshes.
// The source is unchanged, closure is exact XY equality, and allocation failures propagate.
std::vector<ExtrusionGeometryMesh> project_extrusion_geometry(
    const ExtrusionEntity &root, const Point &shift,
    const std::map<GCodeExtrusionRole, size_t> &destinations = {});

// Borrow paths in source order with their nearest inherited attributes. Property-only
// commands contribute no geometry; the source must stay immutable and allocations may throw.
std::vector<ExtrusionGeometryUnit> collect_extrusion_geometry_units(const ExtrusionEntity &root);

// Split at missing attributes, point-only paths and incompatible XY/Z joins.
// Missing Z samples mean zero; no points or 3D transitions are fabricated.
std::vector<ExtrusionGeometryRun> extrusion_geometry_runs(const ExtrusionGeometryUnit &unit);

// Aggregate only a supported run's roles, ignoring Travel when another role exists.
// This analysis query never changes the source nodes' role or attributes.
ExtrusionRole extrusion_geometry_role(const ExtrusionGeometryRun &run);

// Copy a supported run without epsilon merging, preserving arcs and local Z samples.
// Source handles and properties stay unchanged; allocation failures propagate.
ArcPolyline extrusion_geometry_polyline(const ExtrusionGeometryRun &run);

} // namespace Slic3r

#endif
