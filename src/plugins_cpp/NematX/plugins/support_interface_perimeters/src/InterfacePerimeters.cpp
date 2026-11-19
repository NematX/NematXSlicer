///|/ Copyright (c) Prusa Research 2016 - 2023 Vojtěch Bubník, Lukáš Hejl, Tomáš Mészáros, Lukáš Matěna
///|/ Copyright (c) SuperSlicer 2018 - 2019 Remi Durand
///|/ Released under the GNU Affero General Public License, version 3 or later.
#include "InterfacePerimeters.hpp"

#include <algorithm>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"
#include "libslic3r/Plugins/Support/Common/SupportPaths.hpp"

/* Generate support interface contours from the remaining printable shell.
   Half-spacing offsets place contour centerlines; a second half-spacing inset
   reserves their footprint before the next contour. The terminal shell is
   expanded for the configured overlap and passed to the original pattern.
   Pattern entities move intact, including inherited attributes and ordering.
   Splits introduce sortable island groups, never sortable contour/fill branches.
   All temporaries belong to the caller's run storage; no print state is cached.

       fill_interface -> add_shells: group independent islands
                          add_shell: emit one contour level or the terminal fill
                            add_centers: reserve contour footprints and recurse
                            inner: invoke the caller's pattern on each inner area */

namespace nematx::support_interfaces {
namespace {
using namespace slic3r_api;
namespace Common = slic3r_api::Support::Common;

struct FillContext {
    storage_handle *storage;
    const Settings &settings;
    const Common::InnerFill &inner;
    raw_surface_type type;
};

// Close a copied ring and give contours and holes their respective orientation.
std::vector<c_point> loop_points(const Polygon &polygon, bool counter_clockwise);
// Move a nonempty temporary tree, preserving its root properties and flags.
void append_nonempty(MutableExtrusionEntity parent, StoredExtrusionEntity &child);
// Keep a fixed branch per island; only independent islands may be reordered.
void add_shells(const FillContext &context, const ExPolygonCollection &shells, unsigned int remaining,
                MutableExtrusionEntity parent, bool wrap_single);
// Add one level of contours or pass the terminal shell to the original pattern.
void add_shell(const FillContext &context, const ExPolygon &shell, unsigned int remaining, MutableExtrusionEntity branch);
// Emit a centerline's contour and holes, then recurse on the remaining printable shell.
void add_centers(const FillContext &context, const ExPolygon &center, unsigned int remaining, MutableExtrusionEntity branch);

std::vector<c_point> loop_points(const Polygon &polygon, bool counter_clockwise)
{
    std::vector<c_point> points = polygon.points();
    if (polygon.is_counter_clockwise() != counter_clockwise)
        std::reverse(points.begin(), points.end());
    if (!points.empty()) points.push_back(points.front());
    return points;
}

void append_nonempty(MutableExtrusionEntity parent, StoredExtrusionEntity &child)
{
    if (!child.empty()) parent.append_child_move(child.mutable_view());
}

void add_shells(const FillContext &context, const ExPolygonCollection &shells, unsigned int remaining,
                MutableExtrusionEntity parent, bool wrap_single)
{
    if (shells.empty()) return;
    if (shells.size() == 1 && !wrap_single) {
        add_shell(context, shells.at(0), remaining, parent);
        return;
    }
    StoredExtrusionEntity islands(context.storage);
    for (const ExPolygon shell : shells) {
        StoredExtrusionEntity branch(context.storage);
        add_shell(context, shell, remaining, branch.mutable_view());
        branch.set_flags(0u);
        append_nonempty(islands.mutable_view(), branch);
    }
    if (shells.size() == 1) {
        Common::move_children(parent, islands.mutable_view());
    } else {
        islands.set_flags(RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);
        append_nonempty(parent, islands);
    }
}

void add_shell(const FillContext &context, const ExPolygon &shell, unsigned int remaining, MutableExtrusionEntity branch)
{
    const Settings &settings = context.settings;
    const ClipperContext clipper(context.storage);
    if (remaining == 0) {
        const double delta = -settings.spacing * (0.5 - std::clamp(settings.encroachment, 0.f, 0.5f));
        StoredExPolygonCollection areas = clipper_offset2(clipper(shell), scale_d(delta), scale_d(settings.spacing / 2.)).to_expolygon_collection();
        areas.ensure_valid();
        for (const ExPolygon area : areas) {
            StoredSurfaceCollection surfaces(context.storage);
            surfaces.append(area, context.type);
            StoredExtrusionEntity pattern(context.storage);
            context.inner(surface_collection_at(surfaces.handle(), 0), pattern.mutable_view());
            append_nonempty(branch, pattern);
        }
        return;
    }

    const double half = scale_d(settings.spacing / 2.);
    const coord_t regularization = coord_t(scale_d(settings.spacing / 4.));
    const StoredExPolygonCollection rounded = clipper_offset2(clipper(shell), -half - regularization,
        double(regularization), CLIPPER_JOIN_MITER, scale_d(settings.spacing) * 10.).to_expolygon_collection();
    const StoredExPolygonCollection limit = clipper_offset(clipper(shell), -half).to_expolygon_collection();
    StoredExPolygonCollection centers = Common::native_expolygons(clipper_intersection(clipper(rounded.readonly()), clipper(limit.readonly())));
    centers.ensure_valid();
    if (centers.size() == 1) {
        add_centers(context, centers.at(0), remaining, branch);
        return;
    }
    StoredExtrusionEntity islands(context.storage);
    for (const ExPolygon center : centers) {
        StoredExtrusionEntity island(context.storage);
        add_centers(context, center, remaining, island.mutable_view());
        island.set_flags(0u);
        append_nonempty(islands.mutable_view(), island);
    }
    islands.set_flags(RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);
    append_nonempty(branch, islands);
}

void add_centers(const FillContext &context, const ExPolygon &center, unsigned int remaining, MutableExtrusionEntity branch)
{
    const Settings &settings = context.settings;
    StoredExtrusionEntity loops(context.storage);
    const EPropertyAttributes attributes = Common::path_attributes(settings.role, settings.flow);
    Common::append_paths(context.storage, loops.mutable_view(), {loop_points(center.contour(), true)}, attributes, !settings.monotonic);
    for (const Polygon hole : center.holes())
        Common::append_paths(context.storage, loops.mutable_view(), {loop_points(hole, false)}, attributes, !settings.monotonic);
    loops.set_flags(RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);
    append_nonempty(branch, loops);
    const ClipperContext clipper(context.storage);
    StoredExPolygonCollection shells = clipper_offset(clipper(center), scale_d(-settings.spacing / 2.)).to_expolygon_collection();
    shells.ensure_valid();
    add_shells(context, shells, remaining - 1, branch, false);
}
}

void fill_interface(storage_handle *storage, const slic3r_api::Surface &surface, const Settings &settings,
                    const slic3r_api::Support::Common::InnerFill &inner, slic3r_api::MutableExtrusionEntity output)
{
    using namespace slic3r_api;
    if (settings.count == 0) {
        StoredSurfaceCollection surfaces(storage);
        surfaces.append(surface.expolygon(), surface.type());
        inner(surface_collection_at(surfaces.handle(), 0), output);
        return;
    }
    const FillContext context{storage, settings, inner, surface.type()};
    const StoredExPolygonCollection areas(storage, surface.expolygon());
    StoredExtrusionEntity root(storage);
    add_shells(context, areas, settings.count, root.mutable_view(), true);
    root.set_flags(RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);
    append_nonempty(output, root);
}
}
