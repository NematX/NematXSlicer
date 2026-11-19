// Copyright (c) 2026 NematX. AGPL-3.0-or-later.
#include "InternalSeam.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/Plugins/SeamPlacer/SeamInitializationCancellation.hpp"
#include "libslic3r/Plugins/SeamPlacer/SeamPlacementConstraints.hpp"

/*
Internal-perimeter attractor placement
======================================
Initialization copies the host's constraint geometry and slices each attractor
once per source object. Cross-section centers use the object's centered frame;
the printing plan supplies the explicit translation for each real instance.
Per-loop bindings summarize actual segment roles, not just a parent's role, so
an Arachne loop containing external geometry is never treated as internal.

The published service owns its host fallback and private caches. Temporary
storage owners release all sliced polygons before publication. Queries do not
change the caches, source geometry or configuration. A callback first asks the
host service, preserving its validation and errors, and returns that answer
unchanged for external, mixed, technical or inactive loops. For an active
internal loop, enforced portions take precedence over free portions and blocked
points are excluded. Projection uses the host's analytic line/arc curves.

    make_internal_seam_instance -> Session::initialize
        |-- bind_tree: actual geometric roles and explicit instance shifts
        |-- SeamPlacementConstraints::initialize_for_regions
        `-- slice volume meshes: transformed centers per source layer
    place_callback -> Session::place
        |-- host base callback (also validates membership)
        |-- constraints.portions: enforced before free, never blocked
        `-- nearest_allowed: analytic projection and integer boundary recheck

All exception messages are copied into thread-local storage at the C callback
boundary. The diagnostic survives until the next callback on that thread; no
C++ exception or temporary string crosses the ABI. Destroy releases the
decorator and its fallback exactly once. No process-global geometry is cached.
*/
namespace nematx_seam {
namespace {
using namespace slic3r_api;
using Constraints = SeamPlacement::SeamPlacementConstraints;
using Status = Constraints::Status;

struct Binding {
    const layer_handle *layer = nullptr;
    c_point shift{};
    bool internal = false;
};
struct Roles {
    bool internal = false;
    bool other = false;
    bool technical = false;
};

class Session {
public:
    // Consume one complete host session; reject incomplete ownership records.
    explicit Session(raw_seam_placer_instance &base);
    // Release the owned fallback after the immutable caches are no longer used.
    ~Session();
    Session(const Session &) = delete;
    Session &operator=(const Session &) = delete;
    // Copy source constraints and bind only the explicitly assigned islands.
    void initialize(const PrintingPlan &plan, const std::vector<PrintRegion> &regions,
                    storage_handle *storage, const plugin_run_context *context);
    // Keep the fallback's regional ranking and diagnostic behavior.
    void priority(const extrusion_entity_handle *loop, const layer_region_handle *region,
                  raw_seam_placer_priority_result *out) const;
    // Return the host point or a validated closest admissible internal attractor point.
    c_point place(const ExtrusionEntity &loop, const LayerRegion &region, c_point start) const;
private:
    // Inherit attributes downward and summarize actual segments upward for atomic loops.
    Roles bind_tree(const ExtrusionEntity &entity, raw_extrusion_role role,
                    Binding binding, bool technical, const plugin_run_context *context);
    raw_seam_placer_instance m_base{};
    Constraints m_constraints;
    std::map<const extrusion_entity_handle *, Binding> m_bindings;
    std::map<const layer_handle *, std::vector<c_point>> m_centers;
};

// Find the nearest rounded point of a portion, rechecking excluded boundaries.
bool nearest_allowed(const Constraints &constraints, const ExtrusionEntity &loop,
                     const Constraints::Portion &portion, c_point target, c_point &point);
// Copy the diagnostic before another service callback can replace its storage.
void require_success(raw_seam_placer_status status, const char *message);
// Release decorator ownership at the C boundary; destruction is nonthrowing.
void destroy_callback(void *session) noexcept;
// Forward the host ranking while containing every C++ exception.
void priority_callback(const void *session, const extrusion_entity_handle *loop,
                       const layer_region_handle *region, raw_seam_placer_priority_result *out) noexcept;
// Publish the selected point or a thread-local error without crossing the C ABI.
void place_callback(const void *session, const extrusion_entity_handle *loop,
                    const layer_region_handle *region, c_point start, raw_seam_placer_result *out) noexcept;

thread_local std::string error_message;
const raw_seam_placer_vtable callbacks{sizeof(raw_seam_placer_vtable), destroy_callback,
                                     priority_callback, place_callback};

Session::Session(raw_seam_placer_instance &base)
{
    if (base.struct_size < sizeof(base) || !base.session || !base.vtable ||
        base.vtable->struct_size < sizeof(raw_seam_placer_vtable) || !base.vtable->destroy ||
        !base.vtable->place_seam || !base.vtable->get_priority)
        throw std::invalid_argument("Internal seam attractor needs a complete host seam service.");
    m_base = base;
    base = {};
}

Session::~Session()
{
    m_base.vtable->destroy(m_base.session);
}

void Session::initialize(const PrintingPlan &plan, const std::vector<PrintRegion> &regions,
                         storage_handle *storage, const plugin_run_context *context)
{
    namespace Cancel = SeamPlacement::InitializationCancellation;
    Cancel::check(context);
    m_constraints.initialize_for_regions(plan, storage, regions, context);
    std::set<const print_region_handle *> assigned;
    for (const PrintRegion &region : regions)
        if (region.valid()) assigned.insert(region.handle());
    std::set<const object_handle *> objects;
    for (uint32_t g = 0; g < plan.group_count(); ++g) {
        const ConstPrintingGroup group = plan.group(g);
        for (uint32_t l = 0; l < group.layer_group_count(); ++l) {
            const ConstPrintingLayerGroup layer = group.layer_group(l);
            for (uint32_t t = 0; t < layer.tool_group_count(); ++t) {
                const ConstPrintingToolGroup tool = layer.tool_group(t);
                for (uint32_t e = 0; e < tool.extrusion_count(); ++e) {
                    Cancel::check(context);
                    const ConstPrintingExtrusion extrusion = tool.extrusion(e);
                    const LayerRegionIsland island = extrusion.region_island();
                    if (!island.valid() || island.region_count() == 0) continue;
                    bool selected = false;
                    for (uint32_t r = 0; r < island.region_count(); ++r)
                        selected |= assigned.count(island.region(r).print_region().handle()) != 0;
                    if (!selected) continue;
                    const Layer source = island.region(0).layer();
                    objects.insert(source.object().handle());
                    bind_tree(extrusion.root(), extrusion.role(),
                              {source.handle(), plan.object_instance_shift(extrusion), false}, false, context);
                }
            }
        }
    }
    for (const object_handle *handle : objects) {
        const Object object(handle);
        std::vector<float> heights;
        for (uint32_t l = 0; l < object.layer_count(); ++l)
            heights.push_back(float(unscaled(object.layer(l).print_z())));
        const c_point center = object.center_offset();
        const c_matrix4d transform = matrix4d_mul(
            matrix4d_translation(-unscaled(center.x), -unscaled(center.y), 0), object.transform());
        for (uint32_t v = 0; v < object.volume_count(); ++v) {
            Cancel::check(context);
            const Volume volume = object.volume(v);
            if (volume.type() != RAW_VOLUME_TYPE_SEAM_POSITION_INTERNAL_CENTER_Z) continue;
            c_mesh_slicing_params params{};
            params.transform = matrix4d_mul(transform, volume.matrix());
            std::vector<StoredPolygonCollection> slices = volume.mesh().slice_to_polygons(storage, params, heights, context);
            for (uint32_t l = 0; l < slices.size(); ++l) {
                Cancel::check_periodic(context, l);
                bool found = false;
                c_point low{}, high{};
                const PolygonCollection polygons = slices[l].readonly();
                for (uint32_t p = 0; p < polygons.size(); ++p) {
                    const Polygon polygon = polygons.at(p);
                    for (uint32_t i = 0; i < polygon.size(); ++i) {
                        const c_point point = polygon.at(i);
                        if (!found) { low = high = point; found = true; }
                        low.x = std::min(low.x, point.x); low.y = std::min(low.y, point.y);
                        high.x = std::max(high.x, point.x); high.y = std::max(high.y, point.y);
                    }
                }
                if (found)
                    m_centers[object.layer(l).handle()].push_back(
                        {low.x + (high.x - low.x) / 2, low.y + (high.y - low.y) / 2});
            }
        }
    }
}

Roles Session::bind_tree(const ExtrusionEntity &entity, raw_extrusion_role role,
                         Binding binding, bool technical, const plugin_run_context *context)
{
    SeamPlacement::InitializationCancellation::check(context);
    const c_extrusion_property_attributes *attributes = static_cast<const c_extrusion_property_attributes *>(
        entity.property_data(EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
    if (attributes) role = attributes->role;
    const c_extrusion_property_perimeter *marker = static_cast<const c_extrusion_property_perimeter *>(
        entity.property_data(EXTRUSION_PROPERTY_TYPE_PERIMETER));
    technical |= marker && (marker->perimeter_flags & C_EXTRUSION_PERIMETER_FLAG_SKIRT) != 0;
    Roles summary{};
    summary.technical = technical;
    if (entity.segment_count() != 0) {
        summary.internal = RAW_EXTRUSION_ROLE_IS_PERIMETER(role) && !RAW_EXTRUSION_ROLE_IS_EXTERNAL_PERIMETER(role);
        summary.other = !summary.internal;
    }
    for (uint32_t i = 0; i < entity.child_count(); ++i) {
        const Roles child = bind_tree(entity.child(i), role, binding, technical, context);
        summary.internal |= child.internal;
        summary.other |= child.other;
        summary.technical |= child.technical;
    }
    binding.internal = summary.internal && !summary.other && !summary.technical;
    m_bindings.emplace(entity.handle(), binding);
    return summary;
}

void Session::priority(const extrusion_entity_handle *loop, const layer_region_handle *region,
                       raw_seam_placer_priority_result *out) const
{
    m_base.vtable->get_priority(m_base.session, loop, region, out);
    require_success(out->status, out->error_message);
    if (!std::isfinite(out->priority)) throw std::runtime_error("Host seam priority is not finite.");
}

c_point Session::place(const ExtrusionEntity &loop, const LayerRegion &region, c_point start) const
{
    raw_seam_placer_result base{};
    base.struct_size = sizeof(base);
    m_base.vtable->place_seam(m_base.session, loop.handle(), region.handle(), start, &base);
    require_success(base.status, base.error_message);
    const std::map<const extrusion_entity_handle *, Binding>::const_iterator binding = m_bindings.find(loop.handle());
    if (binding == m_bindings.end() || !binding->second.internal) return base.point;
    const std::map<const layer_handle *, std::vector<c_point>>::const_iterator centers = m_centers.find(binding->second.layer);
    if (centers == m_centers.end() || centers->second.empty()) return base.point;
    const Constraints::Result portions = m_constraints.portions(loop, region, true);
    Status wanted = Status::Free;
    for (const Constraints::Portion &portion : portions.portions)
        if (portion.status == Status::Enforced) { wanted = Status::Enforced; break; }
    c_point best = base.point;
    double distance = std::numeric_limits<double>::infinity();
    for (const c_point center : centers->second) {
        const c_point target{center.x + binding->second.shift.x, center.y + binding->second.shift.y};
        for (const Constraints::Portion &portion : portions.portions) {
            if (portion.status != wanted) continue;
            c_point candidate{};
            if (!nearest_allowed(m_constraints, loop, portion, target, candidate)) continue;
            const double candidate_distance = std::hypot(double(candidate.x) - target.x, double(candidate.y) - target.y);
            if (candidate_distance < distance) { best = candidate; distance = candidate_distance; }
        }
    }
    return best;
}

bool nearest_allowed(const Constraints &constraints, const ExtrusionEntity &loop,
                      const Constraints::Portion &portion, c_point target, c_point &point)
{
    double t = portion.curve.closest_parameter(target, portion.begin, portion.end);
    const double middle = (portion.begin + portion.end) / 2;
    const double span = portion.end - portion.begin;
    double inset = std::min(span / 2, 2.0 / std::max(1.0, portion.curve.length()));
    // Integer rounding may return an excluded boundary. Move only toward the
    // interior of this analytically classified portion, never across a zone.
    for (;;) {
        point = portion.curve.point(t);
        const bool endpoint_allowed = (t != portion.begin || portion.include_begin) &&
                                      (t != portion.end || portion.include_end);
        if (endpoint_allowed && constraints.classify_point(loop, point, portion.paint_margin) == portion.status)
            return true;
        if (t == middle || inset == 0) return false;
        t = t < middle ? std::min(middle, portion.begin + inset) : std::max(middle, portion.end - inset);
        inset = std::min(span / 2, inset * 2);
    }
}

void require_success(raw_seam_placer_status status, const char *message)
{
    if (status != RAW_SEAM_PLACER_STATUS_SUCCESS)
        throw std::runtime_error(message ? message : "Host seam service failed without a diagnostic.");
}

void destroy_callback(void *session) noexcept
{
    delete static_cast<Session *>(session);
}

void priority_callback(const void *session, const extrusion_entity_handle *loop,
                       const layer_region_handle *region, raw_seam_placer_priority_result *out) noexcept
{
    if (!out || out->struct_size < sizeof(*out)) return;
    out->status = RAW_SEAM_PLACER_STATUS_ERROR;
    out->priority = 0; out->error_message = nullptr;
    try {
        if (!session) throw std::invalid_argument("Missing internal seam session.");
        static_cast<const Session *>(session)->priority(loop, region, out);
    } catch (const std::exception &error) {
        error_message = error.what(); out->status = RAW_SEAM_PLACER_STATUS_ERROR;
        out->error_message = error_message.c_str();
    } catch (...) {
        out->status = RAW_SEAM_PLACER_STATUS_ERROR;
        out->error_message = "Unknown internal seam priority failure.";
    }
}

void place_callback(const void *session, const extrusion_entity_handle *loop,
                    const layer_region_handle *region, c_point start, raw_seam_placer_result *out) noexcept
{
    if (!out || out->struct_size < sizeof(*out)) return;
    out->status = RAW_SEAM_PLACER_STATUS_ERROR;
    out->point = {}; out->error_message = nullptr;
    try {
        if (!session) throw std::invalid_argument("Missing internal seam session.");
        out->point = static_cast<const Session *>(session)->place(ExtrusionEntity(loop), LayerRegion(region), start);
        out->status = RAW_SEAM_PLACER_STATUS_SUCCESS;
    } catch (const std::exception &error) {
        error_message = error.what(); out->error_message = error_message.c_str();
    } catch (...) {
        out->error_message = "Unknown internal seam placement failure.";
    }
}
} // namespace

raw_seam_placer_instance make_internal_seam_instance(
    raw_seam_placer_instance &base, const slic3r_api::PrintingPlan &plan,
    const std::vector<slic3r_api::PrintRegion> &regions, storage_handle *storage,
    const plugin_run_context *context)
{
    std::unique_ptr<Session> session(new Session(base));
    session->initialize(plan, regions, storage, context);
    return {sizeof(raw_seam_placer_instance), session.release(), &callbacks};
}
} // namespace nematx_seam
