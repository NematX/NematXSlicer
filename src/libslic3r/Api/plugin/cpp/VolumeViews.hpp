///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_VolumeViews_hpp_
#define slic3r_Api_plugin_cpp_VolumeViews_hpp_

#include <atomic>
#include <cassert>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_utils.h"
#include "libslic3r/Api/plugin/c/slic3r_volume.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/PolygonOps.hpp"

namespace slic3r_api {

namespace detail {

// Settings of the post-processing of the raw slices (see TriangleMesh::slice_to_expolygons), converted once:
// the distances are scaled units truncated like the native scale_i(), and every branch tests these integers.
struct SliceFinishSettings {
    coord_t closing_radius = 0;
    coord_t model_resolution = 0;
    coord_t extra_offset = 0;
    coord_t resolution = 0;
    raw_mesh_slicing_mode mode = RAW_MESH_SLICING_MODE_REGULAR;
    raw_mesh_slicing_mode mode_below = RAW_MESH_SLICING_MODE_REGULAR;
    uint32_t normal_below_layer = 0;
};

// Shared state of the parallel loop that finishes the layers. A job never lets an exception escape: it keeps the
// first one here, because the loop runs inside C frames of the host.
struct SliceFinishJobs {
    SliceFinishSettings settings;
    std::vector<StoredPolygonCollection> *rings = nullptr;
    std::vector<StoredExPolygonCollection> *out = nullptr;
    const plugin_run_context *context = nullptr;
    std::atomic<uint32_t> finished{0};
    std::atomic<bool> failed{false};
    std::mutex error_mutex;
    std::exception_ptr error;
};

// Turns the raw rings of one layer into its final ExPolygons, as the native slice_mesh_ex() followed by the
// final ensure_valid() of the host do (TriangleMeshSlicer.cpp slice_mesh_ex / make_expolygons). rings is
// modified in place; temporary collections are made in scratch; the result is moved into out.
inline void finish_slice_layer(const SliceFinishSettings &settings,
                               StoredPolygonCollection &rings,
                               StoredExPolygonCollection &out,
                               uint32_t layer,
                               storage_handle *scratch)
{
    // The mode of the layer decides the fill rule and the largest-contour filter, not the raw slicing.
    const raw_mesh_slicing_mode mode = layer < settings.normal_below_layer ? settings.mode_below : settings.mode;

    rings.ensure_valid(SCALED_EPSILON);
    if (settings.model_resolution > 0)
        remove_collinear(rings, settings.model_resolution);

    const clipper_fill_rule_t fill_rule = mode == RAW_MESH_SLICING_MODE_EVEN_ODD ? CLIPPER_FILL_EVENODD :
        mode == RAW_MESH_SLICING_MODE_POSITIVE_LARGEST_CONTOUR ? CLIPPER_FILL_POSITIVE : CLIPPER_FILL_NONZERO;
    StoredExPolygonCollection slices = clipper_union_ex(scratch, rings, fill_rule);

    // Closing (grow then shrink) and extra offset, in one or two offsets as the native make_expolygons() does.
    double offset_out = 0.;
    double offset_in = 0.;
    if (settings.closing_radius >= settings.extra_offset) {
        offset_out = +(settings.closing_radius);
        offset_in = -(settings.closing_radius - settings.extra_offset);
    } else {
        offset_out = +(settings.extra_offset);
        offset_in = 0.;
    }
    if (offset_out > 0 && offset_in < 0)
        slices = clipper_offset2_ex(scratch, slices, offset_out, offset_in, CLIPPER_JOIN_MITER, 3.);
    else if (offset_out > 0)
        slices = clipper_offset_ex(scratch, slices, offset_out, CLIPPER_JOIN_MITER, 3.);
    else if (offset_in < 0)
        slices = clipper_offset_ex(scratch, slices, offset_in, CLIPPER_JOIN_MITER, 3.);

    // keep_largest_contour_only(): the island whose contour has the largest signed area, the first one on a tie.
    if (mode == RAW_MESH_SLICING_MODE_POSITIVE_LARGEST_CONTOUR && slices.size() > 1) {
        double max_area = 0.;
        uint32_t largest = 0;
        bool found = false;
        for (uint32_t idx = 0; idx < slices.size(); ++idx) {
            const double area = slices.at(idx).contour().area();
            if (area > max_area) {
                max_area = area;
                largest = idx;
                found = true;
            }
        }
        // found is false only when no contour has a positive area: the islands are then left unchanged, as the native
        // keep_largest_contour_only() does. The union and the offsets give counter-clockwise contours, so this does not occur.
        if (found) {
            StoredExPolygon kept = slices.extract(largest);
            slices.clear();
            slices.push_back_move(std::move(kept));
        }
    }

    if (settings.resolution != 0) {
        // The safety offset (native union_safety_offset_ex) merges the islands that touch, then the simplification.
        slices = clipper_offset_ex(scratch, slices, 10., CLIPPER_JOIN_MITER, 3.);
        slices.ensure_valid(settings.resolution);
    } else {
        slices.ensure_valid(SCALED_EPSILON);
    }
    // The host applied one more simplification to the result, with a resolution that can be 0.
    slices.ensure_valid(settings.resolution);
    out.move_from(slices);
}

// Callback of slic3r_parallel_for_storage(): one layer. It must not throw, nor leave anything allocated in scratch.
inline void slice_finish_job(uint32_t layer, storage_handle *scratch, void *user_data)
{
    SliceFinishJobs &jobs = *static_cast<SliceFinishJobs *>(user_data);
    try {
        if (jobs.failed.load() || is_cancelled(jobs.context))
            return;
        finish_slice_layer(jobs.settings, (*jobs.rings)[layer], (*jobs.out)[layer], layer, scratch);
        jobs.finished.fetch_add(1);
    } catch (...) {
        jobs.failed.store(true);
        std::lock_guard<std::mutex> lock(jobs.error_mutex);
        if (!jobs.error)
            jobs.error = std::current_exception();
    }
}

} // namespace detail

/*
Read-only helpers over Volume and TriangleMesh.

The views in this file are borrowed: they never own or free the underlying host
objects. They are meant for slicing plugins that need to inspect the source model
volumes through an Object. The only owner here is StoredTriangleMesh, which releases
a mesh that plugin code created in its storage.

Typical use:
    for (uint32_t i = 0; i < object.volume_count(); ++i) {
        Volume volume = object.volume(i);
        if (!volume.is_model_part())
            continue;
        TriangleMesh mesh = volume.mesh();
        ...
    }
*/

class TriangleMesh : public ConstDataTreeHandleView<triangle_mesh_handle>
{
public:
    using ConstDataTreeHandleView<triangle_mesh_handle>::ConstDataTreeHandleView;

    uint32_t vertex_count() const { return triangle_mesh_vertex_count(handle()); }
    uint32_t triangle_count() const { return triangle_mesh_triangle_count(handle()); }
    bool empty() const { return triangle_count() == 0; }

    /* Return a storage-owned projection. A native validation failure is an
       error; only a successful projection may represent empty geometry. */
    StoredPolygonCollection project_to_polygons(storage_handle *storage, c_matrix4d transform) const
    {
        polygon_collection_handle *projected = triangle_mesh_project_to_polygons(storage, handle(), transform);
        if (projected == nullptr)
            throw std::invalid_argument("The TriangleMesh projection could not be created.");
        return StoredPolygonCollection::adopt(storage, projected);
    }

    c_vec3f vertex(uint32_t idx) const
    {
        assert(idx < vertex_count());
        return triangle_mesh_vertex_at(handle(), idx);
    }

    c_triangle_indices triangle(uint32_t idx) const
    {
        assert(idx < triangle_count());
        return triangle_mesh_triangle_at(handle(), idx);
    }

    /*
    Raw loops of the native slicer, one StoredPolygonCollection per slice Z (see triangle_mesh_slice_to_polygons()
    in slic3r_volume.h). Z values are unscaled float millimeters, finite and non-decreasing; if the source is
    Layer::slice_z(), convert it with unscaled() first. Only mode, slicing_mode_normal_below_layer, mode_below and
    transform are used: the polygons are not unioned, offset or simplified, so closing_radius, extra_offset,
    resolution and model_resolution only have to be valid. transform must be set (for example with
    matrix4d_identity()): a zeroed struct holds a null matrix, which crushes the mesh. The returned collections own
    their storage allocations and may be moved into later steps; the storage must outlive them.
    A cancelled context (or a cancelled surrounding parallel group) throws PluginCancelled; an invalid argument
    throws std::invalid_argument and an internal host error std::runtime_error. context may be null (never
    cancelled by the context).
    */
    std::vector<StoredPolygonCollection> slice_to_polygons(storage_handle *storage,
                                                           const c_mesh_slicing_params &params,
                                                           const std::vector<float> &slice_zs,
                                                           const plugin_run_context *context) const
    {
        std::vector<StoredPolygonCollection> out;
        out.reserve(slice_zs.size());
        for (size_t idx = 0; idx < slice_zs.size(); ++idx)
            out.emplace_back(storage);

        std::vector<polygon_collection_handle *> out_handles;
        out_handles.reserve(out.size());
        for (StoredPolygonCollection &polygons : out)
            out_handles.push_back(polygons.mutable_handle());

        // On any failure the host leaves the outputs empty and `out` frees them while the exception propagates.
        const int32_t status = triangle_mesh_slice_to_polygons(handle(), &params, slice_zs.data(), out_handles.data(),
                                                               static_cast<uint32_t>(slice_zs.size()), context);
        switch (status) {
        case RAW_MESH_SLICE_STATUS_DONE:
            return out;
        case RAW_MESH_SLICE_STATUS_CANCELLED:
            throw PluginCancelled();
        case RAW_MESH_SLICE_STATUS_INVALID_ARGUMENT:
            throw std::invalid_argument("The TriangleMesh slicing arguments are invalid.");
        default:
            throw std::runtime_error("The TriangleMesh slicing failed inside the host.");
        }
    }

    /*
    ExPolygon slicing path, byte-identical to the native slice_mesh_ex() followed by the final ensure_valid() that the
    host applied before the volume ABI 2.0. Use it from slicing plugins that reproduce the native slicing. The raw loops
    come from the cancellable triangle_mesh_slice_to_polygons() (with PositiveLargestContour replaced by Positive), and
    the rest is done here through the plugin API, layer by layer in parallel:
      1. polygons_ensure_valid at SCALED_EPSILON;
      2. remove_collinear(model_resolution) on every ring when model_resolution > 0;
      3. clipper_union_ex with the fill rule of the layer mode (EvenOdd: even-odd, PositiveLargestContour: positive,
         else non-zero), where the layer mode is mode_below under slicing_mode_normal_below_layer;
      4. the closing / extra offset (clipper_offset2_ex or clipper_offset_ex);
      5. only the largest contour for PositiveLargestContour;
      6. with a resolution: a safety offset and ensure_valid(resolution), else ensure_valid at SCALED_EPSILON;
      7. a last ensure_valid(resolution), also when it is 0.
    closing_radius, extra_offset, model_resolution and resolution are millimeters, truncated to scaled units like
    scale_i(). transform must be set, as for slice_to_polygons(). The result is one collection per Z, in the
    storage, which must outlive it.
    A cancelled context, or a cancellation of the surrounding parallel group, throws PluginCancelled and returns
    nothing; an error of a layer is rethrown. context may be null (never cancelled).
    */
    std::vector<StoredExPolygonCollection> slice_to_expolygons(storage_handle *storage,
                                                               const c_mesh_slicing_params &params,
                                                               const std::vector<float> &slice_zs,
                                                               const plugin_run_context *context) const
    {
        // The raw slicing keeps the polygons of PositiveLargestContour as they are for the fill rule below.
        c_mesh_slicing_params raw_params = params;
        if (raw_params.mode == RAW_MESH_SLICING_MODE_POSITIVE_LARGEST_CONTOUR)
            raw_params.mode = RAW_MESH_SLICING_MODE_POSITIVE;
        if (raw_params.mode_below == RAW_MESH_SLICING_MODE_POSITIVE_LARGEST_CONTOUR)
            raw_params.mode_below = RAW_MESH_SLICING_MODE_POSITIVE;
        std::vector<StoredPolygonCollection> rings = slice_to_polygons(storage, raw_params, slice_zs, context);

        // The outputs exist before the loop, so the jobs only fill them.
        std::vector<StoredExPolygonCollection> out;
        out.reserve(slice_zs.size());
        for (size_t idx = 0; idx < slice_zs.size(); ++idx)
            out.emplace_back(storage);

        detail::SliceFinishJobs jobs;
        jobs.settings.closing_radius = scale_i(params.closing_radius);
        jobs.settings.model_resolution = scale_i(params.model_resolution);
        jobs.settings.extra_offset = scale_i(params.extra_offset);
        jobs.settings.resolution = scale_i(params.resolution);
        jobs.settings.mode = params.mode;
        jobs.settings.mode_below = params.mode_below;
        jobs.settings.normal_below_layer = params.slicing_mode_normal_below_layer;
        jobs.rings = &rings;
        jobs.out = &out;
        jobs.context = context;
        slic3r_parallel_for_storage(0, static_cast<uint32_t>(slice_zs.size()), &jobs, &detail::slice_finish_job);

        // A layer that did not run (cancelled context, or a cancelled surrounding group) leaves the count short.
        if (jobs.error)
            std::rethrow_exception(jobs.error);
        if (jobs.finished.load() != slice_zs.size() || is_cancelled(context))
            throw PluginCancelled();
        return out;
    }
};

/*
StoredTriangleMesh owns a mesh created in plugin storage from plugin arrays.

It is a TriangleMesh view, so the read and slice methods apply directly. The
destructor releases the handle with storage_free(); the storage must outlive the
wrapper. While the owner is alive, storage_free() on its mesh or storage_clear() on
its storage must not run, or the destructor would release the mesh twice. The
wrapper is move-only so exactly one owner releases the mesh.
*/
class StoredTriangleMesh : public TriangleMesh
{
public:
    // Allocate the mesh in storage; a failed creation throws std::invalid_argument.
    StoredTriangleMesh(storage_handle *storage,
                       const std::vector<c_vec3f> &vertices,
                       const std::vector<c_triangle_indices> &triangles) :
        TriangleMesh(create(storage, vertices, triangles)), m_storage(storage) {}

    StoredTriangleMesh(const StoredTriangleMesh &) = delete;
    StoredTriangleMesh &operator=(const StoredTriangleMesh &) = delete;

    // Take over the allocation; the moved-from wrapper is left empty.
    StoredTriangleMesh(StoredTriangleMesh &&other) noexcept :
        TriangleMesh(other), m_storage(other.m_storage) {
        other.m_handle = nullptr;
        other.m_storage = nullptr;
    }

    // Release the current allocation, then take over the source's; the source is left empty.
    StoredTriangleMesh &operator=(StoredTriangleMesh &&other) noexcept {
        if (this == &other)
            return *this;
        reset();
        m_handle = other.m_handle;
        m_storage = other.m_storage;
        other.m_handle = nullptr;
        other.m_storage = nullptr;
        return *this;
    }

    // Release the owned mesh; the storage must still be alive.
    ~StoredTriangleMesh() { reset(); }

private:
    // Allocate the storage mesh; a NULL result (invalid arrays or an internal failure such as allocation) throws.
    static triangle_mesh_handle *create(storage_handle *storage,
                                        const std::vector<c_vec3f> &vertices,
                                        const std::vector<c_triangle_indices> &triangles)
    {
        if (storage == nullptr)
            throw std::invalid_argument("StoredTriangleMesh requires a storage handle.");
        triangle_mesh_handle *handle = storage_new_triangle_mesh(storage,
                                                                 vertices.data(),
                                                                 static_cast<uint32_t>(vertices.size()),
                                                                 triangles.data(),
                                                                 static_cast<uint32_t>(triangles.size()));
        if (handle == nullptr)
            throw std::invalid_argument("The TriangleMesh could not be created from the given arrays.");
        return handle;
    }

    // Release the allocation once; a moved-from wrapper holds no handle and does nothing.
    void reset() noexcept {
        if (m_storage == nullptr || m_handle == nullptr)
            return;
        const bool freed = storage_free(m_storage, const_cast<triangle_mesh_handle *>(m_handle)) != 0;
        assert(freed);
        m_storage = nullptr;
        m_handle = nullptr;
    }

    storage_handle *m_storage = nullptr;
};

class Volume : public ConstDataTreeHandleView<volume_handle>
{
public:
    using ConstDataTreeHandleView<volume_handle>::ConstDataTreeHandleView;

    uint64_t id() const { return volume_get_id(handle()); }
    raw_volume_type type() const { return volume_get_type(handle()); }
    Config config() const { return Config(volume_get_config(handle())); }

    int32_t extruder_id() const { return volume_get_extruder_id(handle()); }

    c_matrix4d matrix() const { return volume_get_matrix(handle()); }
    c_matrix4d matrix_no_offset() const { return volume_get_matrix_no_offset(handle()); }

    bool is_model_part() const { return type() == RAW_VOLUME_TYPE_MODEL_PART; }
    bool is_negative() const { return type() == RAW_VOLUME_TYPE_NEGATIVE_VOLUME; }
    bool is_modifier() const { return type() == RAW_VOLUME_TYPE_PARAMETER_MODIFIER; }
    bool is_support_enforcer() const { return type() == RAW_VOLUME_TYPE_SUPPORT_ENFORCER; }
    bool is_support_blocker() const { return type() == RAW_VOLUME_TYPE_SUPPORT_BLOCKER; }
    // Support-demand modifiers: they add or remove object surfaces that ask for support.
    bool is_support_modifier() const { return is_support_blocker() || is_support_enforcer(); }
    bool is_support_column() const { return type() == RAW_VOLUME_TYPE_SUPPORT_COLUMN; }
    bool is_no_support_zone() const { return type() == RAW_VOLUME_TYPE_NO_SUPPORT_ZONE; }
    // Support-geometry modifiers: they add to or remove from the support itself, not from its demand.
    bool is_support_geometry_modifier() const { return is_support_column() || is_no_support_zone(); }
    bool is_seam_position() const
    {
        const raw_volume_type value = type();
        return value == RAW_VOLUME_TYPE_SEAM_POSITION_CENTER ||
               value == RAW_VOLUME_TYPE_SEAM_POSITION_CENTER_Z ||
               value == RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE_CENTER ||
               value == RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE;
    }
    bool is_brim() const { return type() == RAW_VOLUME_TYPE_BRIM_PATCH || type() == RAW_VOLUME_TYPE_BRIM_NEGATIVE; }

    bool has_painting(const char *paint_key) const { return volume_has_painting(handle(), paint_key) != 0; }
    bool is_fdm_support_painted() const { return has_painting(RAW_FACET_PAINTING_FDM_SUPPORT); }
    bool is_seam_painted() const { return has_painting(RAW_FACET_PAINTING_SEAM); }
    bool is_mm_painted() const { return has_painting(RAW_FACET_PAINTING_MMU_SEGMENTATION); }

    TriangleMesh mesh() const { return TriangleMesh(volume_get_mesh(handle())); }
};

/*
Borrowed views over PrintObjectRegions' layer ranges and volume-region entries.

Use these when helper geometry already has a 2D subject and only needs to ask
"which PrintRegion settings apply at this Z?". The views expose the same mapping
that slicing plugins use, but they are not tied to STEP_SLICING. They are
read-only and remain valid only while the Object and its shared region table are
valid.
*/
class SlicingVolumeRegion : public ConstDataTreeHandleView<slicing_volume_region_handle>
{
public:
    using ConstDataTreeHandleView<slicing_volume_region_handle>::ConstDataTreeHandleView;

    Volume volume() const { return Volume(slicing_volume_region_get_volume(handle())); }
    int32_t parent() const { return slicing_volume_region_get_parent(handle()); }
    int32_t layer_region_idx() const { return slicing_volume_region_get_layer_region_idx(handle()); }
    c_bounding_box3f bbox() const { return slicing_volume_region_get_bbox(handle()); }
};

class SlicingLayerRange : public ConstDataTreeHandleView<slicing_layer_range_handle>
{
public:
    using ConstDataTreeHandleView<slicing_layer_range_handle>::ConstDataTreeHandleView;

    coord_t z_min() const { return slicing_layer_range_get_z_min(handle()); }
    coord_t z_max() const { return slicing_layer_range_get_z_max(handle()); }
    Config config() const { return Config(slicing_layer_range_get_config(handle())); }

    uint32_t volume_region_count() const { return slicing_layer_range_count_volume_region(handle()); }
    SlicingVolumeRegion volume_region(uint32_t idx) const {
        return SlicingVolumeRegion(slicing_layer_range_get_volume_region(handle(), idx));
    }
};

inline std::vector<StoredPolygonCollection> project_painting_to_polygons(storage_handle *storage,
                                                                         const Object &object,
                                                                         const char *paint_key,
                                                                         int32_t painting_value)
{
    std::vector<StoredPolygonCollection> out;
    out.reserve(object.layer_count());
    for (uint32_t idx = 0; idx < object.layer_count(); ++idx)
        out.emplace_back(storage);

    std::vector<polygon_collection_handle *> out_handles;
    out_handles.reserve(out.size());
    for (StoredPolygonCollection &polygons : out)
        out_handles.push_back(polygons.mutable_handle());

    object_project_painting_to_polygons(object.handle(), paint_key, painting_value, out_handles.data(),
                                        static_cast<uint32_t>(out_handles.size()));
    return out;
}

inline uint32_t Object::volume_count() const
{
    return object_volume_count(handle());
}

inline Volume Object::volume(uint32_t idx) const
{
    assert(idx < volume_count());
    return Volume(object_volume_at(handle(), idx));
}

inline uint32_t object_slicing_layer_range_count(const Object &object)
{
    return object_count_slicing_layer_range(object.handle());
}

inline SlicingLayerRange object_slicing_layer_range(const Object &object, uint32_t idx)
{
    assert(idx < object_slicing_layer_range_count(object));
    return SlicingLayerRange(object_get_slicing_layer_range(object.handle(), idx));
}

} // namespace slic3r_api


#endif // slic3r_Api_plugin_cpp_VolumeViews_hpp_
