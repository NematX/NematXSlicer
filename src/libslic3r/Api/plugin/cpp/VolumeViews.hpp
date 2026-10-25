///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_VolumeViews_hpp_
#define slic3r_Api_plugin_cpp_VolumeViews_hpp_

#include <cassert>
#include <stdexcept>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_volume.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"

namespace slic3r_api {

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

    ///*
    //Fill one already-created StoredPolygonCollection per slice Z. Z values are
    //unscaled float coordinates, matching TriangleMesh vertices. If the source is
    //Layer::slice_z(), convert it with unscaled() before calling this helper.
    //*/
    //void slice_to_polygons(storage_handle *storage,
    //                       c_matrix4d transform,
    //                       const float *slice_zs,
    //                       uint32_t slice_count,
    //                       polygon_collection_handle **out_by_layer) const
    //{
    //    triangle_mesh_slice_to_polygons(storage, handle(), transform, slice_zs, slice_count, out_by_layer);
    //}

    /*
    Convenience overload for plugin code. The returned StoredPolygonCollection
    objects own their storage allocations and may be moved into later steps.
    */
    std::vector<StoredPolygonCollection> slice_to_polygons(storage_handle *storage,
                                                           c_matrix4d transform,
                                                           const std::vector<float> &slice_zs) const
    {
        std::vector<StoredPolygonCollection> out;
        out.reserve(slice_zs.size());
        for (size_t idx = 0; idx < slice_zs.size(); ++idx)
            out.emplace_back(storage);

        std::vector<polygon_collection_handle *> out_handles;
        out_handles.reserve(out.size());
        for (StoredPolygonCollection &polygons : out)
            out_handles.push_back(polygons.mutable_handle());

        assert(slice_zs.size() == out_handles.size());
        triangle_mesh_slice_to_polygons(handle(), transform, slice_zs.data(), out_handles.data(), static_cast<uint32_t>(slice_zs.size()));
        return out;
    }

    /*
    Native-equivalent ExPolygon slicing path. Use this from slicing plugins when
    reproducing PrintObjectSlice.cpp: it applies closing radius, extra offset,
    contour simplification and slicing mode inside the host.
    */
    std::vector<StoredExPolygonCollection> slice_to_expolygons(storage_handle *storage,
                                                               const c_mesh_slicing_params &params,
                                                               const std::vector<float> &slice_zs) const
    {
        std::vector<StoredExPolygonCollection> out;
        out.reserve(slice_zs.size());
        for (size_t idx = 0; idx < slice_zs.size(); ++idx)
            out.emplace_back(storage);

        std::vector<expolygon_collection_handle *> out_handles;
        out_handles.reserve(out.size());
        for (StoredExPolygonCollection &expolygons : out)
            out_handles.push_back(expolygons.mutable_handle());

        assert(slice_zs.size() == out_handles.size());
        triangle_mesh_slice_to_expolygons_with_params(handle(), &params, slice_zs.data(), out_handles.data(),
                                                      static_cast<uint32_t>(slice_zs.size()));
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
    bool is_support_modifier() const { return is_support_blocker() || is_support_enforcer(); }
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
