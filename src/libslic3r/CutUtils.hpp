///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2023 Oleksandra Iushchenko @YuSanka
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_CutUtils_hpp_
#define slic3r_CutUtils_hpp_

#include <vector>

#include "enum_bitmask.hpp"
#include "Model.hpp"
#include "Point.hpp"

/* Cut provides plane, contour and groove cuts for scene editors.
   Objects in a cut group share an ID and a consistency counter.

   Call update_object_cut_id() on the source before constructing Cut.
   Call the chosen perform_*() method, then copy its results into the scene.
   Finish with synchronize_model_after_cut() to update the remaining group members.
   The returned objects belong to Cut. Copy them before Cut is destroyed or reused. */

namespace Slic3r {

using ModelObjectPtrs = std::vector<ModelObject*>;

enum class ModelObjectCutAttribute : int { KeepUpper, KeepLower, KeepAsParts, FlipUpper, FlipLower, PlaceOnCutUpper, PlaceOnCutLower, CreateDowels, InvalidateCutInfo };
using ModelObjectCutAttributes = enum_bitmask<ModelObjectCutAttribute>;
ENABLE_ENUM_BITMASK_OPERATORS(ModelObjectCutAttribute);


class Cut {

    Model                       m_model;
    ModelObjectPtrs             m_result_objects;
    int                         m_instance;
    const Transform3d           m_cut_matrix;
    ModelObjectCutAttributes    m_attributes;

    void post_process(ModelObject* object, ModelObjectPtrs& objects, bool keep, bool place_on_cut, bool flip);
    void post_process(ModelObject* upper_object, ModelObject* lower_object, ModelObjectPtrs& objects);
    void finalize(const ModelObjectPtrs& objects);

public:

    Cut(const ModelObject* object, int instance, const Transform3d& cut_matrix, 
        ModelObjectCutAttributes attributes = ModelObjectCutAttribute::KeepUpper |
                                              ModelObjectCutAttribute::KeepLower |
                                              ModelObjectCutAttribute::KeepAsParts );
    ~Cut() { m_model.clear_objects(); }

    struct Groove
    {
        float depth{ 0.f };
        float width{ 0.f };
        float flaps_angle{ 0.f };
        float angle{ 0.f };
        float depth_init{ 0.f };
        float width_init{ 0.f };
        float flaps_angle_init{ 0.f };
        float angle_init{ 0.f };
        float depth_tolerance{ 0.1f };
        float width_tolerance{ 0.1f };
    };

    struct Part
    {
        bool selected;
        bool is_modifier;
    };

    const ModelObjectPtrs& perform_with_plane();
    const ModelObjectPtrs& perform_by_contour(std::vector<Part> parts, int dowels_count);
    const ModelObjectPtrs& perform_with_groove(const Groove& groove, const Transform3d& rotation_m, bool keep_as_parts = false);

}; // namespace Cut

// When both halves are kept, create the shared ID if needed, then advance its consistency counter.
// Separate dowels also count. Partial cuts or InvalidateCutInfo leave this state unchanged.
void update_object_cut_id(CutObjectBase &cut_id, ModelObjectCutAttributes attributes, int dowels_count);

// Copy the updated ID and counter to cut objects with the same group ID.
// Leave objects from other groups unchanged.
void synchronize_model_after_cut(Model &model, const CutObjectBase &cut_id);

} // namespace Slic3r

#endif /* slic3r_CutUtils_hpp_ */
