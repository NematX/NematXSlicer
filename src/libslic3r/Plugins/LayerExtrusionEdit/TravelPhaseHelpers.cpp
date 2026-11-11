///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "TravelPhaseHelpers.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <stdexcept>

#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace TravelPhase {

namespace {

/* Append the segmented leaves below entity, resolving roles and Z offsets on the way down. */
bool append_leaves(MutableExtrusionEntity entity, std::optional<raw_extrusion_role> role,
                   coord_t z_offset, std::vector<TravelLeaf> &leaves);

bool append_leaves(MutableExtrusionEntity entity, std::optional<raw_extrusion_role> role,
                   coord_t z_offset, std::vector<TravelLeaf> &leaves)
{
    if (const EPropertyAttributes *attributes = entity.get(EPropertyAttributes::key))
        role = attributes->extrusion_role();
    if (const EPropertyZOffset *offset = entity.get(EPropertyZOffset::key))
        z_offset = offset->get();
    if (entity.is_deposit())
        return false;
    if (entity.segment_count() != 0) {
        if (!role || entity.child_count() != 0)
            return false;
        leaves.push_back(TravelLeaf{entity, *role, z_offset});
        return true;
    }
    for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
        if (!append_leaves(entity.child_mutable(idx), role, z_offset, leaves))
            return false;
    return true;
}

} // namespace

bool collect_travel_leaves(MutableExtrusionEntity phase, const coord_t inherited_z_offset,
                           std::vector<TravelLeaf> &leaves)
{
    leaves.clear();
    if (!phase.valid())
        return true;
    if (!append_leaves(phase, std::nullopt, inherited_z_offset, leaves)) {
        leaves.clear();
        return false;
    }
    for (size_t idx = 1; idx < leaves.size(); ++idx) {
        const ExtrusionEntity &previous = leaves[idx - 1].entity.readonly();
        const ExtrusionEntity &next = leaves[idx].entity.readonly();
        const c_extrusion_segment tail = previous.segment(previous.segment_count() - 1);
        const c_extrusion_segment head = next.segment(0);
        // Compare absolute heights in 64 bits so that distinct offsets cannot overflow.
        const int64_t tail_z = int64_t(leaves[idx - 1].z_offset) + tail.z_offset_b;
        const int64_t head_z = int64_t(leaves[idx].z_offset) + head.z_offset_a;
        if (c_point_distance_to(tail.point_b, head.point_a) > SCALED_EPSILON ||
            std::llabs(tail_z - head_z) > SCALED_EPSILON) {
            leaves.clear();
            return false;
        }
    }
    return true;
}

distf_t segment_planar_length(const c_extrusion_segment &segment)
{
    const distf_t chord = c_point_distance_to(segment.point_a, segment.point_b);
    if (segment.radius == 0.f)
        return chord;
    const double radius = std::abs(double(segment.radius));
    if (!(radius > 0.0) || chord > 2.0 * radius + double(SCALED_EPSILON))
        throw std::invalid_argument("A path contains an invalid arc radius.");
    double angle = 2.0 * std::asin(std::clamp(
        double(chord) / (2.0 * radius), 0.0, 1.0));
    if (segment.radius < 0.f)
        angle = 2.0 * std::acos(-1.0) - angle;
    return radius * angle;
}

}}} // namespace slic3r_api::LayerExtrusionEdit::TravelPhase
