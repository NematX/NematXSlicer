///|/ Copyright (c) 2026 NematX
///|/
///|/ Released under the GNU Affero General Public License, version 3 or later.
#pragma once

// Tree pass of the regional fan overrides, separated from the plugin so that
// SDK tests can run it on stored entities. The plugin reads the settings and
// builds the SourceRequests of each source; see RegionalFan.cpp.

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

namespace nematx_regional_fan {

using namespace slic3r_api;

// Index of each setting in Requests::percent.
enum Setting { General, Perimeters, SolidInfill, Bridge, SettingCount };

// The enabled overrides of one region, in percent; no value: setting disabled.
struct Requests
{
    std::array<std::optional<float>, SettingCount> percent;

    bool any() const
    {
        return std::any_of(percent.begin(), percent.end(), [](const std::optional<float> &p) { return p.has_value(); });
    }
    bool operator==(const Requests &rhs) const { return percent == rhs.percent; }

    // The role-specific override first, then the general one, as in the legacy slicer.
    std::optional<float> for_role(raw_extrusion_role role) const
    {
        Setting specific = SettingCount;
        if (role == RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER || role == RAW_EXTRUSION_ROLE_INTERNAL_PERIMETER)
            specific = Perimeters;
        else if (RAW_EXTRUSION_ROLE_IS_SOLID_INFILL(role))
            specific = RAW_EXTRUSION_ROLE_IS_BRIDGE(role) ? Bridge : SolidInfill;
        if (specific != SettingCount && percent[specific])
            return percent[specific];
        return percent[General];
    }
};

// Requests of one extrusion source. When its regions have different requests,
// areas holds one mask per value (in printing-plan coordinates) and requests the
// matching value; otherwise areas is empty and uniform applies. outside is used
// for skirt, brim and support, printed outside the masks.
struct SourceRequests
{
    Requests uniform;
    std::vector<StoredExPolygonCollection> areas;
    std::vector<Requests> requests;
    Requests outside;

    bool requires_split() const { return !areas.empty(); }
    bool any() const
    {
        return uniform.any() || outside.any() ||
               std::any_of(requests.begin(), requests.end(), [](const Requests &r) { return r.any(); });
    }
};

// The deposition leaves of the host fan policies: printed object paths, support
// and skirt/brim, without the paths whose E is replaced by a raw EXTRUSION command.
inline bool is_fan_deposit(const MutableExtrusionEntity &leaf, raw_extrusion_role role)
{
    if (role == RAW_EXTRUSION_ROLE_NONE || RAW_EXTRUSION_ROLE_IS_TRAVEL(role) ||
        RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_WIPE_TOWER | RAW_EXTRUSION_ROLE_MILL))
        return false;
    const EPropertySpecialCommand *command = leaf.get(EPropertySpecialCommand::key);
    if (command != nullptr && command->code == C_EXTRUSION_SPECIAL_COMMAND_EXTRUSION)
        return false;
    if (role == RAW_EXTRUSION_ROLE_GAP_FILL)
        return true;
    return !RAW_EXTRUSION_ROLE_IS_MIXED(role) &&
           (RAW_EXTRUSION_ROLE_IS_PERIMETER(role) || RAW_EXTRUSION_ROLE_IS_INFILL(role) ||
            RAW_EXTRUSION_ROLE_IS_SKIRT(role) || RAW_EXTRUSION_ROLE_IS_SUPPORT(role));
}

inline void set_request(MutableExtrusionEntity leaf, const std::optional<float> &percent)
{
    if (percent)
        leaf.get_or_add(EPropertyFan::key).set(*percent, FAN_REQUEST_MODE_OVERRIDE);
}

// Refuse to repeat a one-shot event in every fragment of a split, rather than
// choosing the region of its first point.
inline void reject_one_shot_events(const MutableExtrusionEntity &leaf)
{
    for (const extrusion_property_type type : { EXTRUSION_PROPERTY_TYPE_CUSTOM_GCODE, EXTRUSION_PROPERTY_TYPE_SPECIAL_COMMAND,
                                                EXTRUSION_PROPERTY_TYPE_HEATER, EXTRUSION_PROPERTY_TYPE_EXTRUSION_AXIS })
        if (leaf.has_property(type))
            throw std::runtime_error("A regional fan override needs to split, at a region boundary, a path that carries a "
                                     "one-shot event (custom G-code, special command, heater or extrusion axis). The "
                                     "export was rejected to avoid repeating the event.");
}

// Check that the splitter's fragments preserve the source path length within tolerance.
// The splitter drops pieces and snaps ends shorter than SCALED_EPSILON at each cut, so allow
// one SCALED_EPSILON per fragment plus a small relative term for accumulated rounding.
inline void validate_split_coverage(double source_length, const std::vector<ExtrusionAreaFragment> &fragments)
{
    double fragment_length = 0.0;
    for (const ExtrusionAreaFragment &fragment : fragments)
        fragment_length += double(fragment.entity.local_length());
    const double tolerance = double(fragments.size()) * double(SCALED_EPSILON) + std::abs(source_length) * 1e-7;
    if (!std::isfinite(source_length) || !std::isfinite(fragment_length) ||
        std::abs(source_length - fragment_length) > tolerance)
        throw std::runtime_error("Regional areas do not cover an entire printable path. "
                                 "The export was rejected to avoid dropping or duplicating geometry.");
}

// Traverse the tree with the inherited attributes and fan request, then give each
// modifiable deposition leaf the request of its role in its region.
inline void apply_tree(MutableExtrusionEntity entity, std::optional<EPropertyAttributes> attributes, EPropertyFan fan,
                const SourceRequests &source, const std::vector<ExPolygonCollection> &views,
                const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (const EPropertyAttributes *direct = entity.get(EPropertyAttributes::key))
        attributes = *direct;
    if (const EPropertyFan *direct = entity.get(EPropertyFan::key))
        fan = direct->resolved_with(fan);
    if (entity.child_count() > 0) {
        for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
            apply_tree(entity.child_mutable(child_idx), attributes, fan, source, views, run_ctx);
        return;
    }
    if (entity.point_count() == 0)
        return;
    if (!attributes)
        throw std::runtime_error("Printable extrusion leaf has no effective EPropertyAttributes; "
                                 "the regional fan plugin cannot identify its role.");
    const raw_extrusion_role role = attributes->extrusion_role();
    if (!is_fan_deposit(entity, role) || !fan.can_modify(FAN_REQUEST_MODE_OVERRIDE))
        return;

    // Skirt, brim and support lie outside the region masks: splitting them would
    // leave uncovered pieces.
    if (RAW_EXTRUSION_ROLE_IS_SKIRT(role) || RAW_EXTRUSION_ROLE_IS_SUPPORT(role)) {
        set_request(entity, source.outside.for_role(role));
        return;
    }
    if (!source.requires_split()) {
        set_request(entity, source.uniform.for_role(role));
        return;
    }

    // Regions may still agree for this role: then no split is needed.
    std::vector<std::optional<float>> by_area;
    for (const Requests &requests : source.requests)
        by_area.push_back(requests.for_role(role));
    if (std::all_of(by_area.begin(), by_area.end(),
                    [&by_area](const std::optional<float> &request) { return request == by_area.front(); })) {
        set_request(entity, by_area.front());
        return;
    }

    // The public splitter replaces this leaf and returns borrowed handles to its
    // fragments; handle them here so the traversal does not visit them again.
    reject_one_shot_events(entity);
    const double source_length = double(entity.local_length());
    const std::vector<ExtrusionAreaFragment> fragments = entity.split_leaf_by_areas(views);
    validate_split_coverage(source_length, fragments);
    for (const ExtrusionAreaFragment &fragment : fragments) {
        if (fragment.area_index >= by_area.size())
            throw std::runtime_error("Regional fan splitter returned an invalid area index.");
        set_request(fragment.entity, by_area[fragment.area_index]);
    }
}

} // namespace nematx_regional_fan
