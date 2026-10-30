///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include "FanPolicy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "ConfigViews.hpp"

/*
These SDK stages provide fan references without reading or changing a plan.
Settings copy one filament's configuration; each stage returns a new value so
downstream plugins can distinguish an absent option, a stop and a computed value.

    read_settings(): copy and normalize configuration for one tool
    make_base(): apply family/default fallbacks before timing
    apply_layer_time(): raise eligible roles using whole-layer duration
    close_first_layers(): apply disable/ramp rules and late fallbacks
    transform_request(): retain protected request pairs during leaf assignment

Contexts retain no borrowed geometry, configuration or device. Validation rejects
invalid phases and numeric inputs before a caller can publish the result.
*/
namespace slic3r_api { namespace FanPolicy {

void Context::validate(raw_fan_policy_phase expected) const
{
    if ((expected != RAW_FAN_POLICY_PHASE_BASE && expected != RAW_FAN_POLICY_PHASE_TIME &&
         expected != RAW_FAN_POLICY_PHASE_CLOSED) ||
        struct_size != sizeof(c_fan_policy_context) || phase != uint32_t(expected) ||
        dynamic_enabled > 1 || dynamic_min < 0 || dynamic_max > 100 || dynamic_min > dynamic_max)
        throw std::invalid_argument("Invalid fan context size, phase or dynamic bounds.");
    for (const c_fan_role_reference &role : roles)
        if (role.configured_percent < -1 || role.configured_percent > 100 ||
            role.percent < -1 || role.percent > 100 || role.fallback_role > RAW_FAN_ROLE_COUNT)
            throw std::invalid_argument("Invalid fan role reference.");
}

Role fan_role(raw_extrusion_role role)
{
    if (role == RAW_EXTRUSION_ROLE_GAP_FILL)
        return RAW_FAN_ROLE_GAP_FILL;
    if (RAW_EXTRUSION_ROLE_IS_SUPPORT(role))
        return RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_EXTERNAL) ?
            RAW_FAN_ROLE_SUPPORT_MATERIAL_INTERFACE : RAW_FAN_ROLE_SUPPORT_MATERIAL;
    if (RAW_EXTRUSION_ROLE_IS_SKIRT(role))
        return RAW_FAN_ROLE_SKIRT;
    if (RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_IRONING))
        return RAW_FAN_ROLE_IRONING;
    if (RAW_EXTRUSION_ROLE_IS_INFILL(role) && RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_BRIDGE))
        return RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_EXTERNAL) ?
            RAW_FAN_ROLE_BRIDGE_INFILL : RAW_FAN_ROLE_INTERNAL_BRIDGE_INFILL;
    if (RAW_EXTRUSION_ROLE_IS_INFILL(role) && RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_SOLID))
        return RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_EXTERNAL) ?
            RAW_FAN_ROLE_TOP_SOLID_INFILL : RAW_FAN_ROLE_SOLID_INFILL;
    if (RAW_EXTRUSION_ROLE_IS_INFILL(role))
        return RAW_FAN_ROLE_INTERNAL_INFILL;
    if (role == RAW_EXTRUSION_ROLE_THIN_WALL)
        return RAW_FAN_ROLE_THIN_WALL;
    if (RAW_EXTRUSION_ROLE_IS_PERIMETER(role) && RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_BRIDGE))
        return RAW_FAN_ROLE_OVERHANG_PERIMETER;
    if (RAW_EXTRUSION_ROLE_IS_PERIMETER(role) && RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_EXTERNAL))
        return RAW_FAN_ROLE_EXTERNAL_PERIMETER;
    if (RAW_EXTRUSION_ROLE_IS_PERIMETER(role))
        return RAW_FAN_ROLE_PERIMETER;
    return RAW_FAN_ROLE_COUNT;
}

bool can_increase(Role role)
{
    return role == RAW_FAN_ROLE_DEFAULT || role == RAW_FAN_ROLE_BRIDGE_INFILL ||
           role == RAW_FAN_ROLE_INTERNAL_BRIDGE_INFILL || role == RAW_FAN_ROLE_EXTERNAL_PERIMETER ||
           role == RAW_FAN_ROLE_THIN_WALL || role == RAW_FAN_ROLE_PERIMETER ||
           role == RAW_FAN_ROLE_SOLID_INFILL || role == RAW_FAN_ROLE_INTERNAL_INFILL ||
           role == RAW_FAN_ROLE_OVERHANG_PERIMETER || role == RAW_FAN_ROLE_GAP_FILL;
}

bool can_ramp(Role role)
{
    return role == RAW_FAN_ROLE_DEFAULT || role == RAW_FAN_ROLE_TOP_SOLID_INFILL ||
           role == RAW_FAN_ROLE_IRONING || role == RAW_FAN_ROLE_SUPPORT_MATERIAL ||
           role == RAW_FAN_ROLE_EXTERNAL_PERIMETER || role == RAW_FAN_ROLE_THIN_WALL ||
           role == RAW_FAN_ROLE_PERIMETER || role == RAW_FAN_ROLE_SOLID_INFILL ||
           role == RAW_FAN_ROLE_INTERNAL_INFILL || role == RAW_FAN_ROLE_GAP_FILL;
}

int normalize_fan_value(int value)
{
    return value == 1 ? 0 : value;
}

Settings read_settings(const Config &config, extruder_index_t extruder_id)
{
    Settings settings;
    const char *const keys[RAW_FAN_ROLE_COUNT] = {
        "default_fan_speed", "perimeter_fan_speed", "external_perimeter_fan_speed",
        "overhangs_fan_speed", "infill_fan_speed", "internal_bridge_fan_speed",
        "solid_infill_fan_speed", "top_fan_speed", "top_fan_speed", "bridge_fan_speed",
        "external_perimeter_fan_speed", "gap_fill_fan_speed", "default_fan_speed",
        "support_material_fan_speed", "support_material_interface_fan_speed"
    };
    for (size_t idx = 0; idx < settings.configured.size(); ++idx)
        settings.configured[idx] = normalize_fan_value(config.vector_int_or_default(keys[idx], extruder_id, -1));
    settings.maximum = std::clamp(config.vector_int_or_default("max_fan_speed", extruder_id, 100), 0, 100);
    settings.slowdown_seconds = config.vector_float_or_default("slowdown_below_layer_time", extruder_id, 0.0);
    settings.fan_seconds = config.vector_float_or_default("fan_below_layer_time", extruder_id, 0.0);
    settings.disabled_layers = config.vector_int_or_default("disable_fan_first_layers", extruder_id, 0);
    settings.full_speed_layer = config.vector_int_or_default("full_fan_speed_layer", extruder_id, 0);
    const ConfigOption dynamic = config.get("overhangs_dynamic_fan_speed");
    settings.dynamic_enabled = dynamic.valid() && size_t(extruder_id) < dynamic.size() && dynamic.is_enabled(extruder_id);
    return settings;
}

Context make_base(const Settings &settings, uint32_t layer_index)
{
    Context result = {};
    result.struct_size = sizeof(c_fan_policy_context);
    result.phase = RAW_FAN_POLICY_PHASE_BASE;
    result.layer_index = layer_index;
    result.disabled_layers = settings.disabled_layers;
    result.full_speed_layer = settings.full_speed_layer;
    result.dynamic_enabled = settings.dynamic_enabled ? 1u : 0u;
    result.dynamic_max = 100;
    for (size_t idx = 0; idx < settings.configured.size(); ++idx)
        result.roles[idx] = {settings.configured[idx], settings.configured[idx], RAW_FAN_ROLE_COUNT};

    // These family fallbacks precede cooling and may intentionally inherit a
    // role with different can_increase/can_ramp rules.
    const std::pair<Role, Role> early[] = {
        {RAW_FAN_ROLE_TOP_SOLID_INFILL, RAW_FAN_ROLE_SOLID_INFILL},
        {RAW_FAN_ROLE_SUPPORT_MATERIAL_INTERFACE, RAW_FAN_ROLE_SUPPORT_MATERIAL},
        {RAW_FAN_ROLE_INTERNAL_BRIDGE_INFILL, RAW_FAN_ROLE_BRIDGE_INFILL}
    };
    for (const std::pair<Role, Role> &edge : early)
        if (result.roles[edge.first].percent < 0)
            result.roles[edge.first].percent = result.roles[edge.second].percent;
    if (settings.dynamic_enabled)
        result.roles[RAW_FAN_ROLE_OVERHANG_PERIMETER].percent = -1;
    const int default_speed = result.roles[RAW_FAN_ROLE_DEFAULT].percent;
    if (default_speed >= 0)
        for (size_t idx = 0; idx < settings.configured.size(); ++idx)
            if (result.roles[idx].percent < 0 && idx != RAW_FAN_ROLE_OVERHANG_PERIMETER)
                result.roles[idx].percent = default_speed;

    // The edges describe only late closure. Ironing/thin-wall aliases are
    // refreshed unconditionally by closure after the references transform.
    result.roles[RAW_FAN_ROLE_BRIDGE_INFILL].fallback_role = RAW_FAN_ROLE_DEFAULT;
    result.roles[RAW_FAN_ROLE_INTERNAL_BRIDGE_INFILL].fallback_role = RAW_FAN_ROLE_BRIDGE_INFILL;
    result.roles[RAW_FAN_ROLE_EXTERNAL_PERIMETER].fallback_role = RAW_FAN_ROLE_PERIMETER;
    result.roles[RAW_FAN_ROLE_TOP_SOLID_INFILL].fallback_role = RAW_FAN_ROLE_SOLID_INFILL;
    result.roles[RAW_FAN_ROLE_IRONING].fallback_role = RAW_FAN_ROLE_TOP_SOLID_INFILL;
    result.roles[RAW_FAN_ROLE_THIN_WALL].fallback_role = RAW_FAN_ROLE_EXTERNAL_PERIMETER;
    result.validate(RAW_FAN_POLICY_PHASE_BASE);
    return result;
}

Context apply_layer_time(const Context &base, const Settings &settings, double duration_seconds)
{
    base.validate(RAW_FAN_POLICY_PHASE_BASE);
    if (!std::isfinite(duration_seconds) || duration_seconds < 0.0 ||
        !std::isfinite(settings.slowdown_seconds) || !std::isfinite(settings.fan_seconds) ||
        settings.slowdown_seconds < 0.0 || settings.fan_seconds < 0.0 ||
        settings.maximum < 0 || settings.maximum > 100)
        throw std::invalid_argument("Invalid layer-time fan parameters.");
    Context result = base;
    const bool very_short = duration_seconds < settings.slowdown_seconds && settings.fan_seconds > 0.0;
    const bool interpolate = duration_seconds < settings.fan_seconds && settings.fan_seconds > settings.slowdown_seconds;
    if (very_short || interpolate) {
        const double ratio = very_short ? 0.0 :
            (duration_seconds - settings.slowdown_seconds) / (settings.fan_seconds - settings.slowdown_seconds);
        for (uint32_t idx = 0; idx < RAW_FAN_ROLE_COUNT; ++idx) {
            int32_t &percent = result.roles[idx].percent;
            if (!can_increase(Role(idx)))
                continue;
            if (very_short)
                percent = std::max(settings.maximum, percent);
            else if (percent >= 0 && percent < settings.maximum)
                percent = std::clamp(int(ratio * percent + (1.0 - ratio) * settings.maximum + 0.5), 0, 100);
        }
        result.dynamic_min = std::max(0, result.roles[RAW_FAN_ROLE_DEFAULT].percent);
    }
    result.phase = RAW_FAN_POLICY_PHASE_TIME;
    result.validate(RAW_FAN_POLICY_PHASE_TIME);
    return result;
}

Context close_first_layers(const Context &time)
{
    time.validate(RAW_FAN_POLICY_PHASE_TIME);
    Context result = time;
    result.phase = RAW_FAN_POLICY_PHASE_CLOSED;
    if (int64_t(result.layer_index) < result.disabled_layers) {
        for (c_fan_role_reference &role : result.roles)
            role.percent = 0;
        result.dynamic_min = result.dynamic_max = 0;
        return result;
    }
    if (int64_t(result.layer_index) + 1 < result.full_speed_layer && result.full_speed_layer > result.disabled_layers) {
        const float factor = float(int64_t(result.layer_index) + 1 - result.disabled_layers) /
                             float(result.full_speed_layer - result.disabled_layers);
        for (uint32_t idx = 0; idx < RAW_FAN_ROLE_COUNT; ++idx)
            if (can_ramp(Role(idx)) && result.roles[idx].percent > 0)
                result.roles[idx].percent = std::clamp(int(float(result.roles[idx].percent) * factor + 0.01f), 0, 100);
        result.dynamic_max = std::max(0, result.roles[RAW_FAN_ROLE_DEFAULT].percent);
    }
    result.dynamic_min = std::min(result.dynamic_min, result.dynamic_max);
    const Role late[] = {RAW_FAN_ROLE_BRIDGE_INFILL, RAW_FAN_ROLE_INTERNAL_BRIDGE_INFILL,
                         RAW_FAN_ROLE_EXTERNAL_PERIMETER, RAW_FAN_ROLE_TOP_SOLID_INFILL};
    for (Role role : late)
        if (result.roles[role].percent < 0 && result.roles[role].fallback_role < RAW_FAN_ROLE_COUNT)
            result.roles[role].percent = result.roles[result.roles[role].fallback_role].percent;
    result.roles[RAW_FAN_ROLE_IRONING].percent = result.roles[RAW_FAN_ROLE_TOP_SOLID_INFILL].percent;
    result.roles[RAW_FAN_ROLE_THIN_WALL].percent = result.roles[RAW_FAN_ROLE_EXTERNAL_PERIMETER].percent;
    result.validate(RAW_FAN_POLICY_PHASE_CLOSED);
    return result;
}

EPropertyFan transform_request(const EPropertyFan &request, float percent, fan_request_mode policy_mode)
{
    EPropertyFan result = request;
    if (request.can_modify(policy_mode))
        result.set(percent, request.is_set() ? request.mode : policy_mode);
    return result;
}

}} // namespace slic3r_api::FanPolicy
