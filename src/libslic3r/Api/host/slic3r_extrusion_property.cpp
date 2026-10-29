///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/internal/ExtrusionPropertyAccess.hpp"
#include "libslic3r/Api/plugin/c/slic3r_extrusion_property.h"
#include "libslic3r/Api/plugin/c/slic3r_bridge.h"
#include "libslic3r/ExtrusionEntity.hpp"

#include "Orchestrator.hpp"

/* Expose native extrusion properties through the C API. Built-in payloads have
   fixed names, sizes and alignments in builtin_property_infos(); custom payload
   definitions belong to the supplied Orchestrator. The bridge flow/strand entries
   are fixed-size payload descriptions, not executions of bridge correction.

   Queries borrow entity/orchestrator handles. Existing payload pointers remain
   owned by the extrusion; new payloads and auxiliary byte resources are allocated
   in that owner's property container. Reacquire pointers after changes to their
   storage. Allocation failures are not caught by this adapter itself; the public
   declarations in slic3r_extrusion_property.h define each operation's contract.

       extrusion_property_get_or_add_data_mutable()
           |-- property_byte_count()
           |   |-- builtin_property_info()
           |   `-- custom_property_info()
           |-- property_alignment()
           `-- ApiInternal::ExtrusionPropertyAccess::get_or_add_property_data_mutable()

   Registration describes payload layouts; property access edits an extrusion.
   Neither operation transfers the owning host or extrusion to the caller.
*/

namespace Slic3r {

struct PropertyInfo
{
    extrusion_property_type type;
    const char *name;
    uint32_t byte_count;
    uint32_t alignment;
};

static ExtrusionEntity *to_extrusion(extrusion_entity_handle *me)
{
    return reinterpret_cast<ExtrusionEntity *>(me);
}

static const ExtrusionEntity *to_extrusion(const extrusion_entity_handle *me)
{
    return reinterpret_cast<const ExtrusionEntity *>(me);
}

static bool is_power_of_two(uint32_t value)
{
    return value != 0 && (value & (value - 1)) == 0;
}

static Orchestrator *to_orchestrator(orchestrator_handle *me)
{
    return me == nullptr ? nullptr : reinterpret_cast<Orchestrator *>(me);
}

static const Orchestrator *to_orchestrator(const orchestrator_handle *me)
{
    return me == nullptr ? nullptr : reinterpret_cast<const Orchestrator *>(me);
}

// Return the process-wide immutable table of built-in payload names, sizes and alignments used by host property lookup.
static const std::vector<PropertyInfo> &builtin_property_infos()
{
    static const std::vector<PropertyInfo> infos = {
        { EXTRUSION_PROPERTY_TYPE_ATTRIBUTES,      "slic3r.extrusion.attributes",      sizeof(c_extrusion_property_attributes),      alignof(c_extrusion_property_attributes) },
        { EXTRUSION_PROPERTY_TYPE_SPEED,           "slic3r.extrusion.speed",           sizeof(c_extrusion_property_speed),           alignof(c_extrusion_property_speed) },
        { EXTRUSION_PROPERTY_TYPE_FAN,             "slic3r.extrusion.fan",             sizeof(c_extrusion_property_fan),             alignof(c_extrusion_property_fan) },
        { EXTRUSION_PROPERTY_TYPE_MODIFIER,        "slic3r.extrusion.modifier",        sizeof(c_extrusion_property_modifier),        alignof(c_extrusion_property_modifier) },
        { EXTRUSION_PROPERTY_TYPE_CUSTOM_GCODE,    "slic3r.extrusion.custom_gcode",    sizeof(c_extrusion_property_custom_gcode),    alignof(c_extrusion_property_custom_gcode) },
        { EXTRUSION_PROPERTY_TYPE_SPECIAL_COMMAND, "slic3r.extrusion.special_command", sizeof(c_extrusion_property_special_command), alignof(c_extrusion_property_special_command) },
        { EXTRUSION_PROPERTY_TYPE_OVERHANG,        "slic3r.extrusion.overhang",        sizeof(c_extrusion_property_overhang),        alignof(c_extrusion_property_overhang) },
        { EXTRUSION_PROPERTY_TYPE_Z_OFFSET,        "slic3r.extrusion.z_offset",        sizeof(c_extrusion_property_z_offset),        alignof(c_extrusion_property_z_offset) },
        { EXTRUSION_PROPERTY_TYPE_PERIMETER,       "slic3r.extrusion.perimeter",       sizeof(c_extrusion_property_perimeter),       alignof(c_extrusion_property_perimeter) },
        { EXTRUSION_PROPERTY_TYPE_INFILL,          "slic3r.extrusion.infill",          sizeof(c_extrusion_property_infill),          alignof(c_extrusion_property_infill) },
        { EXTRUSION_PROPERTY_TYPE_EXTRUSION_AXIS,  "slic3r.extrusion.axis",            sizeof(c_extrusion_property_extrusion_axis),  alignof(c_extrusion_property_extrusion_axis) },
        { EXTRUSION_PROPERTY_TYPE_HEATER,           "slic3r.extrusion.heater",          sizeof(c_extrusion_property_heater),          alignof(c_extrusion_property_heater) },
        { SLIC3R_PROPERTY_TYPE_BRIDGE_VOLUME, "bridge_preparation.volume.v1", sizeof(c_bridge_volume_property), alignof(c_bridge_volume_property) },
        { SLIC3R_PROPERTY_TYPE_BRIDGE_STRAND, "bridge_preparation.atomic_strand.v1", sizeof(c_bridge_strand_property), alignof(c_bridge_strand_property) },
    };
    return infos;
}

static const PropertyInfo *builtin_property_info(extrusion_property_type type)
{
    const std::vector<PropertyInfo> &infos = builtin_property_infos();
    for (const PropertyInfo &info : infos)
        if (info.type == type)
            return &info;
    return nullptr;
}

static const Orchestrator::CustomExtrusionPropertyInfo*
custom_property_info(const Orchestrator *orchestrator, extrusion_property_type type)
{
    return orchestrator == nullptr ? nullptr : orchestrator->custom_extrusion_property_info(type);
}

static const Orchestrator::CustomExtrusionPropertyInfo*
custom_property_info(const Orchestrator *orchestrator, const char *name)
{
    return orchestrator == nullptr ? nullptr : orchestrator->custom_extrusion_property_info(name);
}

static uint32_t property_byte_count(const Orchestrator *orchestrator, extrusion_property_type type)
{
    if (const PropertyInfo *info = builtin_property_info(type))
        return info->byte_count;
    if (const Orchestrator::CustomExtrusionPropertyInfo *info = custom_property_info(orchestrator, type))
        return info->byte_count;
    return 0;
}

static uint32_t property_alignment(const Orchestrator *orchestrator, extrusion_property_type type)
{
    if (const PropertyInfo *info = builtin_property_info(type))
        return info->alignment;
    if (const Orchestrator::CustomExtrusionPropertyInfo *info = custom_property_info(orchestrator, type))
        return info->alignment;
    return 0;
}

} // namespace Slic3r

extern "C" {

extrusion_property_type extrusion_property_register_type(orchestrator_handle *orch,
                                                         const char *namespaced_name,
                                                         uint32_t byte_count,
                                                         uint32_t alignment)
{
    if (namespaced_name == nullptr || namespaced_name[0] == '\0' ||
        byte_count == 0 || !Slic3r::is_power_of_two(alignment))
        return EXTRUSION_PROPERTY_TYPE_INVALID;

    const std::vector<Slic3r::PropertyInfo> &builtin_infos = Slic3r::builtin_property_infos();
    for (const Slic3r::PropertyInfo &info : builtin_infos)
        if (std::string(info.name) == namespaced_name)
            return EXTRUSION_PROPERTY_TYPE_INVALID;

    Slic3r::Orchestrator *orchestrator = Slic3r::to_orchestrator(orch);
    return orchestrator == nullptr ?
        EXTRUSION_PROPERTY_TYPE_INVALID :
        orchestrator->register_custom_extrusion_property(namespaced_name, byte_count, alignment);
}

uint32_t extrusion_property_byte_count(const orchestrator_handle *orch, extrusion_property_type type)
{
    return Slic3r::property_byte_count(Slic3r::to_orchestrator(orch), type);
}

uint32_t extrusion_property_alignment(const orchestrator_handle *orch, extrusion_property_type type)
{
    return Slic3r::property_alignment(Slic3r::to_orchestrator(orch), type);
}

const char *extrusion_property_name(const orchestrator_handle *orch, extrusion_property_type type)
{
    if (const Slic3r::PropertyInfo *info = Slic3r::builtin_property_info(type))
        return info->name;
    if (const Slic3r::Orchestrator::CustomExtrusionPropertyInfo *info =
            Slic3r::custom_property_info(Slic3r::to_orchestrator(orch), type))
        return info->name.c_str();
    return nullptr;
}

uint32_t extrusion_property_count(const extrusion_entity_handle *entity)
{
    if (entity == nullptr)
        return 0;
    return static_cast<uint32_t>(
        Slic3r::ApiInternal::ExtrusionPropertyAccess::property_count(*Slic3r::to_extrusion(entity)));
}

extrusion_property_type extrusion_property_type_at(const extrusion_entity_handle *entity, uint32_t idx)
{
    if (entity == nullptr)
        return EXTRUSION_PROPERTY_TYPE_INVALID;
    return Slic3r::ApiInternal::ExtrusionPropertyAccess::property_type_at(*Slic3r::to_extrusion(entity), idx);
}

int32_t extrusion_property_has(const extrusion_entity_handle *entity, extrusion_property_type type)
{
    return entity != nullptr &&
        Slic3r::ApiInternal::ExtrusionPropertyAccess::has_property(*Slic3r::to_extrusion(entity), type);
}

const void *extrusion_property_data(const extrusion_entity_handle *entity, extrusion_property_type type)
{
    if (entity == nullptr)
        return nullptr;
    return Slic3r::ApiInternal::ExtrusionPropertyAccess::property_data(*Slic3r::to_extrusion(entity), type);
}

void *extrusion_property_data_mutable(extrusion_entity_handle *entity, extrusion_property_type type)
{
    if (entity == nullptr)
        return nullptr;
    return Slic3r::ApiInternal::ExtrusionPropertyAccess::property_data_mutable(*Slic3r::to_extrusion(entity), type);
}

void *extrusion_property_get_or_add_data_mutable(orchestrator_handle *orch,
                                                 extrusion_entity_handle *entity,
                                                 extrusion_property_type type)
{
    if (entity == nullptr)
        return nullptr;

    const Slic3r::Orchestrator *orchestrator = Slic3r::to_orchestrator(orch);
    const uint32_t byte_count = Slic3r::property_byte_count(orchestrator, type);
    const uint32_t alignment = Slic3r::property_alignment(orchestrator, type);
    if (byte_count == 0 || alignment == 0)
        return nullptr;
    return Slic3r::ApiInternal::ExtrusionPropertyAccess::get_or_add_property_data_mutable(
        *Slic3r::to_extrusion(entity), type, byte_count, alignment);
}

int32_t extrusion_property_remove(extrusion_entity_handle *entity, extrusion_property_type type)
{
    return entity != nullptr &&
        Slic3r::ApiInternal::ExtrusionPropertyAccess::remove_property(*Slic3r::to_extrusion(entity), type);
}

extrusion_data_id extrusion_store_data_aligned(extrusion_entity_handle *entity,
                                               const void *data,
                                               uint32_t byte_size,
                                               uint32_t alignment)
{
    if (entity == nullptr || !Slic3r::is_power_of_two(alignment) || (byte_size > 0 && data == nullptr))
        return EXTRUSION_DATA_ID_INVALID;
    return Slic3r::ApiInternal::ExtrusionPropertyAccess::store_data_aligned(
        *Slic3r::to_extrusion(entity), data, byte_size, alignment);
}

extrusion_data_id extrusion_property_store_data_aligned(extrusion_entity_handle *entity,
                                                        extrusion_property_type owner_type,
                                                        extrusion_data_id *field,
                                                        const void *data,
                                                        uint32_t byte_size,
                                                        uint32_t alignment)
{
    if (entity == nullptr || owner_type == EXTRUSION_PROPERTY_TYPE_INVALID ||
        field == nullptr || !Slic3r::is_power_of_two(alignment) || (byte_size > 0 && data == nullptr))
        return EXTRUSION_DATA_ID_INVALID;
    return Slic3r::ApiInternal::ExtrusionPropertyAccess::store_property_data_aligned(
        *Slic3r::to_extrusion(entity), owner_type, field, data, byte_size, alignment);
}

const void *extrusion_data(const extrusion_entity_handle *entity,
                           extrusion_data_id data_id,
                           uint32_t *byte_size_out)
{
    if (byte_size_out != nullptr)
        *byte_size_out = 0;
    if (entity == nullptr || data_id == EXTRUSION_DATA_ID_INVALID)
        return nullptr;
    return Slic3r::ApiInternal::ExtrusionPropertyAccess::stored_data(
        *Slic3r::to_extrusion(entity), data_id, byte_size_out);
}

int32_t extrusion_free_data(extrusion_entity_handle *entity, extrusion_data_id data_id)
{
    return entity != nullptr && data_id != EXTRUSION_DATA_ID_INVALID &&
        Slic3r::ApiInternal::ExtrusionPropertyAccess::free_data(*Slic3r::to_extrusion(entity), data_id);
}

} // extern "C"
