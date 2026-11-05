///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GCodeExportMetadataViews_hpp_
#define slic3r_GCodeExportMetadataViews_hpp_

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string_view>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_export.h"

/*
G-code export metadata views
============================

The host lends export metadata only during one synchronous EXPORT call. These
views copy no strings or entry arrays; a pair of empty extrusion-width strings
remains visible as the separator between regions. The statistics list was appended
after the two first lists; a structure that stops before it is presented as an empty list.

The numeric statistics exist from export contract 4.2 and are appended as one group after
the statistics list: a structure that stops before the end of the group (a 4.1 structure,
which ends right after the statistics list, or a shorter one) is presented with no numeric
statistics (has_numeric_statistics() is false and every value reads zero). Their units are seconds, millimeters, cubic
millimeters and grams. layer_count and max_print_z are plan facts, valid once STEP_GCODE
has finished (zero before). The times and the material (the per-tool list and the totals)
are measured: they are valid if and only if the numeric statistics exist and the statistics
list is not empty (has_measured_statistics()), else zero. The per-tool list borrows the host memory like the
text lists and is valid during the EXPORT call only.

Construct GCodeExportMetadataView from the metadata pointer lent to export_impl(),
then read its configuration, width and statistics views during that call only.
The views do not extend the lifetime of any host array or string. When using
numeric statistics, test their availability before interpreting a zero value
as a measured result rather than a missing field or an unread print.
*/

namespace slic3r_api {

struct GCodeExportMetadataEntryView
{
    std::string_view key;
    std::string_view value;
};

class GCodeExportMetadataListView
{
public:
    // Copy one borrowed list header and reject a non-empty list without entries.
    explicit GCodeExportMetadataListView(const raw_gcode_export_list &list) : m_list(list)
    {
        if (m_list.count != 0 && m_list.items == nullptr)
            throw std::invalid_argument("G-code export metadata list has no entries.");
    }

    // Return the entry count without taking ownership of the list storage.
    std::size_t size() const noexcept { return m_list.count; }

    // Return borrowed UTF-8 strings at one valid index; reject out-of-range indices or null strings.
    GCodeExportMetadataEntryView at(std::size_t index) const
    {
        if (index >= m_list.count)
            throw std::out_of_range("G-code export metadata index is out of range.");
        const raw_gcode_export_entry &entry = m_list.items[index];
        if (entry.key == nullptr || entry.value == nullptr)
            throw std::invalid_argument("G-code export metadata contains a null string.");
        return {std::string_view(entry.key), std::string_view(entry.value)};
    }

private:
    raw_gcode_export_list m_list = {};
};

// The material of one tool, copied by value from the borrowed array.
struct GCodeExportFilamentUsageView
{
    uint32_t tool = 0;
    double length_mm = 0.;
    double volume_mm3 = 0.;
    double weight_g = 0.;
};

// Indexed access to the borrowed per-tool material array.
class GCodeExportFilamentListView
{
public:
    // Copy one borrowed list header and reject a non-empty list without entries.
    explicit GCodeExportFilamentListView(const raw_gcode_export_filament_usage_list &list) : m_list(list)
    {
        if (m_list.count != 0 && m_list.items == nullptr)
            throw std::invalid_argument("G-code export filament list has no entries.");
    }

    // Return the number of tools of the export configuration (zero without measured statistics).
    std::size_t size() const noexcept { return m_list.count; }

    // Return a copy of the material of the tool at one valid position; reject an out-of-range position.
    GCodeExportFilamentUsageView at(std::size_t index) const
    {
        if (index >= m_list.count)
            throw std::out_of_range("G-code export filament index is out of range.");
        const raw_gcode_export_filament_usage &usage = m_list.items[index];
        return {usage.tool, usage.length_mm, usage.volume_mm3, usage.weight_g};
    }

private:
    raw_gcode_export_filament_usage_list m_list = {};
};

// The scalar numeric statistics of one export; all zero when the structure predates contract 4.2.
struct GCodeExportNumericStatisticsView
{
    double estimated_time_seconds = 0.;
    double first_layer_time_seconds = 0.;
    uint32_t layer_count = 0;
    double max_print_z = 0.;
    double total_length_mm = 0.;
    double total_volume_mm3 = 0.;
    double total_weight_g = 0.;
};

class GCodeExportMetadataView
{
public:
    // Borrow metadata and reject a null pointer, short prefix, or invalid list pointer before reading entries.
    explicit GCodeExportMetadataView(const raw_gcode_export_metadata *metadata) : m_metadata(metadata)
    {
        const std::size_t minimum_size =
            offsetof(raw_gcode_export_metadata, extrusion_widths) + sizeof(metadata->extrusion_widths);
        if (m_metadata == nullptr || m_metadata->struct_size < minimum_size)
            throw std::invalid_argument("G-code export metadata has an incomplete prefix.");
        validate_list(m_metadata->configuration);
        validate_list(m_metadata->extrusion_widths);
        if (has_statistics_field())
            validate_list(m_metadata->statistics);
        if (has_numeric_statistics())
            (void)GCodeExportFilamentListView(m_metadata->filaments);
    }

    // Return a borrowed indexed view of the filtered configuration entries.
    GCodeExportMetadataListView configuration() const
    {
        return GCodeExportMetadataListView(m_metadata->configuration);
    }

    // Return a borrowed indexed view of extrusion-width entries and region separators.
    GCodeExportMetadataListView extrusion_widths() const
    {
        return GCodeExportMetadataListView(m_metadata->extrusion_widths);
    }

    // Return a borrowed indexed view of the print statistics; empty when the host sent none or the structure stops before the statistics list.
    GCodeExportMetadataListView statistics() const
    {
        return GCodeExportMetadataListView(has_statistics_field() ? m_metadata->statistics : raw_gcode_export_list{});
    }

    // Tell whether the structure is large enough to carry the numeric statistics (contract 4.2); older hosts give none.
    bool has_numeric_statistics() const noexcept
    {
        return m_metadata->struct_size >=
               offsetof(raw_gcode_export_metadata, total_weight_g) + sizeof(m_metadata->total_weight_g);
    }

    // Tell whether the time and material values are measured: the numeric statistics exist and the statistics list is not
    // empty. It is the only test of validity of the measured values.
    bool has_measured_statistics() const noexcept
    {
        return has_numeric_statistics() && m_metadata->statistics.count != 0;
    }

    // Return a copy of the scalar numeric statistics, all zero when the structure has no numeric group.
    GCodeExportNumericStatisticsView numeric_statistics() const
    {
        GCodeExportNumericStatisticsView view;
        if (!has_numeric_statistics())
            return view;
        view.estimated_time_seconds = m_metadata->estimated_time_seconds;
        view.first_layer_time_seconds = m_metadata->first_layer_time_seconds;
        view.layer_count = m_metadata->layer_count;
        view.max_print_z = m_metadata->max_print_z;
        view.total_length_mm = m_metadata->total_length_mm;
        view.total_volume_mm3 = m_metadata->total_volume_mm3;
        view.total_weight_g = m_metadata->total_weight_g;
        return view;
    }

    // Return a borrowed indexed view of the per-tool material; empty without numeric statistics or measured values.
    GCodeExportFilamentListView filaments() const
    {
        return GCodeExportFilamentListView(has_numeric_statistics() ? m_metadata->filaments :
                                                                      raw_gcode_export_filament_usage_list{});
    }

private:
    // Tell whether the borrowed structure is large enough to hold the statistics list.
    bool has_statistics_field() const noexcept
    {
        return m_metadata->struct_size >= offsetof(raw_gcode_export_metadata, statistics) + sizeof(m_metadata->statistics);
    }

    // Reject any non-empty list whose entry storage is absent before exposing either list.
    static void validate_list(const raw_gcode_export_list &list)
    {
        if (list.count != 0 && list.items == nullptr)
            throw std::invalid_argument("G-code export metadata list has no entries.");
    }

    const raw_gcode_export_metadata *m_metadata = nullptr;
};

} // namespace slic3r_api

#endif // slic3r_GCodeExportMetadataViews_hpp_
