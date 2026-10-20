///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GCodeExportMetadataViews_hpp_
#define slic3r_GCodeExportMetadataViews_hpp_

#include <cstddef>
#include <stdexcept>
#include <string_view>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_export.h"

/*
G-code export metadata views
============================

The host lends export metadata only during one synchronous EXPORT call. These
views copy no strings or entry arrays; a pair of empty extrusion-width strings
remains visible as the separator between regions. The statistics list exists from
ABI 2.2; a shorter structure (2.1) is presented as an empty list.

Construct GCodeExportMetadataView from the metadata pointer lent to export_impl(),
then read its configuration, width and statistics views during that call only.
The views do not extend the lifetime of any host array or string. Read text entries through configuration(),
extrusion_widths() and statistics(); copy any text needed after EXPORT returns.
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

class GCodeExportMetadataView
{
public:
    // Borrow metadata and reject a null pointer, short 2.1 prefix, or invalid list pointer before reading entries.
    explicit GCodeExportMetadataView(const raw_gcode_export_metadata *metadata) : m_metadata(metadata)
    {
        const std::size_t minimum_size =
            offsetof(raw_gcode_export_metadata, extrusion_widths) + sizeof(metadata->extrusion_widths);
        if (m_metadata == nullptr || m_metadata->struct_size < minimum_size)
            throw std::invalid_argument("G-code export metadata has an incomplete 2.1 prefix.");
        validate_list(m_metadata->configuration);
        validate_list(m_metadata->extrusion_widths);
        if (has_statistics_field())
            validate_list(m_metadata->statistics);
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

    // Return a borrowed indexed view of the print statistics; empty when the host sent none or the structure predates ABI 2.2.
    GCodeExportMetadataListView statistics() const
    {
        return GCodeExportMetadataListView(has_statistics_field() ? m_metadata->statistics : raw_gcode_export_list{});
    }

private:
    // Tell whether the borrowed structure is large enough to hold the ABI 2.2 statistics list.
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
