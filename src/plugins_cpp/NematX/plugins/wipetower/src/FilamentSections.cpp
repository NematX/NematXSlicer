///|/ Copyright (c) NematX 2026
///|/ Released under the terms of the AGPLv3 or higher
#include "FilamentSections.hpp"

#include <algorithm>

/* Section planning in two passes over the band heights: the first pass sums
the heights of each tool on each layer and keeps the largest sum as the depth of
the tool's section; once every depth is known, the sections are placed one
after the other by tool number, and the second pass places each band at its
tool's cursor on its layer.

    layout_sections    depths, section starts, band starts
    find_section */

namespace nematx_wipetower {

SectionLayout layout_sections(const std::vector<std::vector<SectionBandInput>> &layers, const double gap)
{
    SectionLayout result;
    result.used.resize(layers.size());
    std::map<int32_t, double> depth;
    for (size_t layer = 0; layer < layers.size(); ++layer)
        for (const SectionBandInput &band : layers[layer]) {
            double &used = result.used[layer][band.tool];
            used += band.height;
            depth[band.tool] = std::max(depth[band.tool], used);
        }
    double y = 0.;
    for (const auto &[tool, length] : depth) {
        if (length <= 0.)
            continue;
        result.sections.push_back({tool, y, length});
        y += length + gap;
    }
    result.band_y0.resize(layers.size());
    for (size_t layer = 0; layer < layers.size(); ++layer) {
        std::map<int32_t, double> cursor;
        for (const SectionBandInput &band : layers[layer]) {
            const FilamentSection *section = find_section(result, band.tool);
            // An empty band of a tool without a section stays at the tower start, with no height.
            const double start = section == nullptr ? 0. : section->y0;
            result.band_y0[layer].push_back(start + cursor[band.tool]);
            cursor[band.tool] += band.height;
        }
    }
    return result;
}

const FilamentSection *find_section(const SectionLayout &layout, const int32_t tool)
{
    for (const FilamentSection &section : layout.sections)
        if (section.tool == tool)
            return &section;
    return nullptr;
}

} // namespace nematx_wipetower
