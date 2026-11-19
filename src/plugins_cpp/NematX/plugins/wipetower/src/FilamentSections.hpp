///|/ Copyright (c) NematX 2026
///|/ Released under the terms of the AGPLv3 or higher
/* Filament sections of a wipe tower (wipe_tower_separate_filament).

With separate sections, each tool purges only in its own band of the tower
depth (local Y), so that incompatible materials never need to stick together.
The sections are planned once for the whole print, so they keep the same place
on every layer: a section is as deep as the most that its tool purges on one
layer, the sections follow each other by tool number, and a gap separates the
purge areas of two neighbouring sections. On each layer, the bands of a tool
follow each other from the start of its section, in their layout order.

Units are millimetres in the tower's local frame; the inputs are band heights
from the regular layout (WipeTowerLayout.hpp).

    std::vector<std::vector<SectionBandInput>> layers = ...; // per tower layer, in band order
    const SectionLayout sections = layout_sections(layers, gap);
    const double y0 = sections.band_y0[layer][band];
*/
#ifndef nematx_wipetower_FilamentSections_hpp_
#define nematx_wipetower_FilamentSections_hpp_

#include <cstdint>
#include <map>
#include <vector>

namespace nematx_wipetower {

struct SectionBandInput {
    // Tool that prints the band, and the band height along local Y.
    int32_t tool;
    double height;
};
struct FilamentSection {
    int32_t tool;
    // Start of the section's purge area along local Y, and its depth.
    double y0;
    double length;
};
struct SectionLayout {
    // Sections in tool order; a tool whose bands are all empty gets no section.
    std::vector<FilamentSection> sections;
    // Start of each band along local Y, per layer and in input order.
    std::vector<std::vector<double>> band_y0;
    // Height used by each tool on each layer, from the start of its section.
    std::vector<std::map<int32_t, double>> used;
};

// Plan the sections of a tower from the band heights of every layer; gap is the
// distance between the purge areas of two neighbouring sections.
SectionLayout layout_sections(const std::vector<std::vector<SectionBandInput>> &layers, double gap);

// Return the section of a tool, or null when the tool has none.
const FilamentSection *find_section(const SectionLayout &layout, int32_t tool);

} // namespace nematx_wipetower

#endif
