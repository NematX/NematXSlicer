// Kernel tests of the filament section planning of the wipe tower; no host is needed.
#include <cmath>
#include <cstdio>
#include <vector>

#include "FilamentSections.hpp"

namespace {
using namespace nematx_wipetower;

int failures = 0;

void check(bool condition, const char *message)
{
    if (!condition) {
        std::printf("FAILED: %s\n", message);
        ++failures;
    }
}

bool near(double a, double b) { return std::abs(a - b) < 1e-9; }
} // namespace

int main()
{
    // Tool 2 purges 3 mm then 1 mm on layer 0 and 2 mm on layer 1; tool 0 purges 1 mm on layer 1;
    // tool 1 only has an empty band. Sections follow the tool order, as deep as the largest use of
    // one layer, with a 0.5 mm gap between them; a tool without height gets no section.
    const std::vector<std::vector<SectionBandInput>> layers = {
        {{2, 3.}, {1, 0.}, {2, 1.}},
        {{0, 1.}, {2, 2.}},
    };
    const SectionLayout layout = layout_sections(layers, 0.5);
    check(layout.sections.size() == 2, "two sections");
    check(layout.sections[0].tool == 0 && near(layout.sections[0].y0, 0.) && near(layout.sections[0].length, 1.),
          "tool 0 section starts the tower and is 1 mm deep");
    check(layout.sections[1].tool == 2 && near(layout.sections[1].y0, 1.5) && near(layout.sections[1].length, 4.),
          "tool 2 section follows after the gap and is 4 mm deep");
    check(find_section(layout, 1) == nullptr, "an empty band makes no section");
    check(near(layout.band_y0[0][0], 1.5) && near(layout.band_y0[0][2], 4.5),
          "the bands of one tool follow each other from the start of its section");
    check(near(layout.band_y0[0][1], 0.), "an empty band without section stays at the tower start");
    check(near(layout.band_y0[1][0], 0.) && near(layout.band_y0[1][1], 1.5), "each layer restarts at the section starts");
    check(near(layout.used[0].at(2), 4.) && near(layout.used[1].at(2), 2.) && near(layout.used[1].at(0), 1.),
          "the used height of each tool on each layer");
    check(layout_sections({}, 1.).sections.empty(), "no layer, no section");
    if (failures != 0)
        return 1;
    std::printf("wipe tower kernel tests passed\n");
    return 0;
}
