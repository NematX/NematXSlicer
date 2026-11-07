///|/ Copyright (c) NematX 2026
///|/ Released under the terms of the AGPLv3 or higher
/* Rounded extrusion flow of the wipe tower lines.

The host's Flow class is not part of the plugin SDK. The tower only needs the
non-bridge flow built from a line width (Flow::new_from_width): the line is a
rectangle with half circles at its ends. This helper repeats the host formulas,
in float like the host, so that the tower lines keep the same spacing and the
same extruded volume as with the default wipe tower manager. */
#ifndef nematx_wipetower_TowerFlow_hpp_
#define nematx_wipetower_TowerFlow_hpp_

#include <stdexcept>

#include "libslic3r/Api/plugin/c/slic3r_def.h"

namespace nematx_wipetower {

struct TowerFlow {
    float width = 0.f;
    float height = 0.f;
    float nozzle_diameter = 0.f;
    // Ratio applied to the overlap between two neighbouring lines; the tower always uses 1.
    float spacing_ratio = 1.f;

    // Line width in mm, nozzle diameter in mm, layer height in mm (Flow::new_from_width).
    static TowerFlow from_width(float width, float nozzle_diameter, float height, float spacing_ratio = 1.f)
    {
        if (height <= 0.f)
            throw std::invalid_argument("A wipe tower flow needs a positive layer height.");
        return TowerFlow{width, height, nozzle_diameter, spacing_ratio};
    }
    // Distance in mm between the centres of two neighbouring lines (Flow::spacing()).
    float spacing() const
    {
        const float result = float(width - height * (1. - 0.25 * k_pi) * spacing_ratio);
        if (result <= 0.f)
            throw std::invalid_argument("A wipe tower flow has a negative spacing.");
        return result;
    }
    coord_t scaled_width() const { return scale_i(width); }
    coord_t scaled_height() const { return scale_to_layer_coord(height); }
    coord_t scaled_spacing() const { return scale_i(spacing()); }
    // Extruded volume per mm of line (Flow::mm3_per_mm()).
    double mm3_per_mm() const
    {
        const float result = float(height * (width - height * (1. - 0.25 * k_pi)));
        if (result <= 0.)
            throw std::invalid_argument("A wipe tower flow has a negative cross section.");
        return result;
    }

    static constexpr double k_pi = 3.14159265358979323846;
};

} // namespace nematx_wipetower

#endif
