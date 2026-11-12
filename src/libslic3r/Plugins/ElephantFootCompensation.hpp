///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2019 - 2023 Vojtěch Bubník @bubnikv, Lukáš Hejl @hejllukas, Tomáš Mészáros @tamasmeszaros
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Plugins_ElephantFootCompensation_hpp_
#define slic3r_Plugins_ElephantFootCompensation_hpp_

#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

/*
First-layer XY compensation and the AdvancedTravel attractor domain use this
SDK-only helper to withdraw material without erasing narrow walls. Callers supply
the withdrawal distance and minimum width. The returned polygons belong to the
caller storage and the input remains borrowed and unchanged. For example:

    auto result = elephant_foot_compensation(storage, island, width_mm, scale_d(0.1));

The width combines external-perimeter width and spacing; the compensation is a
positive scaled distance. See the implementation for its native-parity rules.
*/

namespace slic3r_api { namespace XYSizeCompensationPlugin { namespace detail {

// Shared SDK helper implementing the active native ElephantFoot path.
// compensation_scaled is a positive distance in scaled coordinate units.
StoredExPolygonCollection elephant_foot_compensation(storage_handle *storage,
                                                     const ExPolygon &input,
                                                     double minimum_contour_width_mm,
                                                     coord_t compensation_scaled);

}}} // namespace slic3r_api::XYSizeCompensationPlugin::detail

#endif // slic3r_Plugins_ElephantFootCompensation_hpp_
