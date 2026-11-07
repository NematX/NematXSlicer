///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_rectilinear_sawtooth_hpp_
#define plugins_infill_rectilinear_sawtooth_hpp_

/*
Sawtooth rectilinear extrusion
==============================

The base rectilinear filler supplies the same XY polylines as the legacy
algorithm. This publisher converts each polyline into the legacy sequence of
normal, rising, clearance, descending and normal extrusion paths, preserving
the per-point Z offsets and flow attributes through the plugin API.
*/

#include "libslic3r/Plugins/Common/PolygonWorkBuffers.hpp"

#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"

namespace slic3r_api { namespace Infill { namespace RectilinearPatternsPlugin {
namespace work = PluginCommon::polygon_work;
namespace sawtooth {

void publish_sawtooth(const run_ctx_infill_pattern &ctx,
                      storage_handle *storage,
                      const work::Polylines &lines,
                      const raw_infill_pattern_params &params,
                      float fill_angle);

} // namespace sawtooth
}}} // namespace slic3r_api::Infill::RectilinearPatternsPlugin

#endif // plugins_infill_rectilinear_sawtooth_hpp_
