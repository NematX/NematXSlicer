///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef plugins_infill_adaptive_paths_hpp_
#define plugins_infill_adaptive_paths_hpp_

#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Plugins/Common/PolygonWorkBuffers.hpp"

struct plugin_run_context;

namespace slic3r_api { namespace Infill { namespace AdaptiveCubic {

namespace work = PluginCommon::polygon_work;

work::Polylines connect_hooks(work::Polylines lines, const ExPolygon &boundary,
                              double spacing, double hook_length, double hook_length_max,
                              const plugin_run_context *run_ctx);

}}}
#endif
