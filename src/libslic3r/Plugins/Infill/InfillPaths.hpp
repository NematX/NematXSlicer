///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef plugins_infill_paths_hpp_
#define plugins_infill_paths_hpp_

// Copies bridge API-owned geometry and mutable work buffers; chaining orders independent paths without joining them.
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Plugins/Common/WorkBuffers.hpp"

struct plugin_run_context;

namespace slic3r_api { namespace Infill { namespace Paths {
namespace work = slic3r_api::PluginCommon::work;

// API collections own their payload; moving each stored path avoids a second coordinate copy.
StoredPolylineCollection store_polylines(storage_handle *storage, const work::Polylines &lines,
                                         const plugin_run_context *run_ctx = nullptr);

// Borrow public point arrays only during the copy; connectors retain independent mutable buffers.
work::Polylines read_polylines(const PolylineCollection &lines, const plugin_run_context *run_ctx = nullptr);

// Greedy chaining and two exchanges preserve independent paths rather than extruding across gaps.
work::Polylines chain_polylines(work::Polylines lines, const plugin_run_context *run_ctx);
}}}
#endif
