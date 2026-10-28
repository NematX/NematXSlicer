///|/ Copyright (c) 2026 NematX
///|/ Released under the GNU Affero General Public License, version 3 or later.
#pragma once
#include "ExtrusionViews.hpp"

namespace slic3r_api {
/*
Late speed editors read this metadata instead of applying a percentage to an
already capped speed. Register the same key against the host orchestrator, then
read it from resolved first-layer leaves:

    const auto key = default_speed_limits_key(orchestrator);
    const DefaultSpeedLimits *limits = leaf.get(key);

The borrowed payload belongs to the extrusion. Read version 1 only; zero means
no first-layer ceiling. The provider attaches no metadata to upstream overrides.
Deep clones and regional splits copy the payload independently of speed hoisting.
*/

// Published by the default speed provider on resolved first-layer leaves.
// Absolute ceiling evaluated against the original role/autospeed base BEFORE
// first-layer minimums and physical caps. Zero means no first-layer ceiling.
// This is metadata, not a firmware speed request. Inheritance replaces the
// entire payload. Regional splits and deep clones must preserve it. Consumers
// must not reconstruct it by applying a percentage to the final speed.
struct DefaultSpeedLimits {
    double first_layer_mm_per_s = 0.;
    uint32_t version = 1;
};
// Register or retrieve the host-owned dynamic key; no extrusion is changed.
inline PluginPropertyKey<DefaultSpeedLimits> default_speed_limits_key(orchestrator_handle *orchestrator) {
    return PluginPropertyKey<DefaultSpeedLimits>::register_dynamic(orchestrator, "slic3r.default_speed.limits.v1");
}
} // namespace slic3r_api
