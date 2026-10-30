///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#pragma once

// Typed registration and validation for the public per-layer timing record.
// No estimator or mutable plan is retained by the payload or its key.
// Register once with printing_layer_time_property_key(host), then validate a
// borrowed layer property before consuming its whole-layer duration:
//
//     const auto key = printing_layer_time_property_key(host);
//     if (const auto *time = layer.properties().get(key))
//         time->validate();
//
// Numeric keys belong to that host. Republish timing after changing speeds;
// RESOLVED describes a timing-model result, not a measured machine duration.
#include <cmath>
#include <stdexcept>

#include "../c/slic3r_printing_layer_time.h"
#include "PluginPropertyKey.hpp"

namespace slic3r_api {

struct PrintingLayerTimeProperty : c_printing_layer_time_property
{
    // Identify a timing-model result produced after speed resolution.
    bool is_resolved() const { return origin == RAW_PRINTING_LAYER_TIME_ORIGIN_RESOLVED; }
    // Throw invalid_argument for an unknown origin or a nonfinite/negative duration.
    void validate() const {
        if ((origin != RAW_PRINTING_LAYER_TIME_ORIGIN_ESTIMATED && !is_resolved()) ||
            !std::isfinite(duration_seconds) || duration_seconds < 0.0)
            throw std::invalid_argument("Invalid PrintingLayerGroup duration or origin.");
    }
};

// Retain the returned key per plugin/orchestrator, never a process-global ID.
inline PluginPropertyKey<PrintingLayerTimeProperty>
printing_layer_time_property_key(orchestrator_handle *orchestrator)
{
    return PluginPropertyKey<PrintingLayerTimeProperty>::register_dynamic(
        orchestrator, PRINTING_LAYER_TIME_PROPERTY_NAME);
}

static_assert(sizeof(PrintingLayerTimeProperty) == sizeof(c_printing_layer_time_property), "Timing ABI layout");
static_assert(alignof(PrintingLayerTimeProperty) == alignof(c_printing_layer_time_property), "Timing ABI alignment");

} // namespace slic3r_api
