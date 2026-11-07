///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include "TemperatureTargets.hpp"

#include <limits>
#include <stdexcept>

#include "ConfigViews.hpp"

/* Supply shared target selection to built-in and external C++ providers.
   Public Config views resolve missing and disabled settings without mutating
   their owner. Regional overrides are checked before filament fallbacks, and
   conversion happens only for the selected value, so unused fallbacks do not
   produce errors. No heater state is read or cached and no events are emitted.

       extrusion_target(): regional first-layer, then normal override
           |-- heater_temperature(): validate a selected regional value
           `-- filament_target(): first-layer, then normal positive filament value
               `-- heater_temperature(): validate the selected filament value
*/

namespace slic3r_api { namespace TemperatureTargets {
namespace {

// Convert a selected non-negative target to the heater ABI value; reject overflow
// with the same error as other configured heater requests.
std::optional<int16_t> heater_temperature(int32_t value);

} // namespace

std::optional<int16_t> extrusion_target(const Config &print_config,
                                      const Config &region_config,
                                      const extruder_index_t tool,
                                      const bool first_layer)
{
    if (first_layer) {
        const int32_t value = region_config.int_or_default("print_first_layer_temperature", -1);
        if (value >= 0)
            return heater_temperature(value);
    }
    const int32_t value = region_config.int_or_default("print_temperature", -1);
    if (value >= 0)
        return heater_temperature(value);
    return filament_target(print_config, tool, first_layer);
}

std::optional<int16_t> filament_target(const Config &print_config,
                                     const extruder_index_t tool,
                                     const bool first_layer)
{
    if (first_layer) {
        const int32_t first = print_config.vector_int_or_default("first_layer_temperature", tool, -1);
        if (first > 0)
            return heater_temperature(first);
    }
    const int32_t normal = print_config.vector_int_or_default("temperature", tool, -1);
    if (normal > 0)
        return heater_temperature(normal);
    return std::nullopt;
}

namespace {

std::optional<int16_t> heater_temperature(const int32_t value)
{
    if (value > (std::numeric_limits<int16_t>::max)())
        throw std::invalid_argument("A configured heater temperature is out of range.");
    return static_cast<int16_t>(value);
}

} // namespace
}} // namespace slic3r_api::TemperatureTargets
