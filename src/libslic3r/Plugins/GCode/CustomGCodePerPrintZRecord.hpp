///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_CustomGCodePerPrintZRecord_hpp_
#define slic3r_Plugins_GCode_CustomGCodePerPrintZRecord_hpp_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "libslic3r/CustomGCode.hpp"

namespace slic3r_api {
class Print;
class PrintingExtrusion;
class PrintingGroup;
class PrintingPlan;
class ConstPrintingPlan;
class ConstPrintingGroup;
class ConstPrintingExtrusion;
class ConstPrintingLayerGroup;
}

namespace slic3r_api::GCodeGeneration::CustomGCodePerPrintZPlugin {

/*
Validated row copied from the host-owned custom_gcode_per_print_z PrintRecord.

The record remains a generic Config table at the ABI boundary. Converting it
once into this value type gives every consumer the same enum checks, column
length checks and model-order semantics without retaining borrowed Config
handles while the PrintingPlan is mutated.
*/
struct CustomGCodeRow
{
    double print_z_mm = 0.0;
    Slic3r::CustomGCode::Type type = Slic3r::CustomGCode::Custom;
    int32_t extruder = 0;
    std::string color;
    std::string extra;
    uint32_t source_row = 0;
};

struct CustomGCodeTable
{
    Slic3r::CustomGCode::Mode mode = Slic3r::CustomGCode::Undef;
    std::vector<CustomGCodeRow> rows;
};

/*
Read and validate the complete record before any plan mutation begins.

An absent channel is accepted as an empty optional so older hosts or tests may
run the plugins without publishing this host-owned table. A present but invalid
channel throws std::runtime_error and must abort the current plugin pass.
*/
std::optional<CustomGCodeTable> read_custom_gcode_per_print_z(const Print &print);

/*
Convert one validated record tool number from its stored one-based form to the
zero-based identity used by the PrintingPlan. The bounds check is repeated at
the conversion boundary so direct test tables cannot bypass the record reader.
*/
extruder_index_t custom_gcode_row_extruder_index(const CustomGCodeRow &row,
                                                 uint32_t extruder_count);

/* Resolve the old Undef mode with the same semantic rules as the Model helper. */
Slic3r::CustomGCode::Mode effective_record_mode(const CustomGCodeTable &table);

/* Infer the print mode from the original region-island tools of planned object extrusions. */
Slic3r::CustomGCode::Mode printing_plan_mode(const Print &print, const ConstPrintingPlan &plan);

/* Return true only for extrusion roots sourced from a normal object layer. */
bool printing_extrusion_is_object(const ConstPrintingPlan &plan,
                                  const ConstPrintingGroup &group,
                                  const ConstPrintingExtrusion &extrusion);

// Rows indexed by plan group and layer; pointers borrow the supplied table.
using CustomGCodePlanRows = std::vector<std::vector<std::vector<const CustomGCodeRow *>>>;
// Owned optional first-tool targets indexed by plan group, then layer; no value imposes no tool.
using CustomGCodeColorTargets = std::vector<std::vector<std::optional<extruder_index_t>>>;

// Resolve the required first color-event tool at each plan location, rejecting conflicts.
CustomGCodeColorTargets color_event_targets(const CustomGCodeTable &table, const ConstPrintingPlan &plan,
    Slic3r::CustomGCode::Mode print_mode, uint32_t extruder_count);

/* Map each row once to the first plan layer at or above its Z, preserving row order. */
CustomGCodePlanRows rows_by_plan(
    const CustomGCodeTable &table,
    const ConstPrintingPlan &plan);

} // namespace slic3r_api::GCodeGeneration::CustomGCodePerPrintZPlugin

#endif // slic3r_Plugins_GCode_CustomGCodePerPrintZRecord_hpp_
