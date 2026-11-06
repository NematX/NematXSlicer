// JSON projection of a slicing result for get_gcode_for_view (spec §5.15):
// the per-result header computed once at publication, and sparse move pages.
#pragma once

#include <memory>
#include <optional>

#include <nlohmann/json.hpp>

#include "engine.hpp"
#include "session.hpp"

namespace Slic3r::Rest {

// Shortest decimal that converts back to exactly `value`, so 0.2f is sent as
// 0.2 rather than its widened double 0.20000000298023224.
double shortest_float(float value);

// Stable API names; nullopt for a move without an extrusion role.
const char *viewer_move_type_name(EMoveType type);
std::optional<const char *> viewer_role_name(GCodeExtrusionRole role);

// Metadata, layers and the processor-to-REST layer map of one result. Layers
// follow the GUI: only processor layers with an extrusion count, each at the
// height of its first extrusion; a move of a layer without extrusion belongs
// to the next counted layer (the last one after the end).
std::shared_ptr<ViewerHeader> make_viewer_header(const GCodeProcessorResult &result);

// The moves of one page, delta-encoded: the first move carries every field,
// later moves carry `points` and only the fields whose JSON value differs from
// the previous move of the page (a change to null is sent explicitly).
nlohmann::json viewer_moves_json(const ViewerPage &page, const ViewerHeader &header);

} // namespace Slic3r::Rest
