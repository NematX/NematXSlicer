/*
 * Headless custom G-code per height (color changes, pauses, templates, custom G-code).
 *
 * The scene stores the list in Model::custom_gcode_per_print_z.  This module owns no
 * state: it computes the mode the slicing engine will use for the session, validates a
 * complete request list into the sorted engine list, and serializes a stored list with
 * its warnings.  Errors are SceneEditError (422, invalid_custom_gcode); element failures
 * carry the zero-based element index and a stable reason.
 */
#pragma once

#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "scene_edit.hpp"

#include <nlohmann/json.hpp>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r::Rest {

// One request element as parsed from JSON; z is in millimeters and extruders are 1-based.
struct CustomGCodeEntry {
    double z = 0.;
    CustomGCode::Type type = CustomGCode::ColorChange;
    std::optional<int> extruder;
    std::optional<std::string> color;
    std::string text; // pause message or custom G-code
};

// Report an invalid request element with its index and a stable reason.
class CustomGCodeItemError final : public SceneEditError {
public:
    // Construct a 422 invalid_custom_gcode error for one element.
    CustomGCodeItemError(std::size_t item, std::string reason);
    // Return the zero-based element index carried by the error.
    std::size_t item() const noexcept;
    // Return the stable reason such as above_objects or duplicate_z.
    const std::string &reason() const noexcept;

private:
    std::size_t item_;
    std::string reason_;
};

// Mode of a list for this scene and session, and the 1-based extruder that prints the whole model in
// MultiAsSingle mode (1 otherwise).
struct CustomGCodeMode {
    CustomGCode::Mode mode = CustomGCode::Undef;
    int extruder = 1;
};

// Compute the mode the slicing engine will use for this scene and session.
CustomGCodeMode current_custom_gcode_mode(const Model &scene, const DynamicPrintConfig &session);

// Validate the entries and build the sorted engine list for the scene and session configuration.
CustomGCode::Info make_custom_gcodes(const std::vector<CustomGCodeEntry> &entries,
    const Model &scene, const DynamicPrintConfig &session);

// Serialize a stored list with its mode, the current mode and the warnings.
nlohmann::json custom_gcodes_json(const CustomGCode::Info &info, const Model &scene,
    const DynamicPrintConfig &session);

// Same, for a caller that already computed the current mode (avoids a second Print apply).
nlohmann::json custom_gcodes_json(const CustomGCode::Info &info, const Model &scene,
    const DynamicPrintConfig &session, CustomGCode::Mode current);

} // namespace Slic3r::Rest
