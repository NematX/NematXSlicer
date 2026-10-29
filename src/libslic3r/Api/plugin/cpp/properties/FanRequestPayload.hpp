///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_Api_plugin_cpp_properties_FanRequestPayload_hpp_
#define slic3r_Api_plugin_cpp_properties_FanRequestPayload_hpp_

/* Shared C-layout fan operations for the SDK facade and native host mirror.
   Policies arbitrate using exclusive modes. The host validates registration
   once before export; firmware only checks active percentages before encoding.
   Unspecified inherits the parent pair and ignores its own percentage.

       auto &request = entity.get_or_add(EPropertyFan::key);
       request.set(40.f, FAN_REQUEST_MODE_AUTOMATIC);
       request.validate(host);

   This mixin owns only the pair of C fields; it retains no host or tree view.
   Consumers copy resolved pairs rather than combining fields from two nodes. */
#include <cmath>
#include <stdexcept>
#include "libslic3r/Api/plugin/c/slic3r_extrusion_property.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

namespace slic3r_api {

template<class Derived>
struct FanRequestPayload : c_extrusion_property_fan
{
    // Distinguish an explicit stop from an absent request.
    bool is_set() const { return mode != FAN_REQUEST_MODE_UNSPECIFIED; }
    // Copy this complete pair when set, otherwise copy the complete parent pair.
    Derived resolved_with(const Derived &parent) const {
        return is_set() ? static_cast<const Derived &>(*this) : parent;
    }
    // Reset both fields to neutral inherited state.
    Derived &clear() { percent = 0.f; mode = FAN_REQUEST_MODE_UNSPECIFIED; return self(); }
    // Set a finite physical request; invalid values throw without changing either field.
    Derived &set(float value, fan_request_mode request_mode) {
        if (request_mode == ORCHESTRATOR_RUNTIME_ID_INVALID ||
            !std::isfinite(value) || value < 0.f || value > 100.f ||
            (request_mode == FAN_REQUEST_MODE_UNSPECIFIED && value != 0.f))
            throw std::invalid_argument("Invalid fan request pair.");
        percent = value;
        mode = request_mode;
        return self();
    }

    // Call after validating the effective pair and the policy's registered mode.
    // Override is never policy-owned, even when my_mode is Override. Explicit
    // producers can still replace a pair using set(), outside policy arbitration.
    bool can_modify(fan_request_mode my_mode) const {
        return my_mode != ORCHESTRATOR_RUNTIME_ID_INVALID && mode != FAN_REQUEST_MODE_OVERRIDE &&
            (mode == FAN_REQUEST_MODE_UNSPECIFIED || mode == FAN_REQUEST_MODE_AUTOMATIC ||
             mode == my_mode);
    }

    // Firmware consumes only the numeric value; Unspecified emits no request.
    bool has_valid_percentage() const {
        return !is_set() || (std::isfinite(percent) && percent >= 0.f && percent <= 100.f);
    }
    // Throw invalid_argument for an active invalid percentage; absent requests are ignored.
    void validate_percentage() const {
        if (!has_valid_percentage())
            throw std::invalid_argument("Invalid active fan percentage.");
    }
    // Full semantic validation belongs at the host's pre-export boundary.
    bool is_valid(const orchestrator_handle *orchestrator) const {
        return mode != ORCHESTRATOR_RUNTIME_ID_INVALID && has_valid_percentage() &&
            orchestrator_runtime_id_name(orchestrator, FAN_REQUEST_MODE_ID_TYPE, mode) != nullptr;
    }
    // Throw invalid_argument unless this host recognizes the mode and active percentage.
    void validate(const orchestrator_handle *orchestrator) const {
        if (!is_valid(orchestrator))
            throw std::invalid_argument("Invalid fan request: unregistered mode or invalid active percentage.");
    }

private:
    // Return the facade that owns these C fields for fluent setters.
    Derived &self() { return static_cast<Derived &>(*this); }
};

} // namespace slic3r_api
#endif
