///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#pragma once

// Shared fan-reference pipeline. Config is read once into value settings;
// numerical kernels consume copies and never mutate a plan or a fan device.
// Register context_property_key(host) before processing, then publish a value
// on each tool group after the three stages have run in order:
//
//     const auto settings = FanPolicy::read_settings(config, tool);
//     const auto base = FanPolicy::make_base(settings);
//     const auto timed = FanPolicy::apply_layer_time(base, settings, seconds);
//     const auto program = FanPolicy::first_layer_program(settings, layer_index);
//     const auto closed = FanPolicy::close_first_layers(timed, program);
//
// Payloads own values only. Keys belong to the registering host. Validate settings
// assembled manually and contexts read from properties before passing them to
// kernels. Input validators throw invalid_argument; kernels check size and phase
// without rescanning validated references.
#include <array>
#include <cstdint>

#include "../c/slic3r_fan_policy.h"
#include "../c/slic3r_extrusions.h"
#include "PluginPropertyKey.hpp"
#include "properties/ExtrusionProperties.hpp"

namespace slic3r_api {
class Config;

namespace FanPolicy {

// Aliases keep host and external C++ code on the public C role numbering.
using Role = raw_fan_role;

struct Settings {
    std::array<int32_t, RAW_FAN_ROLE_COUNT> configured;
    int32_t maximum = 100;
    double slowdown_seconds = 0.0;
    double fan_seconds = 0.0;
    int32_t disabled_layers = 0;
    int32_t full_speed_layer = 0;
    bool dynamic_enabled = false;
    // Start with absent references and disabled time/first-layer transforms.
    Settings() { configured.fill(-1); }
    // Validate once after reading configuration or assembling settings manually.
    void validate() const;
};

struct Context : c_fan_policy_context {
    // Validate a published property before passing it to a numerical kernel.
    // Kernels only recheck size/phase; they trust the validated references.
    void validate(raw_fan_policy_phase expected) const;
};

// First-layer program of one layer and tool. An optional provider publishes it
// before the host first-layer stage; otherwise the host derives it from the
// disable_fan_first_layers/full_fan_speed_layer settings.
struct FirstLayerProgram {
    enum State : uint32_t {
        Normal = 1, // references and requests unchanged
        Off = 2,    // every automatic reference and request stops
        Ramp = 3    // positive rampable references and requests are scaled by factor
    };
    uint32_t version = 1;
    uint32_t state = Normal;
    float factor = 1.f; // in [0, 1], used by Ramp only
    // Validate a published program before applying it.
    void validate() const;
};

// Resolve compound extrusion roles in specificity order; COUNT means ineligible.
Role fan_role(raw_extrusion_role role);
// Role exceptions for reference-table and leaf transformations.
bool can_increase(Role role);
bool can_ramp(Role role);
// Normalize historical filament spelling 1 -> stop; regional requests bypass it.
int normalize_fan_value(int value);
// Read and validate existing filament options for one concrete extruder.
Settings read_settings(const Config &config, extruder_index_t extruder_id);
// Stage-specific readers avoid consuming another policy's configuration.
Settings read_base_settings(const Config &config, extruder_index_t extruder_id);
// Read and validate only timing thresholds and the maximum fan reference.
Settings read_layer_time_settings(const Config &config, extruder_index_t extruder_id);
// Read and validate only the first-layer program's configuration.
Settings read_first_layer_settings(const Config &config, extruder_index_t extruder_id);
// Shared scalar rules for references and modifiable leaf requests. Settings and
// duration are prevalidated by the owning policy before its leaf traversal.
float layer_time_percent(float percent, Role role, const Settings &settings, double duration_seconds);
// Apply the disable/ramp program once to a validated modifiable request.
float first_layer_percent(float percent, Role role, const FirstLayerProgram &program);
// D/F program for layer_index, the ordinal within the current PrintingGroup.
FirstLayerProgram first_layer_program(const Settings &settings, uint32_t layer_index);
// All stages require validated Settings. Apply initial family/default fallbacks
// while retaining configured references; no plan/configuration metadata is kept.
Context make_base(const Settings &settings);
// Apply legacy time thresholds/rounding to a BASE table, producing TIME.
Context apply_layer_time(const Context &base, const Settings &settings, double duration_seconds);
// Apply a validated first-layer program and late fallbacks once, producing CLOSED.
Context close_first_layers(const Context &time, const FirstLayerProgram &program);

// Transform a logical request only when the policy owns it; never rewrite a
// protected pair or normalize its percentage using filament-option rules.
EPropertyFan transform_request(const EPropertyFan &request, float percent, fan_request_mode policy_mode);

// Register during initialization, then retain the key per owning orchestrator.
inline PluginPropertyKey<Context> context_property_key(orchestrator_handle *orchestrator)
{
    return PluginPropertyKey<Context>::register_dynamic(orchestrator, FAN_POLICY_CONTEXT_PROPERTY_NAME);
}

// Published on PrintingToolGroup::properties(), at most once per run.
inline PluginPropertyKey<FirstLayerProgram> first_layer_program_key(orchestrator_handle *orchestrator)
{
    return PluginPropertyKey<FirstLayerProgram>::register_dynamic(orchestrator, "slic3r.printing_tool.fan_first_layer_program.v1");
}

static_assert(sizeof(Context) == sizeof(c_fan_policy_context), "Fan context ABI layout");
static_assert(alignof(Context) == alignof(c_fan_policy_context), "Fan context ABI alignment");
static_assert(std::extent<decltype(c_fan_policy_context::roles)>::value == RAW_FAN_ROLE_COUNT, "Fan role extent");

} // namespace FanPolicy
} // namespace slic3r_api
