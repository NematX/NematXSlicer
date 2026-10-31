// Exercise public, value-only fan stages without Config, plan geometry or a
// firmware. Integration and replay use the ordered-plan fixtures next door.
//
//     TEST_CASE
//     |-- fan_policy_c_layout(): compare native C and SDK payload offsets
//     |-- make_base() / apply_layer_time() / close_first_layers(): check copied values
//     `-- transform_request(): retain protected modes and explicit zero requests
//
// Inputs are owned local records. Invalid phases, references and durations must
// fail visibly, and transformations must leave their input records unchanged.
#include <catch2/catch.hpp>
#include <cstddef>
#include <limits>

#include "libslic3r/Api/plugin/cpp/FanPolicy.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingLayerTimeProperty.hpp"

extern "C" size_t fan_policy_c_layout(unsigned int field);

using namespace slic3r_api;
namespace Cooling = slic3r_api::FanPolicy;

TEST_CASE("Public fan and timing records have matching C and C++ layouts",
          "[plugins][fan-policy][properties]")
{
    CHECK(fan_policy_c_layout(0) == sizeof(Cooling::Context));
    CHECK(fan_policy_c_layout(1) == sizeof(c_fan_role_reference));
    CHECK(fan_policy_c_layout(2) == offsetof(c_fan_policy_context, roles));
    CHECK(fan_policy_c_layout(3) == sizeof(PrintingLayerTimeProperty));
    CHECK(fan_policy_c_layout(4) == offsetof(c_printing_layer_time_property, origin));
    CHECK(fan_policy_c_layout(5) == RAW_FAN_ROLE_COUNT);
    CHECK(sizeof(Cooling::Context) == 136);
    CHECK(sizeof(c_fan_role_reference) == 8);
    CHECK(SLIC3R_PLUGIN_API_FAN_POLICY_MAJOR == 2);
    CHECK(RAW_PRINTING_LAYER_TIME_ORIGIN_RESOLVED == 1);
    CHECK(Cooling::normalize_fan_value(1) == 0);
    CHECK(Cooling::normalize_fan_value(0) == 0);
    CHECK(Cooling::normalize_fan_value(-1) == -1);
    CHECK(Cooling::normalize_fan_value(37) == 37);

    PrintingLayerTimeProperty duration = {};
    CHECK_NOTHROW(duration.validate());
    duration.origin = RAW_PRINTING_LAYER_TIME_ORIGIN_RESOLVED;
    duration.duration_seconds = 123;
    CHECK(duration.is_resolved());
    CHECK_NOTHROW(duration.validate());
    for (double invalid : {-1., std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        duration.duration_seconds = invalid;
        CHECK_THROWS_AS(duration.validate(), std::invalid_argument);
    }
    duration.duration_seconds = 0;
    duration.origin = raw_printing_layer_time_origin(42);
    CHECK_THROWS_AS(duration.validate(), std::invalid_argument);
}

TEST_CASE("Fan stages preserve configured references and enforce phase order",
          "[plugins][fan-policy][properties]")
{
    Cooling::Settings settings;
    settings.configured[RAW_FAN_ROLE_DEFAULT] = 20;
    settings.configured[RAW_FAN_ROLE_PERIMETER] = 40;
    settings.slowdown_seconds = 5;
    settings.fan_seconds = 10;
    settings.validate();
    const Cooling::Context base = Cooling::make_base(settings);
    CHECK_NOTHROW(base.validate(RAW_FAN_POLICY_PHASE_BASE));
    CHECK(base.roles[RAW_FAN_ROLE_PERIMETER].percent == 40);
    CHECK(base.roles[RAW_FAN_ROLE_TOP_SOLID_INFILL].configured_percent == -1);
    CHECK(base.roles[RAW_FAN_ROLE_TOP_SOLID_INFILL].percent == 20);
    const Cooling::Context timed = Cooling::apply_layer_time(base, settings, 7.5);
    CHECK_NOTHROW(timed.validate(RAW_FAN_POLICY_PHASE_TIME));
    CHECK(timed.phase == RAW_FAN_POLICY_PHASE_TIME);
    CHECK(timed.roles[RAW_FAN_ROLE_DEFAULT].percent == 60);
    CHECK(timed.roles[RAW_FAN_ROLE_PERIMETER].percent == 70);
    CHECK(timed.roles[RAW_FAN_ROLE_PERIMETER].configured_percent == 40);
    CHECK(timed.roles[RAW_FAN_ROLE_TOP_SOLID_INFILL].percent == 20);
    CHECK(timed.roles[RAW_FAN_ROLE_SUPPORT_MATERIAL].percent == 20);
    CHECK(timed.dynamic_min == 60);
    const Cooling::Context closed = Cooling::close_first_layers(timed, settings, 6);
    CHECK_NOTHROW(closed.validate(RAW_FAN_POLICY_PHASE_CLOSED));
    CHECK(closed.phase == RAW_FAN_POLICY_PHASE_CLOSED);
    CHECK(closed.roles[RAW_FAN_ROLE_DEFAULT].percent == 60);
    CHECK(base.roles[RAW_FAN_ROLE_DEFAULT].percent == 20); // inputs are immutable
    CHECK_THROWS_AS(Cooling::close_first_layers(base, settings, 6), std::invalid_argument);
    CHECK_THROWS_AS(Cooling::apply_layer_time(timed, settings, 7.5), std::invalid_argument);
    CHECK_THROWS_AS(Cooling::close_first_layers(closed, settings, 6), std::invalid_argument);

    Cooling::Context malformed = base;
    malformed.struct_size = 0;
    CHECK_THROWS_AS(Cooling::apply_layer_time(malformed, settings, 7.5), std::invalid_argument);
    malformed = base;
    malformed.roles[RAW_FAN_ROLE_DEFAULT].percent = 101;
    CHECK_THROWS_AS(malformed.validate(RAW_FAN_POLICY_PHASE_BASE), std::invalid_argument);
    malformed = base;
    malformed.roles[RAW_FAN_ROLE_DEFAULT].configured_percent = -2;
    CHECK_THROWS_AS(malformed.validate(RAW_FAN_POLICY_PHASE_BASE), std::invalid_argument);
    malformed = base;
    malformed.dynamic_min = malformed.dynamic_max + 1;
    CHECK_THROWS_AS(malformed.validate(RAW_FAN_POLICY_PHASE_BASE), std::invalid_argument);
    malformed = base;
    malformed.phase = 42;
    CHECK_THROWS_AS(malformed.validate(raw_fan_policy_phase(42)), std::invalid_argument);
    for (double invalid : {-1., std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()})
        CHECK_THROWS_AS(Cooling::apply_layer_time(base, settings, invalid), std::invalid_argument);
}

TEST_CASE("Fan time thresholds disabled references and D F closure retain legacy branches",
          "[plugins][fan-policy][layer-extrusion-edit]")
{
    Cooling::Settings settings;
    settings.configured[RAW_FAN_ROLE_DEFAULT] = 20;
    settings.slowdown_seconds = 5;
    settings.fan_seconds = 10;
    settings.validate();
    const Cooling::Context base = Cooling::make_base(settings);
    CHECK(Cooling::apply_layer_time(base, settings, 5).roles[RAW_FAN_ROLE_DEFAULT].percent == 100);
    CHECK(Cooling::apply_layer_time(base, settings, 10).roles[RAW_FAN_ROLE_DEFAULT].percent == 20);
    settings.fan_seconds = settings.slowdown_seconds;
    CHECK(Cooling::apply_layer_time(base, settings, 5).roles[RAW_FAN_ROLE_DEFAULT].percent == 20);
    CHECK(Cooling::apply_layer_time(base, settings, 4).roles[RAW_FAN_ROLE_DEFAULT].percent == 100);
    settings.fan_seconds = 0;
    CHECK(Cooling::apply_layer_time(base, settings, 0).roles[RAW_FAN_ROLE_DEFAULT].percent == 20);

    settings.configured.fill(-1);
    settings.configured[RAW_FAN_ROLE_PERIMETER] = 0;
    settings.fan_seconds = 10;
    const Cooling::Context absent = Cooling::make_base(settings);
    CHECK(absent.roles[RAW_FAN_ROLE_DEFAULT].percent == -1);
    CHECK(absent.roles[RAW_FAN_ROLE_PERIMETER].percent == 0);
    CHECK(Cooling::apply_layer_time(absent, settings, 7.5).roles[RAW_FAN_ROLE_DEFAULT].percent == -1);
    CHECK(Cooling::apply_layer_time(absent, settings, 7.5).roles[RAW_FAN_ROLE_PERIMETER].percent == 50);
    CHECK(Cooling::apply_layer_time(absent, settings, 0).roles[RAW_FAN_ROLE_DEFAULT].percent == 100);

    settings.configured.fill(80);
    settings.disabled_layers = 1;
    settings.full_speed_layer = 4;
    const Cooling::Context off = Cooling::close_first_layers(
        Cooling::apply_layer_time(Cooling::make_base(settings), settings, 0), settings, 0);
    for (const c_fan_role_reference &role : off.roles)
        CHECK(role.percent == 0);
    CHECK(off.dynamic_min == 0);
    CHECK(off.dynamic_max == 0);
    const Cooling::Context ramp = Cooling::close_first_layers(
        Cooling::apply_layer_time(Cooling::make_base(settings), settings, 20), settings, 1);
    CHECK(ramp.roles[RAW_FAN_ROLE_DEFAULT].percent == 26);
    CHECK(ramp.roles[RAW_FAN_ROLE_BRIDGE_INFILL].percent == 80);
    CHECK(ramp.roles[RAW_FAN_ROLE_SUPPORT_MATERIAL_INTERFACE].percent == 80);
    CHECK(ramp.dynamic_max == 26);
    CHECK(ramp.roles[RAW_FAN_ROLE_IRONING].percent == ramp.roles[RAW_FAN_ROLE_TOP_SOLID_INFILL].percent);
    CHECK(ramp.roles[RAW_FAN_ROLE_THIN_WALL].percent == ramp.roles[RAW_FAN_ROLE_EXTERNAL_PERIMETER].percent);

    // Reuse one numerical result with explicit closure inputs: neither the
    // layer ordinal nor D/F settings are hidden snapshots inside the context.
    const Cooling::Context timed = Cooling::apply_layer_time(Cooling::make_base(settings), settings, 20);
    CHECK(Cooling::close_first_layers(timed, settings, 0).roles[RAW_FAN_ROLE_DEFAULT].percent == 0);
    CHECK(Cooling::close_first_layers(timed, settings, 4).roles[RAW_FAN_ROLE_DEFAULT].percent == 80);
    settings.disabled_layers = 0;
    CHECK(Cooling::close_first_layers(timed, settings, 1).roles[RAW_FAN_ROLE_DEFAULT].percent == 40);
}

TEST_CASE("Common fan request transformations preserve protected mode pairs",
          "[plugins][fan-policy][fan-request]")
{
    EPropertyFan request = {};
    for (float percent : {0.f, 40.f}) {
        request.set(percent, FAN_REQUEST_MODE_OVERRIDE);
        const EPropertyFan untouched = Cooling::transform_request(request, 100.f, FAN_REQUEST_MODE_AUTOMATIC);
        CHECK(untouched.percent == percent);
        CHECK(untouched.mode == FAN_REQUEST_MODE_OVERRIDE);
    }
    const fan_request_mode mine = ORCHESTRATOR_RUNTIME_ID_DYNAMIC_BEGIN;
    const fan_request_mode other = mine + 1;
    request.set(40.f, other); // caller supplies its prevalidated runtime identity
    CHECK(Cooling::transform_request(request, 70.f, mine).percent == 40.f);
    request.set(40.f, mine);
    const EPropertyFan owned = Cooling::transform_request(request, 70.f, mine);
    CHECK(owned.mode == mine);
    CHECK(owned.percent == 70.f);
    request.clear();
    CHECK(Cooling::transform_request(request, 1.f, mine).percent == 1.f); // no filament normalization
    CHECK(Cooling::transform_request(request, 1.f, mine).mode == mine);
    request.set(0.f, FAN_REQUEST_MODE_AUTOMATIC);
    CHECK(Cooling::transform_request(request, 60.f, mine).mode == FAN_REQUEST_MODE_AUTOMATIC);
    CHECK(Cooling::transform_request(request, 60.f, mine).percent == 60.f);
}

TEST_CASE("Fan reference tables keep role exceptions and close family fallbacks in order",
          "[plugins][fan-policy][properties]")
{
    Cooling::Settings settings;
    settings.configured.fill(80);
    settings.slowdown_seconds = 5;
    settings.fan_seconds = 10;
    settings.full_speed_layer = 4;
    // At half-time the increasable roles become 90. On layer 1 (index0),
    // rampable roles then become one quarter, with truncation after +0.01.
    settings.validate();
    const Cooling::Context time = Cooling::apply_layer_time(Cooling::make_base(settings), settings, 7.5);
    const Cooling::Context closed = Cooling::close_first_layers(time, settings, 0);
    const int expected_time[RAW_FAN_ROLE_COUNT] = {90,90,90,90,90,90,90,80,80,90,90,90,80,80,80};
    const int expected_closed[RAW_FAN_ROLE_COUNT] = {22,22,22,90,22,90,22,20,20,90,22,22,80,20,80};
    for (size_t role = 0; role < RAW_FAN_ROLE_COUNT; ++role) {
        INFO(role);
        CHECK(time.roles[role].percent == expected_time[role]);
        CHECK(closed.roles[role].percent == expected_closed[role]);
        CHECK(closed.roles[role].configured_percent == 80);
    }
    CHECK(closed.dynamic_min == 22);
    CHECK(closed.dynamic_max == 22);

    settings.configured.fill(-1);
    settings.configured[RAW_FAN_ROLE_PERIMETER] = 40;
    settings.configured[RAW_FAN_ROLE_SOLID_INFILL] = 60;
    settings.configured[RAW_FAN_ROLE_BRIDGE_INFILL] = 80;
    settings.configured[RAW_FAN_ROLE_SUPPORT_MATERIAL] = 30;
    settings.full_speed_layer = 0;
    const Cooling::Context base = Cooling::make_base(settings);
    CHECK(base.roles[RAW_FAN_ROLE_TOP_SOLID_INFILL].percent == 60); // early fallback
    CHECK(base.roles[RAW_FAN_ROLE_INTERNAL_BRIDGE_INFILL].percent == 80);
    CHECK(base.roles[RAW_FAN_ROLE_SUPPORT_MATERIAL_INTERFACE].percent == 30);
    CHECK(base.roles[RAW_FAN_ROLE_EXTERNAL_PERIMETER].percent == -1); // late fallback
    const Cooling::Context resolved = Cooling::close_first_layers(Cooling::apply_layer_time(base, settings, 20), settings, 0);
    CHECK(resolved.roles[RAW_FAN_ROLE_EXTERNAL_PERIMETER].percent == 40);
    CHECK(resolved.roles[RAW_FAN_ROLE_THIN_WALL].percent == 40);
    CHECK(resolved.roles[RAW_FAN_ROLE_IRONING].percent == 60);
    settings.configured[RAW_FAN_ROLE_OVERHANG_PERIMETER] = 75;
    settings.dynamic_enabled = true;
    const Cooling::Context dynamic = Cooling::make_base(settings);
    CHECK(dynamic.roles[RAW_FAN_ROLE_OVERHANG_PERIMETER].configured_percent == 75);
    CHECK(dynamic.roles[RAW_FAN_ROLE_OVERHANG_PERIMETER].percent == -1);
    settings.full_speed_layer = 4;
    const Cooling::Context disabled_default = Cooling::close_first_layers(
        Cooling::apply_layer_time(dynamic, settings, 20), settings, 0);
    CHECK(disabled_default.roles[RAW_FAN_ROLE_DEFAULT].percent == -1);
    CHECK(disabled_default.dynamic_max == 25);
}

TEST_CASE("Fan settings are validated once at the configuration boundary",
          "[plugins][fan-policy][properties]")
{
    Cooling::Settings settings;
    CHECK_NOTHROW(settings.validate());
    for (int32_t invalid : {-2, 101}) {
        settings.configured[RAW_FAN_ROLE_DEFAULT] = invalid;
        CHECK_THROWS_AS(settings.validate(), std::invalid_argument);
    }
    settings.configured[RAW_FAN_ROLE_DEFAULT] = -1;
    for (int32_t invalid : {-1, 101}) {
        settings.maximum = invalid;
        CHECK_THROWS_AS(settings.validate(), std::invalid_argument);
    }
    settings.maximum = 100;
    for (double invalid : {-1., std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        settings.slowdown_seconds = invalid;
        CHECK_THROWS_AS(settings.validate(), std::invalid_argument);
        settings.slowdown_seconds = 0;
        settings.fan_seconds = invalid;
        CHECK_THROWS_AS(settings.validate(), std::invalid_argument);
        settings.fan_seconds = 0;
    }
    CHECK_NOTHROW(settings.validate());
}
