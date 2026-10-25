#include <catch2/catch.hpp>

#include <cstdint>
#include <limits>
#include <stdexcept>

#include "libslic3r/Api/plugin/cpp/gcode/HeaterState.hpp"

// Heater requests remain uncorrected; only the effective target is bounded.
using slic3r_api::GCodeGeneration::HeaterState;

TEST_CASE("Heater offsets bound effective targets without reserving a raw zero",
          "[plugins][gcode][firmware][state][heater]")
{
    struct Example { int16_t request; int16_t offset; int16_t effective; };
    const Example examples[] = {
        {0, 5, 5}, {5, 5, 10}, {5, -5, 0}, {0, -5, 0}, {6, -5, 1},
        {1998, 5, 2000}, {2000, 5, 2000},
        {std::numeric_limits<int16_t>::max(), std::numeric_limits<int16_t>::max(), 2000},
        {std::numeric_limits<int16_t>::min(), std::numeric_limits<int16_t>::min(), 0}
    };
    for (const Example &example : examples) {
        INFO("request=" << example.request << " offset=" << example.offset);
        HeaterState state;
        state.setup(example.offset);
        CHECK_THROWS_AS(state.effective_temperature(), std::logic_error);
        state.request_temperature(example.request);
        CHECK(state.requested_temperature() == example.request);
        CHECK(state.effective_temperature() == example.effective);
        state.mark_encoded_with_wait();
        CHECK(state.encoded_temperature() == example.effective);
        CHECK(state.encoded_temperature_with_wait() == example.effective);
        CHECK_FALSE(state.needs_encoding());
        CHECK_FALSE(state.needs_wait_encoding());
    }
}

TEST_CASE("External heater targets retain their inverse offset and wait state",
          "[plugins][gcode][firmware][state][heater]")
{
    for (const int16_t offset : {int16_t(5), int16_t(-5)}) {
        for (const int16_t target : {int16_t(0), int16_t(5)}) {
            for (const bool waited : {false, true}) {
                INFO("offset=" << offset << " target=" << target << " waited=" << waited);
                HeaterState state;
                state.setup(offset);
                state.request_temperature(200);
                state.mark_encoded_with_wait();
                state.synchronize_after_external_gcode(target, waited);
                CHECK(state.requested_temperature() == int16_t(target - offset));
                CHECK(state.effective_temperature() == target);
                CHECK(state.encoded_temperature() == target);
                CHECK_FALSE(state.needs_encoding());
                CHECK(state.needs_wait_encoding() == !waited);
                if (waited)
                    CHECK(state.encoded_temperature_with_wait() == target);
                else
                    CHECK_FALSE(state.encoded_temperature_with_wait());
                state.clear_encoded();
                CHECK(state.needs_encoding());
                state.mark_encoded();
                CHECK(state.encoded_temperature() == target);
                CHECK_FALSE(state.needs_encoding());
            }
        }
    }
}

TEST_CASE("Heater runtime transfers apply the destination offset to raw requests",
          "[plugins][gcode][firmware][state][heater]")
{
    for (const int16_t source_offset : {int16_t(5), int16_t(-5)}) {
        HeaterState source;
        source.setup(source_offset);
        source.request_temperature(0);
        source.mark_encoded_with_wait();
        HeaterState destination;
        destination.setup(int16_t(-source_offset));
        destination.synchronize_runtime_from(source);
        CHECK(destination.requested_temperature() == 0);
        CHECK(destination.effective_temperature() == (source_offset > 0 ? 0 : 5));
        CHECK(destination.needs_encoding());
        destination.mark_encoded_with_wait();
        CHECK_FALSE(destination.needs_wait_encoding());
        destination.reset_runtime_state();
        CHECK_FALSE(destination.requested_temperature());
        CHECK_THROWS_AS(destination.effective_temperature(), std::logic_error);
    }
}
