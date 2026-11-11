#include <catch2/catch.hpp>

#include <string>
#include <tuple>

#include "libslic3r/GCode/TemperatureMover.hpp"

/*
Tests cover how the temperature mover recognizes nozzle temperature commands
before moving them earlier in the G-code. Lines are parsed directly, without a
print: standard M104/M109 targets, the M109 R form, comments and malformed or
unrelated lines.

    TEST_CASE
        `-- TemperatureMover_func::parse_nozzle_temperature()
*/

using namespace Slic3r;

namespace {
// Return the parsed temperature of one line for tool 2.
int16_t temperature_of(const std::string &line);

int16_t temperature_of(const std::string &line)
{
    const auto [temperature, tool] = TemperatureMover_func::parse_nozzle_temperature(line, 2);
    CHECK(tool == 2);
    return temperature;
}
} // namespace

TEST_CASE("Temperature mover reads standard nozzle temperature commands", "[plugins][temperature-mover]")
{
    CHECK(temperature_of("M104 S200") == 200);
    CHECK(temperature_of("M109 S215 ; wait for the nozzle") == 215);
    CHECK(temperature_of("M104 T1 S205.4") == 205);
    CHECK(temperature_of("M109 R190") == 190);
    CHECK(temperature_of("M104 S0") == 0);
}

TEST_CASE("Temperature mover ignores lines without a nozzle temperature target", "[plugins][temperature-mover]")
{
    CHECK(temperature_of("M104") == -1);
    CHECK(temperature_of("M104 R190") == -1);
    CHECK(temperature_of("M104 ; S200") == -1);
    CHECK(temperature_of("M104 S") == -1);
    CHECK(temperature_of("M104 Sabc") == -1);
    CHECK(temperature_of("M104 S-5") == -1);
    CHECK(temperature_of("M1040 S200") == -1);
    CHECK(temperature_of("M140 S60") == -1);
    CHECK(temperature_of("G1 X10 S200") == -1);
    CHECK(temperature_of("M104 =200") == -1);
}
