#include <catch2/catch.hpp>

#include <limits>
#include <locale>
#include <string>

#include "libslic3r/Api/plugin/cpp/StringHelpers.hpp"
#include "libslic3r/LocalesUtils.hpp"

namespace {

// A locale whose decimal separator is a comma, to check that the helper does
// not depend on the global C++ locale.
struct CommaDecimal : std::numpunct<char>
{
    char do_decimal_point() const override { return ','; }
};

// Install a global locale for the lifetime of the object, then restore the previous one.
class ScopedGlobalLocale
{
public:
    explicit ScopedGlobalLocale(const std::locale &locale) : m_previous(std::locale::global(locale)) {}
    ~ScopedGlobalLocale() { std::locale::global(m_previous); }

private:
    std::locale m_previous;
};

} // namespace

TEST_CASE("SDK to_string_nozero writes plain decimals without trailing zeros",
          "[plugins][api][string-helpers]")
{
    using slic3r_api::to_string_nozero;
    CHECK(to_string_nozero(2., 6) == "2");
    CHECK(to_string_nozero(-3., 6) == "-3");
    CHECK(to_string_nozero(0., 6) == "0");
    CHECK(to_string_nozero(1.25, 6) == "1.25");
    CHECK(to_string_nozero(0.1, 15) == "0.1");
    CHECK(to_string_nozero(-0.5, 6) == "-0.5");
    CHECK(to_string_nozero(123456.789, 15) == "123456.789");
    CHECK(to_string_nozero(1.23456, 2) == "1.23");
    CHECK(to_string_nozero(2.5e20, 6) == "250000000000000000000");

    // Values below the last kept decimal become zero, never "-0".
    CHECK(to_string_nozero(1e-7, 6) == "0");
    CHECK(to_string_nozero(-1e-7, 6) == "0");
    CHECK(to_string_nozero(1e-7, 15) == "0.0000001");

    CHECK(to_string_nozero(std::numeric_limits<double>::quiet_NaN(), 6) == "nan");
    CHECK(to_string_nozero(std::numeric_limits<double>::infinity(), 6) == "inf");
    CHECK(to_string_nozero(-std::numeric_limits<double>::infinity(), 6) == "-inf");
}

TEST_CASE("SDK to_string_nozero ignores the global locale and matches the host helper",
          "[plugins][api][string-helpers]")
{
    {
        ScopedGlobalLocale comma(std::locale(std::locale::classic(), new CommaDecimal));
        CHECK(slic3r_api::to_string_nozero(1.5, 6) == "1.5");
    }

    for (const double value : { 0., 7., -12., 0.2, 1.25, -0.75, 3.14159265358979, 80.000000000000014,
                                123456.789, 1e-4, 98765432.123 })
        for (const int32_t precision : { 2, 6, 15 }) {
            INFO("value " << value << ", precision " << precision);
            CHECK(slic3r_api::to_string_nozero(value, precision) == Slic3r::to_string_nozero(value, precision));
        }
}
