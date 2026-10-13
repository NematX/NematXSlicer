#include <catch2/catch.hpp>

#include <cstdint>
#include <vector>

#include "libslic3r/GCode/GCodeProcessor.hpp"

/* Tests map displayed G-code Z entries to result layer heights. A viewer layer is one displayed Z and the ID of its
   opening move; missing heights use the layer slider's current Z-gap rule. */

namespace Slic3r {

// Checks result-height lookup and the exact Z-gap fallback for viewer layer indices.
TEST_CASE("G-code layer-height helper maps viewer layers to result heights", "[layer_height]")
{
    SECTION("uses each opening move ID to select a result height") {
        const std::vector<double> layer_zs{ 0.2, 0.5, 0.8 };
        const std::vector<uint16_t> opening_layer_ids{ 0, 1, 2 };
        const std::vector<float> layers_max_height{ 0.3f, 0.25f, 0.4f };

        const std::vector<double> heights = calculate_gcode_layer_heights(
            layer_zs, opening_layer_ids, layers_max_height);

        REQUIRE(heights.size() == 3);
        CHECK(heights[0] == Approx(0.3));
        CHECK(heights[1] == Approx(0.25));
        CHECK(heights[2] == Approx(0.4));
    }

    SECTION("uses the current Z rule for zero and missing result heights") {
        const std::vector<double> layer_zs{ 0.2, 0.5, 0.8 };
        const std::vector<uint16_t> opening_layer_ids{ 0, 1, 2 };
        const std::vector<float> layers_max_height{ 0.0f, 0.0f };

        const std::vector<double> heights = calculate_gcode_layer_heights(
            layer_zs, opening_layer_ids, layers_max_height);

        REQUIRE(heights.size() == 3);
        CHECK(heights[0] == Approx(0.2));
        CHECK(heights[1] == Approx(0.5));
        CHECK(heights[2] == Approx(0.3));
    }

    SECTION("maps opening IDs that skip a result layer") {
        const std::vector<double> layer_zs{ 0.2, 0.5 };
        const std::vector<uint16_t> opening_layer_ids{ 0, 2 };
        const std::vector<float> layers_max_height{ 0.3f, 0.0f, 0.4f };

        const std::vector<double> heights = calculate_gcode_layer_heights(
            layer_zs, opening_layer_ids, layers_max_height);

        REQUIRE(heights.size() == 2);
        CHECK(heights[0] == Approx(0.3));
        CHECK(heights[1] == Approx(0.4));
    }

    SECTION("uses the same result height for repeated opening IDs") {
        const std::vector<double> layer_zs{ 0.2, 0.5, 0.5 };
        const std::vector<uint16_t> opening_layer_ids{ 0, 2, 2 };
        const std::vector<float> layers_max_height{ 0.3f, 0.0f, 0.4f };

        const std::vector<double> heights = calculate_gcode_layer_heights(
            layer_zs, opening_layer_ids, layers_max_height);

        REQUIRE(heights.size() == 3);
        CHECK(heights[0] == Approx(0.3));
        CHECK(heights[1] == Approx(0.4));
        CHECK(heights[2] == Approx(0.4));
    }

    SECTION("uses the Z fallback for vase layers without opening IDs") {
        const std::vector<double> layer_zs{ 0.2, 0.5, 0.8 };
        const std::vector<uint16_t> no_opening_layer_ids;
        const std::vector<float> layers_max_height{ 0.4f, 0.6f, 0.9f };

        const std::vector<double> heights = calculate_gcode_layer_heights(
            layer_zs, no_opening_layer_ids, layers_max_height);

        REQUIRE(heights.size() == 3);
        CHECK(heights[0] == Approx(0.2));
        CHECK(heights[1] == Approx(0.5));
        CHECK(heights[2] == Approx(0.3));
    }

    SECTION("falls back when the height vector is empty and returns no entries for no layers") {
        const std::vector<double> layer_zs{ 0.2, 0.5, 0.8 };
        const std::vector<uint16_t> opening_layer_ids{ 0, 1, 2 };
        const std::vector<float> no_layer_heights;

        const std::vector<double> heights = calculate_gcode_layer_heights(
            layer_zs, opening_layer_ids, no_layer_heights);
        const std::vector<double> no_heights = calculate_gcode_layer_heights(
            {}, {}, no_layer_heights);

        REQUIRE(heights.size() == 3);
        CHECK(heights[0] == Approx(0.2));
        CHECK(heights[1] == Approx(0.5));
        CHECK(heights[2] == Approx(0.3));
        CHECK(no_heights.empty());
    }
}

} // namespace Slic3r
