#include <catch2/catch.hpp>

#include <cmath>
#include <string>
#include <utility>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Plugins/ElephantFootCompensation.hpp"
#include "clipper_offset_test_helpers.hpp"

/*
This regression checks that a minimum width of 0.01 mm keeps thin bands usable
when the elephant-foot compensation helper withdraws material from a shape.
It covers fourteen shapes, two band widths and three withdrawal distances,
each with and without rotation and translation. Three calls in separate storage
must return the same geometry. The component and hole counts must survive polygon
cleanup and simplification. On the finger, neck and round-neck shapes, the cleaned
output must keep the requested cross section and cover the paths along the bands.

    TEST_CASE: compare the result with the original shape and the requested minimum width
    |-- make_fixture: build one connected shape and its measurement paths
    |   `-- strip / disc: build straight and curved pieces of the shape
    |-- transform_fixture: rotate and translate the shape and its paths together
    |-- measure: run the compensation helper and compare the three results
    `-- inspect: check the raw, cleaned and simplified geometry
        |-- area_mm2: measure positive output area and material added outside the input
        `-- covered_length: clip measurement paths to the filled area
*/

namespace {

using namespace Slic3r;
using clipper_offset_test::rectangle_mm;
using clipper_offset_test::view;

struct Fixture {
    ExPolygon source;
    Polyline section;
    Polylines probes;
};

struct Measurement {
    ExPolygons output;
    bool deterministic = true;
};

struct Inspection {
    size_t simplified_components = 0;
    size_t simplified_holes = 0;
    double output_section = 0.;
    double probe_coverage = 0.;
    bool geometry_ok = false;
};

// Construct one connected source and paths through its narrow features.
Fixture make_fixture(const std::string &family, double width, double length);
// Join a constant-width strip to its neighbors through overlapping end caps.
ExPolygon strip(double ax, double ay, double bx, double by, double width);
// Construct a polygonal disc for curved-boundary fixtures.
ExPolygon disc(double cx, double cy, double radius);
// Rotate and translate every measured point together with its source geometry.
void transform_fixture(Fixture &fixture);
// Compare the exact geometry from three calls in independent storage and retain a copy of the first result.
Measurement measure(const Fixture &fixture, double minimum_width, double withdrawal);
// Check raw and simplified topology, and measure the section and path coverage in cleaned material.
Inspection inspect(const Fixture &fixture, const ExPolygons &raw);
// Sum filled area in square millimetres without changing ring orientations.
double area_mm2(const ExPolygons &areas);
// Sum the portions of diagnostic paths covered by the supplied material.
double covered_length(const Polylines &paths, const ExPolygons &areas);

} // namespace

TEST_CASE("Elephant foot thin bands remain usable above the cleanup tolerance",
          "[plugins][xy-size-compensation][thin-withdrawal]")
{
    const std::string family = GENERATE("bar", "neck", "bent_neck", "finger", "tapered_finger", "fork", "comb",
                                        "u_band", "serpentine", "ring", "hole_web", "hole_finger", "hourglass", "round_neck");
    const double width = GENERATE(0.05, 0.4);
    const double withdrawal = GENERATE(0.3, 0.6, 1.2);
    const bool transformed = GENERATE(false, true);
    Fixture fixture = make_fixture(family, width, 8.);
    if (transformed)
        transform_fixture(fixture);
    INFO(family << " width=" << width << " withdrawal=" << withdrawal << " transformed=" << transformed);
    const double minimum_width = 0.01;
    const Measurement measurement = measure(fixture, minimum_width, withdrawal);
    const Inspection inspection = inspect(fixture, measurement.output);
    CHECK(measurement.deterministic);
    CHECK(inspection.geometry_ok);
    CHECK(inspection.simplified_components == 1);
    CHECK(inspection.simplified_holes == fixture.source.holes.size());
    if (family == "finger" || family == "neck" || family == "round_neck") {
        CHECK(inspection.output_section == Approx(minimum_width).margin(2. * unscaled(double(SCALED_EPSILON))));
        CHECK(inspection.probe_coverage == Approx(1.));
    }
}

namespace {

Fixture make_fixture(const std::string &family, double width, double length)
{
    Fixture fixture;
    const double w = width, l = length;
    ExPolygons pieces;
    fixture.section = Polyline{Point::new_scale(l * 0.5, -w), Point::new_scale(l * 0.5, w)};
    fixture.probes = {Polyline{Point::new_scale(l * 0.2, 0.), Point::new_scale(l * 0.8, 0.)}};
    if (family == "bar") {
        pieces.emplace_back(rectangle_mm(0., -w * 0.5, l, w * 0.5));
    } else if (family == "neck" || family == "round_neck") {
        if (family == "neck") {
            pieces.emplace_back(rectangle_mm(-8., -4., 0., 4.));
            pieces.emplace_back(rectangle_mm(l, -4., l + 8., 4.));
        } else {
            pieces.push_back(disc(-4., 0., 4.));
            pieces.push_back(disc(l + 4., 0., 4.));
        }
        pieces.emplace_back(rectangle_mm(-0.5, -w * 0.5, l + 0.5, w * 0.5));
    } else if (family == "bent_neck") {
        pieces.emplace_back(rectangle_mm(-8., -4., 0., 4.));
        pieces.push_back(strip(0., 0., l, 0., w));
        pieces.push_back(strip(l, 0., l, l, w));
        pieces.push_back(strip(l, l, 2. * l, l, w));
        pieces.emplace_back(rectangle_mm(2. * l, l - 4., 2. * l + 8., l + 4.));
        fixture.probes.push_back(Polyline{Point::new_scale(l, l * 0.2), Point::new_scale(l, l * 0.8)});
    } else if (family == "finger" || family == "tapered_finger") {
        pieces.emplace_back(rectangle_mm(-8., -4., 0., 4.));
        if (family == "finger")
            pieces.emplace_back(rectangle_mm(-0.5, -w * 0.5, l, w * 0.5));
        else
            pieces.emplace_back(Points{Point::new_scale(-0.5, -w), Point::new_scale(l, 0.),
                                       Point::new_scale(-0.5, w)});
    } else if (family == "fork") {
        pieces.emplace_back(rectangle_mm(-8., -4., 0., 4.));
        pieces.push_back(strip(0., 0., l, 0., w));
        pieces.push_back(strip(l, 0., 2. * l, l, w));
        pieces.push_back(strip(l, 0., 2. * l, -l, w));
        fixture.probes.push_back(Polyline{Point::new_scale(1.2 * l, 0.2 * l), Point::new_scale(1.8 * l, 0.8 * l)});
        fixture.probes.push_back(Polyline{Point::new_scale(1.2 * l, -0.2 * l), Point::new_scale(1.8 * l, -0.8 * l)});
    } else if (family == "comb") {
        const double pitch = w + 0.75;
        pieces.emplace_back(rectangle_mm(-8., -3. * pitch - 2., 0., 3. * pitch + 2.));
        fixture.probes.clear();
        for (int tooth = -3; tooth <= 3; ++tooth) {
            const double y = double(tooth) * pitch;
            pieces.emplace_back(rectangle_mm(-0.5, y - w * 0.5, l, y + w * 0.5));
            fixture.probes.push_back(Polyline{Point::new_scale(l * 0.2, y), Point::new_scale(l * 0.8, y)});
        }
    } else if (family == "u_band") {
        pieces.push_back(strip(0., l, 0., 0., w));
        pieces.push_back(strip(0., 0., l, 0., w));
        pieces.push_back(strip(l, 0., l, l, w));
        fixture.probes.push_back(Polyline{Point::new_scale(0., l * 0.2), Point::new_scale(0., l * 0.8)});
        fixture.probes.push_back(Polyline{Point::new_scale(l, l * 0.2), Point::new_scale(l, l * 0.8)});
    } else if (family == "serpentine") {
        const double pitch = 3. * w + 1.;
        pieces.push_back(strip(0., 0., l, 0., w));
        pieces.push_back(strip(l, 0., l, pitch, w));
        pieces.push_back(strip(l, pitch, 0., pitch, w));
        pieces.push_back(strip(0., pitch, 0., 2. * pitch, w));
        pieces.push_back(strip(0., 2. * pitch, l, 2. * pitch, w));
        for (const double y : {pitch, 2. * pitch})
            fixture.probes.push_back(Polyline{Point::new_scale(l * 0.2, y), Point::new_scale(l * 0.8, y)});
    } else if (family == "ring") {
        ExPolygon ring(rectangle_mm(-w, -w, l + w, l + w));
        Polygon hole = rectangle_mm(0., 0., l, l);
        hole.make_clockwise();
        ring.holes.push_back(std::move(hole));
        pieces.push_back(std::move(ring));
        fixture.section = Polyline{Point::new_scale(l * 0.5, -1.5 * w), Point::new_scale(l * 0.5, 0.5 * w)};
        fixture.probes = {Polyline{Point::new_scale(l * 0.2, -0.5 * w), Point::new_scale(l * 0.8, -0.5 * w)}};
    } else if (family == "hole_web") {
        ExPolygon holes(rectangle_mm(-8., -4., 8., l + 4.));
        for (const std::pair<double, double> &bounds : {std::make_pair(-4., -0.5 * w), std::make_pair(0.5 * w, 4.)}) {
            Polygon hole = rectangle_mm(bounds.first, 0., bounds.second, l);
            hole.make_clockwise();
            holes.holes.push_back(std::move(hole));
        }
        pieces.push_back(std::move(holes));
        fixture.section = Polyline{Point::new_scale(-w, l * 0.5), Point::new_scale(w, l * 0.5)};
        fixture.probes = {Polyline{Point::new_scale(0., l * 0.2), Point::new_scale(0., l * 0.8)}};
    } else if (family == "hole_finger") {
        ExPolygon frame(rectangle_mm(-8., -l - 4., l + 8., l + 4.));
        Polygon hole = rectangle_mm(0., -l, l + 4., l);
        hole.make_clockwise();
        frame.holes.push_back(std::move(hole));
        pieces.push_back(std::move(frame));
        pieces.emplace_back(rectangle_mm(-0.5, -w * 0.5, l, w * 0.5));
    } else if (family == "hourglass") {
        pieces.emplace_back(Points{Point::new_scale(-8., -4.), Point::new_scale(0., -4.),
            Point::new_scale(l * 0.5, -w * 0.5), Point::new_scale(l, -4.), Point::new_scale(l + 8., -4.),
            Point::new_scale(l + 8., 4.), Point::new_scale(l, 4.), Point::new_scale(l * 0.5, w * 0.5),
            Point::new_scale(0., 4.), Point::new_scale(-8., 4.)});
    }
    const ExPolygons joined = union_ex(pieces);
    REQUIRE(joined.size() == 1);
    fixture.source = joined.front();
    return fixture;
}

void transform_fixture(Fixture &fixture)
{
    const double angle = 0.37;
    const Point shift = Point::new_scale(120., 90.);
    fixture.source.rotate(angle);
    fixture.source.translate(shift);
    fixture.section.rotate(angle);
    fixture.section.translate(shift);
    for (Polyline &probe : fixture.probes) {
        probe.rotate(angle);
        probe.translate(shift);
    }
}

Measurement measure(const Fixture &fixture, double minimum_width, double withdrawal)
{
    Measurement result;
    for (size_t repeat = 0; repeat < 3; ++repeat) {
        PluginStorage storage;
        const slic3r_api::StoredExPolygonCollection output = slic3r_api::XYSizeCompensationPlugin::detail::elephant_foot_compensation(
            reinterpret_cast<storage_handle *>(&storage), view(fixture.source), minimum_width, scale_i(withdrawal));
        const ExPolygons &native = *reinterpret_cast<const ExPolygons *>(output.handle());
        if (repeat == 0)
            result.output = native;
        else
            result.deterministic = result.deterministic && native == result.output;
    }
    return result;
}

Inspection inspect(const Fixture &fixture, const ExPolygons &raw)
{
    Inspection result;
    const ExPolygons clean = union_ex(raw);
    ExPolygons simplified;
    for (const ExPolygon &area : clean)
        append(simplified, area.simplify(SCALED_EPSILON));
    result.simplified_components = simplified.size();
    double perimeter = fixture.source.contour.length();
    for (const Polygon &hole : fixture.source.holes)
        perimeter += hole.length();
    for (const ExPolygon &area : simplified)
        result.simplified_holes += area.holes.size();
    result.output_section = unscaled(covered_length({fixture.section}, clean));
    const double input_probe_length = covered_length(fixture.probes, {fixture.source});
    result.probe_coverage = covered_length(fixture.probes, clean) / input_probe_length;
    // Tolerate a strip one epsilon wide of material added outside the source by rounding.
    // A lost branch is caught by the section and path checks, not by this area check.
    const double outside_tolerance = unscaled(perimeter) * unscaled(double(SCALED_EPSILON));
    result.geometry_ok = raw.size() == 1 && clean.size() == 1 &&
        raw.front().holes.size() == fixture.source.holes.size() && clean.front().holes.size() == fixture.source.holes.size() &&
        raw.front().contour.size() >= 3 && raw.front().contour.is_counter_clockwise() &&
        area_mm2(raw) > 0. && area_mm2(diff_ex(clean, {fixture.source})) <= outside_tolerance;
    if (result.geometry_ok)
        for (const Polygon &hole : raw.front().holes)
            result.geometry_ok = result.geometry_ok && hole.size() >= 3 && hole.is_clockwise();
    return result;
}

double area_mm2(const ExPolygons &areas)
{
    double sum = 0.;
    for (const ExPolygon &area : areas)
        sum += area.area();
    return unscaled(unscaled(sum));
}

double covered_length(const Polylines &paths, const ExPolygons &areas)
{
    double sum = 0.;
    for (const Polyline &part : intersection_pl(paths, areas))
        sum += part.length();
    return sum;
}

ExPolygon strip(double ax, double ay, double bx, double by, double width)
{
    const double length = std::hypot(bx - ax, by - ay);
    const double dx = (bx - ax) / length * width * 0.5;
    const double dy = (by - ay) / length * width * 0.5;
    return ExPolygon(Points{Point::new_scale(ax - dx + dy, ay - dy - dx),
        Point::new_scale(bx + dx + dy, by + dy - dx), Point::new_scale(bx + dx - dy, by + dy + dx),
        Point::new_scale(ax - dx - dy, ay - dy + dx)});
}

ExPolygon disc(double cx, double cy, double radius)
{
    Points points;
    for (size_t index = 0; index < 96; ++index) {
        const double angle = 2. * PI * double(index) / 96.;
        points.push_back(Point::new_scale(cx + radius * std::cos(angle), cy + radius * std::sin(angle)));
    }
    return ExPolygon(std::move(points));
}

} // namespace
