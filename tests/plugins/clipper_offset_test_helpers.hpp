/*
 * Shared helpers for Clipper offset parity tests.
 *
 * The helpers keep canonical geometry comparison, diagnostic formatting,
 * native/API conversions, and deterministic test-shape construction identical
 * across the one-stage, two-stage, and contact-focused tests.
 */

#ifndef CLIPPER_OFFSET_TEST_HELPERS_HPP
#define CLIPPER_OFFSET_TEST_HELPERS_HPP

#include <initializer_list>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"

namespace clipper_offset_test {

using CanonicalRing = std::vector<std::pair<Slic3r::coord_t, Slic3r::coord_t>>;

struct CanonicalExPolygon {
    CanonicalRing contour;
    std::vector<CanonicalRing> holes;
};

using CanonicalShape = std::vector<CanonicalExPolygon>;

struct IntegerRect {
    int min_x;
    int min_y;
    int max_x;
    int max_y;
};

struct Contacts {
    Slic3r::Points points;
    size_t self_touching_rings = 0;
};

CanonicalShape canonical(const Slic3r::ExPolygons &expolygons);
CanonicalRing canonical_ring(const Slic3r::Polygon &polygon);
bool operator==(const CanonicalExPolygon &first, const CanonicalExPolygon &second);
bool operator<(const CanonicalExPolygon &first, const CanonicalExPolygon &second);

// Check nonempty contour and hole orientation, attaching the supplied context to failures.
void check_orientation(const std::string &info, const Slic3r::ExPolygons &expolygons);
void compare(const std::string &info,
             const Slic3r::ExPolygons &api,
             const Slic3r::ExPolygons &legacy,
             bool documentary);
std::string describe(const Slic3r::ExPolygons &expolygons);
double symmetric_area_mm2(const Slic3r::ExPolygons &first,
                          const Slic3r::ExPolygons &second);

Slic3r::ExPolygons boolean_ordered(const Slic3r::ExPolygons &expolygons);
bool needs_manual_polygon_conversion(const std::vector<Slic3r::ExPolygon> &expolygons);
Slic3r::Polygons flatten_polygons(const std::vector<Slic3r::ExPolygon> &expolygons);

Slic3r::ExPolygons to_native(const slic3r_api::ExPolygonCollection &collection);
slic3r_api::ExPolygon view(const Slic3r::ExPolygon &expolygon);
slic3r_api::Polygon view(const Slic3r::Polygon &polygon);

Slic3r::Polygon rectangle_mm(double min_x, double min_y, double max_x, double max_y);
Slic3r::ExPolygon square_with_holes(const Slic3r::Polygon &outer,
                                    std::initializer_list<Slic3r::Polygon> holes);
Slic3r::ExPolygon rotated(Slic3r::ExPolygon expolygon, double angle_rad);
Slic3r::ExPolygon start_shifted(Slic3r::ExPolygon expolygon, size_t shift);
Slic3r::Polygon star_mm(double x,
                        double y,
                        double outer_radius,
                        double inner_radius,
                        size_t tips);
Slic3r::Polygon comb_mm(double tooth_width, size_t teeth, double tooth_height);
Slic3r::ExPolygon with_holes(const Slic3r::Polygon &contour,
                             const Slic3r::Polygons &holes);
std::vector<Slic3r::ExPolygon> random_disjoint_holed_rectangles(std::mt19937 &rng);

Slic3r::ClipperLib::JoinType legacy_join(clipper_join_type_t join);
const char *join_name(clipper_join_type_t join);

bool separated_by_one_mm(const IntegerRect &first, const IntegerRect &second);
bool on_segment(const Slic3r::Point &point,
                const Slic3r::Point &a,
                const Slic3r::Point &b);
Contacts find_contacts(const Slic3r::ExPolygons &expolygons);

} // namespace clipper_offset_test

#endif
