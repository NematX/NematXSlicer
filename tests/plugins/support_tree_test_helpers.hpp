/* Shared helpers of the tree-family tests: the fixtures that route and generate the trees of a sliced object, the meshes of the
   legacy comparison, and the tools of the baseline dump (test_support_tree_dump.cpp).

   The baseline dump recorded what the tree family computed from native types, so that the plugin API implementation can
   be compared with it. A dump is a list of records, each with a key, free text lines compared as text, and rings compared as areas:

       @ <key>
       = <text line>
       r <signed area, informative> <point count> x0 y0 x1 y1 ...

   Rings are written in a canonical form so that two runs that differ only by the start point or the order of the rings give the
   same text: each ring is rotated to its lexicographically smallest point (the orientation is kept: contours counterclockwise and
   holes clockwise are how Clipper results carry their meaning), and the rings of a record are sorted by (signed area, first
   point, point count, points). Nothing in a dump is an address or depends on the machine.

   compare_dumps() lists every record whose content differs. A record whose rings are not canonically equal but whose symmetric
   difference is below ring_tolerance_mm2 with the same ring count is a drift (accepted, but listed so that it gets explained);
   a different ring count, a larger area, a text difference or a missing record is a change.

   A reference file is read with parse_dump(), which checks every line it extracts, and with duplicate_keys(): a malformed or
   ambiguous reference must stop the comparison, not be compared partially.

   The routing and the tip sampler need a STEP_SUPPORT context with the pattern callbacks: SupportPatternContext
   (support_pattern_test_context.hpp) builds one from a test-owned service, and Routed owns one. The generators of the slabs
   (generate_tree_slabs) still run with a null context, as the other tree tests do.

   canonical_ring() below is not the one of clipper_offset_test_helpers.hpp: that one starts the ring at its first smallest point
   only, while this one breaks ties between equal points by the points that follow, so that the form is unique for any ring. */
#ifndef support_tree_test_helpers_hpp_
#define support_tree_test_helpers_hpp_
#include "support_grid_test_helpers.hpp"
#include "support_pattern_test_context.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeRouting.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeSettings.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace Slic3r::Test::SupportTree {
namespace TF = Slic3r::TreeFamily;

// The positions of the routed nodes are API points; the checks that compare them with native geometry convert them.
inline Point to_point(c_point point) { return Point(point.x, point.y); }
inline c_point to_c_point(const Point &point) { return c_point{ point.x(), point.y() }; }

// One API ring as a native polygon (the points are copied through the view).
inline Polygon to_native_ring(const slic3r_api::Polygon &ring)
{
    const multipoint_const_view points = ring.view();
    Polygon polygon;
    polygon.points.reserve(points.size);
    for (uint32_t point_idx = 0; point_idx < points.size; ++point_idx)
        polygon.points.emplace_back(points.array[point_idx].x, points.array[point_idx].y);
    return polygon;
}

// The rings of an API polygon collection as native polygons (ring order kept); the checks that compare slabs with native geometry
// (Clipper, areas, distances) read them this way.
inline Polygons to_native(const slic3r_api::PolygonCollection &polygons)
{
    Polygons native;
    native.reserve(polygons.size());
    for (uint32_t ring_idx = 0; ring_idx < polygons.size(); ++ring_idx)
        native.push_back(to_native_ring(polygons.at(ring_idx)));
    return native;
}

// The expolygons of an API collection as native expolygons: the contour and the holes are copied through the views.
inline ExPolygons to_native(const slic3r_api::ExPolygonCollection &expolygons)
{
    ExPolygons native;

    native.reserve(expolygons.size());
    for (uint32_t idx = 0; idx < expolygons.size(); ++idx) {
        const slic3r_api::ExPolygon view = expolygons.at(idx);
        ExPolygon expolygon;
        expolygon.contour = to_native_ring(view.contour());
        for (uint32_t hole_idx = 0; hole_idx < view.hole_size(); ++hole_idx)
            expolygon.holes.push_back(to_native_ring(view.hole(hole_idx)));
        native.push_back(std::move(expolygon));
    }
    return native;
}

// Replaces the rings of a stored collection by native polygons (a slab filled by hand for a test).
inline void assign_native(slic3r_api::StoredPolygonCollection &collection, const Polygons &polygons)
{
    collection.clear();
    for (const Polygon &polygon : polygons) {
        slic3r_api::StoredPolygon ring(collection.storage());
        for (const Point &point : polygon.points)
            ring.push_back(c_point{ point.x(), point.y() });
        collection.push_back_move(std::move(ring));
    }
}

// Three simple objects without raft: an overhanging slab above the bed, a T whose bar overhangs both sides of its stem, and a
// shelf whose overhang lands partly on a lower lobe of the object, where branches stand on the object. Model 3 is the slab on a
// low platform of the tree volumes tests.
inline TriangleMesh comparison_mesh(int model_id)
{
    if (model_id == 0) return SupportGrid::slab();
    if (model_id == 3) return SupportGrid::slab(true);
    if (model_id == 1) {
        TriangleMesh stem = make_cube(4., 4., 6.);
        stem.translate(6., 2., 0.);
        TriangleMesh bar = make_cube(16., 4., 2.);
        bar.translate(0., 2., 6.);
        stem.merge(bar);
        return stem;
    }
    TriangleMesh post = make_cube(4., 8., 8.);
    TriangleMesh shelf = make_cube(20., 8., 1.);
    shelf.translate(0., 0., 8.);
    TriangleMesh lobe = make_cube(6., 8., 3.);
    lobe.translate(10., 0., 0.);
    post.merge(shelf);
    post.merge(lobe);
    return post;
}

// The tree settings of an object of a sliced print, read through the plugin API views as the family does.
inline TF::TreeSettings tree_settings(const Print &print, const PrintObject &object, const slic3r_api::Support::Common::SupportParams &params)
{
    return TF::TreeSettings(SupportGrid::object_view(object), SupportGrid::print_view(print), params);
}

// The routed trees of the first object of a sliced print, with the storage that owns the contacts they support and the STEP_SUPPORT
// context (pattern callbacks included) the tips are sampled with. The storage is declared first so that it outlives the contacts
// that live in it.
struct Routed {
    PluginStorage storage;
    slic3r_api::Support::Common::GeneratedSupport support{reinterpret_cast<storage_handle *>(&storage)};
    const slic3r_api::Support::Common::SupportParams params;
    const TF::TreeSettings settings;
    SupportPatternContext patterns;
    std::vector<TF::RoutedTree> trees;
    explicit Routed(Print &print)
        : params(SupportGrid::params(print, print.object(0))), settings(Slic3r::Test::SupportTree::tree_settings(print, print.object(0), params)),
          patterns(print, reinterpret_cast<storage_handle *>(&storage)),
          trees(TF::route_trees(SupportGrid::print_view(print), SupportGrid::object_view(print.object(0)), params, settings, support,
                                patterns.context())) {}
};

/* ============================ rings and records ============================ */

using Pt = std::pair<long long, long long>;

// A closed ring in the canonical form described above.
struct Ring {
    std::vector<Pt> points;
    bool operator==(const Ring &rhs) const { return points == rhs.points; }
    // Signed shoelace area in scaled units squared (positive counterclockwise).
    double area() const
    {
        double sum = 0.;
        for (size_t i = 0; i < points.size(); ++i) {
            const Pt &a = points[i], &b = points[(i + 1) % points.size()];
            sum += double(a.first) * double(b.second) - double(b.first) * double(a.second);
        }
        return 0.5 * sum;
    }
};

// Rotate the points so that the sequence starts at the lexicographically smallest rotation.
inline Ring canonical_ring(const std::vector<Pt> &points)
{
    Ring ring;
    const size_t n = points.size();
    if (n == 0)
        return ring;
    size_t best = 0;
    for (size_t i = 1; i < n; ++i)
        for (size_t k = 0; k < n; ++k) {
            const Pt &a = points[(i + k) % n], &b = points[(best + k) % n];
            if (a == b)
                continue;
            if (a < b)
                best = i;
            break;
        }
    for (size_t k = 0; k < n; ++k)
        ring.points.push_back(points[(best + k) % n]);
    return ring;
}

// Sort the rings by (signed area, first point, point count, points).
inline void sort_rings(std::vector<Ring> &rings)
{
    std::sort(rings.begin(), rings.end(), [](const Ring &a, const Ring &b) {
        const double area_a = a.area(), area_b = b.area();
        if (area_a != area_b)
            return area_a < area_b;
        return std::make_tuple(a.points.front(), a.points.size(), a.points) < std::make_tuple(b.points.front(), b.points.size(), b.points);
    });
}

// The canonical rings of native polygons.
inline std::vector<Ring> rings_of(const Polygons &polygons)
{
    std::vector<Ring> rings;
    for (const Polygon &polygon : polygons) {
        if (polygon.points.empty())
            continue;
        std::vector<Pt> points;
        for (const Point &point : polygon.points)
            points.emplace_back(point.x(), point.y());
        rings.push_back(canonical_ring(points));
    }
    sort_rings(rings);
    return rings;
}

// The canonical rings of an API polygon collection (stored or not): it only needs size() and operator[] giving points().
template<class Collection> inline std::vector<Ring> rings_of_view(const Collection &collection)
{
    std::vector<Ring> rings;
    for (uint32_t idx = 0; idx < collection.size(); ++idx) {
        std::vector<Pt> points;
        for (const c_point &point : collection[idx].points())
            points.emplace_back(point.x, point.y);
        if (points.empty())
            continue;
        rings.push_back(canonical_ring(points));
    }
    sort_rings(rings);
    return rings;
}

// One thing recorded: free text lines that must be equal, and rings that are compared as areas.
struct Record {
    std::string key;
    std::vector<std::string> lines;
    std::vector<Ring> rings;
};
struct Dump {
    std::vector<Record> records;
    // Appends an empty record and returns it. The reference is only valid until the next add(): fill it at once.
    Record &add(const std::string &key)
    {
        records.push_back(Record{key, {}, {}});
        return records.back();
    }
};

// A double as text, with max_digits10 significant digits so that two different doubles never give the same text. The "C" locale
// is used so that the decimal separator is a point whatever the locale of the machine.
inline std::string number(double value)
{
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    return out.str();
}

// The text of a dump in the format of the header. The "C" locale is used for the numbers, so the same dump gives the same text
// on every machine. Rings are written as they are held: callers store canonical rings.
inline std::string serialize(const Dump &dump)
{
    std::ostringstream out;
    out.imbue(std::locale::classic());
    for (const Record &record : dump.records) {
        out << "@ " << record.key << "\n";
        for (const std::string &line : record.lines)
            out << "= " << line << "\n";
        for (const Ring &ring : record.rings) {
            out << "r " << std::fixed << std::setprecision(1) << ring.area() << " " << ring.points.size();
            for (const Pt &point : ring.points)
                out << " " << point.first << " " << point.second;
            out << "\n";
        }
    }
    return out.str();
}

// Reads the text of a dump. Every line is checked as it is extracted: an unknown line, a text or ring line before any record, an
// empty key, and a ring with a missing, non-numeric or surplus field or no point are errors. Each error is appended to errors
// as "<label>:<line number>: <what>", and the offending line is skipped, so the caller can list every problem at once and must
// not use a dump that has errors. CRLF line ends are accepted.
inline Dump parse_dump(const std::string &text, const std::string &label, std::vector<std::string> &errors)
{
    Dump dump;
    std::istringstream in(text);
    in.imbue(std::locale::classic());
    std::string line;
    size_t number_of_line = 0;
    while (std::getline(in, line)) {
        ++number_of_line;
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        const std::string where = label + ":" + std::to_string(number_of_line) + ": ";
        if (line.size() >= 2 && line[0] == '@' && line[1] == ' ') {
            if (line.size() == 2)
                errors.push_back(where + "empty record key");
            else
                dump.add(line.substr(2));
        } else if (line.size() >= 2 && line[0] == '=' && line[1] == ' ') {
            if (dump.records.empty())
                errors.push_back(where + "text line before any record");
            else
                dump.records.back().lines.push_back(line.substr(2));
        } else if (line.size() >= 2 && line[0] == 'r' && line[1] == ' ') {
            if (dump.records.empty()) {
                errors.push_back(where + "ring before any record");
                continue;
            }
            std::istringstream fields(line.substr(2));
            fields.imbue(std::locale::classic());
            double area = 0.;
            size_t count = 0;
            if (!(fields >> area >> count) || count == 0) {
                errors.push_back(where + "ring line without a signed area and a positive point count: [" + line + "]");
                continue;
            }
            Ring ring;
            bool complete = true;
            for (size_t i = 0; i < count && complete; ++i) {
                long long x = 0, y = 0;
                complete = bool(fields >> x >> y);
                ring.points.emplace_back(x, y);
            }
            std::string extra;
            if (!complete)
                errors.push_back(where + "ring announces " + std::to_string(count) + " points but its coordinates are missing or not numbers");
            else if (fields >> extra)
                errors.push_back(where + "ring has more coordinates than its " + std::to_string(count) + " points");
            else
                dump.records.back().rings.push_back(std::move(ring));
        } else
            errors.push_back(where + "unknown line [" + line + "]");
    }
    return dump;
}

// The keys that appear more than once in a dump, as "<label>: duplicated key [<key>] in records <i> and <j>". A key must name one
// record: the comparison matches records by key, so a duplicate would hide one of them.
inline std::vector<std::string> duplicate_keys(const Dump &dump, const std::string &label)
{
    std::vector<std::string> errors;
    std::map<std::string, size_t> first_index;
    for (size_t i = 0; i < dump.records.size(); ++i) {
        const std::pair<std::map<std::string, size_t>::iterator, bool> inserted = first_index.emplace(dump.records[i].key, i);
        if (!inserted.second)
            errors.push_back(label + ": duplicated key [" + dump.records[i].key + "] in records " + std::to_string(inserted.first->second) + " and " + std::to_string(i));
    }
    return errors;
}

/* ============================== comparison ============================== */

// Symmetric difference below this, with the same ring count, is a drift and not a change (mm2, per record).
constexpr double ring_tolerance_mm2 = 1e-3;

struct Comparison {
    size_t identical = 0;
    std::vector<std::string> drifts, changes;
};

// Native polygons made of the points of the rings, in the same order and orientation.
inline Polygons polygons_of(const std::vector<Ring> &rings)
{
    Polygons polygons;
    for (const Ring &ring : rings) {
        Polygon polygon;
        for (const Pt &point : ring.points)
            polygon.points.emplace_back(coord_t(point.first), coord_t(point.second));
        polygons.push_back(std::move(polygon));
    }
    return polygons;
}

// Area in mm2 of the points that are in exactly one of the two ring sets (holes are the clockwise rings). Used to size a drift
// when two canonical ring lists differ.
inline double symmetric_difference_mm2(const std::vector<Ring> &a, const std::vector<Ring> &b)
{
    const Polygons pa = polygons_of(a), pb = polygons_of(b);
    return unscaled(unscaled(std::abs(area(diff(pa, pb))) + std::abs(area(diff(pb, pa)))));
}

// Compare a dump with its reference: records are matched by key, the text lines must be equal, the rings equal or a drift.
inline Comparison compare_dumps(const Dump &reference, const Dump &current)
{
    Comparison result;
    std::map<std::string, const Record *> by_key;
    for (const Record &record : current.records)
        by_key[record.key] = &record;
    std::map<std::string, bool> seen;
    for (const Record &ref : reference.records) {
        seen[ref.key] = true;
        const auto found = by_key.find(ref.key);
        if (found == by_key.end()) {
            result.changes.push_back(ref.key + ": missing from the current dump");
            continue;
        }
        const Record &now = *found->second;
        bool same = true;
        if (ref.lines.size() != now.lines.size()) {
            result.changes.push_back(ref.key + ": " + std::to_string(ref.lines.size()) + " text lines, now " + std::to_string(now.lines.size()));
            same = false;
        } else {
            for (size_t i = 0; i < ref.lines.size(); ++i)
                if (ref.lines[i] != now.lines[i]) {
                    result.changes.push_back(ref.key + ": line " + std::to_string(i) + " was [" + ref.lines[i] + "] now [" + now.lines[i] + "]");
                    same = false;
                }
        }
        if (ref.rings.size() != now.rings.size()) {
            result.changes.push_back(ref.key + ": ring count " + std::to_string(ref.rings.size()) + " now " + std::to_string(now.rings.size()) +
                                     ", symmetric difference " + number(symmetric_difference_mm2(ref.rings, now.rings)) + " mm2");
            same = false;
        } else if (!(ref.rings == now.rings)) {
            const double difference = symmetric_difference_mm2(ref.rings, now.rings);
            if (difference < ring_tolerance_mm2)
                result.drifts.push_back(ref.key + ": drift, symmetric difference " + number(difference) + " mm2");
            else
                result.changes.push_back(ref.key + ": symmetric difference " + number(difference) + " mm2");
            same = false;
        }
        if (same)
            ++result.identical;
    }
    for (const Record &now : current.records)
        if (!seen.count(now.key))
            result.changes.push_back(now.key + ": not in the reference");
    return result;
}
} // namespace Slic3r::Test::SupportTree
#endif
