///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "ClipperShapes.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#ifdef _MSC_VER
#include <intrin.h>
#endif

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Int128.hpp"
#include "libslic3r/Polyline.hpp"

#include "Orchestrator.hpp"

namespace Slic3r {

// Retain the shape through the locked storage registry; null shapes return null and insertion errors leave no tracked handle.
static ApiClipper::ClipperShapes *store_shape(PluginStorage &plugin_storage, std::unique_ptr<ApiClipper::ClipperShapes> shape)
{
    return plugin_storage.store(std::move(shape));
}

static ClipperLib::Paths concat_paths(const ApiClipper::ClipperShapes *first, const ApiClipper::ClipperShapes *second)
{
    ClipperLib::Paths out = first != nullptr ? first->to_paths() : ClipperLib::Paths{};
    if (second != nullptr) {
        ClipperLib::Paths other = second->to_paths();
        out.insert(out.end(),
                   std::make_move_iterator(other.begin()),
                   std::make_move_iterator(other.end()));
    }
    return out;
}

namespace ApiClipper {

// Signed 128-bit value split into words: ordering the pairs (hi, lo) orders the values.
struct WideKey {
    int64_t hi;
    uint64_t lo;
};

// One ring edge on its support line: the direction is reduced by the gcd of its
// components and taken in the half-plane dx > 0, or dx == 0 and dy > 0, so every
// edge of one line has the same direction. offset identifies the line, begin and
// end bound the edge along it. Key is int64_t when all products fit 64 bits.
template<typename Key>
struct LineEdge {
    int64_t direction_x;
    int64_t direction_y;
    Key offset;
    Key begin;
    Key end;
};

// Collect the rings of a node and of all its descendants.
static void collect_tree_rings(const ClipperLib::PolyNode &node, std::vector<const ClipperLib::Path *> &rings);

// Check the edges of the rings, with 64-bit keys when they fit, else 128-bit ones.
static bool rings_have_overlapping_edges(const std::vector<const ClipperLib::Path *> &rings);

// Append the edges of every ring; false when one does not fit the key type.
template<typename Key>
static bool append_line_edges(const std::vector<const ClipperLib::Path *> &rings, std::vector<LineEdge<Key>> &edges);

// Greatest common divisor of two magnitudes, binary method.
static uint64_t binary_gcd(uint64_t first, uint64_t second);

// Number of trailing zero bits of a non-zero value.
static int trailing_zeros(uint64_t value);

// Fill the line keys of an edge from its reduced direction and its end points;
// the 64-bit version refuses an edge whose products could overflow.
static bool fill_line_keys(LineEdge<int64_t> &edge, const ClipperLib::IntPoint &a, const ClipperLib::IntPoint &b);
static bool fill_line_keys(LineEdge<WideKey> &edge, const ClipperLib::IntPoint &a, const ClipperLib::IntPoint &b);

// Sort the edges along their support lines and look for two that overlap.
template<typename Key>
static bool line_edges_overlap(std::vector<LineEdge<Key>> &edges);

static bool operator<(const WideKey &first, const WideKey &second);
static bool operator==(const WideKey &first, const WideKey &second);

static Polygons polygons_from_paths(const ClipperLib::Paths &paths)
{
    Polygons out;
    out.reserve(paths.size());
    for (const ClipperLib::Path &path : paths)
        out.emplace_back(path);
    return out;
}

// The islands of a tree, ordered from outside to inside (see traverse_pt()).
static ExPolygons expolygons_from_polytree(const ClipperLib::PolyTree &tree)
{
    ExPolygons out;
    traverse_pt(&tree, &out);
    return out;
}

static ClipperLib::PolyTree polytree_from_paths(const ClipperLib::Paths &paths, bool do_union)
{
    ClipperLib::Clipper clipper;
    clipper.AddPaths(paths, ClipperLib::ptSubject, true);
    ClipperLib::PolyTree tree;
    const ClipperLib::PolyFillType fill_type = do_union ? ClipperLib::pftNonZero : ClipperLib::pftEvenOdd;
    clipper.Execute(ClipperLib::ctUnion, tree, fill_type, fill_type);
    return tree;
}

static Point point_from_clipper(const ClipperLib::IntPoint &point)
{
    return Point(point.x(), point.y());
}

static void merge_path_bounding_box(BoundingBox &out, const ClipperLib::Path &path)
{
    for (const ClipperLib::IntPoint &point : path)
        out.merge(point_from_clipper(point));
}

/*
Count concrete PolyTree contours without materializing them as Paths.

Clipper's PolyTree root is a container node with an empty contour, so the root
does not contribute to the result. Every real child contour, including hole
contours, is counted when it stores at least one point.
*/
template<e_ordering ordering = e_ordering::OFF>
void traverse_pt_path_count(const ClipperLib::PolyNode *tree, size_t *size_out)
{
    if (tree == nullptr || size_out == nullptr)
        return;

    if (!tree->Contour.empty())
        ++*size_out;

    foreach_node<ordering>(tree->Childs, [size_out](const ClipperLib::PolyNode *child) {
        traverse_pt_path_count<ordering>(child, size_out);
    });
}

/*
Accumulate a PolyTree bounding box by visiting each node contour directly.

The PolyTree root only owns children and usually has no contour. That is fine:
empty contours add nothing, while all real child contours, including holes and
nested islands, contribute their points to the final box.
*/
template<e_ordering ordering = e_ordering::OFF>
void traverse_pt_bounding_box(const ClipperLib::PolyNode *tree, BoundingBox *out)
{
    if (tree == nullptr || out == nullptr)
        return;

    merge_path_bounding_box(*out, tree->Contour);

    foreach_node<ordering>(tree->Childs, [out](const ClipperLib::PolyNode *child) {
        traverse_pt_bounding_box<ordering>(child, out);
    });
}

class PathListShapes final : public ClipperShapes
{
public:
    explicit PathListShapes(ClipperLib::Paths paths) : m_paths(std::move(paths)) {}

    void add_to_clipper(ClipperLib::Clipper &clipper, ClipperLib::PolyType type, bool closed) const override
    {
        clipper.AddPaths(m_paths, type, closed);
    }

    ClipperLib::Paths to_paths() const override { return m_paths; }
    Polygons to_polygons() const override { return polygons_from_paths(m_paths); }
    ExPolygons to_expolygons() const override { return expolygons_from_polytree(polytree_from_paths(m_paths, false)); }
    bool empty() const override
    {
        for (const ClipperLib::Path &path : m_paths)
            if (!path.empty())
                return false;
        return true;
    }
    uint32_t path_count() const override
    {
        uint32_t out = 0;
        for (const ClipperLib::Path &path : m_paths)
            if (!path.empty())
                ++out;
        return out;
    }
    BoundingBox bounding_box() const override
    {
        BoundingBox out;
        for (const ClipperLib::Path &path : m_paths)
            merge_path_bounding_box(out, path);
        return out;
    }

    ClipperShapes *concat(const ClipperShapes &other, Slic3r::PluginStorage &) override
    {
        ClipperLib::Paths other_paths = other.to_paths();
        m_paths.insert(m_paths.end(),
                       std::make_move_iterator(other_paths.begin()),
                       std::make_move_iterator(other_paths.end()));
        return this;
    }

private:
    ClipperLib::Paths m_paths;
};

ClipperShapes *ClipperShapes::concat(const ClipperShapes &other, Slic3r::PluginStorage &storage)
{
    return Slic3r::store_shape(storage, make_path_list_shapes(Slic3r::concat_paths(this, &other)));
}

class PolyTreeShapes final : public ClipperShapes
{
public:
    explicit PolyTreeShapes(ClipperLib::PolyTree tree) : m_tree(std::move(tree)) {}

    void add_to_clipper(ClipperLib::Clipper &clipper, ClipperLib::PolyType type, bool closed) const override
    {
        ClipperLib::Paths paths = to_paths();
        clipper.AddPaths(paths, type, closed);
    }

    ClipperLib::Paths to_paths() const override
    {
        // Start from the root's children: the root is a container with an empty contour.
        Polygons polygons;
        traverse_pt(m_tree.Childs, &polygons);
        ClipperLib::Paths out;
        out.reserve(polygons.size());
        for (const Polygon &polygon : polygons)
            out.emplace_back(polygon.points);
        return out;
    }

    Polygons to_polygons() const override
    {
        Polygons out;
        traverse_pt(m_tree.Childs, &out);
        return out;
    }

    ExPolygons to_expolygons() const override
    {
        ExPolygons out;
        traverse_pt(&m_tree, &out);
        return out;
    }

    bool empty() const override { return m_tree.ChildCount() == 0; }
    bool has_normalized_rings() const override { return true; }
    uint32_t path_count() const override
    {
        size_t out = 0;
        traverse_pt_path_count(&m_tree, &out);
        return static_cast<uint32_t>(out);
    }
    BoundingBox bounding_box() const override
    {
        BoundingBox out;
        traverse_pt_bounding_box(&m_tree, &out);
        return out;
    }

private:
    ClipperLib::PolyTree m_tree;
};

class PolygonShapes final : public ClipperShapes
{
public:
    explicit PolygonShapes(const Polygon *polygon) : m_polygon(polygon) {}

    void add_to_clipper(ClipperLib::Clipper &clipper, ClipperLib::PolyType type, bool) const override
    {
        if (m_polygon != nullptr)
            clipper.AddPath(m_polygon->points, type, true);
    }

    ClipperLib::Paths to_paths() const override
    {
        return m_polygon == nullptr ? ClipperLib::Paths{} : ClipperLib::Paths{m_polygon->points};
    }

    Polygons to_polygons() const override
    {
        return m_polygon == nullptr ? Polygons{} : Polygons{*m_polygon};
    }

    ExPolygons to_expolygons() const override
    {
        return make_path_list_shapes(to_paths())->to_expolygons();
    }

    bool empty() const override { return m_polygon == nullptr || m_polygon->points.empty(); }
    uint32_t path_count() const override { return empty() ? 0 : 1; }
    BoundingBox bounding_box() const override
    {
        return m_polygon == nullptr ? BoundingBox{} : BoundingBox(m_polygon->points);
    }

private:
    const Polygon *m_polygon;
};

class PolylineShapes final : public ClipperShapes
{
public:
    explicit PolylineShapes(const Polyline *polyline) : m_polyline(polyline) {}

    void add_to_clipper(ClipperLib::Clipper &clipper, ClipperLib::PolyType type, bool) const override
    {
        if (m_polyline != nullptr)
            clipper.AddPath(m_polyline->points, type, false);
    }

    ClipperLib::Paths to_paths() const override
    {
        return m_polyline == nullptr ? ClipperLib::Paths{} : ClipperLib::Paths{m_polyline->points};
    }

    Polygons to_polygons() const override { return polygons_from_paths(to_paths()); }
    ExPolygons to_expolygons() const override { return make_path_list_shapes(to_paths())->to_expolygons(); }
    bool empty() const override { return m_polyline == nullptr || m_polyline->points.empty(); }
    uint32_t path_count() const override { return empty() ? 0 : 1; }
    BoundingBox bounding_box() const override
    {
        return m_polyline == nullptr ? BoundingBox{} : BoundingBox(m_polyline->points);
    }

private:
    const Polyline *m_polyline;
};

class PolygonsShapes final : public ClipperShapes
{
public:
    explicit PolygonsShapes(const Polygons *polygons) : m_polygons(polygons) {}

    void add_to_clipper(ClipperLib::Clipper &clipper, ClipperLib::PolyType type, bool closed) const override
    {
        if (m_polygons == nullptr)
            return;
        for (const Polygon &polygon : *m_polygons)
            clipper.AddPath(polygon.points, type, closed);
    }

    ClipperLib::Paths to_paths() const override
    {
        ClipperLib::Paths out;
        if (m_polygons != nullptr) {
            out.reserve(m_polygons->size());
            for (const Polygon &polygon : *m_polygons)
                out.emplace_back(polygon.points);
        }
        return out;
    }

    Polygons to_polygons() const override
    {
        return m_polygons == nullptr ? Polygons{} : *m_polygons;
    }

    ExPolygons to_expolygons() const override { return make_path_list_shapes(to_paths())->to_expolygons(); }

    bool empty() const override
    {
        if (m_polygons == nullptr)
            return true;
        for (const Polygon &polygon : *m_polygons)
            if (!polygon.points.empty())
                return false;
        return true;
    }

    uint32_t path_count() const override
    {
        if (m_polygons == nullptr)
            return 0;

        uint32_t out = 0;
        for (const Polygon &polygon : *m_polygons)
            if (!polygon.points.empty())
                ++out;
        return out;
    }

    BoundingBox bounding_box() const override
    {
        BoundingBox out;
        if (m_polygons != nullptr) {
            for (const Polygon &polygon : *m_polygons)
                out.merge(polygon.points);
        }
        return out;
    }

private:
    const Polygons *m_polygons;
};

class ExPolygonShapes final : public ClipperShapes
{
public:
    explicit ExPolygonShapes(const ExPolygon *expolygon) : m_expolygon(expolygon) {}

    void add_to_clipper(ClipperLib::Clipper &clipper, ClipperLib::PolyType type, bool) const override
    {
        if (m_expolygon == nullptr)
            return;
        clipper.AddPath(m_expolygon->contour.points, type, true);
        for (const Polygon &hole : m_expolygon->holes)
            clipper.AddPath(hole.points, type, true);
    }

    ClipperLib::Paths to_paths() const override
    {
        ClipperLib::Paths out;
        if (m_expolygon != nullptr) {
            out.reserve(m_expolygon->holes.size() + 1);
            out.emplace_back(m_expolygon->contour.points);
            for (const Polygon &hole : m_expolygon->holes)
                out.emplace_back(hole.points);
        }
        return out;
    }

    Polygons to_polygons() const override { return polygons_from_paths(to_paths()); }
    ExPolygons to_expolygons() const override
    {
        return m_expolygon == nullptr ? ExPolygons{} : ExPolygons{*m_expolygon};
    }

    bool empty() const override { return m_expolygon == nullptr || m_expolygon->contour.points.empty(); }
    bool has_normalized_rings() const override { return true; }
    uint32_t path_count() const override
    {
        if (m_expolygon == nullptr)
            return 0;

        uint32_t out = m_expolygon->contour.points.empty() ? 0 : 1;
        for (const Polygon &hole : m_expolygon->holes)
            if (!hole.points.empty())
                ++out;
        return out;
    }
    BoundingBox bounding_box() const override
    {
        BoundingBox out;
        if (m_expolygon != nullptr) {
            out.merge(m_expolygon->contour.points);
            for (const Polygon &hole : m_expolygon->holes)
                out.merge(hole.points);
        }
        return out;
    }

private:
    const ExPolygon *m_expolygon;
};

class ExPolygonsShapes final : public ClipperShapes
{
public:
    explicit ExPolygonsShapes(const ExPolygons *expolygons) : m_expolygons(expolygons) {}

    void add_to_clipper(ClipperLib::Clipper &clipper, ClipperLib::PolyType type, bool) const override
    {
        if (m_expolygons == nullptr)
            return;
        for (const ExPolygon &expolygon : *m_expolygons) {
            clipper.AddPath(expolygon.contour.points, type, true);
            for (const Polygon &hole : expolygon.holes)
                clipper.AddPath(hole.points, type, true);
        }
    }

    ClipperLib::Paths to_paths() const override
    {
        ClipperLib::Paths out;
        if (m_expolygons != nullptr) {
            for (const ExPolygon &expolygon : *m_expolygons) {
                out.emplace_back(expolygon.contour.points);
                for (const Polygon &hole : expolygon.holes)
                    out.emplace_back(hole.points);
            }
        }
        return out;
    }

    Polygons to_polygons() const override { return polygons_from_paths(to_paths()); }
    ExPolygons to_expolygons() const override
    {
        return m_expolygons == nullptr ? ExPolygons{} : *m_expolygons;
    }

    bool empty() const override
    {
        if (m_expolygons == nullptr)
            return true;
        for (const ExPolygon &expolygon : *m_expolygons)
            if (!expolygon.contour.points.empty())
                return false;
        return true;
    }
    bool has_normalized_rings() const override { return true; }
    uint32_t path_count() const override
    {
        if (m_expolygons == nullptr)
            return 0;

        uint32_t out = 0;
        for (const ExPolygon &expolygon : *m_expolygons) {
            if (!expolygon.contour.points.empty())
                ++out;
            for (const Polygon &hole : expolygon.holes)
                if (!hole.points.empty())
                    ++out;
        }
        return out;
    }
    BoundingBox bounding_box() const override
    {
        BoundingBox out;
        if (m_expolygons != nullptr) {
            for (const ExPolygon &expolygon : *m_expolygons) {
                out.merge(expolygon.contour.points);
                for (const Polygon &hole : expolygon.holes)
                    out.merge(hole.points);
            }
        }
        return out;
    }

private:
    const ExPolygons *m_expolygons;
};

/* ---- Zero-width bridge detection -------------------------------------
Two edges overlap along a positive length only if they lie on the same line.
Each edge gets its reduced direction, the cross product of that direction and a
point (the line offset) and the dot products of its ends (its interval along the
line). Sorting on (direction, offset, begin) makes the edges of one line
consecutive and ordered by start. The products fit 64 bits for directions below
2^26 and coordinates below 2^35 (34 m); otherwise every key is computed exactly
on 128 bits, so plates of any size are handled.
*/

bool has_overlapping_edges(const ClipperLib::PolyTree &tree)
{
    std::vector<const ClipperLib::Path *> rings;
    for (const ClipperLib::PolyNode *child : tree.Childs)
        collect_tree_rings(*child, rings);
    return rings_have_overlapping_edges(rings);
}

bool has_overlapping_edges(const ClipperLib::Paths &rings)
{
    std::vector<const ClipperLib::Path *> pointers;
    pointers.reserve(rings.size());
    for (const ClipperLib::Path &ring : rings)
        pointers.push_back(&ring);
    return rings_have_overlapping_edges(pointers);
}

static void collect_tree_rings(const ClipperLib::PolyNode &node, std::vector<const ClipperLib::Path *> &rings)
{
    rings.push_back(&node.Contour);
    for (const ClipperLib::PolyNode *child : node.Childs)
        collect_tree_rings(*child, rings);
}

static bool rings_have_overlapping_edges(const std::vector<const ClipperLib::Path *> &rings)
{
    std::vector<LineEdge<int64_t>> narrow_edges;
    if (append_line_edges(rings, narrow_edges))
        return line_edges_overlap(narrow_edges);
    std::vector<LineEdge<WideKey>> wide_edges;
    append_line_edges(rings, wide_edges);
    return line_edges_overlap(wide_edges);
}

template<typename Key>
static bool append_line_edges(const std::vector<const ClipperLib::Path *> &rings, std::vector<LineEdge<Key>> &edges)
{
    for (const ClipperLib::Path *ring : rings)
        for (size_t index = 0; index < ring->size(); ++index) {
            const ClipperLib::IntPoint &a = (*ring)[index];
            const ClipperLib::IntPoint &b = (*ring)[(index + 1) % ring->size()];
            int64_t direction_x = b.x() - a.x();
            int64_t direction_y = b.y() - a.y();
            if (direction_x == 0 && direction_y == 0)
                continue;
            const int64_t divisor = int64_t(binary_gcd(uint64_t(std::abs(direction_x)), uint64_t(std::abs(direction_y))));
            direction_x /= divisor;
            direction_y /= divisor;
            // Orient every direction the same way so opposite edges share a line.
            if (direction_x < 0 || (direction_x == 0 && direction_y < 0)) {
                direction_x = -direction_x;
                direction_y = -direction_y;
            }
            LineEdge<Key> edge;
            edge.direction_x = direction_x;
            edge.direction_y = direction_y;
            if (!fill_line_keys(edge, a, b))
                return false;
            edges.push_back(edge);
        }
    return true;
}

static uint64_t binary_gcd(uint64_t first, uint64_t second)
{
    // Stein's algorithm: shifts and subtractions instead of divisions.
    if (first == 0)
        return second;
    if (second == 0)
        return first;
    const int shift = trailing_zeros(first | second);
    first >>= trailing_zeros(first);
    do {
        second >>= trailing_zeros(second);
        if (first > second)
            std::swap(first, second);
        second -= first;
    } while (second != 0);
    return first << shift;
}

static int trailing_zeros(uint64_t value)
{
#ifdef _MSC_VER
    unsigned long index = 0;
    _BitScanForward64(&index, value);
    return int(index);
#else
    return __builtin_ctzll(value);
#endif
}

static bool fill_line_keys(LineEdge<int64_t> &edge, const ClipperLib::IntPoint &a, const ClipperLib::IntPoint &b)
{
    // Products of a component below 2^26 and a coordinate below 2^35 stay
    // below 2^61, so their sums cannot overflow.
    const int64_t direction_limit = int64_t(1) << 26;
    const int64_t coordinate_limit = int64_t(1) << 35;
    if (std::abs(edge.direction_x) >= direction_limit || std::abs(edge.direction_y) >= direction_limit ||
        std::abs(a.x()) >= coordinate_limit || std::abs(a.y()) >= coordinate_limit ||
        std::abs(b.x()) >= coordinate_limit || std::abs(b.y()) >= coordinate_limit)
        return false;
    const int64_t begin = edge.direction_x * a.x() + edge.direction_y * a.y();
    const int64_t end = edge.direction_x * b.x() + edge.direction_y * b.y();
    edge.offset = edge.direction_x * a.y() - edge.direction_y * a.x();
    edge.begin = std::min(begin, end);
    edge.end = std::max(begin, end);
    return true;
}

static bool fill_line_keys(LineEdge<WideKey> &edge, const ClipperLib::IntPoint &a, const ClipperLib::IntPoint &b)
{
    const Int128 offset = Int128::multiply(edge.direction_x, a.y()) - Int128::multiply(edge.direction_y, a.x());
    const Int128 begin = Int128::multiply(edge.direction_x, a.x()) + Int128::multiply(edge.direction_y, a.y());
    const Int128 end = Int128::multiply(edge.direction_x, b.x()) + Int128::multiply(edge.direction_y, b.y());
    edge.offset = {offset.hi(), offset.lo()};
    edge.begin = begin < end ? WideKey{begin.hi(), begin.lo()} : WideKey{end.hi(), end.lo()};
    edge.end = begin < end ? WideKey{end.hi(), end.lo()} : WideKey{begin.hi(), begin.lo()};
    return true;
}

template<typename Key>
static bool line_edges_overlap(std::vector<LineEdge<Key>> &edges)
{
    std::sort(edges.begin(), edges.end(), [](const LineEdge<Key> &first, const LineEdge<Key> &second) {
        return std::tie(first.direction_x, first.direction_y, first.offset, first.begin) <
               std::tie(second.direction_x, second.direction_y, second.offset, second.begin);
    });

    // Edges of one line are sorted by their start: one overlaps an earlier edge
    // when it starts strictly before the furthest end reached so far on that line.
    size_t furthest = 0;
    for (size_t index = 1; index < edges.size(); ++index) {
        const LineEdge<Key> &previous = edges[index - 1];
        const LineEdge<Key> &current = edges[index];
        const bool same_line = current.direction_x == previous.direction_x &&
                               current.direction_y == previous.direction_y && current.offset == previous.offset;
        if (!same_line) {
            furthest = index;
            continue;
        }
        if (current.begin < edges[furthest].end)
            return true;
        if (edges[furthest].end < current.end)
            furthest = index;
    }
    return false;
}

static bool operator<(const WideKey &first, const WideKey &second)
{
    return first.hi < second.hi || (first.hi == second.hi && first.lo < second.lo);
}

static bool operator==(const WideKey &first, const WideKey &second)
{
    return first.hi == second.hi && first.lo == second.lo;
}

std::unique_ptr<ClipperShapes> make_path_list_shapes(ClipperLib::Paths paths)
{
    return std::make_unique<PathListShapes>(std::move(paths));
}

std::unique_ptr<ClipperShapes> make_polytree_shapes(ClipperLib::PolyTree tree)
{
    return std::make_unique<PolyTreeShapes>(std::move(tree));
}

std::unique_ptr<ClipperShapes> make_polyline_shapes(const Polyline *polyline)
{
    return std::make_unique<PolylineShapes>(polyline);
}

std::unique_ptr<ClipperShapes> make_polygon_shapes(const Polygon *polygon)
{
    return std::make_unique<PolygonShapes>(polygon);
}

std::unique_ptr<ClipperShapes> make_polygons_shapes(const Polygons *polygons)
{
    return std::make_unique<PolygonsShapes>(polygons);
}

std::unique_ptr<ClipperShapes> make_expolygon_shapes(const ExPolygon *expolygon)
{
    return std::make_unique<ExPolygonShapes>(expolygon);
}

std::unique_ptr<ClipperShapes> make_expolygons_shapes(const ExPolygons *expolygons)
{
    return std::make_unique<ExPolygonsShapes>(expolygons);
}

} // namespace ApiClipper
} // namespace Slic3r
