///|/ Copyright (c) NematXSlicer 2026 Durand Remi @supermerill
///|/
///|/ NematXSlicer is released under the terms of the AGPLv3 or higher

#ifndef nematx_firmware_NematXObjectMetadata_hpp_
#define nematx_firmware_NematXObjectMetadata_hpp_

// Object and plater descriptions written at the start of a NematX G-code file,
// for the NematX controller and its web interface. Plain C++, so the text
// format can be unit-tested without the slicer.
//
// Each printed object instance gets one JSON comment line:
//   ; object:{"name":"cube id:0 copy 0","id":"0","object_id":0,"copy":0,
//             "object_center":[x,y,0],"boundingbox_center":[x,y,z],
//             "boundingbox_size":[x,y,z],"scale":[x,y,z],"outline":[[x,y],...]}
// and the whole plater one more:
//   ; plater:{"object_center":[x,y,0],"boundingbox_center":[x,y,z],
//             "boundingbox_size":[x,y,z],"outline":[[x,y],...]}
// (each on a single line in the file). All values are in millimetres, with
// three decimals.
//
// The controller reads lines of at most 4096 characters. A long outline is
// therefore continued on new comment lines starting with "; ": the JSON text
// is the concatenation of the lines without their "; " prefix. As in the legacy
// slicer, an object line is cut once it exceeds 3000 characters, a plater line
// once it exceeds 4000.

#include <algorithm>
#include <array>
#include <cstddef>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

namespace nematx_firmware {

using Point2 = std::array<double, 2>;
using Point3 = std::array<double, 3>;

// Axis-aligned 3D box; empty until the first point is added.
struct Box3
{
    Point3 min{};
    Point3 max{};
    bool defined = false;

    void merge(const Point3 &point)
    {
        if (!defined) {
            min = max = point;
            defined = true;
            return;
        }
        for (size_t axis = 0; axis < 3; ++axis) {
            min[axis] = std::min(min[axis], point[axis]);
            max[axis] = std::max(max[axis], point[axis]);
        }
    }
    void merge(const Box3 &other)
    {
        if (other.defined) {
            merge(other.min);
            merge(other.max);
        }
    }
    Point3 center() const { return { (min[0] + max[0]) / 2, (min[1] + max[1]) / 2, (min[2] + max[2]) / 2 }; }
    Point3 size() const { return { max[0] - min[0], max[1] - min[1], max[2] - min[2] }; }
};

// What the controller needs to know about one printed object instance.
struct ObjectMetadata
{
    std::string name;      // same name as in the "; printing object" comments
    size_t id = 0;         // position of the instance among all instances of the print
    size_t copy = 0;       // instance index inside its object
    Point2 center{};       // centroid of the outline
    Box3 box;              // 3D bounding box of the instance on the plater
    Point3 scale{ 1, 1, 1 };
    std::vector<Point2> outline; // simplified projection of the instance on the plater
};

// Line lengths after which an outline continues on a new comment line.
constexpr size_t k_object_line_limit = 3000;
constexpr size_t k_plater_line_limit = 4000;

namespace metadata_detail {

// JSON needs a decimal point regardless of the system locale.
inline void append_list(std::string &out, const double *values, size_t count)
{
    std::ostringstream list;
    list.imbue(std::locale::classic());
    list << std::fixed << std::setprecision(3) << '[';
    for (size_t i = 0; i < count; ++i) {
        if (i)
            list << ',';
        list << values[i];
    }
    list << ']';
    out += list.str();
}

// Write a name as a JSON string; the slicer names contain no quote, but a
// quote or backslash would otherwise break the JSON.
inline void append_string(std::string &out, const std::string &text)
{
    out += '"';
    for (char c : text) {
        if (c == '"' || c == '\\')
            out += '\\';
        out += c;
    }
    out += '"';
}

// Append "outline":[[x,y],...]}" to the current line (the part of `out` after
// `line_start`), starting a new "; " comment line when the current one is
// longer than `limit` before the next point. The legacy slicer cut the line
// after a point instead; the cuts are the same, except that a cut after the
// last point is no longer possible (it left a trailing comma in the JSON).
inline void append_outline(std::string &out, size_t line_start, const std::vector<Point2> &outline, size_t limit)
{
    out += ",\"outline\":[";
    for (size_t i = 0; i < outline.size(); ++i) {
        if (i) {
            out += ',';
            if (out.size() - line_start > limit) {
                out += "\n; ";
                line_start = out.size() - 2;
            }
        }
        append_list(out, outline[i].data(), 2);
    }
    out += "]}\n";
}

} // namespace metadata_detail

// One object line (possibly continued on several comment lines).
inline std::string format_object_metadata(const ObjectMetadata &object)
{
    using namespace metadata_detail;
    std::string out = "; object:{\"name\":";
    append_string(out, object.name);
    out += ",\"id\":\"" + std::to_string(object.id) + "\"";
    out += ",\"object_id\":" + std::to_string(object.id);
    out += ",\"copy\":" + std::to_string(object.copy);
    const double center[3] = { object.center[0], object.center[1], 0.0 };
    out += ",\"object_center\":";
    append_list(out, center, 3);
    out += ",\"boundingbox_center\":";
    append_list(out, object.box.center().data(), 3);
    out += ",\"boundingbox_size\":";
    append_list(out, object.box.size().data(), 3);
    out += ",\"scale\":";
    append_list(out, object.scale.data(), 3);
    append_outline(out, 0, object.outline, k_object_line_limit);
    return out;
}

// Convex hull of a set of points, counter-clockwise, starting from the point
// with the smallest x (then y). Collinear points are dropped.
inline std::vector<Point2> convex_hull(std::vector<Point2> points)
{
    std::sort(points.begin(), points.end());
    points.erase(std::unique(points.begin(), points.end()), points.end());
    if (points.size() < 3)
        return points;
    const auto cross = [](const Point2 &o, const Point2 &a, const Point2 &b) {
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    };
    // Andrew's monotone chain: lower hull from left to right, then upper hull back.
    std::vector<Point2> hull(2 * points.size());
    size_t k = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], points[i]) <= 0)
            --k;
        hull[k++] = points[i];
    }
    for (size_t i = points.size() - 1, lower = k + 1; i > 0; --i) {
        while (k >= lower && cross(hull[k - 2], hull[k - 1], points[i - 1]) <= 0)
            --k;
        hull[k++] = points[i - 1];
    }
    hull.resize(k - 1); // the last point is the first one again
    return hull;
}

// Area centroid of a polygon; the mean of its points when its area is zero.
inline Point2 polygon_centroid(const std::vector<Point2> &polygon)
{
    double area2 = 0, x = 0, y = 0;
    for (size_t i = 0; i < polygon.size(); ++i) {
        const Point2 &a = polygon[i];
        const Point2 &b = polygon[(i + 1) % polygon.size()];
        const double cross = a[0] * b[1] - b[0] * a[1];
        area2 += cross;
        x += (a[0] + b[0]) * cross;
        y += (a[1] + b[1]) * cross;
    }
    if (area2 != 0)
        return { x / (3 * area2), y / (3 * area2) };
    Point2 mean{};
    for (const Point2 &p : polygon) {
        mean[0] += p[0] / double(polygon.size());
        mean[1] += p[1] / double(polygon.size());
    }
    return mean;
}

// All the object lines, then the plater line, preceded and followed by a blank
// line as in the legacy slicer. Empty when no object is printed.
inline std::string format_objects_metadata(const std::vector<ObjectMetadata> &objects)
{
    using namespace metadata_detail;
    if (objects.empty())
        return {};
    std::string out = "\n";
    std::vector<Point2> all_points;
    Box3 plater_box;
    for (const ObjectMetadata &object : objects) {
        out += format_object_metadata(object);
        all_points.insert(all_points.end(), object.outline.begin(), object.outline.end());
        plater_box.merge(object.box);
    }
    // The plater outline is the convex hull of the object outlines. Unlike the
    // legacy line, the JSON starts without a stray comma: "plater:{"object_center"...
    const std::vector<Point2> hull = convex_hull(std::move(all_points));
    const Point2 hull_center = polygon_centroid(hull);
    const size_t line_start = out.size();
    out += "; plater:{\"object_center\":";
    const double center[3] = { hull_center[0], hull_center[1], 0.0 };
    append_list(out, center, 3);
    out += ",\"boundingbox_center\":";
    append_list(out, plater_box.center().data(), 3);
    out += ",\"boundingbox_size\":";
    append_list(out, plater_box.size().data(), 3);
    append_outline(out, line_start, hull, k_plater_line_limit);
    out += '\n';
    return out;
}

} // namespace nematx_firmware

#endif // nematx_firmware_NematXObjectMetadata_hpp_
