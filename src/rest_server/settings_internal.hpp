/*
 * Shared value conversion helpers for the REST settings catalog and codec.
 *
 * These helpers translate stored configuration values into the JSON primitives
 * used by both modules; they carry no state and are safe to inline in either
 * translation unit.
 */

#pragma once

#include <cstdint>
#include <string>

#include <nlohmann/json.hpp>

#include "libslic3r/Config/ConfigOption.hpp"

namespace Slic3r::Rest::detail {

using Json = nlohmann::json;

// Wire names match the specification; COUNT is an internal sentinel and keeps
// the linear fallback.
inline const char *graph_type_name(GraphData::GraphType type)
{
    switch (type) {
    case GraphData::SQUARE:
        return "square";
    case GraphData::LINEAR:
        return "linear";
    case GraphData::SPLINE:
        return "spline";
    case GraphData::COUNT:
        break;
    }
    return "linear";
}

// Config values converted to the JSON primitives used by the catalog: scalars,
// points and graphs.
inline Json json_value(const std::string &value)
{
    return value;
}

inline Json json_value(bool value)
{
    return value;
}

inline Json json_value(unsigned char value)
{
    return value != 0;
}

inline Json json_value(std::int32_t value)
{
    return value;
}

inline Json json_value(double value)
{
    return value;
}

inline Json json_value(const Vec2d &value)
{
    return Json::array({value.x(), value.y()});
}

inline Json json_value(const Vec3d &value)
{
    return Json::array({value.x(), value.y(), value.z()});
}

inline Json json_value(const GraphData &value)
{
    Json points = Json::array();
    for (const Vec2d &point : value.data())
        points.push_back(Json::array({point.x(), point.y()}));
    return Json{{"type", graph_type_name(value.type)}, {"points", std::move(points)}};
}


// The vector flag is stored in the type value; the catalog exposes the element
// type.
inline ConfigOptionType scalar_type(ConfigOptionType type)
{
    if ((static_cast<int>(type) & static_cast<int>(coVectorType)) == 0)
        return type;
    return static_cast<ConfigOptionType>(static_cast<int>(type) - static_cast<int>(coVectorType));
}

} // namespace Slic3r::Rest::detail
