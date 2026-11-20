///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Cubic octree interface
======================

The private octree owns the model-derived cells used by Adaptive Cubic and
Support Cubic. It exposes only construction and untrimmed line generation, so
the two pattern instances share geometry mechanics without sharing caches.
*/

#ifndef plugins_infill_adaptivecubic_adaptiveoctree_hpp_
#define plugins_infill_adaptivecubic_adaptiveoctree_hpp_

#include <array>
#include <memory>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_geometry.h"
#include "libslic3r/Api/plugin/c/slic3r_geometry_types.h"

struct plugin_run_context;

/*
Cubic octree
============

The cache stores transformed triangles in the octree frame and subdivides
intersected cells. Support mode filters model triangles by upward-facing normal
after retaining the complete mesh bounds; marked bridge triangles are always
inserted. Generated cube centers are rotated back before line extraction.
*/

namespace slic3r_api { namespace Infill { namespace AdaptiveCubic {

struct Vec3
{
    double x = 0.;
    double y = 0.;
    double z = 0.;
};

struct Triangle
{
    Vec3 a;
    Vec3 b;
    Vec3 c;
};

enum class CubicMode
{
    Adaptive,
    Support
};

struct CubeProperties
{
    double edge_length = 0.;
    double height = 0.;
    double diagonal_length = 0.;
    double line_z_distance = 0.;
    double line_xy_distance = 0.;
};

struct Cube
{
    explicit Cube(Vec3 cube_center) : center(cube_center) {}

    Vec3 center;
    std::array<std::unique_ptr<Cube>, 8> children = {};
};

class Octree
{
public:
    static std::unique_ptr<Octree> build(const std::vector<Triangle> &triangle_mesh,
                                         const std::vector<Triangle> &overhang_triangles,
                                         double line_spacing,
                                         CubicMode mode,
                                         const plugin_run_context *run_ctx = nullptr);

    std::vector<std::pair<c_point, c_point>> generate_lines(double z_position,
                                                          const plugin_run_context *run_ctx = nullptr) const;

private:
    Octree(Vec3 origin, std::vector<CubeProperties> properties);

    Vec3                 m_origin;
    std::vector<CubeProperties> m_cubes_properties;
    std::unique_ptr<Cube> m_root_cube;

};

}}} // namespace slic3r_api::Infill::AdaptiveCubic

#endif // plugins_infill_adaptivecubic_adaptiveoctree_hpp_
