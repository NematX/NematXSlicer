///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Cost seam model
===============
This private implementation surface contains the scalar and geometric costs
used by the built-in Cost seam provider. The model deliberately lives beside
the provider: it has no dependency on the legacy GCode seam placer and is not
part of the plugin ABI.

An optional process-wide observer gates visibility work in test executables.
Install it before starting workers; it must remain callable until all workers
finish. Ordinary applications leave it unset and retain the same cost model.
*/

#ifndef libslic3r_Plugins_SeamPlacer_SeamCostModel_hpp_
#define libslic3r_Plugins_SeamPlacer_SeamCostModel_hpp_

#include <functional>
#include <vector>
#include "libslic3r/Api/plugin/c/slic3r_mesh_queries.h"

#include "libslic3r/Api/plugin/c/slic3r_geometry.h"

namespace slic3r_api {
class Object;
}

namespace Slic3r {
namespace SeamCostModel {

// Called before visibility preparation or raycasting; path identifies the caller.
// A test observer may reject the work. It must be safe to call from worker threads.
using VisibilityCalculationObserver = void (*)(const char *path);
// Replace the observer outside active calculations; nullptr leaves visibility unrestricted.
void set_visibility_calculation_observer(VisibilityCalculationObserver observer);
// Invoke the installed observer, if any, before entering an expensive visibility stage.
void check_visibility_calculation(const char *path);

struct WeightedSplinePoint {
    double x = 0;
    double y = 0;
    double z = 0;
    double weight = 1;
};

float gauss(float value, float mean_x_coord, float mean_value, float falloff_speed);
float compute_angle_penalty(float ccw_angle);
std::vector<float> calculate_cost_angles(const std::vector<c_point> &points, float min_arm_length);
std::vector<float> calculate_cost_visibility(const slic3r_api::Object &object,
                                             const std::vector<c_vec3f> &points,
                                             const mesh_query_cancel *cancel = nullptr,
                                             const std::function<void(double)> &report_progress = {});
bool fit_weighted_cubic_bspline(const std::vector<WeightedSplinePoint> &points,
                                std::vector<c_vec2d> &fitted_points);

} // namespace SeamCostModel
} // namespace Slic3r

#endif
