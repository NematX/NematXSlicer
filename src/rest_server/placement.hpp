/*
 * Scene placement operations for REST sessions.
 *
 * This module keeps arrangement and automatic orientation independent from
 * HTTP: callers resolve REST IDs to engine instance IDs, then apply the
 * operation to a private Model copy and publish it only when it changed.
 */
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "libslic3r/Arrange/ArrangeSettingsView.hpp"
#include "libslic3r/Point.hpp"

namespace Slic3r {
class DynamicPrintConfig;
class Model;
}

namespace Slic3r::Rest {

// Instance engine IDs selected by a route; empty means the operation's default target set.
using InstanceTargets = std::vector<std::uint64_t>;

struct ArrangeRequest {
    std::optional<double> distance;
    double distance_from_bed = 0.;
    bool rotations = false;
    arr2::ArrangeSettingsView::XLPivots alignment = arr2::ArrangeSettingsView::xlpFrontLeft;
    arr2::ArrangeSettingsView::GeometryHandling geometry = arr2::ArrangeSettingsView::ghConvex;
    arr2::ArrangeSettingsView::ArrangeStrategy strategy = arr2::ArrangeSettingsView::asAuto;
};

struct ArrangedInstance {
    std::uint64_t instance_id;
    std::optional<int> bed;
};

struct ArrangeOutcome {
    std::vector<ArrangedInstance> instances;
    bool changed = false;
};

struct OrientRequest {
    bool min_area = true;
    double overhang_angle = 60.;
};

struct OrientedInstance {
    std::uint64_t instance_id;
    Transform3d matrix;
};

struct OrientOutcome {
    std::vector<OrientedInstance> instances;
    bool changed = false;
};

// Return the GUI's distance default and the sequential-printing safety clearance.
double default_arrange_distance(const DynamicPrintConfig &config);
double minimum_arrange_distance(const DynamicPrintConfig &config);
// Return the largest millimeter distance that remains safe in scaled engine coordinates.
double maximum_arrange_distance_mm() noexcept;

// Arrange selected instances on the configured bed and report physical or virtual bed placement.
ArrangeOutcome arrange_instances(
    Model &scene,
    const DynamicPrintConfig &config,
    const InstanceTargets &targets,
    const ArrangeRequest &request);

// Orient selected instances from each instance's own transformed mesh.
OrientOutcome orient_instances(
    Model &scene,
    const InstanceTargets &targets,
    const OrientRequest &request);

} // namespace Slic3r::Rest
