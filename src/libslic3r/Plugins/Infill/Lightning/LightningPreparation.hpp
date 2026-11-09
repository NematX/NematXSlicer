///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef plugins_lightning_preparation_hpp_
#define plugins_lightning_preparation_hpp_

// Physical layer snapshots own outlines and parameters, never handles to mutable scratch geometry.
#include "libslic3r/Plugins/Common/PolygonWorkBuffers.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"

namespace slic3r_api { class Object; class Print; }

namespace slic3r_api { namespace Infill { namespace Lightning {
namespace work = PluginCommon::polygon_work;

struct RegionSettings
{
    double density = 0.;
    double width = 0.;
    bool width_percent = false;
};

struct Parameters
{
    float width_scaled = 0.f;
    coord_t supporting_radius = 0;
};

struct LayerInput
{
    const layer_handle *identity = nullptr;
    coord_t height = 0;
    coord_t print_z = 0;
    coord_t wall_radius = 0;
    coord_t prune_length = 0;
    coord_t straightening_distance = 0;
    work::Polygons outlines;
};

struct Inputs
{
    Parameters parameters;
    std::vector<LayerInput> layers;
};

// Region means are unweighted; percent widths use the automatic width, not the nozzle directly.
Parameters make_parameters(const std::vector<RegionSettings> &regions, double maximum_nozzle);
// Copy all internal sparse/void outlines; views and scratch handles never escape setup preparation.
Inputs read_inputs(const Print &print, const Object &object, storage_handle *scratch,
                    const plugin_run_context *run_ctx);
}}}
#endif
