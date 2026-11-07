///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

// Snapshot whole-object sparse outlines and physical layer deltas; only configured Lightning regions set density and width.
#include "LightningPreparation.hpp"

#include <cmath>
#include <stdexcept>

#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"

namespace slic3r_api { namespace Infill { namespace Lightning {

Parameters make_parameters(const std::vector<RegionSettings> &regions, double maximum_nozzle)
{
    const double automatic_width = 1.125f * float(maximum_nozzle);
    double density = 0., width = 0.;
    size_t count = 0;
    for (const RegionSettings &region : regions) {
        if (region.density <= 0.)
            continue;
        density += region.density;
        width += region.width == 0. ? automatic_width :
                 region.width_percent ? automatic_width * region.width / 100. : region.width;
        ++count;
    }
    if (count == 0)
        throw std::runtime_error("Lightning requires a configured Lightning region with positive density.");
    Parameters parameters;
    parameters.width_scaled = float(scale_d(width / double(count)));
    parameters.supporting_radius = coord_t(parameters.width_scaled / (density / double(count)));
    return parameters;
}

Inputs read_inputs(const Print &print, const Object &object, storage_handle *scratch,
                    const plugin_run_context *run_ctx)
{
    namespace clipper = PluginCommon::clipper;
    Inputs inputs;
    std::vector<RegionSettings> regions;
    for (uint32_t index = 0; index < object.print_region_count(); ++index) {
        throw_if_cancelled(run_ctx);
        const Config config = object.print_region(index).config();
        std::string pattern = config.get("fill_pattern").serialize();
        if (!pattern.empty() && pattern.front() == '!')
            pattern.erase(pattern.begin());
        if (pattern != "lightning")
            continue;
        const c_float_or_percent width = config.float_or_percent_or_default("infill_extrusion_width", {});
        regions.push_back({config.percent_or_default("fill_density", 0.), width.value, width.percent != 0});
    }
    double nozzle = 0.;
    const Config print_config = print.config();
    for (uint32_t index = 0; index < print_config.size("nozzle_diameter", 0); ++index)
        nozzle = std::max(nozzle, print_config.vector_float_or_default("nozzle_diameter", index, 0.));
    inputs.parameters = make_parameters(regions, nozzle);
    inputs.layers.reserve(object.layer_count());
    for (uint32_t index = 0; index < object.layer_count(); ++index) {
        throw_if_cancelled(run_ctx);
        const Layer layer = object.layer(index);
        LayerInput &input = inputs.layers.emplace_back();
        input.identity = layer.handle();
        input.height = layer.height();
        input.print_z = layer.print_z();
        // The wall supports this layer; pruning spans the actual distance to the layer below.
        input.wall_radius = coord_t(double(input.height) * std::tan(PI / 4.));
        if (index > 0) {
            const coord_t distance = input.print_z - inputs.layers[index - 1].print_z;
            input.prune_length = coord_t(double(distance) * std::tan(PI / 4.));
            input.straightening_distance = input.prune_length;
        }
        for (uint32_t island_index = 0; island_index < layer.island_count(); ++island_index) {
            const LayerIsland island = layer.island(island_index);
            for (uint32_t region_index = 0; region_index < island.region_island_count(); ++region_index) {
                const LayerRegionIsland region = island.region_island(region_index);
                for (const Surface surface : region.fill_surfaces()) {
                    if (!surface.has_all_flags(RAW_SURFACE_TYPE_POS_INTERNAL) ||
                        !surface.has_any_flag(RAW_SURFACE_TYPE_DENS_SPARSE | RAW_SURFACE_TYPE_DENS_VOID))
                        continue;
                    const ExPolygon polygon = surface.expolygon();
                    input.outlines.emplace_back().points = clipper::from_polygon_view(polygon.contour());
                    for (const Polygon hole : polygon.holes())
                        input.outlines.emplace_back().points = clipper::from_polygon_view(hole);
                }
            }
        }
        const work::ExPolygons united = clipper::union_ex(scratch, input.outlines);
        input.outlines.clear();
        for (const work::ExPolygon &polygon : united) {
            input.outlines.push_back(polygon.contour);
            input.outlines.insert(input.outlines.end(), polygon.holes.begin(), polygon.holes.end());
        }
    }
    return inputs;
}

}}}
