///|/ Copyright (c) 2026 NematX
///|/ Released under the GNU Affero General Public License, version 3 or later.
#pragma once
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionTreeVisitors.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Api/plugin/cpp/DefaultSpeedLimits.hpp"

namespace nematx_gridvarspeed {
constexpr const char *pattern_id="gridvarspeed";
constexpr const char *speed_id="nematx_gridvarspeed_speed";
constexpr const char *recipe_id="nematx_gridvarspeed_recipe";
constexpr const char *ratio_key="fill_gridvarspeed_width_diff";
constexpr const char *zhop_key="fill_gridvarspeed_zhop";
inline const char *const no_dependencies[]={nullptr};
struct SegmentProperty { double factor; uint32_t version; };
struct AppliedProperty { uint32_t version; };
inline bool is_raw_filament_command(const slic3r_api::EPropertySpecialCommand *command) {
    return command && command->code==C_EXTRUSION_SPECIAL_COMMAND_EXTRUSION;
}
inline double inherited_speed(double parent,const slic3r_api::EPropertySpeed *direct) {
    return direct && direct->speed_mm_per_s>0. ? double(direct->speed_mm_per_s) : parent;
}
struct Keys {
    slic3r_api::PluginPropertyKey<SegmentProperty> segment;
    slic3r_api::PluginPropertyKey<AppliedProperty> applied;
    slic3r_api::PluginPropertyKey<slic3r_api::DefaultSpeedLimits> speed_limits;
    explicit Keys(orchestrator_handle *o):
        segment(slic3r_api::PluginPropertyKey<SegmentProperty>::register_dynamic(o,"nematx.gridvarspeed.segment.v1")),
        applied(slic3r_api::PluginPropertyKey<AppliedProperty>::register_dynamic(o,"nematx.gridvarspeed.applied.v1")),
        speed_limits(slic3r_api::default_speed_limits_key(o)) {}
};

inline void order(slic3r_api::MutableExtrusionEntity entity,bool sortable,bool reversible) {
    uint32_t flags=entity.flags() & ~uint32_t(RAW_EXTRUSION_FLAG_SORTABLE|RAW_EXTRUSION_FLAG_REVERSIBLE);
    if (sortable) flags|=RAW_EXTRUSION_FLAG_SORTABLE;
    if (reversible) flags|=RAW_EXTRUSION_FLAG_REVERSIBLE;
    if (!entity.set_flags(flags)) throw std::runtime_error("GridVarSpeed: cannot set ordering flags");
}
void register_process(orchestrator_handle *);
} // namespace nematx_gridvarspeed
