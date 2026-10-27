///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ Copyright (c) 2026 NematX
///|/ Released under the GNU Affero General Public License, version 3 or later.
#include "GridVarSpeedKernel.hpp"
#include "GridVarSpeedProperties.hpp"
#include "rectilinear/RectilinearFills.hpp"
#include <cstring>
#include <atomic>
#include <sstream>

namespace nematx_gridvarspeed {
using namespace slic3r_api;
namespace fills=slic3r_api::GridVarSpeedCopy::RectilinearPatternsPlugin::fills;
namespace {
const raw_used_config_key used[]={
    {ratio_key,RAW_CO_PERCENT,RAW_CONTAINER_TYPE_REGION,RAW_PRESET_TYPE_FFF_PRINT},
    {zhop_key,RAW_CO_FLOAT_OR_PERCENT,RAW_CONTAINER_TYPE_REGION,RAW_PRESET_TYPE_FFF_PRINT},
    {"infill_first",RAW_CO_BOOL,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
};
const char *const defined[]={ratio_key,zhop_key};

class Pattern final:public PluginBase {
    Keys keys;
    mutable std::atomic<bool> offset_warned{false};
public:
    explicit Pattern(orchestrator_handle *o):PluginBase(o),keys(o) {}
private:
    void setup_impl(const plugin_run_context *,uint32_t) const override {offset_warned.store(false);}
    const char *id_impl() const noexcept override {return pattern_id;}
    const char *name_impl() const noexcept override {return "Grid with varying speed and Z";}
    const char *description_impl() const noexcept override {return "Two connected grid passes with reduced crossing flow and compensated speed.";}
    slicing_step_t step_impl() const noexcept override {return INFILL_PATTERN;}
    const char *const *dependencies_impl() const noexcept override {return no_dependencies;}
    int32_t priority_impl() const noexcept override {return 30;}
    int32_t used_config_keys(raw_used_config_key *out) const noexcept override {
        if (out) std::copy(std::begin(used),std::end(used),out);
        return int32_t(std::size(used));
    }
    int32_t defined_config_keys(const char **out) const noexcept override {
        if (out) std::copy(std::begin(defined),std::end(defined),out);
        return int32_t(std::size(defined));
    }
    void inilialize_impl(storage_handle *) const override {
        for (bool is_ratio:{true,false}) {
            raw_config_option_def d=raw_config_option_def_init();
            d.opt_key=is_ratio?ratio_key:zhop_key;
            d.type=is_ratio?RAW_CO_PERCENT:RAW_CO_FLOAT_OR_PERCENT;
            d.container_type=RAW_CONTAINER_TYPE_REGION;
            d.option_preset_type=RAW_PRESET_TYPE_FFF_PRINT;
            d.printer_technology=RAW_PT_FFF;
            d.invalidates_step=STEP_INFILL;
            d.category=RAW_OPTION_CATEGORY_INFILL;
            d.level=RAW_OPTION_LEVEL_EXPERT;
            d.mode=RAW_CONFIG_OPTION_MODE_EXPERT;
            d.label=d.full_label=is_ratio?"Grid with varying speed: minimum width":"Grid with varying speed: Z offset";
            d.tooltip=is_ratio?
                "Local crossing flow ratio. 50% halves crossing flow and requests doubled speed, subject to speed limits. "
                "The legacy crossing-zone length also depends on this ratio: 0% (zero zone length) and 100% produce uniform passes, not zero-E travel.":
                "Offset in mm or percent of the extrusion height. With infill before perimeters, lowers the FIRST pass; otherwise raises the second. "
                "Clamped to half the effective extrusion height with a warning if exceeded. Flow and height are unchanged. Raised infill requires adequate travel clearance; test before printing.";
            d.sidetext=is_ratio?"%":"mm or %";
            d.default_serialized_value=is_ratio?"50%":"0";
            d.has_min=1; d.min_value=0.;
            if (is_ratio) {d.has_max=1; d.max_value=100.;}
            if (orchestrator_create_option_def(m_orchestrator,&d)!=OPTION_DEF_ERROR_OK)
                throw std::runtime_error("GridVarSpeed: option definition failed");
        }
        for (const char *list:{"fill_pattern","top_fill_pattern"}) {
            key_value_string_pair_t value={pattern_id,"Grid with varying speed and Z"};
            raw_option_def_patch patch=raw_option_def_patch_init();
            patch.kind=RAW_OPTION_DEF_PATCH_ADD_ENUM_VALUES;
            patch.opt_key=list; patch.enum_values={&value,1};
            if (orchestrator_modify_option_def(m_orchestrator,&patch)!=OPTION_DEF_ERROR_OK)
                throw std::runtime_error(std::string("GridVarSpeed: cannot add pattern to ")+list);
        }
    }
    void run_impl(const plugin_run_context *run) const override {
        throw_if_cancelled(run);
        const auto *ctx=plugin_ctx_as_infill_pattern(run);
        if (!ctx || !ctx->surface || !ctx->params || !ctx->output || !ctx->primary_region || !run->plugin_storage)
            throw std::runtime_error("GridVarSpeed: incomplete infill context");
        const auto &p=*ctx->params;
        if (p.infill_flags & (C_INFILL_FLAG_MONOTONIC|C_INFILL_FLAG_USE_ARACHNE))
            throw std::runtime_error("GridVarSpeed: monotonic/Arachne recipes are not supported");
        if (p.infill_flags & C_INFILL_FLAG_FILL_EXACTLY)
            throw std::runtime_error("GridVarSpeed: recipe modifier must run after any pattern replacement");
        if (!std::isfinite(p.density) || p.density<=0.0001f || p.density>1.f ||
            !std::isfinite(p.spacing) || p.spacing<=0. || !std::isfinite(p.angle) ||
            !std::isfinite(p.max_sparse_infill_spacing) || p.max_sparse_infill_spacing<0. ||
            !std::isfinite(p.bridge_angle) || !std::isfinite(p.flow_mult) || p.flow_mult<=0. ||
            !std::isfinite(p.flow.mm3_per_mm) || p.flow.mm3_per_mm<=0. ||
            p.flow.width<=0 || p.flow.height<=0 || p.flow.spacing<=0)
            throw std::runtime_error("GridVarSpeed: invalid infill recipe");
        const Config region=LayerRegion(ctx->primary_region).print_region().config();
        const double ratio=region.percent_or_default(ratio_key,0.5);
        if (!std::isfinite(ratio) || ratio<0. || ratio>1.)
            throw std::runtime_error("GridVarSpeed: minimum width must be between 0% and 100%");
        const double height=unscaled(p.flow.height);
        const double requested_offset=region.effective_float_or_percent_or_default(zhop_key,height,0.);
        const double effective_offset=bounded_offset(requested_offset,height);
        if (requested_offset>effective_offset+1e-12 && run->report_warning && !offset_warned.exchange(true)) {
            std::ostringstream message;
            message<<"GridVarSpeed: requested Z offset "<<requested_offset<<" mm limited to "<<effective_offset
                   <<" mm (half extrusion height) at Z="<<p.z<<" mm. Further affected extrusions are also limited.";
            run->report_warning(run->host_context,message.str().c_str());
        }
        const auto offsets=pass_offsets(effective_offset,
                                         height,p.z,region.bool_or_default("infill_first",false));
        fills::FillRectilinear filler;
        fills::configure_filler(filler,p,run->plugin_storage);
        Polylines passes[2];
        const ExPolygon surface=Surface(ctx->surface).expolygon();
        for (int pass=0;pass<2;++pass) {
            if (!filler.fill_surface_by_lines(surface,p,float(pass*3.14159265358979323846/2.),0.f,passes[pass]))
                throw std::runtime_error("GridVarSpeed: rectilinear sweep failed");
            remove_near_points(passes[pass],p.flow.spacing);
            throw_if_cancelled(run);
        }
        StoredExtrusionEntity root(run->plugin_storage);
        const double q0=p.flow.mm3_per_mm*p.flow_mult*legacy_flow_multiplier;
        const double w0=unscaled(p.flow.width)*p.flow_mult*legacy_flow_multiplier;
        if (!std::isfinite(q0) || q0<=0. || !std::isfinite(w0) || w0<=0. || w0>std::numeric_limits<float>::max())
            throw std::runtime_error("GridVarSpeed: invalid nominal extrusion attributes");
        for (int pass=0;pass<2;++pass) {
            Chains chains=split_crossings(passes[pass],passes[1-pass],ratio,p.flow.spacing);
            throw_if_cancelled(run);
            if (chains.empty()) continue;
            StoredExtrusionEntity group(run->plugin_storage);
            // Explicit zero on P0/P1 also prevents an unexpected ancestor offset
            // from silently altering their relative heights.
            group.get_or_add(EPropertyZOffset::key).set(pass?offsets.second:offsets.first);
            for (const Chain &chain:chains) {
                StoredExtrusionEntity path(run->plugin_storage);
                for (const Fragment &fragment:chain) {
                    std::vector<c_point> points;
                    for (Point pnt:fragment.points) points.push_back({pnt.x(),pnt.y()});
                    StoredExtrusionEntity leaf(run->plugin_storage,points);
                    leaf.get_or_add(EPropertyAttributes::key).extrusion_role(p.extrusion_role)
                        .mm3_per_mm(q0*fragment.factor).width(float(w0*fragment.factor)).height(float(height));
                    auto &metadata=leaf.get_or_add(keys.segment);
                    metadata.factor=fragment.factor; metadata.version=1;
                    order(leaf.mutable_view(),false,false);
                    path.append_child_move(leaf.mutable_view());
                }
                order(path.mutable_view(),false,true);
                group.append_child_move(path.mutable_view());
            }
            order(group.mutable_view(),true,true);
            root.append_child_move(group.mutable_view());
        }
        order(root.mutable_view(),false,false);
        if (root.child_count()) MutableExtrusionEntity(ctx->output).append_child_move(root.mutable_view());
    }
};

class Recipe final:public PluginBase {
public:
    explicit Recipe(orchestrator_handle *o):PluginBase(o) {}
private:
    const char *id_impl() const noexcept override {return recipe_id;}
    const char *name_impl() const noexcept override {return "Grid variable speed: legacy volume policy";}
    const char *description_impl() const noexcept override {return "Disables final full-fill normalization only for gridvarspeed.";}
    slicing_step_t step_impl() const noexcept override {return INFILL_SURFACE_RECIPE_MODIFIER;}
    const char *const *dependencies_impl() const noexcept override {return no_dependencies;}
    int32_t priority_impl() const noexcept override {return 1000;}
    void run_impl(const plugin_run_context *run) const override {
        throw_if_cancelled(run);
        const auto *ctx=plugin_ctx_as_infill_surface_recipe_modifier(run);
        if (!ctx || !ctx->params || !ctx->pattern_plugin_id)
            throw std::runtime_error("GridVarSpeed: incomplete recipe context");
        const char *id=ctx->pattern_plugin_id(ctx->generate_infill_ctx,ctx->params->pattern_id);
        if (id && std::strcmp(id,pattern_id)==0) ctx->params->infill_flags &= ~uint32_t(C_INFILL_FLAG_FILL_EXACTLY);
    }
};
} // namespace
// Kept outside the C entry point so native loader exception handling follows the
// same C++ registration path as the other SDK plugins.
void register_all(orchestrator_handle *o) {
    static Pattern pattern(o);
    static Recipe recipe(o);
    orchestrator_register_plugin(o,pattern.c_instance());
    orchestrator_register_plugin(o,recipe.c_instance());
    register_process(o);
    const char *members[]={pattern_id,recipe_id,speed_id};
    if (!orchestrator_register_activation_group(o,"nematx_gridvarspeed",{members,3}))
        throw std::runtime_error("GridVarSpeed: activation group registration failed");
}
} // namespace nematx_gridvarspeed

extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *o) {
    nematx_gridvarspeed::register_all(o);
}

// Keep last: includes the ABI requirements of every public header used here.
#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"
