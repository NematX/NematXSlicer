///|/ Copyright (c) 2026 NematX
///|/ Released under the GNU Affero General Public License, version 3 or later.
#include "GridVarSpeedProperties.hpp"
#include "GridVarSpeedKernel.hpp"
#include <cstring>

namespace nematx_gridvarspeed {
using namespace slic3r_api;
namespace {
const raw_used_config_key used[]={
    {"max_print_speed",RAW_CO_FLOAT_OR_PERCENT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"max_volumetric_speed",RAW_CO_FLOAT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"filament_max_speed",RAW_CO_VECTOR_FLOAT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"filament_max_volumetric_speed",RAW_CO_VECTOR_FLOAT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"first_layer_speed",RAW_CO_FLOAT_OR_PERCENT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"first_layer_infill_speed",RAW_CO_FLOAT_OR_PERCENT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
};

class ProcessVisitor final:public ExtrusionTreeVisitor<> {
    struct State {
        double speed=0.,flow=0.;
        raw_extrusion_role role=RAW_EXTRUSION_ROLE_NONE;
        SegmentProperty segment{1.,0};
        bool fixed_extrusion=false;
        DefaultSpeedLimits limits{0.,0};
    };
    std::vector<State> stack;
    const Keys &keys;
    const plugin_run_context *run;
    const Print print;
    const PrintingToolGroup tool;
    const PrintingExtrusion source;
    orchestrator_handle *orchestrator;
    bool provider_checked=false;
public:
    ProcessVisitor(const Keys &k,const plugin_run_context *r,Print p,PrintingToolGroup t,
                   PrintingExtrusion s,orchestrator_handle *o):keys(k),run(r),print(p),tool(t),source(s),orchestrator(o) {}
private:
    void enter_node(MutableExtrusionEntity entity) override {
        throw_if_cancelled(run);
        State state=stack.empty()?State{}:stack.back();
        state.speed=inherited_speed(state.speed,entity.get(EPropertySpeed::key));
        if (const auto *p=entity.get(EPropertyAttributes::key)) {
            state.flow=p->mm3_per_mm(); state.role=raw_extrusion_role(p->extrusion_role());
        }
        if (const auto *p=entity.get(keys.segment)) state.segment=*p;
        if (const auto *p=entity.get(keys.speed_limits)) state.limits=*p;
        if (const auto *p=entity.get(EPropertySpecialCommand::key))
            state.fixed_extrusion=is_raw_filament_command(p);
        stack.push_back(state);
    }
    void leave_node(MutableExtrusionEntity) override {stack.pop_back();}
    void visit_leaf(MutableExtrusionEntity entity) override {
        const State s=stack.back();
        if (s.segment.version==0 || !entity.segment_count() || !RAW_EXTRUSION_ROLE_IS_INFILL(s.role) ||
            RAW_EXTRUSION_ROLE_IS_TRAVEL(s.role) || RAW_EXTRUSION_ROLE_IS_WIPE(s.role) ||
            RAW_EXTRUSION_ROLE_IS_RETRACT(s.role) || RAW_EXTRUSION_ROLE_IS_UNRETRACT(s.role) ||
            s.fixed_extrusion) return;
        if (s.segment.version!=1 || !std::isfinite(s.flow) || s.flow<=0.)
            throw std::runtime_error("GridVarSpeed: invalid segment metadata or flow");
        if (!provider_checked) {
            const auto *provider=orchestrator_select_plugin(orchestrator,print.config().handle(),STEP_LAYER_EXTRUSION_EDIT,"layer_extrusion_edit.speed");
            const char *id=orchestrator_plugin_id(provider);
            if (!id || std::strcmp(id,"layer_extrusion_edit.speed.default")!=0)
                throw std::runtime_error("GridVarSpeed requires the default extrusion speed provider");
            provider_checked=true;
        }
        if (entity.get(keys.applied))
            throw std::runtime_error("GridVarSpeed: speed already applied; expected a fresh ordering-plan clone");
        if (s.segment.factor==1.) return;
        const auto island=source.region_island();
        if (!island.valid() || !island.region_count() || tool.extruder_id()==EXTRUDER_ID_UNKNOWN)
            throw std::runtime_error("GridVarSpeed: missing process source or extruder");
        const auto layer=island.region(0).layer();
        const Config cfg=print.config();
        const auto extruder=tool.extruder_id();
        double cap=std::numeric_limits<double>::infinity();
        const auto add_cap=[&](double value) {
            if (!std::isfinite(value) || value<0.) throw std::runtime_error("GridVarSpeed: invalid speed limit");
            if (value>0.) cap=std::min(cap,value);
        };
        add_cap(cfg.computed_float_or_default("max_print_speed",extruder,0.));
        add_cap(cfg.float_or_default("max_volumetric_speed",0.)/s.flow);
        add_cap(cfg.vector_float_or_default("filament_max_speed",extruder,0.));
        add_cap(cfg.vector_float_or_default("filament_max_volumetric_speed",extruder,0.)/s.flow);
        if (layer.bottom_z()<=0) {
            if (s.limits.version!=1)
                throw std::runtime_error("GridVarSpeed: first-layer compensation requires resolved limits from the default speed provider");
            add_cap(s.limits.first_layer_mm_per_s);
        }
        const double speed=adjusted_speed(s.speed,s.segment.factor,cap);
        if (!entity.get(EPropertySpeed::key)) {
            auto &p=entity.get_or_add(EPropertySpeed::key);
            p.speed_mm_per_s=-1.f; p.accel_mm_per_s2=-1.f;
            p.pressure_adv=-1.f;
        }
        entity.get_or_add(EPropertySpeed::key).speed_mm_per_s=float(speed);
        entity.get_or_add(keys.applied).version=1;
    }
};

class Process final:public PluginBase {
    Keys keys;
public:
    explicit Process(orchestrator_handle *o):PluginBase(o),keys(o) {}
private:
    const char *id_impl() const noexcept override {return speed_id;}
    const char *name_impl() const noexcept override {return "Grid variable speed: crossing compensation";}
    const char *description_impl() const noexcept override {return "Adjusts only marked grid fragments after default speed, retaining physical speed limits.";}
    slicing_step_t step_impl() const noexcept override {return STEP_LAYER_EXTRUSION_EDIT;}
    const char *const *dependencies_impl() const noexcept override {return no_dependencies;}
    int32_t priority_impl() const noexcept override {return 2;}
    int32_t used_config_keys(raw_used_config_key *out) const noexcept override {
        if (out) std::copy(std::begin(used),std::end(used),out);
        return int32_t(std::size(used));
    }
    void run_impl(const plugin_run_context *run) const override {
        throw_if_cancelled(run);
        const auto *ctx=plugin_ctx_as_layer_extrusion_edition(run);
        if (!ctx || !ctx->print || !ctx->layer_group) throw std::runtime_error("GridVarSpeed: incomplete layer edit context");
        PrintingLayerGroup group(ctx->layer_group);
        for (uint32_t ti=0;ti<group.tool_group_count();++ti) {
            auto tool=group.tool_group(ti);
            for (uint32_t ei=0;ei<tool.extrusion_count();++ei) {
                auto extrusion=tool.extrusion(ei);
                ProcessVisitor visitor(keys,run,Print(ctx->print),tool,extrusion,m_orchestrator);
                visitor.traverse(extrusion.mutable_root());
            }
        }
    }
};
} // namespace

void register_process(orchestrator_handle *o) {
    static Process process(o);
    orchestrator_register_plugin(o,process.c_instance());
}
} // namespace nematx_gridvarspeed
