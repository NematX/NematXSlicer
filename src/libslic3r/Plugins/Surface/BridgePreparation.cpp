///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Move bridge edge gaps over lower material and give bridge threads longer anchors.
   Grow their fill areas uniformly, then along the direction in which they print.

   This plugin also marks the first solid cover over internal sparse or void as
   an internal bridge, including a top cover. It retains the internal width and
   expansion settings; external margins never grow an internal bridge.

   This plugin chooses a direction for each unsupported bottom area, then extends
   into nearby internal infill areas above the model's lower slices. Uniform growth
   can add peripheral rows outside the unsupported area. The experimental volume provider checks
   their actual lower contacts when selected. Preparation does not generate supports or change flow.

   The resulting surfaces carry BridgeSurfaceInfo and are called prepared bridges.
   The recipe pass tells infill generation to print them as separate parallel rows.
   A recipe is the set of pattern, flow and connection parameters passed to an
   infill pattern. The experimental provider in BridgeVolume.cpp checks actual end contacts
   and can adjust flow; the surface preparation itself does neither.

   An island is one connected slice on a layer. Its region groups can share
   geometry but have different material settings. Partition those inputs by their
   physical regions before reserving any receiving area. Lower-layer queries use
   already prepared private outputs when available, so a lower bridge extension
   counts as solid before preparing the next cover.

   Representative direct calls, with external/internal branches shown separately:

       Preparation::run_impl()
           |-- prepare_island()
           |   |-- lower_sparse() [eligible lower sparse/void, by owning region]
           |   |-- internal_candidates() [reserve first solid covers, including top]
           |   |-- direction() [external bridges]
           |   |-- make_groups() [compatible external bridges]
           |   |-- expand_bridge() [uniform growth, then axial ends]
           |   |-- append_prepared() [external geometry and metadata]
           |   `-- prepare_internal()
           |       |-- direction() [detection, or perpendicular sparse fallback]
           |       |-- expand_bridge() [internal end expansion only]
           |       `-- append_prepared() [internal classification and metadata]
           `-- set_region_island_fill_surfaces() [host callback, one collection at a time]

   Recipe::run_impl() sets the direction, requests separate rows and fits their pitch,
   within bridge_overlap and bridge_overlap_min, and their phase to the area (BridgeRows.hpp). It runs as
   an infill-recipe modifier, not as a call from Preparation::run_impl(). The infill
   generator calls validate_recipe() after every modifier has run, so a later
   modifier cannot silently replace the prepared direction or parallel rows, nor space them outside the settings.

   Input surfaces stay unchanged while output collections are built in plugin
   storage. They are checked before replacement, but each collection is replaced
   through a separate host call, not one object-wide exchange. One object run is
   sequential; the host may run different objects in parallel.
*/
#include "BridgePreparation.hpp"
#include "BridgeGeometry.hpp"
#include "BridgeRows.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <map>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include "libslic3r/Api/plugin/c/steps/slic3r_step_surface_generation.h"
#include "libslic3r/Api/plugin/cpp/BridgeDetectorViews.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/BridgeProperties.hpp"

namespace slic3r_api { namespace BridgePreparation {
namespace {
using namespace BridgeGeometry;
constexpr raw_surface_type bridge_type=RAW_SURFACE_TYPE_POS_BOTTOM|RAW_SURFACE_TYPE_DENS_SOLID|RAW_SURFACE_TYPE_MOD_BRIDGE;
const char *surface_dependencies[]={"surface.solid_shells","bridge_detector.default",nullptr};
const char *recipe_dependencies[]={"surface.bridge_preparation",nullptr};
const raw_used_config_key used_keys[]={
    {"bridge_angle",RAW_CO_FLOAT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"bridged_infill_margin",RAW_CO_FLOAT_OR_PERCENT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"bridged_infill_uniform_margin",RAW_CO_FLOAT_OR_PERCENT,RAW_CONTAINER_TYPE_REGION,RAW_PRESET_TYPE_FFF_PRINT},
    {"bridge_precision",RAW_CO_FLOAT_OR_PERCENT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"bridge_type",RAW_CO_ENUM,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"bridge_flow_ratio",RAW_CO_PERCENT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"internal_bridge_min_width",RAW_CO_FLOAT_OR_PERCENT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"internal_bridge_expansion",RAW_CO_BOOL,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"fill_angle",RAW_CO_FLOAT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"fill_angle_increment",RAW_CO_FLOAT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"fill_angle_cross",RAW_CO_BOOL,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"fill_angle_template",RAW_CO_VECTOR_FLOAT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"fill_density",RAW_CO_PERCENT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"perimeters",RAW_CO_INT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"solid_infill_extruder",RAW_CO_INT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
    {"nozzle_diameter",RAW_CO_VECTOR_FLOAT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE}
};
struct Destination {
    LayerIsland island;
    std::vector<LayerRegion> regions;
    extruder_index_t tool;
    const layer_region_island_handle *target;
    StoredSurfaceCollection output;
};
struct Piece {
    Surface source;
    LayerRegion region;
    size_t destination;
    StoredExPolygonCollection area;
};
struct Candidate {
    size_t piece;
    double angle;
    bool from_user;
    ExpansionMargins margins;
    extruder_index_t tool;
    uint64_t key;
    StoredExPolygonCollection core;
};
struct InternalCandidate { size_t piece; StoredExPolygonCollection core; };
// Stop debug builds on a broken invariant of this plugin, then refuse the island
// like any other failed check so that it keeps its input surfaces.
[[noreturn]] void internal_error(const char *message)
{
    assert(false && "Broken bridge preparation invariant");
    throw std::runtime_error(message);
}
// Choose directions and extend bridge areas in one island without changing its
// input surfaces. Throw if region ownership or the resulting area partition is invalid;
// the caller then drops the island's private outputs, so it keeps its input surfaces.
void prepare_island(orchestrator_handle *orch, const run_ctx_surface_generation &ctx, storage_handle *storage,
    const LayerIsland &island, uint32_t layer_idx, uint32_t island_idx,
    std::vector<Destination> &outputs, const plugin_run_context *run_ctx);
// Return a printing direction in [0, PI) radians and report whether bridge_angle
// supplied an enabled user override. Otherwise detect it from scaled core/lower
// geometry. If detection fails and fallback is supplied, calculate that layer's
// normal fill angle (template/base, increment and alternating cross), then add
// fallback_turn degrees. prepare_internal() turns by 90 so the bridge crosses the
// lower sparse direction: it prefers the first overlapping lower sparse region's
// configuration, otherwise the current region's one. An external bridge keeps the
// current region's fill direction (turn 0), silently as in legacy SuperSlicer.
// Missing flow, failed detection without fallback or an invalid resulting angle throws.
double direction(orchestrator_handle *orch, const Config &print_config, const LayerRegion &region,
    const ExPolygonCollection &core, const ExPolygonCollection &lower, uint32_t layer_idx, bool &from_user,
    const Config *fallback=nullptr, uint32_t fallback_layer=0, double fallback_turn=90.);
// Copy the source surface's data onto the extended area, store its chosen angle
// and mark it with BridgeSurfaceInfo. A failed angle write throws before replacement.
void append_prepared(const run_ctx_surface_generation &ctx, Destination &destination, const Piece &source,
    const ExPolygonCollection &areas, double angle, const BridgeSurfaceInfo &info);
// Group nearby bridges before they take area from the surrounding infill. Each
// member must match the group's representative, not just its immediate neighbor.
std::vector<std::vector<size_t>> make_groups(storage_handle *storage,const std::vector<Candidate> &candidates,
    const std::vector<Piece> &pieces,const ExPolygonCollection &lower,const ExPolygonCollection &internal_cores,
    const plugin_run_context *run_ctx);
// Find wide enough first solid covers before either bridge family claims anchors.
// Their cores must not become receivers for neighboring external bridges.
std::vector<InternalCandidate> internal_candidates(storage_handle *storage,const std::vector<Piece> &pieces,const ExPolygonCollection &sparse);
// Find lower internal sparse/void, excluding full-density sparse and perimeter bands.
// The returned union is owned by storage and uses each lower region's own density.
StoredExPolygonCollection lower_sparse(storage_handle *storage,const LayerIsland &island,const std::vector<Destination> &outputs);
// Convert first solid covers and optionally extend their ends within this region's
// eligible infill. Reserve each claimed area once before ordinary surfaces publish.
void prepare_internal(orchestrator_handle *orch,const run_ctx_surface_generation &ctx,storage_handle *storage,
    const LayerIsland &island,uint32_t layer_idx,uint32_t island_idx,const std::vector<Piece> &pieces,
    const std::vector<InternalCandidate> &candidates,const ExPolygonCollection &sparse,const ExPolygonCollection &lower,size_t first_destination,
    std::vector<Destination> &outputs,std::map<const layer_region_handle *,StoredExPolygonCollection> &claimed,
    const plugin_run_context *run_ctx);

class Preparation final : public PluginBase {
public:
    // Bind this process-held provider to the loader's long-lived host.
    explicit Preparation(orchestrator_handle *orch) : PluginBase(orch) {}
private:
    // Give activation and diagnostics a stable surface-provider id.
    const char *id_impl() const noexcept override { return "surface.bridge_preparation"; }
    // Name the bridge-area pass in the plugin UI.
    const char *name_impl() const noexcept override { return "Bridge surface preparation"; }
    // Describe its geometry changes without claiming a physical bridge simulation.
    const char *description_impl() const noexcept override { return "Chooses bridge directions, widens bridge areas and extends their anchors within available infill."; }
    // Run while typed fill surfaces can still be reassigned.
    slicing_step_t step_impl() const noexcept override { return STEP_SURFACE_GENERATION; }
    // Wait for solid areas to be classified and for the bridge direction detector.
    const char *const *dependencies_impl() const noexcept override { return surface_dependencies; }
    // Run after solid shells and before top expansion or dense infill.
    int32_t priority_impl() const noexcept override { return 15; }
    // Return the key count, or also fill a caller-sized buffer with static key names.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override {
        if (keys!=nullptr) std::copy(std::begin(used_keys),std::end(used_keys),keys);
        return int32_t(std::size(used_keys));
    }
    // Define the uniform bridge margin used by the surface and recipe preparation passes.
    int32_t defined_config_keys(const char **keys) const noexcept override {
        if (keys!=nullptr) keys[0]="bridged_infill_uniform_margin";
        return 1;
    }
    // Register the uniform margin beside the existing infill anchor controls.
    void inilialize_impl(storage_handle *) const override;
    // Build and validate private object outputs, then replace their surface collections.
    // An island that fails keeps its input surfaces, and one warning per object counts
    // them; only cancellation stops the run. Each host call publishes one collection.
    void run_impl(const plugin_run_context *run_ctx) const override;
};

class Recipe final : public PluginBase {
public:
    // Bind the recipe provider to the same long-lived host as preparation.
    explicit Recipe(orchestrator_handle *orch) : PluginBase(orch) {}
private:
    // Identify the recipe member of the shared activation group.
    const char *id_impl() const noexcept override { return "infill.recipe.bridge_preparation"; }
    // Name the modifier in the plugin UI.
    const char *name_impl() const noexcept override { return "Prepared bridge recipe"; }
    // Modify each prepared surface's recipe before pattern generation.
    slicing_step_t step_impl() const noexcept override { return INFILL_SURFACE_RECIPE_MODIFIER; }
    // Require the earlier pass that marks bridge surfaces with BridgeSurfaceInfo.
    const char *const *dependencies_impl() const noexcept override { return recipe_dependencies; }
    // Place the modifier in the normal recipe chain; final validation follows all modifiers.
    int32_t priority_impl() const noexcept override { return 20; }
    // Return the key count, or also fill a caller-sized buffer with the overlap settings that bound the row pitch.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override {
        if (keys!=nullptr) {
            keys[0]={"bridge_overlap",RAW_CO_PERCENT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE};
            keys[1]={"bridge_overlap_min",RAW_CO_PERCENT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE};
        }
        return 2;
    }
    // Use the chosen bridge angle, request separate parallel rows and fit their pitch and phase to the area
    // (BridgeRows.hpp). Ignore a missing run payload or a surface without BridgeSurfaceInfo.
    void run_impl(const plugin_run_context *run_ctx) const override;
};

void Preparation::inilialize_impl(storage_handle *) const
{
    raw_config_option_def def=raw_config_option_def_init();
    def.opt_key="bridged_infill_uniform_margin";def.type=RAW_CO_FLOAT_OR_PERCENT;
    def.container_type=RAW_CONTAINER_TYPE_REGION;def.option_preset_type=RAW_PRESET_TYPE_FFF_PRINT;def.printer_technology=RAW_PT_FFF;
    def.category=RAW_OPTION_CATEGORY_INFILL;def.invalidates_step=STEP_PERIMETER;
    def.mode=RAW_CONFIG_OPTION_MODE_EXPERT|RAW_CONFIG_OPTION_MODE_SUSI;
    def.label="Uniform bridge margin";def.sidetext="mm or %";
    def.tooltip="Grow bridge infill in every direction before applying the additional bridge margin along the threads. "
        "Growth uses only available internal infill above lower model slices, to move edge gaps over lower material. "
        "A percentage uses the external perimeter width, as for Bridge margin. Set 0 to disable only uniform growth.";
    def.ratio_over="external_perimeter_extrusion_width";def.has_min=1;def.min_value=0.;
    def.has_max_literal=1;def.max_literal_value=50.;def.max_literal_is_percent=1;
    def.default_serialized_value="25%";
    orchestrator_create_option_def(m_orchestrator,&def);
    orchestrator_add_ui_fragment(m_orchestrator,"print.ui","surface.bridge_preparation",
        "page:Infill\n"
        "group:Advanced\n"
        "line:insert$afterline$Anchor solid infill by X mm:Uniform bridge margin\n"
        "setting:bridged_infill_uniform_margin\n"
        "end_line\n",1);
}

void Preparation::run_impl(const plugin_run_context *run_ctx) const
{
    const run_ctx_surface_generation *ctx=plugin_ctx_as_surface_generation(run_ctx);
    if (ctx==nullptr || ctx->object==nullptr || ctx->append_surface_like==nullptr || ctx->set_region_island_fill_surfaces==nullptr) {
        assert(false && "Bridge preparation requires the surface publication capabilities.");
        report_warning(run_ctx,"Bridge preparation skipped: the host provides no surface publication capabilities.");
        return;
    }
    const Object object(ctx->object);
    std::vector<Destination> outputs;
    size_t refused=0;
    std::string first_cause;
    for (uint32_t layer_idx=1; layer_idx<object.layer_count(); ++layer_idx) {
        const Layer layer=object.layer(layer_idx);
        for (uint32_t island_idx=0; island_idx<layer.island_count(); ++island_idx) {
            throw_if_cancelled(run_ctx);
            const size_t first_output=outputs.size();
            try {
                prepare_island(m_orchestrator,*ctx,run_ctx->plugin_storage,layer.island(island_idx),layer_idx,island_idx,outputs,run_ctx);
            } catch (const PluginCancelled &) {
                throw;
            } catch (const std::exception &error) {
                // Drop the island's private outputs: its input surfaces stay published unchanged.
                while (outputs.size()>first_output) outputs.pop_back();
                if (refused++==0)
                    first_cause="layer "+std::to_string(layer_idx)+", island "+std::to_string(island_idx)+": "+error.what();
            }
        }
    }
    if (refused>0)
        report_warning(run_ctx,("Object '"+object.name()+"': bridge preparation skipped "+std::to_string(refused)+
            " island(s), whose bridges keep their unprepared surfaces. First cause: "+first_cause).c_str());
    // Allocation of group descriptors precedes publication of any collection.
    for (Destination &out : outputs)
        if (out.target==nullptr)
            out.target=out.island.get_or_create_region_island(out.regions,out.tool).handle();
    throw_if_cancelled(run_ctx);
    for (Destination &out : outputs)
        ctx->set_region_island_fill_surfaces(const_cast<layer_region_island_handle *>(out.target),out.output.mutable_handle());
}

void prepare_island(orchestrator_handle *orch, const run_ctx_surface_generation &ctx, storage_handle *storage,
    const LayerIsland &island, uint32_t layer_idx, uint32_t island_idx,
    std::vector<Destination> &outputs, const plugin_run_context *run_ctx)
{
    bool has_bridges=false;
    for (uint32_t idx=0; idx<island.region_island_count(); ++idx)
        for (const Surface surface : island.region_island(idx).fill_surfaces_collection())
            has_bridges=has_bridges || surface.type()==bridge_type;
    StoredExPolygonCollection sparse=lower_sparse(storage,island,outputs);
    bool has_solid=false;
    for (uint32_t idx=0;idx<island.region_island_count();++idx)
        for (const Surface surface : island.region_island(idx).fill_surfaces_collection())
            has_solid=has_solid || (surface_type_is_solid(surface.type()) && !surface_type_is_bridge(surface.type()));
    if (!has_bridges && (!has_solid || sparse.empty())) return;
    const size_t first_destination=outputs.size();
    std::vector<Piece> pieces;
    ClipperContext clipper(storage);
    const double partition_tolerance=double(SCALED_EPSILON)*SCALED_EPSILON;
    const auto defect_area=[](const ExPolygonCollection &areas) {
        double area=0.;
        for (const ExPolygon polygon : areas)
            area+=std::abs(polygon.area());
        return area;
    };
    // Split shared region groups by physical ownership before assigning any tool
    // or anchor, so material settings cannot leak across a region boundary.
    for (uint32_t idx=0; idx<island.region_island_count(); ++idx) {
        const LayerRegionIsland source=island.region_island(idx);
        const size_t destination=outputs.size();
        outputs.push_back({island,source.regions(),source.extruder_id(),source.handle(),StoredSurfaceCollection(storage)});
        for (const Surface surface : source.fill_surfaces_collection()) {
            StoredExPolygonCollection original(storage);
            original.push_back(surface.expolygon());
            StoredExPolygonCollection covered(storage);
            for (const LayerRegion region : source.regions()) {
                StoredExPolygonCollection part=intersect(storage,original.readonly(),region.slices());
                if (part.empty()) continue;
                StoredExPolygonCollection overlap=intersect(storage,part.readonly(),covered.readonly());
                StoredExPolygonCollection opened=clipper_offset2(clipper(overlap.readonly()),-double(SCALED_EPSILON),double(SCALED_EPSILON)).to_expolygon_collection();
                if (defect_area(opened.readonly())>partition_tolerance)
                    throw std::runtime_error("Bridge preparation found overlapping physical region ownership.");
                covered.append_copy_from(part.readonly());
                pieces.push_back({surface,region,destination,std::move(part)});
            }
            StoredExPolygonCollection missing=subtract(storage,original.readonly(),covered.readonly());
            // A degenerate source sliver, thinner than the boolean precision, can
            // vanish from every region intersection. Tolerate the same defects as
            // the overlap check above; such a sliver is then dropped.
            StoredExPolygonCollection opened=clipper_offset2(clipper(missing.readonly()),-double(SCALED_EPSILON),double(SCALED_EPSILON)).to_expolygon_collection();
            if (defect_area(opened.readonly())>partition_tolerance)
                throw std::runtime_error("Bridge preparation cannot partition a source surface by its physical regions.");
        }
    }
    StoredExPolygonCollection lower(storage);
    for (const LayerIsland linked : island.lower_islands()) lower.push_back(linked.slice());
    lower=unite(storage,lower.readonly());
    const std::vector<InternalCandidate> internals=internal_candidates(storage,pieces,sparse.readonly());
    StoredExPolygonCollection internal_cores(storage);
    for (const InternalCandidate &candidate : internals) internal_cores.append_copy_from(candidate.core.readonly());
    internal_cores=unite(storage,internal_cores.readonly());
    std::vector<Candidate> candidates;
    const Config print_config=Print(ctx.print).config();
    for (size_t idx=0; idx<pieces.size(); ++idx) {
        Piece &piece=pieces[idx];
        if (piece.source.type()!=bridge_type) continue;
        bool from_user=false;
        const Config region_config=piece.region.print_region().config();
        const double angle=direction(orch,print_config,piece.region,piece.area.readonly(),lower.readonly(),layer_idx,from_user,
            &region_config,layer_idx,0.);
        const ExpansionMargins margins=expansion_margins(piece.region.handle());
        const extruder_index_t tool=layer_region_get_role_extruder(piece.region.handle(),RAW_EXTRUSION_ROLE_SOLID_INFILL);
        if (tool==EXTRUDER_ID_UNKNOWN) throw std::runtime_error("A prepared bridge has no resolved solid-infill tool.");
        candidates.push_back({idx,angle,from_user,margins,tool,geometry_key(piece.area.readonly()),piece.area.readonly().clone(storage)});
    }
    // Stable geometry order makes competing bridges claim receiver area once,
    // independently of the source surface enumeration order.
    std::sort(candidates.begin(),candidates.end(),[](const Candidate &a,const Candidate &b) {
        return a.key!=b.key ? a.key<b.key : geometry_less(a.core.readonly(),b.core.readonly());
    });
    const std::vector<std::vector<size_t>> groups=make_groups(storage,candidates,pieces,lower.readonly(),internal_cores.readonly(),run_ctx);
    std::map<const layer_region_handle *,StoredExPolygonCollection> claimed;
    for (const std::vector<size_t> &group : groups) {
        throw_if_cancelled(run_ctx);
        const Candidate &candidate=candidates[group.front()];
        Piece &source=pieces[candidate.piece];
        StoredExPolygonCollection receivers(storage);
        // Anchors may consume supported ordinary internal infill, not another
        // region's material, another bridge core or an exposed top/bottom surface.
        for (const Piece &piece : pieces)
            if (piece.region.handle()==source.region.handle() && surface_type_is_internal(piece.source.type()) &&
                !surface_type_is_bridge(piece.source.type()) &&
                (surface_type_is_solid(piece.source.type()) || surface_type_is_sparse(piece.source.type())))
                receivers.append_copy_from(piece.area.readonly());
        receivers=intersect(storage,receivers.readonly(),lower.readonly());
        receivers=subtract(storage,receivers.readonly(),internal_cores.readonly());
        std::map<const layer_region_handle *,StoredExPolygonCollection>::iterator previous=claimed.find(source.region.handle());
        if (previous!=claimed.end()) receivers=subtract(storage,receivers.readonly(),previous->second.readonly());
        StoredExPolygonCollection core(storage);
        for (const size_t member : group)
            core.append_copy_from(candidates[member].core.readonly());
        core=unite(storage,core.readonly());
        StoredExPolygonCollection domain=receivers.readonly().clone(storage);
        domain.append_copy_from(core.readonly());
        domain=unite(storage,domain.readonly());
        StoredExPolygonCollection expanded=expand_bridge(storage,core.readonly(),domain.readonly(),candidate.angle,
            candidate.margins.axial,candidate.margins.uniform,run_ctx);
        StoredExPolygonCollection added=subtract(storage,expanded.readonly(),core.readonly());
        // Do not store a second polygon in the bridge property. Added anchors must
        // lie above lower slices, so subtracting those slices recovers the original area.
        StoredExPolygonCollection invalid=subtract(storage,added.readonly(),lower.readonly());
        StoredExPolygonCollection reconstructed=subtract(storage,expanded.readonly(),lower.readonly());
        StoredExPolygonCollection lost=subtract(storage,core.readonly(),reconstructed.readonly());
        StoredExPolygonCollection extra=subtract(storage,reconstructed.readonly(),core.readonly());
        // Clipper can add or drop sub-epsilon slivers along nearly parallel edges, as
        // when a long core edge runs almost along the printing direction. Ignore the
        // defects that vanish under the same opening as the island partition check.
        for (const ExPolygonCollection defect : {invalid.readonly(),lost.readonly(),extra.readonly()}) {
            StoredExPolygonCollection opened=clipper_offset2(clipper(defect),-double(SCALED_EPSILON),double(SCALED_EPSILON)).to_expolygon_collection();
            if (defect_area(opened.readonly())>partition_tolerance)
                throw std::runtime_error("A bridge anchor does not preserve its reconstructible core.");
        }
        if (previous==claimed.end()) claimed.emplace(source.region.handle(),added.readonly().clone(storage));
        else previous->second.append_copy_from(added.readonly());
        size_t destination=outputs.size();
        for (size_t target=first_destination; target<outputs.size(); ++target)
            if (outputs[target].regions.size()==1 && outputs[target].regions.front().handle()==source.region.handle() && outputs[target].tool==candidate.tool) {
                destination=target; break;
            }
        if (destination==outputs.size())
            outputs.push_back({island,{source.region},candidate.tool,nullptr,StoredSurfaceCollection(storage)});
        BridgeSurfaceInfo info={};
        info.schema_version=1; info.core_rule=1;
        uint32_t region_idx=0;
        const Layer layer=source.region.layer();
        while (region_idx<layer.region_count() && layer.region(region_idx).handle()!=source.region.handle()) ++region_idx;
        if (region_idx==layer.region_count()) throw std::runtime_error("A bridge region is not owned by its layer.");
        info.logical_key=geometry_key(core.readonly());
        for (const uint64_t value : {uint64_t(layer_idx),uint64_t(island_idx),uint64_t(region_idx),uint64_t(candidate.tool)})
            info.logical_key=(info.logical_key^value)*1099511628211ull;
        info.policy_signature=(1469598103934665603ull^uint64_t(candidate.margins.axial))*1099511628211ull;
        info.policy_signature=(info.policy_signature^uint64_t(candidate.margins.uniform))*1099511628211ull;
        info.angle_from_user=candidate.from_user; info.extension_requested=candidate.margins.axial>0 || candidate.margins.uniform>0;
        append_prepared(ctx,outputs[destination],source,expanded.readonly(),candidate.angle,info);
    }
    prepare_internal(orch,ctx,storage,island,layer_idx,island_idx,pieces,internals,sparse.readonly(),lower.readonly(),
        first_destination,outputs,claimed,run_ctx);
    for (const Piece &piece : pieces) {
        if (piece.source.type()==bridge_type) continue;
        StoredExPolygonCollection remaining=piece.area.readonly().clone(storage);
        const std::map<const layer_region_handle *,StoredExPolygonCollection>::const_iterator reserved=claimed.find(piece.region.handle());
        if (reserved!=claimed.end()) remaining=subtract(storage,remaining.readonly(),reserved->second.readonly());
        if (!remaining.empty()) ctx.append_surface_like(outputs[piece.destination].output.mutable_handle(),piece.source.handle(),remaining.handle());
    }

    // Check for missing, overlapping or extra fill area before changing the island.
    // Use the surface step's same inward/outward offset to ignore sub-precision defects.
    StoredExPolygonCollection output_union(storage), overlaps(storage);
    for (size_t idx=first_destination; idx<outputs.size(); ++idx)
        for (const Surface surface : outputs[idx].output.readonly()) {
            StoredExPolygonCollection single(storage);
            single.push_back(surface.expolygon());
            StoredExPolygonCollection overlap=intersect(storage,single.readonly(),output_union.readonly());
            overlaps.append_copy_from(overlap.readonly());
            output_union.append_copy_from(single.readonly());
            output_union=unite(storage,output_union.readonly());
        }
    overlaps=unite(storage,overlaps.readonly());
    StoredExPolygonCollection missing=subtract(storage,island.infill_areas(),output_union.readonly());
    StoredExPolygonCollection extra=subtract(storage,output_union.readonly(),island.infill_areas());
    for (const ExPolygonCollection defect : {overlaps.readonly(),missing.readonly(),extra.readonly()}) {
        StoredExPolygonCollection opened=clipper_offset2(clipper(defect),-double(SCALED_EPSILON),double(SCALED_EPSILON)).to_expolygon_collection();
        if (defect_area(opened.readonly())>partition_tolerance)
            throw std::runtime_error("Bridge preparation does not preserve the complete island surface partition.");
    }
}

std::vector<std::vector<size_t>> make_groups(storage_handle *storage,const std::vector<Candidate> &candidates,
    const std::vector<Piece> &pieces,const ExPolygonCollection &lower,const ExPolygonCollection &internal_cores,
    const plugin_run_context *run_ctx)
{
    namespace bg=boost::geometry;
    namespace bgi=boost::geometry::index;
    using Point=bg::model::point<double,2,bg::cs::cartesian>;
    using Box=bg::model::box<Point>;
    using Entry=std::pair<Box,size_t>;
    std::vector<Entry> bounds;
    std::vector<std::vector<size_t>> members(candidates.size());
    std::vector<size_t> parent(candidates.size());
    for (size_t idx=0; idx<candidates.size(); ++idx) {
        const Candidate &candidate=candidates[idx];
        double min_x=std::numeric_limits<double>::infinity(),min_y=min_x,max_x=-min_x,max_y=-min_x;
        for (const ExPolygon area : candidate.core.readonly())
            for (const c_point point : area.contour().points()) {
                min_x=std::min(min_x,double(point.x));max_x=std::max(max_x,double(point.x));
                min_y=std::min(min_y,double(point.y));max_y=std::max(max_y,double(point.y));
            }
        const double reach=double(candidate.margins.axial)+double(candidate.margins.uniform);
        bounds.emplace_back(Box(Point(min_x-reach,min_y-reach),Point(max_x+reach,max_y+reach)),idx);
        members[idx].push_back(idx);parent[idx]=idx;
    }
    const bgi::rtree<Entry,bgi::quadratic<16>> index(bounds.begin(),bounds.end());
    // The box index only proposes pairs; actual expanded areas decide connectivity.
    std::vector<std::pair<size_t,size_t>> pairs;
    for (const Entry &bound : bounds) {
        std::vector<Entry> nearby;
        index.query(bgi::intersects(bound.first),std::back_inserter(nearby));
        for (const Entry &other : nearby)
            if (bound.second<other.second) pairs.emplace_back(bound.second,other.second);
    }
    std::sort(pairs.begin(),pairs.end());
    const auto root=[&parent](size_t idx) { while (parent[idx]!=idx) idx=parent[idx];return idx; };
    const auto compatible=[&](size_t a,size_t b) {
        const Candidate &x=candidates[a],&y=candidates[b];
        const Piece &source=pieces[x.piece],&other=pieces[y.piece];
        return source.region.handle()==other.region.handle() && x.tool==y.tool &&
            x.margins.axial==y.margins.axial && x.margins.uniform==y.margins.uniform &&
            std::abs(std::remainder(x.angle-y.angle,PI))<=1e-9 &&
            surface_metadata_compatible_for_bridge(source.source.handle(),other.source.handle());
    };
    std::map<const layer_region_handle *,StoredExPolygonCollection> receivers;
    for (const Piece &piece : pieces)
        if (surface_type_is_internal(piece.source.type()) && !surface_type_is_bridge(piece.source.type()) &&
            (surface_type_is_solid(piece.source.type()) || surface_type_is_sparse(piece.source.type()))) {
            std::map<const layer_region_handle *,StoredExPolygonCollection>::iterator entry=receivers.find(piece.region.handle());
            if (entry==receivers.end()) entry=receivers.emplace(piece.region.handle(),StoredExPolygonCollection(storage)).first;
            entry->second.append_copy_from(piece.area.readonly());
        }
    for (auto &[region, areas] : receivers) {
        areas=intersect(storage,areas.readonly(),lower);
        areas=subtract(storage,areas.readonly(),internal_cores);
    }
    for (const auto &[first,second] : pairs) {
        throw_if_cancelled(run_ctx);
        size_t a=root(first),b=root(second);
        if (a==b || !compatible(first,second)) continue;
        if (b<a) std::swap(a,b);
        // Checking every member against the representative prevents an otherwise
        // compatible chain of neighbors from joining incompatible end members.
        bool coherent=true;
        for (const size_t member : members[b]) coherent=coherent && compatible(a,member);
        for (const size_t member : members[a]) coherent=coherent && compatible(a,member);
        if (!coherent) continue;
        StoredExPolygonCollection cores=candidates[first].core.readonly().clone(storage);
        cores.append_copy_from(candidates[second].core.readonly());
        cores=unite(storage,cores.readonly());
        bool connected=cores.size()<candidates[first].core.size()+candidates[second].core.size();
        if (!connected) {
            StoredExPolygonCollection domain=cores.readonly().clone(storage);
            const std::map<const layer_region_handle *,StoredExPolygonCollection>::const_iterator available=receivers.find(pieces[candidates[first].piece].region.handle());
            if (available!=receivers.end()) domain.append_copy_from(available->second.readonly());
            domain=unite(storage,domain.readonly());
            StoredExPolygonCollection left=expand_bridge(storage,candidates[first].core.readonly(),domain.readonly(),candidates[first].angle,
                candidates[first].margins.axial,candidates[first].margins.uniform,run_ctx);
            StoredExPolygonCollection right=expand_bridge(storage,candidates[second].core.readonly(),domain.readonly(),candidates[second].angle,
                candidates[second].margins.axial,candidates[second].margins.uniform,run_ctx);
            StoredExPolygonCollection overlap=intersect(storage,left.readonly(),right.readonly());
            connected=!overlap.empty();
        }
        if (connected) {
            parent[b]=a;
            members[a].insert(members[a].end(),members[b].begin(),members[b].end());
            members[b].clear();
        }
    }
    std::vector<std::vector<size_t>> groups;
    for (size_t idx=0; idx<members.size(); ++idx)
        if (!members[idx].empty()) groups.push_back(std::move(members[idx]));
    return groups;
}

double direction(orchestrator_handle *orch, const Config &print_config, const LayerRegion &region,
    const ExPolygonCollection &core, const ExPolygonCollection &lower, uint32_t layer_idx, bool &from_user,
    const Config *fallback,uint32_t fallback_layer,double fallback_turn)
{
    const Config config=region.print_region().config();
    const ConfigOption override_angle=config.get("bridge_angle");
    from_user=override_angle.is_enabled();
    double angle=-1.;
    if (from_user) angle=override_angle.get_float()*PI/180.;
    else {
        const c_flow_mm flow=layer_region_get_infill_bridge_flow_mm(region.handle(),RAW_EXTRUSION_ROLE_SOLID_INFILL);
        if (!(flow.spacing>0.) || !std::isfinite(flow.spacing)) throw std::runtime_error("Bridge reference flow is unavailable.");
        bridge_detector_create_input input={};
        input.expolygons=core.handle(); input.lower_slices=lower.handle();
        input.spacing=scale_i(flow.spacing);
        input.precision=scale_i(print_config.get("bridge_precision").get_effective_value(flow.spacing));
        input.layer_id=layer_idx;
        BridgeDetector detector(orchestrator_create_bridge_detector(orch,&input));
        if (detector.valid() && detector.detect_angle()) angle=detector.angle();
        else if (fallback!=nullptr) {
            const ConfigOption angles=fallback->get("fill_angle_template");
            const double base=angles.size()>0 ? angles.get_float(fallback_layer%angles.size()) : fallback->get("fill_angle").get_float();
            const double cross=fallback->get("fill_angle_cross").get_bool() && (fallback_layer&1u)!=0 ? 90. : 0.;
            angle=(base+fallback->get("fill_angle_increment").get_float()*fallback_layer+cross+fallback_turn)*PI/180.;
            angle=std::fmod(angle,PI);if (angle<0.) angle+=PI;
        } else throw std::runtime_error("No bridge direction could be detected; specify bridge_angle or disable bridge preparation.");
    }
    if (!std::isfinite(angle) || angle<0.) internal_error("Invalid canonical bridge direction.");
    return std::fmod(angle,PI);
}

void append_prepared(const run_ctx_surface_generation &ctx, Destination &destination, const Piece &source,
    const ExPolygonCollection &areas, double angle, const BridgeSurfaceInfo &info)
{
    const uint32_t start=destination.output.size();
    ctx.append_surface_like(destination.output.mutable_handle(),source.source.handle(),areas.handle());
    for (uint32_t idx=start; idx<destination.output.size(); ++idx) {
        if (info.core_rule==2 && !destination.output.set_type(idx,source.source.type()|RAW_SURFACE_TYPE_MOD_BRIDGE))
            internal_error("The internal bridge classification could not be stored.");
        if (!destination.output.set_bridge_angle(idx,angle)) internal_error("The prepared bridge angle could not be stored.");
        destination.output.mutable_at(idx).get_or_add(BridgeSurfaceInfo::key)=info;
    }
}

StoredExPolygonCollection lower_sparse(storage_handle *storage,const LayerIsland &island,const std::vector<Destination> &outputs)
{
    StoredExPolygonCollection sparse(storage);
    const auto append_sparse=[&](const SurfaceCollection &surfaces,const std::vector<LayerRegion> &regions) {
        for (const Surface surface : surfaces) {
            if (!surface_type_is_internal(surface.type()) ||
                (!surface_type_is_sparse(surface.type()) && !surface_type_is_void(surface.type()))) continue;
            StoredExPolygonCollection area(storage);area.push_back(surface.expolygon());
            for (const LayerRegion owner : regions)
                if (surface_type_is_void(surface.type()) || surface_type_is_bridge(surface.type()) ||
                    owner.print_region().config().get("fill_density").get_effective_value(1.)<1.) {
                    StoredExPolygonCollection part=intersect(storage,area.readonly(),owner.slices());sparse.append_copy_from(part.readonly());
                }
        }
    };
    for (const LayerIsland lower : island.lower_islands()) {
        // Earlier private outputs already include lower bridge extensions. They
        // are solid now, even though the live source still describes sparse there.
        bool prepared=false;
        for (const Destination &out : outputs) if (out.island.handle()==lower.handle()) {
            prepared=true;append_sparse(out.output.readonly(),out.regions);
        }
        if (!prepared) for (uint32_t idx=0;idx<lower.region_island_count();++idx) {
            const LayerRegionIsland group=lower.region_island(idx);append_sparse(group.fill_surfaces_collection(),group.regions());
        }
    }
    return unite(storage,sparse.readonly());
}

std::vector<InternalCandidate> internal_candidates(storage_handle *storage,const std::vector<Piece> &pieces,const ExPolygonCollection &sparse)
{
    std::vector<InternalCandidate> candidates;
    ClipperContext clipper(storage);
    for (size_t idx=0;idx<pieces.size();++idx) {
        const Piece &piece=pieces[idx];const raw_surface_type type=piece.source.type();
        if (!surface_type_is_solid(type) || surface_type_is_bridge(type) ||
            (!surface_type_is_internal(type) && !surface_type_is_top(type))) continue;
        const Config config=piece.region.print_region().config();
        const c_flow solid=piece.region.flow(RAW_EXTRUSION_ROLE_SOLID_INFILL);
        const coord_t minimum=scale_i(config.get("internal_bridge_min_width").get_effective_value(unscaled(solid.spacing)));
        StoredExPolygonCollection core=intersect(storage,piece.area.readonly(),sparse),wide(storage);
        // A narrow component stays ordinary solid; never delete its fill area.
        for (const ExPolygon polygon : core.readonly()) {
            StoredExPolygonCollection single(storage);single.push_back(polygon);
            const StoredExPolygonCollection inset=clipper_offset(clipper(single.readonly()),-double(minimum)*.5).to_expolygon_collection();
            if (!inset.empty()) wide.append_copy_from(single.readonly());
        }
        if (!wide.empty()) candidates.push_back({idx,std::move(wide)});
    }
    std::sort(candidates.begin(),candidates.end(),[](const InternalCandidate &a,const InternalCandidate &b) {
        return geometry_less(a.core.readonly(),b.core.readonly());
    });
    return candidates;
}

void prepare_internal(orchestrator_handle *orch,const run_ctx_surface_generation &ctx,storage_handle *storage,
    const LayerIsland &island,uint32_t layer_idx,uint32_t island_idx,const std::vector<Piece> &pieces,
    const std::vector<InternalCandidate> &candidates,const ExPolygonCollection &sparse,const ExPolygonCollection &lower,size_t first_destination,
    std::vector<Destination> &outputs,std::map<const layer_region_handle *,StoredExPolygonCollection> &claimed,
    const plugin_run_context *run_ctx)
{
    StoredExPolygonCollection full=subtract(storage,lower,sparse);
    for (const InternalCandidate &candidate : candidates) {
        throw_if_cancelled(run_ctx);
        const Piece &piece=pieces[candidate.piece];const Config config=piece.region.print_region().config();
        Config fallback=config;uint32_t fallback_layer=layer_idx;
        const c_flow solid=piece.region.flow(RAW_EXTRUSION_ROLE_SOLID_INFILL);
        double reach=unscaled(solid.spacing);
        bool found=false;
        for (const LayerIsland lower_island : island.lower_islands())
            for (uint32_t j=0;j<lower_island.region_island_count();++j)
                for (const LayerRegion owner : lower_island.region_island(j).regions()) {
                    const StoredExPolygonCollection overlap=intersect(storage,candidate.core.readonly(),owner.slices());
                    if (overlap.empty()) continue;
                    const Config lower_config=owner.print_region().config();
                    const double density=lower_config.get("fill_density").get_effective_value(1.);
                    if (density>0. && density<1.) {
                        // One lower sparse pitch reaches the next possible supporting
                        // line without importing extrusion footprints into this pass.
                        reach=std::max(reach,unscaled(owner.flow(RAW_EXTRUSION_ROLE_INTERNAL_INFILL).spacing)/density);
                        if (!found) { fallback=lower_config;fallback_layer=layer_idx-1;found=true; }
                    }
                }
        bool from_user=false;
        const double angle=direction(orch,Print(ctx.print).config(),piece.region,candidate.core.readonly(),full.readonly(),layer_idx,from_user,&fallback,fallback_layer);
        const bool extension=config.get("internal_bridge_expansion").get_bool();
        StoredExPolygonCollection receivers(storage);
        for (const Piece &receiver : pieces)
            if (receiver.region.handle()==piece.region.handle() &&
                (surface_type_is_internal(receiver.source.type()) || receiver.source.handle()==piece.source.handle()) &&
                !surface_type_is_bridge(receiver.source.type()) &&
                (surface_type_is_solid(receiver.source.type()) || surface_type_is_sparse(receiver.source.type())))
                receivers.append_copy_from(receiver.area.readonly());
        for (const InternalCandidate &other : candidates) receivers=subtract(storage,receivers.readonly(),other.core.readonly());
        std::map<const layer_region_handle *,StoredExPolygonCollection>::iterator reserved=claimed.find(piece.region.handle());
        if (reserved!=claimed.end()) receivers=subtract(storage,receivers.readonly(),reserved->second.readonly());
        receivers.append_copy_from(candidate.core.readonly());receivers=unite(storage,receivers.readonly());
        StoredExPolygonCollection expanded=expand_bridge(storage,candidate.core.readonly(),receivers.readonly(),angle,extension ? scale_i(reach) : 0,0,run_ctx);
        if (reserved==claimed.end()) claimed.emplace(piece.region.handle(),expanded.readonly().clone(storage));
        else reserved->second.append_copy_from(expanded.readonly());
        const extruder_index_t tool=layer_region_get_role_extruder(piece.region.handle(),RAW_EXTRUSION_ROLE_SOLID_INFILL);
        if (tool==EXTRUDER_ID_UNKNOWN) throw std::runtime_error("An internal bridge has no resolved solid-infill tool.");
        size_t destination=first_destination;
        while (destination<outputs.size() && !(outputs[destination].regions.size()==1 &&
            outputs[destination].regions.front().handle()==piece.region.handle() && outputs[destination].tool==tool)) ++destination;
        if (destination==outputs.size()) outputs.push_back({island,{piece.region},tool,nullptr,StoredSurfaceCollection(storage)});
        BridgeSurfaceInfo info={};info.schema_version=1;info.core_rule=2;
        info.logical_key=geometry_key(candidate.core.readonly());
        const Layer layer=piece.region.layer();uint32_t region_idx=0;
        while (region_idx<layer.region_count() && layer.region(region_idx).handle()!=piece.region.handle()) ++region_idx;
        if (region_idx==layer.region_count()) throw std::runtime_error("An internal bridge region is not owned by its layer.");
        for (const uint64_t value : {uint64_t(layer_idx),uint64_t(island_idx),uint64_t(region_idx),uint64_t(tool),uint64_t(2)})
            info.logical_key=(info.logical_key^value)*1099511628211ull;
        info.policy_signature=(uint64_t(extension)^uint64_t(scale_i(reach)))*1099511628211ull;
        info.angle_from_user=from_user;info.extension_requested=extension;
        append_prepared(ctx,outputs[destination],piece,expanded.readonly(),angle,info);
    }
}

void Recipe::run_impl(const plugin_run_context *run_ctx) const
{
    const run_ctx_infill_surface_recipe_modifier *ctx=plugin_ctx_as_infill_surface_recipe_modifier(run_ctx);
    if (ctx==nullptr || ctx->surface==nullptr || Surface(ctx->surface).get(BridgeSurfaceInfo::key)==nullptr) return;
    raw_infill_pattern_params &params=*ctx->params;
    params.bridge_angle=surface_get_bridge_angle(ctx->surface);
    params.connection=C_INFILL_CONNECTION_NOT_CONNECTED;
    params.anchor_length=0.; params.anchor_length_max=0.; params.link_max_length=0;
    params.infill_flags &= ~C_INFILL_FLAG_CAN_ANGLE_CROSS;
    // Fit the rows: the pitch within the overlap settings and the phase that cover the bridge best. Without a fitting row the
    // recipe keeps its nominal pitch and lets the pattern choose the phase.
    if (ctx->primary_region==nullptr || !(params.spacing>0.)) return;
    const BridgeRows::PitchRange range=BridgeRows::pitch_range(LayerRegion(ctx->primary_region).print_region().config(),
        params.spacing,unscaled(params.flow.nozzle_diameter));
    const BridgeRows::Fit fit=BridgeRows::fit(run_ctx->plugin_storage,Surface(ctx->surface).expolygon(),params.bridge_angle,params.spacing,range);
    if (!fit.found()) return;
    params.density=float(params.spacing/fit.pitch);
    params.bridge_offset=fit.phase;
    params.infill_flags |= C_INFILL_FLAG_DONT_ADJUST;
}
}

ExpansionMargins expansion_margins(const layer_region_handle *handle)
{
    const LayerRegion region(handle);
    const Config config=region.print_region().config();
    const c_flow external=region.flow(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER), internal=region.flow(RAW_EXTRUSION_ROLE_INTERNAL_PERIMETER);
    const double width=unscaled(external.width);
    const ConfigOption axial=config.get("bridged_infill_margin");
    ExpansionMargins margins{axial.is_enabled() ? scale_i(std::max(0.,axial.get_effective_value(width))) : 0,
        scale_i(std::max(0.,config.get("bridged_infill_uniform_margin").get_effective_value(width)))};
    if (config.get("fill_density").get_float()<=0.) {
        const int32_t count=std::max(0,config.get("perimeters").get_int());
        const coord_t shell=count==0 ? 0 : coord_t(.5*external.width+external.spacing)+(count-1)*internal.spacing;
        margins.uniform=std::min(margins.uniform,shell);
        margins.axial=std::min(margins.axial,shell-margins.uniform);
    }
    return margins;
}

void validate_recipe(const surface_handle *surface, const raw_infill_pattern_params &params,
    const char *pattern_id, const layer_region_handle *region)
{
    if (surface==nullptr || Surface(surface).get(BridgeSurfaceInfo::key)==nullptr) return;
    const double angle=surface_get_bridge_angle(surface);
    if (pattern_id==nullptr || (std::strcmp(pattern_id,"rectilinear")!=0 && std::strcmp(pattern_id,"monotonic")!=0) ||
        !std::isfinite(params.bridge_angle) || angle<0. || std::abs(std::remainder(params.bridge_angle-angle,PI))>1e-9 ||
        params.connection!=C_INFILL_CONNECTION_NOT_CONNECTED || params.anchor_length!=0. || params.anchor_length_max!=0. ||
        params.link_max_length!=0 ||
        (params.infill_flags&(C_INFILL_FLAG_CAN_ANGLE_CROSS|C_INFILL_FLAG_ADD_GAP_FILL|C_INFILL_FLAG_GAP_FILL_ENABLED))!=0)
        throw std::runtime_error("A final bridge recipe no longer describes disconnected parallel rows.");
    const c_flow_mm reference=layer_region_get_infill_bridge_flow_mm(region,RAW_EXTRUSION_ROLE_SOLID_INFILL);
    const c_flow transport=layer_region_get_infill_bridge_flow(region,RAW_EXTRUSION_ROLE_SOLID_INFILL);
    const extruder_index_t tool=layer_region_get_role_extruder(region,RAW_EXTRUSION_ROLE_SOLID_INFILL);
    const bool internal=Surface(surface).get(BridgeSurfaceInfo::key)->core_rule==2;
    // The recipe pass may space the rows within the overlap settings and place them with bridge_offset.
    const BridgeRows::PitchRange range=BridgeRows::pitch_range(LayerRegion(region).print_region().config(),reference.spacing,
        unscaled(transport.nozzle_diameter));
    const double pitch=params.density>0.f ? params.spacing/double(params.density) : 0.;
    const bool spaced=params.density==1.f ||
        (pitch>=range.minimum*(1.-1e-6) && pitch<=range.maximum*(1.+1e-6) && params.bridge_offset>=0);
    if (!params.flow.is_bridge || params.flow_mult!=1.f || !spaced ||
        params.extrusion_role!=(internal ? RAW_EXTRUSION_ROLE_INTERNAL_BRIDGE_INFILL : RAW_EXTRUSION_ROLE_BRIDGE_INFILL) || params.surface_type!=surface_get_type(surface) ||
        tool==EXTRUDER_ID_UNKNOWN || params.extruder!=uint16_t(tool+1) ||
        params.flow.spacing!=scale_i(reference.spacing) || std::abs(params.spacing-reference.spacing)>1e-7 ||
        params.flow.width!=transport.width || params.flow.height!=transport.height ||
        params.flow.nozzle_diameter!=transport.nozzle_diameter ||
        params.flow.mm3_per_mm!=transport.mm3_per_mm || params.flow.spacing_ratio!=transport.spacing_ratio)
        throw std::runtime_error("A final bridge recipe changed the prepared reference flow or spacing.");
}

void register_surface(orchestrator_handle *orch)
{
    const char *members[]={"surface.bridge_preparation","infill.recipe.bridge_preparation"};
    if (!orchestrator_register_activation_group(orch,"bridge_preparation",{members,2}))
        throw std::runtime_error("Cannot register the bridge-preparation activation group.");
    static Preparation plugin(orch);
    orchestrator_register_plugin(orch,plugin.c_instance());
}
void register_recipe(orchestrator_handle *orch) { static Recipe plugin(orch); orchestrator_register_plugin(orch,plugin.c_instance()); }
}}
