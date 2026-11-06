///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Provide the experimental bridge-flow finalizer, using the occupied space of
   lower extrusion paths rather than assuming that every model slice is filled.
   It runs after model/support path generation and before extrusion ordering.

   BridgePreparation.cpp supplies surfaces with a chosen direction and a
   BridgeSurfaceInfo marker. For external bridges, subtract linked lower slices
   to recover the original unsupported core. For internal bridges, the complete
   prepared area is the working core. Generated infill rows refer back to their
   source surface, so this pass can check their direction, role and allowed area.

   An extrusion footprint is its XY path widened by half its width, together
   with the Z interval filled by the thread. index_layer() collects these from
   model and support paths. Support uses its actual auxiliary-layer Z, not the
   neighboring model layer's Z. Process model layers from bottom to top: correct
   a lower bridge's private copy before indexing it beneath a higher bridge.
   The current layer is not treated as material below itself.

   BridgeClearance::Index measures two different things. clearance() accounts
   for the bridge width when finding vertical space; contacts() asks which known
   lower paths reach the row's centerline. The contact check detects missing or
   one-sided anchors. Rows added beside the original core by uniform expansion
   must cross the uniformly expanded source area and are checked through their
   individual contact gaps; they need not cross the original core. A local support
   request can explain a missing anchor, but is never counted as deposited material.

   Circular incoming flow can be reduced by flattening its section while keeping
   width and printing Z. section_pieces() chooses constant-flow portions and
   transitions without going below the printable height floor. Unknown portions
   or unsupported incoming shapes retain their original flow without a fit claim.
   A floor-limited result can still overlap lower material. Disabling regional
   compensation retains flow but does not bypass row validation or contact checks.

   Selected calls are not a timeline. traverse() is inherited from
   ExtrusionTreeVisitors.hpp and invokes visit_leaf() as a virtual callback.

       Finalizer::run_impl()
           |-- index_layer()
           |   `-- OccupationVisitor::traverse() [inherited traversal]
           |       `-- OccupationVisitor::visit_leaf() [virtual callback]
           |           |-- occupied_segments()
           |           `-- BridgeClearance::Index::add()
           |-- CorrectionVisitor::traverse() [inherited traversal]
           |   `-- CorrectionVisitor::visit_leaf() [virtual callback]
           |       `-- correct_leaf()
           |           |-- BridgeClearance::Index::clearance()
           |           |-- BridgeClearance::Index::contacts() [known contacts only]
           |           |-- BridgeClearance::section_pieces()
           |           `-- publish_pieces()
           `-- replace_infill_buckets() [host publication callback]

   A strand is one continuous row, even when flow changes split it into pieces.
   publish_pieces() splits a private row where flow changes, preserves its joins
   and stores incoming/assigned flow in BridgeVolumeInfo on each piece. The
   containing collection is marked BridgeAtomicStrand so ordering keeps its
   pieces connected and reverses them only together. Input and geometry signatures
   make an unchanged replay a no-op; changed inputs require nominal infill to be
   regenerated. path_signature() also lets the host check later geometry edits.

   Work is done on copies owned by plugin storage; host objects are borrowed.
   Construction and correction errors leave the live infill unchanged. The host
   checks the complete replacement batch before exchanging any bucket content;
   root identities remain stable and the copies receive the old contents. This
   publication guarantee does not undo a completed exchange if later work fails.
   One object run is sequential; the host may process different objects in parallel.
   Geometry calculations use mm and cross-sectional areas use mm2 (mm3/mm).
*/
#include "BridgeVolume.hpp"
#include "BridgeClearance.hpp"
#include "libslic3r/Plugins/Surface/BridgeGeometry.hpp"
#include "libslic3r/Plugins/Surface/BridgePreparation.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_finalize_slicing.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionTreeVisitors.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/BridgeProperties.hpp"

namespace slic3r_api { namespace BridgeVolume {
namespace {
using namespace BridgeGeometry;
const char *dependencies[]={"surface.bridge_preparation","infill.recipe.bridge_preparation",nullptr};
// Keep the bridge's source surface and settings while correcting its paths.
// External core is recovered by subtracting lower slices; an internal bridge
// uses its whole prepared area because its void is inside the lower model.
// uniform_core is its clipped uniform expansion before the additional end margin.
struct SurfaceWork {
    Surface surface;
    LayerIsland island;
    LayerRegion region;
    StoredExPolygonCollection core;
    StoredExPolygonCollection uniform_core;
    uint64_t input_signature;
    double minimum_height;
    bool enabled;
    bool peripheral_rows;
    double layer_height;
    double bottom_z;
};
// A bucket is an extrusion tree for one region-island and role. Keep its original
// root beside the corrected copy; after replacement, output owns the old contents.
struct Bucket {
    const extrusion_entity_handle *source;
    uint32_t layer_idx;
    StoredExtrusionEntity output;
};
// Accumulate warning categories for one object rather than warning for every piece.
struct Diagnostics { bool unknown=false,floor=false,one_sided=false,unanchored=false; };
// Add one value to a hash used to detect changed inputs, without hashing padding.
uint64_t mix(uint64_t key,uint64_t value);
// Describe the space filled by one lower extrusion segment, in mm. Its XY shape
// is a line thickened by half the extrusion width, with round ends. Approximate
// arcs with short lines enlarged by their error; invalid arcs mark that space unknown.
// Reject arcs requiring more than one million chords.
std::vector<BridgeClearance::Occupation> occupied_segments(const c_extrusion_segment &segment,
    const EPropertyAttributes &attributes,double z,double offset);
// Add this layer's extrusion footprints and heights to the lower-material index;
// use corrected copies for infill that this run has already changed.
void index_layer(const Layer &layer,BridgeClearance::Index &index,uint64_t &signature,
    const std::map<const extrusion_entity_handle *,size_t> &clones,const std::vector<Bucket> &buckets,
    bool &unknown,const plugin_run_context *run_ctx);
// Validate a prepared row, check its end contacts and measure lower-path clearance.
// Disabling compensation retains flow but still checks the row and contacts. Unknown
// portions keep incoming flow. Invalid paths or outdated correction records throw
// before the private infill copies are published.
void correct_leaf(MutableExtrusionEntity leaf,const EPropertyAttributes &attributes,const SurfaceWork &surface,
    const BridgeClearance::Index &index,double z,const Config &print_config,
    const run_ctx_finalize_slicing &ctx,uint64_t signature,bool unknown,
    Diagnostics &diagnostics,const plugin_run_context *run_ctx,const EPropertyInfill &recipe);
// Split the copied path where its cross section changes and record each piece's
// incoming and adjusted flow. Too-short intervals, failed splits and real gaps throw.
void publish_pieces(MutableExtrusionEntity leaf,const EPropertyAttributes &attributes,
    const std::vector<BridgeClearance::SectionPiece> &pieces,uint64_t logical_key,uint64_t signature,
    const plugin_run_context *run_ctx);

class OccupationVisitor final : public ExtrusionTreeConstVisitor<> {
public:
    BridgeClearance::Index &index;
    uint64_t &signature;
    double z;
    bool &unknown;
    const plugin_run_context *ctx;
    // Borrow the index, signature and run state for this synchronous traversal.
    OccupationVisitor(BridgeClearance::Index &index,uint64_t &signature,double z,bool &unknown,const plugin_run_context *ctx)
        : index(index),signature(signature),z(z),unknown(unknown),ctx(ctx) {}
private:
    // Record the space filled by a path using its inherited extrusion dimensions.
    // Missing or unsupported data marks that space unknown instead of empty.
    void visit_leaf(ExtrusionEntity leaf) override;
};
class CorrectionVisitor final : public ExtrusionTreeVisitor<> {
public:
    const std::map<uint64_t,SurfaceWork> &surfaces;
    const BridgeClearance::Index &index;
    double z;
    const Config &print_config;
    const run_ctx_finalize_slicing &context;
    uint64_t signature;
    bool unknown;
    Diagnostics &diagnostics;
    const plugin_run_context *ctx;
    // Borrow immutable surface/index inputs and mutable diagnostics until traversal ends.
    CorrectionVisitor(const std::map<uint64_t,SurfaceWork> &surfaces,const BridgeClearance::Index &index,double z,
        const Config &config,const run_ctx_finalize_slicing &context,uint64_t signature,bool unknown,
        Diagnostics &diagnostics,const plugin_run_context *ctx)
        : surfaces(surfaces),index(index),z(z),print_config(config),context(context),signature(signature),
          unknown(unknown),diagnostics(diagnostics),ctx(ctx) {}
private:
    // Correct paths whose source surface was marked by BridgePreparation.cpp;
    // add object and bridge identifiers to errors without changing that surface.
    void visit_leaf(MutableExtrusionEntity leaf) override;
};
class Finalizer final : public PluginBase {
public:
    // Bind the process-held finalizer to the loader's long-lived host.
    explicit Finalizer(orchestrator_handle *orch) : PluginBase(orch) {}
    // Describe the native model selector after registering this member. The host
    // owns its active choices, default, UI and dispatch; no model enum is defined here.
    void register_group() const;
private:
    // Keep each model independently activatable under its stable provider id.
    const char *id_impl() const noexcept override { return "bridge.finalize_volume"; }
    // Name the plugin's flow correction and end-contact checks in the UI.
    const char *name_impl() const noexcept override { return "Experimental bridge clearance from extrusions"; }
    // Let the host select this member for the bridge-volume finalization job.
    const char *exclusive_group_impl() const noexcept override { return "bridge_volume"; }
    // Keep the group preference independent of execution order or registration order.
    int32_t exclusive_group_priority_impl() const noexcept override { return 10; }
    // Describe the experimental footprint audit without claiming a physical simulation.
    const char *description_impl() const noexcept override { return "Experimental: compensates bridge volume locally against actual lower extrusion footprints."; }
    // Wait for support paths so they count as lower material, before paths are ordered.
    slicing_step_t step_impl() const noexcept override { return STEP_FINALIZE_SLICING; }
    // Require the bridge areas and printing directions chosen by BridgePreparation.cpp.
    const char *const *dependencies_impl() const noexcept override { return dependencies; }
    // Alternatives have the same execution priority; the exclusive group chooses one.
    int32_t priority_impl() const noexcept override { return 20; }
    // Return the used-key count, optionally filling a caller-sized buffer with static names.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Share the regional flow switch between members of this exclusive group.
    int32_t defined_config_keys(const char **keys) const noexcept override {
        if (keys!=nullptr) keys[0]="bridge_volume_compensation_enabled";
        return 1;
    }
    // Register only the flow switch; the host generates the model selector.
    void inilialize_impl(storage_handle *) const override;
    // Check bridge contacts and adjust copied infill trees, then replace all of
    // this object's affected trees together. Errors and cancellation stop the run.
    void run_impl(const plugin_run_context *run_ctx) const override;
};

void Finalizer::register_group() const
{
    raw_exclusive_group_def definition=raw_exclusive_group_def_init();
    definition.exclusive_group="bridge_volume";
    definition.option_key="bridge_volume_compensation";
    definition.label="Bridge volume model";
    definition.tooltip="Selects the active plugin used to adjust bridge volume. The experimental model uses actual extrusion footprints.";
    definition.container_type=RAW_CONTAINER_TYPE_PROJECT;
    definition.category=RAW_OPTION_CATEGORY_INFILL;
    definition.has_invalidates_step=1;definition.invalidates_step=STEP_FINALIZE_SLICING;
    definition.ui_page="Width & Flow";definition.ui_group="Flow";
    register_exclusive_group(STEP_FINALIZE_SLICING,definition);
}

uint64_t mix(uint64_t key,uint64_t value) { return (key^value)*1099511628211ull; }

std::vector<BridgeClearance::Occupation> occupied_segments(const c_extrusion_segment &segment,
    const EPropertyAttributes &attributes,double z,double offset)
{
    const c_vec2d a{unscaled(segment.point_a.x),unscaled(segment.point_a.y)},b{unscaled(segment.point_b.x),unscaled(segment.point_b.y)};
    // The whole footprint uses the lowest bottom and highest top of the segment,
    // conservatively bounding material with varying per-point Z offsets.
    const double low=z+offset+unscaled(std::min(segment.z_offset_a,segment.z_offset_b))-attributes.c_extrusion_property_attributes::height;
    const double high=z+offset+unscaled(std::max(segment.z_offset_a,segment.z_offset_b));
    const double width=attributes.c_extrusion_property_attributes::width;
    if (segment.radius==0.) return {{a,b,width*.5,low,high,false}};
    const double radius=std::abs(unscaled(double(segment.radius))),chord=std::hypot(b.x-a.x,b.y-a.y);
    if (radius<=0. || chord<=0. || chord>2.*radius+1e-5)
        return {{a,b,width*.5+radius,low,high,true}};
    const double side=(segment.orientation==RAW_EXTRUSION_ARC_ORIENTATION_CCW ? 1. : -1.)*(segment.radius<0. ? -1. : 1.);
    const double h=std::sqrt(std::max(0.,radius*radius-chord*chord*.25));
    const c_vec2d center{(a.x+b.x)*.5-side*(b.y-a.y)/chord*h,(a.y+b.y)*.5+side*(b.x-a.x)/chord*h};
    const double start=std::atan2(a.y-center.y,a.x-center.x),finish=std::atan2(b.y-center.y,b.x-center.x);
    double sweep=finish-start;
    if (segment.orientation==RAW_EXTRUSION_ARC_ORIENTATION_CCW) { while(sweep<=0.) sweep+=2.*PI; }
    else { while(sweep>=0.) sweep-=2.*PI; }
    const double error=.001;
    // Enlarge every chord by its approximation error so flattening cannot miss
    // an obstacle touched by the original circular footprint.
    const double step=2.*std::acos(std::clamp(1.-error/radius,-1.,1.));
    const size_t count=std::max<size_t>(1,size_t(std::ceil(std::abs(sweep)/std::max(1e-6,step))));
    if (count>1000000) throw std::runtime_error("An occupied extrusion arc exceeds the bridge footprint limit.");
    std::vector<BridgeClearance::Occupation> result;result.reserve(count);
    c_vec2d previous=a;
    for (size_t idx=1; idx<=count; ++idx) {
        const double angle=start+sweep*double(idx)/count;
        const c_vec2d next=idx==count ? b : c_vec2d{center.x+radius*std::cos(angle),center.y+radius*std::sin(angle)};
        result.push_back({previous,next,width*.5+error,low,high,false});previous=next;
    }
    return result;
}

void OccupationVisitor::visit_leaf(ExtrusionEntity leaf)
{
    throw_if_cancelled(ctx);
    const EPropertySpecialCommand *command=current_property(EPropertySpecialCommand::key);
    // A positive stationary deposit has no capsule-volume model in this pass.
    if (command!=nullptr && command->code==C_EXTRUSION_SPECIAL_COMMAND_DEPOSIT && command->extra_data>0.) { unknown=true;return; }
    const EPropertyAttributes *attr=current_property(EPropertyAttributes::key);
    if (leaf.segment_count()==0) return;
    if (attr==nullptr) {
        unknown=true;
        return;
    }
    if (attr->mm3_per_mm()<=0.) return;
    if (!std::isfinite(attr->mm3_per_mm()) || !std::isfinite(attr->c_extrusion_property_attributes::width) ||
        !std::isfinite(attr->c_extrusion_property_attributes::height) ||
        !(attr->c_extrusion_property_attributes::width>0.) || !(attr->c_extrusion_property_attributes::height>0.))
        throw std::runtime_error("An occupied extrusion has invalid physical attributes.");
    const EPropertyZOffset *offset=current_property(EPropertyZOffset::key);
    const double offset_mm=offset==nullptr ? 0. : unscaled(offset->z_offset);
    signature=mix(signature,path_signature(leaf.handle()));
    uint64_t bits=0;const double amount=attr->mm3_per_mm();std::memcpy(&bits,&amount,sizeof(bits));signature=mix(signature,bits);
    for (const double value : {double(attr->c_extrusion_property_attributes::width),double(attr->c_extrusion_property_attributes::height),offset_mm}) {
        std::memcpy(&bits,&value,sizeof(bits));
        signature=mix(signature,bits);
    }
    signature=mix(signature,attr->c_extrusion_property_attributes::role);
    for (const c_extrusion_segment &segment : leaf.segments())
        for (const BridgeClearance::Occupation &o : occupied_segments(segment,*attr,z,offset_mm)) index.add(o);
}

void index_layer(const Layer &layer,BridgeClearance::Index &index,uint64_t &signature,
    const std::map<const extrusion_entity_handle *,size_t> &clones,const std::vector<Bucket> &buckets,
    bool &unknown,const plugin_run_context *run_ctx)
{
    signature=mix(signature,uint64_t(layer.print_z()));
    OccupationVisitor visitor(index,signature,unscaled(layer.print_z()),unknown,run_ctx);
    for (uint32_t island_idx=0; island_idx<layer.island_count(); ++island_idx)
        for (uint32_t region_idx=0; region_idx<layer.island(island_idx).region_island_count(); ++region_idx) {
            const LayerRegionIsland region=layer.island(island_idx).region_island(region_idx);
            for (uint32_t bucket=0; bucket<layer_region_island_extrusion_count(region.handle()); ++bucket) {
                const raw_extrusion_role role=layer_region_island_extrusion_role_at(region.handle(),bucket);
                const extrusion_entity_handle *root=region.extrusion(role);
                if (root==nullptr) continue;
                const std::map<const extrusion_entity_handle *,size_t>::const_iterator replacement=clones.find(root);
                visitor.traverse(replacement==clones.end() ? ExtrusionEntity(root) : buckets[replacement->second].output.readonly());
            }
        }
}

void CorrectionVisitor::visit_leaf(MutableExtrusionEntity leaf)
{
    const EPropertyInfill *infill=current_property(EPropertyInfill::key);
    if (infill==nullptr) return;
    const std::map<uint64_t,SurfaceWork>::const_iterator found=surfaces.find(infill->source_surface_id);
    if (found==surfaces.end()) return;
    const EPropertyAttributes *attr=current_property(EPropertyAttributes::key);
    if (attr==nullptr) throw std::runtime_error("A prepared bridge leaf has no physical attributes.");
    const bool internal=found->second.surface.get(BridgeSurfaceInfo::key)->core_rule==2;
    if (attr->c_extrusion_property_attributes::role!=(internal ? RAW_EXTRUSION_ROLE_INTERNAL_BRIDGE_INFILL : RAW_EXTRUSION_ROLE_BRIDGE_INFILL))
        throw std::runtime_error("A prepared bridge lost its bridge-infill extrusion role.");
    const EPropertyZOffset *offset=current_property(EPropertyZOffset::key);
    if (offset!=nullptr && offset->z_offset!=0)
        throw std::runtime_error("A prepared bridge has a nonzero inherited Z offset.");
    const EPropertyAttributes attributes=*attr;
    try {
        correct_leaf(leaf,attributes,found->second,index,z,print_config,context,signature,unknown,diagnostics,ctx,*infill);
    } catch (const PluginCancelled &) {
        throw;
    } catch (const std::exception &error) {
        const BridgeSurfaceInfo &info=*found->second.surface.get(BridgeSurfaceInfo::key);
        throw std::runtime_error("Object '"+Object(context.object).name()+"', bridge "+std::to_string(info.logical_key)+
            ", Z="+std::to_string(z)+" mm: "+error.what());
    }
}

void correct_leaf(MutableExtrusionEntity leaf,const EPropertyAttributes &attributes,const SurfaceWork &work,
    const BridgeClearance::Index &index,double z,const Config &print_config,
    const run_ctx_finalize_slicing &ctx,uint64_t signature,bool unknown,
    Diagnostics &diagnostics,const plugin_run_context *run_ctx,const EPropertyInfill &recipe)
{
    throw_if_cancelled(run_ctx);
    if (leaf.segment_count()==0) return;
    const double width=attributes.c_extrusion_property_attributes::width,height=attributes.c_extrusion_property_attributes::height;
    if (!(width>0.) || !(height>0.) || !std::isfinite(width) || !std::isfinite(height) ||
        !(attributes.mm3_per_mm()>0.) || !std::isfinite(attributes.mm3_per_mm()))
        throw std::runtime_error("A bridge has invalid flow dimensions.");
    const BridgeSurfaceInfo &info=*work.surface.get(BridgeSurfaceInfo::key);
    signature=mix(signature,work.input_signature);
    // BridgeVolumeInfo means this path has already been corrected. If its inputs
    // changed, rebuild the original path first; do not reduce its flow a second time.
    if (const BridgeVolumeInfo *stamp=leaf.get(BridgeVolumeInfo::key)) {
        if (stamp->signature!=signature || stamp->geometry_signature!=path_signature(leaf.handle()) ||
            std::abs(stamp->effective_area_mm2-attributes.mm3_per_mm())>1e-8 || std::abs(stamp->effective_height_mm-height)>1e-7)
            throw std::runtime_error("A finalized bridge changed; regenerate nominal infill before applying compensation again.");
        return;
    }
    const c_point start=leaf.front(),finish=leaf.back();
    const double length=unscaled(norm(finish-start));
    if (length<=0.) throw std::runtime_error("A prepared bridge strand has zero length.");
    const double ux=(finish.x-start.x)/scale_d(length),uy=(finish.y-start.y)/scale_d(length);
    const double angle=work.surface.bridge_angle();
    if (!std::isfinite(angle) || angle<0.) throw std::runtime_error("A prepared bridge lost its canonical direction.");
    const coord_t fill_resolution=std::min(scale_i(width)/16,
        std::max(SCALED_EPSILON,scale_i(print_config.get("resolution_internal").get_float())));
    const double tolerance=unscaled(std::max(4*SCALED_EPSILON,fill_resolution));
    double previous=-tolerance;
    for (const c_extrusion_segment &segment : leaf.segments()) {
        const double projection=unscaled(double(segment.point_b.x-start.x)*ux+double(segment.point_b.y-start.y)*uy);
        const double across=unscaled(std::abs(double(segment.point_b.x-start.x)*uy-double(segment.point_b.y-start.y)*ux));
        if (segment.radius!=0. || segment.z_offset_a!=0 || segment.z_offset_b!=0 || across>tolerance || projection<previous-tolerance ||
            std::abs(ux*std::sin(angle)-uy*std::cos(angle))>1e-6)
            throw std::runtime_error("A prepared bridge generated a non-axial, non-planar or returning strand.");
        previous=projection;
    }
    StoredPolyline line(run_ctx->plugin_storage);line.push_back(start);line.push_back(finish);
    StoredExPolygonCollection prepared(run_ctx->plugin_storage);prepared.push_back(work.surface.expolygon());
    ClipperContext clipper(run_ctx->plugin_storage);
    prepared=clipper_offset(clipper(prepared.readonly()),double(SCALED_EPSILON)).to_expolygon_collection();
    const StoredPolylineCollection outside=clipper_diff_polyline_expolygons(run_ctx->plugin_storage,line.readonly(),prepared.readonly());
    for (const Polyline part : outside.readonly())
        if (part.length()>SCALED_EPSILON) throw std::runtime_error("A bridge row lies outside its prepared surface.");
    StoredPolylineCollection inside=clipper_intersection_polyline_expolygons(run_ctx->plugin_storage,line.readonly(),work.core.readonly());
    double core_length=0.,first_core=length,last_core=0.;
    for (const Polyline part : inside) {
        core_length+=unscaled(part.length());
        for (const c_point point : part.points()) {
            const double position=unscaled(double(point.x-start.x)*ux+double(point.y-start.y)*uy);
            first_core=std::min(first_core,position);last_core=std::max(last_core,position);
        }
    }
    const bool peripheral=core_length<=tolerance;
    if (peripheral) {
        if (!work.peripheral_rows) throw std::runtime_error("A generated bridge row does not traverse its original core.");
        const StoredPolylineCollection through_uniform=clipper_intersection_polyline_expolygons(run_ctx->plugin_storage,line.readonly(),work.uniform_core.readonly());
        double uniform_length=0.;
        for (const Polyline part : through_uniform.readonly()) uniform_length+=unscaled(part.length());
        if (uniform_length<=tolerance)
            throw std::runtime_error("A peripheral bridge row does not traverse its uniformly expanded core.");
    }
    const c_vec2d a{unscaled(start.x),unscaled(start.y)},b{unscaled(finish.x),unscaled(finish.y)};
    std::vector<BridgeClearance::Interval> intervals=index.clearance(a,b,width,height,z);
    const double nominal=BridgeClearance::area(width,width);
    const bool modeled=std::abs(width-height)<2e-6 && std::abs(nominal-attributes.mm3_per_mm())<=std::max(1e-8,nominal*1e-5);
    // Flow dimensions are floats, whereas layer Z is exact on its coordinate grid.
    // A rounding difference alone must not trigger flow changes on a thread that fits.
    const double height_roundoff=double(std::nextafter(float(height),std::numeric_limits<float>::infinity()))-height;
    if (modeled)
        for (BridgeClearance::Interval &interval : intervals)
            if (interval.clearance<height && interval.clearance>=height-height_roundoff)
                interval.clearance=height;
    if (unknown || !modeled)
        for (BridgeClearance::Interval &interval : intervals) interval.unknown=true;
    bool contacts_unknown=unknown;
    for (const BridgeClearance::Interval &interval : intervals)
        if (interval.unknown && (peripheral || interval.begin<first_core || interval.end>last_core)) contacts_unknown=true;
    if (contacts_unknown || !ctx.support_demand_computed) diagnostics.unknown=true;
    else {
        const std::vector<std::pair<double,double>> contacts=index.contacts(a,b,height,z);
        std::vector<std::pair<double,double>> gaps;
        if (peripheral || info.core_rule==2) {
            // Lower slices authorize this row, but sparse infill may leave real
            // gaps. A fully contacted row needs no separate end anchors.
            double position=0.;
            for (const std::pair<double,double> &contact : contacts) {
                if (contact.first>position+tolerance) gaps.emplace_back(position,contact.first);
                position=std::max(position,contact.second);
            }
            if (length>position+tolerance) gaps.emplace_back(position,length);
        } else gaps.emplace_back(first_core,last_core);
        // Check peripheral gaps independently: a row supported in the middle
        // has two one-sided tails, not a whole unsupported span with no anchors.
        for (const std::pair<double,double> &gap : gaps) {
            double left=0.,right=0.;
            for (const std::pair<double,double> &contact : contacts) {
                left+=std::max(0.,std::min(contact.second,gap.first)-std::max(0.,contact.first));
                right+=std::max(0.,std::min(contact.second,length)-std::max(gap.second,contact.first));
            }
            const int count=int(left>=tolerance)+int(right>=tolerance);
            if (count==2) continue;
            // Suppress missing-anchor diagnostics only on the area covered by a
            // local support request. The request itself does not count as lower material.
            ClipperContext clipper(run_ctx->plugin_storage);
            StoredExPolygonCollection deficient(run_ctx->plugin_storage);
            if (peripheral || info.core_rule==2) {
                StoredPolyline unsupported(run_ctx->plugin_storage);
                for (const double position : {gap.first,gap.second})
                    unsupported.push_back({start.x+scale_i(position*ux),start.y+scale_i(position*uy)});
                deficient=clipper_offset(clipper(unsupported.readonly()),scale_d(width*.5),CLIPPER_JOIN_ROUND,3.,CLIPPER_END_OPEN_ROUND).to_expolygon_collection();
                StoredExPolygonCollection prepared(run_ctx->plugin_storage);prepared.push_back(work.surface.expolygon());
                deficient=intersect(run_ctx->plugin_storage,deficient.readonly(),prepared.readonly());
            } else {
                StoredExPolygonCollection footprint=clipper_offset(clipper(line.readonly()),scale_d(width*.5),CLIPPER_JOIN_ROUND,3.,CLIPPER_END_OPEN_ROUND).to_expolygon_collection();
                deficient=intersect(run_ctx->plugin_storage,footprint.readonly(),work.core.readonly());
            }
            const ExPolygonCollection requested(ctx.get_requested_support_areas(work.island.handle()));
            deficient=subtract(run_ctx->plugin_storage,deficient.readonly(),requested);
            if (!deficient.empty()) {
                if (count==0 && info.extension_requested)
                    throw std::runtime_error(peripheral ? "A peripheral bridge gap has no modeled anchor and no local support request." :
                        "A prepared bridge has no modeled anchor and no local support request.");
                if (count==0) diagnostics.unanchored=true; else diagnostics.one_sided=true;
            }
        }
    }
    if (!work.enabled) return;
    const double floor=std::min(width,work.minimum_height);
    std::vector<BridgeClearance::SectionPiece> pieces=BridgeClearance::section_pieces(intervals,width,floor);
    for (BridgeClearance::SectionPiece &piece : pieces) {
        if (piece.confidence==BRIDGE_CONFIDENCE_UNKNOWN) {
            piece.area=attributes.mm3_per_mm();piece.height=height;diagnostics.unknown=true;
        } else if (piece.area==nominal) {
            // Native bridge flow rounds its area to float. An unconstrained
            // portion keeps that exact input rather than gaining material.
            piece.area=attributes.mm3_per_mm();piece.height=height;
        } else if (piece.area>attributes.mm3_per_mm()) {
            piece.area=attributes.mm3_per_mm();
            piece.height=BridgeClearance::height_for_area(width,piece.area);
        }
        if (piece.confidence==BRIDGE_CONFIDENCE_FLOOR_LIMITED) diagnostics.floor=true;
    }
    // Adjacent parts needing the same flow can print as one piece even when
    // different obstacles limit them. Keep the smaller allowed area when joining them.
    std::vector<BridgeClearance::SectionPiece> effective;
    for (const BridgeClearance::SectionPiece &piece : pieces) {
        if (!effective.empty()) {
            BridgeClearance::SectionPiece &previous=effective.back();
            if (previous.end==piece.begin && previous.area==piece.area && previous.height==piece.height &&
                previous.confidence==piece.confidence) {
                previous.end=piece.end;
                previous.clearance=std::min(previous.clearance,piece.clearance);
                previous.cap=std::min(previous.cap,piece.cap);
                continue;
            }
        }
        effective.push_back(piece);
    }
    effective=BridgeClearance::coalesce_short_sections(effective,2.*unscaled(SCALED_EPSILON));
    publish_pieces(leaf,attributes,effective,info.logical_key,signature,run_ctx);
}

struct Fragment { extrusion_entity_handle *entity; uint32_t area; };
struct Collector { std::vector<Fragment> fragments; size_t count=0; bool overflow=false; };
// Borrow fragment handles from the private leaf during the synchronous C split.
// Ownership stays with that leaf. The noexcept callback never allocates; overflow
// sets a flag for its caller to reject after the C call returns.
void collect_fragment(extrusion_entity_handle *entity,uint32_t area,void *user) noexcept;
void collect_fragment(extrusion_entity_handle *entity,uint32_t area,void *user) noexcept
{
    Collector &collector=*static_cast<Collector *>(user);
    if (collector.count==collector.fragments.size()) { collector.overflow=true;return; }
    collector.fragments[collector.count++]={entity,area};
}

void publish_pieces(MutableExtrusionEntity leaf,const EPropertyAttributes &attributes,
    const std::vector<BridgeClearance::SectionPiece> &pieces,uint64_t logical_key,uint64_t signature,
    const plugin_run_context *run_ctx)
{
    if (pieces.empty()) throw std::runtime_error("A bridge correction has no covering pieces.");
    const c_point start=leaf.front(),finish=leaf.back();
    const double length=unscaled(norm(finish-start)),ux=unscaled(finish.x-start.x)/length,uy=unscaled(finish.y-start.y)/length;
    const double across=std::max(1.,double(attributes.c_extrusion_property_attributes::width));
    std::vector<StoredExPolygonCollection> areas;areas.reserve(pieces.size());
    std::vector<const expolygon_collection_handle *> handles;handles.reserve(pieces.size());
    // Strips straddle the straight centerline. The first and last extend beyond
    // its endpoints so integer clipping cannot shorten the deposited strand.
    for (size_t idx=0; idx<pieces.size(); ++idx) {
        const double begin=idx==0 ? -1. : pieces[idx].begin,end=idx+1==pieces.size() ? length+1. : pieces[idx].end;
        if (end-begin<=2.*unscaled(SCALED_EPSILON)) {
            std::ostringstream message;message.precision(17);
            message<<"A bridge clearance interval is smaller than the extrusion geometry precision. Row "
                <<unscaled(start.x)<<","<<unscaled(start.y)<<" -> "<<unscaled(finish.x)<<","<<unscaled(finish.y)<<"; pieces:";
            for (size_t nearby=idx-1; nearby<=idx+1 && nearby<pieces.size(); ++nearby) {
                const BridgeClearance::SectionPiece &piece=pieces[nearby];
                message<<" ["<<piece.begin<<","<<piece.end<<" area="<<piece.area<<" height="<<piece.height
                    <<" cap="<<piece.cap<<" confidence="<<int(piece.confidence)<<"]";
            }
            throw std::runtime_error(message.str());
        }
        StoredExPolygon polygon(run_ctx->plugin_storage);
        multipoint_handle *ring=polygon_as_multipoint(expolygon_contour(polygon.mutable_handle()));
        for (const c_vec2d p : {c_vec2d{begin,-across},c_vec2d{end,-across},c_vec2d{end,across},c_vec2d{begin,across}})
            multipoint_push_back(ring,{start.x+scale_i(ux*p.x-uy*p.y),start.y+scale_i(uy*p.x+ux*p.y)});
        areas.emplace_back(run_ctx->plugin_storage);areas.back().push_back_move(std::move(polygon));
        handles.push_back(areas.back().handle());
    }
    leaf.get_or_add(EPropertyAttributes::key)=attributes;
    // Reserve callback memory before entering C; unexpected fragment counts are
    // reported after the call instead of allocating or throwing through its callback.
    Collector collector;collector.fragments.resize(pieces.size()+1);
    const raw_extrusion_split_status status=extrusion_split_leaf_by_areas(leaf.mutable_handle(),handles.data(),uint32_t(handles.size()),
        SCALED_EPSILON,&collect_fragment,&collector);
    if (status!=RAW_EXTRUSION_SPLIT_STATUS_SUCCESS || collector.overflow || collector.count==0)
        throw std::runtime_error("The private bridge strand could not be split into its clearance intervals.");
    c_point previous=start;
    for (size_t idx=0; idx<collector.count; ++idx) {
        MutableExtrusionEntity fragment(collector.fragments[idx].entity);
        // Independent native cuts may round a shared endpoint differently.
        // Reuse one point at each join, but never bridge a real clipping gap.
        if (norm(fragment.front()-previous)>SCALED_EPSILON || !fragment.set_point(0,previous))
            throw std::runtime_error("A split bridge strand lost continuity.");
        if (idx+1==collector.count) {
            if (norm(fragment.back()-finish)>SCALED_EPSILON || !fragment.set_point(fragment.point_count()-1,finish))
                throw std::runtime_error("A split bridge strand lost its final endpoint.");
        }
        previous=fragment.back();
        const BridgeClearance::SectionPiece &piece=pieces.at(collector.fragments[idx].area);
        EPropertyAttributes &effective=fragment.get_or_add(EPropertyAttributes::key);
        effective=attributes;effective.mm3_per_mm(piece.area).height(float(piece.height));
        BridgeVolumeInfo &stamp=fragment.get_or_add(BridgeVolumeInfo::key);
        stamp={};stamp.logical_key=logical_key;stamp.signature=signature;stamp.geometry_signature=path_signature(fragment.handle());
        stamp.nominal=attributes;stamp.available_height_mm=piece.clearance;stamp.area_cap_mm2=piece.cap;
        stamp.effective_area_mm2=piece.area;stamp.effective_height_mm=float(piece.height);
        stamp.confidence=piece.confidence;
        stamp.correction_state=piece.confidence==BRIDGE_CONFIDENCE_UNKNOWN ? BRIDGE_CORRECTION_KEPT_NOMINAL_UNKNOWN : BRIDGE_CORRECTION_APPLIED;
    }
    if (leaf.child_count()>0) {
        // Record corrected flow on each piece. The containing collection only
        // tells ordering to keep those pieces connected and reverse them together.
        leaf.remove(BridgeVolumeInfo::key);
        leaf.disable_sort();leaf.get_or_add(BridgeAtomicStrand::key).atomic_orientation=1;
    }
}

int32_t Finalizer::used_config_keys(raw_used_config_key *keys) const noexcept
{
    const raw_used_config_key values[]={
        {"bridge_volume_compensation_enabled",RAW_CO_BOOL,RAW_CONTAINER_TYPE_REGION,RAW_PRESET_TYPE_FFF_PRINT},
        {"bridged_infill_margin",RAW_CO_FLOAT_OR_PERCENT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
        {"bridged_infill_uniform_margin",RAW_CO_FLOAT_OR_PERCENT,RAW_CONTAINER_TYPE_REGION,RAW_PRESET_TYPE_FFF_PRINT},
        {"fill_density",RAW_CO_PERCENT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
        {"perimeters",RAW_CO_INT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
        {"min_layer_height",RAW_CO_VECTOR_FLOAT_OR_PERCENT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE},
        {"resolution_internal",RAW_CO_FLOAT,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE}
    };
    if (keys!=nullptr) std::copy(std::begin(values),std::end(values),keys);
    return int32_t(std::size(values));
}
void Finalizer::inilialize_impl(storage_handle *) const
{
    raw_config_option_def def=raw_config_option_def_init();
    def.opt_key="bridge_volume_compensation_enabled";def.type=RAW_CO_BOOL;
    def.container_type=RAW_CONTAINER_TYPE_REGION;def.option_preset_type=RAW_PRESET_TYPE_FFF_PRINT;def.printer_technology=RAW_PT_FFF;
    def.category=RAW_OPTION_CATEGORY_INFILL;def.invalidates_step=STEP_FINALIZE_SLICING;
    def.mode=RAW_CONFIG_OPTION_MODE_EXPERT|RAW_CONFIG_OPTION_MODE_SUSI;
    def.label="Bridge volume compensation";
    def.tooltip="Adjust bridge deposition using the selected bridge volume model. Disable to retain nominal flow while keeping prepared-strand checks.";
    def.default_serialized_value="1";
    if (orchestrator_create_option_def(m_orchestrator,&def)!=OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register the bridge volume compensation switch.");
    orchestrator_add_ui_fragment(m_orchestrator,"print.ui","bridge.volume.enabled",
        "page:Width & Flow\n"
        "group:Flow\n"
        "line:insert$afterline$Bridge type:Bridge volume compensation\n"
        "setting:bridge_volume_compensation_enabled\n"
        "end_line\n",1);
}
void Finalizer::run_impl(const plugin_run_context *run_ctx) const
{
    const run_ctx_finalize_slicing *ctx=plugin_ctx_as_finalize_slicing(run_ctx);
    if (ctx==nullptr || ctx->replace_infill_buckets==nullptr || ctx->get_requested_support_areas==nullptr)
        throw std::runtime_error("Bridge finalization requires support-request access and transactional infill publication.");
    const Object object(ctx->object);const Config print_config=Print(ctx->print).config();
    std::map<uint64_t,SurfaceWork> surfaces;
    std::vector<Bucket> buckets;
    std::map<const extrusion_entity_handle *,size_t> clones;
    // Remember prepared geometry and settings before changing flow. External bridges
    // reconstruct their unsupported core; internal bridges use the whole prepared area.
    for (uint32_t layer_idx=0; layer_idx<object.layer_count(); ++layer_idx) {
        const Layer layer=object.layer(layer_idx);
        for (uint32_t island_idx=0; island_idx<layer.island_count(); ++island_idx) {
            const LayerIsland island=layer.island(island_idx);
            StoredExPolygonCollection lower(run_ctx->plugin_storage);
            for (const LayerIsland linked : island.lower_islands()) lower.push_back(linked.slice());
            lower=unite(run_ctx->plugin_storage,lower.readonly());
            bool affected=false;
            for (uint32_t region_idx=0; region_idx<island.region_island_count(); ++region_idx) {
                const LayerRegionIsland region=island.region_island(region_idx);
                for (const Surface surface : region.fill_surfaces_collection()) {
                    const BridgeSurfaceInfo *info=surface.get(BridgeSurfaceInfo::key);
                    if (info==nullptr) continue;
                    if (info->schema_version!=1 || (info->core_rule!=1 && info->core_rule!=2))
                        throw std::runtime_error("A prepared bridge has an unsupported provenance rule.");
                    if (region.region_count()!=1) throw std::runtime_error("A prepared bridge lost its single-region ownership.");
                    const LayerRegion owner=region.region(0);
                    const Config config=owner.print_region().config();
                    StoredExPolygonCollection prepared(run_ctx->plugin_storage);prepared.push_back(surface.expolygon());
                    StoredExPolygonCollection core=info->core_rule==2 ? prepared.readonly().clone(run_ctx->plugin_storage) :
                        subtract(run_ctx->plugin_storage,prepared.readonly(),lower.readonly());
                    if (core.empty()) throw std::runtime_error("A prepared bridge fragment lost its original core.");
                    uint64_t input=mix(info->logical_key,info->policy_signature);
                    for (const char *id=id_impl();*id!='\0';++id) input=mix(input,uint8_t(*id));
                    input=mix(mix(input,geometry_key(core.readonly())),geometry_key(prepared.readonly()));
                    input=mix(input,ctx->support_demand_computed);
                    if (ctx->support_demand_computed)
                        input=mix(input,geometry_key(ExPolygonCollection(ctx->get_requested_support_areas(island.handle()))));
                    const double angle=surface.bridge_angle();
                    uint64_t bits=0;std::memcpy(&bits,&angle,sizeof(bits));input=mix(input,bits);
                    for (const char *key : {"min_layer_height","resolution_internal","bridge_volume_compensation_enabled","bridged_infill_uniform_margin"}) {
                        const bool regional=std::strcmp(key,"bridge_volume_compensation_enabled")==0 || std::strcmp(key,"bridged_infill_uniform_margin")==0;
                        const std::string serialized=(regional ? config : print_config).get(key).serialize();
                        for (unsigned char c : serialized) input=mix(input,c);
                    }
                    const extruder_index_t tool=layer_region_get_role_extruder(owner.handle(),RAW_EXTRUSION_ROLE_SOLID_INFILL);
                    const c_flow_mm flow=layer_region_get_infill_bridge_flow_mm(owner.handle(),RAW_EXTRUSION_ROLE_SOLID_INFILL);
                    if (tool==EXTRUDER_ID_UNKNOWN || !(flow.nozzle_diameter>0.)) throw std::runtime_error("A bridge has no resolved reference nozzle.");
                    const double minimum=print_config.get("min_layer_height").get_effective_value(flow.nozzle_diameter,uint32_t(tool));
                    const double layer_height=unscaled(layer.height());
                    const double floor=std::min(layer_height,minimum>0. ? minimum : layer_height);
                    const bool enabled=config.get("bridge_volume_compensation_enabled").get_bool();
                    const BridgePreparation::ExpansionMargins margins=info->core_rule==2 ? BridgePreparation::ExpansionMargins{0,0} :
                        BridgePreparation::expansion_margins(owner.handle());
                    input=mix(mix(input,uint64_t(margins.axial)),uint64_t(margins.uniform));
                    StoredExPolygonCollection uniform_core=expand_bridge(run_ctx->plugin_storage,core.readonly(),prepared.readonly(),angle,0,margins.uniform,run_ctx);
                    surfaces.emplace(surface.id(),SurfaceWork{surface,island,owner,std::move(core),std::move(uniform_core),input,floor,enabled,margins.uniform>0,
                        layer_height,unscaled(layer.print_z()-layer.height())});affected=true;
                }
            }
            if (affected)
                for (uint32_t region_idx=0; region_idx<island.region_island_count(); ++region_idx) {
                    const LayerRegionIsland region=island.region_island(region_idx);
                    for (uint32_t bucket=0; bucket<layer_region_island_extrusion_count(region.handle()); ++bucket) {
                        const raw_extrusion_role role=layer_region_island_extrusion_role_at(region.handle(),bucket);
                        if ((role&RAW_EXTRUSION_ROLE_INFILL)==0 && role!=RAW_EXTRUSION_ROLE_GAP_FILL) continue;
                        const extrusion_entity_handle *root=region.extrusion(role);
                        if (root!=nullptr) { clones.emplace(root,buckets.size());buckets.push_back({root,layer_idx,ExtrusionEntity(root).clone(run_ctx->plugin_storage)}); }
                    }
                }
        }
    }
    if (surfaces.empty()) return;
    BridgeClearance::Index index(run_ctx);
    Diagnostics diagnostics;
    // Missing support paths may mean generation has not finished, not empty space.
    // Only the completion flag lets an empty support result be used as known input.
    bool unknown=!ctx->support_material_generated;
    uint64_t signature=mix(1469598103934665603ull,ctx->support_material_generated);
    std::vector<Layer> auxiliary;
    for (uint32_t idx=0; idx<object.auxiliary_layer_count(); ++idx) {
        const Layer layer=object.auxiliary_layer(idx);
        if (layer.properties().has(LayerSupportProperty::key)) auxiliary.push_back(layer);
    }
    std::stable_sort(auxiliary.begin(),auxiliary.end(),[](const Layer &a,const Layer &b) { return a.print_z()<b.print_z(); });
    size_t next_auxiliary=0;
    // Do not count the bridge's own layer as material below it. Support layers
    // use their own Z heights, which may lie between the object's layer heights.
    for (uint32_t idx=0; idx<object.layer_count(); ++idx) {
        throw_if_cancelled(run_ctx);
        const Layer layer=object.layer(idx);
        while (next_auxiliary<auxiliary.size() && auxiliary[next_auxiliary].print_z()<layer.print_z())
            index_layer(auxiliary[next_auxiliary++],index,signature,clones,buckets,unknown,run_ctx);
        for (Bucket &bucket : buckets)
            if (bucket.layer_idx==idx) {
                CorrectionVisitor visitor(surfaces,index,unscaled(layer.print_z()),print_config,*ctx,signature,unknown,diagnostics,run_ctx);
                visitor.traverse(bucket.output.mutable_view());
            }
        index_layer(layer,index,signature,clones,buckets,unknown,run_ctx);
    }
    std::vector<raw_final_infill_replacement> replacements;
    for (Bucket &bucket : buckets) replacements.push_back({bucket.source,bucket.output.mutable_handle()});
    // Finish allocation and cancellation checks before the host validates and
    // swaps the complete batch; no partially corrected live root is exposed.
    throw_if_cancelled(run_ctx);
    if (!ctx->replace_infill_buckets(ctx,run_ctx->plugin_storage,replacements.data(),uint32_t(replacements.size())))
        throw std::runtime_error("The final bridge infill transaction was refused.");
    if (run_ctx->report_warning!=nullptr) {
        const std::string prefix="Object '"+object.name()+"': ";
        if (diagnostics.unknown) run_ctx->report_warning(run_ctx->host_context,(prefix+"bridge compensation is incomplete; unknown portions retain nominal flow.").c_str());
        if (diagnostics.floor) run_ctx->report_warning(run_ctx->host_context,(prefix+("the printable bridge height floor leaves a local geometric clearance limit unsatisfied.")).c_str());
        if (diagnostics.one_sided || diagnostics.unanchored) run_ctx->report_warning(run_ctx->host_context,(prefix+"bridge portions without a local support request have missing modeled anchors.").c_str());
    }
}
}
uint64_t path_signature(const extrusion_entity_handle *entity)
{
    uint64_t key=1469598103934665603ull;
    for (const c_extrusion_segment &segment : ExtrusionEntity(entity).segments()) {
        key=mix(mix(key,uint64_t(segment.point_a.x)),uint64_t(segment.point_a.y));
        key=mix(mix(key,uint64_t(segment.point_b.x)),uint64_t(segment.point_b.y));
        key=mix(mix(key,uint64_t(segment.z_offset_a)),uint64_t(segment.z_offset_b));
        uint32_t radius=0;
        std::memcpy(&radius,&segment.radius,sizeof(radius));
        key=mix(mix(key,radius),segment.orientation);
    }
    return key;
}
void register_plugin(orchestrator_handle *orch)
{
    static Finalizer plugin(orch);
    orchestrator_register_plugin(orch,plugin.c_instance());plugin.register_group();
}
}}
