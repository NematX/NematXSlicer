/*
 * Copyright (c) 2026 NematX
 * Released under the GNU Affero General Public License, version 3 or later.
 *
 * This file is the SDK (the host's public plugin interface) adapter for the
 * corner-stretch plugin.  The companion CornerStretchGeometry.hpp is the
 * small, host-independent geometry kernel:
 * this adapter reads slicer trees and settings, converts at the boundary, and
 * writes the kernel's result back to the SDK tree.  Slicer points are scaled
 * integer coordinates, not millimetres; Point is converted to and from those
 * coordinates only at this boundary.  Serialization here means writing the
 * result back as SDK segment records.  Deposited flow q (material volume per
 * path length) is measured in mm3/mm, speed is measured in mm/s, and a
 * compensated fragment's ratio means original path length divided by its new
 * path length.
 *
 * An extrusion is a tree.  Its root may contain collections, and collections
 * eventually contain leaves with segments.  A leaf inherits attributes from
 * its ancestors.  Process data (speed, acceleration, pressure advance, and
 * fan value) is inherited field by field, so one child can change only one
 * process field.  Handles read from the plan are borrowed; replacement trees
 * are temporary storage-backed clones and are published only after they are
 * complete.  An event property records a scope change without path geometry;
 * event and unknown properties stay attached to their parent scope, where
 * they act as barriers to reinterpretation.
 *
 * Registration supplies metadata, inilialize_impl (the binary-interface
 * spelling required by the ABI) defines options, and run_impl edits each layer
 * group.  The layer-extrusion-edit step runs after ordering and speed/flow
 * preparation; priority 5 selects this plugin's position.  Layer callbacks
 * may run in parallel, so each callback mutates only its own group.
 * plugin_storage is shared and this plugin never
 * clears it.  A failure is not a transaction for the whole printing plan:
 * earlier groups or leaves are not promised to roll back.
 *
 * A source region is the layer mask and settings area from which an extrusion
 * was made.  Regional settings use object coordinates.  Printing coordinates
 * are shifted by the instance offset, so the lookup subtracts that shift.
 * Only the original corner vertex selects a region; a deformation may cross a
 * region boundary.  Outside or ambiguous lookups leave that corner unchanged.
 * A source with no active stretch option is skipped before tree traversal.
 *
 * The adapter walks the true loop order, respects hole orientation and role or
 * perimeter flags, and excludes zero-flow, variable-Z, and vase paths.  A
 * seam connection is not treated as a new corner.  Event-only leaves remain
 * as barriers even though they have no geometry.  The kernel finds candidates
 * and transparent collinear branches, makes requests, allocates each branch
 * proportionally without a solver, and reduces the two displacement
 * components.  It tracks unit directions u/v, retained fractions ki/ko,
 * first/last/vertex windows, and owner/state provenance.
 *
 * Reconstruction distributes windows back into the original leaves.  It keeps
 * nominal and compensated pieces distinct, uses native arcs when allowed and
 * straight fallbacks when not, quantizes shared endpoints and refits arc
 * centres, then restores volume by process state before serializing and
 * discretizing.  The final length correction is intentional: it repairs
 * quantization/serialization error rather than applying the corner correction
 * twice.  Only changed leaves are replaced, and all replacement children are
 * prepared in source order before publication.  The upstream wipe is kept on
 * undeformed geometry and loop endpoints remain fixed.  The speed multiplier
 * has a historical x10 ceiling; this is not a machine XY-speed limiter.
 */
#include "CornerStretchGeometry.hpp"
#include <iterator>
#include <map>
#include <optional>
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp"

namespace nematx_corner {
using namespace slic3r_api;

// One registered option: its key, SDK value type, default text, and label.
struct Option { const char *key; raw_config_option_type type; const char *value; const char *label; };
static const Option options[] = {
    {"stretch_corners",RAW_CO_BOOL,"0","Stretch corners"},
    {"stretch_corners_arc",RAW_CO_BOOL,"1","Stretch with arcs"},
    {"stretch_corners_max_angle",RAW_CO_FLOAT,"120","Stretch maximum angle"},
    {"stretch_corners_distance_concave",RAW_CO_FLOAT_OR_PERCENT,"100%","Concave stretch distance"},
    {"stretch_corners_distance_convex",RAW_CO_FLOAT_OR_PERCENT,"100%","Convex stretch distance"},
    {"stretch_corners_distance_first_layer",RAW_CO_FLOAT_OR_PERCENT,"100%","First layer stretch distance"},
    {"stretch_corners_deviation_concave",RAW_CO_FLOAT_OR_PERCENT,"50%","Concave stretch deviation"},
    {"stretch_corners_deviation_convex",RAW_CO_FLOAT_OR_PERCENT,"50%","Convex stretch deviation"},
    {"stretch_corners_deviation_first_layer",RAW_CO_FLOAT_OR_PERCENT,"50%","First layer stretch deviation"},
    {"stretch_corners_entering_section",RAW_CO_PERCENT,"100%","Entering section influence"},
    {"stretch_corners_exiting_section",RAW_CO_PERCENT,"100%","Exiting section influence"},
    {"stretch_corners_inner_perimeters",RAW_CO_PERCENT,"0%","Inner perimeter influence"}
};
static const char *const no_dependencies[]={nullptr};

// Convert one SDK scaled point to the kernel's calculation point.  The value
// is still in scaled slicer units; no millimetre conversion happens here.
inline Point point(c_point p) { return {double(p.x),double(p.y)}; }

// Round a kernel point back to the SDK's integer scaled coordinate at the
// serialization boundary.  The kernel also snaps selected construction and
// shared endpoints explicitly; the remaining calculations use doubles.
inline c_point point(Point p) { return {coord_t(std::llround(p.x)),coord_t(std::llround(p.y))}; }

// Effective state carried while descending the extrusion tree.  Attributes,
// perimeter flags, and Z are inherited as whole values; process values below
// are resolved one field at a time.  Other properties are remembered by type
// so an event or unknown scope change cannot be crossed silently.
struct State {
    std::optional<EPropertyAttributes> attr;
    c_extrusion_property_speed process{-1,-1,-1,-1};
    uint16_t perimeter=0;
    coord_t z=0;
    // Unknown and event properties remain on their original nodes. Their identity
    // prevents moving geometry across a change of scope we cannot interpret.
    std::map<extrusion_property_type,const void*> other;
};

// Merge the current node's properties into inherited state.  Direct attributes
// replace the inherited attribute set, while process fields replace only the
// fields that contain usable values; the returned state belongs to this walk.
State effective(MutableExtrusionEntity node, State s) {
    if(const auto *p=node.get(EPropertyAttributes::key)) s.attr=*p;
    if(const auto *p=node.get(EPropertyPerimeter::key)) s.perimeter=p->perimeter_flags();
    if(const auto *p=node.get(EPropertyZOffset::key)) s.z=p->z_offset;
    if(const auto *p=node.get(EPropertySpeed::key)) {
        if(p->speed_mm_per_s>0) s.process.speed_mm_per_s=p->speed_mm_per_s;
        if(p->accel_mm_per_s2>=0) s.process.accel_mm_per_s2=p->accel_mm_per_s2;
        if(p->pressure_adv>=0) s.process.pressure_adv=p->pressure_adv;
        if(p->fan_speed_percent>=0) s.process.fan_speed_percent=p->fan_speed_percent;
    }
    for(uint32_t i=0;i<node.property_count();++i) {
        auto t=node.property_type_at(i);
        if(t!=EXTRUSION_PROPERTY_TYPE_ATTRIBUTES && t!=EXTRUSION_PROPERTY_TYPE_SPEED &&
           t!=EXTRUSION_PROPERTY_TYPE_PERIMETER && t!=EXTRUSION_PROPERTY_TYPE_Z_OFFSET)
            s.other[t]=node.property_data(t);
    }
    return s;
}

// Compare all state that can change the meaning or ownership of a path.  A
// missing attribute set is never equivalent, so geometry without resolved
// extrusion data cannot become eligible by accident.
bool equivalent(const State&a,const State&b) {
    if(!a.attr || !b.attr) return false;
    const auto &x=static_cast<const c_extrusion_property_attributes&>(*a.attr);
    const auto &y=static_cast<const c_extrusion_property_attributes&>(*b.attr);
    return x.mm3_per_mm==y.mm3_per_mm && x.width==y.width && x.height==y.height && x.role==y.role &&
        x.no_seam==y.no_seam && a.z==b.z && a.perimeter==b.perimeter && a.other==b.other &&
        a.process.speed_mm_per_s==b.process.speed_mm_per_s && a.process.accel_mm_per_s2==b.process.accel_mm_per_s2 &&
        a.process.pressure_adv==b.process.pressure_adv && a.process.fan_speed_percent==b.process.fan_speed_percent;
}

// A borrowed source leaf, its resolved inherited state, and its original
// segments.  The node handle is not owned by this record and is never replaced
// until a complete temporary replacement has been prepared.
struct Leaf { MutableExtrusionEntity node; State state; std::vector<c_extrusion_segment> segments; };

// Decode one SDK segment into a kernel curve.  Straight segments keep a zero
// sweep; arcs calculate a centre and signed sweep from radius/orientation.
// Invalid source arcs throw because silently changing their shape would alter
// the source path.
Curve curve(const c_extrusion_segment&s) {
    Curve c{point(s.point_a),point(s.point_b),{},0};
    if(s.radius!=0) {
        const Point delta=c.b-c.a; const double chord=norm(delta),radius=std::abs(s.radius);
        if(chord==0 || chord>2*radius+1) throw std::runtime_error("Corner stretch: invalid source arc");
        const double side=(s.orientation==RAW_EXTRUSION_ARC_ORIENTATION_CCW?1.:-1.)*(s.radius>0?1.:-1.);
        c.center=(c.a+c.b)*.5+Point{-delta.y,delta.x}*(side*std::sqrt(std::max(0.,radius*radius-chord*chord/4))/chord);
        c.sweep=std::atan2(cross(c.a-c.center,c.b-c.center),dot(c.a-c.center,c.b-c.center));
        if(s.orientation==RAW_EXTRUSION_ARC_ORIENTATION_CCW && c.sweep<=0) c.sweep+=2*pi;
        if(s.orientation==RAW_EXTRUSION_ARC_ORIENTATION_CW && c.sweep>=0) c.sweep-=2*pi;
    }
    return c;
}

// SDK-facing plugin adapter.  It selects eligible perimeter loops, supplies
// regional settings and inherited process state to the geometry kernel, and
// publishes only changed leaf content back to the borrowed plan tree.
class CornerStretch final : public PluginBase {
public:
    // Keep the orchestrator handle used by PluginBase for registration and
    // option-definition calls; ownership remains with the host.
    explicit CornerStretch(orchestrator_handle *o):PluginBase(o) {}
private:
    // These metadata callbacks describe the plugin identity, execution step,
    // ordering priority, dependencies, and configuration keys as one SDK
    // surface.  They do not edit a printing plan.
    const char *id_impl() const noexcept override { return "nematx_corner_stretch"; }
    const char *name_impl() const noexcept override { return "NematX corner stretch"; }
    const char *description_impl() const noexcept override { return "Stretches perimeter corners while preserving deposited volume."; }
    slicing_step_t step_impl() const noexcept override { return STEP_LAYER_EXTRUSION_EDIT; }
    int32_t priority_impl() const noexcept override { return 5; }
    const char *const *dependencies_impl() const noexcept override { return no_dependencies; }
    int32_t defined_config_keys(const char **keys) const noexcept override {
        if(keys) for(size_t i=0;i<std::size(options);++i) keys[i]=options[i].key;
        return int32_t(std::size(options));
    }
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override {
        if(keys) for(size_t i=0;i<std::size(options);++i)
            keys[i]={options[i].key,options[i].type,RAW_CONTAINER_TYPE_REGION,RAW_PRESET_TYPE_FFF_PRINT};
        if(keys) keys[std::size(options)]={"arc_fitting",RAW_CO_ENUM,RAW_CONTAINER_TYPE_NONE,RAW_PRESET_TYPE_NONE};
        return int32_t(std::size(options)+1);
    }

    // Register every corner-stretch option with the host.  The SDK owns the
    // resulting definitions; failure of any definition rejects initialization.
    void inilialize_impl(storage_handle *) const override {
        for(size_t i=0;i<std::size(options);++i) {
            raw_config_option_def d=raw_config_option_def_init();
            d.opt_key=options[i].key;d.type=options[i].type;d.default_serialized_value=options[i].value;
            d.label=options[i].label;d.full_label=options[i].label;
            d.tooltip=i==2?"Full influence below 90 degrees, decreasing linearly to zero at this angle.":
                i>=6 && i<=8?"Transition length in mm or percent of the angle-adjusted stretch distance.":
                "Perimeter corner stretching. Settings are selected at the original corner; the deformation may cross region boundaries.";
            d.container_type=RAW_CONTAINER_TYPE_REGION;d.option_preset_type=RAW_PRESET_TYPE_FFF_PRINT;
            d.printer_technology=RAW_PT_FFF;d.invalidates_step=STEP_LAYER_EXTRUSION_EDIT;
            d.category=RAW_OPTION_CATEGORY_WIDTH;d.level=RAW_OPTION_LEVEL_EXPERT;d.mode=RAW_CONFIG_OPTION_MODE_EXPERT;
            d.has_min=1;d.min_value=0;
            const char *alias=i==4?"stretch_corners_distance":i==7?"stretch_corners_deviation":nullptr;
            if(alias) { d.aliases.items=&alias;d.aliases.size=1; }
            if(orchestrator_create_option_def(m_orchestrator,&d)!=OPTION_DEF_ERROR_OK)
                throw std::runtime_error(std::string("Cannot define ")+d.opt_key);
        }
    }

    // Flatten a tree into ordered leaves while resolving inherited state.
    // Empty event/property leaves are retained as barriers, cancellation is
    // checked during traversal, and all returned node handles are borrowed.
    void collect(MutableExtrusionEntity node,State state,std::vector<Leaf>&leaves,const plugin_run_context *ctx) const {
        throw_if_cancelled(ctx);state=effective(node,std::move(state));
        if(node.child_count()) for(uint32_t i=0;i<node.child_count();++i) collect(node.child_mutable(i),state,leaves,ctx);
        else if(node.segment_count() || node.property_count()) leaves.push_back({node,state,node.segments()});
    }

    // Edit one source loop in its original leaf order.  It filters eligible
    // edges, resolves the setting at each original corner, applies first-layer
    // limits when first is true, calls the pure geometry kernel, serializes its
    // output into temporary clones, and then replaces only changed leaves.  A
    // later error does not undo earlier publications in this non-transactional
    // callback.
    void edit_loop(MutableExtrusionEntity root,State inherited,const LayerRegionIsland&source,c_point shift,
                   bool first,bool native_arcs,const plugin_run_context *ctx) const {
        // Collecting keeps event-only leaves in the sequence, so they separate
        // process/property scopes even though they contribute no edge.
        std::vector<Leaf> leaves;collect(root,inherited,leaves,ctx);
        std::vector<Edge> edges;std::vector<unsigned> source_segment_indices;
        unsigned state_id=0;
        double area=0;std::optional<Point> origin;
        for(unsigned l=0;l<leaves.size();++l) {
            // A property-only leaf is a real barrier.  Equivalent, adjacent
            // geometric leaves may share a state; all other changes start a
            // new process-state identity for volume correction.
            if(l && (leaves[l-1].segments.empty() || leaves[l].segments.empty() ||
                     !equivalent(leaves[l-1].state,leaves[l].state))) ++state_id;
            const auto &st=leaves[l].state;
            // Only planar perimeter loops with positive flow and width are
            // eligible.  Vase paths, variable-Z segments, travel roles, and
            // zero-flow data stay in the edge stream but cannot be stretched.
            const bool eligible=st.attr && RAW_EXTRUSION_ROLE_IS_PERIMETER(st.attr->extrusion_role()) &&
                (st.perimeter&C_EXTRUSION_PERIMETER_FLAG_LOOP) && !(st.perimeter&C_EXTRUSION_PERIMETER_FLAG_VASE) &&
                static_cast<const c_extrusion_property_attributes&>(*st.attr).height!=-2 &&
                st.attr->mm3_per_mm()>0 && static_cast<const c_extrusion_property_attributes&>(*st.attr).width>0;
            for(unsigned j=0;j<leaves[l].segments.size();++j) {
                const auto &s=leaves[l].segments[j];Curve c=curve(s);
                if(norm(c.b-c.a)==0 && c.sweep==0 && s.z_offset_a==s.z_offset_b) continue;
                if(eligible) {
                    if(!origin) origin=c.a;
                    // The signed loop area identifies the outer/hole direction;
                    // the later convex test combines it with the hole flag.
                    area+=cross(c.a-*origin,c.b-*origin)+(c.sweep==0?0:dot(c.a-c.center,c.a-c.center)*(c.sweep-std::sin(c.sweep)));
                }
                edges.push_back({c,l,state_id,eligible && s.z_offset_a==s.z_offset_b});source_segment_indices.push_back(j);
            }
        }
        // An open/empty or zero-area source has no usable loop orientation and
        // therefore cannot produce a corner request.
        if(edges.empty() || std::abs(area)<1) return;
        std::vector<Settings> configs(edges.size());
        RegionSettings::OptionKeyGroup keys;for(const auto&o:options) keys.push_back(o.key);
        for(size_t i=0;i+1<edges.size();++i) {
            throw_if_cancelled(ctx);
            // A request is made only at an adjacent eligible straight junction
            // with matching process state.  Existing arcs and barriers stop
            // propagation; the loop seam is not joined to the first edge.
            if(!edges[i].eligible || !edges[i+1].eligible || edges[i].state!=edges[i+1].state ||
                edges[i].curve.sweep!=0 || edges[i+1].curve.sweep!=0) continue;
            const Point incoming=edges[i].curve.b-edges[i].curve.a,outgoing=edges[i+1].curve.b-edges[i+1].curve.a;
            const double a=norm(incoming),b=norm(outgoing);
            if(a<=0 || b<=0) continue;
            const Point p=edges[i].curve.b;
            // Region masks are in object coordinates.  This is the one place
            // where the print-instance shift is removed before a lookup; the
            // original vertex, not a later displaced point, chooses settings.
            const auto lookup=RegionSettings::lookup_at_point(source,keys,point(p-point(shift)));
            // Outside or ambiguous lookup, and an inactive source option, keep
            // this original corner unchanged rather than clipping its output.
            if(!lookup.found() || !lookup.value.get_bool(options[0].key)) continue;
            const auto &v=lookup.value;Settings s;s.enabled=true;s.arcs=v.get_bool(options[1].key);
            // Read a finite, non-negative effective value in the option's
            // native scaled-unit form.  Distances and deviations are converted
            // to continuous scaled units only after the corner is selected.
            auto number=[&](size_t key,double base) {
                double value=v.get_effective_value(base,options[key].key);
                if(!std::isfinite(value) || value<0) throw std::runtime_error(std::string("Invalid ")+options[key].key);
                return value;
            };
            s.angle=number(2,1);const double threshold=s.angle<=0 || s.angle>180?180:s.angle;
            const auto &attrs=static_cast<const c_extrusion_property_attributes&>(*leaves[edges[i].owner].state.attr);
            const bool external=RAW_EXTRUSION_ROLE_IS_EXTERNAL_PERIMETER(attrs.role);
            const double inner=external?1:number(11,1);
            if(inner==0) continue;
            const double theta=std::acos(std::clamp(-dot(incoming,outgoing)/(a*b),-1.,1.))*180/pi;
            if(theta>=threshold) continue;
            const bool hole=(leaves[edges[i].owner].state.perimeter&C_EXTRUSION_PERIMETER_FLAG_HOLE)!=0;
            const bool convex=(cross(incoming,outgoing)>0) != ((hole && area>0)||(!hole && area<0));
            if(!std::isfinite(attrs.mm3_per_mm)||attrs.mm3_per_mm<=0 || !std::isfinite(attrs.width)||attrs.width<=0 ||
                !std::isfinite(leaves[edges[i].owner].state.process.speed_mm_per_s)||leaves[edges[i].owner].state.process.speed_mm_per_s<=0)
                throw std::runtime_error("Corner stretch requires positive finite flow, width and resolved speed");
            s.distance=number(convex?4:3,attrs.width);
            if(first) s.distance=std::min(s.distance,number(5,attrs.width));
            const double alpha=threshold<=90?1:(std::max(theta,90.)-threshold)/(90-threshold);
            const double distance=s.distance*alpha;
            s.deviation=number(convex?7:6,distance);
            if(first) s.deviation=std::min(s.deviation,number(8,distance));
            s.distance/=double(SCALING_FACTOR);
            s.deviation/=double(SCALING_FACTOR);
            s.entering=number(9,1);s.exiting=number(10,1);
            s.entering*=inner;s.exiting*=inner;
            configs[i]=s;
        }
        // The kernel is not called when no original corner requested a change.
        if(std::none_of(configs.begin(),configs.end(),[](const Settings&s){return s.enabled;})) return;
        auto output=stretch(edges,configs,double(SCALED_EPSILON));
        std::vector<bool> changed(leaves.size(),false);
        // Compare kernel output with the original edge.  A leaf is replaced as
        // one unit when any owned edge differs, preserving its child order.
        for(size_t i=0;i<edges.size();++i) if(output[i].size()!=1 || output[i][0].ratio!=1 ||
            norm(output[i][0].curve.a-edges[i].curve.a)>0 || norm(output[i][0].curve.b-edges[i].curve.b)>0)
            changed[edges[i].owner]=true;
        if(std::none_of(changed.begin(),changed.end(),[](bool b){return b;})) return;
        // Prepare all replacement trees before touching any borrowed source leaf.
        std::vector<std::optional<StoredExtrusionEntity>> replacements(leaves.size());
        for(size_t l=0;l<leaves.size();++l) if(changed[l]) {
            // Clone the source leaf into shared plugin storage, clear only its
            // child content, and keep its parent-level properties/events.
            replacements[l].emplace(ctx->plugin_storage,leaves[l].node.readonly());
            replacements[l]->clear_content();
        }
        const double tolerance=std::max(double(SCALED_EPSILON),.01*UNSCALING_FACTOR);
        for(size_t i=0;i<edges.size();++i) if(changed[edges[i].owner]) {
            throw_if_cancelled(ctx);const unsigned l=edges[i].owner;const auto &original=leaves[l].segments[source_segment_indices[i]];
            for(const auto &f:output[i]) {
                // Reuse an unchanged source segment so metadata and exact SDK
                // arc representation survive when no serialization is needed.
                if(output[i].size()==1 && f.ratio==1 &&
                   norm(f.curve.a-edges[i].curve.a)==0 && norm(f.curve.b-edges[i].curve.b)==0 &&
                   f.curve.sweep==edges[i].curve.sweep) {
                    auto child=replacements[l]->emplace_child();
                    if(!child.set_segments(&original,1)) throw std::runtime_error("Cannot preserve source segment");
                    continue;
                }
                std::vector<c_extrusion_segment> serialized;
                const double radius=norm(f.curve.a-f.curve.center);
                unsigned count=1;
                // Native arcs are kept when requested.  Otherwise split the
                // curve using the 0.01 mm chord-error tolerance converted to
                // scaled units, with a straight fallback for impossible arcs.
                if(!native_arcs && f.curve.sweep!=0) {
                    const double step=radius>0?2*std::acos(std::clamp(1-tolerance/radius,-1.,1.)):pi;
                    count=unsigned(std::max(1.,std::ceil(std::abs(f.curve.sweep)/std::max(step,.001))));
                }
                double measured=0;
                for(unsigned n=0;n<count;++n) {
                    // Quantize each serialized endpoint, discard zero-length
                    // pieces, and measure the actual serialized curve.
                    c_extrusion_segment s=original;s.point_a=point(f.curve.at(double(n)/count));s.point_b=point(f.curve.at(double(n+1)/count));
                    if(s.point_a.x==s.point_b.x && s.point_a.y==s.point_b.y) continue;
                    s.radius=!native_arcs || f.curve.sweep==0?0:float(radius*(std::abs(f.curve.sweep)>pi?-1:1));
                    s.orientation=s.radius==0?RAW_EXTRUSION_ARC_ORIENTATION_UNKNOWN:
                        f.curve.sweep>0?RAW_EXTRUSION_ARC_ORIENTATION_CCW:RAW_EXTRUSION_ARC_ORIENTATION_CW;
                    measured+=curve(s).length();serialized.push_back(s);
                }
                if(serialized.empty()) {
                    if(f.curve.length()>double(SCALED_EPSILON)) throw std::runtime_error("Corner segment collapsed during quantization");
                    continue;
                }
                // ratio is original length / new length.  Applying it to q and
                // advertised width preserves deposited volume; speed uses the
                // reciprocal, capped at x10, rather than a machine XY limit.
                const double ratio=f.ratio*f.curve.length()/measured;
                auto child=replacements[l]->emplace_child();
                if(!child.set_segments(serialized)) throw std::runtime_error("Cannot publish corner segment");
                if(ratio!=1) {
                    auto &attr=child.get_or_add(EPropertyAttributes::key);attr=*leaves[l].state.attr;
                    attr.mm3_per_mm(attr.mm3_per_mm()*ratio);
                    attr.width(float(static_cast<const c_extrusion_property_attributes&>(*leaves[l].state.attr).width*ratio));
                    auto &process=child.get_or_add(EPropertySpeed::key);
                    static_cast<c_extrusion_property_speed&>(process)=leaves[l].state.process;
                    process.speed_mm_per_s=float(leaves[l].state.process.speed_mm_per_s*std::min(10.,1/ratio));
                }
            }
        }
        // Publish in original leaf order only after every replacement child is
        // ready.  Sorting and reversing stay disabled so event/property order
        // cannot be reordered after the append operations.
        for(size_t l=0;l<leaves.size();++l) if(changed[l]) {
            replacements[l]->disable_sort().disable_reverse();
            if(!extrusion_copy_from(leaves[l].node.mutable_handle(),replacements[l]->handle()))
                throw std::runtime_error("Cannot replace corner extrusion");
        }
    }

    // Visit the extrusion tree in host order.  A loop is edited as one local
    // group; collections and roots only pass their resolved inherited state to
    // children.
    void walk(MutableExtrusionEntity node,State inherited,const LayerRegionIsland&source,c_point shift,bool first,bool native_arcs,
              const plugin_run_context *ctx) const {
        throw_if_cancelled(ctx);State state=effective(node,inherited);
        if(node.is_loop() && (state.perimeter&C_EXTRUSION_PERIMETER_FLAG_LOOP)) {
            edit_loop(node,inherited,source,shift,first,native_arcs,ctx);return;
        }
        for(uint32_t i=0;i<node.child_count();++i) walk(node.child_mutable(i),state,source,shift,first,native_arcs,ctx);
    }

    // Run once for the host's current layer group.  It validates the SDK
    // context, skips synthetic extrusions and sources without an active option,
    // and leaves other layer groups untouched when callbacks are parallel.
    void run_impl(const plugin_run_context *ctx) const override {
        const auto *c=plugin_ctx_as_layer_extrusion_edition(ctx);
        if(!c || !c->print || !c->plan || !c->layer_group || !ctx->plugin_storage) throw std::runtime_error("Invalid corner stretch context");
        PrintingPlan plan(c->plan);PrintingLayerGroup group(c->layer_group);
        const bool native_arcs=Print(c->print).config().int_or_default("arc_fitting",0)!=0;
        for(uint32_t t=0;t<group.tool_group_count();++t) {
            auto tool=group.tool_group(t);
            for(uint32_t i=0;i<tool.extrusion_count();++i) {
                auto ex=tool.extrusion(i);auto source=ex.region_island();
                // Synthetic work has no regional source and is not a perimeter
                // loop from the object, so there is nothing to edit.
                if(!source.valid()) continue;
                bool active=false;
                for(uint32_t r=0;r<source.region_count();++r)
                    active|=source.region(r).print_region().config().bool_or_default("stretch_corners",false);
                // Avoid even collecting the tree when every source region has
                // the option disabled.  A later region boundary cannot enable
                // a corner that was not selected at its original vertex.
                if(!active) continue;
                walk(ex.mutable_root(),{},source,plan.object_instance_shift(ex),source.region(0).layer().bottom_z()==0,native_arcs,ctx);
            }
        }
    }
};
}
extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *o) {
    // Construct the static adapter and hand its SDK instance to the host
    // orchestrator; the host owns the registration lifetime.
    static nematx_corner::CornerStretch plugin(o);orchestrator_register_plugin(o,plugin.c_instance());
}
#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"
