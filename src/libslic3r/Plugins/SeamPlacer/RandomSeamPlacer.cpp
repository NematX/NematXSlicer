///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/*
RandomSeamPlacer: from a printing plan to cached seam positions
=============================================================
This provider chooses a reproducible random starting point for each atomic loop.
If vertices-only mode has admissible vertices, it samples uniformly among
distinct admissible vertices (including actual constraint boundaries). Otherwise,
it makes best-effort length-weighted samples along admissible arclength. Integer
rounding may reject sampled points; after 64 attempts it falls back to a
representable random vertex.
Inner loops may instead align with an external seam. Arrival positions are ignored.

Preparation has three stages. First, walk the plan, resolve source regions and
inherited extrusion properties, and obtain the admissible domains from
SeamPlacementConstraints. Painted blockers override painted or volume enforcers;
free portions are the second choice, and a fully blocked loop uses its whole path.
Second, draw one random seam per loop. Finally, align internal loops against the
already chosen external seams on the same layer and instance, and publish only
the resulting immutable lookup table. No extrusion is split or mutated.

An enclosing outer contour is the alignment reference unless an enclosed hole
improves the distance by at least one third of the internal extrusion width.
An admissible vertex within three widths is preferred; otherwise use a closest
point on an admissible line or arc, even when vertex-only mode is enabled.
Curve calculations use actual arcs rather than their chords. Vertex candidates
exclude redundant intersections introduced only to partition the geometry.

Positive object seeds reproduce the same plan/configuration traversal; zero draws
fresh entropy once per object per session. The integer generator and conversion
are specified here instead of relying on STL distributions. Changing the plan's
structure can change subsequent draws. Request order and thread scheduling cannot.

Positions, radii, distances and margins use scaled XY units in the plan's platter
frame; source slices are object-local and require the explicit instance shift.
Attribute widths are converted from millimetres. Returned curve points are rounded
to integer coordinates and checked against constraints. Paint uses the public
projection helper and its width margin, not the legacy 3D triangle-distance test.

Only public C contracts and their C++ views are used. Initialization borrows host
objects and temporary plugin storage; local RAII owners release copied geometry,
constraints and temporary allocations before publication. The session then owns
only result values and non-owning loop/region identities, which the host must keep
alive. Queries are const cache lookups; adding loops or changing configuration or
geometry requires a new session. The service adapter owns session destruction.
*/

#include "RandomSeamPlacer.hpp"
#include "SeamInitializationCancellation.hpp"
#include "SeamPlacementConstraints.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <iterator>
#include <map>
#include <random>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_extrusion_property.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace SeamPlacement { namespace RandomPlugin {
namespace {
using Curve = SeamPlacementConstraints::Curve;
using Portion = SeamPlacementConstraints::Portion;
using Status = SeamPlacementConstraints::Status;
constexpr double pi = 3.14159265358979323846;
const char *const keys[] = {"seam_random_vertices_only", "seam_random_align_inner_seams", "seam_random_seed"};

struct RegionFilter
{
    bool plan_wide = false;
    std::unordered_set<const print_region_handle *> assigned;
};

/* Resolve borrowed region views once so plan traversal does constant-time membership tests. */
RegionFilter make_region_filter(const std::vector<PrintRegion> &regions);
/* Include a multi-region island when any one source region is assigned. */
bool island_is_assigned(const LayerRegionIsland &island, const RegionFilter &filter);
/* Test one candidate region without dereferencing an invalid view. */
bool region_is_assigned(const LayerRegion &region, const RegionFilter &filter);

struct Candidate { c_point point{}; double width = 0; };
struct Domain { std::vector<Portion> ranges; std::vector<Candidate> vertices; Status status = Status::Free; };
struct Loop {
    ExtrusionEntity entity;
    LayerRegion region;
    uint16_t instance = 0;
    bool external = false, hole = false, align = true;
    std::vector<Curve> curves;
    Domain domain;
    c_point seam{};
};

/* SplitMix64 fixes both the generator and the mapping across standard libraries. */
uint64_t next_random(uint64_t &state);
/* Rejection removes modulo bias, including lists with a non-power-of-two size. */
size_t random_index(uint64_t &state, size_t count);
/* Compare value snapshots, never handles or diagnostic native arc caches. */
bool same_curve(const Curve &a, const Curve &b);
/* Euclidean distance between scaled points; subtract in double to avoid overflow. */
double distance(c_point a, c_point b);
/* Merge redundant partitions and retain actual admissibility boundaries only. */
Domain make_domain(const SeamPlacementConstraints &constraints, const ExtrusionEntity &loop, const LayerRegion &region);
/* Round a curve point and verify it still belongs to the selected closed zones. */
bool admissible_point(const SeamPlacementConstraints &constraints, const Loop &loop, const Portion &range, double t, c_point &out);
/* Sample vertices uniformly or make best-effort length-weighted arclength
   draws; after 64 rejected rounded points, use a representable random vertex. */
c_point random_seam(const SeamPlacementConstraints &constraints, const Loop &loop, bool vertices_only, uint64_t &state);
/* Closest admissible curve point, returning its inherited internal width as well. */
Candidate nearest(const SeamPlacementConstraints &constraints, const Loop &loop,
                  c_point target, const plugin_run_context *run_context);
/* Ray crossing splits arcs at their Y extrema, so half-open endpoints are counted once. */
bool point_inside(const std::vector<Curve> &boundary, c_point point,
                  const plugin_run_context *run_context);
/* Exact line/circle intersections partition the child before containment tests. */
bool includes(const std::vector<Curve> &outer, const std::vector<Curve> &inner,
              const plugin_run_context *run_context);
/* Signed Green integral on lines and arcs, used to select the smallest enclosure. */
double area(const std::vector<Curve> &curves,
            const plugin_run_context *run_context);
/* Snapshot all path segments recursively without linearizing arcs. */
void collect_curves(const ExtrusionEntity &entity, std::vector<Curve> &curves,
                    const plugin_run_context *run_context);
/* Source slices are object-local; test a platter loop after removing instance shift. */
bool touches_region(const std::vector<Curve> &curves, const LayerRegion &region,
                    c_point shift, const plugin_run_context *run_context);
/* Reconstruct inherited tags before an atomic loop is recorded. */
void collect_loops(const ConstPrintingExtrusion &source, const ExtrusionEntity &entity,
                   uint16_t object_instance_idx,
                   raw_extrusion_role role, uint16_t flags, const RegionFilter &filter,
                   std::vector<Loop> &loops,
                   const plugin_run_context *run_context);
/* Align each internal loop against immutable external random seams. */
void align_loops(const SeamPlacementConstraints &constraints,
                 std::vector<Loop> &loops,
                 const plugin_run_context *run_context);

class RandomSession final : public SeamPlacerSession {
public:
    explicit RandomSession(storage_handle *storage,
                           std::function<uint64_t()> entropy,
                           const plugin_run_context *run_context = nullptr) :
        m_storage(storage), m_entropy(std::move(entropy)),
        m_run_context(run_context) {}
    /* All allocations and entropy reads occur before publication. */
    void initialize(const PrintingPlan &plan, const std::vector<PrintRegion> &regions) override;
    /* Equal rank preserves source region order, as in FixedDirection. */
    float get_priority(const ExtrusionEntity &, const LayerRegion &) const override { return 50; }
    /* Borrowed handle identity remains valid until the host destroys the plan. */
    c_point place_seam(const ExtrusionEntity &loop, const LayerRegion &region, c_point) const override;
private:
    storage_handle *m_storage;
    std::function<uint64_t()> m_entropy;
    const plugin_run_context *m_run_context;
    std::map<const extrusion_entity_handle *, std::pair<const layer_region_handle *, c_point>> m_results;
};

class RandomProvider final : public PluginBase {
public:
    explicit RandomProvider(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}
private:
    const char *id_impl() const noexcept override { return "seam_placer.random"; }
    const char *name_impl() const noexcept override { return "Random"; }
    const char *description_impl() const noexcept override { return "Reproducible random seams respecting painted and volume constraints, with optional inner seam alignment."; }
    const char *exclusive_group_impl() const noexcept override { return "seam_placer_plugin"; }
    slicing_step_t step_impl() const noexcept override { return SEAM_PLACER; }
    // Return the common seam execution priority. Nothing is changed or transferred, and the call cannot fail.
    int32_t priority_impl() const noexcept override { return 600; }
    // Return this seam algorithm's preference. Nothing is changed or transferred, and the call cannot fail.
    int32_t exclusive_group_priority_impl() const noexcept override { return 40; }
    const char *const *dependencies_impl() const noexcept override { static const char *const dependencies[]={nullptr}; return dependencies; }
    /* Register both visible booleans and the object-only seed definition. */
    void inilialize_impl(storage_handle *) const override;
    /* Declare dependencies so object seed edits invalidate ordering. */
    int32_t used_config_keys(raw_used_config_key *out) const noexcept override;
    int32_t defined_config_keys(const char **out) const noexcept override;
    /* Publish only fully initialized sessions; the C++ adapter owns failure cleanup. */
    void run_impl(const plugin_run_context *context) const override;
};

uint64_t next_random(uint64_t &state)
{
    uint64_t value = (state += UINT64_C(0x9e3779b97f4a7c15));
    value = (value ^ (value >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    value = (value ^ (value >> 27)) * UINT64_C(0x94d049bb133111eb);
    return value ^ (value >> 31);
}

size_t random_index(uint64_t &state, size_t count)
{
    const uint64_t bound = uint64_t(count), threshold = (uint64_t(0)-bound)%bound;
    uint64_t value;
    do { value=next_random(state); } while (value<threshold);
    return size_t(value%bound);
}

bool same_curve(const Curve &a, const Curve &b)
{
    return a.segment.point_a.x==b.segment.point_a.x && a.segment.point_a.y==b.segment.point_a.y &&
           a.segment.point_b.x==b.segment.point_b.x && a.segment.point_b.y==b.segment.point_b.y &&
           a.segment.radius==b.segment.radius && a.segment.orientation==b.segment.orientation;
}

double distance(c_point a, c_point b)
{
    return std::hypot(double(a.x)-b.x,double(a.y)-b.y);
}

Domain make_domain(const SeamPlacementConstraints &constraints, const ExtrusionEntity &loop, const LayerRegion &region)
{
    const SeamPlacementConstraints::Result parts=constraints.portions(loop,region,true);
    Domain domain;
    domain.status=Status::Blocked;
    for (const Portion &part:parts.portions) domain.status=std::max(domain.status,part.status);
    // Adjacent intervals with the same category are one admissible portion.
    // An excluded point between them prevents merging, even if it has zero length.
    for (const Portion &part:parts.portions) {
        if (part.status!=domain.status || part.end<=part.begin) continue;
        if (!domain.ranges.empty()) {
            Portion &last=domain.ranges.back();
            if (same_curve(last.curve,part.curve) && last.end==part.begin &&
                last.include_end && part.include_begin && last.paint_margin==part.paint_margin) {
                last.end=part.end;
                last.include_end=part.include_end;
                continue;
            }
        }
        domain.ranges.push_back(part);
    }
    std::map<std::pair<coord_t,coord_t>,size_t> seen;
    // Preserve first traversal order while de-duplicating all coordinates. At
    // shared endpoints the largest incident width allows either segment's test.
    const std::function<void(c_point,double)> add=[&](c_point point,double width) {
        const std::pair<coord_t,coord_t> key(point.x,point.y);
        const std::pair<std::map<std::pair<coord_t,coord_t>,size_t>::iterator,bool> inserted=seen.emplace(key,domain.vertices.size());
        if (inserted.second) domain.vertices.push_back({point,width});
        else domain.vertices[inserted.first->second].width=std::max(domain.vertices[inserted.first->second].width,width);
    };
    for (const Portion &range:domain.ranges) {
        const double inset=range.curve.length()>0 ? std::min((range.end-range.begin)/2,2/range.curve.length()) : 0;
        for (double t:{range.begin+(range.include_begin?0:inset),range.end-(range.include_end?0:inset)}) {
            c_point point=range.curve.point(t);
            if (constraints.classify_point(loop,point,range.paint_margin)!=domain.status)
                point=range.curve.point(t<(range.begin+range.end)/2 ? std::min(range.end,t+inset) : std::max(range.begin,t-inset));
            if (constraints.classify_point(loop,point,range.paint_margin)==domain.status) add(point,range.paint_margin);
        }
    }
    // Isolated admissible points (for example a tangency) have no interval.
    for (const Portion &part:parts.portions) {
        if (part.begin!=part.end || part.status!=domain.status) continue;
        bool interior=false;
        for (const Portion &range:domain.ranges)
            if (same_curve(range.curve,part.curve) && part.begin>range.begin && part.begin<range.end) { interior=true; break; }
        if (!interior) {
            const c_point point=part.curve.point(part.begin);
            if (constraints.classify_point(loop,point,part.paint_margin)==domain.status) add(point,part.paint_margin);
        }
    }
    return domain;
}

bool admissible_point(const SeamPlacementConstraints &constraints, const Loop &loop, const Portion &range, double t, c_point &out)
{
    out=range.curve.point(t);
    return constraints.classify_point(loop.entity,out,range.paint_margin)==loop.domain.status;
}

c_point random_seam(const SeamPlacementConstraints &constraints, const Loop &loop, bool vertices_only, uint64_t &state)
{
    if (vertices_only && !loop.domain.vertices.empty())
        return loop.domain.vertices[random_index(state,loop.domain.vertices.size())].point;
    double total=0;
    for (const Portion &range:loop.domain.ranges) total+=range.curve.length()*(range.end-range.begin);
    if (total>0) {
        // Integer rounding may land on an excluded edge. Rejection preserves
        // the length distribution rather than biasing all such draws to a vertex.
        for (uint32_t attempt=0;attempt<64;++attempt) {
            double position=double(next_random(state)>>11)*0x1.0p-53*total;
            for (const Portion &range:loop.domain.ranges) {
                const double length=range.curve.length()*(range.end-range.begin);
                if (length<=0) continue;
                if (position<length) {
                    c_point point;
                    if (admissible_point(constraints,loop,range,range.begin+(range.end-range.begin)*position/length,point)) return point;
                    break;
                }
                position-=length;
            }
        }
    }
    if (!loop.domain.vertices.empty()) return loop.domain.vertices[random_index(state,loop.domain.vertices.size())].point;
    throw std::runtime_error("Random seam has no representable admissible point.");
}

Candidate nearest(const SeamPlacementConstraints &constraints, const Loop &loop,
                  c_point target, const plugin_run_context *run_context)
{
    Candidate best;
    double best_distance=std::numeric_limits<double>::infinity();
    size_t vertex_index=0;
    for (const Candidate &vertex:loop.domain.vertices) {
        InitializationCancellation::check_periodic(run_context,vertex_index++);
        const double d=distance(vertex.point,target);
        if (d<best_distance) { best=vertex; best_distance=d; }
    }
    size_t range_index=0;
    for (const Portion &range:loop.domain.ranges) {
        InitializationCancellation::check_periodic(run_context,range_index++);
        const double inset=range.curve.length()>0 ? std::min((range.end-range.begin)/2,2/range.curve.length()) : 0;
        const double begin=range.begin+(range.include_begin?0:inset), end=range.end-(range.include_end?0:inset);
        const double t=range.curve.closest_parameter(target,begin,end);
        for (double candidate:{t,std::min(end,t+inset),std::max(begin,t-inset)}) {
            c_point point;
            if (!admissible_point(constraints,loop,range,candidate,point)) continue;
            const double d=distance(point,target);
            if (d<best_distance) { best={point,range.paint_margin}; best_distance=d; }
        }
    }
    if (!std::isfinite(best_distance)) throw std::runtime_error("Random seam cannot project onto its admissible domain.");
    return best;
}

bool point_inside(const std::vector<Curve> &boundary, c_point point,
                  const plugin_run_context *run_context)
{
    bool inside=false;
    size_t curve_index=0;
    for (const Curve &curve:boundary) {
        InitializationCancellation::check_periodic(run_context,curve_index++);
        if (distance(curve.point(curve.closest_parameter(point)),point)<=1) return true;
        std::vector<double> cuts={0,1};
        if (curve.radius>0) {
            for (double angle:{pi/2,3*pi/2}) {
                const double t=curve.parameter(angle);
                if (t>0 && t<1) cuts.push_back(t);
            }
        }
        std::sort(cuts.begin(),cuts.end());
        for (size_t i=1;i<cuts.size();++i) {
            const c_point a=curve.point(cuts[i-1]), b=curve.point(cuts[i]);
            if ((a.y>point.y)==(b.y>point.y)) continue;
            double x;
            if (curve.radius==0) x=double(a.x)+(double(point.y)-a.y)*(double(b.x)-a.x)/(double(b.y)-a.y);
            else {
                const double dy=double(point.y)-curve.cy;
                const double offset=std::sqrt(std::max(0.,curve.radius*curve.radius-dy*dy));
                const double angle=curve.start+(cuts[i-1]+cuts[i])/2*curve.sweep;
                x=curve.cx+(std::cos(angle)>=0?offset:-offset);
            }
            if (x>point.x) inside=!inside;
        }
    }
    return inside;
}

bool includes(const std::vector<Curve> &outer, const std::vector<Curve> &inner,
              const plugin_run_context *run_context)
{
    if (outer.empty() || inner.empty()) return false;
    size_t curve_index=0;
    for (const Curve &curve:inner) {
        InitializationCancellation::check_periodic(run_context,curve_index++);
        std::vector<double> cuts={0,1};
        size_t edge_index=0;
        for (const Curve &edge:outer) {
            InitializationCancellation::check_periodic(run_context,edge_index++);
            curve.intersection_cuts(edge,cuts);
        }
        std::sort(cuts.begin(),cuts.end());
        for (size_t i=0;i<cuts.size();++i) {
            InitializationCancellation::check_periodic(run_context,i);
            if (!point_inside(outer,curve.point(cuts[i]),run_context)) return false;
            if (i>0 && !point_inside(outer,curve.point((cuts[i-1]+cuts[i])/2),run_context)) return false;
        }
    }
    return true;
}

double area(const std::vector<Curve> &curves,
            const plugin_run_context *run_context)
{
    long double sum=0;
    size_t curve_index=0;
    for (const Curve &curve:curves) {
        InitializationCancellation::check_periodic(run_context,curve_index++);
        const c_point a=curve.segment.point_a,b=curve.segment.point_b;
        if (curve.radius==0) sum+=static_cast<long double>(a.x)*b.y-static_cast<long double>(a.y)*b.x;
        else sum+=curve.cx*(double(b.y)-a.y)-curve.cy*(double(b.x)-a.x)+curve.radius*curve.radius*curve.sweep;
    }
    return double(std::abs(sum)/2);
}

void collect_curves(const ExtrusionEntity &entity, std::vector<Curve> &curves,
                    const plugin_run_context *run_context)
{
    for (uint32_t i=0;i<entity.segment_count();++i) {
        InitializationCancellation::check_periodic(run_context,i);
        curves.push_back(Curve::from_segment(entity.segment(i)));
    }
    if (entity.segment_count()==0 && entity.point_count()==1) {
        c_extrusion_segment segment{};
        segment.point_a=segment.point_b=entity.point(0);
        curves.push_back(Curve::from_segment(segment));
    }
    for (uint32_t i=0;i<entity.child_count();++i) {
        InitializationCancellation::check_periodic(run_context,i);
        collect_curves(entity.child(i),curves,run_context);
    }
}

bool touches_region(const std::vector<Curve> &curves, const LayerRegion &region,
                    c_point shift, const plugin_run_context *run_context)
{
    const ExPolygonCollection slices=region.slices();
    for (uint32_t i=0;i<slices.size();++i) {
        InitializationCancellation::check_periodic(run_context,i);
        const ExPolygon shape=slices.at(i);
        std::vector<Curve> edges;
        const std::function<void(const Polygon &)> append=[&](const Polygon &polygon) {
            for (uint32_t j=0;j<polygon.size();++j) {
                InitializationCancellation::check_periodic(run_context,j);
                c_extrusion_segment segment{};
                segment.point_a=polygon.at(j); segment.point_b=polygon.at((j+1)%polygon.size());
                edges.push_back(Curve::from_segment(segment));
            }
        };
        append(shape.contour());
        for (const Polygon &hole:shape.holes()) append(hole);
        size_t curve_index=0;
        for (Curve curve:curves) {
            InitializationCancellation::check_periodic(run_context,curve_index++);
            curve.segment.point_a.x-=shift.x; curve.segment.point_a.y-=shift.y;
            curve.segment.point_b.x-=shift.x; curve.segment.point_b.y-=shift.y;
            curve.cx-=shift.x; curve.cy-=shift.y;
            std::vector<double> cuts={0,1};
            size_t edge_index=0;
            for (const Curve &edge:edges) {
                InitializationCancellation::check_periodic(run_context,edge_index++);
                curve.intersection_cuts(edge,cuts);
            }
            std::sort(cuts.begin(),cuts.end());
            for (size_t j=0;j<cuts.size();++j) {
                InitializationCancellation::check_periodic(run_context,j);
                if (shape.contains(curve.point(cuts[j])) || (j>0 && shape.contains(curve.point((cuts[j-1]+cuts[j])/2)))) return true;
            }
        }
    }
    return false;
}

RegionFilter make_region_filter(const std::vector<PrintRegion> &regions)
{
    RegionFilter filter;
    filter.plan_wide=regions.empty();
    filter.assigned.reserve(regions.size());
    for (const PrintRegion &region:regions)
        if (region.valid()) filter.assigned.insert(region.handle());
    return filter;
}

bool island_is_assigned(const LayerRegionIsland &island, const RegionFilter &filter)
{
    if (!island.valid() || island.region_count()==0) return false;
    if (filter.plan_wide) return true;
    for (uint32_t i=0;i<island.region_count();++i) {
        const PrintRegion source=island.region(i).print_region();
        if (source.valid() && filter.assigned.find(source.handle())!=filter.assigned.end()) return true;
    }
    return false;
}

bool region_is_assigned(const LayerRegion &region, const RegionFilter &filter)
{
    if (filter.plan_wide) return true;
    const PrintRegion source=region.print_region();
    return source.valid() && filter.assigned.find(source.handle())!=filter.assigned.end();
}

void collect_loops(const ConstPrintingExtrusion &source, const ExtrusionEntity &entity,
                   const uint16_t object_instance_idx,
                   raw_extrusion_role role, uint16_t flags, const RegionFilter &filter,
                   std::vector<Loop> &loops,
                   const plugin_run_context *run_context)
{
    InitializationCancellation::check(run_context);
    const c_extrusion_property_attributes *attributes=static_cast<const c_extrusion_property_attributes *>(entity.property_data(EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
    const c_extrusion_property_perimeter *perimeter=static_cast<const c_extrusion_property_perimeter *>(entity.property_data(EXTRUSION_PROPERTY_TYPE_PERIMETER));
    if (attributes!=nullptr) role=attributes->role;
    if (perimeter!=nullptr) flags=perimeter->perimeter_flags;
    if (!entity.is_loop()) {
        for (uint32_t i=0;i<entity.child_count();++i) {
            InitializationCancellation::check_periodic(run_context,i);
            collect_loops(source,entity.child(i),object_instance_idx,role,flags,
                          filter,loops,run_context);
        }
        return;
    }
    const LayerRegionIsland island=source.region_island();
    if (!island_is_assigned(island,filter)) return;
    Loop loop;
    loop.entity=entity;
    loop.instance=object_instance_idx;
    loop.external=RAW_EXTRUSION_ROLE_IS_EXTERNAL_PERIMETER(role);
    loop.hole=(flags&C_EXTRUSION_PERIMETER_FLAG_HOLE)!=0;
    collect_curves(entity,loop.curves,run_context);
    const Object object=island.region(0).layer().object();
    const c_point shift=object.instance_shift(loop.instance);
    // A provider only prepares regions assigned to it. Within that set equal
    // policy priorities preserve source-island order, including boundary hits.
    LayerRegion fallback;
    for (uint32_t i=0;i<island.region_count();++i) {
        InitializationCancellation::check_periodic(run_context,i);
        const LayerRegion candidate=island.region(i);
        if (!region_is_assigned(candidate,filter)) continue;
        if (!fallback.valid()) fallback=candidate;
        if (touches_region(loop.curves,candidate,shift,run_context)) { loop.region=candidate; break; }
    }
    if (!loop.region.valid()) loop.region=fallback;
    if (loop.region.valid()) loops.push_back(std::move(loop));
}

void align_loops(const SeamPlacementConstraints &constraints,
                 std::vector<Loop> &loops,
                 const plugin_run_context *run_context)
{
    // A plan may contain thousands of layers. Limit geometric comparisons to
    // external loops on the same layer and instance, rather than scanning it all.
    using Location = std::pair<const layer_handle *,uint16_t>;
    std::map<Location,std::vector<const Loop *>> externals;
    size_t loop_index=0;
    for (const Loop &loop:loops) {
        InitializationCancellation::check_periodic(run_context,loop_index++);
        if (loop.external) externals[{loop.region.layer().handle(),loop.instance}].push_back(&loop);
    }
    loop_index=0;
    for (Loop &loop:loops) {
        InitializationCancellation::check_periodic(run_context,loop_index++);
        if (loop.external || !loop.align) continue;
        const Loop *contour=nullptr, *hole=nullptr;
        Candidate contour_projection, hole_projection;
        double contour_area=std::numeric_limits<double>::infinity(), hole_distance=std::numeric_limits<double>::infinity();
        const std::map<Location,std::vector<const Loop *>>::const_iterator location=externals.find({loop.region.layer().handle(),loop.instance});
        if (location==externals.end()) continue;
        size_t external_index=0;
        for (const Loop *external:location->second) {
            InitializationCancellation::check_periodic(
                run_context,external_index++);
            const Loop &other=*external;
            if (!other.hole && includes(other.curves,loop.curves,run_context)) {
                const double candidate_area=area(other.curves,run_context);
                if (candidate_area<contour_area) {
                    contour=&other; contour_area=candidate_area;
                    contour_projection=nearest(
                        constraints,loop,other.seam,run_context);
                }
            } else if (other.hole && includes(loop.curves,other.curves,run_context)) {
                const Candidate projection=nearest(
                    constraints,loop,other.seam,run_context);
                const double d=distance(projection.point,other.seam);
                if (d<hole_distance) { hole=&other; hole_distance=d; hole_projection=projection; }
            }
        }
        const Loop *reference=contour;
        Candidate projection=contour_projection;
        if (hole!=nullptr && (contour==nullptr || distance(contour_projection.point,contour->seam)-hole_distance>=hole_projection.width/3)) {
            reference=hole; projection=hole_projection;
        }
        if (reference==nullptr) continue;
        // A close vertex wins over the continuous projection. This exception
        // applies even when vertex-only random selection was requested.
        double closest=std::numeric_limits<double>::infinity();
        loop.seam=projection.point;
        size_t vertex_index=0;
        for (const Candidate &vertex:loop.domain.vertices) {
            InitializationCancellation::check_periodic(
                run_context,vertex_index++);
            const double d=distance(vertex.point,reference->seam);
            if (d<vertex.width*3 && d<closest) { closest=d; loop.seam=vertex.point; }
        }
    }
}

void RandomSession::initialize(const PrintingPlan &plan, const std::vector<PrintRegion> &regions)
{
    if (!plan.valid() || m_storage==nullptr) throw std::invalid_argument("Random seam needs a plan and temporary storage.");
    InitializationCancellation::check(m_run_context);
    SeamPlacementConstraints constraints;
    // A factory call with no assigned regions is the historical all-regions
    // mode. An explicit nonempty assignment must filter preparation as well as
    // loop collection, so foreign provider data cannot break this session.
    if (regions.empty()) constraints.initialize(plan,m_storage,m_run_context);
    else constraints.initialize_for_regions(plan,m_storage,regions,m_run_context);
    const RegionFilter filter=make_region_filter(regions);
    std::vector<Loop> loops;
    for (uint32_t g=0;g<plan.group_count();++g) {
        InitializationCancellation::check(m_run_context);
        const ConstPrintingGroup group=plan.group(g);
        for (uint32_t l=0;l<group.layer_group_count();++l) {
            const ConstPrintingLayerGroup layer=group.layer_group(l);
            for (uint32_t t=0;t<layer.tool_group_count();++t) {
                const ConstPrintingToolGroup tool=layer.tool_group(t);
                for (uint32_t e=0;e<tool.extrusion_count();++e) {
                    InitializationCancellation::check_periodic(m_run_context,e);
                    const ConstPrintingExtrusion extrusion=tool.extrusion(e);
                    // Reject foreign-provider work before consulting its
                    // object-instance metadata. Such work is outside this
                    // session and may legitimately be incomplete here.
                    if (!filter.plan_wide && !island_is_assigned(extrusion.region_island(),filter)) continue;
                    const size_t object_instance_idx = printing_plan_object_instance_idx(plan, extrusion);
                    if (object_instance_idx == size_t(-1) || object_instance_idx > std::numeric_limits<uint16_t>::max())
                        throw std::runtime_error("Random seam extrusion has no valid printing-plan instance.");
                    collect_loops(extrusion,extrusion.root(),static_cast<uint16_t>(object_instance_idx),
                                  extrusion.role(),0,filter,loops,m_run_context);
                }
            }
        }
    }
    std::map<const object_handle *,uint64_t> streams;
    for (Loop &loop:loops) {
        InitializationCancellation::check(m_run_context);
        const Object object=loop.region.layer().object();
        if (streams.find(object.handle())==streams.end()) {
            const int32_t configured=object.config().int_or_default(keys[2],42);
            if (configured<0) throw std::invalid_argument("Random seam seed must be nonnegative.");
            uint64_t seed=uint64_t(configured);
            if (configured==0) {
                if (m_entropy) seed=m_entropy();
                else {
                    std::random_device entropy;
                    seed=(uint64_t(entropy())<<32)^uint64_t(entropy());
                }
            }
            seed^=uint64_t(streams.size())*UINT64_C(0xd6e8feb86659fd93);
            streams.emplace(object.handle(),seed);
        }
        uint64_t state=next_random(streams.at(object.handle()));
        loop.domain=make_domain(constraints,loop.entity,loop.region);
        const Config config=loop.region.print_region().config();
        loop.align=config.bool_or_default(keys[1],true);
        loop.seam=random_seam(constraints,loop,config.bool_or_default(keys[0],true),state);
    }
    align_loops(constraints,loops,m_run_context);
    InitializationCancellation::check(m_run_context);
    for (const Loop &loop:loops)
        if (!m_results.emplace(loop.entity.handle(),std::make_pair(loop.region.handle(),loop.seam)).second)
            throw std::invalid_argument("Random seam loop occurs more than once in the initialized plan.");
    // Geometry/constraints die here. Published sessions own only value results;
    // the remaining handle keys are non-owning identities valid for this plan.
    m_storage=nullptr;
    m_entropy={};
    m_run_context=nullptr;
}

c_point RandomSession::place_seam(const ExtrusionEntity &loop, const LayerRegion &region, c_point) const
{
    const std::map<const extrusion_entity_handle *,std::pair<const layer_region_handle *,c_point>>::const_iterator found=m_results.find(loop.handle());
    if (found==m_results.end()) throw std::invalid_argument("Random seam loop is absent from the initialized plan.");
    if (region.handle()!=found->second.first) throw std::invalid_argument("Random seam region differs from its initialized source region.");
    return found->second.second;
}

void RandomProvider::inilialize_impl(storage_handle *) const
{
    const char *const labels[]={"Vertices only","Align inner seams","Random seam seed"};
    const char *const tooltips[]={
        "Choose uniformly among distinct vertices and constraint boundaries. Inner alignment may project between vertices.",
        "Align internal seams with an enclosing contour or a closer hole, while respecting seam constraints.",
        "Master seed for random seams on this object. Positive values reproduce the same plan; zero draws a fresh seed for each run."};
    for (uint32_t i=0;i<3;++i) {
        raw_config_option_def def=raw_config_option_def_init();
        def.opt_key=keys[i]; def.type=i==2?RAW_CO_INT:RAW_CO_BOOL;
        def.container_type=i==2?RAW_CONTAINER_TYPE_OBJECT:RAW_CONTAINER_TYPE_REGION;
        def.option_preset_type=RAW_PRESET_TYPE_FFF_PRINT; def.printer_technology=RAW_PT_FFF;
        def.label=labels[i]; def.full_label=labels[i]; def.tooltip=tooltips[i];
        def.category=RAW_OPTION_CATEGORY_PERIMETER;
        def.mode=RAW_CONFIG_OPTION_MODE_SIM_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
        def.invalidates_step=STEP_ORDERING; def.default_serialized_value=i==2?"42":"1";
        if (i==2) { def.has_min=1; def.min_value=0; }
        if (orchestrator_create_option_def(m_orchestrator,&def)!=OPTION_DEF_ERROR_OK)
            throw std::runtime_error("Cannot register random seam configuration.");
    }
    // Deliberately omit seed from the preset page. Its object definition remains
    // available in the existing add-setting/modifier selection mechanism.
    orchestrator_add_ui_fragment(m_orchestrator,"print.ui","seam_placer.random",
        "page:Perimeters & Shell\ngroup:Seam\nsetting:seam_random_vertices_only\nsetting:seam_random_align_inner_seams\n",1);
}

int32_t RandomProvider::used_config_keys(raw_used_config_key *out) const noexcept
{
    if (out!=nullptr) for (uint32_t i=0;i<3;++i)
        out[i]={keys[i],i==2?RAW_CO_INT:RAW_CO_BOOL,i==2?RAW_CONTAINER_TYPE_OBJECT:RAW_CONTAINER_TYPE_REGION,RAW_PRESET_TYPE_FFF_PRINT};
    return 3;
}

int32_t RandomProvider::defined_config_keys(const char **out) const noexcept
{
    if (out!=nullptr) std::copy(std::begin(keys),std::end(keys),out);
    return 3;
}

void RandomProvider::run_impl(const plugin_run_context *context) const
{
    run_ctx_seam_placer *payload=plugin_ctx_as_seam_placer(context);
    if (payload==nullptr || payload->plan==nullptr || (payload->region_count!=0 && payload->regions==nullptr))
        throw std::invalid_argument("Random seam received an invalid run context.");
    std::vector<PrintRegion> regions;
    for (uint32_t i=0;i<payload->region_count;++i) regions.emplace_back(payload->regions[i]);
    InitializationCancellation::check(context);
    payload->instance=make_seam_placer_instance(
        std::unique_ptr<SeamPlacerSession>(new RandomSession(
            context->plugin_storage, std::function<uint64_t()>(), context)),
                                                PrintingPlan(payload->plan),regions);
}
} // namespace

std::unique_ptr<SeamPlacerSession> make_random_seam_session(storage_handle *storage, std::function<uint64_t()> entropy)
{
    return std::unique_ptr<SeamPlacerSession>(new RandomSession(storage,std::move(entropy)));
}

void register_random_seam_placer_plugin(orchestrator_handle *orchestrator)
{
    static RandomProvider provider(orchestrator);
    orchestrator_register_plugin(orchestrator,provider.c_instance());
}
}}} // namespace slic3r_api::SeamPlacement::RandomPlugin
