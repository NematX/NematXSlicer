///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Calculate where a bridge thread overlaps lower material and how much its
   extrusion cross section can be reduced to avoid that overlap.

   A lower extrusion occupies its thickened XY path between its bottom and top Z.
   The nearest such top limits the vertical space below the bridge. The calculation
   uses lower paths supplied by BridgeVolume.cpp, not whole filled model slices.
   Thread width stays fixed while height and area decrease. A flattened section
   consists of a rectangle with semicircular ends (see area()).

   Index::add() records copies of lower extrusion descriptions. Its R-tree uses
   XY boxes only to select candidates; capsule_range() tests their round-ended
   footprints. clearance() widens the query by the bridge radius, while contacts()
   uses its centerline. The highest intersecting occupied top determines each gap.

   Representative direct calls:

       Index::clearance(), Index::contacts()
           `-- Impl::query()
               `-- capsule_range()

       section_pieces()
           `-- ramp_domain() [each contiguous range of known intervals]
               |-- area() [gap cap and printable floor]
               `-- height_for_area() [recover height from the assigned area]

   Known flow changes approach a constrained section from both sides, over at most
   one thread width, with constant-area steps no larger than 5% of nominal area.
   Unknown intervals break those transitions and retain the original circle.
   coalesce_short_sections() can absorb a known sub-grid piece only into a neighbor
   with no greater area or height, retaining the strictest cap and the neighbor's flow.

   All geometry is in mm and cross-sectional areas are in mm2. The index owns
   copies of the lower-path descriptions and borrows only the run context used
   to check cancellation. Calls are sequential and create no threads.
*/
#include "BridgeClearance.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace slic3r_api { namespace BridgeClearance {
namespace bg=boost::geometry;
namespace bgi=boost::geometry::index;
namespace {
using Point=bg::model::point<double,2,bg::cs::cartesian>;
using Box=bg::model::box<Point>;
using Entry=std::pair<Box,size_t>;
struct Range { double begin,end,top; bool unknown; };
// Return a clipped distance range on target a-b, including circular end caps;
// discard overlaps no longer than 1e-10 mm without changing stored footprints.
bool capsule_range(c_vec2d a,c_vec2d b,const Occupation &obstacle,double inflate,double &begin,double &end);
// Choose sections for a path portion whose lower material is known. Approach
// smaller sections gradually from both sides, but never print below the height floor.
void ramp_domain(std::vector<SectionPiece> &result,const std::vector<Interval> &input,size_t first,size_t last,double width,double floor);

bool capsule_range(c_vec2d a,c_vec2d b,const Occupation &obstacle,double inflate,double &begin,double &end)
{
    const double dx=b.x-a.x,dy=b.y-a.y,length=std::hypot(dx,dy);
    if (length<=0.) return false;
    const double ux=dx/length,uy=dy/length,r=obstacle.radius+inflate;
    double lo=std::numeric_limits<double>::infinity(),hi=-lo;
    // A capsule is the union of its end disks and the rectangle between them.
    for (const c_vec2d p : {obstacle.a,obstacle.b}) {
        const double along=(p.x-a.x)*ux+(p.y-a.y)*uy;
        const double cross=(p.x-a.x)*uy-(p.y-a.y)*ux;
        const double square=r*r-cross*cross;
        if (square>0.) { const double span=std::sqrt(square); lo=std::min(lo,along-span); hi=std::max(hi,along+span); }
    }
    const double vx=obstacle.b.x-obstacle.a.x,vy=obstacle.b.y-obstacle.a.y,vlen=std::hypot(vx,vy);
    if (vlen>0.) {
        double left=0.,right=length;
        const double tx=vx/vlen,ty=vy/vlen;
        const double p=(a.x-obstacle.a.x)*tx+(a.y-obstacle.a.y)*ty;
        const double q=(a.x-obstacle.a.x)*(-ty)+(a.y-obstacle.a.y)*tx;
        const double dp=ux*tx+uy*ty,dq=-ux*ty+uy*tx;
        // A parallel line either stays inside this slab or misses it; do not
        // divide by its nearly zero projected slope.
        const auto clip=[&left,&right](double initial,double slope,double minimum,double maximum) {
            if (std::abs(slope)<1e-14) return initial>=minimum && initial<=maximum;
            double x=(minimum-initial)/slope,y=(maximum-initial)/slope;
            if (x>y) std::swap(x,y);
            left=std::max(left,x);right=std::min(right,y);
            return right>left;
        };
        if (clip(p,dp,0.,vlen) && clip(q,dq,-r,r) && right>left) { lo=std::min(lo,left);hi=std::max(hi,right); }
    }
    begin=std::max(0.,lo);end=std::min(length,hi);
    return end-begin>1e-10;
}
}

struct Index::Impl {
    std::vector<Occupation> entries;
    bgi::rtree<Entry,bgi::quadratic<16>> tree;
    const plugin_run_context *context=nullptr;
    // Find lower paths whose occupied space reaches a-b, using XY boxes to skip
    // distant paths. An unknown shape or a top reaching bridge Z prevents a known gap.
    std::vector<Range> query(c_vec2d a,c_vec2d b,double inflate,double height,double z) const;
};

Index::Index(const plugin_run_context *context) : m_impl(std::make_unique<Impl>()) { m_impl->context=context; }
Index::~Index()=default;
void Index::add(const Occupation &o)
{
    if (!(o.radius>0.) || !std::isfinite(o.z_high) || !std::isfinite(o.z_low))
        throw std::runtime_error("Invalid occupied extrusion prism.");
    const size_t index=m_impl->entries.size();
    if ((index&255u)==0) throw_if_cancelled(m_impl->context);
    m_impl->entries.push_back(o);
    m_impl->tree.insert({Box(Point(std::min(o.a.x,o.b.x)-o.radius,std::min(o.a.y,o.b.y)-o.radius),
        Point(std::max(o.a.x,o.b.x)+o.radius,std::max(o.a.y,o.b.y)+o.radius)),index});
}
std::vector<Range> Index::Impl::query(c_vec2d a,c_vec2d b,double inflate,double height,double z) const
{
    std::vector<Entry> candidates;
    const Box box(Point(std::min(a.x,b.x)-inflate,std::min(a.y,b.y)-inflate),Point(std::max(a.x,b.x)+inflate,std::max(a.y,b.y)+inflate));
    tree.query(bgi::intersects(box),std::back_inserter(candidates));
    std::sort(candidates.begin(),candidates.end(),[](const Entry &a,const Entry &b) { return a.second<b.second; });
    std::vector<Range> result;
    for (const Entry &entry : candidates) {
        throw_if_cancelled(context);
        const Occupation &o=entries[entry.second];
        if (o.z_high<=z-height || o.z_low>=z) continue;
        double begin=0.,end=0.;
        if (capsule_range(a,b,o,inflate,begin,end)) result.push_back({begin,end,o.z_high,o.unknown || o.z_high>=z-1e-9});
    }
    return result;
}
std::vector<Interval> Index::clearance(c_vec2d a,c_vec2d b,double width,double height,double z) const
{
    const std::vector<Range> ranges=m_impl->query(a,b,width*.5,height,z);
    std::vector<double> events{0.,std::hypot(b.x-a.x,b.y-a.y)};
    for (const Range &r : ranges) { events.push_back(r.begin);events.push_back(r.end); }
    std::sort(events.begin(),events.end());events.erase(std::unique(events.begin(),events.end()),events.end());
    std::vector<Interval> result;
    // Every footprint endpoint is an event; between events the highest occupied
    // Z and the set of unknown obstacles are constant.
    for (size_t idx=1; idx<events.size(); ++idx) {
        if ((idx&63u)==0) throw_if_cancelled(m_impl->context);
        const double middle=(events[idx-1]+events[idx])*.5;
        double top=-std::numeric_limits<double>::infinity();bool unknown=false;
        for (const Range &r : ranges) if (middle>r.begin && middle<r.end) { top=std::max(top,r.top);unknown=unknown||r.unknown; }
        result.push_back({events[idx-1],events[idx],z-top,unknown});
    }
    return result;
}
std::vector<std::pair<double,double>> Index::contacts(c_vec2d a,c_vec2d b,double height,double z) const
{
    const std::vector<Range> ranges=m_impl->query(a,b,0.,height+1e-9,z);
    std::vector<std::pair<double,double>> result;
    for (const Range &r : ranges) if (!r.unknown) result.emplace_back(r.begin,r.end);
    std::sort(result.begin(),result.end());
    std::vector<std::pair<double,double>> merged;
    for (const std::pair<double,double> &r : result)
        if (merged.empty() || merged.back().second<r.first) merged.push_back(r);
        else merged.back().second=std::max(merged.back().second,r.second);
    return merged;
}

double area(double width,double height)
{
    return width*height-(1.-3.14159265358979323846/4.)*height*height;
}
double height_for_area(double width,double value)
{
    const double alpha=1.-3.14159265358979323846/4.;
    // Rationalize the smaller quadratic root to avoid cancellation for thin sections.
    return 2.*value/(width+std::sqrt(std::max(0.,width*width-4.*alpha*value)));
}
namespace {
void ramp_domain(std::vector<SectionPiece> &result,const std::vector<Interval> &input,size_t first,size_t last,double width,double floor)
{
    const double nominal=area(width,width),minimum=area(width,floor),slope=nominal/width;
    std::vector<double> knots(last-first+1),values(last-first+1),targets(last-first);
    for (size_t idx=first; idx<last; ++idx) {
        const double cap=area(width,std::clamp(input[idx].clearance,0.,width));
        targets[idx-first]=std::min(nominal,std::max(minimum,cap));
        knots[idx-first]=input[idx].begin;
    }
    knots.back()=input[last-1].end;
    for (size_t idx=0; idx<values.size(); ++idx)
        values[idx]=idx==0 ? targets.front() : idx==targets.size() ? targets.back() : std::min(targets[idx-1],targets[idx]);
    // Approach a constrained section from both ends so reversing the path gives
    // the same flow. The transition can extend at most one thread width away.
    for (size_t idx=1; idx<values.size(); ++idx) values[idx]=std::min(values[idx],values[idx-1]+slope*(knots[idx]-knots[idx-1]));
    for (size_t idx=values.size()-1; idx>0; --idx) values[idx-1]=std::min(values[idx-1],values[idx]+slope*(knots[idx]-knots[idx-1]));
    for (size_t idx=0; idx<targets.size(); ++idx) {
        const double begin=knots[idx],end=knots[idx+1],target=targets[idx];
        std::vector<double> cuts{begin,end};
        // Cut where either approach line reaches the plateau, or the two lines
        // meet, so each remaining span has one linear or constant minimum.
        for (const double cut : {begin+(target-values[idx])/slope,end-(target-values[idx+1])/slope,
                (values[idx+1]-values[idx]+slope*(begin+end))/(2.*slope)})
            if (cut>begin+1e-10 && cut<end-1e-10) cuts.push_back(cut);
        std::sort(cuts.begin(),cuts.end());
        const auto value=[&](double s) { return std::min({target,values[idx]+slope*(s-begin),values[idx+1]+slope*(end-s)}); };
        for (size_t part=1; part<cuts.size(); ++part) {
            const double a=cuts[part-1],b=cuts[part];
            // G-code holds flow constant within a move. Use the lower endpoint
            // value and steps of at most 5% of nominal area, never an average above it.
            const int count=std::max(1,int(std::ceil(std::abs(value(a)-value(b))/(.05*nominal))));
            for (int step=0; step<count; ++step) {
                const double lo=a+(b-a)*step/count,hi=a+(b-a)*(step+1)/count;
                const double q=std::max(minimum,std::min(value(lo),value(hi)));
                const Interval &source=input[first+idx];
                const double cap=area(width,std::clamp(source.clearance,0.,width));
                result.push_back({lo,hi,q,height_for_area(width,q),source.clearance,cap,
                    uint8_t(q>cap+1e-10 ? BRIDGE_CONFIDENCE_FLOOR_LIMITED : BRIDGE_CONFIDENCE_CERTIFIED)});
            }
        }
    }
}
}
std::vector<SectionPiece> section_pieces(const std::vector<Interval> &intervals,double diameter,double floor)
{
    if (!(diameter>0.) || !(floor>0.) || floor>diameter) throw std::invalid_argument("Invalid printable bridge height floor.");
    std::vector<SectionPiece> result;
    // Keep the original circle where the gap is unknown. A reduction from a known
    // part must not continue through an unknown part of the path.
    for (size_t idx=0; idx<intervals.size();) {
        if (intervals[idx].unknown) {
            result.push_back({intervals[idx].begin,intervals[idx].end,area(diameter,diameter),diameter,0.,0.,BRIDGE_CONFIDENCE_UNKNOWN});
            ++idx;
        } else {
            size_t last=idx+1;
            while (last<intervals.size() && !intervals[last].unknown) ++last;
            ramp_domain(result,intervals,idx,last,diameter,floor);
            idx=last;
        }
    }
    std::vector<SectionPiece> merged;
    for (const SectionPiece &piece : result) {
        if (!merged.empty()) {
            SectionPiece &previous=merged.back();
            if (previous.end==piece.begin && previous.area==piece.area && previous.height==piece.height &&
                previous.clearance==piece.clearance && previous.cap==piece.cap && previous.confidence==piece.confidence) {
                previous.end=piece.end;
                continue;
            }
        }
        merged.push_back(piece);
    }
    return merged;
}

std::vector<SectionPiece> coalesce_short_sections(const std::vector<SectionPiece> &pieces,double minimum_length)
{
    std::vector<SectionPiece> result=pieces;
    for (size_t idx=0; idx<result.size() && result.size()>1;) {
        const SectionPiece &piece=result[idx];
        if (piece.end-piece.begin>minimum_length || piece.confidence==BRIDGE_CONFIDENCE_UNKNOWN) { ++idx;continue; }
        size_t neighbor=result.size();
        // Extending an already smaller section changes only the tiny interval,
        // not the flow along a potentially long plateau beside it.
        for (const size_t candidate : {idx==0 ? result.size() : idx-1,idx+1})
            if (candidate<result.size() && result[candidate].confidence!=BRIDGE_CONFIDENCE_UNKNOWN &&
                result[candidate].area<=piece.area && result[candidate].height<=piece.height &&
                (neighbor==result.size() || result[candidate].area>result[neighbor].area)) neighbor=candidate;
        if (neighbor==result.size()) { ++idx;continue; }
        const size_t first=std::min(idx,neighbor),last=std::max(idx,neighbor);
        SectionPiece combined=result[neighbor];
        combined.begin=result[first].begin;combined.end=result[last].end;
        combined.clearance=std::min(combined.clearance,piece.clearance);
        combined.cap=std::min(combined.cap,piece.cap);
        const bool density=combined.confidence==BRIDGE_CONFIDENCE_DENSITY_ESTIMATE || combined.confidence==BRIDGE_CONFIDENCE_DENSITY_FLOOR_LIMITED;
        combined.confidence=combined.area>combined.cap+1e-10 ?
            (density ? BRIDGE_CONFIDENCE_DENSITY_FLOOR_LIMITED : BRIDGE_CONFIDENCE_FLOOR_LIMITED) :
            (density ? BRIDGE_CONFIDENCE_DENSITY_ESTIMATE : BRIDGE_CONFIDENCE_CERTIFIED);
        result[first]=combined;
        result.erase(result.begin()+last);
        idx=first==0 ? 0 : first-1;
    }
    return result;
}
}}
