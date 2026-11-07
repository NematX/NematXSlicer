///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Widen bridge areas onto nearby model regions and give their threads longer ends.
   BridgePreparation.cpp uses these polygon helpers before generating extrusion
   paths. Uniform growth makes edge gaps fall over lower material when space permits.

   Rotate the coordinates so s runs along the printing direction and t across it.
   At each t, the bridge and the allowed area are intervals along s. Extend each
   bridge interval at both ends, limited by the containing allowed interval.
   Between polygon vertex levels the bounds are straight lines, so only their
   crossings can change which bound stops the extension.

   expand_bridge() first clips a round uniform offset to the allowed area and
   keeps only components that meet the original core. axial_expand() then extends
   the ends of those rows without adding rows beside them. Its sweep builds
   strips between vertex/crossing levels, clips the rounded result to the allowed
   area, and retains the exact input core where cleanup removes a tiny tip.

   Representative direct calls, not a timeline:

       expand_bridge()
           |-- unite() [assemble allowed and connected areas]
           |-- clipper_offset() [uniform growth]
           |-- intersect() [clip growth and test connection to the core]
           `-- axial_expand()
               |-- append_edges() [rotate boundaries and collect vertex levels]
               |-- intervals_at() [find filled intervals within a sweep band]
               |-- add_crossing() [split where the limiting boundary changes]
               |-- unite() [combine extended strips with the input core]
               `-- intersect() [clip rounded geometry to the allowed area]

       geometry_key()
           `-- geometry_order() [normalize ring starts and sort rings]

       geometry_less()
           `-- geometry_order() [compare the same normalized geometry sequences]

   The normalized order makes grouping independent of input contour enumeration;
   it does not discard winding or change coordinates. XY coordinates and margins
   use scaled integer units; angles use radians. Inputs stay unchanged. Temporary
   and returned geometry belong to the caller's storage and must be released before
   that storage. Work vectors are local to each call; no threads are created here.
*/
#include "BridgeGeometry.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace slic3r_api { namespace BridgeGeometry {
namespace {
// Coordinates along (s) and across (t) the bridge axis, in scaled XY units.
struct Vertex { double s, t; };
struct Edge {
    Vertex a, b;
    // Interpolate the axial boundary position at transverse level t.
    double at(double t) const { return a.s+(b.s-a.s)*(t-a.t)/(b.t-a.t); }
};
struct Interval { Edge left, right; };
using GeometryOrder=std::vector<std::vector<std::pair<coord_t,coord_t>>>;
// Normalize contour starts and collection order without relying on a hash value.
GeometryOrder geometry_order(const ExPolygonCollection &areas);
// Rotate contour and hole edges into s/t coordinates and record every vertex's t.
void append_edges(const ExPolygonCollection &areas, double c, double s, std::vector<Edge> &edges, std::vector<double> &events);
// Pair crossings of normalized polygons at an interior scanline; an odd count
// throws because it cannot describe disjoint filled intervals.
std::vector<Interval> intervals_at(const std::vector<Edge> &edges, double t);
// Add the t at which a shifted straight boundary crosses another boundary,
// but only inside the band from lo to hi.
void add_crossing(std::vector<double> &events, const Edge &a, double shift, const Edge &b, double lo, double hi);

void append_edges(const ExPolygonCollection &areas, double c, double s, std::vector<Edge> &edges, std::vector<double> &events)
{
    for (const ExPolygon area : areas) {
        std::vector<Polygon> rings{area.contour()};
        for (const Polygon hole : area.holes())
            rings.push_back(hole);
        for (const Polygon ring : rings) {
            const std::vector<c_point> points = ring.points();
            for (size_t idx = 0; idx < points.size(); ++idx) {
                const c_point a = points[idx], b = points[(idx+1)%points.size()];
                Edge edge{{c*a.x+s*a.y, -s*a.x+c*a.y}, {c*b.x+s*b.y, -s*b.x+c*b.y}};
                events.push_back(edge.a.t);
                // An edge on a sweep level does not cross an interior scanline.
                if (edge.a.t != edge.b.t)
                    edges.push_back(edge);
            }
        }
    }
}

std::vector<Interval> intervals_at(const std::vector<Edge> &edges, double t)
{
    std::vector<Edge> active;
    for (const Edge &edge : edges)
        if (t > std::min(edge.a.t,edge.b.t) && t < std::max(edge.a.t,edge.b.t))
            active.push_back(edge);
    std::sort(active.begin(), active.end(), [t](const Edge &a, const Edge &b) { return a.at(t)<b.at(t); });
    if (active.size()%2 != 0)
        throw std::runtime_error("An axial bridge scanline has an odd boundary count.");
    std::vector<Interval> result;
    for (size_t idx=0; idx<active.size(); idx+=2)
        result.push_back({active[idx],active[idx+1]});
    return result;
}

void add_crossing(std::vector<double> &events, const Edge &a, double shift, const Edge &b, double lo, double hi)
{
    const double first=a.at(lo)+shift-b.at(lo), last=a.at(hi)+shift-b.at(hi);
    if ((first<0. && last>0.) || (first>0. && last<0.))
        events.push_back(lo+(hi-lo)*first/(first-last));
}

GeometryOrder geometry_order(const ExPolygonCollection &areas)
{
    GeometryOrder rings;
    for (const ExPolygon area : areas) {
        std::vector<Polygon> source{area.contour()};
        for (const Polygon hole : area.holes()) source.push_back(hole);
        for (const Polygon ring : source) {
            std::vector<std::pair<coord_t,coord_t>> points;
            for (const c_point point : ring.points()) points.emplace_back(point.x,point.y);
            if (points.empty()) continue;
            std::rotate(points.begin(),std::min_element(points.begin(),points.end()),points.end());
            rings.push_back(std::move(points));
        }
    }
    std::sort(rings.begin(),rings.end());
    return rings;
}
}

StoredExPolygonCollection unite(storage_handle *storage, const ExPolygonCollection &areas)
{
    if (areas.empty()) return StoredExPolygonCollection(storage);
    ClipperContext clipper(storage);
    return clipper_union(clipper(areas)).to_expolygon_collection();
}
StoredExPolygonCollection subtract(storage_handle *storage, const ExPolygonCollection &a, const ExPolygonCollection &b)
{
    if (a.empty()) return StoredExPolygonCollection(storage);
    if (b.empty()) return a.clone(storage);
    ClipperContext clipper(storage);
    return clipper_diff(clipper(a),clipper(b)).to_expolygon_collection();
}
StoredExPolygonCollection intersect(storage_handle *storage, const ExPolygonCollection &a, const ExPolygonCollection &b)
{
    if (a.empty() || b.empty()) return StoredExPolygonCollection(storage);
    ClipperContext clipper(storage);
    return clipper_intersection(clipper(a),clipper(b)).to_expolygon_collection();
}

StoredExPolygonCollection expand_bridge(storage_handle *storage, const ExPolygonCollection &core,
    const ExPolygonCollection &domain, double angle, coord_t axial_margin, coord_t uniform_margin,
    const plugin_run_context *ctx)
{
    if (uniform_margin<=0 || core.empty()) return axial_expand(storage,core,domain,angle,axial_margin,ctx);
    throw_if_cancelled(ctx);
    StoredExPolygonCollection allowed=domain.clone(storage);
    allowed.append_copy_from(core);
    allowed=unite(storage,allowed.readonly());
    ClipperContext clipper(storage);
    // Sampling arcs finer than the boolean cleanup epsilon lets repeated clips
    // remove their tangent endpoints. Use the same geometric precision for both.
    StoredExPolygonCollection grown=clipper_offset(clipper(core),double(uniform_margin),CLIPPER_JOIN_ROUND,double(SCALED_EPSILON)).to_expolygon_collection();
    grown=intersect(storage,grown.readonly(),allowed.readonly());
    StoredExPolygonCollection connected=core.clone(storage);
    // Clipping an offset may leave a detached patch across a forbidden area.
    // Only components still meeting an original core may become bridge infill.
    for (const ExPolygon area : grown.readonly()) {
        throw_if_cancelled(ctx);
        StoredExPolygonCollection single(storage);single.push_back(area);
        if (!intersect(storage,single.readonly(),core).empty()) connected.push_back(area);
    }
    connected=unite(storage,connected.readonly());
    return axial_expand(storage,connected.readonly(),allowed.readonly(),angle,axial_margin,ctx);
}

StoredExPolygonCollection axial_expand(storage_handle *storage, const ExPolygonCollection &core,
    const ExPolygonCollection &domain, double angle, coord_t margin, const plugin_run_context *ctx)
{
    if (margin<=0 || core.empty()) return core.clone(storage);
    StoredExPolygonCollection allowed=domain.clone(storage);
    allowed.append_copy_from(core);
    allowed=unite(storage,allowed.readonly());
    const double c=std::cos(angle), s=std::sin(angle);
    std::vector<Edge> core_edges, domain_edges;
    std::vector<double> events;
    append_edges(core,c,s,core_edges,events);
    append_edges(allowed.readonly(),c,s,domain_edges,events);
    std::sort(events.begin(),events.end());
    events.erase(std::unique(events.begin(),events.end()),events.end());
    StoredExPolygonCollection pieces(storage);
    pieces.append_copy_from(core);
    // Vertex levels fix edge ordering; only a margin/boundary crossing can
    // change which linear bound clips an extension inside one such band.
    for (size_t idx=1; idx<events.size(); ++idx) {
        if ((idx&63u)==0) throw_if_cancelled(ctx);
        const double lo=events[idx-1], hi=events[idx], middle=(lo+hi)*.5;
        // Rotation can leave numerically zero-height bands between equal grid levels.
        if (hi-lo<1e-7) continue;
        const std::vector<Interval> cores=intervals_at(core_edges,middle), domains=intervals_at(domain_edges,middle);
        for (const Interval &seed : cores) {
            const Interval *containing=nullptr;
            for (const Interval &candidate : domains)
                if (candidate.left.at(middle)<=seed.left.at(middle)+SCALED_EPSILON &&
                    candidate.right.at(middle)>=seed.right.at(middle)-SCALED_EPSILON) {
                    containing=&candidate;
                    break;
                }
            if (containing==nullptr) {
                // Polygon normalization may remove a sub-epsilon tip created
                // by rotating integer vertices. Keep its core, without anchors.
                if (hi-lo<=SCALED_EPSILON) continue;
                throw std::runtime_error("The bridge core is outside its allowed axial domain at slab "+
                    std::to_string(lo)+".."+std::to_string(hi)+".");
            }
            std::vector<double> cuts{lo,hi};
            add_crossing(cuts,seed.left,-double(margin),containing->left,lo,hi);
            add_crossing(cuts,seed.right,double(margin),containing->right,lo,hi);
            std::sort(cuts.begin(),cuts.end());
            for (size_t part=1; part<cuts.size(); ++part) {
                const double a=cuts[part-1], b=cuts[part];
                const double l0=std::max(seed.left.at(a)-margin,containing->left.at(a));
                const double l1=std::max(seed.left.at(b)-margin,containing->left.at(b));
                const double r0=std::min(seed.right.at(a)+margin,containing->right.at(a));
                const double r1=std::min(seed.right.at(b)+margin,containing->right.at(b));
                StoredExPolygon quad(storage);
                polygon_handle *contour=expolygon_contour(quad.mutable_handle());
                multipoint_handle *points=polygon_as_multipoint(contour);
                std::vector<c_point> vertices;
                for (const Vertex p : {Vertex{l0,a},Vertex{r0,a},Vertex{r1,b},Vertex{l1,b}}) {
                    const c_point point{coord_t(std::llround(c*p.s-s*p.t)),coord_t(std::llround(s*p.s+c*p.t))};
                    if (vertices.empty() || vertices.back().x!=point.x || vertices.back().y!=point.y) vertices.push_back(point);
                }
                if (vertices.size()>1 && vertices.front().x==vertices.back().x && vertices.front().y==vertices.back().y) vertices.pop_back();
                if (vertices.size()<3) continue;
                for (const c_point point : vertices) multipoint_push_back(points,point);
                if (std::abs(quad.area())>0.) pieces.push_back_move(std::move(quad));
            }
            if (pieces.size()>1000000) throw std::runtime_error("Axial bridge expansion exceeds one million pieces.");
        }
    }
    // Clip rounded trapezoids to the allowed domain, then retain the exact
    // original core even where polygon cleanup removes a tiny boundary tip.
    StoredExPolygonCollection merged=unite(storage,pieces.readonly());
    StoredExPolygonCollection clipped=intersect(storage,merged.readonly(),allowed.readonly());
    clipped.append_copy_from(core);
    return unite(storage,clipped.readonly());
}

uint64_t geometry_key(const ExPolygonCollection &areas)
{
    const GeometryOrder rings=geometry_order(areas);
    uint64_t key=1469598103934665603ull;
    for (const std::vector<std::pair<coord_t,coord_t>> &ring : rings) {
        key=(key^ring.size())*1099511628211ull;
        for (const std::pair<coord_t,coord_t> &p : ring) {
            key=(key^uint64_t(p.first))*1099511628211ull;
            key=(key^uint64_t(p.second))*1099511628211ull;
        }
    }
    return key;
}

bool geometry_less(const ExPolygonCollection &a, const ExPolygonCollection &b)
{
    return geometry_order(a)<geometry_order(b);
}
}}
