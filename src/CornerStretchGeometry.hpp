/*
 * Copyright (c) 2026 NematX
 * Released under the GNU Affero General Public License, version 3 or later.
 *
 * This header is the host-independent geometry kernel for corner stretch.
 * CornerStretch.cpp is the SDK adapter: it supplies an ordered loop as edges,
 * reads slicer settings, and serializes the result.  All coordinates and
 * lengths here use the slicer's scaled integer coordinate unit, not mm.  The
 * adapter converts only at the boundary; keeping that rule makes this file
 * usable without host headers.
 *
 * An Edge is one source segment.  A transparent run is a sequence of adjacent
 * eligible straight edges with the same process state (the effective flow,
 * speed, and property scope).  The kernel finds
 * corner candidates, grows collinear branches, creates displacement requests,
 * allocates each request proportionally without an iterative solver, and then
 * reduces the two displacement components.  u/v are unit directions, ki/ko
 * are retained fractions of the incoming/outgoing branch, and first/last/
 * vertex identify the original window.  owner and state preserve the source
 * leaf and uniform process-run provenance.
 *
 * A Fragment keeps a ratio: original length divided by new length.  Nominal
 * means unchanged original geometry, so ratio 1 is nominal; another ratio
 * means the deposited flow q (mm3/mm) must be
 * adjusted when the adapter rebuilds the leaf.  The kernel reconstructs
 * windows in their original edges, separates nominal and compensated pieces,
 * uses circular rounded transitions or straight fallbacks, snaps shared
 * endpoints, refits arc centres, and conserves volume per process state.  A
 * final ratio correction repairs endpoint quantization and serialization length
 * changes; it is not a second application of the corner deformation.
 */
#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace nematx_corner {
constexpr double pi = 3.14159265358979323846;

// A 2D point in scaled slicer units.  It is a calculation value, not a mm
// value; the SDK adapter performs the integer boundary conversion.
struct Point { double x=0, y=0; };

// Add two scaled-unit points or vectors component by component.
inline Point operator+(Point a, Point b) { return {a.x+b.x,a.y+b.y}; }

// Subtract two scaled-unit points or vectors component by component.
inline Point operator-(Point a, Point b) { return {a.x-b.x,a.y-b.y}; }

// Scale a point or vector without changing the coordinate unit.
inline Point operator*(Point a, double s) { return {a.x*s,a.y*s}; }

// Return the signed 2D cross product used for turn direction and area sign.
inline double cross(Point a, Point b) { return a.x*b.y-a.y*b.x; }

// Return the dot product used for projections, angles, and collinearity.
inline double dot(Point a, Point b) { return a.x*b.x+a.y*b.y; }

// Return Euclidean length in scaled units.
inline double norm(Point a) { return std::hypot(a.x,a.y); }

// Snap a continuous kernel point to the nearest scaled coordinate.  This is
// used for shared serialization endpoints, not for the main continuous math.
inline Point quantize(Point a) { return {std::round(a.x),std::round(a.y)}; }

// A source edge represented as a straight segment or a circular arc.  Sweep
// is signed radians; zero means a straight segment.
struct Curve {
    Point a,b,center;
    double sweep=0; // signed radians; zero is a straight segment

    // Measure the curve in scaled units: chord length for a line, or radius
    // times absolute sweep for an arc.
    double length() const { return sweep == 0 ? norm(b-a) : norm(a-center)*std::abs(sweep); }

    // Return the point at normalized position t, clamping the two endpoints.
    // Straight curves interpolate linearly; arcs rotate around center.
    Point at(double t) const {
        if(t<=0) return a;
        if(t>=1) return b;
        if(sweep==0) return a+(b-a)*t;
        const Point v=a-center; const double c=std::cos(sweep*t),s=std::sin(sweep*t);
        return center+Point{v.x*c-v.y*s,v.x*s+v.y*c};
    }
};

// Settings selected at one original corner.  Distances and deviations are
// already in scaled units when the adapter passes them to the kernel.
struct Settings {
    bool enabled=false, arcs=true;
    double angle=120, distance=0, deviation=0, entering=1, exiting=1;
};

// One ordered source edge, with its owning leaf and process-state identity.
// eligible says whether this edge may participate in a corner request.
struct Edge { Curve curve; unsigned owner=0, state=0; bool eligible=false; };

// One output piece of an original edge.  ratio is original length/new length;
// ratio 1 marks nominal geometry and another value marks compensation.
struct Fragment { Curve curve; double ratio=1; };

// One rounded-transition piece.  compensated distinguishes the part whose
// flow must be adjusted from nominal straight extension geometry.
struct Bend { Curve curve; bool compensated=true; };

// Build a circular rounded transition (fillet) between two rays.  Unequal legs
// retain nominal straight ends, while impossible or disabled arcs fall back to
// one straight curve from a to b.  Inputs are scaled-unit points; eps is the
// geometric tolerance; output pieces are ordered from a to b.
inline std::vector<Bend> raccord(Point a, Point c, Point b, bool arcs, double eps) {
    const double la=norm(c-a),lb=norm(b-c);
    if(!arcs || la<=eps || lb<=eps) return {{{a,b,{},0},true}};
    const Point u=(c-a)*(1/la),v=(b-c)*(1/lb);
    const double turn=std::atan2(cross(u,v),dot(u,v));
    if(std::abs(turn)<1e-6 || std::abs(turn)>pi-1e-6) return {{{a,b,{},0},true}};
    const double d=std::min(la,lb);
    const Point s=quantize(c-u*d),e=quantize(c+v*d);
    const Point normal{-u.y,u.x};
    const double denominator=2*dot(e-s,normal);
    if(std::abs(denominator)<eps) return {{{a,b,{},0},true}};
    const Point center=s+normal*(dot(e-s,e-s)/denominator);
    double sweep=std::atan2(cross(s-center,e-center),dot(s-center,e-center));
    if(sweep*turn<=0 || std::abs(sweep)>=pi) return {{{a,b,{},0},true}};
    std::vector<Bend> out;
    if(norm(a-s)>eps) out.push_back({{a,s,{},0},false});
    out.push_back({{s,e,center,sweep},true});
    if(norm(e-b)>eps) out.push_back({{e,b,{},0},false});
    return out;
}

// Stretch ordered source edges using settings evaluated at each edge's end.
// The function grows transparent collinear branches, allocates overlapping
// requests proportionally without a solver, and returns fragments grouped by
// their original edge.  All coordinates/lengths use scaled units; endpoints
// are quantized and arc centres are refit before volume is corrected per state.
inline std::vector<std::vector<Fragment>> stretch(const std::vector<Edge>& edges,
                                                 const std::vector<Settings>& settings,
                                                 double eps) {
    if(settings.size()!=edges.size()) throw std::invalid_argument("Corner settings size mismatch");

    // A patch is a window of one original edge covered by a replacement path.
    // from/to are normalized positions in that edge.
    struct Patch { double from,to; std::vector<Fragment> fragments; };

    std::vector<std::vector<Patch>> patches(edges.size());

    // A corner request records its original vertex and branch limits.  first,
    // last, and vertex are edge indices; u/v are incoming/outgoing directions;
    // ki/ko are the fractions retained after sharing space with other corners.
    struct Corner { size_t vertex,first,last; double in=0,out=0,ki=1,ko=1; Point u,v; Settings cfg; };
    std::vector<Corner> corners;

    // Two adjacent edges are transparent only when a deformation can pass
    // through them without crossing a process/property barrier.
    auto transparent=[&](size_t a,size_t b) {
        return edges[a].eligible && edges[b].eligible && edges[a].state==edges[b].state &&
            edges[a].curve.sweep==0 && edges[b].curve.sweep==0 && norm(edges[a].curve.b-edges[b].curve.a)==0;
    };

    // Collinear means same forward direction within scaled geometry tolerance;
    // it lets a corner request use a branch split into several source edges.
    auto collinear=[&](size_t a,size_t b) {
        Point u=edges[a].curve.b-edges[a].curve.a,v=edges[b].curve.b-edges[b].curve.a;
        return dot(u,v)>0 && norm(u)>0 && std::abs(cross(u,v))<=1e-3*norm(u)*norm(v) &&
            std::abs(cross(u,v))/norm(u+v)<=eps;
    };

    // Evaluate candidate vertices.  Settings are attenuated by the corner
    // angle here, and a request below the minimum distance is discarded.
    for(size_t i=0;i+1<edges.size();++i) {
        const Settings s=settings[i];
        if(!s.enabled || !transparent(i,i+1) || collinear(i,i+1)) continue;
        const Point a=edges[i].curve.a,c=edges[i].curve.b,b=edges[i+1].curve.b;
        const double l=norm(c-a),r=norm(b-c);
        if(l<=eps || r<=eps) continue;
        const Point u=(c-a)*(1/l),v=(c-b)*(1/r);
        const double theta=std::acos(std::clamp(dot(u,v),-1.,1.))*180/pi;
        const double threshold=s.angle<=0 || s.angle>180 ? 180:s.angle;
        if(theta>=threshold) continue;
        const double alpha=threshold<=90?1:(std::max(theta,90.)-threshold)/(90-threshold);
        Settings cfg=s; cfg.distance*=alpha;
        // deviation is already evaluated by the caller using the attenuated distance.
        if(cfg.distance<10*eps) continue;
        size_t first=i,last=i+1;
        // Extend over transparent collinear splits, but never across an event,
        // state change, arc, gap, or other non-transparent edge.
        while(first>0 && transparent(first-1,first) && collinear(first-1,first)) --first;
        while(last+1<edges.size() && transparent(last,last+1) && collinear(last,last+1)) ++last;
        corners.push_back({i,first,last,cfg.deviation*cfg.entering,cfg.deviation*cfg.exiting,1,1,u,v,cfg});
    }

    // Allocate each straight branch once.  Each branch is capped by its real
    // available length, and adjacent requests share a close branch by one
    // proportional factor instead of solving an iterative system.
    for(size_t k=0;k<corners.size();++k) {
        auto &c=corners[k]; double in=0,out=0;
        for(size_t i=c.first;i<=c.vertex;++i) in+=edges[i].curve.length();
        for(size_t i=c.vertex+1;i<=c.last;++i) out+=edges[i].curve.length();
        if(c.in>in) c.ki=in/c.in;
        if(c.out>out) c.ko=out/c.out;
        if(k>0) {
            auto &p=corners[k-1];
            if(p.last==c.vertex && c.first==p.vertex+1) {
                const double sum=p.out+c.in;
                const double factor=sum>0?std::min(1.,in/sum):1.;
                p.ko=factor; c.ki=factor;
            }
        }
    }

    // Copy a requested replacement path into the windows of original edges.
    // The path can contain nominal and compensated pieces; only the latter get
    // the supplied ratio.  Empty or zero-length requests are ignored.
    auto distribute=[&](size_t first,size_t last,double start,double amount,const std::vector<Bend>& path,double ratio) {
        double path_length=0; for(const auto &p:path) path_length+=p.curve.length();
        if(amount<=eps || path_length<=eps) return;
        double cursor=0;
        for(size_t i=first;i<=last;++i) {
            const double len=edges[i].curve.length();
            const double lo=std::max(start,cursor),hi=std::min(start+amount,cursor+len);
            if(hi>lo) {
                Patch patch{(lo-cursor)/len,(hi-cursor)/len,{}};
                const double begin=(lo-start)/amount*path_length,end=(hi-start)/amount*path_length;
                double pos=0;
                for(const auto &p:path) {
                    const double plen=p.curve.length(),a=std::max(begin,pos),b=std::min(end,pos+plen);
                    if(plen>0 && b>a) {
                        Curve part=p.curve; part.a=p.curve.at((a-pos)/plen); part.b=p.curve.at((b-pos)/plen);
                        part.sweep=p.curve.sweep*(b-a)/plen;
                        patch.fragments.push_back({part,p.compensated?ratio:1});
                    }
                    pos+=plen;
                }
                patches[i].push_back(std::move(patch));
            }
            cursor+=len;
        }
    };

    // Construct each corner's displaced path.  The two displacement components
    // are reduced independently by ki/ko, then distributed back to source
    // leaves through the incoming and outgoing windows.
    for(const auto &c:corners) {
        const Point center=edges[c.vertex].curve.b;
        const double incoming=c.in*c.ki,outgoing=c.out*c.ko;
        if(incoming+outgoing<=eps) continue;
        const Point s=quantize(center-c.u*incoming),e=quantize(center-c.v*outgoing);
        const Point moved=quantize(center+(c.u*(c.cfg.exiting*c.ko)+c.v*(c.cfg.entering*c.ki))*(c.cfg.distance/2));
        auto left=raccord(s,center,moved,c.cfg.arcs,eps),right=raccord(moved,center,e,c.cfg.arcs,eps);
        // Only the outer extensions are nominal.  The two inner extensions
        // toward the displaced tip belong to the compensated detour.
        if(left.size()>1) left.back().compensated=true;
        if(right.size()>1) right.front().compensated=true;
        double nominal=0,changed=0;
        for(const auto *path:{&left,&right}) for(const auto &p:*path)
            (p.compensated?changed:nominal)+=p.curve.length();
        const double old=norm(s-center)+norm(e-center)-nominal;
        if(old<=eps || changed<=eps) continue;
        const double ratio=old/changed;
        if(!std::isfinite(ratio) || ratio<=0) throw std::runtime_error("Invalid corner volume ratio");
        double before=0; for(size_t i=c.first;i<=c.vertex;++i) before+=edges[i].curve.length();
        // A one-sided branch still receives the complete ordered path; a
        // two-sided corner distributes each side into its own original window.
        if(incoming<=eps) {
            left.insert(left.end(),right.begin(),right.end());
            distribute(c.vertex+1,c.last,0,outgoing,left,ratio);
        } else if(outgoing<=eps) {
            left.insert(left.end(),right.begin(),right.end());
            distribute(c.first,c.vertex,before-incoming,incoming,left,ratio);
        } else {
            distribute(c.first,c.vertex,before-incoming,incoming,left,ratio);
            distribute(c.vertex+1,c.last,0,outgoing,right,ratio);
        }
    }

    // Fill every original edge's untouched intervals around its sorted
    // replacement windows.  Overlapping windows are an internal geometry error
    // rather than silently dropped output.
    std::vector<std::vector<Fragment>> result(edges.size());
    for(size_t i=0;i<edges.size();++i) {
        auto &ps=patches[i]; std::sort(ps.begin(),ps.end(),[](const Patch&a,const Patch&b){return a.from<b.from;});
        double cursor=0;
        auto normal=[&](double a,double b) { if(b>a) { Curve c=edges[i].curve;c.a=c.at(a);c.b=edges[i].curve.at(b);c.sweep*=b-a;result[i].push_back({c,1}); } };
        for(const auto &p:ps) { if(p.from<cursor-1e-9) throw std::runtime_error("Overlapping corner windows");normal(cursor,p.from);result[i].insert(result[i].end(),p.fragments.begin(),p.fragments.end());cursor=p.to; }
        normal(cursor,1);
    }

    // Use common quantized endpoints, including competing windows.  Refit arcs
    // after snapping so their endpoints still lie on the same circle; this
    // keeps adjacent fragments connected after SDK integer serialization.
    Point previous{};bool have_previous=false;
    for(size_t i=0;i<result.size();++i) {
        if(i && norm(edges[i-1].curve.b-edges[i].curve.a)!=0) have_previous=false;
        for(auto &f:result[i]) {
            Point a=quantize(f.curve.a),b=quantize(f.curve.b);
            if(have_previous && norm(a-previous)<=2*eps+2) a=previous;
            f.curve.a=a;f.curve.b=b;
            if(f.curve.sweep!=0) {
                const Point d=b-a;
                f.curve.center=(a+b)*.5+Point{-d.y,d.x}*(.5/std::tan(f.curve.sweep/2));
            }
            previous=b;have_previous=true;
        }
    }

    // Snapping and provenance splits perturb lengths slightly.  Restore volume
    // within each uniform process run, changing only compensated portions; the
    // adapter later serializes these ratios into flow and speed properties.
    unsigned max_state=0;for(const auto&e:edges) max_state=std::max(max_state,e.state);
    // Volume totals separate original length from nominal and compensated
    // output so the nominal path is never used to hide a correction.
    struct Volume { double old=0,normal=0,compensated=0; };
    std::vector<Volume> volumes(size_t(max_state)+1);
    for(size_t i=0;i<edges.size();++i) {
        auto &v=volumes[edges[i].state];v.old+=edges[i].curve.length();
        for(const auto&f:result[i]) (f.ratio==1?v.normal:v.compensated)+=f.ratio*f.curve.length();
    }
    for(size_t i=0;i<edges.size();++i) {
        const auto &v=volumes[edges[i].state];
        if(v.compensated>0) {
            const double correction=(v.old-v.normal)/v.compensated;
            if(!std::isfinite(correction)||correction<=0) throw std::runtime_error("Invalid quantized corner volume");
            for(auto&f:result[i]) if(f.ratio!=1) f.ratio*=correction;
        }
    }
    return result;
}
}
