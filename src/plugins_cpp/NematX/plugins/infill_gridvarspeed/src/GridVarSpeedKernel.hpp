///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ Copyright (c) 2026 NematX
///|/ Released under the GNU Affero General Public License, version 3 or later.
#pragma once

// Local adaptation of FillRectilinear::change_flow_intersection (2.7).
// Geometry and pure process policy: no host calls, G-code or mutable global state.
#include "rectilinear/WorkBuffers.hpp"
#include <stdexcept>

namespace nematx_gridvarspeed {
namespace geometry = slic3r_api::GridVarSpeedCopy::work;
using geometry::Point;
using geometry::Points;
using geometry::Polyline;
using geometry::Polylines;

constexpr double legacy_flow_multiplier = 0.776284;
struct Fragment { Points points; double factor = 1.; };
using Chain = std::vector<Fragment>;
using Chains = std::vector<Chain>;

// Legacy Line::point_at truncates coordinates, unlike rotation (which rounds).
inline Point point_at(Point a, Point b, double distance) {
    const double len = a.distance_to(b);
    if (len == 0.) return a;
    const double t = distance / len;
    return {coord_t(double(a.x()) + double(b.x()-a.x())*t),
            coord_t(double(a.y()) + double(b.y()-a.y())*t)};
}

inline bool intersection(Point a, Point b, Point c, Point d, Point &out) {
    const auto cross = [](Point u, Point v) { return double(u.x())*double(v.y())-double(u.y())*double(v.x()); };
    const Point u = b-a, v = d-c, w = a-c;
    const double denom = cross(u,v);
    // Legacy ignores collinear overlaps (not a point crossing).
    if (std::abs(denom) < EPSILON) return false;
    const double t = cross(v,w)/denom, s = cross(u,w)/denom;
    if (t < 0. || t > 1. || s < 0. || s > 1.) return false;
    out = a + Point(coord_t(t*u.x()), coord_t(t*u.y()));
    return true;
}

inline double length(const Points &p) {
    double result = 0.;
    for (size_t i=1; i<p.size(); ++i) result += p[i-1].distance_to(p[i]);
    return result;
}

inline void append(Chain &chain, Points points, double factor) {
    points.erase(std::unique(points.begin(), points.end()), points.end());
    if (points.size()<2) return;
    if (!std::isfinite(factor) || factor<=0. || factor>1.)
        throw std::runtime_error("GridVarSpeed: invalid flow factor");
    if (!chain.empty() && chain.back().points.back()!=points.front())
        throw std::runtime_error("GridVarSpeed: discontinuous fragment chain");
    // Legacy 1% flow merge: retain the preceding factor, not an average.
    if (!chain.empty() && std::abs(chain.back().factor-factor)<factor*0.01) {
        chain.back().points.insert(chain.back().points.end(), points.begin()+1, points.end());
    } else chain.push_back({std::move(points),factor});
}

inline void clip_end(Points &points, double distance) {
    while (points.size()>1 && distance>0.) {
        const Point b=points.back(), a=points[points.size()-2];
        const double len=a.distance_to(b);
        if (len<=distance) { distance-=len; points.pop_back(); }
        else { points.back()=point_at(a,b,len-distance); break; }
    }
}

inline void remove_near_points(Polylines &lines, coord_t spacing) {
    if (spacing<=0) throw std::runtime_error("GridVarSpeed: invalid flow spacing");
    const double sq=double(spacing)*spacing;
    const auto shortcut=[](Point a,Point b,Point c) {
        const double dx=double(c.x()-a.x()),dy=double(c.y()-a.y());
        const double bx=double(b.x()-a.x()),by=double(b.y()-a.y());
        const double norm=dx*dx+dy*dy,dot=bx*dx+by*dy;
        const double cross=bx*dy-by*dx;
        // The old unconditional removal could cut across a concavity/hole.
        // Keep all real turns; only collapse points on the same forward chord.
        return norm>0. && dot>=0. && dot<=norm && cross*cross<=double(SCALED_EPSILON)*SCALED_EPSILON*norm;
    };
    for (Polyline &line: lines) {
        if (line.points.empty()) continue;
        line.remove_duplicate_points();
        for (size_t i=1; i+1<line.size();) {
            if (line.points[i-1].distance_to_square(line.points[i])<sq &&
                shortcut(line.points[i-1],line.points[i],line.points[i+1]))
                line.points.erase(line.points.begin()+ptrdiff_t(i));
            else ++i;
        }
        if (line.size()>2 && line.points[line.size()-2].distance_to_square(line.points.back())<sq &&
            shortcut(line.points[line.size()-3],line.points[line.size()-2],line.points.back()))
            line.points.erase(line.points.end()-2);
    }
    lines.erase(std::remove_if(lines.begin(),lines.end(),[](const Polyline &p){return p.size()<2;}),lines.end());
}

// Integer rotation/intersection rounding can create sub-epsilon edges. The host
// ArcPolyline rejects those even though their length is nonzero. Snap across a
// flow boundary together (never independently on adjacent fragments), retaining
// the exact source endpoints. This changes geometry only within scaled epsilon.
inline void remove_tiny_segments(Chain &chain) {
    if (chain.empty()) return;
    const Point first=chain.front().points.front(),last=chain.back().points.back();
    Point cursor=first;
    Chain clean;
    const auto tiny=[](Point a,Point b) {
        return std::abs(a.x()-b.x())<=SCALED_EPSILON && std::abs(a.y()-b.y())<=SCALED_EPSILON;
    };
    for (const Fragment &f:chain) for (size_t i=1;i<f.points.size();++i) {
        Point end=f.points[i];
        if (tiny(cursor,end)) continue;
        append(clean,{cursor,end},f.factor);
        cursor=end;
    }
    if (!clean.empty() && cursor!=last) {
        auto &points=clean.back().points;
        points.back()=last;
        // If moving the final endpoint shrank the last edge, absorb its start.
        while (points.size()>2 && tiny(points[points.size()-2],points.back())) points.erase(points.end()-2);
        if (points.size()==2 && tiny(points.front(),points.back())) {
            clean.pop_back();
            if (!clean.empty()) clean.back().points.back()=last;
        }
    }
    chain=std::move(clean);
}

inline Chains split_crossings(const Polylines &lines, const Polylines &other, double ratio, coord_t spacing) {
    if (!std::isfinite(ratio) || ratio<0. || ratio>1. || spacing<=0)
        throw std::runtime_error("GridVarSpeed: ratio must be in [0,1], spacing positive");
    const coord_t half=coord_t(double(spacing/2)*ratio);
    const double transition=(1.+ratio)/2.;
    Chains result;
    for (const Polyline &line: lines) {
        if (line.size()<2) continue;
        Chain chain;
        if (ratio==1. || half==0) {
            append(chain,line.points,1.);
            remove_tiny_segments(chain);
            if (!chain.empty()) result.push_back(std::move(chain));
            continue;
        }
        Points pending{line.points.front()};
        for (size_t i=1; i<line.size(); ++i) {
            const Point a=line.points[i-1], b=line.points[i];
            if (a==b) continue;
            Points collisions;
            for (const Polyline &op: other) for (size_t j=1; j<op.size(); ++j) {
                Point hit;
                if (intersection(a,b,op.points[j-1],op.points[j],hit)) collisions.push_back(hit);
            }
            std::sort(collisions.begin(),collisions.end(),[&](Point p,Point q){return a.distance_to_square(p)<a.distance_to_square(q);});
            // Shared vertices must not be counted once per adjacent edge.
            Points unique;
            for (Point p: collisions)
                if (unique.empty() || unique.back().distance_to_square(p)>double(SCALED_EPSILON)*SCALED_EPSILON)
                    unique.push_back(p);
            collisions=std::move(unique);
            if (collisions.empty()) {pending.push_back(b); continue;}

            if (pending.back().distance_to(collisions.front())<SCALED_EPSILON+half) {
                pending.push_back(collisions.front());
                const double plen=length(pending);
                const double my_factor=plen>half ? 1.-(1.-ratio)*half/plen : ratio;
                // Preserve legacy near-start policy, but never extend beyond this
                // source segment or normalize a zero vector at a shared vertex.
                const Point c=collisions.front();
                pending.back()=point_at(c,b,std::min(double(half),c.distance_to(b)));
                collisions.front()=pending.back();
                // Extension may overtake other very close crossings: consume them
                // once rather than creating a backwards, outside-source segment.
                collisions.erase(std::remove_if(collisions.begin()+1,collisions.end(),[&](Point p){
                    return a.distance_to_square(p)<=a.distance_to_square(collisions.front());
                }),collisions.end());
                if (!chain.empty() && std::abs(chain.back().factor-my_factor)<0.01)
                    append(chain,pending,chain.back().factor);
                else append(chain,pending,ratio);
            } else {
                pending.push_back(collisions.front());
                clip_end(pending,half);
                append(chain,pending,1.);
            }
            // A collision exactly at the start can yield no preceding fragment.
            Point previous=chain.empty()?line.points.front():chain.back().points.back();
            for (size_t ci=0; ci<collisions.size(); ++ci) {
                const Point c=collisions[ci], next=ci+1<collisions.size()?collisions[ci+1]:b;
                Point end_accel=previous, start_decel, next_end;
                coord_t reduced_a=half,reduced_b=half,accel=0,decel=0;
                const double distance=previous.distance_to(c);
                if (distance>SCALED_EPSILON) {
                    if (distance<half+SCALED_EPSILON) reduced_a=coord_t(distance);
                    else {
                        const coord_t available=coord_t(distance-half);
                        if (available<half/2-SCALED_EPSILON) accel=available*2;
                        else {
                            coord_t steady=available-half/2;
                            accel=half;
                            if (steady<half/2) {accel=available-half/2; steady=half/2;}
                            Point end=point_at(previous,c,steady);
                            append(chain,{previous,end},1.);
                            previous=end;
                        }
                        reduced_a-=accel/2;
                        end_accel=point_at(previous,c,std::max(0.,previous.distance_to(c)-reduced_a));
                    }
                }
                if (c.distance_to(next)/2<half+SCALED_EPSILON) {
                    start_decel=(next+c)/2;
                    reduced_b=coord_t(c.distance_to(start_decel));
                    next_end=start_decel;
                } else {
                    decel=std::min(half/2,coord_t(next.distance_to(c)/2-half));
                    reduced_b-=decel;
                    decel*=2;
                    start_decel=point_at(end_accel,next,reduced_a+reduced_b);
                    next_end=point_at(start_decel,next,decel);
                }
                // Legacy computes changed_flow here but never applies it. Keep
                // the geometric merge, not that unused correction of attributes.
                if (accel>0 && accel<(reduced_a+reduced_b)/5) {
                    reduced_a+=accel; accel=0; end_accel=previous;
                }
                if (decel>0 && decel<(reduced_a+reduced_b)/5) {
                    reduced_b+=decel; decel=0; start_decel=next_end;
                }
                if (accel>0) append(chain,{previous,end_accel},transition);
                append(chain,{end_accel,start_decel},ratio);
                if (decel>0) append(chain,{start_decel,next_end},transition);
                previous=next_end;
            }
            // Preserve even a short positive tail: don't disconnect from b.
            append(chain,{previous,b},1.);
            pending={b};
        }
        // Fix the legacy >2 test: a valid two-point tail is printable too.
        append(chain,std::move(pending),1.);
        remove_tiny_segments(chain);
        if (!chain.empty()) result.push_back(std::move(chain));
    }
    return result;
}

// Pure process policy, tested independently of config resolution.
inline double adjusted_speed(double speed,double factor,double cap) {
    if (!std::isfinite(speed) || speed<=0. || !std::isfinite(factor) || factor<=0. || factor>1. ||
        std::isnan(cap) || cap<=0.) throw std::runtime_error("GridVarSpeed: invalid process speed");
    const double candidate=std::max(speed,std::min(speed/factor,cap));
    if (!std::isfinite(candidate) || candidate>std::numeric_limits<float>::max())
        throw std::runtime_error("GridVarSpeed: speed overflow");
    return candidate;
}

inline double bounded_offset(double dz,double height) {
    if (!std::isfinite(dz) || !std::isfinite(height) || height<=0. || dz<0.)
        throw std::runtime_error("GridVarSpeed: invalid Z offset or extrusion height");
    return dz<=height/2.+1e-12 ? dz : height/2.;
}

inline std::pair<coord_t,coord_t> pass_offsets(double dz,double height,double print_z,bool infill_first) {
    dz=bounded_offset(dz,height);
    if (!std::isfinite(dz) || !std::isfinite(height) || !std::isfinite(print_z) ||
        print_z<=0. || (infill_first && print_z-dz<=0.))
        throw std::runtime_error("GridVarSpeed: Z offset must be <= half the extrusion height and stay above Z=0");
    const coord_t offset=scale_i(dz);
    return infill_first?std::make_pair(-offset,coord_t(0)):std::make_pair(coord_t(0),offset);
}
} // namespace nematx_gridvarspeed
