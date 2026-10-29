// Copyright (c) 2026 NematX. AGPL-3.0-or-later.
// Uses only public SDK, linked to the existing host DLL (no host rebuild).
#include "GridVarSpeedKernel.hpp"
#include "GridVarSpeedProperties.hpp"
#include "rectilinear/RectilinearFills.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include <exception>
#include <iostream>

namespace api=slic3r_api;
namespace rect=api::GridVarSpeedCopy::RectilinearPatternsPlugin;
using namespace nematx_gridvarspeed;
void check(bool v,const char *m) {if (!v) throw std::runtime_error(m);}
c_point cp(double x,double y) {return {scale_i(x),scale_i(y)};}
void ring(polygon_handle *p,std::initializer_list<c_point> points) {
    for (c_point point:points) multipoint_push_back(polygon_as_multipoint(p),point);
}

// True if any POSITIVE interval of a segment is inside the open rectangular
// void, not just its endpoints or midpoint.
bool crosses_hole(Point a,Point b) {
    double low=0.,high=1.;
    for (int axis=0;axis<2;++axis) {
        const double start=unscaled(a(axis)),delta=unscaled(b(axis)-a(axis));
        if (std::abs(delta)<1e-12) {if (start<=-4. || start>=4.) return false;}
        else {
            double u=(-4.-start)/delta,v=(4.-start)/delta;
            if (u>v) std::swap(u,v);
            low=std::max(low,u); high=std::min(high,v);
        }
    }
    return high-low>1e-9 && high>0. && low<1.;
}

void test(storage_handle *storage) {
    api::EPropertySpecialCommand command={};
    command.set(C_EXTRUSION_SPECIAL_COMMAND_EXTRUSION,1.25);
    check(is_raw_filament_command(&command),"raw filament commands bypass compensation without a height sentinel");
    command.set(C_EXTRUSION_SPECIAL_COMMAND_TOOLCHANGE,0.);
    check(!is_raw_filament_command(&command) && !is_raw_filament_command(nullptr),"ordinary extrusion is not a raw filament command");
    api::EPropertySpeed fields={};
    fields.speed_mm_per_s=-1.f;
    check(inherited_speed(30.,&fields)==30.,"unset speed retains inherited speed");
    fields.speed_mm_per_s=45.f;
    check(inherited_speed(30.,&fields)==45.,"direct speed overrides ancestor");
    check(inherited_speed(30.,nullptr)==30.,"absent speed retains ancestor");
    // Exercise public clone/split mechanics without constructing a private host
    // orchestrator. Dynamic gridvarspeed keys are covered by real pattern/speed exports;
    // creating those payloads separately requires the live orchestrator handle.
    api::StoredExtrusionEntity original(storage,std::vector<c_point>{cp(-10,0),cp(10,0)});
    original.get_or_add(api::EPropertyAttributes::key).extrusion_role(RAW_EXTRUSION_ROLE_INTERNAL_INFILL)
        .mm3_per_mm(0.04).width(0.225f).height(0.2f);
    original.get_or_add(api::EPropertyZOffset::key).set(scale_i(0.05));
    original.get_or_add(api::EPropertyFan::key).set(65.f,FAN_REQUEST_MODE_OVERRIDE);
    api::MutableExtrusionEntity clone(extrusion_clone(storage,original.handle()));
    check(clone.get(api::EPropertyZOffset::key)->get()==scale_i(0.05),"clone preserves direct Z property");
    check(clone.get(api::EPropertyFan::key)->percent==65.f &&
          clone.get(api::EPropertyFan::key)->mode==FAN_REQUEST_MODE_OVERRIDE,"clone preserves fan request pair");
    api::StoredExPolygonCollection left(storage),right(storage);
    for (int side=0;side<2;++side) {
        api::StoredExPolygon region(storage);
        const double x1=side?0.:-11.,x2=side?11.:0.;
        ring(expolygon_contour(region.mutable_handle()),{cp(x1,-1),cp(x2,-1),cp(x2,1),cp(x1,1)});
        (side?right:left).push_back_move(std::move(region));
    }
    const auto pieces=clone.split_leaf_by_areas({left.readonly(),right.readonly()});
    check(pieces.size()==2,"regional split produces two pieces");
    for (const auto &piece:pieces) {
        check(piece.entity.get(api::EPropertyZOffset::key)->get()==scale_i(0.05),"regional split preserves Z property");
        check(piece.entity.get(api::EPropertyAttributes::key)->mm3_per_mm()==0.04,"regional split preserves flow property");
        const auto *fan=piece.entity.get(api::EPropertyFan::key);
        check(fan && fan->percent==65.f && fan->mode==FAN_REQUEST_MODE_OVERRIDE,"regional split preserves fan request pair");
    }
    for (int shape:{0,1,2,3}) for (double density:{0.18,0.5,1.}) for (double angle:{0.,0.7853981633974483}) {
        const bool hole=shape==1;
        api::StoredExPolygon polygon(storage);
        if (shape<2) ring(expolygon_contour(polygon.mutable_handle()),{cp(-15,-15),cp(15,-15),cp(15,15),cp(-15,15)});
        else if (shape==2) // concave L
            ring(expolygon_contour(polygon.mutable_handle()),{cp(-15,-15),cp(15,-15),cp(15,-5),cp(-5,-5),cp(-5,15),cp(-15,15)});
        else // dumbbell: a thin neck disappears on inner offset, creating islands
            ring(expolygon_contour(polygon.mutable_handle()),{cp(-15,-15),cp(-5,-15),cp(-5,-0.1),cp(5,-0.1),cp(5,-15),cp(15,-15),
                 cp(15,15),cp(5,15),cp(5,0.1),cp(-5,0.1),cp(-5,15),cp(-15,15)});
        if (hole) {
            polygon.holes_resize(1);
            ring(expolygon_hole_at(polygon.mutable_handle(),0),{cp(-4,-4),cp(-4,4),cp(4,4),cp(4,-4)});
        }
        check(polygon.is_valid(),"source polygon valid");
        raw_infill_pattern_params p={};
        p.density=float(density); p.spacing=0.42; p.angle=angle; p.bridge_angle=-1.;
        p.bridge_offset=-1; p.layer_id=3; p.z=0.8; p.fill_resolution=scale_i(0.005);
        p.object_bounding_box={cp(-15,-15),cp(15,15)};
        p.flow.width=scale_i(0.45); p.flow.height=scale_i(0.2); p.flow.spacing=scale_i(0.42);
        p.flow.mm3_per_mm=0.08; p.flow_mult=1.;
        p.infill_flags=C_INFILL_FLAG_CAN_ANGLE_CROSS;
        rect::fills::FillRectilinear filler;
        rect::fills::configure_filler(filler,p,storage);
        Polylines passes[2];
        for (int pass=0;pass<2;++pass) {
            check(filler.fill_surface_by_lines(polygon.readonly(),p,float(pass*3.14159265358979323846/2.),0.,passes[pass]),"raster success");
            check(!passes[pass].empty(),"nonempty pass");
            remove_near_points(passes[pass],p.flow.spacing);
        }
        for (int pass=0;pass<2;++pass) for (double ratio:{0.,0.01,0.25,0.5,0.99,1.}) {
            const auto chains=split_crossings(passes[pass],passes[1-pass],ratio,p.flow.spacing);
            check(chains.size()==passes[pass].size(),"all source chains retained");
            for (size_t ci=0;ci<chains.size();++ci) {
                const auto &chain=chains[ci];
                check(chain.front().points.front()==passes[pass][ci].points.front(),"chain first endpoint");
                check(chain.back().points.back()==passes[pass][ci].points.back(),"chain last endpoint");
                double covered=0.;
                for (size_t fi=0;fi<chain.size();++fi) {
                    const auto &fragment=chain[fi];
                    if (fi) check(chain[fi-1].points.back()==fragment.points.front(),"chain continuity");
                    std::vector<c_point> points;
                    for (Point pt:fragment.points) points.push_back({pt.x(),pt.y()});
                    api::StoredExtrusionEntity leaf(storage,points); // exercises host ArcPolyline validation
                    check(leaf.segment_count()>0,"host accepts printable fragment");
                    for (size_t j=1;j<fragment.points.size();++j) {
                        const Point a=fragment.points[j-1],b=fragment.points[j];
                        check(a.distance_to(b)>0.,"no zero edge");
                        if (hole) check(!crosses_hole(a,b),"no fragment crosses the hole interior");
                        if (shape>=2) {
                            // Partition at ALL source boundary intersections,
                            // checking each interval. Not only a midpoint check.
                            std::vector<double> ts{0.,1.};
                            const auto contour=polygon.contour();
                            const double total=a.distance_to(b);
                            for (uint32_t edge=0;edge<contour.size();++edge) {
                                Point hit;
                                if (intersection(a,b,Point(contour.at(edge)),Point(contour.at((edge+1)%contour.size())),hit))
                                    ts.push_back(std::clamp(a.distance_to(hit)/total,0.,1.));
                            }
                            std::sort(ts.begin(),ts.end());
                            for (size_t interval=1;interval<ts.size();++interval) {
                                if ((ts[interval]-ts[interval-1])*total<=SCALED_EPSILON*2) continue;
                                const Point mid=point_at(a,b,(ts[interval]+ts[interval-1])*0.5*total);
                                if (!polygon.contains({mid.x(),mid.y()})) {
                                    std::cerr<<"shape="<<shape<<" density="<<density<<" angle="<<angle<<" pass="<<pass<<" ratio="<<ratio
                                             <<" segment "<<unscaled(a.x())<<','<<unscaled(a.y())<<" -> "<<unscaled(b.x())<<','<<unscaled(b.y())
                                             <<" midpoint "<<unscaled(mid.x())<<','<<unscaled(mid.y())<<'\n';
                                    check(false,"no fragment outside concave/island surface");
                                }
                            }
                        }
                    }
                    covered+=length(fragment.points);
                }
                // Only sub-epsilon cleanup at flow boundaries may differ.
                check(std::abs(covered-passes[pass][ci].length())<scale_i(0.005),"source coverage preserved");
            }
        }
        // Odd/even layers must NOT exchange the pass orientations.
        p.layer_id=4;
        rect::fills::configure_filler(filler,p,storage);
        Polylines even;
        filler.fill_surface_by_lines(polygon.readonly(),p,0.f,0.f,even);
        remove_near_points(even,p.flow.spacing);
        check(even.size()==passes[0].size(),"fixed layer direction");
        for (size_t i=0;i<even.size();++i) check(even[i].points==passes[0][i].points,"no odd/even alternation");
    }
}

void callback(uint32_t,storage_handle *storage,void *user) {
    // Never propagate an exception through the C callback.
    try {test(storage);} catch (...) {*static_cast<std::exception_ptr*>(user)=std::current_exception();}
}
int main() {
    try {
        std::exception_ptr error;
        slic3r_parallel_for_storage(0,1,&error,callback);
        if (error) std::rethrow_exception(error);
        std::cout<<"gridvarspeed public-SDK raster/fragment tests passed\n";
        return 0;
    } catch (const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
