// Copyright (c) 2026 NematX. AGPL-3.0-or-later.
#include "GridVarSpeedKernel.hpp"
#include <iostream>
#include <limits>

using namespace nematx_gridvarspeed;
void check(bool value,const char *message) {if (!value) throw std::runtime_error(message);}
void near(double a,double b,double tolerance=1e-6) {check(std::abs(a-b)<=tolerance,"numeric expectation");}
Point pt(double x,double y=0.) {return {scale_i(x),scale_i(y)};}
Polyline line(double x1,double y1,double x2,double y2) {return Polyline{pt(x1,y1),pt(x2,y2)};}

void invariant(const Chains &out,const Polyline &src) {
    check(out.size()==1,"one chain per source");
    const Chain &chain=out.front();
    check(!chain.empty(),"nonempty chain");
    check(chain.front().points.front()==src.points.front(),"start preserved");
    check(chain.back().points.back()==src.points.back(),"end preserved");
    double len=0.;
    for (size_t i=0;i<chain.size();++i) {
        check(chain[i].factor>0. && chain[i].factor<=1.,"positive bounded flow");
        check(chain[i].points.size()>=2 && length(chain[i].points)>0.,"positive length");
        if (i) check(chain[i-1].points.back()==chain[i].points.front(),"continuous chain");
        len+=length(chain[i].points);
    }
    near(len,src.length(),4.); // coordinate rounding only
}

void geometry_tests() {
    const Polyline src=line(-10,0,10,0);
    const Polylines other{line(0,-10,0,10)};
    for (double r:{0.,0.00000001,0.25,0.5,0.99,1.}) {
        const auto out=split_crossings({src},other,r,scale_i(4.));
        invariant(out,src);
        if (r==0. || r==1. || r==0.00000001) check(out[0].size()==1,"uniform endpoint/zero-rounded zone");
    }
    const auto nominal=split_crossings({src},other,0.5,scale_i(4.));
    check(nominal[0].size()==4,"first crossing is asymmetric");
    const double factors[]={1.,0.5,0.75,1.};
    const double ends[]={-1.,0.5,1.5,10.};
    for (size_t i=0;i<4;++i) {
        near(nominal[0][i].factor,factors[i]);
        near(unscaled(nominal[0][i].points.back().x()),ends[i]);
    }
    double integrated=0.;
    for (const auto &f:nominal[0]) integrated+=length(f.points)*f.factor;
    near(unscaled(integrated),19.); // loss=2*h*(1-r)=1 mm
    for (double d:{2.5,3.,3.5,8.}) {
        auto out=split_crossings({src},{line(-5,-5,-5,5),line(-5+d,-5,-5+d,5)},0.5,scale_i(4.));
        invariant(out,src);
    }
    // Vertex duplicates, endpoints and close collisions must never generate a
    // backwards segment or an extension past the source line.
    for (double cross:{-10.,-9.9,-9.,0.,9.9,10.}) {
        auto out=split_crossings({src},{line(cross,-2,cross,0),line(cross,0,cross,2)},0.5,scale_i(4.));
        invariant(out,src);
        for (const auto &f:out[0]) for (Point p:f.points) check(p.x()>=pt(-10).x() && p.x()<=pt(10).x() && p.y()==0,"inside source segment");
    }
    auto dedup=split_crossings({src},{line(0,-2,0,0),line(0,0,0,2)},0.5,scale_i(4.));
    check(dedup[0].size()==nominal[0].size(),"deduplicate shared vertex");
    auto untouched=split_crossings({src},{},0.5,scale_i(4.));
    check(untouched[0].size()==1,"retain two-point path without collisions");
    invariant(untouched,src);
    auto collinear=split_crossings({src},{line(-2,0,2,0)},0.5,scale_i(4.));
    check(collinear[0].size()==1,"legacy ignores collinear overlap");
    // Translation to negative coordinates and two-point tails after a corner.
    Polyline corner{pt(-20,-20),pt(0,-20),pt(0,0)};
    invariant(split_crossings({corner},{line(-10,-25,-10,-15)},0.5,scale_i(4.)),corner);
    Polylines short_points{Polyline{pt(0),pt(0.1),pt(4),pt(4.1)}};
    remove_near_points(short_points,scale_i(1.));
    check(short_points[0].points.front()==pt(0) && short_points[0].points.back()==pt(4.1),"cleanup retains endpoints");
    Polylines turn{Polyline{pt(0),pt(0.1),pt(0.1,5)}};
    remove_near_points(turn,scale_i(1.));
    check(turn[0].size()==3,"cleanup preserves a short real corner");
    // Dense fuzz of crossing positions on a straight line, deterministic.
    for (int i=0;i<300;++i) {
        const double r=0.01+(i%98)*0.01;
        Polylines crossings;
        for (int j=0;j<5;++j) {double x=-9.9+((i*37+j*71)%1980)*0.01; crossings.push_back(line(x,-1,x,1));}
        invariant(split_crossings({src},crossings,r,scale_i(4.)),src);
    }
}

template<class F> void rejects(F f) {bool caught=false; try {f();} catch(const std::runtime_error &) {caught=true;} check(caught,"invalid input rejected");}
void process_tests() {
    const double inf=std::numeric_limits<double>::infinity();
    near(adjusted_speed(30.,0.5,inf),60.);
    near(adjusted_speed(30.,0.75,inf),40.);
    near(adjusted_speed(30.,0.5,45.),45.);
    near(adjusted_speed(30.,0.5,20.),30.); // upstream minimum already validated
    rejects([&]{adjusted_speed(0.,0.5,inf);});
    rejects([&]{adjusted_speed(30.,0.,inf);});
    rejects([&]{split_crossings({}, {}, -1.,1);});
    rejects([&]{split_crossings({}, {}, std::numeric_limits<double>::quiet_NaN(),1);});
    auto low=pass_offsets(0.05,0.2,0.2,true),high=pass_offsets(0.05,0.2,0.2,false);
    check(low.first==-scale_i(0.05) && low.second==0,"lower P0");
    check(high.first==0 && high.second==scale_i(0.05),"raise P1");
    check(pass_offsets(0.1,unscaled(scale_i(0.2)),0.2,true).first==-scale_i(0.1),"half-height boundary tolerates conversion roundoff");
    check(pass_offsets(0.11,0.2,0.2,false).second==scale_i(0.1),"clamp excessive Z offset");
    check(pass_offsets(0.11,0.1,0.4,true).first==-scale_i(0.05),"clamp follows variable height");
    rejects([]{bounded_offset(-0.1,0.2);});
    rejects([]{bounded_offset(std::numeric_limits<double>::infinity(),0.2);});
    check(pass_offsets(0.08,0.2,0.05,false).second==scale_i(0.08),"positive raised pass does not require room below nominal Z");
    rejects([]{pass_offsets(0.08,0.2,0.05,true);});
    rejects([]{pass_offsets(0.,0.2,std::numeric_limits<double>::quiet_NaN(),false);});
    rejects([]{pass_offsets(0.1,0.2,0.1,true);});
}

int main() {
    try {geometry_tests();process_tests();std::cout<<"gridvarspeed kernel/process tests passed\n";return 0;}
    catch (const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
