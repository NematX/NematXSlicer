/*
 * Copyright (c) 2026 NematX
 * Released under the GNU Affero General Public License, version 3 or later.
 */
#include "CornerStretchGeometry.hpp"
#include <iostream>
using namespace nematx_corner;
void require(bool v,const char *message) { if(!v) throw std::runtime_error(message); }
void check(std::vector<Edge> e,std::vector<Settings> s) {
    const auto output=stretch(e,s,0.001);
    double before=0,after=0;
    Point cursor=e.front().curve.a;
    for(size_t i=0;i<e.size();++i) {
        before+=e[i].curve.length();
        for(const auto &p:output[i]) {
            require(norm(cursor-p.curve.a)<2,"discontinuous output");
            require(std::isfinite(p.ratio)&&p.ratio>0,"invalid ratio");
            after+=p.ratio*p.curve.length();cursor=p.curve.b;
        }
    }
    require(std::abs(before-after)<3,"volume not conserved");
}
int main() {
    try {
        std::vector<Edge> e={{{{0,0},{10000,0},{},0},0,0,true},
                             {{{10000,0},{10000,10000},{},0},1,0,true},
                             {{{10000,10000},{20000,10000},{},0},2,0,true}};
        Settings on{true,false,120,1000,500,1,1};
        check(e,{on,on,{}});
        auto output=stretch(e,{on,on,{}},.001);
        require(output[0].size()>1,"corner not modified");
        auto disabled=stretch(e,{{},{},{}},.001);
        for(size_t i=0;i<e.size();++i) require(disabled[i].size()==1 && disabled[i][0].ratio==1,"OFF changed geometry");
        auto split=e;
        split[0].curve.b={5000,0};
        split.insert(split.begin()+1,{{{5000,0},{10000,0},{},0},3,0,true});
        check(split,{{},on,on,{}});
        double original_length=0,split_length=0;
        for(auto &row:output) for(auto &f:row) original_length+=f.curve.length();
        for(auto &row:stretch(split,{{},on,on,{}},.001)) for(auto &f:row) split_length+=f.curve.length();
        require(std::abs(original_length-split_length)<1e-6,"transparent split changed result");
        check(e,{{},{},{}});
        on.arcs=true;check(e,{on,on,{}});
        e[1].curve.b={10000,100};e[2].curve.a={10000,100};e[2].curve.b={20000,100};
        check(e,{on,on,{}}); // competing close corners
        auto close=stretch(e,{on,on,{}},.001);
        for(auto &row:close) for(auto &p:row) require(p.ratio>.01,"thin spike on close corners");
        on.angle=90;check(e,{on,on,{}});
        on.angle=120;on.entering=0;check(e,{on,on,{}});
        auto asymmetric=stretch(e,{on,on,{}},.001);
        bool modified=false;
        for(auto &row:asymmetric) for(auto &p:row) modified|=p.ratio!=1;
        require(modified,"zero entering influence incorrectly disabled exiting influence");
        on.entering=1;on.exiting=0;check(e,{on,on,{}});
        on.exiting=2;on.entering=3;check(e,{on,on,{}});
        for(int i=0;i<200;++i) {
            on.entering=.1+(i%11)*.3;on.exiting=.1+(i%7)*.4;
            e[1].curve.b.y=10+i*31;e[2].curve.a=e[1].curve.b;
            e[2].curve.b.y=e[1].curve.b.y;
            check(e,{on,on,{}});
        }
        e[1].state=1;check(e,{on,on,{}});
        auto barrier=stretch(e,{on,on,{}},.001);
        require(barrier[0].size()==1 && barrier[0][0].ratio==1,"crossed process-state barrier");
        std::vector<Edge> gap={{{{0,0},{10000,0},{},0},0,0,false},
                               {{{10001,0},{20000,0},{},0},1,0,false}};
        auto kept=stretch(gap,{{},{}},10);
        require(kept[1][0].curve.a.x==10001,"closed an original gap");
        std::cout<<"corner_stretch_geometry: passed\n";
    } catch(const std::exception &e) { std::cerr<<e.what()<<'\n';return 1; }
}
