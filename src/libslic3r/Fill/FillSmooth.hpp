///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_FillSmooth_hpp_
#define slic3r_FillSmooth_hpp_

/* Use FillSmooth through Fill::fill_surface_extrusion() after configuring its
   layer and spacing. Deposition and ironing share native filler recipes but
   remain separate ordered passes. Each pass owns its generic output subtree;
   centerline-only fill_surface() is not supported by this pattern.
*/

#include "FillBase.hpp"

namespace Slic3r {

class FillSmooth : public Fill
{
public:
    FillSmooth() : Fill() {
        nbPass = 2;
        anglePass[0] = 0;
        anglePass[1] = float(M_PI/2);
        anglePass[2] = 0;
        fillPattern[0] = InfillPattern::ipMonotonic;//ipRectilinearWGapFill;
        has_gap_fill[0] = true;
        fillPattern[1] = InfillPattern::ipRectilinear;
        has_gap_fill[1] = false;
        fillPattern[2] = InfillPattern::ipRectilinear;
        has_gap_fill[2] = false;
        rolePass[0] = ExtrusionRole::None;// ExtrusionRole::TopSolidInfill;
        rolePass[1] = ExtrusionRole::Ironing;
        rolePass[2] = ExtrusionRole::Ironing;
        percentWidth[0] = 1;
        percentWidth[1] = 2;
        percentWidth[2] = 1.0;
        percentFlow[0] = 0.8;
        percentFlow[1] = 0.2;
        percentFlow[2] = 0.0;
        double extrusionMult = 1.0;
        percentFlow[0] *= extrusionMult;
        percentFlow[1] *= extrusionMult;
        percentFlow[2] *= extrusionMult;
        has_overlap[0] = false;
        has_overlap[1] = true;
        has_overlap[2] = false;
    }
    virtual Fill* clone() const{ return new FillSmooth(*this); }

    virtual Polylines fill_surface(const Surface *surface, const FillParams &params) const override;
    virtual void fill_surface_extrusion(const Surface *surface, const FillParams &params, ExtrusionEntitiesPtr &out) const override;
    
protected:
    int nbPass=2;
    // this parameter is now erased by fill_smooth_width when available.
    double percentWidth[3];
    // this parameter is now modified by fill_smooth_distribution when available. (note that fill_top_flow_ratio can also increasse the params.flow_mult passed by fill.cpp)
    double percentFlow[3];
    //angle to add to base angle
    float anglePass[3];
    // gapfill?
    bool has_gap_fill[3];
    //if false, it won't overlap inside the perimeters
    bool has_overlap[3];
    // profile for base width, speed, etc.
    ExtrusionRole rolePass[3] = {ExtrusionRole::None, ExtrusionRole::None, ExtrusionRole::None};
    //fill algorithm to call
    InfillPattern fillPattern[3];

    // Append one pass after resolving its density, flow distribution and overlap region.
    void perform_single_fill(const int idx, ExtrusionEntity &eecroot, const Surface &srf_source,
        const FillParams &params) const;
    // Acquire this pass's native filler output into the supplied owned group.
    void fill_expolygon(const int idx, ExtrusionEntity &eec, const Surface &srf_to_fill,
        const FillParams &params) const;
};


} // namespace Slic3r

#endif // slic3r_FillSmooth_hpp_
