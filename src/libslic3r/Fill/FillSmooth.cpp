///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "FillSmooth.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityVisitors.hpp"
#include "libslic3r/Surface.hpp"

/* Supply native deposition followed by smoothing passes to Fill consumers.
   Each pass resolves density, flow distribution and overlap independently,
   then delegates geometry to its selected native filler. Generic owned groups
   preserve pass order while retaining each pass's internal permissions.
   Empty passes are omitted; the caller owns the completed nonempty result.

       fill_surface_extrusion()
           `-- perform_single_fill(): resolve one pass's recipe and domain
               `-- fill_expolygon(): create the native filler and acquire output
*/

namespace Slic3r {

    Polylines FillSmooth::fill_surface(const Surface *surface, const FillParams &params) const
    {
        //ERROR: you shouldn't call that. Default to the rectilinear one.
        printf("FillSmooth::fill_surface() : you call the wrong method (fill_surface instead of fill_surface_extrusion).\n");
        assert(false);
        Polylines polylines_out;
        return polylines_out;
    }

    void FillSmooth::perform_single_fill(const int idx, ExtrusionEntity &eecroot, const Surface &srf_source,
        const FillParams &params) const {
        if (srf_source.expolygon.empty()) return;
        
        // Save into layer smoothing path.
        ExtrusionEntityUPtr eec = std::make_unique<ExtrusionEntity>(
            ExtrusionEntity::Children{}, !params.monotonic, !params.monotonic, false);
        FillParams params_modifided = params;
        if (params.config != NULL && idx > 0) params_modifided.density /= (float)params.config->fill_smooth_width.get_effective_value(1);
        else if (params.config != NULL && idx == 0) params_modifided.density *= 1;
        else params_modifided.density *= (float)percentWidth[idx];
        // reduce flow for each increase in density
        params_modifided.flow_mult *= params.density;
        params_modifided.flow_mult /= params_modifided.density;
        // split the flow between steps
        if (params.config != NULL && idx > 0) params_modifided.flow_mult *= (float)params.config->fill_smooth_distribution.get_effective_value(1);
        else if (params.config != NULL && idx == 0) params_modifided.flow_mult *= (1.f - (float)params.config->fill_smooth_distribution.get_effective_value(1));
        else params_modifided.flow_mult *= (float)percentFlow[idx];
        //set role
        if (rolePass[idx] != ExtrusionRole::None)
            params_modifided.role = rolePass[idx];

        //choose if we are going to extrude with or without overlap
        if ((params.flow.bridge() && idx == 0) || has_overlap[idx] || this->no_overlap_expolygons.empty()){
            this->fill_expolygon(idx, *eec, srf_source, params_modifided);
        }
        else{
            Surface surfaceNoOverlap(srf_source);
            //use half overlap instead of none.
            ExPolygons half_overlap = offset_ex(this->no_overlap_expolygons, scale_d(this->overlap / 2));
            half_overlap = intersection_ex(ExPolygons{ srf_source.expolygon }, half_overlap);
            for (const ExPolygon &poly : half_overlap) {
                if (poly.empty()) continue;
                surfaceNoOverlap.expolygon = poly;
                this->fill_expolygon(idx, *eec, surfaceNoOverlap, params_modifided);
            }
        }
        
        if (eec->child_count() != 0)
            eecroot.append_child(std::move(eec));
    }
    
    void FillSmooth::fill_expolygon(const int idx, ExtrusionEntity &eec, const Surface &srf_to_fill,
        const FillParams &params_init) const {
        
        FillParams params = params_init;
        params.add_gap_fill = has_gap_fill[idx];
        std::unique_ptr<Fill> f2 = std::unique_ptr<Fill>(Fill::new_from_type(fillPattern[idx]));
        f2->bounding_box = this->bounding_box;
        f2->init_spacing(this->get_spacing(), params);
        f2->layer_id = this->layer_id;
        f2->z = this->z;
        f2->angle = (this->can_angle_cross ? anglePass[idx] : 0) + this->angle;
        // Maximum length of the perimeter segment linking two infill lines.
        f2->link_max_length = this->link_max_length;
        // Used by the concentric infill pattern to clip the loops to create extrusion paths.
        f2->loop_clipping = this->loop_clipping;

        f2->fill_surface_extrusion(&srf_to_fill, params, eec);


//        Polylines polylines_layer = f2->fill_surface(&srf_to_fill, params);
//
//        if (!polylines_layer.empty()) {
//            //get the role
//            ExtrusionRole good_role = params.role;
//            if (good_role == ExtrusionRole::None) {
//                good_role = params.flow.bridge() && idx == 0 ? ExtrusionRole::BridgeInfill : rolePass[idx];
//            }
//            //get the flow
//            float mult_flow = 1;
//            if (params.fill_exactly) {
//
//                // compute the volume to extrude
//                double volume_to_occupy = compute_unscaled_volume_to_fill(&srf_to_fill, params);
//
//                //compute the path of the nozzle
//                double length_tot = 0;
//                int nb_lines = 0;
//                for (Polyline& pline : polylines_layer) {
//                    Lines lines = pline.lines();
//                    for (Line& line : lines) {
//                        length_tot += unscaled(line.length());
//                        nb_lines++;
//                    }
//                }
//                //compute flow to remove spacing_ratio from the equation
//                double extruded_volume = 0;
//                if (params.flow.spacing_ratio() < 1.f && !params.flow.bridge()) {
//                    // the spacing is larger than usual. get the flow from the current spacing
//                    Flow test_flow = Flow::new_from_spacing(params.flow.spacing(), params.flow.nozzle_diameter(), params.flow.height(), 1, params.flow.bridge());
//                    extruded_volume = test_flow.mm3_per_mm() * length_tot / params.density;
//                } else
//                    extruded_volume = params.flow.mm3_per_mm() * length_tot / params.density;
//                if (extruded_volume == 0) extruded_volume = volume_to_occupy;
//
//                // print
//                mult_flow = (float)std::min(2., volume_to_occupy / extruded_volume);
//                BOOST_LOG_TRIVIAL(info) << "Layer " << layer_id << " Ironing process " << idx << " extrude " << extruded_volume << " mm3 for a volume of " << volume_to_occupy << " mm3 : we mult the flow by " << mult_flow;
//                
//            }
//#if _DEBUG
//            this->debug_verify_flow_mult = mult_flow;
//#endif
//            extrusion_entities_append_paths(
//                eec,
//                std::move(polylines_layer),
//                ExtrusionAttributes{good_role,
//                                    ExtrusionFlow{params.flow.mm3_per_mm() * params.flow_mult * mult_flow,
//                                                    // min-reduced flow width for a better view (it's mostly a gui
//                                                    // thing, but some support code can want to mess with it)
//                                                    (float) (params.flow.width() * (params.flow_mult * mult_flow < 0.1 ? 0.1 : params.flow_mult * mult_flow)),
//                                                    (float) params.flow.height()}},
//                true);
//        }
    }


    void FillSmooth::fill_surface_extrusion(const Surface *surface, const FillParams &params, ExtrusionEntitiesPtr &out) const
    {
        coordf_t init_spacing = this->get_spacing();

        //create root node
        ExtrusionEntityUPtr eecroot = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children{}, false, false, false);

        // first infill
        FillParams first_pass_params = params;
        //if(first_pass_params.role !=  ExtrusionRole::SupportMaterial && first_pass_params.role !=  ExtrusionRole::SupportMaterialInterface)
        //s    first_pass_params.role =  ExtrusionRole::SolidInfill;
        perform_single_fill(0, *eecroot, *surface, first_pass_params);

        //use monotonic for ironing pass
        FillParams monotonic_params = params;
        monotonic_params.monotonic = true;

        //second infill
        if (nbPass > 1){
            perform_single_fill(1, *eecroot, *surface, monotonic_params);
        }

        // third infill
        if (nbPass > 2){
            perform_single_fill(2, *eecroot, *surface, monotonic_params);
        }
        
        if (eecroot->child_count() != 0) {
#ifdef _DEBUGINFO
            LoopAssertVisitor().traverse(*eecroot);
#endif
            out.push_back(eecroot.get());
            eecroot.release();
        }
    }

} // namespace Slic3r
