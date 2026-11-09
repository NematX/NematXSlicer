///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/ Copyright (c) Prusa Research 2016 - 2023 Vojtěch Bubník @bubnikv, Lukáš Hejl @hejllukas
///|/
///|/ ported from lib/Slic3r/Fill/Concentric.pm:
///|/ Copyright (c) Prusa Research 2016 Vojtěch Bubník @bubnikv
///|/ Copyright (c) Slic3r 2011 - 2015 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2012 Mark Hindess
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "FillConcentric.hpp"

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/EdgeGrid.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityVisitors.hpp"
#include "libslic3r/Geometry/MedialAxis.hpp"
#include "libslic3r/Surface.hpp"

#include "Arachne/WallToolPaths.hpp"

/* Supply native concentric fill as centerlines or owned extrusion trees.
   Repeated offsets create shells; nesting associates each shell with its parent
   before the next gap-fill pass. Generic fixed groups retain those dependencies,
   while separate branches remain sortable. Ring leaves own their flow values.
   MedialAxis supplies gap-fill geometry and the combined output is corrected
   for the requested volume before ownership passes to the caller.

       _fill_surface_single(): offset and connect centerlines
       fill_surface_extrusion()
           |-- derive shells and gaps from the input surface
           |-- append_concentric_loop(): emit exactly closed owned rings
           |-- Geometry::thin_variable_width(): fill narrow gaps
           `-- correct flow on the combined tree and release owned output
*/

namespace Slic3r {

namespace {

// Read optional plugin-owned region settings, falling back when their provider is disabled.
double fill_region_option_effective_value(const PrintRegionConfig *config, const char *key, double fallback);

// Consume a valid polygon into an exactly closed generic ring with leaf-local flow.
void append_concentric_loop(ExtrusionEntity &storage, ExtrusionRole role, const FillParams &params, Polygon &polygon);

double fill_region_option_effective_value(const PrintRegionConfig *config, const char *key, double fallback)
{
    const ConfigOption *option = config != nullptr ? config->option(key) : nullptr;
    return option != nullptr ? option->get_effective_value(fallback) : fallback;
}

void append_concentric_loop(ExtrusionEntity &storage, ExtrusionRole role, const FillParams &params, Polygon &polygon)
{
    if (ensure_valid(polygon, params.fill_resolution)) {
        polygon.make_counter_clockwise();
        ArcPolyline polyline;
        polyline.append(std::move(polygon.points));
        polyline.append(polyline.front());
        ExtrusionEntityUPtr path = std::make_unique<ExtrusionEntity>(false, std::move(polyline));
        path->add_property(ExtrusionAttributes{role, ExtrusionFlow{params.flow.mm3_per_mm(),
            float(params.flow.width()), float(params.flow.height())}});
        ExtrusionEntity::Children children;
        children.emplace_back(std::move(path));
        storage.append_child(std::make_unique<ExtrusionEntity>(std::move(children), false, false, true));
    }
}

} // namespace

void
FillConcentric::init_spacing(coordf_t spacing, const FillParams &params)
{
    Fill::init_spacing(spacing, params);
    if (params.density > 0.9999f && !params.dont_adjust) {
        this->spacing_priv = unscaled(this->_adjust_solid_spacing(bounding_box.size()(0), _line_spacing_for_density(params)));
    }
}

void
FillConcentric::_fill_surface_single(
    const FillParams                &params,
    unsigned int                     thickness_layers,
    const std::pair<float, Point>   &direction,
    ExPolygon                        expolygon,
    Polylines                       &polylines_out) const
{
    // no rotation is supported for this infill pattern
    BoundingBox bounding_box = expolygon.contour.bounding_box();
    
    coord_t distance = _line_spacing_for_density(params);
    if (params.density > 0.9999f && !params.dont_adjust) {
        //it's == Slic3r::FillConcentric::_adjust_solid_spacing(bounding_box.size()(0), _line_spacing_for_density(params.density)) because of the init_spacing()
        distance = scale_i(this->get_spacing());
    }

    Polygons   loops = to_polygons(expolygon);
    ExPolygons last { std::move(expolygon) };
    while (! last.empty()) {
        // offset3 to clean the polygon up to fill_resolution
        last = offset_ex(offset2_ex(last, -double(distance + scale_d(this->get_spacing()) / 2),
                                    +double(scale_d(this->get_spacing()) / 2) + params.fill_resolution / 2),
                         -params.fill_resolution / 2);
        append(loops, to_polygons(last));
    }

    // generate paths from the outermost to the innermost, to avoid
    // adhesion problems of the first central tiny loops
    loops = union_pt_chained_outside_in(loops);

    // split paths using a nearest neighbor search
    size_t iPathFirst = polylines_out.size();
    Point last_pos(0, 0);
    for (const Polygon &loop : loops) {
        polylines_out.emplace_back(loop.split_at_index(nearest_point_index(loop.points, last_pos)));
        last_pos = polylines_out.back().last_point();
    }

    // clip the paths to prevent the extruder from getting exactly on the first point of the loop
    // Keep valid paths only.
    size_t j = iPathFirst;
    for (size_t i = iPathFirst; i < polylines_out.size(); ++ i) {
        polylines_out[i].clip_end(coordf_t(this->loop_clipping));
        if (polylines_out[i].is_valid()) {
            if (j < i)
                polylines_out[j] = std::move(polylines_out[i]);
            ++ j;
        }
    }
    if (j < polylines_out.size())
        polylines_out.erase(polylines_out.begin() + int(j), polylines_out.end());
    //TODO: return ExtrusionLoop objects to get better chained paths,
    // otherwise the outermost loop starts at the closest point to (0, 0).
    // We want the loops to be split inside the G-code generator to get optimum path planning.
    assert_valid(polylines_out);
}

void
FillConcentric::fill_surface_extrusion(
    const Surface *surface, 
    const FillParams &params,
    ExtrusionEntitiesPtr &out) const {

    //with or without gapfill?
    if (!params.add_gap_fill) {
        //without gapfill, use the normal function
        return Fill::fill_surface_extrusion(surface, params, out);
    }

    ExtrusionEntitiesPtr out_to_check;

    double min_gapfill_area = double(params.flow.scaled_width()) * double(params.flow.scaled_width());
    if (params.config != nullptr) min_gapfill_area = scale_d(fill_region_option_effective_value(params.config, "gap_fill_min_area", params.flow.width())) * double(params.flow.scaled_width());
    // Perform offset. //FIXME: can miss gapfill outside of this first perimeter
    Slic3r::ExPolygons expp = offset_ex(surface->expolygon, scale_d(0 - 0.5 * this->get_spacing()));
    // Create the infills for each of the regions.
    Polylines polylines_out;
    for (size_t i = 0; i < expp.size(); ++i) {
        //_fill_surface_single(
        //params,
        //surface->thickness_layers,
        //_infill_direction(surface),
        //expp[i],
        //polylines_out);
        ExPolygon expolygon = expp[i];

        coordf_t init_spacing = this->get_spacing();

        // no rotation is supported for this infill pattern
        BoundingBox bounding_box = expolygon.contour.bounding_box();

        coord_t distance = _line_spacing_for_density(params);
        if (params.density > 0.9999f && !params.dont_adjust) {
            distance = scale_i(this->get_spacing());
        }
        std::vector<std::vector<Polygons>> bunch_2_shell_2_loops;
        bunch_2_shell_2_loops.emplace_back(); // create a new bunch before a gap
        bunch_2_shell_2_loops.back().push_back(to_polygons(expolygon)); // add first shell
        std::vector<ExPolygons> bunch_2_gaps; // size = bunch_2_shell_2_loops.size() (-1)
        ExPolygons last = { expolygon };
        bool first = true;
        while (!last.empty()) {
            ExPolygons next_onion = offset2_ex(last, -(distance + scale_d(this->get_spacing()) / 2), +(scale_d(this->get_spacing()) / 2));
            ExPolygons new_gaps = diff_ex(
                offset_ex(last, -0.5f * distance),
                offset_ex(next_onion, 0.5f * distance + 10));  // 10 = safety offset
            //add next shell (into the last collection)
            bunch_2_shell_2_loops.back().push_back(to_polygons(next_onion));
            //if there is some gaps, then we need to create a new collection for the next shells, so they will be peirnted after the gaps.
            if (!new_gaps.empty()) {
                if (first && !this->no_overlap_expolygons.empty()) {
                    new_gaps = intersection_ex(new_gaps, this->no_overlap_expolygons);
                }
                bunch_2_gaps.push_back(std::move(new_gaps));
                if (!bunch_2_shell_2_loops.back().empty() && bunch_2_shell_2_loops.back().back().empty()) {
                    bunch_2_shell_2_loops.back().pop_back();
                }
                //create a new collection for next bunch (if the loop don't stop here).
                if (!next_onion.empty()) {
                    bunch_2_shell_2_loops.emplace_back();
                }
            }
            // refresh before next iteration
            last = next_onion;
            first = false;
        }
        if (!bunch_2_shell_2_loops.back().empty() && bunch_2_shell_2_loops.back().back().empty()) {
            bunch_2_shell_2_loops.back().pop_back();
        }
        if (bunch_2_shell_2_loops.back().empty()) {
            assert(bunch_2_shell_2_loops.size() >= bunch_2_gaps.size());
            if (bunch_2_shell_2_loops.size() == bunch_2_gaps.size()) {
                assert(bunch_2_gaps.size() > 1);
                if (bunch_2_gaps.size() > 1) {
                    // merge last two gaps (inside the last perimeter & after the last perimeter are both extruded
                    // after the last perimeter)
                    append(bunch_2_gaps[bunch_2_gaps.size() - 2], bunch_2_gaps.back());
                    bunch_2_gaps.pop_back();
                }
            }
            bunch_2_shell_2_loops.pop_back();
            if (!bunch_2_shell_2_loops.empty() && !bunch_2_shell_2_loops.back().empty() && bunch_2_shell_2_loops.back().back().empty()) {
                bunch_2_shell_2_loops.back().pop_back();
            }
        }
        //check no empty shell
        assert(!bunch_2_shell_2_loops.back().empty());
        //check no empty gap
        assert(std::find_if(bunch_2_gaps.begin(), bunch_2_gaps.end(), [](const auto &vec) { return vec.empty(); }) ==
               bunch_2_gaps.end());
        // check size there is 3 posibilities:
        //   1) no gap: 1 shell
        //   2) gap but not in the last to iteration: X xhells, x-1 gap
        //   3) gap and there is some to print after the last shell: X shell, X gap
        assert(bunch_2_shell_2_loops.size() == bunch_2_gaps.size() + 1 ||
               bunch_2_shell_2_loops.size() == bunch_2_gaps.size());

        // generate paths from the outermost to the innermost, to avoid
        // adhesion problems of the first central tiny loops
        //note: useless if we don't apply no_sort flag
        //loops = union_pt_chained(loops, false);

        //get the role
        ExtrusionRole good_role = getRoleFromSurfaceType(params, surface);

        ExtrusionEntityUPtr root_collection_nosort = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children{}, false, false, false);

        //pattern (don't modify/move it)
        const ExtrusionEntity eec_pattern_no_sort(ExtrusionEntity::Children{}, false, false, false);

        assert(bunch_2_shell_2_loops.size() == bunch_2_gaps.size() || bunch_2_shell_2_loops.size() == bunch_2_gaps.size() + 1);
        //for each "shell" (loop to print before a gap)
        for (int idx_bunch = 0; idx_bunch < bunch_2_shell_2_loops.size(); idx_bunch++) {

            //we have some "starting loops". for each 'shell', we get each loop and find (by searching which one it fit inside) its island.
            // if there is none or multiple, then we have to start again from these new loops.
            std::vector<Polygons>& shells = bunch_2_shell_2_loops[idx_bunch];
            assert(!shells.empty());

            // leafs where you can add a loop (only one per shell, or you need to split it)
            std::vector<ExtrusionEntity*> leafs;

            //initialisation with first shell
            ExtrusionEntity *root_sortable = &root_collection_nosort->append_child(
                std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children{}, true, true, false));

            struct Leaf {
                size_t count;
                ExtrusionEntity* sortable;
            };
            // for each shell of this bunch
            
            for (size_t idx_shell = 0; idx_shell < shells.size(); ++idx_shell) {
                Polygons& islands = shells[idx_shell];
                std::vector<Leaf> nb_childs(leafs.size(), Leaf{ 0, nullptr });
                assert(nb_childs.size() == leafs.size());
                // for each island
                for (size_t idx_island = 0; idx_island < islands.size(); ++idx_island) {
                    //find a leafs to append it.
                    size_t added_idx = size_t(-1);
                    if (islands[idx_island].is_clockwise()) {
                        //hole -> it's the leafs that should be inside me
                        islands[idx_island].make_counter_clockwise();
                        for (size_t idx_child = 0; idx_child < nb_childs.size(); ++idx_child) {
                            assert(leafs[idx_child]->child_count() != 0);
                            assert(leafs[idx_child]->children().back()->is_loop());
                            if (islands[idx_island].contains(leafs[idx_child]->children().back()->first_point())) {
                                added_idx = idx_child;
                                break;
                            }
                        }
                    } else {
                        for (size_t idx_child = 0; idx_child < nb_childs.size(); ++idx_child) {
                            assert(leafs[idx_child]->child_count() != 0);
                            assert(leafs[idx_child]->children().back()->is_loop());
                            if (leafs[idx_child]->children().back()->is_loop() &&
                                Slic3r::polygon(*leafs[idx_child]->children().back()).contains(islands[idx_island].first_point())) {
                                added_idx = idx_child;
                                break;
                            }
                        }
                    }
                    if (added_idx != size_t(-1)) {
                        Leaf& leaf_count = nb_childs[added_idx];
                        ExtrusionEntity* leaf_coll = leafs[added_idx];
                        //check if it's taken
                        if (leaf_count.count == 0) {
                            append_concentric_loop(*leaf_coll, good_role, params, islands[idx_island]);
                        } else if (leaf_count.count == 1) {
                            //remove last entity (from the count==0) to put it into a new collection
                            ExtrusionEntityUPtr elt = std::move(leaf_coll->children().back());
                            leaf_coll->children().pop_back();
                            //add sortbale collection inside
                            leaf_count.sortable = &leaf_coll->append_child(
                                std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children{}, true, true, false));
                            ExtrusionEntity new_coll_nosort(ExtrusionEntity::Children{}, false, false, false);
                            new_coll_nosort.append_child(std::move(elt));
                            leaf_count.sortable->append_child(std::move(new_coll_nosort));
                        }
                        if (leaf_count.sortable) {
                            //add new collection
                            ExtrusionEntity new_coll_nosort(ExtrusionEntity::Children{}, false, false, false);
                            append_concentric_loop(new_coll_nosort, good_role, params, islands[idx_island]);
                            leaf_count.sortable->append_child(std::move(new_coll_nosort));
                        }
                        ++leaf_count.count;

                    } else {
                        //create new root (should only happen on the first shell 'initialisation')
                        leafs.push_back(&root_sortable->append_child(eec_pattern_no_sort));
                        append_concentric_loop(*leafs.back(), good_role, params, islands[idx_island]);
                    }
                }
                //remove old leafs
                assert(leafs.size() >= nb_childs.size());
                for (size_t idx_child = nb_childs.size() - 1; idx_child < nb_childs.size(); ++idx_child) {
                    if (nb_childs[idx_child].count > 1) {
                        leafs.erase(leafs.begin() + idx_child);
                    }
                }
            }
            //TODO: move items that are alone in a collection to the upper collection.

            //add gapfills
            if (idx_bunch < bunch_2_gaps.size() && !bunch_2_gaps[idx_bunch].empty() && params.density >= 1) {
                // get parameters 
                coordf_t min = 0.2 * distance * (1 - INSET_OVERLAP_TOLERANCE);
                //be sure we don't gapfill where the perimeters are already touching each other (negative spacing).
                min = std::max(min, double(Flow::new_from_spacing((float)EPSILON, (float)params.flow.nozzle_diameter(), (float)params.flow.height(), (float)params.flow.spacing_ratio(), false).scaled_width()));
                coordf_t real_max = 2.5 * distance;
                const coordf_t minwidth = scale_d(params.config->option("gap_fill_min_width")->get_effective_value(params.flow.width()));
                const coordf_t maxwidth = scale_d(params.config->option("gap_fill_max_width")->get_effective_value(params.flow.width()));
                const coord_t minlength = scale_i(params.config->option("gap_fill_min_length")->get_effective_value(params.flow.width()));
                if (minwidth > 0) {
                    min = std::max(min, minwidth);
                }
                coordf_t max = real_max;
                if (maxwidth > 0) {
                    max = std::min(max, maxwidth);
                }
                const coord_t gapfill_extension = scale_i(params.config->option("gap_fill_extension")->get_effective_value(params.flow.width()));

                // collapse 
                ExPolygons gaps_ex = diff_ex(
                    offset2_ex(bunch_2_gaps[idx_bunch], -min / 2, +min / 2),
                    offset2_ex(bunch_2_gaps[idx_bunch], -max / 2, +max / 2),
                    ApplySafetyOffset::Yes);
                ThickPolylines polylines;
                for (const ExPolygon& ex : gaps_ex) {
                    //remove too small gaps that are too hard to fill.
                    //ie one that are smaller than an extrusion with width of min and a length of max.
                    if (ex.area() > min_gapfill_area) {
                        Geometry::MedialAxis md{ ex, coord_t(real_max), coord_t(min), scale_i(params.flow.height()) };
                        if (minlength > 0) {
                            md.set_min_length(minlength);
                        }
                        if (gapfill_extension > 0) {
                            md.set_extension_length(gapfill_extension);
                        }
                        md.set_biggest_width(max);
                        md.build(polylines);
                    }
                }
                ////search if we can add some at the end of a leaf
                //for (size_t idx_polyline = 0; idx_polyline < polylines.size(); ++idx_polyline) {
                //    ThickPolyline& poly = polylines[idx_polyline];
                //    assert(!poly.empty());
                //    for (size_t idx_leaf = 0; idx_leaf < leafs.size(); ++idx_leaf) {
                //        assert(!leafs[idx_leaf]->entities().empty());
                //        const ExtrusionEntitiesPtr& leaf_entities = leafs[idx_leaf]->entities();
                //        //get last loop
                //        size_t idx_last_loop = leaf_entities.size() - 1;
                //        while (!leaf_entities[idx_last_loop]->is_loop()) {
                //            if (idx_last_loop == 0) {
                //                assert(false);
                //                //goto goto_next_polyline;
                //            }
                //            --idx_last_loop;
                //        }
                //        //test
                //        assert(leafs[idx_leaf]->entities()[idx_last_loop]->is_loop());
                //        if (leafs[idx_leaf]->entities()[idx_last_loop]->is_loop() &&
                //            static_cast<ExtrusionLoop*>(leafs[idx_leaf]->entities()[idx_last_loop])->polygon().contains(poly.points[poly.size() / 2])) {
                //            //do gapfill locally
                //            leafs[idx_leaf]->append(
                //                Geometry::variable_width(
                //                    poly, ExtrusionRole::GapFill, 
                //                    params.flow, 
                //                    scale_i(params.config->get_computed_value("resolution_internal")), 
                //                    params.flow.scaled_width() / 10)
                //            );
                //            polylines.erase(polylines.begin() + idx_polyline);
                //            --idx_polyline;
                //            break;
                //        }
                //    }
                //    //goto_next_polyline:
                //}
                bool fill_bridge = good_role.is_bridge() || params.flow.bridge();
                // allow bridged gapfill, mostly for support bottom interface.
                assert(!good_role.is_bridge());
                if (!polylines.empty()) {
                    ExtrusionEntitiesPtr gap_fill_entities = Geometry::thin_variable_width(polylines, ExtrusionRole::GapFill, params.flow, scale_i(params.config->get_computed_value("resolution_internal")), true);
                    if (!gap_fill_entities.empty()) {
                        // set role if needed
                        if (fill_bridge || (good_role != ExtrusionRole::SolidInfill && good_role != ExtrusionRole::TopSolidInfill)) {
                            ExtrusionSetRole set_good_role(good_role);
                            for (ExtrusionEntity* ptr : gap_fill_entities)
                                ptr->visit(set_good_role);
                        }

                        //move them into the collection
                        if (gap_fill_entities.size() == 1) {
                            root_collection_nosort->append_child(ExtrusionEntityUPtr(gap_fill_entities.front()));
                            gap_fill_entities.clear();
                        } else {
                            ExtrusionEntity gaps_collection(ExtrusionEntity::Children{}, true, true, false);
                            for (ExtrusionEntity *gap : gap_fill_entities)
                                gaps_collection.append_child(ExtrusionEntityUPtr(gap));
                            gap_fill_entities.clear();
                            root_collection_nosort->append_child(std::move(gaps_collection));
                        }
                    }
                }
            }
        }


        if (root_collection_nosort->child_count() != 0) {
            out_to_check.push_back(root_collection_nosort.get());
            root_collection_nosort.release();
        }
    }

    // external gapfill
    ExPolygons gapfill_areas = diff_ex(ExPolygons{ surface->expolygon }, offset_ex(expp, scale_d(0.5 * this->get_spacing())));
    gapfill_areas = union_safety_offset_ex(gapfill_areas);
    if (gapfill_areas.size() > 0 && no_overlap_expolygons.size() > 0) {
        double minarea = double(params.flow.scaled_width()) * double(params.flow.scaled_width());
        if (params.config != nullptr) minarea = scale_d(fill_region_option_effective_value(params.config, "gap_fill_min_area", params.flow.width())) * double(params.flow.scaled_width());
        for (int i = 0; i < gapfill_areas.size(); i++) {
            if (gapfill_areas[i].area() < minarea) {
                gapfill_areas.erase(gapfill_areas.begin() + i);
                i--;
            }
        }
        FillParams params2{ params };
        params2.role = ExtrusionRole::GapFill;

        do_gap_fill(intersection_ex(gapfill_areas, no_overlap_expolygons), params2, out_to_check);
    }

    // check volume coverage
    if (!out_to_check.empty()) {
        double mult_flow = 1;
        // check if not over-extruding
        if (!params.dont_adjust && params.full_infill() && !params.flow.bridge() && params.fill_exactly) {
            // compute the path of the nozzle -> extruded volume
            double length_tot = 0;
            int    nb_lines   = 0;
            ExtrusionVolume get_volume;
            for (ExtrusionEntity *ee : out_to_check) get_volume.traverse(*ee);
            // compute flow to remove spacing_ratio from the equation
            // compute real volume to fill
            double polyline_volume = compute_unscaled_volume_to_fill(surface, params);
            if (get_volume.volume != 0 && polyline_volume != 0)
                mult_flow = polyline_volume / get_volume.volume;
            // failsafe, it can happen
            if (mult_flow > 1.3)
                mult_flow = 1.3;
            if (mult_flow < 0.8)
                mult_flow = 0.8;
            BOOST_LOG_TRIVIAL(debug) << "concentric Infill (with gapfil) process extrude " << get_volume.volume
                                    << " mm3 for a volume of " << polyline_volume << " mm3 : we mult the flow by "
                                    << mult_flow;
#if _DEBUG
            this->debug_verify_flow_mult = mult_flow;
#endif
        }
        mult_flow *= params.flow_mult;
        if (mult_flow != 1) {
            // apply to extrusions
            ExtrusionModifyFlow visitor(mult_flow);
            for (ExtrusionEntity *ee : out_to_check) {
                visitor.traverse(*ee);
            }
        }
    }

    out.insert(out.end(), out_to_check.begin(), out_to_check.end());
}

void FillConcentric::_fill_surface_single(const FillParams              &params,
                                          unsigned int                   thickness_layers,
                                          const std::pair<float, Point> &direction,
                                          ExPolygon                      expolygon,
                                          ThickPolylines                &thick_polylines_out) const
{
    assert(params.use_arachne);
    assert(this->print_config != nullptr && this->print_object_config != nullptr);

    // no rotation is supported for this infill pattern
    Point   bbox_size   = expolygon.contour.bounding_box().size();
    coord_t min_spacing = scale_i(this->get_spacing());
    coord_t min_width = params.flow.scaled_width();

    if (params.density > 0.9999f && !params.dont_adjust) {
        coord_t                loops_count = std::max(bbox_size.x(), bbox_size.y()) / min_spacing + 1;
        Polygons               polygons    = offset(expolygon, float(min_spacing) / 2.f);
        Arachne::WallToolPaths wallToolPaths(polygons, min_spacing, min_width, min_spacing, min_width, loops_count, 0, params.layer_height, *params.config, *this->print_config);

        std::vector<Arachne::VariableWidthLines>    loops = wallToolPaths.getToolPaths();
        std::vector<const Arachne::ExtrusionLine *> all_extrusions;
        for (Arachne::VariableWidthLines &loop : loops) {
            if (loop.empty())
                continue;
            for (const Arachne::ExtrusionLine &wall : loop)
                all_extrusions.emplace_back(&wall);
        }

        // Split paths using a nearest neighbor search.
        size_t firts_poly_idx = thick_polylines_out.size();
        Point  last_pos(0, 0);
        for (const Arachne::ExtrusionLine *extrusion : all_extrusions) {
            if (extrusion->empty())
                continue;

            ThickPolyline thick_polyline = Arachne::to_thick_polyline(*extrusion);
            if (extrusion->is_closed)
                thick_polyline.start_at_index(nearest_point_index(thick_polyline.points, last_pos));
            thick_polylines_out.emplace_back(std::move(thick_polyline));
            last_pos = thick_polylines_out.back().back();
        }

        // clip the paths to prevent the extruder from getting exactly on the first point of the loop
        // Keep valid paths only.
        size_t j = firts_poly_idx;
        for (size_t i = firts_poly_idx; i < thick_polylines_out.size(); ++i) {
            thick_polylines_out[i].clip_end(this->loop_clipping);
            if (thick_polylines_out[i].is_valid()) {
                if (j < i)
                    thick_polylines_out[j] = std::move(thick_polylines_out[i]);
                ++j;
            }
        }
        if (j < thick_polylines_out.size())
            thick_polylines_out.erase(thick_polylines_out.begin() + int(j), thick_polylines_out.end());
    } else {
        Polylines polylines;
        this->_fill_surface_single(params, thickness_layers, direction, expolygon, polylines);
        append(thick_polylines_out, to_thick_polylines(std::move(polylines), min_spacing));
    }
}

} // namespace Slic3r
