///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/*
Tree supports by Thomas Rahm, losely based on Tree Supports by CuraEngine.
   Original CuraEngine copyright:
   Copyright (c) 2021 Ultimaker B.V.
   CuraEngine is released under the terms of the AGPLv3 or higher. */
#include "TreeRouting.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <numeric>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/log/trivial.hpp>

#include <oneapi/tbb/parallel_for.h>

#include "libslic3r/AABBTreeIndirect.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/EdgeGrid.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/MultiPoint.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Support/SupportParameters.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Plugins/Support/Common/SupportLayers.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"
#include "TreeElements.hpp"
#include "TreeSettings.hpp"
#include "TreeStorage.hpp"
#include "TreeVolumes.hpp"

/* Routing of the tree branches of one object: tips, influence areas propagated down the object layers, and nodes.

   The routing runs once per object, in this order:
   1. route_trees() adds the top contacts that the island support demand asks for, through the shared helper
      (Common/SupportZPlacement.hpp), and lists the tip sources: the contact area and the object layer where its tips start.
   2. TreeVolumes precalculates the collision, avoidance and placeable areas of the object layers.
   3. generate_initial_areas() samples the tips under each contact and creates one influence area per tip.
   4. create_layer_pathing() moves every influence area one object layer down at a time, within the maximum move of
      the layer, and merges the areas that come to overlap.
   5. create_nodes_from_area() chooses the point each influence area is drawn through.
   6. build_trees() connects the placed elements through their parents into routed trees.

   The code is the legacy tree support (Support/TreeSupport.cpp), copied into this namespace. Drawing, roofs, rafts
   and organic branches are not part of it. The tips start under the top interface stack, and the influence areas use
   the real height of each layer for their maximum move.
*/

namespace Slic3r::TreeFamily {

namespace Common = slic3r_api::Support::Common;

using namespace std::literals;
using FFFSupport::SupportParameters;

struct SupportElementInfluenceAreas;
struct SupportElementMerging;

// A top contact that the tips support: its contact area, the object layer where its tips start, the number of
// interface layers that the tips stay vertical under, and the object layer of the contacts it stands for.
struct TipSource {
    Polygons area;
    LayerIndex layer;
    size_t dont_move_until;
    size_t object_above;
};

// Declarations of the functions of this file, in the order of their definitions below.
// Adds the top contacts of the object layers with support demand, through the shared helper.
static void generate_contacts(const slic3r_api::Print &print, const slic3r_api::Object &object_view, const PrintObject &object,
                       const Common::SupportParams &params, const TreeSettings &settings, Common::GeneratedSupport &support);
// Gives a top contact its geometry: the contact area in its polygons and its contact and overhang polygons, kept in the storage of the support.
static void store_top_contact(Common::GeneratorLayer &slab, storage_handle *storage, const ExPolygons &contact, const ExPolygons &overhang);
// Lists the tip sources of the top contacts: one per connected part of the contact areas, each with the object layer where its tips start.
static std::vector<TipSource> tip_sources(const PrintObject &object, const TreeSettings &settings, const slic3r_api::Config &config,
                                   const Common::GeneratedSupport &support);
// Keeps on each top contact only the components that a placed tip reaches, and drops the contacts left empty.
static void trim_contacts_to_placed_tips(const std::vector<SupportElements> &move_bounds, const std::vector<TipSource> &sources,
                                         const PrintObject &object, Common::GeneratedSupport &support);
// Places the tips under every tip source and creates their influence areas.
static void generate_initial_areas(const TreeVolumes &volumes, const TreeSettings &config, const FFFSupport::SupportParameters &support_params,
                            const std::vector<TipSource> &sources, std::vector<SupportElements> &move_bounds);
// Places evenly spaced points along polylines, at most the given distance apart.
[[nodiscard]] static Polylines ensure_maximum_distance_polyline(const Polylines &input, double distance, size_t min_points);
// Finds the point at the given distance from a start point, along a polyline.
static std::optional<std::pair<Point, size_t>> polyline_sample_next_point_at_distance(const Points &polyline, const Point &start_pt, size_t start_idx, double dist);
// Lines of the support pattern that cover a polygon, used as the tip sampling lines.
[[nodiscard]] static Polylines generate_support_infill_lines(
    const Polygons          &polygon,
    const SupportParameters &support_params,
    LayerIndex layer_idx, coord_t support_infill_distance);
// Converts polylines into lines whose points know which avoidance they are valid in.
[[nodiscard]] static LineInformations convert_lines_to_internal(
    const TreeVolumes &volumes, const TreeSettings &config,
    const Polylines &polylines, LayerIndex layer_idx);
// Offsets a polygon in steps that each stay outside the collision.
[[nodiscard]] static Polygons safe_offset_inc(const Polygons& me, coord_t distance, const Polygons& collision, coord_t safe_step_size, coord_t last_step_offset_without_check, size_t min_amount_offset);
// Unions two polygon sets, keeping the result non empty when an input is.
[[nodiscard]] static Polygons safe_union(const Polygons first, const Polygons second = {});
// Propagates the influence areas down the object layers, merging the ones that overlap.
static void create_layer_pathing(const TreeVolumes &volumes, const TreeSettings &config, std::vector<SupportElements> &move_bounds, std::function<void()> throw_on_cancel);
// Merges the influence areas of one layer that overlap, bucket by bucket.
static void merge_influence_areas(
    const TreeVolumes             &volumes,
    const TreeSettings          &config,
    const LayerIndex                    layer_idx,
    std::vector<SupportElementMerging> &influence_areas,
    std::function<void()>               throw_on_cancel);
// Merges two buckets of influence areas into one bucket.
static SupportElementMerging* merge_influence_areas_two_sets(
    const TreeVolumes &volumes, const TreeSettings &config, const LayerIndex layer_idx,
    SupportElementMerging * const dst_begin, SupportElementMerging *       dst_end,
    SupportElementMerging *       src_begin, SupportElementMerging * const src_end);
// Merges the influence areas of the leaves of a bucket tree.
static SupportElementMerging* merge_influence_areas_leaves(
    const TreeVolumes &volumes, const TreeSettings &config, const LayerIndex layer_idx,
    SupportElementMerging * const dst_begin, SupportElementMerging *dst_end);
// Merges two influence areas of one layer when they overlap enough.
static bool merge_influence_areas_two_elements(
    const TreeVolumes &volumes, const TreeSettings &config, const LayerIndex layer_idx,
    SupportElementMerging &dst, SupportElementMerging &src);
// Combines the states of two elements that merge into one.
[[nodiscard]] static SupportElementState merge_support_element_states(
    const SupportElementState &first, const SupportElementState &second, const Point &next_position, const coord_t layer_idx,
    const TreeSettings &config);
// Computes the influence areas of the layer below from the elements of the layer above.
static void increase_areas_one_layer(
    const TreeVolumes              &volumes,
    const TreeSettings           &config,
    std::vector<SupportElementMerging>  &merging_areas,
    const LayerIndex                     layer_idx,
    SupportElements                     &layer_elements,
    const bool                           mergelayer,
    std::function<void()>                throw_on_cancel);
// Computes the influence area of one element on the layer below, within the maximum move.
[[nodiscard]] static std::optional<SupportElementState> increase_single_area(
    const TreeVolumes      &volumes,
    const TreeSettings   &config,
    const AreaIncreaseSettings  &settings,
    const LayerIndex             layer_idx,
    const SupportElement        &parent,
    const Polygons              &relevant_offset,
    Polygons                    &to_bp_data,
    Polygons                    &to_model_data,
    Polygons                    &increased,
    const coord_t                overspeed,
    const bool                   mergelayer);
// Returns the point, moved inside the polygons when it lies outside them.
static Point move_inside_if_outside(const Polygons &polygons, Point from, int distance = 0, int64_t maxDist2 = std::numeric_limits<int64_t>::max());
// Moves a point to the closest polygon inside, within the given distance.
static unsigned int move_inside(const Polygons &polygons, Point &from, int distance = 0, int64_t maxDist2 = std::numeric_limits<int64_t>::max());
// Chooses the point that each influence area is drawn through, from the bottom layer up.
static void create_nodes_from_area(
    const TreeVolumes       &volumes,
    const TreeSettings    &config,
    std::vector<SupportElements> &move_bounds,
    std::function<void()>         throw_on_cancel);
// Sets the point of each element's influence area from the element above it.
static void set_points_on_areas(const SupportElement &elem, SupportElements *layer_above);
// Places the last element of a branch that rests on the model, at its nearest valid point.
static void set_to_model_contact_simple(SupportElement &elem);
// Places the branch that rests on the model, over its last layers that stay valid.
static void set_to_model_contact_to_model_gracious(
    const TreeVolumes          &volumes,
    const TreeSettings       &config,
    std::vector<SupportElements>    &move_bounds,
    SupportElement                  &first_elem,
    std::function<void()>            throw_on_cancel);
// Removes the deleted elements and renumbers the parents of the others.
static void remove_deleted_elements(std::vector<SupportElements> &move_bounds);
// Gathers the placed elements into routed trees, from the top layer down.
static std::vector<RoutedTree> build_trees(const PrintObject &object, const TreeSettings &config, const std::vector<SupportElements> &move_bounds);
// Finds the representative of an element in the union-find forest.
static size_t find_root(std::vector<size_t> &forest, size_t idx);

// Routes the tree branches of one object: contacts from the shared helper, tips under them, influence areas propagated down the
// object layers, then the nodes of each layer. The plugin views wrap the same handles as the native objects they are built from.
std::vector<RoutedTree> route_trees(const Print &print, const PrintObject &object, const Common::SupportParams &params, const TreeSettings &settings,
                                    Common::GeneratedSupport &support)
{
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const slic3r_api::Object object_view(reinterpret_cast<const object_handle *>(&object));
    generate_contacts(print_view, object_view, object, params, settings, support);
    const std::vector<TipSource> sources = tip_sources(object, settings, object_view.config(), support);
    if (sources.empty())
        return {};
    LayerIndex max_layer = 0;
    for (const TipSource &source : sources)
        max_layer = std::max(max_layer, source.layer);
    TreeVolumes volumes(object, settings);
    volumes.precalculate(max_layer, [] {});
    const FFFSupport::SupportParameters support_params(object);
    std::vector<SupportElements> move_bounds(object.layer_count());
    generate_initial_areas(volumes, settings, support_params, sources, move_bounds);
    create_layer_pathing(volumes, settings, move_bounds, [] {});
    create_nodes_from_area(volumes, settings, move_bounds, [] {});
    trim_contacts_to_placed_tips(move_bounds, sources, object, support);
    return build_trees(object, settings, move_bounds);
}

// Creates the top contact of every object layer that has support demand. The contact area is the demand minus the
// object layer below, grown by the XY distance of overhang outlines; the shared helper places the contact in Z and
// merges the close contacts.
static void generate_contacts(const slic3r_api::Print &print, const slic3r_api::Object &object_view, const PrintObject &object,
                       const Common::SupportParams &params, const TreeSettings &settings, Common::GeneratedSupport &support)
{
    const slic3r_api::Config print_config = print.config();
    std::vector<slic3r_api::Layer> layers;
    for (uint32_t id = 0; id < object_view.layer_count(); ++id)
        layers.push_back(object_view.layer(id));
    for (size_t id = 1; id < object.layer_count(); ++id) {
        ExPolygons demand;
        for (const LayerSliceIsland &island : object.layer(id).islands())
            append(demand, island.support_demand());
        if (demand.empty())
            continue;
        demand = union_ex(demand);
        const ExPolygons margin = offset_ex(object.layer(id - 1).lslices(), float(settings.xy_min_distance), ClipperLib::jtMiter, 1.2);
        const ExPolygons contact = diff_ex(demand, margin);
        if (contact.empty())
            continue;
        const Common::TopContactSlabs slabs = Common::add_top_contacts(layers, print_config, params, id, support);
        if (!slabs.contact)
            continue;
        store_top_contact(*slabs.contact, support.storage, contact, demand);
        if (slabs.bridge)
            store_top_contact(*slabs.bridge, support.storage, contact, demand);
    }
    Common::merge_contacts(params, support);
}

// Gives a top contact its geometry: the contact area, which is also its printed polygons, and the demand it came from.
static void store_top_contact(Common::GeneratorLayer &slab, storage_handle *storage, const ExPolygons &contact, const ExPolygons &overhang)
{
    slab.contact_polygons.emplace(storage);
    slab.overhang_polygons.emplace(storage);
    native_polygons(slab.polygons) = to_polygons(contact);
    native_polygons(*slab.contact_polygons) = to_polygons(contact);
    native_polygons(*slab.overhang_polygons) = to_polygons(overhang);
}

// Lists where the tips of each object layer with top contacts start. The stack of interface slabs hangs under the contact extreme, so the tips
// start on the highest object layer that does not rise above the bottom of that stack. Without interface layers the stack is
// the contact extreme itself. The contacts of one object layer (a bridge and the normal contact, which the merge may keep apart,
// and the demand that the merge adds to a contact) share one source: its area is the union, and it starts from the lowest contact.
static std::vector<TipSource> tip_sources(const PrintObject &object, const TreeSettings &settings, const slic3r_api::Config &config,
                                   const Common::GeneratedSupport &support)
{
    const coord_t step = settings.dense_slab_height;
    const int32_t interfaces = std::max(0, config.get("support_material_interface_layers").get_int());
    constexpr size_t no_source = std::numeric_limits<size_t>::max();
    std::vector<size_t> source_of_layer(object.layer_count(), no_source);
    std::vector<TipSource> sources;
    // The top contacts are sorted by print Z, so the first contact of an object layer is its lowest one.
    for (const Common::GeneratorLayer *contact : support.top) {
        const Polygons &area = native_polygons(*contact->contact_polygons);
        size_t &source = source_of_layer[contact->object_above];
        if (source != no_source) {
            sources[source].area = union_(sources[source].area, area);
            continue;
        }
        source = sources.size();
        const coord_t stack_bottom = contact->extreme_z() - coord_t(interfaces) * step;
        LayerIndex layer = 0;
        for (LayerIndex idx = 0; idx < LayerIndex(object.layer_count()) && object.layer(idx).scaled_print_z() <= stack_bottom; ++ idx)
            layer = idx;
        sources.push_back(TipSource{ area, layer, size_t(interfaces), contact->object_above });
    }
    // Each connected part of a contact area is a source of its own, so that a part without a placed tip is removed alone.
    std::vector<TipSource> parts;
    for (const TipSource &source : sources)
        for (const ExPolygon &part : union_ex(source.area))
            parts.push_back(TipSource{ to_polygons(part), source.layer, source.dont_move_until, source.object_above });
    return parts;
}

// A part of a top contact that no placed tip reaches is cut from the contact, since its roof would have no branch under it.
static void trim_contacts_to_placed_tips(const std::vector<SupportElements> &move_bounds, const std::vector<TipSource> &sources,
                                         const PrintObject &object, Common::GeneratedSupport &support)
{
    std::vector<bool> reached(sources.size(), false);
    for (const SupportElements &layer : move_bounds)
        for (const SupportElement &elem : layer)
            if (elem.tip_source < sources.size() && elem.state.result_on_layer_is_set())
                reached[elem.tip_source] = true;
    std::vector<Polygons> placed(object.layer_count());
    for (size_t idx = 0; idx < sources.size(); ++idx)
        if (reached[idx])
            placed[sources[idx].object_above].insert(placed[sources[idx].object_above].end(), sources[idx].area.begin(), sources[idx].area.end());
    for (Common::GeneratorLayer *contact : support.top) {
        const Polygons reached_area = union_(placed[contact->object_above]);
        native_polygons(contact->polygons) = intersection(native_polygons(contact->polygons), reached_area);
        native_polygons(*contact->contact_polygons) = intersection(native_polygons(*contact->contact_polygons), reached_area);
    }
    support.top.erase(std::remove_if(support.top.begin(), support.top.end(),
        [](const Common::GeneratorLayer *contact) { return native_polygons(contact->polygons).empty(); }), support.top.end());
}

// Inserts the tips of one object as influence areas on their object layer. The tip sources are sampled in parallel, so the
// insertions are guarded; a point already covered by a tip of the same layer is not inserted twice.
class TipPlacer {
public:
    TipPlacer(const TreeSettings &config, bool min_xy_dist, std::vector<SupportElements> &move_bounds);
    // Insert every point of the lines as a tip on insert_layer, which stays in place for dont_move_until layers; source is the
    // index of the tip source that the tips belong to.
    void add_points_along_lines(const LineInformations &lines, LayerIndex insert_layer, size_t dont_move_until, size_t source);

private:
    // Insert one point as a tip with a small base circle, unless a tip already covers it.
    void add_point(const std::pair<Point, LineStatus> &point, LayerIndex insert_layer, size_t dont_move_until, bool skip_ovalisation,
                   size_t source);

    const TreeSettings &m_config;
    const bool m_min_xy_dist;
    std::vector<SupportElements> &m_move_bounds;
    std::mutex m_mutex;
    std::vector<std::unordered_set<Point, PointHash>> m_already_inserted;
    const Polygon m_base_circle;
};

TipPlacer::TipPlacer(const TreeSettings &config, bool min_xy_dist, std::vector<SupportElements> &move_bounds) :
    m_config(config), m_min_xy_dist(min_xy_dist), m_move_bounds(move_bounds),
    m_already_inserted(move_bounds.size()), m_base_circle(make_circle(scale_d(0.01), SUPPORT_TREE_CIRCLE_RESOLUTION))
{}

// Places the tips under every tip source. The contact area is grown into the free space of the tip layer as far as the
// tip radius needs, then each of its parts is sampled along lines spaced by the branch distance.
static void generate_initial_areas(const TreeVolumes &volumes, const TreeSettings &config, const FFFSupport::SupportParameters &support_params,
                            const std::vector<TipSource> &sources, std::vector<SupportElements> &move_bounds)
{
    using AvoidanceType = TreeVolumes::AvoidanceType;
    const bool min_xy_dist = config.xy_distance > config.xy_min_distance;
    const coord_t connect_length = (config.support_line_width * 100. / config.top_rate) + std::max(2. * config.min_radius - 1.0 * config.support_line_width, 0.0);
    // As r*r=x*x+y*y (circle equation): the distance along x at which a circle of radius min_radius dips by half a line width.
    const coord_t circle_length_to_half_linewidth_change = config.min_radius < config.support_line_width ?
        config.min_radius / 2 :
        scale_i(sqrt(sqr(unscaled(config.min_radius)) - sqr(unscaled(config.min_radius - config.support_line_width / 2))));
    // Extra outset that compensates for the tip radius, and for the XY distance when the XY distance overrides the Z distance.
    const coord_t extra_outset = std::max(coord_t(0), config.min_radius - config.support_line_width / 2) + (min_xy_dist ? config.support_line_width / 2 : 0);
    TipPlacer placer(config, min_xy_dist, move_bounds);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, sources.size()),
        [&](const tbb::blocked_range<size_t> &range) {
        for (size_t source_idx = range.begin(); source_idx < range.end(); ++ source_idx) {
            const TipSource &source = sources[source_idx];
            const LayerIndex layer_idx = source.layer;
            // The least restrictive forbidden area for the tip radius.
            Polygons relevant_forbidden;
            {
                const Polygons &relevant_forbidden_raw = config.support_rests_on_model ?
                    volumes.getCollision(config.getRadius(0), layer_idx, min_xy_dist) :
                    volumes.getAvoidance(config.getRadius(0), layer_idx, AvoidanceType::Fast, false, min_xy_dist);
                // prevent rounding errors down the line, points placed directly on the line of the forbidden area may not be added otherwise.
                relevant_forbidden = offset(union_ex(relevant_forbidden_raw), scale_d(0.005), jtMiter, 1.2);
            }
            Polygons overhang_regular = safe_offset_inc(source.area, 0, relevant_forbidden, config.min_radius * 1.75 + config.xy_min_distance, 0, 1);
            Polygons remaining_overhang = intersection(
                diff(source.area, offset(union_ex(overhang_regular), config.support_line_width * 0.5, jtMiter, 1.2)),
                relevant_forbidden);
            // Grow the supported area in steps, while the remaining overhang still needs support.
            for (coord_t extra_total_offset_acc = 0; ! remaining_overhang.empty() && extra_total_offset_acc + config.support_line_width / 8 < extra_outset; ) {
                const coord_t offset_current_step = std::min(
                    extra_total_offset_acc + 2 * config.support_line_width > config.min_radius ?
                        config.support_line_width / 8 :
                        circle_length_to_half_linewidth_change,
                    extra_outset - extra_total_offset_acc);
                extra_total_offset_acc += offset_current_step;
                const Polygons &raw_collision = volumes.getCollision(0, layer_idx, true);
                const coord_t offset_step = config.xy_min_distance + config.support_line_width;
                // Reduce the remaining overhang by the areas already supported, then extend the supported area by what is left.
                remaining_overhang = diff(remaining_overhang, safe_offset_inc(overhang_regular, 1.5 * extra_total_offset_acc, raw_collision, offset_step, 0, 1));
                overhang_regular = union_(overhang_regular, diff(safe_offset_inc(remaining_overhang, extra_total_offset_acc, raw_collision, offset_step, 0, 1), relevant_forbidden));
            }
            for (ExPolygon &part : union_ex(overhang_regular)) {
                const Polygons part_area = to_polygons(std::move(part));
                Polylines polylines = ensure_maximum_distance_polyline(
                    generate_support_infill_lines(part_area, support_params, layer_idx, config.branch_distance), connect_length, 1);
                size_t point_count = 0;
                for (const Polyline &poly : polylines)
                    point_count += poly.size();
                const size_t min_support_points = std::max(coord_t(1), std::min(coord_t(3), coord_t(total_length(part_area) / connect_length)));
                if (point_count <= min_support_points) {
                    // Too few points to hold the part: sample the outline instead, so that the tips support the outer wall.
                    const Polygons reduced_overhang_area = offset(union_ex(part_area), - config.support_line_width / 2.2, jtMiter, 1.2);
                    polylines = ensure_maximum_distance_polyline(
                        to_polylines(
                            ! reduced_overhang_area.empty() &&
                            area(offset(diff_ex(part_area, reduced_overhang_area), std::max(config.support_line_width, connect_length), jtMiter, 1.2)) < sqr(scale_d(0.001)) ?
                            reduced_overhang_area :
                            part_area),
                        connect_length, min_support_points);
                }
                const LineInformations overhang_lines = convert_lines_to_internal(volumes, config, polylines, layer_idx);
                placer.add_points_along_lines(overhang_lines, layer_idx, source.dont_move_until, source_idx);
            }
        }
    });
}

void TipPlacer::add_points_along_lines(const LineInformations &lines, LayerIndex insert_layer, size_t dont_move_until, size_t source)
{
    for (const LineInformation &line : lines) {
        // A line of many tips simulates a support pattern, so its tips keep their ovalisation disabled to stay a clean line.
        const bool disable_ovalistation = m_config.min_radius < 3 * m_config.support_line_width && line.size() > 5;
        for (const std::pair<Point, LineStatus> &point_data : line)
            add_point(point_data, insert_layer, dont_move_until, disable_ovalistation, source);
    }
}

void TipPlacer::add_point(const std::pair<Point, LineStatus> &point, LayerIndex insert_layer, size_t dont_move_until, bool skip_ovalisation,
                          size_t source)
{
    const bool to_bp = point.second == LineStatus::TO_BP || point.second == LineStatus::TO_BP_SAFE;
    const bool gracious = to_bp || point.second == LineStatus::TO_MODEL_GRACIOUS || point.second == LineStatus::TO_MODEL_GRACIOUS_SAFE;
    const bool safe_radius = point.second == LineStatus::TO_BP_SAFE || point.second == LineStatus::TO_MODEL_GRACIOUS_SAFE;
    if (! m_config.support_rests_on_model && ! to_bp) {
        BOOST_LOG_TRIVIAL(warning) << "Tried to add an invalid support point";
        return;
    }
    Polygons circle{ m_base_circle };
    circle.front().translate(point.first);
    // Points closer than a tenth of the tip radius are the same tip for the purpose of the insertion.
    const Point hash_pos = point.first / ((m_config.min_radius + 1) / 10);
    std::lock_guard<std::mutex> critical_section(m_mutex);
    if (m_already_inserted[insert_layer].count(hash_pos))
        return;
    m_already_inserted[insert_layer].emplace(hash_pos);
    SupportElementState state;
    state.target_height = insert_layer;
    state.target_position = point.first;
    state.next_position = point.first;
    state.layer_idx = insert_layer;
    state.effective_radius_height = 0;
    state.to_buildplate = to_bp;
    state.distance_to_top = 0;
    state.result_on_layer = point.first;
    state.increased_to_model_radius = 0;
    state.to_model_gracious = gracious;
    state.elephant_foot_increases = 0;
    state.use_min_xy_dist = m_min_xy_dist;
    state.dont_move_until = dont_move_until;
    state.can_use_safe_radius = safe_radius;
    state.skip_ovalisation = skip_ovalisation;
    m_move_bounds[insert_layer].emplace_back(state, std::move(circle)).tip_source = source;
}

/*!
 * \brief Eensures that every line segment is about distance in length. The resulting lines may differ from the original but all points are on the original
 *
 * \param input[in] The lines on which evenly spaced points should be placed.
 * \param distance[in] The distance the points should be from each other.
 * \param min_points[in] The amount of points that have to be placed. If not enough can be placed the distance will be reduced to place this many points.
 * \return A Polygons object containing the evenly spaced points. Does not represent an area, more a collection of points on lines.
 */
[[nodiscard]] static Polylines ensure_maximum_distance_polyline(const Polylines &input, double distance, size_t min_points)
{
    Polylines result;
    for (Polyline part : input) {
        if (part.empty())
            continue;

        double len = length(part.points);
        Polyline line;
        double current_distance = std::max(distance, scale_d(0.1));
        if (len < 2 * distance && min_points <= 1)
        {
            // Insert the opposite point of the first one.
            //FIXME pretty expensive
            Polyline pl(part);
            pl.clip_end(len / 2);
            line.points.emplace_back(pl.points.back());
        }
        else
        {
            size_t optimal_end_index = part.size() - 1;

            if (part.front() == part.back()) {
                size_t optimal_start_index = 0;
                // If the polyline was a polygon, there is a high chance it was an overhang. Overhangs that are <60� tend to be very thin areas, so lets get the beginning and end of them and ensure that they are supported.
                // The first point of the line will always be supported, so rotate the order of points in this polyline that one of the two corresponding points that are furthest from each other is in the beginning.
                // The other will be manually added (optimal_end_index)
                coord_t max_dist2_between_vertecies = 0;
                for (size_t idx = 0; idx < part.size() - 1; ++ idx) {
                    for (size_t inner_idx = 0; inner_idx < part.size() - 1; inner_idx++) {
                        if ((part[idx] - part[inner_idx]).cast<double>().squaredNorm() > max_dist2_between_vertecies) {
                            optimal_start_index = idx;
                            optimal_end_index = inner_idx;
                            max_dist2_between_vertecies = (part[idx] - part[inner_idx]).cast<double>().squaredNorm();
                        }
                    }
                }
                std::rotate(part.begin(), part.begin() + optimal_start_index, part.end() - 1);
                part[part.size() - 1] = part[0]; // restore that property that this polyline ends where it started.
                optimal_end_index = (part.size() + optimal_end_index - optimal_start_index - 1) % (part.size() - 1);
            }

            while (line.size() < min_points && current_distance >= scale_d(0.1))
            {
                line.clear();
                Point current_point = part[0];
                line.points.emplace_back(part[0]);
                if (min_points > 1 || (part[0] - part[optimal_end_index]).cast<double>().norm() > current_distance)
                    line.points.emplace_back(part[optimal_end_index]);
                size_t current_index = 0;
                std::optional<std::pair<Point, size_t>> next_point;
                double next_distance = current_distance;
                // Get points so that at least min_points are added and they each are current_distance away from each other. If that is impossible, decrease current_distance a bit.
                // The input are lines, that means that the line from the last to the first vertex does not have to exist, so exclude all points that are on this line!
                while ((next_point = polyline_sample_next_point_at_distance(part.points, current_point, current_index, next_distance))) {
                    // Not every point that is distance away, is valid, as it may be much closer to another point. This is especially the case when the overhang is very thin.
                    // So this ensures that the points are actually a certain distance from each other.
                    // This assurance is only made on a per polygon basis, as different but close polygon may not be able to use support below the other polygon.
                    double min_distance_to_existing_point = std::numeric_limits<double>::max();
                    for (Point p : line)
                        min_distance_to_existing_point = std::min(min_distance_to_existing_point, (p - next_point->first).cast<double>().norm());
                    if (min_distance_to_existing_point >= current_distance) {
                        // viable point was found. Add to possible result.
                        line.points.emplace_back(next_point->first);
                        current_point = next_point->first;
                        current_index = next_point->second;
                        next_distance = current_distance;
                    } else {
                        if (current_point == next_point->first) {
                            // In case a fixpoint is encountered, better aggressively overcompensate so the code does not become stuck here...
                            BOOST_LOG_TRIVIAL(warning) << "Tree Support: Encountered a fixpoint in polyline_sample_next_point_at_distance. This is expected to happen if the distance (currently " << next_distance <<
                                ") is smaller than 100";
                            BOOST_LOG_TRIVIAL(error) << "Encountered issue while placing tips. Some tips may be missing."sv;
                            if (next_distance > 2 * current_distance)
                                // This case should never happen, but better safe than sorry.
                                break;
                            next_distance += current_distance;
                            continue;
                        }
                        // if the point was too close, the next possible viable point is at least distance-min_distance_to_existing_point away from the one that was just checked.
                        next_distance = std::max(current_distance - min_distance_to_existing_point, scale_d(0.1));
                        current_point = next_point->first;
                        current_index = next_point->second;
                    }
                }
                current_distance *= 0.9;
            }
        }
        result.emplace_back(std::move(line));
    }
    return result;
}

// Ported from CURA's PolygonUtils::getNextPointWithDistance()
// Sample a next point at distance "dist" from start_pt on polyline segment (start_idx, start_idx + 1).
// Returns sample point and start index of its segment on polyline if such sample exists.
static std::optional<std::pair<Point, size_t>> polyline_sample_next_point_at_distance(const Points &polyline, const Point &start_pt, size_t start_idx, double dist)
{
    const double                dist2  = sqr(dist);
    const auto                  dist2i = int64_t(dist2);
    static constexpr const auto eps    = scale_d(0.01);

    for (size_t i = start_idx + 1; i < polyline.size(); ++ i) {
        const Point p1 = polyline[i];
        if ((p1 - start_pt).cast<int64_t>().squaredNorm() >= dist2i) {
            // The end point is outside the circle with center "start_pt" and radius "dist".
            const Point p0  = polyline[i - 1];
            Vec2d       v   = (p1 - p0).cast<double>();
            double      l2v = v.squaredNorm();
            if (l2v < sqr(eps)) {
                // Very short segment.
                Point c = (p0 + p1) / 2;
                if (std::abs((start_pt - c).cast<double>().norm() - dist) < eps)
                    return std::pair<Point, size_t>{ c, i - 1 };
                else
                    continue;
            }
            Vec2d p0f = (start_pt - p0).cast<double>();
            // Foot point of start_pt into v.
            Vec2d foot_pt = v * (p0f.dot(v) / l2v);
            // Vector from foot point of "start_pt" to "start_pt".
            Vec2d xf = p0f - foot_pt;
            // Squared distance of "start_pt" from the ray (p0, p1).
            double l2_from_line = xf.squaredNorm();
            // Squared distance of an intersection point of a circle with center at the foot point.
            if (double l2_intersection = dist2 - l2_from_line;
                l2_intersection > - SCALED_EPSILON) {
                // The ray (p0, p1) touches or intersects a circle centered at "start_pt" with radius "dist".
                // Distance of the circle intersection point from the foot point.
                l2_intersection = std::max(l2_intersection, 0.);
                if ((v - foot_pt).cast<double>().squaredNorm() >= l2_intersection) {
                    // Intersection of the circle with the segment (p0, p1) is on the right side (close to p1) from the foot point.
                    Point p = p0 + (foot_pt + v * sqrt(l2_intersection / l2v)).cast<coord_t>();
                    return std::pair<Point, size_t>{ p, i - 1 };
                }
            }
        }
    }
    return {};
}

/*!
 * \brief Returns Polylines representing the (infill) lines that will result in slicing the given area
 *
 * \param area[in] The area that has to be filled with infill.
 * \param layer_idx[in] The current layer index.
 * \param support_infill_distance[in] The distance that should be between the infill lines.
 *
 * \return A Polygons object that represents the resulting infill lines.
 */
[[nodiscard]] static Polylines generate_support_infill_lines(
    // Polygon to fill in with a zig-zag pattern supporting an overhang.
    const Polygons          &polygon,
    const SupportParameters &support_params,
    LayerIndex layer_idx, coord_t support_infill_distance)
{
    const Flow &flow = support_params.support_material_flow;
    std::unique_ptr<Fill>  filler = std::unique_ptr<Fill>(Fill::new_from_type(support_params.base_fill_pattern));
    FillParams             fill_params;

    filler->layer_id = layer_idx;
    filler->angle = support_params.base_angle;

    fill_params.density     = float(scale_d(flow.spacing()) / (scale_d(flow.spacing()) + float(support_infill_distance)));
    fill_params.dont_adjust = true;
    fill_params.config = &support_params.default_region_config;

    filler->init_spacing(flow.spacing(), fill_params);

    Polylines out;
    for (ExPolygon &expoly : union_ex(polygon)) {
        // The surface type does not matter.
        assert(area(expoly) > 0.);
        assert(intersecting_edges(to_polygons(expoly)).empty());
        Surface surface(stPosInternal | stDensSparse, std::move(expoly));
        try {
            Polylines pl = filler->fill_surface(&surface, fill_params);
            assert(pl.empty() || get_extents(surface.expolygon).inflated(SCALED_EPSILON).contains(get_extents(pl)));
            append(out, std::move(pl));
        } catch (InfillFailedException &) {
        }
    }
    return out;
}

/*!
 * \brief Converts a Polygons object representing a line into the internal format.
 *
 * \param polylines[in] The Polyline that will be converted.
 * \param layer_idx[in] The current layer.
 * \return All lines of the \p polylines object, with information for each point regarding in which avoidance it is currently valid in.
 */
// Called by generate_initial_areas()
[[nodiscard]] static LineInformations convert_lines_to_internal(
    const TreeVolumes &volumes, const TreeSettings &config,
    const Polylines &polylines, LayerIndex layer_idx)
{
    const bool min_xy_dist = config.xy_distance > config.xy_min_distance;

    LineInformations result;
    // Also checks if the position is valid, if it is NOT, it deletes that point
    for (const Polyline &line : polylines) {
        LineInformation res_line;
        for (Point p : line) {
            if (! contains(volumes.getAvoidance(config.getRadius(0), layer_idx, TreeVolumes::AvoidanceType::FastSafe, false, min_xy_dist), p))
                res_line.emplace_back(p, LineStatus::TO_BP_SAFE);
            else if (! contains(volumes.getAvoidance(config.getRadius(0), layer_idx, TreeVolumes::AvoidanceType::Fast, false, min_xy_dist), p))
                res_line.emplace_back(p, LineStatus::TO_BP);
            else if (config.support_rests_on_model && ! contains(volumes.getAvoidance(config.getRadius(0), layer_idx, TreeVolumes::AvoidanceType::FastSafe, true, min_xy_dist), p))
                res_line.emplace_back(p, LineStatus::TO_MODEL_GRACIOUS_SAFE);
            else if (config.support_rests_on_model && ! contains(volumes.getAvoidance(config.getRadius(0), layer_idx, TreeVolumes::AvoidanceType::Fast, true, min_xy_dist), p))
                res_line.emplace_back(p, LineStatus::TO_MODEL_GRACIOUS);
            else if (config.support_rests_on_model && ! contains(volumes.getCollision(config.getRadius(0), layer_idx, min_xy_dist), p))
                res_line.emplace_back(p, LineStatus::TO_MODEL);
            else if (!res_line.empty()) {
                result.emplace_back(res_line);
                res_line.clear();
            }
        }
        if (!res_line.empty()) {
            result.emplace_back(res_line);
            res_line.clear();
        }
    }

    return result;
}

/*!
 * \brief Offsets (increases the area of) a polygons object in multiple steps to ensure that it does not lag through over a given obstacle.
 * \param me[in] Polygons object that has to be offset.
 * \param distance[in] The distance by which me should be offset. Expects values >=0.
 * \param collision[in] The area representing obstacles.
 * \param last_step_offset_without_check[in] The most it is allowed to offset in one step.
 * \param min_amount_offset[in] How many steps have to be done at least. As this uses round offset this increases the amount of vertices, which may be required if Polygons get very small. Required as arcTolerance is not exposed in offset, which should result with a similar result.
 * \return The resulting Polygons object.
 */
[[nodiscard]] static Polygons safe_offset_inc(const Polygons& me, coord_t distance, const Polygons& collision, coord_t safe_step_size, coord_t last_step_offset_without_check, size_t min_amount_offset)
{
    bool do_final_difference = last_step_offset_without_check == 0;
    Polygons ret = safe_union(me); // ensure sane input

    // Trim the collision polygons with the region of interest for diff() efficiency.
    Polygons collision_trimmed_buffer;
    auto collision_trimmed = [&collision_trimmed_buffer, &collision, &ret, distance]() -> const Polygons& {
        if (collision_trimmed_buffer.empty() && ! collision.empty())
            collision_trimmed_buffer = ClipperUtils::clip_clipper_polygons_with_subject_bbox(collision, get_extents(ret).inflated(std::max(coord_t(0), distance) + SCALED_EPSILON));
        return collision_trimmed_buffer;
    };

    if (distance == 0)
        return do_final_difference ? diff(ret, collision_trimmed()) : union_(ret);
    if (safe_step_size < 0 || last_step_offset_without_check < 0) {
        BOOST_LOG_TRIVIAL(error) << "Offset increase got invalid parameter!";
        BOOST_LOG_TRIVIAL(error) << "Negative offset distance... How did you manage this ?"sv;
        return do_final_difference ? diff(ret, collision_trimmed()) : union_(ret);
    }

    coord_t step_size = safe_step_size;
    int     steps = distance > last_step_offset_without_check ? (distance - last_step_offset_without_check) / step_size : 0;
    if (distance - steps * step_size > last_step_offset_without_check) {
        if ((steps + 1) * step_size <= distance)
            // This will be the case when last_step_offset_without_check >= safe_step_size
            ++ steps;
        else
            do_final_difference = true;
    }
    if (steps + (distance < last_step_offset_without_check || (distance % step_size) != 0) < int(min_amount_offset) && min_amount_offset > 1) {
        // yes one can add a bool as the standard specifies that a result from compare operators has to be 0 or 1
        // reduce the stepsize to ensure it is offset the required amount of times
        step_size = distance / min_amount_offset;
        if (step_size >= safe_step_size) {
            // effectivly reduce last_step_offset_without_check
            step_size = safe_step_size;
            steps = min_amount_offset;
        } else
            steps = distance / step_size;
    }
    // offset in steps
    for (int i = 0; i < steps; ++ i) {
        ret = diff(offset(ret, step_size, ClipperLib::jtRound, scale_d(0.01)), collision_trimmed());
        // ensure that if many offsets are done the performance does not suffer extremely by the new vertices of jtRound.
        if (i % 10 == 7)
            ret = polygons_simplify(ret, scale_d(0.015), polygons_strictly_simple);
    }
    // offset the remainder
    float last_offset = distance - steps * step_size;
    if (last_offset > SCALED_EPSILON)
        ret = offset(ret, distance - steps * step_size, ClipperLib::jtRound, scale_d(0.01));
    ret = polygons_simplify(ret, scale_d(0.015), polygons_strictly_simple);

    if (do_final_difference)
        ret = diff(ret, collision_trimmed());
    return union_(ret);
}

/*!
 * \brief Unions two Polygons. Ensures that if the input is non empty that the output also will be non empty.
 * \param first[in] The first Polygon.
 * \param second[in] The second Polygon.
 * \return The union of both Polygons
 */
[[nodiscard]] static Polygons safe_union(const Polygons first, const Polygons second)
{
    // unionPolygons can slowly remove Polygons under certain circumstances, because of rounding issues (Polygons that have a thin area).
    // This does not cause a problem when actually using it on large areas, but as influence areas (representing centerpoints) can be very thin, this does occur so this ugly workaround is needed
    // Here is an example of a Polygons object that will loose vertices when unioning, and will be gone after a few times unionPolygons was called:
    /*
    Polygons example;
    Polygon exampleInner;
    exampleInner.add(Point(120410,83599));//A
    exampleInner.add(Point(120384,83643));//B
    exampleInner.add(Point(120399,83618));//C
    exampleInner.add(Point(120414,83591));//D
    exampleInner.add(Point(120423,83570));//E
    exampleInner.add(Point(120419,83580));//F
    example.add(exampleInner);
    for(int i=0;i<10;i++){
         log("Iteration %d Example area: %f\n",i,area(example));
         example=example.unionPolygons();
    }
*/

    Polygons result;
    if (! first.empty() || ! second.empty()) {
        result = union_(first, second);
        if (result.empty()) {
            BOOST_LOG_TRIVIAL(debug) << "Caught an area destroying union, enlarging areas a bit.";
            // just take the few lines we have, and offset them a tiny bit. Needs to be offsetPolylines, as offset may aleady have problems with the area.
            result = union_(offset(to_polylines(first), scale_d(0.002), jtMiter, 1.2), offset(to_polylines(second), scale_d(0.002), jtMiter, 1.2));
        }
    }

    return result;
}

struct SupportElementInfluenceAreas {
    // All influence areas: both to build plate and model.
    Polygons                        influence_areas;
    // Influence areas just to build plate.
    Polygons                        to_bp_areas;
    // Influence areas just to model.
    Polygons                        to_model_areas;

    void clear() {
        this->influence_areas.clear();
        this->to_bp_areas.clear();
        this->to_model_areas.clear();
    }
};

struct SupportElementMerging {
    SupportElementState                     state;
    /*!
     * \brief All elements in the layer above the current one that are supported by this element
     */
    SupportElement::ParentIndices           parents;

    SupportElementInfluenceAreas            areas;
    // Bounding box of all influence areas.
    Eigen::AlignedBox<coord_t, 2>           bbox_data;

    const Eigen::AlignedBox<coord_t, 2>&    bbox() const { return bbox_data;}
    const Point                             centroid() const { return (bbox_data.min() + bbox_data.max()) / 2; }
    void                                    set_bbox(const BoundingBox& abbox)
        { Point eps { coord_t(SCALED_EPSILON), coord_t(SCALED_EPSILON) }; bbox_data = { abbox.min - eps, abbox.max + eps }; }

    // Called by the AABBTree builder to get an index into the vector of source elements.
    // Not needed, thus zero is returned.
    static size_t                           idx() { return 0; }
};

/*!
 * \brief Propagates influence downwards, and merges overlapping ones.
 *
 * \param move_bounds[in,out] All currently existing influence areas
 */
static void create_layer_pathing(const TreeVolumes &volumes, const TreeSettings &config, std::vector<SupportElements> &move_bounds, std::function<void()> throw_on_cancel)
{

    auto dur_inc   = std::chrono::duration_values<std::chrono::nanoseconds>::zero();
    auto dur_total = std::chrono::duration_values<std::chrono::nanoseconds>::zero();

    LayerIndex last_merge_layer_idx = move_bounds.size();
    bool new_element = false;

    // Ensures at least one merge operation per 3mm height, 50 layers, 1 mm movement of slow speed or 5mm movement of fast speed (whatever is lowest). Values were guessed.
    // The merge budget per pass shrinks with the largest move of the object.
    const coord_t max_move = *std::max_element(config.max_move.begin(), config.max_move.end());
    const coord_t max_move_slow = *std::max_element(config.max_move_slow.begin(), config.max_move_slow.end());
    size_t max_merge_every_x_layers = std::min(std::min(5000 / (std::max(max_move, coord_t(100))), 1000 / std::max(max_move_slow, coord_t(20))), 3000 / config.layer_height);
    size_t merge_every_x_layers = 1;
    // Calculate the influence areas for each layer below (Top down)
    // This is done by first increasing the influence area by the allowed movement distance, and merging them with other influence areas if possible
    for (int layer_idx = int(move_bounds.size()) - 1; layer_idx > 0; -- layer_idx)
        if (SupportElements &prev_layer = move_bounds[layer_idx]; ! prev_layer.empty()) {
            // merging is expensive and only parallelized to a max speedup of 2. As such it may be useful in some cases to only merge every few layers to improve performance.
            bool had_new_element = new_element;
            const bool merge_this_layer = had_new_element || size_t(last_merge_layer_idx - layer_idx) >= merge_every_x_layers;
            if (had_new_element)
                merge_every_x_layers = 1;
            const auto ta               = std::chrono::high_resolution_clock::now();

            // ### Increase the influence areas by the allowed movement distance
            std::vector<SupportElementMerging> influence_areas;
            influence_areas.reserve(prev_layer.size());
            for (int32_t element_idx = 0; element_idx < int32_t(prev_layer.size()); ++ element_idx) {
                SupportElement &el = prev_layer[element_idx];
                assert(!el.influence_area.empty());
                SupportElement::ParentIndices parents;
                parents.emplace_back(element_idx);
                influence_areas.push_back({ el.state, parents });
            }
            increase_areas_one_layer(volumes, config, influence_areas, layer_idx, prev_layer, merge_this_layer, throw_on_cancel);

            // Place already fully constructed elements to the output, remove them from influence_areas.
            SupportElements &this_layer = move_bounds[layer_idx - 1];
            influence_areas.erase(std::remove_if(influence_areas.begin(), influence_areas.end(),
                [&this_layer, layer_idx](SupportElementMerging &elem) {
                    if (elem.areas.influence_areas.empty())
                        // This area was removed completely due to collisions.
                        return true;
                    if (elem.areas.to_bp_areas.empty() && elem.areas.to_model_areas.empty()) {
                        if (area(elem.areas.influence_areas) < tiny_area_threshold) {
                            BOOST_LOG_TRIVIAL(error) << "Insert Error of Influence area bypass on layer " << layer_idx - 1;
                            BOOST_LOG_TRIVIAL(error) << "Insert error of area after bypassing merge.\n"sv;
                        }
                        // Move the area to output.
                        this_layer.emplace_back(elem.state, std::move(elem.parents), std::move(elem.areas.influence_areas));
                        return true;
                    }
                    // Keep the area.
                    return false;
                }),
                influence_areas.end());

            dur_inc += std::chrono::high_resolution_clock::now() - ta;
            new_element = ! move_bounds[layer_idx - 1].empty();
            if (merge_this_layer) {
                bool reduced_by_merging = false;
                if (size_t count_before_merge = influence_areas.size(); count_before_merge > 1) {
                    // ### Calculate which influence areas overlap, and merge them into a new influence area (simplified: an intersection of influence areas that have such an intersection)
                    merge_influence_areas(volumes, config, layer_idx, influence_areas, throw_on_cancel);
                    reduced_by_merging = count_before_merge > influence_areas.size();
                }
                last_merge_layer_idx = layer_idx;
                if (! reduced_by_merging && ! had_new_element)
                    merge_every_x_layers = std::min(max_merge_every_x_layers, merge_every_x_layers + 1);
            }

            dur_total += std::chrono::high_resolution_clock::now() - ta;

            // Save calculated elements to output, and allocate Polygons on heap, as they will not be changed again.
            for (SupportElementMerging &elem : influence_areas)
                if (! elem.areas.influence_areas.empty()) {
                    Polygons new_area = safe_union(elem.areas.influence_areas);
                    if (area(new_area) < tiny_area_threshold) {
                        BOOST_LOG_TRIVIAL(error) << "Insert Error of Influence area on layer " << layer_idx - 1 << ". Origin of " << elem.parents.size() << " areas. Was to bp " << elem.state.to_buildplate;
                        BOOST_LOG_TRIVIAL(error) << "Insert error of area after merge.\n"sv;
                    }
                    this_layer.emplace_back(elem.state, std::move(elem.parents), std::move(new_area));
                }

            throw_on_cancel();
        }

    BOOST_LOG_TRIVIAL(info) << "Time spent with creating influence areas' subtasks: Increasing areas " << dur_inc.count() / 1000000 <<
        " ms merging areas: " << (dur_total - dur_inc).count() / 1000000 << " ms";
}

/*!
 * \brief Merges Influence Areas at one layer if possible.
 *
 * Branches which do overlap have to be merged. This manages the helper and uses a divide and conquer approach to parallelize this problem. This parallelization can at most accelerate the merging by a factor of 2.
 *
 * \param to_bp_areas[in] The Elements of the current Layer that will reach the buildplate.
 *  Value is the influence area where the center of a circle of support may be placed.
 * \param to_model_areas[in] The Elements of the current Layer that do not have to reach the buildplate. Also contains main as every element that can reach the buildplate is not forced to.
 *  Value is the influence area where the center of a circle of support may be placed.
 * \param influence_areas[in] The Elements of the current Layer without avoidances removed. This is the largest possible influence area for this layer.
 *  Value is the influence area where the center of a circle of support may be placed.
 * \param layer_idx[in] The current layer.
 */
static void merge_influence_areas(
    const TreeVolumes             &volumes,
    const TreeSettings          &config,
    const LayerIndex                    layer_idx,
    std::vector<SupportElementMerging> &influence_areas,
    std::function<void()>               throw_on_cancel)
{
    const size_t input_size = influence_areas.size();
    if (input_size == 0)
        return;

    // Merging by divide & conquer.
    // The majority of time is consumed by Clipper polygon operations, intersection is accelerated by bounding boxes.
    // Sorting input into an AABB tree helps to perform most of the intersections at first iterations,
    // thus reducing computation when merging larger subtrees.
    // The actual merge logic is found in merge_influence_areas_two_sets.

    // Build an AABB tree over the influence areas.
    //FIXME A full tree does not need to be built, the lowest level branches will be always bucketed.
    // However the additional time consumed is negligible.
    AABBTreeIndirect::Tree<2, coord_t> tree;
    // Sort influence_areas in place.
    tree.build_modify_input(influence_areas);

    throw_on_cancel();

    // Prepare the initial buckets as ranges of influence areas. The initial buckets contain power of 2 influence areas to follow
    // the branching of the AABB tree.
    // Vectors of ranges of influence areas, following the branching of the AABB tree:
    std::vector<std::pair<SupportElementMerging*, SupportElementMerging*>> buckets;
    // Initial number of buckets for 1st round of merging.
    size_t num_buckets_initial;
    {
        // How many buckets per first merge iteration?
        const size_t num_threads     = tbb::this_task_arena::max_concurrency();
        // 4 buckets per thread if possible,
        const size_t num_buckets_min = (input_size + 2) / 4;
        // 2 buckets per thread otherwise.
        const size_t num_buckets_max = input_size / 2;
        num_buckets_initial          = num_buckets_min >= num_threads ? num_buckets_min : num_buckets_max;
        const size_t bucket_size     = num_buckets_min >= num_threads ? 4 : 2;
        // Fill in the buckets.
        SupportElementMerging *it = influence_areas.data();
        // Reserve one more bucket to keep a single influence area which will not be merged in the first iteration.
        buckets.reserve(num_buckets_initial + 1);
        for (size_t i = 0; i < num_buckets_initial; ++ i, it += bucket_size)
            buckets.emplace_back(std::make_pair(it, it + bucket_size));
        SupportElementMerging *it_end = influence_areas.data() + influence_areas.size();
        if (buckets.back().second >= it_end) {
            // Last bucket is less than size 4, but bigger than size 1.
            buckets.back().second = std::min(buckets.back().second, it_end);
        } else {
            // Last bucket is size 1, it will not be merged in the first iteration.
            assert(it + 1 == it_end);
            buckets.emplace_back(std::make_pair(it, it_end));
        }
    }

    // 1st merge iteration, merge one with each other.
    tbb::parallel_for(tbb::blocked_range<size_t>(0, num_buckets_initial),
        [&](const tbb::blocked_range<size_t> &range) {
        for (size_t idx = range.begin(); idx < range.end(); ++ idx) {
            // Merge bucket_count adjacent to each other, merging uneven bucket numbers into even buckets
            buckets[idx].second = merge_influence_areas_leaves(volumes, config, layer_idx, buckets[idx].first, buckets[idx].second);
            throw_on_cancel();
        }
    });

    // Further merge iterations, merging one AABB subtree with another one, hopefully minimizing intersections between the elements
    // of each of the subtree.
    while (buckets.size() > 1) {
        tbb::parallel_for(tbb::blocked_range<size_t>(0, buckets.size() / 2),
            [&](const tbb::blocked_range<size_t> &range) {
            for (size_t idx = range.begin(); idx < range.end(); ++ idx) {
                const size_t bucket_pair_idx = idx * 2;
                // Merge bucket_count adjacent to each other, merging uneven bucket numbers into even buckets
                buckets[bucket_pair_idx].second = merge_influence_areas_two_sets(volumes, config, layer_idx,
                    buckets[bucket_pair_idx].first, buckets[bucket_pair_idx].second,
                    buckets[bucket_pair_idx + 1].first, buckets[bucket_pair_idx + 1].second);
                throw_on_cancel();
            }
        });
        // Remove odd buckets, which were merged into even buckets.
        size_t new_size = (buckets.size() + 1) / 2;
        for (size_t i = 1; i < new_size; ++ i)
            buckets[i] = std::move(buckets[i * 2]);
        buckets.erase(buckets.begin() + new_size, buckets.end());
    }
}

static SupportElementMerging* merge_influence_areas_two_sets(
    const TreeVolumes &volumes, const TreeSettings &config, const LayerIndex layer_idx,
    SupportElementMerging * const dst_begin, SupportElementMerging *       dst_end,
    SupportElementMerging *       src_begin, SupportElementMerging * const src_end)
{
    // Merging src into dst.
    // Areas of src should not overlap with areas of another elements of src.
    // Areas of dst should not overlap with areas of another elements of dst.
    // The memory from dst_begin to src_end is reserved for the merging operation,
    // src follows dst.
    assert(src_begin < src_end);
    assert(dst_begin < dst_end);
    assert(dst_end <= src_begin);
    for (SupportElementMerging *src = src_begin; src != src_end; ++ src) {
        SupportElementMerging         *dst      = dst_begin;
        SupportElementMerging         *merged   = nullptr;
        for (; dst != dst_end; ++ dst)
            if (merge_influence_areas_two_elements(volumes, config, layer_idx, *dst, *src)) {
                merged = dst ++;
                if (src != src_begin)
                    // Compactify src.
                    *src = std::move(*src_begin);
                ++ src_begin;
                break;
            }
        for (; dst != dst_end;)
            if (merge_influence_areas_two_elements(volumes, config, layer_idx, *merged, *dst)) {
                // Compactify dst.
                if (dst != -- dst_end)
                    *dst = std::move(*dst_end);
            } else
                ++ dst;
    }
    // Compactify src elements that were not merged with dst to the end of dst.
    assert(dst_end <= src_begin);
    if (dst_end == src_begin)
        dst_end = src_end;
    else
        while (src_begin != src_end)
            *dst_end ++ = std::move(*src_begin ++);

    return dst_end;
}

/*!
 * \brief Merges Influence Areas if possible.
 *
 * Branches which do overlap have to be merged. This helper merges all elements in input with the elements into reduced_new_layer.
 * Elements in input_aabb are merged together if possible, while elements reduced_new_layer_aabb are not checked against each other.
 *
 * \param reduced_aabb[in,out] The already processed elements.
 * \param input_aabb[in] Not yet processed elements
 * \param to_bp_areas[in] The Elements of the current Layer that will reach the buildplate. Value is the influence area where the center of a circle of support may be placed.
 * \param to_model_areas[in] The Elements of the current Layer that do not have to reach the buildplate. Also contains main as every element that can reach the buildplate is not forced to.
 * Value is the influence area where the center of a circle of support may be placed.
 * \param influence_areas[in] The influence areas without avoidance removed.
 * \param insert_bp_areas[out] Elements to be inserted into the main dictionary after the Helper terminates.
 * \param insert_model_areas[out] Elements to be inserted into the secondary dictionary after the Helper terminates.
 * \param insert_influence[out] Elements to be inserted into the dictionary containing the largest possibly valid influence area (ignoring if the area may not be there because of avoidance)
 * \param erase[out] Elements that should be deleted from the above dictionaries.
 * \param layer_idx[in] The Index of the current Layer.
 */

static SupportElementMerging* merge_influence_areas_leaves(
    const TreeVolumes &volumes, const TreeSettings &config, const LayerIndex layer_idx,
    SupportElementMerging * const dst_begin, SupportElementMerging *dst_end)
{
    // Merging at the lowest level of the AABB tree. Checking one against each other, O(n^2).
    assert(dst_begin < dst_end);
    for (SupportElementMerging *i = dst_begin; i + 1 < dst_end;) {
        for (SupportElementMerging *j = i + 1; j != dst_end;)
            if (merge_influence_areas_two_elements(volumes, config, layer_idx, *i, *j)) {
                // i was merged with j, j is empty.
                if (j != -- dst_end)
                    *j = std::move(*dst_end);
                goto merged;
            } else
                ++ j;
        // not merged
        ++ i;
    merged:
        ;
    }
    return dst_end;
}

static bool merge_influence_areas_two_elements(
    const TreeVolumes &volumes, const TreeSettings &config, const LayerIndex layer_idx,
    SupportElementMerging &dst, SupportElementMerging &src)
{
    // Don't merge gracious with a non gracious area as bad placement could negatively impact reliability of the whole subtree.
    const bool merging_gracious_and_non_gracious = dst.state.to_model_gracious != src.state.to_model_gracious;
    // Could cause some issues with the increase of one area, as it is assumed that if the smaller is increased
    // by the delta to the larger it is engulfed by it already. But because a different collision
    // may be removed from the in draw_area() generated circles, this assumption could be wrong.
    const bool merging_min_and_regular_xy        = dst.state.use_min_xy_dist != src.state.use_min_xy_dist;

    if (merging_gracious_and_non_gracious || merging_min_and_regular_xy)
        return false;

    const bool dst_radius_bigger = support_element_collision_radius(config, dst.state) > support_element_collision_radius(config, src.state);
    const SupportElementMerging &smaller_rad = dst_radius_bigger ? src : dst;
    const SupportElementMerging &bigger_rad  = dst_radius_bigger ? dst : src;
    const coord_t real_radius_delta = std::abs(support_element_radius(config, bigger_rad.state) - support_element_radius(config, smaller_rad.state));
    {
        // Testing intersection of bounding boxes.
        // Expand the smaller radius branch bounding box to match the lambda intersect_small_with_bigger() below.
        // Because the lambda intersect_small_with_bigger() applies a rounded offset, a snug offset of the bounding box
        // is sufficient. On the other side, if a mitered offset was used by the lambda,
        // the bounding box expansion would have to account for the mitered extension of the sharp corners.
        Eigen::AlignedBox<coord_t, 2> smaller_bbox = smaller_rad.bbox();
        smaller_bbox.min() -= Point{ real_radius_delta, real_radius_delta };
        smaller_bbox.max() += Point{ real_radius_delta, real_radius_delta };
        if (! smaller_bbox.intersects(bigger_rad.bbox()))
            return false;
    }

    // Accumulator of a radius increase of a "to model" branch by merging in a "to build plate" branch.
    coord_t increased_to_model_radius = 0;
    const bool merging_to_bp                     = dst.state.to_buildplate && src.state.to_buildplate;
    if (! merging_to_bp) {
        // Get the real radius increase as the user does not care for the collision model.
        if (dst.state.to_buildplate != src.state.to_buildplate) {
            // Merging a "to build plate" branch with a "to model" branch.
            // Don't allow merging a thick "to build plate" branch into a thinner "to model" branch.
            const coord_t rdst = support_element_radius(config, dst.state);
            const coord_t rsrc = support_element_radius(config, src.state);
            if (dst.state.to_buildplate) {
                if (rsrc < rdst)
                    increased_to_model_radius = src.state.increased_to_model_radius + rdst - rsrc;
            } else {
                if (rsrc > rdst)
                    increased_to_model_radius = dst.state.increased_to_model_radius + rsrc - rdst;
            }
            if (increased_to_model_radius > config.max_to_model_radius_increase)
                return false;
        }
        // if a merge could place a stable branch on unstable ground, would be increasing the radius further
        // than allowed to when merging to model and to_bp trees or would merge to model before it is known
        // they will even been drawn the merge is skipped
        if (std::max(src.state.distance_to_top, dst.state.distance_to_top) < config.min_dtt_to_model)
            return false;
    }

    // Area of the bigger radius is used to ensure correct placement regarding the relevant avoidance,
    // so if that would change an invalid area may be created.
    if (! bigger_rad.state.can_use_safe_radius && smaller_rad.state.can_use_safe_radius)
        return false;

    // the bigger radius is used to verify that the area is still valid after the increase with the delta.
    // If there were a point where the big influence area could be valid with can_use_safe_radius
    // the element would already be can_use_safe_radius.
    // the smaller radius, which gets increased by delta may reach into the area where use_min_xy_dist is no longer required.
    const bool use_min_radius = bigger_rad.state.use_min_xy_dist && smaller_rad.state.use_min_xy_dist;

    // The idea is that the influence area with the smaller collision radius is increased by the radius difference.
    // If this area has any intersections with the influence area of the larger collision radius, a branch (of the larger collision radius) placed in this intersection, has already engulfed the branch of the smaller collision radius.
    // Because of this a merge may happen even if the influence areas (that represent possible center points of branches) do not intersect yet.
    // Remember that collision radius <= real radius as otherwise this assumption would be false.
    const coord_t   smaller_collision_radius    = support_element_collision_radius(config, smaller_rad.state);
    const Polygons &collision                   = volumes.getCollision(smaller_collision_radius, layer_idx - 1, use_min_radius);
    auto            intersect_small_with_bigger = [real_radius_delta, smaller_collision_radius, &collision, &config](const Polygons &small, const Polygons &bigger) {
        return intersection(
            safe_offset_inc(
                small, real_radius_delta, collision,
                // -3 avoids possible rounding errors
                2 * (config.xy_distance + smaller_collision_radius - 3), 0, 0),
            bigger);
    };
//#define TREES_MERGE_RATHER_LATER
    Polygons intersect =
        intersect_small_with_bigger(
        merging_to_bp ? smaller_rad.areas.to_bp_areas : smaller_rad.areas.to_model_areas,
        merging_to_bp ? bigger_rad.areas.to_bp_areas : bigger_rad.areas.to_model_areas);

    // dont use empty as a line is not empty, but for this use-case it very well may be (and would be one layer down as union does not keep lines)
    // check if the overlap is large enough (Small ares tend to attract rounding errors in clipper).
    if (area(intersect) <= tiny_area_threshold)
        return false;

    // While 0.025 was guessed as enough, i did not have reason to change it.
    if (area(offset(intersect, scale_d(-0.025), jtMiter, 1.2)) <= tiny_area_threshold)
        return false;


    // Do the actual merge now that the branches are confirmed to be able to intersect.
    // calculate which point is closest to the point of the last merge (or tip center if no merge above it has happened)
    // used at the end to estimate where to best place the branch on the bottom most layer
    // could be replaced with a random point inside the new area
    Point new_pos = move_inside_if_outside(intersect, dst.state.next_position);

    SupportElementState new_state = merge_support_element_states(dst.state, src.state, new_pos, layer_idx - 1, config);
    new_state.increased_to_model_radius = increased_to_model_radius == 0 ?
        // increased_to_model_radius was not set yet. Propagate maximum.
        std::max(dst.state.increased_to_model_radius, src.state.increased_to_model_radius) :
        increased_to_model_radius;

    // Rather unioning with "intersect" due to some rounding errors.
    Polygons influence_areas = safe_union(
        intersect_small_with_bigger(smaller_rad.areas.influence_areas, bigger_rad.areas.influence_areas),
        intersect);

    Polygons to_model_areas;
    if (merging_to_bp && config.support_rests_on_model)
        to_model_areas = new_state.to_model_gracious ?
            // Rather unioning with "intersect" due to some rounding errors.
            safe_union(
                intersect_small_with_bigger(smaller_rad.areas.to_model_areas, bigger_rad.areas.to_model_areas),
                intersect) :
            influence_areas;

    dst.parents.insert(dst.parents.end(), src.parents.begin(), src.parents.end());
    dst.state = new_state;
    dst.areas.influence_areas = std::move(influence_areas);
    dst.areas.to_bp_areas.clear();
    dst.areas.to_model_areas.clear();
    if (merging_to_bp) {
        dst.areas.to_bp_areas = std::move(intersect);
        if (config.support_rests_on_model)
            dst.areas.to_model_areas = std::move(to_model_areas);
    } else
        dst.areas.to_model_areas = std::move(intersect);
    // Update the bounding box.
    BoundingBox bbox(get_extents(dst.areas.influence_areas));
    bbox.merge(get_extents(dst.areas.to_bp_areas));
    bbox.merge(get_extents(dst.areas.to_model_areas));
    dst.set_bbox(bbox);
    // Clear the source data.
    src.areas.clear();
    src.parents.clear();
    return true;
}

[[nodiscard]] static SupportElementState merge_support_element_states(
    const SupportElementState &first, const SupportElementState &second, const Point &next_position, const coord_t layer_idx,
    const TreeSettings &config)
{
    SupportElementState out;
    out.next_position   = next_position;
    out.layer_idx       = layer_idx;
    out.use_min_xy_dist = first.use_min_xy_dist || second.use_min_xy_dist;
    out.dont_move_until = std::max(first.dont_move_until, second.dont_move_until);
    out.can_use_safe_radius = first.can_use_safe_radius || second.can_use_safe_radius;
    out.skip_ovalisation = false;
    if (first.target_height > second.target_height) {
        out.target_height   = first.target_height;
        out.target_position = first.target_position;
    } else {
        out.target_height   = second.target_height;
        out.target_position = second.target_position;
    }
    out.effective_radius_height = std::max(first.effective_radius_height, second.effective_radius_height);
    out.distance_to_top = std::max(first.distance_to_top, second.distance_to_top);

    out.to_buildplate = first.to_buildplate && second.to_buildplate;
    out.to_model_gracious = first.to_model_gracious && second.to_model_gracious; // valid as we do not merge non-gracious with gracious

    out.elephant_foot_increases = 0;
    if (config.bp_radius_increase_per_layer > 0) {
        coord_t foot_increase_radius = std::abs(std::max(support_element_collision_radius(config, second), support_element_collision_radius(config, first)) - support_element_collision_radius(config, out));
        // elephant_foot_increases has to be recalculated, as when a smaller tree with a larger elephant_foot_increases merge with a larger branch
        // the elephant_foot_increases may have to be lower as otherwise the radius suddenly increases. This results often in a non integer value.
        out.elephant_foot_increases = foot_increase_radius / (config.bp_radius_increase_per_layer - config.branch_radius_increase_per_layer);
    }

    // set last settings to the best out of both parents. If this is wrong, it will only cause a small performance penalty instead of weird behavior.
    out.last_area_increase = {
        std::min(first.last_area_increase.type, second.last_area_increase.type),
        std::min(first.last_area_increase.increase_speed, second.last_area_increase.increase_speed),
        first.last_area_increase.increase_radius || second.last_area_increase.increase_radius,
        first.last_area_increase.no_error || second.last_area_increase.no_error,
        first.last_area_increase.use_min_distance && second.last_area_increase.use_min_distance,
        first.last_area_increase.move || second.last_area_increase.move };

    return out;
}

/*!
 * \brief Increases influence areas as far as required.
 *
 * Calculates influence areas of the layer below, based on the influence areas of the current layer.
 * Increases every influence area by maximum_move_distance_slow. If this is not enough, as in it would change the gracious or to_buildplate status, the influence areas are instead increased by maximum_move_distance.
 * Also ensures that increasing the radius of a branch, does not cause it to change its status (like to_buildplate ). If this were the case, the radius is not increased instead.
 *
 * Warning: The used format inside this is different as the SupportElement does not have a valid area member. Instead this area is saved as value of the dictionary. This was done to avoid not needed heap allocations.
 *
 * \param to_bp_areas[out] Influence areas that can reach the buildplate
 * \param to_model_areas[out] Influence areas that do not have to reach the buildplate. This has overlap with new_layer_data, as areas that can reach the buildplate are also considered valid areas to the model.
 * This redundancy is required if a to_buildplate influence area is allowed to merge with a to model influence area.
 * \param influence_areas[out] Area than can reach all further up support points. No assurance is made that the buildplate or the model can be reached in accordance to the user-supplied settings.
 * \param bypass_merge_areas[out] Influence areas ready to be added to the layer below that do not need merging.
 * \param last_layer[in] Influence areas of the current layer.
 * \param layer_idx[in] Number of the current layer.
 * \param mergelayer[in] Will the merge method be called on this layer. This information is required as some calculation can be avoided if they are not required for merging.
 */
static void increase_areas_one_layer(
    const TreeVolumes              &volumes,
    const TreeSettings           &config,
    // New areas at the layer below layer_idx
    std::vector<SupportElementMerging>  &merging_areas,
    // Layer above merging_areas.
    const LayerIndex                     layer_idx,
    // Layer elements above merging_areas.
    SupportElements                     &layer_elements,
    // If false, the merging_areas will not be merged for performance reasons.
    const bool                           mergelayer,
    std::function<void()>                throw_on_cancel)
{
    using AvoidanceType = TreeVolumes::AvoidanceType;

    tbb::parallel_for(tbb::blocked_range<size_t>(0, merging_areas.size(), 1),
        [&](const tbb::blocked_range<size_t> &range) {
        for (size_t merging_area_idx = range.begin(); merging_area_idx < range.end(); ++ merging_area_idx) {
            SupportElementMerging   &merging_area   = merging_areas[merging_area_idx];
            assert(merging_area.parents.size() == 1);
            SupportElement          &parent         = layer_elements[merging_area.parents.front()];
            SupportElementState      elem           = SupportElementState::propagate_down(parent.state);
            const Polygons          &wall_restriction =
                // Abstract representation of the model outline. If an influence area would move through it, it could teleport through a wall.
                volumes.getWallRestriction(support_element_collision_radius(config, parent.state), layer_idx, parent.state.use_min_xy_dist);


            Polygons to_bp_data, to_model_data;
            coord_t radius = support_element_collision_radius(config, elem);

            // When the radius increases, the outer "support wall" of the branch will have been moved farther away from the center (as this is the definition of radius).
            // As it is not specified that the support_tree_angle has to be one of the center of the branch, it is here seen as the smaller angle of the outer wall of the branch, to the outer wall of the same branch one layer above.
            // As the branch may have become larger the distance between these 2 walls is smaller than the distance of the center points.
            // These extra distance is added to the movement distance possible for this layer.

            coord_t extra_speed = 5; // The extra speed is added to both movement distances. Also move 5 microns faster than allowed to avoid rounding errors, this may cause issues at VERY VERY small layer heights.
            coord_t extra_slow_speed = 0; // Only added to the slow movement distance.
            const coord_t ceiled_parent_radius = volumes.ceilRadius(support_element_collision_radius(config, parent.state), parent.state.use_min_xy_dist);
            coord_t projected_radius_increased = config.getRadius(parent.state.effective_radius_height + 1, parent.state.elephant_foot_increases);
            coord_t projected_radius_delta = projected_radius_increased - support_element_collision_radius(config, parent.state);

            // When z distance is more than one layer up and down the Collision used to calculate the wall restriction will always include the wall (and not just the xy_min_distance) of the layer above and below like this (d = blocked area because of z distance):
            /*
             *  layer z+1:dddddiiiiiioooo
             *  layer z+0:xxxxxdddddddddd
             *  layer z-1:dddddxxxxxxxxxx
             *  For more detailed visualisation see calculateWallRestrictions
             */
            const coord_t safe_movement_distance =
                (elem.use_min_xy_dist ? config.xy_min_distance : config.xy_distance) +
                (std::min(config.z_distance_top_layers, config.z_distance_bottom_layers) > 0 ? config.min_feature_size : 0);
            if (ceiled_parent_radius == volumes.ceilRadius(projected_radius_increased, parent.state.use_min_xy_dist) ||
                projected_radius_increased < config.increase_radius_until_radius)
                // If it is guaranteed possible to increase the radius, the maximum movement speed can be increased, as it is assumed that the maximum movement speed is the one of the slower moving wall
                extra_speed += projected_radius_delta;
            else
                // if a guaranteed radius increase is not possible, only increase the slow speed
                // Ensure that the slow movement distance can not become larger than the fast one.
                extra_slow_speed += std::min(projected_radius_delta, (config.maximum_move_distance(layer_idx) + extra_speed) - (config.maximum_move_distance_slow(layer_idx) + extra_slow_speed));

            if (config.layer_start_bp_radius > layer_idx &&
                config.recommendedMinRadius(layer_idx - 1) < config.getRadius(elem.effective_radius_height + 1, elem.elephant_foot_increases)) {
                // can guarantee elephant foot radius increase
                if (ceiled_parent_radius == volumes.ceilRadius(config.getRadius(parent.state.effective_radius_height + 1, parent.state.elephant_foot_increases + 1), parent.state.use_min_xy_dist))
                    extra_speed += config.bp_radius_increase_per_layer;
                else
                    extra_slow_speed += std::min(coord_t(config.bp_radius_increase_per_layer),
                                                 config.maximum_move_distance(layer_idx) - (config.maximum_move_distance_slow(layer_idx) + extra_slow_speed));
            }

            const coord_t fast_speed = config.maximum_move_distance(layer_idx) + extra_speed;
            const coord_t slow_speed = config.maximum_move_distance_slow(layer_idx) + extra_speed + extra_slow_speed;

            Polygons offset_slow, offset_fast;

            bool add = false;
            bool bypass_merge = false;
            constexpr bool increase_radius = true, no_error = true, use_min_radius = true, move = true; // aliases for better readability

            // Determine in which order configurations are checked if they result in a valid influence area. Check will stop if a valid area is found
            std::vector<AreaIncreaseSettings> order;
            auto insertSetting = [&](AreaIncreaseSettings settings, bool back) {
                if (std::find(order.begin(), order.end(), settings) == order.end()) {
                    if (back)
                        order.emplace_back(settings);
                    else
                        order.insert(order.begin(), settings);
                }
            };

            const bool parent_moved_slow = elem.last_area_increase.increase_speed < config.maximum_move_distance(layer_idx);
            const bool avoidance_speed_mismatch = parent_moved_slow && elem.last_area_increase.type != AvoidanceType::Slow;
            if (elem.last_area_increase.move && elem.last_area_increase.no_error && elem.can_use_safe_radius && !mergelayer &&
                !avoidance_speed_mismatch && (elem.distance_to_top >= config.tip_layers || parent_moved_slow)) {
                // assume that the avoidance type that was best for the parent is best for me. Makes this function about 7% faster.
                insertSetting({ elem.last_area_increase.type, elem.last_area_increase.increase_speed < config.maximum_move_distance(layer_idx) ? slow_speed : fast_speed,
                    increase_radius, elem.last_area_increase.no_error, !use_min_radius, elem.last_area_increase.move }, true);
                insertSetting({ elem.last_area_increase.type, elem.last_area_increase.increase_speed < config.maximum_move_distance(layer_idx) ? slow_speed : fast_speed,
                    !increase_radius, elem.last_area_increase.no_error, !use_min_radius, elem.last_area_increase.move }, true);
            }
            // branch may still go though a hole, so a check has to be done whether the hole was already passed, and the regular avoidance can be used.
            if (!elem.can_use_safe_radius) {
                // if the radius until which it is always increased can not be guaranteed, move fast. This is to avoid holes smaller than the real branch radius.
                // This does not guarantee the avoidance of such holes, but ensures they are avoided if possible.
                // order.emplace_back(AvoidanceType::Slow,!increase_radius,no_error,!use_min_radius,move);
                insertSetting({ AvoidanceType::Slow, slow_speed, increase_radius, no_error, !use_min_radius, !move }, true); // did we go through the hole
                // in many cases the definition of hole is overly restrictive, so to avoid unnecessary fast movement in the tip, it is ignored there for a bit.
                // This CAN cause a branch to go though a hole it otherwise may have avoided.
                if (elem.distance_to_top < round_up_divide(config.tip_layers, size_t(2)))
                    insertSetting({ AvoidanceType::Fast, slow_speed, increase_radius, no_error, !use_min_radius, !move }, true);
                insertSetting({ AvoidanceType::FastSafe, fast_speed, increase_radius, no_error, !use_min_radius, !move }, true); // did we manage to avoid the hole
                insertSetting({ AvoidanceType::FastSafe, fast_speed, !increase_radius, no_error, !use_min_radius, move }, true);
                insertSetting({ AvoidanceType::Fast, fast_speed, !increase_radius, no_error, !use_min_radius, move }, true);
            } else {
                insertSetting({ AvoidanceType::Slow, slow_speed, increase_radius, no_error, !use_min_radius, move }, true);
                // while moving fast to be able to increase the radius (b) may seems preferable (over a) this can cause the a sudden skip in movement,
                // which looks similar to a layer shift and can reduce stability.
                // as such idx have chosen to only use the user setting for radius increases as a friendly recommendation.
                insertSetting({ AvoidanceType::Slow, slow_speed, !increase_radius, no_error, !use_min_radius, move }, true); // a
                if (elem.distance_to_top < config.tip_layers)
                    insertSetting({ AvoidanceType::FastSafe, slow_speed, increase_radius, no_error, !use_min_radius, move }, true);
                insertSetting({ AvoidanceType::FastSafe, fast_speed, increase_radius, no_error, !use_min_radius, move }, true); // b
                insertSetting({ AvoidanceType::FastSafe, fast_speed, !increase_radius, no_error, !use_min_radius, move }, true);
            }

            if (elem.use_min_xy_dist) {
                std::vector<AreaIncreaseSettings> new_order;
                // if the branch currently has to use min_xy_dist check if the configuration would also be valid
                // with the regular xy_distance before checking with use_min_radius (Only happens when Support Distance priority is z overrides xy )
                for (AreaIncreaseSettings settings : order) {
                    new_order.emplace_back(settings);
                    new_order.push_back({ settings.type, settings.increase_speed, settings.increase_radius, settings.no_error, use_min_radius, settings.move });
                }
                order = new_order;
            }
            if (elem.to_buildplate || (elem.to_model_gracious && intersection(parent.influence_area, volumes.getPlaceableAreas(radius, layer_idx, throw_on_cancel)).empty())) {
                // error case
                // it is normal that we wont be able to find a new area at some point in time if we wont be able to reach layer 0 aka have to connect with the model
                insertSetting({ AvoidanceType::Fast, fast_speed, !increase_radius, !no_error, elem.use_min_xy_dist, move }, true);
            }
            if (elem.distance_to_top < elem.dont_move_until && elem.can_use_safe_radius) // only do not move when holes would be avoided in every case.
                // Only do not move when already in a no hole avoidance with the regular xy distance.
                insertSetting({ AvoidanceType::Slow, 0, increase_radius, no_error, !use_min_radius, !move }, false);

            Polygons inc_wo_collision;
            // Check whether it is faster to calculate the area increased with the fast speed independently from the slow area, or time could be saved by reusing the slow area to calculate the fast one.
            // Calculated by comparing the steps saved when calcualting idependently with the saved steps when not.
            bool offset_independant_faster = radius / safe_movement_distance - int(config.maximum_move_distance(layer_idx) + extra_speed < radius + safe_movement_distance) >
                                             round_up_divide((extra_speed + extra_slow_speed + config.maximum_move_distance_slow(layer_idx)), safe_movement_distance);
            for (const AreaIncreaseSettings &settings : order) {
                if (settings.move) {
                    if (offset_slow.empty() && (settings.increase_speed == slow_speed || ! offset_independant_faster)) {
                        // offsetting in 2 steps makes our offsetted area rounder preventing (rounding) errors created by to pointy areas. At this point one can see that the Polygons class
                        // was never made for precision in the single digit micron range.
                        offset_slow = safe_offset_inc(parent.influence_area, extra_speed + extra_slow_speed + config.maximum_move_distance_slow(layer_idx),
                            wall_restriction, safe_movement_distance, offset_independant_faster ? safe_movement_distance + radius : 0, 2);
                    }
                    if (offset_fast.empty() && settings.increase_speed != slow_speed) {
                        if (offset_independant_faster)
                            offset_fast = safe_offset_inc(parent.influence_area, extra_speed + config.maximum_move_distance(layer_idx),
                                wall_restriction, safe_movement_distance, offset_independant_faster ? safe_movement_distance + radius : 0, 1);
                        else {
                            const coord_t delta_slow_fast = config.maximum_move_distance(layer_idx) - (config.maximum_move_distance_slow(layer_idx) + extra_slow_speed);
                            offset_fast = safe_offset_inc(offset_slow, delta_slow_fast, wall_restriction, safe_movement_distance, safe_movement_distance + radius, offset_independant_faster ? 2 : 1);
                        }
                    }
                }
                std::optional<SupportElementState> result;
                inc_wo_collision.clear();
                if (!settings.no_error) {
                    // ERROR CASE
                    // if the area becomes for whatever reason something that clipper sees as a line, offset would stop working, so ensure that even if if wrongly would be a line, it still actually has an area that can be increased
                    Polygons lines_offset = offset(to_polylines(parent.influence_area), scale_d(0.005), jtMiter, 1.2);
                    Polygons base_error_area = union_(parent.influence_area, lines_offset);
                    result = increase_single_area(volumes, config, settings, layer_idx, parent,
                        base_error_area, to_bp_data, to_model_data, inc_wo_collision, (config.maximum_move_distance(layer_idx) + extra_speed) * 1.5, mergelayer);
                    BOOST_LOG_TRIVIAL(warning)
                          << "Influence area could not be increased! Data about the Influence area: "
                             "Radius: " << radius << " at layer: " << layer_idx - 1 << " NextTarget: " << elem.layer_idx << " Distance to top: " << elem.distance_to_top <<
                             " Elephant foot increases " << elem.elephant_foot_increases << " use_min_xy_dist " << elem.use_min_xy_dist << " to buildplate " << elem.to_buildplate <<
                             " gracious " << elem.to_model_gracious << " safe " << elem.can_use_safe_radius << " until move " << elem.dont_move_until << " \n "
                             "Parent " << &parent << ": Radius: " << support_element_collision_radius(config, parent.state) << " at layer: " << layer_idx << " NextTarget: " << parent.state.layer_idx <<
                             " Distance to top: " << parent.state.distance_to_top << " Elephant foot increases " << parent.state.elephant_foot_increases << "  use_min_xy_dist " << parent.state.use_min_xy_dist <<
                             " to buildplate " << parent.state.to_buildplate << " gracious " << parent.state.to_model_gracious << " safe " << parent.state.can_use_safe_radius << " until move " << parent.state.dont_move_until;
                    BOOST_LOG_TRIVIAL(error) << "Potentially lost branch!"sv;
                    if (result)
                        result->lost = true;
                } else
                    result = increase_single_area(volumes, config, settings, layer_idx, parent,
                        settings.increase_speed == slow_speed ? offset_slow : offset_fast, to_bp_data, to_model_data, inc_wo_collision, 0, mergelayer);

                if (result) {
                    elem = *result;
                    radius = support_element_collision_radius(config, elem);
                    elem.last_area_increase = settings;
                    add = true;
                    // do not merge if the branch should not move or the priority has to be to get farther away from the model.
                    bypass_merge = !settings.move || (settings.use_min_distance && elem.distance_to_top < config.tip_layers);
                    if (settings.move)
                        elem.dont_move_until = 0;
                    else
                        elem.result_on_layer = parent.state.result_on_layer;

                    elem.can_use_safe_radius = settings.type != AvoidanceType::Fast;

                    if (!settings.use_min_distance)
                        elem.use_min_xy_dist = false;
                    if (!settings.no_error)
                        BOOST_LOG_TRIVIAL(info)
                            << "Trying to keep area by moving faster than intended: Success";
                    break;
                } else if (!settings.no_error)
                    BOOST_LOG_TRIVIAL(error) << "Trying to keep area by moving faster than intended: FAILURE! WRONG BRANCHES LIKLY!";
            }

            if (add) {
                // Union seems useless, but some rounding errors somewhere can cause to_bp_data to be slightly bigger than it should be.
                assert(! inc_wo_collision.empty() || ! to_bp_data.empty() || ! to_model_data.empty());
                Polygons max_influence_area = safe_union(
                    diff_clipped(inc_wo_collision, volumes.getCollision(radius, layer_idx - 1, elem.use_min_xy_dist)),
                    safe_union(to_bp_data, to_model_data));
                merging_area.state = elem;
                assert(!max_influence_area.empty());
                merging_area.set_bbox(get_extents(max_influence_area));
                merging_area.areas.influence_areas = std::move(max_influence_area);
                if (! bypass_merge) {
                    if (elem.to_buildplate)
                        merging_area.areas.to_bp_areas = std::move(to_bp_data);
                    if (config.support_rests_on_model)
                        merging_area.areas.to_model_areas = std::move(to_model_data);
                }
            } else {
                // If the bottom most point of a branch is set, later functions will assume that the position is valid, and ignore it.
                // But as branches connecting with the model that are to small have to be culled, the bottom most point has to be not set.
                // A point can be set on the top most tip layer (maybe more if it should not move for a few layers).
                parent.state.result_on_layer_reset();
                parent.state.to_model_gracious = false;
                parent.state.verylost = true;
            }

            throw_on_cancel();
        }
    }, tbb::simple_partitioner());
}

/*!
 * \brief Checks if an influence area contains a valid subsection and returns the corresponding metadata and the new Influence area.
 *
 * Calculates an influence areas of the layer below, based on the influence area of one element on the current layer.
 * Increases every influence area by maximum_move_distance_slow. If this is not enough, as in we would change our gracious or to_buildplate status the influence areas are instead increased by maximum_move_distance_slow.
 * Also ensures that increasing the radius of a branch, does not cause it to change its status (like to_buildplate ). If this were the case, the radius is not increased instead.
 *
 * Warning: The used format inside this is different as the SupportElement does not have a valid area member. Instead this area is saved as value of the dictionary. This was done to avoid not needed heap allocations.
 *
 * \param settings[in] Which settings have to be used to check validity.
 * \param layer_idx[in] Number of the current layer.
 * \param parent[in] The metadata of the parents influence area.
 * \param relevant_offset[in] The maximal possible influence area. No guarantee regarding validity with current layer collision required, as it is ensured in-function!
 * \param to_bp_data[out] The part of the Influence area that can reach the buildplate.
 * \param to_model_data[out] The part of the Influence area that do not have to reach the buildplate. This has overlap with new_layer_data.
 * \param increased[out]  Area than can reach all further up support points. No assurance is made that the buildplate or the model can be reached in accordance to the user-supplied settings.
 * \param overspeed[in] How much should the already offset area be offset again. Usually this is 0.
 * \param mergelayer[in] Will the merge method be called on this layer. This information is required as some calculation can be avoided if they are not required for merging.
 * \return A valid support element for the next layer regarding the calculated influence areas. Empty if no influence are can be created using the supplied influence area and settings.
 */
[[nodiscard]] static std::optional<SupportElementState> increase_single_area(
    const TreeVolumes      &volumes,
    const TreeSettings   &config,
    const AreaIncreaseSettings  &settings,
    const LayerIndex             layer_idx,
    const SupportElement        &parent,
    const Polygons              &relevant_offset,
    Polygons                    &to_bp_data,
    Polygons                    &to_model_data,
    Polygons                    &increased,
    const coord_t                overspeed,
    const bool                   mergelayer)
{
    SupportElementState current_elem{ SupportElementState::propagate_down(parent.state) };
    Polygons check_layer_data;
    if (settings.increase_radius)
        current_elem.effective_radius_height += 1;
    coord_t radius = support_element_collision_radius(config, current_elem);

    if (settings.move) {
        increased = relevant_offset;
        if (overspeed > 0) {
            const coord_t safe_movement_distance =
                (current_elem.use_min_xy_dist ? config.xy_min_distance : config.xy_distance) +
                (std::min(config.z_distance_top_layers, config.z_distance_bottom_layers) > 0 ? config.min_feature_size : 0);
            // The difference to ensure that the result not only conforms to wall_restriction, but collision/avoidance is done later.
            // The higher last_safe_step_movement_distance comes exactly from the fact that the collision will be subtracted later.
            increased = safe_offset_inc(increased, overspeed, volumes.getWallRestriction(support_element_collision_radius(config, parent.state), layer_idx, parent.state.use_min_xy_dist),
                safe_movement_distance, safe_movement_distance + radius, 1);
        }
        if (settings.no_error && settings.move)
            // as ClipperLib::jtRound has to be used for offsets this simplify is VERY important for performance.
            polygons_simplify(increased, scale_d(0.025), polygons_strictly_simple);
    } else
        // if no movement is done the areas keep parent area as no move == offset(0)
        increased = parent.influence_area;

    if (mergelayer || current_elem.to_buildplate) {
        to_bp_data = safe_union(diff_clipped(increased, volumes.getAvoidance(radius, layer_idx - 1, settings.type, false, settings.use_min_distance)));
        if (! current_elem.to_buildplate && area(to_bp_data) > tiny_area_threshold) {
            // mostly happening in the tip, but with merges one should check every time, just to be sure.
            current_elem.to_buildplate = true; // sometimes nodes that can reach the buildplate are marked as cant reach, tainting subtrees. This corrects it.
            BOOST_LOG_TRIVIAL(debug) << "Corrected taint leading to a wrong to model value on layer " << layer_idx - 1 << " targeting " <<
                current_elem.target_height << " with radius " << radius;
        }
    }
    if (config.support_rests_on_model) {
        if (mergelayer || current_elem.to_model_gracious)
            to_model_data = safe_union(diff_clipped(increased, volumes.getAvoidance(radius, layer_idx - 1, settings.type, true, settings.use_min_distance)));

        if (!current_elem.to_model_gracious) {
            if (mergelayer && area(to_model_data) >= tiny_area_threshold) {
                current_elem.to_model_gracious = true;
                BOOST_LOG_TRIVIAL(debug) << "Corrected taint leading to a wrong non gracious value on layer " << layer_idx - 1 << " targeting " <<
                    current_elem.target_height << " with radius " << radius;
            } else
                // Cannot route to gracious areas. Push the tree away from object and route it down anyways.
                to_model_data = safe_union(diff_clipped(increased, volumes.getCollision(radius, layer_idx - 1, settings.use_min_distance)));
        }
    }

    check_layer_data = current_elem.to_buildplate ? to_bp_data : to_model_data;

    if (settings.increase_radius && area(check_layer_data) > tiny_area_threshold) {
        auto validWithRadius = [&](coord_t next_radius) {
            if (volumes.ceilRadius(next_radius, settings.use_min_distance) <= volumes.ceilRadius(radius, settings.use_min_distance))
                return true;

            Polygons to_bp_data_2;
            if (current_elem.to_buildplate)
                // regular union as output will not be used later => this area should always be a subset of the safe_union one (i think)
                to_bp_data_2 = diff_clipped(increased, volumes.getAvoidance(next_radius, layer_idx - 1, settings.type, false, settings.use_min_distance));
            Polygons to_model_data_2;
            if (config.support_rests_on_model && !current_elem.to_buildplate)
                to_model_data_2 = diff_clipped(increased,
                    current_elem.to_model_gracious ?
                        volumes.getAvoidance(next_radius, layer_idx - 1, settings.type, true, settings.use_min_distance) :
                        volumes.getCollision(next_radius, layer_idx - 1, settings.use_min_distance));
            Polygons check_layer_data_2 = current_elem.to_buildplate ? to_bp_data_2 : to_model_data_2;
            return area(check_layer_data_2) > tiny_area_threshold;
        };
        coord_t ceil_radius_before = volumes.ceilRadius(radius, settings.use_min_distance);

        if (support_element_collision_radius(config, current_elem) < config.increase_radius_until_radius && support_element_collision_radius(config, current_elem) < support_element_radius(config, current_elem)) {
            coord_t target_radius = std::min(support_element_radius(config, current_elem), config.increase_radius_until_radius);
            coord_t current_ceil_radius = volumes.getRadiusNextCeil(radius, settings.use_min_distance);

            while (current_ceil_radius < target_radius && validWithRadius(volumes.getRadiusNextCeil(current_ceil_radius + 1, settings.use_min_distance)))
                current_ceil_radius = volumes.getRadiusNextCeil(current_ceil_radius + 1, settings.use_min_distance);
            size_t resulting_eff_dtt = current_elem.effective_radius_height;
            while (resulting_eff_dtt + 1 < current_elem.distance_to_top &&
                config.getRadius(resulting_eff_dtt + 1, current_elem.elephant_foot_increases) <= current_ceil_radius &&
                config.getRadius(resulting_eff_dtt + 1, current_elem.elephant_foot_increases) <= support_element_radius(config, current_elem))
                ++ resulting_eff_dtt;
            current_elem.effective_radius_height = resulting_eff_dtt;
        }
        radius = support_element_collision_radius(config, current_elem);

        const coord_t foot_radius_increase = std::max(config.bp_radius_increase_per_layer - config.branch_radius_increase_per_layer, 0.0);
        // Is nearly all of the time 1, but sometimes an increase of 1 could cause the radius to become bigger than recommendedMinRadius,
        // which could cause the radius to become bigger than precalculated.
        double planned_foot_increase = std::min(1.0, double(config.recommendedMinRadius(layer_idx - 1) - support_element_radius(config, current_elem)) / foot_radius_increase);
//FIXME
        bool increase_bp_foot = planned_foot_increase > 0 && current_elem.to_buildplate;
//        bool increase_bp_foot = false;

        if (increase_bp_foot && support_element_radius(config, current_elem) >= config.branch_radius && support_element_radius(config, current_elem) >= config.increase_radius_until_radius)
            if (validWithRadius(config.getRadius(current_elem.effective_radius_height, current_elem.elephant_foot_increases + planned_foot_increase))) {
                current_elem.elephant_foot_increases += planned_foot_increase;
                radius = support_element_collision_radius(config, current_elem);
            }

        if (ceil_radius_before != volumes.ceilRadius(radius, settings.use_min_distance)) {
            if (current_elem.to_buildplate)
                to_bp_data = safe_union(diff_clipped(increased, volumes.getAvoidance(radius, layer_idx - 1, settings.type, false, settings.use_min_distance)));
            if (config.support_rests_on_model && (!current_elem.to_buildplate || mergelayer))
                to_model_data = safe_union(diff_clipped(increased,
                    current_elem.to_model_gracious ?
                        volumes.getAvoidance(radius, layer_idx - 1, settings.type, true, settings.use_min_distance) :
                        volumes.getCollision(radius, layer_idx - 1, settings.use_min_distance)
                ));
            check_layer_data = current_elem.to_buildplate ? to_bp_data : to_model_data;
            if (area(check_layer_data) < tiny_area_threshold) {
                BOOST_LOG_TRIVIAL(error) << "Lost area by doing catch up from " << ceil_radius_before << " to radius " <<
                    volumes.ceilRadius(support_element_collision_radius(config, current_elem), settings.use_min_distance);
                BOOST_LOG_TRIVIAL(error) << "Area lost catching up radius. May not cause visible malformation."sv;
            }
        }
    }

    return area(check_layer_data) > tiny_area_threshold ? std::optional<SupportElementState>(current_elem) : std::optional<SupportElementState>();
}

static Point move_inside_if_outside(const Polygons &polygons, Point from, int distance, int64_t maxDist2)
{
    if (! contains(polygons, from))
        move_inside(polygons, from);
    return from;
}

static unsigned int move_inside(const Polygons &polygons, Point &from, int distance, int64_t maxDist2)
{
    Point  ret = from;
    double bestDist2 = std::numeric_limits<double>::max();
    auto   bestPoly = static_cast<unsigned int>(-1);
    bool   is_already_on_correct_side_of_boundary = false; // whether [from] is already on the right side of the boundary
    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); ++ poly_idx) {
        const Polygon &poly = polygons[poly_idx];
        if (poly.size() < 2)
            continue;
        Point p0 = poly[poly.size() - 2];
        Point p1 = poly.back();
        // because we compare with vSize2 here (no division by zero), we also need to compare by vSize2 inside the loop
        // to avoid integer rounding edge cases
        bool projected_p_beyond_prev_segment = (p1 - p0).cast<int64_t>().dot((from - p0).cast<int64_t>()) >= (p1 - p0).cast<int64_t>().squaredNorm();
        for (const Point& p2 : poly) {
            // X = A + Normal(B-A) * (((B-A) dot (P-A)) / VSize(B-A));
            //   = A +       (B-A) *  ((B-A) dot (P-A)) / VSize2(B-A);
            // X = P projected on AB
            const Point& a = p1;
            const Point& b = p2;
            const Point& p = from;
            auto ab = (b - a).cast<int64_t>();
            auto ap = (p - a).cast<int64_t>();
            int64_t ab_length2 = ab.squaredNorm();
            if (ab_length2 <= 0) { //A = B, i.e. the input polygon had two adjacent points on top of each other.
                p1 = p2; //Skip only one of the points.
                continue;
            }
            int64_t dot_prod = ab.dot(ap);
            if (dot_prod <= 0) { // x is projected to before ab
                if (projected_p_beyond_prev_segment) {
                    //  case which looks like:   > .
                    projected_p_beyond_prev_segment = false;
                    Point& x = p1;

                    auto dist2 = (x - p).cast<int64_t>().squaredNorm();
                    if (dist2 < bestDist2) {
                        bestDist2 = dist2;
                        bestPoly = poly_idx;
                        if (distance == 0)
                            ret = x;
                        else {
                            Vec2d  abd   = ab.cast<double>();
                            Vec2d  p1p2  = (p1 - p0).cast<double>();
                            double lab   = abd.norm();
                            double lp1p2 = p1p2.norm();
                            // inward direction irrespective of sign of [distance]
                            auto inward_dir = perp(abd * (scale_d(10.0) / lab) + p1p2 * (scale_d(10.0) / lp1p2));
                            // MM2INT(10.0) to retain precision for the eventual normalization
                            ret = x + (inward_dir * (distance / inward_dir.norm())).cast<coord_t>();
                            is_already_on_correct_side_of_boundary = inward_dir.dot((p - x).cast<double>()) * distance >= 0;
                        }
                    }
                } else {
                    projected_p_beyond_prev_segment = false;
                    p0 = p1;
                    p1 = p2;
                    continue;
                }
            } else if (dot_prod >= ab_length2) {
                // x is projected to beyond ab
                projected_p_beyond_prev_segment = true;
                p0 = p1;
                p1 = p2;
                continue;
            } else {
                // x is projected to a point properly on the line segment (not onto a vertex). The case which looks like | .
                projected_p_beyond_prev_segment = false;
                Point x = a + (ab.cast<double>() * (double(dot_prod) / double(ab_length2))).cast<coord_t>();
                auto dist2 = (p - x).cast<int64_t>().squaredNorm();
                if (dist2 < bestDist2) {
                    bestDist2 = dist2;
                    bestPoly = poly_idx;
                    if (distance == 0)
                        ret = x;
                    else {
                        Vec2d abd = ab.cast<double>();
                        Vec2d inward_dir = perp(abd * (distance / abd.norm())); // inward or outward depending on the sign of [distance]
                        ret = x + inward_dir.cast<coord_t>();
                        is_already_on_correct_side_of_boundary = inward_dir.dot((p - x).cast<double>()) >= 0;
                    }
                }
            }
            p0 = p1;
            p1 = p2;
        }
    }
    // when the best point is already inside and we're moving inside, or when the best point is already outside and we're moving outside
    if (is_already_on_correct_side_of_boundary) {
        if (bestDist2 < distance * distance)
            from = ret;
        else {
            // from = from; // original point stays unaltered. It is already inside by enough distance
        }
        return bestPoly;
    } else if (bestDist2 < maxDist2) {
        from = ret;
        return bestPoly;
    }
    return -1;
}

/*!
 * \brief Set the result_on_layer point for all influence areas
 *
 * \param move_bounds[in,out] All currently existing influence areas
 */
static void create_nodes_from_area(
    const TreeVolumes       &volumes,
    const TreeSettings    &config,
    std::vector<SupportElements> &move_bounds,
    std::function<void()>         throw_on_cancel)
{
    // Initialize points on layer 0, with a "random" point in the influence area.
    // Point is chosen based on an inaccurate estimate where the branches will split into two, but every point inside the influence area would produce a valid result.
    {
        SupportElements *layer_above = move_bounds.size() > 1 ? &move_bounds[1] : nullptr;
        if (layer_above) {
            for (SupportElement &elem : *layer_above)
                elem.state.marked = false;
        }
        for (SupportElement &init : move_bounds.front()) {
            init.state.result_on_layer = move_inside_if_outside(init.influence_area, init.state.next_position);
            // Also set the parent nodes, as these will be required for the first iteration of the loop below and mark the parent nodes.
            set_points_on_areas(init, layer_above);
        }
    }

    throw_on_cancel();

    for (LayerIndex layer_idx = 1; layer_idx < LayerIndex(move_bounds.size()); ++ layer_idx) {
        auto &layer       = move_bounds[layer_idx];
        auto *layer_above = layer_idx + 1 < LayerIndex(move_bounds.size()) ? &move_bounds[layer_idx + 1] : nullptr;
        if (layer_above)
            for (SupportElement &elem : *layer_above)
                elem.state.marked = false;
        for (SupportElement &elem : layer) {
            assert(! elem.state.deleted);
            assert(elem.state.layer_idx == layer_idx);
            // check if the resulting center point is not yet set
            if (! elem.state.result_on_layer_is_set()) {
                // Buildplate only: a branch that cannot reach the build plate is dropped, as it may not rest on the model.
                if (elem.state.to_buildplate || elem.state.distance_to_top < config.min_dtt_to_model || ! config.support_rests_on_model) {
                    if (elem.state.to_buildplate) {
                        BOOST_LOG_TRIVIAL(error) << "Uninitialized Influence area targeting " << elem.state.target_position.x() << "," << elem.state.target_position.y() << ") "
                            "at target_height: " << elem.state.target_height << " layer: " << layer_idx;
                        BOOST_LOG_TRIVIAL(error) << "Uninitialized support element! A branch could be missing or exist partially."sv;
                    }
                    // we dont need to remove yet the parents as they will have a lower dtt and also no result_on_layer set
                    elem.state.deleted = true;
                } else {
                    // set the point where the branch will be placed on the model
                    if (elem.state.to_model_gracious)
                        set_to_model_contact_to_model_gracious(volumes, config, move_bounds, elem, throw_on_cancel);
                    else
                        set_to_model_contact_simple(elem);
                }
            }
            if (! elem.state.deleted && ! elem.state.marked && elem.state.target_height == layer_idx)
                // Just a tip surface with no supporting element.
                elem.state.deleted = true;
            if (elem.state.deleted) {
                for (int32_t parent_idx : elem.parents)
                    // A top element that did not move may already have its result set.
                    // As this branch needs to be removed => all parents result_on_layer have to be invalidated.
                    (*layer_above)[parent_idx].state.result_on_layer_reset();
            }
            if (! elem.state.deleted) {
                // Element is valid now setting points in the layer above and mark the parent nodes.
                set_points_on_areas(elem, layer_above);
            }
        }
        throw_on_cancel();
    }

    // Verify the tree connectivity including the branch slopes.
    for (LayerIndex layer_idx = 0; layer_idx + 1 < LayerIndex(move_bounds.size()); ++ layer_idx) {
        auto &layer = move_bounds[layer_idx];
        auto &above = move_bounds[layer_idx + 1];
        for (SupportElement &elem : layer)
            if (! elem.state.deleted) {
                for (int32_t iparent : elem.parents) {
                    SupportElement &parent = above[iparent];
                    assert(! parent.state.deleted);
                    assert(elem.state.result_on_layer_is_set() == parent.state.result_on_layer_is_set());
                    if (elem.state.result_on_layer_is_set()) {
                        double radius_increase = support_element_radius(config, elem) - support_element_radius(config, parent);
                        assert(radius_increase >= 0);
                        double shift = (elem.state.result_on_layer - parent.state.result_on_layer).cast<double>().norm();
                        //FIXME this assert fails a lot. Is it correct?
//                        assert(shift < radius_increase + 2. * config.maximum_move_distance_slow(layer_idx));
                    }
                }
            }
        }

    remove_deleted_elements(move_bounds);

    // Verify the tree connectivity including the branch slopes.
    for (LayerIndex layer_idx = 0; layer_idx + 1 < LayerIndex(move_bounds.size()); ++ layer_idx) {
        auto &layer = move_bounds[layer_idx];
        auto &above = move_bounds[layer_idx + 1];
        for (SupportElement &elem : layer) {
            assert(! elem.state.deleted);
            for (int32_t iparent : elem.parents) {
                SupportElement &parent = above[iparent];
                assert(! parent.state.deleted);
                assert(elem.state.result_on_layer_is_set() == parent.state.result_on_layer_is_set());
                if (elem.state.result_on_layer_is_set()) {
                    double radius_increase = support_element_radius(config, elem) - support_element_radius(config, parent);
                    assert(radius_increase >= 0);
                    double shift = (elem.state.result_on_layer - parent.state.result_on_layer).cast<double>().norm();
                    //FIXME this assert fails a lot. Is it correct?
//                    assert(shift < radius_increase + 2. * config.maximum_move_distance_slow(layer_idx));
                }
            }
        }
    }
}

/*!
 * \brief Sets the result_on_layer for all parents based on the SupportElement supplied.
 *
 * \param elem[in] The SupportElements, which parent's position should be determined.
 */
static void set_points_on_areas(const SupportElement &elem, SupportElements *layer_above)
{
    assert(!elem.state.deleted);
    assert(layer_above != nullptr || elem.parents.empty());

    // Based on the branch center point of the current layer, the point on the next (further up) layer is calculated.
    if (! elem.state.result_on_layer_is_set()) {
        BOOST_LOG_TRIVIAL(error) << "Uninitialized support element";
        BOOST_LOG_TRIVIAL(error) << "Uninitialized support element. A branch may be missing.\n"sv;
        return;
    }

    if (layer_above)
        for (int32_t next_elem_idx : elem.parents) {
            assert(next_elem_idx >= 0);
            SupportElement &next_elem = (*layer_above)[next_elem_idx];
            assert(! next_elem.state.deleted);
            // Keep a result set earlier: it belongs to a branch that did not move.
            if (! next_elem.state.result_on_layer_is_set()) {
                // Move inside has edgecases (see tests) so DONT use Polygons.inside to confirm correct move, Error with distance 0 is <= 1
                // it is not required to check if how far this move moved a point as is can be larger than maximum_movement_distance.
                // While this seems like a problem it may for example occur after merges.
                next_elem.state.result_on_layer = move_inside_if_outside(next_elem.influence_area, elem.state.result_on_layer);
                // do not call recursive because then amount of layers would be restricted by the stack size
            }
            // Mark the parent element as accessed from a valid child element.
            next_elem.state.marked = true;
        }
}

static void set_to_model_contact_simple(SupportElement &elem)
{
    const Point best = move_inside_if_outside(elem.influence_area, elem.state.next_position);
    elem.state.result_on_layer = best;
    BOOST_LOG_TRIVIAL(debug) << "Added NON gracious Support On Model Point (" << best.x() << "," << best.y() << "). The current layer is " << elem.state.layer_idx;
}

/*!
 * \brief Get the best point to connect to the model and set the result_on_layer of the relevant SupportElement accordingly.
 *
 * \param move_bounds[in,out] All currently existing influence areas
 * \param first_elem[in,out] SupportElement that did not have its result_on_layer set meaning that it does not have a child element.
 * \param layer_idx[in] The current layer.
 */
static void set_to_model_contact_to_model_gracious(
    const TreeVolumes          &volumes,
    const TreeSettings       &config,
    std::vector<SupportElements>    &move_bounds,
    SupportElement                  &first_elem,
    std::function<void()>            throw_on_cancel)
{
    SupportElement *last_successfull_layer = nullptr;

    // check for every layer upwards, up to the point where this influence area was created (either by initial insert or merge) if the branch could be placed on it, and highest up layer index.
    {
        SupportElement *elem = &first_elem;
        for (LayerIndex layer_check = elem->state.layer_idx;
            ! intersection(elem->influence_area, volumes.getPlaceableAreas(support_element_collision_radius(config, elem->state), layer_check, throw_on_cancel)).empty();
            elem = &move_bounds[++ layer_check][elem->parents.front()]) {
            assert(elem->state.layer_idx == layer_check);
            assert(! elem->state.deleted);
            assert(elem->state.to_model_gracious);
            last_successfull_layer = elem;
            if (elem->parents.size() != 1)
                // Reached merge point.
                break;
        }
    }

    // Could not find valid placement, even though it should exist => error handling
    if (last_successfull_layer == nullptr) {
        BOOST_LOG_TRIVIAL(warning) << "No valid placement found for to model gracious element on layer " << first_elem.state.layer_idx;
        BOOST_LOG_TRIVIAL(error) << "Could not fine valid placement on model! Just placing it down anyway. Could cause floating branches."sv;
        first_elem.state.to_model_gracious = false;
        set_to_model_contact_simple(first_elem);
    } else {
        // Found a gracious area above first_elem. Remove all below last_successfull_layer.
        {
            LayerIndex parent_layer_idx = first_elem.state.layer_idx;
            for (SupportElement *elem = &first_elem; elem != last_successfull_layer; elem = &move_bounds[++ parent_layer_idx][elem->parents.front()]) {
                assert(! elem->state.deleted);
                elem->state.deleted = true;
            }
        }
        // Guess a point inside the influence area, in which the branch will be placed in.
        const Point best = move_inside_if_outside(last_successfull_layer->influence_area, last_successfull_layer->state.next_position);
        last_successfull_layer->state.result_on_layer = best;
        BOOST_LOG_TRIVIAL(debug) << "Added gracious Support On Model Point (" << best.x() << "," << best.y() << "). The current layer is " << last_successfull_layer;
    }
}

// Remove elements marked as "deleted", update indices to parents.
static void remove_deleted_elements(std::vector<SupportElements> &move_bounds)
{
    std::vector<int32_t> map_parents;
    std::vector<int32_t> map_current;
    for (LayerIndex layer_idx = LayerIndex(move_bounds.size()) - 1; layer_idx >= 0; -- layer_idx) {
        SupportElements &layer = move_bounds[layer_idx];
        map_current.clear();
        for (int32_t i = 0; i < int32_t(layer.size());) {
            SupportElement &element = layer[i];
            if (element.state.deleted) {
                if (map_current.empty()) {
                    // Initialize with identity map.
                    map_current.assign(layer.size(), 0);
                    std::iota(map_current.begin(), map_current.end(), 0);
                }
                // Delete all "deleted" elements from the end of the layer vector.
                while (i < int32_t(layer.size()) && layer.back().state.deleted) {
                    layer.pop_back();
                    // Mark as deleted in the map.
                    map_current[layer.size()] = -1;
                }
                assert(i == layer.size() || i + 1 < layer.size());
                if (i + 1 < int32_t(layer.size())) {
                    element = std::move(layer.back());
                    layer.pop_back();
                    // Mark the current element as deleted.
                    map_current[i] = -1;
                    // Mark the moved element as moved to index i.
                    map_current[layer.size()] = i;
                }
            } else {
                // Current element is not deleted. Update its parent indices.
                if (! map_parents.empty())
                    for (int32_t &parent_idx : element.parents)
                        parent_idx = map_parents[parent_idx];
                ++ i;
            }
        }
        std::swap(map_current, map_parents);
    }
}

// Turns the placed elements into routed trees. Elements linked through their parents belong to the same tree; the trees are
// built from the top layer down, so each node finds its carried nodes already in place.
static std::vector<RoutedTree> build_trees(const PrintObject &object, const TreeSettings &config, const std::vector<SupportElements> &move_bounds)
{
    constexpr size_t none = std::numeric_limits<size_t>::max();
    // Elements of all layers get one flat index: the layer offset plus the index in the layer.
    std::vector<size_t> first_index(move_bounds.size() + 1, 0);
    for (size_t layer_idx = 0; layer_idx < move_bounds.size(); ++ layer_idx)
        first_index[layer_idx + 1] = first_index[layer_idx] + move_bounds[layer_idx].size();
    std::vector<size_t> forest(first_index.back());
    std::iota(forest.begin(), forest.end(), size_t(0));
    for (size_t layer_idx = 0; layer_idx + 1 < move_bounds.size(); ++ layer_idx)
        for (size_t idx = 0; idx < move_bounds[layer_idx].size(); ++ idx) {
            const SupportElement &elem = move_bounds[layer_idx][idx];
            if (! elem.state.result_on_layer_is_set())
                continue;
            for (int32_t parent : elem.parents) {
                if (! move_bounds[layer_idx + 1][parent].state.result_on_layer_is_set())
                    continue;
                const size_t root = find_root(forest, first_index[layer_idx] + idx);
                const size_t parent_root = find_root(forest, first_index[layer_idx + 1] + size_t(parent));
                if (root != parent_root)
                    forest[root] = parent_root;
            }
        }

    std::vector<RoutedTree> trees;
    std::vector<size_t> tree_of_root(first_index.back(), none);
    std::vector<size_t> local_index(first_index.back(), none);
    for (size_t layer_idx = move_bounds.size(); layer_idx -- > 0; )
        for (size_t idx = 0; idx < move_bounds[layer_idx].size(); ++ idx) {
            const SupportElement &elem = move_bounds[layer_idx][idx];
            if (! elem.state.result_on_layer_is_set())
                continue;
            const size_t root = find_root(forest, first_index[layer_idx] + idx);
            if (tree_of_root[root] == none) {
                tree_of_root[root] = trees.size();
                trees.emplace_back();
            }
            RoutedTree &tree = trees[tree_of_root[root]];
            TreeNode node;
            node.layer = layer_idx;
            node.print_z = object.layer(layer_idx).unscaled_print_z();
            node.position = elem.state.result_on_layer;
            node.radius = support_element_radius(config, elem);
            node.to_buildplate = elem.state.to_buildplate;
            node.to_model_gracious = elem.state.to_model_gracious;
            node.lost = elem.state.lost;
            node.verylost = elem.state.verylost;
            for (int32_t parent : elem.parents)
                if (const size_t carried = local_index[first_index[layer_idx + 1] + size_t(parent)]; carried != none)
                    node.carries.push_back(carried);
            local_index[first_index[layer_idx] + idx] = tree.nodes.size();
            tree.nodes.push_back(std::move(node));
        }
    return trees;
}

// Find the representative of an element in the union-find forest, halving the path on the way.
static size_t find_root(std::vector<size_t> &forest, size_t idx)
{
    while (forest[idx] != idx) {
        forest[idx] = forest[forest[idx]];
        idx = forest[idx];
    }
    return idx;
}

} // namespace Slic3r::TreeFamily
