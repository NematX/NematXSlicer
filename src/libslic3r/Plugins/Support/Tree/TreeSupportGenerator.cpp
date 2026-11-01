///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "TreeSupportGenerator.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/ParallelFor.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Plugins/Support/Common/SupportInterfaces.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"
#include "libslic3r/Plugins/Support/Common/SupportCounters.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"
#include "TreeBranchMesh.hpp"
#include "TreeElements.hpp"
#include "TreeGeometry.hpp"
#include "TreeRouting.hpp"
#include "TreeSettings.hpp"
#include "TreeSmoothing.hpp"
#include "TreeZPlanner.hpp"

/* Implementation of the tree support slabs (see TreeSupportGenerator.hpp).

   The routed nodes become segments between Z levels. A tip reaches the object layer above it, where its stack of
   interfaces starts. A root reaches the bed, or the object layer where its unsupported part comes to rest. The steps run
   in this order:
   1. smooth_organic_branches() relaxes the nodes of organic trees, so that the planner measures their smoothed paths;
   2. root_rests() finds the object layer that each root standing on the model rests on, and add_floors() makes its contact, and the contact of
       each resting part of the top contacts (the part over the object on the tip layer, which has no tip: its stack rests on the floor, as in the grid);
   3. place_slabs() fills the span of every interval between extremes, with the middle planned by plan_branch_slabs(); for
      the organic branches, refine_branch_slabs() then cuts the middle slabs whose drawn sections overhang more than the limit;
   4. align_slab_bounds() snaps every slab bound that lies within the close-Z tolerance (max(z_step / 2, 0.5 um)) of an object print Z onto it,
      merges the support bounds that lie within the tolerance of each other, and merges the slabs that collapse;
   5. the contacts are trimmed by the object outline within both Z gaps, except against the layer they support or rest on; the
      floors and, with interface layers, the top contacts are then trimmed by the bridge footprints of the object layers above
      them (trim_bridge_footprints(), from the top of each slab); with two or more interface layers the contact footprints are
      then projected into the base slabs under them;
   6. draw_branches() adds the branches that cross each base slab, then trims the slab by the object outline within both Z gaps
      and, from its own top, by the bridge footprints: circles for the tree branches and the root stems, branch meshes for the
      organic branches (TreeBranchMesh.hpp). Without interface layers, draw_tip_contacts() replaces each contact footprint by
      the branches drawn at the middle of the contact, with the tip stretches that cross it, and trims it by the object outline
      within both Z gaps, except against its own roof; the rebuilt contacts are trimmed by the bridge footprints again, from their
      own top; then clear_under_floors() removes the floor footprints from the base slabs and these contacts;
   7. carve_interfaces() splits the dense interfaces out; expand_first_layer_foot() then widens the first base slab when it
      starts on the bed (the first tree base slab, or the first organic branch slab, whose flange follows it; tips on the bed
      without a base slab are not widened) by raft_first_layer_expansion; the support ignores the brim and the skirt; the base slabs
      take their branch type.

   Everything is written on the plugin API: the object and the print are views, the areas are polygon collections of the plugin
   storage. The slabs, which the family publishes, live in the run storage (the polygons of Common::GeneratorLayer). Every
   temporary area (the keep-out outlines, the branch sections, the intermediate results of the Clipper operations) lives in one
   scratch storage that exists for the whole of generate_tree_slabs() and is destroyed when it returns; a result that a slab keeps
   is moved into the polygons of the slab (move_from / append_move_from). A collection of the scratch storage must therefore never be
   kept after generate_tree_slabs() returns. Drawing indexes inclusive segment and tip Z spans against the requested section
   heights before visiting sections; every section's index list follows segment or tree/node input order. */

namespace Slic3r::TreeFamily {
namespace Common = slic3r_api::Support::Common;
using slic3r_api::PolygonCollection;
using slic3r_api::StoredPolygonCollection;

namespace {
// One area per object layer or per Z, in the scratch storage.
using PolygonList = std::vector<StoredPolygonCollection>;
// The layer views of the object, indexed by layer id.
using LayerList = std::vector<slic3r_api::Layer>;

// A stretch of a branch between two Z levels, with the centre and radius of the branch at each end (scaled).
// top_layer is the object layer whose span the stretch covers, the layer that its overhang belongs to. A stem is the vertical
// part under a root that rests on the object or the bed; both styles draw the stems as circles.
struct Segment {
    coord_t bottom_z = 0, top_z = 0;
    c_point bottom_centre{ 0, 0 }, top_centre{ 0, 0 };
    coord_t bottom_radius = 0, top_radius = 0;
    size_t top_layer = 0;
    bool stem = false;
};

// A root that stands on the model or on the bed: its object layer, the layer that its unsupported part rests on (-1 when
// none, as on the bed), the bottom of its stem, and the centre and radius of its branch.
struct RootRest {
    size_t top_layer = 0;
    LayerIndex landing = -1;
    coord_t bottom_z = 0;
    c_point centre{ 0, 0 };
    coord_t radius = 0;
};

// The object outline of every layer grown by the XY distance, in the scratch storage; the slabs keep clear of it.
PolygonList keep_out_outlines(const LayerList &layers, storage_handle *scratch, coord_t gap_xy);
// Indices of the nodes that no node carries: the lowest nodes of the branches, where they stop.
std::vector<size_t> root_nodes(const RoutedTree &tree);
// Where a root that stands on the model comes to rest: the object layer below it that takes its unsupported part, or none
// when the walk reaches the bed or its limit.
RootRest root_rest(const LayerList &layers, const TreeNode &root, const PolygonList &keep_out, const TreeSettings &settings,
                   storage_handle *scratch);
// The polygon of a branch cross-section, appended to circles: a circle around the centre, with the resolution the legacy tree draws it
// with. The ring is built in storage and moved into the collection.
void add_branch_circle(StoredPolygonCollection &circles, storage_handle *storage, const Vec2d &centre, double radius);
// The rests of the roots of every tree: the bed roots of the first layer, and the roots that stand on the model.
std::vector<RootRest> root_rests(const LayerList &layers, const std::vector<RoutedTree> &trees, const PolygonList &keep_out,
                                 const TreeSettings &settings, storage_handle *scratch);
// The stretches of every tree: between linked nodes, from a tip up to the layer above it, and the stem of every root.
std::vector<Segment> branch_segments(const LayerList &layers, const std::vector<RoutedTree> &trees, const std::vector<RootRest> &rests);
// The overhang rate of every object layer: the largest horizontal outline overhang per unit of height among the stretches that
// enter it, where the overhang is the centre shift plus the radius growth upwards.
std::vector<OverhangLayer> overhang_layers(const LayerList &layers, const std::vector<Segment> &segments);
// The overhang rate of every object layer of an organic print, measured on the sections that its branches draw.
std::vector<OverhangLayer> section_overhang_layers(const LayerList &layers, const std::vector<RoutedTree> &trees,
                                                   const std::vector<Segment> &segments, storage_handle *scratch,
                                                   const plugin_run_context *context);
// The branch sections at the given Z values, before the object is removed, in the scratch storage: the circles of the stems and of
// the tree branches, or for organic trees the branch meshes; the parts of each section are united.
PolygonList branch_sections(const std::vector<Segment> &segments, const std::vector<RoutedTree> &trees, bool organic,
                            const std::vector<coord_t> &zs, storage_handle *scratch, const plugin_run_context *context);
// Lists the drawing spans reaching each Z, with inclusive ends and span indices in input order.
std::vector<std::vector<uint32_t>> drawing_indices(const std::vector<coord_t> &zs, const std::vector<std::pair<coord_t, coord_t>> &spans);
// Appends circles of the indexed stretches crossing the slab middle, interpolating centre/radius; organic indices contain stems only.
void add_branch_circles(StoredPolygonCollection &circles, storage_handle *storage, const std::vector<Segment> &segments, coord_t middle_z,
                         const std::vector<uint32_t> &indices);
// A ring of the lower section of an overhang measure, with the bounding box of its points: a point outside the box cannot be inside the ring or
// on its border, and the ring is at least as far from a point as its box is.
struct SectionRing {
    multipoint_const_view points;
    c_bounding_box box;
};
// The outline overhang of a section over the section below it, in scaled units; an empty section below gives none. The result is the exact
// maximum, except that the measure stops as soon as the maximum exceeds `exceed` and then returns a value above it (for a caller that only
// compares the overhang with a limit).
double section_overhang(const PolygonCollection &upper, const PolygonCollection &lower,
                        double exceed = std::numeric_limits<double>::infinity());
// The rings of a section with their boxes, in ring order.
std::vector<SectionRing> section_rings(const PolygonCollection &section);
// Whether a point lies inside an odd number of the rings of a section, so that the holes of the section are outside of it.
bool inside_section(const std::vector<SectionRing> &rings, c_point point);
// The distance in scaled units from a point to the nearest edge of the rings, or any value not above `cutoff` as soon as the nearest edge
// found so far is not farther than the cutoff.
double distance_to_outline(const std::vector<SectionRing> &rings, c_point point, double cutoff);
// Creates the bottom contacts of the roots that rest on the object and of the resting parts of the contacts: one contact per object layer, on the part of
// its top surfaces that the root circles and the resting parts cover.
void add_floors(const LayerList &layers, const std::vector<RootRest> &rests, const std::vector<RestingPart> &resting_parts, const PolygonList &keep_out,
                const Common::SupportParams &params, storage_handle *scratch, Common::GeneratedSupport &out);
// Places the slabs of every interval between extremes; the middle of each interval is planned from the branch overhangs. Returns the
// middle slabs, the ones that carry branches; the dense interface slabs are not among them.
std::vector<Common::GeneratorLayer *> place_slabs(const LayerList &layers, const Common::SupportParams &params, const TreeSettings &settings,
                                                  const std::vector<OverhangLayer> &overhang,
                                                  Common::GeneratedSupport &out);
// Cuts the middle slabs where the sections drawn at their middles overhang more than the limit, as far as the minimum height allows.
void refine_branch_slabs(const std::vector<Segment> &segments, const std::vector<RoutedTree> &trees, const TreeSettings &settings,
                         std::vector<Common::GeneratorLayer *> &branch_slabs, storage_handle *scratch,
                         const plugin_run_context *context, Common::GeneratedSupport &out);
// The Z at which the branches of a slab are drawn: the middle of its span.
coord_t slab_middle_z(const Common::GeneratorLayer &slab);
// The Z where a middle slab is cut in two: the grid point nearest its middle from below, when both parts reach the minimum height;
// the bottom of the slab when it cannot be cut.
coord_t cut_height(const Common::GeneratorLayer &slab, coord_t grid, coord_t min_height);
// Cuts a middle slab at the given Z: the lower part becomes a new middle slab listed before it, and the slab keeps the upper part.
void cut_branch_slab(Common::GeneratedSupport &out, Common::GeneratorLayer &slab, coord_t cut,
                     std::vector<Common::GeneratorLayer *> &branch_slabs);
// The print Z of every object layer of every object of the print, sorted and without duplicates.
std::vector<coord_t> object_print_zs(const slic3r_api::Print &print);
// The object print Z within the tolerance of z (the one above first), or z itself when none lies that close.
coord_t aligned_z(const std::vector<coord_t> &object_zs, coord_t z, coord_t tolerance);
// The listed slabs of the support: the top and bottom contacts and the base slabs, which publication prints.
Common::GeneratorLayers listed_slabs(const Common::GeneratedSupport &out);
// Moves every bound of the listed slabs that lies within the tolerance of the last kept bound onto that bound; object
// print Z values are always kept.
void merge_close_bounds(const std::vector<coord_t> &object_zs, Common::GeneratedSupport &out, coord_t tolerance);
// Gives the polygons of every listed slab that collapsed to zero height to a neighbor, then unlists that slab.
void merge_collapsed_slabs(Common::GeneratedSupport &out);
// The listed slab of positive height that takes the polygons of a collapsed slab: the same type below, the same type above, then
// any slab above, then any slab below.
Common::GeneratorLayer *collapse_neighbor(const Common::GeneratorLayers &listed, const Common::GeneratorLayer &collapsed);
// Starts the stem of every root that rests on a floor at the top of that floor.
void raise_stems_onto_floors(const Common::GeneratedSupport &out, std::vector<RootRest> &rests);
// Removes from the slab the outline of every object layer that lies within its Z span grown by the Z gaps (gap_above over the top,
// gap_below under the bottom), the outline being grown by the XY distance of the routing. A gap not larger than the Z tolerance counts as zero, and
// a gap larger than it is shortened by that tolerance, as in the routing windows of TreeSettings. The layer the slab supports or
// rests on (supported_layer, size_t(-1) for none) is kept: the contact placement sets its clearance, and aligning the slab bounds
// can move them by more than the tolerance.
void trim_slab(Common::GeneratorLayer &slab, const LayerList &layers, const PolygonList &keep_out, coord_t gap_above, coord_t gap_below,
               size_t supported_layer, storage_handle *scratch);
// Removes from each slab the footprint of the object bridges whose threads hang over it, grown by the XY gap, so that no support is
// printed where a bridge thread is extruded. The object layers within the slab's Z span grown by both Z gaps are already removed by
// trim_slab() with the keep_out outline (the XY distance of the routing), but the bridges are searched from the slab top, not from
// the gap limit, so that the bridges of the window layers are removed too, with the same XY gap as the keep_out outline. A slab
// that ends under the bridge threads keeps its footprint: it supports them.
void trim_bridge_footprints(const Common::GeneratorLayers &slabs, const LayerList &layers, const slic3r_api::Config &print_config,
                            const Common::SupportParams &params, storage_handle *scratch, Common::BridgeFootprintCache &bridge_cache);
// Adds to every base slab the branches that cross it, then trims each slab against the object within both Z gaps. Tree branches are
// circles; organic branches are the sections of their meshes, with the stems as circles.
void draw_branches(const std::vector<Segment> &segments, const std::vector<RoutedTree> &trees, bool organic, const LayerList &layers,
                   const PolygonList &keep_out, coord_t gap_above, coord_t gap_below, storage_handle *scratch,
                   const plugin_run_context *context, Common::GeneratedSupport &out);
// Without interface layers a top contact holds only the branches drawn at its middle and the tip stretches that cross it, and it
// takes the branch type of the base slabs; its footprint is not kept. It is trimmed within both Z gaps, except against its own roof.
void draw_tip_contacts(const std::vector<Segment> &segments, const std::vector<RoutedTree> &trees, bool organic, const LayerList &layers,
                       const PolygonList &keep_out, coord_t gap_above, coord_t gap_below, storage_handle *scratch,
                       const plugin_run_context *context, Common::GeneratedSupport &out);
// The circles of the tip stretches that cross a Z, appended to circles. A tip stretch runs from its tip up to the object layer above
// it, as in the tree style; an organic tip is only a half sphere on its own node, which can stop below that Z.
void add_tip_circles(StoredPolygonCollection &circles, storage_handle *storage, const std::vector<const TreeNode *> &tips,
                     const std::vector<uint32_t> &indices);
// Removes the footprint of every floor from the branch slabs (the base slabs and the tip contacts) that lie below the top of the
// floor and above the object under it.
void clear_under_floors(const LayerList &layers, const Common::GeneratorLayers &slabs, const Common::GeneratorLayers &floors,
                        storage_handle *scratch);
} // namespace

// Route the branches of one object, then build its slabs from the routed trees.
// Borrows the run's parameters, derives tree settings once, then routes and draws into caller-owned slabs; object layers must be sliced, and cancellation or geometry/allocation failures propagate.
void generate_tree_support(const plugin_run_context *context, const slic3r_api::Print &print, const slic3r_api::Object &object,
                           const Common::SupportParams &params, bool organic,
                           Common::GeneratedSupport &out)
{
    const TreeSettings settings(object, print, params);
    std::vector<RestingPart> resting;
    std::vector<RoutedTree> trees = route_trees(print, object, params, settings, out, context, resting);
    generate_tree_slabs(context, print, object, params, settings, trees, organic, out, resting);
}

// Build the slabs of the given routed trees in the order of the file header; organic trees are smoothed in place first. resting_parts are the resting
// parts that route_trees() returned with the trees (the parts of the contacts over the object on their tip layer, which have no tip): add_floors()
// gives them their floors, and the stacks of their contacts shorten down to those floors, as in the grid. Trees built by hand pass an empty collection.
// Borrows matching run parameters, tree settings and routed nodes, builds owned slabs in out and smooths organic nodes in place; resting parts must come from the same routing, and cancellation or geometry/allocation failures propagate.
void generate_tree_slabs(const plugin_run_context *context, const slic3r_api::Print &print, const slic3r_api::Object &object,
                         const Common::SupportParams &params, const TreeSettings &settings,
                         std::vector<RoutedTree> &trees, bool organic, Common::GeneratedSupport &out, const std::vector<RestingPart> &resting_parts)
{
    const LayerList layers = Common::object_layers(object);
    const slic3r_api::Config print_config = print.config();
    // Without interface layers the contacts are rebuilt below from the branches, and trimmed again there.
    const bool without_interfaces = !params.top_interfaces_enabled;
    // The Z gaps between the support and the object, with which the drawn branches are trimmed.
    const coord_t gap_above = scale_to_layer_coord(params.gap_above);
    const coord_t gap_below = scale_to_layer_coord(params.gap_below);
    // Every temporary area of the job lives in this storage, which is destroyed when the job returns.
    with_scratch_storage([&](storage_handle *scratch) {
        // All bridge-trimming passes below visit slabs sequentially; pieces live until this scratch-storage job ends.
        Common::BridgeFootprintCache bridge_cache;
        const PolygonList keep_out = keep_out_outlines(layers, scratch, settings.xy_distance);
        if (organic)
            smooth_organic_branches(settings.layer_print_z_mm, keep_out, trees, context);
        std::vector<RootRest> rests = root_rests(layers, trees, keep_out, settings, scratch);
        add_floors(layers, rests, resting_parts, keep_out, params, scratch, out);
        raise_stems_onto_floors(out, rests);
        const std::vector<Segment> segments = branch_segments(layers, trees, rests);
        const std::vector<OverhangLayer> overhang = organic ? section_overhang_layers(layers, trees, segments, scratch, context)
                                                            : overhang_layers(layers, segments);
        std::vector<Common::GeneratorLayer *> branch_slabs = place_slabs(layers, params, settings, overhang, out);
        if (organic)
            refine_branch_slabs(segments, trees, settings, branch_slabs, scratch, context, out);
        const std::vector<coord_t> object_zs = object_print_zs(print);
        align_slab_bounds(object_zs, out, Common::close_z_tolerance(params.z_step, object_zs));
        // A top contact lies gap_above under the object layer it supports, and a floor lies gap_below over the object layer it rests
        // on, but the other object layers near it (the neighbouring steps of a slope) must stay a full gap away too: only the layer
        // it supports or rests on is kept.
        for (Common::GeneratorLayer *contact : out.top)
            trim_slab(*contact, layers, keep_out, gap_above, gap_below, contact->object_above, scratch);
        for (Common::GeneratorLayer *contact : out.bottom)
            trim_slab(*contact, layers, keep_out, gap_above, gap_below, contact->object_below, scratch);
        // Trim contacts before projection so the projected areas follow the grid's order: it trims before carving interfaces.
        if (!without_interfaces)
            trim_bridge_footprints(out.top, layers, print_config, params, scratch, bridge_cache);
        trim_bridge_footprints(out.bottom, layers, print_config, params, scratch, bridge_cache);
        Common::project_interface_contacts(params, out);
        slic3r_api::throw_if_cancelled(context);
        draw_branches(segments, trees, organic, layers, keep_out, gap_above, gap_below, scratch, context, out);
        // Trim slabs too: a bridge thicker than one layer can reach into their Z span.
        // The search starts from the slab top and uses the bridges of the gap window and the keep-out XY gap.
        trim_bridge_footprints(out.base, layers, print_config, params, scratch, bridge_cache);
        Common::GeneratorLayers drawn_slabs = out.base;
        if (without_interfaces) {
            draw_tip_contacts(segments, trees, organic, layers, keep_out, gap_above, gap_below, scratch, context, out);
            trim_bridge_footprints(out.top, layers, print_config, params, scratch, bridge_cache);
            drawn_slabs.insert(drawn_slabs.end(), out.top.begin(), out.top.end());
        }
        clear_under_floors(layers, drawn_slabs, out.bottom, scratch);
        Common::carve_interfaces(params, false, out);
        Common::expand_first_layer_foot(object, layers, params, out);
        for (Common::GeneratorLayer *slab : out.base)
            slab->type = organic ? Common::LayerType::Branch : Common::LayerType::Base;
    });
}

// Snaps every bound of the slabs onto the object print Z within the tolerance of it. The snap is a pure function of
// the bound, so a bound shared by two slabs keeps one value and the slabs stay contiguous. merge_close_bounds() then merges the
// bounds of the listed slabs that lie within the tolerance of the last kept bound, so that two distinct print Z of the support
// never lie within the tolerance of each other. The tolerance is Common::close_z_tolerance(): max(z_step / 2, 0.5 um), bounded by half the
// smallest gap between object Z. The object
// print Z are multiples of the Z step, so two distinct ones are at least a step apart and the object Z, which never move, are never
// merged with each other. Each bound moves by at most the tolerance, so a slab changes its height by twice that at most; the minimum
// and maximum heights hold within that, an accepted limit. Slabs whose two bounds end on one Z collapse, and merge_collapsed_slabs()
// moves their polygons to a neighbor.
// Borrows sorted object Z values, updates caller-owned slab bounds and counts changed bounds in each pass; collapsed polygons move to neighboring slabs, and allocation failures propagate.
void align_slab_bounds(const std::vector<coord_t> &object_zs, Common::GeneratedSupport &out, coord_t tolerance)
{
    for (Common::GeneratorLayer &slab : out.slabs) {
        const coord_t bottom_print_z = aligned_z(object_zs, slab.bottom_print_z(), tolerance);
        if (Common::Counters::enabled)
            Common::Counters::add(Common::Counters::totals.tree_bounds_moved,
                uint64_t(bottom_print_z != slab.bottom_print_z()) + uint64_t(aligned_z(object_zs, slab.print_z, tolerance) != slab.print_z) +
                uint64_t(aligned_z(object_zs, slab.bottom_z, tolerance) != slab.bottom_z));
        slab.print_z = aligned_z(object_zs, slab.print_z, tolerance);
        slab.bottom_z = aligned_z(object_zs, slab.bottom_z, tolerance);
        slab.height = slab.print_z - bottom_print_z;
    }
    merge_close_bounds(object_zs, out, tolerance);
    merge_collapsed_slabs(out);
}

namespace {
// Each outline is the rings of the slices of the layer offset with round joins, as the legacy tree grows them.
PolygonList keep_out_outlines(const LayerList &layers, storage_handle *scratch, coord_t gap_xy)
{
    PolygonList outlines;
    outlines.reserve(layers.size());
    for (const slic3r_api::Layer &layer : layers)
        outlines.push_back(polygons_offset(scratch, layer.slices(), double(gap_xy), CLIPPER_JOIN_ROUND));
    return outlines;
}

// Selects nodes without a lower carrier from the shared link map, preserving node-index order; tree is borrowed, each carried node must have one carrier, and allocation failures propagate.
std::vector<size_t> root_nodes(const RoutedTree &tree)
{
    const std::vector<size_t> below = nodes_below(tree);
    std::vector<size_t> roots;
    for (size_t idx = 0; idx < tree.nodes.size(); ++idx)
        if (below[idx] == no_node)
            roots.push_back(idx);
    return roots;
}

// The legacy stop rules bound the walk: it ends when the unsupported part is smaller than the larger of a fifth of the root
// area and half the branch area, and it goes down at most five root radii in real Z. A root that hangs in the air goes down to the bed.
// Without a stop, the stem ends at the lowest layer that the walk reached, and it stays in the air.
RootRest root_rest(const LayerList &layers, const TreeNode &root, const PolygonList &keep_out, const TreeSettings &settings,
                   storage_handle *scratch)
{
    const double stop_area = std::max(0.2 * PI * sqr(double(root.radius)), 0.5 * PI * sqr(double(settings.branch_radius)));
    const LayerIndex lowest = root.verylost ? 0 : settings.lowest_layer_within(LayerIndex(root.layer), coord_t(5. * double(root.radius)));
    StoredPolygonCollection unsupported(scratch);
    add_branch_circle(unsupported, scratch, to_vec2d(root.position), double(root.radius));
    RootRest rest{ root.layer, -1, 0, root.position, root.radius };
    for (LayerIndex layer = LayerIndex(root.layer) - 1; layer >= lowest; --layer) {
        unsupported.move_from(polygons_diff(scratch, unsupported, keep_out[size_t(layer)]));
        if (polygons_area(unsupported) < stop_area) {
            rest.landing = layer;
            rest.bottom_z = layers[size_t(layer)].print_z();
            return rest;
        }
    }
    rest.bottom_z = lowest > 0 ? layers[size_t(lowest)].bottom_z() : 0;
    return rest;
}

// The ring is made around the origin by make_circle(), then moved to the centre rounded to the nearest unit.
void add_branch_circle(StoredPolygonCollection &circles, storage_handle *storage, const Vec2d &centre, double radius)
{
    const c_point offset{ coord_t(std::llround(centre.x())), coord_t(std::llround(centre.y())) };
    std::vector<c_point> points = make_circle(radius, SUPPORT_TREE_CIRCLE_RESOLUTION);
    for (c_point &point : points)
        point = point + offset;
    slic3r_api::StoredPolygon ring(storage);
    ring.insert_array(0, points.data(), uint32_t(points.size()));
    circles.push_back_move(std::move(ring));
}

std::vector<RootRest> root_rests(const LayerList &layers, const std::vector<RoutedTree> &trees, const PolygonList &keep_out,
                                 const TreeSettings &settings, storage_handle *scratch)
{
    std::vector<RootRest> rests;
    for (const RoutedTree &tree : trees)
        for (const size_t idx : root_nodes(tree)) {
            const TreeNode &root = tree.nodes[idx];
            if (root.layer == 0)
                rests.push_back(RootRest{ 0, -1, 0, root.position, root.radius });
            else if (!root.to_buildplate)
                rests.push_back(root_rest(layers, root, keep_out, settings, scratch));
        }
    return rests;
}

// A tip has no node above it, so its stretch to the layer above only gives the branch its top. A root stands on the bed when it is on
// the first layer, otherwise its stem runs down to the top of its floor or to the layer where it rests (root_rest). The stretches
// of a root that lies inside its floor are cut away by clear_under_floors() once drawn.
std::vector<Segment> branch_segments(const LayerList &layers, const std::vector<RoutedTree> &trees, const std::vector<RootRest> &rests)
{
    const size_t layer_count = layers.size();
    std::vector<Segment> segments;
    for (const RoutedTree &tree : trees) {
        for (const TreeNode &node : tree.nodes) {
            const coord_t z = layers[node.layer].print_z();
            if (node.carries.empty() && node.layer + 1 < layer_count)
                segments.push_back(Segment{ z, layers[node.layer + 1].print_z(), node.position, node.position, node.radius, node.radius,
                    node.layer + 1 });
            for (const size_t upper : node.carries) {
                const TreeNode &above = tree.nodes[upper];
                segments.push_back(Segment{ z, layers[above.layer].print_z(), node.position, above.position, node.radius, above.radius,
                    above.layer });
            }
        }
    }
    for (const RootRest &rest : rests) {
        const coord_t top_z = layers[rest.top_layer].print_z();
        if (rest.bottom_z < top_z)
            segments.push_back(Segment{ rest.bottom_z, top_z, rest.centre, rest.centre, rest.radius, rest.radius, rest.top_layer, true });
    }
    return segments;
}

std::vector<OverhangLayer> overhang_layers(const LayerList &layers, const std::vector<Segment> &segments)
{
    std::vector<OverhangLayer> overhang(layers.size());
    for (size_t id = 0; id < overhang.size(); ++id) {
        overhang[id].bottom_z = layers[id].bottom_z();
        overhang[id].top_z = layers[id].print_z();
    }
    for (const Segment &segment : segments) {
        const double shift = to_vec2d(segment.top_centre - segment.bottom_centre).norm();
        const double rise = std::max(0., shift + double(segment.top_radius) - double(segment.bottom_radius));
        double &rate = overhang[segment.top_layer].rate;
        rate = std::max(rate, rise / double(segment.top_z - segment.bottom_z));
    }
    return overhang;
}

// The overhang rate of every object layer, from the sections at the layer top over the sections at its bottom. It starts the
// planning only: the branches are drawn at the middle of each slab, where a tilted branch can widen inside one layer, so
// refine_branch_slabs() checks the drawn sections.
std::vector<OverhangLayer> section_overhang_layers(const LayerList &layers, const std::vector<RoutedTree> &trees,
                                                   const std::vector<Segment> &segments, storage_handle *scratch,
                                                   const plugin_run_context *context)
{
    std::vector<OverhangLayer> overhang(layers.size());
    std::vector<coord_t> zs;
    for (size_t id = 0; id < overhang.size(); ++id) {
        overhang[id].bottom_z = layers[id].bottom_z();
        overhang[id].top_z = layers[id].print_z();
        zs.push_back(overhang[id].bottom_z);
        zs.push_back(overhang[id].top_z);
    }
    std::sort(zs.begin(), zs.end());
    zs.erase(std::unique(zs.begin(), zs.end()), zs.end());
    const PolygonList sections = branch_sections(segments, trees, true, zs, scratch, context);
    // The rate of a layer depends on two read-only sections only, so the layers are independent jobs and each writes its own slot.
    slic3r_api::parallel_for_storage_with_progress(0, uint32_t(overhang.size()), context, nullptr,
        [&overhang, &sections, &zs](uint32_t id, storage_handle *) {
            OverhangLayer &layer = overhang[id];
            const StoredPolygonCollection &top = sections[size_t(std::lower_bound(zs.begin(), zs.end(), layer.top_z) - zs.begin())];
            const StoredPolygonCollection &bottom = sections[size_t(std::lower_bound(zs.begin(), zs.end(), layer.bottom_z) - zs.begin())];
            layer.rate = section_overhang(top, bottom) / double(layer.top_z - layer.bottom_z);
        });
    return overhang;
}

// Builds owned per-Z span-index lists from borrowed inclusive spans; arbitrary Z order and duplicate Z values keep their slots,
// each list follows span input order, inputs stay unchanged, counts must fit uint32_t, and allocation failures propagate.
// Entries equal the number of (Z, span) intersections, at worst zs.size() * spans.size(); each entry is small relative to
// the many-point circle it selects. Lookup temporaries are freed on return; returned lists are freed when the drawing caller returns.
std::vector<std::vector<uint32_t>> drawing_indices(const std::vector<coord_t> &zs, const std::vector<std::pair<coord_t, coord_t>> &spans)
{
    assert(zs.size() <= std::numeric_limits<uint32_t>::max());
    assert(spans.size() <= std::numeric_limits<uint32_t>::max());
    std::vector<std::pair<coord_t, uint32_t>> ordered_zs;
    ordered_zs.reserve(zs.size());
    for (size_t idx = 0; idx < zs.size(); ++idx)
        ordered_zs.emplace_back(zs[idx], uint32_t(idx));
    std::sort(ordered_zs.begin(), ordered_zs.end());
    std::vector<std::vector<uint32_t>> indices(zs.size());
    for (size_t span = 0; span < spans.size(); ++span) {
        const std::vector<std::pair<coord_t, uint32_t>>::const_iterator first = std::lower_bound(ordered_zs.cbegin(), ordered_zs.cend(), spans[span].first,
            [](const std::pair<coord_t, uint32_t> &entry, coord_t z) { return entry.first < z; });
        for (std::vector<std::pair<coord_t, uint32_t>>::const_iterator it = first;
             it != ordered_zs.cend() && it->first <= spans[span].second; ++it)
            indices[it->second].push_back(uint32_t(span));
    }
    return indices;
}

// Indexes borrowed segment spans once for these Z values, then appends selected circles after any organic mesh sections;
// scratch owns returned sections, inclusive bounds and segment order determine operands, and geometry/cancellation/allocation failures propagate.
PolygonList branch_sections(const std::vector<Segment> &segments, const std::vector<RoutedTree> &trees, bool organic,
                            const std::vector<coord_t> &zs, storage_handle *scratch, const plugin_run_context *context)
{
    PolygonList sections;
    if (organic) {
        sections = slice_branch_meshes(scratch, trees, zs, context);
    } else {
        sections.reserve(zs.size());
        for (size_t idx = 0; idx < zs.size(); ++idx)
            sections.emplace_back(scratch);
    }
    std::vector<std::pair<coord_t, coord_t>> spans;
    spans.reserve(segments.size());
    // A reversed span is empty; organic non-stems are supplied by the meshes, not by circles.
    for (const Segment &segment : segments)
        spans.push_back(organic && !segment.stem ? std::pair<coord_t, coord_t>{1, 0} :
            std::pair<coord_t, coord_t>{segment.bottom_z, segment.top_z});
    const std::vector<std::vector<uint32_t>> indices = drawing_indices(zs, spans);
    for (size_t idx = 0; idx < zs.size(); ++idx) {
        StoredPolygonCollection &section = sections[idx];
        add_branch_circles(section, scratch, segments, zs[idx], indices[idx]);
        if (!section.empty())
            section.move_from(polygons_union(scratch, section));
    }
    return sections;
}

// Appends interpolated circles from valid borrowed segment indices reaching middle_z in segment order; storage owns new rings,
// radii at or below circle resolution add nothing, and geometry/allocation failures propagate.
void add_branch_circles(StoredPolygonCollection &circles, storage_handle *storage, const std::vector<Segment> &segments, coord_t middle_z,
                         const std::vector<uint32_t> &indices)
{
    for (uint32_t idx : indices) {
        const Segment &segment = segments[idx];
        const double t = double(middle_z - segment.bottom_z) / double(segment.top_z - segment.bottom_z);
        const double radius = double(segment.bottom_radius) + double(segment.top_radius - segment.bottom_radius) * t;
        // A section whose radius does not exceed the chord error has no circle: that keeps make_circle() inside its domain
        // (error < 2 * radius) and gives the circle at least three vertices.
        if (radius <= SUPPORT_TREE_CIRCLE_RESOLUTION)
            continue;
        const Vec2d centre = to_vec2d(segment.bottom_centre) + to_vec2d(segment.top_centre - segment.bottom_centre) * t;
        add_branch_circle(circles, storage, centre, radius);
    }
}

// A point of the upper section that lies outside the lower one overhangs by its distance to the lower outline. A section with
// nothing below it is not measured: it starts on a branch or a stem that is printed below.
// The result is a maximum of minimum distances, so it does not depend on the order of the work and only work that cannot change it is skipped:
// a point inside the lower section adds nothing, a point whose nearest edge is found to be not farther than the maximum so far cannot raise
// it (distance_to_outline() stops there), and the measure ends when the maximum exceeds `exceed`, a limit that the caller only compares with.
double section_overhang(const PolygonCollection &upper, const PolygonCollection &lower, double exceed)
{
    if (lower.empty())
        return 0.;
    const std::vector<SectionRing> rings = section_rings(lower);
    double largest = 0.;
    for (uint32_t ring_idx = 0; ring_idx < upper.size(); ++ring_idx) {
        const multipoint_const_view ring = upper.at(ring_idx).view();
        for (uint32_t point_idx = 0; point_idx < ring.size; ++point_idx) {
            if (inside_section(rings, ring.array[point_idx]))
                continue;
            largest = std::max(largest, distance_to_outline(rings, ring.array[point_idx], largest));
            if (largest > exceed)
                return largest;
        }
    }
    return largest;
}

// The view of every ring and the box of its points, taken once for all the points measured against the section. A ring without points
// gets an inverted box, which rejects every point.
std::vector<SectionRing> section_rings(const PolygonCollection &section)
{
    std::vector<SectionRing> rings;
    rings.reserve(section.size());
    for (uint32_t ring_idx = 0; ring_idx < section.size(); ++ring_idx) {
        SectionRing ring{ section.at(ring_idx).view(), slic3r_api::bounding_box_empty() };
        for (uint32_t idx = 0; idx < ring.points.size; ++idx)
            slic3r_api::bounding_box_merge(ring.box, ring.points.array[idx]);
        rings.push_back(ring);
    }
    return rings;
}

// Each ring answers with the contains() of the host (border inside), and the answers are counted, so a point in a hole is outside.
// ring_crossing() is the crossing test of the host contains() (ClipperLib::PointInPolygon), so a ring answers true for an inside point
// and for a border point; a point outside the box of a ring is outside the ring and on no border of it, and is not tested.
// It is kept beside point_in_polygons() on purpose: on a hole border it counts two rings and answers false, so section_overhang()
// still computes the distance, including floating-point projection rounding, where point_in_polygons(border = true)
// would answer true and skip it.
bool inside_section(const std::vector<SectionRing> &rings, c_point point)
{
    size_t containing = 0;
    for (const SectionRing &ring : rings) {
        if (point.x < ring.box.min.x || point.x > ring.box.max.x || point.y < ring.box.min.y || point.y > ring.box.max.y)
            continue;
        containing += ring_crossing(ring.points, point) != 0 ? 1 : 0;
    }
    return containing % 2 == 1;
}

// The minimum of the distances to every edge. A ring is skipped when its box is farther than the nearest edge found so far by more than
// one scaled unit, and the scan stops when the minimum is not above the cutoff, since the final minimum can only be lower. The margin covers the
// scaled object coordinates (within about +-1.5e8 units, and differences of that order), where the rounding of the distance formula in double
// is about 1e-7 unit, far below the margin, so a skipped ring could not have lowered the minimum.
double distance_to_outline(const std::vector<SectionRing> &rings, c_point point, double cutoff)
{
    const Vec2d position = to_vec2d(point);
    double best = std::numeric_limits<double>::max();
    for (const SectionRing &ring : rings) {
        const double gap_x = std::max({ double(ring.box.min.x) - position.x(), 0., position.x() - double(ring.box.max.x) });
        const double gap_y = std::max({ double(ring.box.min.y) - position.y(), 0., position.y() - double(ring.box.max.y) });
        if (std::sqrt(gap_x * gap_x + gap_y * gap_y) - 1. >= best)
            continue;
        for (uint32_t idx = 0; idx < ring.points.size; ++idx) {
            const Vec2d a = to_vec2d(ring.points.array[idx]);
            const Vec2d ab = to_vec2d(ring.points.array[(idx + 1) % ring.points.size]) - a;
            const Vec2d ap = position - a;
            const double len2 = ab.squaredNorm();
            const double t = len2 > 0. ? std::clamp(ap.dot(ab) / len2, 0., 1.) : 0.;
            best = std::min(best, (ap - ab * t).norm());
            if (best <= cutoff)
                return best;
        }
    }
    return best;
}

// A resting part is a contact area whose stack rests on the object: its floor is made like the floor of a root, on the top surfaces of the object
// layers under it. The floors stop at the first support, as in the grid (project_columns() in GridSupportGenerator.cpp, which removes the object
// from its projection layer by layer): the walk goes down from the layer under the contact; on each layer the part of the remaining area that meets a
// placeable top surface gets its footprint there and leaves the remaining area, and the walk goes on with the rest until nothing remains or the
// lowest layer is reached. A cavity under the first support therefore gets no floor. The footprints join the circles of the roots, and the
// union of both is one floor per layer.
// Known limit: the top surfaces are the outlines grown by the XY distance (the rule of the root floors), so a band of up to that distance at the edge
// of a resting part may meet none of them and get no floor; every other point of a part meets a top surface between the tip layer (inside the object)
// and the contact (outside it).
// The place_slabs() step then shortens the stack down to the floor (the fallback of place_handed_over_interval()).
void add_floors(const LayerList &layers, const std::vector<RootRest> &rests, const std::vector<RestingPart> &resting_parts, const PolygonList &keep_out,
                const Common::SupportParams &params, storage_handle *scratch, Common::GeneratedSupport &out)
{
    PolygonList resting;
    resting.reserve(keep_out.size());
    for (size_t id = 0; id < keep_out.size(); ++id)
        resting.emplace_back(scratch);
    for (const RootRest &rest : rests)
        if (rest.landing >= 0)
            add_branch_circle(resting[size_t(rest.landing)], scratch, to_vec2d(rest.centre), double(rest.radius));
    // The top surface of the layer under a contact is never under its area, so the walk starts one layer below that one.
    for (const RestingPart &part : resting_parts) {
        StoredPolygonCollection remaining = part.area.readonly().clone(scratch);
        for (size_t id = std::min(resting.size() - 1, part.object_above - 1); id-- > 0 && ! remaining.empty();) {
            const StoredPolygonCollection placeable = polygons_diff(scratch, keep_out[id], keep_out[id + 1]);
            const StoredPolygonCollection touching = polygons_intersection(scratch, remaining, placeable);
            if (touching.empty())
                continue;
            resting[id].append_copy_from(touching);
            StoredPolygonCollection rest = polygons_diff(scratch, remaining, touching);
            remaining.move_from(rest);
        }
    }
    for (size_t id = 0; id + 1 < resting.size(); ++id) {
        if (resting[id].empty())
            continue;
        // The top surfaces of layer id: its outline grown by the XY distance, minus the same outline of the layer above.
        const StoredPolygonCollection placeable = polygons_diff(scratch, keep_out[id], keep_out[id + 1]);
        const StoredPolygonCollection covered = polygons_union(scratch, resting[id]);
        const StoredPolygonCollection touching = polygons_intersection(scratch, covered, placeable);
        if (touching.empty())
            continue;
        Common::add_bottom_contact(layers, params, id, touching, placeable, out);
    }
}

// Splits contact intervals with the borrowed run's shared counts and tree-specific steps, appending owned slabs in order; layers and settings.layer_print_z_mm each have one entry per object layer with the same indices, and geometry/allocation failures propagate.
std::vector<Common::GeneratorLayer *> place_slabs(const LayerList &layers, const Common::SupportParams &params, const TreeSettings &settings,
                                                  const std::vector<OverhangLayer> &overhang,
                                                  Common::GeneratedSupport &out)
{
    Common::SlabSpacing spacing;
    spacing.first_z = scale_to_layer_coord(params.first_layer_height);
    spacing.min_height = settings.min_slab_height;
    spacing.machine_step = settings.z_step;
    spacing.base_step = settings.max_slab_height;
    spacing.interface_step = params.top_interfaces_enabled ? settings.dense_slab_height : settings.max_slab_height;
    Common::set_interface_counts(spacing, params);
    spacing.soluble = params.soluble;
    std::vector<Common::GeneratorLayer *> branch_slabs;
    const Common::MiddleSplitter split_middle = [&](coord_t from, coord_t to) {
        coord_t bottom = from;
        for (const coord_t top : plan_branch_slabs(from, to, overhang, settings.max_overhang, settings.min_slab_height,
                 settings.max_slab_height, settings.z_step)) {
            out.base.push_back(&Common::add_layer(out, Common::LayerType::Intermediate, top, bottom, top - bottom));
            branch_slabs.push_back(out.base.back());
            bottom = top;
        }
    };
    Common::place_intermediate_slabs(spacing, layers, out, split_middle);
    return branch_slabs;
}

// Each round draws the sections at the slab middles, which are the sections that get printed. An offending pair cuts its upper slab,
// or else its lower one, and a round that cuts nothing ends the refinement. A pair that no cut can help stays, without assertion.
// Borrows routed geometry and settings, counts check rounds and adds cuts to caller-owned slabs using scratch sections; generation/cancellation/allocation failures propagate.
void refine_branch_slabs(const std::vector<Segment> &segments, const std::vector<RoutedTree> &trees, const TreeSettings &settings,
                         std::vector<Common::GeneratorLayer *> &branch_slabs, storage_handle *scratch,
                         const plugin_run_context *context, Common::GeneratedSupport &out)
{
    const coord_t grid = planning_grid(settings.z_step);
    for (;;) {
        Common::Counters::add(Common::Counters::totals.refine_rounds);
        std::vector<coord_t> middles;
        middles.reserve(out.base.size());
        for (const Common::GeneratorLayer *slab : out.base)
            middles.push_back(slab_middle_z(*slab));
        const PolygonList sections = branch_sections(segments, trees, true, middles, scratch, context);
        // Whether each pair of neighbouring slabs overhangs more than the limit. The pairs are independent jobs writing their own slot, and the
        // measure stops once a pair exceeds the limit, since only that comparison is used; the cuts are then chosen serially, in slab order.
        std::vector<char> exceeds(out.base.size(), 0);
        if (out.base.size() > 1)
            slic3r_api::parallel_for_storage_with_progress(1, uint32_t(out.base.size()), context, nullptr,
                [&exceeds, &sections, &settings](uint32_t idx, storage_handle *) {
                    exceeds[idx] = section_overhang(sections[idx], sections[idx - 1], settings.max_overhang) > settings.max_overhang ? 1 : 0;
                });
        std::vector<std::pair<Common::GeneratorLayer *, coord_t>> cuts;
        for (size_t idx = 1; idx < out.base.size(); ++idx) {
            if (!exceeds[idx])
                continue;
            for (Common::GeneratorLayer *slab : { out.base[idx], out.base[idx - 1] }) {
                if (std::find(branch_slabs.begin(), branch_slabs.end(), slab) == branch_slabs.end())
                    continue;
                const coord_t cut = cut_height(*slab, grid, settings.min_slab_height);
                if (cut == slab->print_z - slab->height)
                    continue;
                const bool listed = std::any_of(cuts.begin(), cuts.end(),
                    [slab](const std::pair<Common::GeneratorLayer *, coord_t> &entry) { return entry.first == slab; });
                if (!listed)
                    cuts.emplace_back(slab, cut);
                break;
            }
        }
        if (cuts.empty())
            return;
        for (const std::pair<Common::GeneratorLayer *, coord_t> &entry : cuts)
            cut_branch_slab(out, *entry.first, entry.second, branch_slabs);
    }
}

coord_t slab_middle_z(const Common::GeneratorLayer &slab)
{
    return slab.bottom_z + (slab.print_z - slab.bottom_z) / 2;
}

// The middle is rounded down to the grid, so that both parts keep grid heights. A slab too thin for two parts of the minimum height
// keeps its bottom as the answer.
coord_t cut_height(const Common::GeneratorLayer &slab, coord_t grid, coord_t min_height)
{
    const coord_t bottom = slab.print_z - slab.height;
    const coord_t minimum = (min_height + grid - 1) / grid * grid;
    const coord_t lower = (slab.height / 2) / grid * grid;
    if (lower < minimum || slab.height - lower < minimum)
        return bottom;
    return bottom + lower;
}

void cut_branch_slab(Common::GeneratedSupport &out, Common::GeneratorLayer &slab, coord_t cut,
                     std::vector<Common::GeneratorLayer *> &branch_slabs)
{
    const coord_t top = slab.print_z;
    const coord_t bottom = top - slab.height;
    Common::GeneratorLayer &lower = Common::add_layer(out, Common::LayerType::Intermediate, cut, bottom, cut - bottom);
    slab.bottom_z = cut;
    slab.height = top - cut;
    out.base.insert(std::find(out.base.begin(), out.base.end(), &slab), &lower);
    branch_slabs.push_back(&lower);
}

std::vector<coord_t> object_print_zs(const slic3r_api::Print &print)
{
    std::vector<coord_t> zs;
    for (uint32_t object_idx = 0; object_idx < print.object_count(); ++object_idx) {
        const slic3r_api::Object other = print.object(object_idx);
        for (uint32_t id = 0; id < other.layer_count(); ++id)
            zs.push_back(other.layer(id).print_z());
    }
    std::sort(zs.begin(), zs.end());
    zs.erase(std::unique(zs.begin(), zs.end()), zs.end());
    return zs;
}

// A bound within the tolerance of an object print Z takes that Z, so the distance stays at the limit that the ordering checks.
coord_t aligned_z(const std::vector<coord_t> &object_zs, coord_t z, coord_t tolerance)
{
    const std::vector<coord_t>::const_iterator upper = std::lower_bound(object_zs.begin(), object_zs.end(), z);
    if (upper != object_zs.end() && *upper - z <= tolerance)
        return *upper;
    if (upper != object_zs.begin() && z - *std::prev(upper) <= tolerance)
        return *std::prev(upper);
    return z;
}

// Maps nearby sorted slab bounds to retained Z values and updates borrowed slabs in place, preserving fixed object Z values; object_zs must be sorted, and allocation failures propagate.
void merge_close_bounds(const std::vector<coord_t> &object_zs, Common::GeneratedSupport &out, coord_t tolerance)
{
    const Common::GeneratorLayers listed = listed_slabs(out);
    std::vector<coord_t> bounds;
    for (const Common::GeneratorLayer *slab : listed) {
        bounds.push_back(slab->print_z);
        bounds.push_back(slab->bottom_print_z());
        bounds.push_back(slab->bottom_z);
    }
    std::sort(bounds.begin(), bounds.end());
    bounds.erase(std::unique(bounds.begin(), bounds.end()), bounds.end());
    std::vector<coord_t> targets(bounds.size());
    coord_t kept = 0;
    for (size_t idx = 0; idx < bounds.size(); ++idx) {
        if (idx == 0 || std::binary_search(object_zs.begin(), object_zs.end(), bounds[idx]) || bounds[idx] - kept > tolerance)
            kept = bounds[idx];
        targets[idx] = kept;
    }
    // Looks up an existing bound in the borrowed sorted list and returns its mapped Z without changing either list.
    const auto target = [&bounds, &targets](coord_t z) {
        return targets[size_t(std::lower_bound(bounds.begin(), bounds.end(), z) - bounds.begin())];
    };
    for (Common::GeneratorLayer *slab : listed) {
        const coord_t print_z = target(slab->print_z);
        const coord_t bottom_print_z = target(slab->bottom_print_z());
        if (Common::Counters::enabled)
            Common::Counters::add(Common::Counters::totals.tree_bounds_moved,
                uint64_t(print_z != slab->print_z) + uint64_t(bottom_print_z != slab->bottom_print_z()) +
                uint64_t(target(slab->bottom_z) != slab->bottom_z));
        slab->bottom_z = target(slab->bottom_z);
        slab->height = print_z - bottom_print_z;
        slab->print_z = print_z;
    }
}

Common::GeneratorLayers listed_slabs(const Common::GeneratedSupport &out)
{
    Common::GeneratorLayers listed = out.top;
    listed.insert(listed.end(), out.bottom.begin(), out.bottom.end());
    listed.insert(listed.end(), out.base.begin(), out.base.end());
    return listed;
}

void merge_collapsed_slabs(Common::GeneratedSupport &out)
{
    const Common::GeneratorLayers listed = listed_slabs(out);
    Common::GeneratorLayers collapsed;
    for (Common::GeneratorLayer *slab : listed)
        if (slab->height == 0)
            collapsed.push_back(slab);
    for (Common::GeneratorLayer *slab : collapsed) {
        Common::GeneratorLayer *neighbor = collapse_neighbor(listed, *slab);
        assert(neighbor != nullptr);
        neighbor->polygons.append_copy_from(slab->polygons);
        slab->polygons.clear();
        for (Common::GeneratorLayers *layers : { &out.top, &out.bottom, &out.base })
            layers->erase(std::remove(layers->begin(), layers->end(), slab), layers->end());
    }
}

Common::GeneratorLayer *collapse_neighbor(const Common::GeneratorLayers &listed, const Common::GeneratorLayer &collapsed)
{
    Common::GeneratorLayer *same_below = nullptr, *same_above = nullptr, *above = nullptr, *below = nullptr;
    for (Common::GeneratorLayer *slab : listed) {
        if (slab->height <= 0)
            continue;
        const bool same_type = slab->type == collapsed.type;
        if (slab->print_z == collapsed.print_z) {
            if (same_type && !same_below)
                same_below = slab;
            if (!below)
                below = slab;
        }
        if (slab->bottom_print_z() == collapsed.print_z) {
            if (same_type && !same_above)
                same_above = slab;
            if (!above)
                above = slab;
        }
    }
    for (Common::GeneratorLayer *candidate : { same_below, same_above, above, below })
        if (candidate)
            return candidate;
    return nullptr;
}

void raise_stems_onto_floors(const Common::GeneratedSupport &out, std::vector<RootRest> &rests)
{
    for (RootRest &rest : rests)
        for (const Common::GeneratorLayer *floor : out.bottom)
            if (rest.landing >= 0 && floor->object_below == size_t(rest.landing))
                rest.bottom_z = floor->print_z;
}

// Searches borrowed Z-ordered layers and concatenates their raw keep-out operands in layer order, replacing slab polygons in place;
// layers and keep_out share indices, scratch owns temporary handles, and geometry/allocation failures propagate.
void trim_slab(Common::GeneratorLayer &slab, const LayerList &layers, const PolygonList &keep_out, coord_t gap_above, coord_t gap_below,
               size_t supported_layer, storage_handle *scratch)
{
    if (slab.polygons.empty())
        return;
    // A clearance within the Z tolerance of the gap is accepted, so the gap is shortened by the tolerance; a gap not above it is zero.
    const coord_t eps = scale_to_layer_coord(EPSILON);
    const coord_t above = gap_above > eps ? gap_above - eps : 0;
    const coord_t below = gap_below > eps ? gap_below - eps : 0;
    // Borrow raw ring operands in layer order; print-Z equality stays outside the clearance window.
    const slic3r_api::ClipperContext clipper(scratch);
    slic3r_api::ClipperOperand outline = slic3r_api::ClipperOperand::create_empty(scratch);
    const LayerList::const_iterator first = std::upper_bound(layers.begin(), layers.end(), slab.bottom_z - below,
        [](coord_t z, const slic3r_api::Layer &layer) { return z < layer.print_z(); });
    for (size_t id = size_t(first - layers.begin()); id < keep_out.size(); ++id) {
        // The layers are sorted by Z: none of the following ones can reach the slab.
        if (layers[id].bottom_z() >= slab.print_z + above)
            break;
        if (id != supported_layer && layers[id].print_z() > slab.bottom_z - below)
            outline += clipper(keep_out[id]);
    }
    if (!outline.empty())
        slab.polygons.move_from(clipper_diff(clipper(slab.polygons), outline).to_polygon_collection());
}

// Edits the polygons of each slab in place. For every slab with polygons, Common::bridge_footprint() builds a temporary collection
// in the scratch storage with the raw offsets of the bridge areas, grown by gap_xy and starting at the object layers that begin at
// or above the slab top (from_z = slab->print_z). The offsets are normalized, united, and subtracted from the slab.
// A slab without polygons, and a slab that no bridge reaches, are left unchanged. The caller's cache holds immutable raw pieces
// in scratch storage across sequential passes, with no parallel readers. Storage outlives the cache; geometry/allocation errors
// and unknown bridge types propagate without publishing an incomplete cache layer.
void trim_bridge_footprints(const Common::GeneratorLayers &slabs, const LayerList &layers, const slic3r_api::Config &print_config,
                            const Common::SupportParams &params, storage_handle *scratch, Common::BridgeFootprintCache &bridge_cache)
{
    for (Common::GeneratorLayer *slab : slabs) {
        if (slab->polygons.empty())
            continue;
        slic3r_api::StoredExPolygonCollection footprint = Common::bridge_footprint(scratch, layers, print_config, params, slab->print_z, *slab, &bridge_cache);
        if (footprint.empty())
            continue;
        footprint.ensure_valid(params.resolution);
        const slic3r_api::StoredExPolygonCollection united = expolygons_union(scratch, footprint);
        slab->polygons.move_from(polygons_diff(scratch, slab->polygons, united));
    }
}

void draw_branches(const std::vector<Segment> &segments, const std::vector<RoutedTree> &trees, bool organic, const LayerList &layers,
                   const PolygonList &keep_out, coord_t gap_above, coord_t gap_below, storage_handle *scratch,
                   const plugin_run_context *context, Common::GeneratedSupport &out)
{
    std::vector<coord_t> middles;
    middles.reserve(out.base.size());
    for (const Common::GeneratorLayer *slab : out.base)
        middles.push_back(slab_middle_z(*slab));
    PolygonList sections = branch_sections(segments, trees, organic, middles, scratch, context);
    for (size_t idx = 0; idx < out.base.size(); ++idx) {
        Common::GeneratorLayer *slab = out.base[idx];
        slab->polygons.append_move_from(sections[idx]);
        if (!slab->polygons.empty())
            slab->polygons.move_from(polygons_union(scratch, slab->polygons));
        // A branch is drawn at the slab middle over the whole slab thickness, so the object must stay a full Z gap away on both sides.
        trim_slab(*slab, layers, keep_out, gap_above, gap_below, size_t(-1), scratch);
    }
}

// Each contact is drawn at its middle, like the base slabs, and the tip stretches that cross the middle are added to it. So a tip
// at or below the middle reaches it, although an organic tip ends in a half sphere on its own node that can stop short of it.
// Indexes borrowed tips once for these contact middles, replaces out-owned contact polygons using scratch-owned sections and
// trims them with layer-indexed keep_out; trees/layers outlive the call, and geometry/cancellation/allocation failures propagate.
void draw_tip_contacts(const std::vector<Segment> &segments, const std::vector<RoutedTree> &trees, bool organic, const LayerList &layers,
                       const PolygonList &keep_out, coord_t gap_above, coord_t gap_below, storage_handle *scratch,
                       const plugin_run_context *context, Common::GeneratedSupport &out)
{
    std::vector<coord_t> middles;
    middles.reserve(out.top.size());
    for (const Common::GeneratorLayer *contact : out.top)
        middles.push_back(slab_middle_z(*contact));
    // Tip intervals and pointers follow tree/node order; all per-contact lists are ready before any circles are appended.
    std::vector<const TreeNode *> tips;
    std::vector<std::pair<coord_t, coord_t>> tip_spans;
    for (const RoutedTree &tree : trees)
        for (const TreeNode &node : tree.nodes) {
            if (!node.carries.empty() || node.layer + 1 >= layers.size()) continue;
            tips.push_back(&node);
            tip_spans.emplace_back(layers[node.layer].print_z(), layers[node.layer + 1].print_z());
        }
    const std::vector<std::vector<uint32_t>> tip_indices = drawing_indices(middles, tip_spans);
    PolygonList sections = branch_sections(segments, trees, organic, middles, scratch, context);
    for (size_t idx = 0; idx < out.top.size(); ++idx) {
        Common::GeneratorLayer *contact = out.top[idx];
        contact->polygons.move_from(sections[idx]);
        add_tip_circles(contact->polygons, scratch, tips, tip_indices[idx]);
        if (!contact->polygons.empty())
            contact->polygons.move_from(polygons_union(scratch, contact->polygons));
        // The roof of the contact is kept: its clearance is set by the contact placement. The neighbouring roofs of a slope are not.
        trim_slab(*contact, layers, keep_out, gap_above, gap_below, contact->object_above, scratch);
        contact->type = organic ? Common::LayerType::Branch : Common::LayerType::Base;
    }
}

// Appends circles from indexed borrowed tip nodes in tree/node order; indices select tips whose inclusive layer-top interval
// reaches the contact middle, storage owns appended rings, and geometry/allocation failures propagate without changing the nodes.
void add_tip_circles(StoredPolygonCollection &circles, storage_handle *storage, const std::vector<const TreeNode *> &tips,
                     const std::vector<uint32_t> &indices)
{
    for (uint32_t idx : indices) {
        const TreeNode &node = *tips[idx];
        add_branch_circle(circles, storage, to_vec2d(node.position), double(node.radius));
    }
}

// Indexes borrowed slab pointers by top Z and subtracts each floor in floor order from selected slabs in place;
// object_below indexes layers, scratch owns boolean temporaries, and allocation/geometry failures propagate.
void clear_under_floors(const LayerList &layers, const Common::GeneratorLayers &slabs, const Common::GeneratorLayers &floors,
                        storage_handle *scratch)
{
    // The input concatenates base and contact lists; sort only borrowed pointers, leaving polygon operands untouched.
    Common::GeneratorLayers ordered = slabs;
    std::stable_sort(ordered.begin(), ordered.end(), [](const Common::GeneratorLayer *a, const Common::GeneratorLayer *b) {
        return a->print_z < b->print_z;
    });
    for (const Common::GeneratorLayer *floor : floors) {
        const coord_t object_top = layers[floor->object_below].print_z();
        for (Common::GeneratorLayers::const_iterator it = std::upper_bound(ordered.begin(), ordered.end(), object_top,
                 [](coord_t z, const Common::GeneratorLayer *slab) { return z < slab->print_z; });
             it != ordered.end() && (*it)->print_z <= floor->print_z; ++it) {
            Common::GeneratorLayer *slab = *it;
            if (slab->print_z > object_top && slab->print_z <= floor->print_z)
                slab->polygons.move_from(polygons_diff(scratch, slab->polygons, floor->polygons));
        }
    }
}
} // namespace
} // namespace Slic3r::TreeFamily
