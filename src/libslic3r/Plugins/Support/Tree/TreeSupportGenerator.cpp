///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "TreeSupportGenerator.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintHelpers.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Plugins/Support/Common/SupportInterfaces.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"
#include "TreeElements.hpp"
#include "TreeRouting.hpp"
#include "TreeSettings.hpp"
#include "TreeStorage.hpp"
#include "TreeZPlanner.hpp"

/* Implementation of the tree support slabs (see TreeSupportGenerator.hpp).

   The routed nodes become segments between Z levels. A tip reaches the object layer above it, where its stack of
   interfaces starts. A root reaches the bed, or the object layer where its unsupported part comes to rest. The steps run
   in this order:
   1. root_rests() finds the object layer that each root standing on the model rests on, and add_floors() makes its contact;
   2. place_slabs() fills the span of every interval between extremes, with the middle planned by plan_branch_slabs();
   3. align_slab_bounds() snaps every slab bound that lies within the layer merge tolerance of an object print Z onto it,
      merges the support bounds that lie within the tolerance of each other, and merges the slabs that collapse;
   4. the contacts are trimmed by the object outline, and their footprints are projected into the base slabs under them;
   5. draw_branches() adds the circles of the segments that cross the middle of each base slab, then trims the slab;
   6. carve_interfaces() splits the dense interfaces out, and the remaining base slabs take their branch type. */

namespace Slic3r::TreeFamily {
namespace Common = slic3r_api::Support::Common;

namespace {
// The tool ordering takes two distinct print Z values that lie within 4 * SCALED_EPSILON of each other for one layer and asserts on them.
constexpr coord_t layer_merge_tolerance = 4 * SCALED_EPSILON;

// A stretch of a branch between two Z levels, with the centre and radius of the branch at each end (scaled).
// top_layer is the object layer whose span the stretch covers, the layer that its overhang belongs to.
struct Segment {
    coord_t bottom_z = 0, top_z = 0;
    Point bottom_centre, top_centre;
    coord_t bottom_radius = 0, top_radius = 0;
    size_t top_layer = 0;
};

// A root that stands on the model or on the bed: its object layer, the layer that its unsupported part rests on (-1 when
// none, as on the bed), the bottom of its stem, and the centre and radius of its branch.
struct RootRest {
    size_t top_layer = 0;
    LayerIndex landing = -1;
    coord_t bottom_z = 0;
    Point centre;
    coord_t radius = 0;
};

// Indices of the nodes that no node carries: the lowest nodes of the branches, where they stop.
std::vector<size_t> root_nodes(const RoutedTree &tree);
// Where a root that stands on the model comes to rest: the object layer below it that takes its unsupported part, or none
// when the walk reaches the bed or its limit.
RootRest root_rest(const PrintObject &object, const TreeNode &root, const std::vector<Polygons> &keep_out, const TreeSettings &settings);
// The rests of the roots of every tree: the bed roots of the first layer, and the roots that stand on the model.
std::vector<RootRest> root_rests(const PrintObject &object, const std::vector<RoutedTree> &trees, const std::vector<Polygons> &keep_out,
                                 const TreeSettings &settings);
// The stretches of every tree: between linked nodes, from a tip up to the layer above it, and the stem of every root.
std::vector<Segment> branch_segments(const PrintObject &object, const std::vector<RoutedTree> &trees, const std::vector<RootRest> &rests);
// The overhang rate of every object layer: the largest horizontal outline overhang per unit of height among the stretches that
// enter it, where the overhang is the centre shift plus the radius growth upwards.
std::vector<OverhangLayer> overhang_layers(const PrintObject &object, const std::vector<Segment> &segments);
// The object outline of every layer grown by the XY distance; the slabs keep clear of it.
std::vector<Polygons> keep_out_outlines(const PrintObject &object, coord_t gap_xy);
// The polygon of a branch cross-section: a circle around the centre, with the resolution the legacy tree draws it with.
Polygon branch_circle(Vec2d centre, double radius);
// The circles of the stretches that cross the middle of the Z span [bottom_z, top_z], at the centre and radius interpolated there.
Polygons branch_circles(const std::vector<Segment> &segments, coord_t bottom_z, coord_t top_z);
// Creates the bottom contacts of the roots that rest on the object: one contact per object layer, on the part of its top surfaces
// that the root circles cover.
void add_floors(const std::vector<slic3r_api::Layer> &layers, const std::vector<RootRest> &rests, const std::vector<Polygons> &keep_out,
                const Common::SupportParams &params, const slic3r_api::Config &config, Common::GeneratedSupport &out);
// Places the slabs of every interval between extremes; the middle of each interval is planned from the branch overhangs.
void place_slabs(const std::vector<slic3r_api::Layer> &layers, const Common::SupportParams &params, const TreeSettings &settings,
                 const std::vector<OverhangLayer> &overhang, const slic3r_api::Config &config, Common::GeneratedSupport &out);
// The print Z of every object layer of every object of the print, sorted and without duplicates.
std::vector<coord_t> object_print_zs(const Print &print);
// The object print Z within the layer merge tolerance of z, or z itself when none lies that close.
coord_t aligned_z(const std::vector<coord_t> &object_zs, coord_t z);
// The listed slabs of the support: the top and bottom contacts and the base slabs, which publication prints.
Common::GeneratorLayers listed_slabs(const Common::GeneratedSupport &out);
// Moves every bound of the listed slabs that lies within the layer merge tolerance of the last kept bound onto that bound; object
// print Z values are always kept.
void merge_close_bounds(const std::vector<coord_t> &object_zs, Common::GeneratedSupport &out);
// Gives the polygons of every listed slab that collapsed to zero height to a neighbor, then unlists that slab.
void merge_collapsed_slabs(Common::GeneratedSupport &out);
// The listed slab of positive height that takes the polygons of a collapsed slab: the same type below, the same type above, then
// any slab above, then any slab below.
Common::GeneratorLayer *collapse_neighbor(const Common::GeneratorLayers &listed, const Common::GeneratorLayer &collapsed);
// Starts the stem of every root that rests on a floor at the top of that floor.
void raise_stems_onto_floors(const Common::GeneratedSupport &out, std::vector<RootRest> &rests);
// Removes the footprint of every floor from the base slabs that lie below the top of the floor and above the object under it.
void clear_under_floors(const PrintObject &object, Common::GeneratedSupport &out);
// Removes from the slab the outline of every object layer that the slab overlaps, grown by the XY distance.
void trim_slab(Common::GeneratorLayer &slab, const PrintObject &object, const std::vector<Polygons> &keep_out);
// Adds the branch circles to every base slab, and trims each slab against the object.
void draw_branches(const std::vector<Segment> &segments, const std::vector<Polygons> &keep_out, const PrintObject &object,
                   Common::GeneratedSupport &out);
} // namespace

// Route the branches of one object, then build its slabs from the routed trees.
void generate_tree_support(const plugin_run_context *context, const Print &print, const PrintObject &object, bool organic,
                           Common::GeneratedSupport &out)
{
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const slic3r_api::Object object_view(reinterpret_cast<const object_handle *>(&object));
    const Common::SupportParams params(print_view, object_view);
    const TreeSettings settings(object, params);
    const std::vector<RoutedTree> trees = route_trees(print, object, params, settings, out);
    generate_tree_slabs(context, print, object, trees, organic, out);
}

// Build the slabs of the given routed trees in the order of the file header.
void generate_tree_slabs(const plugin_run_context *context, const Print &print, const PrintObject &object, const std::vector<RoutedTree> &trees,
                         bool organic, Common::GeneratedSupport &out)
{
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const slic3r_api::Object object_view(reinterpret_cast<const object_handle *>(&object));
    const Common::SupportParams params(print_view, object_view);
    const TreeSettings settings(object, params);
    std::vector<slic3r_api::Layer> layers;
    for (uint32_t id = 0; id < object_view.layer_count(); ++id)
        layers.push_back(object_view.layer(id));
    const slic3r_api::Config config = object_view.config();
    const std::vector<Polygons> keep_out = keep_out_outlines(object, settings.xy_min_distance);
    std::vector<RootRest> rests = root_rests(object, trees, keep_out, settings);
    add_floors(layers, rests, keep_out, params, config, out);
    raise_stems_onto_floors(out, rests);
    const std::vector<Segment> segments = branch_segments(object, trees, rests);
    place_slabs(layers, params, settings, overhang_layers(object, segments), config, out);
    align_slab_bounds(object_print_zs(print), out);
    for (Common::GeneratorLayer *contact : out.top)
        trim_slab(*contact, object, keep_out);
    for (Common::GeneratorLayer *contact : out.bottom)
        trim_slab(*contact, object, keep_out);
    Common::project_interface_contacts(params, out);
    slic3r_api::throw_if_cancelled(context);
    draw_branches(segments, keep_out, object, out);
    clear_under_floors(object, out);
    Common::carve_interfaces(params, false, out);
    for (Common::GeneratorLayer *slab : out.base)
        slab->type = organic ? Common::LayerType::Branch : Common::LayerType::Base;
}

// Snaps every bound of the slabs onto the object print Z within the layer merge tolerance of it. The snap is a pure function of
// the bound, so a bound shared by two slabs keeps one value and the slabs stay contiguous. merge_close_bounds() then merges the
// bounds of the listed slabs that lie within the tolerance of the last kept bound, so that two distinct print Z of the support
// never lie within the tolerance of each other. Each bound moves by at most the tolerance (about 0.5 um), so a slab changes its
// height by about 1 um at most; the minimum and maximum heights hold within that, an accepted limit. Slabs whose two bounds end
// on one Z collapse, and merge_collapsed_slabs() moves their polygons to a neighbor.
void align_slab_bounds(const std::vector<coord_t> &object_zs, Common::GeneratedSupport &out)
{
    for (Common::GeneratorLayer &slab : out.slabs) {
        const coord_t bottom_print_z = aligned_z(object_zs, slab.bottom_print_z());
        slab.print_z = aligned_z(object_zs, slab.print_z);
        slab.bottom_z = aligned_z(object_zs, slab.bottom_z);
        slab.height = slab.print_z - bottom_print_z;
    }
    merge_close_bounds(object_zs, out);
    merge_collapsed_slabs(out);
}

namespace {
std::vector<size_t> root_nodes(const RoutedTree &tree)
{
    std::vector<bool> carried(tree.nodes.size(), false);
    for (const TreeNode &node : tree.nodes)
        for (const size_t upper : node.carries)
            carried[upper] = true;
    std::vector<size_t> roots;
    for (size_t idx = 0; idx < tree.nodes.size(); ++idx)
        if (!carried[idx])
            roots.push_back(idx);
    return roots;
}

// The legacy stop rules bound the walk: it ends when the unsupported part is smaller than the larger of a fifth of the root
// area and half the branch area, and it goes down at most five root radii in layers. A root that hangs in the air goes down to the bed.
// Without a stop, the stem ends at the lowest layer that the walk reached, and it stays in the air.
RootRest root_rest(const PrintObject &object, const TreeNode &root, const std::vector<Polygons> &keep_out, const TreeSettings &settings)
{
    const double stop_area = std::max(0.2 * M_PI * sqr(double(root.radius)), 0.5 * M_PI * sqr(double(settings.branch_radius)));
    const LayerIndex lowest = root.verylost ? 0 :
        std::max<LayerIndex>(0, LayerIndex(root.layer) - LayerIndex(5. * double(root.radius) / double(settings.layer_height)));
    Polygons unsupported{ branch_circle(root.position.cast<double>(), root.radius) };
    RootRest rest{ root.layer, -1, 0, root.position, root.radius };
    for (LayerIndex layer = LayerIndex(root.layer) - 1; layer >= lowest; --layer) {
        unsupported = diff(unsupported, keep_out[size_t(layer)]);
        if (area(unsupported) < stop_area) {
            rest.landing = layer;
            rest.bottom_z = object.layer(size_t(layer)).scaled_print_z();
            return rest;
        }
    }
    rest.bottom_z = lowest > 0 ? object.layer(size_t(lowest)).scaled_bottom_z() : 0;
    return rest;
}

std::vector<RootRest> root_rests(const PrintObject &object, const std::vector<RoutedTree> &trees, const std::vector<Polygons> &keep_out,
                                 const TreeSettings &settings)
{
    std::vector<RootRest> rests;
    for (const RoutedTree &tree : trees)
        for (const size_t idx : root_nodes(tree)) {
            const TreeNode &root = tree.nodes[idx];
            if (root.layer == 0)
                rests.push_back(RootRest{ 0, -1, 0, root.position, root.radius });
            else if (!root.to_buildplate)
                rests.push_back(root_rest(object, root, keep_out, settings));
        }
    return rests;
}

// A tip has no node above it, so its stretch to the layer above only gives the branch its top. A root stands on the bed when
// it is on the first layer, otherwise its stem runs down to the top of its floor or to the layer where it rests (root_rest).
// The stretches of a root that lies inside its floor are cut away by clear_under_floors() once drawn.
std::vector<Segment> branch_segments(const PrintObject &object, const std::vector<RoutedTree> &trees, const std::vector<RootRest> &rests)
{
    const size_t layer_count = object.layer_count();
    std::vector<Segment> segments;
    for (const RoutedTree &tree : trees) {
        for (const TreeNode &node : tree.nodes) {
            const coord_t z = object.layer(node.layer).scaled_print_z();
            if (node.carries.empty() && node.layer + 1 < layer_count)
                segments.push_back(Segment{ z, object.layer(node.layer + 1).scaled_print_z(), node.position, node.position,
                    node.radius, node.radius, node.layer + 1 });
            for (const size_t upper : node.carries) {
                const TreeNode &above = tree.nodes[upper];
                segments.push_back(Segment{ z, object.layer(above.layer).scaled_print_z(), node.position, above.position,
                    node.radius, above.radius, above.layer });
            }
        }
    }
    for (const RootRest &rest : rests) {
        const coord_t top_z = object.layer(rest.top_layer).scaled_print_z();
        if (rest.bottom_z < top_z)
            segments.push_back(Segment{ rest.bottom_z, top_z, rest.centre, rest.centre, rest.radius, rest.radius, rest.top_layer });
    }
    return segments;
}

std::vector<OverhangLayer> overhang_layers(const PrintObject &object, const std::vector<Segment> &segments)
{
    std::vector<OverhangLayer> layers(object.layer_count());
    for (size_t id = 0; id < layers.size(); ++id) {
        layers[id].bottom_z = object.layer(id).scaled_bottom_z();
        layers[id].top_z = object.layer(id).scaled_print_z();
    }
    for (const Segment &segment : segments) {
        const double shift = (segment.top_centre - segment.bottom_centre).cast<double>().norm();
        const double overhang = std::max(0., shift + double(segment.top_radius) - double(segment.bottom_radius));
        double &rate = layers[segment.top_layer].rate;
        rate = std::max(rate, overhang / double(segment.top_z - segment.bottom_z));
    }
    return layers;
}

std::vector<Polygons> keep_out_outlines(const PrintObject &object, coord_t gap_xy)
{
    std::vector<Polygons> outlines;
    outlines.reserve(object.layer_count());
    for (size_t id = 0; id < object.layer_count(); ++id)
        outlines.push_back(to_polygons(offset_ex(object.layer(id).lslices(), float(gap_xy), ClipperLib::jtRound)));
    return outlines;
}

Polygon branch_circle(Vec2d centre, double radius)
{
    Polygon circle = make_circle(radius, SUPPORT_TREE_CIRCLE_RESOLUTION);
    circle.translate(Point(coord_t(std::llround(centre.x())), coord_t(std::llround(centre.y()))));
    return circle;
}

Polygons branch_circles(const std::vector<Segment> &segments, coord_t bottom_z, coord_t top_z)
{
    const coord_t middle = bottom_z + (top_z - bottom_z) / 2;
    Polygons circles;
    for (const Segment &segment : segments) {
        if (segment.bottom_z > middle || segment.top_z < middle)
            continue;
        const double t = double(middle - segment.bottom_z) / double(segment.top_z - segment.bottom_z);
        const double radius = double(segment.bottom_radius) + double(segment.top_radius - segment.bottom_radius) * t;
        if (radius <= SUPPORT_TREE_CIRCLE_RESOLUTION)
            continue;
        const Vec2d centre = segment.bottom_centre.cast<double>() + (segment.top_centre - segment.bottom_centre).cast<double>() * t;
        circles.push_back(branch_circle(centre, radius));
    }
    return circles;
}

void add_floors(const std::vector<slic3r_api::Layer> &layers, const std::vector<RootRest> &rests, const std::vector<Polygons> &keep_out,
                const Common::SupportParams &params, const slic3r_api::Config &config, Common::GeneratedSupport &out)
{
    std::vector<Polygons> resting(keep_out.size());
    for (const RootRest &rest : rests)
        if (rest.landing >= 0)
            resting[size_t(rest.landing)].push_back(branch_circle(rest.centre.cast<double>(), rest.radius));
    const Common::ContactDistanceType distance_type = Common::contact_distance_type(config);
    for (size_t id = 0; id + 1 < resting.size(); ++id) {
        if (resting[id].empty())
            continue;
        // The top surfaces of layer id: its outline grown by the XY distance, minus the same outline of the layer above.
        const Polygons placeable = diff(keep_out[id], keep_out[id + 1]);
        const Polygons touching = intersection(union_(resting[id]), placeable);
        if (touching.empty())
            continue;
        slic3r_api::StoredPolygonCollection touching_view(out.storage);
        native_polygons(touching_view) = touching;
        slic3r_api::StoredPolygonCollection top_view(out.storage);
        native_polygons(top_view) = placeable;
        Common::add_bottom_contact(layers, params, distance_type, id, touching_view, top_view, out);
    }
}

void place_slabs(const std::vector<slic3r_api::Layer> &layers, const Common::SupportParams &params, const TreeSettings &settings,
                 const std::vector<OverhangLayer> &overhang, const slic3r_api::Config &config, Common::GeneratedSupport &out)
{
    const int32_t interfaces = std::max(0, config.get("support_material_interface_layers").get_int());
    const slic3r_api::ConfigOption bottom_option = config.get("support_material_bottom_interface_layers");
    const bool bottom_enabled = config_option_is_enabled(bottom_option.handle(), -1) != 0;
    Common::SlabSpacing spacing;
    spacing.first_z = scale_to_layer_coord(params.first_layer_height);
    spacing.min_height = settings.min_slab_height;
    spacing.machine_step = settings.z_step;
    spacing.base_step = settings.max_slab_height;
    spacing.interface_step = interfaces > 0 ? settings.dense_slab_height : settings.max_slab_height;
    spacing.interface_layers = interfaces;
    spacing.bottom_interface_layers = bottom_enabled ? std::max(0, bottom_option.get_int()) : interfaces;
    spacing.soluble = params.soluble;
    const auto split_middle = [&](coord_t from, coord_t to) {
        coord_t bottom = from;
        for (const coord_t top : plan_branch_slabs(from, to, overhang, settings.max_overhang, settings.min_slab_height,
                 settings.max_slab_height, settings.z_step)) {
            out.base.push_back(&Common::add_layer(out, Common::LayerType::Intermediate, top, bottom, top - bottom));
            bottom = top;
        }
    };
    Common::place_intermediate_slabs(spacing, layers, out, split_middle);
}

std::vector<coord_t> object_print_zs(const Print &print)
{
    std::vector<coord_t> zs;
    for (const PrintObject &object : print.objects())
        for (size_t id = 0; id < object.layer_count(); ++id)
            zs.push_back(object.layer(id).scaled_print_z());
    std::sort(zs.begin(), zs.end());
    zs.erase(std::unique(zs.begin(), zs.end()), zs.end());
    return zs;
}

// A bound within the tolerance of an object print Z takes that Z, so the distance stays at the limit that the ordering checks.
coord_t aligned_z(const std::vector<coord_t> &object_zs, coord_t z)
{
    const auto upper = std::lower_bound(object_zs.begin(), object_zs.end(), z);
    if (upper != object_zs.end() && *upper - z <= layer_merge_tolerance)
        return *upper;
    if (upper != object_zs.begin() && z - *std::prev(upper) <= layer_merge_tolerance)
        return *std::prev(upper);
    return z;
}

void merge_close_bounds(const std::vector<coord_t> &object_zs, Common::GeneratedSupport &out)
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
        if (idx == 0 || std::binary_search(object_zs.begin(), object_zs.end(), bounds[idx]) || bounds[idx] - kept > layer_merge_tolerance)
            kept = bounds[idx];
        targets[idx] = kept;
    }
    const auto target = [&bounds, &targets](coord_t z) {
        return targets[size_t(std::lower_bound(bounds.begin(), bounds.end(), z) - bounds.begin())];
    };
    for (Common::GeneratorLayer *slab : listed) {
        const coord_t print_z = target(slab->print_z);
        const coord_t bottom_print_z = target(slab->bottom_print_z());
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

void clear_under_floors(const PrintObject &object, Common::GeneratedSupport &out)
{
    for (const Common::GeneratorLayer *floor : out.bottom) {
        const coord_t object_top = object.layer(floor->object_below).scaled_print_z();
        const Polygons &footprint = native_polygons(floor->polygons);
        for (Common::GeneratorLayer *slab : out.base)
            if (slab->print_z > object_top && slab->print_z <= floor->print_z)
                native_polygons(slab->polygons) = diff(native_polygons(slab->polygons), footprint);
    }
}

void trim_slab(Common::GeneratorLayer &slab, const PrintObject &object, const std::vector<Polygons> &keep_out)
{
    Polygons &area = native_polygons(slab.polygons);
    if (area.empty())
        return;
    Polygons outline;
    for (size_t id = 0; id < keep_out.size(); ++id) {
        const Layer &layer = object.layer(id);
        if (layer.scaled_bottom_z() < slab.print_z && layer.scaled_print_z() > slab.bottom_z)
            outline.insert(outline.end(), keep_out[id].begin(), keep_out[id].end());
    }
    if (!outline.empty())
        area = diff(area, outline);
}

void draw_branches(const std::vector<Segment> &segments, const std::vector<Polygons> &keep_out, const PrintObject &object,
                   Common::GeneratedSupport &out)
{
    for (Common::GeneratorLayer *slab : out.base) {
        Polygons &area = native_polygons(slab->polygons);
        const Polygons circles = branch_circles(segments, slab->bottom_z, slab->print_z);
        area.insert(area.end(), circles.begin(), circles.end());
        if (!area.empty())
            area = union_(area);
        trim_slab(*slab, object, keep_out);
    }
}
} // namespace
} // namespace Slic3r::TreeFamily
