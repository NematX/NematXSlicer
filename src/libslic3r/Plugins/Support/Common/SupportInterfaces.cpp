///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportInterfaces.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"

/* Implementation of the shared interface carving and of the first-layer foot widening and brim removal (see SupportInterfaces.hpp).

   Both entry points read the same windows. For base slab i, the top contacts above it are taken when their bottom lies under
   the top of the window of the top interface count, and the bottom contacts below it when their top lies above the bottom of
   the window of the bottom interface count. Projection copies those footprints into the slab; carving intersects them with
   the slab's regularized union, so the interface is the part of the slab under the contacts.

   The file has four entry points. project_interface_contacts() and carve_interfaces() build the interfaces, in this order for a family
   whose base slabs do not cover the contacts. expand_first_layer_foot() and remove_first_layer_brim() then act on the slabs of the
   first layer, in this order: the first widens the first base slab, the second cuts the reserved brim area out of the slabs. The
   widening uses the finalized contacts and interfaces (it keeps the foot clear of those on the bed), so it runs after carve_interfaces(). */

namespace slic3r_api::Support::Common {
namespace {
using Polys = StoredPolygonCollection;
constexpr clipper_join_type_t join_type = CLIPPER_JOIN_SQUARE;

// Collect the top and bottom contacts whose interface windows reach base slab i, in the order of out.top and out.bottom.
void interface_contacts(const SupportParams &p, const GeneratedSupport &out, size_t i, GeneratorLayers &top, GeneratorLayers &bottom);
// Reverse the point order of every polygon, like the native polygons_reverse().
void reverse(Polys &polys);
// Whether the object has a brim under its first layer, the native PrintObject::has_brim().
bool has_brim(const Object &object, double brim_width, double brim_width_interior, int raft_layers);
} // namespace

// Copy the footprints of the reaching contacts into each base slab; contacts keep their own footprints.
void project_interface_contacts(const SupportParams &p, GeneratedSupport &out)
{
    for (size_t i = 0; i < out.base.size(); ++i) {
        GeneratorLayers top, bottom;
        interface_contacts(p, out, i, top, bottom);
        for (const GeneratorLayer *contact : top) out.base[i]->polygons.append_copy_from(contact->polygons);
        for (const GeneratorLayer *contact : bottom) out.base[i]->polygons.append_copy_from(contact->polygons);
    }
}

// Unite the reaching contact footprints, intersect them with each base slab, and split the result into top and bottom interfaces.
void carve_interfaces(const SupportParams &p, bool snug, GeneratedSupport &out)
{
    const ClipperContext clipper(out.storage);
    const double smoothing = p.interface_flow.spacing * 1.5;
    const double radius = p.interface_flow.spacing / p.interface_density;
    // Snug interfaces are closed then smoothed outwards; grid ones are only united with a safety offset.
    const auto regularize = [&](const Polys &polys) {
        if (!snug)
            return clipper_union_with_safety_offset(clipper(polys)).to_polygon_collection();
        Polys closed = clipper_offset2(clipper(polys), smoothing + radius, -smoothing, join_type, 0.).to_polygon_collection();
        closed.smooth_outward(coord_t(smoothing));
        return closed;
    };
    for (size_t i = 0; i < out.base.size(); ++i) {
        Common::GeneratorLayer &base = *out.base[i];
        GeneratorLayers top_contacts, bottom_contacts;
        interface_contacts(p, out, i, top_contacts, bottom_contacts);
        Polys top(out.storage), bottom(out.storage);
        for (const GeneratorLayer *contact : top_contacts)
            top.append_copy_from(snug ? *contact->overhang_polygons : contact->polygons);
        for (const GeneratorLayer *contact : bottom_contacts)
            bottom.append_copy_from(contact->polygons);
        if (top.empty() && bottom.empty()) continue;
        Polys united = bottom.readonly().clone(out.storage);
        united.append_copy_from(top);
        const Polys regular = regularize(united);
        united = clipper_intersection(clipper(regular), clipper(base.polygons)).to_polygon_collection();
        united.ensure_valid(p.resolution);
        if (united.empty()) continue;
        // When both projections meet, the regularized top projection takes the overlap and the bottom interface takes the remaining union.
        Polys top_part(out.storage);
        if (!top.empty() && bottom.empty())
            top_part = united.readonly().clone(out.storage);
        else if (!top.empty()) {
            const Polys regular_top = regularize(top);
            top_part = clipper_intersection(clipper(united), clipper(regular_top)).to_polygon_collection();
        }
        Polys bottom_part = clipper_diff(clipper(united), clipper(top_part)).to_polygon_collection();
        for (const std::pair<LayerType, Polys *> &part : {
                std::pair<LayerType, Polys *>{LayerType::TopInterface, &top_part},
                std::pair<LayerType, Polys *>{LayerType::BottomInterface, &bottom_part}}) {
            if (part.second->empty()) continue;
            GeneratorLayer &layer = add_layer(out, part.first, base.print_z, base.bottom_z, base.height);
            layer.polygons = std::move(*part.second);
            out.interfaces.push_back(&layer);
        }
        base.polygons = clipper_diff(clipper(base.polygons), clipper(united)).to_polygon_collection();
        base.polygons.ensure_valid(p.resolution);
    }
}

// Widen the first base slab, the foot of the support, to improve its adhesion to the bed.
// Mechanism: the foot is out.base.front() when it starts on the bed (bottom_print_z() == 0); otherwise nothing is done, so a
// support without a base slab on the bed, such as tips standing alone on the bed, is not widened. The foot is grown by the
// raft_first_layer_expansion setting of the object (minus EPSILON), in at least 5 steps of at most the first layer flow width,
// each step being cut by the first object layer slices grown by p.gap_xy, so the foot never crosses the object. With no expansion
// the foot is only cut. The footprints of the top contacts and interfaces that start on the bed, grown by EPSILON, are then
// subtracted, so that the dense slabs keep their area.
// Inputs: layers must contain the object's first layer as layers[0]; out.storage must stay valid, it holds the temporary areas.
// The foot is normalized at p.resolution (ensure_valid) at the end, and p.first_flow.width must be positive: it sets the step size.
// Effects and ownership: the polygons of the foot are replaced in place and the slab keeps its owner (out); the other slabs are
// not changed. Called before remove_first_layer_brim(), which can cut the widened foot again.
// Families: the grid's first base slab is a base slab; the tree style's is its first base slab (type Intermediate until the tree
// types it as Base), and the organic style's is its first branch slab, which the flange of the first layer then follows.
void expand_first_layer_foot(const Object &object, const std::vector<Layer> &layers, const SupportParams &p, GeneratedSupport &out)
{
    if (out.base.empty() || out.base.front()->bottom_print_z() != 0) return;
    const ClipperContext clipper(out.storage);
    Polys interface(out.storage);
    for (GeneratorLayers *slabs : {&out.top, &out.interfaces})
        for (const GeneratorLayer *slab : *slabs)
            if (slab->bottom_print_z() == 0)
                interface.append_move_from(clipper_offset(clipper(slab->polygons), float(scale_d(EPSILON)), join_type, 0.).to_polygon_collection());
    Polys &foot = out.base.front()->polygons;
    const Polys trimming = clipper_offset(clipper(layers[0].slices()), double(p.gap_xy), join_type, 0.).to_polygon_collection();
    const double expansion = std::max(0., scale_d(object.config().get("raft_first_layer_expansion").get_float()) - scale_d(EPSILON));
    if (expansion > SCALED_EPSILON) {
        const int steps = std::max(5, int(std::ceil(expansion / p.first_flow.width)));
        for (int i = 0; i < steps; ++i)
            foot = clipper_diff(clipper_offset(clipper(foot), float(expansion / steps)), clipper(trimming)).to_polygon_collection();
    } else foot = clipper_diff(clipper(foot), clipper(trimming)).to_polygon_collection();
    foot = clipper_diff(clipper(foot), clipper(interface)).to_polygon_collection();
    foot.ensure_valid(p.resolution);
}

// Reserve the brim area around the first object layer and cut it out of the support slabs of the first layer.
// Mechanism: when the object has a brim (the native PrintObject::has_brim(): a brim patch volume, or brim_width > 0 and
// brim_width_interior > 0 with no raft), the slices of layers[0] are grown into the reserved brim area, and that area is
// subtracted from every slab of out.top, out.base and out.interfaces whose print Z does not exceed p.first_layer_height. The
// floors (out.bottom) do not stand on the bed and are left alone. The area is reserved, not exactly the printed brim: with two
// different widths the printed brim is not the same shape.
// Inputs: layers must contain the object's first layer as layers[0]; the brim settings are read from the object config;
// out.storage must stay valid, it holds the temporary areas.
// Effects and ownership: the polygons of the slabs are replaced in place, the slabs keep their owner (out) and stay in their
// lists. A slab whose whole footprint lies in the brim area ends empty; it stays listed and the publication skips it, so nothing
// is printed for it.
// brim_separation is not read: the brim loops stay within brim_width from the object (Brim.cpp:1119), so this area covers them.
// The legacy support cut brim_width + brim_separation, a band that carries no loop.
// Limits, unchanged from the grid: nothing is removed unless has_brim() is true,
// so a brim with only one of the two widths positive removes nothing; and when both widths are positive the holes of the slices
// are shrunk by the outer width (brim_width), not by brim_width_interior.
void remove_first_layer_brim(const Object &object, const std::vector<Layer> &layers, const SupportParams &p, GeneratedSupport &out)
{
    const Config config = object.config();
    const double brim_width = config.get("brim_width").get_float();
    const double brim_width_interior = config.get("brim_width_interior").get_float();
    if (!has_brim(object, brim_width, brim_width_interior, config.get("raft_layers").get_int())) return;
    const ClipperContext clipper(out.storage);
    const coord_t first = scale_to_layer_coord(p.first_layer_height);
    Polys brim(out.storage);
    const double distance = scale_d(brim_width);
    const double round_tolerance = float(scale_d(0.1));
    for (const ExPolygon slice : layers[0].slices()) {
        if (brim_width > 0 && brim_width_interior > 0)
            brim.append_move_from(clipper_offset(clipper(slice), distance).to_polygon_collection());
        else {
            if (brim_width > 0)
                brim.append_move_from(clipper_offset(clipper(slice.contour()), distance, CLIPPER_JOIN_ROUND, round_tolerance).to_polygon_collection());
            else brim.push_back(slice.contour());
            Polys holes(out.storage);
            for (const Polygon hole : slice.holes()) holes.push_back(hole);
            if (brim_width_interior > 0) {
                reverse(holes);
                holes = clipper_offset(clipper(holes), -distance, CLIPPER_JOIN_ROUND, round_tolerance).to_polygon_collection();
                reverse(holes);
            }
            brim.append_move_from(holes);
        }
    }
    brim = clipper_union(clipper(brim)).to_polygon_collection();
    for (GeneratorLayers *slabs : {&out.top, &out.base, &out.interfaces})
        for (GeneratorLayer *slab : *slabs)
            if (slab->print_z <= first) slab->polygons = clipper_diff(clipper(slab->polygons), clipper(brim)).to_polygon_collection();
}

namespace {
void reverse(Polys &polys)
{
    for (uint32_t idx = 0; idx < polys.size(); ++idx)
        multipoint_reverse(polygon_as_multipoint(polygons_at(polys.mutable_handle(), idx)));
}

bool has_brim(const Object &object, double brim_width, double brim_width_interior, int raft_layers)
{
    for (uint32_t idx = 0; idx < object.volume_count(); ++idx)
        if (object.volume(idx).type() == RAW_VOLUME_TYPE_BRIM_PATCH) return true;
    return brim_width > 0 && brim_width_interior > 0 && raft_layers <= 0;
}

void interface_contacts(const SupportParams &p, const GeneratedSupport &out, size_t i, GeneratorLayers &top, GeneratorLayers &bottom)
{
    const GeneratorLayer &base = *out.base[i];
    if (p.top_interfaces > 0) {
        const coord_t z = out.base[std::min(out.base.size() - 1, i + p.top_interfaces - 1)]->print_z;
        for (GeneratorLayer *contact : out.top)
            if (contact->print_z >= base.print_z && contact->bottom_z <= z)
                top.push_back(contact);
    }
    if (p.bottom_interfaces > 0) {
        const coord_t z = out.base[i + 1 > p.bottom_interfaces ? i + 1 - p.bottom_interfaces : 0]->bottom_z;
        for (GeneratorLayer *contact : out.bottom)
            if (contact->print_z >= z && contact->print_z <= base.bottom_z)
                bottom.push_back(contact);
    }
}
} // namespace
} // namespace slic3r_api::Support::Common
