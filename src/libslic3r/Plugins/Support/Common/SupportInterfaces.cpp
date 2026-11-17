///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportInterfaces.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <utility>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"

/* Implementation of the shared interface carving and of the first-layer foot widening (see SupportInterfaces.hpp).

   Both entry points read the same windows. For base slab i, the top contacts above it are taken when their bottom lies under
   the top of the window of the top interface count, and the bottom contacts below it when their top lies above the bottom of
   the window of the bottom interface count. Projection copies those footprints into the slab; carving intersects them with
   the slab's regularized union, so the interface is the part of the slab under the contacts.

   The file has three entry points. project_interface_contacts() and carve_interfaces() build the interfaces, in this order for a family
   whose base slabs do not cover the contacts. expand_first_layer_foot() then widens the first base slab. The widening uses the
   finalized contacts and interfaces (it keeps the foot clear of those on the bed), so it runs after carve_interfaces().
   contact_cones() is independent of the windows: it walks the slabs away from each contact and shrinks the contact once per
   slab whose column meets it, so a slab of another column at the same Z does not deepen the cone.
    The support ignores the brim and the skirt: it uses all the room it wants on the first layer.
    Projection and carving change polygons, not contact Z or contact order. Each
    entry validates the enabled contact lists once before its base-slab loop.

        project_interface_contacts() / carve_interfaces()
        |-- assert_contact_order(): validate the binary-search precondition
        `-- interface_contacts(): select reaching contacts for each base slab
*/

namespace slic3r_api::Support::Common {
namespace {
using Polys = StoredPolygonCollection;
constexpr clipper_join_type_t join_type = CLIPPER_JOIN_SQUARE;

// Check the stable binary-search preconditions for lists that will be queried;
// an empty base list or disabled interface side needs no sorted contacts.
void assert_contact_order(const SupportParams &p, const GeneratedSupport &out);

// Collect the top and bottom contacts whose interface windows reach base slab i, in the order of out.top and out.bottom.
void interface_contacts(const SupportParams &p, const GeneratedSupport &out, size_t i, GeneratorLayers &top, GeneratorLayers &bottom);
} // namespace

// Copy the footprints of the reaching contacts into each base slab; contacts keep their own footprints.
void project_interface_contacts(const SupportParams &p, GeneratedSupport &out)
{
    assert_contact_order(p, out);
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
    assert_contact_order(p, out);
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

std::vector<Polys> contact_cones(coord_t step, size_t max_depth, const GeneratedSupport &out, const GeneratorLayers &slabs,
                                 const std::vector<Polys> &columns)
{
    assert(columns.size() == slabs.size());
    assert(std::is_sorted(slabs.begin(), slabs.end(), [](const GeneratorLayer *a, const GeneratorLayer *b) {
        return a->print_z < b->print_z;
    }));
    std::vector<Polys> cones;
    cones.reserve(slabs.size());
    for (size_t i = 0; i < slabs.size(); ++i) cones.emplace_back(out.storage);
    if (max_depth == 0 || slabs.empty()) return cones;
    const ClipperContext clipper(out.storage);
    // Walk from the slab next to the contact, away from it, and give each slab whose column meets the contact its cone.
    const auto walk = [&](const GeneratorLayer &contact, size_t first, bool down) {
        size_t depth = 1;
        for (size_t i = first; i < slabs.size() && depth <= max_depth; down ? --i : ++i) {
            if (clipper_intersection(clipper(columns[i]), clipper(contact.polygons)).empty()) continue;
            Polys cone = clipper_offset(clipper(contact.polygons), -double(step * coord_t(depth)), join_type, 0.)
                .to_polygon_collection();
            if (cone.empty()) break;
            cones[i].append_move_from(cone);
            ++depth;
        }
    };
    const auto print_z_less = [](const GeneratorLayer *slab, coord_t z) { return slab->print_z < z; };
    for (const GeneratorLayer *contact : out.top) {
        if (contact->polygons.empty()) continue;
        // The last slab whose top lies at or under the bottom of the contact; size_t(-1) ends the downward walk.
        const size_t end = size_t(std::upper_bound(slabs.begin(), slabs.end(), contact->bottom_z,
            [](coord_t z, const GeneratorLayer *slab) { return z < slab->print_z; }) - slabs.begin());
        if (end > 0) walk(*contact, end - 1, true);
    }
    for (const GeneratorLayer *contact : out.bottom) {
        if (contact->polygons.empty()) continue;
        // The first slab whose bottom lies at or above the top of the contact.
        size_t first = size_t(std::lower_bound(slabs.begin(), slabs.end(), contact->print_z, print_z_less) - slabs.begin());
        while (first < slabs.size() && slabs[first]->bottom_print_z() < contact->print_z) ++first;
        walk(*contact, first, false);
    }
    for (Polys &cone : cones)
        if (!cone.empty()) cone = clipper_union(clipper(cone)).to_polygon_collection();
    return cones;
}

// Widen the first base slab, the foot of the support, to improve its adhesion to the bed.
// Mechanism: the foot is out.base.front() when it starts on the bed (bottom_print_z() == 0); otherwise nothing is done, so a
// support without a base slab on the bed, such as tips standing alone on the bed, is not widened. The foot is grown by the
// raft_first_layer_expansion setting of the object (minus EPSILON), in at least 5 steps of at most the first layer flow width,
// each step being cut by the first object layer slices and its no-support zones grown by p.gap_xy, so the foot never crosses the
// object or a zone. With no expansion
// the foot is only cut. The footprints of the top contacts and interfaces that start on the bed, grown by EPSILON, are then
// subtracted, so that the dense slabs keep their area.
// Inputs: layers must contain the object's first layer as layers[0]; out.storage must stay valid, it holds the temporary areas.
// The foot is normalized at p.resolution (ensure_valid) at the end, and p.first_flow.width must be positive: it sets the step size.
// Effects and ownership: the polygons of the foot are replaced in place and the slab keeps its owner (out); the other slabs are
// not changed. The brim and the skirt are ignored: the foot may use all the room of the first layer. widen_first_layer_area()
// holds this widening, which the support columns apply to their own bed sections.
// Families: the grid's first base slab is a base slab; the tree style's is its first base slab (type Intermediate until the tree
// types it as Base), and the organic style's is its first branch slab, which the flange of the first layer then follows.
void expand_first_layer_foot(const Object &object, const std::vector<Layer> &layers, const SupportParams &p, GeneratedSupport &out)
{
    if (out.base.empty() || out.base.front()->bottom_print_z() != 0) return;
    out.base.front()->polygons = widen_first_layer_area(object, layers, p, out, out.base.front()->polygons);
}

Polys widen_first_layer_area(const Object &object, const std::vector<Layer> &layers, const SupportParams &p, const GeneratedSupport &out,
                             const Polys &area)
{
    const ClipperContext clipper(out.storage);
    Polys interface(out.storage);
    for (const GeneratorLayers *slabs : {&out.top, &out.interfaces})
        for (const GeneratorLayer *slab : *slabs)
            if (slab->bottom_print_z() == 0)
                interface.append_move_from(clipper_offset(clipper(slab->polygons), float(scale_d(EPSILON)), join_type, 0.).to_polygon_collection());
    Polys foot = area.readonly().clone(out.storage);
    const Polys trimming = clipper_offset(support_obstacle(clipper, layers[0], 0, out.no_support_zones), double(p.gap_xy), join_type,
                                          0.).to_polygon_collection();
    const double expansion = std::max(0., scale_d(object.config().get("raft_first_layer_expansion").get_float()) - scale_d(EPSILON));
    if (expansion > SCALED_EPSILON) {
        const int steps = std::max(5, int(std::ceil(expansion / p.first_flow.width)));
        for (int i = 0; i < steps; ++i)
            foot = clipper_diff(clipper_offset(clipper(foot), float(expansion / steps)), clipper(trimming)).to_polygon_collection();
    } else foot = clipper_diff(clipper(foot), clipper(trimming)).to_polygon_collection();
    foot = clipper_diff(clipper(foot), clipper(interface)).to_polygon_collection();
    foot.ensure_valid(p.resolution);
    return foot;
}

namespace {
void assert_contact_order(const SupportParams &p, const GeneratedSupport &out)
{
    assert(out.base.empty() || p.top_interfaces == 0 ||
        std::is_sorted(out.top.begin(), out.top.end(), [](const GeneratorLayer *a, const GeneratorLayer *b) {
            return a->print_z < b->print_z;
        }));
    assert(out.base.empty() || p.bottom_interfaces == 0 ||
        std::is_sorted(out.bottom.begin(), out.bottom.end(), [](const GeneratorLayer *a, const GeneratorLayer *b) {
            return a->print_z < b->print_z;
        }));
}

// Appends borrowed contacts in non-decreasing print Z: top contacts have print_z >= base.print_z and bottom_z <= the top
// window's print_z; bottom contacts have print_z in [the bottom window's bottom_z, base.bottom_z]. Equal print Z is included.
// i indexes out.base, result vectors borrow contacts, and allocation failures propagate.
void interface_contacts(const SupportParams &p, const GeneratedSupport &out, size_t i, GeneratorLayers &top, GeneratorLayers &bottom)
{
    const GeneratorLayer &base = *out.base[i];
    if (p.top_interfaces > 0) {
        const coord_t z = out.base[std::min(out.base.size() - 1, i + p.top_interfaces - 1)]->print_z;
        for (GeneratorLayers::const_iterator it = std::lower_bound(out.top.begin(), out.top.end(), base.print_z,
                 [](const GeneratorLayer *contact, coord_t value) { return contact->print_z < value; }); it != out.top.end(); ++it) {
            GeneratorLayer *contact = *it;
            if (contact->print_z >= base.print_z && contact->bottom_z <= z)
                top.push_back(contact);
        }
    }
    if (p.bottom_interfaces > 0) {
        const coord_t z = out.base[i + 1 > p.bottom_interfaces ? i + 1 - p.bottom_interfaces : 0]->bottom_z;
        const GeneratorLayers::const_iterator first = std::lower_bound(out.bottom.begin(), out.bottom.end(), z,
            [](const GeneratorLayer *contact, coord_t value) { return contact->print_z < value; });
        const GeneratorLayers::const_iterator end = std::upper_bound(first, out.bottom.end(), base.bottom_z,
            [](coord_t value, const GeneratorLayer *contact) { return value < contact->print_z; });
        for (GeneratorLayers::const_iterator it = first; it != end; ++it) {
            GeneratorLayer *contact = *it;
            if (contact->print_z >= z && contact->print_z <= base.bottom_z)
                bottom.push_back(contact);
        }
    }
}
} // namespace
} // namespace slic3r_api::Support::Common
