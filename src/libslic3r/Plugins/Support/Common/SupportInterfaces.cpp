///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportInterfaces.hpp"

#include <algorithm>
#include <utility>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"

/* Implementation of the shared interface carving (see SupportInterfaces.hpp).

   Both entry points read the same windows. For base slab i, the top contacts above it are taken when their bottom lies under
   the top of the window of the top interface count, and the bottom contacts below it when their top lies above the bottom of
   the window of the bottom interface count. Projection copies those footprints into the slab; carving intersects them with
   the slab's regularized union, so the interface is the part of the slab under the contacts. */

namespace slic3r_api::Support::Common {
namespace {
using Polys = StoredPolygonCollection;
constexpr clipper_join_type_t join_type = CLIPPER_JOIN_SQUARE;

// Collect the top and bottom contacts whose interface windows reach base slab i, in the order of out.top and out.bottom.
void interface_contacts(const SupportParams &p, const GeneratedSupport &out, size_t i, GeneratorLayers &top, GeneratorLayers &bottom);
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

namespace {
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
