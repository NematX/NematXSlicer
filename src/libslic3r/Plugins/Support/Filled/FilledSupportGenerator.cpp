///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "FilledSupportGenerator.hpp"

#include <algorithm>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"
#include "libslic3r/Plugins/Support/Common/SupportInterfaces.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"

/* Filled support projects final demand downward, subtracting each layer's XY-grown object slices cumulatively.
   The no-support zones are subtracted with the slices, so no filled support is printed in a zone or below it.
   generate_support calls project_layer and add_layer_slabs separately; Common::expand_first_layer_foot leaves an expansion ring.
   Splitting at the upper object's slices separates contact from intermediate interface. Common publication assigns
   contact marking and interface ids. Demand is final after post-infill bridge removal. Run storage owns all results;
   borrowed object geometry is unchanged. Only soluble support without a raft is supported.
   In contact mode (SupportParams::contact_mode) the contact is projected again with gap_xy_contact instead of gap_xy, and
   every layer keeps an interface slab, possibly empty, into which Common::merge_contact_cones() merges the cones of the
   contacts above it; the slabs left empty are dropped at the end. The support carried downwards keeps gap_xy. */

namespace slic3r_api::Support::Filled {
namespace {
using ExPolys = StoredExPolygonCollection;
using Polys = StoredPolygonCollection;
// Combine carried support and upper-layer demand, subtract the slices and no-support zones of layer (object layer layer_idx;
// zones holds one collection per object layer, or nothing) grown by gap and return storage-owned polygons.
// Borrowed inputs stay unchanged; geometry and allocation failures propagate.
ExPolys project_layer(storage_handle *storage, const ExPolys &carried, const Layer &upper, const Layer &layer, size_t layer_idx,
                      const std::vector<ExPolys> &zones, coord_t gap, const Common::SupportParams &params);
// Split support at layers[id] and append nonempty slabs at layers[id-1]; id must be in [1,layers.size()). contact_support,
// when given, replaces support for the contact part. With keep_interface the interface slab is appended even when empty.
// Out owns the slabs in run storage; borrowed layers stay unchanged and geometry/allocation failures propagate.
void add_layer_slabs(storage_handle *storage, const std::vector<Layer> &layers, size_t id, const ExPolys &support,
                     const ExPolys *contact_support, bool keep_interface, Common::GeneratedSupport &out);
// Merge the cones of the contacts into the interface slabs, less the contacts at their Z and their object layer (layer_ids,
// following out.interfaces) grown by gap_xy_contact, then drop the interface slabs left empty.
void add_contact_cones(const std::vector<Layer> &layers, const std::vector<size_t> &layer_ids, const Common::SupportParams &params,
                       Common::GeneratedSupport &out);
}

void generate_support(const plugin_run_context *context, const Print &, const Object &object,
                      const Common::SupportParams &params, Common::GeneratedSupport &out)
{
    if (params.distance_type != Common::ContactDistanceType::None)
        throw std::runtime_error("Filled support needs support_material_contact_distance_type 'none'.");
    if (object.config().get("raft_layers").get_int() > 0)
        throw std::runtime_error("Filled support needs raft_layers set to 0.");
    const std::vector<Layer> layers = Common::object_layers(object);
    ExPolys carried(out.storage);
    // Contact mode: the object layer of each interface slab, top first like out.interfaces.
    std::vector<size_t> layer_ids;
    for (size_t id = layers.size(); id-- > 1;) {
        throw_if_cancelled(context);
        ExPolys support = project_layer(out.storage, carried, layers[id], layers[id - 1], id - 1, out.no_support_zones, params.gap_xy, params);
        if (params.contact_mode) {
            const ExPolys contact = project_layer(out.storage, carried, layers[id], layers[id - 1], id - 1, out.no_support_zones,
                                                  params.gap_xy_contact, params);
            add_layer_slabs(out.storage, layers, id, support, &contact, true, out);
            layer_ids.push_back(id - 1);
        } else if (!support.empty())
            add_layer_slabs(out.storage, layers, id, support, nullptr, false, out);
        carried = std::move(support);
    }
    std::reverse(out.top.begin(), out.top.end());
    std::reverse(out.interfaces.begin(), out.interfaces.end());
    if (params.contact_mode) {
        std::reverse(layer_ids.begin(), layer_ids.end());
        add_contact_cones(layers, layer_ids, params, out);
    }
    if (!carried.empty()) {
        const Layer &first = layers.front();
        Common::GeneratorLayer &foot = Common::add_layer(out, Common::LayerType::Base,
            first.print_z(), first.bottom_z(), first.height());
        foot.polygons = Common::to_polygons(out.storage, carried);
        out.base.push_back(&foot);
        Common::expand_first_layer_foot(object, layers, params, out);
    }
}

namespace {
ExPolys project_layer(storage_handle *storage, const ExPolys &carried, const Layer &upper, const Layer &layer, size_t layer_idx,
                      const std::vector<ExPolys> &zones, coord_t gap, const Common::SupportParams &params)
{
    const ClipperContext clipper(storage);
    ClipperOperand projected = clipper(carried);
    for (uint32_t island = 0; island < upper.island_count(); ++island)
        projected += clipper(upper.island(island).support_demand());
    if (projected.empty())
        return ExPolys(storage);
    const ClipperOperand mask = clipper_offset(Common::support_obstacle(clipper, layer, layer_idx, zones), double(gap),
                                               CLIPPER_JOIN_SQUARE, 0.);
    ExPolys support = Common::native_expolygons(clipper_diff(projected, mask));
    support.ensure_valid(params.resolution);
    return support;
}

void add_layer_slabs(storage_handle *storage, const std::vector<Layer> &layers, size_t id, const ExPolys &support,
                     const ExPolys *contact_support, bool keep_interface, Common::GeneratedSupport &out)
{
    const ClipperContext clipper(storage);
    const Layer &layer = layers[id - 1];
    const std::tuple<ClipperOperand, ClipperOperand> parts = clipper_split(clipper(support), clipper(layers[id].slices()));
    // Append a storage-owned slab and a borrowed list pointer only for a nonempty part, unless it must be kept.
    const auto add_slab = [&](Common::LayerType type, const ExPolys &area, Common::GeneratorLayers &list, bool keep) {
        if (area.empty() && !keep) return;
        Common::GeneratorLayer &slab = Common::add_layer(out, type, layer.print_z(), layer.bottom_z(), layer.height());
        slab.polygons = Common::to_polygons(storage, area);
        list.push_back(&slab);
    };
    if (contact_support)
        add_slab(Common::LayerType::TopContact,
                 Common::native_expolygons(clipper_intersection(clipper(*contact_support), clipper(layers[id].slices()))), out.top, false);
    else
        add_slab(Common::LayerType::TopContact, Common::native_expolygons(std::get<0>(parts)), out.top, false);
    add_slab(Common::LayerType::TopInterface, Common::native_expolygons(std::get<1>(parts)), out.interfaces, keep_interface);
}

void add_contact_cones(const std::vector<Layer> &layers, const std::vector<size_t> &layer_ids, const Common::SupportParams &params,
                       Common::GeneratedSupport &out)
{
    const ClipperContext clipper(out.storage);
    Common::merge_contact_cones(params.contact_cone_step, params.resolution, out, out.interfaces, [&](size_t i, const Polys &cone) {
        const Common::GeneratorLayer &slab = *out.interfaces[i];
        ClipperOperand mask = clipper_offset(Common::support_obstacle(clipper, layers[layer_ids[i]], layer_ids[i], out.no_support_zones),
                                             double(params.gap_xy_contact), CLIPPER_JOIN_SQUARE, 0.);
        for (const Common::GeneratorLayer *contact : out.top)
            if (contact->print_z == slab.print_z) mask += clipper(contact->polygons);
        return clipper_diff(clipper(cone), mask).to_polygon_collection();
    });
    out.interfaces.erase(std::remove_if(out.interfaces.begin(), out.interfaces.end(),
        [](const Common::GeneratorLayer *slab) { return slab->polygons.empty(); }), out.interfaces.end());
}
}
}
