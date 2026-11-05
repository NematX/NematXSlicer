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
   generate_support calls project_layer and add_layer_slabs separately; Common::expand_first_layer_foot leaves an expansion ring.
   Splitting at the upper object's slices separates contact from intermediate interface. Common publication assigns
   contact marking and interface ids. Demand is final after post-infill bridge removal. Run storage owns all results;
   borrowed object geometry is unchanged. Only soluble support without a raft is supported. */

namespace slic3r_api::Support::Filled {
namespace {
using ExPolys = StoredExPolygonCollection;
// Combine carried support and upper-layer demand, subtract XY-grown slices and return storage-owned polygons.
// Borrowed inputs stay unchanged; geometry and allocation failures propagate.
ExPolys project_layer(storage_handle *storage, const ExPolys &carried, const Layer &upper,
                      const Layer &layer, const Common::SupportParams &params);
// Split support at layers[id] and append nonempty slabs at layers[id-1]; id must be in [1,layers.size()).
// Out owns the slabs in run storage; borrowed layers stay unchanged and geometry/allocation failures propagate.
void add_layer_slabs(storage_handle *storage, const std::vector<Layer> &layers, size_t id,
                     const ExPolys &support, Common::GeneratedSupport &out);
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
    for (size_t id = layers.size(); id-- > 1;) {
        throw_if_cancelled(context);
        ExPolys support = project_layer(out.storage, carried, layers[id], layers[id - 1], params);
        if (!support.empty()) add_layer_slabs(out.storage, layers, id, support, out);
        carried = std::move(support);
    }
    std::reverse(out.top.begin(), out.top.end());
    std::reverse(out.interfaces.begin(), out.interfaces.end());
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
ExPolys project_layer(storage_handle *storage, const ExPolys &carried, const Layer &upper, const Layer &layer,
                      const Common::SupportParams &params)
{
    const ClipperContext clipper(storage);
    ClipperOperand projected = clipper(carried);
    for (uint32_t island = 0; island < upper.island_count(); ++island)
        projected += clipper(upper.island(island).support_demand());
    if (projected.empty())
        return ExPolys(storage);
    const ClipperOperand mask = clipper_offset(clipper(layer.slices()), double(params.gap_xy), CLIPPER_JOIN_SQUARE, 0.);
    ExPolys support = Common::native_expolygons(clipper_diff(projected, mask));
    support.ensure_valid(params.resolution);
    return support;
}

void add_layer_slabs(storage_handle *storage, const std::vector<Layer> &layers, size_t id, const ExPolys &support,
                     Common::GeneratedSupport &out)
{
    const ClipperContext clipper(storage);
    const Layer &layer = layers[id - 1];
    const std::tuple<ClipperOperand, ClipperOperand> parts = clipper_split(clipper(support), clipper(layers[id].slices()));
    // Append a storage-owned slab and a borrowed list pointer only for a nonempty part.
    const auto add_slab = [&](Common::LayerType type, const ExPolys &area, Common::GeneratorLayers &list) {
        if (area.empty()) return;
        Common::GeneratorLayer &slab = Common::add_layer(out, type, layer.print_z(), layer.bottom_z(), layer.height());
        slab.polygons = Common::to_polygons(storage, area);
        list.push_back(&slab);
    };
    add_slab(Common::LayerType::TopContact, Common::native_expolygons(std::get<0>(parts)), out.top);
    add_slab(Common::LayerType::TopInterface, Common::native_expolygons(std::get<1>(parts)), out.interfaces);
}
}
}
