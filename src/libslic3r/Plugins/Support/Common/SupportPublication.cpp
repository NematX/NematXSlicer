///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportPublication.hpp"
#include "SupportCounters.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/DataTreeProperties.hpp"
#include "libslic3r/Api/plugin/cpp/properties/BridgeProperties.hpp"
#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"

/* Store a support algorithm's results as layers that can receive extrusion paths.
   Each input slab describes an area between a bottom and top Z; GeneratedSupport
   holds these slabs for the support algorithm.
   Slabs sharing the same exact print Z and printable thickness become one auxiliary layer, whose
   raw slice is their union in object region zero, without modifier splitting; pieces that the closing
   by +SCALED_EPSILON then -SCALED_EPSILON of the legacy toolpath fill would join form one joined island, the others
   keep the exact geometry of the union (published_islands()). Rebuilding slices
   creates islands before region membership is locked. Each island then gets one work region per
   extruder that prints on it, holding dense top or bottom surfaces, or sparse internal support
   surfaces (base and branch slabs), each typed by its role. The areas of the contact slabs of a side
   (and every bridging lower area) are marked as contact; the other dense areas are the interfaces of the side. A contact keeps the area it shares with
   an interface, so the total area of a side does not depend on the marking, and a contact merged into the base (no interface layer on its side) is base.
   Publication gives each area one owner: the lower surfaces lose the areas the upper ones print,
   the branch and the base lose the areas any interface covers, and the base also loses the areas a
   branch prints. It does not separate bridged lower slabs from non-bridged ones, so the family must
   keep those two disjoint.
   Extruder zero
   means the current extruder, stored as EXTRUDER_ID_UNKNOWN. Temporary API collections use shared
   plugin storage but are released individually; a run never clears that shared storage. Groups of
   one print Z share the native interface id of that Z, and they are published thinnest first so the
   host's ordered insertion keeps the thickest layer first at that Z.
   Slab ring bounding boxes are cached per height group. Each island intersection keeps only rings whose boxes
   can touch its contour box, in original order; enclosing rings and boundary touches remain in the operand.
   run_support_family() shares the style and demand filters, constructs one parameter object, slices the object's
   no-support zones once, calls the family's slab generator, publishes the slabs and finishes progress for valid payloads.
   Publication first removes the zones from every slab (remove_no_support_zones()), after everything the family grew.
   A matching family marks its surface calculation complete even for empty demand;
   a different family cannot certify that result.

   Representative calls:

       run_support_family()
           |-- slice_support_volumes() [the no-support zones of every object layer]
           |-- generate() [the selected family's slab-generator callback]
           `-- publish()
               |-- remove_no_support_zones() [only when the object has zones]
               |-- collect_publication_z() [group slabs by exact heights]
               `-- publish_group() [publish groups thinnest first at each Z]

   The generator callback chooses support geometry; publication assigns its layer,
   region, role and completion metadata. These are separate responsibilities. */

namespace slic3r_api::Support::Common {
namespace {
// Role rings owned by publication storage, kept in separate collections until overlap resolution. top_surfaces and bottom_surfaces hold every upper
// and every non-bridging lower ring, contacts included, so the owners of the areas are resolved on the same operands whatever the contact marking;
// top_contact_surfaces and bottom_contact_surfaces repeat the rings of the contact slabs of their side. Bridging lower rings are always contacts.
// After resolve_island_surfaces() the contact collections hold the contact areas and top_surfaces and bottom_surfaces the interface areas only.
struct IslandSurfaces {
    StoredPolygonCollection top_surfaces, top_contact_surfaces, bridge_surfaces, bottom_surfaces, bottom_contact_surfaces, sparse_surfaces, branch_surfaces;
};

// Group ranges and contact/base metadata for one exact print Z in the sorted borrowed slab list.
struct PublicationZ {
    std::vector<std::pair<size_t, size_t>> groups;
    size_t interface_count = 0, top_count = 0, end = 0;
    coord_t contact_bottom = 0, min_height = 0;
    bool has_base = false;
};
// Extruder assignments and contact-typing flags shared by all published groups.
struct PublicationTools {
    extruder_index_t base_extruder, interface_extruder;
    bool top_enabled, bottom_enabled, merge_regions;
};
// Collects equal-height ranges and interface/base metadata at one print Z.
PublicationZ collect_publication_z(const GeneratorLayers &sorted, size_t begin, coord_t z);
// Publishes one nonempty height group, its islands and their role surfaces.
void publish_group(const plugin_run_context *context, const object_handle *object, orchestrator_handle *orchestrator,
    storage_handle *storage, const ClipperContext &clipper, const GeneratorLayers &sorted, size_t begin, size_t end,
    coord_t z, uint32_t interface_id, std::map<coord_t, uint32_t> &installed_ids, const PublicationZ &metadata,
    const PublicationTools &tools);
// The islands of a layer from the union of its slabs: the plain pieces of the union (native_expolygons()), except that pieces which the closing
// by +SCALED_EPSILON then -SCALED_EPSILON joins are replaced by the closed island that contains them. The closing is first tried on each cluster of
// pieces and dropped rings that are close enough to interact, and the whole layer is closed only when one of the clusters joins its pieces.
StoredExPolygonCollection published_islands(storage_handle *storage, const ClipperContext &clipper, const ClipperOperand &united);
// For each closed island, the indices of the listed plain pieces that it covers most (see the definition); a piece that no island overlaps is in none.
// The indices in `pieces` are valid indices of `plain`, plain_boxes and closed_boxes hold one box per area of their collection, the inputs are borrowed
// and unchanged, the result is owned, and geometry/allocation failures propagate.
std::vector<std::vector<uint32_t>> assign_pieces(const ClipperContext &clipper, const StoredExPolygonCollection &plain,
    const std::vector<c_bounding_box> &plain_boxes, const std::vector<uint32_t> &pieces, const StoredExPolygonCollection &closed,
    const std::vector<c_bounding_box> &closed_boxes);
// The cluster of every box: boxes at most `gap` apart (or overlapping) on both axes, directly or through other boxes, share a cluster. The value is
// the smallest index of the cluster; the pairs are found by sweeping the boxes sorted by their minimum x. The boxes are borrowed and unchanged, the
// result is owned (one entry per box), and allocation failures propagate.
std::vector<uint32_t> box_clusters(const std::vector<c_bounding_box> &boxes, coord_t gap);
// Records each slab ring's bounding box in slab/ring order for repeated island clipping.
std::vector<std::vector<c_bounding_box>> publication_slab_bounds(const GeneratorLayers &sorted, size_t begin, size_t end);
// Collects clipped slab rings for one island in slab order.
IslandSurfaces collect_island_surfaces(storage_handle *storage, const ClipperContext &clipper, const GeneratorLayers &sorted,
    size_t begin, size_t end, const std::vector<std::vector<c_bounding_box>> &slab_bounds, const ExPolygon &island_slice, bool top_enabled, bool bottom_enabled,
    bool merge_regions, bool z_has_base, coord_t z_min_height);
// Copies only rings whose bounding boxes can touch the island, preserving their original order.
PolygonCollection island_slab_polygons(StoredPolygonCollection &filtered, const PolygonCollection &polygons,
    const std::vector<c_bounding_box> &bounds, const c_bounding_box &island_bounds);
// Removes role overlaps in top/lower/branch/base precedence, then splits each side into its contact and interface areas.
void resolve_island_surfaces(const ClipperContext &clipper, IslandSurfaces &areas);
// Installs typed surfaces by extruder on the borrowed island.
void install_island_surfaces(const plugin_run_context *context, storage_handle *storage, const ClipperContext &clipper,
    layer_island_handle *island, const IslandSurfaces &areas, extruder_index_t base_extruder, extruder_index_t interface_extruder);
// Removes from every listed slab the no-support zones of the object layers whose Z span overlaps the slab's printable interval
// (print_z - height to print_z). The family already keeps its slabs clear of the zones with the object clearance; this catches
// what grew after that trim (interface regularization, the first-layer foot), so no published surface enters a zone.
void remove_no_support_zones(const Object &object, const SupportParams &params, const ClipperContext &clipper, GeneratedSupport &support);

} // namespace

// Borrows a valid support payload, filters its style and demand, and publishes generated owned slabs before finishing progress; invalid payloads return unfinished, while cancellation and generation/publication failures propagate.
void run_support_family(const plugin_run_context *context, orchestrator_handle *orchestrator, PluginProgress &progress,
                        SupportStyle first_style, SupportStyle second_style, GenerateSupport generate)
{
    throw_if_cancelled(context);
    const run_ctx_generate_support *payload = plugin_ctx_as_generate_support(context);
    if (!payload || !payload->object || !payload->print)
        return;
    const Print print(payload->print);
    const Object object(payload->object);
    const SupportStyle style = support_style(object.config());
    if (style == first_style || style == second_style) {
        if (object_has_support_demand(object)) {
            const SupportParams params(print, object);
            GeneratedSupport support(context->plugin_storage);
            support.no_support_zones = slice_support_volumes(print, object, RAW_VOLUME_TYPE_NO_SUPPORT_ZONE, support.storage, context);
            generate(context, print, object, params, support);
            publish(context, object, params, support, orchestrator);
        }
        // Empty demand is a completed result for this style, not a missing producer.
        object.mutable_properties().get_or_add(SupportPipelineInfo::key).surfaces_generated=1;
    }
    progress.finish_run();
}

// Groups borrowed slabs by exact print Z and thickness, publishes object-owned auxiliary layers thinnest first per Z and advances
// interface ids once per Z. Storage owns temporary rings; rejected writes check cancellation then throw runtime_error, and
// geometry/allocation failures propagate with any completed publication intact.
void publish(const plugin_run_context *context, const Object &object, const SupportParams &params, GeneratedSupport &support, orchestrator_handle *orchestrator)
{
    Counters::add(Counters::totals.publication_calls);
    storage_handle *storage = support.storage;
    const ClipperContext clipper(storage);
    if (!support.no_support_zones.empty())
        remove_no_support_zones(object, params, clipper, support);
    GeneratorLayers sorted = support.top;
    sorted.insert(sorted.end(), support.bottom.begin(), support.bottom.end());
    sorted.insert(sorted.end(), support.base.begin(), support.base.end());
    sorted.insert(sorted.end(), support.interfaces.begin(), support.interfaces.end());
    std::sort(sorted.begin(), sorted.end(), [](const GeneratorLayer *a, const GeneratorLayer *b) { return *a < *b; });
    const int32_t base_tool = params.base_tool;
    const int32_t interface_tool = params.interface_tool;
    const bool top_enabled = params.top_published_interfaces_enabled;
    const bool bottom_enabled = params.bottom_published_interfaces_enabled;
    const extruder_index_t base_extruder = base_tool > 0 ? base_tool - 1 : EXTRUDER_ID_UNKNOWN;
    const extruder_index_t interface_extruder = interface_tool > 0 ? interface_tool - 1 : EXTRUDER_ID_UNKNOWN;
    // Contacts can join the base only when both roles share one extruder.
    const bool merge_regions = base_tool == interface_tool;
    const object_handle *object_ptr = object.handle();
    uint32_t interface_id = 0;
    std::map<coord_t, uint32_t> installed_ids;
    const PublicationTools tools{base_extruder, interface_extruder, top_enabled, bottom_enabled, merge_regions};
    for (size_t i = 0; i < sorted.size();) {
        const coord_t z = sorted[i]->print_z;
        const PublicationZ metadata = collect_publication_z(sorted, i, z);
        const bool contacts_only = metadata.top_count > 0 && metadata.top_count == metadata.interface_count;
        const uint32_t this_id = contacts_only && installed_ids.count(metadata.contact_bottom) ? installed_ids.at(metadata.contact_bottom) + 1 : interface_id;
        if (metadata.interface_count && !contacts_only) ++interface_id;
        // Equal-Z insertion precedes existing layers, so thinnest-first publication leaves the thickest layer first.
        for (std::vector<std::pair<size_t, size_t>>::const_reverse_iterator group = metadata.groups.rbegin(); group != metadata.groups.rend(); ++group)
            publish_group(context, object_ptr, orchestrator, storage, clipper, sorted, group->first, group->second,
                z, this_id, installed_ids, metadata, tools);
        i = metadata.end;
    }
}

namespace {
// The object layers are in print-Z order, so the layers overlapping a slab are one run found by binary search; slabs without
// overlapping zone keep their polygons unchanged.
void remove_no_support_zones(const Object &object, const SupportParams &params, const ClipperContext &clipper, GeneratedSupport &support)
{
    const std::vector<Layer> layers = object_layers(object);
    for (GeneratorLayers *slabs : {&support.top, &support.bottom, &support.base, &support.interfaces})
        for (GeneratorLayer *slab : *slabs) {
            if (slab->polygons.empty())
                continue;
            const coord_t lower = slab->print_z - slab->height;
            std::vector<Layer>::const_iterator layer = std::upper_bound(layers.begin(), layers.end(), lower,
                [](coord_t z, const Layer &candidate) { return z < candidate.print_z(); });
            std::optional<ClipperOperand> zones;
            for (; layer != layers.end() && layer->bottom_z() < slab->print_z; ++layer) {
                const size_t layer_idx = size_t(layer - layers.begin());
                if (layer_idx >= support.no_support_zones.size() || support.no_support_zones[layer_idx].empty())
                    continue;
                if (zones)
                    *zones += clipper(support.no_support_zones[layer_idx]);
                else
                    zones.emplace(clipper(support.no_support_zones[layer_idx]));
            }
            if (!zones)
                continue;
            slab->polygons = clipper_diff(clipper(slab->polygons), *zones).to_polygon_collection();
            slab->polygons.ensure_valid(params.resolution);
        }
}

// Walks borrowed sorted slabs from a valid begin index through the equal-Z run, collecting height ranges and metadata in order;
// returns owned index ranges without changing slabs, ignores empty rings for counts, and allocation failures propagate.
PublicationZ collect_publication_z(const GeneratorLayers &sorted, size_t begin, coord_t z)
{
    PublicationZ result;
    result.end = begin;
    for (size_t end = begin; end < sorted.size() && sorted[end]->print_z == z;) {
        const coord_t height = sorted[end]->height;
        size_t group_end = end + 1;
        while (group_end < sorted.size() && sorted[group_end]->print_z == z && sorted[group_end]->height == height) ++group_end;
        for (size_t j = end; j < group_end; ++j) {
            const GeneratorLayer &slab = *sorted[j];
            if (slab.polygons.empty()) continue;
            if (slab.type == LayerType::Base || slab.type == LayerType::Intermediate || slab.type == LayerType::Branch) result.has_base = true;
            else ++result.interface_count;
            result.min_height = result.min_height == 0 ? slab.height : std::min(result.min_height, slab.height);
            if (slab.type == LayerType::TopContact) { ++result.top_count; result.contact_bottom = slab.bottom_z; }
        }
        result.groups.emplace_back(end, group_end);
        result.end = group_end;
        end = group_end;
    }
    return result;
}

// Copies borrowed slab rings [begin,end) in order, creates an object-owned auxiliary layer and installs its slices/island surfaces;
// an empty group does nothing. Storage owns temporaries; rejected writes check cancellation then throw runtime_error, while
// geometry/allocation failures propagate with prior object writes intact. installed_ids records successful layer marking.
void publish_group(const plugin_run_context *context, const object_handle *object, orchestrator_handle *orchestrator,
    storage_handle *storage, const ClipperContext &clipper, const GeneratorLayers &sorted, size_t begin, size_t end,
    coord_t z, uint32_t interface_id, std::map<coord_t, uint32_t> &installed_ids, const PublicationZ &metadata,
    const PublicationTools &tools)
{
    StoredPolygonCollection all(storage);
    for (size_t j = begin; j < end; ++j) all.append_copy_from(sorted[j]->polygons);
    if (all.empty()) return;
    const coord_t height = sorted[begin]->height;
    layer_handle *layer = object_add_auxiliary_layer(object, height, z, z - height / 2);
    if (!layer) {
        throw_if_cancelled(context);
        throw std::runtime_error("Cannot create support layer");
    }
    LayerSupportProperty *property = static_cast<LayerSupportProperty *>(plugin_property_get_or_add_data_mutable(
        orchestrator, layer_get_properties_mutable(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT));
    if (!property) {
        throw_if_cancelled(context);
        throw std::runtime_error("Cannot mark support layer");
    }
    property->interface_id = interface_id;
    property->reserved = 0;
    installed_ids[z] = interface_id;
    /* The islands are what the legacy toolpaths unite: they fill the surfaces of one role from all the islands of the layer together, closing the
       sparse ones by +SCALED_EPSILON then -SCALED_EPSILON (LegacySupportPaths.cpp), so two pieces closer than 2 x SCALED_EPSILON, or touching at a vertex, form one fill
       area and a path may cross both. published_islands() therefore joins such pieces into one island and leaves every other piece exactly as the plain
       union made it, so that the geometry only changes where pieces are joined. The ring boxes are those of the published islands, and the joined island
       contains its pieces within the one-unit rounding of the closing, so every slab ring still lies in an island up to that rounding; the role surfaces are clipped by the island, hence stay inside the slabs. The slices
       keep the outside-to-inside order that the host asserts. The host copies them as the raw slice of region 0: with one region it takes them as they are,
       with several it adds a safety offset of 10 units to their union, which does not undo the joining; in both cases its ensure_valid() then simplifies
       the contours (resolution), so the identity with the plain union holds for the result of published_islands(), not for the published island slices. */
    const StoredExPolygonCollection slices = published_islands(storage, clipper, clipper_union(clipper(all)));
    expolygons_copy(layer_region_borrow_mutable_slices(layer_get_region_mutable(layer, 0)), slices.handle());
    layer_recompute_slices_and_islands_from_layer_regions(layer);
    layer_add_regions_to_islands(layer);
    if (Counters::enabled) {
        Counters::add(Counters::totals.groups);
        Counters::add(Counters::totals.slabs, end - begin);
        Counters::add(Counters::totals.islands, layer_count_island(layer));
        Counters::maximum(Counters::totals.max_islands_slabs, uint64_t(layer_count_island(layer)) * (end - begin));
    }
    const std::vector<std::vector<c_bounding_box>> slab_bounds = publication_slab_bounds(sorted, begin, end);
    for (uint32_t island_idx = 0; island_idx < layer_count_island(layer); ++island_idx) {
        layer_island_handle *island = layer_get_island_mutable(layer, island_idx);
        const ExPolygon island_slice(layer_island_get_slice(island));
        IslandSurfaces areas = collect_island_surfaces(storage, clipper, sorted, begin, end, slab_bounds, island_slice,
            tools.top_enabled, tools.bottom_enabled, tools.merge_regions, metadata.has_base, metadata.min_height);
        resolve_island_surfaces(clipper, areas);
        install_island_surfaces(context, storage, clipper, island, areas, tools.base_extruder, tools.interface_extruder);
    }
}

/* The reference: the closed islands (the closing by +SCALED_EPSILON then -SCALED_EPSILON of the union operand, with the offset parameters of the sparse
   fill closing) give the order and the joins. Each plain piece is assigned to the closed island that covers most of its area (assign_pieces()). A closed
   island with exactly one piece is replaced by that piece, whose geometry is then bit-identical to the plain union; one with several pieces is kept, since
   it is the joined island. When no closed island has several pieces, nothing is joined and the plain islands are returned as they are, in their own order.
   Otherwise a piece that no closed island overlaps (not expected) is kept after the others so that no geometry is lost. The closed islands are in the
   outside-to-inside order that the host asserts, and a replaced island and its piece are the same shape (the closing fills the gaps between pieces and its
   unions clean the vertices within SCALED_EPSILON of their neighbours, but leaves a piece with nothing near it where it was), so the order is preserved.

   The reference closes the whole layer, which is costly, so the plain islands are returned without it when the layer can be shown to join nothing. The
   plain pieces come from native_expolygons(), which drops the rings of small area after the union, but the closing sees the whole operand, so the
   unfiltered union (raw areas, each with its holes) is used too.
   - The offset grows an outline by SCALED_EPSILON along its edges and, with the mitered joins and the miter limit used below, by at most
     miter_limit * SCALED_EPSILON at a sharp corner. Two outlines can only meet if their boxes are at most 2 * miter_limit * SCALED_EPSILON apart on
     both axes (with one unit of margin for the rounding of the offsets). The boxes of the raw areas (contour and holes) and of the plain pieces are grouped in clusters of
     boxes that are closer than that, directly or through others. Every plain piece comes from raw areas whose box overlaps its own, so it shares their cluster.
   - The grown outlines of two clusters are disjoint. The per-ring offsets, the filter of small rings and the union of the whole layer therefore split
     into those of the clusters, and the reference restricted to the pieces of a cluster is the assignment of the pieces of the cluster to the closed islands of
     the closing of its raw areas (contour and holes) alone. That closing is the same geometry as the part of the closing of the layer, not necessarily
     the same vertices: the order of the rings in the operand differs, a cluster that has a single ring after the dilation takes another internal path of the
     offset, and the union that removes overlapping edges is decided on the whole layer. None of that changes the geometry of clean rings, and the decisions
     below use only counts of pieces and positive overlap areas.
   - A cluster with at most one plain piece is not checked: it cannot join pieces, and closed islands without piece are skipped.
   - A cluster with several pieces is checked: its raw areas, copied exactly into a normalized operand (like the union operand, so the offset treats the
     rings the same way), are closed and its pieces assigned as the reference does. When a closed island gets several pieces, the full reference runs.
   A layer with at most one piece joins nothing and is returned at once (the reference returns the same collection).
   Reads the borrowed union operand, returns an owned collection in storage, and geometry/allocation failures propagate. */
StoredExPolygonCollection published_islands(storage_handle *storage, const ClipperContext &clipper, const ClipperOperand &united)
{
    StoredExPolygonCollection plain = native_expolygons(united);
    if (plain.size() <= 1)
        return plain;
    const StoredExPolygonCollection raw = united.to_expolygon_collection();
    // The boxes of the contours, computed once for the pre-filters of the overlaps.
    const auto contour_boxes = [](const StoredExPolygonCollection &areas) {
        std::vector<c_bounding_box> boxes;
        boxes.reserve(areas.size());
        for (const ExPolygon area : areas)
            boxes.push_back(area.contour().bounding_box());
        return boxes;
    };
    const std::vector<c_bounding_box> plain_boxes = contour_boxes(plain);
    // The miter limit of the closings below, and the largest distance at which they can bridge two outlines.
    constexpr double closing_miter_limit = 3.;
    const coord_t reach = coord_t(std::ceil(closing_miter_limit * double(SCALED_EPSILON))) + 1;
    // One clustering of the raw areas (indices 0..raw.size()-1) and of the plain pieces (the following indices).
    std::vector<c_bounding_box> all_boxes = contour_boxes(raw);
    const uint32_t raw_count = raw.size();
    all_boxes.insert(all_boxes.end(), plain_boxes.begin(), plain_boxes.end());
    const std::vector<uint32_t> cluster_of = box_clusters(all_boxes, 2 * reach);
    std::vector<std::vector<uint32_t>> raw_areas_of(all_boxes.size()), pieces_of(all_boxes.size());
    for (uint32_t raw_idx = 0; raw_idx < raw_count; ++raw_idx)
        raw_areas_of[cluster_of[raw_idx]].push_back(raw_idx);
    for (uint32_t plain_idx = 0; plain_idx < plain.size(); ++plain_idx)
        pieces_of[cluster_of[raw_count + plain_idx]].push_back(plain_idx);
    bool joined_cluster = false;
    for (uint32_t cluster = 0; cluster < all_boxes.size() && !joined_cluster; ++cluster) {
        const std::vector<uint32_t> &pieces = pieces_of[cluster], &areas = raw_areas_of[cluster];
        if (pieces.size() < 2)
            continue;
        StoredExPolygonCollection cluster_raw(storage);
        expolygons_resize(cluster_raw.mutable_handle(), uint32_t(areas.size()));
        for (uint32_t idx = 0; idx < areas.size(); ++idx)
            expolygon_copy(expolygons_at(cluster_raw.mutable_handle(), idx), raw.at(areas[idx]).handle());
        const StoredExPolygonCollection cluster_closed = native_expolygons(clipper_offset2(clipper(cluster_raw), double(SCALED_EPSILON), -double(SCALED_EPSILON),
                                                                                           CLIPPER_JOIN_MITER, closing_miter_limit));
        const std::vector<std::vector<uint32_t>> cluster_members = assign_pieces(clipper, plain, plain_boxes, pieces, cluster_closed, contour_boxes(cluster_closed));
        for (const std::vector<uint32_t> &members : cluster_members)
            joined_cluster = joined_cluster || members.size() > 1;
    }
    if (!joined_cluster)
        return plain;

    // The reference on the whole layer.
    const StoredExPolygonCollection closed = native_expolygons(clipper_offset2(united, double(SCALED_EPSILON), -double(SCALED_EPSILON),
                                                                               CLIPPER_JOIN_MITER, closing_miter_limit));
    std::vector<uint32_t> all_pieces(plain.size());
    for (uint32_t plain_idx = 0; plain_idx < all_pieces.size(); ++plain_idx)
        all_pieces[plain_idx] = plain_idx;
    const std::vector<std::vector<uint32_t>> members = assign_pieces(clipper, plain, plain_boxes, all_pieces, closed, contour_boxes(closed));
    std::vector<bool> claimed(plain.size(), false);
    bool joined = false;
    for (const std::vector<uint32_t> &closed_members : members) {
        joined = joined || closed_members.size() > 1;
        for (const uint32_t plain_idx : closed_members)
            claimed[plain_idx] = true;
    }
    if (!joined)
        return plain;
    // The sources in publication order, borrowed from plain and closed.
    std::vector<ExPolygon> sources;
    for (uint32_t closed_idx = 0; closed_idx < closed.size(); ++closed_idx)
        if (members[closed_idx].size() == 1)
            sources.push_back(plain.at(members[closed_idx].front()));
        else if (members[closed_idx].size() > 1)
            sources.push_back(closed.at(closed_idx));
    for (uint32_t plain_idx = 0; plain_idx < plain.size(); ++plain_idx)
        if (!claimed[plain_idx])
            sources.push_back(plain.at(plain_idx));
    /* The collection is sized first and each slot is assigned by a plain copy of the source area. Inserting an area (expolygons_insert_copy) copies it
       then moves it into the vector, and moving a Polygon pops a near-closing last vertex, so a piece would not be bit-identical to the plain union. */
    StoredExPolygonCollection islands(storage);
    expolygons_resize(islands.mutable_handle(), uint32_t(sources.size()));
    for (uint32_t idx = 0; idx < sources.size(); ++idx)
        expolygon_copy(expolygons_at(islands.mutable_handle(), idx), sources[idx].handle());
    return islands;
}

/* Each listed plain piece goes to the closed island that covers most of its area (strictly more than none): the closing fills gaps and its unions clean the
   vertices within SCALED_EPSILON of their neighbours, so the closed outline is not the piece outline and a vertex test would be unreliable. The boxes are a
   pre-filter of the intersections. The result lists, for every closed island, the indices of its pieces in the order of `pieces`; the inputs are borrowed
   and unchanged and the failures of the geometry operations propagate. */
std::vector<std::vector<uint32_t>> assign_pieces(const ClipperContext &clipper, const StoredExPolygonCollection &plain,
    const std::vector<c_bounding_box> &plain_boxes, const std::vector<uint32_t> &pieces, const StoredExPolygonCollection &closed,
    const std::vector<c_bounding_box> &closed_boxes)
{
    std::vector<std::vector<uint32_t>> members(closed.size());
    for (const uint32_t plain_idx : pieces) {
        const ExPolygon piece = plain.at(plain_idx);
        const c_bounding_box &piece_box = plain_boxes[plain_idx];
        double best_area = 0.;
        uint32_t best = 0;
        for (uint32_t closed_idx = 0; closed_idx < closed.size(); ++closed_idx) {
            const c_bounding_box &box = closed_boxes[closed_idx];
            if (piece_box.max.x < box.min.x || box.max.x < piece_box.min.x || piece_box.max.y < box.min.y || box.max.y < piece_box.min.y)
                continue;
            const StoredExPolygonCollection common = clipper_intersection(clipper(piece), clipper(closed.at(closed_idx))).to_expolygon_collection();
            double common_area = 0.;
            for (const ExPolygon part : common)
                common_area += part.area();
            if (common_area > best_area) {
                best_area = common_area;
                best = closed_idx;
            }
        }
        if (best_area > 0.)
            members[best].push_back(plain_idx);
    }
    return members;
}

/* The boxes are visited by increasing minimum x. A box is compared with the following ones while their minimum x is within `gap` of its maximum x
   (the later ones are farther, since the minimum x only grows), and a pair that is also within `gap` on y is united in a union-find forest whose roots are
   the smallest index of each cluster, so the result does not depend on the sweep order. */
std::vector<uint32_t> box_clusters(const std::vector<c_bounding_box> &boxes, coord_t gap)
{
    std::vector<uint32_t> parent(boxes.size());
    for (uint32_t idx = 0; idx < parent.size(); ++idx)
        parent[idx] = idx;
    const auto find_root = [&parent](uint32_t idx) {
        while (parent[idx] != idx) {
            parent[idx] = parent[parent[idx]];
            idx = parent[idx];
        }
        return idx;
    };
    std::vector<uint32_t> order(boxes.size());
    for (uint32_t idx = 0; idx < order.size(); ++idx)
        order[idx] = idx;
    std::sort(order.begin(), order.end(), [&boxes](uint32_t a, uint32_t b) { return boxes[a].min.x < boxes[b].min.x; });
    for (size_t first = 0; first < order.size(); ++first) {
        const c_bounding_box &box = boxes[order[first]];
        for (size_t second = first + 1; second < order.size(); ++second) {
            const c_bounding_box &other = boxes[order[second]];
            if (other.min.x - box.max.x > gap)
                break;
            if (other.min.y - box.max.y <= gap && box.min.y - other.max.y <= gap) {
                const uint32_t root_a = find_root(order[first]), root_b = find_root(order[second]);
                parent[std::max(root_a, root_b)] = std::min(root_a, root_b);
            }
        }
    }
    std::vector<uint32_t> cluster_of(boxes.size());
    for (uint32_t idx = 0; idx < cluster_of.size(); ++idx)
        cluster_of[idx] = find_root(idx);
    return cluster_of;
}

// Reads borrowed slabs in [begin,end) without changing geometry, returning owned boxes in exactly their polygon order;
// slabs must remain unchanged while boxes are used, and view-access/allocation failures propagate.
std::vector<std::vector<c_bounding_box>> publication_slab_bounds(const GeneratorLayers &sorted, size_t begin, size_t end)
{
    std::vector<std::vector<c_bounding_box>> result;
    result.reserve(end - begin);
    for (size_t j = begin; j < end; ++j) {
        std::vector<c_bounding_box> bounds;
        bounds.reserve(sorted[j]->polygons.size());
        for (const Polygon polygon : sorted[j]->polygons)
            bounds.push_back(polygon.bounding_box());
        result.push_back(std::move(bounds));
    }
    return result;
}

// Intersects borrowed slabs [begin,end) with an island, preserving slab order and contact typing; returned role rings use storage,
// which outlives them. Bounds match the unchanged slabs/rings in [begin,end); strict-disjoint rings are omitted without offsets.
// Sorted slabs and the Z-wide base metadata are required; geometry/allocation failures propagate.
IslandSurfaces collect_island_surfaces(storage_handle *storage, const ClipperContext &clipper, const GeneratorLayers &sorted,
    size_t begin, size_t end, const std::vector<std::vector<c_bounding_box>> &slab_bounds, const ExPolygon &island_slice, bool top_enabled, bool bottom_enabled,
    bool merge_regions, bool z_has_base, coord_t z_min_height)
{
    // Clip every slab of this layer to the island and collect the role surfaces: upper
    // (top), lower (bottom, bridged or not), sparse base and branch bodies.
    StoredPolygonCollection top_surfaces(storage), top_contact_surfaces(storage), bridge_surfaces(storage), bottom_surfaces(storage),
        bottom_contact_surfaces(storage), sparse_surfaces(storage), branch_surfaces(storage);
    const c_bounding_box island_bounds = island_slice.contour().bounding_box();
    for (size_t j = begin; j < end; ++j) {
        const GeneratorLayer &slab = *sorted[j];
        const bool base = slab.type == LayerType::Base || slab.type == LayerType::Intermediate;
        const bool branch = slab.type == LayerType::Branch;
        const bool top = slab.type == LayerType::TopContact || slab.type == LayerType::TopInterface;
        const bool bridge = slab.type == LayerType::BottomContact && slab.bridging;
        /* A contact is typed as base when no interface layer is configured on its side
           and the native generator would have moved that contact area into the base
           layer: the user asked for no interface there, so the dense strip that would
           have joined the base is just base, and the sparse algorithm fills base and
           strip in one pass. The conditions are the native merge ones: the two support
           roles share one extruder, the contact does not bridge (the base never
           bridges), and the merged base layer of this print Z either has the contact's
           thickness, the smallest of that Z, or does not exist. Otherwise the contact
           keeps its own type and its own algorithm prints
           it with the base settings. This is decided here because each algorithm prints
           only the surfaces of its own type. */
        const bool mergeable_contact = (slab.type == LayerType::TopContact && !top_enabled) ||
            (slab.type == LayerType::BottomContact && !bottom_enabled);
        // The merged base layer of a print Z carries the smallest thickness of its slabs,
        // so it takes this contact only when the contact has exactly that thickness.
        const bool base_takes_contact = z_has_base && slab.height == z_min_height;
        const bool typed_as_base = mergeable_contact &&
            Internal::merged_into_base(merge_regions, base_takes_contact, z_has_base, slab.bridging);
        StoredPolygonCollection filtered(storage);
        const PolygonCollection nearby = island_slab_polygons(filtered, slab.polygons, slab_bounds[j - begin], island_bounds);
        Counters::add(Counters::totals.intersections);
        StoredPolygonCollection clipped = clipper_intersection(clipper(nearby), clipper(island_slice)).to_polygon_collection();
        if (branch) branch_surfaces.append_move_from(clipped);
        else if (base || typed_as_base) sparse_surfaces.append_move_from(clipped);
        else if (top) {
            // The areas of a top contact slab are also kept apart, to be marked as contact.
            if (slab.type == LayerType::TopContact)
                top_contact_surfaces.append_copy_from(clipped);
            top_surfaces.append_move_from(clipped);
        } else if (bridge) bridge_surfaces.append_move_from(clipped);
        else {
            if (slab.type == LayerType::BottomContact)
                bottom_contact_surfaces.append_copy_from(clipped);
            bottom_surfaces.append_move_from(clipped);
        }
    }
    return IslandSurfaces{std::move(top_surfaces), std::move(top_contact_surfaces), std::move(bridge_surfaces), std::move(bottom_surfaces),
        std::move(bottom_contact_surfaces), std::move(sparse_surfaces), std::move(branch_surfaces)};
}

// Copies borrowed rings into storage only when their cached boxes are not strictly disjoint from the island contour box;
// touching and enclosing boxes are kept, with no offset or ring reordering. Bounds must match polygons in count/order;
// empty/degenerate rings have no Clipper area, so omitting disjoint ones has no effect. Returns the borrowed source when all
// rings remain, otherwise a view of filtered, whose storage must outlive use; view-access/allocation failures propagate.
PolygonCollection island_slab_polygons(StoredPolygonCollection &filtered, const PolygonCollection &polygons,
    const std::vector<c_bounding_box> &bounds, const c_bounding_box &island_bounds)
{
    std::vector<uint32_t> retained;
    for (uint32_t index = 0; index < polygons.size(); ++index) {
        const c_bounding_box &box = bounds[index];
        if (box.max.x < island_bounds.min.x || island_bounds.max.x < box.min.x ||
            box.max.y < island_bounds.min.y || island_bounds.max.y < box.min.y)
            continue;
        retained.push_back(index);
    }
    if (retained.size() == polygons.size()) return polygons;
    // Size once before filling: later vector growth could move a populated native Polygon and remove its near-closing vertex.
    polygons_resize(filtered.mutable_handle(), uint32_t(retained.size()));
    for (uint32_t index = 0; index < retained.size(); ++index)
        multipoint_copy(polygon_as_multipoint(polygons_at(filtered.mutable_handle(), index)),
            polygon_as_multipoint_const(polygons.at(retained[index]).handle()));
    return filtered.readonly();
}

// Resolves caller-owned role rings in top/lower/branch/base order using the borrowed Clipper context; collections stay in their
// storage, counters record each operation, and geometry/allocation failures propagate after any completed replacements.
void resolve_island_surfaces(const ClipperContext &clipper, IslandSurfaces &areas)
{
    StoredPolygonCollection &top_surfaces = areas.top_surfaces, &bridge_surfaces = areas.bridge_surfaces,
        &bottom_surfaces = areas.bottom_surfaces, &sparse_surfaces = areas.sparse_surfaces, &branch_surfaces = areas.branch_surfaces,
        &top_contact_surfaces = areas.top_contact_surfaces, &bottom_contact_surfaces = areas.bottom_contact_surfaces;
    // One owner per area: the upper surfaces keep every area they share with a lower
    // one, the branch keeps the areas no interface covers, and the base keeps only the
    // areas that neither an interface nor a branch covers.
    Counters::add(Counters::totals.unions);
    const StoredPolygonCollection top_union = clipper_union(clipper(top_surfaces)).to_polygon_collection();
    bridge_surfaces = Internal::resolve_overlap(bridge_surfaces, top_union);
    bottom_surfaces = Internal::resolve_overlap(bottom_surfaces, top_union);
    Counters::add(Counters::totals.unions);
    const StoredPolygonCollection lower_union =
        clipper_union2(clipper(bridge_surfaces), clipper(bottom_surfaces)).to_polygon_collection();
    Counters::add(Counters::totals.unions);
    const StoredPolygonCollection covered = clipper_union2(clipper(top_union), clipper(lower_union)).to_polygon_collection();
    branch_surfaces = Internal::resolve_overlap(branch_surfaces, covered);
    sparse_surfaces = Internal::resolve_overlap(sparse_surfaces, covered);
    sparse_surfaces = Internal::resolve_overlap(sparse_surfaces, branch_surfaces);
    // Split each side into the contact areas and the interface areas, after the owners are known, so that the owners do not depend on the marking. A contact
    // keeps the area it shares with an interface of its side, and a lower contact loses what the upper surfaces print, as the lower surfaces do. Bridging
    // lower areas stay whole: they are contacts.
    if (!top_contact_surfaces.empty()) {
        Counters::add(Counters::totals.unions);
        top_contact_surfaces = clipper_union(clipper(top_contact_surfaces)).to_polygon_collection();
        top_surfaces = Internal::resolve_overlap(top_surfaces, top_contact_surfaces);
    }
    if (!bottom_contact_surfaces.empty()) {
        bottom_contact_surfaces = Internal::resolve_overlap(bottom_contact_surfaces, top_union);
        Counters::add(Counters::totals.unions);
        bottom_contact_surfaces = clipper_union(clipper(bottom_contact_surfaces)).to_polygon_collection();
        bottom_surfaces = Internal::resolve_overlap(bottom_surfaces, bottom_contact_surfaces);
    }
}

// Normalizes borrowed role rings in type order, then installs surfaces on island-owned work regions in extruder order;
// storage owns temporary geometry, failed writes check cancellation before throwing runtime_error, and geometry/allocation failures propagate.
void install_island_surfaces(const plugin_run_context *context, storage_handle *storage, const ClipperContext &clipper,
    layer_island_handle *island, const IslandSurfaces &areas, extruder_index_t base_extruder, extruder_index_t interface_extruder)
{
    const StoredPolygonCollection &top_surfaces = areas.top_surfaces, &bridge_surfaces = areas.bridge_surfaces,
        &bottom_surfaces = areas.bottom_surfaces, &sparse_surfaces = areas.sparse_surfaces, &branch_surfaces = areas.branch_surfaces,
        &top_contact_surfaces = areas.top_contact_surfaces, &bottom_contact_surfaces = areas.bottom_contact_surfaces;
    // Surfaces of one extruder, in the native order of their types.
    std::map<extruder_index_t, std::vector<std::pair<StoredExPolygonCollection, raw_surface_type>>> surfaces;
    // Borrows role rings, counts a nonempty union and stores its converted result by extruder/type; storage owns geometry and allocation/Clipper failures propagate.
    const auto add = [&](extruder_index_t extruder, const StoredPolygonCollection &area, raw_surface_type type) {
        if (!area.empty()) {
            // The explicit union and native_expolygons' normalization each call Clipper.
            Counters::add(Counters::totals.unions, 2);
            surfaces[extruder].emplace_back(native_expolygons(clipper_union(clipper(area))), type);
        }
    };
    // Contacts before the interfaces of their side; a bridging lower area is a contact.
    add(interface_extruder, top_contact_surfaces, srf_type().top().solid().support().contact().raw());
    add(interface_extruder, top_surfaces, srf_type().top().solid().support().raw());
    add(interface_extruder, bridge_surfaces, srf_type().bottom().solid().bridge().support().contact().raw());
    add(interface_extruder, bottom_contact_surfaces, srf_type().bottom().solid().support().contact().raw());
    add(interface_extruder, bottom_surfaces, srf_type().bottom().solid().support().raw());
    add(base_extruder, sparse_surfaces, srf_type().internal().sparse().support().raw());
    add(base_extruder, branch_surfaces, srf_type().internal().sparse().support().raw());
    for (const auto &[extruder, collection] : surfaces) {
        StoredSurfaceCollection stored(storage);
        for (const auto &[expolygons, type] : collection)
            for (const ExPolygon expolygon : expolygons) stored.append(expolygon, type);
        if (stored.empty()) continue;
        layer_region_island_handle *region = layer_island_get_or_create_region_island(island, nullptr, 0, extruder);
        if (!region || !layer_region_island_set_fill_surfaces(region, stored.mutable_handle())) {
            throw_if_cancelled(context);
            throw std::runtime_error("Cannot publish support surfaces");
        }
    }
}
} // namespace

// Decide whether the native generator would have moved a contact area into the base layer: the
// support regions must share one extruder, the contact must not bridge (the base never bridges, and
// the native merge also requires one bridging flag), and the merged base layer of that print Z must
// either have the contact's thickness or not exist at that print Z. That merged base layer is one
// layer per print Z, so its thickness is the smallest thickness among the slabs of the Z: a thicker
// base in another (Z, thickness) group of the same print Z does not prevent the contact joining it.
bool Internal::merged_into_base(bool merge, bool base_takes_contact, bool z_has_base, bool bridging)
{
    if (!merge || bridging) return false;
    return base_takes_contact || !z_has_base;
}

// Subtract the covering areas from the surfaces and leave the covering ones untouched; both sets
// stay owned by the caller, and an empty input short-circuits the Clipper difference.
StoredPolygonCollection Internal::resolve_overlap(const StoredPolygonCollection &surfaces, const StoredPolygonCollection &covering)
{
    if (surfaces.empty() || covering.empty()) return surfaces.readonly().clone(surfaces.storage());
    Counters::add(Counters::totals.differences);
    return clipper_diff(ClipperOperand(surfaces.storage(), surfaces.readonly()),
                        ClipperOperand(surfaces.storage(), covering.readonly())).to_polygon_collection();
}
} // namespace slic3r_api::Support::Common
