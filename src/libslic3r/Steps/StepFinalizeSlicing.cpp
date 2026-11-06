///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Let plugins make their last changes to sliced extrusion paths, and reject
   later edits that would invalidate a bridge's volume correction.

   These plugins run at STEP_FINALIZE_SLICING, after model and support paths have
   been calculated but before ordering builds the printing plan. They can use the
   support paths to adjust infill and replace all affected trees of one object
   together. Replacement checks every copied tree before changing any original.

   BridgeVolumeInfo records the incoming and adjusted flow on each bridge piece.
   This file checks those records against the actual paths and remembers a hash
   of the object's model/support paths and bridge surfaces. If a later plugin
   changes that data or removes a record, the step fails rather than exporting
   a correction based on paths that are no longer present.

   A certificate here is a fingerprint of the correction records and the data
   they depend on, not independent proof that a bridge can be printed physically.
   run_step() compares those fingerprints before and after each joined plugin run.

   Selected direct calls, not a timeline:

       run_step()
           |-- object_certificates() [before dispatch and after each provider]
           |   `-- collect_certificates() [validate records and hash path data]
           |       `-- collect_certificates() [recursive children]
           `-- Detail::run_object_step_plugin() [joined object dispatch]

       validate_post()
           `-- object_certificates()

   A dispatched plugin can call payload.replace_infill_buckets(), whose host
   implementation is replace_infill(). This optional callback validates storage
   ownership and every destination/donor before its nonthrowing content swaps;
   rejection or cancellation leaves both sides unchanged and no exception crosses C.
   It is not a direct call from run_step().

   clear() and prepare() retain no state; validate_pre() accepts the incoming tree.

   Plugins run in sequence and their object runs may be concurrent. Run contexts
   borrow the print, object and plugin storage until all object runs return.
   Replacing tree contents keeps the original roots' identities. The copied roots
   then own the old contents. A failed later plugin does not undo earlier replacements.
*/
#include "StepFinalizeSlicing.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_finalize_slicing.h"
#include "libslic3r/Api/plugin/cpp/properties/BridgeProperties.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Plugins/Infill/BridgeVolume.hpp"
#include "libslic3r/Plugins/Infill/BridgeClearance.hpp"
#include "StepRunner.hpp"

namespace Slic3r::Steps::StepFinalizeSlicing {
namespace {
// One joined object's callback state; pointers to these records never escape its run.
struct FinalHostContext {
    Orchestrator &orchestrator;
    Print &print;
    PrintObject &object;
    PluginStorage &storage;
};
// Borrow the payload/array for one synchronous publication. Validate unique live
// destinations and storage-owned donors, then return 1 after nonthrowing swaps.
// Refusal/cancellation returns 0 unchanged; caught failures request stop, never cross C.
int32_t replace_infill(const run_ctx_finalize_slicing *ctx, storage_handle *storage,
    const raw_final_infill_replacement *entries, uint32_t count) noexcept;
// Check paths carrying BridgeVolumeInfo against their recorded flow and geometry,
// updating signature/count. Return false with an optional error on a mismatch.
bool collect_certificates(const ExtrusionEntity &entity, const ExtrusionAttributes *inherited,
    coord_t inherited_z, uint64_t &signature, size_t &count, std::string *error);
// Remember the geometry used by each object's bridge correction in out. Objects
// without correction records need no entry. Bad records return false; allocations throw.
bool object_certificates(const Print &print, std::map<const PrintObject *, uint64_t> &out, std::string *error);

int32_t replace_infill(const run_ctx_finalize_slicing *ctx, storage_handle *storage,
    const raw_final_infill_replacement *entries, uint32_t count) noexcept
{
    if (ctx == nullptr || ctx->host_context == nullptr || (count > 0 && entries == nullptr))
        return 0;
    FinalHostContext &host = *static_cast<FinalHostContext *>(ctx->host_context);
    try {
        if (reinterpret_cast<PluginStorage *>(storage) != &host.storage)
            return 0;
        std::set<const ExtrusionEntity *> allowed, destinations, donors;
        const std::vector<const ExtrusionEntity *> stored = host.storage.stored_extrusions();
        const std::set<const ExtrusionEntity *> owned(stored.begin(), stored.end());
        // Only existing object infill roots may receive content; support and
        // perimeter roots are inputs to the finalizer, not publication targets.
        for (Layer &layer : host.object.layers())
            for (LayerSliceIsland &island : layer.islands())
                for (LayerRegionIsland &region : island.regions_islands())
                    for (const auto &[role, root] : region.extrusion_buckets())
                        if (role.is_infill() || role == LayerRegionIsland::GAP_FILLS)
                            allowed.insert(&root);
        std::vector<std::pair<ExtrusionEntity *, ExtrusionEntity *>> swaps;
        swaps.reserve(count);
        for (uint32_t idx = 0; idx < count; ++idx) {
            const ExtrusionEntity *destination = reinterpret_cast<const ExtrusionEntity *>(entries[idx].destination);
            ExtrusionEntity *donor = reinterpret_cast<ExtrusionEntity *>(entries[idx].prepared);
            if (allowed.count(destination) == 0 || donor == nullptr || owned.count(donor) == 0 ||
                !destinations.insert(destination).second || !donors.insert(donor).second || allowed.count(donor) != 0)
                return 0;
            swaps.emplace_back(const_cast<ExtrusionEntity *>(destination), donor);
        }
        host.print.throw_if_canceled();
        if (host.orchestrator.is_plugin_cancelled())
            return 0;
        // Allocation, validation and cancellation are complete: this loop cannot
        // expose a partially exchanged batch through a failed operation.
        for (const std::pair<ExtrusionEntity *, ExtrusionEntity *> &entry : swaps)
            entry.first->swap_content(*entry.second);
        return 1;
    } catch (...) {
        Detail::contain_callback_failure(host.orchestrator, STEP_FINALIZE_SLICING);
        return 0;
    }
}

bool collect_certificates(const ExtrusionEntity &entity, const ExtrusionAttributes *inherited,
    coord_t inherited_z, uint64_t &signature, size_t &count, std::string *error)
{
    const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
    if (attributes == nullptr)
        attributes = inherited;
    const ExtrusionPropertyZOffset *offset = entity.get_property<ExtrusionPropertyZOffset>();
    const coord_t z_offset = offset == nullptr ? inherited_z : offset->z_offset;
    if (entity.has_polyline()) {
        signature = (signature ^ slic3r_api::BridgeVolume::path_signature(
            reinterpret_cast<const extrusion_entity_handle *>(&entity))) * 1099511628211ull;
        signature = (signature ^ uint64_t(z_offset)) * 1099511628211ull;
        if (attributes != nullptr) {
            for (const double value : {attributes->mm3_per_mm,double(attributes->width),double(attributes->height)}) {
                uint64_t bits=0;
                std::memcpy(&bits,&value,sizeof(bits));
                signature = (signature ^ bits) * 1099511628211ull;
            }
            signature = (signature ^ attributes->role) * 1099511628211ull;
        }
    }
    if (const slic3r_api::BridgeVolumeInfo *stamp = entity.get_property<slic3r_api::BridgeVolumeInfo>()) {
        const uint64_t geometry = slic3r_api::BridgeVolume::path_signature(
            reinterpret_cast<const extrusion_entity_handle *>(&entity));
        const bool known = stamp->confidence == BRIDGE_CONFIDENCE_CERTIFIED || stamp->confidence == BRIDGE_CONFIDENCE_FLOOR_LIMITED;
        const bool state_valid = known ? stamp->correction_state == BRIDGE_CORRECTION_APPLIED :
            stamp->confidence == BRIDGE_CONFIDENCE_UNKNOWN && stamp->correction_state == BRIDGE_CORRECTION_KEPT_NOMINAL_UNKNOWN;
        const bool valid = entity.has_polyline() && entity.polyline_ref().size() >= 2 && attributes != nullptr && state_valid && z_offset == 0 &&
            std::isfinite(attributes->mm3_per_mm) && attributes->mm3_per_mm > 0. &&
            attributes->mm3_per_mm == stamp->effective_area_mm2 && attributes->height == stamp->effective_height_mm &&
            attributes->width == stamp->nominal.width && attributes->role == stamp->nominal.role &&
            attributes->mm3_per_mm <= stamp->nominal.mm3_per_mm + 1e-10 &&
            stamp->geometry_signature == geometry;
        // A known gap uses a flattened section with round ends; Unknown keeps
        // the incoming shape. Only Certified must fit within the available area.
        const bool shape_valid = !known || (attributes != nullptr && std::abs(attributes->mm3_per_mm -
            slic3r_api::BridgeClearance::area(attributes->width, attributes->height)) < 1e-7);
        const bool cap_valid = stamp->confidence != BRIDGE_CONFIDENCE_CERTIFIED ||
            stamp->effective_area_mm2 <= stamp->area_cap_mm2 + std::max(1e-12,stamp->nominal.mm3_per_mm*1e-6);
        if (!valid || !shape_valid || !cap_valid) {
            if (error != nullptr)
                *error = "A finalized bridge no longer matches its certified path, flow or clearance.";
            return false;
        }
        signature = (signature ^ geometry) * 1099511628211ull;
        signature = (signature ^ stamp->signature) * 1099511628211ull;
        signature = (signature ^ stamp->logical_key) * 1099511628211ull;
        signature = (signature ^ stamp->confidence) * 1099511628211ull;
        signature = (signature ^ stamp->correction_state) * 1099511628211ull;
        for (const double value : {stamp->area_cap_mm2,stamp->available_height_mm,stamp->nominal.mm3_per_mm,
                double(stamp->nominal.width),double(stamp->nominal.height)}) {
            uint64_t bits=0;
            std::memcpy(&bits,&value,sizeof(bits));
            signature = (signature ^ bits) * 1099511628211ull;
        }
        ++count;
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        if (child && !collect_certificates(*child, attributes, z_offset, signature, count, error))
            return false;
    return true;
}

bool object_certificates(const Print &print, std::map<const PrintObject *, uint64_t> &out, std::string *error)
{
    for (const PrintObject &object : print.objects()) {
        uint64_t signature = 1469598103934665603ull;
        size_t count = 0;
        std::vector<const Layer *> layers;
        for (const Layer &layer : object.layers()) layers.push_back(&layer);
        for (const Layer &layer : object.auxiliary_layers()) layers.push_back(&layer);
        for (const Layer *layer : layers) {
            signature = (signature ^ uint64_t(layer->scaled_print_z())) * 1099511628211ull;
            signature = (signature ^ uint64_t(layer->scaled_height())) * 1099511628211ull;
            for (const LayerSliceIsland &island : layer->islands())
                for (const LayerRegionIsland &region : island.regions_islands()) {
                    signature = (signature ^ uint64_t(region.extruder_id())) * 1099511628211ull;
                    for (const Surface &surface : region.fill_surfaces())
                        if (const slic3r_api::BridgeSurfaceInfo *info=surface.get_property<slic3r_api::BridgeSurfaceInfo>()) {
                            uint64_t angle=0;
                            std::memcpy(&angle,&surface.bridge_angle,sizeof(angle));
                            for (const uint64_t value : {angle,info->logical_key,info->policy_signature,uint64_t(info->schema_version),
                                    uint64_t(info->core_rule),uint64_t(surface.surface_type)})
                                signature = (signature ^ value) * 1099511628211ull;
                            std::vector<const Polygon *> rings{&surface.expolygon.contour};
                            for (const Polygon &hole : surface.expolygon.holes) rings.push_back(&hole);
                            for (const Polygon *ring : rings) {
                                signature = (signature ^ ring->points.size()) * 1099511628211ull;
                                for (const Point &point : ring->points) {
                                    signature = (signature ^ uint64_t(point.x())) * 1099511628211ull;
                                    signature = (signature ^ uint64_t(point.y())) * 1099511628211ull;
                                }
                            }
                        }
                    for (const auto &[role, root] : region.extrusion_buckets()) {
                        signature = (signature ^ uint64_t(role())) * 1099511628211ull;
                        if (!collect_certificates(root, nullptr, 0, signature, count, error))
                            return false;
                    }
                }
        }
        if (count > 0)
            out.emplace(&object, (signature ^ count) * 1099511628211ull);
    }
    return true;
}
}

void clear(Print &) {}
void prepare(Print &) {}
bool validate_pre(const Print &, std::string *) { return true; }
bool validate_post(const Print &print, std::string *error)
{
    std::map<const PrintObject *, uint64_t> certificates;
    return object_certificates(print, certificates, error);
}

void run_step(Orchestrator &orchestrator, Print &print)
{
    std::map<const PrintObject *, uint64_t> certificates;
    std::string error;
    if (!object_certificates(print, certificates, &error))
        throw std::runtime_error(error);
    for (Plugin *plugin : selected_or_active_plugins_for_step(orchestrator, STEP_FINALIZE_SLICING, &print.full_print_config())) {
        // Finish building stable callback records before any object receives
        // their addresses; the dispatcher joins all uses before destruction.
        std::vector<FinalHostContext> contexts;
        contexts.reserve(print.objects().size());
        for (PrintObject &object : print.objects())
            contexts.push_back({orchestrator, print, object, orchestrator.plugin_storage()[plugin]});
        Detail::run_object_step_plugin(orchestrator, print, STEP_FINALIZE_SLICING, *plugin,
            print.objects().size(), [&print, &contexts](size_t idx) {
                run_ctx_finalize_slicing payload = {};
                payload.print = reinterpret_cast<const print_handle *>(&print);
                payload.object = reinterpret_cast<const object_handle *>(&print.object(idx));
                payload.host_context = &contexts[idx];
                const slic3r_api::SupportPipelineInfo *state = print.object(idx).get_property<slic3r_api::SupportPipelineInfo>();
                payload.support_demand_computed = state == nullptr ? 0 : state->demand_computed;
                payload.support_material_generated = state == nullptr ? 0 : state->material_generated;
                payload.get_requested_support_areas = &layer_island_get_support_demand;
                payload.replace_infill_buckets = &replace_infill;
                return payload;
            });
        print.pipeline_progress().complete_plugin(STEP_FINALIZE_SLICING, plugin->get_id());
        std::map<const PrintObject *, uint64_t> after;
        if (!object_certificates(print, after, &error))
            throw std::runtime_error(error);
        // Include objects that lost all stamps: absence from after must not let
        // a later provider silently discard an earlier provider's certificate.
        for (const auto &[object, signature] : certificates) {
            const std::map<const PrintObject *, uint64_t>::const_iterator found = after.find(object);
            if (found == after.end() || found->second != signature)
                throw std::runtime_error("A finalization plugin removed or changed a finalized bridge certificate.");
        }
        certificates.swap(after);
    }
}
}
