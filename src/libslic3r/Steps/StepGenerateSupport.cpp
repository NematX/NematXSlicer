///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "StepGenerateSupport.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"

#include <algorithm>
#include <cassert>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/log/trivial.hpp>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Api/plugin/cpp/properties/BridgeProperties.hpp"
#include "InfillPatternService.hpp"
#include "SupportPatternCallbacks.hpp"
// Host moved-bound diagnostics use the support counters module; it lives on the support side because support plugins use only the plugin API.
#include "libslic3r/Plugins/Support/Common/SupportCounters.hpp"
#include "StepRunner.hpp"

/*
Build support surfaces on separate layers so path generators can later turn
them into printed support material. STEP_SUPPORT runs active support families
once per object.
A family groups plugins that implement one kind of support generation.
Families publish support data through the data tree. Their callbacks may sample
filling patterns through InfillPatternService, an object that prepares and runs patterns.
register_selector_traits copies support group presentation defaults into
Orchestrator at host startup.

clear() resets surface/material completion without deleting geometry; a Slicing
replay owns auxiliary-layer cleanup. run_step() resets those flags before dispatch
so only a completed matching family can certify its surface result.

The first pattern request constructs the shared service. Other requests wait for
construction to finish, then only read it. A failed construction can be retried.
No service is built when the families never request a pattern. Building it runs
setup() for every available pattern and can fail on pattern errors or print
cancellation. std::call_once leaves its flag unset on failure so a later caller
can retry (see InfillPatternService.hpp).

C callbacks also serve Python callers, so no C++ exception may escape.
A failed callback clears its output and returns failure (0, or INVALID for a resolver).
contain_pattern_callback_failure(), shared with STEP_SUPPORT_TOOLPATH, requests
a stop for this Print. A pattern's own failure keeps that pattern as the known cause.
PluginStopped adds no diagnostic. Other failures record an error when possible.
The runner stops after the family returns; user cancellation is rethrown there.

   Print z of the support (host pass at the end of run_step). The tool ordering and the print plan need two distinct z of the print to be
   more than 4 * SCALED_EPSILON (512 units, 0.5 um, the minimum tolerance) apart, and the families choose the z of their layers without seeing each other (the
   layers of two objects, or the grid slabs against the layers of their own object). Once every family has published, align_support_z()
   moves the print z of the auxiliary layers (the supports).
   - Scope: one pass per object when the print is printed object by object (complete_objects or parallel_objects_step > 0; both options
     invalidate STEP_SUPPORT), where only the z of an object matter to its own plan; one pass for all objects otherwise.
   - Fixed values: the print z of the object layers (never moved). Movable values: the tops and the bottoms (print_z - height) of the
     auxiliary layers, so two contiguous layers stay contiguous. Rule (close_z_mapping()): a movable value within the tolerance
     (inclusive) of a fixed value takes the nearest one (the lower on a tie); the others are swept in ascending order, a value within
     that distance of the last kept one takes it, otherwise it is kept. A support z moves by at most the tolerance.
   - Tolerance: max(z_step / 2, 4 * SCALED_EPSILON (0.5 um)), the same rule as the tree family (Common::close_z_tolerance()), with z_step
     the option of the print (0 for none). The object print z are multiples of the Z step (the layer heights, the first layer height and the
     raft and support heights are rounded by check_z_step), so two distinct object z are at least a step apart; a support z is within half a
     step of a second object z only at the exact midpoint of two object z one step apart, where the tie rules decide (the tree takes the
     upper one, this pass the lower one; the choice of the tree is then fixed for this pass). Should the object layers be closer than that (a plugin choosing the layer heights), the tolerance is bounded by half
     the smallest distance between two object z of the pass, but never goes below 4 * SCALED_EPSILON, which the tool ordering requires.
   - The toolpath step recognises the contacts with a margin that covers the moves of the tree family and of this pass (the contact match
     in LegacySupportPaths.cpp, computed from the same Z step).
   - Precondition: the object layers are already more than the tolerance apart (two object layers of different objects closer than
     that are a native case, out of scope). Without it the guarantee does not hold.
   - After the move the layers of each object are sorted again by print_z, the thickest first at an equal print_z, which is the order of
     the publication (sort_support_layers()). Layer::id and the interface_id of LayerSupportProperty are creation identifiers and are not
     renumbered: interface_id drives the alternation of the angle of the interface layers, so it follows the creation order, not the
     z order, which is an accepted limit.
   - A layer whose new height would not be above the minimum height of a layer (ApiInternal::LayerAccess::min_scaled_height, the limit of
     the Layer constructor) makes the step throw std::runtime_error; the families publish layers far thicker than 1 um.

   Selected direct calls:

       run_step()
           |-- Steps::Detail::run_object_step_plugins() [join family object runs]
           `-- align_support_z() [after every family has published]
               |-- Detail::close_z_mapping()
               |-- ApiInternal::LayerAccess::set_z()
               `-- Detail::sort_support_layers()

   Pattern callbacks are invoked by a support family through its payload; they
   are not direct calls from run_step(). The shared lazy service outlives those
   joined object runs. */

namespace Slic3r::Steps::StepGenerateSupport {
// Copy support group presentation defaults into the borrowed host without changing print data or provider ownership;
// registration conflicts assert and allocation failures propagate.
void register_selector_traits(Orchestrator &orchestrator);

void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits traits;
    traits.category = RAW_OPTION_CATEGORY_SUPPORT;
    traits.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    traits.invalidates_step = STEP_SUPPORT;
    traits.show_with_one_provider = false;
    const int32_t registered = orchestrator.register_builtin_step_selector(STEP_SUPPORT, std::move(traits));
    assert(registered == 1);
    (void)registered;
}

namespace {

// The pattern service of one step run, built on first use.
class LazyPatternService
{
public:
    // The service is built with the object count as the run count advertised to the pattern progress.
    LazyPatternService(Orchestrator &orchestrator, Print &print) : m_orchestrator(orchestrator), m_print(print) {}
    LazyPatternService(const LazyPatternService &) = delete;
    LazyPatternService &operator=(const LazyPatternService &) = delete;

    // The service, built by the first caller; other callers wait until it is complete. Throws when the
    // construction fails, and the next caller builds it again.
    // Patterns needing the STEP_INFILL initialization pass are refused because this step has none.
    InfillPatternService &get()
    {
        std::call_once(m_built, [this] {
            m_service.emplace(m_orchestrator, m_print, m_print.objects().size(),
                              InfillPatternService::SetupPatterns::Refuse);
        });
        return *m_service;
    }

    // The orchestrator that receives the failures contained by the callbacks.
    Orchestrator &orchestrator() { return m_orchestrator; }

    // Reports lazy access as available without construction, mutation, ownership changes or failure.
    bool available() const noexcept { return true; }

    // Read this service's Print cancellation and execution-stop flags; ignore the payload Print.
    // A stopped call returns failure without constructing the service or adding a message.
    bool stop_requested(const print_handle *) const noexcept { return m_orchestrator.is_plugin_cancelled(&m_print) || m_print.canceled(); }
private:
    Orchestrator &m_orchestrator;
    Print &m_print;
    std::once_flag m_built;
    std::optional<InfillPatternService> m_service;
};

// Bind the shared noexcept relay to this step's lazy service policy.
using PatternCallbacks = SupportPatternCallbacks<run_ctx_generate_support, LazyPatternService, STEP_SUPPORT>;

} // namespace

namespace Detail {
// Map movable heights to nearby retained heights. Inputs use scaled units (1 unit = 1 nm);
// they may be unsorted or contain duplicates. Fixed heights are never changed.
// First, use the nearest fixed height within tolerance, inclusive. A tie chooses the lower height.
// Then scan the remaining movable heights in ascending order. Keep the first; map each next
// height within tolerance of the last kept height to it. Otherwise keep the new height.
// With fixed heights more than tolerance apart, distinct results remain more than tolerance apart.
// No height moves by more than tolerance. Tool ordering requires tolerance >= 4 * SCALED_EPSILON.
CloseZMapping close_z_mapping(std::vector<coord_t> fixed, std::vector<coord_t> movable, coord_t tolerance)
{
    assert(tolerance >= slic3r_api::Support::Common::min_close_z_tolerance);
    std::sort(fixed.begin(), fixed.end());
    fixed.erase(std::unique(fixed.begin(), fixed.end()), fixed.end());
    std::sort(movable.begin(), movable.end());
    movable.erase(std::unique(movable.begin(), movable.end()), movable.end());
    CloseZMapping mapping;
    coord_t kept = 0;
    bool    has_kept = false;
    for (const coord_t z : movable) {
        const auto above = std::lower_bound(fixed.begin(), fixed.end(), z);
        coord_t    nearest = z, distance = tolerance + 1;
        if (above != fixed.end() && *above - z < distance) {
            nearest  = *above;
            distance = *above - z;
        }
        if (above != fixed.begin() && z - *(above - 1) <= distance) { // the lower one wins a tie
            nearest  = *(above - 1);
            distance = z - *(above - 1);
        }
        if (distance <= tolerance) {
            if (nearest != z)
                mapping.emplace_back(z, nearest);
            continue;
        }
        if (has_kept && z - kept <= tolerance) {
            mapping.emplace_back(z, kept);
        } else {
            kept     = z;
            has_kept = true;
        }
    }
    return mapping; // ascending in the source, since movable is
}

// Binary search of z among the sources of the mapping, which close_z_mapping() returns in ascending order.
coord_t close_z_target(const CloseZMapping &mapping, coord_t z)
{
    const auto it = std::lower_bound(mapping.begin(), mapping.end(), z, [](const std::pair<coord_t, coord_t> &entry, coord_t value) { return entry.first < value; });
    return it != mapping.end() && it->first == z ? it->second : z;
}
// The order of the publication: print_z ascending, the thickest first at an equal print_z. The sort is stable, so equal layers keep their order.
void sort_support_layers(std::vector<std::unique_ptr<Layer>> &layers)
{
    std::stable_sort(layers.begin(), layers.end(), [](const std::unique_ptr<Layer> &a, const std::unique_ptr<Layer> &b) {
        return a->scaled_print_z() < b->scaled_print_z() || (a->scaled_print_z() == b->scaled_print_z() && a->scaled_height() > b->scaled_height());
    });
}
} // namespace Detail

namespace {
// Align support layers of the supplied objects using one common height mapping.
// Object-layer heights are fixed. Support-layer tops and bottoms may move together,
// preserving shared boundaries between adjacent layers. Count each moved bound.
// The caller passes one object at a time for complete_objects or parallel_objects_step > 0;
// otherwise it passes all objects. Both settings invalidate this support stage.
// z_step is the printer's Z increment in mm, or 0. Common::close_z_tolerance() starts
// from max(z_step / 2, 4 * SCALED_EPSILON), about 0.5 um at the minimum (512 scaled units).
// It caps tolerance at half the nearest distinct object-height gap, but never below that minimum.
// Object layers must already be more than tolerance apart. check_z_step normally puts
// object, raft and support layer heights on Z increments. Closer heights from another
// producer must still meet the separation precondition; otherwise the guarantee does not hold.
// At a half-step tie, this mapping chooses the lower height; the tree family chooses the upper.
// LegacySupportPaths.cpp uses a contact margin covering both families' height adjustments.
// Sort updated layers by print_z, thickest first on a tie. Layer IDs keep their creation order;
// the interface-angle alternation therefore follows creation order, not the new height order.
// A height at or below Layer's minimum raises std::runtime_error; normal family layers exceed 1 um.
// Objects are borrowed; their support layers are modified. Allocation errors also propagate.
void align_support_z(const std::vector<PrintObject *> &objects, double z_step)
{
    std::vector<coord_t> fixed, movable;
    for (const PrintObject *object : objects) {
        for (const Layer &layer : object->layers())
            fixed.push_back(layer.scaled_print_z());
        for (const Layer &layer : object->auxiliary_layers()) {
            movable.push_back(layer.scaled_print_z());
            movable.push_back(layer.scaled_bottom_z());
        }
    }
    const coord_t tolerance = slic3r_api::Support::Common::close_z_tolerance(z_step, fixed);
    const Detail::CloseZMapping mapping = Detail::close_z_mapping(std::move(fixed), std::move(movable), tolerance);
    if (mapping.empty())
        return;
    for (PrintObject *object : objects) {
        for (const LayerUPtr &layer : object->mutable_auxiliary_layers()) {
            const coord_t print_z = Detail::close_z_target(mapping, layer->scaled_print_z());
            const coord_t bottom  = Detail::close_z_target(mapping, layer->scaled_bottom_z());
            if (print_z == layer->scaled_print_z() && bottom == layer->scaled_bottom_z())
                continue;
            if (print_z - bottom <= ApiInternal::LayerAccess::min_scaled_height)
                throw std::runtime_error("A support layer of the object '" + object->model_object()->name + "' at z = " +
                    std::to_string(unscaled(layer->scaled_print_z())) + " mm would be only " + std::to_string(unscaled(print_z - bottom)) +
                    " mm thick after its print z were kept " + std::to_string(unscaled(tolerance)) + " mm apart from the others.");
            if (slic3r_api::Support::Common::Counters::enabled)
                slic3r_api::Support::Common::Counters::add(slic3r_api::Support::Common::Counters::totals.host_bounds_moved,
                    uint64_t(print_z != layer->scaled_print_z()) + uint64_t(bottom != layer->scaled_bottom_z()));
            ApiInternal::LayerAccess::set_z(*layer, print_z, print_z - bottom);
        }
        Detail::sort_support_layers(object->mutable_auxiliary_layers());
    }
}
} // namespace

void clear(Print &print)
{
    for (PrintObject &object : print.objects())
        if (slic3r_api::SupportPipelineInfo *state=object.get_property<slic3r_api::SupportPipelineInfo>()) {
            state->surfaces_generated=0;
            state->material_generated=0;
        }
}
void prepare(Print &) {}

bool validate_pre(const Print &, std::string *)
{
    return true;
}

bool validate_post(const Print &, std::string *)
{
    return true;
}

// Call the active support families in priority order for each object; they publish auxiliary surfaces and runner errors stop the step.
// Then align_support_z() keeps the support z apart from the other z of the print: per object when the print is printed object by object,
// otherwise once for all objects.
// Borrows the run inputs, owns the lazy pattern service until joined object runs finish, and propagates runner/cancellation/allocation failures; families publish surfaces, which the islands of the object's auxiliary layers own after publication.
void run_step(Orchestrator &orchestrator, Print &print)
{
    // A rerun must not inherit completion from the preceding family invocation.
    clear(print);
    // Lives for the whole step; only built when a family calls a pattern callback.
    LazyPatternService patterns(orchestrator, print);
    Steps::Detail::run_object_step_plugins(orchestrator,
                                   print,
                                   STEP_SUPPORT,
                                   &print.full_print_config(),
                                   print.objects().size(),
                                    // Builds a borrowed payload for a valid object index; the joined run owns its lifetime, and no geometry is copied.
                                    [&print, &patterns](const size_t object_idx) {
        run_ctx_generate_support context = {};
        context.print = reinterpret_cast<const print_handle *>(&print);
        context.object = reinterpret_cast<const object_handle *>(&print.object(object_idx));
        context.host_context = &patterns;
        context.resolve_pattern_id = &PatternCallbacks::resolve;
        context.generate_pattern = &PatternCallbacks::generate;
        return context;
    });
    print.throw_if_canceled();
    std::vector<PrintObject *> objects;
    for (size_t idx = 0; idx < print.objects().size(); ++idx)
        objects.push_back(&print.object(idx));
    // Printed object by object: the same criterion as ByLayerPlanBuilder::use_object_plan (Plugins/Ordering/ByLayerPlanBuilder.cpp, local to that file)
    // and Print.cpp:1565. Both options invalidate STEP_SUPPORT (complete_objects through STEP_SKIRT_BRIM, which replays from the same checkpoint),
    // so the scope of the pass is always rebuilt when one of them changes.
    // The Z step is read like the support plugins do (SupportParams: the z_step option of the print config, in mm).
    const double z_step = print.config().z_step.value;
    if (print.config().complete_objects.value || print.config().parallel_objects_step.value > 0)
        for (PrintObject *object : objects)
            align_support_z({ object }, z_step);
    else
        align_support_z(objects, z_step);
}

} // namespace Slic3r::Steps::StepGenerateSupport
