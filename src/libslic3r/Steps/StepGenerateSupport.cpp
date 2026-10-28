///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "StepGenerateSupport.hpp"

#include <algorithm>
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
#include "InfillPatternService.hpp"
#include "SupportPatternCallbacks.hpp"
#include "StepRunner.hpp"

/* STEP_SUPPORT calls every active support family once per object. A family may
   sample INFILL_PATTERN plugins through the payload callbacks. They forward to
   one InfillPatternService shared by every object run, which may be concurrent.

   Lazy service: building the service runs the setup() of every pattern that
   this step may call, which costs time, so the first callback builds it
   (std::call_once: the construction completes before any concurrent call reads
   the service, which is read-only afterwards, see InfillPatternService.hpp). A
   run whose families never call a pattern does not build it. A construction
   that throws (a pattern setup() reports an error, or the print is cancelled)
   leaves the flag unset, so the next caller builds it again.

   Errors: the callbacks are called through the C ABI, by C++ families but also
   by Python families through ctypes, which cannot catch a C++ exception. They
   therefore catch everything: the failing call empties the caller's output and
   returns 0 (INVALID for the resolver). The error is recorded as a plugin error
   message and the plugin cancellation is requested as a host failure (a pattern
   that reports an error has itself requested it as the failing plugin). A
   PluginStopped adds no message: a known cause has its own message, an Unknown
   cause has none and the final step error says a stop was requested. The family keeps running until it returns; the standard end-of-phase
   check of the runner (throw_if_plugin_stopped) then stops the step. The
   containment is shared with STEP_SUPPORT_TOOLPATH (contain_pattern_callback_failure(),
   see InfillPatternService.hpp). A cancellation of the print is not recorded: the same check rethrows it. The
   output is not published by this step (the families publish through the data tree); the callbacks only read the patterns.

   Print z of the support (host pass at the end of run_step). The tool ordering and the print plan need two distinct z of the print to be
   more than 4 * SCALED_EPSILON (512 units, 0.5 um) apart, and the families choose the z of their layers without seeing each other (the
   layers of two objects, or the grid slabs against the layers of their own object). Once every family has published, align_support_z()
   moves the print z of the auxiliary layers (the supports).
   - Scope: one pass per object when the print is printed object by object (complete_objects or parallel_objects_step > 0; both options
     invalidate STEP_SUPPORT), where only the z of an object matter to its own plan; one pass for all objects otherwise.
   - Fixed values: the print z of the object layers (never moved). Movable values: the tops and the bottoms (print_z - height) of the
     auxiliary layers, so two contiguous layers stay contiguous. Rule (close_z_mapping()): a movable value within 4 * SCALED_EPSILON
     (inclusive) of a fixed value takes the nearest one (the lower on a tie); the others are swept in ascending order, a value within
     that distance of the last kept one takes it, otherwise it is kept. A support z moves by at most 4 * SCALED_EPSILON (0.5 um).
   - The toolpath step recognises the contacts with a margin that covers this move (CONTACT_MATCH_TOLERANCE in LegacySupportPaths.cpp).
   - Precondition: the object layers are already more than 4 * SCALED_EPSILON apart (two object layers of different objects closer than
     that are a native case, out of scope). Without it the guarantee does not hold.
   - After the move the layers of each object are sorted again by print_z, the thickest first at an equal print_z, which is the order of
     the publication (sort_support_layers()). Layer::id and the interface_id of LayerSupportProperty are creation identifiers and are not
     renumbered: interface_id drives the alternation of the angle of the interface layers, so it follows the creation order, not the
     z order, which is an accepted limit.
   - A layer whose new height would not be above the minimum height of a layer (ApiInternal::LayerAccess::min_scaled_height, the limit of
     the Layer constructor) makes the step throw std::runtime_error; the families publish layers far thicker than 1 um. */

namespace Slic3r::Steps::StepGenerateSupport {
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

    // Reads stop flags from the bound orchestrator/print, ignoring the payload print, without mutation or failure;
    // callbacks then refuse without constructing the service or adding a message.
    bool stop_requested(const print_handle *) const noexcept { return m_orchestrator.is_plugin_cancelled() || m_print.canceled(); }
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
// Both inputs are in scaled units (1 unit = 1 nm), unsorted and may repeat. Pass 1: a movable
// value within 4 * SCALED_EPSILON (inclusive) of a fixed value takes the nearest fixed value (the lower one on a tie). Pass 2: an ascending
// sweep over the other movable values keeps the first, and each next value within 4 * SCALED_EPSILON of the last kept one takes its value;
// otherwise it becomes the kept one. Hence, when the fixed values are more than 4 * SCALED_EPSILON apart, two distinct results (fixed values
// included) are more than 4 * SCALED_EPSILON apart, and no value moves by more than 4 * SCALED_EPSILON.
CloseZMapping close_z_mapping(std::vector<coord_t> fixed, std::vector<coord_t> movable)
{
    constexpr coord_t tolerance = 4 * SCALED_EPSILON;
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
// Moves the auxiliary layers of the objects through one mapping computed on all of them: fixed values are the object layers,
// movable ones the tops and bottoms of the auxiliary layers. A layer that collapses is an error (the families publish layers far
// thicker than 1 um; the limit is the minimum height of the Layer constructor). The container is then sorted again (sort_support_layers()). The
// interface ids of LayerSupportProperty and Layer::id are creation identifiers, not renumbered.
void align_support_z(const std::vector<PrintObject *> &objects)
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
    const Detail::CloseZMapping mapping = Detail::close_z_mapping(std::move(fixed), std::move(movable));
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
                    " mm thick after its print z were kept 0.5 um apart from the others.");
            ApiInternal::LayerAccess::set_z(*layer, print_z, print_z - bottom);
        }
        Detail::sort_support_layers(object->mutable_auxiliary_layers());
    }
}
} // namespace

void clear(Print &) {}
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
    // Printed object by object: the same criterion as DefaultPlanBuilder::use_object_plan (Plugins/Ordering/DefaultPlanBuilder.cpp, local to that file)
    // and Print.cpp:1565. Both options invalidate STEP_SUPPORT (complete_objects through STEP_SKIRT_BRIM, which replays from the same checkpoint),
    // so the scope of the pass is always rebuilt when one of them changes.
    if (print.config().complete_objects.value || print.config().parallel_objects_step.value > 0)
        for (PrintObject *object : objects)
            align_support_z({ object });
    else
        align_support_z(objects);
}

} // namespace Slic3r::Steps::StepGenerateSupport
