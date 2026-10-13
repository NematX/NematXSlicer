
#include "StepGenerateSupport.hpp"

#include <mutex>
#include <optional>
#include <string>

#include <boost/log/trivial.hpp>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Print.hpp"
#include "InfillPatternService.hpp"
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
   output is never published by this step; the family only reads it. */

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

    // True when the plugin cancellation is requested or the print is cancelled: the callbacks then fail at
    // once, without building the service or adding a message, because the runner is about to stop the step.
    bool stop_requested() const noexcept { return m_orchestrator.is_plugin_cancelled() || m_print.canceled(); }
private:
    Orchestrator &m_orchestrator;
    Print &m_print;
    std::once_flag m_built;
    std::optional<InfillPatternService> m_service;
};

// Payload callback: resolve a pattern plugin id through the step's service. Never throws; returns INVALID
// at once, without building the service, when the step is already stopping.
infill_pattern_runtime_id resolve_pattern_id_callback(const run_ctx_generate_support *ctx,
                                                      const char *pattern_plugin_id) noexcept;
// Payload callback: fill one surface through the step's service. Layer, island, region island, primary
// region and no-overlap areas are forwarded as given, NULL included. Never throws: a failure empties
// output, is recorded (no message for a PluginStopped), and returns 0; a step already stopping empties output and returns 0 without a message.
int32_t generate_pattern_callback(const run_ctx_generate_support *ctx,
                                  infill_pattern_runtime_id pattern_id,
                                  const layer_handle *layer,
                                  const layer_island_handle *island,
                                  const layer_region_island_handle *region_island,
                                  const layer_region_handle *primary_region,
                                  const surface_handle *surface,
                                  const expolygon_collection_handle *no_overlap_areas,
                                  const raw_infill_pattern_params *params,
                                  extrusion_entity_handle *output) noexcept;

// A null context is an ordinary refusal. A stopping step gives INVALID before the service is touched, so a
// failed construction is not retried. Only C++ code runs inside the try; a failure is contained after the
// catch (recorded once, plugin cancellation requested).
infill_pattern_runtime_id resolve_pattern_id_callback(const run_ctx_generate_support *ctx,
                                                      const char *pattern_plugin_id) noexcept
{
    if (ctx == nullptr || ctx->host_context == nullptr)
        return INFILL_PATTERN_RUNTIME_ID_INVALID;
    LazyPatternService &patterns = *static_cast<LazyPatternService *>(ctx->host_context);
    if (patterns.stop_requested())
        return INFILL_PATTERN_RUNTIME_ID_INVALID;
    try {
        return patterns.get().resolve_pattern_id(pattern_plugin_id);
    } catch (...) {
        contain_pattern_callback_failure(patterns.orchestrator(), STEP_SUPPORT, nullptr);
        return INFILL_PATTERN_RUNTIME_ID_INVALID;
    }
}

// Null arguments are an ordinary refusal (0, output untouched). A stopping step empties output and returns 0
// without a message. Otherwise the payload of the pattern plugin is copied from the arguments and the service
// runs it; an exception is contained: output is emptied and the plugin cancellation is requested. A PluginStopped
// adds no diagnostic (a known cause already has its own message; an Unknown cause has none, and the final step
// error says a stop was requested); any other exception is recorded as a host failure. A print cancellation is
// left to the runner. Only C++ code runs inside the try.
int32_t generate_pattern_callback(const run_ctx_generate_support *ctx,
                                  infill_pattern_runtime_id pattern_id,
                                  const layer_handle *layer,
                                  const layer_island_handle *island,
                                  const layer_region_island_handle *region_island,
                                  const layer_region_handle *primary_region,
                                  const surface_handle *surface,
                                  const expolygon_collection_handle *no_overlap_areas,
                                  const raw_infill_pattern_params *params,
                                  extrusion_entity_handle *output) noexcept
{
    if (ctx == nullptr || ctx->host_context == nullptr || params == nullptr || output == nullptr)
        return 0;

    LazyPatternService &patterns = *static_cast<LazyPatternService *>(ctx->host_context);
    if (patterns.stop_requested()) {
        empty_output(output);
        return 0;
    }
    try {
        run_ctx_infill_pattern payload = {};
        payload.print = ctx->print;
        payload.object = ctx->object;
        payload.layer = layer;
        payload.island = island;
        payload.region_island = region_island;
        payload.primary_region = primary_region;
        payload.surface = surface;
        payload.no_overlap_areas = no_overlap_areas;
        payload.params = params;
        payload.output = output;
        return patterns.get().generate_pattern(pattern_id, payload);
    } catch (...) {
        contain_pattern_callback_failure(patterns.orchestrator(), STEP_SUPPORT, output);
        return 0;
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
void run_step(Orchestrator &orchestrator, Print &print)
{
    // Lives for the whole step; only built when a family calls a pattern callback.
    LazyPatternService patterns(orchestrator, print);
    Detail::run_object_step_plugins(orchestrator,
                                   print,
                                   STEP_SUPPORT,
                                   &print.full_print_config(),
                                   print.objects().size(),
                                   [&print, &patterns](const size_t object_idx) {
        run_ctx_generate_support context = {};
        context.print = reinterpret_cast<const print_handle *>(&print);
        context.object = reinterpret_cast<const object_handle *>(&print.object(object_idx));
        context.host_context = &patterns;
        context.resolve_pattern_id = &resolve_pattern_id_callback;
        context.generate_pattern = &generate_pattern_callback;
        return context;
    });
}

} // namespace Slic3r::Steps::StepGenerateSupport
