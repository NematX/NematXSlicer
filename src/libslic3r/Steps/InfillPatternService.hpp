///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/* Host side of the INFILL_PATTERN service.

   INFILL_PATTERN plugins are not slicing steps: another step plugin asks the
   host to run one of them on one surface. STEP_INFILL generators do this
   through the C callbacks resolve_pattern_id() and generate_pattern() of their
   payload (other steps may offer the same callbacks). Those callbacks
   forward to one InfillPatternService owned by the step run.

   How to use it from a step:
   1. Build one service before any plugin of the step can call a pattern, or
      build it on the first call and make that construction complete before
      any other thread reads the service. The constructor gives every active
      pattern plugin a compact runtime id (1, 2, ...; 0 stays
      INFILL_PATTERN_RUNTIME_ID_INVALID), its guarded host context, its error
      flag and a prepared plugin_run_context. It throws when a setup() fails;
      a lazy owner must keep the exception from crossing the C ABI. The same
      holds for the later calls: generate_pattern() throws when a pattern fails
      or the print is cancelled, and resolve_pattern_id() only when allocating
      its diagnostics fails. Every owner of a payload callback that forwards to
      them must catch everything and call contain_pattern_callback_failure()
      (declared below) from its catch block. STEP_INFILL uses the same helper
      for failures from its pattern, discovery, modifier and extrusion callbacks.
   2. Let the step plugin translate the configured string id with
      resolve_pattern_id(), then fill a run_ctx_infill_pattern and call
      generate_pattern().
   3. setup() of a pattern must run once in the step before its first call,
      because it binds per-run plugin state such as progress reporting.
      STEP_INFILL calls initialize_pattern() once per used pattern with its
      discovered work groups (lightning builds per-object caches there); a
      service built with SetupPatterns::Refuse does it in the constructor with
      an empty payload.

   Concurrency: after construction the service is never modified. Lookups only
   read its maps, a call copies the prepared run context, and the orchestrator
   message list and plugin storage have their own locks. Several threads may
   therefore call resolve_pattern_id() and generate_pattern() at the same time,
   which a step that runs objects in parallel relies on. */
#ifndef steps_infillpatternservice_hpp_
#define steps_infillpatternservice_hpp_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"

namespace Slic3r {
class Plugin;
class Print;

namespace Steps {

class InfillPatternService
{
public:
    // Whether patterns that need setup-time initialization (lightning,
    // adaptive cubic, support cubic) may be generated. Only STEP_INFILL runs
    // the discovery and initialization pass these patterns depend on; it then
    // calls initialize_pattern() itself. With Refuse, the constructor calls
    // setup() of every other pattern with an empty initialization payload.
    // STEP_INFILL uses Allow; Refuse is the mode planned for the steps that do not run that pass (the support steps).
    enum class SetupPatterns { Allow, Refuse };

    // Prepare every active INFILL_PATTERN plugin for this run. run_count is the
    // number of calls advertised to each pattern's progress reporting. Throws
    // like initialize_pattern() when a setup() call fails.
    InfillPatternService(Orchestrator &orchestrator, Print &print, size_t run_count, SetupPatterns setup_patterns);
    InfillPatternService(const InfillPatternService &) = delete;
    InfillPatternService &operator=(const InfillPatternService &) = delete;

    // True when no pattern plugin is active, so every resolve fails.
    bool empty() const { return m_patterns.empty(); }

    // Translate a configured pattern plugin id into its runtime id. An inactive
    // id falls back to the first active pattern with a warning; with no active
    // pattern it returns INFILL_PATTERN_RUNTIME_ID_INVALID.
    infill_pattern_runtime_id resolve_pattern_id(const char *pattern_plugin_id) const;

    // Return the stable plugin id; never NULL, and "" for an unknown id.
    const char *pattern_plugin_id(infill_pattern_runtime_id pattern_id) const noexcept;

    // Call setup() of one pattern with the initialization payload built by
    // STEP_INFILL. Unknown ids are ignored.
    void initialize_pattern(infill_pattern_runtime_id pattern_id,
                            run_ctx_infill_pattern_init payload,
                            uint32_t surface_count);

    // Run setup_run() and run() of the selected pattern on one surface. Returns
    // 0 without generating anything for an unknown id or a refused pattern,
    // and 1 otherwise. Throws when the pattern reports an error or the print is
    // cancelled, like every guarded plugin phase.
    int32_t generate_pattern(infill_pattern_runtime_id pattern_id, run_ctx_infill_pattern payload);

private:
    // Everything one call of a pattern needs, created by the constructor. The
    // map node never moves, so run_context.host_context stays valid.
    struct Pattern
    {
        Plugin *plugin = nullptr;
        std::atomic_bool execution_error { false };
        plugin_host_context host_context = {};
        plugin_run_context run_context = {};
    };

    // Look up a runtime id for generation, warning when it is unknown.
    Pattern *pattern_for_generation(infill_pattern_runtime_id pattern_id);

    Orchestrator &m_orchestrator;
    Print &m_print;
    SetupPatterns m_setup_patterns;
    std::map<std::string, infill_pattern_runtime_id> m_runtime_ids_by_plugin_id;
    std::map<infill_pattern_runtime_id, Pattern> m_patterns;
};

// Contain the exception being handled by the calling catch block at a pattern callback boundary (it
// rethrows it internally, so it must only be called from an active catch block): a print cancellation is
// left to the runner; any other exception is logged and added as an error message of the step, and the
// plugin stop is requested (see Detail::contain_callback_failure in StepRunner.hpp). A non-null output is
// then left as an empty collection. Never throws.
void contain_pattern_callback_failure(Orchestrator &orchestrator,
                                      slicing_step_t step,
                                      extrusion_entity_handle *output) noexcept;

// Leave output as an empty collection: remove its children, and a polyline written directly on it. Never throws.
void empty_output(extrusion_entity_handle *output) noexcept;

} // namespace Steps
} // namespace Slic3r

#endif // steps_infillpatternservice_hpp_
