/* A STEP_SUPPORT run context for the code that samples the infill patterns from a support family (the tip sampler of the tree
   family, slic3r_api::Support::Common::PatternService), without running the support step.

   The host builds the pattern callbacks of run_ctx_generate_support (ABI 1.2) in StepGenerateSupport.cpp over a lazy
   InfillPatternService. This fixture builds the same payload over a service that the test owns, as test_support_common.cpp does for
   the toolpath step: resolve_pattern_id() and generate_pattern() forward to the service. A failure inside a pattern is contained
   like the host does (the call returns 0 and the message is kept in errors()); the host additionally requests the plugin
   cancellation, which the context reports through is_cancelled() here too.

       Print print; ... SupportGrid::prefix(print);
       PluginStorage storage;
       SupportPatternContext context(print, reinterpret_cast<storage_handle *>(&storage));
       route_trees(..., context.context());

   The test runtime leaves the supportbase pattern inactive: the fixture activates it for its lifetime (before the service is
   built, which prepares the active patterns) and restores the previous state. It counts the calls by pattern plugin id. */
#ifndef support_pattern_test_context_hpp_
#define support_pattern_test_context_hpp_

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <catch2/catch.hpp>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "libslic3r/Print.hpp"
#include "libslic3r/Steps/InfillPatternService.hpp"

namespace Slic3r::Test::SupportTree {

class SupportPatternContext
{
public:
    // The print must stay alive as long as the context. storage is the plugin storage of the context (the run storage).
    SupportPatternContext(Print &print, storage_handle *storage)
        : m_supportbase(), m_service(Orchestrator::instance(), print, 1, Steps::InfillPatternService::SetupPatterns::Refuse)
    {
        m_payload.print = reinterpret_cast<const print_handle *>(&print);
        m_payload.object = print.objects().empty() ? nullptr : reinterpret_cast<const object_handle *>(&print.object(0));
        m_payload.host_context = this;
        m_payload.resolve_pattern_id = &SupportPatternContext::resolve_pattern_id;
        m_payload.generate_pattern = &SupportPatternContext::generate_pattern;
        m_context.step = STEP_SUPPORT;
        m_context.print = m_payload.print;
        m_context.plugin_storage = storage;
        m_context.data = &m_payload;
        m_context.host_context = reinterpret_cast<plugin_host_context *>(this);
        m_context.is_cancelled = &SupportPatternContext::is_cancelled;
    }
    SupportPatternContext(const SupportPatternContext &) = delete;
    SupportPatternContext &operator=(const SupportPatternContext &) = delete;

    // The context to give to the code under test, and its STEP_SUPPORT payload.
    const plugin_run_context *context() const { return &m_context; }
    const run_ctx_generate_support *payload() const { return &m_payload; }

    // The number of pattern calls, by pattern plugin id ("" for an unknown runtime id).
    std::map<std::string, size_t> calls() const
    {
        const std::lock_guard<std::mutex> lock(m_mutex);
        return m_calls;
    }
    size_t calls(const std::string &pattern_plugin_id) const
    {
        const std::map<std::string, size_t> all = calls();
        const std::map<std::string, size_t>::const_iterator found = all.find(pattern_plugin_id);
        return found == all.end() ? 0 : found->second;
    }
    // The total number of pattern calls.
    size_t total_calls() const
    {
        size_t total = 0;
        for (const std::pair<const std::string, size_t> &entry : calls())
            total += entry.second;
        return total;
    }
    // Requests the cancellation of the context once this many pattern calls have returned (0: never), as a plugin error would.
    void cancel_after_calls(size_t count) { m_cancel_after = count; }
    // The messages of the exceptions contained in the pattern calls.
    std::vector<std::string> errors() const
    {
        const std::lock_guard<std::mutex> lock(m_mutex);
        return m_errors;
    }

private:
    // Makes supportbase active until the end of the fixture; the service must be built after it.
    struct ActiveSupportBase
    {
        bool was_active = Orchestrator::instance().is_plugin_active("supportbase");
        ActiveSupportBase() { REQUIRE(Orchestrator::instance().set_plugin_active("supportbase", true)); }
        ~ActiveSupportBase() { Orchestrator::instance().set_plugin_active("supportbase", was_active); }
    };

    static SupportPatternContext &self(const run_ctx_generate_support *payload) { return *static_cast<SupportPatternContext *>(payload->host_context); }

    static infill_pattern_runtime_id resolve_pattern_id(const run_ctx_generate_support *payload, const char *pattern_plugin_id)
    {
        return self(payload).m_service.resolve_pattern_id(pattern_plugin_id);
    }

    static int32_t generate_pattern(const run_ctx_generate_support *payload, infill_pattern_runtime_id pattern_id, const layer_handle *layer,
                                    const layer_island_handle *island, const layer_region_island_handle *region_island,
                                    const layer_region_handle *primary_region, const surface_handle *surface,
                                    const expolygon_collection_handle *no_overlap_areas, const raw_infill_pattern_params *params,
                                    extrusion_entity_handle *output)
    {
        SupportPatternContext &fixture = self(payload);
        {
            const std::lock_guard<std::mutex> lock(fixture.m_mutex);
            ++fixture.m_calls[fixture.m_service.pattern_plugin_id(pattern_id)];
        }
        run_ctx_infill_pattern pattern_payload = {};
        pattern_payload.print = payload->print;
        pattern_payload.object = payload->object;
        pattern_payload.layer = layer;
        pattern_payload.island = island;
        pattern_payload.region_island = region_island;
        pattern_payload.primary_region = primary_region;
        pattern_payload.surface = surface;
        pattern_payload.no_overlap_areas = no_overlap_areas;
        pattern_payload.params = params;
        pattern_payload.output = output;
        try {
            const int32_t ran = fixture.m_service.generate_pattern(pattern_id, pattern_payload);
            if (fixture.m_cancel_after != 0 && fixture.total_calls() >= fixture.m_cancel_after)
                fixture.m_cancelled = true;
            return ran;
        } catch (const std::exception &error) {
            const std::lock_guard<std::mutex> lock(fixture.m_mutex);
            fixture.m_errors.push_back(error.what());
            fixture.m_cancelled = true;
            return 0;
        }
    }

    static int is_cancelled(plugin_host_context *host_context)
    {
        return reinterpret_cast<SupportPatternContext *>(host_context)->m_cancelled ? 1 : 0;
    }

    ActiveSupportBase m_supportbase;
    Steps::InfillPatternService m_service;
    run_ctx_generate_support m_payload = {};
    plugin_run_context m_context = {};
    mutable std::mutex m_mutex;
    std::map<std::string, size_t> m_calls;
    std::vector<std::string> m_errors;
    std::atomic_bool m_cancelled { false };
    std::atomic<size_t> m_cancel_after { 0 };
};
} // namespace Slic3r::Test::SupportTree

#endif
