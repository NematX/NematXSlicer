///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_defaultinfillgenerator_hpp_
#define plugins_infill_defaultinfillgenerator_hpp_

#include <map>
#include <memory>
#include <mutex>

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace Slic3r { class PrintObject; }

namespace slic3r_api { namespace Infill { namespace DefaultInfillGeneratorPlugin {

// Recipes discovered by setup_run() for one object, reused by run().
struct DefaultInfillObjectWorkCache;

// Baseline STEP_INFILL plugin.
//
// The plugin owns the high-level infill loop: it walks the fill surfaces
// created by previous steps, resolves the per-surface settings, delegates line
// generation to INFILL_PATTERN plugins, then asks the host to publish the
// resulting extrusion tree. The host still owns plugin selection and data-tree
// mutation through callbacks exposed in run_ctx_generate_infill.
//
// setup_run() performs a discovery pass per object: it resolves every fill
// surface recipe, groups surfaces that share one recipe and reports the groups
// for the host to initialize the used patterns for the whole print. run() then
// generates from that cache instead of recomputing the recipes.
class DefaultInfillGenerator : public PluginBase
{
public:
    static DefaultInfillGenerator &instance(orchestrator_handle *orch);

private:
    DefaultInfillGenerator(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *exclusive_group_impl() const noexcept override { return "generator"; }
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    int32_t defined_config_keys(const char **keys) const noexcept override;
    void inilialize_impl(storage_handle *storage) const override;
    const char *progress_message_format_impl() const noexcept override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;

    // Recipes discovered during setup_run() and reused by run().
    mutable std::map<const Slic3r::PrintObject *, std::shared_ptr<DefaultInfillObjectWorkCache>> m_object_caches;
    mutable std::mutex m_cache_mutex;
};

void register_default_infill_generator_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Infill::DefaultInfillGeneratorPlugin

#endif // plugins_infill_defaultinfillgenerator_hpp_
