///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_bridgedetector_hpp_
#define plugins_bridgedetector_hpp_

#include "libslic3r/Api/plugin/c/slic3r_plugin.h"
#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

namespace slic3r_api { namespace BridgeDetectorPlugin {

// Service plugin that exposes the native BridgeDetector implementation through
// the C plugin ABI. Other plugins ask the orchestrator to create a detector, then
// use the returned bridge_detector_instance function table.
class BridgeDetector
{
public:
    static BridgeDetector &instance(orchestrator_handle *orch);
    BridgeDetector(orchestrator_handle *orch) {}

    plugin_instance c_instance() const;

    const char *id() const noexcept;
    const char *name() const noexcept;
    const char *description() const noexcept;
    const char *exclusive_group() const noexcept;
    slicing_step_t step() const noexcept;
    const char *const *dependencies() const noexcept;
    int32_t priority() const noexcept;
    void initialize(storage_handle *storage) const;
    void setup(const plugin_run_context *run_ctx, uint32_t run_count) const;
    void setup_run(const plugin_run_context *run_ctx) const;
    void run(const plugin_run_context *run_ctx) const;

    static const char *get_id_bridge(void *plugin_ctx);
    static const char *get_name_bridge(void *plugin_ctx);
    static const char *get_description_bridge(void *plugin_ctx);
    static const char *get_exclusive_group_bridge(void *plugin_ctx);
    static slicing_step_t get_step_bridge(void *plugin_ctx);
    static const_strings_t get_dependencies_bridge(void *plugin_ctx);
    static int32_t get_priority_bridge(void *plugin_ctx);
    static int32_t used_config_keys_bridge(void *plugin_ctx, raw_used_config_key *keys);
    static int32_t defined_config_keys_bridge(void *plugin_ctx, const char **keys);
    // Initialize borrowed detector/storage without transferring ownership; contain exceptions as diagnostic return codes.
    static int32_t initialize_bridge(void *plugin_ctx, storage_handle *storage);
    static void setup_bridge(void *plugin_ctx, const plugin_run_context *run_ctx, uint32_t run_count);
    static void setup_run_bridge(void *plugin_ctx, const plugin_run_context *run_ctx);
    static void run_bridge(void *plugin_ctx, const plugin_run_context *run_ctx);
};

// Register the default bridge detector implementation. Hosts may later replace
// it by registering another BRIDGE_DETECTOR plugin with higher priority.
void register_bridge_detector_plugin(orchestrator_handle *orch);

}} // namespace slic3r_api::BridgeDetectorPlugin

#endif // plugins_bridgedetector_hpp_
