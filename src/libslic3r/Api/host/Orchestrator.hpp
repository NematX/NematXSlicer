///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Orchestrator_hpp_
#define slic3r_Orchestrator_hpp_

#include <atomic>
#include <cassert>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <stddef.h>
#include <stdint.h>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_bridge_detector.h"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_extrusion_property.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/GCode/ThumbnailData.hpp"
#include "libslic3r/MultiPoint.hpp"
#include "libslic3r/Polygon.hpp"

#include "libslic3r/Config/GuiRuleRegistry.hpp"

#include "Plugin.hpp"

namespace Slic3r {

class Orchestrator;
class Print;
struct GCodeProcessorResult;
class MultiPoint;
class Polyline;
class Polygon;
class ExPolygon;
class SurfaceCollection;
class ExtrusionEntity;
class TriangleMesh;
class PluginStorage;
class DynamicConfig;
class DynamicPrintConfig;
struct OptionKeyIdx;
namespace ApiClipper { class ClipperShapes; }

// Referenced by Orchestrator's constructor so static-library linkers retain
// the translation unit containing C exports used only by external plugins.
void link_config_definition_api() noexcept;

} // namespace Slic3r

// Host-side state passed back to callbacks through the opaque C ABI handle.
// It is intentionally not exposed in Api/plugin/c: plugins only see
// plugin_host_context* and must use the callbacks from plugin_run_context.
struct plugin_host_context
{
    Slic3r::Orchestrator *orchestrator = nullptr;
    Slic3r::Print *print = nullptr;
    Slic3r::Plugin *plugin = nullptr;
    slicing_step_t step = STEP_LAYER_HEIGHT;
    size_t object_idx = 0;
    size_t object_count = 0;
    // Generic executions point this at invocation-local state so a reported
    // plugin error can be distinguished from an unrelated cancellation.
    std::atomic_bool *execution_error = nullptr;
    // Normal pipeline failures cancel dependent work. Independent detection
    // probes set this false so one malformed candidate cannot suppress the
    // remaining candidates.
    bool cancel_on_error = true;
};

namespace Slic3r {

struct GenericFacetsAnnotationDefinition
{
    // The key is the stable identity of a facet annotation kind. Project files,
    // Model data and plugin API calls use this key instead of runtime indexes.
    std::string key;

    // User-facing text used by the generic seam-like annotation tool. Built-in GUI
    // classes may replace these with translated strings, while plugins provide
    // plain UTF-8 labels through the C API.
    std::string label;
    std::string enforce_label;
    std::string block_label;
    // Gettext catalog which owns the labels. An empty domain selects the
    // application catalog for definitions created by the host.
    std::string translation_domain;

    // Built-in tools may point to an icon stored in resources/icons. This stays
    // separate from plugin icons so libslic3r does not need to know how wx code
    // stores or rasterizes GUI bitmaps.
    std::string icon_filename;

    // Plugins provide SVG source code directly. The GUI registers this string
    // in BitmapCache under key, so normal wx code can later call
    // get_bmp_bundle(key). The OpenGL toolbar also reads this data directly
    // when it builds its sprite atlas.
    std::string icon_svg;
};

inline GenericFacetsAnnotationDefinition builtin_seam_facets_annotation_definition()
{
    GenericFacetsAnnotationDefinition def;
    def.key = "builtin:seam";
    def.label = "Seam painting";
    def.enforce_label = "Enforce seam";
    def.block_label = "Block seam";
    def.translation_domain = "Slic3r";
    def.icon_filename = "seam.svg";
    return def;
}

// Package loading happens before plugin activation. These values describe the
// package-level result retained for the current process, so updater UI can
// explain why installed code is unavailable without parsing log files.
enum class PluginPackageLoadState
{
    Loading,
    Loaded,
    LoadedWithErrors,
    Failed
};

enum class PluginPackageLoadErrorCode
{
    PackageMissing,
    InvalidPackage,
    LibraryOpenFailed,
    DependencyMissing,
    MissingAbiExport,
    ApiHeaderVersionMismatch,
    MissingRegistrationExport,
    RegistrationFailed,
    NoPluginsRegistered,
    PythonRuntimeUnavailable,
    PythonReadFailed,
    PythonCompileFailed,
    PythonImportFailed,
    PythonRegistrationFailed,
    ConfiguredPluginMissing
};

struct PluginPackageLoadIssue
{
    PluginPackageLoadErrorCode code = PluginPackageLoadErrorCode::InvalidPackage;
    std::string detail;
    std::string plugin_id;
    uint32_t system_error = 0;
};

struct PluginPackageLoadReport
{
    std::string package_id;
    std::string package_path;
    PluginPackageLoadState state = PluginPackageLoadState::Loading;
    bool allows_no_plugins = false;
    std::vector<std::string> registered_plugin_ids;
    std::vector<PluginPackageLoadIssue> issues;
};

class Orchestrator
{
public:
    struct TranslationCatalog
    {
        std::string domain;
        std::string locale_directory;
        std::string package_root;
    };

    class PluginRegistrationScope
    {
    public:
        PluginRegistrationScope(const PluginRegistrationScope &) = delete;
        PluginRegistrationScope &operator=(const PluginRegistrationScope &) = delete;
        // Moving transfers the scope token; the source no longer closes the scope.
        PluginRegistrationScope(PluginRegistrationScope &&other) noexcept;
        // Close this scope before taking ownership of another scope token.
        PluginRegistrationScope &operator=(PluginRegistrationScope &&other) noexcept;
        // Close the scope when this is the last owner of its token.
        ~PluginRegistrationScope();

    private:
        friend class Orchestrator;
        explicit PluginRegistrationScope(Orchestrator *orchestrator) : m_orchestrator(orchestrator) {}

        Orchestrator *m_orchestrator = nullptr;
    };

    struct CustomExtrusionPropertyInfo
    {
        slic3r_property_type type;
        std::string name;
        uint32_t byte_count;
        uint32_t alignment;
    };

    using PropertyInfo = CustomExtrusionPropertyInfo;

    struct DynamicStepInfo
    {
        // name is the stable cross-plugin identity. id is only a compact key
        // inside this orchestrator, while invalidates_step tells generated
        // provider selectors which ordinary pipeline result becomes stale.
        slicing_step_t id = STEP_NONE;
        std::string name;
        slicing_step_t invalidates_step = STEP_ANY;
    };

    struct PluginUiFragment
    {
        // One plugin contribution to a .ui file.
        //
        // fragment_id is the de-duplication key inside target_file. This is
        // deliberately separate from plugin_id: alternative plugins may expose
        // the same logical controls and therefore share one fragment id.
        std::string target_file;
        std::string fragment_id;
        std::string content;
        std::string plugin_id;
        std::string exclusive_group;
        int32_t priority = 0;
        uint64_t order = 0;
    };

    // GUI activation rules are owned by GuiRuleRegistry; these aliases keep the
    // existing Orchestrator-qualified type names usable.
    using PluginGuiRuleClause = GuiRuleRegistry::PluginGuiRuleClause;
    using PluginGuiRule = GuiRuleRegistry::PluginGuiRule;
    using SettingActivation = GuiRuleRegistry::SettingActivation;
    using GuiRuleApplication = GuiRuleRegistry::GuiRuleApplication;
    using GuiRuleRegistration = GuiRuleRegistry::GuiRuleRegistration;

    struct ConfigOptionOwner
    {
        // Tracks which plugin first introduced a dynamic option key.
        //
        // The owner is not about memory ownership. It is a compatibility rule:
        // another plugin may re-declare the same key only when it is the same
        // plugin or a plugin from the same non-empty exclusive group.
        std::string plugin_id;
        std::string exclusive_group;
    };

    // One deferred modification of an existing option definition. Patches are
    // recorded while plugins initialize and applied together once every plugin
    // has been loaded, so the result does not depend on initialization order.
    struct OptionDefPatch
    {
        raw_option_def_patch_kind kind = RAW_OPTION_DEF_PATCH_NONE;
        std::string opt_key;
        std::vector<std::pair<std::string, std::string>> enum_values;
        std::vector<std::string> remove_values;
        std::string default_serialized_value;
        std::string source_option_key;
        bool prepend = false;
        // Plugin that recorded the patch; empty for host code.
        std::string plugin_id;
    };

    enum class PluginMessageLevel
    {
        Warning,
        Error
    };

    struct PluginMessage
    {
        PluginMessageLevel level = PluginMessageLevel::Warning;
        std::string plugin_id;
        slicing_step_t step = STEP_NONE;
        std::string message;
        // Zero denotes text-only diagnostics; registered ids remain resolvable after the callback.
        int32_t diagnostic_id = PLUGIN_ERROR_OK;
    };

    // Return the process-wide orchestrator that owns registered plugins and host services.
    static Orchestrator &instance();
    orchestrator_handle * handle() { return reinterpret_cast<orchestrator_handle *>(this); }

    // Return borrowed pointers to every registered plugin.
    std::vector<Plugin *> registered_plugins() const;
    // Return all registered plugins for a step, including inactive plugins.
    std::vector<Plugin *> get_all_plugins_for_step(slicing_step_t step) const;
    // Return the active registered plugins for a step.
    std::vector<Plugin *> get_active_plugins_for_step(slicing_step_t step) const;
    // Return the active plugins currently selected for a step.
    std::vector<Plugin *> get_current_plugins_for_step(slicing_step_t step) const;
    // Find a plugin by ID; the returned pointer remains owned by the orchestrator.
    const Plugin *get_plugin(const std::string &plugin_id) const;
    // Find a mutable plugin by ID; return nullptr when the ID is not registered.
    Plugin *get_plugin(const std::string &plugin_id);
    // Return true only when plugin is one of this orchestrator's owned,
    // registered instances. Public borrowed handles are validated with this
    // check before they are dereferenced or executed.
    bool owns_plugin(const Plugin *plugin) const;
    // Associate a registered plugin with a pipeline step.
    void add_plugin_to_step(Plugin *plugin, slicing_step_t step);
    // Query activation for a plugin handle.
    bool is_plugin_active(const Plugin *plugin) const;
    // Query activation by plugin ID; unknown IDs are inactive.
    bool is_plugin_active(const std::string &plugin_id) const;
    // Deactivate all plugins without removing their registrations.
    void clear_active_plugins();
    // Set activation for a non-null plugin owned by this orchestrator. This
    // internal overload trusts the pointer's origin and does not verify it.
    bool set_plugin_active(Plugin *plugin, bool active);
    // Set activation by plugin ID and report success.
    bool set_plugin_active(const std::string &plugin_id, bool active);

    // Diagnose missing/inactive direct dependencies after the complete active
    // set is established. This read-only check does not block activation or
    // validate ordering cycles or per-print exclusive-provider selection.
    std::vector<std::string> active_plugin_dependency_errors() const;
    // A non-mutating checkbox proposal. Lists describe additional changes by
    // cause; selected is the complete resulting activation set.
    struct ActivationChange {
        std::set<std::string> selected;
        std::set<std::string> solidarity_changes;
        std::set<std::string> dependency_changes;
        std::set<std::string> groups;
        std::string error;
    };
    // Declarations accumulate members; registrations may arrive afterward.
    bool register_activation_group(const std::string &id, const std::vector<std::string> &members,
                                   std::string &error);
    // Return the activation-solidarity declarations by borrowed reference.
    const std::map<std::string, std::set<std::string>> &activation_groups() const { return m_activation_groups; }
    // Report incomplete declarations even when nobody requested activation.
    std::map<std::string, std::string> incomplete_activation_groups() const;
    // Report invalid members in a proposed set, without altering runtime state.
    std::map<std::string, std::string> plugin_activation_errors(const std::set<std::string> &selected) const;
    // Expand solidarity and dependency effects to a fixed point. The caller
    // decides whether to accept; execution order and runtime state are untouched.
    ActivationChange propose_plugin_activation(const std::vector<std::string> &current,
        const std::vector<std::string> &requested, bool active) const;
    // Expand requested IDs to their transitive dependency closure, without activation.
    // Missing registrations fail atomically; cycles terminate through the visited set.
    bool plugin_dependency_closure(const std::vector<std::string> &ids,
                                   std::vector<std::string> &closure, std::string &error) const;
    // Called after the entire activation set is assembled. Reject invalid consumers
    // transitively and retain diagnostics for the startup notification and catalog.
    void block_unsatisfied_plugin_dependencies(const std::vector<std::string> &requested = {});
    // Return the reasons recorded when plugin activation was blocked.
    const std::map<std::string, std::string> &blocked_plugin_activations() const { return m_blocked_plugin_activations; }
    // Return active plugin handles; the orchestrator retains their ownership.
    const std::unordered_set<Plugin *> &active_plugins() const { return m_active_plugins; }

    // Preflight a future active-plugin set before it is written to disk by the
    // plugin configuration dialog.
    //
    // This catches option-key ownership conflicts early enough to show a GUI
    // error without restarting. It intentionally does not replace plugin
    // initialization: the full raw_config_option_def compatibility check still
    // happens when an active plugin actually registers its options.
    bool validate_plugin_activation(const std::vector<std::string> &plugin_ids,
                                    std::string &error_message) const;


    // Register a dynamic print option provided by a plugin.
    //
    // This is the commit point for plugin settings. Compatible duplicate
    // definitions are accepted only inside the same exclusive group, where
    // several alternative plugins intentionally publish the same option.
    option_def_error_code create_new_print_config(const raw_config_option_def *def);

    // Record a deferred option definition patch. Nothing is applied until
    // finalize_plugin_option_defs() runs.
    option_def_error_code modify_print_config(const raw_option_def_patch *patch);

    // Move the invalidation step of every setting read by an active plugin no
    // later than the pipeline step running that plugin. Then apply every
    // recorded patch, prune enum values whose plugin is missing or inactive
    // when the definition advertises contains_plugin_id, and rebuild the
    // scripted enum maps. Missing target keys are ignored. A plugin-id list
    // that becomes empty reports a non-fatal plugin error and stays empty.
    // Runs once, after initialize_plugins() and before PrintConfigDef::finalize().
    void finalize_plugin_option_defs();

    // Register a plugin instance and transfer its lifetime to the orchestrator.
    bool register_plugin(plugin_instance plugin);

    // Register and query plugin-defined service steps. Runtime ids are scoped
    // to this orchestrator; callers exchange the stable name, not the number.
    slicing_step_t register_step(const char *namespaced_name,
                                 slicing_step_t invalidates_step);
    // Look up runtime step metadata by numeric ID.
    const DynamicStepInfo *step_info(slicing_step_t step) const;
    // Look up runtime step metadata by stable namespaced name.
    const DynamicStepInfo *step_info(const char *namespaced_name) const;

    // Execute the normal setup/setup_run/run lifecycle for one plugin. The
    // payload array remains owned by the caller for the duration of this call.
    raw_plugin_execution_status execute_plugin(Plugin &plugin,
                                               Print *print,
                                               const raw_plugin_run_payload *payloads,
                                               uint32_t run_count,
                                               bool cancel_on_error = true);

    // Start and complete one installed package load. Loaders add precise
    // issues as they encounter them; successful registrations are associated
    // automatically through plugin_registration_scope().
    void begin_plugin_package_load(const std::string &package_id,
                                   const std::string &package_path,
                                   bool allows_no_plugins = false);
    void report_plugin_package_load_issue(const std::string &package_id,
                                          PluginPackageLoadIssue issue);
    void finish_plugin_package_load(const std::string &package_id);
    // Clear load diagnostics without unloading registered plugins.
    void clear_plugin_package_load_reports();
    // Return a borrowed report, or nullptr when the package has no report.
    const PluginPackageLoadReport *plugin_package_load_report(const std::string &package_id) const;
    // Return all package reports by borrowed reference.
    const std::map<std::string, PluginPackageLoadReport> &plugin_package_load_reports() const
        { return m_plugin_package_load_reports; }

    // The dynamic library loader establishes this scope while it calls a
    // plugin's register_plugin() export. It gives catalog registration a
    // package-relative root without exposing filesystem details through C.
    PluginRegistrationScope plugin_registration_scope(std::string package_root, bool external_plugin);
    // Register a package-relative catalog while a registration scope is active.
    int32_t register_translation_catalog(const char *domain, const char *locale_directory);
    // Return the registered translation catalogs by borrowed reference.
    const std::vector<TranslationCatalog> &translation_catalogs() const { return m_translation_catalogs; }

    // Add one plugin UI fragment to a target .ui file.
    //
    // target_file + fragment_id is de-duplicated so an installed layout or an
    // alternative plugin can say "this logical fragment is already provided".
    bool add_ui_fragment(const char *target_file,
                         const char *fragment_id,
                         const char *content,
                         int32_t priority);
    // Return a sorted value snapshot of fragments registered for one UI file.
    std::vector<PluginUiFragment> ui_fragments_for_file(const std::string &target_file) const;
    // Merge registered fragments into a base layout using default filtering.
    std::string merged_ui_layout(const std::string &target_file, const std::string &base_content) const;
    // Merge registered fragments while excluding already implemented fragment IDs.
    std::string merged_ui_layout(const std::string &target_file,
                                 const std::string &base_content,
                                 const std::unordered_set<std::string> &implemented_fragment_ids) const;
    // GUI activation rules: registration, per-setting activation groups,
    // queued settings and evaluation live in the registry.
    GuiRuleRegistry &gui_rules() { return m_gui_rule_registry; }
    const GuiRuleRegistry &gui_rules() const { return m_gui_rule_registry; }
    // Owner attributed to rules registered through the C API while a plugin is
    // initializing; "core" outside plugin initialization.
    std::string current_gui_rule_owner() const;

    // Create a bridge-detector service handle from an ABI input.
    bridge_detector_instance create_bridge_detector(const bridge_detector_create_input &input);
    // Run the slicing pipeline synchronously for one print.
    void slice(Print &print_to_slice);

    // Export the already-sliced print through the G-code side of the step
    // pipeline. The path_template is resolved with Print::output_filepath()
    // before STEP_GCODE runs, so the selected G-code plugin receives the exact
    // file path it must create.
    std::string export_gcode(Print &print_to_export,
                             const std::string &path_template,
                             GCodeProcessorResult *result,
                             ThumbnailsGeneratorCallback thumbnail_cb = nullptr);

    Orchestrator(const Orchestrator&) = delete;
    Orchestrator& operator=(const Orchestrator&) = delete;
    Orchestrator(Orchestrator&&) = delete;
    Orchestrator& operator=(Orchestrator&&) = delete;

    // Access storage owned by registered plugins; clearing it invalidates exported handles.
    std::map<Plugin *, PluginStorage> &plugin_storage() { return m_plugin_storage; }
    // Read-only access to plugin-owned storage.
    const std::map<Plugin *, PluginStorage> &plugin_storage() const { return m_plugin_storage; }
    // Build the host context used by callbacks for one step and plugin.
    plugin_host_context prepare_plugin_host_context(slicing_step_t step,
                                                    Plugin *plugin,
                                                    Print *print = nullptr);
    // Build the run context; the optional host context must outlive the run.
    plugin_run_context prepare_plugin_run_context(slicing_step_t step,
                                                  Plugin *plugin,
                                                  plugin_host_context *host_context = nullptr);
    // Remember that this plugin prepared state for the given Print and step, so
    // a later replay can ask it to release that state.
    void track_plugin_setup(Plugin &plugin, Print &print, slicing_step_t step);
    // Ask plugins that prepared state for this Print at or after
    // first_invalid_step to release it. Returns false when a callback failed;
    // failed entries stay registered for a retry.
    bool clear_plugin_participants(Print &print, slicing_step_t first_invalid_step);
    // Queue a plugin diagnostic for the GUI thread. The message is copied so C,
    // C++ and Python plugins may pass temporary buffers safely.
    void add_plugin_message(PluginMessageLevel level,
                            const Plugin *plugin,
                            slicing_step_t step,
                            const char *message,
                            int32_t diagnostic_id = PLUGIN_ERROR_OK);
    // Drain queued plugin diagnostics. The Plater uses this to turn plugin
    // warnings and errors into ImGui notifications without opening modal dialogs.
    std::vector<PluginMessage> consume_plugin_messages();
    // Copy one UTF-8 diagnostic into the process-lifetime error registry and
    // return its id, or PLUGIN_ERROR_GENERAL when the message cannot be stored.
    int32_t store_error_message(const char *message);
    // Resolve a diagnostic id: static text for the reserved ids, registered text
    // for dynamic ids, generic text for anything unknown. Never fails and never
    // borrows registry storage.
    std::string error_message(int32_t id) const;
    // Return whether cancellation was requested by the host.
    bool is_plugin_cancelled() const;
    // Request cancellation; plugin code must observe it at a cancellation point.
    void request_plugin_cancel();
    // Clear cancellation before starting an independent operation.
    void reset_plugin_cancel();
    // Initialize active plugins after registration and activation are finalized.
    void initialize_plugins();
    // Register a custom property descriptor and return its runtime type ID.
    slic3r_property_type register_property(const char *namespaced_name,
                                           uint32_t byte_count,
                                           uint32_t alignment);
    // Find a property descriptor by runtime type; the pointer is borrowed.
    const PropertyInfo *property_info(slic3r_property_type type) const;
    // Find a property descriptor by namespaced name; the pointer is borrowed.
    const PropertyInfo *property_info(const char *namespaced_name) const;
    // Register a custom extrusion property and return its runtime type ID.
    extrusion_property_type register_custom_extrusion_property(const char *namespaced_name,
                                                               uint32_t byte_count,
                                                               uint32_t alignment);
    // Find a custom extrusion property by runtime type; the pointer is borrowed.
    const CustomExtrusionPropertyInfo *custom_extrusion_property_info(extrusion_property_type type) const;
    // Find a custom extrusion property by namespaced name; the pointer is borrowed.
    const CustomExtrusionPropertyInfo *custom_extrusion_property_info(const char *namespaced_name) const;
    // Register or retrieve a runtime ID within one semantic ID namespace.
    orchestrator_runtime_id register_runtime_id(const char *id_type,
                                                const char *namespaced_name);
    // Resolve a runtime ID without creating a new registration.
    const std::string *runtime_id_name(const char *id_type,
                                      orchestrator_runtime_id id) const;
    // Register metadata for one generic facets annotation.
    bool register_generic_facets_annotation(GenericFacetsAnnotationDefinition def);
    // Return generic facet annotations by borrowed reference.
    const std::vector<GenericFacetsAnnotationDefinition> &generic_facets_annotations() const { return m_generic_facets_annotations; }

private:
    // One bidirectional intern table per semantic id type. Keeping this host
    // detail private prevents callers from bypassing static reservations or
    // assigning their own numeric values.
    struct RuntimeIdRegistry
    {
        std::map<std::string, orchestrator_runtime_id> ids_by_name;
        std::map<orchestrator_runtime_id, std::string> names_by_id;
        orchestrator_runtime_id next_dynamic_id = ORCHESTRATOR_RUNTIME_ID_DYNAMIC_BEGIN;
    };

    struct PluginRegistrationSource
    {
        std::string package_root;
        std::string package_id;
        bool external_plugin = false;
    };

    // Construct the empty process-wide registry.
    Orchestrator();
    // Close one registration scope; the caller must own an active scope.
    void end_plugin_registration_scope();
    // Resolve an optional plugin domain to the host domain.
    std::string resolved_translation_domain(const char *requested_domain) const;
    // Check whether a domain is available from the given package root.
    bool is_translation_domain_available(const std::string &domain, const std::string &package_root) const;
    // Reserve a host-defined runtime ID and reject conflicting mappings.
    bool register_static_runtime_id(const char *id_type,
                                    const char *namespaced_name,
                                    orchestrator_runtime_id id);

    std::vector<std::unique_ptr<Plugin>> m_registered_plugins;
    std::unordered_set<Plugin *> m_active_plugins;
    std::map<std::string, std::string> m_blocked_plugin_activations;
    std::map<std::string, std::set<std::string>> m_activation_groups;
    std::map<slicing_step_t, std::vector<Plugin *>> m_plugins_by_step;
    std::map<Plugin *, PluginStorage> m_plugin_storage;
    std::vector<PluginUiFragment> m_ui_fragments;
    uint64_t m_next_ui_fragment_order { 0 };
    GuiRuleRegistry m_gui_rule_registry;
    std::map<std::string, ConfigOptionOwner> m_config_option_owners;
    std::map<std::string, std::vector<OptionDefPatch>> m_option_def_patches;
    std::vector<TranslationCatalog> m_translation_catalogs;
    std::vector<PluginRegistrationSource> m_plugin_registration_sources;
    std::map<std::string, PluginPackageLoadReport> m_plugin_package_load_reports;
    std::vector<PropertyInfo> m_custom_property_infos;
    slic3r_property_type m_next_custom_property_type { SLIC3R_PROPERTY_TYPE_CUSTOM_BEGIN };
    std::map<slicing_step_t, DynamicStepInfo> m_dynamic_step_infos;
    uint32_t m_next_dynamic_step { uint32_t(SLICING_STEP_CUSTOM_BEGIN) };
    std::map<std::string, RuntimeIdRegistry> m_runtime_id_registries;
    std::vector<GenericFacetsAnnotationDefinition> m_generic_facets_annotations;
    std::atomic_bool m_plugin_cancel_requested { false };
    std::mutex m_plugin_messages_mutex;
    std::vector<PluginMessage> m_plugin_messages;
    // Process-lifetime diagnostics registered through store_error_message().
    // Entries are append-only and an id is never reused, so concurrent callbacks
    // cannot overwrite each other's message.
    mutable std::mutex m_error_messages_mutex;
    std::vector<std::string> m_error_messages;
    int32_t m_next_error_message_id { PLUGIN_ERROR_FIRST_DYNAMIC };
    Plugin *m_initializing_plugin { nullptr };
    bool m_initializing_plugin_failed { false };
    std::string m_initializing_plugin_failure;
};

// Own plugin-created objects while keeping their addresses stable across vector growth.
template<class T> class StableOwnedVector
{
public:
    using Storage = std::vector<std::unique_ptr<T>>;
    using iterator = typename Storage::iterator;
    using const_iterator = typename Storage::const_iterator;

    // Construct and retain one object; the returned reference is borrowed.
    template<class... Args> T &emplace_back(Args&&... args) {
        m_items.emplace_back(std::make_unique<T>(std::forward<Args>(args)...));
        return *m_items.back();
    }

    // Insert an already-owned object and return a borrowed reference.
    T &push_back(std::unique_ptr<T> item) {
        assert(item != nullptr);
        m_items.push_back(std::move(item));
        return *m_items.back();
    }

    // Iterators follow vector invalidation rules; erase and clear invalidate affected iterators.
    iterator begin() { return m_items.begin(); }
    iterator end() { return m_items.end(); }
    const_iterator begin() const { return m_items.begin(); }
    const_iterator end() const { return m_items.end(); }

    // Erase one object and return the next iterator.
    iterator erase(iterator it) { return m_items.erase(it); }
    // Destroy all objects and invalidate their references.
    void clear() { m_items.clear(); }

private:
    Storage m_items;
};

class PluginStorage
{
public:
    // Start with no tracked plugin allocations.
    PluginStorage();
    // Release every allocation still owned by this storage.
    ~PluginStorage();

    // Retain an allocation and its exported handle together under the storage lock.
    // Configs supply their adjusted ConfigBase handle; other objects use their own address.
    template<class T> T *store(std::unique_ptr<T> item, void *handle = nullptr)
    {
        if (item == nullptr)
            return nullptr;
        T *object = item.get();
        if (handle == nullptr)
            handle = object;
        const std::lock_guard<std::mutex> lock(m_mutex);
        generic_storage.insert(handle);
        try {
            if constexpr (std::is_same_v<T, Polyline>)
                polylines.push_back(std::move(item));
            else if constexpr (std::is_same_v<T, Polygon>)
                polygons.push_back(std::move(item));
            else if constexpr (std::is_same_v<T, ExPolygon>)
                expolygons.push_back(std::move(item));
            else if constexpr (std::is_same_v<T, std::vector<Polyline>>)
                polyline_collections.push_back(std::move(item));
            else if constexpr (std::is_same_v<T, Polygons>)
                polygon_collections.push_back(std::move(item));
            else if constexpr (std::is_same_v<T, std::vector<ExPolygon>>)
                expolygon_collections.push_back(std::move(item));
            else if constexpr (std::is_same_v<T, SurfaceCollection>)
                surface_collections.push_back(std::move(item));
            else if constexpr (std::is_same_v<T, ExtrusionEntity>)
                extrusions.push_back(std::move(item));
            else if constexpr (std::is_same_v<T, DynamicConfig>)
                configs.push_back(std::move(item));
            else if constexpr (std::is_same_v<T, TriangleMesh>)
                triangle_meshes.push_back(std::move(item));
            else {
                static_assert(std::is_same_v<T, ApiClipper::ClipperShapes>, "Unsupported plugin storage type");
                clipper_shapes.push_back(std::move(item));
            }
        } catch (...) {
            generic_storage.erase(handle);
            throw;
        }
        return object;
    }

    // Release all tracked allocations; previously returned pointers become invalid.
    void clear();
    // Return the number of currently tracked allocations under the storage lock.
    size_t size() const;
    // Test whether an exported handle is tracked under the storage lock.
    bool contains(void *ptr) const;
    // Release one allocation under the storage lock; unknown pointers are rejected.
    bool free(void *ptr);
    // Copy the stored extrusion pointers under the lock for inspection while their owners remain alive.
    std::vector<const ExtrusionEntity *> stored_extrusions() const;

private:
    // TODO: Replace this temporary lock with private storage per object, location or sub-plugin call; keep setup/clear and print-wide storage persistent.
    // See the planned rework in doc/development/plugin-data-lifetimes.md.
    mutable std::mutex m_mutex;

    // Stored objects live behind unique_ptr so their addresses stay stable even
    // if the owning vector reallocates. Plugins may keep these handles until
    // storage_free() or clear().
    StableOwnedVector<Polyline> polylines;
    StableOwnedVector<Polygon> polygons;
    StableOwnedVector<ExPolygon> expolygons;
    StableOwnedVector<std::vector<Polyline>> polyline_collections;
    StableOwnedVector<Polygons> polygon_collections;
    StableOwnedVector<std::vector<ExPolygon>> expolygon_collections;
    StableOwnedVector<SurfaceCollection> surface_collections;
    StableOwnedVector<ExtrusionEntity> extrusions;
    StableOwnedVector<DynamicConfig> configs;
    StableOwnedVector<TriangleMesh> triangle_meshes;
    std::vector<std::unique_ptr<ApiClipper::ClipperShapes>> clipper_shapes;
    std::unordered_set<void *> generic_storage;

};

} // namespace Slic3r


#endif // slic3r_Orchestrator_hpp_
