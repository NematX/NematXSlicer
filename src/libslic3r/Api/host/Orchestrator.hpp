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
#include <unordered_map>
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
#include "StepSelectorTraits.hpp"

/*
This header exposes plugin registration, selection settings and job execution.
The host is the slicer application. Orchestrator is its process-wide manager
of plugins, components that perform jobs at processing stages called steps.

Use instance(), plugin_registration_scope() and register_plugin() to introduce
instances under an installed package directory. PluginRegistrationScope ends
that association when destroyed. Registration does not enable an instance.

An exclusive group holds alternative plugins for one step. Its selector is
a setting naming a member. ExclusiveGroupDefinition combines text and package
identity with StepSelectorTraits, its placement and selection rules. Register
descriptions, enable members, then elect_exclusive_group_definitions() and
create_selector_print_config() retain descriptions and create settings.
finalize_selector_option() filters choices without replacing their text.
exclusive_group_definition_or_default() reads the retained description;
its untranslated flag requires verbatim display without catalogue lookup.

initialize_plugins() reads initialization result codes before execute_plugin()
prepares and runs callable jobs. Contexts passed to plugin functions borrow
application data for those calls only. Package reports and execution diagnostics
distinguish non-fatal warnings from failures.

For concurrent prints, create one RunContext per execution. It owns the progress
counters and stop state. Hold the RunScope returned by enter_run(context, &print)
until the execution's worker threads have finished. Destroying the scope restores
the previous thread association. It does not free plugin allocations.

PluginStorage remains shared. Print asks clear(print, storage) to release its
cached data when replaying from a saved pipeline boundary or during destruction.

For file export, the host first asks the selected exporter whether it accepts
the configuration and firmware. That QUERY returns compatibility information
without writing output. EXPORT then uses a completed G-code reference and an
export-specific configuration. The reference belongs to the print; it is not
the destination file and must remain unchanged by the exporter.

The export entry points connect that two-operation contract to plugin execution.
They are separate from plugin registration and ordinary geometry processing:
choosing a file format is not a request to recompute sliced geometry.
*/

namespace Slic3r {

class Orchestrator;
class ConfigOptionDef;
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
    // Borrow the Print's RunContext, or the thread's context when no Print is supplied.
    // Copies keep the same pointer. Its owner must keep it alive until all callbacks return.
    void *run_context = nullptr;
    // Borrowed invocation-local output for one payload; report_error writes here instead of queueing a GUI message.
    std::string *reported_error = nullptr;
};

namespace Slic3r {

// Identity of the built-in exporter and the printer-profile key that selects an exporter.
inline constexpr const char *k_default_gcode_exporter_id = "gcode.exporter.simple";
inline constexpr const char *k_gcode_exporter_option_key = "gcode_exporter_plugin";

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
    // Non-fatal package diagnostics; warnings do not change compatibility or the load result.
    std::vector<std::string> warnings;
};

class Orchestrator
{
public:
    // First known cause of a plugin stop; registered Plugin pointers remain valid for the process lifetime.
    struct PluginStopCause
    {
        enum class Kind { Unknown, Plugin, Host };
        Kind kind = Kind::Unknown;
        const Plugin *plugin = nullptr;
    };

    // Own the progress counters, stop flag and first known stop cause for one execution.
    // Keep this object alive until all RunScopes and callbacks using it have finished.
    class RunContext
    {
    public:
        // Create an initially healthy execution with no progress counters allocated.
        RunContext();
        // Release progress counters after every callback using them has returned.
        ~RunContext();
        RunContext(const RunContext &) = delete;
        RunContext &operator=(const RunContext &) = delete;

        // Stop this execution, retaining the first known cause; a null plugin adds no cause.
        void request_plugin_cancel(const Plugin *failed_plugin = nullptr) noexcept;
        // Stop this execution because the host failed, unless a cause is already recorded.
        void request_plugin_cancel_after_host_failure() noexcept;
        // Read the stop flag without taking the diagnostic mutex.
        bool plugin_cancel_requested() const noexcept;
        // Copy the first known cause under the diagnostic mutex.
        PluginStopCause plugin_stop_cause() const;
        // Clear the flag and cause between phases, never while workers are running.
        void reset_plugin_cancel();

    private:
        struct State;
        std::unique_ptr<State> state_;
        friend class Orchestrator;
    };

    // Associate a RunContext with this thread and optionally with a Print.
    // Destruction restores the previous thread association. Plugin allocations stay alive.
    class RunScope
    {
    public:
        RunScope(const RunScope &) = delete;
        RunScope &operator=(const RunScope &) = delete;
        // Restore the previous thread association when this scope ends.
        ~RunScope();

    private:
        friend class Orchestrator;
        // Borrow context for this thread. A non-null print also registers that Print with the context.
        RunScope(Orchestrator &orchestrator, RunContext *context, Print *print);
        Orchestrator *orchestrator_;
        RunContext *context_;
        RunContext *previous_context_;
        Print *print_;
        bool owns_registration_ = false;
    };

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

    struct GCodeExporterSelection
    {
        std::string plugin_id;
        std::string extension;

        // Tell whether a typed suffix (with its dot, any case) is this exporter's extension or a text alias of the built-in exporter.
        bool accepts_extension(const std::string &suffix) const;
        // Return a unique path in the system temporary directory, `prefix` + random part + the exporter extension; nothing is created.
        std::string temporary_path(const std::string &prefix) const;
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


    // Validate and copy a plugin definition; compatible duplicates are unrestricted outside plugin initialization.
    // During initialization, duplicates require the same plugin owner or the same non-empty exclusive group.
    // Copy under the initializer's package identity; reserved selector keys are rejected and initialization failure is recorded.
    // Inputs remain caller-owned, print data is unchanged, and allocations may throw.
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
    // Recompute selector defaults without choosing new descriptions; rebuilt storage is application-owned and allocations may throw.
    void finalize_plugin_option_defs();

    // Check table size and equal execution priority among alternatives, then register a borrowed plugin with copied package identity.
    // Rejection leaves it unregistered; print data is unchanged, and allocations may throw.
    bool register_plugin(plugin_instance plugin);

    // Register and query plugin-defined service steps. Runtime ids are scoped
    // to this orchestrator; callers exchange the stable name, not the number.
    slicing_step_t register_step(const char *namespaced_name,
                                 slicing_step_t invalidates_step);
    // Look up runtime step metadata by numeric ID.
    const DynamicStepInfo *step_info(slicing_step_t step) const;
    // Look up runtime step metadata by stable namespaced name.
    const DynamicStepInfo *step_info(const char *namespaced_name) const;

    // Copy a built-in selector declaration; return 1 if added, 0 if identical, -1 if invalid, or -2 if conflicting.
    // Existing declarations and print data are unchanged on rejection; allocations may throw.
    int32_t register_builtin_step_selector(slicing_step_t step, StepSelectorTraits traits);
    // Borrow retained rules for a named group, then stage-wide rules, or null when neither exists.
    // Any election or register_builtin_exclusive_group() call may invalidate the borrowed pointer; temporary-key allocation may throw.
    const StepSelectorTraits *step_selector(slicing_step_t step, const std::string &exclusive_group) const;
    // Return whether this exact step and group has a built-in declaration, without falling back to the whole step.
    bool step_selector_is_builtin(slicing_step_t step, const std::string &exclusive_group) const;
    // Return the layout filenames accepted by selector declarations and searched for UI fragments.
    static const std::vector<std::string> &ui_layout_files();

    enum class SelectorIdentityKind { WholeStep, PluginGroup };
    struct SelectorIdentity {
        SelectorIdentityKind kind = SelectorIdentityKind::PluginGroup;
        slicing_step_t step = STEP_NONE;
        std::string group;
    };
    struct ExclusiveGroupDefinition {
        StepSelectorTraits traits;
        bool untranslated = false;
        std::string label, tooltip, translation_domain, plugin_id, package_id, package_root;
        // Optional host layout line text; empty preserves the caller's whole-step identity line.
        std::string line_label;
    };
    // Copy an application-supplied group description and reserve its key together; return added, identical or rejected status.
    // Plugins and print data are unchanged; allocation failures preserve existing descriptions.
    int32_t register_builtin_exclusive_group(slicing_step_t step, ExclusiveGroupDefinition definition);
    // Check the named member's loading package and copy its group description.
    // The application owns copied text, package identity and key; invalid input changes nothing, and allocations may throw.
    int32_t record_exclusive_group_declaration(slicing_step_t step, ExclusiveGroupDefinition definition);
    // Borrow the retained group description without choosing another member's text.
    // Any election or register_builtin_exclusive_group() call may invalidate the pointer; activation is not checked again, and key allocation may throw.
    const ExclusiveGroupDefinition *exclusive_group_definition(slicing_step_t step, const std::string &group) const;
    // Copy a retained description, or default group text and setting rules when none exists.
    // No member is chosen again; group ids supply missing labels, returned strings are owned, and allocations may throw.
    ExclusiveGroupDefinition exclusive_group_definition_or_default(slicing_step_t step, const std::string &group) const;
    // Retain group descriptions, preferring enabled members that supplied them before considering disabled members.
    // Text without a package catalogue stays verbatim; swap completed copies so allocation failure preserves previous descriptions.
    void elect_exclusive_group_definitions();
    // Retain one group's preferred description without changing other groups.
    // Use the complete build's ordering and translation policy; allocation failures preserve its previous description.
    void elect_exclusive_group_definition(slicing_step_t step, const std::string &group);
    // Record which stage and group own a selector key and whether it permits automatic selection.
    // Copy strings without taking plugin ownership; invalid identities throw, and allocation failure removes an incomplete insertion.
    void register_selector_option(const std::string &key, const SelectorIdentity &identity, bool auto_enabled);
    // Remove unavailable selector choices and update the default without changing the retained description.
    // Preserve choice order in the caller-owned description; text and application records are unchanged, and allocations may throw.
    void finalize_selector_option(ConfigOptionDef &definition, const SelectorIdentity &identity, bool auto_enabled) const;
    // Check whether a registered selector permits automatic selection without changing the borrowed key or stored data.
    bool selector_auto_enabled(const std::string &key) const;
    // Copy a setting key/value condition and preferred member in registration order.
    // Return added, identical or rejected status; rejection records initializer failure, earlier choices survive, and allocations may throw.
    int32_t register_selector_auto_choice(SelectorAutoChoice choice);
    // Borrow stored automatic-selection choices in registration order without mutation or failure; insertion may invalidate iterators.
    const std::vector<SelectorAutoChoice> &selector_auto_choices() const;
    // Copy the current initializer's first failure and queue its explanation once; calls outside initialization do nothing.
    // Caller ownership and plugin ownership are unchanged, and allocation failures cannot escape.
    void record_plugin_initialization_failure(const char *reason) noexcept;
    // Create a selector using its retained description, package identity and translation policy.
    // Missing catalogues keep text verbatim and queue a warning; failure removes new selector records but preserves key reservations.
    option_def_error_code create_selector_print_config(const raw_config_option_def &def,
        const SelectorIdentity &identity, bool auto_enabled);

    // Execute setup/setup_run/run for caller-owned payloads; reported_error is call-local and requires at most one payload.
    raw_plugin_execution_status execute_plugin(Plugin &plugin,
                                               Print *print,
                                               const raw_plugin_run_payload *payloads,
                                               uint32_t run_count,
                                               bool cancel_on_error = true,
                                               std::string *reported_error = nullptr);

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

    // Resolve and query the exporter chosen by a borrowed configuration copy, returning its ID and extension or throwing.
    GCodeExporterSelection select_gcode_exporter(const DynamicPrintConfig &config);
    // Export a borrowed reference to a destination and atomically publish it; failures preserve both inputs and throw.
    std::string export_print(Print &print,
                             const DynamicPrintConfig &config,
                             const std::string &reference_path,
                             const std::string &destination_path,
                             const GCodeExporterSelection &selection);
    // Generate and reread a private reference beside the requested destination, export its finalized path, and remove the reference.
    std::string export_gcode_cli(Print &print, const std::string &requested_output_path);

    Orchestrator(const Orchestrator&) = delete;
    Orchestrator& operator=(const Orchestrator&) = delete;
    Orchestrator(Orchestrator&&) = delete;
    Orchestrator& operator=(Orchestrator&&) = delete;

    // Associate context with this thread and the optional Print.
    // Keep the returned scope alive until all execution workers have finished.
    RunScope enter_run(RunContext &context, Print *print = nullptr);
    // Use host_context's execution on this thread until the returned scope ends.
    // This does not register a Print or take ownership of the context.
    RunScope bind_run(const plugin_host_context *host_context);
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
    // Read the stop flag. The stop functions below use the same context selection:
    // a non-null Print selects its registered execution; no Print selects this thread's execution.
    // If that lookup finds no context, use the host's shared stop state.
    bool is_plugin_cancelled(const Print *print = nullptr) const;
    // The three request functions never throw. A locking failure still requests a stop,
    // but may leave its cause unknown. An already recorded cause is preserved.
    // Request a stop without adding an error cause, for example a user cancellation.
    void request_plugin_cancel(const Print *print = nullptr) noexcept;
    // Request a stop after failed_plugin reports an error. Keep the first known cause.
    void request_plugin_cancel(const Plugin &failed_plugin, const Print *print = nullptr) noexcept;
    // Request a stop after a host failure. Keep the first known cause.
    void request_plugin_cancel_after_host_failure(const Print *print = nullptr) noexcept;
    // Return the first known cause recorded since the last reset (Unknown when none).
    PluginStopCause plugin_stop_cause(const Print *print = nullptr) const;
    // Clear the selected stop flag and recorded cause before the next phase or run.
    // Call only after the previous workers have finished.
    void reset_plugin_cancel(const Print *print = nullptr);
    // Initialize enabled plugins with borrowed storage, resolve diagnostic ids and disable failures without repeating recorded rejections.
    // Then validate required settings; plugin ownership is unchanged, and allocation failures may propagate.
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
    // Return the supplied Print's context, or this thread's context when print is null.
    // The pointer is borrowed. Return null when the selected association is absent.
    RunContext *find_run_context(const Print *print) const;
    mutable std::mutex m_run_contexts_mutex;
    std::map<const Print *, RunContext *> m_run_contexts;
    // Protect access to the map of plugin storages during execution. Startup is serialized separately.
    // Each PluginStorage protects its allocations after this map lock is released.
    std::mutex m_plugin_storage_map_mutex;
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

    // Construct the process-wide registry with copied built-in selector declarations.
    Orchestrator();
    // Close one registration scope; the caller must own an active scope.
    void end_plugin_registration_scope();
    // Resolve an optional plugin domain to the host domain.
    std::string resolved_translation_domain(const char *requested_domain) const;
    // Copy and log a warning under the member's recorded package without a live loading scope, ignoring duplicate text.
    // The plugin stays borrowed and load errors are unchanged; allocations may throw.
    void report_internal_plugin_warning(const std::string &plugin_id, const std::string &message);
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
    std::map<std::string, PluginRegistrationSource> m_plugin_sources;
    std::map<std::string, PluginPackageLoadReport> m_plugin_package_load_reports;
    std::vector<PropertyInfo> m_custom_property_infos;
    slic3r_property_type m_next_custom_property_type { SLIC3R_PROPERTY_TYPE_CUSTOM_BEGIN };
    std::map<slicing_step_t, DynamicStepInfo> m_dynamic_step_infos;
    // Only declarations installed by built-in step modules belong in this registry.
    std::map<std::pair<slicing_step_t, std::string>, StepSelectorTraits> m_step_selectors;
    std::map<std::pair<slicing_step_t, std::string>, ExclusiveGroupDefinition> m_host_groups;
    std::map<std::pair<slicing_step_t, std::string>, std::map<std::string, ExclusiveGroupDefinition>> m_group_declarations;
    std::map<std::pair<slicing_step_t, std::string>, ExclusiveGroupDefinition> m_elected_group_definitions;
    std::map<std::string, SelectorIdentity> m_selector_key_owners;
    std::set<std::string> m_auto_selector_keys;
    std::vector<SelectorAutoChoice> m_selector_auto_choices;
    // Borrow the preferred enabled member's description, or a registered member's description if none is enabled.
    // Application descriptions take precedence; stored data is unchanged, and temporary allocations may throw.
    const ExclusiveGroupDefinition *active_member_group_definition(slicing_step_t step, const std::string &group) const;
    // Check a description and reserve its explicit or generated key without creating a setting.
    // Stage-wide settings reserve no empty-group key; conflicts return errors, and allocation failures preserve existing reservations.
    int32_t reserve_selector_key(slicing_step_t step, const StepSelectorTraits &traits, bool public_declaration);
    // Copy a checked setting description using its package identity and requested translation policy.
    // explicit_domain keeps the catalogue name and untranslated forbids lookup; conflicts return errors without replacement, and allocations may throw.
    option_def_error_code create_print_config_impl(const raw_config_option_def *def, const std::string &package_root, bool explicit_domain = false, bool untranslated = false);
    uint32_t m_next_dynamic_step { uint32_t(SLICING_STEP_CUSTOM_BEGIN) };
    std::map<std::string, RuntimeIdRegistry> m_runtime_id_registries;
    std::vector<GenericFacetsAnnotationDefinition> m_generic_facets_annotations;
    std::atomic_bool m_plugin_cancel_requested { false };
    // Protects m_plugin_stop_cause. A request records the cause, then sets the flag, inside this mutex;
    // is_plugin_cancelled() only reads the atomic flag and never takes it.
    mutable std::mutex m_plugin_stop_cause_mutex;
    PluginStopCause m_plugin_stop_cause;
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

// Owns every object a plugin creates through the C API (polylines, extrusions, configs, meshes, ...)
// and hands out stable pointers ("handles") to them.
//
// Mechanism: one registry maps each exported handle to an Entry that owns the object. The
// handle is the object's own address, except for configs, whose handle is the adjusted
// ConfigBase subobject address (see ApiHost::to_config_handle). Registry bookkeeping (lookup,
// insertion, removal) is average O(1) whatever the object type and the number of stored objects,
// so releasing N objects costs O(N) bookkeeping, which matters when a large structure (e.g.
// thousands of tree volumes) releases all its handles at once. This excludes the cost of destroying
// an object itself, which is proportional to its content (an extrusion tree, a PolyTree).
// Addresses stay stable because objects are heap-allocated and the registry nodes never move.
//
// Entries of ExtrusionEntity are additionally chained in insertion order so that
// stored_extrusions() can return them in storage order.
//
// Threading: one mutex protects the registry and the extrusion chain. Objects are destroyed
// after the mutex is released, so a destructor never runs under the lock.
class PluginStorage
{
public:
    // Start with no tracked plugin allocations.
    PluginStorage();
    // Release every allocation still owned by this storage.
    ~PluginStorage();

    // Take ownership of an allocation and register its exported handle under the storage lock.
    // Configs supply their adjusted ConfigBase handle; other objects use their own address.
    // The handle must not be registered yet (asserted). If registration throws (allocation failure),
    // the registry is unchanged, the exception propagates and the item parameter, which still owns
    // the allocation, destroys it. On success item is empty after the move.
    template<class T> T *store(std::unique_ptr<T> item, void *handle = nullptr)
    {
        static_assert(std::disjunction_v<std::is_same<T, Polyline>, std::is_same<T, Polygon>, std::is_same<T, ExPolygon>,
                                         std::is_same<T, std::vector<Polyline>>, std::is_same<T, Polygons>,
                                         std::is_same<T, std::vector<ExPolygon>>, std::is_same<T, SurfaceCollection>,
                                         std::is_same<T, ExtrusionEntity>, std::is_same<T, DynamicConfig>,
                                         std::is_same<T, TriangleMesh>, std::is_same<T, ApiClipper::ClipperShapes>>,
                      "Unsupported plugin storage type");
        if (item == nullptr)
            return nullptr;
        T *object = item.get();
        if (handle == nullptr)
            handle = object;
        const std::lock_guard<std::mutex> lock(m_mutex);
        // The only step that can throw. Once the entry exists, ownership moves into it.
        const std::pair<std::unordered_map<void *, Entry>::iterator, bool> inserted = m_entries.try_emplace(handle);
        // A duplicated handle would drop this entry and, for an extrusion, link an already linked entry
        // a second time, corrupting the extrusion chain.
        assert(inserted.second);
        Entry &entry = inserted.first->second;
        // ExtrusionEntity and ClipperShapes have virtual destructors, so deleting through T is correct.
        // destroy is set before object because ~Entry calls it as soon as object is non-null.
        entry.destroy = [](void *owned) noexcept { delete static_cast<T *>(owned); };
        entry.object = item.release();
        if constexpr (std::is_same_v<T, ExtrusionEntity>)
            link_extrusion(entry);
        return object;
    }

    // Release all tracked allocations; previously returned pointers become invalid.
    void clear();
    // Return the number of currently tracked allocations under the storage lock.
    size_t size() const;
    // Test whether an exported handle is tracked under the storage lock.
    bool contains(void *ptr) const;
    // Release one allocation by its exported handle with average O(1) bookkeeping (the object's own
    // destruction cost comes on top). The entry is removed from the registry under the lock, then the
    // object is destroyed after unlocking. contains(), size() and
    // stored_extrusions() stop reporting it as soon as the lock is released; this does not protect a
    // reader that still uses the object while another callback frees it, which the run-context
    // contract forbids. A null, unknown or already freed handle is a programming error: it asserts
    // in Debug and returns false otherwise.
    bool free(void *ptr);
    // Copy the stored extrusion pointers, in storage order, under the lock for inspection while their
    // owners remain alive.
    std::vector<const ExtrusionEntity *> stored_extrusions() const;

private:
    // One owned object. Destroying the entry destroys the object, so a registry node that is
    // extracted or swapped out of the registry releases its object wherever the node dies.
    struct Entry
    {
        // Empty entry; store() fills destroy then object right after the registry node is created.
        Entry() = default;
        // Not copyable: two entries must never delete the same object.
        Entry(const Entry &) = delete;
        Entry &operator=(const Entry &) = delete;
        // Delete the owned object through its real type, if any; destroy must be set whenever object is.
        ~Entry()
        {
            if (object != nullptr)
                destroy(object);
        }

        // Owned object, deleted through its real type by `destroy`.
        void *object { nullptr };
        void (*destroy)(void *) noexcept { nullptr };
        // Neighbours in the insertion-ordered chain of ExtrusionEntity entries; unused (and
        // is_extrusion false) for other types. Registry nodes never move, so the pointers stay valid.
        Entry *prev_extrusion { nullptr };
        Entry *next_extrusion { nullptr };
        bool   is_extrusion { false };
    };

    // Append an entry at the tail of the extrusion chain; call with m_mutex held.
    void link_extrusion(Entry &entry);
    // Remove an entry from the extrusion chain in O(1); call with m_mutex held.
    void unlink_extrusion(Entry &entry);

    // TODO: Replace this temporary lock with private storage per object, location or sub-plugin call; keep setup/clear and print-wide storage persistent.
    // See the planned rework in doc/development/plugin-data-lifetimes.md.
    mutable std::mutex m_mutex;

    // Exported handle -> owning entry. Plugins may keep a handle until storage_free() or clear().
    std::unordered_map<void *, Entry> m_entries;
    // Ends of the insertion-ordered chain of ExtrusionEntity entries (both null when empty).
    Entry *m_first_extrusion { nullptr };
    Entry *m_last_extrusion { nullptr };
};

} // namespace Slic3r


#endif // slic3r_Orchestrator_hpp_
