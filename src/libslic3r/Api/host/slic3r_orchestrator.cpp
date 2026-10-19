///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <boost/log/trivial.hpp>
#include <boost/filesystem/path.hpp>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_extrusion_property.h"
#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"
#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/Print.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

#include "Orchestrator.hpp"
#include "Plugin.hpp"

/*
This file implements the C functions that register plugins, describe settings and run jobs.
The host is the slicer application. An orchestrator_handle is a borrowed
reference to its Orchestrator manager, not memory the caller may delete.

Call orchestrator_register_plugin() to introduce an instance. Package loaders
use orchestrator_begin_plugin_package_load(), register instances and resources,
then orchestrator_finish_plugin_package_load(). This interval records the
current installed package directory for resource ownership and load diagnostics.

An exclusive group contains alternatives for one processing stage; a selector
is its member-choice setting. orchestrator_register_exclusive_group() copies
descriptions into the manager. orchestrator_register_selector_auto_choice()
copies conditions used for automatic selection.

orchestrator_find_plugin() finds enabled instances for execution;
orchestrator_is_plugin_registered() also recognizes disabled instances.
Execution functions forward borrowed data to the manager. Error-reporting
functions copy explanations and contain exceptions at the C calling boundary.
orchestrator_send_log() writes text through the application's logger without
attributing a package failure. Initialization callbacks return stored error
identifiers rather than invoking a separate failure-reporting function.

Operation notes:
- orchestrator_register_exclusive_group
- Orchestrator::record_exclusive_group_declaration

Representative direct calls (branches are not a timeline):

    orchestrator_register_plugin()
        |-- Slic3r::Orchestrator::instance()
        `-- orchestrator->register_plugin()

    orchestrator_execute_plugin()
        |-- to_orchestrator()
        |-- host_orchestrator->owns_plugin()
        `-- host_orchestrator->execute_plugin()
*/

extern "C" {
// Check registered ids, including disabled plugins, without changing ownership or activation; invalid input or failure returns zero.
// The application is borrowed, and exceptions cannot cross C.
int32_t orchestrator_is_plugin_registered(orchestrator_handle *orch, const char *plugin_id);
// Log borrowed UTF-8 text at the requested severity; a null manager is allowed because logging is process-wide.
// Invalid inputs are ignored, and exceptions cannot cross C.
void orchestrator_send_log(orchestrator_handle *orch, raw_plugin_log_level severity, const char *message);
// Check and copy a group description for the borrowed application; return -1 for invalid input or -2 for other failures.
// Plugin ownership is unchanged, strings/package identity are retained only after checks, and exceptions cannot cross C.
int32_t orchestrator_register_exclusive_group(orchestrator_handle *orch, slicing_step_t step, const raw_exclusive_group_def *def);
// Copy an automatic-selection condition; rejection records failure of the current initializer without changing existing choices or geometry.
// The application and caller memory stay borrowed; exceptions return -2.
int32_t orchestrator_register_selector_auto_choice(orchestrator_handle *orch, slicing_step_t step,
    const char *group, const char *key, const char *value, const char *provider);
// Open a package loading record and retain its current-directory record until the matching finish call on this thread.
// Application/path are borrowed; copied records are owned here, slicing data is unchanged, and allocation failures are contained.
void orchestrator_begin_plugin_package_load(orchestrator_handle *orch, const char *package_root);
// Finish loading and remove the matching current-directory record even if reporting fails; inputs remain borrowed.
// Slicing data and plugin ownership are unchanged, and exceptions cannot cross C.
void orchestrator_finish_plugin_package_load(orchestrator_handle *orch, const char *package_root);
}

namespace {

// Keep each package's current-directory record on its loading thread until the matching finish call.
struct PackageRegistrationScope
{
    Slic3r::Orchestrator *host;
    std::string root;
    Slic3r::Orchestrator::PluginRegistrationScope scope;
};
thread_local std::vector<PackageRegistrationScope> package_registration_scopes;

struct GenericPropertyInfo
{
    slic3r_property_type type;
    const char *name;
    uint32_t byte_count;
    uint32_t alignment;
};

// Initialize and lend the process-owned immutable payload metadata table without arguments; the caller must not free it and static vector allocation failures propagate.
const std::vector<GenericPropertyInfo> &builtin_property_infos()
{
    static const std::vector<GenericPropertyInfo> infos = {
        { SLIC3R_PROPERTY_TYPE_EXTRUSION_ATTRIBUTES,      "slic3r.extrusion.attributes",      sizeof(c_extrusion_property_attributes),      alignof(c_extrusion_property_attributes) },
        { SLIC3R_PROPERTY_TYPE_EXTRUSION_SPEED,           "slic3r.extrusion.speed",           sizeof(c_extrusion_property_speed),           alignof(c_extrusion_property_speed) },
        { SLIC3R_PROPERTY_TYPE_EXTRUSION_MODIFIER,        "slic3r.extrusion.modifier",        sizeof(c_extrusion_property_modifier),        alignof(c_extrusion_property_modifier) },
        { SLIC3R_PROPERTY_TYPE_EXTRUSION_CUSTOM_GCODE,    "slic3r.extrusion.custom_gcode",    sizeof(c_extrusion_property_custom_gcode),    alignof(c_extrusion_property_custom_gcode) },
        { SLIC3R_PROPERTY_TYPE_EXTRUSION_SPECIAL_COMMAND, "slic3r.extrusion.special_command", sizeof(c_extrusion_property_special_command), alignof(c_extrusion_property_special_command) },
        { SLIC3R_PROPERTY_TYPE_EXTRUSION_OVERHANG,        "slic3r.extrusion.overhang",        sizeof(c_extrusion_property_overhang),        alignof(c_extrusion_property_overhang) },
        { SLIC3R_PROPERTY_TYPE_EXTRUSION_Z_OFFSET,        "slic3r.extrusion.z_offset",        sizeof(c_extrusion_property_z_offset),        alignof(c_extrusion_property_z_offset) },
        { SLIC3R_PROPERTY_TYPE_EXTRUSION_PERIMETER,       "slic3r.extrusion.perimeter",       sizeof(c_extrusion_property_perimeter),       alignof(c_extrusion_property_perimeter) },
        { SLIC3R_PROPERTY_TYPE_EXTRUSION_INFILL,          "slic3r.extrusion.infill",          sizeof(c_extrusion_property_infill),          alignof(c_extrusion_property_infill) },
        { SLIC3R_PROPERTY_TYPE_EXTRUSION_AXIS,            "slic3r.extrusion.axis",            sizeof(c_extrusion_property_extrusion_axis),  alignof(c_extrusion_property_extrusion_axis) },
        { SLIC3R_PROPERTY_TYPE_EXTRUSION_HEATER,          "slic3r.extrusion.heater",          sizeof(c_extrusion_property_heater),          alignof(c_extrusion_property_heater) },
        { SLIC3R_PROPERTY_TYPE_LAYER_SUPPORT,             "slic3r.layer.support",             sizeof(c_layer_support_property),             alignof(c_layer_support_property) },
        { SLIC3R_PROPERTY_TYPE_LAYER_BRIM,                "slic3r.layer.brim",                sizeof(c_layer_brim_property),                alignof(c_layer_brim_property) },
        { SLIC3R_PROPERTY_TYPE_LAYER_ADHESION,            "slic3r.layer.adhesion",            sizeof(c_layer_adhesion_property),            alignof(c_layer_adhesion_property) },
        { SLIC3R_PROPERTY_TYPE_LAYER_INSTANCE,            "slic3r.layer.instance",            sizeof(c_layer_instance_property),            alignof(c_layer_instance_property) },
    };
    return infos;
}

const GenericPropertyInfo *builtin_property_info(slic3r_property_type type)
{
    for (const GenericPropertyInfo &info : builtin_property_infos())
        if (info.type == type)
            return &info;
    return nullptr;
}

bool is_power_of_two(uint32_t value)
{
    return value != 0 && (value & (value - 1)) == 0;
}

Slic3r::Orchestrator *to_orchestrator(orchestrator_handle *orch)
{
    return orch == nullptr ? &Slic3r::Orchestrator::instance() :
                             reinterpret_cast<Slic3r::Orchestrator *>(orch);
}

const Slic3r::Orchestrator *to_orchestrator(const orchestrator_handle *orch)
{
    return orch == nullptr ? &Slic3r::Orchestrator::instance() :
                             reinterpret_cast<const Slic3r::Orchestrator *>(orch);
}

Slic3r::PluginPackageLoadErrorCode plugin_package_load_error_code(raw_plugin_package_load_error_code code)
{
    switch (code) {
    case RAW_PLUGIN_PACKAGE_LOAD_ERROR_PACKAGE_MISSING:
        return Slic3r::PluginPackageLoadErrorCode::PackageMissing;
    case RAW_PLUGIN_PACKAGE_LOAD_ERROR_LIBRARY_OPEN_FAILED:
        return Slic3r::PluginPackageLoadErrorCode::LibraryOpenFailed;
    case RAW_PLUGIN_PACKAGE_LOAD_ERROR_MISSING_ABI_EXPORT:
    case RAW_PLUGIN_PACKAGE_LOAD_ERROR_RESERVED_MISSING_API_VERSIONS_EXPORT:
        return Slic3r::PluginPackageLoadErrorCode::MissingAbiExport;
    case RAW_PLUGIN_PACKAGE_LOAD_ERROR_RESERVED_ABI_MISMATCH:
    case RAW_PLUGIN_PACKAGE_LOAD_ERROR_API_HEADER_VERSION_MISMATCH:
        return Slic3r::PluginPackageLoadErrorCode::ApiHeaderVersionMismatch;
    case RAW_PLUGIN_PACKAGE_LOAD_ERROR_MISSING_REGISTRATION_EXPORT:
        return Slic3r::PluginPackageLoadErrorCode::MissingRegistrationExport;
    case RAW_PLUGIN_PACKAGE_LOAD_ERROR_REGISTRATION_FAILED:
        return Slic3r::PluginPackageLoadErrorCode::RegistrationFailed;
    case RAW_PLUGIN_PACKAGE_LOAD_ERROR_NO_PLUGINS_REGISTERED:
        return Slic3r::PluginPackageLoadErrorCode::NoPluginsRegistered;
    case RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_RUNTIME_UNAVAILABLE:
        return Slic3r::PluginPackageLoadErrorCode::PythonRuntimeUnavailable;
    case RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_READ_FAILED:
        return Slic3r::PluginPackageLoadErrorCode::PythonReadFailed;
    case RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_COMPILE_FAILED:
        return Slic3r::PluginPackageLoadErrorCode::PythonCompileFailed;
    case RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_IMPORT_FAILED:
        return Slic3r::PluginPackageLoadErrorCode::PythonImportFailed;
    case RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_REGISTRATION_FAILED:
        return Slic3r::PluginPackageLoadErrorCode::PythonRegistrationFailed;
    case RAW_PLUGIN_PACKAGE_LOAD_ERROR_CONFIGURED_PLUGIN_MISSING:
        return Slic3r::PluginPackageLoadErrorCode::ConfiguredPluginMissing;
    case RAW_PLUGIN_PACKAGE_LOAD_ERROR_INVALID_PACKAGE:
    default:
        return Slic3r::PluginPackageLoadErrorCode::InvalidPackage;
    }
}

} // namespace

extern "C" {

/* Construct the generated header before writing caller storage, and catch every exception so none crosses the C ABI boundary. */
int32_t orchestrator_get_version_info(raw_slic3r_version_info *info)
{
    if (info == nullptr)
        return 0;

    const size_t struct_size = info->struct_size;
    constexpr size_t minimum_version_info_size =
        offsetof(raw_slic3r_version_info, generated_header) + sizeof(info->generated_header);
    if (struct_size < minimum_version_info_size)
        return 0;

    static thread_local std::string generated_header;
    try {
        generated_header = Slic3r::header_slic3r_generated();
        if (generated_header.empty())
            return 0;
    } catch (...) {
        return 0;
    }

// Field bounds prepare the API for fields appended after the stable version 3.4 prefix.
#define SET_VERSION_INFO_FIELD(field, value) \
    do { \
        if (offsetof(raw_slic3r_version_info, field) + sizeof(info->field) <= struct_size) \
            info->field = value; \
    } while (false)
    SET_VERSION_INFO_FIELD(application_name, SLIC3R_APP_NAME);
    SET_VERSION_INFO_FIELD(application_key, SLIC3R_APP_KEY);
    SET_VERSION_INFO_FIELD(application_command, SLIC3R_APP_CMD);
    SET_VERSION_INFO_FIELD(version, SLIC3R_VERSION);
    SET_VERSION_INFO_FIELD(version_full, SLIC3R_VERSION_FULL);
    SET_VERSION_INFO_FIELD(build_id, SLIC3R_BUILD_ID);
    SET_VERSION_INFO_FIELD(gcodeviewer_name, GCODEVIEWER_APP_NAME);
    SET_VERSION_INFO_FIELD(gcodeviewer_key, GCODEVIEWER_APP_KEY);
    SET_VERSION_INFO_FIELD(gcodeviewer_command, GCODEVIEWER_APP_CMD);
    SET_VERSION_INFO_FIELD(gcodeviewer_build_id, GCODEVIEWER_BUILD_ID);
    SET_VERSION_INFO_FIELD(based_on, SLIC3R_BASED_ON);
    SET_VERSION_INFO_FIELD(github_url, SLIC3R_GITHUB);
    SET_VERSION_INFO_FIELD(download_url, SLIC3R_DOWNLOAD);
    SET_VERSION_INFO_FIELD(documentation_url, SLIC3R_DOC_URL);
    SET_VERSION_INFO_FIELD(intro_url, SLIC3R_INTRO);
    SET_VERSION_INFO_FIELD(generated_header, generated_header.c_str());
#undef SET_VERSION_INFO_FIELD
    return 1;
}

int32_t orchestrator_is_plugin_registered(orchestrator_handle *orch, const char *plugin_id)
{
    if (orch == nullptr || plugin_id == nullptr || *plugin_id == '\0') return 0;
    try { return to_orchestrator(orch)->get_plugin(plugin_id) != nullptr ? 1 : 0; }
    catch (...) { return 0; }
}

void orchestrator_send_log(orchestrator_handle *orch, raw_plugin_log_level severity, const char *message)
{
    (void)orch;
    if (message == nullptr) return;
    try {
        switch (severity) {
        case RAW_PLUGIN_LOG_FATAL: BOOST_LOG_TRIVIAL(fatal) << message; break;
        case RAW_PLUGIN_LOG_ERROR: BOOST_LOG_TRIVIAL(error) << message; break;
        case RAW_PLUGIN_LOG_WARNING: BOOST_LOG_TRIVIAL(warning) << message; break;
        case RAW_PLUGIN_LOG_INFO: BOOST_LOG_TRIVIAL(info) << message; break;
        case RAW_PLUGIN_LOG_DEBUG: BOOST_LOG_TRIVIAL(debug) << message; break;
        case RAW_PLUGIN_LOG_TRACE: BOOST_LOG_TRIVIAL(trace) << message; break;
        default: break;
        }
    } catch (...) {}
}

void orchestrator_register_plugin(orchestrator_handle *orch, plugin_instance plugin) {
    Slic3r::Orchestrator *orchestrator = orch == nullptr ? &Slic3r::Orchestrator::instance() :
                                                           reinterpret_cast<Slic3r::Orchestrator *>(orch);
    if (orchestrator != nullptr) {
        orchestrator->register_plugin(plugin);
    }
}

void orchestrator_register_plugin_from_package(orchestrator_handle *orch,
                                               plugin_instance plugin,
                                               const char *package_root)
{
    if (package_root == nullptr || package_root[0] == '\0')
        return;
    Slic3r::Orchestrator *orchestrator = to_orchestrator(orch);
    if (orchestrator == nullptr)
        return;

    Slic3r::Orchestrator::PluginRegistrationScope registration_scope(
        orchestrator->plugin_registration_scope(package_root, true));
    orchestrator->register_plugin(plugin);
}

// Keep registration failures inside the C barrier and identify the offending
// declaration in the log. No member lookup occurs until loading is complete.
int32_t orchestrator_register_activation_group(orchestrator_handle *orch,
    const char *group_id, const_strings_t members)
{
    try {
        if (orch == nullptr || group_id == nullptr || members.items == nullptr || members.size < 2)
            throw std::invalid_argument("Activation group requires an orchestrator, an ID and at least two members.");
        std::vector<std::string> ids;
        ids.reserve(members.size);
        for (uint32_t index = 0; index < members.size; ++index) {
            if (members.items[index] == nullptr)
                throw std::invalid_argument("Activation group contains a null member.");
            ids.emplace_back(members.items[index]);
        }
        std::string error;
        if (!to_orchestrator(orch)->register_activation_group(group_id, ids, error))
            throw std::invalid_argument(error);
        return 1;
    } catch (const std::exception &error) {
        BOOST_LOG_TRIVIAL(error) << "Cannot register activation group '" << (group_id ? group_id : "<null>")
                                 << "': " << error.what();
    } catch (...) {
        BOOST_LOG_TRIVIAL(error) << "Unexpected activation group registration failure.";
    }
    return 0;
}

int32_t orchestrator_register_exclusive_group(orchestrator_handle *orch, slicing_step_t step,
                                             const raw_exclusive_group_def *def)
{
    if (orch == nullptr || def == nullptr ||
        def->struct_size < offsetof(raw_exclusive_group_def, value_when_absent) + sizeof(def->value_when_absent)) return -1;
    try {
        if (def->has_value_when_absent && def->value_when_absent == nullptr) return -1;
        Slic3r::Orchestrator::ExclusiveGroupDefinition owned;
        owned.plugin_id = def->declaring_plugin_id != nullptr ? def->declaring_plugin_id : "";
        owned.label = def->label != nullptr ? def->label : "";
        owned.tooltip = def->tooltip != nullptr ? def->tooltip : "";
        Slic3r::StepSelectorTraits &traits = owned.traits;
        traits.exclusive_group = def->exclusive_group != nullptr ? def->exclusive_group : "";
        traits.container_type = def->container_type;
        traits.category = def->category;
        traits.option_preset_type = def->option_preset_type;
        traits.ui_target_file = def->ui_target_file != nullptr && *def->ui_target_file ? def->ui_target_file : "print.ui";
        traits.ui_page = def->ui_page != nullptr ? def->ui_page : "";
        traits.ui_group = def->ui_group != nullptr ? def->ui_group : "";
        traits.option_key = def->option_key != nullptr ? def->option_key : "";
        if (def->has_invalidates_step) traits.invalidates_step = def->invalidates_step;
        traits.strict_selection = def->strict_selection != 0;
        traits.read_with_one_provider = def->read_with_one_provider != 0;
        traits.show_with_one_provider = def->show_with_one_provider != 0;
        traits.provider_settings_rules = def->provider_settings_rules != 0;
        traits.auto_value = def->auto_value != 0;
        if (def->has_value_when_absent) traits.value_when_absent = def->value_when_absent;
        return to_orchestrator(orch)->record_exclusive_group_declaration(step, std::move(owned));
    } catch (...) { return -2; }
}

int32_t orchestrator_register_selector_auto_choice(orchestrator_handle *orch, slicing_step_t step,
    const char *group, const char *key, const char *value, const char *provider)
{
    if (orch == nullptr) return -1;
    Slic3r::Orchestrator &host = *to_orchestrator(orch);
    try {
        Slic3r::SelectorAutoChoice choice;
        choice.step = step;
        choice.exclusive_group = group != nullptr ? group : "";
        choice.condition_key = key != nullptr ? key : "";
        choice.condition_value = value != nullptr ? value : "";
        choice.provider_id = provider != nullptr ? provider : "";
        return host.register_selector_auto_choice(std::move(choice));
    } catch (...) {
        char reason[512];
        std::snprintf(reason, sizeof(reason),
            "Cannot copy or register automatic selector preference: step %d, group '%s', condition '%s=%s', provider '%s'",
            int(step), group != nullptr ? group : "<null>", key != nullptr ? key : "<null>",
            value != nullptr ? value : "<null>", provider != nullptr ? provider : "<null>");
        host.record_plugin_initialization_failure(reason);
        return -2;
    }
}

slicing_step_t orchestrator_register_step(orchestrator_handle *orch,
                                          const char *namespaced_name,
                                          slicing_step_t invalidates_step)
{
    try {
        return to_orchestrator(orch)->register_step(namespaced_name, invalidates_step);
    } catch (...) {
        return STEP_NONE;
    }
}

orchestrator_runtime_id orchestrator_register_runtime_id(
    orchestrator_handle *orch, const char *id_type, const char *namespaced_name)
{
    if (orch == nullptr)
        return ORCHESTRATOR_RUNTIME_ID_INVALID;
    try {
        return to_orchestrator(orch)->register_runtime_id(id_type, namespaced_name);
    } catch (...) {
        return ORCHESTRATOR_RUNTIME_ID_INVALID;
    }
}

const char *orchestrator_runtime_id_name(
    const orchestrator_handle *orch, const char *id_type, orchestrator_runtime_id id)
{
    if (orch == nullptr)
        return nullptr;
    try {
        const Slic3r::Orchestrator *orchestrator = to_orchestrator(orch);
        const std::string *name = orchestrator->runtime_id_name(id_type, id);
        return name == nullptr ? nullptr : name->c_str();
    } catch (...) {
        return nullptr;
    }
}

const char *orchestrator_step_name(const orchestrator_handle *orch, slicing_step_t step)
{
    try {
        const Slic3r::Orchestrator::DynamicStepInfo *info = to_orchestrator(orch)->step_info(step);
        return info == nullptr ? nullptr : info->name.c_str();
    } catch (...) {
        return nullptr;
    }
}

const orchestrator_plugin_handle *orchestrator_find_plugin(orchestrator_handle *orch,
                                                           const char *plugin_id)
{
    if (plugin_id == nullptr || plugin_id[0] == '\0')
        return nullptr;
    try {
        Slic3r::Orchestrator *orchestrator = to_orchestrator(orch);
        const Slic3r::Plugin *plugin = orchestrator->get_plugin(plugin_id);
        if (plugin == nullptr || !orchestrator->is_plugin_active(plugin))
            return nullptr;
        return reinterpret_cast<const orchestrator_plugin_handle *>(plugin);
    } catch (...) {
        return nullptr;
    }
}

const orchestrator_plugin_handle *orchestrator_select_plugin(orchestrator_handle *orch,
                                                             const config_handle *config,
                                                             slicing_step_t step,
                                                             const char *exclusive_group)
{
    if (exclusive_group == nullptr || exclusive_group[0] == '\0')
        return nullptr;
    try {
        Slic3r::Plugin *plugin = Slic3r::Steps::selected_active_plugin_from_group(
            *to_orchestrator(orch),
            step,
            exclusive_group,
            Slic3r::ApiHost::to_config(config));
        return reinterpret_cast<const orchestrator_plugin_handle *>(plugin);
    } catch (...) {
        return nullptr;
    }
}

const char *orchestrator_plugin_id(const orchestrator_plugin_handle *plugin)
{
    const Slic3r::Plugin *host_plugin = reinterpret_cast<const Slic3r::Plugin *>(plugin);
    return host_plugin == nullptr ? nullptr : host_plugin->get_id().c_str();
}

slicing_step_t orchestrator_plugin_step(const orchestrator_plugin_handle *plugin)
{
    const Slic3r::Plugin *host_plugin = reinterpret_cast<const Slic3r::Plugin *>(plugin);
    return host_plugin == nullptr ? STEP_NONE : host_plugin->get_step();
}

raw_plugin_execution_status orchestrator_execute_plugin(
    orchestrator_handle *orch,
    const orchestrator_plugin_handle *plugin,
    const print_handle *print,
    const raw_plugin_run_payload *payloads,
    uint32_t run_count)
{
    try {
        Slic3r::Orchestrator *host_orchestrator = to_orchestrator(orch);
        Slic3r::Plugin *host_plugin = reinterpret_cast<Slic3r::Plugin *>(
            const_cast<orchestrator_plugin_handle *>(plugin));
        if (host_orchestrator == nullptr || !host_orchestrator->owns_plugin(host_plugin))
            return RAW_PLUGIN_EXECUTION_INVALID_ARGUMENT;

        Slic3r::Print *host_print = reinterpret_cast<Slic3r::Print *>(
            const_cast<print_handle *>(print));
        return host_orchestrator->execute_plugin(*host_plugin, host_print, payloads, run_count);
    } catch (...) {
        return RAW_PLUGIN_EXECUTION_PLUGIN_ERROR;
    }
}

void orchestrator_begin_plugin_package_load(orchestrator_handle *orch, const char *package_root)
{
    if (orch == nullptr || package_root == nullptr || package_root[0] == '\0')
        return;
    try {
        const boost::filesystem::path root(package_root);
        Slic3r::Orchestrator *host = to_orchestrator(orch);
        host->begin_plugin_package_load(root.filename().string(), root.string());
        package_registration_scopes.push_back({host, package_root, host->plugin_registration_scope(root.string(), true)});
    } catch (...) {
    }
}

void orchestrator_report_plugin_package_load_error(orchestrator_handle *orch,
                                                   const char *package_root,
                                                   raw_plugin_package_load_error_code code,
                                                   const char *detail)
{
    if (package_root == nullptr || package_root[0] == '\0')
        return;
    try {
        const boost::filesystem::path root(package_root);
        Slic3r::PluginPackageLoadIssue issue;
        issue.code = plugin_package_load_error_code(code);
        if (detail != nullptr)
            issue.detail = detail;
        to_orchestrator(orch)->report_plugin_package_load_issue(root.filename().string(), std::move(issue));
    } catch (...) {
    }
}

void orchestrator_finish_plugin_package_load(orchestrator_handle *orch, const char *package_root)
{
    if (orch == nullptr || package_root == nullptr || package_root[0] == '\0')
        return;
    try {
        const boost::filesystem::path root(package_root);
        to_orchestrator(orch)->finish_plugin_package_load(root.filename().string());
    } catch (...) {
    }
    if (!package_registration_scopes.empty() && package_registration_scopes.back().host == to_orchestrator(orch) &&
        package_registration_scopes.back().root == package_root)
        package_registration_scopes.pop_back();
}

int32_t orchestrator_register_translation_catalog(orchestrator_handle *orch,
                                                  const char *domain,
                                                  const char *locale_directory)
{
    try {
        Slic3r::Orchestrator *orchestrator = to_orchestrator(orch);
        return orchestrator == nullptr ? -1 :
                                       orchestrator->register_translation_catalog(domain, locale_directory);
    } catch (...) {
        return -3;
    }
}

bridge_detector_instance orchestrator_create_bridge_detector(orchestrator_handle *orch,
                                                             const bridge_detector_create_input *input) {
    bridge_detector_instance out = {};
    if (input == nullptr)
        return out;
    Slic3r::Orchestrator *orchestrator = orch == nullptr ? &Slic3r::Orchestrator::instance() :
                                                           reinterpret_cast<Slic3r::Orchestrator *>(orch);
    return orchestrator == nullptr ? out : orchestrator->create_bridge_detector(*input);
}

slic3r_property_type orchestrator_register_property(orchestrator_handle *orch,
                                                    const char *namespaced_name,
                                                    uint32_t byte_count,
                                                    uint32_t alignment)
{
    if (namespaced_name == nullptr || namespaced_name[0] == '\0' ||
        byte_count == 0 || !is_power_of_two(alignment))
        return SLIC3R_PROPERTY_TYPE_INVALID;

    /*
    Built-in names are reserved. A plugin may use the numeric built-in id
    directly, but it must not re-register the same name with a different ABI
    contract.
    */
    for (const GenericPropertyInfo &info : builtin_property_infos())
        if (std::string(info.name) == namespaced_name)
            return info.byte_count == byte_count && info.alignment == alignment ?
                info.type :
                SLIC3R_PROPERTY_TYPE_INVALID;

    Slic3r::Orchestrator *orchestrator = to_orchestrator(orch);
    return orchestrator == nullptr ?
        SLIC3R_PROPERTY_TYPE_INVALID :
        orchestrator->register_property(namespaced_name, byte_count, alignment);
}

uint32_t orchestrator_property_byte_count(const orchestrator_handle *orch, slic3r_property_type type)
{
    if (const GenericPropertyInfo *info = builtin_property_info(type))
        return info->byte_count;
    const Slic3r::Orchestrator *orchestrator = to_orchestrator(orch);
    const Slic3r::Orchestrator::PropertyInfo *info =
        orchestrator == nullptr ? nullptr : orchestrator->property_info(type);
    return info == nullptr ? 0u : info->byte_count;
}

uint32_t orchestrator_property_alignment(const orchestrator_handle *orch, slic3r_property_type type)
{
    if (const GenericPropertyInfo *info = builtin_property_info(type))
        return info->alignment;
    const Slic3r::Orchestrator *orchestrator = to_orchestrator(orch);
    const Slic3r::Orchestrator::PropertyInfo *info =
        orchestrator == nullptr ? nullptr : orchestrator->property_info(type);
    return info == nullptr ? 0u : info->alignment;
}

const char *orchestrator_property_name(const orchestrator_handle *orch, slic3r_property_type type)
{
    if (const GenericPropertyInfo *info = builtin_property_info(type))
        return info->name;
    const Slic3r::Orchestrator *orchestrator = to_orchestrator(orch);
    const Slic3r::Orchestrator::PropertyInfo *info =
        orchestrator == nullptr ? nullptr : orchestrator->property_info(type);
    return info == nullptr ? nullptr : info->name.c_str();
}

int32_t orchestrator_register_generic_facets_annotation(
    orchestrator_handle *orch,
    const raw_generic_facets_annotation_def *def)
{
    if (def == nullptr || def->key == nullptr || def->label == nullptr ||
        def->enforce_label == nullptr || def->block_label == nullptr ||
        def->icon_svg == nullptr || def->icon_svg[0] == '\0')
        return -1;

    try {
        Slic3r::Orchestrator *orchestrator = orch == nullptr ? &Slic3r::Orchestrator::instance() :
                                                               reinterpret_cast<Slic3r::Orchestrator *>(orch);
        if (orchestrator == nullptr)
            return -1;

        Slic3r::GenericFacetsAnnotationDefinition native_def;
        native_def.key = def->key;
        native_def.label = def->label;
        native_def.enforce_label = def->enforce_label;
        native_def.block_label = def->block_label;
        if (def->translation_domain != nullptr)
            native_def.translation_domain = def->translation_domain;
        native_def.icon_svg = def->icon_svg;
        return orchestrator->register_generic_facets_annotation(std::move(native_def)) ? 1 : -2;
    } catch (...) {
        return -3;
    }
}

int32_t orchestrator_add_ui_fragment(orchestrator_handle *orch,
                                     const char *target_file,
                                     const char *fragment_id,
                                     const char *ui_fragment,
                                     int32_t priority)
{
    if (target_file == nullptr || fragment_id == nullptr || ui_fragment == nullptr)
        return -1;

    try {
        Slic3r::Orchestrator *orchestrator = orch == nullptr ? &Slic3r::Orchestrator::instance() :
                                                               reinterpret_cast<Slic3r::Orchestrator *>(orch);
        if (orchestrator == nullptr)
            return -1;
        return orchestrator->add_ui_fragment(target_file, fragment_id, ui_fragment, priority) ? 1 : 0;
    } catch (...) {
        return -2;
    }
}

int32_t orchestrator_add_gui_rule(orchestrator_handle *orch, const raw_gui_rule *rule)
{
    if (rule == nullptr)
        return -1;

    try {
        Slic3r::Orchestrator *orchestrator = orch == nullptr ? &Slic3r::Orchestrator::instance() :
                                                               reinterpret_cast<Slic3r::Orchestrator *>(orch);
        if (orchestrator == nullptr)
            return -1;
        return int32_t(orchestrator->gui_rules().add(rule, orchestrator->current_gui_rule_owner().c_str()));
    } catch (...) {
        return -2;
    }
}

int orchestrator_plugin_is_cancelled(plugin_host_context *host_context)
{
    if (host_context == nullptr)
        return 0;

    // Poll user cancellation on the Print and plugin cancellation on the supplied execution.
    // If none is supplied, Orchestrator uses this thread's execution or the shared stop flag.
    const bool print_cancelled =
        host_context->print != nullptr && host_context->print->canceled();
    const bool plugin_cancelled = host_context->run_context != nullptr ?
        static_cast<Slic3r::Orchestrator::RunContext *>(host_context->run_context)->plugin_cancel_requested() :
        (host_context->orchestrator != nullptr && host_context->orchestrator->is_plugin_cancelled());
    return print_cancelled || plugin_cancelled ? 1 : 0;
}

// Logs the plugin warning, then queues a copy of the message on the orchestrator.
// A NULL host context, plugin or message is tolerated; the caller keeps ownership
// of the message. A lost warning must neither cross the C boundary nor stop the
// step, so any failure is ignored (no message, no plugin stop).
void orchestrator_plugin_report_warning(plugin_host_context *host_context, const char *message)
{
    try {
        const char *plugin_id = host_context != nullptr && host_context->plugin != nullptr ?
                                    host_context->plugin->get_id().c_str() :
                                    "<unknown>";
        BOOST_LOG_TRIVIAL(warning) << "Plugin warning from " << plugin_id << ": " << (message != nullptr ? message : "");

        if (host_context != nullptr && host_context->orchestrator != nullptr)
            host_context->orchestrator->add_plugin_message(Slic3r::Orchestrator::PluginMessageLevel::Warning,
                                                           host_context->plugin,
                                                           host_context->step,
                                                           message);
    } catch (...) {
        // A lost warning must neither cross the C boundary nor stop the step: ignore the failure.
    }
}

// Mark the call failed, optionally copy its diagnostic to caller-owned storage, and otherwise queue the GUI message.
void orchestrator_plugin_report_error(plugin_host_context *host_context, const char *message)
{
    if (host_context != nullptr && host_context->execution_error != nullptr)
        host_context->execution_error->store(true, std::memory_order_relaxed);
    if (host_context != nullptr && host_context->orchestrator != nullptr && host_context->cancel_on_error) {
        // Record the failing plugin only if no cause is known yet, so later errors keep the first cause.
        if (host_context->run_context != nullptr)
            static_cast<Slic3r::Orchestrator::RunContext *>(host_context->run_context)->request_plugin_cancel(host_context->plugin);
        else if (host_context->plugin != nullptr)
            host_context->orchestrator->request_plugin_cancel(*host_context->plugin, host_context->print);
        else
            host_context->orchestrator->request_plugin_cancel(host_context->print);
    }

    try {
        const char *plugin_id = host_context != nullptr && host_context->plugin != nullptr ?
                                    host_context->plugin->get_id().c_str() :
                                    "<unknown>";
        BOOST_LOG_TRIVIAL(error) << "Plugin error from " << plugin_id << ": " << (message != nullptr ? message : "");

        if (host_context != nullptr && host_context->reported_error != nullptr)
            *host_context->reported_error = message != nullptr ? message : "";

        if (host_context != nullptr && host_context->orchestrator != nullptr &&
            host_context->reported_error == nullptr) {
            host_context->orchestrator->add_plugin_message(Slic3r::Orchestrator::PluginMessageLevel::Error,
                                                           host_context->plugin,
                                                           host_context->step,
                                                           message);
        }
    } catch (...) {
        // Diagnostic allocation failures must not escape the C ABI or erase the error state.
    }
}

// Clamps the progress to [0, 1], rounds it to a percentage and publishes it as the
// secondary status, with the message or "Plugin <id>" when the message is NULL or
// empty. A NULL host context, print or plugin is tolerated; the caller keeps
// ownership of the message. A non-finite progress is ignored: NaN has no defined
// integer conversion and the infinities are not a valid progress. A lost progress update must neither
// cross the C boundary nor stop the step, so any failure is ignored.
void orchestrator_plugin_report_progress(plugin_host_context *host_context, double progress, const char *message)
{
    if (host_context == nullptr || host_context->print == nullptr)
        return;
    if (!std::isfinite(progress))
        return;

    try {
        const int percent = int(std::clamp(progress, 0.0, 1.0) * 100.0 + 0.5);
        if (message != nullptr && message[0] != '\0') {
            host_context->print->set_status(percent,
                                            message,
                                            Slic3r::PrintBase::SlicingStatus::SECONDARY_STATE);
            return;
        }

        const std::string plugin_id = host_context->plugin != nullptr ? host_context->plugin->get_id() : "<unknown>";
        const std::string message_text = "Plugin " + plugin_id;
        host_context->print->set_status(percent,
                                        message_text,
                                        Slic3r::PrintBase::SlicingStatus::SECONDARY_STATE);
    } catch (...) {
        // A lost progress update must neither cross the C boundary nor stop the step: ignore the failure.
    }
}

int32_t orchestrator_set_error_message(orchestrator_handle *orchestrator, const char *message)
{
    if (orchestrator == nullptr)
        return PLUGIN_ERROR_GENERAL;
    try {
        return to_orchestrator(orchestrator)->store_error_message(message);
    } catch (...) {
        return PLUGIN_ERROR_GENERAL;
    }
}

}
