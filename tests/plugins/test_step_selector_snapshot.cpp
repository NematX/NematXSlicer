#include <catch2/catch.hpp>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#include <boost/filesystem/path.hpp>

#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

/* Exclusive selectors are enums whose serialized values name active providers. This snapshot
   records generated metadata and resolves stored choices through both public selection paths.
   Raw group definitions borrow their own string storage, so copies refresh those pointers.
   Activation changes are scoped; the optional output file contains a pasteable C++ initializer.
   Python-package origins are read from recorded package roots; their providers and dependent selectors are excluded. */

namespace {
using namespace Slic3r;

// Escape text into one snapshot line, or into a C++ string literal when literal is true.
std::string escaped(const std::string &text, bool literal = false);
// Append a generated definition's owned text, raw fields, enum and fragment without changing the host.
void append_group(std::vector<std::string> &rows, const std::string &prefix, Steps::StepExclusiveGroup group);
// Capture registered definitions and every startup layout fragment matching borrowed candidate groups and native ids.
void append_registered(std::vector<std::string> &rows, const Orchestrator &host,
                       const std::set<std::string> &keys, const std::set<std::string> &groups,
                       const std::set<std::string> &provider_ids, const std::set<std::string> &excluded_keys);
// Identify providers by their recorded package root and the Python packages installed by the test runtime.
bool python_package_provider(const Plugin &plugin);
// Read native finalized selector ids and require each original id to name a registered member of the requested group.
std::vector<std::string> finalized_members(const Orchestrator &host, slicing_step_t step,
                                         const std::string &group, const std::string &key);
// Record group and step resolutions separately using borrowed configuration and active providers.
void append_selection(std::vector<std::string> &rows, Orchestrator &host, const std::string &scenario,
                      slicing_step_t step, const std::string &group, const ConfigBase *config);
// Capture stale, valid, absent and null selections while restoring every provider's activation on exit.
void append_scenarios(std::vector<std::string> &rows, Orchestrator &host, slicing_step_t dynamic_step);

class SnapshotProbe final : public slic3r_api::PluginBase
{
public:
    // Borrow the host and retain scalar metadata for a provider with no configuration or execution effects.
    SnapshotProbe(orchestrator_handle *host, const char *id = "test.snapshot.selector",
                  slicing_step_t step = STEP_POST_INFILL, int priority = 0)
        : PluginBase(host), m_id(id), m_step(step), m_priority(priority) {}
private:
    // Return the stable borrowed provider identifier; its literal outlives this instance and cannot fail.
    const char *id_impl() const noexcept override { return m_id; }
    // Return the stable borrowed provider name without effects or failures.
    const char *name_impl() const noexcept override { return m_id; }
    // Return a borrowed description literal without effects or failures.
    const char *description_impl() const noexcept override { return "Selector snapshot probe"; }
    // Return the borrowed group identifier without effects or failures.
    const char *exclusive_group_impl() const noexcept override { return "test.snapshot.selector"; }
    // Return the copied step identity without effects or failures.
    slicing_step_t step_impl() const noexcept override { return m_step; }
    // Report no dependencies without transferring ownership or changing state.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Return the copied execution priority without effects or failures.
    int32_t priority_impl() const noexcept override { return m_priority; }
    // Ignore the borrowed execution context; this provider produces no output and cannot fail.
    void run_impl(const plugin_run_context *) const override {}
    const char *m_id;
    slicing_step_t m_step;
    int m_priority;
};

class ActivationGuard
{
public:
    // Borrow the host and copy every activation; allocation failures propagate before any activation changes.
    explicit ActivationGuard(Orchestrator &host) : m_host(host)
    {
        for (Plugin *plugin : host.registered_plugins())
            m_saved.emplace_back(plugin, host.is_plugin_active(plugin));
    }
    // Restore the saved activation flags; the borrowed host/providers must still exist and no ownership changes.
    ~ActivationGuard()
    {
        for (const std::pair<Plugin *, bool> &entry : m_saved)
            m_host.set_plugin_active(entry.first, entry.second);
    }
private:
    Orchestrator &m_host;
    std::vector<std::pair<Plugin *, bool>> m_saved;
};

// Encode borrowed bytes without mutation; the owned result preserves newlines and allocation failures propagate.
std::string escaped(const std::string &text, bool literal)
{
    std::string result;
    for (char ch : text) {
        if (ch == '\n') result += "\\n";
        else if (ch == '\r') result += "\\r";
        else if (ch == '\t') result += "\\t";
        else if (ch == '\\') result += "\\\\";
        else if (ch == '"' && literal) result += "\\\"";
        else result += ch;
    }
    return result;
}

// Refresh the value copy's borrowed raw pointers and append owned metadata; no host state changes and allocations may throw.
void append_group(std::vector<std::string> &rows, const std::string &prefix, Steps::StepExclusiveGroup group)
{
    group.refresh_storage_pointers();
    const raw_config_option_def &def = group.option_def;
    std::ostringstream row;
    row << prefix << "|key=" << group.option_key_storage << "|label=" << escaped(group.label_storage)
        << "|tooltip=" << escaped(group.tooltip_storage) << "|type=" << int(def.type)
        << "|container=" << int(def.container_type) << "|preset=" << int(def.option_preset_type)
        << "|category=" << int(def.category) << "|invalidates=" << int(def.invalidates_step)
        << "|mode=" << uint64_t(def.mode) << "|gui=" << int(def.gui_type)
        << "|plugin_ids=" << int(def.contains_plugin_id) << "|technology=" << int(def.printer_technology)
        << "|default=" << escaped(def.default_serialized_value ? def.default_serialized_value : "") << "|values=";
    for (size_t i = 0; i < group.enum_values.size(); ++i) {
        if (i != 0) row << ',';
        row << escaped(group.enum_values[i]);
    }
    row << "|ui=" << escaped(group.ui_fragment);
    rows.push_back(row.str());
}

// Read only present definitions and group-matching fragments; map fragment keys using the borrowed startup id set.
// No host state changes; rows own their strings and allocation failures propagate.
void append_registered(std::vector<std::string> &rows, const Orchestrator &host,
                       const std::set<std::string> &keys, const std::set<std::string> &groups,
                       const std::set<std::string> &provider_ids, const std::set<std::string> &excluded_keys)
{
    for (const std::string &key : keys) {
        const ConfigOptionDef *def = PrintConfigDef::instance().get(key);
        if (def == nullptr) continue;
        const Plugin *default_provider = def->default_value ? host.get_plugin(def->default_value->serialize()) : nullptr;
        INFO("Selector default must be native: " << key << " => " <<
             (default_provider ? default_provider->get_id() : "none"));
        REQUIRE((default_provider == nullptr || !python_package_provider(*default_provider)));
        std::ostringstream row;
        row << "def|" << key << "|label=" << escaped(def->label) << "|tooltip=" << escaped(def->tooltip)
            << "|type=" << int(def->type) << "|container=" << int(def->container_type)
            << "|preset=" << def->option_preset_type << "|category=" << int(def->category)
            << "|invalidates=" << int(def->invalidates_step) << "|mode=" << uint64_t(def->mode)
            << "|gui=" << int(def->gui_type) << "|plugin_ids=" << int(def->contains_plugin_id)
            << "|technology=" << int(def->printer_technology)
            << "|default=" << escaped(def->default_value ? def->default_value->serialize() : "") << "|values=";
        if (def->enum_def) {
            const std::vector<std::string> &values = def->enum_def->values();
            bool first = true;
            for (const std::string &value : values) {
                const Plugin *provider = host.get_plugin(value);
                REQUIRE(provider != nullptr);
                if (python_package_provider(*provider)) continue;
                if (!first) row << ',';
                row << escaped(value);
                first = false;
            }
        }
        rows.push_back(row.str());
    }
    const std::set<std::pair<std::string, std::string>> &fragment_ids = Test::Plugins::plugin_test_startup_sets().ui_fragments;
    std::map<std::pair<std::string, std::string>, std::string> native_fragments;
    for (const std::pair<std::string, std::string> &identity : fragment_ids) {
        if (groups.count(identity.second) == 0) continue;
        std::string key = identity.second;
        for (const Plugin *plugin : host.registered_plugins())
            if (provider_ids.count(plugin->get_id()) != 0 && plugin->get_exclusive_group() == identity.second)
                key = Steps::plugin_exclusive_group_option_key(host, plugin->get_step(), identity.second);
        for (const std::pair<const slicing_step_t, Steps::StepExclusiveGroup> &entry : Steps::get_exclusive_steps())
            if (entry.second.group_id == identity.second) key = entry.second.option_key_storage;
        if (excluded_keys.count(key) == 0) native_fragments.emplace(identity, key);
    }
    std::set<std::pair<std::string, std::string>> captured_fragments;
    const char *const files[] = {"print.ui", "printer_fff.ui", "filament.ui", "printer_sla.ui"};
    for (const char *file : files)
        for (const Orchestrator::PluginUiFragment &fragment : host.ui_fragments_for_file(file)) {
            if (groups.count(fragment.fragment_id) == 0) continue;
            const std::map<std::pair<std::string, std::string>, std::string>::const_iterator native =
                native_fragments.find({file, fragment.fragment_id});
            if (native == native_fragments.end()) continue;
            // A fragment referencing an absent selector cannot display a native selector; provider-setting fragments remain valid.
            if (PrintConfigDef::instance().get(native->second) == nullptr &&
                fragment.content.find(native->second) != std::string::npos) {
                native_fragments.erase(native);
                continue;
            }
            captured_fragments.emplace(file, fragment.fragment_id);
            rows.push_back("ui|" + native->second + "|" + file + "|" + fragment.fragment_id +
                           "|priority=" + std::to_string(fragment.priority) + "|" + escaped(fragment.content));
        }
    REQUIRE(captured_fragments.size() == native_fragments.size());
}

// Match the host-owned package-root basename against packages copied beside the Python loader by tests/plugins/CMakeLists.txt.
// Provider ids are not classified by spelling; inputs remain borrowed, no state changes, and path/allocation errors propagate.
bool python_package_provider(const Plugin &plugin)
{
    const std::string package = boost::filesystem::path(plugin.get_package_root()).filename().string();
    static const std::set<std::string> packages = {
        "python", "python.example.noop", "python.perimeter.post_process.hairy_object",
        "python.surface.initial_typed_surface_builder", "python.polyholes.high_level", "python.polyholes",
        "python.perimeter.generator.simple", "python.external.package_root", "python.test.clear.invalid_registration"
    };
    return packages.count(package) != 0;
}

// Copy ids from the finalized definition, validating borrowed host membership without changing activation.
// Missing definitions, enums or providers fail REQUIRE; strings are owned by the result and allocation errors propagate.
std::vector<std::string> finalized_members(const Orchestrator &host, slicing_step_t step,
                                         const std::string &group, const std::string &key)
{
    const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
    REQUIRE(definition != nullptr);
    REQUIRE(definition->enum_def);
    std::vector<std::string> ids;
    for (const std::string &id : definition->enum_def->values()) {
        const Plugin *plugin = host.get_plugin(id);
        REQUIRE(plugin != nullptr);
        REQUIRE(plugin->get_step() == step);
        REQUIRE(plugin->get_exclusive_group() == group);
        if (!python_package_provider(*plugin)) ids.push_back(id);
    }
    return ids;
}

// Resolve both borrowed public paths without editing settings; record all step ids, not just its first provider.
// Empty results are represented by none; returned rows own text and allocation/selection failures propagate.
void append_selection(std::vector<std::string> &rows, Orchestrator &host, const std::string &scenario,
                      slicing_step_t step, const std::string &group, const ConfigBase *config)
{
    Plugin *selected = Steps::selected_active_plugin_from_group(host, step, group, config);
    REQUIRE((selected == nullptr || !python_package_provider(*selected)));
    rows.push_back("select|" + scenario + "_group|" + (selected ? selected->get_id() : "none"));
    const std::vector<Plugin *> selected_step = Steps::selected_or_active_plugins_for_step(host, step, config);
    std::string ids;
    for (const Plugin *plugin : selected_step) {
        REQUIRE(!python_package_provider(*plugin));
        if (!ids.empty()) ids += ',';
        ids += plugin->get_id();
    }
    rows.push_back("select|" + scenario + "_step|" + (ids.empty() ? "none" : ids));
}

// Use static no-op probes on the named dynamic step and scope all activation mutations with a guard.
// Configurations own their values and borrow finalized enum maps; new host registrations retain their initial activation.
// Missing runtime providers fail REQUIRE; allocation errors propagate without losing saved activation flags.
void append_scenarios(std::vector<std::string> &rows, Orchestrator &host, slicing_step_t dynamic_step)
{
    static SnapshotProbe low(reinterpret_cast<orchestrator_handle *>(&host), "test.snapshot.selector.low", dynamic_step, 0);
    static SnapshotProbe high(reinterpret_cast<orchestrator_handle *>(&host), "test.snapshot.selector.high", dynamic_step, 10);
    if (host.get_plugin("test.snapshot.selector.low") == nullptr) {
        REQUIRE(host.register_plugin(low.c_instance()));
        REQUIRE(host.register_plugin(high.c_instance()));
    }
    ActivationGuard guard(host);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const slicing_step_t steps[] = {SEAM_PLACER, GCODE_THUMBNAIL, dynamic_step};
    const char *const groups[] = {"seam_placer_plugin", "thumbnail_encoder_plugin", "test.snapshot.selector"};
    const char *const scenarios[] = {"seam_unknown", "thumbnail_unknown", "probe_unknown"};
    for (size_t i = 0; i < 3; ++i) {
        const std::string key = Steps::plugin_exclusive_group_option_key(host, steps[i], groups[i]);
        const std::vector<std::string> ids = i < 2 ? finalized_members(host, steps[i], groups[i], key) :
            std::vector<std::string>{"test.snapshot.selector.low", "test.snapshot.selector.high"};
        std::vector<Plugin *> members;
        for (Plugin *plugin : host.registered_plugins())
            if (plugin->get_step() == steps[i]) {
                const bool member = std::find(ids.begin(), ids.end(), plugin->get_id()) != ids.end();
                REQUIRE(host.set_plugin_active(plugin, member));
                if (member) members.push_back(plugin);
            }
        REQUIRE(members.size() >= 2);
        if (i < 2) {
            REQUIRE(config.optptr(key) != nullptr);
            config.optptr(key)->set_int(1000000);
            REQUIRE(config.option(key)->serialize().empty());
        } else config.set_key_value(key, new ConfigOptionString("obsolete.snapshot.provider"));
        append_selection(rows, host, scenarios[i], steps[i], groups[i], &config);
        DynamicConfig absent;
        append_selection(rows, host, std::string(scenarios[i]) + "_absent", steps[i], groups[i], &absent);
        append_selection(rows, host, std::string(scenarios[i]) + "_null", steps[i], groups[i], nullptr);
        const std::vector<Plugin *> active = host.get_active_plugins_for_step(steps[i]);
        REQUIRE(!active.empty());
        Plugin *first = active.front();
        for (Plugin *member : members) REQUIRE(host.set_plugin_active(member, member == first));
        append_selection(rows, host, std::string(scenarios[i]) + "_single", steps[i], groups[i], &config);
        if (i < 2) config.set_deserialize_strict(key, first->get_id());
        else config.set_key_value(key, new ConfigOptionString(first->get_id()));
        append_selection(rows, host, std::string(scenarios[i]) + "_valid_single", steps[i], groups[i], &config);
    }
    const char *const ordering_ids[] = {"ordering.extrusion_tree.advanced", "ordering.extrusion_tree.xy_serpentine",
                                      "ordering.extrusion_tree.default"};
    for (Plugin *plugin : host.registered_plugins())
        if (plugin->get_step() == STEP_ORDERING) REQUIRE(host.set_plugin_active(plugin, false));
    for (const char *id : ordering_ids) {
        REQUIRE(host.get_plugin(id) != nullptr);
        REQUIRE(!python_package_provider(*host.get_plugin(id)));
        REQUIRE(host.set_plugin_active(id, true));
    }
    DynamicConfig absent;
    for (size_t i = 0; i < 3; ++i) {
        if (i == 0)
            for (Steps::StepExclusivePluginGroup &entry : Steps::active_exclusive_plugin_groups(host)) {
                if (entry.group.step != STEP_ORDERING || entry.group.group_id != "ordering.extrusion_tree") continue;
                std::vector<std::pair<std::string, std::string>> providers;
                for (const Plugin *plugin : entry.plugins) providers.emplace_back(plugin->get_id(), plugin->get_name());
                entry.group.set_enum_plugins(providers);
                append_group(rows, "group|" + std::to_string(int(entry.group.step)) +
                             "|ordering_all|ordering.extrusion_tree", entry.group);
            }
        append_selection(rows, host, "ordering_active_" + std::to_string(3 - i), STEP_ORDERING,
                         "ordering.extrusion_tree", &absent);
        REQUIRE(host.set_plugin_active(ordering_ids[i], false));
    }
}
} // namespace

// Normalize borrowed host activations to recorded native startup providers, then collect owned rows.
// Guards restore activations on failure; metadata mismatches fail assertions and optional file writes must succeed.
TEST_CASE("Generated exclusive selectors keep their keys, scopes, categories and placement",
          "[plugins][step-pipeline][selector-snapshot]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    const slicing_step_t dynamic_step = host.register_step("slic3r.test.selector_snapshot", STEP_INFILL);
    REQUIRE(dynamic_step != STEP_NONE);
    SnapshotProbe probe(reinterpret_cast<orchestrator_handle *>(&host));
    Plugin first(probe.c_instance(), "Slic3r", "");
    std::vector<std::string> rows;
    std::set<std::string> keys;
    std::set<std::string> groups;
    for (const std::pair<const slicing_step_t, Steps::StepExclusiveGroup> &entry : Steps::get_exclusive_steps()) {
        append_group(rows, "table|" + std::to_string(int(entry.first)), entry.second);
        keys.insert(entry.second.option_key_storage);
        groups.insert(entry.second.group_id);
    }
    // Startup identities include every loaded provider and generated selector, independently of later test registrations.
    const Test::Plugins::PluginTestStartupSets &startup = Test::Plugins::plugin_test_startup_sets();
    keys.insert(startup.selector_keys.begin(), startup.selector_keys.end());
    std::set<std::string> provider_ids;
    std::set<std::string> excluded_keys;
    for (const std::string &key : keys) {
        const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
        if (definition == nullptr) {
            excluded_keys.insert(key);
            continue;
        }
        REQUIRE(definition->enum_def);
        const Plugin *default_provider = definition->default_value ?
            host.get_plugin(definition->default_value->serialize()) : nullptr;
        INFO("Selector default must be native: " << key << " => " <<
             (default_provider ? default_provider->get_id() : "none"));
        REQUIRE((default_provider == nullptr || !python_package_provider(*default_provider)));
        size_t native_count = 0;
        bool removed_python = false;
        for (const std::string &id : definition->enum_def->values()) {
            const Plugin *provider = host.get_plugin(id);
            REQUIRE(provider != nullptr);
            if (python_package_provider(*provider)) removed_python = true;
            else {
                ++native_count;
            }
        }
        // Host generation needs two alternatives except for whole-step G-code/firmware selectors.
        // Exclude selectors pushed below that threshold by Python removal; native-only finalized singletons stay intact.
        const size_t threshold = key == "step_gcode_plugin" || key == "gcode_firmware_plugin" ? 1 : 2;
        if (removed_python && native_count < threshold) excluded_keys.insert(key);
    }
    for (const std::string &key : excluded_keys) keys.erase(key);
    for (const std::string &id : startup.provider_ids) {
        const Plugin *provider = host.get_plugin(id);
        REQUIRE(provider != nullptr);
        if (!python_package_provider(*provider)) provider_ids.insert(id);
    }
    ActivationGuard guard(host);
    for (Plugin *plugin : host.registered_plugins()) {
        const bool active = provider_ids.count(plugin->get_id()) != 0 && startup.active_provider_ids.count(plugin->get_id()) != 0;
        REQUIRE(host.set_plugin_active(plugin, active));
    }
    const slicing_step_t steps[] = {
        STEP_LAYER_HEIGHT, STEP_SLICING, STEP_POST_SLICING, STEP_ALERT_SUPPORTS_NEEDED,
        STEP_PRE_PERIMETER, STEP_PERIMETER, STEP_POST_PERIMETER, STEP_SURFACE_GENERATION, STEP_SKIRT_BRIM,
        STEP_PRE_INFILL, STEP_INFILL_GROUP, STEP_INFILL, STEP_POST_INFILL, STEP_SUPPORT_DEMAND, STEP_SUPPORT,
        STEP_FINALIZE_SLICING, STEP_CHECK_CONFLICT, STEP_ORDERING, STEP_WIPETOWER,
        STEP_SUPPORT_SPOT, STEP_LAYER_EXTRUSION_EDIT, STEP_LAYER_STICHING, STEP_EXTRUSION_EDIT,
        STEP_EXTRUSION_SIMPLIFICATION, STEP_GCODE, INFILL_PATTERN, INFILL_SURFACE_RECIPE_MODIFIER, BRIDGE_DETECTOR,
        PERIMETER_GENERATION_MODULE, SEAM_PLACER, GCODE_FIRMWARE, GCODE_THUMBNAIL, GCODE_READER, WIPE_TOWER_WIPE,
        dynamic_step, slicing_step_t(5000)
    };
    for (slicing_step_t step : steps) {
        const std::string label = step == dynamic_step ? "dynamic" : std::to_string(int(step));
        append_group(rows, "group|" + label + "|synthetic|snapshot_group",
                     Steps::plugin_exclusive_group_definition(host, step, "snapshot_group", first));
    }
    const std::pair<slicing_step_t, const char *> real_groups[] = {
        {SEAM_PLACER, "seam_placer_plugin"}, {GCODE_THUMBNAIL, "thumbnail_encoder_plugin"},
        {GCODE_FIRMWARE, "gcode_firmware_plugin"}, {STEP_ORDERING, "ordering.extrusion_tree"}
    };
    for (const std::pair<slicing_step_t, const char *> &identity : real_groups) {
        const std::vector<Plugin *> providers = host.get_active_plugins_for_step(identity.first);
        const Plugin *member = nullptr;
        for (const Plugin *plugin : providers)
            if (plugin->get_exclusive_group() == identity.second) { member = plugin; break; }
        REQUIRE(member != nullptr);
        Steps::StepExclusiveGroup group = Steps::plugin_exclusive_group_definition(
            host, identity.first, identity.second, *member);
        std::vector<std::pair<std::string, std::string>> values;
        for (const Plugin *plugin : providers)
            if (plugin->get_exclusive_group() == identity.second)
                values.emplace_back(plugin->get_id(), plugin->get_name());
        group.set_enum_plugins(values);
        append_group(rows, "group|" + std::to_string(int(identity.first)) + "|real|" + identity.second, group);
    }
    for (Plugin *plugin : host.registered_plugins()) {
        if (provider_ids.count(plugin->get_id()) == 0) continue;
        if (plugin->get_exclusive_group().empty()) continue;
        const std::string key = Steps::plugin_exclusive_group_option_key(host, plugin->get_step(), plugin->get_exclusive_group());
        if (excluded_keys.count(key) == 0) keys.insert(key);
        groups.insert(plugin->get_exclusive_group());
    }
    // Enumerate every native startup alternative, including providers disabled after shared initialization.
    // Normal-runtime group metadata above and the ordering resolution below retain the recorded startup activation.
    for (Plugin *plugin : host.registered_plugins())
        REQUIRE(host.set_plugin_active(plugin, provider_ids.count(plugin->get_id()) != 0));
    for (Steps::StepExclusivePluginGroup &entry : Steps::active_exclusive_plugin_groups(host)) {
        if (excluded_keys.count(entry.group.option_key_storage) != 0) continue;
        std::vector<std::pair<std::string, std::string>> providers;
        for (const Plugin *plugin : entry.plugins) providers.emplace_back(plugin->get_id(), plugin->get_name());
        entry.group.set_enum_plugins(providers);
        const std::string label = entry.group.step == dynamic_step ? "dynamic" : std::to_string(int(entry.group.step));
        append_group(rows, "group|" + label + "|runtime|" + entry.group.group_id, entry.group);
    }
    append_registered(rows, host, keys, groups, provider_ids, excluded_keys);
    {
        ActivationGuard normal_guard(host);
        for (Plugin *plugin : host.registered_plugins())
            REQUIRE(host.set_plugin_active(plugin, provider_ids.count(plugin->get_id()) != 0 &&
                                           startup.active_provider_ids.count(plugin->get_id()) != 0));
        append_selection(rows, host, "ordering_runtime", STEP_ORDERING, "ordering.extrusion_tree", nullptr);
    }
    append_scenarios(rows, host, dynamic_step);
    std::sort(rows.begin(), rows.end());

    if (const char *output = std::getenv("SLIC3R_SELECTOR_SNAPSHOT")) {
        std::ofstream file(output, std::ios::binary | std::ios::trunc);
        REQUIRE(file.is_open());
        file << "const char *const k_expected[] = {\n";
        for (const std::string &row : rows) file << "    \"" << escaped(row, true) << "\",\n";
        file << "};\n";
        file.close();
        REQUIRE(!file.fail());
        return;
    }
    const char *const k_expected[] = {
    "def|exclusive_group_10250_seam_placer_plugin_plugin|label=Seam placement|tooltip=Selects the service used to choose the starting point of closed extrusion loops.|type=9|container=5|preset=1|category=2|invalidates=1500|mode=22|gui=0|plugin_ids=1|technology=1|default=seam_placer.cost|values=seam_placer.cost,seam_placer.aligned,seam_placer.contiguous,seam_placer.fixed,seam_placer.random,slic3r.test.seam_placer.regional",
    "def|exclusive_group_300_test_exclusive_selector_pruning_plugin|label=test.exclusive.selector_pruning|tooltip=Choose which active plugin owns this exclusive plugin group.|type=9|container=1|preset=1|category=2|invalidates=300|mode=22|gui=0|plugin_ids=1|technology=1|default=slic3r.test.exclusive_pruning.valid|values=slic3r.test.exclusive_pruning.valid",
    "def|exclusive_group_700_perimeter_post_process_extra_perimeters_on_overhangs_plugin|label=Extra overhang perimeter strategy|tooltip=Choose the algorithm used to add extra perimeter anchors under overhangs.|type=9|container=1|preset=1|category=1|invalidates=700|mode=22|gui=0|plugin_ids=1|technology=1|default=perimeter.post_process.extra_perimeters_on_overhangs|values=perimeter.post_process.extra_perimeters_on_overhangs,perimeter.post_process.extra_perimeter_overhang_wave",
    "def|gcode_firmware_plugin|label=G-code firmware|tooltip=Selects the firmware session used by PrintingPlan output writers.|type=9|container=1|preset=7|category=11|invalidates=2000|mode=22|gui=0|plugin_ids=1|technology=1|default=gcode.firmware.marlin2|values=gcode.firmware.marlin2,gcode.firmware.marlin1,gcode.firmware.prusa,gcode.firmware.reprap,gcode.firmware.sprinter,gcode.firmware.klipper",
    "def|step_gcode_plugin|label=G-code plugin|tooltip=Selects how STEP_GCODE creates the output artifact.|type=9|container=1|preset=7|category=11|invalidates=2000|mode=22|gui=0|plugin_ids=1|technology=1|default=gcode.printing_plan_file_writer|values=gcode.printing_plan_file_writer",
    "def|step_layer_height_plugin|label=Layer height plugin|tooltip=Choose which active plugin owns this exclusive slicing step.|type=9|container=1|preset=1|category=2|invalidates=100|mode=22|gui=0|plugin_ids=1|technology=1|default=standard_layer_height_generator|values=standard_layer_height_generator,flat_area_layer_height",
    "def|step_perimeter_plugin|label=Perimeter plugin|tooltip=Choose which active plugin owns this exclusive slicing step.|type=9|container=1|preset=1|category=1|invalidates=600|mode=22|gui=0|plugin_ids=1|technology=1|default=perimeter.generator.simple|values=perimeter.generator.simple,perimeter.generator.classic,perimeter.generator.arachne",
    "def|thumbnails_format|label=Thumbnail encoding|tooltip=Selects the encoding used for thumbnails embedded in G-code files.|type=9|container=1|preset=7|category=11|invalidates=2000|mode=22|gui=0|plugin_ids=1|technology=1|default=PNG|values=PNG,JPG,QOI,BIQU",
    "group|10000|synthetic|snapshot_group|key=exclusive_group_10000_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=10000|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_10000_snapshot_group_plugin\\nend_line\\n",
    "group|1000|synthetic|snapshot_group|key=exclusive_group_1000_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=3|invalidates=1000|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_1000_snapshot_group_plugin\\nend_line\\n",
    "group|10050|synthetic|snapshot_group|key=exclusive_group_10050_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=10050|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_10050_snapshot_group_plugin\\nend_line\\n",
    "group|100|runtime|step_layer_height_plugin|key=step_layer_height_plugin|label=Layer height plugin|tooltip=Choose which active plugin owns this exclusive slicing step.|type=8|container=1|preset=1|category=2|invalidates=100|mode=22|gui=7|plugin_ids=1|technology=1|default=standard_layer_height_generator|values=standard_layer_height_generator,flat_area_layer_height|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Layer height step plugin\\nsetting:step_layer_height_plugin\\nend_line\\n",
    "group|100|synthetic|snapshot_group|key=exclusive_group_100_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=100|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_100_snapshot_group_plugin\\nend_line\\n",
    "group|10100|synthetic|snapshot_group|key=exclusive_group_10100_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=10100|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_10100_snapshot_group_plugin\\nend_line\\n",
    "group|10200|synthetic|snapshot_group|key=exclusive_group_10200_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=10200|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_10200_snapshot_group_plugin\\nend_line\\n",
    "group|10250|real|seam_placer_plugin|key=exclusive_group_10250_seam_placer_plugin_plugin|label=Seam placement|tooltip=Selects the service used to choose the starting point of closed extrusion loops.|type=8|container=5|preset=1|category=2|invalidates=1500|mode=22|gui=7|plugin_ids=1|technology=1|default=seam_placer.fixed|values=seam_placer.fixed|ui=page:Perimeters & Shell\\ngroup:Seam\\nline:Seam placement\\nsetting:exclusive_group_10250_seam_placer_plugin_plugin\\nend_line\\n",
    "group|10250|runtime|seam_placer_plugin|key=exclusive_group_10250_seam_placer_plugin_plugin|label=Seam placement|tooltip=Selects the service used to choose the starting point of closed extrusion loops.|type=8|container=5|preset=1|category=2|invalidates=1500|mode=22|gui=7|plugin_ids=1|technology=1|default=seam_placer.cost|values=seam_placer.cost,seam_placer.aligned,seam_placer.contiguous,seam_placer.fixed,seam_placer.random,slic3r.test.seam_placer.regional|ui=page:Perimeters & Shell\\ngroup:Seam\\nline:Seam placement\\nsetting:exclusive_group_10250_seam_placer_plugin_plugin\\nend_line\\n",
    "group|10250|synthetic|snapshot_group|key=exclusive_group_10250_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=1|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_10250_snapshot_group_plugin\\nend_line\\n",
    "group|10300|real|gcode_firmware_plugin|key=exclusive_group_10300_gcode_firmware_plugin_plugin|label=G-code firmware|tooltip=Selects the firmware session used by PrintingPlan output writers.|type=8|container=1|preset=7|category=11|invalidates=2000|mode=22|gui=7|plugin_ids=1|technology=1|default=gcode.firmware.marlin2|values=gcode.firmware.marlin2,gcode.firmware.marlin1,gcode.firmware.prusa,gcode.firmware.reprap,gcode.firmware.sprinter,gcode.firmware.klipper|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:G-code firmware\\nsetting:exclusive_group_10300_gcode_firmware_plugin_plugin\\nend_line\\n",
    "group|10300|runtime|gcode_firmware_plugin|key=gcode_firmware_plugin|label=G-code firmware|tooltip=Selects the firmware session used by PrintingPlan output writers.|type=8|container=1|preset=7|category=11|invalidates=2000|mode=22|gui=7|plugin_ids=1|technology=1|default=gcode.firmware.marlin2|values=gcode.firmware.marlin2,gcode.firmware.marlin1,gcode.firmware.prusa,gcode.firmware.reprap,gcode.firmware.sprinter,gcode.firmware.klipper|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:G-code firmware\\nsetting:gcode_firmware_plugin\\nend_line\\n",
    "group|10300|synthetic|snapshot_group|key=exclusive_group_10300_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=7|category=11|invalidates=2000|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_10300_snapshot_group_plugin\\nend_line\\n",
    "group|10350|real|thumbnail_encoder_plugin|key=thumbnails_format|label=Thumbnail encoding|tooltip=Selects the encoding used for thumbnails embedded in G-code files.|type=8|container=1|preset=7|category=11|invalidates=2000|mode=22|gui=7|plugin_ids=1|technology=1|default=PNG|values=PNG,JPG,QOI,BIQU|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Thumbnail encoding\\nsetting:thumbnails_format\\nend_line\\n",
    "group|10350|runtime|thumbnail_encoder_plugin|key=thumbnails_format|label=Thumbnail encoding|tooltip=Selects the encoding used for thumbnails embedded in G-code files.|type=8|container=1|preset=7|category=11|invalidates=2000|mode=22|gui=7|plugin_ids=1|technology=1|default=PNG|values=PNG,JPG,QOI,BIQU|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Thumbnail encoding\\nsetting:thumbnails_format\\nend_line\\n",
    "group|10350|synthetic|snapshot_group|key=exclusive_group_10350_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=7|category=11|invalidates=2000|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_10350_snapshot_group_plugin\\nend_line\\n",
    "group|10400|synthetic|snapshot_group|key=exclusive_group_10400_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=7|category=11|invalidates=2000|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_10400_snapshot_group_plugin\\nend_line\\n",
    "group|10450|synthetic|snapshot_group|key=exclusive_group_10450_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=11|invalidates=10450|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_10450_snapshot_group_plugin\\nend_line\\n",
    "group|1100|synthetic|snapshot_group|key=exclusive_group_1100_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=3|invalidates=1100|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_1100_snapshot_group_plugin\\nend_line\\n",
    "group|1200|synthetic|snapshot_group|key=exclusive_group_1200_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=7|invalidates=1200|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_1200_snapshot_group_plugin\\nend_line\\n",
    "group|1300|synthetic|snapshot_group|key=exclusive_group_1300_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=7|invalidates=1300|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_1300_snapshot_group_plugin\\nend_line\\n",
    "group|1400|synthetic|snapshot_group|key=exclusive_group_1400_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=1400|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_1400_snapshot_group_plugin\\nend_line\\n",
    "group|1450|synthetic|snapshot_group|key=exclusive_group_1450_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=1450|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_1450_snapshot_group_plugin\\nend_line\\n",
    "group|1500|ordering_all|ordering.extrusion_tree|key=exclusive_group_1500_ordering_extrusion_tree_plugin|label=Extrusion-tree ordering|tooltip=Selects how STEP_ORDERING fixes the order inside each PrintingExtrusion tree.|type=8|container=1|preset=1|category=11|invalidates=1500|mode=22|gui=7|plugin_ids=1|technology=1|default=ordering.extrusion_tree.advanced|values=ordering.extrusion_tree.advanced,ordering.extrusion_tree.xy_serpentine,ordering.extrusion_tree.default|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Extrusion-tree ordering\\nsetting:exclusive_group_1500_ordering_extrusion_tree_plugin\\nend_line\\n",
    "group|1500|real|ordering.extrusion_tree|key=exclusive_group_1500_ordering_extrusion_tree_plugin|label=Extrusion-tree ordering|tooltip=Selects how STEP_ORDERING fixes the order inside each PrintingExtrusion tree.|type=8|container=1|preset=1|category=11|invalidates=1500|mode=22|gui=7|plugin_ids=1|technology=1|default=ordering.extrusion_tree.default|values=ordering.extrusion_tree.default|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Extrusion-tree ordering\\nsetting:exclusive_group_1500_ordering_extrusion_tree_plugin\\nend_line\\n",
    "group|1500|runtime|ordering.extrusion_tree|key=exclusive_group_1500_ordering_extrusion_tree_plugin|label=Extrusion-tree ordering|tooltip=Selects how STEP_ORDERING fixes the order inside each PrintingExtrusion tree.|type=8|container=1|preset=1|category=11|invalidates=1500|mode=22|gui=7|plugin_ids=1|technology=1|default=ordering.extrusion_tree.advanced|values=ordering.extrusion_tree.advanced,ordering.extrusion_tree.xy_serpentine,ordering.extrusion_tree.default|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Extrusion-tree ordering\\nsetting:exclusive_group_1500_ordering_extrusion_tree_plugin\\nend_line\\n",
    "group|1500|synthetic|snapshot_group|key=exclusive_group_1500_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=11|invalidates=1500|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_1500_snapshot_group_plugin\\nend_line\\n",
    "group|1600|synthetic|snapshot_group|key=exclusive_group_1600_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=11|invalidates=1600|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_1600_snapshot_group_plugin\\nend_line\\n",
    "group|1625|synthetic|snapshot_group|key=exclusive_group_1625_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=7|invalidates=1625|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_1625_snapshot_group_plugin\\nend_line\\n",
    "group|1650|runtime|layer_extrusion_edit.travel|key=exclusive_group_1650_layer_extrusion_edit_travel_plugin|label=Travel generation|tooltip=Selects how discontinuities between ordered extrusion leaves become travels.|type=8|container=1|preset=1|category=2|invalidates=1650|mode=22|gui=7|plugin_ids=1|technology=1|default=layer_extrusion_edit.travel.default|values=layer_extrusion_edit.travel.default,layer_extrusion_edit.travel.avoid_crossing_perimeters|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Travel generation\\nsetting:exclusive_group_1650_layer_extrusion_edit_travel_plugin\\nend_line\\n",
    "group|1650|synthetic|snapshot_group|key=exclusive_group_1650_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=1650|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_1650_snapshot_group_plugin\\nend_line\\n",
    "group|1700|synthetic|snapshot_group|key=exclusive_group_1700_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=11|invalidates=1700|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_1700_snapshot_group_plugin\\nend_line\\n",
    "group|1800|synthetic|snapshot_group|key=exclusive_group_1800_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=1800|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_1800_snapshot_group_plugin\\nend_line\\n",
    "group|1900|synthetic|snapshot_group|key=exclusive_group_1900_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=1900|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_1900_snapshot_group_plugin\\nend_line\\n",
    "group|2000|runtime|step_gcode_plugin|key=step_gcode_plugin|label=G-code plugin|tooltip=Selects how STEP_GCODE creates the output artifact.|type=8|container=1|preset=7|category=11|invalidates=2000|mode=22|gui=7|plugin_ids=1|technology=1|default=gcode.printing_plan_file_writer|values=gcode.printing_plan_file_writer|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:G-code plugin\\nsetting:step_gcode_plugin\\nend_line\\n",
    "group|2000|synthetic|snapshot_group|key=exclusive_group_2000_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=7|category=11|invalidates=2000|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_2000_snapshot_group_plugin\\nend_line\\n",
    "group|200|synthetic|snapshot_group|key=exclusive_group_200_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=200|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_200_snapshot_group_plugin\\nend_line\\n",
    "group|300|runtime|test.exclusive.selector_pruning|key=exclusive_group_300_test_exclusive_selector_pruning_plugin|label=test.exclusive.selector_pruning|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=300|mode=22|gui=7|plugin_ids=1|technology=1|default=slic3r.test.exclusive_pruning.failed|values=slic3r.test.exclusive_pruning.failed,slic3r.test.exclusive_pruning.valid|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:test.exclusive.selector_pruning\\nsetting:exclusive_group_300_test_exclusive_selector_pruning_plugin\\nend_line\\n",
    "group|300|synthetic|snapshot_group|key=exclusive_group_300_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=300|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_300_snapshot_group_plugin\\nend_line\\n",
    "group|400|synthetic|snapshot_group|key=exclusive_group_400_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=400|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_400_snapshot_group_plugin\\nend_line\\n",
    "group|5000|synthetic|snapshot_group|key=exclusive_group_5000_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=1|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_5000_snapshot_group_plugin\\nend_line\\n",
    "group|500|synthetic|snapshot_group|key=exclusive_group_500_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=1|invalidates=500|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_500_snapshot_group_plugin\\nend_line\\n",
    "group|600|runtime|step_perimeter_plugin|key=step_perimeter_plugin|label=Perimeter plugin|tooltip=Choose which active plugin owns this exclusive slicing step.|type=8|container=1|preset=1|category=1|invalidates=600|mode=22|gui=7|plugin_ids=1|technology=1|default=perimeter.generator.simple|values=perimeter.generator.simple,perimeter.generator.classic,perimeter.generator.arachne|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Perimeter step plugin\\nsetting:step_perimeter_plugin\\nend_line\\n",
    "group|600|synthetic|snapshot_group|key=exclusive_group_600_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=1|invalidates=600|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_600_snapshot_group_plugin\\nend_line\\n",
    "group|700|runtime|perimeter.post_process.extra_perimeters_on_overhangs|key=exclusive_group_700_perimeter_post_process_extra_perimeters_on_overhangs_plugin|label=Extra overhang perimeter strategy|tooltip=Choose the algorithm used to add extra perimeter anchors under overhangs.|type=8|container=1|preset=1|category=1|invalidates=700|mode=22|gui=7|plugin_ids=1|technology=1|default=perimeter.post_process.extra_perimeters_on_overhangs|values=perimeter.post_process.extra_perimeters_on_overhangs,perimeter.post_process.extra_perimeter_overhang_wave|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Extra overhang perimeter strategy\\nsetting:exclusive_group_700_perimeter_post_process_extra_perimeters_on_overhangs_plugin\\nend_line\\n",
    "group|700|synthetic|snapshot_group|key=exclusive_group_700_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=1|invalidates=700|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_700_snapshot_group_plugin\\nend_line\\n",
    "group|750|synthetic|snapshot_group|key=exclusive_group_750_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=750|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_750_snapshot_group_plugin\\nend_line\\n",
    "group|800|synthetic|snapshot_group|key=exclusive_group_800_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=800|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_800_snapshot_group_plugin\\nend_line\\n",
    "group|900|synthetic|snapshot_group|key=exclusive_group_900_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=3|invalidates=900|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_900_snapshot_group_plugin\\nend_line\\n",
    "group|950|synthetic|snapshot_group|key=exclusive_group_950_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=3|invalidates=950|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_950_snapshot_group_plugin\\nend_line\\n",
    "group|dynamic|synthetic|snapshot_group|key=exclusive_group_slic3r_test_selector_snapshot_snapshot_group_plugin|label=snapshot_group|tooltip=Choose which active plugin owns this exclusive plugin group.|type=8|container=1|preset=1|category=2|invalidates=1000|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:snapshot_group\\nsetting:exclusive_group_slic3r_test_selector_snapshot_snapshot_group_plugin\\nend_line\\n",
    "select|ordering_active_1_group|ordering.extrusion_tree.default",
    "select|ordering_active_1_step|ordering.extrusion_tree.default",
    "select|ordering_active_2_group|ordering.extrusion_tree.xy_serpentine",
    "select|ordering_active_2_step|ordering.extrusion_tree.xy_serpentine",
    "select|ordering_active_3_group|ordering.extrusion_tree.advanced",
    "select|ordering_active_3_step|ordering.extrusion_tree.advanced",
    "select|ordering_runtime_group|ordering.extrusion_tree.default",
    "select|ordering_runtime_step|ordering.plan_builder.default,ordering.object_instance_catalog,ordering.custom_gcode_tool_overrides,ordering.tool_groups.default,ordering.custom_gcode_event_tools,ordering.printing_extrusion.presort.default,ordering.extrusion_tree.default,ordering.wipe_into_first.default",
    "select|probe_unknown_absent_group|test.snapshot.selector.low",
    "select|probe_unknown_absent_step|test.snapshot.selector.low",
    "select|probe_unknown_group|test.snapshot.selector.low",
    "select|probe_unknown_null_group|test.snapshot.selector.low",
    "select|probe_unknown_null_step|test.snapshot.selector.low",
    "select|probe_unknown_single_group|test.snapshot.selector.low",
    "select|probe_unknown_single_step|test.snapshot.selector.low",
    "select|probe_unknown_step|test.snapshot.selector.low",
    "select|probe_unknown_valid_single_group|test.snapshot.selector.low",
    "select|probe_unknown_valid_single_step|test.snapshot.selector.low",
    "select|seam_unknown_absent_group|seam_placer.cost",
    "select|seam_unknown_absent_step|seam_placer.cost",
    "select|seam_unknown_group|none",
    "select|seam_unknown_null_group|seam_placer.cost",
    "select|seam_unknown_null_step|seam_placer.cost",
    "select|seam_unknown_single_group|seam_placer.cost",
    "select|seam_unknown_single_step|seam_placer.cost",
    "select|seam_unknown_step|none",
    "select|seam_unknown_valid_single_group|seam_placer.cost",
    "select|seam_unknown_valid_single_step|seam_placer.cost",
    "select|thumbnail_unknown_absent_group|PNG",
    "select|thumbnail_unknown_absent_step|PNG",
    "select|thumbnail_unknown_group|none",
    "select|thumbnail_unknown_null_group|PNG",
    "select|thumbnail_unknown_null_step|PNG",
    "select|thumbnail_unknown_single_group|none",
    "select|thumbnail_unknown_single_step|PNG",
    "select|thumbnail_unknown_step|none",
    "select|thumbnail_unknown_valid_single_group|PNG",
    "select|thumbnail_unknown_valid_single_step|PNG",
    "table|1000|key=step_infill_plugin|label=Infill plugin|tooltip=Choose which active plugin owns this exclusive slicing step.|type=8|container=1|preset=1|category=3|invalidates=1000|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Infill step plugin\\nsetting:step_infill_plugin\\nend_line\\n",
    "table|100|key=step_layer_height_plugin|label=Layer height plugin|tooltip=Choose which active plugin owns this exclusive slicing step.|type=8|container=1|preset=1|category=2|invalidates=100|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Layer height step plugin\\nsetting:step_layer_height_plugin\\nend_line\\n",
    "table|10300|key=gcode_firmware_plugin|label=G-code firmware|tooltip=Choose which active plugin owns this exclusive slicing step.|type=8|container=1|preset=7|category=11|invalidates=2000|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:G-code firmware\\nsetting:gcode_firmware_plugin\\nend_line\\n",
    "table|1300|key=step_support_plugin|label=Support plugin|tooltip=Choose which active plugin owns this exclusive slicing step.|type=8|container=1|preset=1|category=7|invalidates=1300|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Support step plugin\\nsetting:step_support_plugin\\nend_line\\n",
    "table|1600|key=step_wipetower_plugin|label=Wipe tower plugin|tooltip=Choose which active plugin owns this exclusive slicing step.|type=8|container=1|preset=1|category=11|invalidates=1600|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Wipe tower step plugin\\nsetting:step_wipetower_plugin\\nend_line\\n",
    "table|1700|key=step_layer_stiching_plugin|label=Layer stitching plugin|tooltip=Choose which active plugin owns this exclusive slicing step.|type=8|container=1|preset=1|category=11|invalidates=1700|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Layer stitching step plugin\\nsetting:step_layer_stiching_plugin\\nend_line\\n",
    "table|2000|key=step_gcode_plugin|label=G-code plugin|tooltip=Choose which active plugin owns this exclusive slicing step.|type=8|container=1|preset=7|category=11|invalidates=2000|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:G-code step plugin\\nsetting:step_gcode_plugin\\nend_line\\n",
    "table|200|key=step_slicing_plugin|label=Slicing plugin|tooltip=Choose which active plugin owns this exclusive slicing step.|type=8|container=1|preset=1|category=2|invalidates=200|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Slicing step plugin\\nsetting:step_slicing_plugin\\nend_line\\n",
    "table|600|key=step_perimeter_plugin|label=Perimeter plugin|tooltip=Choose which active plugin owns this exclusive slicing step.|type=8|container=1|preset=1|category=1|invalidates=600|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Perimeter step plugin\\nsetting:step_perimeter_plugin\\nend_line\\n",
    "table|950|key=step_infill_group_plugin|label=Infill grouping plugin|tooltip=Choose which active plugin owns this exclusive slicing step.|type=8|container=1|preset=1|category=3|invalidates=950|mode=22|gui=7|plugin_ids=1|technology=1|default=|values=|ui=page:Notes\\ngroup:Exclusive step plugins\\nline:Infill grouping step plugin\\nsetting:step_infill_group_plugin\\nend_line\\n",
    "ui|exclusive_group_100_flat_area_layer_height_plugin|print.ui|flat_area_layer_height|priority=0|page:Slicing\\ngroup:Layer height\\nline:Flat area layer matching\\nsetting:layer_height_min_flat_area\\nend_line\\n",
    "ui|exclusive_group_10250_seam_placer_plugin_plugin|print.ui|seam_placer_plugin|priority=0|page:Perimeters & Shell\\ngroup:Seam\\nline:Seam placement\\nsetting:exclusive_group_10250_seam_placer_plugin_plugin\\nend_line\\n",
    "ui|exclusive_group_1200_support_demand_bridge_removal_plugin|print.ui|support.demand.bridge_removal|priority=0|page:Support material\\ngroup:Options for support material and raft\\nsetting:insert$aftersetting$support_material_xy_spacing:dont_support_bridges\\n",
    "ui|exclusive_group_1800_nozzle_cleaning_plugin|printer_fff.ui|nozzle_cleaning|priority=0|page:Custom G-code\\ngroup:Nozzle cleaning\\nsetting:nozzle_cleaning_gcode\\nsetting:nozzle_cleaning_interval_layers\\nsetting:nozzle_cleaning_interval_volume\\nsetting:nozzle_cleaning_interval_retractions\\nsetting:nozzle_cleaning_interval_toolchanges\\nsetting:nozzle_cleaning_when\\nend_group\\n",
    "ui|exclusive_group_300_max_overhang_threshold_plugin|print.ui|max_overhang_threshold|priority=1|page:Slicing\\ngroup:Modifying slices\\nline:Overhangs cut\\nsetting:overhangs_max_slope\\nsetting:overhangs_bridge_threshold\\nsetting:overhangs_bridge_upper_layers\\nend_line\\n",
    "ui|exclusive_group_300_overhang_slope_fill_plugin|print.ui|overhang_slope_fill|priority=1|page:Slicing\\ngroup:Modifying slices\\nline:Overhangs fill\\nsetting:overhangs_fill_slope\\nsetting:overhangs_fill_bridge_threshold\\nend_line\\n",
    "ui|exclusive_group_700_perimeter_post_process_extra_perimeters_on_overhangs_plugin|print.ui|perimeter.post_process.extra_perimeters_on_overhangs|priority=1|page:Perimeters & Shell\\ngroup:Quality\\nline:Extra perimeters\\nsetting:insert$aftersetting$extra_perimeters_on_overhangs:exclusive_group_700_perimeter_post_process_extra_perimeters_on_overhangs_plugin\\nend_line\\n",
    "ui|gcode_firmware_plugin|printer_fff.ui|gcode_firmware_plugin|priority=1|page:General:printer\\ngroup:Firmware\\n\\tsetting:insert$aftersetting$step_gcode_plugin:gcode_firmware_plugin\\n",
    "ui|step_gcode_plugin|printer_fff.ui|step_gcode_plugin|priority=0|page:General:printer\\ngroup:Firmware\\n\\tsetting:insert$beforesetting$gcode_flavor:step_gcode_plugin\\n",
    "ui|step_layer_height_plugin|print.ui|step_layer_height_plugin|priority=0|page:Notes\\ngroup:Exclusive step plugins\\nline:Layer height step plugin\\nsetting:step_layer_height_plugin\\nend_line\\n",
    "ui|step_perimeter_plugin|print.ui|step_perimeter_plugin|priority=0|page:Notes\\ngroup:Exclusive step plugins\\nline:Perimeter step plugin\\nsetting:step_perimeter_plugin\\nend_line\\n",
    "ui|thumbnails_format|printer_fff.ui|thumbnail_encoder_plugin|priority=1|page:General:printer\\ngroup:Firmware\\n\\tsetting:insert$beforesetting$thumbnails:thumbnails_format\\n",
    };
    std::vector<std::string> expected;
    for (const char *row : k_expected) if (row != nullptr) expected.emplace_back(row);
    std::sort(expected.begin(), expected.end());
    std::vector<std::string> missing, extra;
    std::set_difference(expected.begin(), expected.end(), rows.begin(), rows.end(), std::back_inserter(missing));
    std::set_difference(rows.begin(), rows.end(), expected.begin(), expected.end(), std::back_inserter(extra));
    for (const std::string &row : missing) UNSCOPED_INFO("Missing: " << row);
    for (const std::string &row : extra) UNSCOPED_INFO("Extra: " << row);
    if (!missing.empty() || !extra.empty()) FAIL("Generated selector snapshot differs");
}
