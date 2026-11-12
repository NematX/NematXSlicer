#include <catch2/catch.hpp>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/OrchestratorViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeThumbnailViews.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

/*
These tests check declarations that describe plugin selectors, settings that choose registered plugins.
StepSelectorTraits.hpp defines these declarations. A declaration for an entire step supplies defaults
to named groups; a declaration for an exact group replaces every field.
Description checks read the plugin manager without registering settings. Selection checks
initialize the shared runtime and restore activation flags afterward. Test providers that do no work
register after startup and remain inactive outside their selection checks. The generated-selector snapshot
checks the serialized definitions and layout strings separately.

Operation notes:
- TEST_CASE (declaration checks)
- Orchestrator::register_builtin_step_selector
- Steps::plugin_exclusive_group_definition
- Steps::make_exclusive_step_group
- TEST_CASE (selection checks)
- check_seam_selection :  check_selection

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- Test::Plugins::ensure_plugin_test_runtime_initialized()
        |-- Orchestrator::instance()
        |-- Steps::plugin_exclusive_group_option_key()
        |-- PrintConfigDef::instance()
        |-- host.registered_plugins()
        `-- check_seam_selection()
*/

namespace {
using namespace Slic3r;

// Check that the group, C API and step selection paths return the expected seam provider, or no provider.
void check_seam_selection(Orchestrator &host, const ConfigBase *config, const Plugin *expected);
// Check that the group, C API and step paths select the expected provider, or no provider.
void check_selection(Orchestrator &host, slicing_step_t step, const char *group,
                     const ConfigBase *config, const Plugin *expected);

class SeamActivationGuard
{
public:
    // Save all seam provider activation flags so the test can change them temporarily.
    explicit SeamActivationGuard(Orchestrator &host) : m_host(host)
    {
        for (Plugin *plugin : host.registered_plugins())
            if (plugin->get_step() == SEAM_PLACER) m_saved.emplace_back(plugin, host.is_plugin_active(plugin));
    }
    // Restore the saved activation flags; the plugin manager and providers must outlive this guard.
    ~SeamActivationGuard()
    {
        for (const std::pair<Plugin *, bool> &entry : m_saved) m_host.set_plugin_active(entry.first, entry.second);
    }
private:
    Orchestrator &m_host;
    std::vector<std::pair<Plugin *, bool>> m_saved;
};

class ThumbnailActivationGuard
{
public:
    // Save activation flags for the supplied step so the test can change them temporarily.
    explicit ThumbnailActivationGuard(Orchestrator &host, slicing_step_t step = GCODE_THUMBNAIL) : m_host(host)
    {
        for (Plugin *plugin : host.registered_plugins())
            if (plugin->get_step() == step) m_saved.emplace_back(plugin, host.is_plugin_active(plugin));
    }
    // Restore the saved activation flags; the plugin manager and providers must outlive this guard.
    ~ThumbnailActivationGuard()
    {
        for (const std::pair<Plugin *, bool> &entry : m_saved) m_host.set_plugin_active(entry.first, entry.second);
    }
private:
    Orchestrator &m_host;
    std::vector<std::pair<Plugin *, bool>> m_saved;
};

class SingletonSelectorProbe final : public slic3r_api::PluginBase
{
public:
    // Keep the plugin manager and id strings, which must outlive this provider, and copy the step and priorities.
    SingletonSelectorProbe(orchestrator_handle *host, slicing_step_t step, const char *id, const char *group,
                           int32_t group_priority = 100, int32_t execution_priority = 0)
        : PluginBase(host), m_step(step), m_id(id), m_group(group), m_group_priority(group_priority),
          m_execution_priority(execution_priority) {}
    // Return the provider id supplied to the constructor.
    const char *id() const noexcept { return m_id; }
private:
    // Return the provider id supplied to the constructor.
    const char *id_impl() const noexcept override { return m_id; }
    // Return the step supplied to the constructor.
    slicing_step_t step_impl() const noexcept override { return m_step; }
    // Return the group id supplied to the constructor.
    const char *exclusive_group_impl() const noexcept override { return m_group; }
    // Report that this provider has no dependencies.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Return the stored execution priority. Nothing is changed or transferred, and the call cannot fail.
    int32_t priority_impl() const noexcept override { return m_execution_priority; }
    // Return the immutable group preference without changing execution priority or provider state.
    int32_t exclusive_group_priority_impl() const noexcept override { return m_group_priority; }
    // Do no work when this provider is run.
    void run_impl(const plugin_run_context *) const override {}
    slicing_step_t m_step;
    const char *m_id;
    const char *m_group;
    int32_t m_group_priority;
    int32_t m_execution_priority;
};

class SelectorTraitsProbe final : public slic3r_api::PluginBase
{
public:
    // Create an unregistered provider used only to supply selector description text.
    explicit SelectorTraitsProbe(orchestrator_handle *host) : PluginBase(host) {}
private:
    // Return the fixed id for this test provider.
    const char *id_impl() const noexcept override { return "slic3r.test.selector_traits.probe"; }
    // Assign this provider to the post-infill step.
    slicing_step_t step_impl() const noexcept override { return STEP_POST_INFILL; }
    // Report that this provider has no dependencies.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Give this provider execution priority zero.
    int32_t priority_impl() const noexcept override { return 0; }
    // Do no work when this provider is run.
    void run_impl(const plugin_run_context *) const override {}
};

void check_seam_selection(Orchestrator &host, const ConfigBase *config, const Plugin *expected)
{
    check_selection(host, SEAM_PLACER, "seam_placer_plugin", config, expected);
}

void check_selection(Orchestrator &host, slicing_step_t step, const char *group,
                     const ConfigBase *config, const Plugin *expected)
{
    CHECK(Steps::selected_active_plugin_from_group(host, step, group, config) == expected);
    CHECK(orchestrator_select_plugin(reinterpret_cast<orchestrator_handle *>(&host),
          reinterpret_cast<const config_handle *>(config), step, group) ==
          reinterpret_cast<const orchestrator_plugin_handle *>(expected));
    const std::vector<Plugin *> selected = Steps::selected_or_active_plugins_for_step(host, step, config);
    if (expected != nullptr) {
        REQUIRE(selected.size() == 1);
        CHECK(selected.front() == expected);
    } else CHECK(selected.empty());
}
} // namespace

// Register permanent no-op plugins as application-supplied descriptions and build only test-owned selectors.
// Temporary activation is restored on failure. The application owns copied descriptions, but no application setting is inserted.
TEST_CASE("Exclusive group definitions prefer active declarants then registered declarants",
          "[plugins][step-pipeline][selector-traits][selector-group-priority]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&host);
    const slicing_step_t step = host.register_step("slic3r.test.definition_fallback", STEP_INFILL);
    REQUIRE(step != STEP_NONE);
    static SingletonSelectorProbe preferred(handle, step, "slic3r.test.definition_fallback.a", "fallback.Mixed-id", 0);
    static SingletonSelectorProbe other(handle, step, "slic3r.test.definition_fallback.b", "fallback.Mixed-id", 20);
    static SingletonSelectorProbe undeclared(handle, step, "slic3r.test.definition_fallback.undeclared", "fallback.Mixed-id", -10);
    {
        Orchestrator::PluginRegistrationScope scope = host.plugin_registration_scope({}, false);
        for (SingletonSelectorProbe *probe : {&other, &preferred, &undeclared}) {
            if (host.get_plugin(probe->id()) == nullptr) REQUIRE(host.register_plugin(probe->c_instance()));
            REQUIRE(host.set_plugin_active(probe->id(), false));
        }
        for (SingletonSelectorProbe *probe : {&preferred, &other}) {
            std::string label = probe == &preferred ? "Preferred label" : "Other label";
            std::string tooltip = probe == &preferred ? "Preferred tooltip" : "Other tooltip";
            std::string owner = probe->id();
            raw_exclusive_group_def declaration = raw_exclusive_group_def_init();
            declaration.exclusive_group = "fallback.Mixed-id";
            declaration.declaring_plugin_id = owner.c_str();
            declaration.label = label.c_str();
            declaration.tooltip = tooltip.c_str();
            declaration.container_type = probe == &preferred ? RAW_CONTAINER_TYPE_REGION : RAW_CONTAINER_TYPE_OBJECT;
            CHECK(orchestrator_register_exclusive_group(handle, step, &declaration) >= 0);
            label.assign("overwritten");
            tooltip.assign("overwritten");
            owner.assign("overwritten");
        }
    }
    ThumbnailActivationGuard guard(host, step);
    REQUIRE(host.set_plugin_active(undeclared.id(), true));
    host.elect_exclusive_group_definition(step, "fallback.Mixed-id");
    const Orchestrator::ExclusiveGroupDefinition inactive = host.exclusive_group_definition_or_default(step, "fallback.Mixed-id");
    CHECK(inactive.plugin_id == preferred.id());
    CHECK(inactive.label == "Preferred label");
    CHECK(inactive.tooltip == "Preferred tooltip");
    CHECK(inactive.traits.container_type == RAW_CONTAINER_TYPE_REGION);
    CHECK(Steps::selected_active_plugin_from_group(host, step, "fallback.Mixed-id", nullptr) == host.get_plugin(undeclared.id()));

    REQUIRE(host.set_plugin_active(other.id(), true));
    CHECK(host.exclusive_group_definition_or_default(step, "fallback.Mixed-id").plugin_id == preferred.id());
    host.elect_exclusive_group_definition(step, "fallback.Mixed-id");
    const Orchestrator::ExclusiveGroupDefinition active = host.exclusive_group_definition_or_default(step, "fallback.Mixed-id");
    CHECK(active.plugin_id == other.id());
    CHECK(active.label == "Other label");
    CHECK(active.tooltip == "Other tooltip");
    CHECK(active.traits.container_type == RAW_CONTAINER_TYPE_OBJECT);
    Steps::StepExclusiveGroup local = Steps::plugin_exclusive_group_definition(host, step, "fallback.Mixed-id", *host.get_plugin(other.id()));
    local.refresh_storage_pointers();
    CHECK(std::string(local.option_def.label) == "Other label");
    CHECK(std::string(local.option_def.tooltip) == "Other tooltip");
    CHECK(local.ui_fragment.find("line:\nsetting:") != std::string::npos);
    REQUIRE(host.set_plugin_active(preferred.id(), true));
    CHECK(host.exclusive_group_definition_or_default(step, "fallback.Mixed-id").plugin_id == other.id());
    host.elect_exclusive_group_definition(step, "fallback.Mixed-id");
    CHECK(host.exclusive_group_definition_or_default(step, "fallback.Mixed-id").plugin_id == preferred.id());
}

// Check complete descriptions through C before retaining descriptions or key reservations, using permanent no-op plugins.
// Rejection preserves accepted data. Test-owned builders never insert settings after application finalization.
TEST_CASE("Plugin steps declare the selector of their exclusive groups",
          "[plugins][orchestrator][service-step][selector-traits]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&host);
    const slicing_step_t step = host.register_step("slic3r.test.complete_descriptor", STEP_INFILL);
    static SingletonSelectorProbe a(handle, step, "slic3r.test.complete_descriptor.a", "descriptor.a");
    static SingletonSelectorProbe b(handle, step, "slic3r.test.complete_descriptor.b", "descriptor.b");
    const slicing_step_t other_step = host.register_step("slic3r.test.complete_descriptor.other", STEP_INFILL);
    static SingletonSelectorProbe elsewhere(handle, other_step, "slic3r.test.complete_descriptor.elsewhere", "descriptor.a");
    {
        Orchestrator::PluginRegistrationScope scope = host.plugin_registration_scope({}, false);
        for (SingletonSelectorProbe *probe : {&a, &b, &elsewhere}) {
            if (host.get_plugin(probe->id()) == nullptr) REQUIRE(host.register_plugin(probe->c_instance()));
            REQUIRE(host.set_plugin_active(probe->id(), false));
        }
    }
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = "descriptor.a";
    definition.declaring_plugin_id = a.id();
    CHECK(orchestrator_register_exclusive_group(handle, step, &definition) == -1);
    Orchestrator::PluginRegistrationScope scope = host.plugin_registration_scope({}, false);
    CHECK(orchestrator_register_exclusive_group(handle, step, nullptr) == -1);
    CHECK(orchestrator_register_exclusive_group(nullptr, step, &definition) == -1);
    raw_exclusive_group_def invalid = definition;
    invalid.struct_size = sizeof(definition) - 1;
    CHECK(orchestrator_register_exclusive_group(handle, step, &invalid) == -1);
    CHECK(orchestrator_register_exclusive_group(handle, STEP_NONE, &definition) == -1);
    CHECK(orchestrator_register_exclusive_group(handle, STEP_ANY, &definition) == -1);
    for (const char *owner : {"", "not.registered", b.id(), elsewhere.id()}) {
        invalid = definition;
        invalid.declaring_plugin_id = owner;
        CHECK(orchestrator_register_exclusive_group(handle, step, &invalid) == -1);
    }
    invalid = definition;
    invalid.declaring_plugin_id = nullptr;
    CHECK(orchestrator_register_exclusive_group(handle, step, &invalid) == -1);
    {
        Orchestrator::PluginRegistrationScope foreign = host.plugin_registration_scope("plugins/descriptor-foreign", true);
        invalid = definition;
        invalid.option_key = "slic3r_test_descriptor_unclaimed";
        CHECK(orchestrator_register_exclusive_group(handle, step, &invalid) == -1);
    }
    for (const char *key : {"1invalid", "invalid key", "invalid.key"}) {
        invalid = definition;
        invalid.option_key = key;
        CHECK(orchestrator_register_exclusive_group(handle, step, &invalid) == -1);
    }
    invalid = definition;
    invalid.option_key = "layer_height";
    CHECK(orchestrator_register_exclusive_group(handle, step, &invalid) == -2);
    invalid = definition;
    invalid.ui_target_file = "nope.ui";
    CHECK(orchestrator_register_exclusive_group(handle, step, &invalid) == -1);
    invalid = definition;
    invalid.ui_page = "Unpaired";
    CHECK(orchestrator_register_exclusive_group(handle, step, &invalid) == -1);
    for (slicing_step_t target : {STEP_ANY, GCODE_THUMBNAIL, slicing_step_t(5000)}) {
        invalid = definition;
        invalid.has_invalidates_step = 1;
        invalid.invalidates_step = target;
        CHECK(orchestrator_register_exclusive_group(handle, step, &invalid) == -1);
    }
    invalid = definition;
    invalid.has_value_when_absent = 1;
    invalid.value_when_absent = nullptr;
    CHECK(orchestrator_register_exclusive_group(handle, step, &invalid) == -1);
    definition.label = "Complete descriptor";
    definition.tooltip = "Complete tooltip";
    definition.option_key = "slic3r_test_complete_descriptor";
    definition.has_invalidates_step = 1;
    definition.invalidates_step = STEP_NONE;
    definition.has_value_when_absent = 1;
    definition.value_when_absent = a.id();
    definition.read_with_one_provider = 1;
    definition.show_with_one_provider = 1;
    definition.strict_selection = 1;
    definition.provider_settings_rules = 0;
    definition.ui_target_file = nullptr;
    CHECK(orchestrator_register_exclusive_group(handle, step, &definition) >= 0);
    definition.ui_target_file = "";
    CHECK(orchestrator_register_exclusive_group(handle, step, &definition) == 0);
    invalid = definition;
    invalid.label = "Conflicting owner";
    CHECK(orchestrator_register_exclusive_group(handle, step, &invalid) == -2);
    invalid = definition;
    invalid.exclusive_group = "descriptor.b";
    invalid.declaring_plugin_id = b.id();
    CHECK(orchestrator_register_exclusive_group(handle, step, &invalid) == -2);
    invalid.option_key = "slic3r_test_descriptor_unclaimed";
    CHECK(orchestrator_register_exclusive_group(handle, step, &invalid) >= 0);
    struct FutureDescriptor { raw_exclusive_group_def definition; uint64_t unknown; } future {definition, 42};
    future.definition.struct_size = sizeof(future);
    CHECK(orchestrator_register_exclusive_group(handle, step, &future.definition) == 0);
    definition.declaring_plugin_id = b.id();
    CHECK_FALSE(a.register_exclusive_group(step, definition));
    CHECK(definition.declaring_plugin_id == b.id());
    slic3r_api::OrchestratorView view(handle);
    CHECK_THROWS_AS(view.register_exclusive_group(step, definition), std::invalid_argument);
    definition.declaring_plugin_id = a.id();
    CHECK_FALSE(view.register_exclusive_group(step, definition));
    host.elect_exclusive_group_definition(step, "descriptor.a");
    const Orchestrator::ExclusiveGroupDefinition retained = host.exclusive_group_definition_or_default(step, "descriptor.a");
    CHECK(retained.label == "Complete descriptor");
    CHECK(retained.tooltip == "Complete tooltip");
    CHECK(retained.traits.ui_target_file == "print.ui");
    CHECK(retained.traits.read_with_one_provider);
    CHECK(retained.traits.show_with_one_provider);
    CHECK_FALSE(retained.traits.provider_settings_rules);
    REQUIRE(retained.traits.value_when_absent.has_value());
    CHECK(*retained.traits.value_when_absent == a.id());
    Steps::StepExclusiveGroup local = Steps::plugin_exclusive_group_definition(host, step, "descriptor.a", *host.get_plugin(a.id()));
    CHECK(local.option_key_storage == "slic3r_test_complete_descriptor");
    CHECK(local.option_def.invalidates_step == STEP_NONE);
}

// Read default group text and application-supplied descriptions through the same accessor and test-owned builders.
// A member description retains copied strings but cannot replace application text or rules. No application setting is inserted.
TEST_CASE("Undeclared group text is exact and protected host group text wins",
          "[plugins][step-pipeline][selector-traits]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&host);
    const Orchestrator::ExclusiveGroupDefinition absent = host.exclusive_group_definition_or_default(STEP_POST_INFILL, "Exact.Mixed-id_42");
    CHECK(absent.label == "Exact.Mixed-id_42");
    CHECK(absent.tooltip.empty());
    CHECK(absent.plugin_id.empty());
    CHECK(absent.traits.category == RAW_OPTION_CATEGORY_INFILL);
    static SingletonSelectorProbe member(handle, STEP_ORDERING, "slic3r.test.protected_group.member", "ordering.extrusion_tree", -100, 1050);
    const Orchestrator::ExclusiveGroupDefinition before = host.exclusive_group_definition_or_default(STEP_ORDERING, "ordering.extrusion_tree");
    {
        Orchestrator::PluginRegistrationScope scope = host.plugin_registration_scope({}, false);
        if (host.get_plugin(member.id()) == nullptr) REQUIRE(host.register_plugin(member.c_instance()));
        REQUIRE(host.set_plugin_active(member.id(), false));
        raw_exclusive_group_def declaration = raw_exclusive_group_def_init();
        declaration.exclusive_group = "ordering.extrusion_tree";
        declaration.declaring_plugin_id = member.id();
        declaration.label = "Member label";
        declaration.tooltip = "Member tooltip";
        declaration.container_type = RAW_CONTAINER_TYPE_REGION;
        CHECK(orchestrator_register_exclusive_group(handle, STEP_ORDERING, &declaration) >= 0);
    }
    ThumbnailActivationGuard guard(host, STEP_ORDERING);
    REQUIRE(host.set_plugin_active(member.id(), true));
    host.elect_exclusive_group_definition(STEP_ORDERING, "ordering.extrusion_tree");
    const Orchestrator::ExclusiveGroupDefinition after = host.exclusive_group_definition_or_default(STEP_ORDERING, "ordering.extrusion_tree");
    CHECK(after.label == before.label);
    CHECK(after.tooltip == before.tooltip);
    CHECK(after.translation_domain == before.translation_domain);
    CHECK(after.traits.container_type == before.traits.container_type);
    CHECK(after.traits.category == before.traits.category);
    CHECK(after.plugin_id.empty());
    const Orchestrator::ExclusiveGroupDefinition whole = host.exclusive_group_definition_or_default(STEP_PERIMETER, "step_perimeter_plugin");
    CHECK(whole.label == "Perimeter plugin");
    CHECK_FALSE(whole.tooltip.empty());
}

// Register permanent no-op alternatives with equal execution priority and different preference numbers.
// The test owns choice lists and restores activation afterward. Wrappers stay borrowed, and no application setting is inserted.
TEST_CASE("Exclusive group priority chooses the active selector default independently of execution",
          "[plugins][step-pipeline][selector-group-priority]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&host);
    const slicing_step_t step = host.register_step("slic3r.test.priority_defaults", STEP_INFILL);
    static SingletonSelectorProbe high(handle, step, "slic3r.test.priority_defaults.high", "priority", 25);
    static SingletonSelectorProbe low(handle, step, "slic3r.test.priority_defaults.low", "priority", 0);
    for (SingletonSelectorProbe *probe : {&high, &low}) {
        if (host.get_plugin(probe->id()) == nullptr) REQUIRE(host.register_plugin(probe->c_instance()));
        REQUIRE(host.set_plugin_active(probe->id(), false));
    }
    ThumbnailActivationGuard guard(host, step);
    REQUIRE(host.set_plugin_active(high.id(), true));
    REQUIRE(host.set_plugin_active(low.id(), true));
    Plugin *preferred = host.get_plugin(low.id());
    std::vector<Plugin *> candidates {host.get_plugin(high.id()), preferred};
    CHECK(Steps::default_provider_from_group(candidates) == preferred);
    Steps::StepExclusiveGroup local = Steps::plugin_exclusive_group_definition(host, step, "priority", *preferred);
    local.set_enum_plugins(host, {{high.id(), "High"}, {low.id(), "Low"}}, low.id());
    CHECK((local.enum_values == std::vector<std::string>{low.id(), high.id()}));
    local.refresh_storage_pointers();
    CHECK(std::string(local.option_def.default_serialized_value) == low.id());
    DynamicPrintConfig config;
    check_selection(host, step, "priority", &config, preferred);
    config.set_key_value(local.option_key_storage, new ConfigOptionString("missing"));
    check_selection(host, step, "priority", &config, preferred);
    config.set_key_value(local.option_key_storage, new ConfigOptionString(high.id()));
    check_selection(host, step, "priority", &config, host.get_plugin(high.id()));
    REQUIRE(host.set_plugin_active(low.id(), false));
    check_selection(host, step, "priority", nullptr, host.get_plugin(high.id()));
    REQUIRE(host.set_plugin_active(low.id(), true));
    check_selection(host, step, "priority", nullptr, preferred);
    CHECK(host.get_plugin(high.id())->get_priority() == preferred->get_priority());
}

// Register automatic choices through C using test-owned settings and choice lists, without inserting application settings.
// The application copies arguments before they expire. Invalid choices return errors, and temporary activation is restored on failure.
TEST_CASE("The auto selector value picks the declared choice, else the group default",
          "[plugins][orchestrator][service-step][selector-auto]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&host);
    const slicing_step_t step = host.register_step("slic3r.test.auto_generic", STEP_INFILL);
    static SingletonSelectorProbe low(handle, step, "slic3r.test.auto_generic.low", "auto_generic", -10);
    static SingletonSelectorProbe high(handle, step, "slic3r.test.auto_generic.high", "auto_generic", 25);
    {
        Orchestrator::PluginRegistrationScope scope = host.plugin_registration_scope({}, false);
        for (SingletonSelectorProbe *probe : {&low, &high}) {
            if (host.get_plugin(probe->id()) == nullptr) REQUIRE(host.register_plugin(probe->c_instance()));
            REQUIRE(host.set_plugin_active(probe->id(), false));
        }
        raw_exclusive_group_def definition = raw_exclusive_group_def_init();
        definition.exclusive_group = "auto_generic";
        definition.declaring_plugin_id = low.id();
        definition.auto_value = 1;
        CHECK(orchestrator_register_exclusive_group(handle, step, &definition) >= 0);
    }
    ThumbnailActivationGuard guard(host, step);
    REQUIRE(host.set_plugin_active(low.id(), true));
    REQUIRE(host.set_plugin_active(high.id(), true));
    host.elect_exclusive_group_definition(step, "auto_generic");
    CHECK(orchestrator_register_selector_auto_choice(handle, step, "auto_generic", "style", "organic", high.id()) >= 0);
    CHECK(orchestrator_register_selector_auto_choice(handle, step, "auto_generic", "style", "organic", high.id()) == 0);
    CHECK(orchestrator_register_selector_auto_choice(handle, step, "auto_generic", "style", "organic", low.id()) == -2);
    CHECK(orchestrator_register_selector_auto_choice(handle, step, "auto_generic", "", "organic", high.id()) == -1);
    CHECK(orchestrator_register_selector_auto_choice(handle, step, "auto_generic", "style", "missing", "not.registered") >= 0);
    Steps::StepExclusiveGroup local = Steps::plugin_exclusive_group_definition(host, step, "auto_generic", *host.get_plugin(low.id()));
    local.set_enum_plugins(host, {{low.id(), "Low"}, {high.id(), "High"}}, low.id());
    local.refresh_storage_pointers();
    REQUIRE(local.enum_values.size() == 3);
    CHECK(local.enum_values.front() == SLIC3R_SELECTOR_AUTO_VALUE);
    CHECK(std::string(local.option_def.default_serialized_value) == SLIC3R_SELECTOR_AUTO_VALUE);
    DynamicPrintConfig config;
    config.set_key_value("style", new ConfigOptionString("organic"));
    check_selection(host, step, "auto_generic", &config, host.get_plugin(high.id()));
    config.set_key_value(local.option_key_storage, new ConfigOptionString(SLIC3R_SELECTOR_AUTO_VALUE));
    check_selection(host, step, "auto_generic", &config, host.get_plugin(high.id()));
    config.set_key_value("style", new ConfigOptionString("!organic"));
    check_selection(host, step, "auto_generic", &config, host.get_plugin(high.id()));
    config.set_key_value("style", new ConfigOptionString("grid"));
    check_selection(host, step, "auto_generic", &config, host.get_plugin(low.id()));
    config.set_key_value("style", new ConfigOptionString("missing"));
    check_selection(host, step, "auto_generic", &config, host.get_plugin(low.id()));
    config.set_key_value("style", new ConfigOptionString("organic"));
    config.set_key_value(local.option_key_storage, new ConfigOptionString(low.id()));
    check_selection(host, step, "auto_generic", &config, host.get_plugin(low.id()));
    config.set_key_value(local.option_key_storage, new ConfigOptionString(SLIC3R_SELECTOR_AUTO_VALUE));
    REQUIRE(host.set_plugin_active(high.id(), false));
    check_selection(host, step, "auto_generic", &config, host.get_plugin(low.id()));
    REQUIRE(host.set_plugin_active(low.id(), false));
    check_selection(host, step, "auto_generic", &config, nullptr);
}

// Compare the two overhang strategies at equal execution priority and restore post-perimeter activation afterward.
// Selection borrows registered plugins without changing settings, retained descriptions, geometry or plugin ownership.
TEST_CASE("Native overhang alternatives share execution priority and retain the shrink strategy default",
          "[plugins][step-pipeline][selector-group-priority]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    ThumbnailActivationGuard guard(host, STEP_POST_PERIMETER);
    const char *group = "perimeter.post_process.extra_perimeters_on_overhangs";
    Plugin *shrink = host.get_plugin(group);
    Plugin *wave = host.get_plugin("perimeter.post_process.extra_perimeter_overhang_wave");
    REQUIRE(shrink != nullptr);
    REQUIRE(wave != nullptr);
    CHECK(shrink->get_priority() == -20);
    CHECK(wave->get_priority() == -20);
    CHECK(shrink->get_exclusive_group_priority() == 0);
    CHECK(wave->get_exclusive_group_priority() == 10);
    for (Plugin *plugin : host.registered_plugins())
        if (plugin->get_step() == STEP_POST_PERIMETER) REQUIRE(host.set_plugin_active(plugin, false));
    REQUIRE(host.set_plugin_active(shrink, true));
    REQUIRE(host.set_plugin_active(wave, true));
    check_selection(host, STEP_POST_PERIMETER, group, nullptr, shrink);
    REQUIRE(host.set_plugin_active(shrink, false));
    check_selection(host, STEP_POST_PERIMETER, group, nullptr, wave);
}

// Check application-supplied preferences using the original registered members, then restore every stage's activation.
// Selection borrows the application and test-owned empty settings. Choice lists, retained descriptions and application settings are unchanged.
TEST_CASE("Native ordering and thumbnail defaults follow successive active preferences",
          "[plugins][step-pipeline][selector-group-priority][thumbnail]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    ThumbnailActivationGuard ordering_guard(host, STEP_ORDERING);
    ThumbnailActivationGuard thumbnail_guard(host, GCODE_THUMBNAIL);
    const char *const ordering[] = {"ordering.extrusion_tree.advanced", "ordering.extrusion_tree.xy_serpentine", "ordering.extrusion_tree.default"};
    const char *const encoders[] = {"PNG", "JPG", "QOI", "BIQU"};
    for (Plugin *plugin : host.registered_plugins())
        if (plugin->get_step() == STEP_ORDERING || plugin->get_step() == GCODE_THUMBNAIL)
            REQUIRE(host.set_plugin_active(plugin, false));
    for (const char *id : ordering) {
        REQUIRE(host.get_plugin(id) != nullptr);
        CHECK(host.get_plugin(id)->get_priority() == 1050);
        REQUIRE(host.set_plugin_active(id, true));
    }
    for (const char *id : ordering) {
        CHECK(Steps::selected_active_plugin_from_group(host, STEP_ORDERING, "ordering.extrusion_tree", nullptr) == host.get_plugin(id));
        REQUIRE(host.set_plugin_active(id, false));
    }
    CHECK(Steps::selected_active_plugin_from_group(host, STEP_ORDERING, "ordering.extrusion_tree", nullptr) == nullptr);
    for (const char *id : encoders) {
        REQUIRE(host.get_plugin(id) != nullptr);
        CHECK(host.get_plugin(id)->get_priority() == 0);
        REQUIRE(host.set_plugin_active(id, true));
    }
    DynamicPrintConfig absent;
    check_selection(host, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", &absent, host.get_plugin("PNG"));
    for (const char *id : encoders) {
        check_selection(host, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", nullptr, host.get_plugin(id));
        REQUIRE(host.set_plugin_active(id, false));
    }
    check_selection(host, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", nullptr, nullptr);
}

// Inspect startup choices after normal finalization without registering the key as a selector.
// The application description remains borrowed and unchanged. Neither a failed initializer nor the reserved automatic value may remain as member ids.
TEST_CASE("Non-selector provider enums do not preserve the reserved auto value",
          "[plugins][step-pipeline][selector-auto]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Orchestrator &host = Orchestrator::instance();
    const char *key = "test.selector.nonselector_auto";
    CHECK_FALSE(host.selector_auto_enabled(key));
    const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
    REQUIRE(definition != nullptr);
    REQUIRE(definition->enum_def.get() != nullptr);
    CHECK(definition->enum_def->values() == std::vector<std::string>{"slic3r.test.exclusive_pruning.valid"});
    REQUIRE(definition->default_value.get() != nullptr);
    CHECK(definition->default_value->serialize() == "slic3r.test.exclusive_pruning.valid");
}

// Compare equally preferred members in both orders using test-owned wrappers without registration or setting changes.
// Candidate objects remain borrowed and empty lists return null. Description and allocation errors reach the test runner.
TEST_CASE("Equal exclusive group priorities choose the smallest provider id",
          "[plugins][step-pipeline][selector-group-priority]")
{
    Orchestrator &host = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&host);
    SingletonSelectorProbe z(handle, STEP_POST_INFILL, "z.equal", "equal", 7);
    SingletonSelectorProbe a(handle, STEP_POST_INFILL, "a.equal", "equal", 7);
    Plugin last(z.c_instance(), "Slic3r", "");
    Plugin first(a.c_instance(), "Slic3r", "");
    CHECK(Steps::default_provider_from_group({&last, &first}) == &first);
    CHECK(Steps::default_provider_from_group({&first, &last}) == &first);
    CHECK(Steps::default_provider_from_group({}) == nullptr);
}

// Require equal execution priority among alternatives before registration while allowing independent groups to differ.
// Permanent test objects remain disabled and caller-owned. The application retains accepted wrappers without creating settings.
TEST_CASE("Exclusive group alternatives require equal execution priorities",
          "[plugins][orchestrator][selector-group-priority]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&host);
    const slicing_step_t step = host.register_step("slic3r.test.execution_equality", STEP_INFILL);
    static SingletonSelectorProbe first(handle, step, "slic3r.test.execution_equality.first", "same", 0, 10);
    static SingletonSelectorProbe rejected(handle, step, "slic3r.test.execution_equality.rejected", "same", -1, 20);
    static SingletonSelectorProbe different(handle, step, "slic3r.test.execution_equality.different", "different", 0, 20);
    if (host.get_plugin(first.id()) == nullptr) REQUIRE(host.register_plugin(first.c_instance()));
    REQUIRE(host.set_plugin_active(first.id(), false));
    CHECK_FALSE(host.register_plugin(rejected.c_instance()));
    CHECK(host.get_plugin(rejected.id()) == nullptr);
    if (host.get_plugin(different.id()) == nullptr) REQUIRE(host.register_plugin(different.c_instance()));
    REQUIRE(host.set_plugin_active(different.id(), false));
    static SingletonSelectorProbe whole(handle, STEP_PERIMETER, "slic3r.test.execution_equality.whole", "distinct", 0, 42);
    CHECK_FALSE(host.register_plugin(whole.c_instance()));
    CHECK(host.get_plugin(whole.id()) == nullptr);
}

// Check that a group which reads a lone provider's stored id rejects a missing id through all three selection paths.
// Compare a group that ignores that id, and restore activation flags after the checks.
TEST_CASE("Strict singleton selectors reject a stale stored provider through every selection path",
          "[plugins][step-pipeline][selector-traits]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    const slicing_step_t step = host.register_step("slic3r.test.strict_singleton", STEP_INFILL);
    REQUIRE(step != STEP_NONE);
    static SingletonSelectorProbe strict(reinterpret_cast<orchestrator_handle *>(&host), step,
        "slic3r.test.strict_singleton.provider", "strict_singleton");
    static SingletonSelectorProbe neutral(reinterpret_cast<orchestrator_handle *>(&host), step,
        "slic3r.test.neutral_singleton.provider", "neutral_singleton");
    for (SingletonSelectorProbe *probe : {&strict, &neutral}) {
        if (host.get_plugin(probe->id()) == nullptr) REQUIRE(host.register_plugin(probe->c_instance()));
        REQUIRE(host.set_plugin_active(host.get_plugin(probe->id()), false));
    }
    StepSelectorTraits traits;
    traits.exclusive_group = "strict_singleton";
    traits.strict_selection = true;
    traits.read_with_one_provider = true;
    REQUIRE(host.register_builtin_step_selector(step, traits) >= 0);
    ThumbnailActivationGuard guard(host, step);
    for (SingletonSelectorProbe *probe : {&strict, &neutral}) {
        Plugin *provider = host.get_plugin(probe->id());
        REQUIRE(provider != nullptr);
        REQUIRE(host.set_plugin_active(provider, true));
        const char *group = probe == &strict ? "strict_singleton" : "neutral_singleton";
        const std::string key = Steps::plugin_exclusive_group_option_key(host, step, group);
        DynamicConfig config;
        config.set_key_value(key, new ConfigOptionString("missing.provider"));
        check_selection(host, step, group, &config, probe == &strict ? nullptr : provider);
        config.set_key_value(key, new ConfigOptionString(probe->id()));
        check_selection(host, step, group, &config, provider);
        DynamicConfig absent;
        check_selection(host, step, group, &absent, provider);
        check_selection(host, step, group, nullptr, provider);
        REQUIRE(host.set_plugin_active(provider, false));
    }
}

// Check that all three selection paths reject an inactive stored encoder even when only PNG is active.
TEST_CASE("Singleton thumbnail selection rejects the inactive stored encoder through group and step paths",
          "[plugins][step-pipeline][selector-traits][thumbnail]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    ThumbnailActivationGuard guard(host);
    Plugin *png = host.get_plugin("PNG");
    REQUIRE(png != nullptr);
    REQUIRE(host.get_plugin("JPG") != nullptr);
    for (Plugin *plugin : host.registered_plugins())
        if (plugin->get_step() == GCODE_THUMBNAIL) REQUIRE(host.set_plugin_active(plugin, plugin == png));
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("thumbnails_format", "JPG");
    REQUIRE(config.option("thumbnails_format")->serialize() == "JPG");
    check_selection(host, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", &config, nullptr);
    config.set_deserialize_strict("thumbnails_format", "PNG");
    check_selection(host, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", &config, png);
    DynamicConfig absent;
    check_selection(host, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", &absent, png);
    check_selection(host, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", nullptr, png);
}

// Check encoder error messages using an empty print; rejected choices need no renderer or acquired session.
TEST_CASE("Thumbnail selection errors name the inactive stored encoder",
          "[plugins][thumbnail][selector-traits]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    ThumbnailActivationGuard guard(host);
    for (Plugin *plugin : host.registered_plugins())
        if (plugin->get_step() == GCODE_THUMBNAIL) REQUIRE(host.set_plugin_active(plugin, plugin->get_id() == "PNG"));
    Print print;
    Model model;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("thumbnails_format", "JPG");
    print.apply(model, config);
    const slic3r_api::OrchestratorView view(reinterpret_cast<orchestrator_handle *>(&host));
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const ConfigBase *base_config = &config;
    const slic3r_api::Config config_view(reinterpret_cast<const config_handle *>(base_config));
    CHECK_THROWS_WITH(slic3r_api::GCodeThumbnailEncoder(view, print_view, config_view, nullptr),
                      "GCODE_THUMBNAIL provider 'JPG' is not active");
    CHECK_THROWS_WITH(slic3r_api::GCodeThumbnailEncoder(view, print_view, config_view, nullptr, "missing.encoder"),
                      "Unknown GCODE_THUMBNAIL provider: missing.encoder");
    for (const Plugin *plugin : host.registered_plugins()) {
        if (plugin->get_step() == GCODE_THUMBNAIL) continue;
        CHECK_THROWS_WITH(slic3r_api::GCodeThumbnailEncoder(view, print_view, config_view, nullptr, plugin->get_id()),
                          "Unknown GCODE_THUMBNAIL provider: " + plugin->get_id());
        break;
    }
    config.set_deserialize_strict("thumbnails_format", "PNG");
    CHECK(print.full_print_config().opt_serialize("thumbnails_format") == "JPG");
    CHECK_NOTHROW(slic3r_api::GCodeThumbnailEncoder(view, print_view, config_view, nullptr));
    config.optptr("thumbnails_format")->set_int(1000000);
    REQUIRE(config.option("thumbnails_format")->serialize().empty());
    CHECK_THROWS_WITH(slic3r_api::GCodeThumbnailEncoder(view, print_view, config_view, nullptr),
                      "No active G-code thumbnail encoder.");
}

// Check explicit lookup with invalid Print and configuration views; reading either would assert in Debug.
TEST_CASE("Explicit thumbnail lookup does not access Print configuration",
          "[plugins][thumbnail][selector-traits]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const slic3r_api::OrchestratorView host(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    const slic3r_api::Print unavailable_print;
    const slic3r_api::Config unavailable_config;
    CHECK_THROWS_WITH(slic3r_api::GCodeThumbnailEncoder(host, unavailable_print, unavailable_config, nullptr, "missing.encoder"),
                      "Unknown GCODE_THUMBNAIL provider: missing.encoder");
}

// Check registration, replacement and validation of selector declarations on a dynamic step.
TEST_CASE("Selector traits are looked up for the group first, then for the whole step",
          "[plugins][step-pipeline][selector-traits]")
{
    Orchestrator &host = Orchestrator::instance();
    const size_t option_count = PrintConfigDef::instance().options.size();
    const slicing_step_t step = host.register_step("slic3r.test.selector_traits", STEP_INFILL);
    REQUIRE(step != STEP_NONE);
    StepSelectorTraits whole;
    whole.category = RAW_OPTION_CATEGORY_INFILL;
    whole.container_type = RAW_CONTAINER_TYPE_OBJECT;
    whole.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    REQUIRE(host.register_builtin_step_selector(step, whole) == 1);
    CHECK(host.register_builtin_step_selector(step, whole) == 0);
    StepSelectorTraits conflict = whole;
    conflict.category = RAW_OPTION_CATEGORY_PERIMETER;
    CHECK(host.register_builtin_step_selector(step, conflict) == -2);
    REQUIRE(host.step_selector(step, "other") != nullptr);
    CHECK(host.step_selector(step, "other")->category == RAW_OPTION_CATEGORY_INFILL);

    StepSelectorTraits special;
    special.exclusive_group = "special";
    special.container_type = RAW_CONTAINER_TYPE_REGION;
    special.invalidates_step = STEP_PERIMETER;
    special.ui_page = "Infill";
    special.ui_group = "Selector test";
    REQUIRE(host.register_builtin_step_selector(step, special) == 1);
    CHECK(host.register_builtin_step_selector(step, special) == 0);
    REQUIRE(host.step_selector(step, "special") != nullptr);
    CHECK(host.step_selector(step, "special")->category == RAW_OPTION_CATEGORY_SLICING);
    CHECK(host.step_selector(step, "special")->option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    CHECK(host.step_selector(step, "special")->container_type == RAW_CONTAINER_TYPE_REGION);
    CHECK(host.step_selector_is_builtin(step, "special"));

    SelectorTraitsProbe probe(reinterpret_cast<orchestrator_handle *>(&host));
    Plugin provider(probe.c_instance(), "Slic3r", "");
    Steps::StepExclusiveGroup other = Steps::plugin_exclusive_group_definition(host, step, "other", provider);
    other.refresh_storage_pointers();
    CHECK(other.option_def.container_type == RAW_CONTAINER_TYPE_OBJECT);
    CHECK(other.option_def.category == RAW_OPTION_CATEGORY_INFILL);
    CHECK(other.option_def.option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
    CHECK(other.option_def.invalidates_step == STEP_INFILL);
    CHECK(other.ui_fragment == "page:Notes\ngroup:Exclusive step plugins\nline:other\nsetting:" +
                              other.option_key_storage + "\nend_line\n");
    Steps::StepExclusiveGroup exact = Steps::plugin_exclusive_group_definition(host, step, "special", provider);
    exact.refresh_storage_pointers();
    CHECK(exact.option_def.container_type == RAW_CONTAINER_TYPE_REGION);
    CHECK(exact.option_def.category == RAW_OPTION_CATEGORY_SLICING);
    CHECK(exact.option_def.option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    CHECK(exact.option_def.invalidates_step == STEP_PERIMETER);
    CHECK(exact.ui_fragment == "page:Infill\ngroup:Selector test\nline:special\nsetting:" +
                              exact.option_key_storage + "\nend_line\n");
    CHECK(host.step_selector(STEP_POST_SLICING, "neutral") == nullptr);
    Steps::StepExclusiveGroup neutral = Steps::plugin_exclusive_group_definition(host, STEP_POST_SLICING, "neutral", provider);
    CHECK(neutral.option_def.container_type == RAW_CONTAINER_TYPE_PROJECT);
    CHECK(neutral.option_def.category == RAW_OPTION_CATEGORY_SLICING);
    CHECK(neutral.option_def.option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);

    CHECK(host.register_builtin_step_selector(STEP_NONE, whole) == -1);
    CHECK(host.register_builtin_step_selector(STEP_ANY, whole) == -1);
    std::vector<std::pair<std::string, StepSelectorTraits>> invalid;
    StepSelectorTraits candidate = whole;
    candidate.ui_target_file = "nope.ui";
    invalid.emplace_back("unknown layout", candidate);
    candidate = whole;
    candidate.ui_page = "Infill";
    invalid.emplace_back("page without group", candidate);
    candidate = whole;
    candidate.option_key = "slic3r_test_selector";
    invalid.emplace_back("stable key on whole step", candidate);
    candidate = whole;
    candidate.container_type = RAW_CONTAINER_TYPE_PLATER;
    invalid.emplace_back("plater scope", candidate);
    candidate = whole;
    candidate.option_preset_type = RAW_PRESET_TYPE_NONE;
    invalid.emplace_back("missing preset", candidate);
    for (const std::pair<std::string, StepSelectorTraits> &entry : invalid) {
        INFO(entry.first);
        CHECK(host.register_builtin_step_selector(step, entry.second) == -1);
        CHECK(host.step_selector(step, "other")->category == RAW_OPTION_CATEGORY_INFILL);
        CHECK(host.step_selector(step, "other")->container_type == RAW_CONTAINER_TYPE_OBJECT);
        CHECK(host.step_selector(step, "special")->category == RAW_OPTION_CATEGORY_SLICING);
    }
    CHECK(PrintConfigDef::instance().options.size() == option_count);
}

// Check that built-in step declarations are present before initializing the shared plugin runtime.
TEST_CASE("Built-in selector traits are declared before any plugin", "[plugins][step-pipeline][selector-traits]")
{
    const Orchestrator &host = Orchestrator::instance();
    const StepSelectorTraits *perimeter = host.step_selector(STEP_PERIMETER, "x");
    REQUIRE(perimeter != nullptr);
    CHECK(perimeter->category == RAW_OPTION_CATEGORY_PERIMETER);
    CHECK(host.step_selector_is_builtin(STEP_PERIMETER, ""));
    CHECK_FALSE(host.step_selector_is_builtin(STEP_PERIMETER, "x"));
    CHECK(host.step_selector(STEP_PERIMETER, "x") == host.step_selector(STEP_PERIMETER, ""));
    const StepSelectorTraits *layer_height = host.step_selector(STEP_LAYER_HEIGHT, "x");
    REQUIRE(layer_height != nullptr);
    CHECK(layer_height->category == RAW_OPTION_CATEGORY_SLICING);
    CHECK(layer_height->option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    REQUIRE(layer_height->invalidates_step.has_value());
    CHECK(*layer_height->invalidates_step == STEP_LAYER_HEIGHT);
    CHECK_FALSE(layer_height->show_with_one_provider);
    CHECK(host.step_selector_is_builtin(STEP_LAYER_HEIGHT, ""));
    CHECK(host.step_selector(STEP_POST_SLICING, "x") == nullptr);
    CHECK_FALSE(host.step_selector_is_builtin(STEP_POST_SLICING, "x"));
}

// Check that built-in declarations produce the expected selector descriptions without registering settings.
TEST_CASE("Selector traits record effective seam and service metadata", "[plugins][step-pipeline][selector-traits]")
{
    Orchestrator &host = Orchestrator::instance();
    SelectorTraitsProbe probe(reinterpret_cast<orchestrator_handle *>(&host));
    Plugin provider(probe.c_instance(), "Slic3r", "");
    const Steps::StepExclusiveGroup seam = Steps::plugin_exclusive_group_definition(
        host, SEAM_PLACER, "seam_placer_plugin", provider);
    CHECK(seam.traits.container_type == RAW_CONTAINER_TYPE_REGION);
    CHECK(seam.traits.container_type == seam.option_def.container_type);
    REQUIRE(seam.traits.invalidates_step.has_value());
    CHECK(*seam.traits.invalidates_step == STEP_ORDERING);
    CHECK(*seam.traits.invalidates_step == seam.option_def.invalidates_step);
    CHECK(seam.traits.ui_page == "Perimeters & Shell");
    CHECK(seam.traits.ui_group == "Seam");
    CHECK(seam.ui_fragment == "page:" + seam.traits.ui_page + "\ngroup:" + seam.traits.ui_group +
                              "\nline:" + seam.label_storage + "\nsetting:" + seam.option_key_storage + "\nend_line\n");
    const std::pair<slicing_step_t, const char *> services[] = {
        {GCODE_THUMBNAIL, "thumbnail_encoder_plugin"}, {GCODE_READER, "reader_metadata_probe"}
    };
    for (const std::pair<slicing_step_t, const char *> &service : services) {
        INFO(int(service.first));
        const Steps::StepExclusiveGroup group = Steps::plugin_exclusive_group_definition(
            host, service.first, service.second, provider);
        REQUIRE(group.traits.invalidates_step.has_value());
        CHECK(*group.traits.invalidates_step == (service.first == GCODE_THUMBNAIL ? STEP_NONE : STEP_GCODE));
        CHECK(*group.traits.invalidates_step == group.option_def.invalidates_step);
        CHECK(group.traits.container_type == group.option_def.container_type);
    }
}

// Build test-owned choice lists from reversed member ids and check separate keys and labels for a group and an entire stage.
// Shared startup owns plugins and exporter activation is restored. Application settings and expected snapshot data are unchanged.
TEST_CASE("Selector dropdown preferences preserve native ordering and historical group layout identities",
          "[plugins][step-pipeline][selector-group-priority][selector-traits]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    const std::vector<std::string> seam_ids {"seam_placer.cost", "seam_placer.aligned", "seam_placer.contiguous", "seam_placer.fixed", "seam_placer.random"};
    const std::vector<std::string> perimeter_ids {"perimeter.generator.classic", "perimeter.generator.arachne", "perimeter.generator.simple"};
    for (const std::pair<slicing_step_t, std::vector<std::string>> &entry :
         std::vector<std::pair<slicing_step_t, std::vector<std::string>>>{{SEAM_PLACER, seam_ids}, {STEP_PERIMETER, perimeter_ids}}) {
        std::vector<std::pair<std::string, std::string>> reversed;
        for (std::vector<std::string>::const_reverse_iterator id = entry.second.rbegin(); id != entry.second.rend(); ++id) {
            REQUIRE(host.get_plugin(*id) != nullptr);
            reversed.emplace_back(*id, *id);
        }
        Steps::StepExclusiveGroup local = Steps::plugin_exclusive_group_definition(host, entry.first,
            entry.first == SEAM_PLACER ? "seam_placer_plugin" : "step_perimeter_plugin", *host.get_plugin(entry.second.front()));
        local.set_enum_plugins(host, reversed, entry.second.front());
        CHECK(local.enum_values == entry.second);
        CHECK(std::string(local.option_def.default_serialized_value) == entry.second.front());
    }
    REQUIRE(host.get_plugin("gcode.firmware.marlin2") != nullptr);
    const Steps::StepExclusiveGroup firmware_group = Steps::plugin_exclusive_group_definition(host, GCODE_FIRMWARE,
        "gcode_firmware_plugin", *host.get_plugin("gcode.firmware.marlin2"));
    CHECK(firmware_group.option_key_storage == "exclusive_group_10300_gcode_firmware_plugin_plugin");
    CHECK(firmware_group.ui_fragment.find("setting:exclusive_group_10300_gcode_firmware_plugin_plugin\n") != std::string::npos);
    const Steps::StepExclusiveGroup firmware_whole = Steps::make_exclusive_step_group(host, GCODE_FIRMWARE,
        Steps::get_exclusive_steps().at(GCODE_FIRMWARE));
    CHECK(firmware_whole.option_key_storage == "gcode_firmware_plugin");
    CHECK(firmware_group.traits.option_key == firmware_whole.traits.option_key);
    CHECK(firmware_group.traits.invalidates_step == firmware_whole.traits.invalidates_step);
    CHECK(Steps::get_exclusive_steps().count(STEP_GCODE) == 0);
    CHECK(PrintConfigDef::instance().get("step_gcode_plugin") == nullptr);
    ThumbnailActivationGuard exporter_guard(host, STEP_EXPORT);
    REQUIRE(host.set_plugin_active("gcode.exporter.simple", true));
    bool found_exporter = false;
    for (const Steps::StepExclusivePluginGroup &entry : Steps::active_exclusive_plugin_groups(host))
        if (entry.group.step == STEP_EXPORT) {
            found_exporter = true;
            CHECK(entry.group.ui_fragment.find("line:G-code exporter\n") != std::string::npos);
        }
    CHECK(found_exporter);
    const Steps::StepExclusiveGroup exporter_table = Steps::make_exclusive_step_group(host, STEP_EXPORT,
        Steps::get_exclusive_steps().at(STEP_EXPORT));
    CHECK(exporter_table.ui_fragment.find("line:G-code exporter\n") != std::string::npos);
}

// Check the keys, labels and declared settings of all nine selectors for entire steps.
TEST_CASE("Whole-step selector traits come from their owner module",
          "[plugins][step-pipeline][selector-snapshot][selector-traits]")
{
    const Orchestrator &host = Orchestrator::instance();
    struct Expected
    {
        slicing_step_t step;
        raw_option_category category;
        raw_option_preset_type preset;
        slicing_step_t invalidation;
        bool show;
    };
    const Expected expected[] = {
        {STEP_LAYER_HEIGHT, RAW_OPTION_CATEGORY_SLICING, RAW_PRESET_TYPE_FFF_PRINT, STEP_LAYER_HEIGHT, false},
        {STEP_SLICING, RAW_OPTION_CATEGORY_SLICING, RAW_PRESET_TYPE_FFF_PRINT, STEP_SLICING, false},
        {STEP_PERIMETER, RAW_OPTION_CATEGORY_PERIMETER, RAW_PRESET_TYPE_FFF_PRINT, STEP_PERIMETER, false},
        {STEP_INFILL_GROUP, RAW_OPTION_CATEGORY_INFILL, RAW_PRESET_TYPE_FFF_PRINT, STEP_INFILL_GROUP, false},
        {STEP_INFILL, RAW_OPTION_CATEGORY_INFILL, RAW_PRESET_TYPE_FFF_PRINT, STEP_INFILL, false},
        {STEP_WIPETOWER, RAW_OPTION_CATEGORY_OUTPUT, RAW_PRESET_TYPE_FFF_PRINT, STEP_WIPETOWER, false},
        {STEP_LAYER_STICHING, RAW_OPTION_CATEGORY_OUTPUT, RAW_PRESET_TYPE_FFF_PRINT, STEP_LAYER_STICHING, false},
        {STEP_EXPORT, RAW_OPTION_CATEGORY_OUTPUT, RAW_PRESET_TYPE_FFF_PRINTER, STEP_NONE, true},
        {GCODE_FIRMWARE, RAW_OPTION_CATEGORY_OUTPUT, RAW_PRESET_TYPE_FFF_PRINTER, STEP_GCODE, true}
    };
    const std::map<slicing_step_t, Steps::StepExclusiveIdentity> &identities = Steps::get_exclusive_steps();
    REQUIRE(identities.size() == 9);
    for (const Expected &entry : expected) {
        INFO(int(entry.step));
        const std::map<slicing_step_t, Steps::StepExclusiveIdentity>::const_iterator identity = identities.find(entry.step);
        REQUIRE(identity != identities.end());
        const StepSelectorTraits *declared = host.step_selector(entry.step, identity->second.key);
        REQUIRE(declared != nullptr);
        CHECK(host.step_selector_is_builtin(entry.step, ""));
        CHECK(declared->category == entry.category);
        CHECK(declared->option_preset_type == entry.preset);
        REQUIRE(declared->invalidates_step.has_value());
        CHECK(*declared->invalidates_step == entry.invalidation);
        CHECK(declared->show_with_one_provider == entry.show);
        Steps::StepExclusiveGroup group = Steps::make_exclusive_step_group(host, entry.step, identity->second);
        group.refresh_storage_pointers();
        CHECK(group.option_key_storage == identity->second.key);
        CHECK(group.label_storage == identity->second.label);
        CHECK(group.option_def.category == entry.category);
        CHECK(group.option_def.option_preset_type == entry.preset);
        CHECK(group.option_def.invalidates_step == entry.invalidation);
        CHECK(group.traits.show_with_one_provider == entry.show);
        CHECK(group.ui_fragment == "page:Notes\ngroup:Exclusive step plugins\nline:" +
                                   std::string(identity->second.line_label) + "\nsetting:" + identity->second.key + "\nend_line\n");
    }
}

// Check that firmware and reader choices replay G-code, while thumbnail and exporter choices affect only export.
TEST_CASE("G-code service selector invalidation comes from the owner modules",
          "[plugins][step-pipeline][selector-traits][gcode]")
{
    Orchestrator &host = Orchestrator::instance();
    SelectorTraitsProbe probe(reinterpret_cast<orchestrator_handle *>(&host));
    Plugin provider(probe.c_instance(), "Slic3r", "");
    const std::pair<slicing_step_t, const char *> services[] = {
        {GCODE_FIRMWARE, "gcode_firmware_plugin"}, {GCODE_THUMBNAIL, "thumbnail_encoder_plugin"},
        {GCODE_READER, "reader_invalidation_probe"}, {STEP_EXPORT, "gcode_exporter_plugin"}
    };
    for (const std::pair<slicing_step_t, const char *> &service : services) {
        INFO(int(service.first));
        const slicing_step_t invalidation = service.first == GCODE_THUMBNAIL || service.first == STEP_EXPORT ?
            STEP_NONE : STEP_GCODE;
        const StepSelectorTraits *declared = host.step_selector(service.first, "");
        REQUIRE(declared != nullptr);
        CHECK(host.step_selector_is_builtin(service.first, ""));
        REQUIRE(declared->invalidates_step.has_value());
        CHECK(*declared->invalidates_step == invalidation);
        CHECK(declared->category == RAW_OPTION_CATEGORY_OUTPUT);
        CHECK(declared->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
        const char *const groups[] = {service.second, "service_invalidation_probe"};
        for (const char *group_id : groups) {
            INFO(group_id);
            const StepSelectorTraits *resolved = host.step_selector(service.first, group_id);
            REQUIRE(resolved != nullptr);
            REQUIRE(resolved->invalidates_step.has_value());
            CHECK(*resolved->invalidates_step == invalidation);
            const Steps::StepExclusiveGroup group = Steps::plugin_exclusive_group_definition(
                host, service.first, group_id, provider);
            REQUIRE(group.traits.invalidates_step.has_value());
            CHECK(*group.traits.invalidates_step == invalidation);
            CHECK(group.option_def.invalidates_step == *declared->invalidates_step);
            CHECK(group.option_def.category == declared->category);
            CHECK(group.option_def.option_preset_type == declared->option_preset_type);
        }
    }
}

// Check the thumbnail declarations and the key, placement and selection rules used to describe encoder choices.
TEST_CASE("Thumbnail selector traits are declared by the G-code module",
          "[plugins][step-pipeline][selector-traits][thumbnail]")
{
    Orchestrator &host = Orchestrator::instance();
    const StepSelectorTraits *whole = host.step_selector(GCODE_THUMBNAIL, "");
    const StepSelectorTraits *encoder = host.step_selector(GCODE_THUMBNAIL, "thumbnail_encoder_plugin");
    REQUIRE(whole != nullptr);
    REQUIRE(encoder != nullptr);
    CHECK(host.step_selector_is_builtin(GCODE_THUMBNAIL, ""));
    CHECK(host.step_selector_is_builtin(GCODE_THUMBNAIL, "thumbnail_encoder_plugin"));
    CHECK(encoder != whole);
    CHECK(whole->exclusive_group.empty());
    CHECK(whole->option_key.empty());
    CHECK_FALSE(whole->strict_selection);
    CHECK(whole->read_with_one_provider);
    CHECK_FALSE(whole->provider_settings_rules);
    CHECK(encoder->exclusive_group == "thumbnail_encoder_plugin");
    CHECK(encoder->option_key == "thumbnails_format");
    CHECK(encoder->strict_selection);
    CHECK(encoder->read_with_one_provider);
    CHECK_FALSE(encoder->provider_settings_rules);
    CHECK_FALSE(encoder->show_with_one_provider);
    CHECK_FALSE(encoder->auto_value);
    CHECK_FALSE(encoder->value_when_absent.has_value());
    CHECK(encoder->container_type == RAW_CONTAINER_TYPE_PROJECT);
    CHECK(encoder->category == RAW_OPTION_CATEGORY_OUTPUT);
    CHECK(encoder->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
    REQUIRE(encoder->invalidates_step.has_value());
    CHECK(*encoder->invalidates_step == STEP_NONE);
    CHECK(encoder->ui_target_file == "print.ui");
    CHECK(encoder->ui_page.empty());
    CHECK(encoder->ui_group.empty());
    CHECK(host.step_selector(GCODE_THUMBNAIL, "other_encoder_group") == whole);

    SelectorTraitsProbe probe(reinterpret_cast<orchestrator_handle *>(&host));
    Plugin provider(probe.c_instance(), "Slic3r", "");
    Steps::StepExclusiveGroup group = Steps::plugin_exclusive_group_definition(
        host, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", provider);
    CHECK(group.option_key_storage == "thumbnails_format");
    CHECK(group.traits.strict_selection);
    CHECK(group.traits.read_with_one_provider);
    CHECK_FALSE(group.traits.provider_settings_rules);
    CHECK(group.ui_fragment == "page:Notes\ngroup:Exclusive step plugins\nline:" + group.label_storage +
                              "\nsetting:thumbnails_format\nend_line\n");
    group.set_enum_plugins(host, {{"JPG", "JPEG"}, {"PNG", "PNG"}}, "PNG");
    REQUIRE(group.option_def.default_serialized_value != nullptr);
    CHECK(std::string(group.option_def.default_serialized_value) == "PNG");
    const Steps::StepExclusiveGroup other = Steps::plugin_exclusive_group_definition(
        host, GCODE_THUMBNAIL, "other_encoder_group", provider);
    CHECK(other.option_key_storage != "thumbnails_format");
    CHECK_FALSE(other.traits.strict_selection);
    CHECK(other.traits.read_with_one_provider);
    CHECK_FALSE(other.traits.provider_settings_rules);
}

// Check export-only declarations and selection with and without the required built-in exporter, restoring activation afterward.
// The application owns the retained descriptions and providers; the test owns configurations and inserts no application setting.
TEST_CASE("Export selector traits preserve the strict built-in default even when unavailable",
          "[plugins][step-pipeline][selector-traits][exporter]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    const StepSelectorTraits *whole = host.step_selector(STEP_EXPORT, "");
    const StepSelectorTraits *exporter = host.step_selector(STEP_EXPORT, "gcode_exporter_plugin");
    REQUIRE(whole != nullptr);
    REQUIRE(exporter != nullptr);
    CHECK(whole != exporter);
    CHECK(host.step_selector_is_builtin(STEP_EXPORT, ""));
    CHECK(host.step_selector_is_builtin(STEP_EXPORT, "gcode_exporter_plugin"));
    CHECK(exporter->exclusive_group == "gcode_exporter_plugin");
    CHECK(exporter->option_key == "gcode_exporter_plugin");
    for (const StepSelectorTraits *traits : {whole, exporter}) {
        REQUIRE(traits->invalidates_step.has_value());
        CHECK(*traits->invalidates_step == STEP_NONE);
        CHECK(traits->category == RAW_OPTION_CATEGORY_OUTPUT);
        CHECK(traits->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
        CHECK(traits->strict_selection);
        CHECK(traits->read_with_one_provider);
        CHECK(traits->show_with_one_provider);
        CHECK_FALSE(traits->auto_value);
        REQUIRE(traits->value_when_absent.has_value());
        CHECK(*traits->value_when_absent == "gcode.exporter.simple");
    }
    ThumbnailActivationGuard guard(host, STEP_EXPORT);
    Plugin *simple = host.get_plugin("gcode.exporter.simple");
    Plugin *binary = host.get_plugin("gcode.exporter.bgcode");
    REQUIRE(simple != nullptr);
    REQUIRE(binary != nullptr);
    for (Plugin *plugin : host.registered_plugins())
        if (plugin->get_step() == STEP_EXPORT) REQUIRE(host.set_plugin_active(plugin, plugin == binary));
    DynamicConfig absent;
    check_selection(host, STEP_EXPORT, "gcode_exporter_plugin", nullptr, nullptr);
    check_selection(host, STEP_EXPORT, "gcode_exporter_plugin", &absent, nullptr);
    DynamicConfig config;
    config.set_key_value("gcode_exporter_plugin", new ConfigOptionString("gcode.exporter.bgcode"));
    check_selection(host, STEP_EXPORT, "gcode_exporter_plugin", &config, binary);
    config.set_key_value("gcode_exporter_plugin", new ConfigOptionString("missing.exporter"));
    check_selection(host, STEP_EXPORT, "gcode_exporter_plugin", &config, nullptr);
    REQUIRE(host.set_plugin_active(simple, true));
    check_selection(host, STEP_EXPORT, "gcode_exporter_plugin", nullptr, simple);
    check_selection(host, STEP_EXPORT, "gcode_exporter_plugin", &absent, simple);
}

// Check that GUI rules do not depend on the thumbnail selector and invalid choices select no encoder, even with one active.
// Use the registered selector's choices and restore activation flags after the checks.
TEST_CASE("Thumbnail selector traits keep encoder settings independent and read singleton ids",
          "[plugins][step-pipeline][selector-traits][thumbnail][gui-rules]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    const std::string key = Steps::plugin_exclusive_group_option_key(host, GCODE_THUMBNAIL, "thumbnail_encoder_plugin");
    REQUIRE(key == "thumbnails_format");
    const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
    REQUIRE(definition != nullptr);
    REQUIRE(definition->enum_def);
    const std::vector<std::string> ids = definition->enum_def->values();
    REQUIRE(ids.size() >= 2);
    for (const std::string &id : ids) {
        const Plugin *member = host.get_plugin(id);
        REQUIRE(member != nullptr);
        REQUIRE(member->get_step() == GCODE_THUMBNAIL);
        REQUIRE(member->get_exclusive_group() == "thumbnail_encoder_plugin");
    }
    bool non_thumbnail_rule = false;
    for (const Orchestrator::PluginGuiRule &rule : host.gui_rules().rules())
        for (const Orchestrator::PluginGuiRuleClause &clause : rule.clauses) {
            CHECK(clause.condition_key != key);
            if (clause.condition == RAW_GUI_RULE_CONDITION_STRING_EQUALS &&
                clause.condition_key == "step_layer_height_plugin") non_thumbnail_rule = true;
        }
    CHECK(non_thumbnail_rule);
    ThumbnailActivationGuard guard(host);
    for (Plugin *plugin : host.registered_plugins())
        if (plugin->get_step() == GCODE_THUMBNAIL)
            REQUIRE(host.set_plugin_active(plugin, std::find(ids.begin(), ids.end(), plugin->get_id()) != ids.end()));
    const std::vector<Plugin *> members = host.get_active_plugins_for_step(GCODE_THUMBNAIL);
    REQUIRE(members.size() == ids.size());
    Plugin *first = Steps::default_provider_from_group(members);
    REQUIRE(first != nullptr);
    Plugin *alternate = members.back();
    REQUIRE(first != alternate);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict(key, alternate->get_id());
    CHECK(Steps::selected_active_plugin_from_group(host, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", &config) == alternate);
    REQUIRE(config.optptr(key) != nullptr);
    config.optptr(key)->set_int(1000000);
    REQUIRE(config.option(key)->serialize().empty());
    CHECK(Steps::selected_active_plugin_from_group(host, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", &config) == nullptr);
    for (Plugin *member : members) REQUIRE(host.set_plugin_active(member, member == first));
    CHECK(Steps::selected_active_plugin_from_group(host, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", &config) == nullptr);
    config.set_deserialize_strict(key, alternate->get_id());
    CHECK(Steps::selected_active_plugin_from_group(host, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", &config) == nullptr);
    config.set_deserialize_strict(key, first->get_id());
    CHECK(Steps::selected_active_plugin_from_group(host, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", &config) == first);
    DynamicConfig absent;
    CHECK(Steps::selected_active_plugin_from_group(host, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", &absent) == first);
    CHECK(Steps::selected_active_plugin_from_group(host, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", nullptr) == first);
}

// Resolve a stored non-default firmware through the C++ group, C ABI and whole-step paths without changing activations.
TEST_CASE("Firmware group selection reads the stored whole-step selector",
          "[plugins][step-pipeline][selector-traits][gcode][firmware-selection]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    const Plugin *firmware = host.get_plugin("gcode.firmware.klipper");
    REQUIRE(firmware != nullptr);
    REQUIRE(host.is_plugin_active(firmware));
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("gcode_firmware_plugin", firmware->get_id());
    check_selection(host, GCODE_FIRMWARE, "gcode_firmware_plugin", &config, firmware);
}

// Restrict whole-step choices to a named group's two probes, restoring perimeter activations even when assertions fail.
TEST_CASE("Named groups on whole-step selectors read the stored choice within their members",
          "[plugins][step-pipeline][selector-traits][gcode][firmware-selection]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&host);
    const char *group = "test.perimeter.selector.named";
    static SingletonSelectorProbe first(handle, STEP_PERIMETER, "test.perimeter.selector.named.first",
                                        "test.perimeter.selector.named", 0);
    static SingletonSelectorProbe second(handle, STEP_PERIMETER, "test.perimeter.selector.named.second",
                                         "test.perimeter.selector.named", 10);
    for (SingletonSelectorProbe *probe : {&first, &second}) {
        if (host.get_plugin(probe->id()) == nullptr) REQUIRE(host.register_plugin(probe->c_instance()));
        REQUIRE(host.set_plugin_active(probe->id(), false));
    }
    ThumbnailActivationGuard guard(host, STEP_PERIMETER);
    for (Plugin *plugin : host.registered_plugins())
        if (plugin->get_step() == STEP_PERIMETER)
            REQUIRE(host.set_plugin_active(plugin, plugin->get_exclusive_group() == group));
    const std::vector<Plugin *> members = host.get_active_plugins_for_step(STEP_PERIMETER);
    REQUIRE(members.size() == 2);
    REQUIRE(Steps::default_provider_from_group(members) == host.get_plugin(first.id()));
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("step_perimeter_plugin", new ConfigOptionString(second.id()));
    const ConfigBase *base_config = &config;
    CHECK(Steps::selected_active_plugin_from_group(host, STEP_PERIMETER, group, &config) == host.get_plugin(second.id()));
    CHECK(orchestrator_select_plugin(handle, reinterpret_cast<const config_handle *>(base_config), STEP_PERIMETER, group) ==
          reinterpret_cast<const orchestrator_plugin_handle *>(host.get_plugin(second.id())));
    config.set_key_value("step_perimeter_plugin", new ConfigOptionString("perimeter.outside.named.group"));
    CHECK(Steps::selected_active_plugin_from_group(host, STEP_PERIMETER, group, &config) == host.get_plugin(first.id()));
    CHECK(orchestrator_select_plugin(handle, reinterpret_cast<const config_handle *>(base_config), STEP_PERIMETER, group) ==
          reinterpret_cast<const orchestrator_plugin_handle *>(host.get_plugin(first.id())));
}

// Check the seam declaration and verify that its scope, placement and invalidation do not apply to other groups.
TEST_CASE("Regional seam selector traits are declared by the ordering module",
          "[plugins][step-pipeline][selector-traits][seam]")
{
    Orchestrator &host = Orchestrator::instance();
    const StepSelectorTraits *seam = host.step_selector(SEAM_PLACER, "seam_placer_plugin");
    REQUIRE(seam != nullptr);
    CHECK(host.step_selector_is_builtin(SEAM_PLACER, "seam_placer_plugin"));
    CHECK(seam->exclusive_group == "seam_placer_plugin");
    CHECK(seam->container_type == RAW_CONTAINER_TYPE_REGION);
    REQUIRE(seam->invalidates_step.has_value());
    CHECK(*seam->invalidates_step == STEP_ORDERING);
    CHECK(seam->category == RAW_OPTION_CATEGORY_SLICING);
    CHECK(seam->option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    CHECK(seam->ui_target_file == "print.ui");
    CHECK(seam->ui_page == "Perimeters & Shell");
    CHECK(seam->ui_group == "Seam");
    CHECK(seam->strict_selection);
    CHECK_FALSE(seam->read_with_one_provider);
    CHECK_FALSE(seam->show_with_one_provider);
    CHECK(seam->provider_settings_rules);
    CHECK_FALSE(seam->auto_value);
    CHECK_FALSE(seam->value_when_absent.has_value());
    CHECK(seam->option_key.empty());
    CHECK(host.step_selector(SEAM_PLACER, "other") == nullptr);
    CHECK_FALSE(host.step_selector_is_builtin(SEAM_PLACER, "other"));
    SelectorTraitsProbe probe(reinterpret_cast<orchestrator_handle *>(&host));
    Plugin provider(probe.c_instance(), "Slic3r", "");
    const Steps::StepExclusiveGroup other = Steps::plugin_exclusive_group_definition(host, SEAM_PLACER, "other", provider);
    CHECK(other.option_def.container_type == RAW_CONTAINER_TYPE_PROJECT);
    CHECK(other.option_def.invalidates_step == STEP_ANY);
    CHECK(other.option_def.category == RAW_OPTION_CATEGORY_SLICING);
    CHECK(other.option_def.option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    CHECK_FALSE(other.traits.strict_selection);
    CHECK_FALSE(other.traits.read_with_one_provider);
    CHECK(other.ui_fragment == "page:Notes\ngroup:Exclusive step plugins\nline:other\nsetting:" +
                              other.option_key_storage + "\nend_line\n");
}

// Check missing, valid and invalid seam choices with multiple active providers, then with only one.
// Use the registered selector's choices and restore activation flags after the checks.
TEST_CASE("Regional seam selection preserves strict multi-provider and unread singleton choices",
          "[plugins][step-pipeline][selector-traits][seam]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    const std::string key = Steps::plugin_exclusive_group_option_key(host, SEAM_PLACER, "seam_placer_plugin");
    const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
    REQUIRE(definition != nullptr);
    REQUIRE(definition->enum_def);
    const std::vector<std::string> ids = definition->enum_def->values();
    REQUIRE(ids.size() >= 2);
    for (const std::string &id : ids) {
        const Plugin *member = host.get_plugin(id);
        REQUIRE(member != nullptr);
        REQUIRE(member->get_step() == SEAM_PLACER);
        REQUIRE(member->get_exclusive_group() == "seam_placer_plugin");
    }
    SeamActivationGuard guard(host);
    for (Plugin *plugin : host.registered_plugins())
        if (plugin->get_step() == SEAM_PLACER)
            REQUIRE(host.set_plugin_active(plugin, std::find(ids.begin(), ids.end(), plugin->get_id()) != ids.end()));
    const std::vector<Plugin *> members = host.get_active_plugins_for_step(SEAM_PLACER);
    REQUIRE(members.size() == ids.size());
    Plugin *first = Steps::default_provider_from_group(members);
    REQUIRE(first != nullptr);
    Plugin *alternate = members.back();
    REQUIRE(first != alternate);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict(key, alternate->get_id());
    check_seam_selection(host, &config, alternate);
    DynamicConfig absent;
    check_seam_selection(host, &absent, first);
    check_seam_selection(host, nullptr, first);
    REQUIRE(config.optptr(key) != nullptr);
    config.optptr(key)->set_int(1000000);
    REQUIRE(config.option(key)->serialize().empty());
    check_seam_selection(host, &config, nullptr);

    // Retained application text is independent of enabled members; the setting rules determine regional placement.
    const Orchestrator::ExclusiveGroupDefinition *seam_definition = host.exclusive_group_definition(SEAM_PLACER, "seam_placer_plugin");
    REQUIRE(seam_definition != nullptr);
    REQUIRE(!seam_definition->label.empty());
    REQUIRE(!seam_definition->tooltip.empty());
    const Steps::StepExclusiveGroup direct = Steps::plugin_exclusive_group_definition(
        host, SEAM_PLACER, "seam_placer_plugin", *first);
    CHECK(direct.label_storage == seam_definition->label);
    CHECK(direct.tooltip_storage == seam_definition->tooltip);
    bool generated_seam = false;
    for (const Steps::StepExclusivePluginGroup &entry : Steps::active_exclusive_plugin_groups(host)) {
        if (entry.group.step != SEAM_PLACER || entry.group.group_id != "seam_placer_plugin") continue;
        generated_seam = true;
        CHECK(entry.group.label_storage == direct.label_storage);
        CHECK(entry.group.tooltip_storage == direct.tooltip_storage);
        CHECK(entry.group.ui_fragment == direct.ui_fragment);
    }
    CHECK(generated_seam);

    for (Plugin *member : members) REQUIRE(host.set_plugin_active(member, member == first));
    check_seam_selection(host, &config, first);
    config.set_deserialize_strict(key, alternate->get_id());
    check_seam_selection(host, &config, first);
    config.set_deserialize_strict(key, first->get_id());
    check_seam_selection(host, &config, first);
    check_seam_selection(host, &absent, first);
    check_seam_selection(host, nullptr, first);
}
