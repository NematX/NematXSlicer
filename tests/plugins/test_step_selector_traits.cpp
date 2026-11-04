#include <catch2/catch.hpp>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeThumbnailViews.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

/* Step modules install host-owned selector declarations during host construction. A whole-step
   declaration supplies defaults to named groups, while an exact group replaces every field.
   Local metadata checks read the process host without registering provider options. Selection checks
   initialize the shared runtime and restore scoped activation changes. Dynamic no-op service providers
   register after startup and remain inactive outside their selection checks. The generated-selector snapshot
   checks the serialized definitions and layout strings separately. */

namespace {
using namespace Slic3r;

// Compare both public seam selection paths against a borrowed expected provider without changing configuration or activation.
void check_seam_selection(Orchestrator &host, const ConfigBase *config, const Plugin *expected);
// Compare group, C API and step selection against the same borrowed expected provider.
void check_selection(Orchestrator &host, slicing_step_t step, const char *group,
                     const ConfigBase *config, const Plugin *expected);

class SeamActivationGuard
{
public:
    // Borrow the host and copy all seam-step activations before mutations; allocation failures propagate without activation changes.
    explicit SeamActivationGuard(Orchestrator &host) : m_host(host)
    {
        for (Plugin *plugin : host.registered_plugins())
            if (plugin->get_step() == SEAM_PLACER) m_saved.emplace_back(plugin, host.is_plugin_active(plugin));
    }
    // Restore the saved flags through the host API; the borrowed host/providers must outlive this noexcept cleanup.
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
    // Borrow the host and copy flags on the supplied step before mutations; allocations may throw without changing activations.
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
    // Borrow a host and permanent identity strings for a no-op provider on the supplied dynamic step.
    SingletonSelectorProbe(orchestrator_handle *host, slicing_step_t step, const char *id, const char *group)
        : PluginBase(host), m_step(step), m_id(id), m_group(group) {}
    // Return the borrowed permanent provider id without effects or failures.
    const char *id() const noexcept { return m_id; }
private:
    // Return the borrowed permanent provider id without effects or failures.
    const char *id_impl() const noexcept override { return m_id; }
    // Return the dynamic step value without effects or failures.
    slicing_step_t step_impl() const noexcept override { return m_step; }
    // Return the borrowed permanent group id without effects or failures.
    const char *exclusive_group_impl() const noexcept override { return m_group; }
    // Report no dependencies without effects or failures.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Return a constant execution priority without effects or failures.
    int32_t priority_impl() const noexcept override { return 0; }
    // Ignore the borrowed context without producing output, changing state or failing.
    void run_impl(const plugin_run_context *) const override {}
    slicing_step_t m_step;
    const char *m_id;
    const char *m_group;
};

class SelectorTraitsProbe final : public slic3r_api::PluginBase
{
public:
    // Borrow the host without registering or transferring ownership of this local metadata provider.
    explicit SelectorTraitsProbe(orchestrator_handle *host) : PluginBase(host) {}
private:
    // Return a stable borrowed id without effects, allocation or failures.
    const char *id_impl() const noexcept override { return "slic3r.test.selector_traits.probe"; }
    // Return a copied step identity without effects, ownership changes or failures.
    slicing_step_t step_impl() const noexcept override { return STEP_POST_INFILL; }
    // Report no dependencies without effects, ownership changes or failures.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Return a constant execution priority without effects or failures.
    int32_t priority_impl() const noexcept override { return 0; }
    // Ignore the borrowed context; no output or state is produced and this callback cannot fail.
    void run_impl(const plugin_run_context *) const override {}
};

// Read borrowed configuration through group and step resolution and assert the borrowed expected provider or empty result.
// No host/configuration mutation or ownership transfer occurs; assertion and serialization/allocation failures propagate.
void check_seam_selection(Orchestrator &host, const ConfigBase *config, const Plugin *expected)
{
    check_selection(host, SEAM_PLACER, "seam_placer_plugin", config, expected);
}

// Resolve borrowed config through all three entry points without mutation; assertion/allocation failures propagate.
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

// Register permanent no-op providers after shared startup and scope their activations; owned string configs test all paths.
// Registrations remain host-owned and inactive afterward; assertion/allocation failures unwind the activation guard.
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

// Scope built-in encoder activations and read serialized enum ids through every selection path.
// Local configs/print own their data; no encoder executes on rejection and saved activations restore on failure.
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

// Apply config with an empty model: rejected selections need no object or renderer and never acquire a session.
// Local model/print own their data; assertions and apply failures propagate while scoped flags restore on failure.
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
    CHECK_THROWS_WITH(slic3r_api::GCodeThumbnailEncoder(view, print_view, nullptr),
                      "GCODE_THUMBNAIL provider 'JPG' is not active");
    CHECK_THROWS_WITH(slic3r_api::GCodeThumbnailEncoder(view, print_view, nullptr, "missing.encoder"),
                      "Unknown GCODE_THUMBNAIL provider: missing.encoder");
    for (const Plugin *plugin : host.registered_plugins()) {
        if (plugin->get_step() == GCODE_THUMBNAIL) continue;
        CHECK_THROWS_WITH(slic3r_api::GCodeThumbnailEncoder(view, print_view, nullptr, plugin->get_id()),
                          "Unknown GCODE_THUMBNAIL provider: " + plugin->get_id());
        break;
    }
    config.optptr("thumbnails_format")->set_int(1000000);
    REQUIRE(config.option("thumbnails_format")->serialize().empty());
    print.apply(model, config);
    CHECK_THROWS_WITH(slic3r_api::GCodeThumbnailEncoder(view, print_view, nullptr),
                      "Invalid GCODE_THUMBNAIL provider selection");
}

// Look up an explicit missing provider with an invalid Print view; accessing its config would assert in Debug.
// Rejection precedes execution and leaves the borrowed host unchanged; only the exact lookup error is accepted.
TEST_CASE("Explicit thumbnail lookup does not access Print configuration",
          "[plugins][thumbnail][selector-traits]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const slic3r_api::OrchestratorView host(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    const slic3r_api::Print unavailable_print;
    CHECK_THROWS_WITH(slic3r_api::GCodeThumbnailEncoder(host, unavailable_print, nullptr, "missing.encoder"),
                      "Unknown GCODE_THUMBNAIL provider: missing.encoder");
}

// Declare owned dynamic-step traits and construct local groups from borrowed provider metadata.
// Assertions report rejected/mutated declarations; no options, providers or activations are registered by this test.
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

// Read constructor-installed host registry entries through the singleton, regardless of later provider registrations.
// Borrowed declarations must have host provenance; no initialization of providers, configuration or activation is requested.
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

// Build local group metadata from borrowed host declarations and provider text without registering options or providers.
// Assertions compare effective fields with the owned traits; allocation failures propagate and host state is unchanged.
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
        CHECK(*group.traits.invalidates_step == STEP_GCODE);
        CHECK(*group.traits.invalidates_step == group.option_def.invalidates_step);
        CHECK(group.traits.container_type == group.option_def.container_type);
    }
}

// Build all ten whole-step definitions through their owner traits, keeping the registered options and layout untouched.
// Expected metadata is checked locally; complete table/definition/UI row equality is checked by the unchanged selector snapshot.
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
        {STEP_SUPPORT, RAW_OPTION_CATEGORY_SUPPORT, RAW_PRESET_TYPE_FFF_PRINT, STEP_SUPPORT, false},
        {STEP_WIPETOWER, RAW_OPTION_CATEGORY_OUTPUT, RAW_PRESET_TYPE_FFF_PRINT, STEP_WIPETOWER, false},
        {STEP_LAYER_STICHING, RAW_OPTION_CATEGORY_OUTPUT, RAW_PRESET_TYPE_FFF_PRINT, STEP_LAYER_STICHING, false},
        {STEP_GCODE, RAW_OPTION_CATEGORY_OUTPUT, RAW_PRESET_TYPE_FFF_PRINTER, STEP_GCODE, true},
        {GCODE_FIRMWARE, RAW_OPTION_CATEGORY_OUTPUT, RAW_PRESET_TYPE_FFF_PRINTER, STEP_GCODE, true}
    };
    const std::map<slicing_step_t, Steps::StepExclusiveIdentity> &identities = Steps::get_exclusive_steps();
    REQUIRE(identities.size() == 10);
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

// Check that firmware, thumbnail and reader declarations make selector changes require G-code generation to run again.
TEST_CASE("G-code service selector invalidation comes from the owner modules",
          "[plugins][step-pipeline][selector-traits][gcode]")
{
    Orchestrator &host = Orchestrator::instance();
    SelectorTraitsProbe probe(reinterpret_cast<orchestrator_handle *>(&host));
    Plugin provider(probe.c_instance(), "Slic3r", "");
    const std::pair<slicing_step_t, const char *> services[] = {
        {GCODE_FIRMWARE, "gcode_firmware_plugin"}, {GCODE_THUMBNAIL, "thumbnail_encoder_plugin"},
        {GCODE_READER, "reader_invalidation_probe"}
    };
    for (const std::pair<slicing_step_t, const char *> &service : services) {
        INFO(int(service.first));
        const StepSelectorTraits *declared = host.step_selector(service.first, "");
        REQUIRE(declared != nullptr);
        CHECK(host.step_selector_is_builtin(service.first, ""));
        REQUIRE(declared->invalidates_step.has_value());
        CHECK(*declared->invalidates_step == STEP_GCODE);
        CHECK(declared->category == RAW_OPTION_CATEGORY_OUTPUT);
        CHECK(declared->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
        const char *const groups[] = {service.second, "service_invalidation_probe"};
        for (const char *group_id : groups) {
            INFO(group_id);
            const StepSelectorTraits *resolved = host.step_selector(service.first, group_id);
            REQUIRE(resolved != nullptr);
            REQUIRE(resolved->invalidates_step.has_value());
            CHECK(*resolved->invalidates_step == STEP_GCODE);
            const Steps::StepExclusiveGroup group = Steps::plugin_exclusive_group_definition(
                host, service.first, group_id, provider);
            REQUIRE(group.traits.invalidates_step.has_value());
            CHECK(*group.traits.invalidates_step == STEP_GCODE);
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
    CHECK(*encoder->invalidates_step == STEP_GCODE);
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
    group.set_enum_plugins({{"JPG", "JPEG"}, {"PNG", "PNG"}});
    REQUIRE(group.option_def.default_serialized_value != nullptr);
    CHECK(std::string(group.option_def.default_serialized_value) == "PNG");
    const Steps::StepExclusiveGroup other = Steps::plugin_exclusive_group_definition(
        host, GCODE_THUMBNAIL, "other_encoder_group", provider);
    CHECK(other.option_key_storage != "thumbnails_format");
    CHECK_FALSE(other.traits.strict_selection);
    CHECK(other.traits.read_with_one_provider);
    CHECK_FALSE(other.traits.provider_settings_rules);
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
    Plugin *first = members.front();
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

// Read the exact host declaration and construct local metadata without registering options or changing activations.
// The neutral group on the same service proves that scope, placement and invalidation are group-specific.
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

// Activate only finalized seam enum members, excluding providers registered by later tests, and restore every saved flag.
// Configurations own their values and borrow finalized enum maps; provider text and selection are read without print changes.
// Missing runtime metadata fails assertions; allocation/serialization failures propagate through the activation guard.
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
    Plugin *first = members.front();
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

    // Provider callbacks supply display text; generic fragment construction applies the declared regional placement.
    REQUIRE(!first->get_exclusive_group_label().empty());
    REQUIRE(!first->get_exclusive_group_tooltip().empty());
    const Steps::StepExclusiveGroup direct = Steps::plugin_exclusive_group_definition(
        host, SEAM_PLACER, "seam_placer_plugin", *first);
    CHECK(direct.label_storage == first->get_exclusive_group_label());
    CHECK(direct.tooltip_storage == first->get_exclusive_group_tooltip());
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
