#include <catch2/catch.hpp>
#include <stdexcept>
#include <string>

#include <boost/filesystem.hpp>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "plugin_test_helpers.hpp"

/* This file tests package translation directories and stored translations of selector text.
   The host is the slicer application; a package is an installed directory of
   plugin code and resources. A translation catalogue supplies translated text
   under a domain, its identifying name, and a language directory.

   register_external_selector_catalog_providers() introduces ExternalSelectorProvider
   objects during the shared test startup, before option definitions are finalized
   for normal use. Each exclusive group contains alternative plugin implementations;
   its selector is a setting naming a member. The test creates descriptions under
   a current package directory and checks matching, missing and other-package
   catalogues. Missing catalogues create internal package warnings, not load errors.

   TEST_CASE is the test runner's case declaration. Cases inspect startup reports and
   options. ConfigDef is a local collection of setting descriptions. Its
   update_translations() uses a deterministic function without a graphical
   interface or process language change. Display accessors must preserve marked
   text even when that function would translate the same words.

   Main calls:
     shared test startup
       register_external_selector_catalog_providers
         Orchestrator::create_selector_print_config
     TEST_CASE
       ConfigDef::update_translations
       translated_label / translated_labels */

namespace Slic3r::Test::Plugins {
// Register permanent borrowed test plugins and create selectors with matching, missing and other-package catalogues before finalization.
// The application owns copied descriptions; paths stay under the test package, and errors throw.
void register_external_selector_catalog_providers(orchestrator_handle *handle);
}

namespace {
using namespace Slic3r;
option_def_error_code missing_catalog_result = OPTION_DEF_ERROR_OK;
option_def_error_code foreign_catalog_result = OPTION_DEF_ERROR_OK;
option_def_error_code accepted_catalog_result = OPTION_DEF_ERROR_INVALID_ARGUMENT;
option_def_error_code ordinary_option_result = OPTION_DEF_ERROR_OK;
PluginPackageLoadReport external_catalog_report;

class ExternalSelectorProvider final : public slic3r_api::PluginBase
{
public:
    // Borrow the startup application and permanent identity strings without registering during construction.
    ExternalSelectorProvider(orchestrator_handle *host, const char *id, const char *group)
        : PluginBase(host), m_id(id), m_group(group) {}
private:
    // Return the permanent id also used as this member's catalogue name.
    const char *id_impl() const noexcept override { return m_id; }
    // Return the stage whose plugin groups can be selected independently.
    slicing_step_t step_impl() const noexcept override { return STEP_POST_INFILL; }
    // Return the permanent group name for this single-member test group.
    const char *exclusive_group_impl() const noexcept override { return m_group; }
    // Report no dependencies without reading host state.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Return the constant execution priority. Nothing is changed or transferred, and the call cannot fail.
    int32_t priority_impl() const noexcept override { return 0; }
    // Ignore the borrowed run context without changing Print data or failing.
    void run_impl(const plugin_run_context *) const override {}
    const char *m_id;
    const char *m_group;
};
}

void Slic3r::Test::Plugins::register_external_selector_catalog_providers(orchestrator_handle *handle)
{
    Orchestrator &host = *reinterpret_cast<Orchestrator *>(handle);
    const boost::filesystem::path packages = boost::filesystem::absolute(boost::filesystem::path(__FILE__).parent_path() / "selector_catalog_package");
    const std::string root = boost::filesystem::canonical(packages / "package-alpha").generic_string();
    const std::string other_root = boost::filesystem::canonical(packages / "package-other").generic_string();
    static ExternalSelectorProvider accepted(handle, "com.example.selector", "test.external.selector");
    static ExternalSelectorProvider foreign(handle, "com.example.selector.foreign", "test.external.selector.foreign");
    static ExternalSelectorProvider missing(handle, "com.example.selector.missing", "test.external.selector.missing");
    {
        Orchestrator::PluginRegistrationScope scope = host.plugin_registration_scope(other_root, true);
        if (orchestrator_register_translation_catalog(handle, "com.example.selector.foreign", "locale") != 1)
            throw std::runtime_error("Cannot register foreign selector catalogue.");
    }
    Orchestrator::PluginRegistrationScope scope = host.plugin_registration_scope(root, true);
    for (ExternalSelectorProvider *provider : {&accepted, &foreign, &missing})
        if (!host.register_plugin(provider->c_instance())) throw std::runtime_error("Cannot register external selector provider.");
    if (orchestrator_register_translation_catalog(handle, "com.example.selector", "locale") != 1)
        throw std::runtime_error("Cannot register external selector catalogue.");
    for (const char *id : {"com.example.selector", "com.example.selector.foreign", "com.example.selector.missing"}) {
        const bool foreign_domain = std::string(id) == "com.example.selector.foreign";
        const bool missing_domain = std::string(id) == "com.example.selector.missing";
        const char *group_id = foreign_domain ? "test.external.selector.foreign" : missing_domain ? "test.external.selector.missing" : "test.external.selector";
        raw_exclusive_group_def descriptor = raw_exclusive_group_def_init();
        descriptor.exclusive_group = group_id;
        descriptor.declaring_plugin_id = id;
        descriptor.label = missing_domain ? "Infill" : "External selector label";
        descriptor.tooltip = "External selector tooltip";
        descriptor.option_key = foreign_domain ? "test_external_selector_foreign" : missing_domain ? "test_external_selector_missing" : "test_external_selector";
        if (orchestrator_register_exclusive_group(handle, STEP_POST_INFILL, &descriptor) != 1)
            throw std::runtime_error("Cannot declare external selector metadata.");
        host.elect_exclusive_group_definition(STEP_POST_INFILL, group_id);
        Steps::StepExclusiveGroup group = Steps::plugin_exclusive_group_definition(host, STEP_POST_INFILL, group_id, *host.get_plugin(id));
        group.set_enum_plugins(host, {{id, id}}, id);
        group.refresh_storage_pointers();
        const Orchestrator::SelectorIdentity identity {Orchestrator::SelectorIdentityKind::PluginGroup, STEP_POST_INFILL, group_id};
        const option_def_error_code result = host.create_selector_print_config(group.option_def, identity, false);
        if (foreign_domain) foreign_catalog_result = result;
        else if (missing_domain) missing_catalog_result = result;
        else accepted_catalog_result = result;
        if (result != OPTION_DEF_ERROR_OK || !host.set_plugin_active(id, true))
            throw std::runtime_error("Cannot create authenticated external selector option.");
    }
    raw_config_option_def ordinary = raw_config_option_def_init();
    ordinary.opt_key = "test_external_strict_option";
    ordinary.type = RAW_CO_BOOL;
    ordinary.container_type = RAW_CONTAINER_TYPE_PROJECT;
    ordinary.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    ordinary.default_serialized_value = "0";
    ordinary.translation_domain = "com.example.selector.missing";
    ordinary_option_result = host.create_new_print_config(&ordinary);
    if (ordinary_option_result != OPTION_DEF_ERROR_INVALID_ARGUMENT || PrintConfigDef::instance().get(ordinary.opt_key) != nullptr)
        throw std::runtime_error("Ordinary plugin option catalogue validation must remain strict.");
    const PluginPackageLoadReport *report = host.plugin_package_load_report("package-alpha");
    if (report == nullptr) throw std::runtime_error("External selector package diagnostics are missing.");
    external_catalog_report = *report;
}

// Translate a test-owned copy of the startup selector using its describing member's catalogue name, not its package directory.
// Application settings, process language, source text and member ids remain unchanged.
TEST_CASE("External selector startup options retain authenticated member translation domains",
          "[plugins][translation][selector-traits]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CHECK(accepted_catalog_result == OPTION_DEF_ERROR_OK);
    const ConfigOptionDef *definition = PrintConfigDef::instance().get("test_external_selector");
    REQUIRE(definition != nullptr);
    CHECK(definition->translation_domain == "com.example.selector");
    CHECK(definition->label == "External selector label");
    CHECK(definition->tooltip == "External selector tooltip");
    CHECK(definition->default_value->serialize() == "com.example.selector");
    ConfigDef local;
    local.options.emplace("test_external_selector", *definition);
    const auto translate = [](const std::string &text, const std::string &domain) -> std::string {
        return domain == "com.example.selector" ? "Translated " + text : text;
    };
    CHECK(local.update_translations(translate) == 1);
    CHECK(local.get("test_external_selector")->translated_label() == "Translated External selector label");
    CHECK(local.get("test_external_selector")->translated_tooltip() == "Translated External selector tooltip");
    CHECK(local.get("test_external_selector")->label == definition->label);
}

// Inspect startup selectors and saved warnings without changing application settings or reports.
// Missing or other-package catalogues require verbatim source text; ordinary settings must still reject missing catalogues.
TEST_CASE("External selector texts without an owned catalogue remain untranslated with package warnings",
          "[plugins][translation][selector-traits]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CHECK(missing_catalog_result == OPTION_DEF_ERROR_OK);
    CHECK(foreign_catalog_result == OPTION_DEF_ERROR_OK);
    CHECK(external_catalog_report.state == PluginPackageLoadState::Loaded);
    CHECK(external_catalog_report.issues.empty());
    REQUIRE(external_catalog_report.warnings.size() == 2);
    for (const std::string &suffix : {std::string("missing"), std::string("foreign")}) {
        const std::string id = "com.example.selector." + suffix;
        const std::string key = "test_external_selector_" + suffix;
        const std::string group = "test.external.selector." + suffix;
        const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
        REQUIRE(definition != nullptr);
        CHECK(definition->untranslated);
        const Orchestrator::ExclusiveGroupDefinition *elected = Orchestrator::instance().exclusive_group_definition(STEP_POST_INFILL, group);
        REQUIRE(elected != nullptr);
        CHECK(elected->untranslated);
        CHECK(definition->translation_domain.empty());
        CHECK(definition->label == (suffix == "missing" ? "Infill" : "External selector label"));
        CHECK(definition->tooltip == "External selector tooltip");
        CHECK(definition->default_value->serialize() == id);
        size_t warnings = 0;
        for (const std::string &warning : external_catalog_report.warnings)
            if (warning.find("plugin '" + id + "'") != std::string::npos) {
                ++warnings;
                CHECK(warning.find("Package 'package-alpha'") != std::string::npos);
                CHECK(warning.find("group '" + group + "'") != std::string::npos);
                CHECK(warning.find("translation domain '" + id + "'") != std::string::npos);
            }
        CHECK(warnings == 1);
        ConfigDef local;
        local.options.emplace(key, *definition);
        const auto translate = [](const std::string &text, const std::string &) -> std::string {
            return "Translated " + text;
        };
        CHECK(local.update_translations(translate) == 0);
        CHECK(local.get(key)->translated_label() == definition->label);
        CHECK(local.get(key)->translated_tooltip() == "External selector tooltip");
        REQUIRE(local.get(key)->enum_def);
        REQUIRE_FALSE(local.get(key)->enum_def->labels().empty());
        CHECK(local.get(key)->enum_def->translated_labels() == local.get(key)->enum_def->labels());
        CHECK(local.get(key)->enum_def->translated_label(0) == local.get(key)->enum_def->label(0));
        ConfigDef host_text;
        host_text.options["host_text"].label = "Infill";
        CHECK(host_text.update_translations(translate) == 1);
        CHECK(host_text.get("host_text")->translated_label() == "Translated Infill");
    }
    CHECK(ordinary_option_result == OPTION_DEF_ERROR_INVALID_ARGUMENT);
    CHECK(PrintConfigDef::instance().get("test_external_strict_option") == nullptr);
}

TEST_CASE("Plugin translation catalogs validate package-relative directories", "[plugins][translation]")
{
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    const boost::filesystem::path package_root =
        boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("slic3r-translation-%%%%-%%%%");
    const boost::filesystem::path locale_directory = package_root / "locale";
    const boost::filesystem::path alternate_directory = package_root / "alternate";
    boost::filesystem::create_directories(locale_directory);
    boost::filesystem::create_directories(alternate_directory);

    const std::string domain = "tests.translation." + boost::filesystem::unique_path("%%%%-%%%%").string();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&orchestrator);

    {
        // The loader owns this scope in production. The test establishes the
        // same package root so the C entry point can validate relative paths.
        Slic3r::Orchestrator::PluginRegistrationScope scope =
            orchestrator.plugin_registration_scope(package_root.string(), true);
        CHECK(orchestrator_register_translation_catalog(handle, nullptr, "locale") == -1);
        CHECK(orchestrator_register_translation_catalog(handle, "", "locale") == -1);
        CHECK(orchestrator_register_translation_catalog(handle, domain.c_str(), nullptr) == -1);
        CHECK(orchestrator_register_translation_catalog(handle, "tests.translation.absolute",
                                                         locale_directory.string().c_str()) == -1);
        CHECK(orchestrator_register_translation_catalog(handle, domain.c_str(), "locale") == 1);
        CHECK(orchestrator_register_translation_catalog(handle, domain.c_str(), "locale") == 0);
        CHECK(orchestrator_register_translation_catalog(handle, domain.c_str(), "alternate") == -2);
        CHECK(orchestrator_register_translation_catalog(handle, "invalid/domain", "locale") == -1);
        CHECK(orchestrator_register_translation_catalog(handle, "tests.translation.escape", "../outside") == -1);
    }

    const std::vector<Slic3r::Orchestrator::TranslationCatalog> &catalogs = orchestrator.translation_catalogs();
    const Slic3r::Orchestrator::TranslationCatalog *registered = nullptr;
    for (const Slic3r::Orchestrator::TranslationCatalog &catalog : catalogs)
        if (catalog.domain == domain)
            registered = &catalog;

    REQUIRE(registered != nullptr);
    CHECK(registered->locale_directory == locale_directory.generic_string());
    CHECK(registered->package_root == package_root.generic_string());

    CHECK(orchestrator_register_translation_catalog(handle, "tests.translation.no_scope", "locale") == -1);

    boost::filesystem::remove_all(package_root);
}
