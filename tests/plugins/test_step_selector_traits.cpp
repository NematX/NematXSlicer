#include <catch2/catch.hpp>

#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

/* Step modules install host-owned selector declarations during host construction. A whole-step
   declaration supplies defaults to named groups, while an exact group replaces every field.
   These tests read the process host and build local metadata without registering provider options;
   the generated-selector snapshot checks the serialized definitions and layout strings separately. */

namespace {
using namespace Slic3r;

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
} // namespace

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
