#include <catch2/catch.hpp>

#include <algorithm>
#include <initializer_list>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "probe_pattern_plugin.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"
#include "libslic3r/Steps/StepRunner.hpp"

/*
Infill pattern discovery and initialization
===========================================

The default STEP_INFILL generator performs a discovery pass during its
setup_run() phase: it resolves every fill surface recipe, groups the surfaces
that share one recipe and reports the groups through
run_ctx_generate_infill::report_pattern_assignments(). The host turns that into
one run_ctx_infill_pattern_init payload per pattern plugin.

These tests drive the generator directly with recording callbacks so its
discovery flow can be observed without a real pattern plugin: grouping,
per-surface pattern calls, and the cached replay in run().

The step-level cases also run the default generator with a failing pattern and
with recipe-modifier probes. They check that callback failures stop the runner
at the correct phase, that modifiers keep earlier edits but do not call a later
provider, and that a modifier can observe contained pattern-callback results.
*/

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

namespace {

constexpr infill_pattern_runtime_id k_gyroid_runtime_id = 7;

// Local copy of the perimeter test scope helper so the infill test can restrict
// the active plugin set without touching the shared helper.
class ScopedActivePlugins
{
public:
    explicit ScopedActivePlugins(std::initializer_list<const char *> plugin_ids) :
        m_orchestrator(Orchestrator::instance())
    {
        m_previous_active_plugins.reserve(m_orchestrator.active_plugins().size());
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous_active_plugins.push_back(plugin);

        m_orchestrator.clear_active_plugins();
        for (const char *plugin_id : plugin_ids)
            REQUIRE(m_orchestrator.set_plugin_active(plugin_id, true));
    }

    ~ScopedActivePlugins()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous_active_plugins)
            m_orchestrator.set_plugin_active(plugin, true);
    }

private:
    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous_active_plugins;
};

constexpr const char *k_recipe_modifier_a = "test.infill.recipe_modifier.a";
constexpr const char *k_recipe_modifier_b = "test.infill.recipe_modifier.b";
constexpr const char *k_recipe_modifier_c = "test.infill.recipe_modifier.c";
constexpr const char *k_recipe_modifier_pattern_callback = "test.infill.recipe_modifier.pattern_callback";
constexpr const char *k_recipe_modifier_pattern_cancellation = "test.infill.recipe_modifier.pattern_cancellation";
constexpr const char *k_recipe_modifier_setup_failure = "test.infill.recipe_modifier.setup_failure";
const char *k_recipe_modifier_dependencies[] = { nullptr };
std::vector<std::string> recipe_modifier_calls;
float recipe_modifier_density_seen = -1.f;

struct RecipeModifierPatternCallbackRecord {
    bool called = false;
    infill_pattern_runtime_id pattern_id = INFILL_PATTERN_RUNTIME_ID_INVALID;
    int32_t first_result = -1;
    bool first_output_empty = false;
    int32_t second_result = -1;
    bool second_output_empty = false;
    infill_pattern_runtime_id resolve_after = INFILL_PATTERN_RUNTIME_ID_INVALID;
};
RecipeModifierPatternCallbackRecord recipe_modifier_pattern_callback_record;

enum class RecipeModifierAction { ChangeDensity, Fail, FailSetupRun, Observe, CallFailingPattern, CallCancelingPattern };

// Register ordered recipe modifiers that edit or fail a shared recipe and two modifiers that exercise pattern callbacks.
void register_recipe_modifier_probes();

// Resolve the active probe through the parent STEP_INFILL table, pass borrowed surface/recipe inputs to generate_pattern,
// and record results from fresh empty modifier-owned collections before destruction. Missing context or callbacks leave
// the record untouched; resolution writes pattern_id before rejecting INVALID, and contained failures return statuses.
void call_probe_pattern_from_modifier(const plugin_run_context *context,
                                      const run_ctx_infill_surface_recipe_modifier &payload,
                                      bool call_after_failure)
{
    if (context == nullptr || context->plugin_storage == nullptr || payload.generate_infill_ctx == nullptr ||
        payload.params == nullptr)
        return;

    const run_ctx_generate_infill *generator = payload.generate_infill_ctx;
    if (generator->resolve_pattern_id == nullptr || generator->generate_pattern == nullptr)
        return;

    RecipeModifierPatternCallbackRecord &record = recipe_modifier_pattern_callback_record;
    record.pattern_id = generator->resolve_pattern_id(generator, Test::ProbePatternPlugin::probe_pattern_id);
    if (record.pattern_id == INFILL_PATTERN_RUNTIME_ID_INVALID)
        return;

    Slic3r::ExtrusionEntityCollection first_output;
    record.first_result = generator->generate_pattern(
        generator, record.pattern_id, payload.layer, payload.island, payload.region_island,
        payload.primary_region, payload.surface, payload.no_overlap_areas, payload.params,
        reinterpret_cast<extrusion_entity_handle *>(&first_output));
    record.first_output_empty = first_output.empty();
    record.called = true;

    if (!call_after_failure)
        return;

    Slic3r::ExtrusionEntityCollection second_output;
    record.second_result = generator->generate_pattern(
        generator, record.pattern_id, payload.layer, payload.island, payload.region_island,
        payload.primary_region, payload.surface, payload.no_overlap_areas, payload.params,
        reinterpret_cast<extrusion_entity_handle *>(&second_output));
    record.second_output_empty = second_output.empty();
    record.resolve_after = generator->resolve_pattern_id(generator, Test::ProbePatternPlugin::probe_pattern_id);
}

class RecipeModifierProbe final : public slic3r_api::PluginBase
{
public:
    // Store the borrowed id and action; the static provider outlives registry use, and construction does not run a recipe.
    RecipeModifierProbe(orchestrator_handle *orchestrator, const char *id, int32_t priority,
                        RecipeModifierAction action)
        : PluginBase(orchestrator), m_id(id), m_priority(priority), m_action(action) {}

    // Return the borrowed stable id used by test activation and registration; no provider state changes.
    const char *id() const { return m_id; }

private:
    // Return the borrowed registry id; the static string outlives the plugin and this call does not fail.
    const char *id_impl() const noexcept override { return m_id; }
    // Identify this provider as a per-surface modifier; this query has no side effects or failure path.
    slicing_step_t step_impl() const noexcept override { return INFILL_SURFACE_RECIPE_MODIFIER; }
    // Return the static null-terminated empty dependency list; no allocation or registry mutation occurs.
    const char *const *dependencies_impl() const noexcept override { return k_recipe_modifier_dependencies; }
    // Return the constructor-supplied order; the host uses it to sort calls and this query does not mutate state.
    int32_t priority_impl() const noexcept override { return m_priority; }
    // Throw from the per-surface setup phase for FailSetupRun, before run_impl can record a call; other actions do nothing.
    void setup_run_impl(const plugin_run_context *) const override
    {
        if (m_action == RecipeModifierAction::FailSetupRun)
            throw std::runtime_error("infill recipe modifier setup_run probe failure");
    }
    // Ignore malformed borrowed payloads; otherwise record this provider and apply its action. A edits shared params,
    // B records then throws, and callback actions retain only results from temporary outputs, never borrowed handles.
    void run_impl(const plugin_run_context *context) const override
    {
        const run_ctx_infill_surface_recipe_modifier *payload =
            plugin_ctx_as_infill_surface_recipe_modifier(context);
        if (payload == nullptr || payload->params == nullptr)
            return;

        recipe_modifier_calls.emplace_back(m_id);
        if (m_action == RecipeModifierAction::ChangeDensity) {
            payload->params->density = 0.375f;
        } else if (m_action == RecipeModifierAction::Fail) {
            recipe_modifier_density_seen = payload->params->density;
            throw std::runtime_error("infill recipe modifier probe failure");
        } else if (m_action == RecipeModifierAction::CallFailingPattern) {
            call_probe_pattern_from_modifier(context, *payload, true);
        } else if (m_action == RecipeModifierAction::CallCancelingPattern) {
            call_probe_pattern_from_modifier(context, *payload, false);
        }
    }

    const char *m_id;
    int32_t m_priority;
    RecipeModifierAction m_action;
};

// Clear plugin cancellation and queued messages before and after one step-failure test.
class ScopedInfillStepDiagnostics
{
public:
    // Clear the singleton stop flag and discard queued messages so this test owns the following diagnostics.
    ScopedInfillStepDiagnostics()
    {
        Orchestrator::instance().reset_plugin_cancel();
        (void) Orchestrator::instance().consume_plugin_messages();
    }
    // Clear the singleton stop flag and drain messages produced during the test; no print or plugin is owned here.
    ~ScopedInfillStepDiagnostics()
    {
        Orchestrator::instance().reset_plugin_cancel();
        (void) Orchestrator::instance().consume_plugin_messages();
    }
    ScopedInfillStepDiagnostics(const ScopedInfillStepDiagnostics &) = delete;
    ScopedInfillStepDiagnostics &operator=(const ScopedInfillStepDiagnostics &) = delete;
};

// Set the shared pattern probe to fail after writing partial output, then restore its run state.
class ScopedFailingInfillPattern
{
public:
    // Set the shared probe's failure mode and clear its witness; the probe owns the atomics, not this scope.
    explicit ScopedFailingInfillPattern(int mode)
    {
        Test::ProbePatternPlugin::pattern_fail_run = mode;
        Test::ProbePatternPlugin::pattern_wrote_before_failure = false;
        Test::ProbePatternPlugin::pattern_cancel_print = nullptr;
    }
    // Reset the shared probe's failure mode and print pointer so later tests see successful pattern runs.
    ~ScopedFailingInfillPattern()
    {
        Test::ProbePatternPlugin::pattern_fail_run = 0;
        Test::ProbePatternPlugin::pattern_wrote_before_failure = false;
        Test::ProbePatternPlugin::pattern_cancel_print = nullptr;
    }
    ScopedFailingInfillPattern(const ScopedFailingInfillPattern &) = delete;
    ScopedFailingInfillPattern &operator=(const ScopedFailingInfillPattern &) = delete;
};

// Register static recipe modifiers, including both nested pattern-callback probes, only when their ids are absent; the
// registry borrows them until shutdown, and registration failure throws before an incomplete provider set can run.
void register_recipe_modifier_probes()
{
    Orchestrator &orchestrator = Orchestrator::instance();
    static RecipeModifierProbe modifier_a(
        reinterpret_cast<orchestrator_handle *>(&orchestrator), k_recipe_modifier_a, 10,
        RecipeModifierAction::ChangeDensity);
    static RecipeModifierProbe modifier_b(
        reinterpret_cast<orchestrator_handle *>(&orchestrator), k_recipe_modifier_b, 20,
        RecipeModifierAction::Fail);
    static RecipeModifierProbe modifier_c(
        reinterpret_cast<orchestrator_handle *>(&orchestrator), k_recipe_modifier_c, 30,
        RecipeModifierAction::Observe);
    static RecipeModifierProbe modifier_setup_failure(
        reinterpret_cast<orchestrator_handle *>(&orchestrator), k_recipe_modifier_setup_failure, 60,
        RecipeModifierAction::FailSetupRun);
    static RecipeModifierProbe modifier_pattern_callback(
        reinterpret_cast<orchestrator_handle *>(&orchestrator), k_recipe_modifier_pattern_callback, 40,
        RecipeModifierAction::CallFailingPattern);
    static RecipeModifierProbe modifier_pattern_cancellation(
        reinterpret_cast<orchestrator_handle *>(&orchestrator), k_recipe_modifier_pattern_cancellation, 50,
        RecipeModifierAction::CallCancelingPattern);

    for (RecipeModifierProbe *probe : { &modifier_a, &modifier_b, &modifier_c, &modifier_setup_failure,
                                        &modifier_pattern_callback, &modifier_pattern_cancellation }) {
        if (orchestrator.get_plugin(probe->id()) == nullptr && !orchestrator.register_plugin(probe->c_instance()))
            throw std::runtime_error("Cannot register infill recipe modifier probe");
    }
}

DynamicPrintConfig gyroid_test_config()
{
    return perimeter_config({
        { "fill_pattern", "gyroid" },
        { "fill_density", "20%" },
        { "top_solid_layers", "0" },
        { "bottom_solid_layers", "0" },
        { "solid_infill_every_layers", "0" },
        { "infill_every_layers", "1" }
    });
}

struct DiscoveryRecorder
{
    infill_pattern_runtime_id reported_pattern_id = INFILL_PATTERN_RUNTIME_ID_INVALID;
    size_t report_calls = 0;
    size_t reported_groups = 0;
    size_t reported_surfaces = 0;
    bool invalid_assignment = false;
    bool all_reported_params_used = true;
    std::set<const raw_infill_pattern_params *> reported_params;
    size_t generate_calls = 0;
    size_t append_calls = 0;
};

int32_t record_report(const run_ctx_generate_infill *ctx,
                      infill_pattern_runtime_id pattern_id,
                      const raw_infill_pattern_assignment *assignments,
                      uint32_t assignment_count)
{
    DiscoveryRecorder *recorder = static_cast<DiscoveryRecorder *>(ctx->host_context);
    recorder->reported_pattern_id = pattern_id;
    ++recorder->report_calls;
    for (uint32_t index = 0; index < assignment_count; ++index) {
        const raw_infill_pattern_assignment &assignment = assignments[index];
        if (assignment.layer_island == nullptr || assignment.region_island == nullptr ||
            assignment.params == nullptr || assignment.surfaces == nullptr || assignment.surface_count == 0) {
            recorder->invalid_assignment = true;
            continue;
        }
        if (assignment.params->pattern_id != pattern_id || assignment.params->density <= 0.f)
            recorder->invalid_assignment = true;
        for (uint32_t surface = 0; surface < assignment.surface_count; ++surface)
            if (assignment.surfaces[surface] == nullptr)
                recorder->invalid_assignment = true;
        ++recorder->reported_groups;
        recorder->reported_surfaces += assignment.surface_count;
        recorder->reported_params.insert(assignment.params);
    }
    return 1;
}

infill_pattern_runtime_id record_resolve_pattern_id(const run_ctx_generate_infill *, const char *plugin_id)
{
    return plugin_id != nullptr && std::string(plugin_id) == "gyroid" ? k_gyroid_runtime_id :
                                                                       INFILL_PATTERN_RUNTIME_ID_INVALID;
}

const char *record_pattern_plugin_id(const run_ctx_generate_infill *, infill_pattern_runtime_id pattern_id)
{
    return pattern_id == k_gyroid_runtime_id ? "gyroid" : "";
}

int32_t record_modify_surface_recipe(const run_ctx_generate_infill *,
                                     const layer_handle *,
                                     const layer_island_handle *,
                                     const layer_region_island_handle *,
                                     const layer_region_handle *,
                                     const surface_handle *,
                                     const expolygon_collection_handle *,
                                     raw_infill_pattern_params *)
{
    return 1;
}

int32_t record_generate_pattern(const run_ctx_generate_infill *ctx,
                                infill_pattern_runtime_id pattern_id,
                                const layer_handle *,
                                const layer_island_handle *,
                                const layer_region_island_handle *,
                                const layer_region_handle *,
                                const surface_handle *,
                                const expolygon_collection_handle *,
                                const raw_infill_pattern_params *params,
                                extrusion_entity_handle *)
{
    DiscoveryRecorder *recorder = static_cast<DiscoveryRecorder *>(ctx->host_context);
    ++recorder->generate_calls;
    if (pattern_id != k_gyroid_runtime_id || params == nullptr ||
        recorder->reported_params.find(params) == recorder->reported_params.end())
        recorder->all_reported_params_used = false;
    // Return without extrusion: the test only observes the call protocol.
    return 0;
}

int32_t record_append_region_island_extrusion(layer_region_island_handle *, raw_extrusion_role, extrusion_entity_handle *)
{
    return 1;
}

} // namespace

TEST_CASE("STEP_INFILL discovery groups surfaces and replays them from the cache",
          "[plugins][infill][init]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, gyroid_test_config());

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        "gyroid"
    });

    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepGeneratePerimeter::clear(prepared.print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::clear(prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);

    Plugin *generator = orchestrator.get_plugin("infill.generator.default");
    REQUIRE(generator != nullptr);

    DiscoveryRecorder recorder;
    plugin_host_context host;
    host.orchestrator = &orchestrator;
    host.print = &prepared.print;
    host.plugin = generator;
    host.step = STEP_INFILL;
    plugin_run_context run_ctx = orchestrator.prepare_plugin_run_context(STEP_INFILL, generator, &host);

    run_ctx_generate_infill payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&prepared.print);
    payload.object = reinterpret_cast<const object_handle *>(&prepared.print.object(0));
    payload.host_context = &recorder;
    payload.resolve_pattern_id = &record_resolve_pattern_id;
    payload.pattern_plugin_id = &record_pattern_plugin_id;
    payload.generate_pattern = &record_generate_pattern;
    payload.modify_surface_recipe = &record_modify_surface_recipe;
    payload.append_region_island_extrusion = &record_append_region_island_extrusion;
    payload.report_pattern_assignments = &record_report;
    run_ctx.data = &payload;

    generator->setup(run_ctx, 1);
    generator->setup_run(run_ctx);

    // Discovery must report at least one complete group for the used pattern.
    CHECK_FALSE(recorder.invalid_assignment);
    CHECK(recorder.report_calls > 0);
    CHECK(recorder.reported_groups > 0);
    CHECK(recorder.reported_surfaces > 0);
    CHECK(recorder.reported_pattern_id == k_gyroid_runtime_id);

    // The run phase replays the discovery result: one pattern call per reported
    // surface, with the exact params pointer that was reported.
    const size_t discovered_surfaces = recorder.reported_surfaces;
    generator->run(run_ctx);
    CHECK(recorder.generate_calls == discovered_surfaces);
    CHECK(recorder.all_reported_params_used);
    CHECK(recorder.append_calls == 0);

    // The cache is consumed by the run. A later run falls back to rebuilding
    // the recipes and still produces the same pattern calls.
    recorder.generate_calls = 0;
    generator->run(run_ctx);
    CHECK(recorder.generate_calls == discovered_surfaces);
}

// Build a test-owned gyroid-configured print, then activate only the shared partial-write pattern so the host's normal
// inactive-id fallback selects it. The probe confirms its write before failure, while the test captures a
// cancelled-during-run stop rather than a reported generator error and checks the original diagnostic appears once.
TEST_CASE("STEP_INFILL contains a failing pattern callback", "[plugins][infill][patterns][callback-failure]")
{
    using namespace Test::ProbePatternPlugin;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    REQUIRE(Orchestrator::instance().get_plugin(probe_pattern_id) != nullptr);

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, gyroid_test_config());

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        probe_pattern_id
    });
    ScopedInfillStepDiagnostics diagnostics;
    ScopedFailingInfillPattern failing(2);
    Orchestrator &orchestrator = Orchestrator::instance();

    Steps::StepGeneratePerimeter::clear(prepared.print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::clear(prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    Steps::StepGenerateInfill::clear(prepared.print);

    std::string failure;
    try {
        Steps::StepGenerateInfill::run_step(orchestrator, prepared.print);
    } catch (const std::exception &error) {
        failure = error.what();
    }
    const std::vector<Orchestrator::PluginMessage> messages = orchestrator.consume_plugin_messages();

    CHECK(pattern_wrote_before_failure);
    CHECK(failure.find("STEP_INFILL stopped after plugin 'infill.generator.default' was cancelled during run") !=
          std::string::npos);
    CHECK(failure.find("reported an error") == std::string::npos);
    CHECK(orchestrator.is_plugin_cancelled());
    CHECK(std::count_if(messages.begin(), messages.end(), [](const Orchestrator::PluginMessage &message) {
        return message.level == Orchestrator::PluginMessageLevel::Error &&
               message.plugin_id == probe_pattern_id && message.message.find("probe pattern run failure") != std::string::npos;
    }) == 1);
}

// Run the modifier chain on a test-owned print: A edits the borrowed recipe, B observes then throws, and C must not run;
// the test captures the setup_run stop and drains diagnostics after the runner returns or throws.
TEST_CASE("STEP_INFILL stops the recipe modifier chain at its first failure",
          "[plugins][infill][patterns][recipe-modifier-failure]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_recipe_modifier_probes();
    recipe_modifier_calls.clear();
    recipe_modifier_density_seen = -1.f;

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({
        { "fill_pattern", "rectilinear" },
        { "fill_density", "20%" },
        { "top_solid_layers", "0" },
        { "bottom_solid_layers", "0" },
        { "solid_infill_every_layers", "0" },
        { "infill_every_layers", "1" }
    }));

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        "rectilinear",
        k_recipe_modifier_a,
        k_recipe_modifier_b,
        k_recipe_modifier_c
    });
    ScopedInfillStepDiagnostics diagnostics;
    Orchestrator &orchestrator = Orchestrator::instance();

    Steps::StepGeneratePerimeter::clear(prepared.print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::clear(prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    Steps::StepGenerateInfill::clear(prepared.print);

    std::string failure;
    try {
        Steps::StepGenerateInfill::run_step(orchestrator, prepared.print);
    } catch (const std::exception &error) {
        failure = error.what();
    }
    const std::vector<Orchestrator::PluginMessage> messages = orchestrator.consume_plugin_messages();

    CHECK(recipe_modifier_calls == std::vector<std::string>{ k_recipe_modifier_a, k_recipe_modifier_b });
    CHECK(recipe_modifier_density_seen == Approx(0.375f));
    CHECK(failure.find("STEP_INFILL stopped after plugin 'infill.generator.default' was cancelled during setup_run") !=
          std::string::npos);
    CHECK(failure.find("reported an error") == std::string::npos);
    CHECK(orchestrator.is_plugin_cancelled());
    CHECK(std::count_if(messages.begin(), messages.end(), [](const Orchestrator::PluginMessage &message) {
        return message.level == Orchestrator::PluginMessageLevel::Error &&
               message.plugin_id == k_recipe_modifier_b &&
               message.message.find("infill recipe modifier probe failure") != std::string::npos;
    }) == 1);
    CHECK(std::none_of(messages.begin(), messages.end(), [](const Orchestrator::PluginMessage &message) {
        return message.message.find("stopped after plugin '" + std::string(k_recipe_modifier_b) + "'") !=
               std::string::npos;
    }));
}

// A modifier failing in setup_run keeps its own diagnostic: the callback returns before run and without blaming it.
TEST_CASE("STEP_INFILL contains a recipe modifier failing in setup_run",
          "[plugins][infill][patterns][recipe-modifier-setup-failure]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_recipe_modifier_probes();
    recipe_modifier_calls.clear();

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({
        { "fill_pattern", "rectilinear" },
        { "fill_density", "20%" },
        { "top_solid_layers", "0" },
        { "bottom_solid_layers", "0" },
        { "solid_infill_every_layers", "0" },
        { "infill_every_layers", "1" }
    }));

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        "rectilinear",
        k_recipe_modifier_setup_failure
    });
    ScopedInfillStepDiagnostics diagnostics;
    Orchestrator &orchestrator = Orchestrator::instance();

    Steps::StepGeneratePerimeter::clear(prepared.print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::clear(prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    Steps::StepGenerateInfill::clear(prepared.print);

    std::string failure;
    try {
        Steps::StepGenerateInfill::run_step(orchestrator, prepared.print);
    } catch (const std::exception &error) {
        failure = error.what();
    }
    const std::vector<Orchestrator::PluginMessage> messages = orchestrator.consume_plugin_messages();

    CHECK(recipe_modifier_calls.empty());
    CHECK(failure.find("STEP_INFILL stopped after plugin 'infill.generator.default' was cancelled during setup_run") !=
          std::string::npos);
    CHECK(orchestrator.is_plugin_cancelled());
    CHECK(std::count_if(messages.begin(), messages.end(), [](const Orchestrator::PluginMessage &message) {
        return message.level == Orchestrator::PluginMessageLevel::Error &&
               message.plugin_id == k_recipe_modifier_setup_failure &&
               message.message.find("infill recipe modifier setup_run probe failure") != std::string::npos;
    }) == 1);
    CHECK(std::none_of(messages.begin(), messages.end(), [](const Orchestrator::PluginMessage &message) {
        return message.message.find("stopped after plugin '" + std::string(k_recipe_modifier_setup_failure) + "'") !=
               std::string::npos;
    }));
}

// White-box coverage calls the parent generate_pattern callback from a recipe modifier; normal modifiers edit recipes
// and do not generate extrusion. Both partial-write forms return zero with empty output, and stopped callbacks refuse.
TEST_CASE("STEP_INFILL recipe modifiers observe a contained pattern failure",
          "[plugins][infill][patterns][recipe-modifier-pattern-callback]")
{
    using namespace Test::ProbePatternPlugin;
    const int how = GENERATE(1, 2);
    CAPTURE(how);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    REQUIRE(Orchestrator::instance().get_plugin(probe_pattern_id) != nullptr);
    register_recipe_modifier_probes();
    recipe_modifier_calls.clear();
    recipe_modifier_pattern_callback_record = RecipeModifierPatternCallbackRecord{};

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, gyroid_test_config());

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        probe_pattern_id,
        k_recipe_modifier_pattern_callback
    });
    ScopedInfillStepDiagnostics diagnostics;
    ScopedFailingInfillPattern failing(how);
    Orchestrator &orchestrator = Orchestrator::instance();

    Steps::StepGeneratePerimeter::clear(prepared.print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::clear(prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    Steps::StepGenerateInfill::clear(prepared.print);
    (void) orchestrator.consume_plugin_messages();

    std::string failure;
    try {
        Steps::StepGenerateInfill::run_step(orchestrator, prepared.print);
    } catch (const std::exception &error) {
        failure = error.what();
    }
    const std::vector<Orchestrator::PluginMessage> messages = orchestrator.consume_plugin_messages();
    const std::string stopped_message =
        "stopped after plugin '" + std::string(probe_pattern_id) + "' reported an error during";

    CHECK(pattern_wrote_before_failure);
    CHECK(recipe_modifier_calls == std::vector<std::string>{ k_recipe_modifier_pattern_callback });
    CHECK(recipe_modifier_pattern_callback_record.called);
    CHECK(recipe_modifier_pattern_callback_record.pattern_id != INFILL_PATTERN_RUNTIME_ID_INVALID);
    CHECK(recipe_modifier_pattern_callback_record.first_result == 0);
    CHECK(recipe_modifier_pattern_callback_record.first_output_empty);
    CHECK(recipe_modifier_pattern_callback_record.second_result == 0);
    CHECK(recipe_modifier_pattern_callback_record.second_output_empty);
    CHECK(recipe_modifier_pattern_callback_record.resolve_after == INFILL_PATTERN_RUNTIME_ID_INVALID);
    CHECK(failure.find("STEP_INFILL stopped after plugin 'infill.generator.default' was cancelled during setup_run") !=
          std::string::npos);
    CHECK(failure.find("reported an error") == std::string::npos);
    CHECK(orchestrator.is_plugin_cancelled());
    CHECK(std::count_if(messages.begin(), messages.end(), [&stopped_message](const Orchestrator::PluginMessage &message) {
        return message.level == Orchestrator::PluginMessageLevel::Error && message.plugin_id == "<unknown>" &&
               message.message.find(stopped_message) != std::string::npos;
    }) == 1);
    CHECK(std::count_if(messages.begin(), messages.end(), [](const Orchestrator::PluginMessage &message) {
        return message.level == Orchestrator::PluginMessageLevel::Error &&
               message.plugin_id == probe_pattern_id &&
               message.message.find("probe pattern run failure") != std::string::npos;
    }) == 1);
    // Two errors remain after the prerequisite steps are drained: the probe's original run failure and the host's
    // INFILL_PATTERN containment report. The modifier callback returns without blaming the modifier for the stop.
    CHECK(std::count_if(messages.begin(), messages.end(), [](const Orchestrator::PluginMessage &message) {
        return message.level == Orchestrator::PluginMessageLevel::Error;
    }) == 2);
    CHECK(std::none_of(messages.begin(), messages.end(), [](const Orchestrator::PluginMessage &message) {
        return message.message.find("stopped after plugin '" + std::string(k_recipe_modifier_pattern_callback) + "'") !=
               std::string::npos;
    }));
}

// Cancel a test-owned print inside the probe pattern reached from a recipe modifier; its local output remains empty,
// the callback returns zero, and the runner propagates CanceledException without error or plugin cancellation.
TEST_CASE("STEP_INFILL recipe modifiers leave pattern print cancellation to the runner",
          "[plugins][infill][patterns][recipe-modifier-print-cancellation]")
{
    using namespace Test::ProbePatternPlugin;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    REQUIRE(Orchestrator::instance().get_plugin(probe_pattern_id) != nullptr);
    register_recipe_modifier_probes();
    recipe_modifier_calls.clear();
    recipe_modifier_pattern_callback_record = RecipeModifierPatternCallbackRecord{};

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, gyroid_test_config());

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        probe_pattern_id,
        k_recipe_modifier_pattern_cancellation
    });
    ScopedInfillStepDiagnostics diagnostics;
    ScopedFailingInfillPattern failing(0);
    Test::ProbePatternPlugin::pattern_cancel_print = &prepared.print;
    Orchestrator &orchestrator = Orchestrator::instance();

    Steps::StepGeneratePerimeter::clear(prepared.print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::clear(prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    Steps::StepGenerateInfill::clear(prepared.print);
    (void) orchestrator.consume_plugin_messages();

    CHECK_THROWS_AS(Steps::StepGenerateInfill::run_step(orchestrator, prepared.print), CanceledException);
    const std::vector<Orchestrator::PluginMessage> messages = orchestrator.consume_plugin_messages();

    CHECK(recipe_modifier_calls == std::vector<std::string>{ k_recipe_modifier_pattern_cancellation });
    CHECK(recipe_modifier_pattern_callback_record.called);
    CHECK(recipe_modifier_pattern_callback_record.pattern_id != INFILL_PATTERN_RUNTIME_ID_INVALID);
    CHECK(recipe_modifier_pattern_callback_record.first_result == 0);
    CHECK(recipe_modifier_pattern_callback_record.first_output_empty);
    CHECK(prepared.print.canceled());
    CHECK_FALSE(orchestrator.is_plugin_cancelled());
    CHECK(std::none_of(messages.begin(), messages.end(), [](const Orchestrator::PluginMessage &message) {
        return message.level == Orchestrator::PluginMessageLevel::Error;
    }));
}
