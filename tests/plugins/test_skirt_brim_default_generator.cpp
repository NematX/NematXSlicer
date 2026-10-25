#include <catch2/catch.hpp>

#include <initializer_list>
#include <string>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PluginProperty.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "libslic3r/Steps/StepSkirtBrim.hpp"

/*
Default skirt/brim generator tests
==================================

These tests exercise the new STEP_SKIRT_BRIM boundary, not the whole print
pipeline. They slice a simple cube far enough to create first-layer slices, then
run the built-in adhesion plugins. This keeps the test focused on three
contracts:

- STEP_SKIRT_BRIM is a non-exclusive chain, while brim and skirt sub-features
  are separate exclusive groups selected by plugin id.
- The default brim/skirt plugins write global, per-object, and first-layer-only
  extrusion through tagged auxiliary layers.
- The host rebuilds the first-layer convex hull from auxiliary adhesion output.
*/

namespace {

using namespace Slic3r;

const char *const STANDARD_LAYER_HEIGHT_GENERATOR = "standard_layer_height_generator";
const char *const SLICE_VOLUME = "slice_volume";
const char *const DEFAULT_BRIM_GENERATOR = "skirt_brim.brim.default";
const char *const DEFAULT_SKIRT_GENERATOR = "skirt_brim.skirt.default";
const char *const DEFAULT_BRIM_SKIRT_TRIM = "skirt_brim.brim_skirt_trim.default";
const char *const FAILING_SKIRT_BRIM_PLUGIN = "skirt_brim.test.report_error";
const char *const BRIM_SEPARATION_READER = "skirt_brim.test.brim_separation_reader";

enum class FailingPluginPhase
{
    Setup,
    SetupRun,
    Run
};

void report_test_error(const plugin_run_context *run_context)
{
    if (run_context != nullptr && run_context->report_error != nullptr)
        run_context->report_error(run_context->host_context, "intentional step plugin error");
}

class FailingSkirtBrimPlugin final : public slic3r_api::PluginBase
{
public:
    explicit FailingSkirtBrimPlugin(orchestrator_handle *orchestrator)
        : PluginBase(orchestrator)
    {}

    void set_phase(FailingPluginPhase phase) { m_phase = phase; }

private:
    const char *id_impl() const noexcept override { return FAILING_SKIRT_BRIM_PLUGIN; }
    slicing_step_t step_impl() const noexcept override { return STEP_SKIRT_BRIM; }
    const char *const *dependencies_impl() const noexcept override
    {
        static const char *dependencies[] = {nullptr};
        return dependencies;
    }
    int32_t priority_impl() const noexcept override { return 0; }

    void setup_impl(const plugin_run_context *run_context, uint32_t) const override
    {
        if (m_phase == FailingPluginPhase::Setup)
            report_test_error(run_context);
    }

    void setup_run_impl(const plugin_run_context *run_context) const override
    {
        if (m_phase == FailingPluginPhase::SetupRun)
            report_test_error(run_context);
    }

    void run_impl(const plugin_run_context *run_context) const override
    {
        if (m_phase == FailingPluginPhase::Run)
            report_test_error(run_context);
    }

    FailingPluginPhase m_phase = FailingPluginPhase::Setup;
};

FailingSkirtBrimPlugin &failing_skirt_brim_plugin()
{
    static FailingSkirtBrimPlugin plugin(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    return plugin;
}

void register_failing_skirt_brim_plugin()
{
    Orchestrator &orchestrator = Orchestrator::instance();
    if (orchestrator.get_plugin(FAILING_SKIRT_BRIM_PLUGIN) == nullptr)
        REQUIRE(orchestrator.register_plugin(failing_skirt_brim_plugin().c_instance()));
}

// Reads brim_separation from a step that runs before support. The plugin is
// registered only after the test runtime finalized the option definitions, so
// it would move the setting's invalidation step only if plugin initialization
// still edited definitions.
class BrimSeparationReader final : public slic3r_api::PluginBase
{
public:
    explicit BrimSeparationReader(orchestrator_handle *orchestrator)
        : PluginBase(orchestrator)
    {}

private:
    const char *id_impl() const noexcept override { return BRIM_SEPARATION_READER; }
    slicing_step_t step_impl() const noexcept override { return STEP_POST_PERIMETER; }
    const char *const *dependencies_impl() const noexcept override
    {
        static const char *dependencies[] = {nullptr};
        return dependencies;
    }
    int32_t priority_impl() const noexcept override { return 0; }

    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            keys[0] = raw_used_config_key{
                "brim_separation", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_OBJECT, RAW_PRESET_TYPE_FFF_PRINT};
        return 1;
    }

    void run_impl(const plugin_run_context *) const override {}
};

class ScopedActivePlugins
{
public:
    explicit ScopedActivePlugins(std::initializer_list<const char *> plugin_ids) :
        ScopedActivePlugins(std::vector<const char *>(plugin_ids))
    {}

    explicit ScopedActivePlugins(const std::vector<const char *> &plugin_ids) :
        m_orchestrator(Orchestrator::instance())
    {
        m_previous_active_plugins.reserve(m_orchestrator.active_plugins().size());
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous_active_plugins.push_back(plugin);

        m_orchestrator.clear_active_plugins();
        for (const char *plugin_id : plugin_ids) {
            INFO("Activating test plugin " << plugin_id);
            REQUIRE(m_orchestrator.set_plugin_active(plugin_id, true));
        }
    }

    ~ScopedActivePlugins()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous_active_plugins)
            m_orchestrator.set_plugin_active(plugin, true);
        m_orchestrator.reset_plugin_cancel();
    }

private:
    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous_active_plugins;
};

DynamicPrintConfig brim_test_config(const bool brim_enabled, const bool per_object)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height", "0.2" },
        { "first_layer_height", "0.2" },
        { "brim_width", brim_enabled ? "3" : "0" },
        { "brim_width_interior", "0" },
        { "brim_per_object", per_object ? "1" : "0" },
        { "brim_ears", "0" },
        { "skirts", "0" },
        { "skirt_brim", "0" },
        { "support_material", "0" },
        { "raft_layers", "0" },
        { "perimeters", "2" },
        { "fill_density", "0" },
        { "step_layer_height_plugin", STANDARD_LAYER_HEIGHT_GENERATOR }
    });
    return config;
}

DynamicPrintConfig skirt_test_config(const int skirts, const int skirt_brim, const bool per_object)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height", "0.2" },
        { "first_layer_height", "0.2" },
        { "brim_width", "0" },
        { "brim_width_interior", "0" },
        { "brim_per_object", "0" },
        { "brim_ears", "0" },
        { "skirts", std::to_string(skirts) },
        { "skirt_height", "1" },
        { "skirt_distance", "3" },
        { "skirt_distance_from_brim", "0" },
        { "skirt_brim", std::to_string(skirt_brim) },
        { "min_skirt_length", "0" },
        { "draft_shield", "disabled" },
        { "complete_objects", per_object ? "1" : "0" },
        { "complete_objects_one_skirt", "0" },
        { "support_material", "0" },
        { "raft_layers", "0" },
        { "perimeters", "2" },
        { "fill_density", "0" },
        { "step_layer_height_plugin", STANDARD_LAYER_HEIGHT_GENERATOR }
    });
    return config;
}

struct PreparedBrimPrint
{
    Model model;
    Print print;
};

void run_prepared_print_until_skirt_brim(PreparedBrimPrint &prepared,
                                         bool include_trim = true);

void run_until_skirt_brim(PreparedBrimPrint &prepared,
                          const DynamicPrintConfig &config,
                          const bool include_trim = true)
{
    Slic3r::Test::init_print({ Slic3r::Test::TestMesh::cube_20x20x20 }, prepared.print, prepared.model, config);
    run_prepared_print_until_skirt_brim(prepared, include_trim);
}

void run_mesh_until_skirt_brim(PreparedBrimPrint &prepared,
                               const TriangleMesh &mesh,
                               const DynamicPrintConfig &config)
{
    Slic3r::Test::init_print({ mesh }, prepared.print, prepared.model, config);
    run_prepared_print_until_skirt_brim(prepared);
}

void run_prepared_print_until_skirt_brim(PreparedBrimPrint &prepared,
                                         const bool include_trim)
{
    std::vector<const char *> active_plugin_ids = {
        STANDARD_LAYER_HEIGHT_GENERATOR,
        SLICE_VOLUME,
        DEFAULT_BRIM_GENERATOR,
        DEFAULT_SKIRT_GENERATOR
    };
    if (include_trim)
        active_plugin_ids.push_back(DEFAULT_BRIM_SKIRT_TRIM);
    ScopedActivePlugins active(active_plugin_ids);

    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, prepared.print);
    Steps::StepSlicing::run_step(orchestrator, prepared.print);
    (void) orchestrator.consume_plugin_messages();
    Steps::StepSkirtBrim::run_step(orchestrator, prepared.print);
    for (const Orchestrator::PluginMessage &message : orchestrator.consume_plugin_messages()) {
        INFO("Plugin " << message.plugin_id << " reported: " << message.message);
        CHECK(message.level != Orchestrator::PluginMessageLevel::Error);
    }
}

double first_layer_slice_area_mm2(const Print &print)
{
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = print.objects().front();
    REQUIRE(object.layer_count() > 0);
    return unscaled(unscaled(std::abs(area(object.layers().front().lslices()))));
}

double hull_area_mm2(const Print &print)
{
    return unscaled(unscaled(std::abs(area(print.first_layer_convex_hull()))));
}

double extrusion_tree_length_mm(const ExtrusionEntity &entity)
{
    ArcPolylines polylines;
    entity.collect_polylines(polylines);

    double length = 0.0;
    for (const ArcPolyline &polyline : polylines)
        length += unscaled(polyline.length());
    return length;
}

std::vector<const Layer *> object_brim_auxiliary_layers(const PrintObject &object)
{
    std::vector<const Layer *> out;
    for (const Layer &layer : object.auxiliary_layers())
        if (LayerAdhesionProperty::layer_is_brim(layer))
            out.push_back(&layer);
    return out;
}

} // namespace

TEST_CASE("STEP_SKIRT_BRIM stops immediately when a plugin reports an error",
          "[plugins][steps][errors]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_failing_skirt_brim_plugin();

    struct Case {
        FailingPluginPhase phase;
        const char *phase_name;
    };

    const Case cases[] = {
        {FailingPluginPhase::Setup, "setup"},
        {FailingPluginPhase::SetupRun, "setup_run"},
        {FailingPluginPhase::Run, "run"}
    };

    for (const Case &test_case : cases) {
        DYNAMIC_SECTION("report_error during " << test_case.phase_name) {
            Orchestrator &orchestrator = Orchestrator::instance();
            orchestrator.reset_plugin_cancel();
            failing_skirt_brim_plugin().set_phase(test_case.phase);
            ScopedActivePlugins active({FAILING_SKIRT_BRIM_PLUGIN});
            Print print;

            try {
                Steps::StepSkirtBrim::run_step(orchestrator, print);
                FAIL("STEP_SKIRT_BRIM should throw after plugin report_error().");
            } catch (const RuntimeError &error) {
                const std::string message = error.what();
                CHECK(message.find("STEP_SKIRT_BRIM stopped after plugin '") != std::string::npos);
                CHECK(message.find(FAILING_SKIRT_BRIM_PLUGIN) != std::string::npos);
                CHECK(message.find("reported an error during " + std::string(test_case.phase_name)) !=
                      std::string::npos);
            }

            CHECK(orchestrator.is_plugin_cancelled());
        }
    }
}

TEST_CASE("STEP_SKIRT_BRIM exposes a brim exclusive group but is not exclusive itself",
          "[plugins][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *plugin = orchestrator.get_plugin(DEFAULT_BRIM_GENERATOR);
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_step() == STEP_SKIRT_BRIM);
    CHECK(plugin->get_exclusive_group() == "skirt_brim.brim");

    Plugin *skirt_plugin = orchestrator.get_plugin(DEFAULT_SKIRT_GENERATOR);
    REQUIRE(skirt_plugin != nullptr);
    CHECK(skirt_plugin->get_step() == STEP_SKIRT_BRIM);
    CHECK(skirt_plugin->get_exclusive_group() == "skirt_brim.skirt");

    Plugin *trim_plugin = orchestrator.get_plugin(DEFAULT_BRIM_SKIRT_TRIM);
    REQUIRE(trim_plugin != nullptr);
    CHECK(trim_plugin->get_step() == STEP_SKIRT_BRIM);
    CHECK(trim_plugin->get_exclusive_group() == "skirt_brim.brim_skirt_trim");

    bool found_brim_setting_fragment = false;
    bool found_trim_setting_fragment = false;
    const std::vector<Orchestrator::PluginUiFragment> print_fragments =
        orchestrator.ui_fragments_for_file("print.ui");
    for (const Orchestrator::PluginUiFragment &fragment : print_fragments) {
        if (fragment.fragment_id == "skirt_brim.brim.default.settings") {
            found_brim_setting_fragment = true;
            CHECK(fragment.content.find("insert$aftersetting$brim_width:brim_inside_holes") !=
                  std::string::npos);
            CHECK(fragment.content.find("insert$aftersetting$brim_width_interior:Brim ears") !=
                  std::string::npos);
            CHECK(fragment.content.find("tags$Simple:sidetext_width$0:brim_ears") !=
                  std::string::npos);
        }
        if (fragment.fragment_id == "brim_skirt_trim") {
            found_trim_setting_fragment = true;
            CHECK(fragment.content.find("group:Brim") != std::string::npos);
            CHECK(fragment.content.find("insert$aftersetting$brim_per_object:brim_skirt_trim") !=
                  std::string::npos);
        }
    }
    CHECK(found_brim_setting_fragment);
    CHECK(found_trim_setting_fragment);

    bool ears_require_outer_brim = false;
    bool inside_holes_require_outer_brim = false;
    bool inside_holes_require_no_inner_brim = false;
    bool pattern_requires_ears = false;
    bool detection_requires_ears = false;
    const auto has_clause = [](const Orchestrator::PluginGuiRule &rule, const char *condition_key,
                               raw_gui_rule_condition condition) {
        for (const Orchestrator::PluginGuiRuleClause &clause : rule.clauses)
            if (clause.condition == condition && clause.condition_key == condition_key)
                return true;
        return false;
    };
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules()) {
        ears_require_outer_brim = ears_require_outer_brim ||
            (rule.target_key == "brim_ears" &&
             has_clause(rule, "brim_width", RAW_GUI_RULE_CONDITION_VALUE_NON_ZERO));
        inside_holes_require_outer_brim = inside_holes_require_outer_brim ||
            (rule.target_key == "brim_inside_holes" &&
             has_clause(rule, "brim_width", RAW_GUI_RULE_CONDITION_VALUE_NON_ZERO));
        inside_holes_require_no_inner_brim = inside_holes_require_no_inner_brim ||
            (rule.target_key == "brim_inside_holes" &&
             has_clause(rule, "brim_width_interior", RAW_GUI_RULE_CONDITION_BOOL_FALSE));
        pattern_requires_ears = pattern_requires_ears ||
            (rule.target_key == "brim_ears_pattern" &&
             has_clause(rule, "brim_ears", RAW_GUI_RULE_CONDITION_BOOL_TRUE));
        detection_requires_ears = detection_requires_ears ||
            (rule.target_key == "brim_ears_detection_length" &&
             has_clause(rule, "brim_ears", RAW_GUI_RULE_CONDITION_BOOL_TRUE));
    }
    CHECK(ears_require_outer_brim);
    CHECK(inside_holes_require_outer_brim);
    CHECK(inside_holes_require_no_inner_brim);
    CHECK(pattern_requires_ears);
    CHECK(detection_requires_ears);

    const std::map<slicing_step_t, Steps::StepExclusiveGroup> &exclusive_steps = Steps::get_exclusive_steps();
    CHECK(exclusive_steps.find(STEP_SKIRT_BRIM) == exclusive_steps.end());
}

TEST_CASE("Default brim generator owns its serialized geometry options", "[plugins][skirt-brim][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const ConfigOptionDef *inside_holes = PrintConfigDef::instance().get("brim_inside_holes");
    const ConfigOptionDef *ears = PrintConfigDef::instance().get("brim_ears");
    const ConfigOptionDef *max_angle = PrintConfigDef::instance().get("brim_ears_max_angle");
    const ConfigOptionDef *detection_length = PrintConfigDef::instance().get("brim_ears_detection_length");
    const ConfigOptionDef *pattern = PrintConfigDef::instance().get("brim_ears_pattern");
    const ConfigOptionDef *separation = PrintConfigDef::instance().get("brim_separation");
    REQUIRE(inside_holes != nullptr);
    REQUIRE(ears != nullptr);
    REQUIRE(max_angle != nullptr);
    REQUIRE(detection_length != nullptr);
    REQUIRE(pattern != nullptr);
    REQUIRE(separation != nullptr);

    /*
    Leaving invalidates_step at STEP_NONE in the raw definitions asks the
    orchestrator to resolve it to the owning plugin's step.
    */
    for (const ConfigOptionDef *def : { inside_holes, ears, max_angle, detection_length, pattern }) {
        CHECK(def->container_type == ConfigOptionContainerType::Object);
        CHECK(def->option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
        CHECK(def->invalidates_step == STEP_SKIRT_BRIM);
    }

    CHECK(inside_holes->type == coBool);
    CHECK(inside_holes->default_value->serialize() == "0");
    CHECK(ears->type == coBool);
    CHECK(ears->default_value->serialize() == "0");
    CHECK(max_angle->type == coFloat);
    CHECK(max_angle->min == 0.0);
    CHECK(max_angle->max == 180.0);
    CHECK(max_angle->default_value->serialize() == "125");
    CHECK(detection_length->type == coFloat);
    CHECK(detection_length->min == 0.0);
    CHECK(detection_length->default_value->serialize() == "1");
    CHECK(pattern->type == coEnum);
    CHECK(pattern->gui_type == ConfigOptionDef::GUIType::select_close);
    CHECK(pattern->has_enum_value("concentric"));
    CHECK(pattern->has_enum_value("rectilinear"));
    CHECK_FALSE(pattern->has_enum_value("grid"));
    CHECK(pattern->default_value->serialize() == "concentric");
    CHECK(separation->type == coFloat);
    CHECK(separation->container_type == ConfigOptionContainerType::Object);
    CHECK(separation->option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    // Support material also reads this option and executes before skirt/brim,
    // so finalizing the plugin definitions tightens the replay point by
    // execution_order().
    CHECK(separation->invalidates_step == STEP_SUPPORT);
    CHECK(separation->sidetext == "mm");
    CHECK(separation->default_value->serialize() == "0");
    REQUIRE(separation->aliases.size() == 1);
    CHECK(separation->aliases.front() == "brim_offset");

    /*
    brim_separation is editable while a brim is requested on either shell, in
    the print scope: one reason per accepted case.
    */
    Orchestrator &orchestrator = Orchestrator::instance();
    size_t separation_rules = 0;
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules()) {
        if (rule.owner != DEFAULT_BRIM_GENERATOR || rule.target_key != "brim_separation")
            continue;
        ++separation_rules;
        CHECK(rule.scope == RAW_PRESET_TYPE_FFF_PRINT);
    }
    CHECK(separation_rules == 2);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "brim_inside_holes", "1" },
        { "brim_ears", "1" },
        { "brim_ears_max_angle", "73.5" },
        { "brim_ears_detection_length", "2.25" },
        { "brim_ears_pattern", "rectilinear" }
    });
    CHECK(config.option("brim_inside_holes")->serialize() == "1");
    CHECK(config.option("brim_ears")->serialize() == "1");
    CHECK(config.option("brim_ears_max_angle")->serialize() == "73.5");
    CHECK(config.option("brim_ears_detection_length")->serialize() == "2.25");
    CHECK(config.option("brim_ears_pattern")->serialize() == "rectilinear");
}

TEST_CASE("Re-running plugin initialization leaves finalized invalidation steps unchanged",
          "[plugins][skirt-brim][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    static BrimSeparationReader reader(reinterpret_cast<orchestrator_handle *>(&orchestrator));
    if (orchestrator.get_plugin(BRIM_SEPARATION_READER) == nullptr)
        REQUIRE(orchestrator.register_plugin(reader.c_instance()));

    const ConfigOptionDef *separation = PrintConfigDef::instance().get("brim_separation");
    REQUIRE(separation != nullptr);
    REQUIRE(separation->invalidates_step == STEP_SUPPORT);

    {
        // Validation accepts the reader, whose step runs before support.
        // Tightening happens only in finalize_plugin_option_defs(), which the
        // finalized runtime never runs again.
        ScopedActivePlugins active({BRIM_SEPARATION_READER});
        orchestrator.initialize_plugins();
        CHECK(orchestrator.is_plugin_active(BRIM_SEPARATION_READER));
    }
    CHECK(separation->invalidates_step == STEP_SUPPORT);
}

TEST_CASE("Default skirt generator owns its serialized option and activation rules",
          "[plugins][skirt-brim][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const ConfigOptionDef *one_skirt = PrintConfigDef::instance().get("complete_objects_one_skirt");
    REQUIRE(one_skirt != nullptr);

    CHECK(one_skirt->container_type == ConfigOptionContainerType::Plater);
    CHECK(one_skirt->option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    CHECK(one_skirt->invalidates_step == STEP_SKIRT_BRIM);
    CHECK(one_skirt->type == coBool);
    CHECK(one_skirt->default_value->serialize() == "0");
    CHECK(PrintConfig::defaults().option("complete_objects_one_skirt") != nullptr);

    /*
    The plugin owns one activation reason per accepted case, in the print scope:
    sequential printing or the parallel-objects stepping both need one skirt.
    */
    Orchestrator &orchestrator = Orchestrator::instance();
    size_t rule_count = 0;
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules()) {
        if (rule.owner != DEFAULT_SKIRT_GENERATOR || rule.target_key != "complete_objects_one_skirt")
            continue;
        ++rule_count;
        CHECK(rule.scope == RAW_PRESET_TYPE_FFF_PRINT);
    }
    CHECK(rule_count == 2);
}

TEST_CASE("Default skirt/brim plugins are active by default for plugin tests", "[plugins][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    std::vector<Plugin *> plugins = Orchestrator::instance().get_active_plugins_for_step(STEP_SKIRT_BRIM);
    bool found_default_brim = false;
    bool found_default_skirt = false;
    bool found_default_trim = false;
    for (Plugin *plugin : plugins)
        found_default_brim = found_default_brim || (plugin != nullptr && plugin->get_id() == DEFAULT_BRIM_GENERATOR);
    for (Plugin *plugin : plugins)
        found_default_skirt = found_default_skirt || (plugin != nullptr && plugin->get_id() == DEFAULT_SKIRT_GENERATOR);
    for (Plugin *plugin : plugins)
        found_default_trim = found_default_trim || (plugin != nullptr && plugin->get_id() == DEFAULT_BRIM_SKIRT_TRIM);
    CHECK(found_default_brim);
    CHECK(found_default_skirt);
    CHECK(found_default_trim);
}

TEST_CASE("Default brim generator leaves output empty when brim is disabled", "[plugins][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedBrimPrint prepared;
    run_until_skirt_brim(prepared, brim_test_config(false, false));

    CHECK(prepared.print.brim().empty());
    REQUIRE(prepared.print.objects().size() == 1);
    CHECK(prepared.print.objects().front().brim().empty());
    CHECK(object_brim_auxiliary_layers(prepared.print.objects().front()).empty());
}

TEST_CASE("Default brim generator creates global brim and expands the first-layer hull",
          "[plugins][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedBrimPrint prepared;
    run_until_skirt_brim(prepared, brim_test_config(true, false));

    CHECK_FALSE(prepared.print.brim().empty());
    REQUIRE(prepared.print.objects().size() == 1);
    CHECK(prepared.print.objects().front().brim().empty());

    /*
    The hull is rebuilt by the host after the plugin chain. If the callback
    output were not considered, the hull would stay near the object slice area.
    */
    CHECK(hull_area_mm2(prepared.print) > first_layer_slice_area_mm2(prepared.print));
}

TEST_CASE("Default brim generator applies its ear geometry options", "[plugins][skirt-brim]")
{
    DynamicPrintConfig concentric_config = brim_test_config(true, false);
    concentric_config.set_deserialize_strict({
        { "brim_ears", "1" },
        { "brim_ears_max_angle", "125" },
        { "brim_ears_detection_length", "1" },
        { "brim_ears_pattern", "concentric" }
    });
    PreparedBrimPrint concentric;
    run_until_skirt_brim(concentric, concentric_config);
    REQUIRE_FALSE(concentric.print.brim().empty());

    DynamicPrintConfig rectilinear_config = concentric_config;
    rectilinear_config.set_deserialize_strict({ { "brim_ears_pattern", "rectilinear" } });
    PreparedBrimPrint rectilinear;
    run_until_skirt_brim(rectilinear, rectilinear_config);
    REQUIRE_FALSE(rectilinear.print.brim().empty());
    CHECK(extrusion_tree_length_mm(rectilinear.print.brim()) !=
          Approx(extrusion_tree_length_mm(concentric.print.brim())));

    /*
    Zero is the documented off value for the angular detector. Reaching an
    empty output here proves that the plugin value reaches make_brim_ears()
    instead of falling back to the former static PrintObjectConfig member.
    */
    DynamicPrintConfig disabled_angle_config = concentric_config;
    disabled_angle_config.set_deserialize_strict({ { "brim_ears_max_angle", "0" } });
    PreparedBrimPrint disabled_angle;
    run_until_skirt_brim(disabled_angle, disabled_angle_config);
    CHECK(disabled_angle.print.brim().empty());
}

TEST_CASE("Default brim generator applies brim inside holes", "[plugins][skirt-brim]")
{
    DynamicPrintConfig outside_only_config = brim_test_config(true, false);
    outside_only_config.set_deserialize_strict({ { "brim_inside_holes", "0" } });
    PreparedBrimPrint outside_only;
    run_mesh_until_skirt_brim(
        outside_only,
        Slic3r::Test::mesh(Slic3r::Test::TestMesh::cube_with_hole, Vec3d::Zero(), Vec3d(1., 1., 0.02)),
        outside_only_config);
    REQUIRE_FALSE(outside_only.print.brim().empty());

    DynamicPrintConfig inside_holes_config = outside_only_config;
    inside_holes_config.set_deserialize_strict({ { "brim_inside_holes", "1" } });
    PreparedBrimPrint inside_holes;
    run_mesh_until_skirt_brim(
        inside_holes,
        Slic3r::Test::mesh(Slic3r::Test::TestMesh::cube_with_hole, Vec3d::Zero(), Vec3d(1., 1., 0.02)),
        inside_holes_config);
    REQUIRE_FALSE(inside_holes.print.brim().empty());

    /*
    Enabling the option preserves holes while the object outline is converted
    into the area from which brim loops are derived. The resulting tree must
    therefore contain additional centerline around the model's opening.
    */
    CHECK(extrusion_tree_length_mm(inside_holes.print.brim()) >
          extrusion_tree_length_mm(outside_only.print.brim()));
}

TEST_CASE("Default brim generator can publish object-owned brim", "[plugins][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedBrimPrint prepared;
    run_until_skirt_brim(prepared, brim_test_config(true, true));

    CHECK(prepared.print.brim().empty());
    REQUIRE(prepared.print.objects().size() == 1);
    PrintObject &object = prepared.print.objects().front();
    CHECK_FALSE(object.brim().empty());
    CHECK(hull_area_mm2(prepared.print) > first_layer_slice_area_mm2(prepared.print));

    const std::vector<const Layer *> brim_layers = object_brim_auxiliary_layers(object);
    REQUIRE(brim_layers.size() == 1);
    const Layer &brim_layer = *brim_layers.front();
    const LayerAdhesionProperty *adhesion = brim_layer.get_property<LayerAdhesionProperty>();
    REQUIRE(adhesion != nullptr);
    CHECK(adhesion->is_brim());
    CHECK(brim_layer.get_property<LayerSupportProperty>() == nullptr);
    CHECK_FALSE(brim_layer.lslices().empty());
    REQUIRE_FALSE(brim_layer.islands().empty());
    REQUIRE_FALSE(brim_layer.island(0).regions_islands().empty());
    const LayerRegionIsland &region_island = brim_layer.island(0).regions_island(0);
    REQUIRE(region_island.has_extrusion(LayerRegionIsland::PERIMETERS));
    CHECK_FALSE(region_island.extrusion(LayerRegionIsland::PERIMETERS).empty());

    Steps::StepSkirtBrim::clear(prepared.print);
    CHECK(object.brim().empty());
    CHECK(object_brim_auxiliary_layers(object).empty());
}

TEST_CASE("Default skirt generator creates global skirt", "[plugins][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedBrimPrint prepared;
    run_until_skirt_brim(prepared, skirt_test_config(1, 0, false));

    CHECK(prepared.print.brim().empty());
    CHECK_FALSE(prepared.print.skirt().empty());
    REQUIRE(prepared.print.objects().size() == 1);
    CHECK(prepared.print.objects().front().skirt().empty());
    CHECK(hull_area_mm2(prepared.print) > first_layer_slice_area_mm2(prepared.print));
}

TEST_CASE("Default skirt generator can publish object-owned skirt", "[plugins][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedBrimPrint prepared;
    run_until_skirt_brim(prepared, skirt_test_config(1, 0, true));

    CHECK(prepared.print.skirt().empty());
    REQUIRE(prepared.print.objects().size() == 1);
    CHECK_FALSE(prepared.print.objects().front().skirt().empty());
    CHECK(hull_area_mm2(prepared.print) > first_layer_slice_area_mm2(prepared.print));
}

TEST_CASE("Default skirt generator publishes skirt-brim as first-layer-only skirt", "[plugins][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedBrimPrint prepared;
    run_until_skirt_brim(prepared, skirt_test_config(0, 2, false));

    CHECK(prepared.print.skirt().empty());
    REQUIRE(prepared.print.skirt_first_layer().has_value());
    CHECK_FALSE(prepared.print.skirt_first_layer()->empty());
    CHECK(hull_area_mm2(prepared.print) > first_layer_slice_area_mm2(prepared.print));
}

TEST_CASE("Default brim/skirt trim setting controls draft-shield brim trimming", "[plugins][skirt-brim]")
{
    DynamicPrintConfig config = skirt_test_config(1, 0, false);
    config.set_deserialize_strict({
        { "brim_width", "5" },
        { "draft_shield", "limited" },
        { "skirt_distance", "1" },
        { "brim_skirt_trim", "0" }
    });

    PreparedBrimPrint untrimmed;
    run_until_skirt_brim(untrimmed, config, true);
    REQUIRE_FALSE(untrimmed.print.brim().empty());
    REQUIRE_FALSE(untrimmed.print.skirt().empty());

    config.set_deserialize_strict({
        { "brim_skirt_trim", "1" }
    });

    PreparedBrimPrint trimmed;
    run_until_skirt_brim(trimmed, config, true);
    REQUIRE_FALSE(trimmed.print.brim().empty());
    REQUIRE_FALSE(trimmed.print.skirt().empty());

    /*
    The legacy trim exists for this exact configuration: a draft shield is
    allowed to stand inside the configured brim width. The trim plugin keeps
    printable brim pieces, but removes the part occupied by the final skirt
    band, so total brim centerline length must decrease.
    */
    CHECK(extrusion_tree_length_mm(trimmed.print.brim()) <
          extrusion_tree_length_mm(untrimmed.print.brim()));
}
