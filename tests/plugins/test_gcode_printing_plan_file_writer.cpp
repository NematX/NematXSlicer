#include <catch2/catch.hpp>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>

#include "plugin_test_helpers.hpp"
#include "gcode_test_helpers.hpp"

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/GCodeScriptProcessor.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_seam_placer.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/DefaultGCodeFirmwareSession.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeFirmwareViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeScriptProcessorViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/MachineEnvelope.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config/ConfigSnapshotSerialization.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityVisitors.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Geometry/ArcWelder.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Plugins/GCode/Firmware/BuiltinGCodeFirmwares.hpp"
#include "libslic3r/Plugins/GCode/Firmware/KlipperGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/Marlin1GCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/Marlin2GCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/PrusaGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/RepRapGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/SprinterGCodeFirmware.hpp"
#include "libslic3r/Plugins/SeamPlacer/SeamCandidatePreparation.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepExtrusionEdition.hpp"
#include "libslic3r/Steps/StepGenerateGcode.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepReadGeneratedGcode.hpp"
#include "test_data.hpp"

/*
Use firmware probes to observe how the printing-plan file writer enters scopes
and consumes returned text. Small plans make print/group/layer/tool boundaries
explicit; failure cases begin with a known destination and check that incomplete
serialization cannot replace it. The writer is tested as a traverser and file
publisher, not as an independent geometry-to-command encoder.

PrintingPlan file-writer tests verify sequential scope traversal, firmware boundary calls and atomic publication.
Shared plan construction and output helpers live in gcode_test_helpers so this file keeps only component-specific behavior.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- Test::init_print()
        |-- print.process()
        |-- orchestrator.export_gcode()
        |-- read_text_file()
        |-- normalized_commands()
        `-- Steps::checkpoint_before_step()
*/

namespace {
using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::GCode;

static_assert(std::is_same_v<decltype(run_ctx_generate_gcode{}.plan),
                             const printing_plan_handle *>);
static_assert(std::is_constructible_v<slic3r_api::ConstPrintingPlan,
                                      const printing_plan_handle *>);
static_assert(!std::is_constructible_v<slic3r_api::PrintingPlan,
                                       const printing_plan_handle *>);

/* Carry the G-code and step results from one configured Cost export. */
struct CostSeamExportResult
{
    std::string gcode;
    bool ordering_completed = false;
    bool wipe_tower_completed = false;
};

/* Export one configured Cost/advanced-ordering print and return its results. */
CostSeamExportResult export_cost_seam_gcode(
    const std::vector<std::pair<std::string, std::string>> &case_settings,
    unsigned int extruder_count,
    const std::function<void(const Print &)> &before_export);

/* Count unmarked loops in the brim auxiliary layers of one processed object. */
size_t count_auxiliary_unmarked_loops(const PrintObject &object);

/* Count unmarked loops in the infill buckets of one processed object. */
size_t count_infill_unmarked_loops(const PrintObject &object);

/* Count unmarked closed loops below one extrusion node. */
size_t count_unmarked_closed_loops(const slic3r_api::ExtrusionEntity &entity);

/* Find a perimeter marker on one extrusion node or any of its descendants. */
bool subtree_has_perimeter_marker(const slic3r_api::ExtrusionEntity &entity);

struct FirmwareRecorder
{
    std::vector<std::string> calls;
    uint32_t destruction_count = 0;
    bool throw_on_extrusion = false;
    std::vector<bool> roots;
    std::vector<bool> region_islands;
    std::vector<raw_extrusion_role> roles;
    std::vector<uint32_t> printing_catalog_ids;
};

class RecordingFirmwareSession final : public slic3r_api::GCodeFirmwareSession
{
public:
    explicit RecordingFirmwareSession(FirmwareRecorder &recorder) : m_recorder(recorder) {}
    ~RecordingFirmwareSession() override { ++m_recorder.destruction_count; }

    std::string begin_print(const slic3r_api::Print &,
                            const slic3r_api::ConstPrintingPlan &plan) override
    {
        m_recorder.calls.emplace_back("begin_print");
        return std::string("begin_print\n") + (plan.events().has_before() ? write_event(plan.events().before()) : std::string());
    }

    std::string begin_group(const slic3r_api::ConstPrintingGroup &) override
    {
        m_recorder.calls.emplace_back("begin_group");
        return "begin_group\n";
    }

    std::string begin_layer(const slic3r_api::ConstPrintingLayerGroup &) override
    {
        m_recorder.calls.emplace_back("begin_layer");
        return "begin_layer\n";
    }

    std::string begin_tool_group(const slic3r_api::ConstPrintingToolGroup &) override
    {
        m_recorder.calls.emplace_back("begin_tool_group");
        return "begin_tool_group\n";
    }

    std::string write_extrusion(const slic3r_api::ConstPrintingExtrusion &extrusion) override
    {
        if (m_recorder.throw_on_extrusion)
            throw std::runtime_error("test firmware extrusion failure");

        m_recorder.calls.emplace_back("write_extrusion");
        m_recorder.roots.push_back(extrusion.root().valid());
        m_recorder.region_islands.push_back(extrusion.region_island().valid());
        m_recorder.roles.push_back(extrusion.role());
        m_recorder.printing_catalog_ids.push_back(extrusion.printing_catalog_id());
        return "write_extrusion\n";
    }

    std::string write_event(const slic3r_api::ExtrusionEntity &) override
    {
        m_recorder.calls.emplace_back("write_event");
        return "write_event\n";
    }

    std::string end_tool_group() override
    {
        m_recorder.calls.emplace_back("end_tool_group");
        return "end_tool_group\n";
    }

    std::string end_layer() override
    {
        m_recorder.calls.emplace_back("end_layer");
        return "end_layer\n";
    }

    std::string end_group() override
    {
        m_recorder.calls.emplace_back("end_group");
        return "end_group\n";
    }

    std::string end_print() override
    {
        m_recorder.calls.emplace_back("end_print");
        return "end_print\n";
    }

private:
    FirmwareRecorder &m_recorder;
};

/* Build printable integration geometry with explicit viewer marker attributes. */
std::unique_ptr<ExtrusionPath> make_integration_path(const ArcPolyline &polyline,
                                                     ExtrusionRole role,
                                                     float width,
                                                     float height)
{
    std::unique_ptr<ExtrusionPath> path(new ExtrusionPath(
        polyline,
        ExtrusionAttributes(role, ExtrusionFlow(0.08, width, height)),
        nullptr,
        true));
    path->add_property(ExtrusionPropertySpeed(20.f, 500.f));
    if (role == ExtrusionRole::ExternalPerimeter)
        path->get_or_add_property<slic3r_api::EPropertyPerimeter>().shell_count(0);
    return path;
}

/*
Set a registered plugin to the requested state for one test scope and restore its
previous state when the guard is destroyed, including exception unwinding. The
Orchestrator and Plugin are borrowed; missing plugins or failed activation fail
the Catch2 test, while a failed restoration is ignored. The guard is noncopyable.
*/
class ScopedPluginActivation
{
public:
    // Activate the registered plugin and remember its previous state.
    ScopedPluginActivation(Orchestrator &orchestrator, const char *plugin_id, bool active = true) :
        m_orchestrator(orchestrator), m_plugin(orchestrator.get_plugin(plugin_id))
    {
        REQUIRE(m_plugin != nullptr);
        m_was_active = orchestrator.is_plugin_active(m_plugin);
        REQUIRE(m_orchestrator.set_plugin_active(m_plugin, active));
    }

    // Restore the activation saved during construction.
    ~ScopedPluginActivation()
    {
        m_orchestrator.set_plugin_active(m_plugin, m_was_active);
    }

    ScopedPluginActivation(const ScopedPluginActivation &) = delete;
    ScopedPluginActivation &operator=(const ScopedPluginActivation &) = delete;

private:
    Orchestrator &m_orchestrator;
    Plugin *m_plugin;
    bool m_was_active = false;
};

/*
For a borrowed extrusion entity, check its own EXTRUSION_PROPERTY_TYPE_PERIMETER
property and those of its descendants without changing the tree.
*/
bool subtree_has_perimeter_marker(const slic3r_api::ExtrusionEntity &entity)
{
    if (entity.property_data(EXTRUSION_PROPERTY_TYPE_PERIMETER) != nullptr)
        return true;
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        if (subtree_has_perimeter_marker(entity.child(child_idx)))
            return true;
    return false;
}

/*
Given a borrowed entity, a provider binds a loop when the loop, an ancestor, or a
descendant carries EXTRUSION_PROPERTY_TYPE_PERIMETER. The caller has already
checked ancestors, so a marker stops descent; count a non-empty closed loop only
when its subtree has no marker.
*/
size_t count_unmarked_closed_loops(const slic3r_api::ExtrusionEntity &entity)
{
    if (entity.property_data(EXTRUSION_PROPERTY_TYPE_PERIMETER) != nullptr)
        return 0;
    if (entity.is_loop() && entity.segment_count() != 0 &&
        !subtree_has_perimeter_marker(entity))
        return 1;
    size_t count = 0;
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        count += count_unmarked_closed_loops(entity.child(child_idx));
    return count;
}

/*
For a borrowed processed object, pass brim auxiliary-layer PERIMETERS trees to
the shared counter without changing print state; layer-access failures propagate.
*/
size_t count_auxiliary_unmarked_loops(const PrintObject &object)
{
    size_t count = 0;
    for (const Layer &layer : object.auxiliary_layers()) {
        const LayerAdhesionProperty *adhesion = layer.get_property<LayerAdhesionProperty>();
        if (adhesion == nullptr || !adhesion->is_brim())
            continue;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region_island : island.regions_islands()) {
                if (!region_island.has_extrusion(LayerRegionIsland::PERIMETERS))
                    continue;
                count += count_unmarked_closed_loops(slic3r_api::ExtrusionEntity(
                    reinterpret_cast<const extrusion_entity_handle *>(
                        &region_island.extrusion(LayerRegionIsland::PERIMETERS))));
            }
    }
    return count;
}

/*
For a borrowed processed object, pass object-layer INFILLS trees to the shared
counter without changing print state; layer-access failures propagate.
*/
size_t count_infill_unmarked_loops(const PrintObject &object)
{
    size_t count = 0;
    for (const Layer &layer : object.layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region_island : island.regions_islands()) {
                if (!region_island.has_extrusion(LayerRegionIsland::INFILLS))
                    continue;
                count += count_unmarked_closed_loops(slic3r_api::ExtrusionEntity(
                    reinterpret_cast<const extrusion_entity_handle *>(
                        &region_island.extrusion(LayerRegionIsland::INFILLS))));
            }
    return count;
}

/*
Build and export a cube through Cost and advanced ordering; settings must suit
extruder_count and before_export must be callable. The callback receives the
helper's local Print after print.process() and before G-code export. Return owned G-code
and step results after destroying the local print/model, restoring plugin state,
and removing output files; processing, callback, or export failures propagate after cleanup.
*/
CostSeamExportResult export_cost_seam_gcode(
    const std::vector<std::pair<std::string, std::string>> &case_settings,
    unsigned int extruder_count,
    const std::function<void(const Print &)> &before_export)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    const std::string seam_selector = Steps::plugin_exclusive_group_option_key(
        orchestrator, SEAM_PLACER, "seam_placer_plugin");
    const std::string ordering_selector = Steps::plugin_exclusive_group_option_key(
        orchestrator, STEP_ORDERING, "ordering.extrusion_tree");
    REQUIRE_FALSE(seam_selector.empty());
    REQUIRE_FALSE(ordering_selector.empty());

    const ScopedPluginActivation cost(orchestrator, "seam_placer.cost");
    const ScopedPluginActivation advanced_ordering(orchestrator, "ordering.extrusion_tree.advanced");
    const ScopedPluginActivation default_ordering(orchestrator, "ordering.extrusion_tree.default", false);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(extruder_count);
    config.set_deserialize_strict({
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"},
        {"seam_visibility", "0"}});
    config.set_deserialize_strict(seam_selector, "seam_placer.cost");
    config.set_deserialize_strict(ordering_selector, "ordering.extrusion_tree.advanced");
    for (const std::pair<std::string, std::string> &setting : case_settings)
        config.set_deserialize_strict(setting.first, setting.second);

    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);

    CostSeamExportResult result;
    try {
        print.process();
        before_export(print);
        const std::string generated_path = orchestrator.export_gcode(
            print, output_path.string(), nullptr);
        REQUIRE(generated_path == output_path.string());
        result.ordering_completed = print.is_step_done(STEP_ORDERING);
        result.wipe_tower_completed = print.is_step_done(STEP_WIPETOWER);
        REQUIRE(boost::filesystem::exists(output_path));
        result.gcode = read_text_file(output_path);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
    return result;
}

} // namespace

TEST_CASE("STEP_GCODE is a normal step whose built-in writer is active by default", "[plugins][gcode][default-writer][step-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    Print print;

    const std::vector<Plugin *> gcode_plugins = orchestrator.get_active_plugins_for_step(STEP_GCODE);
    REQUIRE(gcode_plugins.size() == 1);
    CHECK(gcode_plugins.front()->get_id() == "gcode.printing_plan_file_writer");
    CHECK(orchestrator.get_plugin("gcode.legacy") == nullptr);

    const std::vector<Plugin *> selected = Steps::selected_or_active_plugins_for_step(
        orchestrator, STEP_GCODE, &print.full_print_config());
    REQUIRE(selected.size() == 1);
    CHECK(selected.front()->get_id() == "gcode.printing_plan_file_writer");

    // The step has no host-defined selector: no exclusive-step entry, no profile option.
    CHECK(Steps::get_exclusive_steps().find(STEP_GCODE) == Steps::get_exclusive_steps().end());
    CHECK(PrintConfigDef::instance().get("step_gcode_plugin") == nullptr);
    CHECK(PrintConfigDef::instance().option_keys(RAW_PRESET_TYPE_FFF_PRINTER).count(
              "step_gcode_plugin") == 0);

    // The firmware selector keeps its place before the flavor, and no layout
    // file mentions a writer selector.
    const std::string merged_printer_layout = orchestrator.merged_ui_layout(
        "printer_fff.ui",
        "page:General:printer\n"
        "group:Firmware\n"
        "\tsetting:gcode_flavor\n");
    const size_t flavor_pos = merged_printer_layout.find("setting:gcode_flavor");
    const size_t firmware_pos = merged_printer_layout.find("setting:gcode_firmware_plugin");
    REQUIRE(flavor_pos != std::string::npos);
    REQUIRE(firmware_pos != std::string::npos);
    CHECK(firmware_pos < flavor_pos);
    CHECK(merged_printer_layout.find("step_gcode_plugin") == std::string::npos);

    for (const char *layout_file : {"printer_fff.ui", "print.ui"})
        for (const Orchestrator::PluginUiFragment &fragment : orchestrator.ui_fragments_for_file(layout_file))
            CHECK(fragment.fragment_id != "step_gcode_plugin");
}
TEST_CASE("STEP_GCODE default writer rejects an invalid plan without publishing output", "[plugins][gcode][default-writer]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    Print print;
    print.mutable_printing_plan().groups.emplace_back().layers.emplace_back().tool_groups.emplace_back().extrusions.emplace_back();
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);

    CHECK_THROWS_AS(Steps::StepGenerateGcode::run_step(
                        orchestrator, print, output_path.string()), RuntimeError);
    orchestrator.reset_plugin_cancel();
    CHECK_FALSE(boost::filesystem::exists(output_path));
    CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
}
TEST_CASE("Firmware C++ adapter validates tables and destroys its session once",
          "[plugins][gcode][firmware]")
{
    FirmwareRecorder recorder;
    {
        FirmwareInstanceOwner owner(slic3r_api::make_gcode_firmware_instance(
            std::unique_ptr<slic3r_api::GCodeFirmwareSession>(new RecordingFirmwareSession(recorder))));
        CHECK_NOTHROW(slic3r_api::GCodeFirmwareView(&owner.instance()));

        raw_gcode_firmware_instance truncated_instance = owner.instance();
        truncated_instance.struct_size = 0;
        CHECK_THROWS_AS(slic3r_api::GCodeFirmwareView(&truncated_instance), std::invalid_argument);

        raw_gcode_firmware_vtable truncated = *owner.instance().vtable;
        truncated.struct_size = 0;
        raw_gcode_firmware_instance invalid_size = owner.instance();
        invalid_size.vtable = &truncated;
        CHECK_THROWS_AS(slic3r_api::GCodeFirmwareView(&invalid_size), std::invalid_argument);

        raw_gcode_firmware_vtable missing_callback = *owner.instance().vtable;
        missing_callback.write_event = nullptr;
        raw_gcode_firmware_instance invalid_callback = owner.instance();
        invalid_callback.vtable = &missing_callback;
        CHECK_THROWS_AS(slic3r_api::GCodeFirmwareView(&invalid_callback), std::invalid_argument);
    }
    CHECK(recorder.destruction_count == 1);
}
TEST_CASE("PrintingPlan file writer serializes populated scope events inside their boundaries",
          "[plugins][gcode][firmware][printing][plan][events]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    FirmwareRecorder recorder;
    FirmwareInstanceOwner firmware(slic3r_api::make_gcode_firmware_instance(
        std::unique_ptr<slic3r_api::GCodeFirmwareSession>(new RecordingFirmwareSession(recorder))));
    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    PrintingGroup &group = plan.groups.front();
    group.layers.emplace_back();
    PrintingLayerGroup &layer = group.layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    const LayerRegionIsland *fake_region =
        reinterpret_cast<const LayerRegionIsland *>(uintptr_t(0x3456));
    append_empty_extrusion(tool_group, ExtrusionRole::None, 0, fake_region);

    // One child makes each fixed event root observable without depending on a
    // specific event property. The recording firmware marks every callback.
    ExtrusionEntity event(true);
    plan.events.append_before(event);
    plan.events.append_after(event);
    group.events.append_before(event);
    group.events.append_after(event);
    layer.events.append_before(event);
    layer.events.append_after(event);
    tool_group.events.append_before(event);
    tool_group.events.append_after(event);

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        run_file_writer_with_firmware(print, output_path, firmware.instance());

        const std::vector<std::string> expected_calls{
            "begin_print", "write_event",
            "begin_group", "write_event",
            "begin_layer", "write_event",
            "begin_tool_group", "write_event",
            "write_extrusion",
            "write_event", "end_tool_group",
            "write_event", "end_layer",
            "write_event", "end_group",
            "write_event", "end_print"
        };
        CHECK(recorder.calls == expected_calls);
        CHECK(read_text_file(output_path) ==
              "begin_print\nwrite_event\n"
              "begin_group\nwrite_event\n"
              "begin_layer\nwrite_event\n"
              "begin_tool_group\nwrite_event\n"
              "write_extrusion\n"
              "write_event\nend_tool_group\n"
              "write_event\nend_layer\n"
              "write_event\nend_group\n"
              "write_event\nend_print\n");
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("PrintingPlan file writer serializes every firmware boundary in order",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    FirmwareRecorder recorder;
    FirmwareInstanceOwner firmware(slic3r_api::make_gcode_firmware_instance(
        std::unique_ptr<slic3r_api::GCodeFirmwareSession>(new RecordingFirmwareSession(recorder))));
    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    layer.tool_groups.front().extruder_id = 2;
    const LayerRegionIsland *fake_region =
        reinterpret_cast<const LayerRegionIsland *>(uintptr_t(0x2345));
    append_empty_extrusion(
        layer.tool_groups.front(), ExtrusionRole::Perimeter, 7, fake_region);
    append_empty_extrusion(
        layer.tool_groups.front(), ExtrusionRole::None, 0, fake_region);

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        run_file_writer_with_firmware(print, output_path, firmware.instance());

        const std::vector<std::string> expected_calls{
            "begin_print",
            "begin_group",
            "begin_layer",
            "begin_tool_group",
            "write_extrusion",
            "write_extrusion",
            "end_tool_group",
            "end_layer",
            "end_group",
            "end_print"
        };
        CHECK(recorder.calls == expected_calls);
        CHECK(read_text_file(output_path) ==
              "begin_print\n"
              "begin_group\n"
              "begin_layer\n"
              "begin_tool_group\n"
              "write_extrusion\n"
              "write_extrusion\n"
              "end_tool_group\n"
              "end_layer\n"
              "end_group\n"
              "end_print\n");
        REQUIRE(recorder.roots.size() == 2);
        CHECK(recorder.roots[0]);
        CHECK(recorder.roots[1]);
        REQUIRE(recorder.region_islands.size() == 2);
        CHECK(recorder.region_islands[0]);
        CHECK(recorder.region_islands[1]);
        REQUIRE(recorder.roles.size() == 2);
        CHECK(recorder.roles[0] == RAW_EXTRUSION_ROLE_PERIMETER);
        CHECK(recorder.roles[1] == RAW_EXTRUSION_ROLE_NONE);
        REQUIRE(recorder.printing_catalog_ids.size() == 2);
        CHECK(recorder.printing_catalog_ids[0] == 7);
        CHECK(recorder.printing_catalog_ids[1] == 0);
        CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Default firmware selects the tool of an otherwise empty PrintingPlan", "[plugins][gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    plan.groups.front().layers.front().tool_groups.emplace_back();
    plan.groups.front().layers.front().tool_groups.front().extruder_id = 0;
    append_empty_extrusion(plan.groups.front().layers.front().tool_groups.front());
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);

    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        REQUIRE(boost::filesystem::exists(output_path));
        CHECK(count_command_lines(read_text_file(output_path), "T0") == 1);
        CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Default firmware publishes a preamble for an empty PrintingPlan", "[plugins][gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    print.mutable_printing_plan();
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);

    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        REQUIRE(boost::filesystem::exists(output_path));
        CHECK(read_text_file(output_path) == "G21\nG90\nM82\n");
        CHECK(print.gcode_reader_source_path() == output_path.string());
        CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));

        Steps::StepGenerateGcode::clear(print);
        CHECK(print.gcode_reader_source_path().empty());
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Generated G-code is streamed into the viewer result",
          "[plugins][gcode][reader][viewer][writer]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();

    Print print;
    configure_standard_firmware(print);
    print.mutable_printing_plan();

    // Padding exceeds the host buffer so this exercises the reusable chunk
    // source as well as the final unterminated line handled by the Reader.
    std::string source;
    while (source.size() <= 70u * 1024u)
        source += "; padding\n";
    source += "T0\nG21\nG90\nM83\nG1 X0 Y0 Z0.2 F1200\nG1 X10 Y0 E1 F1200";

    const boost::filesystem::path source_path = temporary_gcode_path();
    remove_output_pair(source_path);
    write_text_file(source_path, source);
    print.set_gcode_reader_source_path(source_path.string());

    GCodeProcessorResult result;
    try {
        Steps::StepReadGeneratedGcode::run(
            orchestrator, print, "gcode.reader.marlin2", result);
        CHECK(result.filename == source_path.string());
        CHECK(result.reader_plugin_id == "gcode.reader.marlin2");

        bool has_extrusion = false;
        for (const GCodeProcessorResult::MoveVertex &move : result.moves)
            has_extrusion = has_extrusion || move.type == EMoveType::Extrude;
        CHECK(has_extrusion);
    } catch (...) {
        remove_output_pair(source_path);
        orchestrator.reset_plugin_cancel();
        throw;
    }
    remove_output_pair(source_path);
    orchestrator.reset_plugin_cancel();
}

TEST_CASE("PrintingPlan markers survive generated file Reader integration",
          "[plugins][gcode][reader][viewer][writer][integration][markers]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();

    Print print;
    configure_standard_firmware(print);
    select_printing_plan_firmware(print, "gcode.firmware.marlin2");
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();

    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &first_layer = plan.groups.front().layers.back();
    first_layer.print_z = scale_i(0.2);
    first_layer.tool_groups.emplace_back();
    PrintingToolGroup &first_tool = first_layer.tool_groups.back();
    first_tool.extruder_id = 0;
    append_path_extrusion(first_tool, make_integration_path(
        ArcPolyline(Points{
            Point::new_scale(0.0, 0.0),
            Point::new_scale(10.0, 0.0),
            Point::new_scale(10.0, 10.0),
            Point::new_scale(0.0, 0.0)
        }),
        ExtrusionRole::ExternalPerimeter, 0.45f, 0.2f));

    std::unique_ptr<ExtrusionPath> wipe = make_firmware_travel(
        ArcPolyline(Points{
            Point::new_scale(0.0, 0.0),
            Point::new_scale(1.0, 0.0)
        }),
        40.f, 800.f);
    slic3r_api::MutableExtrusionEntity wipe_view(
        reinterpret_cast<extrusion_entity_handle *>(wipe.get()));
    wipe_view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(
        RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE);
    append_path_extrusion(first_tool, std::move(wipe));

    append_path_extrusion(first_tool, make_integration_path(
        ArcPolyline(Points{
            Point::new_scale(1.0, 0.0),
            Point::new_scale(6.0, 0.0)
        }),
        ExtrusionRole::InternalInfill, 0.50f, 0.25f));

    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &second_layer = plan.groups.front().layers.back();
    second_layer.print_z = scale_i(0.4);
    second_layer.tool_groups.emplace_back();
    PrintingToolGroup &second_tool = second_layer.tool_groups.back();
    second_tool.extruder_id = 0;
    append_path_extrusion(second_tool, make_integration_path(
        ArcPolyline(Points{
            Point::new_scale(6.0, 0.0),
            Point::new_scale(12.0, 0.0),
            Point::new_scale(12.0, 6.0),
            Point::new_scale(6.0, 0.0)
        }),
        ExtrusionRole::ExternalPerimeter, 0.42f, 0.2f));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    GCodeProcessorResult result;
    try {
        Steps::StepGenerateGcode::clear(print);
        const std::string reader_plugin_id = Steps::StepGenerateGcode::run_step(
            orchestrator, print, output_path.string());
        REQUIRE(reader_plugin_id == "gcode.reader.marlin2");
        REQUIRE(print.gcode_reader_source_path() == output_path.string());

        const std::string generated = read_text_file(output_path);
        INFO(generated);
        CHECK(count_occurrences(generated, ";LAYER_CHANGE\n") == 2);
        CHECK(count_occurrences(generated, ";Z:") == 2);
        CHECK(count_occurrences(generated, ";HEIGHT:") == 3);
        CHECK(count_occurrences(generated, ";TYPE:") == 3);
        CHECK(count_occurrences(generated, ";WIDTH:") == 3);
        CHECK(count_occurrences(generated, ";WIPE_START\n") == 1);
        CHECK(count_occurrences(generated, ";WIPE_END\n") == 1);
        CHECK(count_occurrences(generated, ";SEAM:") == 2);

        Steps::StepReadGeneratedGcode::run(
            orchestrator, print, reader_plugin_id, result);
    } catch (...) {
        remove_output_pair(output_path);
        orchestrator.reset_plugin_cancel();
        throw;
    }
    remove_output_pair(output_path);
    orchestrator.reset_plugin_cancel();

    CHECK(result.filename == output_path.string());
    CHECK(result.reader_plugin_id == "gcode.reader.marlin2");

    const GCodeProcessorResult::MoveVertex *external_first = nullptr;
    const GCodeProcessorResult::MoveVertex *infill_first = nullptr;
    const GCodeProcessorResult::MoveVertex *external_second = nullptr;
    size_t visible_extrusions = 0;
    size_t wipes = 0;
    size_t seams = 0;
    for (const GCodeProcessorResult::MoveVertex &move : result.moves) {
        if (move.type == EMoveType::Wipe)
            ++wipes;
        else if (move.type == EMoveType::Seam)
            ++seams;
        if (move.type != EMoveType::Extrude || move.internal_only)
            continue;
        CHECK(std::isfinite(move.mm3_per_mm));
        CHECK(move.mm3_per_mm > 0.0f);
        ++visible_extrusions;
        if (move.layer_id == 0 &&
            move.extrusion_role == GCodeExtrusionRole::ExternalPerimeter)
            external_first = &move;
        else if (move.layer_id == 0 &&
                 move.extrusion_role == GCodeExtrusionRole::InternalInfill)
            infill_first = &move;
        else if (move.layer_id == 1 &&
                 move.extrusion_role == GCodeExtrusionRole::ExternalPerimeter)
            external_second = &move;
    }

    CHECK(visible_extrusions >= 7);
    CHECK(wipes == 1);
    CHECK(seams == 2);
    REQUIRE(external_first != nullptr);
    REQUIRE(infill_first != nullptr);
    REQUIRE(external_second != nullptr);
    CHECK(external_first->width == Approx(0.45f));
    CHECK(external_first->height == Approx(0.2f));
    CHECK(infill_first->width == Approx(0.50f));
    CHECK(infill_first->height == Approx(0.25f));
    CHECK(external_second->width == Approx(0.42f));
    CHECK(external_second->height == Approx(0.2f));

    const std::vector<float> &layer_times =
        result.print_statistics.modes[0].layers_times;
    REQUIRE(layer_times.size() >= 2);
    CHECK(layer_times[0] > 0.0f);
    CHECK(layer_times[1] > 0.0f);
}
TEST_CASE("PrintingPlan firmware executes host-owned scripts",
          "[plugins][gcode][firmware][custom-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    ExtrusionNop script(ExtrusionPropertyCustomGcodeText(
        ExtrusionPropertyCustomGcodeText::Code::SCRIPT,
        "M117 X{position[0]}"));
    plan.events.append_before(script);

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(
            Orchestrator::instance(), print, output_path.string());
        REQUIRE(boost::filesystem::exists(output_path));
        const std::string output = read_text_file(output_path);
        CHECK(output.find("M117 X0") != std::string::npos);
        CHECK(output.find("{position") == std::string::npos);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("PrintingPlan scripts cannot replace a previously published file",
          "[plugins][gcode][firmware][custom-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    ExtrusionNop script(ExtrusionPropertyCustomGcodeText(
        ExtrusionPropertyCustomGcodeText::Code::SCRIPT,
        "M117 {undefined_script_variable}"));
    plan.events.append_before(script);

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    write_text_file(output_path, "previous output\n");
    CHECK_THROWS_AS(
        Steps::StepGenerateGcode::run_step(
            Orchestrator::instance(), print, output_path.string()),
        RuntimeError);
    Orchestrator::instance().reset_plugin_cancel();
    REQUIRE(boost::filesystem::exists(output_path));
    CHECK(read_text_file(output_path) == "previous output\n");
    CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    remove_output_pair(output_path);
}
TEST_CASE("Firmware exceptions prevent final output publication", "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    FirmwareRecorder recorder;
    recorder.throw_on_extrusion = true;
    FirmwareInstanceOwner firmware(slic3r_api::make_gcode_firmware_instance(
        std::unique_ptr<slic3r_api::GCodeFirmwareSession>(new RecordingFirmwareSession(recorder))));
    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    plan.groups.front().layers.front().tool_groups.emplace_back();
    append_empty_extrusion(plan.groups.front().layers.front().tool_groups.front());

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    write_text_file(output_path, "previous output\n");
    CHECK_THROWS_AS(run_file_writer_with_firmware(print, output_path, firmware.instance()), RuntimeError);
    REQUIRE(boost::filesystem::exists(output_path));
    CHECK(read_text_file(output_path) == "previous output\n");
    CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    remove_output_pair(output_path);
}
TEST_CASE("STEP_GCODE fails when the output directory is unavailable", "[plugins][gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    Print print;
    select_printing_plan_firmware(print);
    print.mutable_printing_plan();
    const boost::filesystem::path missing_directory =
        boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("slic3r-missing-output-dir-%%%%-%%%%");
    const boost::filesystem::path output_path = missing_directory / "out.gcode";

    CHECK_THROWS_AS(Steps::StepGenerateGcode::run_step(orchestrator, print, output_path.string()),
                    RuntimeError);
    orchestrator.reset_plugin_cancel();
    CHECK_FALSE(boost::filesystem::exists(output_path));
    CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
}
TEST_CASE("Orchestrator export_gcode still routes through ordering and STEP_GCODE", "[plugins][gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("start_gcode")->value.clear();
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    for (slicing_step_t step : Steps::execution_order())
        print.mark_step_executed(step);

    GCodeProcessorResult result;
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    std::vector<PrintBase::SlicingStatus> statuses;
    print.set_status_callback([&statuses](const PrintBase::SlicingStatus &status) {
        statuses.push_back(status);
    });

    try {
        const std::string generated_path =
            Orchestrator::instance().export_gcode(print, output_path.string(), &result);
        CHECK(generated_path == output_path.string());
        CHECK(result.filename == output_path.string());
        CHECK(result.reader_plugin_id == "gcode.reader.marlin2");
        REQUIRE(boost::filesystem::exists(output_path));
        CHECK(read_text_file(output_path) == "G21\nG90\nM82\n");
        REQUIRE(print.printing_plan() != nullptr);
        CHECK(print.printing_plan()->groups.size() == 1);

        // Step labels are not boost::format templates. A leftover path argument
        // would make the GUI throw boost::io::too_many_args while showing this
        // otherwise successful export.
        bool saw_ordering_status = false;
        for (const PrintBase::SlicingStatus &status : statuses) {
            if (status.main_text != "Ordering extrusions")
                continue;
            saw_ordering_status = true;
            CHECK(status.args.empty());
        }
        CHECK(saw_ordering_status);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}

namespace {

/*
Behavior of the STEP_GCODE test modifier. It runs after the built-in writer
(priority 100 against 0), finds the published file and appends one movement,
then optionally reports an error or requests the cancellation of the plugins.
*/
enum class StepGcodeModifierMode { Append, AppendThenError, AppendThenCancel };

// Identity shared by the test plugins below: the C callbacks read it through the plugin context.
struct StepProbeState
{
    StepProbeState(const char *plugin_id, slicing_step_t plugin_step, int32_t plugin_priority) :
        id(plugin_id), step(plugin_step), priority(plugin_priority) {}

    const char *id;
    slicing_step_t step;
    int32_t priority;
};

// The STEP_GCODE modifier: appends a movement to the output and fails as its mode requests.
struct StepGcodeModifierState : StepProbeState
{
    StepGcodeModifierState() : StepProbeState("test.step_gcode.modifier", STEP_GCODE, 100) {}

    StepGcodeModifierMode mode = StepGcodeModifierMode::Append;
    std::atomic_uint32_t run_count{0};
    std::atomic_bool file_existed{false};
};

// A STEP_EXTRUSION_EDIT plugin that does nothing, or fails once when asked to.
struct ExtrusionEditProbeState : StepProbeState
{
    ExtrusionEditProbeState(const char *plugin_id, int32_t plugin_priority) :
        StepProbeState(plugin_id, STEP_EXTRUSION_EDIT, plugin_priority) {}

    std::atomic_bool fail_next{false};
    std::atomic_uint32_t run_count{0};
};

StepGcodeModifierState g_step_gcode_modifier;
ExtrusionEditProbeState g_extrusion_edit_probe_first("test.step_gcode.edit_probe.first", 1000);
ExtrusionEditProbeState g_extrusion_edit_probe_second("test.step_gcode.edit_probe.second", 1001);

// Text appended by the modifier: an absolute movement the Reader turns into a move.
const char *const k_step_gcode_modifier_line = "G1 X10 Y10 F1000\n";

using ProbeRunFn = void (*)(void *, const plugin_run_context *);

// Callbacks of the test plugins' C vtable. Each one answers from the StepProbeState behind the context.
const char *probe_id(void *ctx);
const char *probe_description(void *ctx);
const char *probe_empty_text(void *ctx);
slicing_step_t probe_step(void *ctx);
int32_t probe_priority(void *ctx);
const_strings_t probe_dependencies(void *ctx);
int32_t probe_used_keys(void *ctx, raw_used_config_key *keys);
int32_t probe_defined_keys(void *ctx, const char **keys);
int32_t probe_initialize(void *ctx, storage_handle *storage);
void probe_setup(void *ctx, const plugin_run_context *run_ctx, uint32_t run_count);
void probe_setup_run(void *ctx, const plugin_run_context *run_ctx);

// Append the movement to the file already published at output_path, then fail as the mode requests.
void step_gcode_modifier_run(void *ctx, const plugin_run_context *run_ctx);

// Count the run and report one error when fail_next was set, which it then clears.
void extrusion_edit_probe_run(void *ctx, const plugin_run_context *run_ctx);

// Build the vtable of a test plugin whose run callback is the given one.
plugin_vtable make_probe_vtable(ProbeRunFn run);

// Register a test plugin once per process; it stays inactive until a test activates it.
void register_probe_plugin(StepProbeState &state, const plugin_vtable *vtable);
void register_step_gcode_modifier();
void register_extrusion_edit_probes();

// Prepare a Print whose plan is empty and whose steps before STEP_GCODE are done, so
// export_gcode() only has to run STEP_GCODE and the Reader.
void prepare_step_gcode_print(Print &print);

// Run export_gcode() and return the error message of the exception it throws, or an empty text.
std::string export_gcode_error(Print &print, const boost::filesystem::path &path, GCodeProcessorResult &result);

const char *probe_id(void *ctx) { return static_cast<StepProbeState *>(ctx)->id; }
const char *probe_description(void *) { return "Test plugin of the STEP_GCODE tests."; }
const char *probe_empty_text(void *) { return ""; }
slicing_step_t probe_step(void *ctx) { return static_cast<StepProbeState *>(ctx)->step; }
int32_t probe_priority(void *ctx) { return static_cast<StepProbeState *>(ctx)->priority; }
const_strings_t probe_dependencies(void *) { return {}; }
int32_t probe_used_keys(void *, raw_used_config_key *) { return 0; }
int32_t probe_defined_keys(void *, const char **) { return 0; }
int32_t probe_initialize(void *, storage_handle *) { return PLUGIN_ERROR_OK; }
void probe_setup(void *, const plugin_run_context *, uint32_t) {}
void probe_setup_run(void *, const plugin_run_context *) {}

void step_gcode_modifier_run(void *ctx, const plugin_run_context *run_ctx)
{
    StepGcodeModifierState &state =
        *static_cast<StepGcodeModifierState *>(static_cast<StepProbeState *>(ctx));
    const run_ctx_generate_gcode *payload = plugin_ctx_as_generate_gcode(run_ctx);
    ++state.run_count;
    if (payload == nullptr || payload->output_path == nullptr)
        return;

    state.file_existed = boost::filesystem::exists(payload->output_path);
    {
        std::ofstream stream(payload->output_path, std::ios::out | std::ios::binary | std::ios::app);
        stream << k_step_gcode_modifier_line;
    }
    if (state.mode == StepGcodeModifierMode::AppendThenError)
        run_ctx->report_error(run_ctx->host_context, "test modifier failure");
    else if (state.mode == StepGcodeModifierMode::AppendThenCancel)
        Orchestrator::instance().request_plugin_cancel();
}

void extrusion_edit_probe_run(void *ctx, const plugin_run_context *run_ctx)
{
    ExtrusionEditProbeState &state =
        *static_cast<ExtrusionEditProbeState *>(static_cast<StepProbeState *>(ctx));
    ++state.run_count;
    if (state.fail_next.exchange(false))
        run_ctx->report_error(run_ctx->host_context, "test probe failure");
}

plugin_vtable make_probe_vtable(ProbeRunFn run)
{
    return plugin_vtable {
        sizeof(plugin_vtable),
        &probe_id,
        &probe_id,
        &probe_description,
        &probe_empty_text,
        &probe_step,
        &probe_dependencies,
        &probe_priority,
        &probe_used_keys,
        &probe_defined_keys,
        &probe_initialize,
        &probe_setup,
        &probe_setup_run,
        run
    };
}

void register_probe_plugin(StepProbeState &state, const plugin_vtable *vtable)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    if (orchestrator.get_plugin(state.id) != nullptr)
        return;
    plugin_instance instance = {&state, vtable};
    REQUIRE(orchestrator.register_plugin(instance));
    Plugin *plugin = orchestrator.get_plugin(state.id);
    REQUIRE(plugin != nullptr);
    REQUIRE(orchestrator.set_plugin_active(plugin, false));
}

void register_step_gcode_modifier()
{
    static const plugin_vtable table = make_probe_vtable(&step_gcode_modifier_run);
    register_probe_plugin(g_step_gcode_modifier, &table);
}

void register_extrusion_edit_probes()
{
    static const plugin_vtable table = make_probe_vtable(&extrusion_edit_probe_run);
    register_probe_plugin(g_extrusion_edit_probe_first, &table);
    register_probe_plugin(g_extrusion_edit_probe_second, &table);
}

void prepare_step_gcode_print(Print &print)
{
    select_printing_plan_firmware(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("start_gcode")->value.clear();
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    print.mutable_printing_plan();
    for (slicing_step_t step : Steps::execution_order())
        print.mark_step_executed(step);
}

std::string export_gcode_error(Print &print, const boost::filesystem::path &path, GCodeProcessorResult &result)
{
    try {
        Orchestrator::instance().export_gcode(print, path.string(), &result);
    } catch (const std::exception &error) {
        return error.what();
    }
    return std::string();
}

} // namespace

TEST_CASE("STEP_GCODE plugins run by priority and the Reader sees the modified file",
          "[plugins][gcode][step-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_step_gcode_modifier();
    Orchestrator &orchestrator = Orchestrator::instance();
    g_step_gcode_modifier.mode = StepGcodeModifierMode::Append;
    g_step_gcode_modifier.run_count = 0;
    g_step_gcode_modifier.file_existed = false;
    const ScopedPluginActivation modifier(orchestrator, g_step_gcode_modifier.id);

    Print print;
    prepare_step_gcode_print(print);
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    GCodeProcessorResult result;
    const Plugin *writer = orchestrator.get_plugin("gcode.printing_plan_file_writer");
    REQUIRE(writer != nullptr);
    const uint64_t writer_runs = writer->get_run_count();

    REQUIRE_NOTHROW(orchestrator.export_gcode(print, output_path.string(), &result));

    // The modifier found the writer's file and its movement follows the preamble.
    CHECK(g_step_gcode_modifier.run_count.load() == 1);
    CHECK(g_step_gcode_modifier.file_existed.load());
    CHECK(writer->get_run_count() == writer_runs + 1);
    CHECK(read_text_file(output_path) == "G21\nG90\nM82\nG1 X10 Y10 F1000\n");
    // The Reader runs after the modifier: its last move is the appended one.
    REQUIRE_FALSE(result.moves.empty());
    CHECK(result.moves.back().position.x() == Approx(10.f));
    CHECK(result.moves.back().position.y() == Approx(10.f));
    CHECK(print.is_step_done(STEP_GCODE));
    CHECK(print.current_checkpoint() == Steps::Checkpoint::GCode);
    remove_output_pair(output_path);
}

TEST_CASE("A failing STEP_GCODE plugin leaves the step undone and the next attempt starts clean",
          "[plugins][gcode][step-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_step_gcode_modifier();
    Orchestrator &orchestrator = Orchestrator::instance();
    const ScopedPluginActivation modifier(orchestrator, g_step_gcode_modifier.id);
    const Plugin *writer = orchestrator.get_plugin("gcode.printing_plan_file_writer");
    REQUIRE(writer != nullptr);

    Print print;
    prepare_step_gcode_print(print);
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    GCodeProcessorResult result;

    struct Scenario
    {
        StepGcodeModifierMode mode;
        const char *expected_message;
    };
    for (const Scenario &scenario : {Scenario{StepGcodeModifierMode::AppendThenError, "reported an error"},
                                     Scenario{StepGcodeModifierMode::AppendThenCancel, "after a stop was requested"}}) {
        g_step_gcode_modifier.mode = scenario.mode;
        g_step_gcode_modifier.run_count = 0;
        const uint64_t writer_runs = writer->get_run_count();

        // The modifier already appended its movement when it failed: the step fails and names it.
        const std::string message = export_gcode_error(print, output_path, result);
        INFO(message);
        CHECK(message.find("test.step_gcode.modifier") != std::string::npos);
        CHECK(message.find(scenario.expected_message) != std::string::npos);
        CHECK_FALSE(print.is_step_done(STEP_GCODE));
        CHECK(print.current_checkpoint() != Steps::Checkpoint::GCode);
        CHECK(g_step_gcode_modifier.run_count.load() == 1);
        CHECK(writer->get_run_count() == writer_runs + 1);
        // The failed attempt left the writer's file with the modifier's movement already appended.
        CHECK(read_text_file(output_path) == "G21\nG90\nM82\nG1 X10 Y10 F1000\n");

        // The next attempt runs both plugins again on the same Print, without slicing again.
        g_step_gcode_modifier.mode = StepGcodeModifierMode::Append;
        REQUIRE_NOTHROW(orchestrator.export_gcode(print, output_path.string(), &result));
        CHECK(g_step_gcode_modifier.run_count.load() == 2);
        CHECK(writer->get_run_count() == writer_runs + 2);
        // Same bytes as after the failed attempt: the writer replaced that file, so the
        // movement appears once and not twice.
        CHECK(read_text_file(output_path) == "G21\nG90\nM82\nG1 X10 Y10 F1000\n");
        CHECK(print.is_step_done(STEP_GCODE));
        CHECK(print.current_checkpoint() == Steps::Checkpoint::GCode);
    }
    remove_output_pair(output_path);
}

TEST_CASE("STEP_GCODE marks each plugin completed in order and leaves the Reader unit pending",
          "[plugins][gcode][step-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_step_gcode_modifier();
    Orchestrator &orchestrator = Orchestrator::instance();
    g_step_gcode_modifier.mode = StepGcodeModifierMode::Append;
    const ScopedPluginActivation modifier(orchestrator, g_step_gcode_modifier.id);

    Print print;
    print.status().set_silent();
    prepare_step_gcode_print(print);
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);

    const std::map<slicing_step_t, std::vector<std::string>> plan {
        { STEP_GCODE, { "gcode.printing_plan_file_writer", g_step_gcode_modifier.id,
                        Steps::k_gcode_read_progress_unit } }
    };
    print.pipeline_progress().initialize(plan);
    print.pipeline_progress().begin_step(STEP_GCODE, "Generating G-code");
    orchestrator.reset_plugin_cancel();

    // The runner alone, without the Reader pass: both plugin units are completed in plan order
    // (complete_plugin throws otherwise), and the Reader unit is still pending.
    REQUIRE_NOTHROW(Steps::StepGenerateGcode::run_step(orchestrator, print, output_path.string()));
    CHECK(print.pipeline_progress().percent() == 66);
    CHECK_THROWS_AS(print.pipeline_progress().verify_step_complete(STEP_GCODE), std::logic_error);
    REQUIRE_NOTHROW(print.pipeline_progress().complete_plugin(STEP_GCODE, Steps::k_gcode_read_progress_unit));
    CHECK_NOTHROW(print.pipeline_progress().verify_step_complete(STEP_GCODE));
    CHECK(print.pipeline_progress().percent() == 100);
    remove_output_pair(output_path);
    orchestrator.reset_plugin_cancel();
}

TEST_CASE("The Reader pass is announced and counted as the last progress unit of STEP_GCODE",
          "[plugins][gcode][step-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_step_gcode_modifier();
    Orchestrator &orchestrator = Orchestrator::instance();
    g_step_gcode_modifier.mode = StepGcodeModifierMode::Append;
    const ScopedPluginActivation modifier(orchestrator, g_step_gcode_modifier.id);

    for (const bool with_result : {true, false}) {
        INFO("viewer result requested: " << with_result);
        Print print;
        prepare_step_gcode_print(print);
        std::vector<PrintBase::SlicingStatus> statuses;
        print.set_status_callback([&statuses](const PrintBase::SlicingStatus &status) {
            statuses.push_back(status);
        });
        const boost::filesystem::path output_path = temporary_gcode_path();
        remove_output_pair(output_path);
        GCodeProcessorResult result;

        REQUIRE_NOTHROW(orchestrator.export_gcode(print, output_path.string(), with_result ? &result : nullptr));

        // Three units (writer, modifier, Reader): the message appears after two of them.
        int reading_percent = -1;
        for (const PrintBase::SlicingStatus &status : statuses)
            if (status.main_text == "Reading the generated G-code")
                reading_percent = status.percent;
        if (with_result)
            CHECK(reading_percent == 66);
        else
            CHECK(reading_percent == -1);
        CHECK_NOTHROW(print.pipeline_progress().verify_step_complete(STEP_GCODE));
        CHECK(print.pipeline_progress().percent() == 100);
        remove_output_pair(output_path);
    }
}

TEST_CASE("A cancellation at the start of the Reader pass leaves a progress table the next export can use",
          "[plugins][gcode][step-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_step_gcode_modifier();
    Orchestrator &orchestrator = Orchestrator::instance();
    g_step_gcode_modifier.mode = StepGcodeModifierMode::Append;
    const ScopedPluginActivation modifier(orchestrator, g_step_gcode_modifier.id);

    Print print;
    prepare_step_gcode_print(print);
    // Cancel the Print as the Reader pass is announced, and raise the cancellation at that start:
    // the plugin units are completed and the Reader unit is not, so only the pipeline can
    // clean the progress table. (The Reader service never throws CanceledException itself.)
    print.set_status_callback([&print](const PrintBase::SlicingStatus &status) {
        if (status.main_text != "Reading the generated G-code")
            return;
        print.cancel();
        throw CanceledException();
    });
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    GCodeProcessorResult result;

    CHECK_THROWS_AS(orchestrator.export_gcode(print, output_path.string(), &result), CanceledException);
    CHECK_FALSE(print.is_step_done(STEP_GCODE));

    // Same Print, no new slicing: the next export starts from a fresh progress table.
    print.set_status_callback(nullptr);
    print.restart();
    REQUIRE_NOTHROW(orchestrator.export_gcode(print, output_path.string(), &result));
    CHECK(print.is_step_done(STEP_GCODE));
    CHECK(read_text_file(output_path) == "G21\nG90\nM82\nG1 X10 Y10 F1000\n");
    CHECK(print.pipeline_progress().percent() == 100);
    remove_output_pair(output_path);
}

TEST_CASE("A failing second plugin of a normal step leaves a progress table the next run can use",
          "[plugins][gcode][step-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_extrusion_edit_probes();
    Orchestrator &orchestrator = Orchestrator::instance();
    const ScopedPluginActivation first(orchestrator, g_extrusion_edit_probe_first.id);
    const ScopedPluginActivation second(orchestrator, g_extrusion_edit_probe_second.id);
    g_extrusion_edit_probe_first.run_count = 0;
    g_extrusion_edit_probe_second.run_count = 0;

    Print print;
    prepare_step_gcode_print(print);
    REQUIRE(print.save_ordering_checkpoint());
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);

    // Replay from the Ordering copy: STEP_EXTRUSION_EDIT runs its two probes, then STEP_GCODE.
    g_extrusion_edit_probe_second.fail_next = true;
    print.request_replay_from_worker(Steps::Checkpoint::Ordering);
    std::string message;
    try {
        Steps::StepPipeline::run_gcode(orchestrator, print, output_path.string());
    } catch (const std::exception &error) {
        message = error.what();
    }
    INFO(message);
    CHECK(message.find(g_extrusion_edit_probe_second.id) != std::string::npos);
    CHECK(g_extrusion_edit_probe_first.run_count.load() == 1);
    CHECK(g_extrusion_edit_probe_second.run_count.load() == 1);
    CHECK_FALSE(print.is_step_done(STEP_EXTRUSION_EDIT));

    // Same Print: the second attempt replays the step and succeeds.
    print.request_replay_from_worker(Steps::Checkpoint::Ordering);
    REQUIRE_NOTHROW(Steps::StepPipeline::run_gcode(orchestrator, print, output_path.string()));
    CHECK(g_extrusion_edit_probe_first.run_count.load() == 2);
    CHECK(g_extrusion_edit_probe_second.run_count.load() == 2);
    CHECK(print.is_step_done(STEP_EXTRUSION_EDIT));
    CHECK(print.is_step_done(STEP_GCODE));
    remove_output_pair(output_path);
}

// Export an unflattened object-owned brim through the real Cost ordering and
// file writer, retaining marked auxiliary loops as distance-only seam targets.
// The Print and Model are locally owned, plugin activations restore on scope exit,
// and output files are removed on success or exception before Catch2 failures propagate.
TEST_CASE("PrintingPlan writer exports a cube with composed brim and cost seams",
          "[plugins][gcode][seam-placer][cost][integration][seambrim-export]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    const ScopedPluginActivation cost(orchestrator, "seam_placer.cost");
    const ScopedPluginActivation advanced_ordering(orchestrator, "ordering.extrusion_tree.advanced");
    const ScopedPluginActivation default_ordering(orchestrator, "ordering.extrusion_tree.default", false);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"},
        {"brim_width", "3.2"}, {"brim_per_object", "1"},
        {"draft_shield", "disabled"}, {"brim_skirt_trim", "0"},
        {"brim_ears", "0"}, {"seam_visibility", "0"}, {"skirts", "0"}});
    const std::string seam_selector = Steps::plugin_exclusive_group_option_key(
        orchestrator, SEAM_PLACER, "seam_placer_plugin");
    const std::string ordering_selector = Steps::plugin_exclusive_group_option_key(
        orchestrator, STEP_ORDERING, "ordering.extrusion_tree");
    REQUIRE_FALSE(seam_selector.empty());
    REQUIRE_FALSE(ordering_selector.empty());
    config.set_deserialize_strict(seam_selector, "seam_placer.cost");
    config.set_deserialize_strict(ordering_selector, "ordering.extrusion_tree.advanced");
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        print.process();
        size_t marked_loops = 0;
        const PrintObject &object = print.object(0);
        for (const Layer &layer : object.auxiliary_layers()) {
            const LayerAdhesionProperty *adhesion = layer.get_property<LayerAdhesionProperty>();
            if (adhesion == nullptr || !adhesion->is_brim()) continue;
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region_island : island.regions_islands()) {
                    if (!region_island.has_extrusion(LayerRegionIsland::PERIMETERS)) continue;
                    const slic3r_api::ExtrusionEntity root(reinterpret_cast<const extrusion_entity_handle *>(
                        &region_island.extrusion(LayerRegionIsland::PERIMETERS)));
                    const std::vector<slic3r_api::SeamPlacement::CandidatePreparation::CollectedLoop>
                        loops = slic3r_api::SeamPlacement::CandidatePreparation::collect_perimeter_loops(
                            root, RAW_EXTRUSION_ROLE_PERIMETER);
                    for (const slic3r_api::SeamPlacement::CandidatePreparation::CollectedLoop &loop : loops) {
                        const slic3r_api::ExtrusionEntity entity(loop.loop);
                        const c_extrusion_property_perimeter *perimeter = static_cast<const c_extrusion_property_perimeter *>(
                            entity.property_data(EXTRUSION_PROPERTY_TYPE_PERIMETER));
                        REQUIRE(perimeter != nullptr);
                        CHECK((perimeter->perimeter_flags & C_EXTRUSION_PERIMETER_FLAG_SKIRT) != 0);
                        CHECK(entity.segment_count() == 0);
                        REQUIRE(entity.child_count() == 1);
                        CHECK(entity.child(0).segment_count() > 0);
                        CHECK(loop.distance_only);
                        CHECK_FALSE(loop.has_external_geometry);
                        ++marked_loops;
                    }
                }
        }
        REQUIRE(marked_loops > 1);
        const std::string generated_path = orchestrator.export_gcode(
            print, output_path.string(), nullptr);
        CHECK(generated_path == output_path.string());
        REQUIRE(boost::filesystem::exists(output_path));
        const std::string gcode = read_text_file(output_path);
        CHECK(gcode.find("G1 ") != std::string::npos);
        CHECK(gcode.find(";TYPE:Skirt") != std::string::npos);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}

/*
Export a cube through Cost and advanced ordering, then verify the generated
extrusion and replay behavior while scoped plugin activations restore prior state.
*/
TEST_CASE("PrintingPlan writer exports a cube with cost seam placement",
          "[plugins][gcode][seam-placer][cost][checkpoint][integration]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    // The selectors below can only choose a plugin from the active set: the test
    // runtime activates ordering.extrusion_tree.default, and the regional seam
    // providers are deactivated after their registration.
    const ScopedPluginActivation cost(orchestrator, "seam_placer.cost");
    const ScopedPluginActivation advanced_ordering(orchestrator, "ordering.extrusion_tree.advanced");
    const ScopedPluginActivation default_ordering(orchestrator, "ordering.extrusion_tree.default", false);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("gcode_firmware_plugin", "gcode.firmware.marlin2");
    // The Cost provider measures every candidate's visibility when seam_visibility
    // is enabled, which does not finish in test-scale time on this print, so the
    // export replays below keep that measurement disabled.
    config.set_deserialize_strict("seam_visibility", "0");
    const std::string seam_selector =
        Steps::plugin_exclusive_group_option_key(orchestrator, SEAM_PLACER, "seam_placer_plugin");
    REQUIRE_FALSE(seam_selector.empty());
    config.set_deserialize_strict(seam_selector, "seam_placer.cost");
    const std::string ordering_selector =
        Steps::plugin_exclusive_group_option_key(orchestrator, STEP_ORDERING, "ordering.extrusion_tree");
    REQUIRE_FALSE(ordering_selector.empty());
    config.set_deserialize_strict(ordering_selector, "ordering.extrusion_tree.advanced");

    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    // Count the steps that really ran, to check which checkpoint each replay
    // resumed from.
    uint32_t slicing_count = 0;
    uint32_t finalize_slicing_count = 0;
    uint32_t ordering_count = 0;
    uint32_t ui_cancel_count = 0;
    bool cancel_after_ordering_restore = false;
    const Printing::PrintingPlan *plan_at_cancellation = nullptr;
    print.status().set_callback([&](const PrintBase::SlicingStatus &status) {
        if ((status.flags & PrintBase::SlicingStatus::MAIN_STATE) == 0)
            return;
        if (status.main_text == "StepSlicing")
            ++slicing_count;
        if (status.main_text == "Finalizing slicing")
            ++finalize_slicing_count;
        if (status.main_text == "Ordering extrusions")
            ++ordering_count;
        if (cancel_after_ordering_restore &&
            print.running_step() == Steps::checkpoint_resume_step(Steps::Checkpoint::Ordering)) {
            cancel_after_ordering_restore = false;
            plan_at_cancellation = print.printing_plan();
            print.cancel();
        }
    });

    // Ignore comments, not motion/extrusion values or command order.
    const auto normalized_commands = [](const std::string &gcode) {
        std::istringstream stream(gcode);
        std::vector<std::string> commands;
        std::string line;
        while (std::getline(stream, line)) {
            line = line.substr(0, line.find(';'));
            const size_t first = line.find_first_not_of(" \t\r");
            if (first != std::string::npos)
                commands.push_back(line.substr(first, line.find_last_not_of(" \t\r") - first + 1));
        }
        return commands;
    };

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        print.process();
        CHECK_FALSE(print.is_step_done(STEP_WIPETOWER));
        CHECK(print.should_execute_step(STEP_WIPETOWER));
        CHECK_FALSE(print.tool_orderings().empty());
        const std::string generated_path =
            orchestrator.export_gcode(print, output_path.string(), nullptr);
        CHECK(generated_path == output_path.string());
        CHECK(print.is_step_done(STEP_WIPETOWER));
        REQUIRE(boost::filesystem::exists(output_path));
        CHECK(read_text_file(output_path).find("G1") != std::string::npos);
        const std::vector<std::string> fresh_commands = normalized_commands(read_text_file(output_path));
        PrintStateBase::TimeStamp previous_pipeline_timestamp = 0;
        for (const slicing_step_t step : Steps::execution_order()) {
            const PrintStateBase::StateWithWarnings state = print.step_state_with_warnings(step);
            CAPTURE(step);
            CHECK(state.is_done());
            CHECK(state.enabled);
            CHECK(state.timestamp > previous_pipeline_timestamp);
            previous_pipeline_timestamp = state.timestamp;
        }
        bool has_extruding_motion = false;
        for (const std::string &command : fresh_commands) {
            if (command.find("G1 ") == 0 && command.find(" E") != std::string::npos &&
                (command.find(" X") != std::string::npos || command.find(" Y") != std::string::npos))
                has_extruding_motion = true;
        }
        REQUIRE(has_extruding_motion);
        CHECK(slicing_count == 1);
        CHECK(finalize_slicing_count == 1);
        CHECK(ordering_count == 1);
        remove_output_pair(output_path);

        // The read-only G-code stage reuses the final plan directly.
        const Printing::PrintingPlan *final_plan = print.printing_plan();
        REQUIRE(final_plan != nullptr);
        REQUIRE(print.is_step_done(STEP_GCODE));
        const size_t first_export_timestamp = print.step_state_with_timestamp(STEP_GCODE).timestamp;
        print.set_cancel_callback([&]() { ++ui_cancel_count; });
        const std::string replayed_gcode_path =
            orchestrator.export_gcode(print, output_path.string(), nullptr);
        print.set_cancel_callback([]() {});
        CHECK(ui_cancel_count == 0);
        CHECK(replayed_gcode_path == output_path.string());
        CHECK(print.printing_plan() == final_plan);
        CHECK(read_text_file(output_path).find("G1") != std::string::npos);
        CHECK(normalized_commands(read_text_file(output_path)) == fresh_commands);
        CHECK(print.is_step_done(STEP_GCODE));
        CHECK(print.step_state_with_timestamp(STEP_GCODE).timestamp > first_export_timestamp);
        CHECK_FALSE(print.should_execute_step(STEP_GCODE));
        CHECK(slicing_count == 1);
        CHECK(finalize_slicing_count == 1);
        CHECK(ordering_count == 1);
        remove_output_pair(output_path);

        // A plan-editor change resumes from the ordering checkpoint. Background
        // slicing stops before plan replay, which belongs to export.
        print.request_replay_from_worker(Steps::checkpoint_before_step(STEP_EXTRUSION_EDIT));
        const Printing::PrintingPlan *plan_before_editor_replay = print.printing_plan();
        print.process();
        CHECK(print.printing_plan() == plan_before_editor_replay);
        // Mark the stale plan: its address may be reused by the restored clone.
        const size_t ordered_group_count = print.printing_plan()->groups.size();
        print.mutable_printing_plan().groups.emplace_back();
        const std::string replayed_editor_path =
            orchestrator.export_gcode(print, output_path.string(), nullptr);
        CHECK(replayed_editor_path == output_path.string());
        REQUIRE(print.printing_plan() != nullptr);
        CHECK(print.printing_plan()->groups.size() == ordered_group_count);
        CHECK(normalized_commands(read_text_file(output_path)) == fresh_commands);
        CHECK(finalize_slicing_count == 1);
        CHECK(ordering_count == 1);
        CHECK(read_text_file(output_path).find("G1") != std::string::npos);
        remove_output_pair(output_path);

        SECTION("Cancellation after Ordering restoration leaves plan replay retryable") {
            REQUIRE(print.has_ordering_checkpoint());
            REQUIRE(print.is_step_done(STEP_GCODE));
            const size_t editor_export_timestamp = print.step_state_with_timestamp(STEP_GCODE).timestamp;
            const Printing::PrintingPlan *plan_before_cancelled_replay = print.printing_plan();
            REQUIRE(plan_before_cancelled_replay != nullptr);
            const size_t checkpoint_group_count = print.printing_plan()->groups.size();
            print.mutable_printing_plan().groups.emplace_back();
            print.request_replay_from_worker(Steps::checkpoint_before_step(STEP_EXTRUSION_EDIT));
            REQUIRE(print.current_checkpoint() == Steps::Checkpoint::Ordering);
            CHECK(print.printing_plan() == plan_before_cancelled_replay);
            cancel_after_ordering_restore = true;
            print.set_cancel_callback([&]() { ++ui_cancel_count; });
            CHECK_THROWS_AS(orchestrator.export_gcode(print, output_path.string(), nullptr),
                            CanceledException);
            print.set_cancel_callback([]() {});
            CHECK(ui_cancel_count == 0);
            CHECK_FALSE(cancel_after_ordering_restore);
            REQUIRE(plan_at_cancellation != nullptr);
            CHECK(plan_at_cancellation->groups.size() == checkpoint_group_count);
            CHECK(print.printing_plan() == plan_at_cancellation);
            CHECK(print.canceled());
            CHECK(print.running_step() == STEP_NONE);
            CHECK(print.has_ordering_checkpoint());
            CHECK(print.current_checkpoint() == Steps::Checkpoint::Ordering);
            CHECK_FALSE(print.is_step_done(STEP_GCODE));
            for (const slicing_step_t step : Steps::execution_order()) {
                CAPTURE(step);
                CHECK(print.should_execute_step(step) ==
                      !Steps::executes_before(step, Steps::checkpoint_resume_step(Steps::Checkpoint::Ordering)));
            }
            CHECK(slicing_count == 1);
            CHECK(finalize_slicing_count == 1);
            CHECK(ordering_count == 1);
            CHECK_FALSE(boost::filesystem::exists(output_path));
            CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));

            print.mutable_printing_plan().groups.emplace_back();
            print.restart();
            print.set_cancel_callback([&]() { ++ui_cancel_count; });
            const std::string resumed_editor_path =
                orchestrator.export_gcode(print, output_path.string(), nullptr);
            print.set_cancel_callback([]() {});
            CHECK(ui_cancel_count == 0);
            CHECK(resumed_editor_path == output_path.string());
            REQUIRE(boost::filesystem::exists(output_path));
            CHECK(normalized_commands(read_text_file(output_path)) == fresh_commands);
            REQUIRE(print.printing_plan() != nullptr);
            CHECK(print.printing_plan()->groups.size() == checkpoint_group_count);
            CHECK(print.has_ordering_checkpoint());
            CHECK(print.current_checkpoint() == Steps::Checkpoint::GCode);
            CHECK(print.is_step_done(STEP_GCODE));
            CHECK(print.step_state_with_timestamp(STEP_GCODE).timestamp > editor_export_timestamp);
            for (const slicing_step_t step : Steps::execution_order()) {
                CAPTURE(step);
                CHECK_FALSE(print.should_execute_step(step));
            }
            CHECK(slicing_count == 1);
            CHECK(finalize_slicing_count == 1);
            CHECK(ordering_count == 1);
            CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
            remove_output_pair(output_path);
        }

        // A selector change can invalidate STEP_ORDERING after the downstream
        // layer extrusion edits were already marked done. Rebuilding the plan
        // must force those edits again so STEP_GCODE never sees speedless roots.
        print.request_replay_from_worker(Steps::checkpoint_before_step(STEP_ORDERING));
        const std::string regenerated_path =
            orchestrator.export_gcode(print, output_path.string(), nullptr);
        CHECK(regenerated_path == output_path.string());
        REQUIRE(boost::filesystem::exists(output_path));
        CHECK(read_text_file(output_path).find("G1") != std::string::npos);
        CHECK(finalize_slicing_count == 1);
        CHECK(ordering_count == 2);

        // Without the ordered checkpoint, editor replay rebuilds Ordering from
        // the retained extrusion trees and then reruns all later steps.
        print.set_keep_ordering_checkpoint(false);
        print.request_replay_from_worker(Steps::checkpoint_before_step(STEP_EXTRUSION_EDIT));
        const std::string replayed_ordering_path =
            orchestrator.export_gcode(print, output_path.string(), nullptr);
        CHECK(replayed_ordering_path == output_path.string());
        CHECK(read_text_file(output_path).find("G1") != std::string::npos);
        CHECK(finalize_slicing_count == 1);
        CHECK(ordering_count == 3);

        // Repeated cleanup must neither retain old extrusion edits nor erase the retained slices.
        for (uint32_t replay = 0; replay < 2; ++replay) {
            CAPTURE(replay);
            print.request_replay_from_worker(Steps::Checkpoint::Slicing);
            REQUIRE(print.current_checkpoint() == Steps::Checkpoint::Slicing);
            if (replay == 0)
                print.process();
            remove_output_pair(output_path);
            CHECK(orchestrator.export_gcode(print, output_path.string(), nullptr) ==
                  output_path.string());
            REQUIRE(boost::filesystem::exists(output_path));
            CHECK(normalized_commands(read_text_file(output_path)) == fresh_commands);
            CHECK(slicing_count == 1);
            CHECK(finalize_slicing_count == 2 + replay);
            CHECK(ordering_count == 4 + replay);
            CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
        }
    } catch (...) {
        print.set_cancel_callback([]() {});
        print.status().set_callback([](const PrintBase::SlicingStatus &) {});
        print.restart();
        orchestrator.reset_plugin_cancel();
        remove_output_pair(output_path);
        throw;
    }
    print.status().set_callback([](const PrintBase::SlicingStatus &) {});
    remove_output_pair(output_path);
}

/*
Rectilinear brim ears are closed loops without a perimeter marker: Brim.cpp builds
them with FillWithPerimeter, which appends one closed perimeter path per ear and
never adds EXTRUSION_PROPERTY_TYPE_PERIMETER. STEP_ORDERING asks a seam for every
closed loop of the plan, so the regional router must project the approach
position onto such an ear instead of sending it to a provider session that never
bound it.
*/
TEST_CASE("PrintingPlan writer exports a cube with rectilinear brim ears and cost seams",
          "[plugins][gcode][seam-placer][cost][integration][brim-ears]")
{
    const CostSeamExportResult exported = export_cost_seam_gcode(
        {{"brim_width", "3.2"}, {"brim_per_object", "1"}, {"brim_ears", "1"},
         {"brim_ears_pattern", "rectilinear"}, {"draft_shield", "disabled"},
         {"brim_skirt_trim", "0"}, {"skirts", "0"}},
        1,
        [](const Print &print) {
            // The ears must exist as unmarked closed loops before ordering runs.
            const size_t unmarked_ears = count_auxiliary_unmarked_loops(print.object(0));
            REQUIRE(unmarked_ears > 0);
            UNSCOPED_INFO("unmarked brim loops: " << unmarked_ears);
        });
    CHECK(exported.ordering_completed);
    CHECK(exported.gcode.find("G1 ") != std::string::npos);
    CHECK(exported.gcode.find(";TYPE:Skirt") != std::string::npos);
}

/*
A two-extruder print needs the wipe tower, and the tower helper prints closed
rings and rectangles without a perimeter marker. Those loops are created in
STEP_WIPETOWER, which Steps::execution_order() runs after STEP_ORDERING, so the
regional router never receives them. This test keeps the two-extruder
cost/advanced combination covered and checks that ordering and the tower both run
and reach the exported file.
*/
TEST_CASE("PrintingPlan writer exports a two-extruder cube with a wipe tower and cost seams",
          "[plugins][gcode][seam-placer][cost][integration][wipe-tower]")
{
    // Every per-extruder vector option needs one entry per nozzle, otherwise the
    // layer-height step asserts on its own bounds.
    const CostSeamExportResult exported = export_cost_seam_gcode(
        {{"nozzle_diameter", "0.4,0.4"}, {"filament_diameter", "1.75,1.75"},
         {"single_extruder_multi_material", "0"},
         {"perimeter_extruder", "1"}, {"infill_extruder", "2"},
         {"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
         {"wiping_volumes_matrix", "0,140,140,0"},
         {"filament_minimal_purge_on_wipe_tower", "15,15"}},
        2,
        [](const Print &print) {
            // The tower is generated after ordering, so it cannot reach the router.
            CHECK_FALSE(print.is_step_done(STEP_WIPETOWER));
        });
    CHECK(exported.ordering_completed);
    CHECK(exported.wipe_tower_completed);
    CHECK(exported.gcode.find("G1 ") != std::string::npos);
    CHECK(exported.gcode.find(";TYPE:Wipe tower") != std::string::npos);
}

/*
The concentric pattern publishes each ring as one polyline, and DefaultInfillGenerator
clips that polyline by the seam gap. A zero seam gap keeps the closing point, so
the ring stays a closed loop without a perimeter marker and STEP_ORDERING must
project the approach position onto it. The legacy FillConcentric gap-fill path is
not part of this pipeline: DefaultInfillGenerator dispatches to pattern plugins
and leaves gap fill to the post-infill step.
*/
TEST_CASE("PrintingPlan writer exports a cube with concentric closed infill loops and cost seams",
          "[plugins][gcode][seam-placer][cost][integration][concentric]")
{
    const CostSeamExportResult exported = export_cost_seam_gcode(
        {{"fill_pattern", "concentric"}, {"solid_fill_pattern", "concentric"},
         {"top_fill_pattern", "concentric"}, {"bottom_fill_pattern", "concentric"},
         {"seam_gap", "0"}, {"skirts", "0"}},
        1,
        [](const Print &print) {
            const size_t unmarked_loops = count_infill_unmarked_loops(print.object(0));
            REQUIRE(unmarked_loops > 0);
            UNSCOPED_INFO("unmarked infill loops: " << unmarked_loops);
        });
    CHECK(exported.ordering_completed);
    CHECK(exported.gcode.find("G1 ") != std::string::npos);
    CHECK(exported.gcode.find(";TYPE:Internal infill") != std::string::npos);
}
