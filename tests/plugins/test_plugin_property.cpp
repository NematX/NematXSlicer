// Guard metadata ownership, read-only capabilities and checkpoint editing windows against regressions.
#include <catch2/catch.hpp>

#include "plugin_test_helpers.hpp"
#include "perimeter_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_toolpath.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionTreeVisitors.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PluginProperty.hpp"
#include "libslic3r/Plugins/PluginApiCompatibility.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintObjectRegion.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "libslic3r/Surface.hpp"

#include <utility>
#include <functional>
#include <type_traits>
#include <stdexcept>
#include <vector>
#include <thread>

namespace {
using namespace Slic3r;

struct TestSurfacePayload
{
    uint32_t priority = 0;
    uint32_t layer_count = 0;
};

/* Payload without a static id, used to exercise the runtime-key API. */
struct RuntimePropertyPayload
{
    uint32_t value = 0;
    uint32_t generation = 0;
};

/* Deliberately incompatible payload used to test namespaced layout checks. */
struct IncompatibleRuntimePropertyPayload
{
    uint8_t value = 0;
};

/* Reads one inherited runtime property from each visited extrusion leaf. */
class RuntimePropertyVisitor : public slic3r_api::ExtrusionTreeVisitor<>
{
public:
    explicit RuntimePropertyVisitor(
        const slic3r_api::PluginPropertyKey<RuntimePropertyPayload> &key) :
        m_key(key)
    {
    }

    uint32_t visited_value = 0;

protected:
    void visit_leaf(slic3r_api::MutableExtrusionEntity) override
    {
        if (const RuntimePropertyPayload *property = current_property(m_key))
            visited_value = property->value;
    }

private:
    slic3r_api::PluginPropertyKey<RuntimePropertyPayload> m_key;
};

orchestrator_handle *test_orchestrator_handle()
{
    return reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance());
}

slic3r_api::PluginPropertyKey<TestSurfacePayload> test_surface_payload_key()
{
    return slic3r_api::PluginPropertyKey<TestSurfacePayload>::register_dynamic(
        test_orchestrator_handle(), "tests.surface.payload");
}

slic3r_api::PluginProperties property_view(PluginPropertyContainer &container)
{
    return slic3r_api::PluginProperties(
        reinterpret_cast<plugin_property_container_handle *>(&container));
}

ExPolygon rectangle_expolygon(double min_x, double min_y, double max_x, double max_y)
{
    return ExPolygon(Polygon({
        Point(scale_i(min_x), scale_i(min_y)),
        Point(scale_i(max_x), scale_i(min_y)),
        Point(scale_i(max_x), scale_i(max_y)),
        Point(scale_i(min_x), scale_i(max_y))
    }));
}

#if defined(SLIC3R_TEST_STRICT_FREEZE)
class StrictFreezeState
{
public:
    StrictFreezeState()
    {
        Orchestrator::instance().consume_plugin_messages();
        Orchestrator::instance().reset_plugin_cancel();
    }
    ~StrictFreezeState()
    {
        Orchestrator::instance().consume_plugin_messages();
        Orchestrator::instance().reset_plugin_cancel();
    }
};

void require_freeze_diagnostic(slicing_step_t step, const char *operation)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    CHECK(orchestrator.is_plugin_cancelled());
    const std::vector<Orchestrator::PluginMessage> messages = orchestrator.consume_plugin_messages();
    REQUIRE(messages.size() == 1);
    CHECK(messages.front().level == Orchestrator::PluginMessageLevel::Error);
    CHECK(messages.front().plugin_id == "<unknown>");
    CHECK(messages.front().step == step);
    CHECK(messages.front().diagnostic_id >= PLUGIN_ERROR_FIRST_DYNAMIC);
    CHECK(orchestrator.error_message(messages.front().diagnostic_id) == messages.front().message);
    CHECK(messages.front().message.find(operation) != std::string::npos);
    orchestrator.reset_plugin_cancel();
}

class DetachedPrintObject : public PrintObject
{
public:
    DetachedPrintObject(Print &print, std::shared_ptr<PrintObjectRegions> regions, bool detach_immediately = true) :
        PrintObject(&print, Vec3crd(scale_i(1.), scale_i(1.), scale_i(1.)),
                    std::move(regions))
    {
        if (detach_immediately)
            detach();
    }
    void detach() { m_print = nullptr; }
};

class FreezeBarrierProbe : public slic3r_api::PluginBase
{
public:
    explicit FreezeBarrierProbe(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}
    mutable bool refused = false;
    mutable size_t runs = 0;

private:
    const char *id_impl() const noexcept override { return "test.freeze.barrier"; }
    const char *name_impl() const noexcept override { return "Freeze barrier probe"; }
    slicing_step_t step_impl() const noexcept override { return STEP_ORDERING; }
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    int32_t priority_impl() const noexcept override { return 0; }
    void setup_impl(const plugin_run_context *context, uint32_t) const override
    {
        Print &print = *const_cast<Print *>(reinterpret_cast<const Print *>(context->print));
        refused = object_get_properties_mutable(reinterpret_cast<object_handle *>(&print.object(0))) == nullptr;
    }
    void run_impl(const plugin_run_context *) const override { ++runs; }
};
#endif

} // namespace

TEST_CASE("Data-tree property getters preserve exact inclusive editing windows",
          "[plugins][properties][freeze]")
{
    static_assert(std::is_same<decltype(object_get_properties(nullptr)), const plugin_property_container_handle *>::value);
    static_assert(std::is_same<decltype(layer_get_properties(nullptr)), const plugin_property_container_handle *>::value);
    static_assert(std::is_same<decltype(layer_region_get_properties(nullptr)), const plugin_property_container_handle *>::value);
    static_assert(std::is_same<decltype(layer_island_get_properties(nullptr)), const plugin_property_container_handle *>::value);
    static_assert(std::is_same<decltype(layer_region_island_get_properties(nullptr)), const plugin_property_container_handle *>::value);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
#if defined(SLIC3R_TEST_STRICT_FREEZE)
    const StrictFreezeState strict_state;
#endif
    Test::PerimeterPluginTests::PreparedPerimeterPrint prepared;
    Test::PerimeterPluginTests::prepare_cube_print(prepared, Test::PerimeterPluginTests::perimeter_config({}));
    Print &print = prepared.print;
    PrintObject &object = print.object(0);
    Layer &main = object.layer(0);
    PrintObject &global = print.mutable_auxiliary_object();
    Layer *auxiliary[2];
    PrintObject *owners[2] = {&object, &global};
    for (size_t idx = 0; idx < 2; ++idx) {
        layer_handle *handle = object_add_auxiliary_layer(reinterpret_cast<const object_handle *>(owners[idx]),
                                                        scale_i(0.2), scale_i(0.2), scale_i(0.1));
        REQUIRE(handle != nullptr);
        auxiliary[idx] = reinterpret_cast<Layer *>(handle);
        *reinterpret_cast<ExPolygons *>(layer_region_borrow_mutable_slices(
            reinterpret_cast<layer_region_handle *>(&auxiliary[idx]->region(0)))) =
            ExPolygons{rectangle_expolygon(0., 0., 10., 10.)};
        layer_recompute_slices_and_islands_from_layer_regions(handle);
        layer_add_regions_to_islands(handle);
        REQUIRE(auxiliary[idx]->islands().size() == 1);
    }
    struct Node {
        slicing_step_t last;
        std::function<plugin_property_container_handle *()> write;
        std::function<const plugin_property_container_handle *()> read;
    };
    std::vector<Node> nodes;
    for (PrintObject *owner : owners) {
        object_handle *handle = reinterpret_cast<object_handle *>(owner);
        nodes.push_back({STEP_FINALIZE_SLICING, [handle]() { return object_get_properties_mutable(handle); },
                        [handle]() { return object_get_properties(handle); }});
    }
    for (Layer *layer : {&main, auxiliary[0], auxiliary[1]}) {
        const slicing_step_t last = layer == &main ? STEP_POST_SLICING : STEP_FINALIZE_SLICING;
        layer_handle *handle = reinterpret_cast<layer_handle *>(layer);
        layer_region_handle *region = reinterpret_cast<layer_region_handle *>(&layer->region(0));
        layer_island_handle *island = reinterpret_cast<layer_island_handle *>(&layer->island(0));
        LayerRegionIsland &bucket = layer->island(0).get_or_add_region_island(layer->island(0).regions(), 0);
        layer_region_island_handle *group = reinterpret_cast<layer_region_island_handle *>(&bucket);
        nodes.push_back({last, [handle]() { return layer_get_properties_mutable(handle); },
                        [handle]() { return layer_get_properties(handle); }});
        nodes.push_back({last, [region]() { return layer_region_get_properties_mutable(region); },
                        [region]() { return layer_region_get_properties(region); }});
        nodes.push_back({last, [island]() { return layer_island_get_properties_mutable(island); },
                        [island]() { return layer_island_get_properties(island); }});
        nodes.push_back({STEP_FINALIZE_SLICING, [group]() { return layer_region_island_get_properties_mutable(group); },
                        [group]() { return layer_region_island_get_properties(group); }});
    }
    for (const Node &node : nodes) {
        print.set_running_step(node.last);
        REQUIRE(print.running_step_may_edit(node.last));
        plugin_property_container_handle *properties = node.write();
        REQUIRE(properties != nullptr);
        c_layer_support_property *payload = static_cast<c_layer_support_property *>(
            plugin_property_get_or_add_data_mutable(test_orchestrator_handle(), properties, PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT));
        REQUIRE(payload != nullptr);
        payload->interface_id = 73;
        CHECK(node.read() == properties);
        print.set_running_step(node.last == STEP_POST_SLICING ? STEP_PRE_PERIMETER : STEP_ORDERING);
        CHECK_FALSE(print.running_step_may_edit(node.last));
#if defined(SLIC3R_TEST_STRICT_FREEZE)
        CHECK(node.write() == nullptr);
        require_freeze_diagnostic(print.running_step(), "properties_mutable");
#endif
        REQUIRE(plugin_property_data(node.read(), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) != nullptr);
        CHECK(static_cast<const c_layer_support_property *>(
            plugin_property_data(node.read(), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT))->interface_id == 73);
    }
    print.set_running_step(STEP_NONE);
    CHECK(object_get_properties_mutable(nullptr) == nullptr);
    CHECK(layer_get_properties_mutable(nullptr) == nullptr);
    CHECK(layer_region_get_properties_mutable(nullptr) == nullptr);
    CHECK(layer_island_get_properties_mutable(nullptr) == nullptr);
    CHECK(layer_region_island_get_properties_mutable(nullptr) == nullptr);
}

#if defined(SLIC3R_TEST_STRICT_FREEZE)
TEST_CASE("Strict freeze refuses slices records and auxiliary edits before mutation",
          "[plugins][properties][freeze][strict-freeze]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const StrictFreezeState strict_state;
    Test::PerimeterPluginTests::PreparedPerimeterPrint prepared;
    Test::PerimeterPluginTests::prepare_cube_print(prepared, Test::PerimeterPluginTests::perimeter_config({}));
    Print &print = prepared.print;
    PrintObject &object = print.object(0);
    PrintObject &global = print.mutable_auxiliary_object();
    Layer *layers[3] = {&object.layer(0), nullptr, nullptr};
    PrintObject *owners[2] = {&object, &global};
    for (size_t idx = 0; idx < 2; ++idx) {
        const object_handle *handle = reinterpret_cast<const object_handle *>(owners[idx]);
        print.set_running_step(STEP_FINALIZE_SLICING);
        layer_handle *aux = object_add_auxiliary_layer(handle, scale_i(0.2), scale_i(0.2), scale_i(0.1));
        REQUIRE(aux != nullptr);
        CHECK(object_remove_auxiliary_layer(handle, aux) == 1);
        aux = object_add_auxiliary_layer(handle, scale_i(0.2), scale_i(0.2), scale_i(0.1));
        REQUIRE(aux != nullptr);
        layers[idx + 1] = reinterpret_cast<Layer *>(aux);
        const size_t count = owners[idx]->auxiliary_layer_count();
        print.set_running_step(STEP_ORDERING);
        CHECK(object_add_auxiliary_layer(handle, scale_i(0.4), scale_i(0.4), scale_i(0.2)) == nullptr);
        require_freeze_diagnostic(STEP_ORDERING, "object_add_auxiliary_layer");
        print.set_running_step(STEP_ORDERING);
        CHECK(object_remove_auxiliary_layer(handle, aux) == 0);
        require_freeze_diagnostic(STEP_ORDERING, "object_remove_auxiliary_layer");
        CHECK(owners[idx]->auxiliary_layer_count() == count);
        CHECK(&owners[idx]->auxiliary_layer(0) == layers[idx + 1]);
    }
    for (Layer *layer : layers) {
        const slicing_step_t last = layer == layers[0] ? STEP_POST_SLICING : STEP_FINALIZE_SLICING;
        const slicing_step_t next = layer == layers[0] ? STEP_PRE_PERIMETER : STEP_ORDERING;
        layer_handle *handle = reinterpret_cast<layer_handle *>(layer);
        const layer_region_handle *region = reinterpret_cast<const layer_region_handle *>(&layer->region(0));
        print.set_running_step(last);
        REQUIRE(layer_borrow_mutable_slices(handle) != nullptr);
        REQUIRE(layer_region_borrow_mutable_slices(region) != nullptr);
        const ExPolygons slices = layer->lslices();
        const ExPolygons raw_slices = layer->region(0).get_raw_slices();
        print.set_running_step(next);
        CHECK(layer_borrow_mutable_slices(handle) == nullptr);
        require_freeze_diagnostic(next, "layer_borrow_mutable_slices");
        print.set_running_step(next);
        CHECK(layer_region_borrow_mutable_slices(region) == nullptr);
        require_freeze_diagnostic(next, "layer_region_borrow_mutable_slices");
        CHECK(layer->lslices() == slices);
        CHECK(layer->region(0).get_raw_slices() == raw_slices);
        CHECK(layer_get_slices(handle) != nullptr);
        CHECK(layer_region_get_slices(region) != nullptr);
    }
    const print_handle *handle = reinterpret_cast<const print_handle *>(&print);
    print.set_running_step(STEP_FINALIZE_SLICING);
    REQUIRE(print_records_get_or_add(handle, "test.strict.removable") != nullptr);
    CHECK(print_records_remove(handle, "test.strict.removable") == 1);
    config_handle *record = print_records_get_or_add(handle, "test.strict.stable");
    REQUIRE(record != nullptr);
    print.records().get_or_add("test.strict.stable").set_key_value("value", new ConfigOptionInt(73));
    const std::vector<std::string> channels = print.records().channels();
    print.set_running_step(STEP_ORDERING);
    CHECK(print_records_get_or_add(handle, "test.strict.new") == nullptr);
    require_freeze_diagnostic(STEP_ORDERING, "print_records_get_or_add");
    print.set_running_step(STEP_ORDERING);
    CHECK(print_records_get_or_add(handle, "test.strict.stable") == nullptr);
    require_freeze_diagnostic(STEP_ORDERING, "print_records_get_or_add");
    print.set_running_step(STEP_ORDERING);
    CHECK(print_records_remove(handle, "test.strict.stable") == 0);
    require_freeze_diagnostic(STEP_ORDERING, "print_records_remove");
    CHECK(print.records().channels() == channels);
    CHECK(print_records_get(handle, "test.strict.stable") == record);
    CHECK(print.records().find("test.strict.stable")->option<ConfigOptionInt>("value")->value == 73);
    print.set_running_step(STEP_NONE);
}

TEST_CASE("Strict freeze leaves null detached and idle accesses free of diagnostics",
          "[plugins][properties][freeze][strict-freeze]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const StrictFreezeState strict_state;
    Print print;
    std::shared_ptr<PrintObjectRegions> regions = std::make_shared<PrintObjectRegions>();
    regions->all_regions = print.mutable_auxiliary_object().shared_regions()->all_regions;
    DetachedPrintObject detached(print, regions);
    print.set_running_step(STEP_GCODE);
    object_handle *object = reinterpret_cast<object_handle *>(&detached);
    REQUIRE(object_get_properties_mutable(object) != nullptr);
    layer_handle *layer = object_add_auxiliary_layer(object, scale_i(0.2), scale_i(0.2), scale_i(0.1));
    REQUIRE(layer != nullptr);
    REQUIRE(layer_get_properties_mutable(layer) != nullptr);
    layer_region_handle *region = layer_get_region_mutable(layer, 0);
    REQUIRE(region != nullptr);
    REQUIRE(layer_region_get_properties_mutable(region) != nullptr);
    REQUIRE(layer_borrow_mutable_slices(layer) != nullptr);
    REQUIRE(layer_region_borrow_mutable_slices(region) != nullptr);
    LayerSliceIsland island(rectangle_expolygon(0., 0., 10., 10.));
    REQUIRE(layer_island_get_properties_mutable(reinterpret_cast<layer_island_handle *>(&island)) != nullptr);
    LayerRegionIsland group({}, EXTRUDER_ID_UNKNOWN);
    REQUIRE(layer_region_island_get_properties_mutable(reinterpret_cast<layer_region_island_handle *>(&group)) != nullptr);
    CHECK(object_remove_auxiliary_layer(object, layer) == 1);

    CHECK(object_get_properties_mutable(nullptr) == nullptr);
    CHECK(layer_get_properties_mutable(nullptr) == nullptr);
    CHECK(layer_region_get_properties_mutable(nullptr) == nullptr);
    CHECK(layer_island_get_properties_mutable(nullptr) == nullptr);
    CHECK(layer_region_island_get_properties_mutable(nullptr) == nullptr);
    CHECK(layer_borrow_mutable_slices(nullptr) == nullptr);
    CHECK(layer_region_borrow_mutable_slices(nullptr) == nullptr);
    CHECK(object_add_auxiliary_layer(nullptr, 1, 1, 1) == nullptr);
    CHECK(object_remove_auxiliary_layer(nullptr, nullptr) == 0);
    CHECK(print_records_get_or_add(nullptr, "test") == nullptr);
    CHECK(print_records_remove(nullptr, "test") == 0);
    const print_handle *handle = reinterpret_cast<const print_handle *>(&print);
    CHECK(print_records_get_or_add(handle, nullptr) == nullptr);
    CHECK(print_records_remove(handle, nullptr) == 0);

    print.set_running_step(STEP_NONE);
    object_handle *global = reinterpret_cast<object_handle *>(&print.mutable_auxiliary_object());
    REQUIRE(object_get_properties_mutable(global) != nullptr);
    layer = object_add_auxiliary_layer(global, scale_i(0.2), scale_i(0.2), scale_i(0.1));
    REQUIRE(layer != nullptr);
    REQUIRE(layer_get_properties_mutable(layer) != nullptr);
    REQUIRE(layer_region_get_properties_mutable(layer_get_region_mutable(layer, 0)) != nullptr);
    REQUIRE(layer_borrow_mutable_slices(layer) != nullptr);
    REQUIRE(layer_region_borrow_mutable_slices(layer_get_region(layer, 0)) != nullptr);
    CHECK(object_remove_auxiliary_layer(global, layer) == 1);
    REQUIRE(print_records_get_or_add(handle, "test.idle") != nullptr);
    CHECK(print_records_remove(handle, "test.idle") == 1);
    CHECK_FALSE(Orchestrator::instance().is_plugin_cancelled());
    CHECK(Orchestrator::instance().consume_plugin_messages().empty());
}

TEST_CASE("Strict freeze C++ views propagate refused capabilities without assertions",
          "[plugins][properties][freeze][strict-freeze]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const StrictFreezeState strict_state;
    Test::PerimeterPluginTests::PreparedPerimeterPrint prepared;
    Test::PerimeterPluginTests::prepare_cube_print(prepared, Test::PerimeterPluginTests::perimeter_config({}));
    Print &print = prepared.print;
    const slic3r_api::Object object(reinterpret_cast<const object_handle *>(&print.object(0)));
    print.set_running_step(STEP_ORDERING);
    slic3r_api::PluginProperties properties = object.mutable_properties();
    CHECK_FALSE(properties.valid());
    CHECK_THROWS_AS(properties.get_or_add<slic3r_api::LayerSupportProperty>(
        test_orchestrator_handle(), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT), std::runtime_error);
    CHECK_THROWS_AS(properties.get_or_add(slic3r_api::LayerSupportProperty::key), std::runtime_error);
    CHECK(properties.get_mutable(slic3r_api::LayerSupportProperty::key) == nullptr);
    CHECK_FALSE(properties.remove(PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT));
    properties.clear();
    properties.copy_from(object.properties());
    require_freeze_diagnostic(STEP_ORDERING, "object_get_properties_mutable");
    const size_t count = print.object(0).auxiliary_layer_count();
    print.set_running_step(STEP_ORDERING);
    CHECK_THROWS_AS(object.add_auxiliary_layer(scale_i(0.2), scale_i(0.2), scale_i(0.1)), std::runtime_error);
    require_freeze_diagnostic(STEP_ORDERING, "object_add_auxiliary_layer");
    CHECK(print.object(0).auxiliary_layer_count() == count);
    const slic3r_api::Print view(reinterpret_cast<const print_handle *>(&print));
    print.set_running_step(STEP_ORDERING);
    CHECK_THROWS_AS(view.records().get_or_add("test.strict.cpp"), std::runtime_error);
    require_freeze_diagnostic(STEP_ORDERING, "print_records_get_or_add");
    CHECK(print.records().find("test.strict.cpp") == nullptr);
    print.set_running_step(STEP_NONE);
}

TEST_CASE("Strict freeze deduplicates concurrent refusals per Print and execution attempt",
          "[plugins][properties][freeze][strict-freeze][concurrency]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const StrictFreezeState strict_state;
    Print first;
    Print second;
    object_handle *owners[2] = {
        reinterpret_cast<object_handle *>(&first.mutable_auxiliary_object()),
        reinterpret_cast<object_handle *>(&second.mutable_auxiliary_object())};
    first.set_running_step(STEP_ORDERING);
    second.set_running_step(STEP_ORDERING);
    std::atomic_uint refusals{0};
    std::vector<std::thread> workers;
    for (size_t index = 0; index < 16; ++index)
        workers.emplace_back([&, index] {
            if (object_get_properties_mutable(owners[index % 2]) == nullptr)
                refusals.fetch_add(1);
        });
    for (std::thread &worker : workers)
        worker.join();
    CHECK(refusals.load() == 16);
    Orchestrator &orchestrator = Orchestrator::instance();
    CHECK(orchestrator.is_plugin_cancelled());
    const std::vector<Orchestrator::PluginMessage> messages = orchestrator.consume_plugin_messages();
    REQUIRE(messages.size() == 2);
    CHECK(messages[0].diagnostic_id != messages[1].diagnostic_id);
    for (const Orchestrator::PluginMessage &message : messages) {
        CHECK(message.message.find("STEP_ORDERING") != std::string::npos);
        CHECK(message.message.find("STEP_FINALIZE_SLICING") != std::string::npos);
        CHECK(orchestrator.error_message(message.diagnostic_id) == message.message);
    }
    for (size_t attempt = 0; attempt < 2; ++attempt) {
        orchestrator.reset_plugin_cancel();
        first.prepare_checkpoint_resume();
        owners[0] = reinterpret_cast<object_handle *>(&first.mutable_auxiliary_object());
        CHECK(object_get_properties_mutable(owners[0]) == nullptr);
        CHECK(orchestrator.is_plugin_cancelled());
        CHECK(orchestrator.consume_plugin_messages().empty());
    }
    for (const slicing_step_t step : {STEP_ORDERING, STEP_GCODE, STEP_GCODE}) {
        first.set_running_step(step);
        CHECK(object_get_properties_mutable(owners[0]) == nullptr);
        require_freeze_diagnostic(step, "object_get_properties_mutable");
    }
    first.set_running_step(STEP_NONE);
    second.set_running_step(STEP_NONE);
}

TEST_CASE("Strict freeze cancellation stops the plugin lifecycle at the setup barrier",
          "[plugins][properties][freeze][strict-freeze]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const StrictFreezeState strict_state;
    Orchestrator &orchestrator = Orchestrator::instance();
    static FreezeBarrierProbe probe(test_orchestrator_handle());
    if (orchestrator.get_plugin("test.freeze.barrier") == nullptr) {
        REQUIRE(orchestrator.register_plugin(probe.c_instance()));
        REQUIRE(orchestrator.set_plugin_active("test.freeze.barrier", false));
    }
    struct DeactivateProbe {
        ~DeactivateProbe() { Orchestrator::instance().set_plugin_active("test.freeze.barrier", false); }
    } deactivate;
    Test::PerimeterPluginTests::PreparedPerimeterPrint prepared;
    Test::PerimeterPluginTests::prepare_cube_print(prepared, Test::PerimeterPluginTests::perimeter_config({}));
    REQUIRE(orchestrator.set_plugin_active("test.freeze.barrier", true));
    Plugin *plugin = orchestrator.get_plugin("test.freeze.barrier");
    REQUIRE(plugin != nullptr);
    const raw_plugin_run_payload payload{};
    probe.runs = 0;
    prepared.print.set_running_step(STEP_ORDERING);
    CHECK(orchestrator.execute_plugin(*plugin, &prepared.print, &payload, 1) == RAW_PLUGIN_EXECUTION_CANCELLED);
    CHECK(probe.refused);
    CHECK(probe.runs == 0);
    require_freeze_diagnostic(STEP_ORDERING, "object_get_properties_mutable");
    prepared.print.set_running_step(STEP_NONE);
    CHECK(orchestrator.execute_plugin(*plugin, &prepared.print, &payload, 1) == RAW_PLUGIN_EXECUTION_SUCCESS);
    CHECK_FALSE(probe.refused);
    CHECK(probe.runs == 1);
}
#endif

TEST_CASE("Surface properties require a scratch editing capability regardless of pipeline phase",
          "[plugins][properties][surface][freeze]")
{
    static_assert(std::is_same<decltype(surface_get_properties(nullptr)), const plugin_property_container_handle *>::value);
    static_assert(std::is_same<decltype(layer_region_island_get_fill_surfaces(nullptr)), const surface_collection_handle *>::value);
    static_assert(std::is_same<decltype(layer_region_island_get_fill_surface(nullptr, 0)), const surface_handle *>::value);
    static_assert(!std::is_constructible<slic3r_api::MutableSurface, surface_handle *>::value);
    static_assert(std::is_same<decltype(std::declval<slic3r_api::Surface>().properties()), slic3r_api::ConstPluginProperties>::value);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    PluginStorage storage;
    slic3r_api::StoredSurfaceCollection scratch(reinterpret_cast<storage_handle *>(&storage));
    ExPolygon geometry = rectangle_expolygon(0., 0., 10., 10.);
    scratch.append(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&geometry)), RAW_SURFACE_TYPE_NONE);
    for (const slicing_step_t step : {STEP_FINALIZE_SLICING, STEP_ORDERING, STEP_GCODE}) {
        print.set_running_step(step);
        scratch.mutable_at(0).mutable_properties().get_or_add(slic3r_api::LayerSupportProperty::key).interface_id = uint32_t(step);
        REQUIRE(scratch.readonly().at(0).properties().get(slic3r_api::LayerSupportProperty::key) != nullptr);
        CHECK(scratch.readonly().at(0).properties().get(slic3r_api::LayerSupportProperty::key)->interface_id == uint32_t(step));
    }
    CHECK(surface_collection_get_properties_mutable(nullptr, 0) == nullptr);
    CHECK(surface_collection_get_properties_mutable(scratch.mutable_handle(), 1) == nullptr);
    print.set_running_step(STEP_NONE);
}

TEST_CASE("Property freeze rejects the previous data-tree ABI major",
          "[plugins][properties][abi]")
{
    static_assert(SLIC3R_PLUGIN_API_DATA_TREE_MAJOR == 5u);
    std::vector<PluginApiRequirement> requirements = {
        {"slic3r_plugin_types.h", {SLIC3R_PLUGIN_API_PLUGIN_TYPES_MAJOR, SLIC3R_PLUGIN_API_PLUGIN_TYPES_MINOR}},
        {"slic3r_data_tree.h", {4, 7}}
    };
    CHECK_FALSE(validate_plugin_api_requirements(requirements).compatible());
    requirements.back().version = {SLIC3R_PLUGIN_API_DATA_TREE_MAJOR, SLIC3R_PLUGIN_API_DATA_TREE_MINOR};
    CHECK(validate_plugin_api_requirements(requirements).compatible());
}

// The new queries add functions without changing existing data-tree layouts.
TEST_CASE("Default configuration queries preserve compatibility with earlier data-tree 5 packages",
          "[plugins][abi][default-region]")
{
    static_assert(SLIC3R_PLUGIN_API_DATA_TREE_MAJOR == 5u);
    for (uint16_t minor : {uint16_t(0), uint16_t(1), uint16_t(2)}) {
        CAPTURE(minor);
        const std::vector<PluginApiRequirement> requirements = {
            {"slic3r_plugin_types.h", {SLIC3R_PLUGIN_API_PLUGIN_TYPES_MAJOR, SLIC3R_PLUGIN_API_PLUGIN_TYPES_MINOR}},
            {"slic3r_data_tree.h", {5, minor}}
        };
        CHECK(validate_plugin_api_requirements(requirements).compatible());
    }
}

// Sparse support algorithms require the role layout without a separate branch selector.
TEST_CASE("Support toolpath rejects the previous role ABI major", "[plugins][abi][support-toolpath]")
{
    static_assert(SLIC3R_PLUGIN_API_STEP_SUPPORT_TOOLPATH_MAJOR == 2u);
    std::vector<PluginApiRequirement> requirements = {
        {"slic3r_plugin_types.h", {SLIC3R_PLUGIN_API_PLUGIN_TYPES_MAJOR, SLIC3R_PLUGIN_API_PLUGIN_TYPES_MINOR}},
        {"steps/slic3r_step_support_toolpath.h", {1, 3}}
    };
    CHECK_FALSE(validate_plugin_api_requirements(requirements).compatible());
    requirements.back().version = {SLIC3R_PLUGIN_API_STEP_SUPPORT_TOOLPATH_MAJOR, SLIC3R_PLUGIN_API_STEP_SUPPORT_TOOLPATH_MINOR};
    CHECK(validate_plugin_api_requirements(requirements).compatible());
}

TEST_CASE("Remaining source editing gates preserve their exact boundaries",
          "[plugins][properties][freeze]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
#if defined(SLIC3R_TEST_STRICT_FREEZE)
    const StrictFreezeState strict_state;
#endif
    Test::PerimeterPluginTests::PreparedPerimeterPrint prepared;
    Test::PerimeterPluginTests::prepare_cube_print(prepared, Test::PerimeterPluginTests::perimeter_config({}));
    Print &print = prepared.print;
    PrintObject &object = print.object(0);
    PrintObject &global = print.mutable_auxiliary_object();
    Layer *layers[3] = {&object.layer(0), nullptr, nullptr};
    PrintObject *owners[2] = {&object, &global};
    for (size_t idx = 0; idx < 2; ++idx) {
        layer_handle *layer = object_add_auxiliary_layer(reinterpret_cast<const object_handle *>(owners[idx]),
                                                       scale_i(0.2), scale_i(0.2), scale_i(0.1));
        REQUIRE(layer != nullptr);
        layers[idx + 1] = reinterpret_cast<Layer *>(layer);
        *reinterpret_cast<ExPolygons *>(layer_region_borrow_mutable_slices(layer_get_region(layer, 0))) =
            ExPolygons{rectangle_expolygon(0., 0., 10., 10.)};
        layer_recompute_slices_and_islands_from_layer_regions(layer);
        layer_add_regions_to_islands(layer);
    }
    PluginStorage storage;
    slic3r_api::StoredExtrusionEntity scratch(reinterpret_cast<storage_handle *>(&storage),
                                             std::vector<c_point>{{0, 0}, {scale_i(1.), 0}});
    for (Layer *layer : layers) {
        layer_island_handle *island = reinterpret_cast<layer_island_handle *>(&layer->island(0));
        const slicing_step_t last_slice = layer == layers[0] ? STEP_POST_SLICING : STEP_FINALIZE_SLICING;
        print.set_running_step(last_slice);
        const expolygon_handle *slice = layer_island_get_slice_mutable(island);
        REQUIRE(slice != nullptr);
        CHECK(layer_island_get_slice(island) == slice);
        const ExPolygon baseline_slice = layer->island(0).get_slice();
#if defined(SLIC3R_TEST_STRICT_FREEZE)
        print.set_running_step(last_slice == STEP_POST_SLICING ? STEP_PRE_PERIMETER : STEP_ORDERING);
        CHECK(layer_island_get_slice_mutable(island) == nullptr);
        require_freeze_diagnostic(print.running_step(), "layer_island_get_slice_mutable");
        CHECK(layer->island(0).get_slice() == baseline_slice);
        CHECK(layer_island_get_slice(island) == slice);
#endif
        print.set_running_step(STEP_FINALIZE_SLICING);
        layer_region_island_handle *group = layer_island_get_or_create_region_island(island, nullptr, 0, 0);
        REQUIRE(group != nullptr);
        REQUIRE(layer_island_get_or_create_region_island(island, nullptr, 0, EXTRUDER_ID_UNKNOWN) != nullptr);
        REQUIRE(layer_island_get_region_island_mutable(island, 0) == group);
        extrusion_entity_handle *root = layer_region_island_get_mutable_extrusion(group, RAW_EXTRUSION_ROLE_PERIMETER);
        REQUIRE(root != nullptr);
        slic3r_api::MutableExtrusionEntity editable(root);
        CHECK(editable.append_child_copy(scratch.readonly()) == 0);
        editable.get_or_add(slic3r_api::EPropertySpeed::key).speed(33.f);
        REQUIRE(layer_region_island_get_extrusion(group, RAW_EXTRUSION_ROLE_PERIMETER) == root);
        const size_t group_count = layer->island(0).regions_islands().size();
        const uint32_t flags = editable.flags();
        const c_point endpoint = editable.child(0).point(1);
#if defined(SLIC3R_TEST_STRICT_FREEZE)
        print.set_running_step(STEP_ORDERING);
        CHECK(layer_island_get_region_island_mutable(island, 0) == nullptr);
        require_freeze_diagnostic(STEP_ORDERING, "layer_island_get_region_island_mutable");
        print.set_running_step(STEP_ORDERING);
        CHECK(layer_island_get_or_create_region_island(island, nullptr, 0, 0) == nullptr);
        require_freeze_diagnostic(STEP_ORDERING, "layer_island_get_or_create_region_island");
        print.set_running_step(STEP_ORDERING);
        CHECK(layer_island_get_or_create_region_island(island, nullptr, 0, 1) == nullptr);
        require_freeze_diagnostic(STEP_ORDERING, "layer_island_get_or_create_region_island");
        print.set_running_step(STEP_ORDERING);
        CHECK(layer_region_island_get_mutable_extrusion(group, RAW_EXTRUSION_ROLE_PERIMETER) == nullptr);
        require_freeze_diagnostic(STEP_ORDERING, "layer_region_island_get_mutable_extrusion");
        print.set_running_step(STEP_ORDERING);
        layer_recompute_slices_and_islands_from_layer_regions(reinterpret_cast<layer_handle *>(layer));
        require_freeze_diagnostic(STEP_ORDERING, "layer_recompute_slices_and_islands_from_layer_regions");
        // Even creating an empty role bucket consumes an extrusion identity.
        const ExtrusionEntity before(false);
        print.set_running_step(STEP_ORDERING);
        CHECK(layer_region_island_get_mutable_extrusion(group, RAW_EXTRUSION_ROLE_GAP_FILL) == nullptr);
        require_freeze_diagnostic(STEP_ORDERING, "layer_region_island_get_mutable_extrusion");
        const ExtrusionEntity after(false);
        CHECK(after.get_id() == before.get_id() + 1);
        const slic3r_api::LayerIsland view(island);
        print.set_running_step(STEP_ORDERING);
        CHECK_THROWS_AS(view.get_or_create_region_island({view.region(0)}), std::runtime_error);
        require_freeze_diagnostic(STEP_ORDERING, "layer_island_get_or_create_region_island");
        print.set_running_step(STEP_ORDERING);
        CHECK_THROWS_AS(view.get_or_create_full_region_island(), std::runtime_error);
        require_freeze_diagnostic(STEP_ORDERING, "layer_island_get_or_create_region_island");
#endif
        const slic3r_api::ExtrusionEntity read(layer_region_island_get_extrusion(group, RAW_EXTRUSION_ROLE_PERIMETER));
        CHECK(layer->island(0).get_slice() == baseline_slice);
        CHECK(layer->island(0).regions_islands().size() == group_count);
        CHECK(layer_island_get_region_island(island, 0) == group);
        CHECK(read.child_count() == 1);
        CHECK(read.flags() == flags);
        CHECK(read.child(0).point(1).x == endpoint.x);
        CHECK(read.child(0).point(1).y == endpoint.y);
        REQUIRE(read.get(slic3r_api::EPropertySpeed::key) != nullptr);
        CHECK(read.get(slic3r_api::EPropertySpeed::key)->speed_mm_per_s == Approx(33.f));

        Printing::PrintingExtrusion clone;
        clone.root.reset(reinterpret_cast<const ExtrusionEntity *>(root)->clone());
        clone.region_island = reinterpret_cast<const LayerRegionIsland *>(group);
        for (const slicing_step_t step : {STEP_ORDERING, STEP_EXTRUSION_EDIT}) {
            print.set_running_step(step);
            extrusion_entity_handle *clone_handle = printing_extrusion_get_root_mutable(
                reinterpret_cast<printing_extrusion_handle *>(&clone));
            REQUIRE(clone_handle != nullptr);
            slic3r_api::MutableExtrusionEntity plan_root(clone_handle);
            plan_root.get_or_add(slic3r_api::EPropertySpeed::key).speed(99.f);
            CHECK(plan_root.child_mutable(0).set_points(std::vector<c_point>{{0, 0}, {scale_i(2.), 0}}));
            CHECK(scratch.set_points(std::vector<c_point>{{0, 0}, {scale_i(3.), 0}}));
            scratch.get_or_add(slic3r_api::EPropertySpeed::key).speed(55.f);
            CHECK(read.get(slic3r_api::EPropertySpeed::key)->speed_mm_per_s == Approx(33.f));
            CHECK(read.child(0).point(1).x == endpoint.x);
        }
        print.set_running_step(STEP_GCODE);
        CHECK(scratch.set_points(std::vector<c_point>{{0, 0}, {scale_i(4.), 0}}));
        print.set_running_step(STEP_NONE);
    }
    CHECK(layer_island_get_slice_mutable(nullptr) == nullptr);
    CHECK(layer_island_get_region_island_mutable(nullptr, 0) == nullptr);
    CHECK(layer_island_get_or_create_region_island(nullptr, nullptr, 0, 0) == nullptr);
    CHECK(layer_region_island_get_mutable_extrusion(nullptr, RAW_EXTRUSION_ROLE_PERIMETER) == nullptr);
}

TEST_CASE("Configuration editing capabilities are idle-only and reads remain available",
          "[plugins][properties][freeze]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
#if defined(SLIC3R_TEST_STRICT_FREEZE)
    const StrictFreezeState strict_state;
#endif
    Test::PerimeterPluginTests::PreparedPerimeterPrint prepared;
    Test::PerimeterPluginTests::prepare_cube_print(prepared, Test::PerimeterPluginTests::perimeter_config({}));
    Print &print = prepared.print;
    print_handle *handle = reinterpret_cast<print_handle *>(&print);
    PrintObject *owners[2] = {&print.object(0), &print.mutable_auxiliary_object()};
    REQUIRE(print_get_config_mutable(handle) != nullptr);
    const PrintConfig print_config = print.config();
    for (PrintObject *object : owners) {
        object_handle *object_handle_value = reinterpret_cast<object_handle *>(object);
        REQUIRE(object_get_config_mutable(object_handle_value) != nullptr);
        const PrintObjectConfig object_config = object->config();
#if defined(SLIC3R_TEST_STRICT_FREEZE)
        for (const slicing_step_t step : Steps::execution_order()) {
            print.set_running_step(step);
            CHECK(object_get_config_mutable(object_handle_value) == nullptr);
            require_freeze_diagnostic(step, "object_get_config_mutable");
            print.set_running_step(step);
            CHECK(print_get_config_mutable(handle) == nullptr);
            require_freeze_diagnostic(step, "print_get_config_mutable");
            CHECK(object_get_config(object_handle_value) != nullptr);
            CHECK(print_get_config(handle) != nullptr);
            CHECK(object->config() == object_config);
            CHECK(print.config() == print_config);
        }
#endif
        print.set_running_step(STEP_NONE);
        CHECK(object_get_config_mutable(object_handle_value) == object_get_config(object_handle_value));
    }
    CHECK(print_get_config_mutable(handle) == print_get_config(handle));
    CHECK(object_get_config_mutable(nullptr) == nullptr);
    CHECK(print_get_config_mutable(nullptr) == nullptr);
}

TEST_CASE("Island reconstruction retains source extrusion owners after its slice boundary",
          "[plugins][properties][freeze]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
#if defined(SLIC3R_TEST_STRICT_FREEZE)
    const StrictFreezeState strict_state;
#endif
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model,
                     Test::PerimeterPluginTests::perimeter_config({}));
    Steps::StepLayerHeightGeneration::run_step(Orchestrator::instance(), print);
    Steps::StepSlicing::run_step(Orchestrator::instance(), print);
    PrintObject *owners[2] = {&print.object(0), &print.mutable_auxiliary_object()};
    Layer *layers[3] = {&print.object(0).layer(0), nullptr, nullptr};
    for (size_t idx = 0; idx < 2; ++idx) {
        layer_handle *layer = object_add_auxiliary_layer(reinterpret_cast<object_handle *>(owners[idx]),
                                                       scale_i(0.2), scale_i(0.2), scale_i(0.1));
        REQUIRE(layer != nullptr);
        layers[idx + 1] = reinterpret_cast<Layer *>(layer);
        *reinterpret_cast<ExPolygons *>(layer_region_borrow_mutable_slices(layer_get_region(layer, 0))) =
            ExPolygons{rectangle_expolygon(0., 0., 10., 10.)};
    }
    for (Layer *layer : layers) {
        const slicing_step_t last = layer == layers[0] ? STEP_POST_SLICING : STEP_FINALIZE_SLICING;
        layer_handle *handle = reinterpret_cast<layer_handle *>(layer);
        print.set_running_step(last);
        layer_recompute_slices_and_islands_from_layer_regions(handle);
        REQUIRE(layer->islands().size() == 1);
        layer_recompute_slices_from_islands(handle);
        CHECK(layer->lslices() == ExPolygons{layer->island(0).get_slice()});
        layer_add_regions_to_islands(handle);
        const LayerSliceIsland *island = &layer->island(0);
        const ExPolygon slice = island->get_slice();
        const ExPolygons merged = layer->lslices();
        LayerRegionIsland &group = layer->island(0).get_or_add_region_island(layer->island(0).regions(), 0);
        const size_t group_count = layer->island(0).regions_islands().size();
#if defined(SLIC3R_TEST_STRICT_FREEZE)
        print.set_running_step(last == STEP_POST_SLICING ? STEP_PRE_PERIMETER : STEP_ORDERING);
        layer_recompute_slices_and_islands_from_layer_regions(handle);
        require_freeze_diagnostic(print.running_step(), "layer_recompute_slices_and_islands_from_layer_regions");
        print.set_running_step(last == STEP_POST_SLICING ? STEP_PRE_PERIMETER : STEP_ORDERING);
        layer_recompute_slices_from_islands(handle);
        require_freeze_diagnostic(print.running_step(), "layer_recompute_slices_from_islands");
        print.set_running_step(last == STEP_POST_SLICING ? STEP_PRE_PERIMETER : STEP_ORDERING);
        layer_add_regions_to_islands(handle);
        require_freeze_diagnostic(print.running_step(), "layer_add_regions_to_islands");
        CHECK(&layer->island(0) == island);
        CHECK(layer->island(0).get_slice() == slice);
        CHECK(layer->lslices() == merged);
        CHECK(layer->island(0).regions_islands().size() == group_count);
        CHECK(&layer->island(0).regions_islands().front() == &group);
#endif
        print.set_running_step(STEP_NONE);
    }
    layer_recompute_slices_and_islands_from_layer_regions(nullptr);
    layer_recompute_slices_from_islands(nullptr);
    layer_add_regions_to_islands(nullptr);
#if defined(SLIC3R_TEST_STRICT_FREEZE)
    CHECK_FALSE(Orchestrator::instance().is_plugin_cancelled());
    CHECK(Orchestrator::instance().consume_plugin_messages().empty());
#endif
}

#if defined(SLIC3R_TEST_STRICT_FREEZE)
TEST_CASE("Remaining source gates allow no-owner and idle capabilities without diagnostics",
          "[plugins][properties][freeze][strict-freeze]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const StrictFreezeState strict_state;
    Print print;
    std::shared_ptr<PrintObjectRegions> regions = std::make_shared<PrintObjectRegions>();
    regions->all_regions = print.mutable_auxiliary_object().shared_regions()->all_regions;
    DetachedPrintObject detached(print, regions, false);
    object_handle *object = reinterpret_cast<object_handle *>(&detached);
    layer_handle *layer = object_add_auxiliary_layer(object, scale_i(0.2), scale_i(0.2), scale_i(0.1));
    REQUIRE(layer != nullptr);
    *reinterpret_cast<ExPolygons *>(layer_region_borrow_mutable_slices(layer_get_region(layer, 0))) =
        ExPolygons{rectangle_expolygon(0., 0., 10., 10.)};
    // Reconstruction reads the live Print's resolution before this fixture is detached.
    layer_recompute_slices_and_islands_from_layer_regions(layer);
    layer_add_regions_to_islands(layer);
    detached.detach();
    print.set_running_step(STEP_GCODE);
    REQUIRE(object_get_config_mutable(object) != nullptr);
    layer_island_handle *island = layer_get_island_mutable(layer, 0);
    REQUIRE(island != nullptr);
    REQUIRE(layer_island_get_slice_mutable(island) != nullptr);
    layer_region_island_handle *group = layer_island_get_or_create_region_island(island, nullptr, 0, 0);
    REQUIRE(group != nullptr);
    CHECK(layer_island_get_region_island_mutable(island, 0) == group);
    REQUIRE(layer_region_island_get_mutable_extrusion(group, RAW_EXTRUSION_ROLE_PERIMETER) != nullptr);
    LayerSliceIsland standalone(rectangle_expolygon(0., 0., 10., 10.));
    REQUIRE(layer_island_get_slice_mutable(reinterpret_cast<layer_island_handle *>(&standalone)) != nullptr);
    LayerRegionIsland standalone_group({}, EXTRUDER_ID_UNKNOWN);
    REQUIRE(layer_region_island_get_mutable_extrusion(
        reinterpret_cast<layer_region_island_handle *>(&standalone_group), RAW_EXTRUSION_ROLE_GAP_FILL) != nullptr);
    print.set_running_step(STEP_NONE);
    PrintObject &owned = print.mutable_auxiliary_object();
    layer = object_add_auxiliary_layer(reinterpret_cast<object_handle *>(&owned), scale_i(0.2), scale_i(0.2), scale_i(0.1));
    REQUIRE(layer != nullptr);
    *reinterpret_cast<ExPolygons *>(layer_region_borrow_mutable_slices(layer_get_region(layer, 0))) =
        ExPolygons{rectangle_expolygon(0., 0., 10., 10.)};
    layer_recompute_slices_and_islands_from_layer_regions(layer);
    layer_add_regions_to_islands(layer);
    island = layer_get_island_mutable(layer, 0);
    REQUIRE(layer_island_get_slice_mutable(island) != nullptr);
    group = layer_island_get_or_create_region_island(island, nullptr, 0, 0);
    REQUIRE(group != nullptr);
    CHECK(layer_island_get_region_island_mutable(island, 0) == group);
    REQUIRE(layer_region_island_get_mutable_extrusion(group, RAW_EXTRUSION_ROLE_PERIMETER) != nullptr);
    CHECK_FALSE(Orchestrator::instance().is_plugin_cancelled());
    CHECK(Orchestrator::instance().consume_plugin_messages().empty());
}
#endif

TEST_CASE("PluginPropertyKey gives built-in and dynamic properties one access model",
          "[plugins][properties][property-key]")
{
    using slic3r_api::LayerSupportProperty;
    using slic3r_api::EPropertyOverhang;
    using slic3r_api::PluginProperties;
    using slic3r_api::PluginPropertyKey;
    using slic3r_api::StoredExtrusionEntity;

    const PluginPropertyKey<RuntimePropertyPayload> first_key =
        PluginPropertyKey<RuntimePropertyPayload>::register_dynamic(
            test_orchestrator_handle(), "tests.plugin-property-key.first");
    const PluginPropertyKey<RuntimePropertyPayload> repeated_key =
        PluginPropertyKey<RuntimePropertyPayload>::register_dynamic(
            test_orchestrator_handle(), "tests.plugin-property-key.first");
    const PluginPropertyKey<RuntimePropertyPayload> second_key =
        PluginPropertyKey<RuntimePropertyPayload>::register_dynamic(
            test_orchestrator_handle(), "tests.plugin-property-key.second");

    CHECK(first_key.type() == repeated_key.type());
    CHECK(first_key.type() != second_key.type());
    CHECK(first_key.orchestrator() == test_orchestrator_handle());

    // Generic data-tree storage uses the key without separately passing an id
    // or orchestrator at the point where the payload is accessed.
    PluginPropertyContainer native_properties;
    PluginProperties properties(reinterpret_cast<plugin_property_container_handle *>(&native_properties));
    first_key.get_or_add(properties).value = 17;
    second_key.get_or_add(properties).value = 29;
    REQUIRE(properties.get(first_key) != nullptr);
    REQUIRE(first_key.get(properties) != nullptr);
    REQUIRE(second_key.get(properties) != nullptr);
    CHECK(first_key.get(properties)->value == 17);
    CHECK(second_key.get(properties)->value == 29);
    REQUIRE(first_key.get_mutable(properties) != nullptr);
    first_key.get_mutable(properties)->generation = 3;
    CHECK(first_key.get(properties)->generation == 3);
    CHECK(first_key.has(properties));

    // A built-in payload uses the same operations; only construction of its
    // key differs because its numeric identity is part of the public ABI.
    CHECK(LayerSupportProperty::key.type() == PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT);
    CHECK(LayerSupportProperty::key.orchestrator() == nullptr);
    properties.get_or_add(LayerSupportProperty::key).interface_id = 42;
    REQUIRE(properties.get(LayerSupportProperty::key) != nullptr);
    REQUIRE(LayerSupportProperty::key.get(properties) != nullptr);
    CHECK(LayerSupportProperty::key.get(properties)->interface_id == 42);

    // Extrusion storage consumes the exact same dynamic key and therefore
    // cannot accidentally use another orchestrator during payload creation.
    PluginStorage storage;
    StoredExtrusionEntity extrusion(reinterpret_cast<storage_handle *>(&storage));
    extrusion.get_or_add(first_key).value = 61;
    REQUIRE(extrusion.get(first_key) != nullptr);
    CHECK(first_key.has(extrusion));
    REQUIRE(first_key.get(extrusion) != nullptr);
    CHECK(first_key.get(extrusion)->value == 61);
    REQUIRE(first_key.get_mutable(extrusion) != nullptr);
    first_key.get_mutable(extrusion)->generation = 8;
    CHECK(first_key.get(extrusion)->generation == 8);
    CHECK(first_key.remove(extrusion));
    CHECK_FALSE(first_key.has(extrusion));

    CHECK(EPropertyOverhang::key.type() == EXTRUSION_PROPERTY_TYPE_OVERHANG);
    CHECK(EPropertyOverhang::key.orchestrator() == nullptr);
    extrusion.get_or_add(EPropertyOverhang::key).start_distance_from_prev_layer = 0.25f;
    REQUIRE(extrusion.get(EPropertyOverhang::key) != nullptr);
    REQUIRE(EPropertyOverhang::key.get(extrusion) != nullptr);
    CHECK(EPropertyOverhang::key.get(extrusion)->start_distance_from_prev_layer == Approx(0.25f));

    // Extrusion-axis operations use the same built-in property mechanism. The
    // semantic operation remains separate from geometry and may therefore be
    // attached to an empty Retract or Unretract event leaf.
    CHECK(slic3r_api::EPropertyExtrusionAxis::key.type() ==
          EXTRUSION_PROPERTY_TYPE_EXTRUSION_AXIS);
    slic3r_api::EPropertyExtrusionAxis &axis =
        extrusion.get_or_add(slic3r_api::EPropertyExtrusionAxis::key);
    axis.retract_to(2.5, true);
    CHECK(axis.operation == C_EXTRUSION_AXIS_OPERATION_RETRACT_TO);
    CHECK(axis.value == Approx(2.5));
    CHECK(axis.toolchange == 1);

    CHECK(first_key.remove(properties));
    CHECK_FALSE(first_key.has(properties));
    CHECK(second_key.has(properties));

    // A stable name may never be rebound to a different binary payload.
    CHECK_THROWS_AS(
        PluginPropertyKey<IncompatibleRuntimePropertyPayload>::register_dynamic(
            test_orchestrator_handle(), "tests.plugin-property-key.first"),
        std::runtime_error);
}

TEST_CASE("Extrusion visitors resolve inherited dynamic properties by key",
          "[plugins][properties][property-key]")
{
    using slic3r_api::PluginPropertyKey;
    using slic3r_api::StoredExtrusionEntity;

    const PluginPropertyKey<RuntimePropertyPayload> property_key =
        PluginPropertyKey<RuntimePropertyPayload>::register_dynamic(
            test_orchestrator_handle(), "tests.plugin-property-key.visitor");

    PluginStorage storage;
    StoredExtrusionEntity root(reinterpret_cast<storage_handle *>(&storage));
    StoredExtrusionEntity child(reinterpret_cast<storage_handle *>(&storage));
    REQUIRE_FALSE(slic3r_api::is_invalid_index(root.append_child_move(child.mutable_view())));
    property_key.get_or_add(root).value = 73;

    RuntimePropertyVisitor visitor(property_key);
    visitor.traverse(root.mutable_view());
    CHECK(visitor.visited_value == 73);
}

TEST_CASE("PluginPropertyContainer stores small typed payloads safely",
          "[plugins][properties]")
{
    const slic3r_api::PluginPropertyKey<TestSurfacePayload> property_key =
        test_surface_payload_key();

    PluginPropertyContainer container;
    slic3r_api::PluginProperties properties = property_view(container);

    // A new payload is zero-initialized, so plugins can fill only the fields
    // they care about without reading uninitialized bytes first.
    TestSurfacePayload &payload = property_key.get_or_add(properties);
    CHECK(payload.priority == 0);
    CHECK(payload.layer_count == 0);

    payload.priority = 7;
    payload.layer_count = 3;

    const TestSurfacePayload *readback = property_key.get(properties);
    REQUIRE(readback != nullptr);
    CHECK(readback->priority == 7);
    CHECK(readback->layer_count == 3);

    // A copied container owns its own bytes. Later mutations on the copy should
    // not change the original payload carried by the source object.
    PluginPropertyContainer copy = container;
    slic3r_api::PluginProperties copy_properties = property_view(copy);
    property_key.get_or_add(copy_properties).priority = 11;
    CHECK(property_key.get(properties)->priority == 7);
    CHECK(property_key.get(copy_properties)->priority == 11);

    // Reusing a type with a different layout is rejected. This catches the most
    // dangerous plugin mistake: two modules agreeing on a name but disagreeing
    // on the binary struct stored under that name.
    void *wrong_layout = container.get_or_add_property_data_mutable(
        property_key.type(), sizeof(uint32_t), alignof(uint32_t));
    CHECK(wrong_layout == nullptr);
}

TEST_CASE("Surface plugin properties survive splits and protect merge checks",
          "[plugins][properties]")
{
    const slic3r_api::PluginPropertyKey<TestSurfacePayload> property_key =
        test_surface_payload_key();

    Surface source(stPosInternal | stDensSparse, rectangle_expolygon(0., 0., 10., 10.));
    TestSurfacePayload &source_payload = property_key.get_or_add(property_view(source));
    source_payload.priority = 4;
    source_payload.layer_count = 2;

    // Splitting a Surface creates new geometry but should keep the logical
    // metadata. A dense-infill plugin, for example, can attach a priority before
    // another plugin clips the area into several pieces.
    Surface split_piece(source, rectangle_expolygon(0., 0., 5., 10.));
    const TestSurfacePayload *split_payload = property_key.get(property_view(split_piece));
    REQUIRE(split_payload != nullptr);
    CHECK(split_payload->priority == 4);
    CHECK(split_payload->layer_count == 2);

    // Surfaces may be merged only when plugin metadata matches too. Otherwise a
    // cleanup pass could accidentally fuse two areas that a later plugin expects
    // to process with different parameters.
    Surface same_metadata(source, rectangle_expolygon(5., 0., 10., 10.));
    CHECK(surfaces_could_merge(split_piece, same_metadata));

    property_key.get_or_add(property_view(same_metadata)).priority = 9;
    CHECK_FALSE(surfaces_could_merge(split_piece, same_metadata));
}

TEST_CASE("Surface runtime ids track logical ownership across copy and move",
          "[plugins][properties][surface-id]")
{
    Surface source(stPosInternal | stDensSparse, rectangle_expolygon(0., 0., 10., 10.));
    REQUIRE(source.id() != 0);

    // A copied Surface is a new infill job with copied geometry and metadata.
    // It must therefore receive a fresh runtime id instead of aliasing the
    // source surface that may generate a different extrusion subtree.
    Surface copied(source);
    CHECK(copied.id() != 0);
    CHECK(copied.id() != source.id());

    // Copy assignment replaces the destination content but keeps destination
    // identity. This protects code that already holds a reference to that
    // Surface object while a collection rewrites its geometry in-place.
    Surface assigned(stPosTop | stDensSolid, rectangle_expolygon(20., 0., 30., 10.));
    const uint64_t assigned_id = assigned.id();
    assigned = source;
    CHECK(assigned.id() == assigned_id);
    CHECK(assigned.id() != source.id());

    // Move construction and move assignment transfer the logical surface job:
    // the same geometry/properties continue under a new C++ object, so the id
    // follows the moved content.
    Surface move_construct_source(stPosBottom | stDensSolid, rectangle_expolygon(40., 0., 50., 10.));
    const uint64_t move_construct_id = move_construct_source.id();
    Surface move_constructed(std::move(move_construct_source));
    CHECK(move_constructed.id() == move_construct_id);

    Surface move_assign_source(stPosInternal | stDensSparse, rectangle_expolygon(60., 0., 70., 10.));
    const uint64_t move_assign_source_id = move_assign_source.id();
    Surface move_assigned(stPosTop | stDensSolid, rectangle_expolygon(80., 0., 90., 10.));
    move_assigned = std::move(move_assign_source);
    CHECK(move_assigned.id() == move_assign_source_id);

    // Surface vectors reallocate frequently while clipping and rebuilding
    // collections. Reallocation must move surfaces, not copy them, otherwise a
    // completely ordinary push_back could silently change ids already used by
    // generated infill subtrees.
    std::vector<Surface> collection;
    collection.reserve(1);
    collection.emplace_back(stPosInternal | stDensSparse, rectangle_expolygon(100., 0., 110., 10.));
    const uint64_t first_id_before_reallocation = collection.front().id();
    collection.emplace_back(stPosInternal | stDensSparse, rectangle_expolygon(120., 0., 130., 10.));
    CHECK(collection.front().id() == first_id_before_reallocation);
}

TEST_CASE("Support auxiliary layers are recognized through the plugin data tree API",
          "[plugins][properties][auxiliary-layers]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Model model;
    Print print;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height", "0.2" },
        { "first_layer_height", "0.2" },
        { "support_material", "0" }
    });
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20}, print, model, config);

    REQUIRE(print.objects().size() == 1);
    PrintObject &object = print.object(0);
    object.clear_auxiliary_layers();

    /*
    Auxiliary layers are generic storage. A newly-created auxiliary layer is
    just a normal Layer until a support generator marks it with the built-in
    support property. This distinction matters because skirt, brim or wipe
    tower layers will use the same container later without becoming support.
    */
    Layer &plain_layer = object.add_auxiliary_layer(0, scale_i(0.2), scale_i(0.2));
    CHECK(plain_layer.get_property<LayerSupportProperty>() == nullptr);

    const object_handle *object_api_handle = reinterpret_cast<const object_handle *>(&object);
    REQUIRE(object_count_auxiliary_layer(object_api_handle) == 1);
    const layer_handle *plain_handle = object_get_auxiliary_layer(object_api_handle, 0);
    REQUIRE(plain_handle != nullptr);
    const plugin_property_container_handle *plain_properties = layer_get_properties(plain_handle);
    REQUIRE(plain_properties != nullptr);
    CHECK(plugin_property_has(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) == 0);
    CHECK(plugin_property_data_size(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) == 0);
    CHECK(plugin_property_data(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) == nullptr);
    CHECK(plugin_property_has(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_BRIM) == 0);
    CHECK(plugin_property_data_size(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_BRIM) == 0);
    CHECK(plugin_property_data(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_BRIM) == nullptr);
    CHECK(plugin_property_has(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_ADHESION) == 0);
    CHECK(plugin_property_data_size(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_ADHESION) == 0);
    CHECK(plugin_property_data(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_ADHESION) == nullptr);

    const slic3r_api::Object object_view(object_api_handle);
    REQUIRE(object_view.auxiliary_layer_count() == 1);
    CHECK(object_view.auxiliary_layer(0).properties().get(slic3r_api::LayerSupportProperty::key) == nullptr);
    CHECK(object_view.auxiliary_layer(0).properties().get(slic3r_api::LayerBrimProperty::key) == nullptr);
    CHECK(object_view.auxiliary_layer(0).properties().get(slic3r_api::LayerAdhesionProperty::key) == nullptr);

    LayerSupportProperty &support_property = plain_layer.get_or_add_property<LayerSupportProperty>();
    support_property.interface_id = 17;
    support_property.reserved = 0;
    REQUIRE(plain_layer.get_property<LayerSupportProperty>() != nullptr);
    CHECK(plain_layer.get_property<LayerSupportProperty>()->interface_id == 17);
    CHECK(plugin_property_has(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) != 0);
    REQUIRE(plugin_property_data_size(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) == sizeof(c_layer_support_property));
    const c_layer_support_property *raw_support_property =
        static_cast<const c_layer_support_property *>(plugin_property_data(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT));
    REQUIRE(raw_support_property != nullptr);
    CHECK(raw_support_property->interface_id == 17);

    const slic3r_api::LayerSupportProperty *view_support_property =
        object_view.auxiliary_layer(0).properties().get(slic3r_api::LayerSupportProperty::key);
    REQUIRE(view_support_property != nullptr);
    CHECK(view_support_property->interface_id == 17);

    LayerBrimProperty &brim_property = plain_layer.get_or_add_property<LayerBrimProperty>();
    brim_property.reserved = 0;
    REQUIRE(plain_layer.get_property<LayerBrimProperty>() != nullptr);
    CHECK(plugin_property_has(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_BRIM) != 0);
    REQUIRE(plugin_property_data_size(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_BRIM) == sizeof(c_layer_brim_property));
    const c_layer_brim_property *raw_brim_property =
        static_cast<const c_layer_brim_property *>(plugin_property_data(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_BRIM));
    REQUIRE(raw_brim_property != nullptr);

    const slic3r_api::LayerBrimProperty *view_brim_property =
        object_view.auxiliary_layer(0).properties().get(slic3r_api::LayerBrimProperty::key);
    REQUIRE(view_brim_property != nullptr);
    CHECK(slic3r_api::layer_is_brim(object_view.auxiliary_layer(0)));

    /*
    Brim and skirt now share one adhesion marker instead of creating one
    built-in property per adhesion feature. The payload is small but explicit:
    kind selects brim/skirt, and flags mark special storage such as
    first-layer-only skirt-brim output.
    */
    LayerAdhesionProperty &adhesion_property = plain_layer.get_or_add_property<LayerAdhesionProperty>();
    adhesion_property.kind = RAW_LAYER_ADHESION_KIND_SKIRT;
    adhesion_property.flags = RAW_LAYER_ADHESION_FLAG_FIRST_LAYER_ONLY;
    REQUIRE(plain_layer.get_property<LayerAdhesionProperty>() != nullptr);
    CHECK(plain_layer.get_property<LayerAdhesionProperty>()->is_skirt());
    CHECK(plain_layer.get_property<LayerAdhesionProperty>()->is_first_layer_only());
    CHECK(plugin_property_has(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_ADHESION) != 0);
    REQUIRE(plugin_property_data_size(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_ADHESION) == sizeof(c_layer_adhesion_property));
    const c_layer_adhesion_property *raw_adhesion_property =
        static_cast<const c_layer_adhesion_property *>(plugin_property_data(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_ADHESION));
    REQUIRE(raw_adhesion_property != nullptr);
    CHECK(raw_adhesion_property->kind == RAW_LAYER_ADHESION_KIND_SKIRT);
    CHECK(raw_adhesion_property->flags == RAW_LAYER_ADHESION_FLAG_FIRST_LAYER_ONLY);

    const slic3r_api::LayerAdhesionProperty *view_adhesion_property =
        object_view.auxiliary_layer(0).properties().get(slic3r_api::LayerAdhesionProperty::key);
    REQUIRE(view_adhesion_property != nullptr);
    CHECK(view_adhesion_property->is_skirt());
    CHECK(view_adhesion_property->is_first_layer_only());
    CHECK(slic3r_api::layer_adhesion_property(object_view.auxiliary_layer(0)) == view_adhesion_property);
    CHECK(slic3r_api::layer_has_adhesion_kind(
        object_view.auxiliary_layer(0), RAW_LAYER_ADHESION_KIND_SKIRT));
    CHECK_FALSE(slic3r_api::layer_is_brim(object_view.auxiliary_layer(0)));
    CHECK_FALSE(slic3r_api::layer_is_normal_skirt(object_view.auxiliary_layer(0)));
    CHECK(slic3r_api::layer_is_skirt_first_layer_only(object_view.auxiliary_layer(0)));
}
