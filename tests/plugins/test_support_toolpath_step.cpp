#include <catch2/catch.hpp>

#include <algorithm>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_toolpath.h"
#include "libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepSupportToolpath.hpp"
#include "libslic3r/TriangleMesh.hpp"

/* Support toolpath tests register process-lifetime recording providers before
   configuration finalization. Scoped activation keeps them out of other tests.
   Calls are recorded under a mutex because objects may run in parallel; each
   role still completes before the next selected role starts. */

namespace {
using namespace Slic3r;

struct RecordedCall {
    std::string id;
    const object_handle *object;
    size_t matched_surfaces;
};

std::mutex call_mutex;
std::vector<RecordedCall> calls;

class SupportToolpathProbe final : public slic3r_api::PluginBase
{
public:
    // Set the provider's id, role group and priority using names that outlive it.
    SupportToolpathProbe(orchestrator_handle *orchestrator, const char *id,
                        const char *group, int32_t priority)
        : PluginBase(orchestrator), m_id(id), m_group(group), m_priority(priority) {}

    // This provider's plugin id.
    const char *id() const { return m_id; }

private:
    // This provider's plugin id.
    const char *id_impl() const noexcept override { return m_id; }
    // Assign the provider to the per-object support toolpath step.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT_TOOLPATH; }
    // Return the role group used by the generated selector.
    const char *exclusive_group_impl() const noexcept override { return m_group; }
    // Declare no dependencies on other plugin ids.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Return the order used between selected role algorithms.
    int32_t priority_impl() const noexcept override { return m_priority; }
    // Find sparse support surfaces and record this object's callback.
    void run_impl(const plugin_run_context *context) const override;

    const char *m_id;
    const char *m_group;
    int32_t m_priority;
};

// Test providers shared by registration and scoped activation.
const std::vector<SupportToolpathProbe *> &probes();
// Run the dispatcher under its real freeze step; reset that step even if a plugin fails.
void run_toolpaths(Print &print);

// Add a two-point extrusion for each sparse support surface and record the call; missing payloads or failed publication throw.
void SupportToolpathProbe::run_impl(const plugin_run_context *context) const
{
    const run_ctx_support_toolpath *payload = plugin_ctx_as_support_toolpath(context);
    if (payload == nullptr || payload->print == nullptr || payload->object == nullptr)
        throw std::runtime_error("Missing support toolpath object payload");

    size_t matched = 0;
    if (std::string(m_group) == "support_toolpath_sparse") {
        for (uint32_t layer_idx = 0; layer_idx < object_count_auxiliary_layer(payload->object); ++layer_idx) {
            const layer_handle *layer = object_get_auxiliary_layer(payload->object, layer_idx);
            if (!plugin_property_has(layer_get_properties(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT))
                continue;
            for (uint32_t island_idx = 0; island_idx < layer_count_island(layer); ++island_idx) {
                const layer_island_handle *island = layer_get_island(layer, island_idx);
                layer_island_handle *mutable_island = layer_get_island_mutable(
                    object_get_auxiliary_layer(payload->object, layer_idx), island_idx);
                for (uint32_t region_idx = 0; region_idx < layer_island_count_region_island(island); ++region_idx) {
                    layer_region_island_handle *region = layer_island_get_region_island_mutable(mutable_island, region_idx);
                    for (uint32_t surface_idx = 0; surface_idx < layer_region_island_count_fill_surface(region); ++surface_idx) {
                        const raw_surface_type type = surface_get_type(layer_region_island_get_fill_surface(region, surface_idx));
                        if ((type & (RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_DENS_SPARSE)) !=
                            (RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_DENS_SPARSE))
                            continue;
                        extrusion_entity_handle *root = layer_region_island_get_mutable_extrusion(region, RAW_EXTRUSION_ROLE_SUPPORT);
                        slic3r_api::StoredExtrusionEntity leaf(context->plugin_storage);
                        const c_point points[] = {{scale_i(1.), scale_i(1.)}, {scale_i(2.), scale_i(1.)}};
                        // Build in scratch storage, then move the content into one
                         // source child. The scratch wrapper then frees the empty temporary.
                        if (!extrusion_polyline_set_points(leaf.mutable_handle(), points, 2) ||
                            extrusion_insert_child_move(root, extrusion_child_count(root), leaf.mutable_handle()) == EXTRUSION_INDEX_INVALID)
                            throw std::runtime_error("Cannot publish test support extrusion");
                        ++matched;
                    }
                }
            }
        }
    }
    const std::lock_guard<std::mutex> lock(call_mutex);
    calls.push_back({m_id, payload->object, matched});
}

// Create the test providers once and share the same instances across tests.
const std::vector<SupportToolpathProbe *> &probes()
{
    orchestrator_handle *orchestrator = reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance());
    static SupportToolpathProbe sparse(orchestrator, "test.support_toolpath.sparse", "support_toolpath_sparse", 0);
    static SupportToolpathProbe bottom(orchestrator, "test.support_toolpath.bottom", "support_toolpath_bottom_interface", 100);
    static SupportToolpathProbe top(orchestrator, "test.support_toolpath.top", "support_toolpath_top_interface", 200);
    static SupportToolpathProbe alternate(orchestrator, "test.support_toolpath.alternate", "support_toolpath_top_interface", 50);
    static const std::vector<SupportToolpathProbe *> providers {&sparse, &bottom, &top, &alternate};
    return providers;
}

class ScopedToolpathProbes
{
public:
    // Activate all test providers and start with an empty call log.
    ScopedToolpathProbes()
    {
        Test::Plugins::ensure_plugin_test_runtime_initialized();
        calls.clear();
        for (SupportToolpathProbe *probe : probes())
            REQUIRE(Orchestrator::instance().set_plugin_active(probe->id(), true));
    }
    // Disable the test providers so they do not run in other tests.
    ~ScopedToolpathProbes()
    {
        for (SupportToolpathProbe *probe : probes())
            Orchestrator::instance().set_plugin_active(probe->id(), false);
    }
};

// Set the running step for source-edit checks, run the algorithms, and reset the step even if a plugin throws.
void run_toolpaths(Print &print)
{
    struct RunningStep {
        Print &print;
        // Restore idle access after the dispatcher finishes or throws.
        ~RunningStep() { print.set_running_step(STEP_NONE); }
    };
    const RunningStep reset {print};
    print.set_running_step(STEP_SUPPORT_TOOLPATH);
    Steps::StepSupportToolpath::prepare(print);
    Steps::StepSupportToolpath::run_step(Orchestrator::instance(), print);
}
} // namespace

namespace Slic3r::Test::Plugins {
// Register and activate the test providers before selector settings are finalized; rejected registration or activation throws.
void register_support_toolpath_test_providers(orchestrator_handle *orchestrator)
{
    Orchestrator &host = *reinterpret_cast<Orchestrator *>(orchestrator);
    for (SupportToolpathProbe *probe : probes()) {
        if (!host.register_plugin(probe->c_instance()) || !host.set_plugin_active(probe->id(), true))
            throw std::runtime_error("Cannot register support toolpath test provider");
    }
}
} // namespace Slic3r::Test::Plugins

TEST_CASE("Support toolpath roles run once per object in priority order", "[plugins][support-toolpath]")
{
    const ScopedToolpathProbes activation;
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_2x20x10}, print, model);
    REQUIRE(print.objects().size() == 2);
    REQUIRE(Steps::get_exclusive_steps().count(STEP_SUPPORT_TOOLPATH) == 0);
    REQUIRE(Orchestrator::instance().set_plugin_active(probes()[3]->id(), false));
    run_toolpaths(print);
    REQUIRE(calls.size() == 6);
    for (size_t role = 0; role < 3; ++role) {
        CHECK(calls[role * 2].id == probes()[role]->id());
        CHECK(calls[role * 2 + 1].id == probes()[role]->id());
        CHECK(calls[role * 2].object != calls[role * 2 + 1].object);
        for (const PrintObject &object : print.objects()) {
            const object_handle *handle = reinterpret_cast<const object_handle *>(&object);
            CHECK(std::count_if(calls.begin(), calls.end(), [role, handle](const RecordedCall &call) {
                return call.id == probes()[role]->id() && call.object == handle;
            }) == 1);
        }
    }
}

TEST_CASE("Support toolpath role selectors choose only one top algorithm", "[plugins][support-toolpath]")
{
    const ScopedToolpathProbes activation;
    const size_t selected = GENERATE(size_t(2), size_t(3));
    const std::string key = Steps::plugin_exclusive_group_option_key(
        Orchestrator::instance(), STEP_SUPPORT_TOOLPATH, "support_toolpath_top_interface");
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    REQUIRE(config.option(key) != nullptr);
    bool found_group = false;
    for (const Steps::StepExclusivePluginGroup &group : Steps::active_exclusive_plugin_groups(Orchestrator::instance())) {
        if (group.group.step == STEP_SUPPORT_TOOLPATH && group.group.group_id == "support_toolpath_top_interface") {
            found_group = true;
            CHECK(group.group.option_def.category == RAW_OPTION_CATEGORY_SUPPORT);
            CHECK(group.group.option_def.option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
        }
    }
    REQUIRE(found_group);
    config.set_deserialize_strict({{key, std::string(probes()[selected]->id())}});
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    run_toolpaths(print);
    REQUIRE(calls.size() == 3);
    CHECK(calls[0].id == probes()[0]->id());
    CHECK(calls[selected == 2 ? 1 : 2].id == probes()[1]->id());
    CHECK(calls[selected == 2 ? 2 : 1].id == probes()[selected]->id());
}

// Publish typed auxiliary surfaces through the API before checking the sparse algorithm's extrusion.
TEST_CASE("Sparse support toolpath reads its surface and publishes a source extrusion", "[plugins][support-toolpath]")
{
    const ScopedToolpathProbes activation;
    REQUIRE(Orchestrator::instance().set_plugin_active(probes()[3]->id(), false));
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
    PrintObject &object = print.object(0);
    const ExPolygon area(Polygon({Point(0, 0), Point(scale_i(4.), 0),
                                  Point(scale_i(4.), scale_i(4.)), Point(0, scale_i(4.))}));
    const ExPolygons subject {area};
    PluginStorage storage;
    const slic3r_api::AuxiliaryLayerBuildResult built = slic3r_api::build_auxiliary_layer_regions_from_subject(
        reinterpret_cast<storage_handle *>(&storage), slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::Object(reinterpret_cast<const object_handle *>(&object)),
        slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&subject)),
        scale_i(0.2), scale_i(0.2), scale_i(0.1));
    REQUIRE(built.created);
    layer_handle *layer = object_get_auxiliary_layer(reinterpret_cast<const object_handle *>(&object), 0);
    REQUIRE(plugin_property_get_or_add_data_mutable(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()), layer_get_properties_mutable(layer),
        PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) != nullptr);
    REQUIRE(layer_count_island(layer) == 1);
    layer_region_island_handle *region = layer_island_get_or_create_region_island(
        layer_get_island_mutable(layer, 0), nullptr, 0, 0);
    REQUIRE(region != nullptr);
    slic3r_api::StoredSurfaceCollection surfaces(reinterpret_cast<storage_handle *>(&storage));
    const slic3r_api::ExPolygon surface_area(reinterpret_cast<const expolygon_handle *>(&area));
    surfaces.append(surface_area, slic3r_api::srf_type().internal().sparse().support().raw());
    surfaces.append(surface_area, slic3r_api::srf_type().top().solid().support().raw());
    REQUIRE(slic3r_api::LayerRegionIsland(region).set_fill_surfaces(&surfaces));
    CHECK(surfaces.empty());
    run_toolpaths(print);
    REQUIRE(calls.size() == 3);
    CHECK(calls.front().matched_surfaces == 1);
    const extrusion_entity_handle *root = layer_region_island_get_extrusion(region, RAW_EXTRUSION_ROLE_SUPPORT);
    REQUIRE(root != nullptr);
    REQUIRE(extrusion_child_count(root) == 1);
    CHECK(extrusion_polyline_point_count(extrusion_child(root, 0)) == 2);
    CHECK(layer_region_island_count_fill_surface(region) == 2);
    Steps::StepSupportToolpath::clear(print);
    CHECK(object.auxiliary_layers().size() == 1);
    CHECK(extrusion_child_count(root) == 1);
}

TEST_CASE("Support toolpath dispatch without active providers does nothing", "[plugins][support-toolpath]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    calls.clear();
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
    run_toolpaths(print);
    CHECK(calls.empty());
    CHECK(print.object(0).auxiliary_layers().empty());
}

TEST_CASE("Support toolpath surfaces are published through the island mutator", "[plugins][support-toolpath][surface]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({make_cube(4., 4., 0.4)}, print, model);
    print.process();
    PrintObject &object = print.object(0);
    REQUIRE_FALSE(object.layers().empty());
    REQUIRE_FALSE(object.layer(0).islands().empty());
    REQUIRE_FALSE(object.layer(0).island(0).regions_islands().empty());
    LayerRegionIsland &object_region = object.layer(0).island(0).regions_island(0);
    const size_t original_count = object_region.fill_surfaces().size();
    REQUIRE(original_count > 0);
    const SurfaceType original_type = object_region.fill_surfaces().at(0).surface_type;

    const ExPolygon area(Polygon({Point(0, 0), Point(scale_i(4.), 0),
                                  Point(scale_i(4.), scale_i(4.)), Point(0, scale_i(4.))}));
    const ExPolygons subject {area};
    PluginStorage storage;
    const slic3r_api::AuxiliaryLayerBuildResult built = slic3r_api::build_auxiliary_layer_regions_from_subject(
        reinterpret_cast<storage_handle *>(&storage), slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::Object(reinterpret_cast<const object_handle *>(&object)),
        slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&subject)),
        scale_i(0.2), scale_i(0.2), scale_i(0.1));
    REQUIRE(built.created);
    layer_handle *layer = object_get_auxiliary_layer(reinterpret_cast<const object_handle *>(&object), 0);
    layer_region_island_handle *region = layer_island_get_or_create_region_island(
        layer_get_island_mutable(layer, 0), nullptr, 0, 0);
    REQUIRE(region != nullptr);
    slic3r_api::StoredSurfaceCollection surfaces(reinterpret_cast<storage_handle *>(&storage));
    const slic3r_api::ExPolygon surface_area(reinterpret_cast<const expolygon_handle *>(&area));
    const raw_surface_type type = slic3r_api::srf_type().internal().sparse().support().raw();

    struct RunningStep {
        Print &print;
        // Restore idle access even if a test assertion stops the loop.
        ~RunningStep() { print.set_running_step(STEP_NONE); }
    } reset {print};
    for (slicing_step_t step : {STEP_SUPPORT, STEP_SUPPORT_TOOLPATH, STEP_FINALIZE_SLICING, STEP_NONE}) {
        print.set_running_step(step);
        surfaces.append(surface_area, type);
        REQUIRE(layer_region_island_set_fill_surfaces(region, surfaces.mutable_handle()) == 1);
        CHECK(surfaces.empty());
        CHECK(layer_region_island_count_fill_surface(region) == 1);
        CHECK(surface_get_type(layer_region_island_get_fill_surface(region, 0)) == type);
    }
    REQUIRE(layer_region_island_set_fill_surfaces(region, nullptr) == 1);
    CHECK(layer_region_island_count_fill_surface(region) == 0);
    surfaces.append(surface_area, type);
    CHECK(layer_region_island_set_fill_surfaces(reinterpret_cast<layer_region_island_handle *>(&object_region),
                                              surfaces.mutable_handle()) == 0);
    CHECK(object_region.fill_surfaces().size() == original_count);
    CHECK(object_region.fill_surfaces().at(0).surface_type == original_type);
    CHECK(surfaces.size() == 1);
    CHECK(layer_region_island_set_fill_surfaces(nullptr, surfaces.mutable_handle()) == 0);
    CHECK(surfaces.size() == 1);
}
