
#include <catch2/catch.hpp>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"

#include <array>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerIslandAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/Api/plugin/c/slic3r_extrusion_entity.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_perimeter.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_post_infill.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_post_perimeter.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_pre_perimeter.h"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepPostInfillGeneration.hpp"
#include "libslic3r/Steps/StepPostPerimeterGeneration.hpp"
#include "libslic3r/Steps/StepPrepareForPeriemters.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

/* Exercise plugin lifecycle, mutable step outputs and direct perimeter publication.
   Recording plugins run on small sliced prints and inspect output after callbacks
   return. Direct publication replaces a whole bucket, retains marked source
   scopes and child handles, and removes cleared buckets. Source destruction must
   not invalidate published owned text. This callback service is tested separately
   from the region-group traversal that computes perimeter and fill domains.

       TEST_CASE -> public step runner -> recording_run()
           |-- record callbacks, object indexes and step payloads
           |-- inspect post-perimeter/post-infill mutable outputs
           `-- check_direct_publication()
               `-- set_region_island_extrusion(): replace, donate children, then clear
*/

namespace {
using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

struct RecordedEvent
{
    std::string plugin_id;
    std::string callback;
    slicing_step_t context_step = STEP_NONE;
    uint32_t run_count = 0;
    const print_handle *print = nullptr;
    const object_handle *object = nullptr;
    size_t object_idx = size_t(-1);
    size_t object_count = 0;
};

struct RecordingPluginState
{
    const char *id = nullptr;
    slicing_step_t step = STEP_NONE;
    int32_t priority = 0;
    const char *exclusive_group = nullptr;
    const char *exclusive_group_label = "";
    const char *exclusive_group_tooltip = "";
    std::vector<RecordedEvent> *events = nullptr;
    std::mutex *mutex = nullptr;
    bool mutate_post_perimeter_outputs = false;
    bool mutate_post_infill_outputs = false;
    bool saw_mutable_extrusion = false;
    bool saw_post_infill_region_island = false;
    bool created_empty_gap_fill_root = false;
    bool changed_fill_areas = false;
    bool rejected_perimeter_bucket = false;
    int32_t group_priority = 100;
    size_t direct_publications = 0;
};

// Exercise replacement and clearing through the generator's public callback on its first island.
void check_direct_publication(RecordingPluginState &state, const plugin_run_context *run_ctx);
// Record one plugin run and perform the mutable-output checks selected by the test state.
void recording_run(void *plugin_ctx, const plugin_run_context *run_ctx);

RecordingPluginState g_direct_publisher = {"test.perimeter.direct_publication", STEP_PERIMETER, 0};

RecordingPluginState g_pre_first  = {"test.pre_perimeter.first", STEP_PRE_PERIMETER, -10};
RecordingPluginState g_pre_second = {"test.pre_perimeter.second", STEP_PRE_PERIMETER, 20};
RecordingPluginState g_pre_inactive = {"test.pre_perimeter.inactive", STEP_PRE_PERIMETER, 0};
RecordingPluginState g_pre_group_first = {
    "test.pre_perimeter.group.first",
    STEP_PRE_PERIMETER,
    -5,
    "test.pre_perimeter.exclusive_group",
    "Test exclusive pre-perimeter group",
    "Choose one test pre-perimeter plugin."
};
RecordingPluginState g_pre_group_second = {
    "test.pre_perimeter.group.second",
    STEP_PRE_PERIMETER,
    -5,
    "test.pre_perimeter.exclusive_group",
    "Second label should not win",
    "Second tooltip should not win."
};
RecordingPluginState g_pre_legacy_base = {
    "test.pre_perimeter.legacy_base",
    STEP_PRE_PERIMETER,
    -3
};
RecordingPluginState g_pre_legacy_alternative = {
    "test.pre_perimeter.legacy_alternative",
    STEP_PRE_PERIMETER,
    -3,
    "test.pre_perimeter.legacy_base",
    "Legacy replacement group",
    "Choose between a legacy plugin and one of its alternatives."
};
RecordingPluginState g_post_first = {"test.post_perimeter.first", STEP_POST_PERIMETER, -10};
RecordingPluginState g_post_second = {"test.post_perimeter.second", STEP_POST_PERIMETER, 20};
RecordingPluginState g_post_inactive = {"test.post_perimeter.inactive", STEP_POST_PERIMETER, 0};
RecordingPluginState g_post_mutator = {"test.post_perimeter.mutator", STEP_POST_PERIMETER, 0};
RecordingPluginState g_post_infill_first = {"test.post_infill.first", STEP_POST_INFILL, -10};
RecordingPluginState g_post_infill_second = {"test.post_infill.second", STEP_POST_INFILL, 20};
RecordingPluginState g_post_infill_inactive = {"test.post_infill.inactive", STEP_POST_INFILL, 0};
RecordingPluginState g_post_infill_mutator = {"test.post_infill.mutator", STEP_POST_INFILL, 0};

RecordingPluginState *const g_recording_plugins[] = {
    &g_pre_first,
    &g_pre_second,
    &g_pre_inactive,
    &g_pre_group_first,
    &g_pre_group_second,
    &g_pre_legacy_base,
    &g_pre_legacy_alternative,
    &g_post_first,
    &g_post_second,
    &g_post_inactive,
    &g_post_mutator,
    &g_post_infill_first,
    &g_post_infill_second,
    &g_post_infill_inactive,
    &g_post_infill_mutator
};
// Register permanent borrowed test objects and copy their group descriptions into the application without changing print data.
// Rejected registrations fail assertions or throw; descriptions become application-owned.
void register_recording_plugins();
// Borrow the complete function table, including the test object's preference function; no state changes or failures occur.
const plugin_vtable *recording_vtable();
// Ignore borrowed initialization arguments and return success without state changes or failure.
int32_t recording_initialize(void *, storage_handle *);
// Return one test state's copied group preference without changing state or transferring ownership; the call cannot fail.
int32_t recording_group_priority(void *plugin_ctx);

const_strings_t recording_get_dependencies(void *)
{
    const_strings_t out = {};
    return out;
}

const char *recording_get_id(void *plugin_ctx)
{
    return static_cast<RecordingPluginState *>(plugin_ctx)->id;
}

const char *recording_get_name(void *plugin_ctx)
{
    return static_cast<RecordingPluginState *>(plugin_ctx)->id;
}

const char *recording_get_description(void *)
{
    return "";
}

const char *recording_get_exclusive_group(void *plugin_ctx)
{
    return static_cast<RecordingPluginState *>(plugin_ctx)->exclusive_group;
}

slicing_step_t recording_get_step(void *plugin_ctx)
{
    return static_cast<RecordingPluginState *>(plugin_ctx)->step;
}

int32_t recording_get_priority(void *plugin_ctx)
{
    return static_cast<RecordingPluginState *>(plugin_ctx)->priority;
}

int32_t recording_used_config_keys(void *, raw_used_config_key *)
{
    return 0;
}

int32_t recording_defined_config_keys(void *, const char **)
{
    return 0;
}

int32_t recording_initialize(void *, storage_handle *) { return PLUGIN_ERROR_OK; }

void fill_payload_event(const plugin_run_context *run_ctx, RecordedEvent &event)
{
    if (run_ctx == nullptr)
        return;

    event.context_step = run_ctx->step;
    const plugin_host_context *host_context = static_cast<const plugin_host_context *>(run_ctx->host_context);
    if (host_context != nullptr) {
        event.object_idx = host_context->object_idx;
        event.object_count = host_context->object_count;
    }

    if (run_ctx->step == STEP_PRE_PERIMETER) {
        const run_ctx_prepare_for_perimeters *payload = plugin_ctx_as_prepare_for_perimeters(run_ctx);
        if (payload != nullptr) {
            event.print = payload->print;
            event.object = payload->object;
        }
    } else if (run_ctx->step == STEP_POST_PERIMETER) {
        const run_ctx_post_perimeter_generation *payload = plugin_ctx_as_post_perimeter_generation(run_ctx);
        if (payload != nullptr) {
            event.print = payload->print;
            event.object = payload->object;
        }
    } else if (run_ctx->step == STEP_POST_INFILL) {
        const run_ctx_post_infill_generation *payload = plugin_ctx_as_post_infill_generation(run_ctx);
        if (payload != nullptr) {
            event.print = payload->print;
            event.object = payload->object;
        }
    }
}

void record_event(RecordingPluginState &state, RecordedEvent event)
{
    if (state.events == nullptr || state.mutex == nullptr)
        return;

    event.plugin_id = state.id;
    std::lock_guard<std::mutex> lock(*state.mutex);
    state.events->push_back(std::move(event));
}

void mutate_post_perimeter_outputs(RecordingPluginState &state, const plugin_run_context *run_ctx)
{
    const run_ctx_post_perimeter_generation *payload = plugin_ctx_as_post_perimeter_generation(run_ctx);
    if (payload == nullptr || payload->object == nullptr ||
        payload->get_region_island_mutable_extrusion == nullptr ||
        payload->set_island_fill_areas == nullptr ||
        payload->set_island_fill_free_areas == nullptr)
        return;

    const uint32_t layer_count = object_count_layer(payload->object);
    for (uint32_t layer_idx = 0; layer_idx < layer_count; ++layer_idx) {
        const layer_handle *layer = object_get_layer(payload->object, layer_idx);
        const uint32_t island_count = layer_count_island(layer);
        for (uint32_t island_idx = 0; island_idx < island_count; ++island_idx) {
            const layer_island_handle *island = layer_get_island(layer, island_idx);
            if (payload->set_island_fill_areas(island, nullptr) != 0 &&
                payload->set_island_fill_free_areas(island, nullptr) != 0)
                state.changed_fill_areas = true;

            const uint32_t region_island_count = layer_island_count_region_island(island);
            for (uint32_t region_island_idx = 0; region_island_idx < region_island_count; ++region_island_idx) {
                const layer_region_island_handle *region_island =
                    layer_island_get_region_island(island, region_island_idx);
                extrusion_entity_handle *root =
                    payload->get_region_island_mutable_extrusion(region_island, RAW_EXTRUSION_ROLE_PERIMETER);
                if (root != nullptr &&
                    extrusion_set_flags(root, extrusion_flags(root) | RAW_EXTRUSION_FLAG_REVERSIBLE) != 0)
                    state.saw_mutable_extrusion = true;
            }
        }
    }
}

void mutate_post_infill_outputs(RecordingPluginState &state, const plugin_run_context *run_ctx)
{
    const run_ctx_post_infill_generation *payload = plugin_ctx_as_post_infill_generation(run_ctx);
    if (payload == nullptr || payload->object == nullptr ||
        payload->get_region_island_mutable_extrusion == nullptr)
        return;

    const uint32_t layer_count = object_count_layer(payload->object);
    for (uint32_t layer_idx = 0; layer_idx < layer_count; ++layer_idx) {
        const layer_handle *layer = object_get_layer(payload->object, layer_idx);
        const uint32_t island_count = layer_count_island(layer);
        for (uint32_t island_idx = 0; island_idx < island_count; ++island_idx) {
            const layer_island_handle *island = layer_get_island(layer, island_idx);

            std::vector<const layer_region_handle *> region_handles;
            const uint32_t region_count = layer_island_count_region(island);
            region_handles.reserve(region_count);
            for (uint32_t region_idx = 0; region_idx < region_count; ++region_idx) {
                const layer_region_handle *region = layer_island_get_region(island, region_idx);
                if (region != nullptr)
                    region_handles.push_back(region);
            }

            // STEP_POST_INFILL now uses the generic data-tree mutation to
            // create or retrieve the destination LayerRegionIsland. The root
            // returned below is still the normal infill bucket of that
            // destination group.
            layer_region_island_handle *destination_region_island =
                layer_island_get_or_create_region_island(
                    const_cast<layer_island_handle *>(island),
                    region_handles.empty() ? nullptr : region_handles.data(),
                    uint32_t(region_handles.size()),
                    0);
            if (destination_region_island != nullptr) {
                state.saw_post_infill_region_island = true;

                extrusion_entity_handle *root =
                    payload->get_region_island_mutable_extrusion(
                        destination_region_island, RAW_EXTRUSION_ROLE_INTERNAL_INFILL);
                if (root != nullptr &&
                    extrusion_set_flags(root, RAW_EXTRUSION_FLAG_REVERSIBLE) != 0)
                    state.saw_mutable_extrusion = true;

                // Valid post-infill roles create an empty root when missing.
                // The step cleanup should remove that empty bucket after the
                // plugin returns, so probing remains harmless.
                if (payload->get_region_island_mutable_extrusion(
                        destination_region_island, RAW_EXTRUSION_ROLE_GAP_FILL) != nullptr)
                    state.created_empty_gap_fill_root = true;
            }

            const uint32_t region_island_count = layer_island_count_region_island(island);
            for (uint32_t region_island_idx = 0; region_island_idx < region_island_count; ++region_island_idx) {
                const layer_region_island_handle *region_island =
                    layer_island_get_region_island(island, region_island_idx);

                // STEP_POST_INFILL deliberately exposes only infill-owned
                // buckets. Perimeters are present in this test, but the
                // post-infill callback must refuse them so plugin authors do
                // not accidentally edit geometry owned by another step.
                if (payload->get_region_island_mutable_extrusion(
                        region_island, RAW_EXTRUSION_ROLE_PERIMETER) == nullptr)
                    state.rejected_perimeter_bucket = true;
            }
        }
    }
}

void recording_setup(void *plugin_ctx, const plugin_run_context *run_ctx, uint32_t run_count)
{
    RecordingPluginState &state = *static_cast<RecordingPluginState *>(plugin_ctx);
    RecordedEvent event;
    event.callback = "setup";
    event.run_count = run_count;
    fill_payload_event(run_ctx, event);
    record_event(state, std::move(event));
}

void recording_setup_run(void *plugin_ctx, const plugin_run_context *run_ctx)
{
    RecordingPluginState &state = *static_cast<RecordingPluginState *>(plugin_ctx);
    RecordedEvent event;
    event.callback = "setup_run";
    fill_payload_event(run_ctx, event);
    record_event(state, std::move(event));
}

void recording_run(void *plugin_ctx, const plugin_run_context *run_ctx)
{
    RecordingPluginState &state = *static_cast<RecordingPluginState *>(plugin_ctx);
    RecordedEvent event;
    event.callback = "run";
    fill_payload_event(run_ctx, event);
    record_event(state, std::move(event));
    if (state.mutate_post_perimeter_outputs)
        mutate_post_perimeter_outputs(state, run_ctx);
    if (state.mutate_post_infill_outputs)
        mutate_post_infill_outputs(state, run_ctx);
    if (state.step == STEP_PERIMETER && state.direct_publications == 0)
        check_direct_publication(state, run_ctx);
}

int32_t recording_group_priority(void *plugin_ctx)
{
    return static_cast<RecordingPluginState *>(plugin_ctx)->group_priority;
}

const plugin_vtable *recording_vtable()
{
    static const plugin_vtable vt = {
        sizeof(plugin_vtable),
        &recording_get_id,
        &recording_get_name,
        &recording_get_description,
        &recording_get_exclusive_group,
        &recording_get_step,
        &recording_get_dependencies,
        &recording_get_priority,
        &recording_used_config_keys,
        &recording_defined_config_keys,
        &recording_initialize,
        &recording_setup,
        &recording_setup_run,
        &recording_run,
        nullptr,
        &recording_group_priority
    };
    return &vt;
}

void register_recording_plugins()
{
    Orchestrator &orchestrator = Orchestrator::instance();
    Orchestrator::PluginRegistrationScope scope = orchestrator.plugin_registration_scope({}, false);
    g_pre_group_first.group_priority = 0;
    g_pre_group_second.group_priority = 10;
    g_pre_legacy_base.group_priority = 0;
    g_pre_legacy_alternative.group_priority = 10;
    for (RecordingPluginState *state : g_recording_plugins) {
        if (orchestrator.get_plugin(state->id) != nullptr)
            continue;

        plugin_instance instance = {};
        instance.ctx = state;
        instance.vt = recording_vtable();
        REQUIRE(orchestrator.register_plugin(instance));
        if (state->exclusive_group_label[0] != '\0' || state->exclusive_group_tooltip[0] != '\0') {
            raw_exclusive_group_def definition = raw_exclusive_group_def_init();
            definition.declaring_plugin_id = state->id;
            definition.exclusive_group = state->exclusive_group;
            definition.label = state->exclusive_group_label;
            definition.tooltip = state->exclusive_group_tooltip;
            definition.category = RAW_OPTION_CATEGORY_PERIMETER;
            REQUIRE(orchestrator_register_exclusive_group(reinterpret_cast<orchestrator_handle *>(&orchestrator),
                                                         state->step, &definition) == 1);
        }
    }
}

class ScopedActivePlugins
{
public:
    explicit ScopedActivePlugins(std::initializer_list<const char *> plugin_ids)
        : m_orchestrator(Orchestrator::instance())
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

class ScopedRecordingEvents
{
public:
    explicit ScopedRecordingEvents(std::vector<RecordedEvent> &events)
    {
        for (RecordingPluginState *state : g_recording_plugins) {
            state->events = &events;
            state->mutex = &m_mutex;
        }
    }

    ~ScopedRecordingEvents()
    {
        for (RecordingPluginState *state : g_recording_plugins) {
            state->events = nullptr;
            state->mutex = nullptr;
        }
    }

private:
    std::mutex m_mutex;
};

using StepRunFn = void (*)(Orchestrator &, Print &);

std::vector<std::string> plugin_ids_for_callback(const std::vector<RecordedEvent> &events,
                                                 const char *callback)
{
    std::vector<std::string> out;
    for (const RecordedEvent &event : events)
        if (event.callback == callback)
            out.push_back(event.plugin_id);
    return out;
}

void require_no_event_for_plugin(const std::vector<RecordedEvent> &events, const char *plugin_id)
{
    for (const RecordedEvent &event : events)
        CHECK(event.plugin_id != plugin_id);
}

void require_setup_counts(const std::vector<RecordedEvent> &events, uint32_t expected_run_count)
{
    size_t setup_count = 0;
    for (const RecordedEvent &event : events)
        if (event.callback == "setup") {
            ++setup_count;
            CHECK(event.run_count == expected_run_count);
        }
    CHECK(setup_count == 2);
}

void require_object_payloads(const std::vector<RecordedEvent> &events,
                             const Print &print,
                             slicing_step_t step)
{
    const print_handle *expected_print = reinterpret_cast<const print_handle *>(&print);
    const object_handle *objects[] = {
        reinterpret_cast<const object_handle *>(&print.object(0)),
        reinterpret_cast<const object_handle *>(&print.object(1))
    };
    std::array<size_t, 2> setup_run_seen = {{0, 0}};
    std::array<size_t, 2> run_seen = {{0, 0}};

    for (const RecordedEvent &event : events) {
        if (event.callback != "setup_run" && event.callback != "run")
            continue;

        CHECK(event.context_step == step);
        CHECK(event.print == expected_print);
        CHECK(event.object_count == 2);

        size_t object_idx = size_t(-1);
        if (event.object == objects[0])
            object_idx = 0;
        else if (event.object == objects[1])
            object_idx = 1;
        INFO("plugin " << event.plugin_id << ", callback " << event.callback);
        REQUIRE(object_idx < 2);
        CHECK(event.object_idx == object_idx);

        if (event.callback == "setup_run")
            ++setup_run_seen[object_idx];
        else
            ++run_seen[object_idx];
    }

    CHECK(setup_run_seen[0] == 2);
    CHECK(setup_run_seen[1] == 2);
    CHECK(run_seen[0] == 2);
    CHECK(run_seen[1] == 2);
}

void set_single_region_area(LayerRegion &region, const ExPolygon &area)
{
    ExPolygons &region_slices = ApiInternal::LayerRegionAccess::slices_mutable(region);
    region_slices = ExPolygons{area};
    ApiInternal::LayerRegionAccess::surfaces_mutable(region).set(region_slices, stPosInternal | stDensSparse);
}

void replace_single_layer_island(Layer &layer, const ExPolygon &area)
{
    // These step-boundary tests need a deterministic perimeter input, not a
    // full slicer fixture. Replacing the slice island and its matching region
    // slice gives the simple perimeter generator a small self-contained island
    // to process.
    ApiInternal::LayerAccess::set_islands(layer, ExPolygons{area});
    set_single_region_area(layer.region(0), area);
    layer.island(0).fill_regions(layer);
}

void rebuild_island_overlap_graph(PrintObject &object)
{
    for (Layer &layer : object.layers())
        for (LayerSliceIsland &island : layer.islands()) {
            island.overlaps_above.clear();
            island.overlaps_below.clear();
        }

    for (size_t layer_idx = 1; layer_idx < object.layer_count(); ++layer_idx)
        Layer::build_up_down_graph(object.layer(layer_idx - 1), object.layer(layer_idx));
}

ExtrusionPath straight_test_path(ExtrusionRole role, const double y_mm)
{
    ExtrusionPath path(ExtrusionAttributes(role, ExtrusionFlow(0.1, 0.4f, 0.2f)), nullptr, true);
    path.polyline().append(Point(scale_i(-4.), scale_i(y_mm)));
    path.polyline().append(Point(scale_i(4.), scale_i(y_mm)));
    return path;
}

void check_direct_publication(RecordingPluginState &state, const plugin_run_context *run_ctx)
{
    const run_ctx_generate_perimeter *payload = plugin_ctx_as_generate_perimeter(run_ctx);
    REQUIRE(payload != nullptr);
    REQUIRE(payload->set_region_island_extrusion != nullptr);
    layer_region_island_handle *handle = layer_island_get_or_create_region_island(
        const_cast<layer_island_handle *>(payload->island), nullptr, 0, -1);
    REQUIRE(handle != nullptr);
    LayerRegionIsland &island = *reinterpret_cast<LayerRegionIsland *>(handle);
    ExtrusionEntityCollection &bucket = island.mutable_extrusion(LayerRegionIsland::PERIMETERS);
    bucket.add_property(ExtrusionPropertySpeed(12.f));
    bucket.append_child(straight_test_path(ExtrusionRole::Perimeter, -1.));
    CHECK(bucket.entities().size() == 1);

    // A source scope must follow its children, while the replaced bucket's properties disappear.
    {
        ExtrusionEntity source(ExtrusionEntity::Children{}, false, false, false);
        source.add_property(ExtrusionPropertyCustomGcodeText("source scope"));
        source.append_child(straight_test_path(ExtrusionRole::Perimeter, 0.));
        source.append_child(straight_test_path(ExtrusionRole::Perimeter, 1.));
        const uint64_t source_id = source.get_id();
        const ExtrusionEntity *first = &source.child(0);
        const ExtrusionEntity *second = &source.child(1);
        REQUIRE(payload->set_region_island_extrusion(handle, RAW_EXTRUSION_ROLE_PERIMETER,
            reinterpret_cast<extrusion_entity_handle *>(&source)) != 0);
        CHECK(source.empty());
        CHECK_FALSE(source.has_properties());
        REQUIRE(bucket.child_count() == 1);
        CHECK_FALSE(bucket.has_properties());
        const ExtrusionEntity &scope = bucket.child(0);
        CHECK(scope.get_id() == source_id);
        REQUIRE(scope.child_count() == 2);
        CHECK(&scope.child(0) == first);
        CHECK(&scope.child(1) == second);
        CHECK(bucket.entities().front() == &scope);
    }
    const ExtrusionEntity &scope = bucket.child(0);
    const ExtrusionPropertyCustomGcode *text = scope.get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(text != nullptr);
    CHECK(scope.custom_gcode_string(*text) == "source scope");

    // An unmarked transport root donates its children directly, preserving their order and addresses.
    ExtrusionEntity replacement(ExtrusionEntity::Children{}, true, true, false);
    replacement.append_child(straight_test_path(ExtrusionRole::Perimeter, 2.));
    replacement.append_child(straight_test_path(ExtrusionRole::Perimeter, 3.));
    const ExtrusionEntity *first = &replacement.child(0);
    const ExtrusionEntity *second = &replacement.child(1);
    REQUIRE(payload->set_region_island_extrusion(handle, RAW_EXTRUSION_ROLE_PERIMETER,
        reinterpret_cast<extrusion_entity_handle *>(&replacement)) != 0);
    CHECK(replacement.empty());
    REQUIRE(bucket.child_count() == 2);
    CHECK(bucket.entities()[0] == first);
    CHECK(bucket.entities()[1] == second);

    // Clearing through the same callback removes the empty bucket rather than leaving a visible shell.
    REQUIRE(payload->set_region_island_extrusion(handle, RAW_EXTRUSION_ROLE_PERIMETER, nullptr) != 0);
    CHECK_FALSE(island.has_extrusion(LayerRegionIsland::PERIMETERS));
    ++state.direct_publications;
}

const Steps::StepExclusivePluginGroup *find_exclusive_group(const std::vector<Steps::StepExclusivePluginGroup> &groups,
                                                            const char *group_id)
{
    for (const Steps::StepExclusivePluginGroup &group : groups)
        if (group.group.group_id == group_id)
            return &group;
    return nullptr;
}

void run_and_check_object_step(slicing_step_t step,
                               const char *first_plugin_id,
                               const char *second_plugin_id,
                               const char *inactive_plugin_id,
                               StepRunFn run_step)
{
    PreparedPerimeterPrint prepared;
    const DynamicPrintConfig config = perimeter_config({});
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20,
                              Slic3r::Test::TestMesh::cube_20x20x20},
                             prepared.print,
                             prepared.model,
                             config);
    REQUIRE(prepared.print.objects().size() == 2);

    std::vector<RecordedEvent> events;
    ScopedRecordingEvents event_scope(events);
    ScopedActivePlugins active_scope({second_plugin_id, first_plugin_id});

    run_step(Orchestrator::instance(), prepared.print);

    require_no_event_for_plugin(events, inactive_plugin_id);
    require_setup_counts(events, 2);

    const std::vector<std::string> setup_ids = plugin_ids_for_callback(events, "setup");
    REQUIRE(setup_ids.size() == 2);
    CHECK(setup_ids[0] == first_plugin_id);
    CHECK(setup_ids[1] == second_plugin_id);

    const std::vector<std::string> setup_run_ids = plugin_ids_for_callback(events, "setup_run");
    REQUIRE(setup_run_ids.size() == 4);
    CHECK(setup_run_ids[0] == first_plugin_id);
    CHECK(setup_run_ids[1] == first_plugin_id);
    CHECK(setup_run_ids[2] == second_plugin_id);
    CHECK(setup_run_ids[3] == second_plugin_id);

    const std::vector<std::string> run_ids = plugin_ids_for_callback(events, "run");
    REQUIRE(run_ids.size() == 4);
    CHECK(run_ids[0] == first_plugin_id);
    CHECK(run_ids[1] == first_plugin_id);
    CHECK(run_ids[2] == second_plugin_id);
    CHECK(run_ids[3] == second_plugin_id);

    require_object_payloads(events, prepared.print, step);
}

} // namespace

TEST_CASE("Perimeter publication replaces whole buckets and preserves source scopes",
          "[plugins][perimeter][steps][publication-host]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    if (orchestrator.get_plugin(g_direct_publisher.id) == nullptr) {
        plugin_instance instance = {};
        instance.ctx = &g_direct_publisher;
        instance.vt = recording_vtable();
        REQUIRE(orchestrator.register_plugin(instance));
    }
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    g_direct_publisher.direct_publications = 0;
    ScopedActivePlugins active({g_direct_publisher.id});
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    CHECK(g_direct_publisher.direct_publications == 1);
}

// "Empty perimeter boundary steps" are the pre/post perimeter extension points.
// They currently do not transform perimeter geometry themselves; their host-side
// job is to run object-level plugins with the right lifecycle and C payload.
//
// This test therefore uses tiny recording plugins instead of real perimeter
// algorithms. It proves the step runner filters inactive plugins, sorts active
// plugins by priority, calls setup/setup_run/run at the expected granularity,
// and passes print/object handles plus object indexes that match the Print.
// Geometry regressions are covered by the generator/module tests, not here.
TEST_CASE("Empty perimeter boundary steps run object plugins", "[plugins][perimeter][steps]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_recording_plugins();

    SECTION("pre-perimeter step")
    {
        run_and_check_object_step(STEP_PRE_PERIMETER,
                                  g_pre_first.id,
                                  g_pre_second.id,
                                  g_pre_inactive.id,
                                  &Steps::StepPrepareForPeriemters::run_step);
    }

    SECTION("post-perimeter step")
    {
        run_and_check_object_step(STEP_POST_PERIMETER,
                                  g_post_first.id,
                                  g_post_second.id,
                                  g_post_inactive.id,
                                  &Steps::StepPostPerimeterGeneration::run_step);
    }

    SECTION("post-infill step")
    {
        run_and_check_object_step(STEP_POST_INFILL,
                                  g_post_infill_first.id,
                                  g_post_infill_second.id,
                                  g_post_infill_inactive.id,
                                  &Steps::StepPostInfillGeneration::run_step);
    }
}

TEST_CASE("Post-perimeter step exposes mutable perimeter outputs", "[plugins][perimeter][steps]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_recording_plugins();

    PreparedPerimeterPrint prepared;
    const DynamicPrintConfig config = perimeter_config({{"perimeters", "2"}});
    prepare_cube_print(prepared, config);
    PrintObject &object = prepared.print.object(0);
    REQUIRE(object.layer_count() > 0);
    Layer &layer = object.layer(0);
    replace_single_layer_island(layer, rectangle_expolygon(-8., -8., 8., 8.));
    rebuild_island_overlap_graph(object);

    {
        ScopedActivePlugins active_scope({SIMPLE_PERIMETER_GENERATOR});
        Steps::StepGeneratePerimeter::clear(prepared.print);
        Steps::StepGeneratePerimeter::run_step(Orchestrator::instance(), prepared.print);
    }

    bool saw_perimeter_extrusions = false;
    const LayerSliceIsland &generated_island = layer.island(0);
    REQUIRE_FALSE(generated_island.infill_areas().empty());
    for (const LayerRegionIsland &region_island : generated_island.regions_islands())
        saw_perimeter_extrusions =
            saw_perimeter_extrusions || region_island.has_extrusion(LayerRegionIsland::PERIMETERS);
    REQUIRE(saw_perimeter_extrusions);

    g_post_mutator.mutate_post_perimeter_outputs = true;
    g_post_mutator.saw_mutable_extrusion = false;
    g_post_mutator.changed_fill_areas = false;
    {
        ScopedActivePlugins active_scope({g_post_mutator.id});
        Steps::StepPostPerimeterGeneration::run_step(Orchestrator::instance(), prepared.print);
    }
    g_post_mutator.mutate_post_perimeter_outputs = false;

    CHECK(g_post_mutator.saw_mutable_extrusion);
    CHECK(g_post_mutator.changed_fill_areas);
    const LayerSliceIsland &mutated_island = layer.island(0);
    CHECK(mutated_island.infill_areas().empty());
    CHECK(mutated_island.infill_free_areas().empty());
    CHECK(mutated_island.infill_areas_bboxes().empty());
}

TEST_CASE("Perimeter clear resets retained region fill caches", "[plugins][perimeter][steps][cache]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    REQUIRE(prepared.print.object(0).layer_count() > 0);
    Layer &layer = prepared.print.object(0).layer(0);
    const ExPolygon raw_slice = rectangle_expolygon(-8., -8., 8., 8.);
    replace_single_layer_island(layer, raw_slice);
    LayerRegion &region = layer.region(0);
    const ExPolygons original_fill{rectangle_expolygon(-6., -6., 6., 6.)};
    ApiInternal::LayerIslandAccess::set_infill_areas(layer.island(0), ExPolygons(original_fill));
    ApiInternal::LayerRegionAccess::fill_no_overlap_expolygons_mutable(region) = original_fill;
    region.set_fill_surfaces().set(original_fill, stPosInternal | stDensSparse);
    const ExPolygons &cached_fill = region.fill_expolygons();
    REQUIRE_FALSE(cached_fill.empty());

    SECTION("step clear empties existing references before the next getter")
    {
        Steps::StepGeneratePerimeter::clear(prepared.print);
        CHECK(cached_fill.empty());
        CHECK(layer.island(0).infill_areas().empty());

        SECTION("empty island recomputes empty fill")
        {
            CHECK(region.fill_expolygons().empty());
        }
        SECTION("replacement fill recomputes without an intervening getter")
        {
            const ExPolygons changed_fill{rectangle_expolygon(-3., -2., 4., 5.)};
            ApiInternal::LayerIslandAccess::set_infill_areas(layer.island(0), ExPolygons(changed_fill));
            const ExPolygons &recomputed = region.fill_expolygons();
            REQUIRE_FALSE(recomputed.empty());
            CHECK(diff_ex(recomputed, changed_fill).empty());
            CHECK(diff_ex(changed_fill, recomputed).empty());
            CHECK_FALSE(diff_ex(original_fill, recomputed).empty());
        }
    }
    SECTION("region clear resets the sentinel even when island addresses are unchanged")
    {
        region.clear();
        CHECK(cached_fill.empty());
        const ExPolygons &recomputed = region.fill_expolygons();
        REQUIRE_FALSE(recomputed.empty());
        CHECK(diff_ex(recomputed, original_fill).empty());
        CHECK(diff_ex(original_fill, recomputed).empty());
    }

    CHECK(region.fill_no_overlap_expolygons().empty());
    CHECK(region.fill_surfaces().empty());
    REQUIRE(region.get_raw_slices().size() == 1);
    CHECK(region.get_raw_slices().front().contour.points == raw_slice.contour.points);
    REQUIRE(region.slices().surfaces.size() == 1);
    CHECK(region.slices().surfaces.front().expolygon.contour.points == raw_slice.contour.points);
    CHECK(region.slices().surfaces.front().surface_type == (stPosInternal | stDensSparse));
}

TEST_CASE("Region fill cache hashes 65 polygons", "[plugins][perimeter][steps][cache]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    REQUIRE(prepared.print.object(0).layer_count() > 0);
    Layer &layer = prepared.print.object(0).layer(0);
    replace_single_layer_island(layer, rectangle_expolygon(-8., -8., 8., 8.));
    ExPolygons fill;
    for (size_t idx = 0; idx < 65; ++idx) {
        const double x = -7. + double(idx % 9) * 1.5;
        const double y = -7. + double(idx / 9) * 1.5;
        fill.push_back(rectangle_expolygon(x, y, x + 1., y + 1.));
    }
    ApiInternal::LayerIslandAccess::set_infill_areas(layer.island(0), ExPolygons(fill));
    for (size_t read_idx = 0; read_idx < 2; ++read_idx) {
        const ExPolygons &cached_fill = layer.region(0).fill_expolygons();
        REQUIRE(cached_fill.size() == 65);
        CHECK(diff_ex(cached_fill, fill).empty());
        CHECK(diff_ex(fill, cached_fill).empty());
    }
}

TEST_CASE("Post-infill step exposes mutable infill outputs", "[plugins][infill][steps]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_recording_plugins();

    PreparedPerimeterPrint prepared;
    const DynamicPrintConfig config = perimeter_config({});
    prepare_cube_print(prepared, config);
    PrintObject &object = prepared.print.object(0);
    REQUIRE(object.layer_count() > 0);
    Layer &layer = object.layer(0);
    replace_single_layer_island(layer, rectangle_expolygon(-8., -8., 8., 8.));
    rebuild_island_overlap_graph(object);

    LayerSliceIsland &island = layer.island(0);
    REQUIRE_FALSE(island.regions().empty());
    LayerRegionIsland &region_island = island.get_or_add_region_island(island.regions(), 0);

    // The post-infill callback edits already-published infill buckets. This
    // test creates one small infill path directly instead of running a full
    // infill pattern; the step boundary does not care which plugin produced the
    // bucket, only that it is present and host-owned.
    region_island.mutable_extrusion(LayerRegionIsland::INFILLS).append(
        straight_test_path(ExtrusionRole::InternalInfill, 0.));

    // A perimeter bucket is present too. STEP_POST_INFILL must not expose it,
    // even though it lives on the same LayerRegionIsland, because perimeter
    // edits belong to STEP_POST_PERIMETER.
    region_island.mutable_extrusion(LayerRegionIsland::PERIMETERS).append(
        straight_test_path(ExtrusionRole::Perimeter, 2.));

    REQUIRE(region_island.has_extrusion(LayerRegionIsland::INFILLS));
    REQUIRE(region_island.has_extrusion(LayerRegionIsland::PERIMETERS));
    CHECK_FALSE(region_island.extrusion(LayerRegionIsland::INFILLS).is_continuous());

    g_post_infill_mutator.mutate_post_infill_outputs = true;
    g_post_infill_mutator.saw_mutable_extrusion = false;
    g_post_infill_mutator.saw_post_infill_region_island = false;
    g_post_infill_mutator.created_empty_gap_fill_root = false;
    g_post_infill_mutator.rejected_perimeter_bucket = false;
    {
        ScopedActivePlugins active_scope({g_post_infill_mutator.id});
        Steps::StepPostInfillGeneration::run_step(Orchestrator::instance(), prepared.print);
    }
    g_post_infill_mutator.mutate_post_infill_outputs = false;

    CHECK(g_post_infill_mutator.saw_mutable_extrusion);
    CHECK(g_post_infill_mutator.saw_post_infill_region_island);
    CHECK(g_post_infill_mutator.created_empty_gap_fill_root);
    CHECK(g_post_infill_mutator.rejected_perimeter_bucket);
    CHECK(region_island.extrusion(LayerRegionIsland::INFILLS).is_continuous());
    CHECK_FALSE(region_island.has_extrusion(LayerRegionIsland::GAP_FILLS));
    CHECK_FALSE(region_island.extrusion(LayerRegionIsland::PERIMETERS).is_continuous());
}

TEST_CASE("Explicit exclusive groups select one object-step plugin", "[plugins][perimeter][steps]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_recording_plugins();

    PreparedPerimeterPrint prepared;
    const DynamicPrintConfig config = perimeter_config({});
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20,
                              Slic3r::Test::TestMesh::cube_20x20x20},
                             prepared.print,
                             prepared.model,
                             config);

    std::vector<RecordedEvent> events;
    ScopedRecordingEvents event_scope(events);
    ScopedActivePlugins active_scope({
        g_pre_first.id,
        g_pre_group_first.id,
        g_pre_group_second.id
    });

    // Rebuilding descriptions must retain the enabled describing member with the smallest preference number.
    Orchestrator::instance().elect_exclusive_group_definition(STEP_PRE_PERIMETER, g_pre_group_first.exclusive_group);
    const std::vector<Steps::StepExclusivePluginGroup> groups =
        Steps::active_exclusive_plugin_groups(Orchestrator::instance());
    const Steps::StepExclusivePluginGroup *group =
        find_exclusive_group(groups, g_pre_group_first.exclusive_group);
    REQUIRE(group != nullptr);
    REQUIRE(group->plugins.size() == 2);
    CHECK(group->plugins[0]->get_id() == g_pre_group_first.id);
    CHECK(group->plugins[1]->get_id() == g_pre_group_second.id);
    CHECK(group->group.label_storage == g_pre_group_first.exclusive_group_label);
    CHECK(group->group.tooltip_storage == g_pre_group_first.exclusive_group_tooltip);

    // No selector option is injected into this synthetic test config, so the
    // runtime falls back to the first plugin in the exclusive group. The normal
    // additive plugin still runs alongside it.
    Steps::StepPrepareForPeriemters::run_step(Orchestrator::instance(), prepared.print);

    require_no_event_for_plugin(events, g_pre_group_second.id);
    require_setup_counts(events, 2);

    const std::vector<std::string> setup_ids = plugin_ids_for_callback(events, "setup");
    REQUIRE(setup_ids.size() == 2);
    CHECK(setup_ids[0] == g_pre_first.id);
    CHECK(setup_ids[1] == g_pre_group_first.id);

    require_object_payloads(events, prepared.print, STEP_PRE_PERIMETER);
}

TEST_CASE("A plugin without explicit group can receive an alternative", "[plugins][perimeter][steps]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_recording_plugins();

    Orchestrator &orchestrator = Orchestrator::instance();
    const Plugin *legacy = orchestrator.get_plugin(g_pre_legacy_base.id);
    REQUIRE(legacy != nullptr);

    // Older plugins may not know that alternatives will exist later. The host
    // still assigns their plugin id as a singleton exclusive group, so a new
    // plugin can opt into that group without modifying the legacy plugin.
    REQUIRE(legacy->get_exclusive_group() == legacy->get_id());

    ScopedActivePlugins active_scope({
        g_pre_legacy_base.id,
        g_pre_legacy_alternative.id
    });

    orchestrator.elect_exclusive_group_definition(STEP_PRE_PERIMETER, g_pre_legacy_base.id);
    const std::vector<Steps::StepExclusivePluginGroup> groups =
        Steps::active_exclusive_plugin_groups(orchestrator);
    const Steps::StepExclusivePluginGroup *group =
        find_exclusive_group(groups, g_pre_legacy_base.id);
    REQUIRE(group != nullptr);
    REQUIRE(group->plugins.size() == 2);
    CHECK(group->plugins[0]->get_id() == g_pre_legacy_base.id);
    CHECK(group->plugins[1]->get_id() == g_pre_legacy_alternative.id);

    // The legacy plugin has no selector text, but the alternative can still
    // provide a human-readable label and tooltip for the shared group.
    CHECK(group->group.label_storage == g_pre_legacy_alternative.exclusive_group_label);
    CHECK(group->group.tooltip_storage == g_pre_legacy_alternative.exclusive_group_tooltip);

    const std::vector<Plugin *> selected =
        Steps::selected_or_active_plugins_for_step(orchestrator, STEP_PRE_PERIMETER, nullptr);
    REQUIRE(selected.size() == 1);
    CHECK(selected.front()->get_id() == g_pre_legacy_base.id);
}
