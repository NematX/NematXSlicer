#include <catch2/catch.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "perimeter_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/Api/internal/ArcPolylineAccess.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_seam_placer.h"
#include "libslic3r/Api/plugin/cpp/OrchestratorViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/SeamPlacerViews.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Geometry/ArcWelder.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Plugins/Ordering/ExtrusionTreeOrderingGeometry.hpp"
#include "libslic3r/Plugins/SeamPlacer/SeamPlacerRouter.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintObjectRegion.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"

/*
Fixed-direction seam placer tests
=================================

These tests execute the registered provider through the public service rather
than reaching into its implementation. Real LayerRegion objects provide the
regional setting and real source islands exercise the ordering provider's
region lookup. The loops remain small so every projection and tie-break is
visible in the expected point.
*/

namespace {

using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::PerimeterPluginTests;

const char *const k_fixed_plugin = "seam_placer.fixed";
const char *const k_seam_group = "seam_placer_plugin";
const char *const k_direction_key = "seam_fixed_direction";
const char *const k_routing_plugin = "slic3r.test.seam_placer.regional";

struct RoutingProviderState
{
    std::vector<const print_region_handle *> initialized_regions;
    uint32_t initialize_count = 0;
    uint32_t priority_count = 0;
    uint32_t place_count = 0;
    float priority = 0.f;

    void reset()
    {
        initialized_regions.clear();
        initialize_count = 0;
        priority_count = 0;
        place_count = 0;
        priority = 0.f;
    }
};

RoutingProviderState g_routing_provider;

class RoutingSeamSession final : public slic3r_api::SeamPlacerSession
{
public:
    void initialize(
        const slic3r_api::PrintingPlan &,
        const std::vector<slic3r_api::PrintRegion> &regions) override
    {
        ++g_routing_provider.initialize_count;
        for (const slic3r_api::PrintRegion &region : regions)
            g_routing_provider.initialized_regions.push_back(region.handle());
    }

    float get_priority(const slic3r_api::ExtrusionEntity &,
                       const slic3r_api::LayerRegion &) const override
    {
        ++g_routing_provider.priority_count;
        return g_routing_provider.priority;
    }

    c_point place_seam(const slic3r_api::ExtrusionEntity &loop,
                       const slic3r_api::LayerRegion &,
                       c_point) const override
    {
        ++g_routing_provider.place_count;
        return loop.front();
    }
};

class RoutingSeamPlugin final : public slic3r_api::PluginBase
{
public:
    explicit RoutingSeamPlugin(orchestrator_handle *orchestrator) :
        PluginBase(orchestrator) {}

private:
    const char *id_impl() const noexcept override { return k_routing_plugin; }
    const char *exclusive_group_impl() const noexcept override { return k_seam_group; }
    slicing_step_t step_impl() const noexcept override { return SEAM_PLACER; }
    const char *const *dependencies_impl() const noexcept override
    {
        static const char *dependencies[] = { nullptr };
        return dependencies;
    }
    int32_t priority_impl() const noexcept override { return 2000; }

    void run_impl(const plugin_run_context *run_context) const override
    {
        run_ctx_seam_placer *context = plugin_ctx_as_seam_placer(run_context);
        if (context == nullptr || context->plan == nullptr ||
            (context->region_count != 0 && context->regions == nullptr))
            throw std::invalid_argument("The regional seam fixture received an invalid context.");
        std::vector<slic3r_api::PrintRegion> regions;
        regions.reserve(context->region_count);
        for (uint32_t idx = 0; idx < context->region_count; ++idx)
            regions.emplace_back(context->regions[idx]);
        context->instance = slic3r_api::make_seam_placer_instance(
            std::unique_ptr<slic3r_api::SeamPlacerSession>(new RoutingSeamSession()),
            slic3r_api::PrintingPlan(context->plan), regions);
    }
};

class ScopedSeamProviders
{
public:
    explicit ScopedSeamProviders(std::initializer_list<const char *> ids) :
        m_orchestrator(Orchestrator::instance())
    {
        const auto &active = m_orchestrator.active_plugins();
        m_previous.assign(active.begin(), active.end());
        // Keep the slicing pipeline available while the real regional fixture
        // is prepared; only add the providers exercised by this test.
        for (const char *id : ids)
            REQUIRE(m_orchestrator.set_plugin_active(id, true));
    }

    ~ScopedSeamProviders()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous)
            m_orchestrator.set_plugin_active(plugin, true);
        g_routing_provider.reset();
    }

private:
    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous;
};

void ensure_routing_provider_registered()
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    REQUIRE(orchestrator.get_plugin(k_routing_plugin) != nullptr);
}

/* Own one sliced cube whose first LayerRegion carries a requested direction. */
struct DirectionRegionFixture
{
    explicit DirectionRegionFixture(const char *direction,
                                    const ExPolygon *replacement = nullptr)
    {
        DynamicPrintConfig config = perimeter_config({{k_direction_key, direction}});
        prepare_cube_print(prepared, config);
        LayerRegion &layer_region = prepared.print.object(0).layer(0).region(0);
        if (replacement != nullptr) {
            ExPolygons &raw_slices = ApiInternal::LayerRegionAccess::slices_mutable(layer_region);
            raw_slices = ExPolygons{*replacement};
            ApiInternal::LayerRegionAccess::surfaces_mutable(layer_region).set(
                raw_slices, stPosInternal | stDensSolid);
        }
        region = &layer_region;
    }

    slic3r_api::LayerRegion view() const
    {
        return slic3r_api::LayerRegion(
            reinterpret_cast<const layer_region_handle *>(region));
    }

    PreparedPerimeterPrint prepared;
    LayerRegion *region = nullptr;
};

/* Convert one native extrusion to a borrowed API view. */
slic3r_api::ExtrusionEntity extrusion_view(const ExtrusionEntity &entity)
{
    return slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(&entity));
}

ExtrusionEntity square_loop()
{
    return ExtrusionEntity(false, ArcPolyline(Points{
        Point::new_scale(0.0, 0.0),
        Point::new_scale(10.0, 0.0),
        Point::new_scale(10.0, 10.0),
        Point::new_scale(0.0, 10.0),
        Point::new_scale(0.0, 0.0)}));
}

ExtrusionEntity composed_loop()
{
    ExtrusionEntity::Children children;
    children.emplace_back(new ExtrusionEntity(
        false, ArcPolyline(Points{Point::new_scale(0.0, 0.0),
                                  Point::new_scale(10.0, 0.0)})));
    children.emplace_back(new ExtrusionEntity(
        false, ArcPolyline(Points{Point::new_scale(10.0, 0.0),
                                  Point::new_scale(10.0, 10.0)})));
    children.emplace_back(new ExtrusionEntity(
        false, ArcPolyline(Points{Point::new_scale(10.0, 10.0),
                                  Point::new_scale(0.0, 10.0),
                                  Point::new_scale(0.0, 0.0)})));
    return ExtrusionEntity(std::move(children), false, false, false);
}

ExtrusionEntity arc_loop()
{
    const coord_t radius = scale_i(10.0);
    Geometry::ArcWelder::Path path;
    path.emplace_back(Point(-radius, 0), 0.f,
                      Geometry::ArcWelder::Orientation::Unknown);
    path.emplace_back(Point(radius, 0), static_cast<float>(radius),
                      Geometry::ArcWelder::Orientation::CCW);
    path.emplace_back(Point(-radius, 0), static_cast<float>(radius),
                      Geometry::ArcWelder::Orientation::CCW);
    return ExtrusionEntity(false, ArcPolyline(path));
}

/* Execute the built-in provider and take ownership of its published session. */
slic3r_api::SeamPlacer create_fixed_session(Print &print, const char *provider_id = k_fixed_plugin)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    slic3r_api::OrchestratorView view(
        reinterpret_cast<orchestrator_handle *>(&orchestrator));
    std::unique_ptr<ScopedSeamProviders> active;
    if (std::string(provider_id)!="seam_placer.fixed") active.reset(new ScopedSeamProviders({provider_id}));
    const slic3r_api::PluginView provider = view.find_plugin(provider_id);
    REQUIRE(provider.valid());

    PrintingPlan &plan = print.mutable_printing_plan();
    const slic3r_api::Print print_view(
        reinterpret_cast<const print_handle *>(&print));
    std::vector<const print_region_handle *> regions;
    for (uint32_t object_idx = 0; object_idx < print_view.object_count(); ++object_idx) {
        const slic3r_api::Object object = print_view.object(object_idx);
        for (uint32_t region_idx = 0; region_idx < object.print_region_count(); ++region_idx)
            regions.push_back(object.print_region(region_idx).handle());
    }
    run_ctx_seam_placer context = {};
    context.plan = reinterpret_cast<printing_plan_handle *>(&plan);
    context.regions = regions.empty() ? nullptr : regions.data();
    context.region_count = uint32_t(regions.size());
    REQUIRE(view.execute(
                provider,
                reinterpret_cast<const print_handle *>(&print),
                context) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(context.instance.session != nullptr);
    return slic3r_api::SeamPlacer(context.instance);
}

bool same_point(const c_point lhs, const c_point rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

c_point mm_point(const double x, const double y)
{
    return c_point{scale_i(x), scale_i(y)};
}

/* Give a closed loop the perimeter marker that seam providers bind. */
void mark_perimeter_loop(slic3r_api::MutableExtrusionEntity loop)
{
    orchestrator_handle *orch = reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance());
    c_extrusion_property_perimeter *perimeter = static_cast<c_extrusion_property_perimeter *>(
        loop.get_or_add_property_data_mutable(orch, EXTRUSION_PROPERTY_TYPE_PERIMETER));
    REQUIRE(perimeter != nullptr);
    perimeter->perimeter_flags = C_EXTRUSION_PERIMETER_FLAG_LOOP;
}

/*
Own two real PrintRegions and one synthetic plan loop that may reference one or
both LayerRegions. The plan stores platter coordinates; source slices remain in
object coordinates so the router must remove the real instance shift.
*/
// Replace fixture associations without modifying any published snapshot.
const PrintRegion *replace_region_config(LayerRegion &layer_region, const PrintRegionConfig &config)
{
    PrintObject &object = *layer_region.layer()->object();
    Print &print = *object.print();
    const PrintRegion *old_region = &layer_region.region();
    const PrintRegion *replacement = &print.register_print_region(config);
    PrintObjectRegions &associations = *const_cast<PrintObjectRegions *>(object.shared_regions());
    for (const PrintRegion *&region : associations.all_regions)
        if (region == old_region) region = replacement;
    for (Layer &layer : object.layers())
        for (LayerRegion &region : layer.regions())
            if (&region.region() == old_region)
                ApiInternal::LayerRegionAccess::set_print_region(region, *replacement);
    for (Layer &layer : object.auxiliary_layers())
        for (LayerRegion &region : layer.regions())
            if (&region.region() == old_region)
                ApiInternal::LayerRegionAccess::set_print_region(region, *replacement);
    print.rebuild_active_print_regions();
    return replacement;
}

struct RegionalRouterFixture
{
    /*
    Build a two-region print and one marked closed plan loop; selector identifiers
    are decoded through the registered enum, and include_fixed_candidate controls
    the source-region set. The helper owns its print, object, and source island;
    preparation or enum failures reach Catch2.
    */
    explicit RegionalRouterFixture(const bool include_fixed_candidate)
    {
        prepare_cube_print(prepared, perimeter_config({{k_direction_key, "rear"}}));
        PrintObject &object = prepared.print.object(0);
        Layer &layer = object.layer(0);
        fixed_region = &layer.region(0);

        const std::string selector_key = Slic3r::Steps::plugin_exclusive_group_option_key(
            Orchestrator::instance(), SEAM_PLACER, k_seam_group);
        const ConfigOptionDef *selector_definition =
            PrintConfigDef::instance().get(selector_key);
        REQUIRE(selector_definition != nullptr);

        const PrintRegion *fixed_print_region = &fixed_region->region();
        PrintRegionConfig fixed_config = fixed_print_region->config();
        fixed_config.add_plugin_option(
            *const_cast<ConfigOptionDef *>(selector_definition));
        REQUIRE(fixed_config.opt<ConfigOptionInt>(selector_key) != nullptr);
        fixed_config.set_deserialize_strict(selector_key, k_fixed_plugin);
        fixed_print_region = replace_region_config(*fixed_region, fixed_config);

        PrintRegionConfig routed_config = fixed_print_region->config();
        REQUIRE(routed_config.opt<ConfigOptionInt>(selector_key) != nullptr);
        routed_config.set_deserialize_strict(selector_key, k_routing_plugin);
        PrintObjectRegions *shared_regions =
            const_cast<PrintObjectRegions *>(object.shared_regions());
        shared_regions->all_regions.emplace_back(
            &prepared.print.register_print_region(routed_config));
        routed_print_region = shared_regions->all_regions.back();
        ApiInternal::LayerAccess::add_region(layer, *routed_print_region);
        routed_region = &layer.region(layer.region_count() - 1);

        // Both source regions cover the test loop. Candidate membership is
        // controlled by LayerRegionIsland, just as it is in a production plan.
        const ExPolygons areas = ApiInternal::LayerRegionAccess::slices_mutable(
            *fixed_region);
        ExPolygons &routed_slices = ApiInternal::LayerRegionAccess::slices_mutable(
            *routed_region);
        routed_slices = areas;
        ApiInternal::LayerRegionAccess::surfaces_mutable(*routed_region).set(
            routed_slices, stPosInternal | stDensSolid);

        LayerRegionSetCPtrs candidates;
        if (include_fixed_candidate)
            candidates.insert(fixed_region);
        candidates.insert(routed_region);

        if (include_fixed_candidate && *candidates.begin() == routed_region) {
            // LayerRegionSetCPtrs defines encounter order. Assign Fixed to its
            // actual first element so the fixture does not depend on addresses.
            ApiInternal::LayerRegionAccess::set_print_region(*routed_region, *fixed_print_region);

            ApiInternal::LayerRegionAccess::set_print_region(*fixed_region, *routed_print_region);
            std::swap(shared_regions->all_regions.front(), shared_regions->all_regions.back());
        }
        source.reset(new LayerRegionIsland(candidates, 0));

        const Point shift = object.instances().front().shift;
        std::unique_ptr<ExtrusionEntity> loop(new ExtrusionEntity(
            false, ArcPolyline(Points{
                shift + Point::new_scale(-5.0, -1.0),
                shift + Point::new_scale(5.0, -1.0),
                shift + Point::new_scale(5.0, 1.0),
                shift + Point::new_scale(-5.0, 1.0),
                shift + Point::new_scale(-5.0, -1.0)})));
        mark_perimeter_loop(slic3r_api::MutableExtrusionEntity(
            reinterpret_cast<extrusion_entity_handle *>(loop.get())));
        source->mutable_extrusion(LayerRegionIsland::PERIMETERS).append(*loop);

        PrintingPlan &plan = prepared.print.mutable_printing_plan();
        const PrintingObjectInstanceId catalog_id = plan.find_or_add_object_instance(object, 0);
        plan.groups.emplace_back();
        plan.groups.back().printing_catalog_ids.push_back(catalog_id);
        plan.groups.back().layers.emplace_back();
        plan.groups.back().layers.back().tool_groups.emplace_back();
        PrintingToolGroup &tool = plan.groups.back().layers.back().tool_groups.back();
        tool.region_islands.push_back(source.get());
        PrintingExtrusion extrusion;
        extrusion.region_island = source.get();
        extrusion.root = std::move(loop);
        extrusion.sregion_island_role = LayerRegionIsland::PERIMETERS;
        extrusion.printing_catalog_id = catalog_id;
        tool.extrusions.push_back(std::move(extrusion));
    }

    slic3r_api::Print print_view() const
    {
        return slic3r_api::Print(
            reinterpret_cast<const print_handle *>(&prepared.print));
    }

    slic3r_api::PrintingPlan plan_view()
    {
        return slic3r_api::PrintingPlan(
            reinterpret_cast<printing_plan_handle *>(
                &prepared.print.mutable_printing_plan()));
    }

    slic3r_api::ExtrusionEntity loop_view()
    {
        PrintingExtrusion &extrusion = prepared.print.mutable_printing_plan()
            .groups.front().layers.front().tool_groups.front().extrusions.front();
        return slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(extrusion.root.get()));
    }

    /* Add a second region assigned to the routing provider but unused by loops. */
    const PrintRegion *add_unused_routing_region()
    {
        PrintObject &object = prepared.print.object(0);
        Layer &layer = object.layer(0);
        PrintRegionConfig config = routed_print_region->config();
        config.perimeters.value += 1;
        PrintObjectRegions *shared_regions =
            const_cast<PrintObjectRegions *>(object.shared_regions());
        shared_regions->all_regions.emplace_back(&prepared.print.register_print_region(config));
        const PrintRegion *added = shared_regions->all_regions.back();
        ApiInternal::LayerAccess::add_region(layer, *added);
        return added;
    }

    /*
    Replace the plan root with two disjoint loops. The first island region is
    configured for Fixed and covers the left loop; the second is configured for
    the recording provider and covers the right loop.
    */
    std::array<slic3r_api::ExtrusionEntity, 2> replace_with_split_root()
    {
        REQUIRE(source->regions().size() == 2);
        LayerRegion *first = const_cast<LayerRegion *>(*source->regions().begin());
        LayerRegion *second = const_cast<LayerRegion *>(*std::next(source->regions().begin()));

        const ExPolygons left_area = {ExPolygon{Polygon::new_scale(
            {{-9.0, -3.0}, {-1.0, -3.0}, {-1.0, 3.0}, {-9.0, 3.0}})}};
        const ExPolygons right_area = {ExPolygon{Polygon::new_scale(
            {{1.0, -3.0}, {9.0, -3.0}, {9.0, 3.0}, {1.0, 3.0}})}};
        ApiInternal::LayerRegionAccess::slices_mutable(*first) = left_area;
        ApiInternal::LayerRegionAccess::surfaces_mutable(*first).set(
            left_area, stPosInternal | stDensSolid);
        ApiInternal::LayerRegionAccess::slices_mutable(*second) = right_area;
        ApiInternal::LayerRegionAccess::surfaces_mutable(*second).set(
            right_area, stPosInternal | stDensSolid);

        const Point shift = prepared.print.object(0).instances().front().shift;
        auto shifted_loop = [shift](const double center_x) {
            std::unique_ptr<ExtrusionEntity> loop(new ExtrusionEntity(
                false, ArcPolyline(Points{
                    shift + Point::new_scale(center_x - 1.0, -1.0),
                    shift + Point::new_scale(center_x + 1.0, -1.0),
                    shift + Point::new_scale(center_x + 1.0, 1.0),
                    shift + Point::new_scale(center_x - 1.0, 1.0),
                    shift + Point::new_scale(center_x - 1.0, -1.0)})));
            mark_perimeter_loop(slic3r_api::MutableExtrusionEntity(
                reinterpret_cast<extrusion_entity_handle *>(loop.get())));
            return loop;
        };
        ExtrusionEntity::Children children;
        children.emplace_back(shifted_loop(-5.0));
        children.emplace_back(shifted_loop(5.0));

        PrintingExtrusion &extrusion = prepared.print.mutable_printing_plan()
            .groups.front().layers.front().tool_groups.front().extrusions.front();
        extrusion.root.reset(new ExtrusionEntity(
            std::move(children), false, false, false));
        return {{
            extrusion_view(extrusion.root->child(0)),
            extrusion_view(extrusion.root->child(1))}};
    }

    void set_routed_selector(const int32_t selected_provider)
    {
        const std::string selector_key = Slic3r::Steps::plugin_exclusive_group_option_key(
            Orchestrator::instance(), SEAM_PLACER, k_seam_group);
        PrintRegionConfig config = routed_print_region->config();
        ConfigOptionInt *selector = config.opt<ConfigOptionInt>(selector_key);
        REQUIRE(selector != nullptr);
        selector->set_int(selected_provider);
        LayerRegion &selected = &fixed_region->region() == routed_print_region ? *fixed_region : *routed_region;
        routed_print_region = replace_region_config(selected, config);
    }

    PreparedPerimeterPrint prepared;
    LayerRegion *fixed_region = nullptr;
    LayerRegion *routed_region = nullptr;
    const PrintRegion *routed_print_region = nullptr;
    std::unique_ptr<LayerRegionIsland> source;
    PluginStorage scratch_storage;
};

} // namespace

namespace Slic3r::Test::Plugins {

void register_regional_seam_test_provider(orchestrator_handle *orchestrator)
{
    Orchestrator &host = *reinterpret_cast<Orchestrator *>(orchestrator);
    if (host.get_plugin(k_routing_plugin) != nullptr)
        return;
    static std::unique_ptr<RoutingSeamPlugin> plugin;
    plugin.reset(new RoutingSeamPlugin(orchestrator));
    if (!host.register_plugin(plugin->c_instance()))
        throw std::runtime_error("Could not register the regional SeamPlacer test provider.");
}

} // namespace Slic3r::Test::Plugins

TEST_CASE("Fixed-direction seam placer registers its regional contract",
          "[plugins][seam-placer][fixed][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *plugin = orchestrator.get_plugin(k_fixed_plugin);
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_name() == "Fixed direction");
    CHECK(plugin->get_step() == SEAM_PLACER);
    CHECK(plugin->get_priority() == 900);
    CHECK(plugin->get_exclusive_group() == k_seam_group);

    const std::vector<Plugin::UsedConfigKey> used = plugin->get_used_config_keys();
    REQUIRE(used.size() == 1);
    CHECK(used[0].key == k_direction_key);
    CHECK(used[0].type == RAW_CO_ENUM);
    CHECK(used[0].container_type == RAW_CONTAINER_TYPE_REGION);

    raw_config_option_def definition = {};
    REQUIRE(orchestrator_get_option_def(
                reinterpret_cast<orchestrator_handle *>(&orchestrator),
                k_direction_key, &definition) == 1);
    CHECK(definition.type == RAW_CO_ENUM);
    CHECK(definition.container_type == RAW_CONTAINER_TYPE_REGION);
    CHECK(definition.invalidates_step == STEP_ORDERING);
    REQUIRE(definition.default_serialized_value != nullptr);
    CHECK(std::string(definition.default_serialized_value) == "rear");
    const std::array<const char *, 8> expected = {{
        "rear", "front", "right", "left",
        "rear_right", "rear_left", "front_right", "front_left"}};
    REQUIRE(definition.enum_def.value_label_pairs.count == expected.size());
    for (size_t idx = 0; idx < expected.size(); ++idx)
        CHECK(std::string(definition.enum_def.value_label_pairs.items[idx].value) == expected[idx]);

    std::ifstream stream(
        std::string(TEST_DATA_DIR) + "/../../src/plugins/default_activated.ini.in");
    REQUIRE(stream.good());
    std::ostringstream contents;
    contents << stream.rdbuf();
    CHECK(contents.str().find("seam_placer.fixed = 1") != std::string::npos);

    const std::vector<Slic3r::Steps::StepExclusivePluginGroup> active_groups =
        Slic3r::Steps::active_exclusive_plugin_groups(orchestrator);
    CHECK(std::none_of(
        active_groups.begin(), active_groups.end(),
        [](const Slic3r::Steps::StepExclusivePluginGroup &candidate) {
            return candidate.group.step == SEAM_PLACER &&
                   candidate.group.group_id == k_seam_group;
        }));
}

/*
Route marked loops through their configured regional providers and verify priority, initialization,
and selector diagnostics. Scoped activation restores prior providers, each local print owns its
regions, and unresolved providers throw for the diagnostic assertions.
*/
TEST_CASE("Regional seam routing initializes providers and ranks overlapping regions",
          "[plugins][seam-placer][router]")
{
    ensure_routing_provider_registered();
    ScopedSeamProviders active({k_fixed_plugin, k_routing_plugin});
    Orchestrator &orchestrator = Orchestrator::instance();

    const std::vector<Slic3r::Steps::StepExclusivePluginGroup> groups =
        Slic3r::Steps::active_exclusive_plugin_groups(orchestrator);
    const auto group = std::find_if(
        groups.begin(), groups.end(), [](const Slic3r::Steps::StepExclusivePluginGroup &candidate) {
            return candidate.group.step == SEAM_PLACER &&
                   candidate.group.group_id == k_seam_group;
        });
    REQUIRE(group != groups.end());
    CHECK(group->group.option_def.container_type == RAW_CONTAINER_TYPE_REGION);
    CHECK(group->group.option_def.invalidates_step == STEP_ORDERING);
    CHECK(group->group.ui_fragment.find("group:Seam\n") != std::string::npos);

    SECTION("a strictly higher priority selects the later region")
    {
        g_routing_provider.reset();
        RegionalRouterFixture fixture(true);
        // Test a strictly higher rank without assuming Fixed's numeric policy.
        slic3r_api::SeamPlacer fixed=create_fixed_session(fixture.prepared.print);
        g_routing_provider.priority = fixed.get_priority(fixture.loop_view(),
            slic3r_api::LayerRegion(reinterpret_cast<const layer_region_handle *>(fixture.fixed_region))) + 5.f;
        slic3r_api::SeamPlacement::SeamPlacerRouter router(
            reinterpret_cast<orchestrator_handle *>(&orchestrator),
            fixture.print_view(), fixture.plan_view(),
            reinterpret_cast<storage_handle *>(&fixture.scratch_storage));

        REQUIRE(g_routing_provider.initialized_regions.size() == 1);
        CHECK(g_routing_provider.initialized_regions.front() ==
              reinterpret_cast<const print_region_handle *>(fixture.routed_print_region));
        const c_point expected = fixture.loop_view().front();
        CHECK(same_point(router.place_seam(fixture.loop_view(), {}), expected));
        CHECK(g_routing_provider.priority_count == 1);
        CHECK(g_routing_provider.place_count == 1);
    }

    SECTION("one provider is initialized once with all of its regions in order")
    {
        g_routing_provider.reset();
        RegionalRouterFixture fixture(true);
        const PrintRegion *second_routed_region = fixture.add_unused_routing_region();
        slic3r_api::SeamPlacement::SeamPlacerRouter router(
            reinterpret_cast<orchestrator_handle *>(&orchestrator),
            fixture.print_view(), fixture.plan_view(),
            reinterpret_cast<storage_handle *>(&fixture.scratch_storage));

        CHECK(g_routing_provider.initialize_count == 1);
        REQUIRE(g_routing_provider.initialized_regions.size() == 2);
        CHECK(g_routing_provider.initialized_regions[0] ==
              reinterpret_cast<const print_region_handle *>(fixture.routed_print_region));
        CHECK(g_routing_provider.initialized_regions[1] ==
              reinterpret_cast<const print_region_handle *>(second_routed_region));
    }

    SECTION("one extrusion root may route different loops to different providers")
    {
        g_routing_provider.reset();
        RegionalRouterFixture fixture(true);
        const std::array<slic3r_api::ExtrusionEntity, 2> loops =
            fixture.replace_with_split_root();
        slic3r_api::SeamPlacement::SeamPlacerRouter router(
            reinterpret_cast<orchestrator_handle *>(&orchestrator),
            fixture.print_view(), fixture.plan_view(),
            reinterpret_cast<storage_handle *>(&fixture.scratch_storage));

        const c_point fixed_seam = router.place_seam(loops[0], {});
        const c_point routed_seam = router.place_seam(loops[1], {});
        CHECK(fixed_seam.y == loops[0].back().y + scale_i(2.0));
        CHECK(same_point(routed_seam, loops[1].front()));
        CHECK(g_routing_provider.priority_count == 0);
        CHECK(g_routing_provider.place_count == 1);
    }

    SECTION("an exact tie preserves the first LayerRegion")
    {
        g_routing_provider.reset();
        g_routing_provider.priority = 50.f;
        RegionalRouterFixture fixture(true);
        slic3r_api::SeamPlacement::SeamPlacerRouter router(
            reinterpret_cast<orchestrator_handle *>(&orchestrator),
            fixture.print_view(), fixture.plan_view(),
            reinterpret_cast<storage_handle *>(&fixture.scratch_storage));

        (void)router.place_seam(fixture.loop_view(), {});
        CHECK(g_routing_provider.priority_count == 1);
        CHECK(g_routing_provider.place_count == 0);
    }

    SECTION("one candidate bypasses priority")
    {
        g_routing_provider.reset();
        g_routing_provider.priority = 5.f;
        RegionalRouterFixture fixture(false);
        slic3r_api::SeamPlacement::SeamPlacerRouter router(
            reinterpret_cast<orchestrator_handle *>(&orchestrator),
            fixture.print_view(), fixture.plan_view(),
            reinterpret_cast<storage_handle *>(&fixture.scratch_storage));

        (void)router.place_seam(fixture.loop_view(), {});
        CHECK(g_routing_provider.priority_count == 0);
        CHECK(g_routing_provider.place_count == 1);
    }

    SECTION("an explicit unavailable provider does not fall back")
    {
        g_routing_provider.reset();
        RegionalRouterFixture fixture(false);
        fixture.set_routed_selector(99);
        CHECK_THROWS_WITH(
            slic3r_api::SeamPlacement::SeamPlacerRouter(
                reinterpret_cast<orchestrator_handle *>(&orchestrator),
                fixture.print_view(), fixture.plan_view(),
                reinterpret_cast<storage_handle *>(&fixture.scratch_storage)),
            Catch::Matchers::Contains(
                "exclusive_group_10250_seam_placer_plugin_plugin' = '#99'"));
    }

    SECTION("an inactive provider identifier is named in the error")
    {
        Plugin *cost = orchestrator.get_plugin("seam_placer.cost");
        REQUIRE(cost != nullptr);
        CHECK_FALSE(orchestrator.is_plugin_active(cost));
        RegionalRouterFixture fixture(false);
        const std::string selector_key = Slic3r::Steps::plugin_exclusive_group_option_key(
            orchestrator, SEAM_PLACER, k_seam_group);
        PrintRegionConfig config = fixture.routed_print_region->config();
        config.set_deserialize_strict(selector_key, "seam_placer.cost");
        fixture.routed_print_region = replace_region_config(*fixture.routed_region, config);
        CHECK_THROWS_WITH(
            slic3r_api::SeamPlacement::SeamPlacerRouter(
                reinterpret_cast<orchestrator_handle *>(&orchestrator),
                fixture.print_view(), fixture.plan_view(),
                reinterpret_cast<storage_handle *>(&fixture.scratch_storage)),
            Catch::Matchers::Contains(selector_key + "' = 'seam_placer.cost'"));
    }
}

TEST_CASE("Legacy rear seam values remain readable but are hidden from choices",
          "[plugins][seam-placer][fixed][legacy]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    REQUIRE_NOTHROW(config.set_deserialize_strict("seam_position", "rear"));
    CHECK(config.opt_enum<SeamPosition>("seam_position") == spRear);

    const ConfigOptionDef *definition = PrintConfigDef::instance().get("seam_position");
    REQUIRE(definition != nullptr);
    REQUIRE(static_cast<bool>(definition->enum_def));
    CHECK(std::find(definition->enum_def->values().begin(),
                    definition->enum_def->values().end(),
                    "rear") == definition->enum_def->values().end());
}

TEST_CASE("Fixed-direction seam placement covers every compass direction",
          "[plugins][seam-placer][fixed][geometry]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print session_print;
    slic3r_api::SeamPlacer placer = create_fixed_session(session_print);
    ExtrusionEntity loop = square_loop();

    const std::array<std::pair<const char *, c_point>, 8> cases = {{
        {"rear", mm_point(10.0, 10.0)},
        {"front", mm_point(0.0, 0.0)},
        {"right", mm_point(10.0, 0.0)},
        {"left", mm_point(0.0, 10.0)},
        {"rear_right", mm_point(10.0, 10.0)},
        {"rear_left", mm_point(0.0, 10.0)},
        {"front_right", mm_point(10.0, 0.0)},
        {"front_left", mm_point(0.0, 0.0)}}};

    for (const std::pair<const char *, c_point> &entry : cases) {
        CAPTURE(entry.first);
        DirectionRegionFixture region(entry.first);
        const c_point first = placer.place_seam(
            extrusion_view(loop), region.view(), mm_point(-500.0, -500.0));
        const c_point second = placer.place_seam(
            extrusion_view(loop), region.view(), mm_point(500.0, 500.0));
        CHECK(same_point(first, entry.second));
        CHECK(same_point(second, entry.second));
    }
}

TEST_CASE("Fixed-direction uses stored vertices in composed and arc loops",
          "[plugins][seam-placer][fixed][geometry]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print session_print;
    slic3r_api::SeamPlacer placer = create_fixed_session(session_print);
    DirectionRegionFixture rear("rear");

    ExtrusionEntity composed = composed_loop();
    CHECK(same_point(
        placer.place_seam(extrusion_view(composed), rear.view(), {}),
        mm_point(10.0, 10.0)));

    ExtrusionEntity arcs = arc_loop();
    // The analytic arc reaches +Y, but that generated extremum is not a stored
    // vertex. Rear therefore ties the two endpoints and then chooses +X.
    CHECK(same_point(
        placer.place_seam(extrusion_view(arcs), rear.view(), {}),
        mm_point(10.0, 0.0)));
}

TEST_CASE("One Fixed-direction session reads each LayerRegion independently",
          "[plugins][seam-placer][fixed][region]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print session_print;
    slic3r_api::SeamPlacer placer = create_fixed_session(session_print);
    DirectionRegionFixture rear("rear");
    DirectionRegionFixture left("left");
    ExtrusionEntity loop = square_loop();

    CHECK(same_point(
        placer.place_seam(extrusion_view(loop), rear.view(), {}),
        mm_point(10.0, 10.0)));
    CHECK(same_point(
        placer.place_seam(extrusion_view(loop), left.view(), {}),
        mm_point(0.0, 10.0)));
}

TEST_CASE("Ordering resolves the first intersecting source LayerRegion",
          "[plugins][seam-placer][fixed][region-selection]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExPolygon left_area = rectangle_expolygon(-20.0, -20.0, 1.0, 20.0);
    const ExPolygon right_area = rectangle_expolygon(-1.0, -20.0, 20.0, 20.0);
    DirectionRegionFixture left("left", &left_area);
    DirectionRegionFixture right("right", &right_area);

    LayerRegionSetCPtrs regions = {left.region, right.region};
    LayerRegionIsland source(regions, 0);
    ExtrusionEntity crossing(false, ArcPolyline(Points{
        Point::new_scale(-5.0, 0.0), Point::new_scale(5.0, 0.0)}));
    source.mutable_extrusion(LayerRegionIsland::PERIMETERS).append(crossing);

    PrintingExtrusion native;
    native.region_island = &source;
    native.sregion_island_role = LayerRegionIsland::PERIMETERS;
    native.root.reset(crossing.clone());
    slic3r_api::PrintingExtrusion extrusion(
        reinterpret_cast<printing_extrusion_handle *>(&native));

    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const slic3r_api::LayerRegion selected =
        slic3r_api::Ordering::TreeOrderingGeometry::resolve_source_layer_region(
            storage, extrusion);
    REQUIRE_FALSE(source.regions().empty());
    CHECK(selected.handle() == reinterpret_cast<const layer_region_handle *>(
                                  *source.regions().begin()));
    plugin_storage.clear();

    LayerRegionIsland unique({right.region}, 0);
    unique.mutable_extrusion(LayerRegionIsland::PERIMETERS).append(crossing);
    native.region_island = &unique;
    const slic3r_api::LayerRegion unique_selected =
        slic3r_api::Ordering::TreeOrderingGeometry::resolve_source_layer_region(
            storage, extrusion);
    CHECK(unique_selected.handle() == reinterpret_cast<const layer_region_handle *>(
                                         right.region));
    plugin_storage.clear();

    // Region lookup follows the analytic source path rather than treating an
    // arc as the straight chord between its stored endpoints.
    const ExPolygon arc_area = rectangle_expolygon(-2.0, 8.0, 2.0, 12.0);
    DirectionRegionFixture arc_region("rear", &arc_area);
    LayerRegionIsland arc_source({arc_region.region}, 0);
    ExtrusionEntity source_arc = arc_loop();
    arc_source.mutable_extrusion(LayerRegionIsland::PERIMETERS).append(source_arc);
    native.region_island = &arc_source;
    native.root.reset(source_arc.clone());
    const slic3r_api::LayerRegion arc_selected =
        slic3r_api::Ordering::TreeOrderingGeometry::resolve_source_layer_region(
            storage, extrusion);
    CHECK(arc_selected.handle() == reinterpret_cast<const layer_region_handle *>(
                                      arc_region.region));
    plugin_storage.clear();

    const ExPolygon distant_area = rectangle_expolygon(100.0, 100.0, 110.0, 110.0);
    DirectionRegionFixture distant("rear", &distant_area);
    LayerRegionIsland missing({distant.region}, 0);
    missing.mutable_extrusion(LayerRegionIsland::PERIMETERS).append(crossing);
    native.region_island = &missing;
    CHECK_THROWS_WITH(
        slic3r_api::Ordering::TreeOrderingGeometry::resolve_source_layer_region(
            storage, extrusion),
        "A PrintingExtrusion does not intersect any LayerRegion from its source island.");
}

#include <future>
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/Plugins/SeamPlacer/SeamPlacementConstraints.hpp"
#include "libslic3r/Plugins/SeamPlacer/RandomSeamPlacer.hpp"

namespace {
/* Quarter arcs avoid the native diagnostic orientation ambiguity at exactly
   180 degrees; their caches are initialized before applying an instance shift. */
ExtrusionEntity constraint_arc_loop(double radius_mm = 10)
{
    const coord_t radius=scale_i(radius_mm);
    Geometry::ArcWelder::Path path;
    path.emplace_back(Point(-radius,0),0.f,Geometry::ArcWelder::Orientation::Unknown);
    path.emplace_back(Point(0,-radius),float(radius),Geometry::ArcWelder::Orientation::CCW);
    path.emplace_back(Point(radius,0),float(radius),Geometry::ArcWelder::Orientation::CCW);
    path.emplace_back(Point(0,radius),float(radius),Geometry::ArcWelder::Orientation::CCW);
    path.emplace_back(Point(-radius,0),float(radius),Geometry::ArcWelder::Orientation::CCW);
    ArcPolyline polyline(path);
    ApiInternal::ArcPolylineAccess::refresh_after_bulk_replace(polyline);
    return ExtrusionEntity(false,polyline);
}

/* Own real source metadata and a plan clone; modifier meshes are expressed in
   centered object coordinates then converted back to model coordinates. */
struct ConstraintFixture : DirectionRegionFixture
{
    ConstraintFixture(const char *direction = "rear") : DirectionRegionFixture(direction), source({region},0) {}
    LayerRegionIsland source;

    ModelVolume *box(double x,double y,double sx,double sy,ModelVolumeType type,
                     EnforcerBlockerType painting=EnforcerBlockerType::NONE,double z=0,double sz=30)
    {
        PrintObject &object=prepared.print.object(0);
        TriangleMesh mesh=make_cube(sx,sy,sz);
        mesh.translate(Vec3f(float(x),float(y),float(z)));
        mesh.transform(object.trafo_centered().inverse());
        ModelVolume *volume=const_cast<ModelObject *>(object.model_object())->add_volume(std::move(mesh),type,false);
        if (painting!=EnforcerBlockerType::NONE) {
            TriangleSelector selector(volume->mesh());
            for (size_t i=0;i<volume->mesh().its.indices.size();++i) selector.set_facet(int(i),painting);
            volume->facets_annotation_mutable("builtin:seam").set(selector);
        }
        return volume;
    }

    slic3r_api::PrintingExtrusion add(const ExtrusionEntity &loop, uint16_t instance=0)
    {
        slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(&prepared.print.mutable_printing_plan()));
        if (plan.group_count()==0) {
            plan.append_group().append_layer_group(view().layer().print_z()).append_tool_group(0);
        }
        const uint32_t instance_id = plan.find_or_add_object_instance(view().layer().object(), instance);
        slic3r_api::PrintingExtrusion extrusion=plan.group(0).layer_group(0).tool_group(0).append_extrusion_clone(
            plan,
            slic3r_api::LayerRegionIsland(reinterpret_cast<const layer_region_island_handle *>(&source)),
            RAW_EXTRUSION_ROLE_PERIMETER,extrusion_view(loop),instance_id);
        const c_point shift=view().layer().object().instance_shift(instance);
        std::function<void(slic3r_api::MutableExtrusionEntity)> translate;
        translate=[&](slic3r_api::MutableExtrusionEntity entity) {
            if (entity.point_count()>0) REQUIRE(entity.translate(shift));
            for (uint32_t i=0;i<entity.child_count();++i) translate(entity.child_mutable(i));
        };
        translate(extrusion.mutable_root());
        return extrusion;
    }

    c_point local(c_point point, uint16_t instance=0) const
    {
        const c_point shift=view().layer().object().instance_shift(instance);
        return {point.x-shift.x,point.y-shift.y};
    }
};

/* Execute RandomSession directly with exactly the provider region assignment. */
slic3r_api::SeamPlacer create_random_session_for(
    Print &print, const std::vector<slic3r_api::PrintRegion> &regions,
    PluginStorage &storage)
{
    PrintingPlan &native_plan = print.mutable_printing_plan();
    raw_seam_placer_instance instance = slic3r_api::make_seam_placer_instance(
        slic3r_api::SeamPlacement::RandomPlugin::make_random_seam_session(
            reinterpret_cast<storage_handle *>(&storage)),
        slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&native_plan)),
        regions);
    return slic3r_api::SeamPlacer(instance);
}

/* Append an invalid catalog reference on an island outside the assigned list. */
void append_unassigned_hostile_extrusion(
    RegionalRouterFixture &fixture,
    std::unique_ptr<LayerRegionIsland> &hostile_source)
{
    const PrintRegion *unused_region = fixture.add_unused_routing_region();
    REQUIRE(unused_region != nullptr);
    Layer &layer = fixture.prepared.print.object(0).layer(0);
    LayerRegion *hostile_region = &layer.region(layer.region_count() - 1);
    REQUIRE(&hostile_region->region() == unused_region);
    hostile_source.reset(new LayerRegionIsland({hostile_region}, 0));

    PrintObject &object = fixture.prepared.print.object(0);
    PrintingPlan &plan = fixture.prepared.print.mutable_printing_plan();
    PrintingToolGroup &tool = plan.groups.front().layers.front().tool_groups.front();
    tool.region_islands.push_back(hostile_source.get());
    const Point shift = object.instances().front().shift;
    std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(
        false, ArcPolyline(Points{
            shift + Point::new_scale(20, 20),
            shift + Point::new_scale(22, 20),
            shift + Point::new_scale(22, 22),
            shift + Point::new_scale(20, 22),
            shift + Point::new_scale(20, 20)})));
    PrintingExtrusion hostile;
    hostile.region_island = hostile_source.get();
    hostile.root = std::move(root);
    hostile.sregion_island_role = LayerRegionIsland::PERIMETERS;
    hostile.printing_catalog_id = plan.find_or_add_object_instance(object, 0);
    tool.extrusions.push_back(std::move(hostile));
    tool.extrusions.back().printing_catalog_id = 999999;
}

/* Append malformed plan-wide work with no source island and no valid catalog entry. */
void append_hostile_extrusion_without_island(RegionalRouterFixture &fixture)
{
    PrintObject &object = fixture.prepared.print.object(0);
    PrintingPlan &plan = fixture.prepared.print.mutable_printing_plan();
    PrintingToolGroup &tool = plan.groups.front().layers.front().tool_groups.front();
    const Point shift = object.instances().front().shift;
    PrintingExtrusion hostile;
    hostile.root.reset(new ExtrusionEntity(
        false, ArcPolyline(Points{
            shift + Point::new_scale(30, 30),
            shift + Point::new_scale(32, 30)})));
    hostile.sregion_island_role = LayerRegionIsland::PERIMETERS;
    hostile.printing_catalog_id = 999999;
    tool.extrusions.push_back(std::move(hostile));
}
}

TEST_CASE("Fixed seam constraints enforce narrow interior segments and true arcs", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture("right");
    const bool arc=GENERATE(false,true);
    fixture.box(arc?5.9:4.9,arc?7.8:-0.1,0.2,0.4,ModelVolumeType::SEAM_POSITION_INSIDE);
    ExtrusionEntity loop=arc?constraint_arc_loop():square_loop();
    const slic3r_api::PrintingExtrusion extrusion=fixture.add(loop);
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    const c_point point=fixture.local(session.place_seam(extrusion.root(),fixture.view(),{0,0}));
    CHECK(unscaled(point.x)>= (arc?5.899:4.899));
    CHECK(unscaled(point.x)<= (arc?6.101:5.101));
    if (arc) {
        CHECK(std::hypot(unscaled(point.x),unscaled(point.y))==Approx(10).margin(0.0001));
        CHECK(unscaled(point.y)>7.8);
    } else CHECK(point.y==0);
    CHECK(extrusion_view(loop).point_count()==extrusion.root().point_count());
}

TEST_CASE("Fixed seam constraints prioritize paint blockers over paint and volume enforcers", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    const bool volume=GENERATE(false,true);
    fixture.box(4,-1,2,2,volume?ModelVolumeType::SEAM_POSITION_INSIDE:ModelVolumeType::MODEL_PART,
                volume?EnforcerBlockerType::NONE:EnforcerBlockerType::ENFORCER);
    const bool block=GENERATE(false,true);
    if (block) fixture.box(3,-2,4,4,ModelVolumeType::MODEL_PART,EnforcerBlockerType::BLOCKER);
    const slic3r_api::PrintingExtrusion extrusion=fixture.add(square_loop());
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    const c_point point=fixture.local(session.place_seam(extrusion.root(),fixture.view(),{0,0}));
    if (block) CHECK(same_point(point,mm_point(10,10)));
    else {
        CHECK(point.y==0);
        CHECK(unscaled(point.x)>3);
        CHECK(unscaled(point.x)<7);
    }
}

TEST_CASE("Fixed seam constraints ignore attractors and absent layers and fall back when fully blocked", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    const int mode=GENERATE(0,1,2,3,4);
    if (mode<3) fixture.box(4,-1,2,2,mode==0?ModelVolumeType::SEAM_POSITION_CENTER:
        mode==1?ModelVolumeType::SEAM_POSITION_CENTER_Z:ModelVolumeType::SEAM_POSITION_INSIDE_CENTER);
    if (mode==3) fixture.box(4,-1,2,2,ModelVolumeType::SEAM_POSITION_INSIDE,EnforcerBlockerType::NONE,25,5);
    if (mode==4) fixture.box(-10,-10,40,40,ModelVolumeType::MODEL_PART,EnforcerBlockerType::BLOCKER);
    const slic3r_api::PrintingExtrusion extrusion=fixture.add(square_loop());
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    CHECK(same_point(fixture.local(session.place_seam(extrusion.root(),fixture.view(),{0,0})),mm_point(10,10)));
    if (mode==4) {
        PluginStorage storage;
        slic3r_api::SeamPlacement::SeamPlacementConstraints constraints;
        constraints.initialize(slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&fixture.prepared.print.mutable_printing_plan())),
                               reinterpret_cast<storage_handle *>(&storage));
        const slic3r_api::SeamPlacement::SeamPlacementConstraints::Result result=constraints.portions(extrusion.root(),fixture.view());
        REQUIRE(result.relevant);
        for (const slic3r_api::SeamPlacement::SeamPlacementConstraints::Portion &portion:result.portions)
            CHECK(portion.status==slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Blocked);
    }
}

TEST_CASE("Fixed seam constraint cache is immutable across parallel requests", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    fixture.box(4,-1,2,2,ModelVolumeType::SEAM_POSITION_INSIDE);
    slic3r_api::PrintingExtrusion extrusion=fixture.add(composed_loop());
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    const c_point expected=session.place_seam(extrusion.root(),fixture.view(),{0,0});
    std::vector<std::future<c_point>> futures;
    for (int i=0;i<8;++i) futures.push_back(std::async(std::launch::async,[&session,&fixture,&extrusion,i] {
        return session.place_seam(extrusion.root(),fixture.view(),{i*100,i*300});
    }));
    for (std::future<c_point> &future:futures) CHECK(same_point(future.get(),expected));
}

TEST_CASE("Seam constraints use the plan instance rather than the incoming position", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    PrintObject &object=fixture.prepared.print.object(0);
    // Synthetic second placement: keep the sliced source tree unchanged.
    PrintInstances &instances=const_cast<PrintInstances &>(object.instances());
    instances.push_back(instances.front());
    instances.back().shift+=Point::new_scale(100,-50);
    fixture.box(4,-1,2,2,ModelVolumeType::SEAM_POSITION_INSIDE);
    fixture.add(square_loop(),0);
    fixture.add(square_loop(),1);
    slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(&fixture.prepared.print.mutable_printing_plan()));
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    for (uint16_t i=0;i<2;++i) {
        const slic3r_api::PrintingExtrusion extrusion=plan.group(0).layer_group(0).tool_group(0).extrusion(i);
        const c_point point=fixture.local(session.place_seam(extrusion.root(),fixture.view(),{-100000,200000}),i);
        CHECK(point.y==0);
        CHECK(unscaled(point.x)==Approx(6).margin(0.001));
    }
}

TEST_CASE("Seam volume slices preserve holes and disconnected contours", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture("left");
    TriangleMesh mesh=make_cube(8,4,30);
    mesh.translate(Vec3f(1,-2,0));
    TriangleMesh hole=make_cube(4,2,30);
    hole.translate(Vec3f(3,-1,0));
    for (stl_triangle_vertex_indices &face:hole.its.indices) std::swap(face[0],face[1]);
    mesh.merge(hole);
    TriangleMesh separate=make_cube(0.2,0.2,30);
    separate.translate(Vec3f(-0.1f,4,0));
    mesh.merge(separate);
    PrintObject &object=fixture.prepared.print.object(0);
    mesh.transform(object.trafo_centered().inverse());
    const_cast<ModelObject *>(object.model_object())->add_volume(std::move(mesh),ModelVolumeType::SEAM_POSITION_INSIDE,false);
    const slic3r_api::PrintingExtrusion extrusion=fixture.add(square_loop());
    PluginStorage storage;
    slic3r_api::SeamPlacement::SeamPlacementConstraints constraints;
    constraints.initialize(slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&fixture.prepared.print.mutable_printing_plan())),
                           reinterpret_cast<storage_handle *>(&storage));
    CHECK(storage.size()==0);
    const c_point shift=fixture.view().layer().object().instance_shift(0);
    const std::pair<c_point,slic3r_api::SeamPlacement::SeamPlacementConstraints::Status> checks[]={
        {mm_point(2,0),slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Enforced},
        {mm_point(5,0),slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Free},
        {mm_point(0,4.1),slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Enforced}
    };
    for (const std::pair<c_point,slic3r_api::SeamPlacement::SeamPlacementConstraints::Status> &check:checks)
        CHECK(constraints.classify_point(extrusion.root(),{check.first.x+shift.x,check.first.y+shift.y},0)==check.second);
}

TEST_CASE("Seam volume transformations are composed in object space", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    PrintObject &object=fixture.prepared.print.object(0);
    TriangleMesh mesh=make_cube(2,0.2,30);
    ModelVolume *volume=const_cast<ModelObject *>(object.model_object())->add_volume(std::move(mesh),ModelVolumeType::SEAM_POSITION_INSIDE,false);
    Transform3d placement=Transform3d::Identity();
    placement.translate(Vec3d(5.1,-1,0));
    placement.rotate(Eigen::AngleAxisd(3.14159265358979323846/2,Vec3d::UnitZ()));
    volume->set_transformation(object.trafo_centered().inverse()*placement);
    const slic3r_api::PrintingExtrusion extrusion=fixture.add(square_loop());
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    const c_point point=fixture.local(session.place_seam(extrusion.root(),fixture.view(),{0,0}));
    CHECK(point.y==0);
    CHECK(unscaled(point.x)==Approx(5.1).margin(0.001));
}

TEST_CASE("Seam paint margins inherit width and fall back to regional flow", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    fixture.box(4,0.5,2,0.1,ModelVolumeType::MODEL_PART,EnforcerBlockerType::ENFORCER);
    slic3r_api::PrintingExtrusion extrusion=fixture.add(composed_loop());
    const bool explicit_width=GENERATE(false,true);
    if (explicit_width) {
        orchestrator_handle *orch=reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance());
        c_extrusion_property_attributes *parent=static_cast<c_extrusion_property_attributes *>(
            extrusion.mutable_root().get_or_add_property_data_mutable(orch,EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
        REQUIRE(parent!=nullptr);
        parent->width=0.8f;
        parent->role=RAW_EXTRUSION_ROLE_PERIMETER;
        c_extrusion_property_attributes *child=static_cast<c_extrusion_property_attributes *>(
            extrusion.mutable_root().child_mutable(1).get_or_add_property_data_mutable(orch,EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
        REQUIRE(child!=nullptr);
        child->width=0.2f;
        child->role=RAW_EXTRUSION_ROLE_PERIMETER;
    }
    PluginStorage storage;
    slic3r_api::SeamPlacement::SeamPlacementConstraints constraints;
    constraints.initialize(slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&fixture.prepared.print.mutable_printing_plan())),
                           reinterpret_cast<storage_handle *>(&storage));
    const slic3r_api::SeamPlacement::SeamPlacementConstraints::Result portions=constraints.portions(extrusion.root(),fixture.view());
    REQUIRE_FALSE(portions.portions.empty());
    CHECK(portions.portions.front().paint_margin==Approx(explicit_width?Slic3r::scale_d(0.8f):fixture.view().flow(RAW_EXTRUSION_ROLE_PERIMETER).width));
    if (explicit_width) {
        bool narrow=false;
        for (const slic3r_api::SeamPlacement::SeamPlacementConstraints::Portion &portion:portions.portions)
            narrow|=std::abs(portion.paint_margin-Slic3r::scale_d(0.2f))<1;
        CHECK(narrow);
        const c_point shift=fixture.view().layer().object().instance_shift(0),point=mm_point(5,0);
        CHECK(constraints.classify_point(extrusion.root(),{point.x+shift.x,point.y+shift.y},Slic3r::scale_d(0.8f))==
            slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Enforced);
        CHECK(constraints.classify_point(extrusion.root(),{point.x+shift.x,point.y+shift.y},Slic3r::scale_d(0.2f))==
            slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Free);
    }
    CHECK(storage.size()==0);
}

TEST_CASE("Seam constraints isolate source objects and layers in one plan", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture first, second;
    first.box(4,-1,2,2,ModelVolumeType::SEAM_POSITION_INSIDE,EnforcerBlockerType::NONE,0,1);
    second.box(1,-1,1,2,ModelVolumeType::SEAM_POSITION_INSIDE);
    const slic3r_api::PrintingExtrusion low=first.add(square_loop());
    PrintObject &object=first.prepared.print.object(0);
    LayerRegion &upper=object.layer(object.layer_count()-1).region(0);
    LayerRegionIsland upper_source({&upper},0);
    slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(&first.prepared.print.mutable_printing_plan()));
    const slic3r_api::LayerRegion upper_view(reinterpret_cast<const layer_region_handle *>(&upper));
    const ExtrusionEntity square=square_loop();
    const uint32_t high_id = plan.find_or_add_object_instance(upper_view.layer().object(), 0);
    slic3r_api::PrintingExtrusion high=plan.group(0).append_layer_group(upper_view.layer().print_z()).append_tool_group(0).append_extrusion_clone(
        plan,
        slic3r_api::LayerRegionIsland(reinterpret_cast<const layer_region_island_handle *>(&upper_source)),
        RAW_EXTRUSION_ROLE_PERIMETER,extrusion_view(square),high_id);
    REQUIRE(high.mutable_root().translate(upper_view.layer().object().instance_shift(0)));
    // A second independently sliced source object shares this synthetic plan.
    const uint32_t other_id = plan.find_or_add_object_instance(second.view().layer().object(), 0);
    slic3r_api::PrintingExtrusion other=plan.append_group().append_layer_group(second.view().layer().print_z()).append_tool_group(0).append_extrusion_clone(
        plan,
        slic3r_api::LayerRegionIsland(reinterpret_cast<const layer_region_island_handle *>(&second.source)),
        RAW_EXTRUSION_ROLE_PERIMETER,extrusion_view(square),other_id);
    REQUIRE(other.mutable_root().translate(second.view().layer().object().instance_shift(0)));
    slic3r_api::SeamPlacer session=create_fixed_session(first.prepared.print);
    CHECK(same_point(first.local(session.place_seam(low.root(),first.view(),{0,0})),mm_point(6,0)));
    CHECK(same_point(first.local(session.place_seam(high.root(),upper_view,{0,0})),mm_point(10,10)));
    const c_point other_point=second.local(session.place_seam(other.root(),second.view(),{0,0}));
    CHECK(other_point.y==0);
    CHECK(unscaled(other_point.x)==Approx(2).margin(0.00001));
}

TEST_CASE("Seam constraint boundaries and degenerate segments remain classified", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    fixture.box(4,-1,2,2,ModelVolumeType::SEAM_POSITION_INSIDE);
    slic3r_api::SeamPlacement::SeamPlacementConstraints::Curve degenerate;
    degenerate.segment.point_a=degenerate.segment.point_b=mm_point(4,0);
    CHECK(same_point(degenerate.point(0.5),mm_point(4,0)));
    const ExtrusionEntity loop(false,ArcPolyline(Points{Point::new_scale(0,0),
        Point::new_scale(10,0),Point::new_scale(10,10),Point::new_scale(0,10),Point::new_scale(0,0)}));
    const slic3r_api::PrintingExtrusion extrusion=fixture.add(loop);
    PluginStorage storage;
    slic3r_api::SeamPlacement::SeamPlacementConstraints constraints;
    constraints.initialize(slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&fixture.prepared.print.mutable_printing_plan())),
                           reinterpret_cast<storage_handle *>(&storage));
    const c_point shift=fixture.view().layer().object().instance_shift(0);
    {
        const slic3r_api::Object object=fixture.view().layer().object();
        const slic3r_api::Volume volume=object.volume(object.volume_count()-1);
        const c_point center=object.center_offset();
        const c_matrix4d transform=matrix4d_mul(
            matrix4d_translation(-unscaled(center.x),-unscaled(center.y),0),object.transform());
        // Use actual sliced boundaries, including native float-to-integer rounding.
        c_mesh_slicing_params slicing_params={};
        slicing_params.transform=matrix4d_mul(transform,volume_get_matrix(volume.handle()));
        const std::vector<slic3r_api::StoredPolygonCollection> slices=volume.mesh().slice_to_polygons(
            reinterpret_cast<storage_handle *>(&storage),slicing_params,
            {float(unscaled(fixture.view().layer().print_z()))},nullptr);
        REQUIRE(slices.front().readonly().size()>0);
        const slic3r_api::Polygon polygon=slices.front().readonly().at(0);
        for (uint32_t i=0;i<polygon.size();++i) {
            const c_point point=polygon.at(i);
            CHECK(constraints.classify_point(extrusion.root(),{point.x+shift.x,point.y+shift.y},0)==
                slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Enforced);
        }
    }
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    const c_point result=fixture.local(session.place_seam(extrusion.root(),fixture.view(),{0,0}));
    CHECK(result.y==0);
    CHECK(unscaled(result.x)==Approx(6).margin(0.00001));
    CHECK(storage.size()==0);
}


TEST_CASE("Seam AABB rejects distant painted edges and their infinite extensions", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    TriangleMesh mesh;
    // Diagonal supporting lines cross the loop although the finite edges are
    // far away. One dense annotated mesh avoids a separate volume per fragment.
    for (int i=0;i<128;++i) {
        TriangleMesh cube=make_cube(0.5,0.5,1);
        cube.rotate_z(float(3.14159265358979323846/4));
        cube.translate(Vec3f(float(30+i),float(30+i),0));
        mesh.merge(cube);
    }
    PrintObject &object=fixture.prepared.print.object(0);
    mesh.transform(object.trafo_centered().inverse());
    ModelVolume *volume=const_cast<ModelObject *>(object.model_object())->add_volume(std::move(mesh),ModelVolumeType::MODEL_PART,false);
    TriangleSelector selector(volume->mesh());
    for (size_t i=0;i<volume->mesh().its.indices.size();++i) selector.set_facet(int(i),EnforcerBlockerType::ENFORCER);
    volume->facets_annotation_mutable("builtin:seam").set(selector);
    const bool arc=GENERATE(false,true);
    const slic3r_api::PrintingExtrusion extrusion=fixture.add(arc?constraint_arc_loop():square_loop());
    PluginStorage storage;
    slic3r_api::SeamPlacement::SeamPlacementConstraints constraints;
    constraints.initialize(slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&fixture.prepared.print.mutable_printing_plan())),
                           reinterpret_cast<storage_handle *>(&storage));
    const slic3r_api::SeamPlacement::SeamPlacementConstraints::Result result=constraints.portions(extrusion.root(),fixture.view());
    CHECK_FALSE(result.relevant);
    // Only the two endpoints and one open interval per segment are necessary.
    CHECK(result.portions.size()==3*extrusion.root().segment_count());
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    const c_point seam=fixture.local(session.place_seam(extrusion.root(),fixture.view(),{}));
    CHECK(same_point(seam,arc?mm_point(0,10):mm_point(10,10)));
    CHECK(storage.size()==0);
}

TEST_CASE("Indexed seam classification preserves reversed paint fragments and deep interiors", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    fixture.box(-20,-20,50,50,ModelVolumeType::MODEL_PART,EnforcerBlockerType::ENFORCER);
    PrintObject &object=fixture.prepared.print.object(0);
    TriangleMesh reversed=make_cube(50,50,1);
    reversed.translate(Vec3f(-20,-20,0));
    for (stl_triangle_vertex_indices &face:reversed.its.indices) std::swap(face[0],face[1]);
    reversed.transform(object.trafo_centered().inverse());
    ModelVolume *volume=const_cast<ModelObject *>(object.model_object())->add_volume(std::move(reversed),ModelVolumeType::MODEL_PART,false);
    TriangleSelector selector(volume->mesh());
    for (size_t i=0;i<volume->mesh().its.indices.size();++i) selector.set_facet(int(i),EnforcerBlockerType::ENFORCER);
    volume->facets_annotation_mutable("builtin:seam").set(selector);
    slic3r_api::PrintingExtrusion extrusion=fixture.add(square_loop());
    PluginStorage storage;
    slic3r_api::SeamPlacement::SeamPlacementConstraints constraints;
    constraints.initialize(slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&fixture.prepared.print.mutable_printing_plan())),
                           reinterpret_cast<storage_handle *>(&storage));
    const slic3r_api::SeamPlacement::SeamPlacementConstraints::Result result=constraints.portions(extrusion.root(),fixture.view());
    REQUIRE(result.relevant);
    for (const slic3r_api::SeamPlacement::SeamPlacementConstraints::Portion &portion:result.portions)
        CHECK(portion.status==slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Enforced);
}

TEST_CASE("Indexed paint capsule endpoints bound admissible line portions", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture("right");
    fixture.box(4,0.3,2,0.1,ModelVolumeType::MODEL_PART,EnforcerBlockerType::ENFORCER,0,1);
    slic3r_api::PrintingExtrusion extrusion=fixture.add(square_loop());
    c_extrusion_property_attributes *attributes=static_cast<c_extrusion_property_attributes *>(
        extrusion.mutable_root().get_or_add_property_data_mutable(
            reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
    REQUIRE(attributes!=nullptr);
    attributes->width=0.5f;
    attributes->role=RAW_EXTRUSION_ROLE_PERIMETER;
    // The rightmost feasible point is on the endpoint cap, not the infinite
    // offset line: (x-6)^2 + 0.3^2 = 0.5^2, giving x=6.4.
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    const c_point seam=fixture.local(session.place_seam(extrusion.root(),fixture.view(),{}));
    CHECK(unscaled(seam.x)==Approx(6.4).margin(0.0001));
    CHECK(seam.y==0);
}


TEST_CASE("Seam index bounds major arcs beyond both stored endpoints", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture("right");
    fixture.box(19.8,9.9,0.4,0.2,ModelVolumeType::SEAM_POSITION_INSIDE);
    Geometry::ArcWelder::Path path;
    path.emplace_back(Point::new_scale(10,0),0.f,Geometry::ArcWelder::Orientation::Unknown);
    // Negative radius encodes the major CCW arc, centered at (10,10).
    path.emplace_back(Point::new_scale(0,10),-float(scale_i(10)),Geometry::ArcWelder::Orientation::CCW);
    path.emplace_back(Point::new_scale(10,0),0.f,Geometry::ArcWelder::Orientation::Unknown);
    ArcPolyline polyline(path);
    ApiInternal::ArcPolylineAccess::refresh_after_bulk_replace(polyline);
    const slic3r_api::PrintingExtrusion extrusion=fixture.add(ExtrusionEntity(false,polyline));
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    const c_point seam=fixture.local(session.place_seam(extrusion.root(),fixture.view(),{}));
    CHECK(unscaled(seam.x)==Approx(20).margin(0.0001));
    CHECK(unscaled(seam.y)==Approx(10).margin(0.0001));
}


// Opt-in benchmark against the pre-index exhaustive paint classification. The
// reference consumes the same native projection, including opposite winding.
TEST_CASE("Seam dense painting index benchmark", "[.][plugins][seam-index-benchmark]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (int count:{64,256,1024}) {
        ConstraintFixture fixture;
        TriangleMesh mesh;
        for (int i=0;i<count;++i) {
            TriangleMesh cube=make_cube(0.25,0.25,1);
            cube.translate(Vec3f(float(20+(i%32)*0.6),float((i/32)*0.6),0));
            mesh.merge(cube);
        }
        PrintObject &object=fixture.prepared.print.object(0);
        mesh.transform(object.trafo_centered().inverse());
        ModelVolume *volume=const_cast<ModelObject *>(object.model_object())->add_volume(std::move(mesh),ModelVolumeType::MODEL_PART,false);
        TriangleSelector selector(volume->mesh());
        for (size_t i=0;i<volume->mesh().its.indices.size();++i) selector.set_facet(int(i),EnforcerBlockerType::ENFORCER);
        volume->facets_annotation_mutable("builtin:seam").set(selector);
        const slic3r_api::PrintingExtrusion extrusion=fixture.add(square_loop());
        PluginStorage storage;
        slic3r_api::SeamPlacement::SeamPlacementConstraints constraints;
        const std::chrono::steady_clock::time_point start=std::chrono::steady_clock::now();
        constraints.initialize(slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&fixture.prepared.print.mutable_printing_plan())),
                               reinterpret_cast<storage_handle *>(&storage));
        const std::chrono::steady_clock::time_point built=std::chrono::steady_clock::now();
        const slic3r_api::Object source=fixture.view().layer().object();
        const std::vector<slic3r_api::StoredPolygonCollection> projected=slic3r_api::project_painting_to_polygons(
            reinterpret_cast<storage_handle *>(&storage),source,RAW_FACET_PAINTING_SEAM,RAW_FACET_PAINTING_ENFORCER);
        std::vector<std::vector<c_point>> rings;
        size_t edge_count=0;
        for (uint32_t i=0;i<projected.front().size();++i) {
            rings.push_back(projected.front().at(i).points());
            edge_count+=rings.back().size();
        }
        const double margin=scale_d(0.4);
        const c_point shift=source.instance_shift(0);
        std::vector<c_point> points;
        for (int i=0;i<512;++i)
            points.push_back(mm_point(19.5+(i%37)*0.51,-0.5+((i*17)%37)*0.51));
        std::vector<bool> fast,slow;
        const std::chrono::steady_clock::time_point query_start=std::chrono::steady_clock::now();
        for (c_point p:points)
            fast.push_back(constraints.classify_point(extrusion.root(),{p.x+shift.x,p.y+shift.y},margin)==
                slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Enforced);
        const std::chrono::steady_clock::time_point queried=std::chrono::steady_clock::now();
        for (c_point p:points) {
            bool inside=false;
            for (const std::vector<c_point> &ring:rings) {
                int winding=0;
                for (size_t i=0;i<ring.size();++i) {
                    const c_point a=ring[i],b=ring[(i+1)%ring.size()];
                    const double dx=double(b.x)-a.x,dy=double(b.y)-a.y,length2=dx*dx+dy*dy;
                    const double t=length2==0?0:std::clamp(((double(p.x)-a.x)*dx+(double(p.y)-a.y)*dy)/length2,0.,1.);
                    if (std::hypot(double(p.x)-a.x-t*dx,double(p.y)-a.y-t*dy)<=margin+1e-5) {inside=true;break;}
                    const double cross=dx*(double(p.y)-a.y)-dy*(double(p.x)-a.x);
                    if (a.y<=p.y && b.y>p.y && cross>0) ++winding;
                    if (a.y>p.y && b.y<=p.y && cross<0) --winding;
                }
                if (inside || winding!=0) {inside=true;break;}
            }
            slow.push_back(inside);
        }
        const std::chrono::steady_clock::time_point done=std::chrono::steady_clock::now();
        CHECK(fast==slow);
        std::cout << "Seam paint fragments=" << rings.size() << " edges=" << edge_count
                  << " prepare_ms=" << std::chrono::duration<double,std::milli>(built-start).count()
                  << " indexed_ms=" << std::chrono::duration<double,std::milli>(queried-query_start).count()
                  << " exhaustive_ms=" << std::chrono::duration<double,std::milli>(done-queried).count() << '\n';
    }
}


namespace {
/* Real source configs expose the dynamic plugin definitions even in their
   static object/region containers. Mutations are confined to test fixtures. */
void random_options(ConstraintFixture &fixture, bool vertices, bool align, int seed=42)
{
    PrintRegionConfig region=fixture.region->region().config();
    REQUIRE_NOTHROW(region.set_deserialize_strict("seam_random_vertices_only",vertices?"1":"0"));
    REQUIRE_NOTHROW(region.set_deserialize_strict("seam_random_align_inner_seams",align?"1":"0"));
    replace_region_config(*fixture.region, region);
    PrintObjectConfig &object=const_cast<PrintObjectConfig &>(fixture.prepared.print.object(0).config());
    REQUIRE_NOTHROW(object.set_deserialize_strict("seam_random_seed",std::to_string(seed)));
}

/* Attach public inherited role, width and hole properties to a plan clone. */
void random_tags(slic3r_api::MutableExtrusionEntity entity, bool external, bool hole=false, float width=0.4f)
{
    orchestrator_handle *orch=reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance());
    c_extrusion_property_attributes *attributes=static_cast<c_extrusion_property_attributes *>(
        entity.get_or_add_property_data_mutable(orch,EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
    REQUIRE(attributes!=nullptr);
    attributes->role=external?RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER:RAW_EXTRUSION_ROLE_PERIMETER;
    attributes->width=width;
    c_extrusion_property_perimeter *perimeter=static_cast<c_extrusion_property_perimeter *>(
        entity.get_or_add_property_data_mutable(orch,EXTRUSION_PROPERTY_TYPE_PERIMETER));
    REQUIRE(perimeter!=nullptr);
    perimeter->perimeter_flags=C_EXTRUSION_PERIMETER_FLAG_LOOP | (hole?C_EXTRUSION_PERIMETER_FLAG_HOLE:0);
}

ExtrusionEntity random_rectangle(double x, double y, double sx, double sy)
{
    return ExtrusionEntity(false,ArcPolyline(Points{Point::new_scale(x,y),Point::new_scale(x+sx,y),
        Point::new_scale(x+sx,y+sy),Point::new_scale(x,y+sy),Point::new_scale(x,y)}));
}
}

TEST_CASE("Random seam registers regional switches and an object seed", "[plugins][seam-placer][random]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orch=Orchestrator::instance();
    REQUIRE(orch.get_plugin("seam_placer.random")!=nullptr);
    raw_config_option_def definition{};
    REQUIRE(orchestrator_get_option_def(reinterpret_cast<orchestrator_handle *>(&orch),"seam_random_seed",&definition));
    CHECK(definition.container_type==RAW_CONTAINER_TYPE_OBJECT);
    CHECK(definition.type==RAW_CO_INT);
    CHECK(std::string(definition.default_serialized_value)=="42");
    CHECK(definition.has_min);
    CHECK(definition.min_value==0);
    PrintObjectConfig object;
    CHECK(object.has("seam_random_seed"));
    PrintRegionConfig region;
    CHECK(region.opt_bool("seam_random_vertices_only"));
    CHECK(region.opt_bool("seam_random_align_inner_seams"));
    const std::string selector=Slic3r::Steps::plugin_exclusive_group_option_key(orch,SEAM_PLACER,"seam_placer_plugin");
    REQUIRE(region.option(selector)!=nullptr);
    CHECK(region.option(selector)->serialize()=="seam_placer.cost");
}

TEST_CASE("Random regional preparation ignores hostile unassigned extrusions",
          "[plugins][seam-placer][random][regions][isolation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    RegionalRouterFixture fixture(true);
    std::unique_ptr<LayerRegionIsland> hostile_source;
    append_unassigned_hostile_extrusion(fixture, hostile_source);

    LayerRegion *assigned_region = &fixture.fixed_region->region() == fixture.routed_print_region ?
        fixture.fixed_region : fixture.routed_region;
    const slic3r_api::PrintRegion assigned(
        reinterpret_cast<const print_region_handle *>(fixture.routed_print_region));
    const std::vector<slic3r_api::PrintRegion> regions = {assigned};
    PluginStorage storage;
    const slic3r_api::SeamPlacer session = create_random_session_for(
        fixture.prepared.print, regions, storage);
    const slic3r_api::LayerRegion assigned_view(
        reinterpret_cast<const layer_region_handle *>(assigned_region));
    const c_point point = session.place_seam(fixture.loop_view(), assigned_view, {});
    const c_point shift = assigned_view.layer().object().instance_shift(0);
    const c_point local = {point.x - shift.x, point.y - shift.y};
    CHECK(unscaled(local.x) >= -5.0);
    CHECK(unscaled(local.x) <= 5.0);
    CHECK(unscaled(local.y) >= -1.0);
    CHECK(unscaled(local.y) <= 1.0);
}

TEST_CASE("Random regional preparation keeps a multi-region island for any assigned region",
          "[plugins][seam-placer][random][regions]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    RegionalRouterFixture fixture(true);
    LayerRegion *assigned_region = &fixture.fixed_region->region() == fixture.routed_print_region ?
        fixture.fixed_region : fixture.routed_region;
    const slic3r_api::PrintRegion assigned(
        reinterpret_cast<const print_region_handle *>(fixture.routed_print_region));
    const std::vector<slic3r_api::PrintRegion> regions = {assigned};
    PluginStorage storage;
    const slic3r_api::SeamPlacer session = create_random_session_for(
        fixture.prepared.print, regions, storage);
    const slic3r_api::LayerRegion assigned_view(
        reinterpret_cast<const layer_region_handle *>(assigned_region));
    CHECK_NOTHROW(session.place_seam(fixture.loop_view(), assigned_view, {}));
}

TEST_CASE("Random regional preparation rejects invalid assigned region views",
          "[plugins][seam-placer][random][regions]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    RegionalRouterFixture fixture(true);
    const slic3r_api::PrintRegion assigned(
        reinterpret_cast<const print_region_handle *>(fixture.routed_print_region));
    const std::vector<slic3r_api::PrintRegion> regions = {
        slic3r_api::PrintRegion(), assigned};
    PluginStorage storage;
    CHECK_THROWS_WITH(
        create_random_session_for(fixture.prepared.print, regions, storage),
        "A seam placer instance received an invalid PrintRegion.");
}

TEST_CASE("Random empty region assignment preserves all-region preparation",
          "[plugins][seam-placer][random][regions]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    RegionalRouterFixture fixture(true);
    std::unique_ptr<LayerRegionIsland> hostile_source;
    append_unassigned_hostile_extrusion(fixture, hostile_source);
    const std::vector<slic3r_api::PrintRegion> regions;
    PluginStorage storage;
    // Empty is the direct/factory shorthand for the historical plan-wide mode;
    // the malformed foreign extrusion must therefore remain visible to setup.
    CHECK_THROWS(create_random_session_for(fixture.prepared.print, regions, storage));
}

TEST_CASE("Random plan-wide preparation still validates work without an island",
          "[plugins][seam-placer][random][regions]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    RegionalRouterFixture fixture(true);
    append_hostile_extrusion_without_island(fixture);
    const std::vector<slic3r_api::PrintRegion> regions;
    PluginStorage storage;
    CHECK_THROWS(create_random_session_for(fixture.prepared.print, regions, storage));
}

TEST_CASE("Random seams deduplicate joins and remain stable across calls and sessions", "[plugins][seam-placer][random]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    random_options(fixture,true,false);
    std::vector<slic3r_api::ExtrusionEntity> loops;
    for (int i=0;i<256;++i) loops.push_back(fixture.add(composed_loop()).root());
    slic3r_api::SeamPlacer first=create_fixed_session(fixture.prepared.print,"seam_placer.random");
    // Its edge support lines cross the loop, but the actual zone is far away.
    // Auxiliary analytic cuts must not become extra vertex-mode candidates.
    fixture.box(4,40,2,1,ModelVolumeType::SEAM_POSITION_INSIDE);
    slic3r_api::SeamPlacer second=create_fixed_session(fixture.prepared.print,"seam_placer.random");
    std::map<std::pair<coord_t,coord_t>,int> histogram;
    for (const slic3r_api::ExtrusionEntity &loop:loops) {
        const c_point point=first.place_seam(loop,fixture.view(),{0,0});
        CHECK(same_point(point,second.place_seam(loop,fixture.view(),{100,200})));
        const c_point local=fixture.local(point);
        CHECK((local.x==0 || local.x==scale_i(10)));
        CHECK((local.y==0 || local.y==scale_i(10)));
        ++histogram[{local.x,local.y}];
        CHECK(loop.child_count()==3);
    }
    REQUIRE(histogram.size()==4);
    for (const std::pair<const std::pair<coord_t,coord_t>,int> &entry:histogram) {
        CHECK(entry.second>40);
        CHECK(entry.second<90);
    }
    std::vector<std::future<c_point>> futures;
    for (int i=0;i<8;++i) futures.push_back(std::async(std::launch::async,[&first,&loops,&fixture,i] {
        return first.place_seam(loops.front(),fixture.view(),{i*100,-i*300});
    }));
    for (std::future<c_point> &future:futures) CHECK(same_point(future.get(),first.place_seam(loops.front(),fixture.view(),{0,0})));
    random_options(fixture,true,false,43);
    slic3r_api::SeamPlacer changed=create_fixed_session(fixture.prepared.print,"seam_placer.random");
    size_t differences=0;
    for (const slic3r_api::ExtrusionEntity &loop:loops)
        differences+=!same_point(first.place_seam(loop,fixture.view(),{0,0}),changed.place_seam(loop,fixture.view(),{0,0}));
    CHECK(differences>100);
}

TEST_CASE("Random continuous seams are weighted by true segment length", "[plugins][seam-placer][random]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    random_options(fixture,false,false);
    std::vector<slic3r_api::ExtrusionEntity> loops;
    for (int i=0;i<256;++i) loops.push_back(fixture.add(random_rectangle(0,0,20,2)).root());
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print,"seam_placer.random");
    int horizontal=0, intermediate=0;
    for (const slic3r_api::ExtrusionEntity &loop:loops) {
        const c_point point=fixture.local(session.place_seam(loop,fixture.view(),{0,0}));
        horizontal+=point.y==0 || point.y==scale_i(2);
        intermediate+=(point.x!=0 && point.x!=scale_i(20)) || (point.y!=0 && point.y!=scale_i(2));
    }
    CHECK(horizontal>215);
    CHECK(horizontal<250);
    CHECK(intermediate==256);
}

TEST_CASE("Random seams include narrow constraint boundaries on lines and arcs", "[plugins][seam-placer][random]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    const bool vertices=GENERATE(false,true), arc=GENERATE(false,true);
    random_options(fixture,vertices,false);
    fixture.box(arc?5.9:4.9,arc?7.8:-0.1,0.2,0.4,ModelVolumeType::SEAM_POSITION_INSIDE);
    const slic3r_api::ExtrusionEntity loop=fixture.add(arc?constraint_arc_loop():square_loop()).root();
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print,"seam_placer.random");
    const c_point point=fixture.local(session.place_seam(loop,fixture.view(),{0,0}));
    CHECK(unscaled(point.x)>=(arc?5.899:4.899));
    CHECK(unscaled(point.x)<=(arc?6.101:5.101));
    if (arc) CHECK(std::hypot(unscaled(point.x),unscaled(point.y))==Approx(10).margin(0.00001));
    else CHECK(point.y==0);
    if (vertices) CHECK(std::min(std::abs(unscaled(point.x)-(arc?5.9:4.9)),std::abs(unscaled(point.x)-(arc?6.1:5.1)))<0.00001);
}

TEST_CASE("Random seams respect painting priority and the fully blocked fallback", "[plugins][seam-placer][random]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    random_options(fixture,true,false);
    const bool fully_blocked=GENERATE(false,true);
    fixture.box(4,-1,2,2,ModelVolumeType::SEAM_POSITION_INSIDE);
    fixture.box(fully_blocked?-10:3,fully_blocked?-10:-2,fully_blocked?40:4,fully_blocked?40:4,
                ModelVolumeType::MODEL_PART,EnforcerBlockerType::BLOCKER);
    const slic3r_api::ExtrusionEntity loop=fixture.add(square_loop()).root();
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print,"seam_placer.random");
    const c_point point=session.place_seam(loop,fixture.view(),{0,0});
    PluginStorage storage;
    slic3r_api::SeamPlacement::SeamPlacementConstraints constraints;
    constraints.initialize(slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&fixture.prepared.print.mutable_printing_plan())),
                           reinterpret_cast<storage_handle *>(&storage));
    CHECK(constraints.classify_point(loop,point,fixture.view().flow(RAW_EXTRUSION_ROLE_PERIMETER).width)==
        (fully_blocked?slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Blocked:slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Free));
    CHECK(storage.size()==0);
}

TEST_CASE("Random inner seams align by projection or nearby vertex", "[plugins][seam-placer][random]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    random_options(fixture,true,true);
    const float width=GENERATE(0.1f,10.f);
    slic3r_api::PrintingExtrusion outer=fixture.add(random_rectangle(0,0,20,20));
    random_tags(outer.mutable_root(),true);
    const slic3r_api::ExtrusionEntity outer_loop=outer.root();
    slic3r_api::PrintingExtrusion inner=fixture.add(random_rectangle(2,2,16,16));
    random_tags(inner.mutable_root(),false,false,width);
    // Force the external seam to the middle of its right edge, without
    // intersecting the internal loop. Projection must then be between vertices.
    fixture.box(19.9,9.9,0.2,0.2,ModelVolumeType::SEAM_POSITION_INSIDE);
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print,"seam_placer.random");
    const c_point external=fixture.local(session.place_seam(outer_loop,fixture.view(),{0,0}));
    const c_point internal=fixture.local(session.place_seam(inner.root(),fixture.view(),{0,0}));
    CHECK(unscaled(internal.x)==Approx(18).margin(0.00001));
    if (width<1) CHECK(internal.y==external.y);
    else CHECK((internal.y==scale_i(2) || internal.y==scale_i(18)));
}

TEST_CASE("Random seed zero stays stable during a session and unknown loops are rejected", "[plugins][seam-placer][random]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    random_options(fixture,false,false,0);
    const slic3r_api::ExtrusionEntity loop=fixture.add(constraint_arc_loop()).root();
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print,"seam_placer.random");
    const c_point first=session.place_seam(loop,fixture.view(),{0,0});
    for (int i=0;i<16;++i) CHECK(same_point(first,session.place_seam(loop,fixture.view(),{i,2*i})));
    CHECK(std::hypot(unscaled(fixture.local(first).x),unscaled(fixture.local(first).y))==Approx(10).margin(0.00001));
    const ExtrusionEntity unrelated=square_loop();
    CHECK_THROWS(session.place_seam(extrusion_view(unrelated),fixture.view(),{0,0}));
}


TEST_CASE("Random alignment prefers contours unless holes improve by a third width", "[plugins][seam-placer][random]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    random_options(fixture,true,true);
    const double improvement=GENERATE(0.99,1.0,1.01);
    const bool contour_present=GENERATE(false,true);
    slic3r_api::ExtrusionEntity contour;
    if (contour_present) {
        slic3r_api::PrintingExtrusion external=fixture.add(random_rectangle(0,-22,20,64));
        random_tags(external.mutable_root(),true);
        contour=external.root();
        fixture.box(19.9,9.999,0.2,0.002,ModelVolumeType::SEAM_POSITION_INSIDE);
    }
    const double right=16+improvement;
    slic3r_api::PrintingExtrusion hole=fixture.add(random_rectangle(5,5,right-5,10));
    random_tags(hole.mutable_root(),true,true);
    const slic3r_api::ExtrusionEntity hole_loop=hole.root();
    fixture.box(right-0.001,11.999,0.002,0.002,ModelVolumeType::SEAM_POSITION_INSIDE);
    slic3r_api::PrintingExtrusion farther_hole=fixture.add(random_rectangle(5,0,3,3));
    random_tags(farther_hole.mutable_root(),true,true);
    slic3r_api::PrintingExtrusion inner=fixture.add(random_rectangle(2,-20,16,60));
    random_tags(inner.mutable_root(),false,false,3.f);
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print,"seam_placer.random");
    const c_point h=fixture.local(session.place_seam(hole_loop,fixture.view(),{0,0}));
    const c_point result=fixture.local(session.place_seam(inner.root(),fixture.view(),{0,0}));
    CHECK(unscaled(h.x)==Approx(right).margin(0.00001));
    CHECK(unscaled(result.x)==Approx(18).margin(0.00001));
    if (!contour_present) CHECK(result.y==h.y);
    else {
        const c_point c=fixture.local(session.place_seam(contour,fixture.view(),{0,0}));
        // Compare actual integer coordinates, including native mesh slicing
        // round-off. Width 3 mm makes the threshold exactly 1 mm.
        const bool use_hole=(double(c.x)-scale_i(18))-(scale_i(18)-double(h.x))>=scale_i(1);
        CHECK(result.y==(use_hole?h.y:c.y));
        if (improvement<1) CHECK_FALSE(use_hole);
        if (improvement>1) CHECK(use_hole);
    }
}

TEST_CASE("Random alignment handles nested real arcs and excludes crossing contours", "[plugins][seam-placer][random]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    random_options(fixture,true,true);
    slic3r_api::PrintingExtrusion external=fixture.add(constraint_arc_loop());
    random_tags(external.mutable_root(),true);
    const slic3r_api::ExtrusionEntity outer=external.root();
    const bool crossing=GENERATE(false,true);
    // A circle of radius 8 fits inside radius 10; a shifted circle crosses it.
    ArcPolyline path=constraint_arc_loop(8).as_polyline();
    if (crossing) path.translate(Point::new_scale(6,0));
    const ExtrusionEntity smaller(false,std::move(path));
    slic3r_api::PrintingExtrusion internal=fixture.add(smaller);
    random_tags(internal.mutable_root(),false,false,0.1f);
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print,"seam_placer.random");
    const c_point point=fixture.local(session.place_seam(internal.root(),fixture.view(),{0,0}));
    if (!crossing) {
        const c_point master=fixture.local(session.place_seam(outer,fixture.view(),{0,0}));
        CHECK(unscaled(point.x)==Approx(unscaled(master.x)*0.8).margin(0.00001));
        CHECK(unscaled(point.y)==Approx(unscaled(master.y)*0.8).margin(0.00001));
    } else {
        random_options(fixture,true,false);
        slic3r_api::SeamPlacer independent=create_fixed_session(fixture.prepared.print,"seam_placer.random");
        CHECK(same_point(point,fixture.local(independent.place_seam(internal.root(),fixture.view(),{0,0}))));
    }
}


TEST_CASE("Random seed entropy is consumed only once per zero-seeded object", "[plugins][seam-placer][random]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    const bool zero=GENERATE(false,true);
    random_options(fixture,false,false,zero?0:42);
    std::vector<slic3r_api::ExtrusionEntity> loops;
    for (int i=0;i<4;++i) loops.push_back(fixture.add(square_loop()).root());
    PluginStorage storage;
    int calls=0;
    std::vector<c_point> previous;
    for (int run=0;run<2;++run) {
        raw_seam_placer_instance instance=slic3r_api::make_seam_placer_instance(
            slic3r_api::SeamPlacement::RandomPlugin::make_random_seam_session(reinterpret_cast<storage_handle *>(&storage),[&] {
                ++calls; return uint64_t(12345);
            }),
            slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&fixture.prepared.print.mutable_printing_plan())),
            {fixture.view().print_region()});
        slic3r_api::SeamPlacer session(instance);
        for (size_t i=0;i<loops.size();++i) {
            const c_point point=session.place_seam(loops[i],fixture.view(),{0,0});
            if (run==0) previous.push_back(point);
            else CHECK(same_point(point,previous[i]));
        }
        CHECK(storage.size()==0);
        CHECK(calls==(zero?run+1:0));
    }
}

TEST_CASE("Random seams keep independent instance coordinates and object seeds", "[plugins][seam-placer][random]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture first, second;
    random_options(first,false,false,41);
    random_options(second,false,false,83);
    PrintInstances &instances=const_cast<PrintInstances &>(first.prepared.print.object(0).instances());
    instances.push_back(instances.front());
    instances.back().shift+=Point::new_scale(100,-50);
    const slic3r_api::ExtrusionEntity a=first.add(square_loop(),0).root();
    const slic3r_api::ExtrusionEntity b=first.add(square_loop(),1).root();
    slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(&first.prepared.print.mutable_printing_plan()));
    const ExtrusionEntity square=square_loop();
    const uint32_t other_id = plan.find_or_add_object_instance(second.view().layer().object(), 0);
    slic3r_api::PrintingExtrusion other=plan.append_group().append_layer_group(second.view().layer().print_z()).append_tool_group(0).append_extrusion_clone(
        plan,
        slic3r_api::LayerRegionIsland(reinterpret_cast<const layer_region_island_handle *>(&second.source)),
        RAW_EXTRUSION_ROLE_PERIMETER,extrusion_view(square),other_id);
    REQUIRE(other.mutable_root().translate(second.view().layer().object().instance_shift(0)));
    PluginStorage storage;
    const std::function<slic3r_api::SeamPlacer()> create=[&] {
        raw_seam_placer_instance instance=slic3r_api::make_seam_placer_instance(
            slic3r_api::SeamPlacement::RandomPlugin::make_random_seam_session(reinterpret_cast<storage_handle *>(&storage)),
            plan,{first.view().print_region(),second.view().print_region()});
        return slic3r_api::SeamPlacer(instance);
    };
    slic3r_api::SeamPlacer session=create();
    const c_point original=session.place_seam(a,first.view(),{0,0});
    for (const std::pair<slic3r_api::ExtrusionEntity,uint16_t> &entry:{std::make_pair(a,uint16_t(0)),std::make_pair(b,uint16_t(1))}) {
        const c_point point=first.local(session.place_seam(entry.first,first.view(),{0,0}),entry.second);
        CHECK(point.x>=0); CHECK(point.x<=scale_i(10)); CHECK(point.y>=0); CHECK(point.y<=scale_i(10));
    }
    const c_point other_original=session.place_seam(other.root(),second.view(),{0,0});
    random_options(second,false,false,84);
    slic3r_api::SeamPlacer changed=create();
    CHECK(same_point(original,changed.place_seam(a,first.view(),{0,0})));
    CHECK_FALSE(same_point(other_original,changed.place_seam(other.root(),second.view(),{0,0})));
    CHECK(storage.size()==0);
}


TEST_CASE("Random curve utilities handle degenerate segments and clockwise major arcs", "[plugins][seam-placer][random]")
{
    using Curve=slic3r_api::SeamPlacement::SeamPlacementConstraints::Curve;
    c_extrusion_segment segment{};
    segment.point_a=segment.point_b=mm_point(3,4);
    const Curve point=Curve::from_segment(segment);
    CHECK(point.length()==0);
    CHECK(same_point(point.point(point.closest_parameter(mm_point(10,10))),mm_point(3,4)));
    segment.point_a=mm_point(10,0);
    segment.point_b=mm_point(0,10);
    segment.radius=-float(scale_i(10));
    segment.orientation=RAW_EXTRUSION_ARC_ORIENTATION_CW;
    const Curve major=Curve::from_segment(segment);
    CHECK(major.length()==Approx(scale_i(10)*3*3.14159265358979323846/2));
    CHECK(same_point(major.point(major.closest_parameter(mm_point(-20,0))),mm_point(-10,0)));
}


/*
Select Random by its registered enum identifier for one marked plan loop and compare the routed
seam with the direct session's point. The provider guard restores activation, the copied region
configuration is replaced in the owned print, and router views borrow that print until completion.
Missing enum values or provider selection errors fail the Catch2 test.
*/
TEST_CASE("Regional seam router selects the configured Random provider", "[plugins][seam-placer][random][router]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedSeamProviders active({"seam_placer.random"});
    ConstraintFixture fixture;
    random_options(fixture,false,false);
    const std::string selector=Slic3r::Steps::plugin_exclusive_group_option_key(Orchestrator::instance(),SEAM_PLACER,"seam_placer_plugin");
    PrintRegionConfig config=fixture.region->region().config();
    REQUIRE(config.opt<ConfigOptionInt>(selector) != nullptr);
    config.set_deserialize_strict(selector, "seam_placer.random");
    replace_region_config(*fixture.region, config);
    slic3r_api::PrintingExtrusion extrusion=fixture.add(square_loop());
    mark_perimeter_loop(extrusion.mutable_root());
    const slic3r_api::ExtrusionEntity loop=extrusion.root();
    slic3r_api::SeamPlacer reference=create_fixed_session(fixture.prepared.print,"seam_placer.random");
    PluginStorage storage;
    slic3r_api::SeamPlacement::SeamPlacerRouter router(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&fixture.prepared.print)),
        slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&fixture.prepared.print.mutable_printing_plan())),
        reinterpret_cast<storage_handle *>(&storage));
    CHECK(same_point(router.place_seam(loop,{0,0}),reference.place_seam(loop,fixture.view(),{100,200})));
    CHECK(storage.size()==0);
}


TEST_CASE("Regional registry survives apply and rebinds retained layers", "[plugins][print-region-registry]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    DynamicPrintConfig config = perimeter_config({{k_direction_key, "rear"}});
    prepare_cube_print(prepared, config);
    Print &print = prepared.print;
    // The fixture ran the slicing prefix directly; record it so apply() keeps
    // those layers as the Slicing checkpoint instead of destroying them.
    for (const slicing_step_t step : {STEP_LAYER_HEIGHT, STEP_SLICING, STEP_POST_SLICING})
        print.mark_step_executed(step);
    const PrintRegion *original = &print.object(0).printing_region(0);
    const PrintRegionConfig saved = original->config();
    Layer *retained = &print.object(0).layer(0);
    const size_t region_count = print.num_print_regions();
    print.apply(prepared.model, config);
    CHECK(&print.object(0).printing_region(0) == original);
    CHECK(&print.object(0).layer(0) == retained);

    config.set_deserialize_strict("perimeters", std::to_string(saved.perimeters.value + 2));
    print.apply(prepared.model, config);
    const PrintRegion *changed = &print.object(0).printing_region(0);
    REQUIRE(changed != original);
    CHECK(original->config() == saved);
    CHECK(&print.object(0).layer(0) == retained);
    CHECK(&retained->region(0).region() == changed);
    CHECK(print.num_print_regions() == region_count);
    for (const PrintRegion &active : print.print_regions()) CHECK(&active != original);
    print.request_replay_from_worker(Steps::Checkpoint::None);
    CHECK(original->config() == saved);
    config.set_deserialize_strict("perimeters", std::to_string(saved.perimeters.value));
    print.apply(prepared.model, config);
    CHECK(&print.object(0).printing_region(0) == original);
    CHECK(&print.mutable_auxiliary_object().printing_region(0) == original);
    CHECK(changed->config().perimeters.value == saved.perimeters.value + 2);
}

TEST_CASE("Independent objects share canonical regions and keep local slicing indices", "[plugins][print-region-registry]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    const DynamicPrintConfig config = perimeter_config({{k_direction_key, "rear"}});
    prepare_cube_print(prepared, config);
    ModelObject *copy = prepared.model.add_object(prepared.model.get_object(0));
    copy->instances.front()->set_offset(Vec3d(50., 0., 0.));
    prepared.print.apply(prepared.model, config);
    REQUIRE(prepared.print.objects().size() == 2);
    CHECK(&prepared.print.object(0).printing_region(0) == &prepared.print.object(1).printing_region(0));
    const size_t shared_count = prepared.print.object(0).num_printing_regions();
    CHECK(prepared.print.num_print_regions() == shared_count);

    // Separate one volume, then merge it back. The historical distinct snapshot
    // survives while current local tables and volume indices are rebuilt.
    copy->volumes.front()->config.set_deserialize_strict("perimeters", "13");
    prepared.print.apply(prepared.model, config);
    const PrintObject &changed_object = prepared.print.object(1);
    REQUIRE(changed_object.num_printing_regions() > shared_count);
    const PrintRegion *historical = &changed_object.printing_region(shared_count);
    for (const PrintObject &object : prepared.print.objects()) {
        const PrintObjectRegions &associations = *object.shared_regions();
        for (const PrintObjectRegions::LayerRangeRegions &range : associations.layer_ranges)
            for (const PrintObjectRegions::VolumeRegion &volume : range.volume_regions)
                if (volume.region != nullptr) {
                    REQUIRE(volume.region_idx >= 0);
                    CHECK(associations.all_regions.at(volume.region_idx) == volume.region);
                }
    }
    copy->volumes.front()->config.erase("perimeters");
    prepared.print.apply(prepared.model, config);
    CHECK(prepared.print.object(1).num_printing_regions() == shared_count);
    CHECK(historical->config().perimeters.value == 13);
    for (const PrintRegion &active : prepared.print.print_regions()) CHECK(&active != historical);
}

TEST_CASE("Seam router initializes an auxiliary-only canonical region", "[plugins][seam-placer][router][print-region-registry]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedSeamProviders active({k_fixed_plugin, k_routing_plugin});
    g_routing_provider.reset();
    RegionalRouterFixture fixture(false);
    Print &print = fixture.prepared.print;
    PrintObject &auxiliary = print.mutable_auxiliary_object();
    PrintRegionConfig config = fixture.routed_print_region->config();
    config.perimeters.value += 5;
    const PrintRegion *region = &print.register_print_region(config);
    PrintObjectRegions &associations = *const_cast<PrintObjectRegions *>(auxiliary.shared_regions());
    associations.all_regions.push_back(region);
    Layer &layer = auxiliary.add_auxiliary_layer(0, scale_i(0.2), scale_i(0.2));
    ApiInternal::LayerAccess::add_region(layer, *region);
    LayerRegion &local = layer.region(0);
    ApiInternal::LayerRegionAccess::slices_mutable(local) =
        ApiInternal::LayerRegionAccess::slices_mutable(*fixture.fixed_region);
    fixture.source.reset(new LayerRegionIsland({&local}, 0));
    PrintingExtrusion &extrusion = print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front().extrusions.front();
    extrusion.region_island = fixture.source.get();
    extrusion.printing_catalog_id = PRINTING_OBJECT_INSTANCE_ID_NONE;
    const Point shift = print.object(0).instances().front().shift;
    slic3r_api::MutableExtrusionEntity root(reinterpret_cast<extrusion_entity_handle *>(extrusion.root.get()));
    REQUIRE(root.translate({-shift.x(), -shift.y()}));
    fixture.source->mutable_extrusion(LayerRegionIsland::PERIMETERS).append(*extrusion.root);
    {
        slic3r_api::SeamPlacement::SeamPlacerRouter router(
            reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
            fixture.print_view(), fixture.plan_view(),
            reinterpret_cast<storage_handle *>(&fixture.scratch_storage));
        CHECK(std::find(g_routing_provider.initialized_regions.begin(), g_routing_provider.initialized_regions.end(),
              reinterpret_cast<const print_region_handle *>(region)) != g_routing_provider.initialized_regions.end());
        CHECK_NOTHROW(router.place_seam(fixture.loop_view(), {}));
    }
    // Destroy a live plan before its auxiliary source layers and registry.
    print.clear();
    CHECK(print.num_print_regions() == 0);
}


TEST_CASE("Painted regions retain object-local indices across reslicing", "[plugins][print-region-registry][slicing]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    DynamicPrintConfig config = perimeter_config({{k_direction_key, "rear"}});
    prepare_cube_print(prepared, config);
    config.set_num_extruders(2);
    config.set_deserialize_strict("nozzle_diameter", "0.4,0.4");
    ModelVolume &volume = *prepared.model.get_object(0).volumes.front();
    TriangleSelector paint(volume.mesh());
    paint.set_facet(0, static_cast<EnforcerBlockerType>(2));
    volume.mm_segmentation_facets.set(paint);
    prepared.print.apply(prepared.model, config);
    const PrintObjectRegions &associations = *prepared.print.object(0).shared_regions();
    size_t painted_count = 0;
    for (const PrintObjectRegions::LayerRangeRegions &range : associations.layer_ranges)
        for (const PrintObjectRegions::PaintedRegion &region : range.painted_regions) {
            ++painted_count;
            REQUIRE(region.region_idx >= 0);
            CHECK(associations.all_regions.at(region.region_idx) == region.region);
        }
    REQUIRE(painted_count > 0);
    Steps::StepLayerHeightGeneration::run_step(Orchestrator::instance(), prepared.print);
    Steps::StepSlicing::run_step(Orchestrator::instance(), prepared.print);
    const PrintObject &object = prepared.print.object(0);
    REQUIRE(object.layer_count() > 0);
    for (const Layer &layer : object.layers()) {
        REQUIRE(layer.region_count() == associations.all_regions.size());
        for (size_t idx = 0; idx < layer.region_count(); ++idx)
            CHECK(&layer.region(idx).region() == associations.all_regions[idx]);
    }
}
