#include <catch2/catch.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/ArcPolylineAccess.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SeamPlacerViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Geometry/ArcWelder.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Plugins/SeamPlacer/AlignedSeamPlacer.hpp"
#include "libslic3r/Plugins/SeamPlacer/ContiguousSeamPlacer.hpp"
#include "libslic3r/Plugins/SeamPlacer/CostSeamPlacer.hpp"
#include "libslic3r/Plugins/SeamPlacer/SeamCandidatePreparation.hpp"
#include "libslic3r/Plugins/SeamPlacer/SeamCostModel.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintObjectRegion.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/TriangleSelector.hpp"

// Cost seam tests construct real placement sessions and model geometry to check candidate scoring, visibility and config defaults.

/*
Cost, Aligned and Contiguous seam provider tests
================================================
These focused checks exercise built-in metadata and published immutable
sessions over real generated PrintingPlans. Functional cases ask the service
adapter to validate every returned point, so line/arc projection cannot
silently return a point outside the requested loop.
*/

namespace {

using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::PerimeterPluginTests;

const char *const k_cost_plugin = "seam_placer.cost";
const char *const k_aligned_plugin = "seam_placer.aligned";
const char *const k_contiguous_plugin = "seam_placer.contiguous";
const char *const k_fixed_plugin = "seam_placer.fixed";
const char *const k_random_plugin = "seam_placer.random";
const char *const k_regional_seam_plugin = "slic3r.test.seam_placer.regional";

struct CancellationProbe {
    size_t checks = 0;
    size_t cancel_after = std::numeric_limits<size_t>::max();
};

struct ProgressProbe {
    std::vector<int> percentages;
};

int cancel_after_checks(plugin_host_context *host_context)
{
    CancellationProbe &probe = *reinterpret_cast<CancellationProbe *>(
        host_context);
    ++probe.checks;
    return probe.checks >= probe.cancel_after ? 1 : 0;
}

void capture_progress(plugin_host_context *host_context,
                      const double progress,
                      const char *message)
{
    ProgressProbe &probe = *reinterpret_cast<ProgressProbe *>(host_context);
    if (message != nullptr && std::string(message) == "Calculating seam visibility")
        probe.percentages.push_back(int(progress * 100.0 + 0.5));
}

bool find_loop(const slic3r_api::ExtrusionEntity &entity,
               slic3r_api::ExtrusionEntity &loop)
{
    if (entity.is_loop() && entity.point_count() != 0) {
        loop = entity;
        return true;
    }
    for (uint32_t index = 0; index < entity.child_count(); ++index)
        if (find_loop(entity.child(index), loop)) return true;
    return false;
}

slic3r_api::SeamPlacer create_cost_session(
    Print &print, const plugin_run_context *run_context = nullptr)
{
    PrintingPlan &native_plan = print.mutable_printing_plan();
    const slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(&native_plan));
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    std::vector<slic3r_api::PrintRegion> regions;
    for (uint32_t object_index = 0; object_index < print_view.object_count(); ++object_index) {
        const slic3r_api::Object object = print_view.object(object_index);
        for (uint32_t region_index = 0; region_index < object.print_region_count(); ++region_index)
            regions.push_back(object.print_region(region_index));
    }
    PluginStorage storage;
    std::unique_ptr<slic3r_api::SeamPlacerSession> session = run_context == nullptr ?
        slic3r_api::SeamPlacement::CostPlugin::make_cost_seam_session(
            reinterpret_cast<storage_handle *>(&storage)) :
        slic3r_api::SeamPlacement::CostPlugin::make_cost_seam_session(
            reinterpret_cast<storage_handle *>(&storage), run_context);
    raw_seam_placer_instance instance = slic3r_api::make_seam_placer_instance(
        std::move(session), plan, regions);
    return slic3r_api::SeamPlacer(instance);
}

slic3r_api::SeamPlacer create_cost_session_for(
    Print &print, const std::vector<slic3r_api::PrintRegion> &regions)
{
    PrintingPlan &native_plan = print.mutable_printing_plan();
    PluginStorage storage;
    raw_seam_placer_instance instance = slic3r_api::make_seam_placer_instance(
        slic3r_api::SeamPlacement::CostPlugin::make_cost_seam_session(
            reinterpret_cast<storage_handle *>(&storage)),
        slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&native_plan)),
        regions);
    return slic3r_api::SeamPlacer(instance);
}

/* Initialize Aligned through the same ABI holder used by the production router. */
slic3r_api::SeamPlacer create_aligned_session(Print &print)
{
    PrintingPlan &native_plan = print.mutable_printing_plan();
    const slic3r_api::PrintingPlan plan(
        reinterpret_cast<printing_plan_handle *>(&native_plan));
    const slic3r_api::Print print_view(
        reinterpret_cast<const print_handle *>(&print));
    std::vector<slic3r_api::PrintRegion> regions;
    for (uint32_t object_index = 0;
         object_index < print_view.object_count(); ++object_index) {
        const slic3r_api::Object object = print_view.object(object_index);
        for (uint32_t region_index = 0;
             region_index < object.print_region_count(); ++region_index)
            regions.push_back(object.print_region(region_index));
    }
    PluginStorage storage;
    raw_seam_placer_instance instance = slic3r_api::make_seam_placer_instance(
        slic3r_api::SeamPlacement::AlignedPlugin::make_aligned_seam_session(
            reinterpret_cast<storage_handle *>(&storage)), plan, regions);
    return slic3r_api::SeamPlacer(instance);
}

/* Initialize Contiguous through the ABI holder used by the production router. */
slic3r_api::SeamPlacer create_contiguous_session(Print &print)
{
    PrintingPlan &native_plan = print.mutable_printing_plan();
    const slic3r_api::PrintingPlan plan(
        reinterpret_cast<printing_plan_handle *>(&native_plan));
    const slic3r_api::Print print_view(
        reinterpret_cast<const print_handle *>(&print));
    std::vector<slic3r_api::PrintRegion> regions;
    for (uint32_t object_index = 0;
         object_index < print_view.object_count(); ++object_index) {
        const slic3r_api::Object object = print_view.object(object_index);
        for (uint32_t region_index = 0;
             region_index < object.print_region_count(); ++region_index)
            regions.push_back(object.print_region(region_index));
    }
    PluginStorage storage;
    raw_seam_placer_instance instance = slic3r_api::make_seam_placer_instance(
        slic3r_api::SeamPlacement::ContiguousPlugin::
            make_contiguous_seam_session(
                reinterpret_cast<storage_handle *>(&storage)),
        plan, regions);
    return slic3r_api::SeamPlacer(instance);
}

ExtrusionEntity rectangle_loop(double left, double bottom, double right, double top,
                               bool reverse = false)
{
    Points points{Point::new_scale(left, bottom), Point::new_scale(right, bottom),
                  Point::new_scale(right, top), Point::new_scale(left, top),
                  Point::new_scale(left, bottom)};
    if (reverse) std::reverse(points.begin(), points.end());
    return ExtrusionEntity(false, ArcPolyline(points));
}

ExtrusionEntity rounded_loop(double leftmost_x, double center_y = 0.0,
                             double radius = 0.15)
{
    Points points;
    points.reserve(13);
    const double center_x = leftmost_x + radius;
    for (size_t index = 0; index <= 12; ++index) {
        const double angle = PI + 2.0 * PI * double(index) / 12.0;
        points.emplace_back(Point::new_scale(
            center_x + radius * std::cos(angle),
            center_y + radius * std::sin(angle)));
    }
    return ExtrusionEntity(false, ArcPolyline(points));
}

ExtrusionEntity arc_loop()
{
    const coord_t radius = scale_i(10.0);
    Geometry::ArcWelder::Path path;
    path.emplace_back(Point(-radius, 0), 0.f, Geometry::ArcWelder::Orientation::Unknown);
    path.emplace_back(Point(0, -radius), float(radius), Geometry::ArcWelder::Orientation::CCW);
    path.emplace_back(Point(radius, 0), float(radius), Geometry::ArcWelder::Orientation::CCW);
    path.emplace_back(Point(0, radius), float(radius), Geometry::ArcWelder::Orientation::CCW);
    path.emplace_back(Point(-radius, 0), float(radius), Geometry::ArcWelder::Orientation::CCW);
    ArcPolyline polyline(path);
    ApiInternal::ArcPolylineAccess::refresh_after_bulk_replace(polyline);
    return ExtrusionEntity(false, polyline);
}

/* Add one model-space seam modifier to a generated object's constraint input. */
ModelVolume *add_seam_constraint_box(
    PrintObject &object, double x, double y, double sx, double sy,
    ModelVolumeType type, EnforcerBlockerType painting,
    double z, double height)
{
    TriangleMesh mesh = make_cube(sx, sy, height);
    mesh.translate(Vec3f(float(x), float(y), float(z)));
    mesh.transform(object.trafo_centered().inverse());
    ModelVolume *volume = const_cast<ModelObject *>(object.model_object())->add_volume(
        std::move(mesh), type, false);
    if (painting != EnforcerBlockerType::NONE) {
        TriangleSelector selector(volume->mesh());
        for (size_t index = 0; index < volume->mesh().its.indices.size(); ++index)
            selector.set_facet(int(index), painting);
        volume->facets_annotation_mutable("builtin:seam").set(selector);
    }
    return volume;
}

/* Attach the metadata layout produced by Arachne, where open child paths carry
   perimeter classification while their continuous parent owns loop identity. */
void mark_perimeter_path(slic3r_api::MutableExtrusionEntity entity,
                         raw_extrusion_role role, float width)
{
    orchestrator_handle *orchestrator = reinterpret_cast<orchestrator_handle *>(
        &Orchestrator::instance());
    c_extrusion_property_attributes *attributes = static_cast<
        c_extrusion_property_attributes *>(entity.get_or_add_property_data_mutable(
            orchestrator, EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
    REQUIRE(attributes != nullptr);
    attributes->role = role;
    attributes->width = width;
    c_extrusion_property_perimeter *perimeter = static_cast<
        c_extrusion_property_perimeter *>(entity.get_or_add_property_data_mutable(
            orchestrator, EXTRUSION_PROPERTY_TYPE_PERIMETER));
    REQUIRE(perimeter != nullptr);
    perimeter->perimeter_flags = C_EXTRUSION_PERIMETER_FLAG_LOOP;
}

struct CostFixture
{
    explicit CostFixture(std::initializer_list<std::pair<std::string, std::string>> settings =
                             {{"seam_visibility", "0"}, {"seam_angle_cost", "0"},
                              {"seam_travel_cost", "0"}},
                         size_t layer_index = 0)
    {
        prepare_cube_print(prepared, perimeter_config(settings));
        object = &prepared.print.object(0);
        REQUIRE(layer_index < object->layer_count());
        region = &object->layer(layer_index).region(0);
        source.reset(new LayerRegionIsland({region}, 0));
        source_ptr = source.get();
        prepared.print.mutable_printing_plan() = PrintingPlan();
    }

    slic3r_api::LayerRegion region_view() const
    {
        return slic3r_api::LayerRegion(reinterpret_cast<const layer_region_handle *>(region));
    }

    ModelVolume *box(double x, double y, double sx, double sy, ModelVolumeType type,
                     EnforcerBlockerType painting = EnforcerBlockerType::NONE,
                     double z = 0, double height = 30)
    {
        return add_seam_constraint_box(
            *object, x, y, sx, sy, type, painting, z, height);
    }

    slic3r_api::PrintingExtrusion add(const ExtrusionEntity &loop, raw_extrusion_role role,
                                      uint16_t instance = 0,
                                      LayerRegionIsland *island = nullptr,
                                      bool perimeter = true)
    {
        PrintingPlan &native_plan = prepared.print.mutable_printing_plan();
        slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(&native_plan));
        if (plan.group_count() == 0)
            plan.append_group().append_layer_group(region_view().layer().print_z()).append_tool_group(0);
        const uint32_t instance_id = plan.find_or_add_object_instance(
            region_view().layer().object(), instance);
        LayerRegionIsland *selected = island == nullptr ? source_ptr : island;
        slic3r_api::PrintingExtrusion extrusion =
            plan.group(0).layer_group(0).tool_group(0).append_extrusion_clone(
                plan,
                slic3r_api::LayerRegionIsland(
                    reinterpret_cast<const layer_region_island_handle *>(selected)),
                RAW_EXTRUSION_ROLE_PERIMETER,
                slic3r_api::ExtrusionEntity(
                    reinterpret_cast<const extrusion_entity_handle *>(&loop)),
                instance_id);
        slic3r_api::MutableExtrusionEntity root = extrusion.mutable_root();
        c_extrusion_property_attributes *attributes =
            static_cast<c_extrusion_property_attributes *>(root.get_or_add_property_data_mutable(
                reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
                EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
        REQUIRE(attributes != nullptr);
        attributes->role = role;
        attributes->width = 0.4f;
        if (perimeter) {
            c_extrusion_property_perimeter *property =
                static_cast<c_extrusion_property_perimeter *>(root.get_or_add_property_data_mutable(
                    reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
                    EXTRUSION_PROPERTY_TYPE_PERIMETER));
            REQUIRE(property != nullptr);
            property->perimeter_flags = C_EXTRUSION_PERIMETER_FLAG_LOOP;
        }
        const c_point shift = region_view().layer().object().instance_shift(instance);
        std::function<void(slic3r_api::MutableExtrusionEntity)> translate;
        translate = [&](slic3r_api::MutableExtrusionEntity entity) {
            if (entity.point_count() != 0) REQUIRE(entity.translate(shift));
            for (uint32_t index = 0; index < entity.child_count(); ++index)
                translate(entity.child_mutable(index));
        };
        translate(root);
        return extrusion;
    }

    c_point local(c_point point, uint16_t instance = 0) const
    {
        const c_point shift = region_view().layer().object().instance_shift(instance);
        return {point.x - shift.x, point.y - shift.y};
    }

    slic3r_api::PrintingExtrusion extrusion(uint32_t index)
    {
        slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(
            &prepared.print.mutable_printing_plan()));
        return plan.group(0).layer_group(0).tool_group(0).extrusion(index);
    }

    PreparedPerimeterPrint prepared;
    PrintObject *object = nullptr;
    LayerRegion *region = nullptr;
    std::unique_ptr<LayerRegionIsland> source;
    LayerRegionIsland *source_ptr = nullptr;
};

// Build the native LayerIsland relationship used by boundary and embedding
// calculations; the compact perimeter fixture intentionally leaves it empty.
LayerRegionIsland *attach_native_source(CostFixture &fixture)
{
    Layer &layer = *fixture.region->layer();
    if (layer.islands().empty()) {
        ExPolygons islands;
        for (const ExPolygon &slice : fixture.region->get_raw_slices())
            islands.push_back(slice);
        ApiInternal::LayerAccess::set_islands(layer, std::move(islands));
    }
    REQUIRE(!layer.islands().empty());
    layer.island(0).mutable_regions_islands().emplace_back(
        new LayerRegionIsland({fixture.region}, 0));
    return &layer.island(0).regions_island(
        layer.island(0).regions_islands().size() - 1);
}

LayerRegion *add_neighbor_region(CostFixture &fixture, const ExPolygon &area)
{
    Layer &layer = *fixture.region->layer();
    PrintRegionConfig config = fixture.region->region().config();
    config.set_deserialize_strict(
        "perimeters", std::to_string(fixture.region->region().config().opt_int("perimeters") + 1));
    const PrintRegion *print_region = &fixture.prepared.print.register_print_region(config);
    PrintObjectRegions &associations = *const_cast<PrintObjectRegions *>(
        fixture.object->shared_regions());
    associations.all_regions.push_back(print_region);
    ApiInternal::LayerAccess::add_region(layer, *print_region);
    LayerRegion &region = layer.region(layer.region_count() - 1);
    ExPolygons &slices = ApiInternal::LayerRegionAccess::slices_mutable(region);
    slices = {area};
    ApiInternal::LayerRegionAccess::surfaces_mutable(region).set(
        slices, stPosInternal | stDensSolid);
    layer.island(0).mutable_regions_islands().emplace_back(
        new LayerRegionIsland({&region}, 0));
    return &region;
}

/* Return the real graph-owned region island for one generated cube layer. */
LayerRegionIsland *native_region_island(Layer &layer)
{
    REQUIRE(!layer.islands().empty());
    LayerSliceIsland &island = layer.island(0);
    if (island.regions_islands().empty())
        island.fill_regions(layer);
    if (island.regions_islands().empty())
        island.mutable_regions_islands().emplace_back(
            new LayerRegionIsland({&layer.region(0)}, 0));
    REQUIRE(!island.regions_islands().empty());
    for (LayerRegionIslandUPtr &region_island : island.mutable_regions_islands())
        if (region_island->regions().count(&layer.region(0)) != 0)
            return region_island.get();
    FAIL("Generated layer has no region-island association");
    return nullptr;
}

LayerRegionIsland *native_region_island_at(Layer &layer, size_t island_index)
{
    REQUIRE(island_index < layer.islands().size());
    LayerSliceIsland &island = layer.island(island_index);
    if (island.regions_islands().empty())
        island.fill_regions(layer);
    if (island.regions_islands().empty())
        island.mutable_regions_islands().emplace_back(
            new LayerRegionIsland({&layer.region(0)}, 0));
    for (LayerRegionIslandUPtr &region_island : island.mutable_regions_islands())
        if (region_island->regions().count(&layer.region(0)) != 0)
            return region_island.get();
    FAIL("Generated layer island has no region-island association");
    return nullptr;
}

void replace_test_layer_islands(Layer &layer, const ExPolygons &areas)
{
    ApiInternal::LayerAccess::set_islands(layer, ExPolygons(areas));
    ApiInternal::LayerRegionAccess::slices_mutable(layer.region(0)) = areas;
    ApiInternal::LayerRegionAccess::surfaces_mutable(layer.region(0)).set(
        areas, stPosInternal | stDensSolid);
    for (LayerSliceIsland &island : layer.islands())
        island.fill_regions(layer);
}

void rebuild_test_island_graph(PrintObject &object)
{
    for (Layer &layer : object.layers())
        for (LayerSliceIsland &island : layer.islands()) {
            island.overlaps_above.clear();
            island.overlaps_below.clear();
        }
    for (size_t layer_index = 1; layer_index < object.layer_count(); ++layer_index)
        Layer::build_up_down_graph(object.layer(layer_index - 1), object.layer(layer_index));
}

c_point aligned_seam(const slic3r_api::SeamPlacer &placer,
                     const slic3r_api::PrintingExtrusion &extrusion)
{
    const slic3r_api::LayerRegion region = extrusion.region_island().region(0);
    return placer.place_seam(extrusion.root(), region, {});
}

/* Resolve one Contiguous target without adding an approach-position bias. */
c_point contiguous_seam(const slic3r_api::SeamPlacer &placer,
                        const slic3r_api::PrintingExtrusion &extrusion)
{
    const slic3r_api::LayerRegion region = extrusion.region_island().region(0);
    return placer.place_seam(extrusion.root(), region, {});
}

/* Append one translated external loop to a chosen layer and object instance. */
slic3r_api::PrintingExtrusion append_aligned_loop(
    Print &print, Layer &layer, LayerRegionIsland &source,
    const ExtrusionEntity &loop, uint16_t instance)
{
    PrintingPlan &native_plan = print.mutable_printing_plan();
    slic3r_api::PrintingPlan plan(
        reinterpret_cast<printing_plan_handle *>(&native_plan));
    slic3r_api::PrintingGroup group = plan.group_count() == 0 ?
        plan.append_group() : plan.group(0);
    slic3r_api::PrintingLayerGroup layer_group =
        group.append_layer_group(slic3r_api::LayerRegion(
            reinterpret_cast<const layer_region_handle *>(&layer.region(0))).layer().print_z());
    slic3r_api::PrintingToolGroup tool = layer_group.append_tool_group(0);
    const uint32_t instance_id = plan.find_or_add_object_instance(
        slic3r_api::Object(reinterpret_cast<const object_handle *>(layer.object())),
        instance);
    slic3r_api::PrintingExtrusion extrusion = tool.append_extrusion_clone(
        plan,
        slic3r_api::LayerRegionIsland(
            reinterpret_cast<const layer_region_island_handle *>(&source)),
        RAW_EXTRUSION_ROLE_PERIMETER,
        slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(&loop)),
        instance_id);
    slic3r_api::MutableExtrusionEntity root = extrusion.mutable_root();
    c_extrusion_property_attributes *attributes =
        static_cast<c_extrusion_property_attributes *>(
            root.get_or_add_property_data_mutable(
                reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
                EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
    REQUIRE(attributes != nullptr);
    attributes->role = RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER;
    attributes->width = 0.4f;
    c_extrusion_property_perimeter *perimeter =
        static_cast<c_extrusion_property_perimeter *>(
            root.get_or_add_property_data_mutable(
                reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
                EXTRUSION_PROPERTY_TYPE_PERIMETER));
    REQUIRE(perimeter != nullptr);
    perimeter->perimeter_flags = C_EXTRUSION_PERIMETER_FLAG_LOOP;
    const Point shift = layer.object()->instances()[instance].shift;
    std::function<void(slic3r_api::MutableExtrusionEntity)> translate;
    translate = [&](slic3r_api::MutableExtrusionEntity entity) {
        if (entity.point_count() != 0)
            REQUIRE(entity.translate({shift.x(), shift.y()}));
        for (uint32_t index = 0; index < entity.child_count(); ++index)
            translate(entity.child_mutable(index));
    };
    translate(root);
    return extrusion;
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

} // namespace

/*
Resolve seam and perimeter providers by enum ID in separate Catch2 sections. Scoped guards restore
provider activation, each DynamicPrintConfig owns its options, and selected Plugin pointers are
borrowed; empty provider results or unexpected IDs fail assertions without changing registration.
*/
TEST_CASE("Exclusive selectors resolve active providers by identifier",
          "[plugins][config][seam-placer]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    SECTION("seam provider identifiers")
    {
        std::vector<std::unique_ptr<ScopedPluginActivation>> seam_activation;
        for (Plugin *plugin : orchestrator.registered_plugins()) {
            if (plugin == nullptr || plugin->get_step() != SEAM_PLACER ||
                plugin->get_exclusive_group() != "seam_placer_plugin")
                continue;
            const std::string &id = plugin->get_id();
            const bool should_be_active = id == k_contiguous_plugin ||
                id == k_fixed_plugin || id == k_random_plugin;
            seam_activation.emplace_back(std::make_unique<ScopedPluginActivation>(
                orchestrator, id.c_str(), should_be_active));
        }

        std::vector<Plugin *> active_seam_providers;
        for (Plugin *plugin : orchestrator.get_active_plugins_for_step(SEAM_PLACER))
            if (plugin->get_exclusive_group() == "seam_placer_plugin")
                active_seam_providers.push_back(plugin);
        REQUIRE(active_seam_providers.size() == 3);
        CHECK(active_seam_providers[0]->get_id() == k_contiguous_plugin);
        CHECK(active_seam_providers[0]->get_priority() == 800);
        CHECK(active_seam_providers[1]->get_id() == k_fixed_plugin);
        CHECK(active_seam_providers[1]->get_priority() == 900);
        CHECK(active_seam_providers[2]->get_id() == k_random_plugin);
        CHECK(active_seam_providers[2]->get_priority() == 1000);
        Plugin *regional_test_provider = orchestrator.get_plugin(k_regional_seam_plugin);
        REQUIRE(regional_test_provider != nullptr);
        CHECK(regional_test_provider->get_step() == SEAM_PLACER);
        CHECK(regional_test_provider->get_exclusive_group() == "seam_placer_plugin");
        CHECK_FALSE(orchestrator.is_plugin_active(regional_test_provider));

        const std::string seam_selector = Steps::plugin_exclusive_group_option_key(
            orchestrator, SEAM_PLACER, "seam_placer_plugin");
        DynamicPrintConfig seam_config;
        seam_config.set_deserialize_strict(seam_selector, k_contiguous_plugin);
        Plugin *selected_seam = Steps::selected_active_plugin_from_group(
            orchestrator, SEAM_PLACER, "seam_placer_plugin", &seam_config);
        REQUIRE(selected_seam != nullptr);
        CHECK(selected_seam->get_id() == k_contiguous_plugin);

        seam_config.set_deserialize_strict(seam_selector, k_random_plugin);
        selected_seam = Steps::selected_active_plugin_from_group(
            orchestrator, SEAM_PLACER, "seam_placer_plugin", &seam_config);
        REQUIRE(selected_seam != nullptr);
        CHECK(selected_seam->get_id() == k_random_plugin);

        seam_config.set_deserialize_strict(seam_selector, k_cost_plugin);
        CHECK(Steps::selected_active_plugin_from_group(
                  orchestrator, SEAM_PLACER, "seam_placer_plugin", &seam_config) == nullptr);
    }

    SECTION("perimeter provider identifiers")
    {
        Plugin *simple = orchestrator.get_plugin("perimeter.generator.simple");
        Plugin *classic = orchestrator.get_plugin("perimeter.generator.classic");
        Plugin *arachne = orchestrator.get_plugin("perimeter.generator.arachne");
        REQUIRE(simple != nullptr);
        REQUIRE(classic != nullptr);
        REQUIRE(arachne != nullptr);
        CHECK(simple->get_step() == STEP_PERIMETER);
        CHECK(simple->get_priority() == 0);
        CHECK(classic->get_step() == STEP_PERIMETER);
        CHECK(classic->get_priority() == 5);
        CHECK(arachne->get_step() == STEP_PERIMETER);
        CHECK(arachne->get_priority() == 10);

        std::vector<std::unique_ptr<ScopedPluginActivation>> perimeter_activation;
        for (Plugin *plugin : orchestrator.registered_plugins()) {
            if (plugin == nullptr || plugin->get_step() != STEP_PERIMETER)
                continue;
            const std::string &id = plugin->get_id();
            const bool should_be_active = id == classic->get_id() || id == arachne->get_id();
            perimeter_activation.emplace_back(std::make_unique<ScopedPluginActivation>(
                orchestrator, id.c_str(), should_be_active));
        }

        const std::map<slicing_step_t, Steps::StepExclusiveGroup> &exclusive_steps =
            Steps::get_exclusive_steps();
        const std::map<slicing_step_t, Steps::StepExclusiveGroup>::const_iterator perimeter_group =
            exclusive_steps.find(STEP_PERIMETER);
        REQUIRE(perimeter_group != exclusive_steps.end());
        CHECK(perimeter_group->second.group_id == "step_perimeter_plugin");
        CHECK(perimeter_group->second.option_key_storage == "step_perimeter_plugin");

        const std::vector<Plugin *> active_generators =
            orchestrator.get_active_plugins_for_step(STEP_PERIMETER);
        REQUIRE(active_generators.size() == 2);
        CHECK(active_generators[0]->get_id() == classic->get_id());
        CHECK(active_generators[1]->get_id() == arachne->get_id());

        DynamicPrintConfig perimeter_config;
        perimeter_config.set_deserialize_strict(
            perimeter_group->second.option_key_storage, arachne->get_id());
        std::vector<Plugin *> selected_generators = Steps::selected_or_active_plugins_for_step(
            orchestrator, STEP_PERIMETER, &perimeter_config);
        REQUIRE(selected_generators.size() == 1);
        CHECK(selected_generators.front()->get_id() == arachne->get_id());

        perimeter_config.set_deserialize_strict(
            perimeter_group->second.option_key_storage, simple->get_id());
        selected_generators = Steps::selected_or_active_plugins_for_step(
            orchestrator, STEP_PERIMETER, &perimeter_config);
        REQUIRE(selected_generators.size() == 1);
        CHECK(selected_generators.front()->get_id() == classic->get_id());
    }
}

TEST_CASE("Cost seam metadata is the default regional provider",
          "[plugins][seam-placer][cost]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    const Plugin *aligned = orchestrator.get_plugin(k_aligned_plugin);
    const Plugin *contiguous = orchestrator.get_plugin(k_contiguous_plugin);
    const Plugin *cost = orchestrator.get_plugin(k_cost_plugin);
    const Plugin *fixed = orchestrator.get_plugin("seam_placer.fixed");
    const Plugin *random = orchestrator.get_plugin("seam_placer.random");
    REQUIRE(aligned != nullptr);
    REQUIRE(contiguous != nullptr);
    REQUIRE(cost != nullptr);
    REQUIRE(fixed != nullptr);
    REQUIRE(random != nullptr);
    CHECK(aligned->get_name() == "Aligned");
    CHECK(aligned->get_priority() == 700);
    CHECK(aligned->get_dependencies().empty());
    CHECK(contiguous->get_name() == "Contiguous");
    CHECK(contiguous->get_priority() == 800);
    CHECK(contiguous->get_dependencies().empty());
    CHECK(cost->get_name() == "Cost-based");
    CHECK(cost->get_priority() == 600);
    CHECK(fixed->get_priority() == 900);
    CHECK(random->get_priority() == 1000);
    CHECK(cost->get_dependencies().empty());
    const std::vector<Plugin::UsedConfigKey> used = cost->get_used_config_keys();
    REQUIRE(used.size() == 3);
    CHECK(used[0].key == "seam_angle_cost");
    CHECK(used[0].type == RAW_CO_PERCENT);
    CHECK(used[1].key == "seam_travel_cost");
    CHECK(used[1].type == RAW_CO_PERCENT);
    CHECK(used[2].key == "seam_visibility");
    CHECK(used[2].type == RAW_CO_BOOL);
    CHECK(used[2].container_type == RAW_CONTAINER_TYPE_REGION);
    CHECK(used[2].option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    const std::vector<Plugin::DefinedConfigKey> defined = cost->get_defined_config_keys();
    REQUIRE(defined.size() == 3);
    CHECK(defined[0].key == "seam_angle_cost");
    CHECK(defined[0].type == RAW_CO_PERCENT);
    CHECK(defined[1].key == "seam_travel_cost");
    CHECK(defined[1].type == RAW_CO_PERCENT);
    CHECK(defined[2].key == "seam_visibility");
    CHECK(defined[2].type == RAW_CO_BOOL);
    for (size_t index = 0; index < 3; ++index) {
        const Plugin::UsedConfigKey &key = used[index];
        CHECK(key.container_type == RAW_CONTAINER_TYPE_REGION);
        CHECK(key.option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    }
    // The shared runtime disables non-Fixed providers after registration. A
    // default-enabled Cost provider must still activate without dependency
    // diagnostics, then return to the shared inactive test state.
    CHECK_FALSE(orchestrator.is_plugin_active(k_cost_plugin));
    CHECK(orchestrator.set_plugin_active(k_cost_plugin, true));
    CHECK(orchestrator.is_plugin_active(k_cost_plugin));
    const std::vector<std::string> dependency_errors =
        orchestrator.active_plugin_dependency_errors();
    CHECK(std::none_of(dependency_errors.begin(), dependency_errors.end(),
                       [](const std::string &error) {
                           return error.find(k_cost_plugin) != std::string::npos;
                       }));
    CHECK(orchestrator.set_plugin_active(k_cost_plugin, false));
    CHECK_FALSE(orchestrator.is_plugin_active(k_cost_plugin));

    PrintRegionConfig region;
    const std::string selector = Steps::plugin_exclusive_group_option_key(
        orchestrator, SEAM_PLACER, "seam_placer_plugin");
    REQUIRE(region.option(selector) != nullptr);
    CHECK(region.option(selector)->serialize() == k_cost_plugin);

    std::ifstream stream(
        std::string(TEST_DATA_DIR) + "/../../src/plugins/default_activated.ini.in");
    REQUIRE(stream.good());
    std::ostringstream contents;
    contents << stream.rdbuf();
    CHECK(contents.str().find("seam_placer.cost = 1") != std::string::npos);
    CHECK(contents.str().find("seam_placer.aligned = 1") != std::string::npos);
    CHECK(contents.str().find("seam_placer.contiguous = 1") != std::string::npos);
}

// Builds defaults and an owned prepared print without a visibility override, checking cached and applied regional
// values before any seam session; missing/wrong-type options fail Catch and preparation/allocation failures propagate.
TEST_CASE("Plugin test defaults disable seam visibility in full and applied configs",
          "[plugins][seam-placer][cost][visibility-default]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    // Reads a borrowed config's bool option without mutation; type/presence/value mismatches fail the test.
    const auto check_disabled = [](const auto &config) {
        const ConfigOptionBool *option = dynamic_cast<const ConfigOptionBool *>(config.option("seam_visibility"));
        REQUIRE(option != nullptr);
        CHECK_FALSE(option->value);
    };
    const DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
    check_disabled(full);
    const PrintRegionConfig region;
    check_disabled(region);
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, full);
    check_disabled(prepared.print.default_region_config());
    REQUIRE(prepared.print.object(0).layer_count() > 0);
    check_disabled(prepared.print.object(0).layer(0).region(0).region().config());
}

TEST_CASE("Cost seam projects candidates and supports parallel placement",
          "[plugins][seam-placer][cost]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({
        {"seam_visibility", "0"}, {"seam_angle_cost", "0"}, {"seam_travel_cost", "100%"}}));
    PrintObject &object = prepared.print.object(0);
    LayerRegion &native_region = object.layer(0).region(0);
    LayerRegionSetCPtrs regions{&native_region};
    std::unique_ptr<LayerRegionIsland> source(new LayerRegionIsland(regions, 0));
    const Point shift = object.instances().front().shift;
    std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(false, ArcPolyline(Points{
        shift + Point::new_scale(-5, -5), shift + Point::new_scale(5, -5),
        shift + Point::new_scale(5, 5), shift + Point::new_scale(-5, 5),
        shift + Point::new_scale(-5, -5)})));
    slic3r_api::MutableExtrusionEntity mutable_root(
        reinterpret_cast<extrusion_entity_handle *>(root.get()));
    orchestrator_handle *orchestrator_handle_value = reinterpret_cast<orchestrator_handle *>(
        &Orchestrator::instance());
    c_extrusion_property_attributes *attributes = static_cast<c_extrusion_property_attributes *>(
        mutable_root.get_or_add_property_data_mutable(
            orchestrator_handle_value, EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
    REQUIRE(attributes != nullptr);
    attributes->role = RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER;
    attributes->width = 0.4f;
    c_extrusion_property_perimeter *perimeter = static_cast<c_extrusion_property_perimeter *>(
        mutable_root.get_or_add_property_data_mutable(
            orchestrator_handle_value, EXTRUSION_PROPERTY_TYPE_PERIMETER));
    REQUIRE(perimeter != nullptr);
    perimeter->perimeter_flags = C_EXTRUSION_PERIMETER_FLAG_LOOP;

    PrintingPlan &native_plan = prepared.print.mutable_printing_plan();
    native_plan = PrintingPlan();
    const PrintingObjectInstanceId catalog_id = native_plan.find_or_add_object_instance(object, 0);
    native_plan.groups.emplace_back();
    native_plan.groups.back().layers.emplace_back();
    native_plan.groups.back().layers.back().layers.push_back(&object.layer(0));
    native_plan.groups.back().layers.back().tool_groups.emplace_back();
    PrintingToolGroup &tool = native_plan.groups.back().layers.back().tool_groups.back();
    tool.region_islands.push_back(source.get());
    tool.extrusions.emplace_back();
    tool.extrusions.back().region_island = source.get();
    tool.extrusions.back().sregion_island_role = LayerRegionIsland::PERIMETERS;
    tool.extrusions.back().printing_catalog_id = catalog_id;
    tool.extrusions.back().root = std::move(root);

    slic3r_api::SeamPlacer placer = create_cost_session(prepared.print);
    REQUIRE(placer.valid());

    slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(
        &prepared.print.mutable_printing_plan()));
    const slic3r_api::PrintingExtrusion extrusion = plan.group(0).layer_group(0).tool_group(0).extrusion(0);
    slic3r_api::ExtrusionEntity loop;
    REQUIRE(find_loop(extrusion.root(), loop));
    const slic3r_api::LayerRegion region = extrusion.region_island().region(0);
    CHECK(placer.get_priority(loop, region) == 50.f);

    const c_point first = placer.place_seam(loop, region, loop.front());
    std::vector<std::future<c_point>> calls;
    for (size_t index = 0; index < 8; ++index)
        calls.push_back(std::async(std::launch::async, [&placer, loop, region, index]() {
            return placer.place_seam(loop, region, index % 2 == 0 ? loop.front() : loop.back());
        }));
    for (std::future<c_point> &call : calls) CHECK_NOTHROW(call.get());
    CHECK_NOTHROW(placer.place_seam(loop, region, first));

    const ExtrusionEntity foreign(false, ArcPolyline(Points{
        Point::new_scale(0, 0), Point::new_scale(1, 0),
        Point::new_scale(1, 1), Point::new_scale(0, 0)}));
    const slic3r_api::ExtrusionEntity foreign_view(
        reinterpret_cast<const extrusion_entity_handle *>(&foreign));
    CHECK_THROWS_WITH(placer.place_seam(foreign_view, region, {0, 0}),
                      "Cost seam loop was not present in the initialized plan.");
}

TEST_CASE("Cost seam takes candidates only from effective external perimeter loops",
          "[plugins][seam-placer][cost][geometry]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    const ExtrusionEntity internal(false, ArcPolyline(Points{
        Point::new_scale(2, 2), Point::new_scale(-2, 2), Point::new_scale(-2, -2),
        Point::new_scale(2, -2), Point::new_scale(2, 2)}));
    const ExtrusionEntity external = rectangle_loop(-5, -5, 5, 5);
    fixture.add(internal, RAW_EXTRUSION_ROLE_PERIMETER);
    fixture.add(external, RAW_EXTRUSION_ROLE_OVERHANG_EXTERNAL_PERIMETER);

    const slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);
    const slic3r_api::PrintingExtrusion internal_plan = fixture.extrusion(0);
    const c_point projected = fixture.local(placer.place_seam(
        internal_plan.root(), fixture.region_view(), internal_plan.root().front()));
    CHECK(projected.x == scale_i(-2));
    CHECK(projected.y == scale_i(-2));
}

TEST_CASE("Cost seam keeps prepared candidates owned by each external loop",
          "[plugins][seam-placer][cost][geometry][regression]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.add(rectangle_loop(5, 9, 6, 10),
                RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    fixture.add(rectangle_loop(0, 0, 10, 8),
                RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::PrintingExtrusion first = fixture.extrusion(0);
    const slic3r_api::PrintingExtrusion second = fixture.extrusion(1);
    const slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);

    const c_point first_seam = placer.place_seam(
        first.root(), fixture.region_view(), {});
    const c_point first_local = fixture.local(first_seam);
    const c_point second_seam = placer.place_seam(
        second.root(), fixture.region_view(), first_seam);
    const c_point second_local = fixture.local(second_seam);

    CHECK(first_local.x == scale_i(5));
    CHECK(first_local.y == scale_i(9));
    CHECK(second_local.x == scale_i(0));
    CHECK(second_local.y == scale_i(0));
    CHECK(slic3r_api::SeamPlacement::CandidatePreparation::project_onto_loop(
        second.root(), second_seam).second == 0.0);
}

TEST_CASE("Cost seam places distance-only loops by direct distance projection",
          "[plugins][seam-placer][cost][geometry]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("distance-only island")
    {
        CostFixture fixture;
        const slic3r_api::PrintingExtrusion skirt = fixture.add(
            rectangle_loop(0, 0, 10, 10), RAW_EXTRUSION_ROLE_SKIRT);

        const slic3r_api::SeamPlacer placer =
            create_cost_session(fixture.prepared.print);
        const c_point start = {skirt.root().front().x + scale_i(4),
                               skirt.root().front().y - scale_i(3)};
        const c_point projected = fixture.local(placer.place_seam(
            skirt.root(), fixture.region_view(), start));

        CHECK(projected.x == scale_i(4));
        CHECK(projected.y == 0);
    }

    SECTION("mixed island")
    {
        CostFixture fixture;
        fixture.add(rectangle_loop(-20, -20, 20, 20),
                    RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
        const slic3r_api::PrintingExtrusion skirt = fixture.add(
            rectangle_loop(0, 0, 10, 10), RAW_EXTRUSION_ROLE_SKIRT);

        const slic3r_api::SeamPlacer placer =
            create_cost_session(fixture.prepared.print);
        const c_point start = {skirt.root().front().x + scale_i(4),
                               skirt.root().front().y - scale_i(3)};
        const c_point projected = fixture.local(placer.place_seam(
            skirt.root(), fixture.region_view(), start));

        CHECK(projected.x == scale_i(4));
        CHECK(projected.y == 0);
    }
}

TEST_CASE("Cost seam binds composed loops and projects analytically onto arcs",
          "[plugins][seam-placer][cost][geometry]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    SECTION("composed line loop")
    {
        CostFixture fixture;
        ExtrusionEntity::Children children;
        children.emplace_back(new ExtrusionEntity(false, ArcPolyline(Points{
            Point::new_scale(0, 0), Point::new_scale(10, 0)})));
        children.emplace_back(new ExtrusionEntity(false, ArcPolyline(Points{
            Point::new_scale(10, 0), Point::new_scale(10, 10)})));
        children.emplace_back(new ExtrusionEntity(false, ArcPolyline(Points{
            Point::new_scale(10, 10), Point::new_scale(0, 10), Point::new_scale(0, 0)})));
        const ExtrusionEntity composed(std::move(children), false, false, false);
        const slic3r_api::PrintingExtrusion extrusion = fixture.add(
            composed, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
        const slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);
        CHECK_NOTHROW(placer.place_seam(extrusion.root(), fixture.region_view(), {}));
    }

    SECTION("arc loop")
    {
        CostFixture fixture({{"seam_visibility", "0"}, {"seam_angle_cost", "0"},
                             {"seam_travel_cost", "100%"}});
        fixture.box(5.9, 7.8, 0.2, 0.4, ModelVolumeType::SEAM_POSITION_INSIDE);
        const slic3r_api::PrintingExtrusion extrusion = fixture.add(
            arc_loop(), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
        const slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);
        const c_point point = fixture.local(placer.place_seam(
            extrusion.root(), fixture.region_view(), {scale_i(6), scale_i(8)}));
        CHECK(std::hypot(unscaled(point.x), unscaled(point.y)) == Approx(10).margin(0.0001));
        CHECK((point.x != extrusion.root().front().x || point.y != extrusion.root().front().y));
    }
}

TEST_CASE("Cost seam binds the atomic parent of Arachne-like perimeter paths",
          "[plugins][seam-placer][cost][geometry]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    ExtrusionEntity::Children children;
    children.emplace_back(new ExtrusionEntity(false, ArcPolyline(Points{
        Point::new_scale(0, 0), Point::new_scale(10, 0)})));
    children.emplace_back(new ExtrusionEntity(false, ArcPolyline(Points{
        Point::new_scale(10, 0), Point::new_scale(10, 10)})));
    children.emplace_back(new ExtrusionEntity(false, ArcPolyline(Points{
        Point::new_scale(10, 10), Point::new_scale(0, 10),
        Point::new_scale(0, 0)})));
    const ExtrusionEntity composed(std::move(children), false, false, false);
    slic3r_api::PrintingExtrusion extrusion = fixture.add(
        composed, RAW_EXTRUSION_ROLE_NONE, 0, nullptr, false);
    slic3r_api::MutableExtrusionEntity root = extrusion.mutable_root();
    for (uint32_t index = 0; index < root.child_count(); ++index)
        mark_perimeter_path(root.child_mutable(index),
                            RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 0.61f);

    const std::vector<slic3r_api::SeamPlacement::CandidatePreparation::CollectedLoop>
        collected = slic3r_api::SeamPlacement::CandidatePreparation::collect_perimeter_loops(
            extrusion.root(), RAW_EXTRUSION_ROLE_NONE);
    REQUIRE(collected.size() == 1);
    CHECK(collected.front().loop == extrusion.root().handle());
    CHECK_FALSE(collected.front().distance_only);
    CHECK(collected.front().has_external_geometry);
    CHECK(collected.front().external_width == Approx(0.61));

    const slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);
    CHECK_NOTHROW(placer.place_seam(
        extrusion.root(), fixture.region_view(), extrusion.root().front()));
}

TEST_CASE("Cost seam applies modifier precedence and all-blocked fallback",
          "[plugins][seam-placer][cost][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool arc = GENERATE(false, true);
    CostFixture fixture;
    fixture.box(arc ? 5.9 : 4.0, arc ? 7.8 : -1.0, arc ? 0.2 : 2.0,
                arc ? 0.4 : 2.0, ModelVolumeType::SEAM_POSITION_INSIDE);
    const slic3r_api::PrintingExtrusion extrusion = fixture.add(
        arc ? arc_loop() : rectangle_loop(0, 0, 10, 10),
        RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);
    c_point point = fixture.local(placer.place_seam(extrusion.root(), fixture.region_view(), {}));
    if (arc) {
        CHECK(std::hypot(unscaled(point.x), unscaled(point.y)) == Approx(10).margin(0.0001));
        CHECK(unscaled(point.x) >= 5.89);
    } else {
        CHECK(point.y == 0);
        CHECK(unscaled(point.x) >= 3.99);
        CHECK(unscaled(point.x) <= 6.01);
    }

    CostFixture blocked;
    blocked.box(-20, -20, 40, 40, ModelVolumeType::MODEL_PART,
                EnforcerBlockerType::BLOCKER);
    const slic3r_api::PrintingExtrusion blocked_loop = blocked.add(
        rectangle_loop(0, 0, 10, 10), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    placer = create_cost_session(blocked.prepared.print);
    CHECK_NOTHROW(placer.place_seam(blocked_loop.root(), blocked.region_view(), {}));

    CostFixture overlap;
    overlap.box(4, -1, 2, 2, ModelVolumeType::SEAM_POSITION_INSIDE);
    overlap.box(3, -2, 4, 4, ModelVolumeType::MODEL_PART,
                EnforcerBlockerType::BLOCKER);
    const slic3r_api::PrintingExtrusion overlap_loop = overlap.add(
        rectangle_loop(0, 0, 10, 10), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    placer = create_cost_session(overlap.prepared.print);
    point = overlap.local(placer.place_seam(overlap_loop.root(), overlap.region_view(), {}));
    CHECK_FALSE((point.y == 0 && unscaled(point.x) > 3 && unscaled(point.x) < 7));
}

TEST_CASE("Cost seam applies legacy angle and dynamic travel weights",
          "[plugins][seam-placer][cost][weights]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExtrusionEntity concave(false, ArcPolyline(Points{
        Point::new_scale(0, 0), Point::new_scale(10, 0), Point::new_scale(10, 10),
        Point::new_scale(5, 5), Point::new_scale(0, 10), Point::new_scale(0, 0)}));
    CostFixture angle({{"seam_visibility", "0"}, {"seam_angle_cost", "1000%"},
                       {"seam_travel_cost", "0"}});
    const slic3r_api::PrintingExtrusion angle_loop = angle.add(
        concave, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer angle_placer = create_cost_session(angle.prepared.print);
    const c_point hidden_corner = angle.local(angle_placer.place_seam(
        angle_loop.root(), angle.region_view(), {}));
    CHECK(hidden_corner.x == scale_i(5));
    CHECK(hidden_corner.y == scale_i(5));

    CostFixture travel({{"seam_visibility", "0"}, {"seam_angle_cost", "0"},
                        {"seam_travel_cost", "1000%"}});
    const slic3r_api::PrintingExtrusion travel_loop = travel.add(
        rectangle_loop(0, 0, 10, 10), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer travel_placer = create_cost_session(travel.prepared.print);
    const c_point requested = {scale_i(10) + travel_loop.root().front().x,
                               scale_i(10) + travel_loop.root().front().y};
    const c_point nearest = travel.local(travel_placer.place_seam(
        travel_loop.root(), travel.region_view(), requested));
    CHECK(nearest.x == scale_i(10));
    CHECK(nearest.y == scale_i(10));
}

TEST_CASE("Cost seam boundary uses half the effective width and follows filter precedence",
          "[plugins][seam-placer][cost][boundary]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const double gap = GENERATE(0.19, 0.21);
    CostFixture fixture;
    fixture.source_ptr = attach_native_source(fixture);
    LayerRegion *neighbor = add_neighbor_region(
        fixture, rectangle_expolygon(5 + gap, -1, 7, 1));
    REQUIRE(fixture.object->layer(0).island(0).regions_islands().size() == 2);
    REQUIRE(neighbor->slices().surfaces.size() == 1);
    const slic3r_api::PrintingExtrusion extrusion = fixture.add(
        rectangle_loop(-5, -5, 5, 5), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);
    const c_point point = fixture.local(placer.place_seam(
        extrusion.root(), fixture.region_view(), extrusion.root().front()));
    if (gap < 0.2) {
        CHECK(point.x == scale_i(5));
        CHECK(std::abs(unscaled(point.y)) == Approx(1.2).margin(0.002));
    } else {
        CHECK(point.x == scale_i(-5));
        CHECK(point.y == scale_i(-5));
    }

    CostFixture precedence;
    precedence.source_ptr = attach_native_source(precedence);
    add_neighbor_region(precedence, rectangle_expolygon(5.19, -1, 7, 1));
    precedence.box(-3, -6, 2, 2, ModelVolumeType::SEAM_POSITION_INSIDE);
    const slic3r_api::PrintingExtrusion precedence_loop = precedence.add(
        rectangle_loop(-5, -5, 5, 5), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer precedence_placer = create_cost_session(
        precedence.prepared.print);
    const c_point enforced = precedence.local(precedence_placer.place_seam(
        precedence_loop.root(), precedence.region_view(), {}));
    CHECK(enforced.y == scale_i(-5));
    CHECK(unscaled(enforced.x) >= -3.01);
    CHECK(unscaled(enforced.x) <= -0.99);
}

TEST_CASE("Cost seam finds boundary transitions in nested line and arc segments",
          "[plugins][seam-placer][cost][boundary]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.source_ptr = attach_native_source(fixture);
    add_neighbor_region(fixture, rectangle_expolygon(5.8, 7.8, 6.2, 8.2));
    ExtrusionEntity::Children inner;
    inner.emplace_back(new ExtrusionEntity(arc_loop()));
    ExtrusionEntity::Children outer;
    outer.emplace_back(new ExtrusionEntity(std::move(inner), false, false, false));
    const ExtrusionEntity nested(std::move(outer), false, false, false);
    const slic3r_api::PrintingExtrusion extrusion = fixture.add(
        nested, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);
    const c_point point = fixture.local(placer.place_seam(
        extrusion.root(), fixture.region_view(), {}));
    CHECK(std::hypot(unscaled(point.x), unscaled(point.y)) == Approx(10).margin(0.0001));
    CHECK(unscaled(point.x) > 5.5);
    CHECK(unscaled(point.x) < 6.5);
    CHECK(unscaled(point.y) > 7.5);
}

TEST_CASE("Cost seam keeps supported candidates ahead of significant overhangs",
          "[plugins][seam-placer][cost][overhang]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExtrusionEntity loop(false, ArcPolyline(Points{
        Point::new_scale(5, -5), Point::new_scale(5, 5), Point::new_scale(-5, 5),
        Point::new_scale(-5, -5), Point::new_scale(5, -5)}));

    CostFixture supported({{"seam_visibility", "0"}, {"seam_angle_cost", "0"},
                           {"seam_travel_cost", "0"}}, 1);
    const slic3r_api::PrintingExtrusion all_supported = supported.add(
        loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer supported_placer = create_cost_session(
        supported.prepared.print);
    const c_point baseline = supported.local(supported_placer.place_seam(
        all_supported.root(), supported.region_view(), {}));
    CHECK(baseline.x == scale_i(5));
    CHECK(baseline.y == scale_i(-5));

    CostFixture partial({{"seam_visibility", "0"}, {"seam_angle_cost", "0"},
                         {"seam_travel_cost", "0"}}, 1);
    ApiInternal::LayerAccess::slices_mutable(partial.object->layer(0)) = {
        rectangle_expolygon(-10, -10, 0, 10)};
    const slic3r_api::PrintingExtrusion partly_overhanging = partial.add(
        loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer partial_placer = create_cost_session(
        partial.prepared.print);
    const c_point preferred = partial.local(partial_placer.place_seam(
        partly_overhanging.root(), partial.region_view(), {}));
    CHECK(preferred.x == scale_i(-5));
    CHECK(preferred.y == scale_i(5));
}

TEST_CASE("Cost seam prefers candidates embedded more than half a millimeter",
          "[plugins][seam-placer][cost][embedding]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExtrusionEntity loop(false, ArcPolyline(Points{
        Point::new_scale(-10, 0), Point::new_scale(0, 0), Point::new_scale(0, 1),
        Point::new_scale(-10, 1), Point::new_scale(-10, 0)}));

    CostFixture single;
    single.source_ptr = attach_native_source(single);
    single.source_ptr->mutable_extrusion(LayerRegionIsland::PERIMETERS).append(loop);
    const slic3r_api::PrintingExtrusion single_loop = single.add(
        loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer single_placer = create_cost_session(single.prepared.print);
    const c_point baseline = single.local(single_placer.place_seam(
        single_loop.root(), single.region_view(), {}));
    CHECK(baseline.x == scale_i(-10));
    CHECK(baseline.y == 0);

    CostFixture multiple;
    Layer &layer = multiple.object->layer(0);
    multiple.source_ptr = attach_native_source(multiple);
    multiple.source_ptr->mutable_extrusion(LayerRegionIsland::PERIMETERS).append(loop);
    layer.island(0).mutable_regions_islands().emplace_back(
        new LayerRegionIsland({multiple.region}, 0));
    LayerRegionIsland &second = layer.island(0).regions_island(
        layer.island(0).regions_islands().size() - 1);
    second.mutable_extrusion(LayerRegionIsland::PERIMETERS).append(loop);
    const slic3r_api::PrintingExtrusion embedded_loop = multiple.add(
        loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer multiple_placer = create_cost_session(
        multiple.prepared.print);
    const c_point embedded = multiple.local(multiple_placer.place_seam(
        embedded_loop.root(), multiple.region_view(), {}));
    CHECK(embedded.x == 0);
    CHECK(embedded.y == 0);
}

TEST_CASE("Cost seam visibility switch changes the selected model-facing point",
          "[plugins][seam-placer][cost][visibility]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExtrusionEntity loop(false, ArcPolyline(Points{
        Point::new_scale(10, -10), Point::new_scale(10, 10),
        Point::new_scale(-10, 10), Point::new_scale(-10, -10),
        Point::new_scale(10, -10)}));
    CostFixture disabled({{"seam_visibility", "0"}, {"seam_angle_cost", "0"},
                         {"seam_travel_cost", "0"}});
    disabled.box(9, -11, 2, 2, ModelVolumeType::MODEL_PART);
    const slic3r_api::PrintingExtrusion disabled_loop = disabled.add(
        loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer disabled_placer = create_cost_session(
        disabled.prepared.print);
    const c_point without_visibility = disabled.local(disabled_placer.place_seam(
        disabled_loop.root(), disabled.region_view(), {}));
    CHECK(without_visibility.x == scale_i(10));
    CHECK(without_visibility.y == scale_i(-10));

    CostFixture enabled({{"seam_visibility", "1"}, {"seam_angle_cost", "0"},
                          {"seam_travel_cost", "0"}});
    REQUIRE(enabled.region->region().config().option("seam_visibility") != nullptr);
    CHECK(enabled.region->region().config().option("seam_visibility")->serialize() == "1");
    enabled.box(9, -11, 2, 2, ModelVolumeType::MODEL_PART);
    const slic3r_api::PrintingExtrusion enabled_loop = enabled.add(
        loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer enabled_placer = create_cost_session(
        enabled.prepared.print);
    const c_point with_visibility = enabled.local(enabled_placer.place_seam(
        enabled_loop.root(), enabled.region_view(), {}));
    CHECK(with_visibility.x == scale_i(-10));
    CHECK(with_visibility.y == scale_i(-10));
}

TEST_CASE("Cost seam visibility reports local progress",
          "[plugins][seam-placer][cost][progress]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture({{"seam_visibility", "1"}, {"seam_angle_cost", "0"},
                         {"seam_travel_cost", "0"}});
    ProgressProbe progress;
    plugin_run_context run_context = {};
    run_context.host_context = reinterpret_cast<plugin_host_context *>(&progress);
    run_context.report_progress = &capture_progress;

    const ExtrusionEntity loop = rectangle_loop(-10, -10, 10, 10, true);
    fixture.add(loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer placer = create_cost_session(
        fixture.prepared.print, &run_context);
    REQUIRE(placer.valid());
    REQUIRE_FALSE(progress.percentages.empty());
    CHECK(progress.percentages.front() == 0);
    CHECK(progress.percentages.back() == 100);
    CHECK(std::is_sorted(progress.percentages.begin(), progress.percentages.end()));
}

TEST_CASE("Cost seam visibility samples at the legacy layer slice Z",
          "[plugins][seam-placer][cost][visibility]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExtrusionEntity loop = rectangle_loop(-10, -10, 10, 10, true);
    CostFixture fixture({{"seam_visibility", "1"}, {"seam_angle_cost", "0"},
                         {"seam_travel_cost", "0"}, {"nozzle_diameter", "20"},
                         {"layer_height", "8"}, {"first_layer_height", "8"}});
    const slic3r_api::Layer layer = fixture.region_view().layer();
    REQUIRE(unscaled(layer.slice_z()) == Approx(4.0));
    REQUIRE(unscaled(layer.print_z()) == Approx(8.0));
    fixture.box(9, -11, 2, 2, ModelVolumeType::MODEL_PART,
                EnforcerBlockerType::NONE, 0, 5);
    const slic3r_api::PrintingExtrusion extrusion = fixture.add(
        loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);
    const c_point seam = fixture.local(placer.place_seam(
        extrusion.root(), fixture.region_view(), {}));

    CHECK(seam.x == scale_i(10));
    CHECK(seam.y == scale_i(10));
}

TEST_CASE("Cost seam reuses object candidates for translated instances and stable islands",
          "[plugins][seam-placer][cost][instances]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    PrintInstances &instances = const_cast<PrintInstances &>(fixture.object->instances());
    instances.push_back(instances.front());
    instances.back().shift += Point::new_scale(75, -40);
    const ExtrusionEntity loop = rectangle_loop(-5, -5, 5, 5);
    fixture.add(loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 0);
    fixture.add(loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 1);
    std::unique_ptr<LayerRegionIsland> other(new LayerRegionIsland({fixture.region}, 0));
    fixture.add(loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 0, other.get());
    const slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);

    const slic3r_api::PrintingExtrusion first = fixture.extrusion(0);
    const slic3r_api::PrintingExtrusion second = fixture.extrusion(1);
    const slic3r_api::PrintingExtrusion other_loop = fixture.extrusion(2);

    const c_point first_point = fixture.local(placer.place_seam(
        first.root(), fixture.region_view(), first.root().front()), 0);
    const c_point second_point = fixture.local(placer.place_seam(
        second.root(), fixture.region_view(), second.root().front()), 1);
    const c_point other_point = fixture.local(placer.place_seam(
        other_loop.root(), fixture.region_view(), other_loop.root().front()), 0);
    CHECK(first_point.x == second_point.x);
    CHECK(first_point.y == second_point.y);
    CHECK(first_point.x == other_point.x);
    CHECK(first_point.y == other_point.y);
}

TEST_CASE("Cost seam initializes only the assigned regional providers",
          "[plugins][seam-placer][cost][regions]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.source_ptr = attach_native_source(fixture);
    LayerRegion *other_region = add_neighbor_region(
        fixture, rectangle_expolygon(5.19, -1, 7, 1));
    LayerSliceIsland &layer_island = fixture.object->layer(0).island(0);
    LayerRegionIsland &other_island = layer_island.regions_island(
        layer_island.regions_islands().size() - 1);
    fixture.add(rectangle_loop(-5, -5, 5, 5),
                RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    fixture.add(rectangle_loop(6, -1, 7, 1),
                RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 0, &other_island);

    const std::vector<slic3r_api::PrintRegion> assigned = {
        fixture.region_view().print_region()};
    const slic3r_api::SeamPlacer placer = create_cost_session_for(
        fixture.prepared.print, assigned);
    const slic3r_api::PrintingExtrusion included = fixture.extrusion(0);
    const slic3r_api::PrintingExtrusion excluded = fixture.extrusion(1);
    CHECK_NOTHROW(placer.place_seam(
        included.root(), fixture.region_view(), included.root().front()));
    const slic3r_api::LayerRegion excluded_region(
        reinterpret_cast<const layer_region_handle *>(other_region));
    CHECK_THROWS_WITH(placer.place_seam(
        excluded.root(), excluded_region, excluded.root().front()),
        "Cost seam LayerRegion was not assigned to this provider session.");
}

TEST_CASE("Cost seam constraints ignore hostile unassigned regional extrusions",
          "[plugins][seam-placer][cost][regions][isolation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.source_ptr = attach_native_source(fixture);
    LayerRegion *other_region = add_neighbor_region(
        fixture, rectangle_expolygon(20, 20, 22, 22));
    LayerSliceIsland &layer_island = fixture.object->layer(0).island(0);
    LayerRegionIsland &other_island = layer_island.regions_island(
        layer_island.regions_islands().size() - 1);
    fixture.add(rectangle_loop(-5, -5, 5, 5),
                RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    fixture.add(rectangle_loop(20, 20, 22, 22),
                RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 0, &other_island);
    PrintingExtrusion &hostile = fixture.prepared.print.mutable_printing_plan()
        .groups.front().layers.front().tool_groups.front().extrusions[1];
    // A full-plan initializer reaches this malformed unassigned extrusion and
    // throws while resolving its catalog entry; the routed initializer must skip it.
    hostile.printing_catalog_id = 999999;

    const std::vector<slic3r_api::PrintRegion> assigned = {
        fixture.region_view().print_region()};
    const slic3r_api::SeamPlacer placer = create_cost_session_for(
        fixture.prepared.print, assigned);
    const slic3r_api::PrintingExtrusion included = fixture.extrusion(0);
    const c_point point = fixture.local(placer.place_seam(
        included.root(), fixture.region_view(), included.root().front()));
    CHECK(point.x == scale_i(-5));
    CHECK(point.y == scale_i(-5));
    CHECK(other_region != nullptr);
}

TEST_CASE("Cost seam includes a multi-region island when any source region is assigned",
          "[plugins][seam-placer][cost][regions]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    LayerRegion *other_region = add_neighbor_region(
        fixture, rectangle_expolygon(20, 20, 22, 22));
    LayerRegionSetCPtrs combined_regions{fixture.region, other_region};
    REQUIRE(combined_regions.size() == 2);
    // Choose the second pointer as assigned so a first-region-only filter is
    // caught regardless of allocation order in this synthetic fixture.
    LayerRegion *assigned_region = *combined_regions.begin() == fixture.region ?
        other_region : fixture.region;
    const slic3r_api::LayerRegion assigned_view(
        reinterpret_cast<const layer_region_handle *>(assigned_region));
    std::unique_ptr<LayerRegionIsland> combined(
        new LayerRegionIsland(combined_regions, 0));
    fixture.box(0, -2, 2, 4, ModelVolumeType::SEAM_POSITION_INSIDE);
    const slic3r_api::PrintingExtrusion extrusion = fixture.add(
        rectangle_loop(-1, -1, 1, 1), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER,
        0, combined.get());
    const std::vector<slic3r_api::PrintRegion> assigned_regions = {
        assigned_view.print_region()};

    const slic3r_api::SeamPlacer placer = create_cost_session_for(
        fixture.prepared.print, assigned_regions);
    const c_point point = fixture.local(placer.place_seam(
        extrusion.root(), assigned_view, extrusion.root().front()));
    CHECK(point.x >= scale_i(0));
}

TEST_CASE("Cost seam sessions reject repeated initialization without changing state",
          "[plugins][seam-placer][cost][lifecycle]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.add(rectangle_loop(-5, -5, 5, 5),
                RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    PrintingPlan &native_plan = fixture.prepared.print.mutable_printing_plan();
    const slic3r_api::PrintingPlan plan(
        reinterpret_cast<printing_plan_handle *>(&native_plan));
    const std::vector<slic3r_api::PrintRegion> regions = {
        fixture.region_view().print_region()};
    PluginStorage storage;
    std::unique_ptr<slic3r_api::SeamPlacerSession> session =
        slic3r_api::SeamPlacement::CostPlugin::make_cost_seam_session(
            reinterpret_cast<storage_handle *>(&storage));
    session->initialize(plan, regions);
    CHECK_THROWS_WITH(session->initialize(plan, regions),
        "Cost-based seam placement sessions may only be initialized once.");

    const slic3r_api::PrintingExtrusion extrusion = fixture.extrusion(0);
    const c_point point = fixture.local(session->place_seam(
        extrusion.root(), fixture.region_view(), extrusion.root().front()));
    CHECK(point.x == scale_i(-5));
    CHECK(point.y == scale_i(-5));
}

TEST_CASE("Cost seam cancellation never publishes a partial session",
          "[plugins][seam-placer][cost][cancellation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("candidate preparation")
    {
        CostFixture fixture;
        fixture.add(rectangle_loop(-5, -5, 5, 5),
                    RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
        PrintingPlan &native_plan =
            fixture.prepared.print.mutable_printing_plan();
        const slic3r_api::PrintingPlan plan(
            reinterpret_cast<printing_plan_handle *>(&native_plan));
        const std::vector<slic3r_api::PrintRegion> regions = {
            fixture.region_view().print_region()};
        PluginStorage storage;
        CancellationProbe probe{0, 1};
        plugin_run_context run_context = {};
        run_context.plugin_storage =
            reinterpret_cast<storage_handle *>(&storage);
        run_context.host_context =
            reinterpret_cast<plugin_host_context *>(&probe);
        run_context.is_cancelled = &cancel_after_checks;
        raw_seam_placer_instance instance = {};

        CHECK_THROWS_AS(
            instance = slic3r_api::make_seam_placer_instance(
                slic3r_api::SeamPlacement::CostPlugin::
                    make_cost_seam_session(
                        reinterpret_cast<storage_handle *>(&storage),
                        &run_context),
                plan, regions),
            slic3r_api::PluginCancelled);
        CHECK(instance.session == nullptr);
    }

    SECTION("visibility model")
    {
        CostFixture baseline({{"seam_visibility", "0"},
                              {"seam_angle_cost", "0"},
                              {"seam_travel_cost", "0"}});
        baseline.add(rectangle_loop(-5, -5, 5, 5),
                     RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
        PrintingPlan &baseline_native_plan =
            baseline.prepared.print.mutable_printing_plan();
        const slic3r_api::PrintingPlan baseline_plan(
            reinterpret_cast<printing_plan_handle *>(
                &baseline_native_plan));
        const std::vector<slic3r_api::PrintRegion> baseline_regions = {
            baseline.region_view().print_region()};
        PluginStorage baseline_storage;
        CancellationProbe baseline_probe;
        plugin_run_context baseline_context = {};
        baseline_context.plugin_storage =
            reinterpret_cast<storage_handle *>(&baseline_storage);
        baseline_context.host_context =
            reinterpret_cast<plugin_host_context *>(&baseline_probe);
        baseline_context.is_cancelled = &cancel_after_checks;
        raw_seam_placer_instance baseline_instance =
            slic3r_api::make_seam_placer_instance(
                slic3r_api::SeamPlacement::CostPlugin::
                    make_cost_seam_session(
                        reinterpret_cast<storage_handle *>(&baseline_storage),
                        &baseline_context),
                baseline_plan, baseline_regions);
        const slic3r_api::SeamPlacer baseline_placer(baseline_instance);
        REQUIRE(baseline_placer.valid());

        CostFixture visible({{"seam_visibility", "1"},
                             {"seam_angle_cost", "0"},
                             {"seam_travel_cost", "0"}});
        visible.add(rectangle_loop(-5, -5, 5, 5),
                    RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
        PrintingPlan &visible_native_plan =
            visible.prepared.print.mutable_printing_plan();
        const slic3r_api::PrintingPlan visible_plan(
            reinterpret_cast<printing_plan_handle *>(
                &visible_native_plan));
        const std::vector<slic3r_api::PrintRegion> visible_regions = {
            visible.region_view().print_region()};
        PluginStorage visible_storage;
        CancellationProbe visible_probe{0, baseline_probe.checks + 1};
        plugin_run_context visible_context = {};
        visible_context.plugin_storage =
            reinterpret_cast<storage_handle *>(&visible_storage);
        visible_context.host_context =
            reinterpret_cast<plugin_host_context *>(&visible_probe);
        visible_context.is_cancelled = &cancel_after_checks;
        raw_seam_placer_instance visible_instance = {};

        CHECK_THROWS_AS(
            visible_instance = slic3r_api::make_seam_placer_instance(
                slic3r_api::SeamPlacement::CostPlugin::
                    make_cost_seam_session(
                        reinterpret_cast<storage_handle *>(&visible_storage),
                        &visible_context),
                visible_plan, visible_regions),
            slic3r_api::PluginCancelled);
        CHECK(visible_probe.checks == visible_probe.cancel_after);
        CHECK(visible_instance.session == nullptr);
    }
}

TEST_CASE("Cost seam diagnoses perimeter islands without external geometry",
          "[plugins][seam-placer][cost][errors]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.add(rectangle_loop(0, 0, 10, 10), RAW_EXTRUSION_ROLE_PERIMETER);
    CHECK_THROWS_WITH(create_cost_session(fixture.prepared.print),
        Catch::Matchers::Contains("found perimeter loops but no external geometric candidate"));
}

TEST_CASE("Cost seam diagnoses invalid source islands",
          "[plugins][seam-placer][cost][errors]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.add(rectangle_loop(0, 0, 10, 10), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    PrintingExtrusion &invalid = fixture.prepared.print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front().extrusions.front();
    invalid.region_island = nullptr;
    CHECK_THROWS_WITH(create_cost_session(fixture.prepared.print),
        "Cost-based seam placement found a perimeter loop with an invalid source LayerRegionIsland.");
}

TEST_CASE("Aligned seam places distance-only loops by direct distance projection",
          "[plugins][seam-placer][aligned][geometry]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("distance-only island")
    {
        CostFixture fixture;
        const slic3r_api::PrintingExtrusion skirt = fixture.add(
            rectangle_loop(0, 0, 10, 10), RAW_EXTRUSION_ROLE_SKIRT);

        const slic3r_api::SeamPlacer placer =
            create_aligned_session(fixture.prepared.print);
        const c_point start = {skirt.root().front().x + scale_i(4),
                               skirt.root().front().y - scale_i(3)};
        const c_point projected = fixture.local(placer.place_seam(
            skirt.root(), fixture.region_view(), start));

        CHECK(projected.x == scale_i(4));
        CHECK(projected.y == 0);
    }

    SECTION("mixed island")
    {
        CostFixture fixture;
        fixture.source_ptr = attach_native_source(fixture);
        fixture.add(rectangle_loop(-20, -20, 20, 20),
                    RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
        const slic3r_api::PrintingExtrusion skirt = fixture.add(
            rectangle_loop(0, 0, 10, 10), RAW_EXTRUSION_ROLE_SKIRT);

        const slic3r_api::SeamPlacer placer =
            create_aligned_session(fixture.prepared.print);
        const c_point start = {skirt.root().front().x + scale_i(4),
                               skirt.root().front().y - scale_i(3)};
        const c_point projected = fixture.local(placer.place_seam(
            skirt.root(), fixture.region_view(), start));

        CHECK(projected.x == scale_i(4));
        CHECK(projected.y == 0);
    }
}

TEST_CASE("Aligned seam projects exact external and nearest internal tracks across instances",
          "[plugins][seam-placer][aligned][geometry][instances][parallel]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.source_ptr = attach_native_source(fixture);
    PrintInstances &instances = const_cast<PrintInstances &>(fixture.object->instances());
    instances.push_back(instances.front());
    instances.back().shift += Point::new_scale(40, -25);
    fixture.add(rectangle_loop(-5, -5, 5, 5),
                RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 0);
    fixture.add(arc_loop(), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 1);
    fixture.add(rectangle_loop(-2, -2, 2, 2),
                RAW_EXTRUSION_ROLE_PERIMETER, 0);
    const slic3r_api::SeamPlacer placer = create_aligned_session(fixture.prepared.print);
    const slic3r_api::PrintingExtrusion line = fixture.extrusion(0);
    const slic3r_api::PrintingExtrusion arc = fixture.extrusion(1);
    const slic3r_api::PrintingExtrusion inner = fixture.extrusion(2);

    CHECK(placer.get_priority(line.root(), fixture.region_view()) == 50.0f);
    const Point far_positive = Point::new_scale(100, 100);
    const Point far_negative = Point::new_scale(-100, -100);
    const c_point line_point = placer.place_seam(
        line.root(), fixture.region_view(),
        {far_positive.x(), far_positive.y()});
    const c_point arc_point = placer.place_seam(
        arc.root(), fixture.region_view(),
        {far_negative.x(), far_negative.y()});
    const c_point inner_point = placer.place_seam(
        inner.root(), fixture.region_view(), {});
    CHECK(slic3r_api::SeamPlacement::CandidatePreparation::project_onto_loop(
        line.root(), line_point).second == 0.0);
    CHECK(slic3r_api::SeamPlacement::CandidatePreparation::project_onto_loop(
        arc.root(), arc_point).second == 0.0);
    CHECK(slic3r_api::SeamPlacement::CandidatePreparation::project_onto_loop(
        inner.root(), inner_point).second == 0.0);

    std::vector<std::future<c_point>> calls;
    for (size_t index = 0; index < 8; ++index)
        calls.push_back(std::async(std::launch::async, [&placer, inner, &fixture]() {
            return placer.place_seam(inner.root(), fixture.region_view(), {});
        }));
    for (std::future<c_point> &call : calls) {
        const c_point point = call.get();
        CHECK(point.x == inner_point.x);
        CHECK(point.y == inner_point.y);
    }

    const ExtrusionEntity foreign = rectangle_loop(-1, -1, 1, 1);
    CHECK_THROWS_WITH(placer.place_seam(
        slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(&foreign)),
        fixture.region_view(), {}),
        "Aligned seam loop was not present in the initialized plan.");
}

TEST_CASE("Aligned candidate filtering preserves ordered preference stages",
          "[plugins][seam-placer][aligned][constraints]")
{
    using slic3r_api::SeamPlacement::CandidatePreparation::Candidate;
    std::vector<Candidate> candidates(5);
    candidates[0].blocked = true;
    candidates[1].enforced = true;
    candidates[2].boundary = true;
    candidates[3].overhang = 1.0;
    candidates[3].width = 0.4;
    candidates[4].embedded = -1.0;
    for (Candidate &candidate : candidates)
        if (candidate.width == 0) candidate.width = 0.4;
    slic3r_api::SeamPlacement::CandidatePreparation::filter_candidates(candidates);
    REQUIRE(candidates.size() == 1);
    CHECK(candidates.front().enforced);

    std::vector<Candidate> all_blocked(2);
    all_blocked[0].blocked = true;
    all_blocked[1].blocked = true;
    slic3r_api::SeamPlacement::CandidatePreparation::filter_candidates(all_blocked);
    CHECK(all_blocked.size() == 2);
}

TEST_CASE("Aligned seam rejects repeated initialization and invalid requests",
          "[plugins][seam-placer][aligned][errors]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.source_ptr = attach_native_source(fixture);
    fixture.add(rectangle_loop(-5, -5, 5, 5),
                RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    PrintingPlan &native_plan = fixture.prepared.print.mutable_printing_plan();
    const slic3r_api::PrintingPlan plan(
        reinterpret_cast<printing_plan_handle *>(&native_plan));
    const slic3r_api::Print print_view(
        reinterpret_cast<const print_handle *>(&fixture.prepared.print));
    std::vector<slic3r_api::PrintRegion> regions{
        print_view.object(0).print_region(0)};
    PluginStorage storage;
    std::unique_ptr<slic3r_api::SeamPlacerSession> session =
        slic3r_api::SeamPlacement::AlignedPlugin::make_aligned_seam_session(
            reinterpret_cast<storage_handle *>(&storage));
    session->initialize(plan, regions);
    CHECK_THROWS_WITH(session->initialize(plan, regions),
        "Aligned seam placement sessions may only be initialized once.");

    const slic3r_api::PrintingExtrusion extrusion = fixture.extrusion(0);
    CHECK_THROWS_WITH(session->place_seam(
        extrusion.root(), slic3r_api::LayerRegion(), {}),
        "Aligned seam placement needs a valid LayerRegion.");
    const ExtrusionEntity open(false, ArcPolyline(Points{
        Point::new_scale(-1, 0), Point::new_scale(1, 0)}));
    CHECK_THROWS_WITH(session->place_seam(
        slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(&open)),
        fixture.region_view(), {}),
        "Aligned seam placement needs a non-empty closed loop.");
}

TEST_CASE("Aligned weighted spline reports valid fits and rejects non-finite input",
          "[plugins][seam-placer][aligned][spline]")
{
    using Slic3r::SeamCostModel::WeightedSplinePoint;
    std::vector<WeightedSplinePoint> points;
    for (size_t index = 0; index < 6; ++index)
        points.push_back({double(index % 2), 0.0, double(index), 1.0});
    std::vector<c_vec2d> fitted;
    REQUIRE(Slic3r::SeamCostModel::fit_weighted_cubic_bspline(points, fitted));
    REQUIRE(fitted.size() == points.size());
    for (const c_vec2d point : fitted) {
        CHECK(std::isfinite(point.x));
        CHECK(std::isfinite(point.y));
    }
    points[2].x = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(Slic3r::SeamCostModel::fit_weighted_cubic_bspline(points, fitted));
    CHECK(fitted.empty());
}

TEST_CASE("Aligned weighted spline keeps long chains in the finite fallback path",
          "[plugins][seam-placer][aligned][spline][regression]")
{
    using Slic3r::SeamCostModel::WeightedSplinePoint;
    std::vector<WeightedSplinePoint> points;
    for (size_t index = 0; index < 6; ++index)
        points.push_back({double(index * 10), 0.0, double(index), 1.0});
    std::vector<c_vec2d> fitted;
    const auto fit = [&] {
        const bool success = Slic3r::SeamCostModel::fit_weighted_cubic_bspline(points, fitted);
        if (success) {
            REQUIRE(fitted.size() == points.size());
            for (const c_vec2d point : fitted) {
                CHECK(std::isfinite(point.x));
                CHECK(std::isfinite(point.y));
            }
        } else {
            CHECK(fitted.empty());
        }
    };
    CHECK_NOTHROW(fit());
}

TEST_CASE("Aligned weighted spline rejects a float-overflowing Z span",
          "[plugins][seam-placer][aligned][spline][overflow]")
{
    using Slic3r::SeamCostModel::WeightedSplinePoint;
    const double maximum = double(std::numeric_limits<float>::max());
    const std::vector<WeightedSplinePoint> points = {
        {0.0, 0.0, -maximum, 1.0},
        {1.0, 0.0, -1.0, 1.0},
        {2.0, 0.0, 1.0, 1.0},
        {3.0, 0.0, maximum, 1.0}};
    std::vector<c_vec2d> fitted;
    const auto fit = [&] {
        const bool success = Slic3r::SeamCostModel::fit_weighted_cubic_bspline(points, fitted);
        if (success) {
            REQUIRE(fitted.size() == points.size());
            for (const c_vec2d point : fitted) {
                CHECK(std::isfinite(point.x));
                CHECK(std::isfinite(point.y));
            }
        } else {
            CHECK(fitted.empty());
        }
    };
    CHECK_NOTHROW(fit());
}

TEST_CASE("Aligned seam smooths six linked layers but keeps broken short chains local",
          "[plugins][seam-placer][aligned][graph][chain]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({
        {"seam_visibility", "0"}, {"seam_angle_cost", "0"},
        {"seam_travel_cost", "0"}}));
    PrintObject &object = prepared.print.object(0);
    REQUIRE(object.layer_count() >= 6);
    prepared.print.mutable_printing_plan() = PrintingPlan();
    for (size_t layer_index = 0; layer_index < 6; ++layer_index) {
        Layer &layer = object.layer(layer_index);
        LayerRegionIsland *source = native_region_island(layer);
        REQUIRE(source != nullptr);
        const double offset = layer_index % 2 == 0 ? 0.0 : 1.0;
        append_aligned_loop(prepared.print, layer, *source,
            rectangle_loop(-5 + offset, -5, 5 + offset, 5), 0);
    }
    const slic3r_api::SeamPlacer linked = create_aligned_session(prepared.print);
    slic3r_api::PrintingPlan linked_plan(
        reinterpret_cast<printing_plan_handle *>(
            &prepared.print.mutable_printing_plan()));
    for (uint32_t layer_index = 0; layer_index < 6; ++layer_index) {
        const slic3r_api::PrintingExtrusion extrusion = linked_plan.group(0)
            .layer_group(layer_index).tool_group(0).extrusion(0);
        const slic3r_api::LayerRegion region = extrusion.region_island().region(0);
        const c_point seam = linked.place_seam(extrusion.root(), region, {});
        CHECK(slic3r_api::SeamPlacement::CandidatePreparation::project_onto_loop(
            extrusion.root(), seam).second == 0.0);
    }

    LayerSliceIsland &lower = object.layer(2).island(0);
    LayerSliceIsland &upper = object.layer(3).island(0);
    lower.overlaps_above.clear();
    upper.overlaps_below.clear();
    const slic3r_api::SeamPlacer broken = create_aligned_session(prepared.print);
    for (uint32_t layer_index = 0; layer_index < 6; ++layer_index) {
        const slic3r_api::PrintingExtrusion extrusion = linked_plan.group(0)
            .layer_group(layer_index).tool_group(0).extrusion(0);
        const slic3r_api::LayerRegion region = extrusion.region_island().region(0);
        const c_point seam = broken.place_seam(extrusion.root(), region, {});
        CHECK(seam.x == extrusion.root().front().x);
        CHECK(seam.y == extrusion.root().front().y);
    }
}

TEST_CASE("Aligned graph ties are stable and leave the unchosen branch available",
          "[plugins][seam-placer][aligned][graph][branch][determinism]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({
        {"seam_visibility", "0"}, {"seam_angle_cost", "0"},
        {"seam_travel_cost", "0"}}));
    PrintObject &object = prepared.print.object(0);
    REQUIRE(object.layer_count() >= 7);

    const ExPolygon whole = rectangle_expolygon(-5, -5, 5, 5);
    const ExPolygon left = rectangle_expolygon(-5, -5, -0.1, 5);
    const ExPolygon right = rectangle_expolygon(0.1, -5, 5, 5);
    replace_test_layer_islands(object.layer(0), {whole});
    for (size_t layer_index = 1; layer_index < 7; ++layer_index)
        replace_test_layer_islands(object.layer(layer_index), {left, right});
    rebuild_test_island_graph(object);

    prepared.print.mutable_printing_plan() = PrintingPlan();
    std::vector<slic3r_api::PrintingExtrusion> tracks;
    tracks.push_back(append_aligned_loop(prepared.print, object.layer(0),
        *native_region_island_at(object.layer(0), 0), rounded_loop(0), 0));
    const std::vector<double> right_roots = {0.5, 0.8, 1.4, 1.1, 1.8, 1.3};
    const std::vector<double> right_centers_y = {0.0, 0.6, -0.6, 0.6, -0.6, 0.6};
    for (size_t layer_index = 1; layer_index < 7; ++layer_index) {
        tracks.push_back(append_aligned_loop(prepared.print, object.layer(layer_index),
            *native_region_island_at(object.layer(layer_index), 0),
            rounded_loop(-0.5), 0));
        const double right_root = right_roots[layer_index - 1];
        tracks.push_back(append_aligned_loop(prepared.print, object.layer(layer_index),
            *native_region_island_at(object.layer(layer_index), 1),
            rounded_loop(right_root, right_centers_y[layer_index - 1]), 0));
    }

    REQUIRE(object.layer(0).island(0).overlaps_above.size() == 2);
    const slic3r_api::SeamPlacer first = create_aligned_session(prepared.print);
    std::vector<c_point> first_seams;
    first_seams.reserve(tracks.size());
    for (const slic3r_api::PrintingExtrusion &track : tracks)
        first_seams.push_back(aligned_seam(first, track));

    // The two branch targets are equal-quality and equal-distance. Reversing
    // the borrowed link order must not change the stable island-index winner.
    std::reverse(object.layer(0).island(0).overlaps_above.begin(),
                 object.layer(0).island(0).overlaps_above.end());
    const slic3r_api::SeamPlacer reordered = create_aligned_session(prepared.print);
    for (size_t index = 0; index < tracks.size(); ++index) {
        const c_point seam = aligned_seam(reordered, tracks[index]);
        CHECK(seam.x == first_seams[index].x);
        CHECK(seam.y == first_seams[index].y);
    }
    const bool unchosen_branch_was_smoothed =
        first_seams[4].x != tracks[4].root().front().x ||
        first_seams[4].y != tracks[4].root().front().y;
    CHECK(unchosen_branch_was_smoothed);
}

TEST_CASE("Aligned graph uses one winner at a merge",
          "[plugins][seam-placer][aligned][graph][merge]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({
        {"seam_visibility", "0"}, {"seam_angle_cost", "0"},
        {"seam_travel_cost", "0"}}));
    PrintObject &object = prepared.print.object(0);
    REQUIRE(object.layer_count() >= 7);

    const ExPolygon left = rectangle_expolygon(-5, -5, -0.1, 5);
    const ExPolygon right = rectangle_expolygon(0.1, -5, 5, 5);
    const ExPolygon whole = rectangle_expolygon(-5, -5, 5, 5);
    replace_test_layer_islands(object.layer(0), {left, right});
    for (size_t layer_index = 1; layer_index < 7; ++layer_index)
        replace_test_layer_islands(object.layer(layer_index), {whole});
    rebuild_test_island_graph(object);

    prepared.print.mutable_printing_plan() = PrintingPlan();
    std::vector<slic3r_api::PrintingExtrusion> tracks;
    tracks.push_back(append_aligned_loop(prepared.print, object.layer(0),
        *native_region_island_at(object.layer(0), 0), rounded_loop(-0.5), 0));
    tracks.push_back(append_aligned_loop(prepared.print, object.layer(0),
        *native_region_island_at(object.layer(0), 1), rounded_loop(0.5), 0));
    const std::vector<double> roots = {0.0, 0.4, 0.8, 1.2, 1.6, 2.0};
    const std::vector<double> centers_y = {0.5, -0.5, 0.5, -0.5, 0.5, -0.5};
    for (size_t layer_index = 1; layer_index < 7; ++layer_index)
        tracks.push_back(append_aligned_loop(prepared.print, object.layer(layer_index),
            *native_region_island_at(object.layer(layer_index), 0),
            rounded_loop(roots[layer_index - 1], centers_y[layer_index - 1]), 0));

    REQUIRE(object.layer(0).island(0).overlaps_above.size() == 1);
    REQUIRE(object.layer(0).island(1).overlaps_above.size() == 1);
    const slic3r_api::SeamPlacer placer = create_aligned_session(prepared.print);
    const c_point unchosen = aligned_seam(placer, tracks[1]);
    CHECK(unchosen.x == tracks[1].root().front().x);
    CHECK(unchosen.y == tracks[1].root().front().y);

    const c_point merged_target = aligned_seam(placer, tracks[2]);
    const bool merged_target_was_smoothed =
        merged_target.x != tracks[2].root().front().x ||
        merged_target.y != tracks[2].root().front().y;
    CHECK(merged_target_was_smoothed);
}

TEST_CASE("Aligned graph ignores links that skip a layer",
          "[plugins][seam-placer][aligned][graph][adjacency]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({
        {"seam_visibility", "0"}, {"seam_angle_cost", "0"},
        {"seam_travel_cost", "0"}}));
    PrintObject &object = prepared.print.object(0);
    REQUIRE(object.layer_count() >= 10);
    const ExPolygon whole = rectangle_expolygon(-5, -5, 5, 5);
    for (size_t layer_index = 0; layer_index < 10; ++layer_index)
        replace_test_layer_islands(object.layer(layer_index), {whole});
    rebuild_test_island_graph(object);

    prepared.print.mutable_printing_plan() = PrintingPlan();
    std::vector<slic3r_api::PrintingExtrusion> tracks;
    tracks.push_back(append_aligned_loop(prepared.print, object.layer(1),
        *native_region_island_at(object.layer(1), 0), rounded_loop(0), 0));
    tracks.push_back(append_aligned_loop(prepared.print, object.layer(2),
        *native_region_island_at(object.layer(2), 0), rounded_loop(0), 0));
    const std::vector<double> roots = {1.0, 1.5, 1.0, 1.5, 1.0, 1.5};
    const std::vector<double> centers_y = {0.0, 0.7, -0.7, 0.7, -0.7, 0.7};
    for (size_t layer_index = 4; layer_index < 10; ++layer_index)
        tracks.push_back(append_aligned_loop(prepared.print, object.layer(layer_index),
            *native_region_island_at(object.layer(layer_index), 0),
            rounded_loop(roots[layer_index - 4], centers_y[layer_index - 4]), 0));

    // The extra edge is deliberately invalid for a one-layer continuation.
    object.layer(2).island(0).overlaps_above.push_back({
        &object.layer(4).island(0), 1.0f});
    REQUIRE(object.layer(2).island(0).overlaps_above.size() == 2);
    const slic3r_api::SeamPlacer placer = create_aligned_session(prepared.print);
    const c_point source = aligned_seam(placer, tracks[1]);
    CHECK(source.x == tracks[1].root().front().x);
    CHECK(source.y == tracks[1].root().front().y);
}

TEST_CASE("Contiguous seam projects a direct lower seam and stops at graph breaks",
          "[plugins][seam-placer][contiguous][graph]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({
        {"seam_visibility", "0"}, {"seam_angle_cost", "0"},
        {"seam_travel_cost", "0"}}));
    PrintObject &object = prepared.print.object(0);
    REQUIRE(object.layer_count() >= 2);
    prepared.print.mutable_printing_plan() = PrintingPlan();

    const slic3r_api::PrintingExtrusion lower = append_aligned_loop(
        prepared.print, object.layer(0),
        *native_region_island(object.layer(0)),
        rectangle_loop(-5, -5, 5, 5), 0);
    const slic3r_api::PrintingExtrusion upper = append_aligned_loop(
        prepared.print, object.layer(1),
        *native_region_island(object.layer(1)),
        rectangle_loop(-10, -3, 0, 7), 0);

    const slic3r_api::SeamPlacer linked =
        create_contiguous_session(prepared.print);
    const c_point lower_seam = contiguous_seam(linked, lower);
    const c_point expected =
        slic3r_api::SeamPlacement::CandidatePreparation::project_onto_loop(
            upper.root(), lower_seam).first;
    const c_point propagated = contiguous_seam(linked, upper);
    CHECK(propagated.x == expected.x);
    CHECK(propagated.y == expected.y);
    CHECK((propagated.x != upper.root().front().x ||
           propagated.y != upper.root().front().y));

    const c_point far_start = {scale_i(100), scale_i(100)};
    const c_point repeated = linked.place_seam(
        upper.root(), upper.region_island().region(0), far_start);
    CHECK(repeated.x == propagated.x);
    CHECK(repeated.y == propagated.y);

    object.layer(0).island(0).overlaps_above.clear();
    object.layer(1).island(0).overlaps_below.clear();
    const slic3r_api::SeamPlacer broken =
        create_contiguous_session(prepared.print);
    const c_point local = contiguous_seam(broken, upper);
    CHECK(local.x == upper.root().front().x);
    CHECK(local.y == upper.root().front().y);
}

TEST_CASE("Contiguous seam lets a local enforcer restart propagation",
          "[plugins][seam-placer][contiguous][constraints][enforcer]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({
        {"seam_visibility", "0"}, {"seam_angle_cost", "0"},
        {"seam_travel_cost", "0"}}));
    PrintObject &object = prepared.print.object(0);
    REQUIRE(object.layer_count() >= 2);
    prepared.print.mutable_printing_plan() = PrintingPlan();

    const slic3r_api::PrintingExtrusion lower = append_aligned_loop(
        prepared.print, object.layer(0),
        *native_region_island(object.layer(0)),
        rectangle_loop(-5, -5, 5, 5), 0);
    const slic3r_api::PrintingExtrusion upper = append_aligned_loop(
        prepared.print, object.layer(1),
        *native_region_island(object.layer(1)),
        rectangle_loop(-10, -3, 0, 7), 0);

    const slic3r_api::SeamPlacer baseline =
        create_contiguous_session(prepared.print);
    const c_point propagated = contiguous_seam(baseline, upper);

    // The modifier spans both layers in Z, but only intersects the upper loop
    // in XY, so the lower reference seam remains unchanged.
    add_seam_constraint_box(
        object, -10.5, -3.5, 1.0, 1.0,
        ModelVolumeType::SEAM_POSITION_INSIDE,
        EnforcerBlockerType::NONE, 0, 30);
    const slic3r_api::SeamPlacer enforced =
        create_contiguous_session(prepared.print);
    const c_point reset = contiguous_seam(enforced, upper);
    CHECK((reset.x != propagated.x || reset.y != propagated.y));

    const Point shift = object.instances().front().shift;
    const c_point local_reset = {reset.x - shift.x(), reset.y - shift.y()};
    CHECK(unscaled(local_reset.x) <= -9.49);
    CHECK(unscaled(local_reset.x) >= -10.01);
    CHECK(unscaled(local_reset.y) >= -3.51);
    CHECK(unscaled(local_reset.y) <= -2.49);
}

TEST_CASE("Contiguous seam uses stable lower-track order at a merge",
          "[plugins][seam-placer][contiguous][graph][merge][determinism]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({
        {"seam_visibility", "0"}, {"seam_angle_cost", "0"},
        {"seam_travel_cost", "0"}}));
    PrintObject &object = prepared.print.object(0);
    REQUIRE(object.layer_count() >= 2);

    replace_test_layer_islands(object.layer(0), {
        rectangle_expolygon(-5, -5, -0.1, 5),
        rectangle_expolygon(0.1, -5, 5, 5)});
    replace_test_layer_islands(object.layer(1), {
        rectangle_expolygon(-5, -5, 5, 5)});
    rebuild_test_island_graph(object);
    prepared.print.mutable_printing_plan() = PrintingPlan();

    const slic3r_api::PrintingExtrusion left = append_aligned_loop(
        prepared.print, object.layer(0),
        *native_region_island_at(object.layer(0), 0),
        rectangle_loop(-5, -5, -0.1, 5), 0);
    const slic3r_api::PrintingExtrusion right = append_aligned_loop(
        prepared.print, object.layer(0),
        *native_region_island_at(object.layer(0), 1),
        rectangle_loop(0.1, -5, 5, 5), 0);
    const ExtrusionEntity upper_loop(false, ArcPolyline(Points{
        Point::new_scale(5, 5), Point::new_scale(-5, 5),
        Point::new_scale(-5, -5), Point::new_scale(5, -5),
        Point::new_scale(5, 5)}));
    const slic3r_api::PrintingExtrusion upper = append_aligned_loop(
        prepared.print, object.layer(1),
        *native_region_island_at(object.layer(1), 0), upper_loop, 0);

    const slic3r_api::SeamPlacer first =
        create_contiguous_session(prepared.print);
    const c_point left_seam = contiguous_seam(first, left);
    const c_point right_seam = contiguous_seam(first, right);
    const std::pair<c_point, double> left_projection =
        slic3r_api::SeamPlacement::CandidatePreparation::project_onto_loop(
            upper.root(), left_seam);
    const std::pair<c_point, double> right_projection =
        slic3r_api::SeamPlacement::CandidatePreparation::project_onto_loop(
            upper.root(), right_seam);
    REQUIRE(left_projection.second == right_projection.second);
    const c_point merged = contiguous_seam(first, upper);
    CHECK(merged.x == left_projection.first.x);
    CHECK(merged.y == left_projection.first.y);

    std::reverse(object.layer(1).island(0).overlaps_below.begin(),
                 object.layer(1).island(0).overlaps_below.end());
    const slic3r_api::SeamPlacer reordered =
        create_contiguous_session(prepared.print);
    const c_point stable = contiguous_seam(reordered, upper);
    CHECK(stable.x == merged.x);
    CHECK(stable.y == merged.y);
}

TEST_CASE("Contiguous seam lets both graph branches reuse one lower seam",
          "[plugins][seam-placer][contiguous][graph][branch]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({
        {"seam_visibility", "0"}, {"seam_angle_cost", "0"},
        {"seam_travel_cost", "0"}}));
    PrintObject &object = prepared.print.object(0);
    REQUIRE(object.layer_count() >= 2);

    replace_test_layer_islands(object.layer(0), {
        rectangle_expolygon(-5, -5, 5, 5)});
    replace_test_layer_islands(object.layer(1), {
        rectangle_expolygon(-5, -5, -0.1, 5),
        rectangle_expolygon(0.1, -5, 5, 5)});
    rebuild_test_island_graph(object);
    prepared.print.mutable_printing_plan() = PrintingPlan();

    const slic3r_api::PrintingExtrusion lower = append_aligned_loop(
        prepared.print, object.layer(0),
        *native_region_island_at(object.layer(0), 0),
        rectangle_loop(-5, -5, 5, 5), 0);
    const slic3r_api::PrintingExtrusion left = append_aligned_loop(
        prepared.print, object.layer(1),
        *native_region_island_at(object.layer(1), 0),
        rectangle_loop(-5, -4, -0.1, 6), 0);
    const slic3r_api::PrintingExtrusion right = append_aligned_loop(
        prepared.print, object.layer(1),
        *native_region_island_at(object.layer(1), 1),
        rectangle_loop(0.1, -4, 5, 6), 0);

    const slic3r_api::SeamPlacer placer =
        create_contiguous_session(prepared.print);
    const c_point source = contiguous_seam(placer, lower);
    for (const slic3r_api::PrintingExtrusion &branch : {left, right}) {
        const c_point expected =
            slic3r_api::SeamPlacement::CandidatePreparation::project_onto_loop(
                branch.root(), source).first;
        const c_point seam = contiguous_seam(placer, branch);
        CHECK(seam.x == expected.x);
        CHECK(seam.y == expected.y);
    }
}

TEST_CASE("Contiguous seam projects technical and internal loops",
          "[plugins][seam-placer][contiguous][geometry][instances][parallel]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.source_ptr = attach_native_source(fixture);
    PrintInstances &instances =
        const_cast<PrintInstances &>(fixture.object->instances());
    instances.push_back(instances.front());
    instances.back().shift += Point::new_scale(40, -25);
    fixture.add(rectangle_loop(-5, -5, 5, 5),
                RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 0);
    fixture.add(arc_loop(), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 1);
    fixture.add(rectangle_loop(-2, -2, 2, 2),
                RAW_EXTRUSION_ROLE_PERIMETER, 0);
    const slic3r_api::PrintingExtrusion skirt = fixture.add(
        rectangle_loop(0, 0, 10, 10), RAW_EXTRUSION_ROLE_SKIRT);

    const slic3r_api::SeamPlacer placer =
        create_contiguous_session(fixture.prepared.print);
    const slic3r_api::PrintingExtrusion line = fixture.extrusion(0);
    const slic3r_api::PrintingExtrusion arc = fixture.extrusion(1);
    const slic3r_api::PrintingExtrusion inner = fixture.extrusion(2);
    REQUIRE(placer.get_priority(line.root(), fixture.region_view()) == 50.0f);

    for (const slic3r_api::PrintingExtrusion &extrusion : {line, arc, inner}) {
        const c_point seam = placer.place_seam(
            extrusion.root(), fixture.region_view(), {});
        CHECK(slic3r_api::SeamPlacement::CandidatePreparation::
                  project_onto_loop(extrusion.root(), seam).second == 0.0);
    }
    const c_point start = {skirt.root().front().x + scale_i(4),
                           skirt.root().front().y - scale_i(3)};
    const c_point projected = fixture.local(placer.place_seam(
        skirt.root(), fixture.region_view(), start));
    CHECK(projected.x == scale_i(4));
    CHECK(projected.y == 0);

    const c_point expected = placer.place_seam(
        inner.root(), fixture.region_view(), {});
    std::vector<std::future<c_point>> calls;
    for (size_t index = 0; index < 8; ++index)
        calls.push_back(std::async(
            std::launch::async, [&placer, inner, &fixture]() {
                return placer.place_seam(
                    inner.root(), fixture.region_view(), {});
            }));
    for (std::future<c_point> &call : calls) {
        const c_point seam = call.get();
        CHECK(seam.x == expected.x);
        CHECK(seam.y == expected.y);
    }
}

TEST_CASE("Contiguous seam rejects repeated initialization and invalid requests",
          "[plugins][seam-placer][contiguous][errors]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.source_ptr = attach_native_source(fixture);
    fixture.add(rectangle_loop(-5, -5, 5, 5),
                RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    PrintingPlan &native_plan = fixture.prepared.print.mutable_printing_plan();
    const slic3r_api::PrintingPlan plan(
        reinterpret_cast<printing_plan_handle *>(&native_plan));
    const std::vector<slic3r_api::PrintRegion> regions = {
        fixture.region_view().print_region()};
    PluginStorage storage;
    std::unique_ptr<slic3r_api::SeamPlacerSession> session =
        slic3r_api::SeamPlacement::ContiguousPlugin::
            make_contiguous_seam_session(
                reinterpret_cast<storage_handle *>(&storage));
    session->initialize(plan, regions);
    CHECK_THROWS_WITH(session->initialize(plan, regions),
        "Contiguous seam placement sessions may only be initialized once.");

    const slic3r_api::PrintingExtrusion extrusion = fixture.extrusion(0);
    CHECK_THROWS_WITH(session->place_seam(
        extrusion.root(), slic3r_api::LayerRegion(), {}),
        "Contiguous seam placement needs a valid LayerRegion.");
    const ExtrusionEntity open(false, ArcPolyline(Points{
        Point::new_scale(-1, 0), Point::new_scale(1, 0)}));
    CHECK_THROWS_WITH(session->place_seam(
        slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(&open)),
        fixture.region_view(), {}),
        "Contiguous seam placement needs a non-empty closed loop.");
}
