#include <catch2/catch.hpp>

#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config/ConfigOption.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepPostPerimeterGeneration.hpp"
#include "libslic3r/Steps/StepPrepareForPeriemters.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "libslic3r/Steps/StepSupportDemand.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace Slic3r;

struct Box
{
    double min_x = 0.;
    double min_y = 0.;
    double min_z = 0.;
    double max_x = 0.;
    double max_y = 0.;
    double max_z = 0.;
};

struct ModifierBox
{
    ModelVolumeType type = ModelVolumeType::INVALID;
    Box box;
};

struct DemandSummary
{
    uint32_t entry_count = 0;
    size_t polygon_count = 0;
    double area_mm2 = 0.;
    size_t anchored_roof_spans = 0;
    size_t free_roof_spans = 0;
    double longest_roof_span_mm = 0.;
};

TriangleMesh make_box(const Box &box)
{
    std::vector<Vec3f> vertices = {
        {float(box.min_x), float(box.min_y), float(box.min_z)},
        {float(box.max_x), float(box.min_y), float(box.min_z)},
        {float(box.max_x), float(box.max_y), float(box.min_z)},
        {float(box.min_x), float(box.max_y), float(box.min_z)},
        {float(box.min_x), float(box.min_y), float(box.max_z)},
        {float(box.max_x), float(box.min_y), float(box.max_z)},
        {float(box.max_x), float(box.max_y), float(box.max_z)},
        {float(box.min_x), float(box.max_y), float(box.max_z)}
    };
    std::vector<Vec3i32> faces = {
        {0, 2, 1}, {0, 3, 2},
        {4, 5, 6}, {4, 6, 7},
        {0, 1, 5}, {0, 5, 4},
        {1, 2, 6}, {1, 6, 5},
        {2, 3, 7}, {2, 7, 6},
        {3, 0, 4}, {3, 4, 7}
    };
    return TriangleMesh(std::move(vertices), std::move(faces));
}

TriangleMesh make_sloped_cube(const double top_x_expansion)
{
    // Same one-sided overhang geometry as the SupportDemandOverhangs tests:
    // bottom footprint X=-10..10, top footprint X=-10..10+top_x_expansion.
    // It gives the blocker tests a known support-demand strip on the +X side.
    const Box base = {-10., -10., 0., 10., 10., 10.};
    const float top_right_x = float(base.max_x + top_x_expansion);
    std::vector<Vec3f> vertices = {
        {float(base.min_x), float(base.min_y), float(base.min_z)},
        {float(base.max_x), float(base.min_y), float(base.min_z)},
        {float(base.max_x), float(base.max_y), float(base.min_z)},
        {float(base.min_x), float(base.max_y), float(base.min_z)},
        {float(base.min_x), float(base.min_y), float(base.max_z)},
        {top_right_x,       float(base.min_y), float(base.max_z)},
        {top_right_x,       float(base.max_y), float(base.max_z)},
        {float(base.min_x), float(base.max_y), float(base.max_z)}
    };
    std::vector<Vec3i32> faces = {
        {0, 2, 1}, {0, 3, 2},
        {4, 5, 6}, {4, 6, 7},
        {0, 1, 5}, {0, 5, 4},
        {1, 2, 6}, {1, 6, 5},
        {2, 3, 7}, {2, 7, 6},
        {3, 0, 4}, {3, 4, 7}
    };
    return TriangleMesh(std::move(vertices), std::move(faces));
}

DynamicPrintConfig support_demand_config(const bool support_material,
                                         const bool support_material_auto,
                                         const int support_material_threshold,
                                         const int support_material_enforce_layers,
                                         const bool dont_support_bridges = false)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", "1"},
        {"first_layer_height", "1"},
        {"nozzle_diameter", "0.4"},
        {"dont_support_bridges", dont_support_bridges ? "1" : "0"},
        {"support_material", support_material ? "1" : "0"},
        {"support_material_auto", support_material_auto ? "1" : "0"},
        {"support_material_threshold", std::to_string(support_material_threshold)},
        {"support_material_enforce_layers", std::to_string(support_material_enforce_layers)}
    });
    return config;
}

ModelVolume *add_volume(ModelObject &object, TriangleMesh &&mesh, const ModelVolumeType type)
{
    ModelVolume *volume = object.add_volume(std::move(mesh), type, false);
    volume->set_type(type);
    return volume;
}

void paint_support_facets(ModelVolume &volume, EnforcerBlockerType type, std::initializer_list<int> facets)
{
    const char *encoded_state = type == EnforcerBlockerType::ENFORCER ? "4" :
                                type == EnforcerBlockerType::BLOCKER  ? "8" : "0";
    volume.supported_facets.reserve(int(facets.size()));
    for (int facet_idx : facets)
        volume.supported_facets.set_triangle_from_string(facet_idx, encoded_state);
    volume.supported_facets.shrink_to_fit();
}

Model make_model(TriangleMesh &&model_part, std::initializer_list<ModifierBox> modifiers)
{
    Model model;
    ModelObject *object = model.add_object();
    object->name = "support_modifier_test.stl";
    add_volume(*object, std::move(model_part), ModelVolumeType::MODEL_PART);
    for (const ModifierBox &modifier : modifiers)
        add_volume(*object, make_box(modifier.box), modifier.type);
    object->add_instance();
    return model;
}

Model make_bridge_model()
{
    // Two pillars carry a one-layer roof. The roof layer spans a central gap,
    // so automatic support demand sees unsupported material, while generated
    // perimeter extrusions have straight anchored spans crossing that gap.
    Model model;
    ModelObject *object = model.add_object();
    object->name = "support_bridge_removal_test.stl";
    TriangleMesh mesh = make_box({-12., -4., 0., -5., 4., 4.});
    mesh.merge(make_box({  5., -4., 0., 12., 4., 4.}));
    mesh.merge(make_box({-12., -4., 4., 12., 4., 5.}));
    add_volume(*object, std::move(mesh), ModelVolumeType::MODEL_PART);
    object->add_instance();
    return model;
}

Model make_cantilever_model()
{
    // One pillar carries only the left side of the roof. The unsupported roof
    // edge has a free end, so it should not be classified as a real bridge by
    // the endpoint-support test.
    Model model;
    ModelObject *object = model.add_object();
    object->name = "support_cantilever_keep_test.stl";
    TriangleMesh mesh = make_box({-12., -4., 0., -5., 4., 4.});
    mesh.merge(make_box({-12., -4., 4., 12., 4., 5.}));
    add_volume(*object, std::move(mesh), ModelVolumeType::MODEL_PART);
    object->add_instance();
    return model;
}

void init_print_from_model(Model &model, Print &print, const DynamicPrintConfig &config)
{
    model.center_instances_around_point({100, 100});
    for (ModelObject &object : model.objects()) {
        object.ensure_on_bed();
        print.auto_assign_extruders(&object);
    }
    print.apply(model, config);
    print.validate();
    print.set_status_silent();
}

void run_until_support_demand_input(Orchestrator &orchestrator, Print &print)
{
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    Steps::StepSlicing::run_step(orchestrator, print);
    Steps::StepPostSlicing::run_step(orchestrator, print);
}

// Run the perimeter steps after STEP_SUPPORT_DEMAND: bridge removal refines the
// demand in STEP_POST_PERIMETER, once the perimeter extrusions exist.
void run_perimeter_steps(Orchestrator &orchestrator, Print &print)
{
    Steps::StepPrepareForPeriemters::prepare(print);
    Steps::StepPrepareForPeriemters::run_step(orchestrator, print);
    Steps::StepGeneratePerimeter::prepare(print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, print);
    Steps::StepPostPerimeterGeneration::prepare(print);
    Steps::StepPostPerimeterGeneration::run_step(orchestrator, print);
    bool has_perimeters = false;
    for (const PrintObject &object : print.objects())
        for (const Layer &layer : object.layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region : island.regions_islands())
                    has_perimeters |= region.has_extrusion(LayerRegionIsland::PERIMETERS);
    REQUIRE(has_perimeters);
}

// Sum the support demand stored in the object's islands; entry_count counts the
// islands that ask for support.
DemandSummary summarize_support_demand(const PrintObject &object)
{
    DemandSummary summary;
    for (const Layer &layer : object.layers())
        for (const LayerSliceIsland &island : layer.islands()) {
            const ExPolygons &polygons = island.support_demand();
            if (polygons.empty())
                continue;
            ++summary.entry_count;
            summary.polygon_count += polygons.size();
            summary.area_mm2 += unscaled(unscaled(area(polygons)));
        }
    return summary;
}

DemandSummary run_support_demand(Model &&model, const DynamicPrintConfig &config, const bool prepare_extrusions = false)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    init_print_from_model(model, print, config);

    Orchestrator &orchestrator = Orchestrator::instance();
    run_until_support_demand_input(orchestrator, print);
    Steps::StepSupportDemand::run_step(orchestrator, print);
    if (prepare_extrusions)
        run_perimeter_steps(orchestrator, print);
    DemandSummary summary = summarize_support_demand(print.object(0));
    if (prepare_extrusions) {
        const PrintObject &object = print.object(0);
        REQUIRE(object.layer_count() >= 2);
        for (size_t layer_idx = 1; layer_idx < object.layer_count(); ++layer_idx) {
            const Layer &roof = object.layer(layer_idx);
            // The fixture's 0.4 mm nozzle gives BridgeRemoval a 0.2 mm lower-support probe.
            const ExPolygons lower_support = offset_ex(object.layer(layer_idx - 1).lslices(),
                                                       scale_d(0.2), ClipperLib::jtSquare);
            const auto supported = [&lower_support](const Point &point) {
                for (const ExPolygon &polygon : lower_support)
                    if (polygon.contains(point))
                        return true;
                return false;
            };
            // Inspect individual post-step segments, never join densified pieces into a synthetic span.
            for (const LayerSliceIsland &island : roof.islands())
                for (const LayerRegionIsland &region : island.regions_islands()) {
                    if (!region.has_extrusion(LayerRegionIsland::PERIMETERS))
                        continue;
                    std::vector<slic3r_api::ExtrusionEntity> pending;
                    pending.emplace_back(reinterpret_cast<const extrusion_entity_handle *>(
                        &region.extrusion(LayerRegionIsland::PERIMETERS)));
                    while (!pending.empty()) {
                        const slic3r_api::ExtrusionEntity entity = pending.back();
                        pending.pop_back();
                        for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
                            pending.push_back(entity.child(idx));
                        for (uint32_t idx = 0; idx < entity.segment_count(); ++idx) {
                            const c_extrusion_segment segment = entity.segment(idx);
                            const Point start(segment.point_a.x, segment.point_a.y);
                            const Point end(segment.point_b.x, segment.point_b.y);
                            const double length_mm = unscaled((end - start).cast<double>().norm());
                            if (segment.radius != 0.f || length_mm <= 10.)
                                continue;
                            const Point midpoint((start.x() + end.x()) / 2, (start.y() + end.y()) / 2);
                            if (supported(midpoint))
                                continue;
                            const bool start_supported = supported(start);
                            const bool end_supported = supported(end);
                            summary.anchored_roof_spans += start_supported && end_supported;
                            summary.free_roof_spans += start_supported != end_supported;
                            summary.longest_roof_span_mm = std::max(summary.longest_roof_span_mm, length_mm);
                        }
                    }
                }
        }
    }
    return summary;
}

bool has_meaningful_demand(const DemandSummary &summary)
{
    return summary.entry_count > 0 && summary.polygon_count > 0 && summary.area_mm2 > 0.1;
}

} // namespace

TEST_CASE("SupportDemandModifiers adds support enforcers", "[plugins][support-demand]")
{
    // The model part is a vertical cube, so SupportDemandOverhangs creates no
    // demand. A support enforcer volume inside the cube should add island-keyed
    // demand clipped to the cube footprint.
    const Box model_box = {-10., -10., 0., 10., 10., 10.};
    const ModifierBox enforcer = {ModelVolumeType::SUPPORT_ENFORCER, {-6., -6., 0., 6., 6., 10.}};

    const DemandSummary summary = run_support_demand(
        make_model(make_box(model_box), {enforcer}),
        support_demand_config(true, false, 0, 0));
    REQUIRE(has_meaningful_demand(summary));
}

TEST_CASE("SupportDemandModifiers removes support with blockers", "[plugins][support-demand]")
{
    // The enforcer and blocker use the same rectangular prism. The enforcer
    // creates demand first, then the blocker removes exactly that area, leaving
    // no support demand.
    const Box model_box = {-10., -10., 0., 10., 10., 10.};
    const Box modifier_box = {-6., -6., 0., 6., 6., 10.};
    const ModifierBox enforcer = {ModelVolumeType::SUPPORT_ENFORCER, modifier_box};
    const ModifierBox blocker = {ModelVolumeType::SUPPORT_BLOCKER, modifier_box};

    const DemandSummary summary = run_support_demand(
        make_model(make_box(model_box), {enforcer, blocker}),
        support_demand_config(true, false, 0, 0));
    REQUIRE_FALSE(has_meaningful_demand(summary));
}

TEST_CASE("SupportDemandModifiers applies support volumes in model volume order", "[plugins][support-demand]")
{
    // All three modifiers cover the same area. If enforcers were unioned first
    // and blockers applied later, the blocker would erase everything. The
    // expected behavior is sequential: enforcer adds, blocker removes, the
    // second enforcer adds the area again.
    const Box model_box = {-10., -10., 0., 10., 10., 10.};
    const Box modifier_box = {-6., -6., 0., 6., 6., 10.};
    const ModifierBox enforcer = {ModelVolumeType::SUPPORT_ENFORCER, modifier_box};
    const ModifierBox blocker = {ModelVolumeType::SUPPORT_BLOCKER, modifier_box};

    const DemandSummary summary = run_support_demand(
        make_model(make_box(model_box), {enforcer, blocker, enforcer}),
        support_demand_config(true, false, 0, 0));
    REQUIRE(has_meaningful_demand(summary));
}

TEST_CASE("SupportDemandModifiers can partially remove enforced support", "[plugins][support-demand]")
{
    // The blocker covers only the right half of the enforcer. The result should
    // still contain demand, but less than the enforcer-only baseline. This
    // catches accidental clear-all behavior in blocker handling.
    const Box model_box = {-10., -10., 0., 10., 10., 10.};
    const ModifierBox enforcer = {ModelVolumeType::SUPPORT_ENFORCER, {-8., -8., 0., 8., 8., 10.}};
    const ModifierBox right_half_blocker = {ModelVolumeType::SUPPORT_BLOCKER, {0., -8., 0., 8., 8., 10.}};
    const DynamicPrintConfig config = support_demand_config(true, false, 0, 0);

    const DemandSummary baseline = run_support_demand(
        make_model(make_box(model_box), {enforcer}), config);
    const DemandSummary partially_blocked = run_support_demand(
        make_model(make_box(model_box), {enforcer, right_half_blocker}), config);

    REQUIRE(has_meaningful_demand(baseline));
    REQUIRE(has_meaningful_demand(partially_blocked));
    REQUIRE(partially_blocked.area_mm2 < baseline.area_mm2);
}

TEST_CASE("SupportDemandModifiers leaves blocker-only input empty", "[plugins][support-demand]")
{
    // A blocker is subtractive only. With a vertical cube and no prior demand,
    // it must not create an empty support-demand entry or synthetic polygons.
    const Box model_box = {-10., -10., 0., 10., 10., 10.};
    const ModifierBox blocker = {ModelVolumeType::SUPPORT_BLOCKER, {-6., -6., 0., 6., 6., 10.}};

    const DemandSummary summary = run_support_demand(
        make_model(make_box(model_box), {blocker}),
        support_demand_config(true, false, 0, 0));
    REQUIRE_FALSE(has_meaningful_demand(summary));
}

TEST_CASE("SupportDemandModifiers clips enforcers to the layer island", "[plugins][support-demand]")
{
    // The support enforcer is entirely outside the model part footprint. The
    // plugin intersects enforcer slices with the current island, so an outside
    // enforcer should have no effect.
    const Box model_box = {-10., -10., 0., 10., 10., 10.};
    const ModifierBox outside_enforcer = {ModelVolumeType::SUPPORT_ENFORCER, {30., -6., 0., 40., 6., 10.}};

    const DemandSummary summary = run_support_demand(
        make_model(make_box(model_box), {outside_enforcer}),
        support_demand_config(true, false, 0, 0));
    REQUIRE_FALSE(has_meaningful_demand(summary));
}

TEST_CASE("SupportDemandModifiers runs after overhang demand and can reject it", "[plugins][support-demand]")
{
    // The sloped model part creates automatic overhang demand on the +X side.
    // Adding a full-footprint blocker checks the whole STEP_SUPPORT_DEMAND
    // sequence: SupportDemandOverhangs must run first, then SupportDemandModifiers
    // subtracts from the existing demand.
    const DynamicPrintConfig config = support_demand_config(true, true, 0, 0);
    const ModifierBox full_blocker = {ModelVolumeType::SUPPORT_BLOCKER, {-20., -20., 0., 20., 20., 10.}};

    const DemandSummary overhang_only = run_support_demand(
        make_model(make_sloped_cube(5.), {}), config);
    const DemandSummary blocked_overhang = run_support_demand(
        make_model(make_sloped_cube(5.), {full_blocker}), config);

    REQUIRE(has_meaningful_demand(overhang_only));
    REQUIRE_FALSE(has_meaningful_demand(blocked_overhang));
}

TEST_CASE("SupportDemandPainting adds painted support enforcers", "[plugins][support-demand]")
{
    // A vertical cube has no automatic overhang demand. Painting the two bottom
    // triangles as support enforcers should create demand from the projected
    // custom facets alone.
    Model model = make_model(make_box({-10., -10., 0., 10., 10., 10.}), {});
    ModelObject &object = *model.objects().begin();
    paint_support_facets(*object.volumes.front(), EnforcerBlockerType::ENFORCER, {0, 1});

    const DemandSummary summary = run_support_demand(
        std::move(model),
        support_demand_config(true, false, 0, 0));
    REQUIRE(has_meaningful_demand(summary));
}

TEST_CASE("SupportDemandPainting applies painted enforcers before painted blockers", "[plugins][support-demand]")
{
    // Two overlapping model volumes project the same bottom facets. The first
    // paints that area as an enforcer, the second paints it as a blocker. If the
    // plugin applies blockers before enforcers, demand would remain; the
    // expected fixed order is enforcer first, blocker second, leaving no demand.
    Model model;
    ModelObject *object = model.add_object();
    object->name = "support_painting_order_test.stl";
    ModelVolume *enforcer_volume = add_volume(*object, make_box({-10., -10., 0., 10., 10., 10.}), ModelVolumeType::MODEL_PART);
    ModelVolume *blocker_volume = add_volume(*object, make_box({-10., -10., 0., 10., 10., 10.}), ModelVolumeType::MODEL_PART);
    paint_support_facets(*enforcer_volume, EnforcerBlockerType::ENFORCER, {0, 1});
    paint_support_facets(*blocker_volume, EnforcerBlockerType::BLOCKER, {0, 1});
    object->add_instance();

    const DemandSummary summary = run_support_demand(
        std::move(model),
        support_demand_config(true, false, 0, 0));
    REQUIRE_FALSE(has_meaningful_demand(summary));
}

TEST_CASE("SupportDemandPainting can reject automatic overhang demand", "[plugins][support-demand]")
{
    // The sloped cube creates automatic overhang demand on the +X side. Painting
    // the two sloped right-side facets as blockers should subtract from that
    // existing demand, proving that painting runs after SupportDemandOverhangs.
    const DynamicPrintConfig config = support_demand_config(true, true, 0, 0);

    Model baseline_model = make_model(make_sloped_cube(5.), {});
    Model painted_model = make_model(make_sloped_cube(5.), {});
    ModelObject &painted_object = *painted_model.objects().begin();
    paint_support_facets(*painted_object.volumes.front(), EnforcerBlockerType::BLOCKER, {6, 7});

    const DemandSummary baseline = run_support_demand(std::move(baseline_model), config);
    const DemandSummary painted_blocked = run_support_demand(std::move(painted_model), config);

    REQUIRE(has_meaningful_demand(baseline));
    REQUIRE(painted_blocked.area_mm2 < baseline.area_mm2);
}

TEST_CASE("SupportDemandBridgeRemoval removes demand below real bridges", "[plugins][support-demand]")
{
    // The model is a roof between two pillars. Automatic support demand marks
    // the central gap as unsupported, then SupportDemandBridgeRemoval inspects
    // generated perimeters. The long straight perimeter spans are anchored on
    // both pillars, so enabling dont_support_bridges should reduce demand.
    DynamicPrintConfig keep_bridges_supported = support_demand_config(true, true, 0, 0, false);
    // The contract needs intact anchored spans, without DetectOverhang flow/speed densification.
    keep_bridges_supported.set_deserialize_strict("overhangs", "0");
    DynamicPrintConfig remove_bridges = keep_bridges_supported;
    remove_bridges.set_deserialize_strict("dont_support_bridges", "1");

    const DemandSummary baseline = run_support_demand(make_bridge_model(), keep_bridges_supported, true);
    const DemandSummary bridge_removed = run_support_demand(make_bridge_model(), remove_bridges, true);

    REQUIRE(has_meaningful_demand(baseline));
    REQUIRE(baseline.anchored_roof_spans > 0);
    REQUIRE(bridge_removed.anchored_roof_spans == baseline.anchored_roof_spans);
    REQUIRE(baseline.longest_roof_span_mm > 10.);
    REQUIRE(bridge_removed.longest_roof_span_mm > 10.);
    REQUIRE(bridge_removed.area_mm2 < baseline.area_mm2);
}

TEST_CASE("SupportDemandBridgeRemoval keeps cantilever demand", "[plugins][support-demand]")
{
    // This roof has only one supporting pillar. The straight perimeter over the
    // gap has a free end, so it is a cantilever rather than a bridge. Enabling
    // dont_support_bridges must not remove that support demand.
    DynamicPrintConfig keep_bridges_supported = support_demand_config(true, true, 0, 0, false);
    // Keep the free-ended span intact so this case isolates the bridge-removal contract.
    keep_bridges_supported.set_deserialize_strict("overhangs", "0");
    DynamicPrintConfig remove_bridges = keep_bridges_supported;
    remove_bridges.set_deserialize_strict("dont_support_bridges", "1");

    const DemandSummary baseline = run_support_demand(make_cantilever_model(), keep_bridges_supported, true);
    const DemandSummary with_bridge_removal = run_support_demand(make_cantilever_model(), remove_bridges, true);

    REQUIRE(has_meaningful_demand(baseline));
    REQUIRE(baseline.free_roof_spans > 0);
    REQUIRE(with_bridge_removal.free_roof_spans == baseline.free_roof_spans);
    REQUIRE(baseline.anchored_roof_spans == 0);
    REQUIRE(with_bridge_removal.anchored_roof_spans == 0);
    REQUIRE(baseline.longest_roof_span_mm > 10.);
    REQUIRE(with_bridge_removal.longest_roof_span_mm > 10.);
    REQUIRE(with_bridge_removal.area_mm2 == Approx(baseline.area_mm2).epsilon(0.01));
}
