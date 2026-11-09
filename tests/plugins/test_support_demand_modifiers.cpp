#include <catch2/catch.hpp>
#include <boost/filesystem.hpp>

#include "gcode_test_helpers.hpp"
#include "plugin_e2e_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "support_grid_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/internal/LayerIslandAccess.hpp"
#include "libslic3r/Plugins/Support/Common/SupportBridgeStrokes.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config/ConfigOption.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/GCodeReader.hpp"
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
#include <cmath>
#include <initializer_list>
#include <limits>
#include <string>
#include <utility>
#include <vector>

/* Exercise support-demand producers and refinements through real slicing steps,
   plus the shared bridge-stroke geometry on owned native trees. Scene fixtures
   compare demand before support generation and the areas the support family
   publishes. Direct helper cases distinguish local/inherited printed widths and
   keep independent or out-of-reach demand when cleaning thin subtraction residue.

       run_support_demand() -> slicing/perimeter steps -> demand measurements
       run_scene() -> SupportGrid::prefix() -> demand -> SupportGrid::family()
       run_with_thin_strip() -> add independent demand -> STEP_POST_INFILL
       direct geometry cases -> bridge strokes -> bounded leftover cleanup
       linear export matrix -> STL round trip -> slicing -> CLI export -> support moves

   Tests own models, prints, trees and scratch storage. Borrowed SDK views stay
   inside those lifetimes; temporary Clipper results are released before storage.
*/

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

// A scene is built from boxes merged into one mesh. first_layer_min_y is the lowest Y of the pillars (the footprint of the first layer);
// X of the left pillar is 0, so a point of the scene is placed in the print from the extents of the first layer.
struct Scene
{
    TriangleMesh mesh;
    double first_layer_min_y = 0.;
};

Scene make_scene(const std::vector<Box> &pillars, const Box &span)
{
    Scene scene;
    scene.first_layer_min_y = pillars.front().min_y;
    bool first = true;
    for (const Box &pillar : pillars) {
        scene.first_layer_min_y = std::min(scene.first_layer_min_y, pillar.min_y);
        if (first)
            scene.mesh = make_box(pillar);
        else
            scene.mesh.merge(make_box(pillar));
        first = false;
    }
    scene.mesh.merge(make_box(span));
    return scene;
}

// Two pillars 10 mm high and 30 mm apart, joined at the top by a span 2.5 mm thick from Z 9.5. The span is span_mm wide; the
// pillars are as deep as the span, but at least 10 mm. The span is a perimeter-only bridge when it is narrow, and carries a bridged
// infill area when it is wide.
Scene scene_pillars_and_span(const double span_mm)
{
    const double depth = std::max(10., span_mm);
    const double y0 = (depth - span_mm) / 2.;
    return make_scene({{0., 0., 0., 10., depth, 10.}, {40., 0., 0., 50., depth, 10.}}, {0., y0, 9.5, 50., y0 + span_mm, 12.});
}

// Two pillars 10 mm deep, 10 mm high and 30 mm apart, and a slab 20 mm wide that spans the gap and overlaps each pillar by 0.5 mm in X.
// Only the lines of the slab along X between the pillars (Y from 0 to 10) are anchored at both ends. The slab ends at X = 9.5 and 40.5
// in the strips beside the pillars (Y from -5 to 0 and from 10 to 15): the lines of those strips have free ends.
Scene scene_shallow_pillars()
{
    return make_scene({{0., 0., 0., 10., 10., 10.}, {40., 0., 0., 50., 10., 10.}}, {9.5, -5., 9.5, 40.5, 15., 12.});
}

// Two bridges in one island: three pillars, the middle one as wide as the slab (Y from -5 to 15) and the outer ones 10 mm deep.
// The slab spans the two gaps; the middle pillar separates their bottom surfaces. In each gap the lines along X for Y from 0 to 10 are
// anchored at both ends (an outer pillar and the middle pillar); the lines of the strips beside the outer pillars have a free end.
Scene scene_two_bridges()
{
    return make_scene({{0., 0., 0., 10., 10., 10.}, {40., -5., 0., 50., 15., 10.}, {80., 0., 0., 90., 10., 10.}},
                      {9.5, -5., 9.5, 80.5, 15., 12.});
}

// One pillar and a slab that leaves it: the slab is anchored on one side only.
Scene scene_cantilever()
{
    return make_scene({{0., 0., 0., 10., 10., 10.}}, {0., 0., 9.5, 30., 10., 12.});
}

struct SpanSupport
{
    // Support demand after the infill step, summed over the object layers.
    double demand_mm2 = 0.;
    // Area of the support islands published by the support family, summed over the support layers.
    double published_mm2 = 0.;
    // The union of the demand of every island after the infill step, in print coordinates.
    ExPolygons demand;
    // The point of the print that has the scene coordinates (0, 0).
    Point origin;

    Point at(const double x_mm, const double y_mm) const { return origin + Point(scale_i(x_mm), scale_i(y_mm)); }

    // A rectangle given in scene coordinates.
    ExPolygon rectangle(const double x0, const double y0, const double x1, const double y1) const
    {
        return ExPolygon(Points{at(x0, y0), at(x1, y0), at(x1, y1), at(x0, y1)});
    }

    // The area of the demand inside a rectangle given in scene coordinates.
    double demand_area_in(const double x0, const double y0, const double x1, const double y1) const
    {
        return unscaled(unscaled(area(intersection_ex(demand, rectangle(x0, y0, x1, y1)))));
    }
};

// Run through post-infill demand refinement, then support generation. lines_along_x selects fill angle 0 without crossing layers;
// otherwise it keeps the default 45 degrees. An optional internal perimeter width leaves the external width unchanged.
SpanSupport run_scene(const Scene &scene, bool dont_support_bridges, bool lines_along_x = false, const char *perimeter_width = nullptr);

SpanSupport run_scene(const Scene &scene, const bool dont_support_bridges, const bool lines_along_x, const char *perimeter_width)
{
    namespace SG = Slic3r::Test::SupportGrid;
    DynamicPrintConfig config = SG::config("grid", false, 3);
    config.set_deserialize_strict({{"dont_support_bridges", dont_support_bridges ? "1" : "0"}});
    if (perimeter_width != nullptr)
        config.set_deserialize_strict("perimeter_extrusion_width", perimeter_width);
    if (lines_along_x)
        config.set_deserialize_strict({{"fill_angle", "0"}, {"fill_angle_cross", "0"}});
    Print print;
    Model model;
    Test::init_print({scene.mesh}, print, model, config);
    SG::prefix(print);

    SpanSupport result;
    const PrintObject &object = print.object(0);
    const BoundingBox first_layer = get_extents(object.layers().front().lslices());
    result.origin = Point(first_layer.min.x(), first_layer.min.y() - scale_i(scene.first_layer_min_y));
    result.demand_mm2 = summarize_support_demand(object).area_mm2;
    ExPolygons demand;
    for (const Layer &layer : object.layers())
        for (const LayerSliceIsland &island : layer.islands())
            append(demand, island.support_demand());
    result.demand = union_ex(demand);
    SG::family(print);
    for (const Layer &layer : object.auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            result.published_mm2 += unscaled(unscaled(area(island.get_slice())));
    return result;
}

struct ThinStrip
{
    // Area of the strip added to the demand of the bridge layer before STEP_POST_INFILL (mm2).
    double added_mm2 = 0.;
    // Area of the strip that is still demand after STEP_POST_INFILL (mm2).
    double kept_mm2 = 0.;
    // Width of a stroke of the bridge lines, width of the strip and distance from the strip to the strokes (scaled).
    double line_width = 0.;
    double strip_width = 0.;
    double gap = 0.;
};

// Add the lines of the bridge infill of an extrusion tree to the outer edge (largest Y, scaled) of the strokes of the long lines along X, with
// the widest printed width (scaled) of those lines. The stroke of a line has the half width 0.5 * width + 0.001 mm.
void outer_edge_of_bridge_strokes(const ExtrusionEntity &entity, const ExtrusionAttributes *attributes, double &edge_y, double &widest)
{
    if (const ExtrusionAttributes *local = entity.get_property<ExtrusionAttributes>())
        attributes = local;
    if (entity.has_polyline() && attributes != nullptr && attributes->role == ExtrusionRole::BridgeInfill)
        for (const ArcPolyline &line : entity.as_polylines())
            for (size_t index = 1; index < line.size(); ++index) {
                const Point from = line.get_point(index - 1), to = line.get_point(index);
                if (std::abs(to.y() - from.y()) > SCALED_EPSILON || std::abs(to.x() - from.x()) < scale_i(10.))
                    continue;
                edge_y = std::max(edge_y, double(from.y()) + 0.5 * scale_d(attributes->width) + scale_d(0.001));
                widest = std::max(widest, scale_d(attributes->width));
            }
    for (const ExtrusionEntityUPtr &child : entity.children())
        outer_edge_of_bridge_strokes(*child, attributes, edge_y, widest);
}

// Slice the scene with the bridge lines along X, add to the demand of the bridge layer a strip along X, 20 mm long, whose width and distance to the
// outer edge of the strokes of the bridge lines are given in stroke widths, then run STEP_POST_INFILL, where the plugin removes the bridge strokes
// from the demand. The strip is not part of the demand the bridge strokes were computed against, as if another source had asked for it.
ThinStrip run_with_thin_strip(const Scene &scene, const double strip_width_in_lines, const double gap_in_lines)
{
    namespace SG = Slic3r::Test::SupportGrid;
    DynamicPrintConfig config = SG::config("grid", false, 3);
    config.set_deserialize_strict({{"dont_support_bridges", "1"}, {"fill_angle", "0"}, {"fill_angle_cross", "0"}});
    Print print;
    Model model;
    Test::init_print({scene.mesh}, print, model, config);

    // The steps of SG::prefix up to the infill generation; the post-infill step runs after the strip is added.
    Orchestrator &host = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::prepare(print);
    Steps::StepLayerHeightGeneration::run_step(host, print);
    Steps::StepSlicing::run_step(host, print);
    Steps::StepPostSlicing::prepare(print);
    Steps::StepPostSlicing::run_step(host, print);
    Steps::StepSupportDemand::prepare(print);
    Steps::StepSupportDemand::run_step(host, print);
    Steps::StepPrepareForPeriemters::prepare(print);
    Steps::StepPrepareForPeriemters::run_step(host, print);
    Steps::StepGeneratePerimeter::prepare(print);
    Steps::StepGeneratePerimeter::run_step(host, print);
    Steps::StepPostPerimeterGeneration::prepare(print);
    Steps::StepPostPerimeterGeneration::run_step(host, print);
    Steps::StepSurfaceGeneration::prepare(print);
    Steps::StepSurfaceGeneration::run_step(host, print);
    Steps::StepPrepareInfill::prepare(print);
    Steps::StepPrepareInfill::run_step(host, print);
    Steps::StepGroupInfillRegions::prepare(print);
    Steps::StepGroupInfillRegions::run_step(host, print);
    Steps::StepGenerateInfill::prepare(print);
    Steps::StepGenerateInfill::run_step(host, print);

    // The island with the most demand is the one of the bridge layer.
    LayerSliceIsland *bridge_island = nullptr;
    double largest_demand = 0.;
    for (Layer &layer : print.object(0).layers())
        for (LayerSliceIsland &island : layer.islands())
            if (area(island.support_demand()) > largest_demand) {
                largest_demand = area(island.support_demand());
                bridge_island = &island;
            }
    REQUIRE(bridge_island != nullptr);

    // The outer edge of the strokes: the outermost long bridge line of the infill bucket of the island, with its stroke half width.
    double edge_y = -std::numeric_limits<double>::max(), widest = 0.;
    for (LayerRegionIsland &region_island : bridge_island->regions_islands())
        if (region_island.has_extrusion(LayerRegionIsland::INFILLS))
            outer_edge_of_bridge_strokes(region_island.extrusion(LayerRegionIsland::INFILLS), nullptr, edge_y, widest);
    REQUIRE(widest > 0.);

    // A strip along X, centred on the island, at the requested distance beyond the strokes.
    ThinStrip result;
    result.line_width = widest + 2. * scale_d(0.001);
    result.strip_width = strip_width_in_lines * result.line_width;
    result.gap = gap_in_lines * result.line_width;
    const BoundingBox infill = get_extents(bridge_island->infill_areas());
    const coord_t x_centre = (infill.min.x() + infill.max.x()) / 2;
    const coord_t y_start = coord_t(std::llround(edge_y + result.gap));
    const coord_t y_end = coord_t(std::llround(edge_y + result.gap + result.strip_width));
    const ExPolygon strip(Points{Point(x_centre - scale_i(10.), y_start), Point(x_centre + scale_i(10.), y_start),
                                 Point(x_centre + scale_i(10.), y_end), Point(x_centre - scale_i(10.), y_end)});
    // Only the part of the strip that is inside the island and not already demand is a new, independent demand.
    const ExPolygons added = diff_ex(intersection_ex(strip, ExPolygons{bridge_island->get_slice()}), bridge_island->support_demand());
    ExPolygons &demand = ApiInternal::LayerIslandAccess::support_demand_mutable(*bridge_island);
    append(demand, added);
    demand = union_ex(demand);

    Steps::StepPostInfillGeneration::prepare(print);
    Steps::StepPostInfillGeneration::run_step(host, print);

    result.added_mm2 = unscaled(unscaled(area(added)));
    result.kept_mm2 = unscaled(unscaled(area(intersection_ex(bridge_island->support_demand(), added))));
    return result;
}

// Deactivate the bridged infill removal for the scope of a test; the destructor restores its activation.
struct ScopedBridgeInfillRemovalOff {
    ScopedBridgeInfillRemovalOff() { REQUIRE(Orchestrator::instance().set_plugin_active("support.demand.bridge_infill_removal", false)); }
    ~ScopedBridgeInfillRemovalOff() { Orchestrator::instance().set_plugin_active("support.demand.bridge_infill_removal", true); }
};

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
    // dont_support_bridges must not remove that support demand. This case runs
    // the perimeter steps; "SupportDemandBridgeInfillRemoval keeps a cantilever
    // demand" runs the whole pipeline through STEP_POST_INFILL, where the
    // bridged infill is removed too, and checks the demand there.
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

TEST_CASE("SupportDemandBridgeRemoval removes the demand under a perimeter-only span", "[plugins][support-demand]")
{
    // A span 2 mm wide has no infill: its perimeters cross the gap and are anchored on both pillars.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const SpanSupport kept = run_scene(scene_pillars_and_span(2.), false);
    const SpanSupport removed = run_scene(scene_pillars_and_span(2.), true);

    REQUIRE(kept.demand_mm2 > 10.);
    REQUIRE(kept.published_mm2 > 0.);
    CHECK(removed.demand_mm2 == Approx(0.).margin(0.01));
    CHECK(removed.published_mm2 == Approx(0.).margin(0.01));
}

TEST_CASE("SupportDemandBridgeRemoval cleans narrow spans with distinct internal widths", "[plugins][support-demand][support-bridge-width]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const char *width : {"125%", "150%"}) {
        INFO("internal perimeter width="<<width<<", external width stays 105%");
        const SpanSupport kept = run_scene(scene_pillars_and_span(2.), false, false, width);
        const SpanSupport removed = run_scene(scene_pillars_and_span(2.), true, false, width);
        REQUIRE(kept.demand_mm2 > 10.);
        REQUIRE(kept.published_mm2 > 0.);
        CHECK(removed.demand_mm2 == Approx(0.).margin(0.01));
        CHECK(removed.published_mm2 == Approx(0.).margin(0.01));
    }
}

TEST_CASE("Perimeter bridge removal exports a linear narrow-span STL matrix", "[plugins][support-demand][plugin-e2e][support-bridge-e2e]")
{
    namespace E2E = Slic3r::Test::PluginE2E;
    namespace GC = Slic3r::Test::GCode;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    DynamicPrintConfig baseline = Slic3r::Test::SupportGrid::config("grid", false, 3, true);
    baseline.set_deserialize_strict({{"dont_support_bridges", "1"},
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"},
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"},
        {"use_relative_e_distances", "0"}, {"filament_diameter", "1.75,1.75"}});
    const std::vector<E2E::Domain> domains{
        {{"dont_support_bridges"}, {{"support retained", {{"dont_support_bridges", "0"}}}}},
        {{"support_material"}, {{"support disabled", {{"support_material", "0"}}}}},
        {{"perimeter_extrusion_width"}, {{"internal 125%", {{"perimeter_extrusion_width", "125%"}}},
            {"internal 150%", {{"perimeter_extrusion_width", "150%"}}}}}};
    const std::vector<E2E::Case> cases = E2E::linear_cases(host,
        {"support.demand.bridge_removal", "support.demand.bridge_infill_removal"}, baseline, domains);
    REQUIRE(cases.size() == 5);
    TriangleMesh mesh;
    boost::filesystem::path stl = GC::temporary_gcode_path();
    stl.replace_extension(".stl");
    try {
        REQUIRE(scene_pillars_and_span(2.).mesh.write_binary(stl.string().c_str()));
        REQUIRE(mesh.ReadSTLFile(stl.string().c_str()));
        boost::filesystem::remove(stl);
    } catch (...) {
        boost::system::error_code ignored;
        boost::filesystem::remove(stl, ignored);
        throw;
    }
    for (const E2E::Case &test : cases) {
        INFO(test.name);
        const DynamicPrintConfig config = E2E::apply_case(baseline, test);
        Model model;
        Print print;
        Slic3r::Test::init_print({mesh}, print, model, config);
        const boost::filesystem::path path = GC::temporary_gcode_path();
        std::string code;
        try {
            print.process();
            code = GC::read_text_file(host.export_gcode_cli(print, path.string()));
            GC::remove_output_pair(path);
        } catch (...) {
            GC::remove_output_pair(path);
            throw;
        }
        size_t moves = 0, support_moves = 0;
        std::string type;
        GCodeReader reader;
        reader.apply_config(config);
        reader.parse_buffer(code, [&](GCodeReader &state, const GCodeReader::GCodeLine &line) {
            if (line.comment().rfind("TYPE:", 0) == 0)
                type = line.comment().substr(5);
            if (line.extruding(state) && line.dist_XY(state) > 0.) {
                ++moves;
                if (type.rfind("Support material", 0) == 0) ++support_moves;
            }
        });
        CHECK(moves > 0);
        if (test.name == "support retained") CHECK(support_moves > 0);
        else CHECK(support_moves == 0);
    }
}

TEST_CASE("SupportDemandBridgeInfillRemoval removes the demand under bridge lines from pillar to pillar", "[plugins][support-demand]")
{
    // The span is 20 mm wide and the pillars as wide as the span. With the fill along X (angle 0, no cross layers) the bridge lines run
    // along X from one pillar to the other: each one has its middle over the void and its two ends over the pillars, so its stroke
    // removes the demand. The lines are as close as their width (a 100% bridge), so the strokes cover the infill without gaps; the two
    // long perimeters at the free sides are bridges for support.demand.bridge_removal. The demand and the support under the span
    // vanish. The strokes of the perimeters (removed earlier) and of the infill do not meet exactly: the plugin removes the leftovers
    // thinner than a line along its strokes, so almost nothing remains (0.5 mm2 of 592 left for rounding), and no sliver is widened
    // into a support wall (the published area stays under 2% of the baseline).
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const SpanSupport kept = run_scene(scene_pillars_and_span(20.), false, true);
    const SpanSupport removed = run_scene(scene_pillars_and_span(20.), true, true);

    REQUIRE(kept.demand_mm2 > 100.);
    REQUIRE(kept.published_mm2 > 0.);
    CHECK(removed.demand_mm2 == Approx(0.).margin(0.5));
    CHECK(removed.published_mm2 < 0.02 * kept.published_mm2);
}

TEST_CASE("SupportDemandBridgeInfillRemoval keeps the demand under diagonal bridge lines", "[plugins][support-demand]")
{
    // At the default fill angle (45 degrees) a bridge line of the 20 mm wide span cannot join the two pillars, which are 30 mm apart:
    // it runs from a pillar to a free side, so it is anchored at one end only and is not a bridge. The plugin removes nothing that
    // the perimeters did not remove already; the comparison keeps 2% for the rounding of the strokes.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const SpanSupport with_plugin = run_scene(scene_pillars_and_span(20.), true);
    SpanSupport without_plugin;
    {
        ScopedBridgeInfillRemovalOff plugin_off;
        without_plugin = run_scene(scene_pillars_and_span(20.), true);
    }

    REQUIRE(without_plugin.demand_mm2 > 100.);
    CHECK(with_plugin.demand_mm2 >= 0.98 * without_plugin.demand_mm2);
}

TEST_CASE("SupportDemandBridgeInfillRemoval keeps a cantilever demand", "[plugins][support-demand]")
{
    // The slab is anchored on one side only: its lines along X start on the pillar and end in the air, so none is a bridge, and
    // the perimeters are not either. The run goes through STEP_POST_INFILL, so every demand refinement has run. The fraction of 99%
    // leaves room for slivers.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const SpanSupport kept = run_scene(scene_cantilever(), false, true);
    const SpanSupport removed = run_scene(scene_cantilever(), true, true);

    REQUIRE(kept.demand_mm2 > 10.);
    CHECK(removed.demand_mm2 >= 0.99 * kept.demand_mm2);
}

TEST_CASE("SupportDemandBridgeInfillRemoval keeps the strips with free ends beside the pillars", "[plugins][support-demand]")
{
    // Along X, the lines of the slab between the pillars (Y from 0 to 10) are anchored at both ends: the middle of the bridge
    // (X from 14 to 36, Y from 3 to 7, far from every border) loses its demand, with 5% left for slivers. The strips beside the
    // pillars (Y from -5 to 0 and from 10 to 15) have lines with free ends: the strip from Y -4 to -1 keeps its demand, with 10% left
    // for the rounding of the borders.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const SpanSupport kept = run_scene(scene_shallow_pillars(), false, true);
    const SpanSupport removed = run_scene(scene_shallow_pillars(), true, true);

    const double middle_area = 22. * 4.;
    INFO("demand in the middle of the bridge: " << kept.demand_area_in(14., 3., 36., 7.) << " mm2 kept, "
         << removed.demand_area_in(14., 3., 36., 7.) << " mm2 with the option");
    REQUIRE(kept.demand_area_in(14., 3., 36., 7.) > 0.5 * middle_area);
    CHECK(removed.demand_area_in(14., 3., 36., 7.) < 0.05 * middle_area);

    const double strip_area = 22. * 3.;
    INFO("demand in the strip: " << kept.demand_area_in(14., -4., 36., -1.) << " mm2 kept, "
         << removed.demand_area_in(14., -4., 36., -1.) << " mm2 with the option");
    REQUIRE(kept.demand_area_in(14., -4., 36., -1.) > 0.5 * strip_area);
    CHECK(removed.demand_area_in(14., -4., 36., -1.) > 0.9 * kept.demand_area_in(14., -4., 36., -1.));
}

TEST_CASE("SupportDemandBridgeInfillRemoval removes the demand under every bridge of an island", "[plugins][support-demand]")
{
    // The middle pillar is as wide as the slab, so the two gaps are two bottom surfaces of the same island. In each one the lines along X
    // for Y from 0 to 10 are anchored at both ends (an outer pillar and the middle pillar): the middle of both bridges
    // (X from 14 to 36 and from 54 to 76, Y from 3 to 7) loses its demand, with 5% left for slivers.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const SpanSupport kept = run_scene(scene_two_bridges(), false, true);
    const SpanSupport removed = run_scene(scene_two_bridges(), true, true);

    const double middle_area = 22. * 4.;
    for (const double x : {14., 54.}) {
        INFO("bridge from X " << x << ": " << kept.demand_area_in(x, 3., x + 22., 7.) << " mm2 kept, "
             << removed.demand_area_in(x, 3., x + 22., 7.) << " mm2 with the option");
        REQUIRE(kept.demand_area_in(x, 3., x + 22., 7.) > 0.5 * middle_area);
        CHECK(removed.demand_area_in(x, 3., x + 22., 7.) < 0.05 * middle_area);
    }
}

namespace {

// The area (mm2) of the strokes of the polyline through the given points (mm, straight segments), over two pillars (X from 0 to 10 and from 40
// to 50, Y from 0 to 10) taken as the lower support, with the default BridgeLineFlow. With `merged` the polyline goes through
// append_polyline_strokes (collinear segments merged), otherwise through append_entity_strokes (each segment as printed).
double polyline_stroke_area(const std::vector<std::pair<double, double>> &points_mm, const bool merged)
{
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    slic3r_api::ClipperContext clipper(handle);

    slic3r_api::StoredPolygonCollection pillars(handle);
    for (const double x0 : {0., 40.}) {
        slic3r_api::StoredPolygon pillar(handle);
        for (const std::pair<double, double> corner : {std::pair<double, double>{0., 0.}, {10., 0.}, {10., 10.}, {0., 10.}})
            pillar.push_back(c_point{scale_i(x0 + corner.first), scale_i(corner.second)});
        pillars.push_back(pillar);
    }
    const slic3r_api::StoredExPolygonCollection lower_support = slic3r_api::clipper_union(clipper(pillars)).to_expolygon_collection();

    std::vector<c_point> points;
    for (const std::pair<double, double> &point : points_mm)
        points.push_back(c_point{scale_i(point.first), scale_i(point.second)});
    slic3r_api::StoredExtrusionEntity line(handle, points);
    const slic3r_api::ExtrusionEntity view(line.handle());

    const slic3r_api::Support::Common::BridgeLineFlow flow;
    slic3r_api::ClipperOperand strokes = clipper.empty();
    if (merged)
        slic3r_api::Support::Common::append_polyline_strokes(handle, lower_support, flow, clipper, view, strokes);
    else
        slic3r_api::Support::Common::append_entity_strokes(handle, lower_support, flow, clipper, view, strokes);
    if (strokes.empty())
        return 0.;
    double total = 0.;
    for (const slic3r_api::ExPolygon stroke : slic3r_api::clipper_union(strokes).to_expolygon_collection())
        total += unscaled(unscaled(stroke.area()));
    return total;
}

} // namespace

TEST_CASE("Perimeter bridge strokes resolve local and inherited widths without counting unanchored lines", "[plugins][support-demand][support-bridge-width]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    namespace Common = slic3r_api::Support::Common;
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    slic3r_api::ClipperContext clipper(handle);
    const ExPolygons pillars{
        ExPolygon(Points{Point(0,0),Point(scale_i(10.),0),Point(scale_i(10.),scale_i(10.)),Point(0,scale_i(10.))}),
        ExPolygon(Points{Point(scale_i(40.),0),Point(scale_i(50.),0),Point(scale_i(50.),scale_i(10.)),Point(scale_i(40.),scale_i(10.))})};
    const slic3r_api::ExPolygonCollection lower_support(reinterpret_cast<const expolygon_collection_handle *>(&pillars));
    ExtrusionEntity root(ExtrusionEntity::Children{},true,true,false);
    root.add_property(ExtrusionAttributes(ExtrusionRole::Perimeter,ExtrusionFlow(.08,.8f,.2f)));
    ExtrusionEntity branch(ExtrusionEntity::Children{},true,true,false);
    ExtrusionEntity inherited(true,ArcPolyline(Points{Point(scale_i(5.),scale_i(7.)),Point(scale_i(45.),scale_i(7.))}));
    branch.append_child(std::move(inherited));
    ExtrusionEntity local(true,ArcPolyline(Points{Point(scale_i(5.),scale_i(3.)),Point(scale_i(45.),scale_i(3.))}));
    local.add_property(ExtrusionAttributes(ExtrusionRole::Perimeter,ExtrusionFlow(.04,.2f,.2f)));
    branch.append_child(std::move(local));
    ExtrusionEntity fallback(true,ArcPolyline(Points{Point(scale_i(5.),scale_i(5.)),Point(scale_i(45.),scale_i(5.))}));
    fallback.add_property(ExtrusionAttributes(ExtrusionRole::Perimeter,ExtrusionFlow(.04,0.f,.2f)));
    branch.append_child(std::move(fallback));
    ExtrusionEntity cantilever(true,ArcPolyline(Points{Point(scale_i(5.),scale_i(9.)),Point(scale_i(25.),scale_i(9.))}));
    cantilever.add_property(ExtrusionAttributes(ExtrusionRole::Perimeter,ExtrusionFlow(.2,2.f,.2f)));
    branch.append_child(std::move(cantilever));
    root.append_child(std::move(branch));

    slic3r_api::ClipperOperand strokes = clipper.empty();
    const Common::BridgeLineFlow flow;
    const double widest = Common::append_entity_strokes(handle,lower_support,flow,clipper,
        slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(&root)),strokes);
    CHECK(unscaled(widest) == Approx(.802).margin(.000002));
    const slic3r_api::StoredExPolygonCollection shapes = slic3r_api::clipper_union(strokes).to_expolygon_collection();
    for (const std::pair<Point,bool> probe : {
        std::pair<Point,bool>{Point(scale_i(25.),scale_i(7.3)),true},
        {Point(scale_i(25.),scale_i(3.08)),true}, {Point(scale_i(25.),scale_i(3.15)),false},
        {Point(scale_i(25.),scale_i(5.19)),true}, {Point(scale_i(25.),scale_i(5.21)),false},
        {Point(scale_i(20.),scale_i(9.)),false}}) {
        INFO("probe="<<unscaled(probe.first.x())<<','<<unscaled(probe.first.y()));
        bool covered = false;
        for (const slic3r_api::ExPolygon shape : shapes)
            covered = covered || shape.contains(c_point{probe.first.x(),probe.first.y()});
        CHECK(covered == probe.second);
    }
}

TEST_CASE("Bridge leftover cleanup requires thinness contact and bounded reach", "[plugins][support-demand][support-bridge-width]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    namespace Common = slic3r_api::Support::Common;
    const auto rectangle = [](double x0,double y0,double x1,double y1) {
        return ExPolygon(Points{Point(scale_i(x0),scale_i(y0)),Point(scale_i(x1),scale_i(y0)),
            Point(scale_i(x1),scale_i(y1)),Point(scale_i(x0),scale_i(y1))});
    };
    for (const std::string kind : {"touching thin","separate thin","wide demand","out of reach"}) {
        INFO(kind);
        PluginStorage storage;
        storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
        slic3r_api::ClipperContext clipper(handle);
        const ExPolygons stroke_shapes{rectangle(0.,0.,10.,1.)};
        const ExPolygon requested = kind == "touching thin" ? rectangle(2.,1.,8.,1.3) :
            kind == "separate thin" ? rectangle(2.,1.5,8.,1.8) :
            kind == "wide demand" ? rectangle(2.,1.,8.,3.) : rectangle(2.,1.,2.3,3.);
        const ExPolygons demand{requested};
        const slic3r_api::ExPolygonCollection stroke_view(reinterpret_cast<const expolygon_collection_handle *>(&stroke_shapes));
        const slic3r_api::ExPolygonCollection demand_view(reinterpret_cast<const expolygon_collection_handle *>(&demand));
        const slic3r_api::ClipperOperand strokes = clipper(stroke_view);
        slic3r_api::ClipperOperand remaining = clipper(demand_view);
        Common::remove_leftovers_along_strokes(clipper,strokes,scale_d(1.),remaining);
        const slic3r_api::StoredExPolygonCollection result = remaining.to_expolygon_collection();
        const ExPolygons &native = *reinterpret_cast<const ExPolygons *>(result.handle());
        if (kind == "touching thin") CHECK(native.empty());
        else {
            CHECK(diff_ex(demand,native).empty());
            CHECK(diff_ex(native,demand).empty());
        }
    }
}

TEST_CASE("Bridge strokes merge the collinear segments of a line", "[plugins][support-demand]")
{
    // The line runs along X from X 5 (over the left pillar) to X 45 (over the right one). Its bridge piece is the part between the
    // pillars, whose extended ends are over them. Split into three collinear segments, each one alone is not a bridge (its extended end
    // falls in the void), but the merged line is, and its stroke equals the one of the line drawn as one segment.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const double whole = polyline_stroke_area({{5., 5.}, {45., 5.}}, false);
    REQUIRE(whole > 1.);
    CHECK(polyline_stroke_area({{5., 5.}, {15., 5.}, {30., 5.}, {45., 5.}}, false) == Approx(0.).margin(1e-9));
    CHECK(polyline_stroke_area({{5., 5.}, {15., 5.}, {30., 5.}, {45., 5.}}, true) == Approx(whole));
}

TEST_CASE("Bridge strokes do not merge a line that bends", "[plugins][support-demand]")
{
    // The second segment turns by one degree: the joint is 0.17 mm off the chord, far above SCALED_EPSILON, so the segments stay apart and
    // neither is a bridge. A back-and-forth does not merge either: only its first segment, the whole bridge, gives a stroke.
    // Arcs are not covered here: an arc segment breaks the chain by its radius, a branch that needs a host extrusion tree with arcs.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const double bend = 5. + 20. * std::tan(M_PI / 180.);
    CHECK(polyline_stroke_area({{5., 5.}, {25., 5.}, {45., bend}}, true) == Approx(0.).margin(1e-9));

    const double whole = polyline_stroke_area({{5., 5.}, {45., 5.}}, true);
    CHECK(polyline_stroke_area({{5., 5.}, {45., 5.}, {25., 5.}}, true) == Approx(whole));
}

TEST_CASE("SupportDemandBridgeInfillRemoval keeps an independent thin demand near the strokes", "[plugins][support-demand]")
{
    // The strip is half a line wide, so the opening by half a line removes it, and its edge is half a line beyond the outer edge of the
    // strokes (computed from the outermost long bridge line of the island): it lies inside the one-line neighbourhood of the strokes, so
    // only the contact rule keeps it, being farther than the contact tolerance. 90% of it stays for the rounding of the borders.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const double contact_tolerance = 4. * double(SCALED_EPSILON);
    const ThinStrip strip = run_with_thin_strip(scene_pillars_and_span(20.), 0.5, 0.5);

    REQUIRE(strip.strip_width < strip.line_width);
    REQUIRE(strip.gap > contact_tolerance);
    REQUIRE(strip.gap < strip.line_width);
    REQUIRE(strip.added_mm2 > 1.);
    CHECK(strip.kept_mm2 >= 0.9 * strip.added_mm2);
}

TEST_CASE("SupportDemandBridgeInfillRemoval removes a thin demand that touches the strokes", "[plugins][support-demand]")
{
    // The same strip right against the strokes (distance 0, inside the contact tolerance) is a leftover of the subtraction: it goes,
    // with 10% left for the rounding of the borders.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const double contact_tolerance = 4. * double(SCALED_EPSILON);
    const ThinStrip strip = run_with_thin_strip(scene_pillars_and_span(20.), 0.5, 0.);

    REQUIRE(strip.strip_width < strip.line_width);
    REQUIRE(strip.gap <= contact_tolerance);
    REQUIRE(strip.added_mm2 > 1.);
    CHECK(strip.kept_mm2 <= 0.1 * strip.added_mm2);
}

TEST_CASE("SupportDemandBridgeInfillRemoval leaves the demand unchanged without dont_support_bridges", "[plugins][support-demand]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const SpanSupport with_plugin = run_scene(scene_pillars_and_span(20.), false, true);
    SpanSupport without_plugin;
    {
        ScopedBridgeInfillRemovalOff plugin_off;
        without_plugin = run_scene(scene_pillars_and_span(20.), false, true);
    }

    REQUIRE(without_plugin.demand_mm2 > 100.);
    CHECK(with_plugin.demand_mm2 == Approx(without_plugin.demand_mm2));
    CHECK(with_plugin.published_mm2 == Approx(without_plugin.published_mm2));
}
