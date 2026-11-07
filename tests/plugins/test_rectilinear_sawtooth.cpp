#include <catch2/catch.hpp>

#include <cmath>
#include <initializer_list>
#include <memory>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/slic3r_utils.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Fill/FillRectilinear.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Plugins/Infill/Rectilinear/RectilinearFills.hpp"
#include "libslic3r/Plugins/Infill/Rectilinear/RectilinearSawtooth.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/Thread.hpp"

/*
Sawtooth rectilinear plugin tests
=================================

The direct tests compare the storage-backed plugin tree with the native
FillRectilinearSawtooth tree, including each path's Z profile and flow. The
remaining tests cover the public random utility and the normal STEP_INFILL
activation path.
*/

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

namespace {

namespace rect_work = slic3r_api::PluginCommon::polygon_work;
namespace rect_fills = slic3r_api::Infill::RectilinearPatternsPlugin::fills;
namespace sawtooth = slic3r_api::Infill::RectilinearPatternsPlugin::sawtooth;

raw_infill_pattern_params make_raw_params(const ExPolygon &surface,
                                          const Flow &flow,
                                          float density,
                                          double angle)
{
    raw_infill_pattern_params params = {};
    params.surface_type = RAW_SURFACE_TYPE_POS_INTERNAL | RAW_SURFACE_TYPE_DENS_SPARSE;
    params.extrusion_role = RAW_EXTRUSION_ROLE_INTERNAL_INFILL;
    params.flow.width = flow.scaled_width();
    params.flow.spacing = flow.scaled_spacing();
    params.flow.height = flow.scaled_height();
    params.flow.nozzle_diameter = scale_i(flow.nozzle_diameter());
    params.flow.spacing_ratio = flow.spacing_ratio();
    params.flow.mm3_per_mm = flow.mm3_per_mm();
    params.density = density;
    params.flow_mult = 0.85f;
    params.connection = C_INFILL_CONNECTION_CONNECTED;
    params.infill_flags = C_INFILL_FLAG_DONT_ADJUST | C_INFILL_FLAG_CAN_ANGLE_CROSS;
    params.spacing = flow.spacing();
    params.angle = angle;
    params.bridge_angle = -1.;
    params.layer_id = 3;
    params.z = 0.2;
    params.fill_resolution = scale_i(0.0125);
    params.anchor_length = 1000.f;
    params.anchor_length_max = 1000.f;

    const BoundingBox bbox = get_extents(surface);
    params.object_bounding_box.min = { bbox.min.x(), bbox.min.y() };
    params.object_bounding_box.max = { bbox.max.x(), bbox.max.y() };
    return params;
}

std::unique_ptr<ExtrusionEntity> render_core(const ExPolygon &surface,
                                             const raw_infill_pattern_params &raw,
                                             const Flow &flow)
{
    std::unique_ptr<FillRectilinearSawtooth> filler(new FillRectilinearSawtooth());
    filler->set_bounding_box(get_extents(surface));
    filler->layer_id = raw.layer_id;
    filler->z = raw.z;
    filler->angle = float(raw.angle);
    filler->can_angle_cross = true;

    FillParams params;
    params.density = raw.density;
    params.flow_mult = raw.flow_mult;
    params.connection = InfillConnection::icConnected;
    params.bridge_offset = raw.bridge_offset;
    params.fill_resolution = raw.fill_resolution;
    params.anchor_length = raw.anchor_length;
    params.anchor_length_max = raw.anchor_length_max;
    params.dont_adjust = (raw.infill_flags & C_INFILL_FLAG_DONT_ADJUST) != 0;
    params.role = ExtrusionRole::InternalInfill;
    params.flow = flow;
    filler->init_spacing(raw.spacing, params);

    Surface core_surface(stPosInternal | stDensSparse, surface);
    ExtrusionEntitiesPtr output;
    filler->fill_surface_extrusion(&core_surface, params, output);
    REQUIRE(output.size() == 1);
    return std::unique_ptr<ExtrusionEntity>(output.front());
}

slic3r_api::StoredExtrusionEntity render_plugin(PluginStorage &plugin_storage,
                                                const ExPolygon &surface,
                                                const raw_infill_pattern_params &raw,
                                                uint32_t seed)
{
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    rect_fills::FillRectilinear filler;
    filler.plugin_storage = storage;
    filler.layer_id = raw.layer_id;
    filler.z = raw.z;
    filler.angle = float(raw.angle);
    filler.can_angle_cross = true;
    filler.bounding_box.min = rect_work::Point(raw.object_bounding_box.min.x, raw.object_bounding_box.min.y);
    filler.bounding_box.max = rect_work::Point(raw.object_bounding_box.max.x, raw.object_bounding_box.max.y);
    filler.bounding_box.defined = true;
    filler.init_spacing(raw.spacing, raw);

    const slic3r_api::ExPolygon surface_view(
        reinterpret_cast<const expolygon_handle *>(&surface));
    const rect_work::Polylines lines = filler.fill_surface(surface_view, raw);

    slic3r_api::StoredExtrusionEntity output(storage);
    run_ctx_infill_pattern context = {};
    context.output = output.mutable_handle();
    // Keep deterministic test seeds local; set_random_seed also affects future threads.
    Slic3r::thread_data().seed_random_generator(seed);
    sawtooth::publish_sawtooth(context, storage, lines, raw, float(raw.angle));
    return output;
}

void compare_tree(const ExtrusionEntity &core,
                  const slic3r_api::ExtrusionEntity &ported,
                  const std::string &path)
{
    INFO(path);
    REQUIRE(typeid(core) == typeid(ExtrusionEntity));
    REQUIRE(core.is_leaf() == ported.is_leaf());
    REQUIRE(core.child_count() == ported.child_count());
    CHECK(core.can_sort() == ported.sortable());
    CHECK(core.can_reverse() == ported.reversible());

    if (!core.is_leaf()) {
        for (size_t idx = 0; idx < core.child_count(); ++idx)
            compare_tree(core.child(idx), ported.child(uint32_t(idx)),
                         path + "." + std::to_string(idx));
        return;
    }

    const ArcPolyline *core_line = core.polyline_or_null();
    REQUIRE(core_line != nullptr);
    const std::vector<c_point> ported_points = ported.points();
    REQUIRE(core_line->size() == ported_points.size());
    CHECK(core_line->has_z_offset() == ported.has_z_offsets());
    REQUIRE(core_line->has_z_offset());
    REQUIRE(ported.has_z_offsets());
    for (size_t idx = 0; idx < ported_points.size(); ++idx) {
        CHECK(core_line->get_point(idx).x() == ported_points[idx].x);
        CHECK(core_line->get_point(idx).y() == ported_points[idx].y);
        CHECK(core_line->z_offset(idx) == ported.z_offset(uint32_t(idx)));
    }

    const ExtrusionAttributes *core_attributes = core.get_property<ExtrusionAttributes>();
    const slic3r_api::EPropertyAttributes *ported_attributes =
        ported.get(slic3r_api::EPropertyAttributes::key);
    REQUIRE(core_attributes != nullptr);
    REQUIRE(ported_attributes != nullptr);
    CHECK(int32_t(core_attributes->role) == ported_attributes->extrusion_role());
    CHECK(core_attributes->mm3_per_mm ==
          Approx(ported_attributes->c_extrusion_property_attributes::mm3_per_mm).epsilon(1e-6));
    CHECK(core_attributes->width ==
          Approx(ported_attributes->c_extrusion_property_attributes::width).epsilon(1e-6));
    CHECK(core_attributes->height ==
          Approx(ported_attributes->c_extrusion_property_attributes::height).epsilon(1e-6));
}

void append_snapshot(const slic3r_api::ExtrusionEntity &entity, std::ostringstream &snapshot)
{
    snapshot << entity.flags() << ':' << entity.point_count() << ':';
    for (uint32_t idx = 0; idx < entity.point_count(); ++idx) {
        const c_point point = entity.point(idx);
        snapshot << point.x << ',' << point.y << ',' << entity.z_offset(idx) << ';';
    }
    const slic3r_api::EPropertyAttributes *attributes =
        entity.get(slic3r_api::EPropertyAttributes::key);
    if (attributes != nullptr)
        snapshot << attributes->extrusion_role() << ','
                 << attributes->c_extrusion_property_attributes::mm3_per_mm << ','
                 << attributes->c_extrusion_property_attributes::width << ','
                 << attributes->c_extrusion_property_attributes::height;
    snapshot << '[';
    for (const slic3r_api::ExtrusionEntity child : entity.children()) {
        append_snapshot(child, snapshot);
        snapshot << '|';
    }
    snapshot << ']';
}

std::string render_snapshot(const ExPolygon &surface,
                            const raw_infill_pattern_params &raw,
                            uint32_t seed)
{
    PluginStorage plugin_storage;
    slic3r_api::StoredExtrusionEntity output = render_plugin(plugin_storage, surface, raw, seed);
    std::ostringstream snapshot;
    append_snapshot(output.readonly(), snapshot);
    return snapshot.str();
}

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

void run_until_infill(PreparedPerimeterPrint &prepared)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepGeneratePerimeter::clear(prepared.print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::clear(prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    Steps::StepGenerateInfill::clear(prepared.print);
    Steps::StepGenerateInfill::run_step(orchestrator, prepared.print);
}

struct PipelineInspection
{
    size_t z_profiled_paths = 0;
    size_t volume_targets = 0;
    size_t profiled_volume_targets = 0;
};

bool contains_z_profile(const ExtrusionEntity &entity)
{
    if (entity.is_leaf()) {
        const ArcPolyline *polyline = entity.polyline_or_null();
        return polyline != nullptr && polyline->has_z_offset();
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        if (child != nullptr && contains_z_profile(*child))
            return true;
    return false;
}

void inspect_pipeline_entity(const ExtrusionEntity &entity,
                             PipelineInspection &inspection)
{
    const ExtrusionPropertyInfill *property = entity.get_property<ExtrusionPropertyInfill>();
    if (property != nullptr && property->area_to_fill_mm2 > 0.) {
        ++inspection.volume_targets;
        if (contains_z_profile(entity))
            ++inspection.profiled_volume_targets;
    }
    if (entity.is_leaf()) {
        const ArcPolyline *polyline = entity.polyline_or_null();
        if (polyline != nullptr && polyline->has_z_offset())
            ++inspection.z_profiled_paths;
        return;
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        if (child != nullptr)
            inspect_pipeline_entity(*child, inspection);
}

// Inspect generated leaves and recipe metadata so missing output cannot look like a disabled correction.
PipelineInspection inspect_pipeline_print(const Print &print)
{
    PipelineInspection inspection;
    for (const PrintObject &object : print.objects())
        for (const Layer &layer : object.layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region_island : island.regions_islands()) {
                    if (!region_island.has_extrusion(LayerRegionIsland::INFILLS))
                        continue;
                    for (const ExtrusionEntity *entity : region_island.extrusion(LayerRegionIsland::INFILLS).entities())
                        if (entity != nullptr)
                            inspect_pipeline_entity(*entity, inspection);
                }
    return inspection;
}

// Run one top-pattern configuration through the normal surface and infill steps.
PipelineInspection run_pipeline_case(const char *top_pattern)
{
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({
        { "fill_pattern", "rectilinear" },
        { "top_fill_pattern", top_pattern },
        { "fill_density", "20%" },
        { "top_solid_layers", "1" },
        { "bottom_solid_layers", "0" },
        { "solid_infill_every_layers", "0" },
        { "infill_every_layers", "1" },
        { "enforce_full_fill_volume", "1" }
    }));
    const ConfigOption *configured_pattern =
        prepared.print.object(0).printing_region(0).config().option("top_fill_pattern");
    REQUIRE(configured_pattern != nullptr);
    CHECK(configured_pattern->serialize() == top_pattern);

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        "rectilinear",
        "sawtooth"
    });
    run_until_infill(prepared);
    return inspect_pipeline_print(prepared.print);
}

} // namespace

TEST_CASE("Rectilinear sawtooth matches the legacy extrusion tree",
          "[plugins][infill][rectilinear][sawtooth]")
{
    const std::vector<Flow> flows = {
        Flow::new_from_spacing(0.8f, 0.4f, 0.2f, 0.5f),
        Flow::new_from_spacing(0.8f, 0.6f, 0.2f, 0.5f)
    };
    const std::vector<ExPolygon> surfaces = {
        rectangle_expolygon(-35., -25., 35., 25.),
        rectangle_with_hole_expolygon(),
        ExPolygon(Polygon({ Point(-scale_i(35.), -scale_i(25.)), Point(scale_i(35.), -scale_i(25.)),
                            Point(scale_i(20.), scale_i(25.)), Point(-scale_i(15.), scale_i(15.)),
                            Point(-scale_i(35.), scale_i(25.)) }))
    };
    const std::vector<float> densities = { 0.35f, 0.65f, 1.f };
    const std::vector<double> angles = { 0., M_PI / 6., -M_PI / 4. };

    for (const Flow &flow : flows)
        for (const ExPolygon &surface : surfaces)
            for (const float density : densities)
                for (const double angle : angles) {
                    INFO("nozzle " << flow.nozzle_diameter() << " density " << density << " angle " << angle);
                    const raw_infill_pattern_params raw = make_raw_params(surface, flow, density, angle);

                    Slic3r::thread_data().seed_random_generator(0x5a17u);
                    std::unique_ptr<ExtrusionEntity> core = render_core(surface, raw, flow);
                    PluginStorage plugin_storage;
                    slic3r_api::StoredExtrusionEntity ported = render_plugin(plugin_storage, surface, raw, 0x5a17u);

                    REQUIRE(ported.child_count() == 1);
                    compare_tree(*core, ported.readonly().child(0), "root");
                }
}

TEST_CASE("Rectilinear sawtooth is deterministic through the public random API",
          "[plugins][infill][rectilinear][sawtooth]")
{
    const Flow flow = Flow::new_from_spacing(0.8f, 0.4f, 0.2f, 0.5f);
    const ExPolygon surface = rectangle_expolygon(-35., -25., 35., 25.);
    const raw_infill_pattern_params raw = make_raw_params(surface, flow, 0.55f, M_PI / 6.);

    const std::string first = render_snapshot(surface, raw, 0x1234u);
    const std::string second = render_snapshot(surface, raw, 0x1234u);
    const std::string different = render_snapshot(surface, raw, 0x5678u);
    CHECK(first == second);
    CHECK(first != different);
}

TEST_CASE("Sawtooth DONT_ADJUST does not change generated geometry",
          "[plugins][infill][rectilinear][sawtooth]")
{
    const Flow flow = Flow::new_from_spacing(0.8f, 0.4f, 0.2f, 0.5f);
    const ExPolygon surface = rectangle_expolygon(-35., -25., 35., 25.);
    const raw_infill_pattern_params with_flag = make_raw_params(surface, flow, 0.55f, M_PI / 6.);
    raw_infill_pattern_params without_flag = with_flag;
    without_flag.infill_flags =
        static_cast<uint16_t>(without_flag.infill_flags & ~uint16_t(C_INFILL_FLAG_DONT_ADJUST));

    CHECK(render_snapshot(surface, with_flag, 0x1234u) ==
          render_snapshot(surface, without_flag, 0x1234u));
}

TEST_CASE("Sawtooth random utility is inclusive, deterministic and versioned",
          "[plugins][api][utils][sawtooth]")
{
    CHECK(SLIC3R_PLUGIN_API_UTILS_MAJOR == 1u);
    CHECK(SLIC3R_PLUGIN_API_UTILS_MINOR == 1u);

    Slic3r::thread_data().seed_random_generator(0x77u);
    CHECK(slic3r_safe_rand(0) == 0);
    for (int idx = 0; idx < 100; ++idx) {
        const int32_t value = slic3r_safe_rand(10);
        CHECK(value >= 0);
        CHECK(value <= 10);
    }

    Slic3r::thread_data().seed_random_generator(0x88u);
    const int32_t first = slic3r_safe_rand(1000000);
    Slic3r::thread_data().seed_random_generator(0x88u);
    CHECK(slic3r_safe_rand(1000000) == first);
}

TEST_CASE("Rectilinear work points use core interpolation rounding",
          "[plugins][infill][rectilinear][sawtooth]")
{
    const rect_work::Point first(-7, 11);
    const rect_work::Point second(10, -8);
    const Point core_first(-7, 11);
    const Point core_second(10, -8);
    const rect_work::Point ported = first.interpolate(0.375, second);
    const Point core = core_first.interpolate(0.375, core_second);
    CHECK(ported.x() == core.x());
    CHECK(ported.y() == core.y());
}

TEST_CASE("Sawtooth is registered as a top pattern and generates profiled infill",
          "[plugins][infill][rectilinear][sawtooth]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    const Plugin *plugin = orchestrator.get_plugin("sawtooth");
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_step() == INFILL_PATTERN);

    const PipelineInspection sawtooth = run_pipeline_case("sawtooth");
    const PipelineInspection rectilinear = run_pipeline_case("rectilinear");

    CHECK(sawtooth.z_profiled_paths > 0);
    CHECK(sawtooth.profiled_volume_targets == 0);
    CHECK(rectilinear.volume_targets > 0);
}
