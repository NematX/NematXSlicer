#include <catch2/catch.hpp>

#include <cmath>
#include <initializer_list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/Fill/FillSmooth.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Plugins/Infill/SmoothPattern.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepPostInfillGeneration.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"
#include "libslic3r/Surface.hpp"

/*
Smooth (Ironing) plugin tests
=============================

The direct test compares the plugin tree with the native FillSmooth tree: the
flags of every collection, then the points, role and flow of every path. The
native reference skips the deposit gap fill, which belongs to the post-infill
step, and only runs where its sub-fillers, which always rotate on odd layers,
agree with the recipe C_INFILL_FLAG_CAN_ANGLE_CROSS that the plugin follows.
The pipeline tests cover the region settings and the full-fill volume
correction of both passes.
Leaf comparisons borrow the stored polyline and direct attributes, so native
legacy paths and generic leaves obey the same geometry and process checks.

    direct TEST_CASE
        |-- render_native() / render_plugin(): independently build owned trees
        `-- compare_tree(): compare flags, hierarchy, stored points and flow
    pipeline TEST_CASE -> run_pipeline() -> collect_roots() -> collect_flows()
*/

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

namespace {

// FillSmooth without the deposit gap fill, which the post-infill step owns.
class NativeSmooth : public FillSmooth
{
public:
    NativeSmooth() { has_gap_fill[0] = false; }
};

// Restrict the active plugin set for one pipeline run, then restore it.
class ScopedActivePlugins
{
public:
    explicit ScopedActivePlugins(std::initializer_list<const char *> plugin_ids) :
        m_orchestrator(Orchestrator::instance())
    {
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

// Layer frame of one direct case.
struct Frame
{
    bool can_angle_cross;
    uint32_t layer_id;
    double angle;
};

// Ironing settings as the region stores them, and as the plugin reads them.
struct SettingsCase
{
    ConfigOptionFloatOrPercent width;
    ConfigOptionPercent distribution;
    slic3r_api::Infill::SmoothSettings settings;
};

// The source surface of a pipeline root, its stamp, and the flow of each pass.
struct SmoothRoot
{
    uint64_t surface_id = 0;
    double target_mm3 = 0.;
    double volume_mm3 = 0.;
    double deposit_mm3_per_mm = 0.;
    double ironing_mm3_per_mm = 0.;
    size_t ironing_paths = 0;
};

// Mirror the per-surface recipe of a top solid surface.
raw_infill_pattern_params make_recipe(const ExPolygon &surface, const Flow &flow, float density, const Frame &frame);
// Run the native FillSmooth on one recipe.
std::unique_ptr<ExtrusionEntity> render_native(const ExPolygon &surface, const ExPolygons *no_overlap,
                                               const raw_infill_pattern_params &raw, const Flow &flow,
                                               const SettingsCase &settings);
// Run the plugin on one recipe; the caller owns the storage of the result.
slic3r_api::StoredExtrusionEntity render_plugin(PluginStorage &storage, const ExPolygon &surface,
                                                const ExPolygons *no_overlap, const raw_infill_pattern_params &raw,
                                                const slic3r_api::Infill::SmoothSettings &settings);
// Compare collection flags, then each leaf's stored points, direct role and flow.
void compare_tree(const ExtrusionEntity &core, const slic3r_api::ExtrusionEntity &ported, const std::string &path);
// Every point of the published tree, in publication order.
std::vector<std::pair<coord_t, coord_t>> tree_points(const slic3r_api::ExtrusionEntity &entity);
// Slice a 20 x 20 mm slab with smooth top and bottom surfaces up to the infill step, or the post-infill step.
std::vector<SmoothRoot> run_pipeline(std::initializer_list<std::pair<std::string, std::string>> overrides,
                                     bool post_infill);
// Read the stamp, volume and pass flows of every infill root that contains ironing.
std::vector<SmoothRoot> collect_roots(const Print &print);
// Record the flow of each pass from the leaves of one root.
void collect_flows(const ExtrusionEntity &entity, SmoothRoot &root);

raw_infill_pattern_params make_recipe(const ExPolygon &surface, const Flow &flow, float density, const Frame &frame)
{
    raw_infill_pattern_params raw = {};
    raw.surface_type = RAW_SURFACE_TYPE_POS_TOP | RAW_SURFACE_TYPE_DENS_SOLID;
    raw.extrusion_role = RAW_EXTRUSION_ROLE_TOP_SOLID_INFILL;
    raw.flow.width = flow.scaled_width();
    raw.flow.spacing = flow.scaled_spacing();
    raw.flow.height = flow.scaled_height();
    raw.flow.nozzle_diameter = scale_i(flow.nozzle_diameter());
    raw.flow.spacing_ratio = flow.spacing_ratio();
    raw.flow.mm3_per_mm = flow.mm3_per_mm();
    raw.flow.is_bridge = flow.bridge() ? 1 : 0;
    raw.density = density;
    raw.flow_mult = 0.95f;
    raw.connection = C_INFILL_CONNECTION_CONNECTED;
    raw.infill_flags = frame.can_angle_cross ? C_INFILL_FLAG_CAN_ANGLE_CROSS : 0;
    raw.spacing = flow.spacing();
    raw.angle = frame.angle;
    raw.bridge_angle = -1.;
    raw.layer_id = frame.layer_id;
    raw.z = 0.2 * (frame.layer_id + 1);
    raw.overlap = 0.3;
    raw.fill_resolution = scale_i(0.0125);
    raw.anchor_length = 1000.f;
    raw.anchor_length_max = 1000.f;
    const BoundingBox box = get_extents(surface);
    raw.object_bounding_box = {{box.min.x(), box.min.y()}, {box.max.x(), box.max.y()}};
    return raw;
}

std::unique_ptr<ExtrusionEntity> render_native(const ExPolygon &surface, const ExPolygons *no_overlap,
                                               const raw_infill_pattern_params &raw, const Flow &flow,
                                               const SettingsCase &settings)
{
    PrintRegionConfig config;
    config.fill_smooth_width = settings.width;
    config.fill_smooth_distribution = settings.distribution;

    NativeSmooth filler;
    filler.set_bounding_box(get_extents(surface));
    filler.layer_id = raw.layer_id;
    filler.z = raw.z;
    filler.angle = float(raw.angle);
    filler.can_angle_cross = float((raw.infill_flags & C_INFILL_FLAG_CAN_ANGLE_CROSS) != 0);
    filler.link_max_length = raw.link_max_length;
    filler.loop_clipping = raw.loop_clipping;
    filler.overlap = raw.overlap;
    if (no_overlap != nullptr)
        filler.no_overlap_expolygons = *no_overlap;

    FillParams params;
    params.density = raw.density;
    params.flow_mult = raw.flow_mult;
    params.connection = InfillConnection::icConnected;
    params.bridge_offset = raw.bridge_offset;
    params.fill_resolution = raw.fill_resolution;
    params.anchor_length = raw.anchor_length;
    params.anchor_length_max = raw.anchor_length_max;
    params.dont_adjust = false;
    params.monotonic = false;
    params.fill_exactly = false;
    params.role = ExtrusionRole(static_cast<ExtrusionRoleModifier>(raw.extrusion_role));
    params.flow = flow;
    params.config = &config;
    filler.init_spacing(raw.spacing, params);

    const Surface top(stPosTop | stDensSolid, surface);
    ExtrusionEntitiesPtr output;
    filler.fill_surface_extrusion(&top, params, output);
    REQUIRE(output.size() == 1);
    return std::unique_ptr<ExtrusionEntity>(output.front());
}

slic3r_api::StoredExtrusionEntity render_plugin(PluginStorage &storage, const ExPolygon &surface,
                                                const ExPolygons *no_overlap, const raw_infill_pattern_params &raw,
                                                const slic3r_api::Infill::SmoothSettings &settings)
{
    const Surface top(stPosTop | stDensSolid, surface);
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    slic3r_api::StoredExtrusionEntity output(handle);
    run_ctx_infill_pattern ctx = {};
    ctx.surface = reinterpret_cast<const surface_handle *>(&top);
    ctx.no_overlap_areas = reinterpret_cast<const expolygon_collection_handle *>(no_overlap);
    ctx.params = &raw;
    ctx.output = output.mutable_handle();
    slic3r_api::Infill::fill_smooth_surface(ctx, handle, nullptr, settings);
    return output;
}

void compare_tree(const ExtrusionEntity &core, const slic3r_api::ExtrusionEntity &ported, const std::string &path)
{
    INFO(path);
    REQUIRE(core.is_leaf() == ported.is_leaf());
    REQUIRE(core.child_count() == ported.child_count());
    CHECK(core.can_sort() == ported.sortable());
    CHECK(core.can_reverse() == ported.reversible());

    if (!core.is_leaf()) {
        for (size_t idx = 0; idx < core.child_count(); ++idx)
            compare_tree(core.child(idx), ported.child(uint32_t(idx)), path + "." + std::to_string(idx));
        return;
    }

    const ArcPolyline *core_path = core.polyline_or_null();
    REQUIRE(core_path != nullptr);
    const std::vector<c_point> ported_points = ported.points();
    REQUIRE(core_path->size() == ported_points.size());
    for (size_t idx = 0; idx < ported_points.size(); ++idx) {
        CHECK(core_path->get_point(idx).x() == ported_points[idx].x);
        CHECK(core_path->get_point(idx).y() == ported_points[idx].y);
    }

    const ExtrusionAttributes *core_attributes = core.get_property<ExtrusionAttributes>();
    const slic3r_api::EPropertyAttributes *ported_attributes = ported.get(slic3r_api::EPropertyAttributes::key);
    REQUIRE(core_attributes != nullptr);
    REQUIRE(ported_attributes != nullptr);
    CHECK(int32_t(core_attributes->role) == ported_attributes->extrusion_role());
    CHECK(core_attributes->mm3_per_mm ==
          Approx(ported_attributes->c_extrusion_property_attributes::mm3_per_mm).epsilon(1e-6));
    // The recipe carries the flow width in scaled integer units.
    CHECK(core_attributes->width == Approx(ported_attributes->c_extrusion_property_attributes::width).epsilon(1e-5));
    CHECK(core_attributes->height == Approx(ported_attributes->c_extrusion_property_attributes::height).epsilon(1e-6));
}

std::vector<std::pair<coord_t, coord_t>> tree_points(const slic3r_api::ExtrusionEntity &entity)
{
    std::vector<std::pair<coord_t, coord_t>> points;
    if (entity.is_leaf()) {
        for (const c_point point : entity.points())
            points.emplace_back(point.x, point.y);
        return points;
    }
    for (const slic3r_api::ExtrusionEntity child : entity.children())
        for (const std::pair<coord_t, coord_t> &point : tree_points(child))
            points.push_back(point);
    return points;
}

std::vector<SmoothRoot> run_pipeline(std::initializer_list<std::pair<std::string, std::string>> overrides,
                                     bool post_infill)
{
    DynamicPrintConfig config = perimeter_config({
        { "fill_pattern", "rectilinear" },
        { "fill_density", "20%" },
        { "top_fill_pattern", "smooth" },
        { "bottom_fill_pattern", "smooth" },
        { "top_solid_layers", "0" },
        { "bottom_solid_layers", "0" },
        { "solid_infill_every_layers", "0" },
        { "infill_every_layers", "1" }
    });
    for (const std::pair<std::string, std::string> &entry : overrides)
        config.set_deserialize_strict(entry.first, entry.second);
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, config);
    // The initial typed surface builder makes the first layer bottom-solid and the last one top-solid.
    replace_all_layer_islands(prepared, rectangle_expolygon(-10., -10., 10., 10.));

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        "rectilinear",
        "smooth",
        "infill.post_process.full_fill_volume"
    });
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepGeneratePerimeter::clear(prepared.print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::clear(prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    Steps::StepGenerateInfill::clear(prepared.print);
    Steps::StepGenerateInfill::run_step(orchestrator, prepared.print);
    if (post_infill) {
        Steps::StepPostInfillGeneration::clear(prepared.print);
        Steps::StepPostInfillGeneration::run_step(orchestrator, prepared.print);
    }
    return collect_roots(prepared.print);
}

std::vector<SmoothRoot> collect_roots(const Print &print)
{
    std::vector<SmoothRoot> roots;
    for (const PrintObject &object : print.objects())
        for (const Layer &layer : object.layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region_island : island.regions_islands()) {
                    if (!region_island.has_extrusion(LayerRegionIsland::INFILLS))
                        continue;
                    for (const ExtrusionEntity *entity : region_island.extrusion(LayerRegionIsland::INFILLS).entities()) {
                        SmoothRoot root;
                        collect_flows(*entity, root);
                        if (root.ironing_paths == 0)
                            continue;
                        root.volume_mm3 = entity->total_volume();
                        const ExtrusionPropertyInfill *property = entity->get_property<ExtrusionPropertyInfill>();
                        if (property != nullptr) {
                            root.surface_id = property->source_surface_id;
                            root.target_mm3 =
                                property->area_to_fill_mm2 * unscaled(property->height) * double(property->flow_mult);
                        }
                        roots.push_back(root);
                    }
                }
    return roots;
}

void collect_flows(const ExtrusionEntity &entity, SmoothRoot &root)
{
    if (!entity.is_leaf()) {
        for (const ExtrusionEntityUPtr &child : entity.children())
            collect_flows(*child, root);
        return;
    }
    const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
    REQUIRE(attributes != nullptr);
    if (attributes->role == ExtrusionRole::Ironing) {
        ++root.ironing_paths;
        root.ironing_mm3_per_mm = attributes->mm3_per_mm;
    } else {
        root.deposit_mm3_per_mm = attributes->mm3_per_mm;
    }
}

} // namespace

TEST_CASE("Smooth plugin matches the native FillSmooth tree", "[plugins][infill][smooth]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExPolygon lshape(Polygon({ Point(-scale_i(35.), -scale_i(25.)), Point(scale_i(35.), -scale_i(25.)),
                                     Point(scale_i(20.), scale_i(25.)), Point(-scale_i(15.), scale_i(15.)),
                                     Point(-scale_i(35.), scale_i(25.)) }));
    const std::vector<ExPolygon> surfaces = { rectangle_expolygon(-35., -25., 35., 25.),
                                              rectangle_with_hole_expolygon(), lshape };
    // Odd layers only agree while the recipe lets the angle cross, like the native sub-fillers.
    const std::vector<Frame> frames = { { true, 3, M_PI / 6. }, { true, 4, 0. }, { false, 4, -M_PI / 4. } };
    const std::vector<SettingsCase> settings = {
        { ConfigOptionFloatOrPercent(50., true), ConfigOptionPercent(10.), { 0.5, 0.1 } },
        { ConfigOptionFloatOrPercent(0.33, false), ConfigOptionPercent(30.), { 0.33, 0.3 } }
    };
    const Flow flow = Flow::new_from_spacing(0.45f, 0.4f, 0.2f, 1.f);
    const Flow bridge = Flow::new_from_spacing(0.45f, 0.4f, 0.2f, 1.f, true);

    for (const ExPolygon &surface : surfaces) {
        const ExPolygons no_overlap = offset_ex(surface, -scale_d(0.4));
        // Without areas, with the half-overlap areas, and on a bridge that ignores them.
        const std::vector<std::pair<const ExPolygons *, const Flow *>> overlaps = {
            { nullptr, &flow }, { &no_overlap, &flow }, { &no_overlap, &bridge } };
        for (const std::pair<const ExPolygons *, const Flow *> &overlap : overlaps)
            for (const Frame &frame : frames)
                for (const SettingsCase &setting : settings)
                    for (const float density : { 1.f, 0.85f }) {
                        CAPTURE(surface.holes.size(), surface.area(), overlap.first != nullptr,
                                overlap.second->bridge(), frame.can_angle_cross, frame.layer_id, frame.angle,
                                setting.settings.width, setting.settings.distribution, density);
                        const raw_infill_pattern_params raw = make_recipe(surface, *overlap.second, density, frame);
                        const std::unique_ptr<ExtrusionEntity> core =
                            render_native(surface, overlap.first, raw, *overlap.second, setting);
                        PluginStorage storage;
                        const slic3r_api::StoredExtrusionEntity ported =
                            render_plugin(storage, surface, overlap.first, raw, setting.settings);
                        REQUIRE(ported.child_count() == 1);
                        compare_tree(*core, ported.readonly().child(0), "root");
                    }
    }
}

TEST_CASE("Smooth fillers keep their angle on odd layers without angle crossing", "[plugins][infill][smooth]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExPolygon surface = rectangle_with_hole_expolygon();
    const Flow flow = Flow::new_from_spacing(0.45f, 0.4f, 0.2f, 1.f);
    const slic3r_api::Infill::SmoothSettings settings;
    PluginStorage storage;
    const slic3r_api::StoredExtrusionEntity odd = render_plugin(
        storage, surface, nullptr, make_recipe(surface, flow, 1.f, { false, 3, M_PI / 6. }), settings);
    const slic3r_api::StoredExtrusionEntity even = render_plugin(
        storage, surface, nullptr, make_recipe(surface, flow, 1.f, { false, 4, M_PI / 6. }), settings);
    const slic3r_api::StoredExtrusionEntity crossing = render_plugin(
        storage, surface, nullptr, make_recipe(surface, flow, 1.f, { true, 3, M_PI / 6. }), settings);
    const std::vector<std::pair<coord_t, coord_t>> odd_points = tree_points(odd.readonly());
    REQUIRE_FALSE(odd_points.empty());
    CHECK(odd_points == tree_points(even.readonly()));
    CHECK(odd_points != tree_points(crossing.readonly()));
}

TEST_CASE("Smooth cancellation discards the unpublished passes", "[plugins][infill][smooth][cancellation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExPolygon polygon = rectangle_with_hole_expolygon();
    const Surface surface(stPosTop | stDensSolid, polygon);
    const raw_infill_pattern_params raw =
        make_recipe(polygon, Flow::new_from_spacing(0.45f, 0.4f, 0.2f, 1.f), 1.f, { true, 3, 0. });
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    slic3r_api::StoredExtrusionEntity output(handle);
    const size_t initial = storage.size();
    run_ctx_infill_pattern ctx = {};
    ctx.surface = reinterpret_cast<const surface_handle *>(&surface);
    ctx.params = &raw;
    ctx.output = output.mutable_handle();
    // Cancel once a collection other than the output holds a filled pass.
    struct Probe { PluginStorage *storage; const extrusion_entity_handle *output; bool ready = false; } probe{&storage, output.handle()};
    plugin_run_context run = {};
    run.host_context = reinterpret_cast<plugin_host_context *>(&probe);
    run.is_cancelled = [](plugin_host_context *host) {
        Probe &probe = *reinterpret_cast<Probe *>(host);
        // Inspect a locked pointer snapshot; this single-threaded probe keeps the temporary entities alive.
        for (const ExtrusionEntity *owned : probe.storage->stored_extrusions()) {
            const slic3r_api::ExtrusionEntity entity(reinterpret_cast<const extrusion_entity_handle *>(owned));
            if (entity.handle() != probe.output && !entity.is_leaf() && !entity.empty() && !entity.child(0).is_leaf()) {
                probe.ready = true;
                return 1;
            }
        }
        return 0;
    };
    CHECK_THROWS_AS(slic3r_api::Infill::fill_smooth_surface(ctx, handle, &run, slic3r_api::Infill::SmoothSettings{}),
                    slic3r_api::PluginCancelled);
    CHECK(probe.ready);
    CHECK(output.empty());
    CHECK(storage.size() == initial);
}

TEST_CASE("Smooth reads its ironing settings from the region", "[plugins][infill][smooth]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Plugin *plugin = Orchestrator::instance().get_plugin("smooth");
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_step() == INFILL_PATTERN);

    struct Expected { const char *width; const char *distribution; double ratio; };
    // Lines width times closer carry width times less flow, then each pass takes its share.
    for (const Expected expected : { Expected{ "50%", "10%", 0.5 * 0.1 / 0.9 }, Expected{ "30%", "25%", 0.3 * 0.25 / 0.75 } }) {
        CAPTURE(expected.width, expected.distribution);
        const std::vector<SmoothRoot> roots = run_pipeline({
            { "fill_smooth_width", expected.width },
            { "fill_smooth_distribution", expected.distribution },
            { "enforce_full_fill_volume", "0" } }, false);
        REQUIRE_FALSE(roots.empty());
        for (const SmoothRoot &root : roots) {
            REQUIRE(root.deposit_mm3_per_mm > 0.);
            CHECK(root.ironing_mm3_per_mm / root.deposit_mm3_per_mm == Approx(expected.ratio).epsilon(1e-5));
        }
    }
}

TEST_CASE("Smooth passes are corrected together by the full-fill volume step", "[plugins][infill][smooth][volume]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::initializer_list<std::pair<std::string, std::string>> overrides = {
        { "enforce_full_fill_volume", "1" } };
    const std::vector<SmoothRoot> before = run_pipeline(overrides, false);
    const std::vector<SmoothRoot> after = run_pipeline(overrides, true);
    REQUIRE_FALSE(before.empty());
    REQUIRE(after.size() == before.size());
    for (size_t index = 0; index < before.size(); ++index) {
        CAPTURE(index);
        REQUIRE(before[index].target_mm3 > 0.);
        // The default overlap keeps the measured volume unnormalized; the ratio stays inside the failsafe bounds.
        const double ratio = before[index].target_mm3 / before[index].volume_mm3;
        REQUIRE(ratio > 0.8);
        REQUIRE(ratio < 1.3);
        CHECK(after[index].volume_mm3 == Approx(before[index].volume_mm3 * ratio).epsilon(1e-6));
        // One factor scales both passes, so their flow split is unchanged.
        CHECK(after[index].ironing_mm3_per_mm / after[index].deposit_mm3_per_mm ==
              Approx(before[index].ironing_mm3_per_mm / before[index].deposit_mm3_per_mm).epsilon(1e-6));
    }
}
