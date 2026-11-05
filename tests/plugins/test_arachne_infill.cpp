#include <catch2/catch.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityVisitors.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Geometry/MedialAxis.hpp"
#include "libslic3r/Geometry/Voronoi.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Plugins/Infill/ArachneInfill.hpp"
#include "libslic3r/Plugins/Infill/ConcentricPattern.hpp"
#include "libslic3r/Plugins/Infill/EnsuringPattern.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"
#include "libslic3r/Surface.hpp"

/*
Arachne infill plugin tests
===========================

The host conversion of variable-width lines is compared with
Geometry::thin_variable_width(), and the plugin settings with the settings the
legacy WallToolPaths constructor resolves.

The concentric plugin is compared with the native FillConcentric Arachne
branch, entity by entity: the native collection may be sorted later, while the
plugin keeps every component in the native wall order, so each component must
appear as one run of native entities in their native order. The pipeline test
checks that the Arachne perimeter generator setting selects the walls.

The ensuring plugin is compared with the native FillEnsuring tree as a whole:
same group flags, same entities in the same order, and the same points up to
SCALED_EPSILON. Both trees are written as text; one linear pass separates the
points from the rest of the text, which must be identical. A dedicated case
checks that this comparison rejects every other difference.
*/

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

namespace {

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

// Region settings of one parity case, as option values.
using Settings = std::vector<std::pair<std::string, std::string>>;

// A deterministic text of a native entity: flags, then the points, role and flow of each leaf.
std::string snapshot(const ExtrusionEntity &entity);
// The same text of an API entity.
std::string snapshot(const slic3r_api::ExtrusionEntity &entity);
// The text of one leaf.
std::string leaf_text(const std::vector<std::pair<coord_t, coord_t>> &points, int32_t role, double mm3_per_mm,
                      float width, float height);
// A region configuration with some settings changed.
PrintRegionConfig region_config(const Settings &settings);
// The recipe of a full top surface, with the flow the legacy wrapper rebuilds from its spacing.
raw_infill_pattern_params make_recipe(const ExPolygon &surface, float spacing_ratio, coord_t loop_clipping);
// Run the native FillConcentric Arachne branch on one recipe.
std::vector<std::string> native_entities(const ExPolygon &polygon, const raw_infill_pattern_params &raw,
                                         const PrintRegionConfig &region, const PrintConfig &print);
// Run the plugin on one recipe and read the entities of its group.
std::vector<std::string> plugin_entities(const ExPolygon &polygon, const raw_infill_pattern_params &raw,
                                         const Arachne::WallToolPathsSettings &settings);
// Layer frame of one ensuring case.
struct Frame
{
    double angle;
    uint32_t layer_id;
    bool can_angle_cross;
    double bridge_angle;
};
// The recipe of an internal solid surface for the ensuring pattern.
raw_infill_pattern_params ensuring_recipe(const ExPolygon &surface, const Frame &frame, double overlap);
// Run the native FillEnsuring on one recipe and read its whole tree.
std::string native_ensuring(const ExPolygon &polygon, const raw_infill_pattern_params &raw,
                            const PrintRegionConfig &region, const PrintConfig &print);
// Run the ensuring plugin on one recipe and read its whole tree.
std::string plugin_ensuring(const ExPolygon &polygon, const raw_infill_pattern_params &raw,
                            const Arachne::WallToolPathsSettings &settings);
// One tree text with each "x,y;" point replaced by ";", and the point coordinates in order.
struct TreeShape
{
    std::string text;
    std::vector<long long> coordinates;
};
// Separate the points of a tree text in one pass. A point is an optional minus sign
// and digits, a comma, another such number and a semicolon; other characters are kept.
TreeShape split_points(const std::string &tree);
// Whether two tree texts are the same, coordinates compared up to SCALED_EPSILON:
// the plugin and the native filler reach the same areas through different Clipper
// operations, whose outputs are cleaned at SCALED_EPSILON. Flags, structure, roles,
// flows and point counts must match exactly.
bool same_tree(const std::string &plugin, const std::string &native);
// Slice a thin slab with a concentric top surface and read the width of every top infill leaf.
std::vector<float> top_infill_widths(const char *perimeter_generator);
// Collect the widths of the leaves of one entity.
void collect_widths(const ExtrusionEntity &entity, std::vector<float> &widths);

std::string leaf_text(const std::vector<std::pair<coord_t, coord_t>> &points, int32_t role, double mm3_per_mm,
                      float width, float height)
{
    std::string text;
    for (const std::pair<coord_t, coord_t> &point : points)
        text += std::to_string(point.first) + "," + std::to_string(point.second) + ";";
    char flow[96];
    std::snprintf(flow, sizeof(flow), "|%d|%.17g|%.9g|%.9g", int(role), mm3_per_mm, double(width), double(height));
    return text + flow;
}

std::string snapshot(const ExtrusionEntity &entity)
{
    std::string text = std::string(entity.is_loop() ? "L" : "") + (entity.can_sort() ? "S" : "") +
                       (entity.can_reverse() ? "R" : "") + "{";
    if (entity.is_leaf()) {
        const ArcPolyline *path = entity.polyline_or_null();
        REQUIRE(path != nullptr);
        std::vector<std::pair<coord_t, coord_t>> points;
        for (size_t index = 0; index < path->size(); ++index)
            points.emplace_back(path->get_point(index).x(), path->get_point(index).y());
        const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
        REQUIRE(attributes != nullptr);
        return text + leaf_text(points, int32_t(attributes->role), attributes->mm3_per_mm, attributes->width,
                                attributes->height) + "}";
    }
    for (size_t index = 0; index < entity.child_count(); ++index)
        text += snapshot(entity.child(index));
    return text + "}";
}

std::string snapshot(const slic3r_api::ExtrusionEntity &entity)
{
    std::string text = std::string(entity.is_loop() ? "L" : "") + (entity.sortable() ? "S" : "") +
                       (entity.reversible() ? "R" : "") + "{";
    if (entity.is_leaf()) {
        std::vector<std::pair<coord_t, coord_t>> points;
        for (const c_point point : entity.points())
            points.emplace_back(point.x, point.y);
        const slic3r_api::EPropertyAttributes *attributes = entity.get(slic3r_api::EPropertyAttributes::key);
        REQUIRE(attributes != nullptr);
        const c_extrusion_property_attributes &flow = *attributes;
        return text + leaf_text(points, attributes->extrusion_role(), flow.mm3_per_mm, flow.width, flow.height) + "}";
    }
    for (const slic3r_api::ExtrusionEntity child : entity.children())
        text += snapshot(child);
    return text + "}";
}

PrintRegionConfig region_config(const Settings &settings)
{
    DynamicPrintConfig changes;
    for (const std::pair<std::string, std::string> &setting : settings)
        changes.set_deserialize_strict(setting.first, setting.second);
    PrintRegionConfig region;
    region.apply(changes, true);
    return region;
}

raw_infill_pattern_params make_recipe(const ExPolygon &surface, float spacing_ratio, coord_t loop_clipping)
{
    const Flow requested = Flow::new_from_spacing(0.45f, 0.4f, 0.2f, spacing_ratio, false);
    // FillParams of the legacy wrapper rebuild the flow from the scaled spacing.
    const Flow flow = Flow::new_from_spacing(float(unscaled(requested.scaled_spacing())), 0.4f, 0.2f, spacing_ratio, false);
    raw_infill_pattern_params raw = {};
    raw.surface_type = RAW_SURFACE_TYPE_POS_TOP | RAW_SURFACE_TYPE_DENS_SOLID;
    raw.extrusion_role = RAW_EXTRUSION_ROLE_TOP_SOLID_INFILL;
    raw.flow.width = flow.scaled_width();
    raw.flow.spacing = flow.scaled_spacing();
    raw.flow.height = flow.scaled_height();
    raw.flow.nozzle_diameter = scale_i(flow.nozzle_diameter());
    raw.flow.spacing_ratio = flow.spacing_ratio();
    raw.flow.mm3_per_mm = flow.mm3_per_mm();
    raw.density = 1.f;
    raw.flow_mult = 0.95f;
    raw.infill_flags = C_INFILL_FLAG_USE_ARACHNE;
    raw.spacing = flow.spacing();
    raw.bridge_angle = -1.;
    raw.layer_height = 0.2;
    raw.loop_clipping = loop_clipping;
    raw.fill_resolution = scale_i(0.0125);
    const BoundingBox box = get_extents(surface);
    raw.object_bounding_box = {{box.min.x(), box.min.y()}, {box.max.x(), box.max.y()}};
    return raw;
}

std::vector<std::string> native_entities(const ExPolygon &polygon, const raw_infill_pattern_params &raw,
                                         const PrintRegionConfig &region, const PrintConfig &print)
{
    static const PrintObjectConfig object;
    const std::unique_ptr<Fill> filler(Fill::new_from_type(ipConcentric));
    filler->set_bounding_box(get_extents(polygon));
    filler->angle = 0.f;
    filler->layer_id = 0;
    filler->loop_clipping = raw.loop_clipping;
    filler->set_config(&print, &object);
    FillParams params;
    params.config = &region;
    params.density = raw.density;
    params.dont_adjust = false;
    params.flow_mult = raw.flow_mult;
    params.fill_resolution = raw.fill_resolution;
    params.use_arachne = true;
    params.layer_height = float(raw.layer_height);
    params.role = ExtrusionRole(static_cast<ExtrusionRoleModifier>(raw.extrusion_role));
    params.flow = Flow::new_from_spacing(float(unscaled(raw.flow.spacing)), float(unscaled(raw.flow.nozzle_diameter)),
                                         float(unscaled(raw.flow.height)), raw.flow.spacing_ratio, false);
    filler->init_spacing(raw.spacing, params);
    const Surface surface(stPosTop | stDensSolid, polygon);
    ExtrusionEntitiesPtr output;
    filler->fill_surface_extrusion(&surface, params, output);
    std::vector<std::string> entities;
    REQUIRE(output.size() <= 1);
    for (ExtrusionEntity *collection : output) {
        CHECK(collection->can_sort());
        for (size_t index = 0; index < collection->child_count(); ++index)
            entities.push_back(snapshot(collection->child(index)));
        delete collection;
    }
    return entities;
}

// Generate into local owned storage from borrowed surface/settings, retain the output view throughout child traversal
// and return owned snapshots; generation/allocation failures propagate and Catch reports shape or permission mismatches.
std::vector<std::string> plugin_entities(const ExPolygon &polygon, const raw_infill_pattern_params &raw,
                                         const Arachne::WallToolPathsSettings &settings)
{
    const Surface surface(stPosTop | stDensSolid, polygon);
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    slic3r_api::StoredExtrusionEntity output(handle);
    run_ctx_infill_pattern ctx = {};
    ctx.surface = reinterpret_cast<const surface_handle *>(&surface);
    ctx.params = &raw;
    ctx.output = output.mutable_handle();
    slic3r_api::Infill::fill_concentric_surface(ctx, handle, nullptr, settings);
    std::vector<std::string> entities;
    REQUIRE(output.child_count() <= 1);
    const slic3r_api::ExtrusionEntity output_view = output.readonly();
    for (const slic3r_api::ExtrusionEntity group : output_view.children()) {
        CHECK_FALSE(group.sortable());
        CHECK_FALSE(group.reversible());
        for (const slic3r_api::ExtrusionEntity child : group.children())
            entities.push_back(snapshot(child));
    }
    return entities;
}

raw_infill_pattern_params ensuring_recipe(const ExPolygon &surface, const Frame &frame, double overlap)
{
    raw_infill_pattern_params raw = make_recipe(surface, 1.f, 0);
    raw.surface_type = RAW_SURFACE_TYPE_POS_INTERNAL | RAW_SURFACE_TYPE_DENS_SOLID;
    raw.extrusion_role = RAW_EXTRUSION_ROLE_SOLID_INFILL;
    raw.angle = frame.angle;
    raw.layer_id = frame.layer_id;
    raw.bridge_angle = frame.bridge_angle;
    raw.infill_flags = C_INFILL_FLAG_USE_ARACHNE | (frame.can_angle_cross ? C_INFILL_FLAG_CAN_ANGLE_CROSS : 0);
    raw.overlap = overlap;
    return raw;
}

std::string native_ensuring(const ExPolygon &polygon, const raw_infill_pattern_params &raw,
                            const PrintRegionConfig &region, const PrintConfig &print)
{
    // Same configuration as the former legacy wrapper of this pattern.
    static const PrintObjectConfig object;
    const std::unique_ptr<Fill> filler(Fill::new_from_type(ipEnsuring));
    filler->set_bounding_box(get_extents(polygon));
    filler->layer_id = raw.layer_id;
    filler->z = raw.z;
    filler->angle = float(raw.angle);
    filler->can_angle_cross = (raw.infill_flags & C_INFILL_FLAG_CAN_ANGLE_CROSS) != 0;
    filler->overlap = raw.overlap;
    filler->set_config(&print, &object);
    FillParams params;
    params.config = &region;
    params.density = raw.density;
    params.dont_adjust = false;
    params.flow_mult = raw.flow_mult;
    params.fill_resolution = raw.fill_resolution;
    params.use_arachne = true;
    params.layer_height = float(raw.layer_height);
    params.role = ExtrusionRole(static_cast<ExtrusionRoleModifier>(raw.extrusion_role));
    params.flow = Flow::new_from_spacing(float(unscaled(raw.flow.spacing)), float(unscaled(raw.flow.nozzle_diameter)),
                                         float(unscaled(raw.flow.height)), raw.flow.spacing_ratio, false);
    filler->init_spacing(raw.spacing, params);
    Surface surface(stPosInternal | stDensSolid, polygon);
    surface.bridge_angle = raw.bridge_angle;
    ExtrusionEntitiesPtr output;
    filler->fill_surface_extrusion(&surface, params, output);
    REQUIRE(output.size() <= 1);
    std::string tree;
    for (ExtrusionEntity *collection : output) {
        tree = snapshot(*collection);
        delete collection;
    }
    return tree;
}

// Generate into local owned storage from borrowed surface/settings, retain the output view throughout child traversal
// and return an owned tree snapshot; generation/allocation failures propagate and Catch reports invalid output shape.
std::string plugin_ensuring(const ExPolygon &polygon, const raw_infill_pattern_params &raw,
                            const Arachne::WallToolPathsSettings &settings)
{
    const Surface surface(stPosInternal | stDensSolid, polygon);
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    slic3r_api::StoredExtrusionEntity output(handle);
    run_ctx_infill_pattern ctx = {};
    ctx.surface = reinterpret_cast<const surface_handle *>(&surface);
    ctx.params = &raw;
    ctx.output = output.mutable_handle();
    slic3r_api::Infill::fill_ensuring_surface(ctx, handle, nullptr, settings);
    REQUIRE(output.child_count() <= 1);
    std::string tree;
    const slic3r_api::ExtrusionEntity output_view = output.readonly();
    for (const slic3r_api::ExtrusionEntity group : output_view.children())
        tree = snapshot(group);
    return tree;
}

TreeShape split_points(const std::string &tree)
{
    // End of an optional minus sign followed by at least one digit, or npos.
    const auto number_end = [&tree](size_t start) {
        size_t end = start < tree.size() && tree[start] == '-' ? start + 1 : start;
        const size_t digits = end;
        while (end < tree.size() && tree[end] >= '0' && tree[end] <= '9')
            ++end;
        return end > digits ? end : std::string::npos;
    };
    TreeShape shape;
    shape.text.reserve(tree.size());
    for (size_t at = 0; at < tree.size();) {
        const size_t x_end = number_end(at);
        if (x_end != std::string::npos && x_end < tree.size() && tree[x_end] == ',') {
            const size_t y_end = number_end(x_end + 1);
            if (y_end != std::string::npos && y_end < tree.size() && tree[y_end] == ';') {
                shape.coordinates.push_back(std::strtoll(tree.c_str() + at, nullptr, 10));
                shape.coordinates.push_back(std::strtoll(tree.c_str() + x_end + 1, nullptr, 10));
                shape.text += ';';
                at = y_end + 1;
                continue;
            }
        }
        shape.text += tree[at++];
    }
    return shape;
}

bool same_tree(const std::string &plugin, const std::string &native)
{
    const TreeShape actual = split_points(plugin), expected = split_points(native);
    if (actual.text != expected.text || actual.coordinates.size() != expected.coordinates.size())
        return false;
    for (size_t index = 0; index < actual.coordinates.size(); ++index)
        if (std::abs(actual.coordinates[index] - expected.coordinates[index]) > SCALED_EPSILON)
            return false;
    return true;
}

std::vector<float> top_infill_widths(const char *perimeter_generator)
{
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({
        { "perimeter_generator", perimeter_generator },
        { "fill_pattern", "rectilinear" },
        { "top_fill_pattern", "concentric" },
        { "fill_density", "20%" },
        { "top_solid_layers", "0" },
        { "bottom_solid_layers", "0" },
        { "solid_infill_every_layers", "0" },
        { "infill_every_layers", "1" }
    }));
    // A width that is not a multiple of the spacing makes Arachne vary its walls.
    replace_all_layer_islands(prepared, rectangle_expolygon(-10., -6.65, 10., 6.65));
    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        "rectilinear",
        "concentric"
    });
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepGeneratePerimeter::clear(prepared.print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::clear(prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    Steps::StepGenerateInfill::clear(prepared.print);
    Steps::StepGenerateInfill::run_step(orchestrator, prepared.print);

    // The initial typed surface builder makes the last layer top-solid.
    std::vector<float> widths;
    for (const PrintObject &object : prepared.print.objects())
        for (const Layer &layer : object.layers()) {
            widths.clear();
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region_island : island.regions_islands())
                    if (region_island.has_extrusion(LayerRegionIsland::INFILLS))
                        for (const ExtrusionEntity *entity : region_island.extrusion(LayerRegionIsland::INFILLS).entities())
                            collect_widths(*entity, widths);
        }
    return widths;
}

void collect_widths(const ExtrusionEntity &entity, std::vector<float> &widths)
{
    if (!entity.is_leaf()) {
        for (const ExtrusionEntityUPtr &child : entity.children())
            collect_widths(*child, widths);
        return;
    }
    const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
    REQUIRE(attributes != nullptr);
    widths.push_back(attributes->width);
}

} // namespace

TEST_CASE("Thick polyline extrusion matches thin_variable_width", "[plugins][api][extrusion][arachne]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ThickPolyline open;
    open.points = { Point(0, 0), Point(scale_i(3.), 0), Point(scale_i(6.), scale_i(1.)), Point(scale_i(9.), scale_i(1.)) };
    open.points_width = { scale_i(0.45), scale_i(0.45), scale_i(0.7), scale_i(0.31) };
    ThickPolyline closed;
    closed.points = { Point(0, 0), Point(scale_i(5.), 0), Point(scale_i(5.), scale_i(5.)), Point(0, scale_i(5.)), Point(0, 0) };
    closed.points_width = { scale_i(0.45), scale_i(0.5), scale_i(0.6), scale_i(0.5), scale_i(0.45) };
    ThickPolyline constant;
    constant.points = { Point(0, 0), Point(scale_i(4.), scale_i(3.)) };
    constant.points_width = { scale_i(0.45), scale_i(0.45) };

    for (const ThickPolyline &line : { open, closed, constant })
        for (const float ratio : { 1.f, 0.7f })
            for (const bool can_reverse : { true, false })
                for (const coord_t tolerance : { coord_t(0), scale_i(0.02) }) {
                    CAPTURE(line.points.size(), ratio, can_reverse, tolerance);
                    // The conversion rebuilds the flow from the scaled spacing.
                    const Flow requested = Flow::new_from_spacing(0.45f, 0.4f, 0.2f, ratio, false);
                    const Flow flow = Flow::new_from_spacing(float(unscaled(requested.scaled_spacing())), 0.4f, 0.2f, ratio, false);
                    const coord_t resolution = flow.scaled_width() / 8;
                    ExtrusionEntitiesPtr native = tolerance > 0 ?
                        Geometry::thin_variable_width({line}, ExtrusionRole::SolidInfill, flow, resolution, tolerance, can_reverse) :
                        Geometry::thin_variable_width({line}, ExtrusionRole::SolidInfill, flow, resolution, can_reverse);
                    REQUIRE(native.size() == 1);
                    // Fill multiplies by the float flow_mult of the recipe.
                    ExtrusionModifyFlow(double(0.9f)).traverse(*native.front());
                    const std::string expected = snapshot(*native.front());
                    delete native.front();

                    c_thick_polyline_extrusion_params params = {};
                    params.role = RAW_EXTRUSION_ROLE_SOLID_INFILL;
                    params.flow.width = flow.scaled_width();
                    params.flow.spacing = flow.scaled_spacing();
                    params.flow.height = flow.scaled_height();
                    params.flow.nozzle_diameter = scale_i(flow.nozzle_diameter());
                    params.flow.spacing_ratio = flow.spacing_ratio();
                    params.width_change_tolerance = tolerance;
                    params.flow_mult = 0.9f;
                    params.flags = THICK_POLYLINE_EXTRUSION_FLOW_FROM_SPACING |
                                   (can_reverse ? THICK_POLYLINE_EXTRUSION_CAN_REVERSE : 0u);
                    std::vector<c_point> points;
                    for (const Point &point : line.points)
                        points.push_back(c_point{point.x(), point.y()});
                    PluginStorage storage;
                    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
                    const std::optional<slic3r_api::StoredExtrusionEntity> converted =
                        slic3r_api::thick_polyline_extrusion(handle, points, line.points_width, params);
                    REQUIRE(converted.has_value());
                    CHECK(snapshot(converted->readonly()) == expected);
                }

    PluginStorage storage;
    c_thick_polyline_extrusion_params params = {};
    params.flow = c_flow{scale_i(0.45), scale_i(0.4), scale_i(0.2), scale_i(0.4), 0, 1.f, 0.};
    params.flow_mult = 1.f;
    CHECK_FALSE(slic3r_api::thick_polyline_extrusion(reinterpret_cast<storage_handle *>(&storage),
                                                     { c_point{0, 0} }, { scale_i(0.45) }, params).has_value());
}

TEST_CASE("Arachne infill settings match the WallToolPaths configuration", "[plugins][infill][arachne]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PrintConfig print;
    print.set_deserialize_strict("nozzle_diameter", "0.6,0.4,0.5");
    const std::vector<Settings> cases = {
        {},
        { { "min_bead_width", "60%" }, { "min_feature_size", "0.1" }, { "wall_transition_length", "0.8" },
          { "wall_transition_filter_deviation", "30%" }, { "wall_transition_angle", "20" },
          { "wall_distribution_count", "3" } }
    };
    for (const Settings &settings : cases) {
        CAPTURE(settings.size());
        const PrintRegionConfig region = region_config(settings);
        const Arachne::WallToolPathsSettings expected = Arachne::wall_tool_paths_settings(region, print);
        const Arachne::WallToolPathsSettings actual = slic3r_api::Infill::ArachneInfill::read_settings(
            slic3r_api::Config(ApiHost::to_config_handle(&region)), slic3r_api::Config(ApiHost::to_config_handle(&print)));
        CHECK(actual.min_nozzle_diameter == expected.min_nozzle_diameter);
        CHECK(actual.min_feature_size == expected.min_feature_size);
        CHECK(actual.min_bead_width == expected.min_bead_width);
        CHECK(actual.wall_transition_filter_deviation == expected.wall_transition_filter_deviation);
        CHECK(actual.wall_transition_length == expected.wall_transition_length);
        CHECK(actual.wall_transition_angle == expected.wall_transition_angle);
        CHECK(actual.wall_distribution_count == expected.wall_distribution_count);
    }
}

TEST_CASE("Concentric plugin matches the native Arachne walls", "[plugins][infill][concentric][arachne]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExPolygons narrow = union_ex(ExPolygons{rectangle_expolygon(-10., -5., -4., 5.),
        rectangle_expolygon(-4., -0.1, 4., 0.1), rectangle_expolygon(4., -5., 10., 5.)});
    const ExPolygons dumbbell = union_ex(ExPolygons{rectangle_expolygon(-12., -5., -4., 5.),
        rectangle_expolygon(-4., -1., 4., 1.), rectangle_expolygon(4., -5., 12., 5.)});
    const ExPolygon lshape(Polygon{Point(0, 0), Point(scale_i(30.), 0), Point(scale_i(30.), scale_i(10.)),
        Point(scale_i(10.), scale_i(10.)), Point(scale_i(10.), scale_i(25.)), Point(0, scale_i(25.))});
    const ExPolygons nested = union_ex(ExPolygons{rectangle_with_hole_expolygon(), rectangle_expolygon(-2., -2., 2., 2.)});
    REQUIRE(narrow.size() == 1);
    REQUIRE(dumbbell.size() == 1);
    REQUIRE(nested.size() == 2);
    ExPolygons cases{rectangle_expolygon(-10., -6.65, 10., 6.65), rectangle_with_hole_expolygon(), lshape,
                     narrow.front(), dumbbell.front()};
    cases.insert(cases.end(), nested.begin(), nested.end());

    PrintConfig print;
    print.set_deserialize_strict("nozzle_diameter", "0.4");
    const std::vector<Settings> settings_cases = {
        {}, { { "min_bead_width", "60%" }, { "wall_transition_length", "0.8" }, { "wall_distribution_count", "2" } } };

    for (const ExPolygon &polygon : cases)
        for (const Settings &settings : settings_cases)
            for (const float ratio : { 1.f, 0.8f })
                for (const coord_t clipping : { coord_t(0), scale_i(0.15) }) {
                    CAPTURE(polygon.holes.size(), polygon.area(), settings.size(), ratio, clipping);
                    const PrintRegionConfig region = region_config(settings);
                    const raw_infill_pattern_params raw = make_recipe(polygon, ratio, clipping);
                    const std::vector<std::string> expected = native_entities(polygon, raw, region, print);
                    const std::vector<std::string> actual =
                        plugin_entities(polygon, raw, Arachne::wall_tool_paths_settings(region, print));
                    REQUIRE_FALSE(expected.empty());
                    REQUIRE(actual.size() == expected.size());
                    // Each component is a run of native entities, in the native order; the narrow
                    // shape splits in two components.
                    std::vector<size_t> native_index;
                    for (const std::string &entity : actual) {
                        const std::vector<std::string>::const_iterator found =
                            std::find(expected.begin(), expected.end(), entity);
                        REQUIRE(found != expected.end());
                        native_index.push_back(size_t(found - expected.begin()));
                    }
                    std::vector<size_t> sorted = native_index;
                    std::sort(sorted.begin(), sorted.end());
                    for (size_t index = 0; index < sorted.size(); ++index)
                        CHECK(sorted[index] == index);
                    size_t runs = 1;
                    for (size_t index = 1; index < native_index.size(); ++index)
                        if (native_index[index] != native_index[index - 1] + 1)
                            ++runs;
                    CHECK(runs <= 2);
                }
}

TEST_CASE("Concentric plugin keeps classic loops for sparse Arachne recipes", "[plugins][infill][concentric][arachne]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExPolygon polygon = rectangle_with_hole_expolygon();
    raw_infill_pattern_params arachne = make_recipe(polygon, 1.f, 0);
    for (const bool sparse : { true, false }) {
        CAPTURE(sparse);
        if (sparse)
            arachne.density = 0.5f;
        else
            arachne.infill_flags |= C_INFILL_FLAG_DONT_ADJUST;
        raw_infill_pattern_params classic = arachne;
        classic.infill_flags &= uint16_t(~uint16_t(C_INFILL_FLAG_USE_ARACHNE));
        CHECK(plugin_entities(polygon, arachne, {}) == plugin_entities(polygon, classic, {}));
    }
}

TEST_CASE("Arachne concentric cancellation discards the unpublished group", "[plugins][infill][concentric][arachne][cancellation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Surface surface(stPosTop | stDensSolid, rectangle_with_hole_expolygon());
    const raw_infill_pattern_params raw = make_recipe(surface.expolygon, 1.f, 0);
    PrintConfig print;
    print.set_deserialize_strict("nozzle_diameter", "0.4");
    const Arachne::WallToolPathsSettings settings = Arachne::wall_tool_paths_settings(PrintRegionConfig(), print);
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    slic3r_api::StoredExtrusionEntity output(handle);
    const size_t initial = storage.size();
    run_ctx_infill_pattern ctx = {};
    ctx.surface = reinterpret_cast<const surface_handle *>(&surface);
    ctx.params = &raw;
    ctx.output = output.mutable_handle();
    // Cancel once the group holds a wall; the walls of this shape are closed loops.
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
    CHECK_THROWS_AS(slic3r_api::Infill::fill_concentric_surface(ctx, handle, &run, settings), slic3r_api::PluginCancelled);
    CHECK(probe.ready);
    CHECK(output.empty());
    CHECK(storage.size() == initial);
}

TEST_CASE("The Arachne perimeter generator gives concentric infill variable widths", "[plugins][infill][concentric][arachne]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::vector<float> arachne = top_infill_widths("arachne");
    const std::vector<float> classic = top_infill_widths("classic");
    REQUIRE_FALSE(arachne.empty());
    REQUIRE_FALSE(classic.empty());
    CHECK(std::all_of(classic.begin(), classic.end(), [&classic](float width) { return width == classic.front(); }));
    CHECK(std::any_of(arachne.begin(), arachne.end(), [&classic](float width) { return width != classic.front(); }));
}

TEST_CASE("Ensuring plugin matches the native FillEnsuring tree", "[plugins][infill][ensuring][arachne]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExPolygons narrow = union_ex(ExPolygons{rectangle_expolygon(-10., -5., -4., 5.),
        rectangle_expolygon(-4., -0.3, 4., 0.3), rectangle_expolygon(4., -5., 10., 5.)});
    const ExPolygon lshape(Polygon{Point(0, 0), Point(scale_i(30.), 0), Point(scale_i(30.), scale_i(10.)),
        Point(scale_i(10.), scale_i(10.)), Point(scale_i(10.), scale_i(25.)), Point(0, scale_i(25.))});
    // Sharp corners leave short sections and gaps for Arachne.
    const ExPolygon sharp(Polygon{Point(-scale_i(35.), -scale_i(25.)), Point(scale_i(35.), -scale_i(25.)),
        Point(scale_i(20.), scale_i(25.)), Point(-scale_i(15.), scale_i(15.)), Point(-scale_i(35.), scale_i(25.))});
    REQUIRE(narrow.size() == 1);
    // A thin sliver is left out: the native filler joins gap walls into duplicate points that
    // thin_variable_width() rejects with an assertion in Debug builds.
    const std::vector<ExPolygon> cases{rectangle_expolygon(-10., -6.65, 10., 6.65), rectangle_with_hole_expolygon(),
                                       lshape, narrow.front(), sharp};
    const std::vector<Frame> frames = {
        { 0., 0, true, -1. }, { M_PI / 6., 3, true, -1. }, { -M_PI / 4., 3, false, -1. }, { 0.2, 2, true, 0.7 } };

    PrintConfig print;
    print.set_deserialize_strict("nozzle_diameter", "0.4");
    const std::vector<Settings> settings_cases = {
        {}, { { "min_bead_width", "60%" }, { "wall_transition_length", "0.8" }, { "wall_distribution_count", "2" } } };

    for (const ExPolygon &polygon : cases)
        for (const Frame &frame : frames)
            for (const double overlap : { 0., 0.15 })
                for (const Settings &settings : settings_cases) {
                    CAPTURE(polygon.holes.size(), polygon.area(), frame.angle, frame.layer_id, frame.can_angle_cross,
                            frame.bridge_angle, overlap, settings.size());
                    const PrintRegionConfig region = region_config(settings);
                    const raw_infill_pattern_params raw = ensuring_recipe(polygon, frame, overlap);
                    const std::string expected = native_ensuring(polygon, raw, region, print);
                    REQUIRE_FALSE(expected.empty());
                    const std::string actual = plugin_ensuring(polygon, raw, Arachne::wall_tool_paths_settings(region, print));
                    INFO("plugin: " << actual);
                    INFO("native: " << expected);
                    CHECK(same_tree(actual, expected));
                }
}

// The tree comparison of the parity test accepts only coordinate differences up to SCALED_EPSILON.
// Any other change of the text, or a point added or removed, makes the trees different.
TEST_CASE("Ensuring tree comparison tolerates only small coordinate differences", "[plugins][infill][ensuring]")
{
    const std::string base = "SR{L{10,-20;30,40;|7|0.05|0.45|0.2}{-5,6;7,8;|8|0.06|0.4|0.2}}";
    // The base text with its first occurrence of from replaced by to.
    const auto with = [&base](const std::string &from, const std::string &to) {
        std::string text = base;
        const size_t at = text.find(from);
        REQUIRE(at != std::string::npos);
        return text.replace(at, from.size(), to);
    };
    const std::string near = std::to_string(10 + SCALED_EPSILON) + ",-20;";
    const std::string far = std::to_string(11 + SCALED_EPSILON) + ",-20;";
    const std::string near_y = "-5," + std::to_string(6 - SCALED_EPSILON) + ";";
    const std::string far_y = "-5," + std::to_string(5 - SCALED_EPSILON) + ";";
    CHECK(split_points(base).text == "SR{L{;;|7|0.05|0.45|0.2}{;;|8|0.06|0.4|0.2}}");
    CHECK(split_points(base).coordinates == std::vector<long long>{10, -20, 30, 40, -5, 6, 7, 8});
    CHECK(same_tree(base, base));
    CHECK(same_tree(with("10,-20;", near), base));
    CHECK(same_tree(base, with("10,-20;", near)));
    CHECK(same_tree(with("-5,6;", near_y), base));
    const std::vector<std::string> changed{
        with("10,-20;", far), with("-5,6;", far_y), with("30,40;", "30,40;31,40;"), with("30,40;", ""),
        with("SR{", "S{"), with("SR{", "R{"), with("L{", "{"), with("|7|", "|8|"), with("|0.05|", "|0.051|"),
        with("|0.45|", "|0.44|"), with("0.2}{", "0.21}{"), with("}{", "{"), base + "{}"};
    for (const std::string &text : changed) {
        CAPTURE(text);
        CHECK_FALSE(same_tree(text, base));
        CHECK_FALSE(same_tree(base, text));
    }
}

TEST_CASE("Ensuring cancellation discards the unpublished group", "[plugins][infill][ensuring][arachne][cancellation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Surface surface(stPosInternal | stDensSolid, rectangle_with_hole_expolygon());
    const raw_infill_pattern_params raw = ensuring_recipe(surface.expolygon, { 0., 0, true, -1. }, 0.);
    PrintConfig print;
    print.set_deserialize_strict("nozzle_diameter", "0.4");
    const Arachne::WallToolPathsSettings settings = Arachne::wall_tool_paths_settings(PrintRegionConfig(), print);
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    slic3r_api::StoredExtrusionEntity output(handle);
    const size_t initial = storage.size();
    run_ctx_infill_pattern ctx = {};
    ctx.surface = reinterpret_cast<const surface_handle *>(&surface);
    ctx.params = &raw;
    ctx.output = output.mutable_handle();
    // Cancel once an unpublished collection holds an extrusion.
    struct Probe { PluginStorage *storage; const extrusion_entity_handle *output; bool ready = false; } probe{&storage, output.handle()};
    plugin_run_context run = {};
    run.host_context = reinterpret_cast<plugin_host_context *>(&probe);
    run.is_cancelled = [](plugin_host_context *host) {
        Probe &probe = *reinterpret_cast<Probe *>(host);
        // Inspect a locked pointer snapshot; this single-threaded probe keeps the temporary entities alive.
        for (const ExtrusionEntity *owned : probe.storage->stored_extrusions()) {
            const slic3r_api::ExtrusionEntity entity(reinterpret_cast<const extrusion_entity_handle *>(owned));
            if (entity.handle() != probe.output && !entity.is_leaf() && !entity.empty()) {
                probe.ready = true;
                return 1;
            }
        }
        return 0;
    };
    CHECK_THROWS_AS(slic3r_api::Infill::fill_ensuring_surface(ctx, handle, &run, settings), slic3r_api::PluginCancelled);
    CHECK(probe.ready);
    CHECK(output.empty());
    CHECK(storage.size() == initial);
}

TEST_CASE("Ensuring is a standalone plugin and the smooth variants are gone", "[plugins][infill][ensuring]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Plugin *plugin = Orchestrator::instance().get_plugin("ensuring");
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_step() == INFILL_PATTERN);
    CHECK(Orchestrator::instance().get_plugin("smoothhilbert") == nullptr);
    CHECK(Orchestrator::instance().get_plugin("smoothtriple") == nullptr);
}

// The island of a branch sheath centreline (a long strip of about 0.3 mm) for which the Voronoi diagram of the rotated input segments has a finite edge
// with a non-finite vertex: the repair by rotation must reject that diagram and try the next angle, whatever the result. The test guarantees that:
// - the diagram of the island lines goes through the repair (its state is repaired or not repaired, not "not needed");
// - the medial axis, built with the parameters of the API call of the branch sheath (MedialAxisExtrusionFactory), returns without aborting;
// - every point of its output lies inside the bounding box of the island grown by the flow width, and every width is in 0..2 times the maximum medial width.
// An empty output is accepted: an island narrower than the nozzle yields no line.
TEST_CASE("The medial axis of a degenerate branch island stays inside the island", "[plugins][voronoi-repair][medial-axis]")
{
    const coord_t flow_width = 419999, height = 300000, min_medial = 299256, max_medial = 629999, min_extrusion = 271239;
    const std::vector<std::pair<coord_t, coord_t>> captured{
        {-4353392, -4178675}, {-4271397, -4009546}, {-440921, -4004738}, {-282523, -4103582}, {-171659, -4113497},
        {37611, -4004139}, {-4872750, -4010301}, {-4814617, -4173323}, {-4584344, -4288775}};
    Points points;
    for (const std::pair<coord_t, coord_t> &point : captured)
        points.emplace_back(point.first, point.second);
    const ExPolygon island(points);

    // The repair was attempted on the lines of the island.
    // Non-const: construct_voronoi is only instantiated for the iterators of a non-const Lines (as MedialAxis uses it).
    Lines lines = island.lines();
    Geometry::VoronoiDiagram diagram;
    diagram.construct_voronoi(lines.begin(), lines.end());
    CHECK((diagram.get_state() == Geometry::VoronoiDiagram::State::REPAIR_SUCCESSFUL ||
           diagram.get_state() == Geometry::VoronoiDiagram::State::REPAIR_UNSUCCESSFUL));

    Geometry::MedialAxis medial_axis(island, max_medial, min_medial, height);
    medial_axis.use_min_real_width(min_extrusion);
    medial_axis.set_biggest_width(max_medial);
    medial_axis.set_stop_at_min_width(true);
    ThickPolylines centrelines;
    medial_axis.build(centrelines);

    BoundingBox bounds = get_extents(island);
    bounds.offset(flow_width);
    for (const ThickPolyline &centreline : centrelines) {
        CHECK(centreline.points.size() == centreline.points_width.size());
        for (const Point &point : centreline.points)
            CHECK(bounds.contains(point));
        for (const coord_t width : centreline.points_width) {
            CHECK(width >= 0);
            CHECK(width <= 2 * max_medial);
        }
    }
}
