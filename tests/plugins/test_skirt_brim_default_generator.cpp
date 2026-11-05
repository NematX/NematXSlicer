#include <catch2/catch.hpp>

#include <initializer_list>
#include <chrono>
#include <iostream>
#include <functional>
#include <fstream>
#include <sstream>
#include <map>
#include <set>
#include <memory>
#include <string>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/PrintAccess.hpp"
#include "libslic3r/Api/internal/ExtrusionPropertyAccess.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Geometry/ConvexHull.hpp"
#include "libslic3r/ExtrusionGenerationWinding.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PluginProperty.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "libslic3r/Steps/StepSkirtBrim.hpp"

/*
Default skirt/brim generator tests
==================================

These tests exercise STEP_SKIRT_BRIM and final placement of adhesion paths.
They slice a cube, run the adhesion plugins and inspect auxiliary layers.
Winding probes compare native, canonical and wrapped stored contours, retaining
entry points, arc/Z data, command phases and independent ordering permissions.

- STEP_SKIRT_BRIM is a non-exclusive chain, while brim and skirt sub-features
  are separate exclusive groups selected by plugin id.
- The default brim/skirt plugins write global, per-object, and first-layer-only
  extrusion through tagged auxiliary layers.
- The host rebuilds the first-layer convex hull from auxiliary adhesion output.

The final-plan placement cases also run the slicing/ordering pipeline.
They identify brim by its source auxiliary layer, measure deposited-width
coverage in plateau coordinates and check each physical instance separately.

The model and Print own the input and generated layers; cases inspect their paths and final plan.

Operation notes:
- Test::init_print :  Print::process :  StepPipeline::run_gcode
- object_brim_auxiliary_layers :  collect_diagnostic_brim_coverage

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- Slic3r::Test::init_print()
        |-- prepared.print.apply()
        |-- prepared.print.process()
        |-- Steps::StepPipeline::run_gcode()
        |-- merge()
        `-- collect_diagnostic_brim_coverage()
*/

namespace {

using namespace Slic3r;

const char *const STANDARD_LAYER_HEIGHT_GENERATOR = "standard_layer_height_generator";
const char *const SLICE_VOLUME = "slice_volume";
const char *const DEFAULT_BRIM_GENERATOR = "skirt_brim.brim.default";
const char *const DEFAULT_SKIRT_GENERATOR = "skirt_brim.skirt.default";
const char *const DEFAULT_BRIM_SKIRT_TRIM = "skirt_brim.brim_skirt_trim.default";
const char *const FAILING_SKIRT_BRIM_PLUGIN = "skirt_brim.test.report_error";
const char *const BRIM_SEPARATION_READER = "skirt_brim.test.brim_separation_reader";

enum class FailingPluginPhase
{
    Setup,
    SetupRun,
    Run
};

void report_test_error(const plugin_run_context *run_context)
{
    if (run_context != nullptr && run_context->report_error != nullptr)
        run_context->report_error(run_context->host_context, "intentional step plugin error");
}

class FailingSkirtBrimPlugin final : public slic3r_api::PluginBase
{
public:
    explicit FailingSkirtBrimPlugin(orchestrator_handle *orchestrator)
        : PluginBase(orchestrator)
    {}

    void set_phase(FailingPluginPhase phase) { m_phase = phase; }

private:
    const char *id_impl() const noexcept override { return FAILING_SKIRT_BRIM_PLUGIN; }
    slicing_step_t step_impl() const noexcept override { return STEP_SKIRT_BRIM; }
    const char *const *dependencies_impl() const noexcept override
    {
        static const char *dependencies[] = {nullptr};
        return dependencies;
    }
    int32_t priority_impl() const noexcept override { return 0; }

    void setup_impl(const plugin_run_context *run_context, uint32_t) const override
    {
        if (m_phase == FailingPluginPhase::Setup)
            report_test_error(run_context);
    }

    void setup_run_impl(const plugin_run_context *run_context) const override
    {
        if (m_phase == FailingPluginPhase::SetupRun)
            report_test_error(run_context);
    }

    void run_impl(const plugin_run_context *run_context) const override
    {
        if (m_phase == FailingPluginPhase::Run)
            report_test_error(run_context);
    }

    FailingPluginPhase m_phase = FailingPluginPhase::Setup;
};

FailingSkirtBrimPlugin &failing_skirt_brim_plugin()
{
    static FailingSkirtBrimPlugin plugin(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    return plugin;
}

void register_failing_skirt_brim_plugin()
{
    Orchestrator &orchestrator = Orchestrator::instance();
    if (orchestrator.get_plugin(FAILING_SKIRT_BRIM_PLUGIN) == nullptr)
        REQUIRE(orchestrator.register_plugin(failing_skirt_brim_plugin().c_instance()));
}

// Reads brim_separation from a step that runs before support. The plugin is
// registered only after the test runtime finalized the option definitions, so
// it would move the setting's invalidation step only if plugin initialization
// still edited definitions.
class BrimSeparationReader final : public slic3r_api::PluginBase
{
public:
    explicit BrimSeparationReader(orchestrator_handle *orchestrator)
        : PluginBase(orchestrator)
    {}

private:
    const char *id_impl() const noexcept override { return BRIM_SEPARATION_READER; }
    slicing_step_t step_impl() const noexcept override { return STEP_POST_PERIMETER; }
    const char *const *dependencies_impl() const noexcept override
    {
        static const char *dependencies[] = {nullptr};
        return dependencies;
    }
    int32_t priority_impl() const noexcept override { return 0; }

    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            keys[0] = raw_used_config_key{
                "brim_separation", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_OBJECT, RAW_PRESET_TYPE_FFF_PRINT};
        return 1;
    }

    void run_impl(const plugin_run_context *) const override {}
};

class ScopedActivePlugins
{
public:
    explicit ScopedActivePlugins(std::initializer_list<const char *> plugin_ids) :
        ScopedActivePlugins(std::vector<const char *>(plugin_ids))
    {}

    explicit ScopedActivePlugins(const std::vector<const char *> &plugin_ids) :
        m_orchestrator(Orchestrator::instance())
    {
        m_previous_active_plugins.reserve(m_orchestrator.active_plugins().size());
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous_active_plugins.push_back(plugin);

        m_orchestrator.clear_active_plugins();
        for (const char *plugin_id : plugin_ids) {
            INFO("Activating test plugin " << plugin_id);
            REQUIRE(m_orchestrator.set_plugin_active(plugin_id, true));
        }
    }

    ~ScopedActivePlugins()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous_active_plugins)
            m_orchestrator.set_plugin_active(plugin, true);
        m_orchestrator.reset_plugin_cancel();
    }

private:
    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous_active_plugins;
};

DynamicPrintConfig brim_test_config(const bool brim_enabled, const bool per_object)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height", "0.2" },
        { "first_layer_height", "0.2" },
        { "brim_width", brim_enabled ? "3" : "0" },
        { "brim_width_interior", "0" },
        { "brim_per_object", per_object ? "1" : "0" },
        { "brim_ears", "0" },
        { "skirts", "0" },
        { "skirt_brim", "0" },
        { "support_material", "0" },
        { "raft_layers", "0" },
        { "perimeters", "2" },
        { "fill_density", "0" },
        { "step_layer_height_plugin", STANDARD_LAYER_HEIGHT_GENERATOR }
    });
    return config;
}

DynamicPrintConfig skirt_test_config(const int skirts, const int skirt_brim, const bool per_object)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height", "0.2" },
        { "first_layer_height", "0.2" },
        { "brim_width", "0" },
        { "brim_width_interior", "0" },
        { "brim_per_object", "0" },
        { "brim_ears", "0" },
        { "skirts", std::to_string(skirts) },
        { "skirt_height", "1" },
        { "skirt_distance", "3" },
        { "skirt_distance_from_brim", "0" },
        { "skirt_brim", std::to_string(skirt_brim) },
        { "min_skirt_length", "0" },
        { "draft_shield", "disabled" },
        { "complete_objects", per_object ? "1" : "0" },
        { "complete_objects_one_skirt", "0" },
        { "support_material", "0" },
        { "raft_layers", "0" },
        { "perimeters", "2" },
        { "fill_density", "0" },
        { "step_layer_height_plugin", STANDARD_LAYER_HEIGHT_GENERATOR }
    });
    return config;
}

struct PreparedBrimPrint
{
    Model model;
    Print print;
};

// Isolate standalone diagnostic cases from the process-wide orchestrator stop cause and diagnostics.
class ScopedBrimTestState
{
public:
    // Borrow the singleton, discard prior diagnostic state and clear cancellation before this independent case; no source data is owned and host failures propagate.
    ScopedBrimTestState()
    {
        Orchestrator::instance().consume_plugin_messages();
        Orchestrator::instance().reset_plugin_cancel();
    }
    // Clear the independent case's stop cause and messages on every exit, including Catch assertion unwinding; no Print/model lifetime is extended.
    ~ScopedBrimTestState()
    {
        Orchestrator::instance().consume_plugin_messages();
        Orchestrator::instance().reset_plugin_cancel();
    }
};

void run_prepared_print_until_skirt_brim(PreparedBrimPrint &prepared,
                                         bool include_trim = true);

void run_until_skirt_brim(PreparedBrimPrint &prepared,
                          const DynamicPrintConfig &config,
                          const bool include_trim = true)
{
    Slic3r::Test::init_print({ Slic3r::Test::TestMesh::cube_20x20x20 }, prepared.print, prepared.model, config);
    run_prepared_print_until_skirt_brim(prepared, include_trim);
}

void run_mesh_until_skirt_brim(PreparedBrimPrint &prepared,
                               const TriangleMesh &mesh,
                               const DynamicPrintConfig &config)
{
    Slic3r::Test::init_print({ mesh }, prepared.print, prepared.model, config);
    run_prepared_print_until_skirt_brim(prepared);
}

void run_prepared_print_until_skirt_brim(PreparedBrimPrint &prepared,
                                         const bool include_trim)
{
    std::vector<const char *> active_plugin_ids = {
        STANDARD_LAYER_HEIGHT_GENERATOR,
        SLICE_VOLUME,
        DEFAULT_BRIM_GENERATOR,
        DEFAULT_SKIRT_GENERATOR
    };
    if (include_trim)
        active_plugin_ids.push_back(DEFAULT_BRIM_SKIRT_TRIM);
    ScopedActivePlugins active(active_plugin_ids);

    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, prepared.print);
    Steps::StepSlicing::run_step(orchestrator, prepared.print);
    (void) orchestrator.consume_plugin_messages();
    Steps::StepSkirtBrim::run_step(orchestrator, prepared.print);
    for (const Orchestrator::PluginMessage &message : orchestrator.consume_plugin_messages()) {
        INFO("Plugin " << message.plugin_id << " reported: " << message.message);
        CHECK(message.level != Orchestrator::PluginMessageLevel::Error);
    }
}

double first_layer_slice_area_mm2(const Print &print)
{
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = print.objects().front();
    REQUIRE(object.layer_count() > 0);
    return unscaled(unscaled(std::abs(area(object.layers().front().lslices()))));
}

double hull_area_mm2(const Print &print)
{
    return unscaled(unscaled(std::abs(area(print.first_layer_convex_hull()))));
}

double extrusion_tree_length_mm(const ExtrusionEntity &entity)
{
    ArcPolylines polylines;
    entity.collect_polylines(polylines);

    double length = 0.0;
    for (const ArcPolyline &polyline : polylines)
        length += unscaled(polyline.length());
    return length;
}

std::vector<const Layer *> object_brim_auxiliary_layers(const PrintObject &object)
{
    std::vector<const Layer *> out;
    for (const Layer &layer : object.auxiliary_layers())
        if (LayerAdhesionProperty::layer_is_brim(layer))
            out.push_back(&layer);
    return out;
}

// Collect width coverage from final-plan leaves without assuming that decorated loop children are native ExtrusionPath objects.
void collect_diagnostic_brim_coverage(const ExtrusionEntity &entity, Polygons &coverage);

// Recursively borrow final-plan children, append each geometric leaf's width coverage to caller-owned polygons and skip command leaves; no tree changes or ownership transfers occur and geometry/allocation failures propagate.
void collect_diagnostic_brim_coverage(const ExtrusionEntity &entity, Polygons &coverage)
{
    if (entity.polyline_or_null() != nullptr) {
        entity.ExtrusionEntity::polygons_covered_by_width(coverage, float(SCALED_EPSILON));
        return;
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        if (child)
            collect_diagnostic_brim_coverage(*child, coverage);
}

// Build two non-reversible curved halves with distinct endpoint Z values for winding checks.
std::unique_ptr<ExtrusionLoop> winding_arc_loop();
// Gather stored paths in traversal order without invoking legacy projection views.
ArcPolyline stored_winding_geometry(const ExtrusionEntity &entity);
// Compare every arc and endpoint Z value, including the closed contour's entry.
void check_winding_geometry(const ArcPolyline &actual, const ArcPolyline &expected);

std::unique_ptr<ExtrusionLoop> winding_arc_loop()
{
    std::unique_ptr<ExtrusionLoop> loop = std::make_unique<ExtrusionLoop>(elrSkirt);
    for (const bool upper : {true, false}) {
        ArcPolyline arc;
        arc.append(upper ? Point(10000, 0) : Point(-10000, 0));
        arc.append(Geometry::ArcWelder::Segment(upper ? Point(0, 10000) : Point(0, -10000),
            10000.f, Geometry::ArcWelder::Orientation::CCW));
        arc.append(Geometry::ArcWelder::Segment(upper ? Point(-10000, 0) : Point(10000, 0),
            10000.f, Geometry::ArcWelder::Orientation::CCW));
        arc.set_z_offset(0, upper ? 100 : 900);
        arc.set_z_offset(1, upper ? 500 : 700);
        arc.set_z_offset(2, upper ? 900 : 100);
        std::unique_ptr<ExtrusionPath> path = std::make_unique<ExtrusionPath>(
            ExtrusionAttributes(ExtrusionRole::Skirt, ExtrusionFlow(0.08, 0.4f, 0.2f)), false);
        path->set_polyline(std::move(arc));
        path->add_property(ExtrusionPropertySpeed(20.f));
        loop->append_child(std::move(path));
    }
    return loop;
}

ArcPolyline stored_winding_geometry(const ExtrusionEntity &entity)
{
    if (const ArcPolyline *path = entity.polyline_or_null())
        return *path;
    ArcPolyline out;
    for (const ExtrusionEntityUPtr &child : entity.children())
        if (child)
            out.append(stored_winding_geometry(*child));
    return out;
}

void check_winding_geometry(const ArcPolyline &actual, const ArcPolyline &expected)
{
    REQUIRE(actual.size() == expected.size());
    CHECK(actual.has_z_offset() == expected.has_z_offset());
    for (size_t index = 0; index < actual.size(); ++index) {
        CHECK(actual.get_point(index) == expected.get_point(index));
        CHECK(actual.get_arc(index).radius == expected.get_arc(index).radius);
        CHECK(actual.get_arc(index).orientation == expected.get_arc(index).orientation);
        if (expected.has_z_offset())
            CHECK(actual.z_offset(index) == expected.z_offset(index));
    }
}

} // namespace

// Slice owned cube models in three placement modes, inspect borrowed final-plan BRIM sources and their width coverage, and assert one correctly placed brim per instance; processing/allocation failures propagate and all Print/model data dies at case exit.
TEST_CASE("Final plan places each generated brim around its source instance", "[plugins][brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ScopedBrimTestState state;
    for (const std::string &mode : {std::string("per-object"), std::string("sequential"), std::string("parallel"), std::string("global"), std::string("ears")}) {
        for (const size_t copies : {size_t(1), size_t(2)}) {
            CAPTURE(mode, copies);
            DynamicPrintConfig config = brim_test_config(true, mode != "global");
            config.set_deserialize_strict({
                {"complete_objects", mode == "sequential" ? "1" : "0"},
                {"parallel_objects_step", mode == "parallel" ? "1" : "0"}, {"xy_size_compensation", "0"},
                {"xy_inner_size_compensation", "0"}, {"hole_size_compensation", "0"},
                {"first_layer_size_compensation", "0"}, {"brim_separation", "0"}});
            if (mode == "ears")
                config.set_deserialize_strict({{"brim_ears", "1"}, {"brim_ears_pattern", "concentric"}});
            PreparedBrimPrint prepared;
            Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20}, prepared.print, prepared.model, config);
            ModelObject &model_object = prepared.model.objects().front();
            model_object.instances.front()->set_offset(Vec3d(40., 50., 0.));
            if (copies == 2)
                model_object.add_instance(*model_object.instances.front())->set_offset(Vec3d(100., 50., 0.));
            prepared.print.apply(prepared.model, config);
            prepared.print.process();
            const boost::filesystem::path reference = Test::GCode::temporary_gcode_path();
            const ScopeGuard remove_reference([&reference]() { Test::GCode::remove_output_pair(reference); });
            Steps::StepPipeline::run_gcode(Orchestrator::instance(), prepared.print, reference.string());
            REQUIRE(prepared.print.objects().size() == 1);
            const PrintObject &object = prepared.print.objects().front();
            REQUIRE(object.instances().size() == copies);
            REQUIRE(object.layer_count() > 0);
            const Printing::PrintingPlan *plan = prepared.print.printing_plan();
            REQUIRE(plan != nullptr);

            std::map<const LayerRegionIsland *, const Layer *> brim_sources;
            // Retain only source identities of first-layer BRIM, never skirt or normal object paths.
            for (const PrintObject *owner : {&object, prepared.print.auxiliary_object()}) {
                if (owner == nullptr)
                    continue;
                for (const Layer &layer : owner->auxiliary_layers()) {
                    if (!LayerAdhesionProperty::layer_is_brim(layer) ||
                        layer.scaled_print_z() != object.layer(0).scaled_print_z())
                        continue;
                    for (const LayerSliceIsland &island : layer.islands())
                        for (const LayerRegionIsland &region : island.regions_islands())
                            brim_sources.emplace(&region, &layer);
                }
            }
            REQUIRE_FALSE(brim_sources.empty());
            std::vector<BoundingBox> expected(copies);
            for (size_t instance = 0; instance < copies; ++instance) {
                for (const ExPolygon &slice : object.layer(0).lslices())
                    expected[instance].merge(slice.contour.points);
                expected[instance].translate(object.instances()[instance].shift);
            }
            std::vector<BoundingBox> actual(copies);
            std::vector<std::set<const Layer *>> sources(copies);
            size_t roots = 0;
            std::vector<size_t> instance_roots(copies, 0);
            for (const Printing::PrintingGroup &group : plan->groups)
                for (const Printing::PrintingLayerGroup &layer : group.layers)
                    for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                        for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                            const std::map<const LayerRegionIsland *, const Layer *>::const_iterator source =
                                brim_sources.find(extrusion.region_island);
                            if (source == brim_sources.end() || !extrusion.root)
                                continue;
                            ++roots;
                            if (extrusion.printing_catalog_id < plan->object_instances.size())
                                ++instance_roots.at(plan->object_instances[extrusion.printing_catalog_id].object_instance_idx);
                            Polygons coverage;
                            collect_diagnostic_brim_coverage(*extrusion.root, coverage);
                            for (const Polygon &polygon : coverage) {
                                BoundingBox box(polygon.points);
                                size_t instance = copies;
                                if (extrusion.printing_catalog_id < plan->object_instances.size()) {
                                    const Printing::PrintingObjectInstance &entry = plan->object_instances[extrusion.printing_catalog_id];
                                    REQUIRE(entry.object == &object);
                                    instance = entry.object_instance_idx;
                                } else {
                                    // Global brim has no catalog id: spatially assign its disconnected coverage to the nearest cube.
                                    double distance = 1e100;
                                    for (size_t candidate = 0; candidate < copies; ++candidate) {
                                        const double candidate_distance = (box.center() - expected[candidate].center()).cast<double>().squaredNorm();
                                        if (candidate_distance < distance) {
                                            distance = candidate_distance;
                                            instance = candidate;
                                        }
                                    }
                                }
                                REQUIRE(instance < copies);
                                actual[instance].merge(box);
                                sources[instance].insert(source->second);
                            }
                        }
            CHECK(roots == (mode == "global" ? 1 : copies));
            // The emitted reference uses absolute XY/E. Associate only positive-extrusion brim segments with their firmware label.
            std::ifstream gcode(reference.string());
            REQUIRE(gcode.good());
            std::vector<BoundingBox> gcode_boxes(copies);
            std::string line;
            std::string feature;
            int label = -1;
            double x = 0., y = 0., e = 0., z = 0.;
            while (std::getline(gcode, line)) {
                if (line.rfind(";TYPE:", 0) == 0)
                    feature = line.substr(6);
                if (line.rfind("; SLIC3R_OBJECT_INSTANCE_ID=", 0) == 0)
                    label = std::stoi(line.substr(line.find('=') + 1));
                if (line.rfind("G92 E", 0) == 0) {
                    e = std::stod(line.substr(5));
                    continue;
                }
                if (line.rfind("G0 ", 0) != 0 && line.rfind("G1 ", 0) != 0)
                    continue;
                double nx = x, ny = y, ne = e;
                bool xy = false;
                std::istringstream words(line.substr(3));
                std::string word;
                while (words >> word) {
                    if (word[0] == ';') break;
                    if (word[0] == 'X') { nx = std::stod(word.substr(1)); xy = true; }
                    if (word[0] == 'Y') { ny = std::stod(word.substr(1)); xy = true; }
                    if (word[0] == 'Z') z = std::stod(word.substr(1));
                    if (word[0] == 'E') ne = std::stod(word.substr(1));
                }
                if (feature == "Skirt/Brim" && z < 0.201 && xy && ne > e) {
                    size_t owner = copies;
                    if (label >= 0 && size_t(label) < plan->object_instances.size())
                        owner = plan->object_instances[size_t(label)].object_instance_idx;
                    else {
                        double distance = 1e100;
                        for (size_t index = 0; index < copies; ++index) {
                            const double d = (Point::new_scale(nx, ny) - expected[index].center()).cast<double>().squaredNorm();
                            if (d < distance) { distance = d; owner = index; }
                        }
                    }
                    REQUIRE(owner < copies);
                    gcode_boxes[owner].merge(Point::new_scale(x, y));
                    gcode_boxes[owner].merge(Point::new_scale(nx, ny));
                }
                x = nx; y = ny; e = ne;
            }
            for (size_t instance = 0; instance < copies; ++instance) {
                CAPTURE(instance);
                const BoundingBox wanted = expected[instance].inflated(scale_i(3.));
                CHECK(sources[instance].size() == 1);
                if (mode != "global")
                    CHECK(instance_roots[instance] == 1);
                CHECK(actual[instance].defined);
                // Width quantization and rounded path caps may differ from nominal brim width by less than one nozzle width.
                CHECK(std::abs(unscaled(actual[instance].min.x() - wanted.min.x())) < 0.6);
                CHECK(std::abs(unscaled(actual[instance].min.y() - wanted.min.y())) < 0.6);
                CHECK(std::abs(unscaled(actual[instance].max.x() - wanted.max.x())) < 0.6);
                CHECK(std::abs(unscaled(actual[instance].max.y() - wanted.max.y())) < 0.6);
                REQUIRE(gcode_boxes[instance].defined);
                CHECK(std::abs(unscaled(gcode_boxes[instance].min.x() - wanted.min.x())) < 1.);
                CHECK(std::abs(unscaled(gcode_boxes[instance].min.y() - wanted.min.y())) < 1.);
                CHECK(std::abs(unscaled(gcode_boxes[instance].max.x() - wanted.max.x())) < 1.);
                CHECK(std::abs(unscaled(gcode_boxes[instance].max.y() - wanted.max.y())) < 1.);
            }
        }
    }
}

// Create owned native layers and borrow SDK views to verify typed owner storage, common/selected placements and invalid-index failure without generating output; property/allocation exceptions propagate.
TEST_CASE("Auxiliary instance ownership has typed storage and validated projections", "[plugins][brim][properties]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedBrimPrint prepared;
    DynamicPrintConfig config = brim_test_config(false, false);
    Test::init_print({Test::TestMesh::cube_20x20x20}, prepared.print, prepared.model, config);
    prepared.model.objects().front().add_instance(*prepared.model.objects().front().instances.front())->set_offset(Vec3d(160., 50., 0.));
    prepared.print.apply(prepared.model, config);
    PrintObject &object = prepared.print.object(0);
    Layer &layer = object.add_auxiliary_layer(0, scale_i(.2), scale_i(.2));
    const slic3r_api::Object view(reinterpret_cast<const object_handle *>(&object));
    const slic3r_api::Layer layer_view(reinterpret_cast<const layer_handle *>(&layer));
    CHECK(layer.get_property<LayerInstanceProperty>() == nullptr);
    CHECK(layer_view.properties().get(slic3r_api::LayerInstanceProperty::key) == nullptr);
    CHECK_FALSE(slic3r_api::layer_instance_owner(layer_view).has_value());
    CHECK(slic3r_api::layer_instance_projections(view, layer_view).size() == 2);
    LayerInstanceProperty &owner = layer.get_or_add_property<LayerInstanceProperty>();
    owner.object_instance_idx = 0;
    owner.reserved = 0;
    CHECK(PLUGIN_PROPERTY_TYPE_LAYER_INSTANCE == 16);
    CHECK(orchestrator_property_byte_count(Orchestrator::instance().handle(), PLUGIN_PROPERTY_TYPE_LAYER_INSTANCE) == sizeof(c_layer_instance_property));
    CHECK(orchestrator_property_alignment(Orchestrator::instance().handle(), PLUGIN_PROPERTY_TYPE_LAYER_INSTANCE) == alignof(c_layer_instance_property));
    CHECK(std::string(orchestrator_property_name(Orchestrator::instance().handle(), PLUGIN_PROPERTY_TYPE_LAYER_INSTANCE)) == "slic3r.layer.instance");
    CHECK(plugin_property_data_size(layer_get_properties(layer_view.handle()), PLUGIN_PROPERTY_TYPE_LAYER_INSTANCE) == sizeof(c_layer_instance_property));
    const slic3r_api::LayerInstanceProperty *sdk_owner = layer_view.properties().get(slic3r_api::LayerInstanceProperty::key);
    REQUIRE(sdk_owner != nullptr);
    CHECK(sdk_owner->object_instance_idx == 0);
    CHECK(sdk_owner->reserved == 0);
    CHECK(slic3r_api::layer_instance_owner(layer_view) == std::optional<uint32_t>(0));
    CHECK(reinterpret_cast<uintptr_t>(sdk_owner) % alignof(c_layer_instance_property) == 0);
    const std::vector<std::pair<uint32_t, c_point>> placements = slic3r_api::layer_instance_projections(view, layer_view);
    REQUIRE(placements.size() == 1);
    CHECK(placements.front().first == 0);
    CHECK(placements.front().second.x == object.instances()[0].shift.x());
    owner.object_instance_idx = 2;
    CHECK_THROWS_WITH(slic3r_api::layer_instance_projections(view, layer_view), "Auxiliary layer owner instance index is outside the Object.");
}

// Generate true patch volumes on two translated copies, inspect local published slices and plateau cache order even after layer publication order is reversed; owns the model/Print and propagates generation failures.
TEST_CASE("Global brim patches retain instance ownership and ordered preview geometry", "[plugins][brim][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedBrimPrint prepared;
    DynamicPrintConfig config = brim_test_config(false, false);
    Test::init_print({Test::TestMesh::cube_20x20x20}, prepared.print, prepared.model, config);
    ModelObject &model_object = prepared.model.objects().front();
    model_object.instances.front()->set_offset(Vec3d(40., 50., 0.));
    model_object.add_instance(*model_object.instances.front())->set_offset(Vec3d(100., 50., 0.));
    TriangleMesh patch = Test::mesh(Test::TestMesh::cube_20x20x20);
    patch.scale(Vec3f(.3f, .3f, .02f));
    patch.translate(Vec3f(-4.f, 6.f, 0.f));
    ModelVolume *volume = model_object.add_volume(patch);
    volume->set_type(ModelVolumeType::BRIM_PATCH);
    prepared.print.apply(prepared.model, config);
    run_prepared_print_until_skirt_brim(prepared);
    PrintObject &object = prepared.print.object(0);
    const std::vector<const Layer *> layers = object_brim_auxiliary_layers(object);
    REQUIRE(layers.size() == 2);
    std::set<uint32_t> owners;
    for (uint32_t index = 0; index < 2; ++index) {
        const LayerInstanceProperty *owner = layers[index]->get_property<LayerInstanceProperty>();
        REQUIRE(owner != nullptr);
        owners.insert(owner->object_instance_idx);
        BoundingBox box;
        for (const ExPolygon &slice : layers[index]->lslices()) box.merge(slice.contour.points);
        CHECK(box.max.x() < scale_i(5.));
        CHECK(box.min.x() > scale_i(-20.));
    }
    CHECK((owners == std::set<uint32_t>{0, 1}));
    std::sort(object.mutable_auxiliary_layers().begin(), object.mutable_auxiliary_layers().end(),
        [](const LayerUPtr &left, const LayerUPtr &right) {
            return left->get_property<LayerInstanceProperty>()->object_instance_idx >
                right->get_property<LayerInstanceProperty>()->object_instance_idx;
        });
    const ExtrusionEntityCollection &cache = object.brim();
    REQUIRE(cache.entities().size() == 2);
    for (size_t index = 0; index < 2; ++index) {
        Polygons coverage;
        collect_diagnostic_brim_coverage(*cache.entities()[index], coverage);
        BoundingBox box;
        for (const Polygon &polygon : coverage) box.merge(polygon.points);
        CHECK(unscaled(box.min.x()) > (index == 0 ? 30. : 90.));
        CHECK(unscaled(box.max.x()) < (index == 0 ? 45. : 105.));
    }
    object.mutable_auxiliary_layers().front()->get_or_add_property<LayerInstanceProperty>().object_instance_idx = 2;
    CHECK_THROWS(Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), prepared.print));
    bool reported_owner_error = false;
    for (const Orchestrator::PluginMessage &message : Orchestrator::instance().consume_plugin_messages())
        reported_owner_error = reported_owner_error || message.message.find("Auxiliary layer owner instance index") != std::string::npos;
    CHECK(reported_owner_error);
    Orchestrator::instance().reset_plugin_cancel();
}

// Check that one shared local patch becomes one correctly placed brim per physical copy in the final plan.
TEST_CASE("Sequential and parallel brim patches reach each instance once", "[plugins][brim][skirt-brim][brim-patch-placement]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ScopedBrimTestState state;
    for (const bool sequential : {true, false}) {
        CAPTURE(sequential);
        DynamicPrintConfig config = brim_test_config(true, true);
        // Disable ordinary brim and skirt so only the model patch can satisfy the placement checks.
        config.set_deserialize_strict({{"complete_objects", sequential ? "1" : "0"},
            {"parallel_objects_step", sequential ? "0" : "1"}, {"parallel_islands", "0"},
            {"complete_objects_one_skirt", "0"}, {"skirts", "0"}, {"brim_width", "0"},
            {"brim_width_interior", "0"}, {"brim_skirt_trim", "0"}, {"xy_size_compensation", "0"},
            {"xy_inner_size_compensation", "0"}, {"first_layer_size_compensation", "0"}});
        PreparedBrimPrint prepared;
        Test::init_print({Test::TestMesh::cube_20x20x20}, prepared.print, prepared.model, config);
        ModelObject &model_object = prepared.model.objects().front();
        model_object.instances.front()->set_offset(Vec3d(40., 50., 0.));
        model_object.add_instance(*model_object.instances.front())->set_offset(Vec3d(100., 70., 0.));
        TriangleMesh patch = Test::mesh(Test::TestMesh::cube_20x20x20);
        patch.scale(Vec3f(.3f, .3f, .02f));
        patch.translate(Vec3f(-8.f, 6.f, 0.f));
        model_object.add_volume(patch)->set_type(ModelVolumeType::BRIM_PATCH);
        prepared.print.apply(prepared.model, config);
        prepared.print.process();
        const boost::filesystem::path output = Test::GCode::temporary_gcode_path();
        const ScopeGuard remove_output([&output]() { Test::GCode::remove_output_pair(output); });
        Steps::StepPipeline::run_gcode(Orchestrator::instance(), prepared.print, output.string());
        const PrintObject &object = prepared.print.object(0);
        REQUIRE(object.instances().size() == 2);
        const std::vector<const Layer *> layers = object_brim_auxiliary_layers(object);
        REQUIRE(layers.size() == 1);
        CHECK(layers.front()->get_property<LayerInstanceProperty>() == nullptr);
        // Identify the patch by its source layer rather than counting unrelated object paths in the plan.
        std::set<const LayerRegionIsland *> sources;
        for (const LayerSliceIsland &island : layers.front()->islands())
            for (const LayerRegionIsland &region : island.regions_islands()) sources.insert(&region);
        REQUIRE_FALSE(sources.empty());
        REQUIRE(prepared.print.printing_plan() != nullptr);
        const Printing::PrintingPlan &plan = *prepared.print.printing_plan();
        std::vector<BoundingBox> boxes(2);
        std::vector<size_t> counts(2, 0);
        for (const Printing::PrintingGroup &group : plan.groups)
            for (const Printing::PrintingLayerGroup &layer : group.layers)
                for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                    for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                        if (sources.count(extrusion.region_island) == 0) continue;
                        REQUIRE(extrusion.printing_catalog_id < plan.object_instances.size());
                        const size_t index = plan.object_instances[extrusion.printing_catalog_id].object_instance_idx;
                        REQUIRE(index < 2);
                        ++counts[index];
                        Polygons coverage;
                        collect_diagnostic_brim_coverage(*extrusion.root, coverage);
                        for (const Polygon &polygon : coverage) boxes[index].merge(polygon.points);
                    }
        for (size_t index = 0; index < 2; ++index) {
            CAPTURE(index);
            CHECK(counts[index] == 1);
            REQUIRE(boxes[index].defined);
            const double x = index == 0 ? 40. : 100.;
            const double y = index == 0 ? 50. : 70.;
            CHECK(std::abs(unscaled(boxes[index].min.x()) - (x - 8.)) < .6);
            CHECK(std::abs(unscaled(boxes[index].max.x()) - (x - 2.)) < .6);
            CHECK(std::abs(unscaled(boxes[index].min.y()) - (y + 6.)) < .6);
            CHECK(std::abs(unscaled(boxes[index].max.y()) - (y + 12.)) < .6);
        }
    }
}

// Generate interior adhesion on a real holed mesh with two physical copies; require local owner-specific auxiliary layers rather than a duplicated shared pattern, with model/layers owned by the case and generation failures propagated.
TEST_CASE("Interior brim retains one local layer per physical copy", "[plugins][brim][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedBrimPrint prepared;
    DynamicPrintConfig config = brim_test_config(false, true);
    config.set_deserialize_strict({{"brim_width_interior", "2"}, {"complete_objects", "0"}, {"parallel_objects_step", "0"}});
    Test::init_print({Test::mesh(Test::TestMesh::cube_with_hole, Vec3d::Zero(), Vec3d(1., 1., .02))},
        prepared.print, prepared.model, config);
    ModelObject &model_object = prepared.model.objects().front();
    model_object.instances.front()->set_offset(Vec3d(40., 50., 0.));
    model_object.add_instance(*model_object.instances.front())->set_offset(Vec3d(140., 50., 0.));
    prepared.print.apply(prepared.model, config);
    run_prepared_print_until_skirt_brim(prepared);
    const PrintObject &object = prepared.print.object(0);
    const std::vector<const Layer *> layers = object_brim_auxiliary_layers(object);
    REQUIRE(layers.size() == 2);
    std::set<uint32_t> owners;
    for (const Layer *layer : layers) {
        const LayerInstanceProperty *owner = layer->get_property<LayerInstanceProperty>();
        REQUIRE(owner != nullptr);
        owners.insert(owner->object_instance_idx);
        CHECK_FALSE(layer->lslices().empty());
    }
    CHECK((owners == std::set<uint32_t>{0, 1}));
}

// Generate neighbouring real objects so two copies have unequal brim shapes; compare projected slices, skirt reach and trimmed instance provenance using owned Prints, leaving generation failures visible.
TEST_CASE("Asymmetric instance brims retain spatial provenance through hull skirt and trim", "[plugins][brim][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ScopedBrimTestState state;
    double untrimmed_length = 0.;
    std::map<uint32_t, double> untrimmed_instance_lengths;
    for (const int scenario : {0, 1, 2}) {
        const bool trim = scenario == 1;
        const bool from_brim = scenario == 2;
        CAPTURE(trim, from_brim);
        PreparedBrimPrint prepared;
        DynamicPrintConfig config = brim_test_config(true, true);
        config.set_deserialize_strict({{"brim_width", "5"}, {"skirts", "1"}, {"skirt_distance", "1"},
            {"skirt_height", "1"}, {"min_skirt_length", "0"}, {"draft_shield", "limited"},
            {"skirt_distance_from_brim", from_brim ? "1" : "0"}, {"brim_skirt_trim", trim ? "1" : "0"},
            {"xy_size_compensation", "0"}, {"xy_inner_size_compensation", "0"}, {"first_layer_size_compensation", "0"}});
        Test::init_print({Test::mesh(Test::TestMesh::cube_20x20x20),
            Test::mesh(Test::TestMesh::cube_20x20x20, Vec3d::Zero(), Vec3d(1., 2., 1.))}, prepared.print, prepared.model, config);
        ModelObject &first = prepared.model.objects()[0];
        first.instances.front()->set_offset(Vec3d(40., 50., 0.));
        first.add_instance(*first.instances.front())->set_offset(Vec3d(100., 50., 0.));
        ModelObject &neighbour = prepared.model.objects()[1];
        neighbour.instances.front()->set_offset(Vec3d(62., 40., 0.));
        neighbour.config.set_deserialize_strict("brim_width", "0");
        prepared.print.apply(prepared.model, config);
        run_prepared_print_until_skirt_brim(prepared);
        PrintObject &object = prepared.print.object(0);
        REQUIRE(object.instances().size() == 2);
        const std::vector<const Layer *> layers = object_brim_auxiliary_layers(object);
        REQUIRE(layers.size() == 2);
        BoundingBox projected;
        std::map<uint32_t, BoundingBox> local_boxes;
        double length = 0.;
        std::map<uint32_t, double> instance_lengths;
        for (const Layer *layer : layers) {
            const LayerInstanceProperty *owner = layer->get_property<LayerInstanceProperty>();
            REQUIRE(owner != nullptr);
            REQUIRE(owner->object_instance_idx < 2);
            BoundingBox local;
            for (const ExPolygon &slice : layer->lslices()) local.merge(slice.contour.points);
            local_boxes.emplace(owner->object_instance_idx, local);
            local.translate(object.instances()[owner->object_instance_idx].shift);
            projected.merge(local);
            for (const LayerSliceIsland &island : layer->islands())
                for (const LayerRegionIsland &region : island.regions_islands())
                    if (region.has_extrusion(LayerRegionIsland::PERIMETERS)) {
                        const double contribution = extrusion_tree_length_mm(region.extrusion(LayerRegionIsland::PERIMETERS));
                        length += contribution;
                        instance_lengths[owner->object_instance_idx] += contribution;
                    }
        }
        REQUIRE(local_boxes.size() == 2);
        CHECK(local_boxes.at(0).max.x() < local_boxes.at(1).max.x() - scale_i(.5));
        CHECK(unscaled(projected.min.x()) > 34.);
        CHECK(unscaled(projected.max.x()) < 126.);
        BoundingBox hull(prepared.print.first_layer_convex_hull().points);
        CHECK(hull.contains(projected));
        CHECK(unscaled(hull.min.x()) > (from_brim ? 30. : 34.));
        CHECK(unscaled(hull.max.x()) < (from_brim ? 135. : 126.));
        Polygons skirt_coverage;
        collect_diagnostic_brim_coverage(prepared.print.skirt(), skirt_coverage);
        REQUIRE_FALSE(skirt_coverage.empty());
        BoundingBox skirt_box;
        for (const Polygon &polygon : skirt_coverage) skirt_box.merge(polygon.points);
        CHECK(unscaled(skirt_box.min.x()) > (from_brim ? 30. : 37.));
        CHECK(unscaled(skirt_box.max.x()) < (from_brim ? 135. : 123.));
        if (scenario == 0) {
            untrimmed_length = length;
            untrimmed_instance_lengths = instance_lengths;
        }
        if (trim) {
            CHECK(length < untrimmed_length);
            CHECK(instance_lengths.at(0) < untrimmed_instance_lengths.at(0));
            CHECK(instance_lengths.at(1) < untrimmed_instance_lengths.at(1));
        }
    }
}

// Slice and write two physical copies in normal/sequential/parallel modes, then compare owned preview geometry with the final plan; common layers/cache remain local until trim marks their owners, and all output files/state are cleaned on failure.
TEST_CASE("Plateau brim preview expands common patterns and trimmed owners exactly once", "[plugins][brim][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ScopedBrimTestState state;
    for (const std::string &mode : {std::string("normal"), std::string("sequential"), std::string("parallel")}) {
        for (const bool trim : {false, true}) {
            CAPTURE(mode, trim);
            DynamicPrintConfig config = brim_test_config(true, true);
            config.set_deserialize_strict({{"complete_objects", mode == "sequential" ? "1" : "0"},
                {"parallel_objects_step", mode == "parallel" ? "1" : "0"}, {"parallel_islands", "0"},
                {"complete_objects_one_skirt", "0"}, {"skirts", trim ? "1" : "0"}, {"skirt_height", "1"},
                {"skirt_distance", "1"}, {"min_skirt_length", "0"}, {"skirt_distance_from_brim", "0"},
                {"draft_shield", trim ? "limited" : "disabled"}, {"brim_skirt_trim", trim ? "1" : "0"},
                {"xy_size_compensation", "0"}, {"xy_inner_size_compensation", "0"}, {"first_layer_size_compensation", "0"}});
            PreparedBrimPrint prepared;
            Test::init_print({Test::TestMesh::cube_20x20x20}, prepared.print, prepared.model, config);
            ModelObject &model_object = prepared.model.objects().front();
            model_object.instances.front()->set_offset(Vec3d(40., 50., 0.));
            model_object.add_instance(*model_object.instances.front())->set_offset(Vec3d(100., 50., 0.));
            prepared.print.apply(prepared.model, config);
            prepared.print.process();
            const boost::filesystem::path reference = Test::GCode::temporary_gcode_path();
            const ScopeGuard remove_reference([&reference]() { Test::GCode::remove_output_pair(reference); });
            Steps::StepPipeline::run_gcode(Orchestrator::instance(), prepared.print, reference.string());
            const PrintObject &object = prepared.print.object(0);
            REQUIRE(object.instances().size() == 2);
            const bool marked = mode == "normal" || trim;
            std::set<const LayerRegionIsland *> sources;
            std::set<uint32_t> owners;
            for (const Layer *layer : object_brim_auxiliary_layers(object)) {
                const LayerInstanceProperty *owner = layer->get_property<LayerInstanceProperty>();
                CHECK((owner != nullptr) == marked);
                if (owner) owners.insert(owner->object_instance_idx);
                for (const LayerSliceIsland &island : layer->islands())
                    for (const LayerRegionIsland &region : island.regions_islands())
                        sources.insert(&region);
            }
            if (marked) CHECK((owners == std::set<uint32_t>{0, 1}));
            const ExtrusionEntityCollection preview = object.brim_in_plateau_coordinates();
            REQUIRE(preview.entities().size() == 2);
            std::vector<BoundingBox> expected(2), preview_boxes(2), plan_boxes(2);
            std::vector<size_t> preview_counts(2, 0), plan_counts(2, 0);
            for (size_t index = 0; index < 2; ++index) {
                for (const ExPolygon &slice : object.layer(0).lslices()) expected[index].merge(slice.contour.points);
                expected[index].translate(object.instances()[index].shift);
                expected[index].offset(scale_i(3.));
            }
            for (const ExtrusionEntity *root : preview.entities()) {
                Polygons coverage;
                collect_diagnostic_brim_coverage(*root, coverage);
                BoundingBox box;
                for (const Polygon &polygon : coverage) box.merge(polygon.points);
                REQUIRE(box.defined);
                const size_t index = box.center().x() < scale_i(80.) ? 0 : 1;
                ++preview_counts[index];
                preview_boxes[index].merge(box);
            }
            REQUIRE(prepared.print.printing_plan() != nullptr);
            const Printing::PrintingPlan &plan = *prepared.print.printing_plan();
            for (const Printing::PrintingGroup &group : plan.groups)
                for (const Printing::PrintingLayerGroup &layer : group.layers)
                    for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                        for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                            if (sources.count(extrusion.region_island) == 0) continue;
                            REQUIRE(extrusion.printing_catalog_id < plan.object_instances.size());
                            const size_t index = plan.object_instances[extrusion.printing_catalog_id].object_instance_idx;
                            REQUIRE(index < 2);
                            ++plan_counts[index];
                            Polygons coverage;
                            collect_diagnostic_brim_coverage(*extrusion.root, coverage);
                            for (const Polygon &polygon : coverage) plan_boxes[index].merge(polygon.points);
                        }
            for (size_t index = 0; index < 2; ++index) {
                CAPTURE(index);
                CHECK(preview_counts[index] == 1);
                CHECK(plan_counts[index] == 1);
                CHECK(preview_boxes[index] == plan_boxes[index]);
                CHECK(std::abs(unscaled(preview_boxes[index].min.x() - expected[index].min.x())) < .6);
                CHECK(std::abs(unscaled(preview_boxes[index].max.x() - expected[index].max.x())) < .6);
                CHECK(std::abs(unscaled(preview_boxes[index].min.y() - expected[index].min.y())) < .6);
                CHECK(std::abs(unscaled(preview_boxes[index].max.y() - expected[index].max.y())) < .6);
            }
            if (!marked) {
                Polygons coverage;
                collect_diagnostic_brim_coverage(object.brim(), coverage);
                BoundingBox local;
                for (const Polygon &polygon : coverage) local.merge(polygon.points);
                CHECK(std::abs(unscaled(local.center().x())) < .01);
                CHECK(std::abs(unscaled(local.center().y())) < .01);
                CHECK(local.max.x() < scale_i(14.));
            }
        }
    }
}

// Generate a common local brim and object-local skirt with distance-from-brim disabled, then prove the skirt hull still encloses the real brim; normal mode uses one plateau skirt root and no origin-space ghost.
TEST_CASE("Object local skirt includes brim without draft shield or distance flag", "[plugins][brim][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ScopedBrimTestState state;
    for (const bool sequential : {true, false}) {
        CAPTURE(sequential);
        DynamicPrintConfig config = brim_test_config(true, true);
        config.set_deserialize_strict({{"complete_objects", sequential ? "1" : "0"}, {"complete_objects_one_skirt", "0"},
            {"parallel_objects_step", "0"}, {"skirts", "1"}, {"skirt_height", "1"}, {"skirt_distance", "1"},
            {"min_skirt_length", "0"}, {"skirt_distance_from_brim", "0"}, {"draft_shield", "disabled"},
            {"brim_skirt_trim", "0"}});
        PreparedBrimPrint prepared;
        Test::init_print({Test::TestMesh::cube_20x20x20}, prepared.print, prepared.model, config);
        ModelObject &model_object = prepared.model.objects().front();
        model_object.instances.front()->set_offset(Vec3d(40., 50., 0.));
        model_object.add_instance(*model_object.instances.front())->set_offset(Vec3d(100., 50., 0.));
        prepared.print.apply(prepared.model, config);
        run_prepared_print_until_skirt_brim(prepared);
        const PrintObject &object = prepared.print.object(0);
        Polygons brim_coverage, skirt_coverage;
        collect_diagnostic_brim_coverage(sequential ? object.brim() : object.brim_in_plateau_coordinates(), brim_coverage);
        collect_diagnostic_brim_coverage(sequential ? object.skirt() : prepared.print.skirt(), skirt_coverage);
        REQUIRE_FALSE(brim_coverage.empty());
        REQUIRE_FALSE(skirt_coverage.empty());
        Points skirt_points;
        for (const Polygon &polygon : skirt_coverage)
            skirt_points.insert(skirt_points.end(), polygon.points.begin(), polygon.points.end());
        const Polygon skirt_hull = Geometry::convex_hull(skirt_points);
        for (const Polygon &polygon : brim_coverage)
            for (const Point &point : polygon.points)
                CHECK(skirt_hull.contains(point));
        BoundingBox skirt_box(skirt_hull.points);
        if (sequential) {
            CHECK(std::abs(unscaled(skirt_box.center().x())) < .01);
            CHECK(std::abs(unscaled(skirt_box.center().y())) < .01);
        } else {
            CHECK(unscaled(skirt_box.min.x()) > 35.);
            CHECK(unscaled(skirt_box.max.x()) < 125.);
            REQUIRE(prepared.print.auxiliary_object() != nullptr);
            size_t skirts = 0;
            for (const Layer &layer : prepared.print.auxiliary_object()->auxiliary_layers())
                if (LayerAdhesionProperty::layer_is_normal_skirt(layer)) ++skirts;
            CHECK(skirts == 1);
        }
    }
}

// Keep safe local skirts and warn instead of publishing skirts that enclose global brim or cross another object.
TEST_CASE("Object local skirts reject foreign first layer geometry", "[plugins][brim][skirt-brim][skirt-global-placement]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ScopedBrimTestState state;
    for (const std::string &mode : {std::string("global"), std::string("single-global"), std::string("close"), std::string("normal")}) {
    CAPTURE(mode);
    Orchestrator::instance().consume_plugin_messages();
    const bool global = mode == "global" || mode == "single-global";
    const size_t object_count = mode == "single-global" ? 1 : 2;
    DynamicPrintConfig config = brim_test_config(mode != "close", mode == "normal");
    config.set_deserialize_strict({{"complete_objects", "1"}, {"complete_objects_one_skirt", "0"},
        {"parallel_objects_step", "0"}, {"skirts", "1"}, {"skirt_height", "1"}, {"skirt_distance", "1"},
        {"min_skirt_length", "0"}, {"skirt_distance_from_brim", "0"}, {"draft_shield", "disabled"},
        {"brim_skirt_trim", "0"}, {"xy_size_compensation", "0"}, {"xy_inner_size_compensation", "0"},
        {"first_layer_size_compensation", "0"}});
    PreparedBrimPrint prepared;
    std::vector<TriangleMesh> meshes(object_count, Test::mesh(Test::TestMesh::cube_20x20x20));
    Test::init_print(std::move(meshes), prepared.print, prepared.model, config);
    REQUIRE(prepared.model.objects().size() == object_count);
    prepared.model.objects()[0].name = "local-skirt-cube-A";
    prepared.model.objects()[0].instances.front()->set_offset(Vec3d(40., 50., 0.));
    if (object_count == 2) {
        prepared.model.objects()[1].name = "local-skirt-cube-B";
        prepared.model.objects()[1].instances.front()->set_offset(mode == "close" ? Vec3d(61., 50., 0.) : Vec3d(100., 70., 0.));
    }
    prepared.print.apply(prepared.model, config);
    prepared.print.process();
    std::vector<Orchestrator::PluginMessage> warnings;
    for (const Orchestrator::PluginMessage &message : Orchestrator::instance().consume_plugin_messages())
        if (message.level == Orchestrator::PluginMessageLevel::Warning &&
            message.message.find("Local skirt omitted") != std::string::npos) warnings.push_back(message);
    CHECK(warnings.size() == (mode == "normal" ? 0 : 1));
    for (const Orchestrator::PluginMessage &warning : warnings) {
        CHECK(warning.message.find("instance 1") != std::string::npos);
        CHECK(warning.message.find("local-skirt-cube-A") != std::string::npos);
        if (object_count == 2) CHECK(warning.message.find("local-skirt-cube-B") != std::string::npos);
        CHECK(warning.message.find(global ? "global first-layer geometry" : "another object or instance") != std::string::npos);
    }
    const boost::filesystem::path output = Test::GCode::temporary_gcode_path();
    const ScopeGuard remove_output([&output]() { Test::GCode::remove_output_pair(output); });
    Steps::StepPipeline::run_gcode(Orchestrator::instance(), prepared.print, output.string());
    REQUIRE(prepared.print.objects().size() == object_count);
    REQUIRE(prepared.print.auxiliary_object() != nullptr);
    size_t global_brim_layers = 0;
    for (const Layer &layer : prepared.print.auxiliary_object()->auxiliary_layers())
        if (LayerAdhesionProperty::layer_is_brim(layer)) ++global_brim_layers;
    Polygons brim_coverage;
    collect_diagnostic_brim_coverage(prepared.print.brim(), brim_coverage);
    CHECK(global_brim_layers == (global ? 1 : 0));
    CHECK(brim_coverage.empty() == !global);
    REQUIRE(prepared.print.printing_plan() != nullptr);
    const Printing::PrintingPlan &plan = *prepared.print.printing_plan();
    for (size_t index = 0; index < object_count; ++index) {
        CAPTURE(index, global_brim_layers);
        const PrintObject &object = prepared.print.object(index);
        std::set<const LayerRegionIsland *> sources;
        for (const Layer &layer : object.auxiliary_layers())
            if (LayerAdhesionProperty::layer_is_normal_skirt(layer))
                for (const LayerSliceIsland &island : layer.islands())
                    for (const LayerRegionIsland &region : island.regions_islands()) sources.insert(&region);
        if (mode != "normal") {
            CHECK(sources.empty());
        } else {
            REQUIRE_FALSE(sources.empty());
        }
        BoundingBox actual, expected;
        size_t count = 0;
        for (const ExPolygon &slice : object.layer(0).lslices()) expected.merge(slice.contour.points);
        expected.translate(object.instances().front().shift);
        expected.offset(scale_i(4.));
        // Source layers distinguish the local skirt from the shared brim and the object extrusion.
        for (const Printing::PrintingGroup &group : plan.groups)
            for (const Printing::PrintingLayerGroup &layer : group.layers)
                for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                    for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                        if (sources.count(extrusion.region_island) == 0) continue;
                        ++count;
                        Polygons coverage;
                        collect_diagnostic_brim_coverage(*extrusion.root, coverage);
                        for (const Polygon &polygon : coverage) actual.merge(polygon.points);
                    }
        if (mode != "normal") {
            CHECK(count == 0);
            continue;
        }
        REQUIRE(count > 0);
        REQUIRE(actual.defined);
        CAPTURE(unscaled(actual.min.x()), unscaled(actual.max.x()), unscaled(actual.min.y()), unscaled(actual.max.y()));
        CHECK(std::abs(unscaled(actual.min.x() - expected.min.x())) < 1.);
        CHECK(std::abs(unscaled(actual.max.x() - expected.max.x())) < 1.);
        CHECK(std::abs(unscaled(actual.min.y() - expected.min.y())) < 1.);
        CHECK(std::abs(unscaled(actual.max.y() - expected.max.y())) < 1.);
    }
    }
}

// Reject only the conflicting copies of a shared object and publish the surviving skirt with its instance owner.
TEST_CASE("Local skirt rejection is per physical instance", "[plugins][brim][skirt-brim][skirt-local-instances]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ScopedBrimTestState state;
    Orchestrator::instance().consume_plugin_messages();
    DynamicPrintConfig config = skirt_test_config(1, 0, true);
    config.set_deserialize_strict({{"skirt_distance", "1"}, {"brim_skirt_trim", "0"}});
    PreparedBrimPrint prepared;
    Test::init_print({Test::TestMesh::cube_20x20x20}, prepared.print, prepared.model, config);
    ModelObject &model = prepared.model.objects().front();
    model.name = "three-copies";
    model.instances.front()->set_offset(Vec3d(40., 50., 0.));
    model.add_instance(*model.instances.front())->set_offset(Vec3d(61., 50., 0.));
    model.add_instance(*model.instances.front())->set_offset(Vec3d(120., 50., 0.));
    prepared.print.apply(prepared.model, config);
    prepared.print.process();
    size_t warnings = 0;
    for (const Orchestrator::PluginMessage &message : Orchestrator::instance().consume_plugin_messages()) {
        if (message.level != Orchestrator::PluginMessageLevel::Warning ||
            message.message.find("Local skirt omitted") == std::string::npos) continue;
        ++warnings;
        CHECK(message.message.find("three-copies") != std::string::npos);
        CHECK(message.message.find("instance 1") != std::string::npos);
        CHECK(message.message.find("instance 2") != std::string::npos);
        CHECK(message.message.find("instance 3") == std::string::npos);
    }
    CHECK(warnings == 1);
    const boost::filesystem::path output = Test::GCode::temporary_gcode_path();
    const ScopeGuard remove_output([&output]() { Test::GCode::remove_output_pair(output); });
    Steps::StepPipeline::run_gcode(Orchestrator::instance(), prepared.print, output.string());
    const PrintObject &object = prepared.print.object(0);
    std::set<const LayerRegionIsland *> sources;
    size_t skirts = 0;
    for (const Layer &layer : object.auxiliary_layers()) {
        if (!LayerAdhesionProperty::layer_is_normal_skirt(layer)) continue;
        ++skirts;
        const LayerInstanceProperty *owner = layer.get_property<LayerInstanceProperty>();
        REQUIRE(owner != nullptr);
        CHECK(owner->object_instance_idx == 2);
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands()) sources.insert(&region);
    }
    CHECK(skirts == 1);
    REQUIRE(prepared.print.printing_plan() != nullptr);
    const Printing::PrintingPlan &plan = *prepared.print.printing_plan();
    size_t paths = 0;
    for (const Printing::PrintingGroup &group : plan.groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                    if (sources.count(extrusion.region_island) == 0) continue;
                    ++paths;
                    REQUIRE(extrusion.printing_catalog_id < plan.object_instances.size());
                    CHECK(plan.object_instances[extrusion.printing_catalog_id].object_instance_idx == 2);
                }
    CHECK(paths == 1);
}

// Compare local and shared envelopes on a stepped object so upper slices and draft-shield brim rules agree.
TEST_CASE("Local skirt preserves configured height and draft shield brim rules", "[plugins][brim][skirt-brim][skirt-local-height]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ScopedBrimTestState state;
    for (const std::string &shield : {std::string("disabled"), std::string("limited"), std::string("enabled")}) {
        CAPTURE(shield);
        std::vector<Polygons> coverage;
        for (const bool local : {true, false}) {
            DynamicPrintConfig config = skirt_test_config(1, 0, local);
            config.set_deserialize_strict({{"skirt_height", "4"}, {"skirt_distance", "1"},
                {"draft_shield", shield}, {"brim_width", shield == "disabled" ? "0" : "6"}, {"brim_per_object", "1"},
                {"skirt_distance_from_brim", "0"}, {"brim_skirt_trim", "0"},
                {"xy_size_compensation", "0"}, {"xy_inner_size_compensation", "0"},
                {"first_layer_size_compensation", "0"}});
            TriangleMesh foot = Test::mesh(Test::TestMesh::cube_20x20x20);
            foot.scale(Vec3f(.5f, .5f, .02f));
            TriangleMesh top = Test::mesh(Test::TestMesh::cube_20x20x20);
            top.scale(Vec3f(1.f, 1.f, .02f));
            top.translate(Vec3f(-5.f, -5.f, .3f));
            foot.merge(top);
            PreparedBrimPrint prepared;
            Test::init_print({foot}, prepared.print, prepared.model, config);
            prepared.model.objects().front().instances.front()->set_offset(Vec3d(40., 50., 0.));
            prepared.print.apply(prepared.model, config);
            run_prepared_print_until_skirt_brim(prepared, false);
            const PrintObject &object = prepared.print.object(0);
            REQUIRE(object.layer_count() >= 3);
            BoundingBox first, upper;
            for (const ExPolygon &slice : object.layer(0).lslices()) first.merge(slice.contour.points);
            for (const ExPolygon &slice : object.layer(2).lslices()) upper.merge(slice.contour.points);
            REQUIRE(upper.size().x() > first.size().x());
            Polygons paths;
            collect_diagnostic_brim_coverage(local ? object.skirt() : prepared.print.skirt(), paths);
            if (!local)
                for (Polygon &path : paths) path.translate(-object.instances().front().shift);
            REQUIRE_FALSE(paths.empty());
            coverage.push_back(std::move(paths));
        }
        // Shared generation for one instance retains the same height and brim rules, with only its placement removed.
        CHECK(diff_ex(coverage[0], coverage[1]).empty());
        CHECK(diff_ex(coverage[1], coverage[0]).empty());
    }
}

// Time only the adhesion step on well-separated copies with a shared local brim; slicing and checks are outside timing.
TEST_CASE("Local skirt many copy timing", "[.][skirt-local-cost]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ScopedBrimTestState state;
    const ScopedActivePlugins active({STANDARD_LAYER_HEIGHT_GENERATOR, SLICE_VOLUME, DEFAULT_BRIM_GENERATOR, DEFAULT_SKIRT_GENERATOR});
    for (const size_t copies : {size_t(25), size_t(100)}) {
        DynamicPrintConfig config = skirt_test_config(1, 0, true);
        config.set_deserialize_strict({{"brim_width", "3"}, {"brim_per_object", "1"}, {"skirt_distance", "1"},
            {"brim_skirt_trim", "0"}});
        TriangleMesh mesh = Test::mesh(Test::TestMesh::cube_20x20x20);
        mesh.scale(Vec3f(1.f, 1.f, .02f));
        PreparedBrimPrint prepared;
        Test::init_print({mesh}, prepared.print, prepared.model, config);
        ModelObject &object = prepared.model.objects().front();
        object.instances.front()->set_offset(Vec3d(20., 20., 0.));
        for (size_t index = 1; index < copies; ++index)
            object.add_instance(*object.instances.front())->set_offset(Vec3d(20. + 40. * (index % 10), 20. + 40. * (index / 10), 0.));
        prepared.print.apply(prepared.model, config);
        Steps::StepLayerHeightGeneration::run_step(Orchestrator::instance(), prepared.print);
        Steps::StepSlicing::run_step(Orchestrator::instance(), prepared.print);
        for (size_t round = 0; round < 1; ++round) {
            Steps::StepSkirtBrim::clear(prepared.print);
            const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            Steps::StepSkirtBrim::run_step(Orchestrator::instance(), prepared.print);
            const std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            REQUIRE(prepared.print.object(0).instances().size() == copies);
            REQUIRE_FALSE(prepared.print.object(0).skirt().empty());
            std::cout << "SKIRT_LOCAL_COST," << copies << ',' << round << ','
                      << std::chrono::duration<double, std::milli>(end - start).count() << std::endl;
        }
    }
}

TEST_CASE("STEP_SKIRT_BRIM stops immediately when a plugin reports an error",
          "[plugins][steps][errors]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_failing_skirt_brim_plugin();

    struct Case {
        FailingPluginPhase phase;
        const char *phase_name;
    };

    const Case cases[] = {
        {FailingPluginPhase::Setup, "setup"},
        {FailingPluginPhase::SetupRun, "setup_run"},
        {FailingPluginPhase::Run, "run"}
    };

    for (const Case &test_case : cases) {
        DYNAMIC_SECTION("report_error during " << test_case.phase_name) {
            Orchestrator &orchestrator = Orchestrator::instance();
            orchestrator.reset_plugin_cancel();
            failing_skirt_brim_plugin().set_phase(test_case.phase);
            ScopedActivePlugins active({FAILING_SKIRT_BRIM_PLUGIN});
            Print print;

            try {
                Steps::StepSkirtBrim::run_step(orchestrator, print);
                FAIL("STEP_SKIRT_BRIM should throw after plugin report_error().");
            } catch (const RuntimeError &error) {
                const std::string message = error.what();
                CHECK(message.find("STEP_SKIRT_BRIM stopped after plugin '") != std::string::npos);
                CHECK(message.find(FAILING_SKIRT_BRIM_PLUGIN) != std::string::npos);
                CHECK(message.find("reported an error during " + std::string(test_case.phase_name)) !=
                      std::string::npos);
            }

            CHECK(orchestrator.is_plugin_cancelled());
        }
    }
}

TEST_CASE("STEP_SKIRT_BRIM exposes a brim exclusive group but is not exclusive itself",
          "[plugins][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *plugin = orchestrator.get_plugin(DEFAULT_BRIM_GENERATOR);
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_step() == STEP_SKIRT_BRIM);
    CHECK(plugin->get_exclusive_group() == "skirt_brim.brim");

    Plugin *skirt_plugin = orchestrator.get_plugin(DEFAULT_SKIRT_GENERATOR);
    REQUIRE(skirt_plugin != nullptr);
    CHECK(skirt_plugin->get_step() == STEP_SKIRT_BRIM);
    CHECK(skirt_plugin->get_exclusive_group() == "skirt_brim.skirt");

    Plugin *trim_plugin = orchestrator.get_plugin(DEFAULT_BRIM_SKIRT_TRIM);
    REQUIRE(trim_plugin != nullptr);
    CHECK(trim_plugin->get_step() == STEP_SKIRT_BRIM);
    CHECK(trim_plugin->get_exclusive_group() == "skirt_brim.brim_skirt_trim");

    bool found_brim_setting_fragment = false;
    bool found_trim_setting_fragment = false;
    const std::vector<Orchestrator::PluginUiFragment> print_fragments =
        orchestrator.ui_fragments_for_file("print.ui");
    for (const Orchestrator::PluginUiFragment &fragment : print_fragments) {
        if (fragment.fragment_id == "skirt_brim.brim.default.settings") {
            found_brim_setting_fragment = true;
            CHECK(fragment.content.find("insert$aftersetting$brim_width:brim_inside_holes") !=
                  std::string::npos);
            CHECK(fragment.content.find("insert$aftersetting$brim_width_interior:Brim ears") !=
                  std::string::npos);
            CHECK(fragment.content.find("tags$Simple:sidetext_width$0:brim_ears") !=
                  std::string::npos);
        }
        if (fragment.fragment_id == "brim_skirt_trim") {
            found_trim_setting_fragment = true;
            CHECK(fragment.content.find("group:Brim") != std::string::npos);
            CHECK(fragment.content.find("insert$aftersetting$brim_per_object:brim_skirt_trim") !=
                  std::string::npos);
        }
    }
    CHECK(found_brim_setting_fragment);
    CHECK(found_trim_setting_fragment);

    bool ears_require_outer_brim = false;
    bool inside_holes_require_outer_brim = false;
    bool inside_holes_require_no_inner_brim = false;
    bool pattern_requires_ears = false;
    bool detection_requires_ears = false;
    const auto has_clause = [](const Orchestrator::PluginGuiRule &rule, const char *condition_key,
                               raw_gui_rule_condition condition) {
        for (const Orchestrator::PluginGuiRuleClause &clause : rule.clauses)
            if (clause.condition == condition && clause.condition_key == condition_key)
                return true;
        return false;
    };
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules()) {
        ears_require_outer_brim = ears_require_outer_brim ||
            (rule.target_key == "brim_ears" &&
             has_clause(rule, "brim_width", RAW_GUI_RULE_CONDITION_VALUE_NON_ZERO));
        inside_holes_require_outer_brim = inside_holes_require_outer_brim ||
            (rule.target_key == "brim_inside_holes" &&
             has_clause(rule, "brim_width", RAW_GUI_RULE_CONDITION_VALUE_NON_ZERO));
        inside_holes_require_no_inner_brim = inside_holes_require_no_inner_brim ||
            (rule.target_key == "brim_inside_holes" &&
             has_clause(rule, "brim_width_interior", RAW_GUI_RULE_CONDITION_BOOL_FALSE));
        pattern_requires_ears = pattern_requires_ears ||
            (rule.target_key == "brim_ears_pattern" &&
             has_clause(rule, "brim_ears", RAW_GUI_RULE_CONDITION_BOOL_TRUE));
        detection_requires_ears = detection_requires_ears ||
            (rule.target_key == "brim_ears_detection_length" &&
             has_clause(rule, "brim_ears", RAW_GUI_RULE_CONDITION_BOOL_TRUE));
    }
    CHECK(ears_require_outer_brim);
    CHECK(inside_holes_require_outer_brim);
    CHECK(inside_holes_require_no_inner_brim);
    CHECK(pattern_requires_ears);
    CHECK(detection_requires_ears);

    const std::map<slicing_step_t, Steps::StepExclusiveIdentity> &exclusive_steps = Steps::get_exclusive_steps();
    CHECK(exclusive_steps.find(STEP_SKIRT_BRIM) == exclusive_steps.end());
}

TEST_CASE("Default brim generator owns its serialized geometry options", "[plugins][skirt-brim][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const ConfigOptionDef *inside_holes = PrintConfigDef::instance().get("brim_inside_holes");
    const ConfigOptionDef *ears = PrintConfigDef::instance().get("brim_ears");
    const ConfigOptionDef *max_angle = PrintConfigDef::instance().get("brim_ears_max_angle");
    const ConfigOptionDef *detection_length = PrintConfigDef::instance().get("brim_ears_detection_length");
    const ConfigOptionDef *pattern = PrintConfigDef::instance().get("brim_ears_pattern");
    const ConfigOptionDef *separation = PrintConfigDef::instance().get("brim_separation");
    REQUIRE(inside_holes != nullptr);
    REQUIRE(ears != nullptr);
    REQUIRE(max_angle != nullptr);
    REQUIRE(detection_length != nullptr);
    REQUIRE(pattern != nullptr);
    REQUIRE(separation != nullptr);

    /*
    Leaving invalidates_step at STEP_NONE in the raw definitions asks the
    orchestrator to resolve it to the owning plugin's step.
    */
    for (const ConfigOptionDef *def : { inside_holes, ears, max_angle, detection_length, pattern }) {
        CHECK(def->container_type == ConfigOptionContainerType::Object);
        CHECK(def->option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
        CHECK(def->invalidates_step == STEP_SKIRT_BRIM);
    }

    CHECK(inside_holes->type == coBool);
    CHECK(inside_holes->default_value->serialize() == "0");
    CHECK(ears->type == coBool);
    CHECK(ears->default_value->serialize() == "0");
    CHECK(max_angle->type == coFloat);
    CHECK(max_angle->min == 0.0);
    CHECK(max_angle->max == 180.0);
    CHECK(max_angle->default_value->serialize() == "125");
    CHECK(detection_length->type == coFloat);
    CHECK(detection_length->min == 0.0);
    CHECK(detection_length->default_value->serialize() == "1");
    CHECK(pattern->type == coEnum);
    CHECK(pattern->gui_type == ConfigOptionDef::GUIType::select_close);
    CHECK(pattern->has_enum_value("concentric"));
    CHECK(pattern->has_enum_value("rectilinear"));
    CHECK_FALSE(pattern->has_enum_value("grid"));
    CHECK(pattern->default_value->serialize() == "concentric");
    CHECK(separation->type == coFloat);
    CHECK(separation->container_type == ConfigOptionContainerType::Object);
    CHECK(separation->option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    // Support material also reads this option and executes before skirt/brim,
    // so finalizing the plugin definitions tightens the replay point by
    // execution_order().
    CHECK(separation->invalidates_step == STEP_SUPPORT);
    CHECK(separation->sidetext == "mm");
    CHECK(separation->default_value->serialize() == "0");
    REQUIRE(separation->aliases.size() == 1);
    CHECK(separation->aliases.front() == "brim_offset");

    /*
    brim_separation is editable while a brim is requested on either shell, in
    the print scope: one reason per accepted case.
    */
    Orchestrator &orchestrator = Orchestrator::instance();
    size_t separation_rules = 0;
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules()) {
        if (rule.owner != DEFAULT_BRIM_GENERATOR || rule.target_key != "brim_separation")
            continue;
        ++separation_rules;
        CHECK(rule.scope == RAW_PRESET_TYPE_FFF_PRINT);
    }
    CHECK(separation_rules == 2);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "brim_inside_holes", "1" },
        { "brim_ears", "1" },
        { "brim_ears_max_angle", "73.5" },
        { "brim_ears_detection_length", "2.25" },
        { "brim_ears_pattern", "rectilinear" }
    });
    CHECK(config.option("brim_inside_holes")->serialize() == "1");
    CHECK(config.option("brim_ears")->serialize() == "1");
    CHECK(config.option("brim_ears_max_angle")->serialize() == "73.5");
    CHECK(config.option("brim_ears_detection_length")->serialize() == "2.25");
    CHECK(config.option("brim_ears_pattern")->serialize() == "rectilinear");
}

TEST_CASE("Re-running plugin initialization leaves finalized invalidation steps unchanged",
          "[plugins][skirt-brim][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    static BrimSeparationReader reader(reinterpret_cast<orchestrator_handle *>(&orchestrator));
    if (orchestrator.get_plugin(BRIM_SEPARATION_READER) == nullptr)
        REQUIRE(orchestrator.register_plugin(reader.c_instance()));

    const ConfigOptionDef *separation = PrintConfigDef::instance().get("brim_separation");
    REQUIRE(separation != nullptr);
    REQUIRE(separation->invalidates_step == STEP_SUPPORT);

    {
        // Validation accepts the reader, whose step runs before support.
        // Tightening happens only in finalize_plugin_option_defs(), which the
        // finalized runtime never runs again.
        ScopedActivePlugins active({BRIM_SEPARATION_READER});
        orchestrator.initialize_plugins();
        CHECK(orchestrator.is_plugin_active(BRIM_SEPARATION_READER));
    }
    CHECK(separation->invalidates_step == STEP_SUPPORT);
}

TEST_CASE("Default skirt generator owns its serialized option and activation rules",
          "[plugins][skirt-brim][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const ConfigOptionDef *one_skirt = PrintConfigDef::instance().get("complete_objects_one_skirt");
    REQUIRE(one_skirt != nullptr);

    CHECK(one_skirt->container_type == ConfigOptionContainerType::Plater);
    CHECK(one_skirt->option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    CHECK(one_skirt->invalidates_step == STEP_SKIRT_BRIM);
    CHECK(one_skirt->type == coBool);
    CHECK(one_skirt->default_value->serialize() == "0");
    CHECK(PrintConfig::defaults().option("complete_objects_one_skirt") != nullptr);

    /*
    The plugin owns one activation reason per accepted case, in the print scope:
    sequential printing or the parallel-objects stepping both need one skirt.
    */
    Orchestrator &orchestrator = Orchestrator::instance();
    size_t rule_count = 0;
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules()) {
        if (rule.owner != DEFAULT_SKIRT_GENERATOR || rule.target_key != "complete_objects_one_skirt")
            continue;
        ++rule_count;
        CHECK(rule.scope == RAW_PRESET_TYPE_FFF_PRINT);
    }
    CHECK(rule_count == 2);
}

TEST_CASE("Default skirt/brim plugins are active by default for plugin tests", "[plugins][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    std::vector<Plugin *> plugins = Orchestrator::instance().get_active_plugins_for_step(STEP_SKIRT_BRIM);
    bool found_default_brim = false;
    bool found_default_skirt = false;
    bool found_default_trim = false;
    for (Plugin *plugin : plugins)
        found_default_brim = found_default_brim || (plugin != nullptr && plugin->get_id() == DEFAULT_BRIM_GENERATOR);
    for (Plugin *plugin : plugins)
        found_default_skirt = found_default_skirt || (plugin != nullptr && plugin->get_id() == DEFAULT_SKIRT_GENERATOR);
    for (Plugin *plugin : plugins)
        found_default_trim = found_default_trim || (plugin != nullptr && plugin->get_id() == DEFAULT_BRIM_SKIRT_TRIM);
    CHECK(found_default_brim);
    CHECK(found_default_skirt);
    CHECK(found_default_trim);
}

TEST_CASE("Default brim generator leaves output empty when brim is disabled", "[plugins][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedBrimPrint prepared;
    run_until_skirt_brim(prepared, brim_test_config(false, false));

    CHECK(prepared.print.brim().empty());
    REQUIRE(prepared.print.objects().size() == 1);
    CHECK(prepared.print.objects().front().brim().empty());
    CHECK(object_brim_auxiliary_layers(prepared.print.objects().front()).empty());
}

TEST_CASE("Default brim generator creates global brim and expands the first-layer hull",
          "[plugins][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedBrimPrint prepared;
    run_until_skirt_brim(prepared, brim_test_config(true, false));

    CHECK_FALSE(prepared.print.brim().empty());
    REQUIRE(prepared.print.objects().size() == 1);
    CHECK(prepared.print.objects().front().brim().empty());

    /*
    The hull is rebuilt by the host after the plugin chain. If the callback
    output were not considered, the hull would stay near the object slice area.
    */
    CHECK(hull_area_mm2(prepared.print) > first_layer_slice_area_mm2(prepared.print));
}

TEST_CASE("Default brim generator applies its ear geometry options", "[plugins][skirt-brim]")
{
    DynamicPrintConfig concentric_config = brim_test_config(true, false);
    concentric_config.set_deserialize_strict({
        { "brim_ears", "1" },
        { "brim_ears_max_angle", "125" },
        { "brim_ears_detection_length", "1" },
        { "brim_ears_pattern", "concentric" }
    });
    PreparedBrimPrint concentric;
    run_until_skirt_brim(concentric, concentric_config);
    REQUIRE_FALSE(concentric.print.brim().empty());

    DynamicPrintConfig rectilinear_config = concentric_config;
    rectilinear_config.set_deserialize_strict({ { "brim_ears_pattern", "rectilinear" } });
    PreparedBrimPrint rectilinear;
    run_until_skirt_brim(rectilinear, rectilinear_config);
    REQUIRE_FALSE(rectilinear.print.brim().empty());
    CHECK(extrusion_tree_length_mm(rectilinear.print.brim()) !=
          Approx(extrusion_tree_length_mm(concentric.print.brim())));

    /*
    Zero is the documented off value for the angular detector. Reaching an
    empty output here proves that the plugin value reaches make_brim_ears()
    instead of falling back to the former static PrintObjectConfig member.
    */
    DynamicPrintConfig disabled_angle_config = concentric_config;
    disabled_angle_config.set_deserialize_strict({ { "brim_ears_max_angle", "0" } });
    PreparedBrimPrint disabled_angle;
    run_until_skirt_brim(disabled_angle, disabled_angle_config);
    CHECK(disabled_angle.print.brim().empty());
}

TEST_CASE("Default brim generator applies brim inside holes", "[plugins][skirt-brim]")
{
    DynamicPrintConfig outside_only_config = brim_test_config(true, false);
    outside_only_config.set_deserialize_strict({ { "brim_inside_holes", "0" } });
    PreparedBrimPrint outside_only;
    run_mesh_until_skirt_brim(
        outside_only,
        Slic3r::Test::mesh(Slic3r::Test::TestMesh::cube_with_hole, Vec3d::Zero(), Vec3d(1., 1., 0.02)),
        outside_only_config);
    REQUIRE_FALSE(outside_only.print.brim().empty());

    DynamicPrintConfig inside_holes_config = outside_only_config;
    inside_holes_config.set_deserialize_strict({ { "brim_inside_holes", "1" } });
    PreparedBrimPrint inside_holes;
    run_mesh_until_skirt_brim(
        inside_holes,
        Slic3r::Test::mesh(Slic3r::Test::TestMesh::cube_with_hole, Vec3d::Zero(), Vec3d(1., 1., 0.02)),
        inside_holes_config);
    REQUIRE_FALSE(inside_holes.print.brim().empty());

    /*
    Enabling the option preserves holes while the object outline is converted
    into the area from which brim loops are derived. The resulting tree must
    therefore contain additional centerline around the model's opening.
    */
    CHECK(extrusion_tree_length_mm(inside_holes.print.brim()) >
          extrusion_tree_length_mm(outside_only.print.brim()));
}

TEST_CASE("Default brim generator can publish object-owned brim", "[plugins][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedBrimPrint prepared;
    run_until_skirt_brim(prepared, brim_test_config(true, true));

    CHECK(prepared.print.brim().empty());
    REQUIRE(prepared.print.objects().size() == 1);
    PrintObject &object = prepared.print.objects().front();
    CHECK_FALSE(object.brim().empty());
    CHECK(hull_area_mm2(prepared.print) > first_layer_slice_area_mm2(prepared.print));

    const std::vector<const Layer *> brim_layers = object_brim_auxiliary_layers(object);
    REQUIRE(brim_layers.size() == 1);
    const Layer &brim_layer = *brim_layers.front();
    const LayerAdhesionProperty *adhesion = brim_layer.get_property<LayerAdhesionProperty>();
    REQUIRE(adhesion != nullptr);
    CHECK(adhesion->is_brim());
    CHECK(brim_layer.get_property<LayerSupportProperty>() == nullptr);
    CHECK_FALSE(brim_layer.lslices().empty());
    REQUIRE_FALSE(brim_layer.islands().empty());
    REQUIRE_FALSE(brim_layer.island(0).regions_islands().empty());
    const LayerRegionIsland &region_island = brim_layer.island(0).regions_island(0);
    REQUIRE(region_island.has_extrusion(LayerRegionIsland::PERIMETERS));
    CHECK_FALSE(region_island.extrusion(LayerRegionIsland::PERIMETERS).empty());

    Steps::StepSkirtBrim::clear(prepared.print);
    CHECK(object.brim().empty());
    CHECK(object_brim_auxiliary_layers(object).empty());
}

// Inspect generated auxiliary geometry before ordering: each closed brim parent
// owns the same complete attributes as its unchanged segment-bearing path.
TEST_CASE("Generated brim loop parents copy complete path attributes",
          "[plugins][skirt-brim][regression][seambrim-source]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedBrimPrint prepared;
    DynamicPrintConfig config = brim_test_config(true, true);
    config.set_deserialize_strict({{"draft_shield", "disabled"}, {"brim_skirt_trim", "0"}});
    run_until_skirt_brim(prepared, config);
    PrintObject &object = prepared.print.objects().front();
    const std::vector<const Layer *> brim_layers = object_brim_auxiliary_layers(object);
    REQUIRE(brim_layers.size() == 1);
    const LayerRegionIsland &island = brim_layers.front()->island(0).regions_island(0);
    REQUIRE(island.has_extrusion(LayerRegionIsland::PERIMETERS));
    const slic3r_api::ExtrusionEntity root(reinterpret_cast<const extrusion_entity_handle *>(
        &island.extrusion(LayerRegionIsland::PERIMETERS)));
    size_t loop_count = 0;
    std::function<void(const slic3r_api::ExtrusionEntity &)> inspect;
    // Borrow generated nodes recursively, compare copied values, and count only
    // SKIRT-marked closed parents; assertions report absent or changed properties.
    inspect = [&](const slic3r_api::ExtrusionEntity &entity) {
        const c_extrusion_property_perimeter *perimeter = static_cast<const c_extrusion_property_perimeter *>(
            entity.property_data(EXTRUSION_PROPERTY_TYPE_PERIMETER));
        if (entity.is_loop() && perimeter != nullptr &&
            (perimeter->perimeter_flags & C_EXTRUSION_PERIMETER_FLAG_SKIRT) != 0) {
            ++loop_count;
            CHECK(entity.segment_count() == 0);
            REQUIRE(entity.child_count() == 1);
            const c_extrusion_property_attributes *parent = static_cast<const c_extrusion_property_attributes *>(
                entity.property_data(EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
            const c_extrusion_property_attributes *leaf = static_cast<const c_extrusion_property_attributes *>(
                entity.child(0).property_data(EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
            REQUIRE(leaf != nullptr);
            REQUIRE(parent != nullptr);
            CHECK(parent != leaf);
            CHECK(parent->role == RAW_EXTRUSION_ROLE_SKIRT);
            CHECK(parent->role == leaf->role);
            CHECK(parent->width == leaf->width);
            CHECK(parent->height == leaf->height);
            CHECK(parent->mm3_per_mm == leaf->mm3_per_mm);
            CHECK(parent->no_seam == leaf->no_seam);
            CHECK(leaf->width > 0);
            CHECK(leaf->height > 0);
            CHECK(leaf->mm3_per_mm > 0);
        }
        for (uint32_t index = 0; index < entity.child_count(); ++index)
            inspect(entity.child(index));
    };
    inspect(root);
    REQUIRE(loop_count > 1);
}

TEST_CASE("Default skirt generator creates global skirt", "[plugins][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedBrimPrint prepared;
    run_until_skirt_brim(prepared, skirt_test_config(1, 0, false));

    CHECK(prepared.print.brim().empty());
    CHECK_FALSE(prepared.print.skirt().empty());
    REQUIRE(prepared.print.objects().size() == 1);
    CHECK(prepared.print.objects().front().skirt().empty());
    CHECK(hull_area_mm2(prepared.print) > first_layer_slice_area_mm2(prepared.print));
}

TEST_CASE("Clockwise skirt normalization works with one or two nonreversible loops",
          "[plugins][skirt-brim][source-winding][source-winding-reproduction]")
{
    using Access = ApiInternal::ExtrusionPropertyAccess;
    for (const bool per_object : {false, true}) {
        for (const int skirts : {1, 2}) {
            CAPTURE(per_object, skirts);
            PreparedBrimPrint prepared;
            DynamicPrintConfig config = skirt_test_config(skirts, 0, per_object);
            config.set_deserialize_strict("perimeter_direction", "cw_ccw");
            run_until_skirt_brim(prepared, config, false);
            const PrintObject *owner = per_object ? &prepared.print.object(0) : prepared.print.auxiliary_object();
            REQUIRE(owner != nullptr);
            std::vector<const ExtrusionEntity *> leaves;
            const auto collect = [&leaves](auto &&self, const ExtrusionEntity &entity) -> void {
                if (entity.polyline_or_null() != nullptr) {
                    leaves.push_back(&entity);
                    return;
                }
                for (const ExtrusionEntityUPtr &child : entity.children())
                    if (child)
                        self(self, *child);
            };
            for (const Layer &layer : owner->auxiliary_layers()) {
                if (!LayerAdhesionProperty::layer_is_normal_skirt(layer))
                    continue;
                for (const LayerSliceIsland &island : layer.islands())
                    for (const LayerRegionIsland &region : island.regions_islands())
                        if (region.has_extrusion(LayerRegionIsland::PERIMETERS))
                            collect(collect, region.extrusion(LayerRegionIsland::PERIMETERS));
            }
            REQUIRE(leaves.size() == size_t(skirts));
            std::vector<ExtrusionEntityUPtr> snapshots;
            for (const ExtrusionEntity *leaf : leaves) {
                REQUIRE(leaf->polyline_ref().front() == leaf->polyline_ref().back());
                CHECK(Polygon(leaf->polyline_ref().to_polyline().points).is_clockwise());
                CHECK_FALSE(leaf->can_reverse());
                snapshots.push_back(leaf->clone_canonical());
            }
            ApiInternal::PrintAccess::normalize_skirt_brim_direction(prepared.print);
            for (size_t index = 0; index < leaves.size(); ++index) {
                const ExtrusionEntity &leaf = *leaves[index];
                const ExtrusionEntity &snapshot = *snapshots[index];
                CHECK(leaf.get_id() == snapshot.get_id());
                CHECK(leaf.polyline_ref().front() == snapshot.polyline_ref().front());
                CHECK(Polygon(leaf.polyline_ref().to_polyline().points).is_clockwise());
                CHECK(leaf.can_sort() == snapshot.can_sort());
                CHECK(leaf.can_reverse() == snapshot.can_reverse());
                CHECK(leaf.can_rotate() == snapshot.can_rotate());
                CHECK(Access::same_property_payload(leaf, EXTRUSION_PROPERTY_TYPE_ATTRIBUTES, snapshot));
                CHECK(Access::same_property_payload(leaf, EXTRUSION_PROPERTY_TYPE_PERIMETER, snapshot));
                CHECK(leaf.polyline_ref().to_polyline().points == snapshot.polyline_ref().to_polyline().points);
            }
        }
    }
}

TEST_CASE("PrintAccess winding treats stored brim contours equally without changing ordering permissions",
          "[plugins][skirt-brim][source-winding]")
{
    using Access = ApiInternal::ExtrusionPropertyAccess;
    for (const std::string direction : {"cw_ccw", "cw_cw", "ccw_cw", "ccw_ccw"}) {
        for (const std::string form : {"local", "native", "canonical", "collection", "event-wrapper"}) {
            CAPTURE(direction, form);
            PreparedBrimPrint prepared;
            DynamicPrintConfig config = brim_test_config(true, true);
            config.set_deserialize_strict("perimeter_direction", direction);
            run_until_skirt_brim(prepared, config, false);
            const std::vector<const Layer *> layers = object_brim_auxiliary_layers(prepared.print.object(0));
            REQUIRE(layers.size() == 1);
            Layer &layer = const_cast<Layer &>(*layers.front());
            REQUIRE_FALSE(layer.islands().empty());
            REQUIRE_FALSE(layer.island(0).regions_islands().empty());
            ExtrusionEntityCollection &bucket = layer.island(0).regions_island(0).mutable_extrusion(LayerRegionIsland::PERIMETERS);
            bucket.clear();
            std::unique_ptr<ExtrusionLoop> source = winding_arc_loop();
            const ArcPolyline original = stored_winding_geometry(*source);
            std::unique_ptr<ExtrusionLoop> expected_loop = std::make_unique<ExtrusionLoop>(*source);
            const bool clockwise = direction.rfind("cw_", 0) == 0;
            if (clockwise)
                expected_loop->reverse(); // Existing generation behavior is the oracle.
            const ArcPolyline expected = stored_winding_geometry(*expected_loop);
            ExtrusionEntityUPtr root;
            if (form == "local") {
                root = std::make_unique<ExtrusionEntity>(false, original);
                root->add_property(ExtrusionAttributes(ExtrusionRole::Skirt, ExtrusionFlow(0.08, 0.4f, 0.2f)));
            } else if (form == "native") {
                root.reset(source->clone());
            } else if (form == "canonical") {
                root = source->clone_canonical();
            } else if (form == "collection") {
                std::unique_ptr<ExtrusionEntityCollection> collection = std::make_unique<ExtrusionEntityCollection>();
                collection->set_can_sort_reverse(false, false);
                collection->append_child(ExtrusionEntityUPtr(source->clone()));
                (void)collection->entities(); // Exercise a primed legacy pointer cache.
                root = std::move(collection);
            } else {
                root = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
                root->set_can_rotate(false);
                ExtrusionEntityUPtr event = std::make_unique<ExtrusionEntity>(false);
                event->add_property(ExtrusionPropertyCustomGcodeText("M117 unmoved phase"));
                root->append_child(std::move(event));
                root->append_child(source->clone_canonical());
            }
            const ExtrusionEntity *identity = root.get();
            const ExtrusionEntity *event = form == "event-wrapper" ? &root->child(0) : nullptr;
            const size_t id = root->get_id();
            const bool sortable = root->can_sort(), reversible = root->can_reverse(), rotatable = root->can_rotate();
            const ExtrusionEntityUPtr before = root->clone_canonical();
            bucket.append(std::move(root));
            // A real non-adhesion object layer is outside this consumer's scope.
            Layer &ordinary = prepared.print.object(0).layers().front();
            REQUIRE_FALSE(ordinary.islands().empty());
            LayerSliceIsland &ordinary_island = ordinary.island(0);
            // Slicing alone has not produced extrusion role buckets yet. Build
            // the non-adhesion witness explicitly instead of indexing a missing one.
            ExtrusionEntityCollection &untouched = ordinary_island.get_or_add_region_island(ordinary_island.regions(), 0)
                .mutable_extrusion(LayerRegionIsland::PERIMETERS);
            untouched.clear();
            untouched.append(source->clone_canonical());
            ApiInternal::PrintAccess::normalize_skirt_brim_direction(prepared.print);
            REQUIRE(bucket.child_count() == 1);
            const ExtrusionEntity &actual = bucket.child(0);
            CHECK(&actual == identity);
            CHECK(actual.get_id() == id);
            CHECK(actual.can_sort() == sortable);
            CHECK(actual.can_reverse() == reversible);
            CHECK(actual.can_rotate() == rotatable);
            check_winding_geometry(stored_winding_geometry(actual), expected);
            check_winding_geometry(stored_winding_geometry(*source), original);
            check_winding_geometry(stored_winding_geometry(untouched), original);
            CHECK(Access::property_count(actual) == Access::property_count(*before));
            for (const extrusion_property_type type : {EXTRUSION_PROPERTY_TYPE_PERIMETER, EXTRUSION_PROPERTY_TYPE_ATTRIBUTES}) {
                const bool present = Access::has_property(actual, type);
                CHECK(present == Access::has_property(*before, type));
                if (present)
                    CHECK(Access::same_property_payload(actual, type, *before));
            }
            if (event != nullptr) {
                CHECK(&actual.child(0) == event);
                REQUIRE(event->get_property<ExtrusionPropertyCustomGcode>() != nullptr);
                CHECK(event->custom_gcode_string(*event->get_property<ExtrusionPropertyCustomGcode>()) == "M117 unmoved phase");
            }
            ApiInternal::PrintAccess::normalize_skirt_brim_direction(prepared.print);
            check_winding_geometry(stored_winding_geometry(actual), expected);
        }
    }
}

TEST_CASE("Generation winding reports fragmented event contours without moving or consuming them",
          "[plugins][skirt-brim][source-winding]")
{
    for (const bool one_shot : {false, true}) {
        CAPTURE(one_shot);
        std::unique_ptr<ExtrusionLoop> source = winding_arc_loop();
        const ExtrusionEntity &view = *source;
        ExtrusionEntityUPtr root = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
        root->append_child(ExtrusionEntityUPtr(view.child(0).clone()));
        if (one_shot) {
            REQUIRE(ApiInternal::ExtrusionPropertyAccess::get_or_add_property_data_mutable(root->child(0),
                EXTRUSION_PROPERTY_TYPE_HEATER, sizeof(c_extrusion_property_heater), alignof(c_extrusion_property_heater)) != nullptr);
        } else {
            ExtrusionEntityUPtr event = std::make_unique<ExtrusionEntity>(false);
            event->add_property(ExtrusionPropertyCustomGcodeText("M117 fragmented phase"));
            root->append_child(std::move(event));
        }
        root->append_child(ExtrusionEntityUPtr(view.child(1).clone()));
        const ArcPolyline before = stored_winding_geometry(*root);
        const ExtrusionEntityUPtr snapshot = root->clone_canonical();
        std::vector<const ExtrusionEntity *> children;
        for (const ExtrusionEntityUPtr &child : static_cast<const ExtrusionEntity &>(*root).children())
            children.push_back(child.get());
        const ExtrusionTreeTransforms::GenerationWindingResult result = ExtrusionTreeTransforms::normalize_generation_winding(*root, true);
        CHECK(result.reversed_contours == 0);
        CHECK(result.deferred_contours == 1);
        REQUIRE(root->child_count() == children.size());
        for (size_t index = 0; index < children.size(); ++index) {
            CHECK(&root->child(index) == children[index]);
            CHECK(root->child(index).get_id() == snapshot->child(index).get_id());
            CHECK(root->child(index).can_sort() == snapshot->child(index).can_sort());
            CHECK(root->child(index).can_reverse() == snapshot->child(index).can_reverse());
            CHECK(root->child(index).can_rotate() == snapshot->child(index).can_rotate());
            using Access = ApiInternal::ExtrusionPropertyAccess;
            CHECK(Access::property_count(root->child(index)) == Access::property_count(snapshot->child(index)));
            for (size_t property = 0; property < Access::property_count(snapshot->child(index)); ++property)
                CHECK(Access::same_property_payload(root->child(index),
                    Access::property_type_at(snapshot->child(index), property), snapshot->child(index)));
        }
        check_winding_geometry(stored_winding_geometry(*root), before);
        CHECK(root->can_rotate());
        if (!one_shot) {
            const ExtrusionPropertyCustomGcode *property = root->child(1).get_property<ExtrusionPropertyCustomGcode>();
            REQUIRE(property != nullptr);
            CHECK(root->child(1).custom_gcode_string(*property) == "M117 fragmented phase");
        }
    }
    ExtrusionNop positioned;
    positioned.set_can_sort_reverse(false, false);
    positioned.position = Point(1234, 5678);
    positioned.set_polyline(stored_winding_geometry(*winding_arc_loop()));
    const ArcPolyline before = positioned.polyline_ref();
    const ExtrusionTreeTransforms::GenerationWindingResult result = ExtrusionTreeTransforms::normalize_generation_winding(positioned, true);
    CHECK(result.reversed_contours == 1);
    CHECK(result.deferred_contours == 0);
    CHECK(positioned.position == Point(1234, 5678));
    ArcPolyline expected = before;
    expected.reverse();
    check_winding_geometry(positioned.polyline_ref(), expected);
    ExtrusionEntity open(false, ArcPolyline(Points{{0, 0}, {10000, 0}, {10000, 10000}}));
    const ArcPolyline unchanged = open.polyline_ref();
    const ExtrusionTreeTransforms::GenerationWindingResult no_loop = ExtrusionTreeTransforms::normalize_generation_winding(open, true);
    CHECK(no_loop.reversed_contours == 0);
    CHECK(no_loop.deferred_contours == 0);
    check_winding_geometry(open.polyline_ref(), unchanged);
}

TEST_CASE("Generation winding reverses authorized contours together with their command phases",
          "[plugins][skirt-brim][source-winding][tree-reverse]")
{
    for (const bool wrapped : {false, true}) {
        CAPTURE(wrapped);
        std::unique_ptr<ExtrusionLoop> source = winding_arc_loop();
        std::unique_ptr<ExtrusionNop> contour = std::make_unique<ExtrusionNop>();
        contour->position = Point(1234, 5678);
        contour->set_can_sort_reverse(false, true);
        contour->set_can_rotate(false);
        ExtrusionEntityUPtr before_event = std::make_unique<ExtrusionEntity>(false);
        before_event->add_property(ExtrusionPropertyCustomGcodeText("M117 contour before"));
        ExtrusionEntity *before = before_event.get();
        contour->append_child(std::move(before_event));
        contour->append_child(ExtrusionEntityUPtr(source->child(0).clone()));
        ExtrusionEntityUPtr after_event = std::make_unique<ExtrusionEntity>(false);
        after_event->add_property(ExtrusionPropertyCustomGcodeText("M117 contour after"));
        ExtrusionEntity *after = after_event.get();
        contour->append_child(std::move(after_event));
        contour->append_child(ExtrusionEntityUPtr(source->child(1).clone()));
        contour->set_can_sort_reverse(false, true);
        REQUIRE(contour->ExtrusionEntity::is_loop());
        ExtrusionNop *target = contour.get();
        ExtrusionEntityUPtr root;
        const ExtrusionEntity *outer_event = nullptr;
        if (wrapped) {
            root = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
            root->append_child(std::make_unique<ExtrusionEntity>(false));
            outer_event = &root->child(0);
            root->append_child(std::move(contour));
        } else root = std::move(contour);
        ArcPolyline expected = stored_winding_geometry(*root);
        expected.reverse();
        const ExtrusionTreeTransforms::GenerationWindingResult result = ExtrusionTreeTransforms::normalize_generation_winding(*root, true);
        CHECK(result.reversed_contours == 1);
        CHECK(result.deferred_contours == 0);
        check_winding_geometry(stored_winding_geometry(*root), expected);
        CHECK(&target->child(1) == after);
        CHECK(&target->child(3) == before);
        CHECK(target->position == Point(1234, 5678));
        CHECK(target->can_reverse());
        CHECK_FALSE(target->can_rotate());
        CHECK(before->custom_gcode_string(*before->get_property<ExtrusionPropertyCustomGcode>()) == "M117 contour before");
        CHECK(after->custom_gcode_string(*after->get_property<ExtrusionPropertyCustomGcode>()) == "M117 contour after");
        if (wrapped) CHECK(&root->child(0) == outer_event);
        const ExtrusionTreeTransforms::GenerationWindingResult again = ExtrusionTreeTransforms::normalize_generation_winding(*root, true);
        CHECK(again.reversed_contours == 0);
        CHECK(again.deferred_contours == 0);
        check_winding_geometry(stored_winding_geometry(*root), expected);
    }
}

TEST_CASE("Generation winding keeps target one shots at their entry and uses exact stored closure",
          "[plugins][skirt-brim][source-winding]")
{
    using Access = ApiInternal::ExtrusionPropertyAccess;
    for (const bool wrapped : {false, true}) {
        CAPTURE(wrapped);
        ExtrusionEntityUPtr geometry = std::make_unique<ExtrusionEntity>(false, stored_winding_geometry(*winding_arc_loop()));
        geometry->set_can_rotate(false);
        REQUIRE(Access::get_or_add_property_data_mutable(*geometry, EXTRUSION_PROPERTY_TYPE_HEATER,
            sizeof(c_extrusion_property_heater), alignof(c_extrusion_property_heater)) != nullptr);
        const ExtrusionEntityUPtr snapshot = geometry->clone_canonical();
        ArcPolyline expected = geometry->polyline_ref();
        expected.reverse();
        ExtrusionEntity *target = geometry.get();
        ExtrusionEntityUPtr root;
        if (wrapped) {
            root = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
            root->set_can_rotate(false);
            ExtrusionEntityUPtr event = std::make_unique<ExtrusionEntity>(false);
            event->add_property(ExtrusionPropertyCustomGcodeText("M117 stationary target phase"));
            root->append_child(std::move(event));
            root->append_child(std::move(geometry));
        } else {
            root = std::move(geometry);
        }
        const ExtrusionEntity *event = wrapped ? &root->child(0) : nullptr;
        const ExtrusionTreeTransforms::GenerationWindingResult result = ExtrusionTreeTransforms::normalize_generation_winding(*root, true);
        CHECK(result.reversed_contours == 1);
        CHECK(result.deferred_contours == 0);
        CHECK(target->get_id() == snapshot->get_id());
        CHECK_FALSE(target->can_rotate());
        CHECK_FALSE(target->can_reverse());
        CHECK(Access::same_property_payload(*target, EXTRUSION_PROPERTY_TYPE_HEATER, *snapshot));
        check_winding_geometry(target->polyline_ref(), expected);
        if (event != nullptr) {
            CHECK(&root->child(0) == event);
            REQUIRE(event->get_property<ExtrusionPropertyCustomGcode>() != nullptr);
            CHECK(event->custom_gcode_string(*event->get_property<ExtrusionPropertyCustomGcode>()) == "M117 stationary target phase");
        }
    }
    std::unique_ptr<ExtrusionLoop> loop = winding_arc_loop();
    ArcPolyline original = stored_winding_geometry(*loop);
    ExtrusionPath path(ExtrusionAttributes{ExtrusionRole::Skirt}, false);
    path.set_polyline(original);
    ExtrusionMultiPath fixed(path);
    REQUIRE_FALSE(fixed.is_loop());
    REQUIRE(fixed.ExtrusionEntity::is_loop());
    REQUIRE_FALSE(fixed.can_rotate());
    const ExtrusionTreeTransforms::GenerationWindingResult result = ExtrusionTreeTransforms::normalize_generation_winding(fixed, true);
    CHECK(result.reversed_contours == 1);
    CHECK(result.deferred_contours == 0);
    CHECK_FALSE(fixed.can_rotate());
    original.reverse();
    check_winding_geometry(stored_winding_geometry(fixed), original);

    ExtrusionPath near_path(ExtrusionAttributes{ExtrusionRole::Skirt}, false);
    near_path.set_polyline(ArcPolyline(Points{{0, 0}, {10000, 0}, {10000, 10000}, {1, 0}}));
    ExtrusionLoop near(near_path);
    REQUIRE(near.is_loop());
    REQUIRE_FALSE(near.ExtrusionEntity::is_loop());
    const ArcPolyline before = stored_winding_geometry(near);
    const ExtrusionTreeTransforms::GenerationWindingResult not_closed = ExtrusionTreeTransforms::normalize_generation_winding(near, true);
    CHECK(not_closed.reversed_contours == 0);
    CHECK(not_closed.deferred_contours == 0);
    check_winding_geometry(stored_winding_geometry(near), before);
}

TEST_CASE("Default skirt generator can publish object-owned skirt", "[plugins][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedBrimPrint prepared;
    run_until_skirt_brim(prepared, skirt_test_config(1, 0, true));

    CHECK(prepared.print.skirt().empty());
    REQUIRE(prepared.print.objects().size() == 1);
    CHECK_FALSE(prepared.print.objects().front().skirt().empty());
    CHECK(hull_area_mm2(prepared.print) > first_layer_slice_area_mm2(prepared.print));
}

TEST_CASE("Default skirt generator publishes skirt-brim as first-layer-only skirt", "[plugins][skirt-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedBrimPrint prepared;
    run_until_skirt_brim(prepared, skirt_test_config(0, 2, false));

    CHECK(prepared.print.skirt().empty());
    REQUIRE(prepared.print.skirt_first_layer().has_value());
    CHECK_FALSE(prepared.print.skirt_first_layer()->empty());
    CHECK(hull_area_mm2(prepared.print) > first_layer_slice_area_mm2(prepared.print));
}

TEST_CASE("Default brim/skirt trim setting controls draft-shield brim trimming", "[plugins][skirt-brim]")
{
    DynamicPrintConfig config = skirt_test_config(1, 0, false);
    config.set_deserialize_strict({
        { "brim_width", "5" },
        { "draft_shield", "limited" },
        { "skirt_distance", "1" },
        { "brim_skirt_trim", "0" }
    });

    PreparedBrimPrint untrimmed;
    run_until_skirt_brim(untrimmed, config, true);
    REQUIRE_FALSE(untrimmed.print.brim().empty());
    REQUIRE_FALSE(untrimmed.print.skirt().empty());

    config.set_deserialize_strict({
        { "brim_skirt_trim", "1" }
    });

    PreparedBrimPrint trimmed;
    run_until_skirt_brim(trimmed, config, true);
    REQUIRE_FALSE(trimmed.print.brim().empty());
    REQUIRE_FALSE(trimmed.print.skirt().empty());

    /*
    The legacy trim exists for this exact configuration: a draft shield is
    allowed to stand inside the configured brim width. The trim plugin keeps
    printable brim pieces, but removes the part occupied by the final skirt
    band, so total brim centerline length must decrease.
    */
    CHECK(extrusion_tree_length_mm(trimmed.print.brim()) <
          extrusion_tree_length_mm(untrimmed.print.brim()));
}
