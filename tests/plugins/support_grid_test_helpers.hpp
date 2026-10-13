/* These support tests run the production slicing prefix explicitly so native
   legacy support can be called before auxiliary layers are frozen. Small box
   assemblies share vertices only through merged meshes, and remain one print
   object. Path measurement sums geometric line volume and the explicit volume of stationary deposits, before the
   extrusion multiplier. Helpers do not change process-wide plugin activation. */
#ifndef support_grid_test_helpers_hpp_
#define support_grid_test_helpers_hpp_
#include <catch2/catch.hpp>
#include <map>
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Plugins/Support/Common/SupportLayers.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"
#include "libslic3r/Plugins/Support/Grid/GridSupportGenerator.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGenerateSupport.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepGroupInfillRegions.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepPostInfillGeneration.hpp"
#include "libslic3r/Steps/StepPostPerimeterGeneration.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepPrepareForPeriemters.hpp"
#include "libslic3r/Steps/StepPrepareInfill.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "libslic3r/Steps/StepSupportDemand.hpp"
#include "libslic3r/Steps/StepSupportToolpath.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"

namespace Slic3r::Test::SupportGrid {
// Build an overhanging slab with a narrow post, optionally adding a low platform beneath the overhang.
inline TriangleMesh slab(bool platform = false)
{
    TriangleMesh post = make_cube(4., 8., 4.);
    TriangleMesh roof = make_cube(12., 8., 1.);
    roof.translate(0., 0., 4.);
    post.merge(roof);
    if (platform) {
        TriangleMesh base = make_cube(12., 8., 0.8);
        post.merge(base);
    }
    return post;
}
// Configure small nozzles, explicit support tools and production toolpath selectors, excluding the registered test probes.
inline DynamicPrintConfig config(const char *style = "grid", bool soluble = false, int interfaces = 3, bool same_tool = false)
{
    Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig result = DynamicPrintConfig::full_print_config();
    result.set_deserialize_strict({{"support_material", "1"}, {"support_material_style", style},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"nozzle_diameter", "0.4,0.4"},
        {"min_layer_height", "0.05,0.05"}, {"max_layer_height", "0.3,0.3"},
        {"support_material_contact_distance_type", soluble ? "none" : "plane"},
        {"support_material_contact_distance", soluble ? "0" : "0.2"},
        {"support_material_bottom_contact_distance", "0.2"},
        {"support_material_interface_layers", std::to_string(interfaces)},
        {"support_material_bottom_interface_layers", std::to_string(interfaces)},
        {"support_material_extruder", "1"}, {"support_material_interface_extruder", same_tool ? "1" : "2"},
        {"support_material_synchronize_layers", "0"}, {"dont_support_bridges", "0"},
        {"raft_first_layer_expansion", "0"}, {"brim_width", "0"}, {"skirts", "0"}});
    for (const std::pair<const char *, const char *> &provider : {
        std::pair<const char *, const char *>{"support_toolpath_sparse", "support.toolpath.legacy_sparse"},
        {"support_toolpath_branch", "support.toolpath.branch_sheath"},
        {"support_toolpath_bottom_interface", "support.toolpath.legacy_bottom"},
        {"support_toolpath_top_interface", "support.toolpath.legacy_top"}}) {
        const std::string key = Steps::plugin_exclusive_group_option_key(Orchestrator::instance(), STEP_SUPPORT_TOOLPATH, provider.first);
        if (!key.empty()) result.set_deserialize_strict({{key, std::string(provider.second)}});
    }
    return result;
}
// Build a print of a mesh whose object has the given layer height profile, as the production slicing reads it.
inline void init_print_with_profile(Print &print, Model &model, TriangleMesh mesh, const std::vector<coordf_t> &profile, const DynamicPrintConfig &config)
{
    ModelObject *object = model.add_object();
    object->name += "object.stl";
    object->add_volume(std::move(mesh));
    object->layer_height_profile.set(profile);
    object->add_instance();
    model.center_instances_around_point({ 100, 100 });
    for (ModelObject &mo : model.objects()) {
        mo.ensure_on_bed();
        print.auto_assign_extruders(&mo);
    }
    print.apply(model, config);
    print.validate();
    print.set_status_silent();
}
// Run the production slicing steps through post-infill, keeping auxiliary geometry editable for either support implementation.
inline void prefix(Print &print)
{
    Plugins::ensure_plugin_test_runtime_initialized();
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
    Steps::StepPostInfillGeneration::prepare(print);
    Steps::StepPostInfillGeneration::run_step(host, print);
}
// Run the registered support families under their real editing step and restore idle access even if generation fails.
inline void family(Print &print)
{
    struct Idle {
        Print &print;
        // Restore idle access when support dispatch ends.
        ~Idle() { print.set_running_step(STEP_NONE); }
    };
    const Idle reset{print};
    print.set_running_step(STEP_SUPPORT);
    Steps::StepGenerateSupport::prepare(print);
    Steps::StepGenerateSupport::run_step(Orchestrator::instance(), print);
}
// Generate support extrusions under the real editing step and restore idle access afterward.
inline void toolpaths(Print &print)
{
    struct Idle {
        Print &print;
        // Restore idle access when support toolpath dispatch ends.
        ~Idle() { print.set_running_step(STEP_NONE); }
    };
    const Idle reset{print};
    print.set_running_step(STEP_SUPPORT_TOOLPATH);
    Steps::StepSupportToolpath::prepare(print);
    Steps::StepSupportToolpath::run_step(Orchestrator::instance(), print);
}
// The plugin API views of a sliced print and of its first object.
inline slic3r_api::Print print_view(const Print &print) { return slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)); }
inline slic3r_api::Object object_view(const PrintObject &object) { return slic3r_api::Object(reinterpret_cast<const object_handle *>(&object)); }
// The support parameters the grid family computes for an object, through the plugin API.
inline slic3r_api::Support::Common::SupportParams params(const Print &print, const PrintObject &object)
{
    return slic3r_api::Support::Common::SupportParams(print_view(print), object_view(object));
}
// Run the grid generator on a sliced object, as the family does, and keep its slabs for inspection.
// The storage is declared first so it outlives the slabs that live in it.
struct Generated {
    PluginStorage storage;
    slic3r_api::Support::Common::GeneratedSupport support{reinterpret_cast<storage_handle *>(&storage)};
    Generated(const Print &print, const PrintObject &object)
    {
        slic3r_api::Support::Grid::generate_support(nullptr, print_view(print), object_view(object), params(print, object), support);
    }
};
// The host polygons behind an API polygon collection.
inline const Polygons &native(const slic3r_api::StoredPolygonCollection &polygons)
{
    return *reinterpret_cast<const Polygons *>(polygons.handle());
}

// Flow-weighted volume and geometric length of the extrusion leaves, per extruder and role.
struct PathMeasure { double length = 0, volume = 0; size_t count = 0; };
using PathMeasures = std::map<std::pair<extruder_index_t, uint16_t>, PathMeasure>;

// Resolve inherited attributes and sum path or stationary deposit volume into result; missing support flow fails the measurement.
inline void measure_paths(const ExtrusionEntity &entity, extruder_index_t extruder, PathMeasures &result, const ExtrusionAttributes *attributes = nullptr)
{
    if (const ExtrusionAttributes *local = entity.get_property<ExtrusionAttributes>()) attributes = local;
    if (entity.has_polyline()) {
        REQUIRE(attributes != nullptr);
        PathMeasure &measure = result[{extruder, uint16_t(attributes->role)}];
        const double length = unscaled(entity.polyline_ref().length());
        measure.length += length;
        const ExtrusionPropertySpecialCommand *command = entity.get_property<ExtrusionPropertySpecialCommand>();
        measure.volume += command != nullptr && command->code == C_EXTRUSION_SPECIAL_COMMAND_DEPOSIT ?
            command->extra_data : length * attributes->mm3_per_mm;
        ++measure.count;
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) measure_paths(*child, extruder, result, attributes);
}

// Read support and interface trees without modifying the layer; the returned map separates roles and tools even when both buckets use one tool.
inline PathMeasures measure_layer(const Layer &layer)
{
    PathMeasures result;
    for (const LayerSliceIsland &island : layer.islands())
        for (const LayerRegionIsland &region : island.regions_islands())
            for (const ExtrusionRole bucket : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE})
                if (region.has_extrusion(bucket)) measure_paths(region.extrusion(bucket), region.extruder_id(), result);
    return result;
}

// Resolve nearest attributes while copying polylines into test-owned paths; missing flow on a geometric leaf fails the comparison instead of dropping geometry.
inline void legacy_paths(const ExtrusionEntity &entity, ExtrusionPaths &paths, const ExtrusionAttributes *attributes = nullptr)
{
    if (const ExtrusionAttributes *local = entity.get_property<ExtrusionAttributes>()) attributes = local;
    if (entity.has_polyline()) {
        REQUIRE(attributes != nullptr);
        ExtrusionPath path(*attributes);
        path.set_polyline(entity.polyline_ref());
        paths.push_back(std::move(path));
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) legacy_paths(*child, paths, attributes);
}

// Preserve loop boundaries while flattening generic wrapper nodes into concrete classes; the legacy print owns the copies and the source tree stays valid until replacement.
inline void legacy_extrusions(const ExtrusionEntity &entity, ExtrusionEntityCollection &out, const ExtrusionAttributes *attributes = nullptr)
{
    if (const ExtrusionAttributes *local = entity.get_property<ExtrusionAttributes>()) attributes = local;
    if (entity.is_loop()) {
        ExtrusionPaths paths;
        legacy_paths(entity, paths, attributes);
        if (!paths.empty()) out.append(ExtrusionLoop(std::move(paths)));
    } else if (entity.has_polyline()) {
        REQUIRE(attributes != nullptr);
        ExtrusionPath path(*attributes);
        path.set_polyline(entity.polyline_ref());
        out.append(std::move(path));
    } else {
        for (const ExtrusionEntityUPtr &child : entity.children()) legacy_extrusions(*child, out, attributes);
    }
}

// Classify raw footprints by adjacent-layer coverage and copy island fills and paths into legacy-readable inputs; only the legacy print's caches and path classes change.
inline void prepare_legacy_inputs(PrintObject &object)
{
    for (Layer &layer : object.layers())
        for (LayerRegion &region : layer.regions()) {
            const ExPolygons &raw = region.get_raw_slices();
            const ExPolygons bottom = layer.lower_layer ? diff_ex(raw, layer.lower_layer->lslices()) : raw;
            const ExPolygons top = diff_ex(layer.upper_layer ? diff_ex(raw, layer.upper_layer->lslices()) : raw, bottom);
            region.m_slices.clear();
            region.m_slices.append(bottom, layer.lower_layer ? stPosBottom | stDensSolid | stModBridge : stPosBottom | stDensSolid);
            region.m_slices.append(top, stPosTop | stDensSolid);
            region.m_slices.append(diff_ex(diff_ex(raw, bottom), top), stPosInternal | stDensSparse);
            SurfaceCollection &fills = region.set_fill_surfaces();
            fills.clear();
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &work : island.regions_islands())
                    if (work.regions().count(&region))
                        for (const Surface &surface : work.fill_surfaces())
                            fills.append(intersection_ex(ExPolygons{surface.expolygon}, raw), surface);
        }
    for (Layer &layer : object.layers())
        for (LayerSliceIsland &island : layer.islands())
            for (LayerRegionIsland &work : island.regions_islands())
                for (const ExtrusionRole role : {LayerRegionIsland::PERIMETERS, LayerRegionIsland::INFILLS}) {
                    if (!work.has_extrusion(role)) continue;
                    ExtrusionEntityCollection concrete;
                    legacy_extrusions(work.extrusion(role), concrete);
                    work.mutable_extrusion(role) = std::move(concrete);
                }
}
}
#endif
