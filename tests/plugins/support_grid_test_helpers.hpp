/* These support tests run the production slicing prefix explicitly so native
   legacy support can be called before auxiliary layers are frozen. Small box
   assemblies share vertices only through merged meshes, and remain one print
   object. Helpers do not change process-wide plugin activation. */
#ifndef support_grid_test_helpers_hpp_
#define support_grid_test_helpers_hpp_
#include <catch2/catch.hpp>
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/ClipperUtils.hpp"
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
        {"support_toolpath_bottom_interface", "support.toolpath.legacy_bottom"},
        {"support_toolpath_top_interface", "support.toolpath.legacy_top"}}) {
        const std::string key = Steps::plugin_exclusive_group_option_key(Orchestrator::instance(), STEP_SUPPORT_TOOLPATH, provider.first);
        if (!key.empty()) result.set_deserialize_strict({{key, std::string(provider.second)}});
    }
    return result;
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
}
#endif
