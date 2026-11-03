///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "StepSlicing.hpp"

#include <atomic>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/host/steps/SlicingStep.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/PrintObjectAccess.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/SurfaceCollection.hpp"
#include "libslic3r/Thread.hpp"

#include "StepRunner.hpp"

/* This module runs the selected slicing plugin and combines its region slices into layer islands.
   Orchestrator is the plugin manager; StepSelectorTraits.hpp defines the selector metadata registered here.
   Call graph:
     register_selector_traits
       Orchestrator::register_builtin_step_selector
     run_step
       validate_pre
       prepare -> recreate_object_layers -> object_layers_from_layer_profile
       selected_or_active_plugin_for_step
       Plugin::setup
       Plugin::setup_run
       Plugin::run
       recompute_layer_slices_from_raw_regions
       validate_post
   Layer profiles determine layer intervals; plugins write region slices. Objects run in parallel. */

namespace Slic3r {
LayerUPtrs new_layers(PrintObject *print_object, const std::vector<double> &object_layers);
}

namespace Slic3r::Steps::StepSlicing {

// Install the slicing selector declaration without changing configuration or print data.
void register_selector_traits(Orchestrator &orchestrator);

void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits traits;
    traits.category = RAW_OPTION_CATEGORY_SLICING;
    traits.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    traits.invalidates_step = STEP_SLICING;
    traits.show_with_one_provider = false;
    const int32_t registered = orchestrator.register_builtin_step_selector(STEP_SLICING, std::move(traits));
    assert(registered == 1);
    (void)registered;
}
namespace {

std::vector<double> object_layers_from_layer_profile(const std::vector<coord_t> &layer_profile)
{
    // STEP_LAYER_HEIGHT returns explicit object-local layer descriptors:
    // [layer_top_z, layer_height, ...]. Keeping the height next to the top Z
    // allows a plugin to leave a deliberate empty Z interval below a layer.
    std::vector<double> out;
    out.reserve(layer_profile.size());
    for (size_t idx = 0; idx + 1 < layer_profile.size(); idx += 2) {
        const double z = unscaled(layer_profile[idx]);
        const double height = unscaled(layer_profile[idx + 1]);
        out.push_back(z - height);
        out.push_back(z);
    }
    return out;
}

void recreate_object_layers(PrintObject &object)
{
    LayerUPtrs object_layers =
        new_layers(&object, object_layers_from_layer_profile(object.layer_profile()));
    for (std::unique_ptr<Layer> &layer : object_layers) {
        ApiInternal::LayerAccess::init_regions_from_object(*layer);
    }
    ApiInternal::PrintObjectAccess::replace_layers_by_moving_contents(object, std::move(object_layers));
}

void recompute_layer_slices_from_raw_regions(PrintObject &object)
{
    for (Layer &layer : object.layers()) {
        ExPolygons slices;
        if (layer.region_count() == 1) {
            slices = layer.region(0).get_raw_slices();
        } else {
            ExPolygons slices_exp;
            for (const LayerRegion &region : layer.regions())
                append(slices_exp, region.get_raw_slices());
            slices = union_safety_offset_ex(slices_exp);
        }
        ensure_valid(slices, std::max(scale_i(object.print()->config().resolution), SCALED_EPSILON));
        ApiInternal::LayerAccess::set_islands(layer, std::move(slices));
    }
}

} // namespace

void clear(Print &print)
{
    parallel_for(size_t(0), print.objects().size(), [&print](const size_t object_idx) {
        print.object(object_idx).clear_layers();
    });
}

void prepare(Print &print)
{
    // Existing layers can contain stale slices or region bindings even when their count matches the profile.
    parallel_for(size_t(0), print.objects().size(), [&print](const size_t object_idx) {
        PrintObject &object = print.object(object_idx);
        recreate_object_layers(object);
    });
}

bool validate_pre(const Print &print, std::string &out_error)
{
    bool ok = true;

    for (size_t object_idx = 0; object_idx < print.objects().size(); ++object_idx) {
        const PrintObject &object = print.objects()[object_idx];

        const std::vector<coord_t> &layer_profile = object.layer_profile();
        if (layer_profile.empty()) {
            ok = false;
            std::ostringstream msg;
            msg << "object " << object_idx << ": layer profile is empty";
            out_error += msg.str();
            continue;
        }

        if ((layer_profile.size() & 1) != 0) {
            ok = false;
            std::ostringstream msg;
            msg << "object " << object_idx << ": layer profile has an odd element count";
            out_error += msg.str();
            continue;
        }

        double previous_hi = 0.;
        const double object_top_z = object.slicing_parameters().object_print_z_height();
        for (size_t idx = 0; idx + 1 < layer_profile.size(); idx += 2) {
            const size_t layer_idx = idx / 2;
            const double hi = unscaled(layer_profile[idx]);
            const double height = unscaled(layer_profile[idx + 1]);
            const double lo = hi - height;
            if (height <= EPSILON) {
                ok = false;
                std::ostringstream msg;
                msg << "object " << object_idx << ", layer " << layer_idx
                    << ": layer height is not positive";
                out_error += msg.str();
                break;
            }

            if (lo < -EPSILON || lo + EPSILON < previous_hi) {
                ok = false;
                std::ostringstream msg;
                msg << "object " << object_idx << ", layer " << layer_idx
                    << ": layer interval overlaps the previous layer";
                out_error += msg.str();
                break;
            }

            const double slice_z = 0.5 * (lo + hi);
            if (slice_z >= object_top_z + EPSILON) {
                ok = false;
                std::ostringstream msg;
                msg << "object " << object_idx << ", layer " << layer_idx
                    << ": slice z is outside the object height";
                out_error += msg.str();
                break;
            }
            previous_hi = hi;
        }
    }

    return ok;
}

bool validate_post(const Print &print, std::string &out_error)
{
    bool ok = true;

    for (size_t object_idx = 0; object_idx < print.objects().size(); ++object_idx) {
        const PrintObject &object = print.objects()[object_idx];

        if (object.layer_count() == 0) {
            ok = false;
            std::ostringstream msg;
            msg << "object " << object_idx << ": no layers were created";
            out_error += msg.str();
        }
    }

    return ok;
}

void run_step(Orchestrator &orchestrator, Print &print)
{
    Detail::validate_or_report(validate_pre, print, "Slicing pre-step validation");

    prepare(print);

    // STEP_SLICING is an exclusive step: several active slicer plugins may be
    // available, but the generated step_slicing_plugin setting selects the one
    // that owns this print.
    Plugin *plugin = selected_or_active_plugin_for_step(orchestrator,
                                                        STEP_SLICING,
                                                        &print.full_print_config());
    if (plugin == nullptr)
        return;

    const size_t run_count = print.objects().size();
    std::atomic_bool execution_error { false };
    plugin_host_context host_context =
        Detail::prepare_guarded_plugin_host_context(orchestrator, STEP_SLICING, *plugin, print, execution_error);
    plugin_run_context run_context = orchestrator.prepare_plugin_run_context(STEP_SLICING, plugin, &host_context);
    plugin->setup(run_context, uint32_t(run_count));
    Detail::throw_if_plugin_stopped(orchestrator, print, STEP_SLICING, *plugin, execution_error, "setup");

    std::vector<std::unique_ptr<ApiHost::Steps::SlicingRunContext>> run_contexts;
    run_contexts.reserve(run_count);
    for (size_t object_idx = 0; object_idx < run_count; ++object_idx)
        run_contexts.push_back(ApiHost::Steps::make_slicing_run_context(print, object_idx));

    parallel_for(size_t(0), run_count, [plugin, &run_context, &run_contexts](const size_t object_idx) {
        plugin_run_context context_copy = run_context;
        if (context_copy.is_cancelled != nullptr && context_copy.is_cancelled(context_copy.host_context))
            return;

        context_copy.data = &run_contexts[object_idx]->context_step;
        plugin->setup_run(context_copy);
    });
    Detail::throw_if_plugin_stopped(orchestrator, print, STEP_SLICING, *plugin, execution_error, "setup_run");

    parallel_for(size_t(0), run_count, [plugin, &run_context, &run_contexts](const size_t object_idx) {
        plugin_run_context context_copy = run_context;
        if (context_copy.is_cancelled != nullptr && context_copy.is_cancelled(context_copy.host_context))
            return;

        context_copy.data = &run_contexts[object_idx]->context_step;
        plugin->run(context_copy);
    });
    Detail::throw_if_plugin_stopped(orchestrator, print, STEP_SLICING, *plugin, execution_error, "run");

    parallel_for(size_t(0), run_count, [&print](const size_t object_idx) {
        PrintObject &object = print.object(object_idx);
        recompute_layer_slices_from_raw_regions(object);
    });

    Detail::validate_or_report(validate_post, print, "Slicing post-plugin validation");
    print.pipeline_progress().complete_plugin(STEP_SLICING, plugin->get_id());
}

} // namespace Slic3r::Steps::StepSlicing
