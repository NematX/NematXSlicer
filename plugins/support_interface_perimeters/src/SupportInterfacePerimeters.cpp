///|/ Released under the GNU Affero General Public License, version 3 or later.
#include "InterfacePerimeters.hpp"

#include <algorithm>
#include <stdexcept>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/GuiRuleHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"
#include "libslic3r/Plugins/Support/Common/SupportToolpathRoles.hpp"
#include "libslic3r/Plugins/Support/Toolpath/LegacySupportPaths.hpp"

/* Four alternative support toolpath providers share the ordinary support
   pipeline and replace only its dense fill above the bed. The count belongs to
   the object; the overlap belongs to its default region. The existing pipeline
   retains contact handling, flow selection, modulation and publication.
   Each callback captures only the current object's count. Static providers hold
   metadata, never print state, so concurrent objects do not share calculations.

       register_plugin -> register four process-lifetime providers
       initialize -> one provider defines the shared count option, layout and activation rule
       run -> generate_paths: original path when count is zero
                InterfaceFill -> fill_interface: contours, then original pattern

   Exceptions in initialization and run are caught and reported by PluginBase
   before returning across the host C ABI. */

namespace nematx::support_interfaces {
namespace {
using namespace slic3r_api;
namespace Common = slic3r_api::Support::Common;
namespace Toolpath = slic3r_api::Support::Toolpath;
constexpr const char *k_count_key = "support_material_interface_perimeters";

// Fill validated dense areas with the effective recipe and the object's default-region overlap.
void fill_pass(const Toolpath::InterfaceFillContext &pass, MutableExtrusionEntity output, unsigned int count);

class Provider final : public PluginBase {
public:
    // Keep only stable metadata; the orchestrator owns the run storage and print data.
    Provider(orchestrator_handle *host, Toolpath::SurfaceRole role, const char *id, const char *group, int32_t priority, bool defines_option)
        : PluginBase(host), m_role(role), m_id(id), m_group(group), m_priority(priority), m_defines_option(defines_option) {}
private:
    // Return the stable activation id.
    const char *id_impl() const noexcept override { return m_id; }
    // Identify this role's provider in plugin lists.
    const char *name_impl() const noexcept override { return m_id; }
    // Describe the dense fill supplied by this provider.
    const char *description_impl() const noexcept override { return "Wrap support interfaces in the configured number of perimeters."; }
    // Run at the ordinary support toolpath step.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT_TOOLPATH; }
    // Replace only this contact or interface role.
    const char *exclusive_group_impl() const noexcept override { return m_group; }
    // Retain the ordinary provider's execution order.
    int32_t priority_impl() const noexcept override { return m_priority; }
    // Prefer this member to legacy rank 100 in automatic selection.
    int32_t exclusive_group_priority_impl() const noexcept override { return 50; }
    // Use the shared pipeline without depending on a particular support family.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Declare the shared pipeline's inputs and the two settings read by this wrapper.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        int32_t count = Common::append_used_config_keys(keys, 0, Common::support_geometry_keys());
        count = Common::append_used_config_keys(keys, count, Common::support_path_keys());
        if (keys) {
            keys[count] = {k_count_key, RAW_CO_INT, RAW_CONTAINER_TYPE_OBJECT, RAW_PRESET_TYPE_FFF_PRINT};
            keys[count + 1] = {"infill_overlap", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
            keys[count + 2] = {"support_material", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        }
        return count + 3;
    }
    // A single member owns the package's shared option definition.
    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (!m_defines_option) return 0;
        if (keys) keys[0] = k_count_key;
        return 1;
    }
    // Define an object-scoped print option; a rejected descriptor fails package initialization visibly.
    void inilialize_impl(storage_handle *) const override
    {
        if (!m_defines_option) return;
        raw_config_option_def option = raw_config_option_def_init();
        option.opt_key = k_count_key;
        option.type = RAW_CO_INT;
        option.container_type = RAW_CONTAINER_TYPE_OBJECT;
        option.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
        option.category = RAW_OPTION_CATEGORY_SUPPORT;
        option.printer_technology = RAW_PT_FFF;
        option.level = RAW_OPTION_LEVEL_ADVANCED;
        option.mode = raw_config_option_mode(RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI);
        option.invalidates_step = STEP_SUPPORT_TOOLPATH;
        option.label = option.full_label = "Interface perimeters";
        option.tooltip = "Number of contour loops around support interfaces, independently of their fill pattern. Zero keeps only the selected fill. The bed layer retains its first-layer contour.";
        option.sidetext = "loops";
        option.has_min = 1;
        option.min_value = 0.;
        option.default_serialized_value = "0";
        option.translation_domain = "Slic3r";
        if (orchestrator_create_option_def(m_orchestrator, &option) != OPTION_DEF_ERROR_OK)
            throw std::runtime_error("Cannot define support_material_interface_perimeters");
        const char *layout = "page:Support material\n"
            "group:Options for support material and raft\n"
            "setting:insert$aftersetting$support_material_interface_contact_loops:support_material_interface_perimeters\n";
        if (orchestrator_add_ui_fragment(m_orchestrator, "print.ui", "nematx.support.interface_perimeters", layout, 0) < 0)
            throw std::runtime_error("Cannot place support interface perimeters in the print settings");
        GuiRule::activate(k_count_key).when(GuiCondition::true_("support_material")).add_to(m_orchestrator);
    }
    // Keep the zero-count path exact; nonzero counts supply a callback local to this object run.
    void run_impl(const plugin_run_context *context) const override
    {
        throw_if_cancelled(context);
        const run_ctx_support_toolpath *payload = plugin_ctx_as_support_toolpath(context);
        if (!payload || !payload->object) throw std::runtime_error("Missing support toolpath object");
        const unsigned int count = unsigned(std::max(0, Object(payload->object).config().int_or_default(k_count_key, 0)));
        if (count == 0) {
            Toolpath::generate_paths(context, m_role);
        } else {
            Toolpath::generate_paths(context, m_role,
                [count](const Toolpath::InterfaceFillContext &pass, MutableExtrusionEntity output) { fill_pass(pass, output, count); });
        }
        progress().finish_run();
    }
    Toolpath::SurfaceRole m_role;
    const char *m_id;
    const char *m_group;
    int32_t m_priority;
    bool m_defines_option;
};

void fill_pass(const Toolpath::InterfaceFillContext &pass, MutableExtrusionEntity output, unsigned int count)
{
    raw_infill_pattern_params recipe = pass.recipe;
    // The composite rectilinear pattern already owns a contour; use its inner pattern so the requested count stays independent of that choice.
    if (pass.pattern == "rectiwithperimeter")
        recipe.pattern_id = pass.payload.resolve_pattern_id(&pass.payload, "rectilinear");
    const Config region = pass.object.default_region_config(Print(pass.payload.print));
    const double flow_spacing = pass.flow_spacing;
    const double overlap = region.get("infill_overlap").get_effective_value(flow_spacing);
    Settings settings;
    settings.count = count;
    settings.spacing = pass.recipe.spacing;
    settings.encroachment = float(std::clamp(overlap / flow_spacing, 0., 0.5));
    settings.role = pass.recipe.extrusion_role;
    settings.flow = pass.recipe.flow;
    settings.monotonic = (pass.recipe.infill_flags & C_INFILL_FLAG_MONOTONIC) != 0;
    const Common::InnerFill inner = Common::pattern_service_fill(pass.context, pass.payload, recipe.pattern_id,
        recipe, pass.layer, nullptr, nullptr, layer_get_region(pass.layer, 0));
    const raw_surface_type type = raw_surface_type(RAW_SURFACE_TYPE_POS_INTERNAL | RAW_SURFACE_TYPE_DENS_SPARSE);
    for (const ExPolygon area : pass.areas)
        fill_interface(pass.storage, Surface(area, type), settings, inner, output);
}
}
}

// Register stable instances and group descriptions; the host copies metadata and borrows each provider for the process lifetime.
extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *orchestrator)
{
    using namespace nematx::support_interfaces;
    namespace Common = slic3r_api::Support::Common;
    using slic3r_api::Support::Toolpath::SurfaceRole;
    static Provider bottom_contact(orchestrator, SurfaceRole::BottomContact, "nematx.support.bottom_contact", Common::k_bottom_contact_role, 90, true);
    static Provider bottom(orchestrator, SurfaceRole::BottomInterface, "nematx.support.bottom_interface", Common::k_bottom_interface_role, 100, false);
    static Provider top_contact(orchestrator, SurfaceRole::TopContact, "nematx.support.top_contact", Common::k_top_contact_role, 190, false);
    static Provider top(orchestrator, SurfaceRole::TopInterface, "nematx.support.top_interface", Common::k_top_interface_role, 200, false);
    struct Member { Provider *provider; const char *label; const char *tooltip; };
    const Member members[]{{&bottom_contact, Common::k_bottom_contact_label, Common::k_bottom_contact_tooltip},
        {&bottom, Common::k_bottom_interface_label, Common::k_bottom_interface_tooltip},
        {&top_contact, Common::k_top_contact_label, Common::k_top_contact_tooltip},
        {&top, Common::k_top_interface_label, Common::k_top_interface_tooltip}};
    for (const Member &member : members) {
        Provider *provider = member.provider;
        orchestrator_register_plugin(orchestrator, provider->c_instance());
        raw_exclusive_group_def definition = raw_exclusive_group_def_init();
        const plugin_instance instance = provider->c_instance();
        definition.exclusive_group = instance.vt->get_exclusive_group(instance.ctx);
        definition.label = member.label;
        definition.tooltip = member.tooltip;
        definition.container_type = RAW_CONTAINER_TYPE_OBJECT;
        definition.category = RAW_OPTION_CATEGORY_SUPPORT;
        definition.auto_value = 1;
        definition.has_invalidates_step = 1;
        definition.invalidates_step = STEP_SUPPORT_TOOLPATH;
        definition.ui_page = "Support material";
        definition.ui_group = "Support toolpaths";
        provider->register_exclusive_group(STEP_SUPPORT_TOOLPATH, definition);
    }
}

// Include last so the ABI manifest covers all public headers consumed above.
#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"
