///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/ Copyright (c) Prusa Research 2019 - 2023 Lukáš Hejl @hejllukas, Vojtěch Bubník @bubnikv, Lukáš Matěna @lukasmatena, Oleksandra Iushchenko @YuSanka, Pavel Mikuš @Godrak, Tomáš Mészáros @tamasmeszaros
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "ConfigManipulation.hpp"

#include <string>
#include <utility>

#include <wx/msgdlg.h>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Config/PresetBundle.hpp"
#include "libslic3r/SLA/SupportTreeStrategies.hpp"

#include "format.hpp"
#include "GUI_App.hpp"
#include "I18N.hpp"
#include "MsgDialog.hpp"

/*
ConfigManipulation validates edited settings and enables their GUI fields.
Print and printer updates read borrowed configurations, apply dependent values
through callbacks and ask for confirmation when a setting combination needs it.
Field toggles combine local printer checks with rules evaluated by the plugin
runtime. Crossing-only wipe follows wipe availability, not perimeter avoidance.

    update_print_fff_config / update_printer_fff_config: validate related values
        `-- apply: copy changed values and reload their controls
    toggle_print_fff_options / toggle_printer_fff_options / toggle_print_sla_options
        |-- toggle_field: forward enabled state to the owning page
        `-- apply_plugin_gui_rules: forward the runtime's resolved field rules
*/

namespace Slic3r {
namespace GUI {

void ConfigManipulation::apply_plugin_gui_rules(DynamicPrintConfig *config, int current_index, raw_option_preset_type scope,
                                                const std::vector<OptionKeyIdx>* visible_settings)
{
    if (config == nullptr)
        return;

    // The rule evaluation and aggregation live in the orchestrator so that the
    // exact enable/disable behavior is testable without the GUI. This function
    // only forwards each resolved target to the active page.
    //
    // current_index == -1 means "global pass".
    // current_index >= 0 means "per-extruder pass for that vector item".
    //
    // visible_settings is the field list of the refreshed page: when provided,
    // only the rules of the displayed settings are evaluated. A null pointer or
    // an empty list falls back to the full scan (page not built yet).
    const std::vector<OptionKeyIdx> empty_filter;
    const std::vector<OptionKeyIdx> &setting_filter = visible_settings != nullptr ? *visible_settings : empty_filter;

    for (const Orchestrator::GuiRuleApplication &application :
         Orchestrator::instance().gui_rules().evaluate(*config, scope, current_index, std::string(), setting_filter)) {
        try {
            this->toggle_field(application.target_key, application.enabled, application.target_index);
        } catch (...) {
            assert(false);
        }
    }
}

void ConfigManipulation::apply(DynamicPrintConfig* config, DynamicPrintConfig* new_config)
{
    bool modified = false;
    for (auto opt_key : config->diff(*new_config)) {
        config->set_key_value(opt_key, new_config->option(opt_key)->clone());
        modified = true;
    }

    if (modified && load_config != nullptr)
        load_config();
}

void ConfigManipulation::toggle_field(const std::string& opt_key, const bool toggle, int opt_index/* = -1*/)
{
    if (local_config) {
        if (local_config->option(opt_key) == nullptr)
            return;
    }
    cb_toggle_field(opt_key, toggle, opt_index);
}

// !! if using cb_value_change(X) or something like that, you need a special code in Field.cpp (search for 'update_print_fff_config')
void ConfigManipulation::update_print_fff_config(DynamicPrintConfig* config, const bool is_global_config)
{
    // #ys_FIXME_to_delete
    //! Temporary workaround for the correct updates of the TextCtrl (like "layer_height"):
    // KillFocus() for the wxSpinCtrl use CallAfter function. So,
    // to except the duplicate call of the update() after dialog->ShowModal(),
    // let check if this process is already started.
    if (is_msg_dlg_already_exist)
        return;

    // layer_height shouldn't be equal to zero
    if (config->opt_float("layer_height") < EPSILON)
    {
        const wxString msg_text = _(L("Layer height is not valid.\n\nThe layer height will be reset to 0.01."));
        MessageDialog dialog(m_msg_dlg_parent, msg_text, _(L("Layer height")), wxICON_WARNING | wxOK);
        DynamicPrintConfig new_conf = *config;
        is_msg_dlg_already_exist = true;
        dialog.ShowModal();
        new_conf.set_key_value("layer_height", new ConfigOptionFloat(0.01));
        apply(config, &new_conf);
        is_msg_dlg_already_exist = false;
    }

    if (config->option<ConfigOptionFloatOrPercent>("first_layer_height")->value < EPSILON)
    {
        const wxString msg_text = _(L("First layer height is not valid.\n\nThe first layer height will be reset to 0.01."));
        MessageDialog dialog(m_msg_dlg_parent, msg_text, _(L("First layer height")), wxICON_WARNING | wxOK);
        DynamicPrintConfig new_conf = *config;
        is_msg_dlg_already_exist = true;
        dialog.ShowModal();
        new_conf.set_key_value("first_layer_height", new ConfigOptionFloatOrPercent(0.01, false));
        apply(config, &new_conf);
        is_msg_dlg_already_exist = false;
    }

    double fill_density = config->option<ConfigOptionPercent>("fill_density")->value;

    if (config->opt_bool("spiral_vase") && !(
        config->opt_int("top_solid_layers") == 0
        && fill_density == 0
        && config->opt_bool("support_material") == false
        && config->opt_int("support_material_enforce_layers") == 0
        && config->opt_enum<PerimeterGeneratorType>("perimeter_generator") == PerimeterGeneratorType::Classic
        // && config->opt_bool("exact_last_layer_height") == false
        && config->opt_bool("infill_dense") == false
        && config->opt_bool("extra_perimeters") == false
        && config->option("extra_perimeters_below_area")->get_float() == 0
        && config->opt_int("extra_perimeters_count") == 0
        && config->opt_bool("extra_perimeters_odd_layers") == false
        && config->opt_bool("extra_perimeters_on_overhangs") == false
        && config->opt_bool("overhangs_reverse") == false
        && config->opt_bool("gap_fill_last") == false
        && config->opt_int("solid_infill_every_layers") == 0
        && config->opt_int("solid_over_perimeters") == 0
        && config->option("seam_notch_all")->get_float() == 0
        && config->option("seam_notch_inner")->get_float() == 0
        && config->option("seam_notch_outer")->get_float() == 0
        )) {
        wxString msg_text = _(L("The Spiral Vase mode requires:\n"
            "- no top solid layers\n"
            "- 0% fill density\n"
            "- classic perimeter slicing\n"
            "- no support material\n"
            "- disabled 'no solid infill over perimeters'\n"
            // "- unchecked 'exact last layer height'\n"
            "- unchecked 'dense infill'\n"
            "- unchecked 'extra perimeters'"
            "- unchecked 'gap fill after last perimeter'"
            "- set 'solid infill every layers' to 0"
            "- disabled  'no solid fill over X perimeters'"
            "- disabled 'seam notch'"));
        if (is_global_config)
            msg_text += "\n\n" + _(L("Shall I adjust those settings in order to enable Spiral Vase?"));
        MessageDialog dialog(m_msg_dlg_parent, msg_text, _(L("Spiral Vase")),
            wxICON_WARNING | (is_global_config ? wxYES | wxNO : wxOK));
        DynamicPrintConfig new_conf = *config;
        auto answer = dialog.ShowModal();
        bool support = true;
        if (!is_global_config) {
            if (this->local_config->get().optptr("spiral_vase"))
                new_conf.set_key_value("spiral_vase", new ConfigOptionBool(false));
            else if (this->local_config->get().optptr("top_solid_layers"))
                new_conf.set_key_value("top_solid_layers", new ConfigOptionInt(0));
            else if (this->local_config->get().optptr("fill_density"))
                new_conf.set_key_value("fill_density", new ConfigOptionPercent(0));
            else if (this->local_config->get().optptr("perimeter_generator"))
                new_conf.set_key_value("perimeter_generator", new ConfigOptionEnum<PerimeterGeneratorType>(PerimeterGeneratorType::Classic));
            else if (this->local_config->get().optptr("support_material"))
                new_conf.set_key_value("support_material", new ConfigOptionBool(false));
            else if (this->local_config->get().optptr("support_material_enforce_layers"))
                new_conf.set_key_value("support_material_enforce_layers", new ConfigOptionInt(0));
            // else if (this->local_config->get().optptr("exact_last_layer_height"))
                // new_conf.set_key_value("exact_last_layer_height", new ConfigOptionBool(false));
            else if (this->local_config->get().optptr("infill_dense"))
                new_conf.set_key_value("infill_dense", new ConfigOptionBool(false));
            else if (this->local_config->get().optptr("extra_perimeters"))
                new_conf.set_key_value("extra_perimeters", new ConfigOptionBool(false));
            else if (this->local_config->get().optptr("extra_perimeters_below_area"))
                new_conf.set_key_value("extra_perimeters_below_area", new ConfigOptionFloatOrPercent(0, false));
            else if (this->local_config->get().optptr("extra_perimeters_count"))
                new_conf.set_key_value("extra_perimeters_count", new ConfigOptionInt(0));
            else if (this->local_config->get().optptr("extra_perimeters_odd_layers"))
                new_conf.set_key_value("extra_perimeters_odd_layers", new ConfigOptionBool(false));
            else if (this->local_config->get().optptr("extra_perimeters_on_overhangs"))
                new_conf.set_key_value("extra_perimeters_on_overhangs", new ConfigOptionBool(false));
            else if (this->local_config->get().optptr("overhangs_reverse"))
                new_conf.set_key_value("overhangs_reverse", new ConfigOptionBool(false));
            else if (this->local_config->get().optptr("gap_fill_last"))
                new_conf.set_key_value("gap_fill_last", new ConfigOptionBool(false));
            else if (this->local_config->get().optptr("solid_infill_every_layers"))
                new_conf.set_key_value("solid_infill_every_layers", new ConfigOptionInt(0));
            else if (this->local_config->get().optptr("solid_over_perimeters"))
                new_conf.set_key_value("solid_over_perimeters", new ConfigOptionInt(0));
            else if (this->local_config->get().optptr("seam_notch_all"))
                new_conf.set_key_value("seam_notch_all", new ConfigOptionFloatOrPercent(0, false));
            else if (this->local_config->get().optptr("seam_notch_inner"))
                new_conf.set_key_value("seam_notch_all", new ConfigOptionFloatOrPercent(0, false));
            else if (this->local_config->get().optptr("seam_notch_outer"))
                new_conf.set_key_value("seam_notch_all", new ConfigOptionFloatOrPercent(0, false));
            this->local_config->apply_only(new_conf, this->local_config->keys(), true);
        } else if (answer == wxID_YES) {
            new_conf.set_key_value("top_solid_layers", new ConfigOptionInt(0));
            new_conf.set_key_value("fill_density", new ConfigOptionPercent(0));
            new_conf.set_key_value("perimeter_generator", new ConfigOptionEnum<PerimeterGeneratorType>(PerimeterGeneratorType::Classic));
            new_conf.set_key_value("support_material", new ConfigOptionBool(false));
            new_conf.set_key_value("support_material_enforce_layers", new ConfigOptionInt(0));
            // new_conf.set_key_value("exact_last_layer_height", new ConfigOptionBool(false));
            new_conf.set_key_value("infill_dense", new ConfigOptionBool(false));
            new_conf.set_key_value("extra_perimeters", new ConfigOptionBool(false));
            new_conf.set_key_value("extra_perimeters_below_area", new ConfigOptionFloatOrPercent(0, false));
            new_conf.set_key_value("extra_perimeters_count", new ConfigOptionInt(0));
            new_conf.set_key_value("extra_perimeters_odd_layers", new ConfigOptionBool(false));
            new_conf.set_key_value("extra_perimeters_on_overhangs", new ConfigOptionBool(false));
            new_conf.set_key_value("overhangs_reverse", new ConfigOptionBool(false));
            new_conf.set_key_value("gap_fill_last", new ConfigOptionBool(false));
            new_conf.set_key_value("solid_infill_every_layers", new ConfigOptionInt(0));
            new_conf.set_key_value("solid_over_perimeters", new ConfigOptionInt(0));
            new_conf.set_key_value("seam_notch_all", new ConfigOptionFloatOrPercent(0, false));
            new_conf.set_key_value("seam_notch_inner", new ConfigOptionFloatOrPercent(0, false));
            new_conf.set_key_value("seam_notch_outer", new ConfigOptionFloatOrPercent(0, false));
            fill_density = 0;
            support = false;
        } else {
            new_conf.set_key_value("spiral_vase", new ConfigOptionBool(false));
        }
        apply(config, &new_conf);
        if (cb_value_change) {
            cb_value_change("fill_density", fill_density);
            if (!support)
                cb_value_change("support_material", false);
        }
    }

    if (config->opt_float("brim_width") > 0 && config->opt_float("brim_separation") >= config->opt_float("brim_width")) {
        wxString msg_text = _(L("It's not possible to use a bigger value for the brim offset than the brim width, as it won't extrude anything."
            " Brim offset have to be lower than the brim width."));
        if (is_global_config) {
            msg_text += "\n\n" + _(L("Shall I switch the brim offset to 0?"));
            wxMessageDialog dialog(nullptr, msg_text, _(L("Brim configuration")),
                wxICON_WARNING | (is_global_config ? wxYES | wxNO : wxOK));
            auto answer = dialog.ShowModal();
            if (!is_global_config || answer == wxID_YES) {
                DynamicPrintConfig new_conf = *config;
                new_conf.set_key_value("brim_separation", new ConfigOptionFloat(0));
                apply(config, &new_conf);
            }
        }
    }

    // Check "support_material" and "overhangs" relations only on global settings level
    if (is_global_config && config->opt_bool("support_material")) {
        // Ask only once.
        if (!m_support_material_overhangs_queried) {
            m_support_material_overhangs_queried = true;
            if (!config->option("overhangs")->get_bool()) {
                wxString msg_text = _(L("Supports work better, if the following feature is enabled:\n"
                    "- overhangs threshold for speed & fan\n"
                    "- overhangs threshold for flow"));
                if (is_global_config) {
                    msg_text += "\n\n" + _(L("Shall I adjust those settings for supports?"));
                    MessageDialog dialog(m_msg_dlg_parent, msg_text, _L("Support Generator"), wxICON_WARNING | wxYES | wxNO);
                        DynamicPrintConfig new_conf = *config;
                        auto answer = dialog.ShowModal();
                    if (answer == wxID_YES) {
                        // Enable "detect bridging perimeters".
                        new_conf.set_key_value("overhangs", new ConfigOptionBool(true));
                        new_conf.set_key_value("overhangs_flow_ratio", config->option("overhangs_flow_ratio")->clone()->set_enabled(true));
                    } else if (answer == wxID_NO) {
                        // Do nothing, leave supports on and "detect bridging perimeters" off.
                    } else if (answer == wxID_CANCEL) {
                        // Disable supports.
                        new_conf.set_key_value("support_material", new ConfigOptionBool(false));
                        m_support_material_overhangs_queried = false;
                    }
                    apply(config, &new_conf);
                }
            }
        }
    }
    else {
        m_support_material_overhangs_queried = false;
    }

    const ConfigOption *fill_pattern_option = config->option("fill_pattern");
    if (config->option<ConfigOptionPercent>("fill_density")->value == 100 && fill_pattern_option != nullptr) {
        std::string fill_pattern_name = fill_pattern_option->serialize();
        if (!fill_pattern_name.empty() && fill_pattern_name.front() == '!')
            fill_pattern_name.erase(fill_pattern_name.begin());

        // Pattern options may be plugin-defined enums, so membership is checked
        // on the serialized value instead of an integer enum index.
        const auto def_has_value = [config](const char *key, const std::string &value) {
            const ConfigOptionDef *def = config->option_def(key);
            return def != nullptr && def->has_enum_value(value);
        };
        const bool correct_100p_fill = def_has_value("top_fill_pattern", fill_pattern_name) ||
                                       def_has_value("bottom_fill_pattern", fill_pattern_name) ||
                                       def_has_value("solid_fill_pattern", fill_pattern_name);
        if (!correct_100p_fill) {
            // get fill_pattern name from enum_labels for using this one at dialog_msg
            const ConfigOptionDef *fill_pattern_def = config->option_def("fill_pattern");
            assert(fill_pattern_def != nullptr);
            const std::optional<int> fill_pattern_index =
                fill_pattern_def->enum_def->value_to_index(fill_pattern_name);
            if (fill_pattern_index.has_value()) {
                wxString msg_text = GUI::format_wxstr(_L("The %1% infill pattern is not supposed to work at 100%% density."),
                                                       I18N::translate_in_domain(fill_pattern_def->enum_def->label(*fill_pattern_index),
                                                                                 fill_pattern_def->translation_domain));
                if (is_global_config) {
                    msg_text += "\n\n" + _L("Shall I switch to rectilinear fill pattern?");
                MessageDialog dialog(m_msg_dlg_parent, msg_text, _L("Infill"),
                        wxICON_WARNING | (is_global_config ? wxYES | wxNO : wxOK));
                    DynamicPrintConfig new_conf;// = *config;
                    auto answer = dialog.ShowModal();
                    std::string key_changed;
                    boost::any new_value;
                    if (!is_global_config || answer == wxID_YES) {
                        new_conf.set_deserialize_strict("fill_pattern", "rectilinear");
                        key_changed = "fill_pattern";
                        new_value = new_conf.option("fill_pattern")->get_any();
                    } else {
                        fill_density = wxGetApp().preset_bundle->fff_prints.get_selected_preset().config.option<ConfigOptionPercent>("fill_density")->value;
                        new_conf.set_key_value("fill_density", new ConfigOptionPercent(fill_density));
                        key_changed = "fill_density";
                        new_value = new_conf.option("fill_density")->get_any();
                    }
                    apply(config, &new_conf);
                    if (cb_value_change)
                        cb_value_change(key_changed, new_value);
                }
            }
        }
    }
}

void ConfigManipulation::toggle_print_fff_options(DynamicPrintConfig* config, const std::vector<OptionKeyIdx>* visible_settings)
{
    // Delegate the print-setting activation to the plugin rule registry.

    this->apply_plugin_gui_rules(config, -1, RAW_PRESET_TYPE_FFF_PRINT, visible_settings);
}


void ConfigManipulation::update_printer_fff_config(DynamicPrintConfig *config,
                                                   const bool          is_global_config)
{
    const std::vector<double> &nozzle_sizes = config->option<ConfigOptionFloats>("nozzle_diameter")->get_values();
    double min_step_size = config->option("z_step")->get_float();
    //for each extruder
    for (size_t extruder_idx = 0; extruder_idx < nozzle_sizes.size(); ++extruder_idx) {
        double min_lh = config->get_computed_value("min_layer_height", extruder_idx);
        double max_lh = config->option("max_layer_height")->is_enabled() ? config->get_computed_value("max_layer_height", extruder_idx) : nozzle_sizes[extruder_idx] * 0.75f;
        if (config->option("max_layer_height")->is_enabled() && (max_lh < min_step_size || max_lh < EPSILON)) {
            const wxString msg_text = _(
                L("Maximum layer height is not valid, it can't be lower than minimum z step, and not 0.\n\nThe maximum layer height will be deactivated (set to 75% of the nozzle diameter)."));
            MessageDialog dialog(m_msg_dlg_parent, msg_text, _(L("Maximum layer height")), wxICON_WARNING | wxOK);
            DynamicPrintConfig new_conf = *config;
            is_msg_dlg_already_exist    = true;
            dialog.ShowModal();
            new_conf.option<ConfigOptionFloatsOrPercents>("max_layer_height")->set_at(FloatOrPercent{75., true}, extruder_idx);
            new_conf.option<ConfigOptionFloatsOrPercents>("max_layer_height")->set_enabled(false, extruder_idx);
            apply(config, &new_conf);
            is_msg_dlg_already_exist = false;
            max_lh = config->get_computed_value("max_layer_height", extruder_idx);
        }
        // now max_lh > nozzle_size is allowed, but a warning is sent when changed
        if (min_lh >= max_lh) {
            const wxString msg_text = _(
                L("Minimum layer height is not valid, it can't be higher or equal to the maximum layer height.\n\nThe minimum layer height will be set to 0."));
            MessageDialog dialog(m_msg_dlg_parent, msg_text, _(L("Minimum layer height")), wxICON_WARNING | wxOK);
            DynamicPrintConfig new_conf = *config;
            is_msg_dlg_already_exist    = true;
            dialog.ShowModal();
            new_conf.option<ConfigOptionFloatsOrPercents>("min_layer_height")->set_at(FloatOrPercent{0.0, false}, extruder_idx);
            apply(config, &new_conf);
            min_lh = config->get_computed_value("min_layer_height", extruder_idx);
            is_msg_dlg_already_exist = false;
        }
        
        bool have_retract_length = config->opt_float("retract_length", extruder_idx) > 0;
        bool use_firmware_retraction = config->opt_bool("use_firmware_retraction");
        bool wipe = config->get_bool("wipe", extruder_idx) && have_retract_length;
        if (use_firmware_retraction && wipe) {
            DynamicPrintConfig new_conf = *config;
            //wxMessageDialog dialog(parent(),
            MessageDialog dialog(m_msg_dlg_parent,
                _(L("The Wipe option is not available when using the Firmware Retraction mode.\n"
                "\nShall I disable it in order to enable Firmware Retraction?")),
                _(L("Firmware Retraction")), wxICON_WARNING | wxYES | wxNO);

            if (dialog.ShowModal() == wxID_YES) {
                new_conf.option<ConfigOptionBools>("wipe")->set_at(uint8_t(false), extruder_idx);
            } else {
                new_conf.option<ConfigOptionBool>("use_firmware_retraction")->value = false;
            }
            apply(config, &new_conf);
        }
    }
}
void ConfigManipulation::toggle_printer_fff_options(DynamicPrintConfig *config, const std::vector<OptionKeyIdx>* visible_settings)
{

    size_t extruder_count = config->option("nozzle_diameter")->size();
    toggle_field("toolchange_gcode", extruder_count > 1);
    toggle_field("single_extruder_multi_material", extruder_count > 1);

    //thumbnails
    bool custom_color = config->option("thumbnail_color_mode")->get_int() == int(ThumbnailColorMode::Uniform);
    toggle_field("thumbnails_color", custom_color);
    const ConfigOption* thumbnails_format = config->option("thumbnails_format");
    
    if (thumbnails_format) {
        toggle_field("thumbnails_end_file", thumbnails_format->serialize() != "BIQU");
        toggle_field("thumbnails_tag_format", thumbnails_format->serialize() != "BIQU");
    }

    bool have_arc_fitting = config->option("arc_fitting")->get_int() != int(ArcFittingType::Disabled);
    toggle_field("arc_fitting_ignore_holes", have_arc_fitting);
    toggle_field("arc_fitting_resolution", have_arc_fitting);
    toggle_field("arc_fitting_tolerance", have_arc_fitting);

    //firmware
    bool have_remaining_times = config->opt_bool("remaining_times");
    toggle_field("remaining_times_type", have_remaining_times);

    bool gcode_min_length = config->get_float("gcode_min_length") > 0 && config->is_enabled("gcode_min_length");
    bool max_gcode_per_second = config->get_float("max_gcode_per_second") > 0 && config->is_enabled("max_gcode_per_second");
    toggle_field("gcode_min_resolution", gcode_min_length || max_gcode_per_second);
    toggle_field("gcode_command_buffer", max_gcode_per_second);

    auto flavor = gcode_flavor(*config);
    bool is_marlin_flavor = flavor == gcfMarlinLegacy || flavor == gcfMarlinFirmware;
    // Disable silent mode for non-marlin firmwares.
    toggle_field("silent_mode", is_marlin_flavor);

    for (size_t i = 0; i < extruder_count; ++i) {
        
        bool have_retract_length = config->opt_float("retract_length", i) > 0;
        
        const bool ramping_lift = config->get_bool("travel_ramping_lift", i);
        //const bool lifts_z = (ramping_lift && config->get_float("travel_max_lift", i) > 0)
        //                  || (! ramping_lift && config->get_float("retract_lift", i) > 0);

        //toggle_field("travel_max_lift", ramping_lift, i);
        toggle_field("travel_slope", ramping_lift, i);
        // toggle_field("retract_lift", ! ramping_lift, i);

        // when using firmware retraction, firmware decides retraction length
        bool use_firmware_retraction = config->opt_bool("use_firmware_retraction");
        toggle_field("retract_length", !use_firmware_retraction, i);

        // retraction only if have retraction length or we're using firmware retraction
        bool retraction = (have_retract_length || use_firmware_retraction);
        
        // user can customize travel length if we have retraction length or we"re using
        // firmware retraction
        toggle_field("retract_before_travel", have_retract_length || use_firmware_retraction, i);
        
        // user can customize other retraction options if retraction is enabled
        //std::vector<std::string> vec = {"retract_layer_change" }; // "retract_lift" "retract_before_travel"
        // now possible outside retraction
        // for (auto el : vec) {
            // toggle_field(el, retraction, i);
        // }
                bool has_lift = /*retraction &&  now possible outside retraction */ config->get_float("retract_lift", i) > 0;
        // retract lift above / below only applies if using retract lift
        // vec.resize(0);
        std::vector<std::string> vec = { "retract_lift_above", "retract_lift_below", "retract_lift_top", "retract_lift_first_layer", "retract_lift_before_travel"};
        for (auto el : vec) {
            toggle_field(el, has_lift, i);
        }

        // some options only apply when not using firmware retraction
        vec.resize(0);
        vec = { "retract_speed", "deretract_speed", "retract_before_wipe",
            "retract_restart_extra", "wipe", "wipe_speed" , "wipe_only_crossing",
            "wipe_return"};
        for (auto el : vec) {
            toggle_field(el, retraction && !use_firmware_retraction, i);
        }

        bool wipe = config->opt_bool("wipe", i) && have_retract_length;
        vec.resize(0);
        vec = { "retract_before_wipe", "wipe_only_crossing", "wipe_return", "wipe_speed" };
        for (auto el : vec) {
            toggle_field(el, wipe, i);
        }

        
        toggle_field("travel_lift_before_obstacle", ramping_lift, i);

        toggle_field("retract_length_toolchange", extruder_count > 1, i);

        bool toolchange_retraction = config->opt_float("retract_length_toolchange", i) > 0;
        toggle_field("retract_restart_extra_toolchange", extruder_count > 1 && toolchange_retraction, i);
        toggle_field("retract_restart_toolchange_on_perimeter", extruder_count > 1 && toolchange_retraction, i);
        toggle_field("retract_restart_wipe_toolchange", extruder_count > 1 && toolchange_retraction, i);

        this->apply_plugin_gui_rules(config, int(i), RAW_PRESET_TYPE_FFF_PRINTER, visible_settings);
    }

    if (config->opt_bool("single_extruder_multi_material") && extruder_count > 1) {
        bool have_advanced_wipe_volume = config->opt_bool("wipe_advanced");
        for (auto el : { "wipe_advanced_nozzle_melted_volume", "wipe_advanced_multiplier", "wipe_advanced_algo" }) {
            toggle_field(el, have_advanced_wipe_volume);
        }
    }

    this->apply_plugin_gui_rules(config, -1, RAW_PRESET_TYPE_FFF_PRINTER, visible_settings);
}

void ConfigManipulation::toggle_print_sla_options(DynamicPrintConfig* config, const std::vector<OptionKeyIdx>* visible_settings)
{
    bool supports_en = config->opt_bool("supports_enable");
    sla::SupportTreeType treetype = config->opt_enum<sla::SupportTreeType>("support_tree_type");
    bool is_default_tree = treetype == sla::SupportTreeType::Default;
    bool is_branching_tree = treetype == sla::SupportTreeType::Branching;

    toggle_field("support_tree_type", supports_en);

    toggle_field("support_head_front_diameter", supports_en && is_default_tree);
    toggle_field("support_head_penetration", supports_en && is_default_tree);
    toggle_field("support_head_width", supports_en && is_default_tree);
    toggle_field("support_pillar_diameter", supports_en && is_default_tree);
    toggle_field("support_small_pillar_diameter_percent", supports_en && is_default_tree);
    toggle_field("support_max_bridges_on_pillar", supports_en && is_default_tree);
    toggle_field("support_pillar_connection_mode", supports_en && is_default_tree);
    toggle_field("support_buildplate_only", supports_en && is_default_tree);
    toggle_field("support_base_diameter", supports_en && is_default_tree);
    toggle_field("support_base_height", supports_en && is_default_tree);
    toggle_field("support_base_safety_distance", supports_en && is_default_tree);
    toggle_field("support_critical_angle", supports_en && is_default_tree);
    toggle_field("support_max_bridge_length", supports_en && is_default_tree);
    toggle_field("support_enforcers_only", supports_en);
    toggle_field("support_max_pillar_link_distance", supports_en && is_default_tree);
    toggle_field("support_pillar_widening_factor", false);
    toggle_field("support_max_weight_on_model", false);

    toggle_field("branchingsupport_head_front_diameter", supports_en && is_branching_tree);
    toggle_field("branchingsupport_head_penetration", supports_en && is_branching_tree);
    toggle_field("branchingsupport_head_width", supports_en && is_branching_tree);
    toggle_field("branchingsupport_pillar_diameter", supports_en && is_branching_tree);
    toggle_field("branchingsupport_small_pillar_diameter_percent", supports_en && is_branching_tree);
    toggle_field("branchingsupport_max_bridges_on_pillar", false);
    toggle_field("branchingsupport_pillar_connection_mode", false);
    toggle_field("branchingsupport_buildplate_only", supports_en && is_branching_tree);
    toggle_field("branchingsupport_base_diameter", supports_en && is_branching_tree);
    toggle_field("branchingsupport_base_height", supports_en && is_branching_tree);
    toggle_field("branchingsupport_base_safety_distance", supports_en && is_branching_tree);
    toggle_field("branchingsupport_critical_angle", supports_en && is_branching_tree);
    toggle_field("branchingsupport_max_bridge_length", supports_en && is_branching_tree);
    toggle_field("branchingsupport_max_pillar_link_distance", false);
    toggle_field("branchingsupport_pillar_widening_factor", supports_en && is_branching_tree);
    toggle_field("branchingsupport_max_weight_on_model", supports_en && is_branching_tree);

    toggle_field("support_points_density_relative", supports_en);
    toggle_field("support_points_minimal_distance", supports_en);

    bool pad_en = config->opt_bool("pad_enable");

    toggle_field("pad_wall_thickness", pad_en);
    toggle_field("pad_wall_height", pad_en);
    toggle_field("pad_brim_size", pad_en);
    toggle_field("pad_max_merge_distance", pad_en);
 // toggle_field("pad_edge_radius", supports_en);
    toggle_field("pad_wall_slope", pad_en);
    toggle_field("pad_around_object", pad_en);
    toggle_field("pad_around_object_everywhere", pad_en);

    bool zero_elev = config->opt_bool("pad_around_object") && pad_en;

    toggle_field("support_object_elevation", supports_en && is_default_tree && !zero_elev);
    toggle_field("branchingsupport_object_elevation", supports_en && is_branching_tree && !zero_elev);
    toggle_field("pad_object_gap", zero_elev);
    toggle_field("pad_around_object_everywhere", zero_elev);
    toggle_field("pad_object_connector_stride", zero_elev);
    toggle_field("pad_object_connector_width", zero_elev);
    toggle_field("pad_object_connector_penetration", zero_elev);

    this->apply_plugin_gui_rules(config, -1, RAW_PRESET_TYPE_SLA_PRINT, visible_settings);
}


} // GUI
} // Slic3r
