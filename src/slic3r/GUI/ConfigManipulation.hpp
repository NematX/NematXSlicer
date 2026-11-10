///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/ Copyright (c) Prusa Research 2019 - 2021 Oleksandra Iushchenko @YuSanka, Vojtěch Bubník @bubnikv
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_ConfigManipulation_hpp_
#define slic3r_ConfigManipulation_hpp_

/*	 Class for validation config options
 *	 and update (enable/disable) IU components
 *
 *	 Used for config validation for global config (Print Settings Tab)
 *	 and local config (overrides options on sidebar)
 * */

#include <functional>
#include <string>
#include <vector>

#include <boost/any.hpp>

#include "libslic3r/Api/plugin/c/slic3r_config_types.h"

#include "Field.hpp"

class wxWindow;

namespace Slic3r {

class DynamicPrintConfig;
class ModelConfig;

namespace GUI {

class ConfigManipulation
{
    bool                is_msg_dlg_already_exist{ false };

    bool                m_is_initialized_support_material_overhangs_queried{ false };
    bool                m_support_material_overhangs_queried{ false };

    // function to loading of changed configuration 
    std::function<void()>                                       load_config = nullptr;
    std::function<void (const std::string&, bool toggle, int opt_index)>   cb_toggle_field = nullptr;
    // callback to propagation of changed value, if needed 
    std::function<void(const std::string&, const boost::any&)>  cb_value_change = nullptr;
    ModelConfig* local_config = nullptr;
    wxWindow*    m_msg_dlg_parent {nullptr};

    // visible_settings is the field list of the refreshed page. When non-null
    // and non-empty, only the activation rules of those settings are evaluated.
    void    apply_plugin_gui_rules(DynamicPrintConfig* config, int current_index = -1,
                                   raw_option_preset_type scope = RAW_PRESET_TYPE_FFF_PRINT,
                                   const std::vector<OptionKeyIdx>* visible_settings = nullptr);

public:
    ConfigManipulation(std::function<void()> load_config,
        std::function<void(const std::string&, bool toggle, int opt_index)> cb_toggle_field,
        std::function<void(const std::string&, const boost::any&)>  cb_value_change,
        ModelConfig* local_config = nullptr,
        wxWindow* msg_dlg_parent  = nullptr) :
        load_config(load_config),
        cb_toggle_field(cb_toggle_field),
        cb_value_change(cb_value_change),
        m_msg_dlg_parent(msg_dlg_parent),
        local_config(local_config) {}
    ConfigManipulation() {}

    ~ConfigManipulation() {
        load_config = nullptr;
        cb_toggle_field = nullptr;
        cb_value_change = nullptr;
    }

    void    apply(DynamicPrintConfig* config, DynamicPrintConfig* new_config);
    void    toggle_field(const std::string& field_key, const bool toggle, int opt_index = -1);

    // FFF print
    void    update_print_fff_config(DynamicPrintConfig* config, const bool is_global_config = false);
    void    toggle_print_fff_options(DynamicPrintConfig* config,
                                     const std::vector<OptionKeyIdx>* visible_settings = nullptr);

    // FFF printer
    void    update_printer_fff_config(DynamicPrintConfig* config, const bool is_global_config = false);
    // Enable printer fields from the borrowed settings and plugin rules; visible_settings restricts the rule evaluation, and callbacks update the page.
    void    toggle_printer_fff_options(DynamicPrintConfig* config,
                                       const std::vector<OptionKeyIdx>* visible_settings = nullptr);

    // SLA print
    void    toggle_print_sla_options(DynamicPrintConfig* config,
                                     const std::vector<OptionKeyIdx>* visible_settings = nullptr);

    bool    is_initialized_support_material_overhangs_queried() { return m_is_initialized_support_material_overhangs_queried; }
    void    initialize_support_material_overhangs_queried(bool queried)
    {
        m_is_initialized_support_material_overhangs_queried = true;
        m_support_material_overhangs_queried = queried;
    }
};

} // GUI
} // Slic3r

#endif /* slic3r_ConfigManipulation_hpp_ */
