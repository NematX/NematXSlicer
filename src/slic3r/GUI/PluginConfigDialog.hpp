///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_GUI_PluginConfigDialog_hpp_
#define slic3r_GUI_PluginConfigDialog_hpp_

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "GUI_Utils.hpp"

class wxCommandEvent;
class wxRect;
class wxWindow;

namespace Slic3r::GUI {

class PluginConfigDialogState;

// Resolve a draft checkbox operation, asking once for all inactive transitive
// dependencies. Refusal leaves requested plugins off. Disabling also removes
// consumers that would otherwise become invalid; runtime state is not changed.
std::vector<std::string> change_plugin_activation(wxWindow *parent,
    const std::vector<std::string> &current, const std::vector<std::string> &requested, bool active);

class PluginConfigDialog : public DPIDialog
{
public:
    explicit PluginConfigDialog(wxWindow *parent);
    ~PluginConfigDialog() override;

private:
    void build();
    // Copy borrowed plugin descriptions and retained group text into dialog-owned entries, preserving text marked verbatim.
    // Application data is unchanged, and allocation failures propagate.
    void build_catalog();
    // Build navigation entries for built-in producer and service steps.
    void build_navigation();
    void refresh_navigation_counts();
    void refresh_plugin_list();
    void refresh_details();
    void update_save_buttons();
    bool has_pending_changes() const;
    std::set<std::string> active_plugin_ids() const;
    bool write_active_plugins(std::string &error_message) const;
    bool save_changes();
    bool prepare_restart() const;
    void save(wxCommandEvent &event);
    void save_and_restart(wxCommandEvent &event);
    void cancel(wxCommandEvent &event);
    void discard_draft_changes();
    void on_dpi_changed(const wxRect &suggested_rect) override;

    std::unique_ptr<PluginConfigDialogState> m_state;
};

} // namespace Slic3r::GUI

#endif // slic3r_GUI_PluginConfigDialog_hpp_
