///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GUI_ExportInputsDialog_hpp_
#define slic3r_GUI_ExportInputsDialog_hpp_

#include <memory>
#include <string>

#include "libslic3r/Api/host/ExportInputs.hpp"

#include "GUI_Utils.hpp"

namespace Slic3r {
namespace GUI {

class ConfigOptionsGroup;

// Check a save filename with the shared FAT-character rule, showing its warning under the borrowed parent when invalid.
bool validate_export_save_filename(wxWindow *parent, const std::string &filename);

/*
Dialog that asks the user for the inputs of an exporter
=======================================================

An exporter can ask for values during an export (see ExportInputForm). This modal dialog is the graphical requester
of such a form. It shows one field per definition, in the order of the form, and it is filled with the proposed values.

How it works:
- The option groups of the application build their fields from a configuration and its definitions (ConfigDef).
  The definitions of a form are not in the global catalogue, so the dialog owns a configuration with its own
  ConfigDef, filled from the form (ExportInputsConfig), and a ConfigOptionsGroup is built on it.
- A field writes its value into that configuration when it is validated (Enter, or the loss of the focus). "OK"
  asks the text fields to validate, so that a value still being typed is not lost, then validates string-valued
  save paths against FAT filename rules and confirms overwrites when enabled before copying values and closing with
  wxID_OK. A rejected path keeps the dialog open. "Cancel" and the closing of the window end with wxID_CANCEL
  and leave the form untouched.
- The dialog has a single outcome, kept in its state: open, validated or cancelled. Once it is not open any more,
  "OK" does nothing, and wxDialog::EndModal() is called at most once.
- request_cancel() lets the application cancel the form from the outside (the export is stopped, the application
  is closing). It marks the dialog cancelled, but the dialog really ends only when no modal window opened from it is
  above it (a file dialog, a message about an invalid value): ending a modal dialog under a nested one would
  destroy the windows the nested one is still using. It is retried when the nested window has closed.
- An exception thrown by one of the handlers of the dialog must not reach wxWidgets: the dialog keeps its message
  (error()) and ends with a cancellation, so that the caller can report it where the export can fail.
*/

// A configuration with the definitions of its own: the options of a form exist for one export only, so they are
// never registered in the global catalogue.
class ExportInputsConfig : public DynamicConfig
{
public:
    // The definitions of the options of this configuration.
    ConfigDef config_def;
    // The definitions this configuration is described by (its own, not the global ones).
    const ConfigDef *def() const override { return &config_def; }
};

class ExportInputsDialog : public DPIDialog
{
public:
    // Build fields from a caller-owned form that must outlive the dialog, fitting preferred path widths before lowering their minima.
    ExportInputsDialog(wxWindow *parent, ExportInputForm &form);
    // Releases the option group before the windows of the dialog go away.
    ~ExportInputsDialog() override;

    // Message of the exception that closed the dialog; empty when it ended normally.
    const std::string &error() const { return m_error; }

    // Cancel the form on the UI thread without background-process locks; repeated calls and calls after closing are harmless.
    void request_cancel();

protected:
    // Rescale the group and the buttons after a change of the screen resolution.
    void on_dpi_changed(const wxRect &suggested_rect) override;
    // Update the group and the buttons after a change of the system colors.
    void on_sys_color_changed() override;

private:
    // How the dialog ended or will end. A state other than Open is final.
    enum class State { Open, Validated, Cancelled };

    // Propagate typed values, validate save filenames and confirm overwrites, then copy accepted answers and close.
    void on_ok(wxCommandEvent &event);
    // Cancel from the buttons of the dialog (the user, so nothing is nested above it).
    void on_user_cancel();
    // Cancel the dialog and keep the message of the exception that forced it.
    void fail(const std::string &message);
    // End the dialog with a cancellation if it is cancelled and nothing is open above it; else remember to retry.
    void settle();
    // Call wxDialog::EndModal() once, and only while the dialog is modal.
    void end_modal(int return_code);
    // Whether a modal dialog opened from this one (a descendant of it) is open above it.
    bool has_nested_modal() const;
    // Retry a postponed cancellation when the application is idle (a nested window may have closed).
    void on_idle(wxIdleEvent &event);

    // The form that is answered; not owned.
    ExportInputForm                    &m_form;
    // The values being edited, with the definitions of the form.
    ExportInputsConfig                  m_config;
    // The fields.
    std::unique_ptr<ConfigOptionsGroup> m_group;
    // Message of the exception that ended the dialog.
    std::string                         m_error;
    // Outcome of the dialog.
    State                               m_state { State::Open };
    // Whether EndModal() was called.
    bool                                m_ended { false };
    // Number of windows opened by this dialog's own handlers (file dialog, validation) that are still open.
    int                                 m_nested_scopes { 0 };
    // Whether a cancellation is waiting for the nested windows to close.
    bool                                m_settle_pending { false };
};

} // namespace GUI
} // namespace Slic3r

#endif // slic3r_GUI_ExportInputsDialog_hpp_
