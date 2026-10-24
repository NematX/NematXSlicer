///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/*
Implementation of the dialog that asks the user for the inputs of an exporter; the mechanism is described in
ExportInputsDialog.hpp. The dialog is created and shown by BackgroundSlicingProcess on the GUI thread.

The dialog copies each request definition and proposed value into its own
configuration, then builds the ordinary settings widgets on that configuration.
Validation first commits text still being edited, then checks file paths and
copies accepted values back to the caller's form. Cancellation leaves that form
unchanged. A nested file/message dialog must finish before an external stop can
close the parent modal dialog.

Representative direct calls (branches are not a timeline):

    ExportInputsDialog::ExportInputsDialog()
        |-- m_config.set_key_value()
        |-- clone()
        |-- m_group->append_single_option_line()
        |-- m_group->activate()
        `-- m_group->reload_config()

    ExportInputsDialog::on_ok()
        |-- text->propagate_value()
        |-- validate_export_save_filename()
        |-- confirmation.ShowModal()
        |-- m_form.values.set_key_value()
        |-- end_modal()
        `-- settle()
*/
#include "ExportInputsDialog.hpp"

#include <exception>
#include <string>

#include <boost/filesystem.hpp>

#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/msgdlg.h>

#include "GUI_App.hpp"
#include "I18N.hpp"
#include "OptionsGroup.hpp"
#include "Plater.hpp"
#include "MsgDialog.hpp"
#include "wxExtensions.hpp"
#include "libslic3r/Config/AppConfig.hpp"

namespace Slic3r {
namespace GUI {

namespace {
// Space around the content of the dialog.
constexpr int border_width = 10;
} // namespace

// Check a borrowed filename with Plater's FAT rule, show the standard warning under the borrowed parent when invalid, and return its validity; conversion and dialog failures propagate.
bool validate_export_save_filename(wxWindow *parent, const std::string &filename)
{
    if (!Plater::has_illegal_filename_characters(filename))
        return true;

    MessageDialog message(parent,
        _L("The provided file name is not valid.") + "\n" +
        _L("The following characters are not allowed by a FAT file system:") + " <>:/\\|?*\"",
        _L("Invalid file name"), wxOK | wxICON_WARNING);
    message.ShowModal();
    return false;
}

// Build parent-owned fields from the caller-owned form, which must outlive the dialog, on the UI thread; preserve the preferred size while lowering path minima, and let construction failures propagate.
ExportInputsDialog::ExportInputsDialog(wxWindow *parent, ExportInputForm &form)
    : DPIDialog(parent, wxID_ANY, _L("Export settings"), wxDefaultPosition, wxSize(45 * wxGetApp().em_unit(), -1),
                wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER, "exportinputs")
    , m_form(form)
{
    SetFont(wxGetApp().normal_font());
#ifndef _WIN32
    SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));
#endif

    // A group creates only options that its configuration defines: the definitions are assigned to the ConfigDef
    // (the registration of ConfigDef::add counts options globally) and the values are copies of the proposed ones.
    for (const ConfigOptionDef &definition : form.definitions) {
        ConfigOptionDef dialog_definition = definition;
        if ((definition.gui_type == ConfigOptionDef::GUIType::save_path || definition.gui_type == ConfigOptionDef::GUIType::open_path) &&
            definition.type == coString)
            dialog_definition.full_width = true;
        m_config.config_def.options[definition.opt_key] = dialog_definition;
        m_config.set_key_value(definition.opt_key, form.values.option(definition.opt_key)->clone());
    }

    m_group = std::make_unique<ConfigOptionsGroup>(this, wxString(), &m_config);
    for (const ConfigOptionDef &definition : form.definitions)
        m_group->append_single_option_line(definition.opt_key);
    m_group->activate();
    m_group->reload_config();

    // A file dialog opened by a path field is a nested modal window the dialog cannot close; an error while choosing
    // a file ends the dialog like an error of "OK" does.
    for (const ConfigOptionDef &definition : form.definitions)
        if (PathCtrl *path = dynamic_cast<PathCtrl *>(m_group->get_field(OptionKeyIdx::scalar(definition.opt_key)))) {
            if (definition.gui_type == ConfigOptionDef::GUIType::save_path)
                path->m_defer_overwrite_confirmation = true;
            path->m_on_browse_error = [this](const std::string &message) { fail(message); };
            path->m_on_browse_modal = [this](bool opened) {
                m_nested_scopes += opened ? 1 : -1;
                if (!opened)
                    settle();
            };
        }

    wxStdDialogButtonSizer *buttons = this->CreateStdDialogButtonSizer(wxOK | wxCANCEL);
    wxButton *ok_button = static_cast<wxButton *>(this->FindWindowById(wxID_OK, this));
    wxGetApp().UpdateDarkUI(ok_button);
    ok_button->Bind(wxEVT_BUTTON, &ExportInputsDialog::on_ok, this);
    wxButton *cancel_button = static_cast<wxButton *>(this->FindWindowById(wxID_CANCEL, this));
    wxGetApp().UpdateDarkUI(cancel_button);
    // The default handlers of the cancel button and of the closing of the window call EndModal() themselves; these
    // replacements keep a single place where the dialog ends.
    cancel_button->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { on_user_cancel(); });
    this->Bind(wxEVT_CLOSE_WINDOW, [this](wxCloseEvent &) { on_user_cancel(); });
    this->Bind(wxEVT_IDLE, &ExportInputsDialog::on_idle, this);

    wxBoxSizer *top_sizer = new wxBoxSizer(wxVERTICAL);
    top_sizer->Add(m_group->sizer, 1, wxEXPAND | wxLEFT | wxTOP | wxRIGHT, border_width);
    top_sizer->Add(buttons, 0, wxEXPAND | wxALL, border_width);
    SetSizer(top_sizer);
    this->Fit();
    const wxSize preferred_size = this->GetSize();
    for (const ConfigOptionDef &definition : form.definitions)
        if (definition.gui_type == ConfigOptionDef::GUIType::save_path || definition.gui_type == ConfigOptionDef::GUIType::open_path)
            if (PathCtrl *path = dynamic_cast<PathCtrl *>(m_group->get_field(OptionKeyIdx::scalar(definition.opt_key))))
                path->set_minimum_width();
    top_sizer->SetSizeHints(this);
    this->SetSize(preferred_size);
    this->Layout();
    this->CenterOnParent();
}

ExportInputsDialog::~ExportInputsDialog() = default;

// UI thread: mark an open form cancelled and end it when no child modal is active; repeated requests keep the first outcome.
void ExportInputsDialog::request_cancel()
{
    if (m_state == State::Open)
        m_state = State::Cancelled;
    settle();
}

// On the UI thread, propagate typed values, use the shared filename warning and overwrite checks, then clone valid answers and close; rejected values stay open, while exceptions cancel the dialog and remain available to the caller.
void ExportInputsDialog::on_ok(wxCommandEvent &)
{
    if (m_state != State::Open)
        return;

    std::string error;
    bool validation_rejected = false;
    // The validation of a field may open a message above the dialog.
    ++m_nested_scopes;
    try {
        // A text field passes its value to the configuration on Enter or when it loses the focus; a value still
        // being typed has not reached it yet.
        for (const ConfigOptionDef &definition : m_form.definitions)
            if (TextCtrl *text = dynamic_cast<TextCtrl *>(m_group->get_field(OptionKeyIdx::scalar(definition.opt_key))))
                text->propagate_value();

        for (const ConfigOptionDef &definition : m_form.definitions) {
            if (definition.gui_type != ConfigOptionDef::GUIType::save_path)
                continue;
            const ConfigOptionString *path_option = m_config.option<ConfigOptionString>(definition.opt_key);
            if (path_option == nullptr)
                continue;
            const boost::filesystem::path path = into_path(from_u8(path_option->value));
            if (!validate_export_save_filename(this, path.filename().string())) {
                validation_rejected = true;
                break;
            }
            if (wxGetApp().app_config->get_show_overwrite_dialog() && boost::filesystem::is_regular_file(path)) {
                MessageDialog confirmation(this,
                    _L("The file already exists. Do you want to overwrite it?"),
                    _L("Confirm overwrite"), wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION);
                if (confirmation.ShowModal() != wxID_YES) {
                    validation_rejected = true;
                    break;
                }
            }
        }

        // The application may have cancelled the form while a message was open: the answers are then not delivered.
        if (m_state == State::Open && !validation_rejected)
            for (const ConfigOptionDef &definition : m_form.definitions)
                m_form.values.set_key_value(definition.opt_key, m_config.option(definition.opt_key)->clone());
    } catch (const std::exception &ex) {
        error = ex.what();
    } catch (...) {
        error = "Unknown error while validating the inputs.";
    }
    --m_nested_scopes;

    if (!error.empty())
        fail(error);
    else if (m_state == State::Open && !validation_rejected) {
        m_state = State::Validated;
        end_modal(wxID_OK);
    } else if (m_state != State::Open)
        settle();
}

// "Cancel" button or closing of the window: the user acts on this dialog, so nothing is open above it and it can end at once.
void ExportInputsDialog::on_user_cancel()
{
    if (m_state == State::Open)
        m_state = State::Cancelled;
    end_modal(wxID_CANCEL);
}

// An exception of a handler (OK, file dialog) cancels the dialog; only the first outcome counts, so the first message is kept.
void ExportInputsDialog::fail(const std::string &message)
{
    if (m_state == State::Open) {
        m_state = State::Cancelled;
        m_error = message;
    }
    settle();
}

void ExportInputsDialog::settle()
{
    if (m_state != State::Cancelled || m_ended)
        return;
    // The idle handler retries: a dialog opened from a field (not tracked by a scope) may be open above.
    m_settle_pending = m_nested_scopes > 0 || has_nested_modal();
    if (!m_settle_pending)
        end_modal(wxID_CANCEL);
}

// The only place that ends the dialog: EndModal() runs at most once, and only while the dialog is modal (it is not before
// ShowModal() or after it returned), whoever asks (OK, the buttons, the application).
void ExportInputsDialog::end_modal(int return_code)
{
    if (m_ended || !IsModal())
        return;
    m_ended = true;
    EndModal(return_code);
}

bool ExportInputsDialog::has_nested_modal() const
{
    // Only the modal dialogs opened from this one (it is among their ancestors) are above it. Another modal dialog of the
    // application, such as the preferences the form was opened over, is below it and must not delay the cancellation.
    // A dialog opened without a parent cannot be told from one below, so it is not seen.
    for (wxWindowList::compatibility_iterator node = wxTopLevelWindows.GetFirst(); node; node = node->GetNext()) {
        const wxDialog *other = dynamic_cast<const wxDialog *>(node->GetData());
        if (other == nullptr || other == this || !other->IsModal())
            continue;
        for (const wxWindow *ancestor = other->GetParent(); ancestor != nullptr; ancestor = ancestor->GetParent())
            if (ancestor == this)
                return true;
    }
    return false;
}

void ExportInputsDialog::on_idle(wxIdleEvent &event)
{
    event.Skip();
    if (m_settle_pending)
        settle();
}
void ExportInputsDialog::on_dpi_changed(const wxRect &)
{
    m_group->msw_rescale();
    msw_buttons_rescale(this, em_unit(), {wxID_OK, wxID_CANCEL});
    Fit();
    Refresh();
}

void ExportInputsDialog::on_sys_color_changed()
{
    m_group->sys_color_changed();
    wxGetApp().UpdateDarkUI(static_cast<wxButton *>(this->FindWindowById(wxID_OK, this)));
    wxGetApp().UpdateDarkUI(static_cast<wxButton *>(this->FindWindowById(wxID_CANCEL, this)));
}

} // namespace GUI
} // namespace Slic3r
