///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_GUI_CalibrationRetractionDialog_hpp_
#define slic3r_GUI_CalibrationRetractionDialog_hpp_

#include "CalibrationAbstractDialog.hpp"
// Construct this dialog with the application and main frame; its controls create
// retraction calibration samples using whole-object printing.

class ComboBox;

namespace Slic3r { 
namespace GUI {

class CalibrationRetractionDialog : public CalibrationAbstractDialog
{

public:
    CalibrationRetractionDialog(GUI_App* app, MainFrame* mainframe) : CalibrationAbstractDialog(app, mainframe, "Retraction calibration") { create(boost::filesystem::path("calibration") / "retraction", "retraction.html", wxSize(900, 500));  }
    virtual ~CalibrationRetractionDialog() {}
    
protected:
    void create_buttons(wxStdDialogButtonSizer* sizer) override;
    void remove_slowdown(wxCommandEvent& event_args);
    // Replace the plate with retraction samples configured for whole-object printing.
    void create_geometry(wxCommandEvent& event_args);

    ComboBox* steps;
    ComboBox* nb_steps;
    //wxComboBox* start_step;
    wxTextCtrl* temp_start;
    ComboBox* decr_temp;
};

} // namespace GUI
} // namespace Slic3r

#endif
