///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_GUI_CalibrationTempDialog_hpp_
#define slic3r_GUI_CalibrationTempDialog_hpp_

#include "CalibrationAbstractDialog.hpp"
// Construct this dialog with the application and main frame; its controls create
// temperature calibration samples using by-layer printing.

class ComboBox;

namespace Slic3r { 
namespace GUI {

class CalibrationTempDialog : public CalibrationAbstractDialog
{

public:
    CalibrationTempDialog(GUI_App* app, MainFrame* mainframe) : CalibrationAbstractDialog(app, mainframe, "Temperature calibration") { create(boost::filesystem::path("calibration") / "filament_temp", "filament_temp.html"); }
    virtual ~CalibrationTempDialog(){ }
    
protected:
    void create_buttons(wxStdDialogButtonSizer* sizer) override;
    // Replace the plate with temperature samples configured for by-layer printing.
    void create_geometry(wxCommandEvent& event_args);

    ComboBox* steps;
    ComboBox* nb_down;
    ComboBox* nb_up;

};

} // namespace GUI
} // namespace Slic3r

#endif
