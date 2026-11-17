///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_GUI_CalibrationFlowDialog_hpp_
#define slic3r_GUI_CalibrationFlowDialog_hpp_

#include "CalibrationAbstractDialog.hpp"
// Construct this dialog with the application and main frame; its controls create
// flow calibration samples on the plater using whole-object printing.

namespace Slic3r { 
namespace GUI {

class CalibrationFlowDialog : public CalibrationAbstractDialog
{

public:
    CalibrationFlowDialog(GUI_App* app, MainFrame* mainframe) : CalibrationAbstractDialog(app, mainframe, "Flow calibration") { create(boost::filesystem::path("calibration") / "filament_flow","filament_flow.html", wxSize(900, 500));  }
    virtual ~CalibrationFlowDialog() {}
    
protected:
    void create_buttons(wxStdDialogButtonSizer* sizer) override;
    // Create whole-object flow samples starting at the requested ratio and increment.
    void create_geometry(float start, float delta);
    void create_geometry_10(wxCommandEvent& event_args);
    void create_geometry_2_5(wxCommandEvent& event_args);

};

} // namespace GUI
} // namespace Slic3r

#endif
