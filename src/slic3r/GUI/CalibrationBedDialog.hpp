///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_GUI_CalibrationBedDialog_hpp_
#define slic3r_GUI_CalibrationBedDialog_hpp_

#include "CalibrationAbstractDialog.hpp"
// Construct this dialog with the application and main frame; its controls create
// bed calibration objects on the plater using whole-object printing.

namespace Slic3r { 
namespace GUI {

class CalibrationBedDialog : public CalibrationAbstractDialog
{

public:
    CalibrationBedDialog(GUI_App* app, MainFrame* mainframe) : CalibrationAbstractDialog(app, mainframe, "Bed leveling calibration") { create(boost::filesystem::path("calibration") / "bed_leveling", "bed_leveling.html");  }
    virtual ~CalibrationBedDialog() {}
protected:
    void create_buttons(wxStdDialogButtonSizer* sizer) override;
private:
    // Replace the plate with bed calibration objects configured for whole-object printing.
    void create_geometry(wxCommandEvent& event_args);

};

} // namespace GUI
} // namespace Slic3r

#endif
