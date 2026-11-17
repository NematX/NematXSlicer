///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "CalibrationTempDialog.hpp"

#include <wx/display.h>
#include <wx/file.h>
#include <wx/scrolwin.h>
#include <wx/wupdlock.h>

#include "libslic3r/Config/AppConfig.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Utils.hpp"

#include "GLCanvas3D.hpp"
#include "GUI.hpp"
#include "GUI_ObjectList.hpp"
#include "I18N.hpp"
#include "Plater.hpp"
#include "Tab.hpp"
#include "Widgets/ComboBox.hpp"
#include "wxExtensions.hpp"
/*
Provides temperature calibration samples to the plater. Geometry creation assigns
the temperature settings and explicitly selects layer-by-layer printing.
    CalibrationTempDialog::create_geometry() -> prepare samples and settings
        set_deserialize_strict() -> select By layer explicitly
*/
#if ENABLE_SCROLLABLE
static wxSize get_screen_size(wxWindow* window)
{
    const auto idx = wxDisplay::GetFromWindow(window);
    wxDisplay display(idx != wxNOT_FOUND ? idx : 0u);
    return display.GetClientArea().GetSize();
}
#endif // ENABLE_SCROLLABLE

namespace Slic3r {
namespace GUI {

void CalibrationTempDialog::create_buttons(wxStdDialogButtonSizer* buttons){
    const wxSize size(6 * em_unit(), wxDefaultCoord);
    wxString choices_steps[] = { "5","10","15","20" };
    //steps = new wxComboBox(this, wxID_ANY, wxString{ "10" }, wxDefaultPosition, wxDefaultSize, 4, choices_steps);
    steps = new ComboBox(this, wxID_ANY, wxString{ "10" }, wxDefaultPosition, size, 4, choices_steps);
    steps->SetToolTip(_L("Select the step in celcius between two tests.\nNote that only multiple of 5 are engraved on the part."));
    steps->SetSelection(1);
    wxString choices_nb[] = { "0","1","2","3","4","5","6","7" };
    //nb_down = new wxComboBox(this, wxID_ANY, wxString{ "2" }, wxDefaultPosition, wxDefaultSize, 8, choices_nb);
    nb_down = new ComboBox(this, wxID_ANY, wxString{ "2" }, wxDefaultPosition, size, 8, choices_nb);
    nb_down->SetToolTip(_L("Select the number of tests with lower temperature than the current one."));
    nb_down->SetSelection(2);
    //nb_up = new wxComboBox(this, wxID_ANY, wxString{ "2" }, wxDefaultPosition, wxDefaultSize, 8, choices_nb);
    nb_up = new ComboBox(this, wxID_ANY, wxString{ "2" }, wxDefaultPosition, size, 8, choices_nb);
    nb_up->SetToolTip(_L("Select the number of tests with higher temperature than the current one."));
    nb_up->SetSelection(2);

    buttons->Add(new wxStaticText(this, wxID_ANY, _L("Nb down:")));
    buttons->Add(nb_down);
    buttons->AddSpacer(15);
    buttons->Add(new wxStaticText(this, wxID_ANY, _L("Nb up:")));
    buttons->Add(nb_up);
    buttons->AddSpacer(40);
    buttons->Add(new wxStaticText(this, wxID_ANY, _L("Steps:")));
    buttons->Add(steps);
    buttons->AddSpacer(40);

    wxButton* bt = new wxButton(this, wxID_FILE1, _L("Generate"));
    bt->Bind(wxEVT_BUTTON, &CalibrationTempDialog::create_geometry, this);
    buttons->Add(bt);
}

void CalibrationTempDialog::create_geometry(wxCommandEvent& event_args) {
    if (!require_plan_builder(false))
        return;
    Plater* plat = this->main_frame->plater();
    Model& model = plat->model();
    if (!plat->new_project(L("Temperature calibration")))
        return;
    // wait for slicing end if needed
    wxGetApp().Yield();
    
    std::unique_ptr<wxWindowUpdateLocker> freeze_gui = std::make_unique<wxWindowUpdateLocker>(this);
    std::vector<size_t> objs_idx = plat->load_files(std::vector<std::string>{
            (boost::filesystem::path(Slic3r::resources_dir()) / "calibration" / "filament_temp" / "Smart_compact_temperature_calibration_item.amf").string()},
        LoadFileOption::LoadModel | LoadFileOption::DontUpdateDirs);

    assert(objs_idx.size() == 1);
    const DynamicPrintConfig* print_config = this->gui_app->get_tab(Preset::TYPE_FFF_PRINT)->get_config();
    const DynamicPrintConfig* filament_config = this->gui_app->get_tab(Preset::TYPE_FFF_FILAMENT)->get_config();
    const DynamicPrintConfig* printer_config = this->gui_app->get_tab(Preset::TYPE_PRINTER)->get_config();

    // -- get temps
    const ConfigOptionInts* temperature_config = filament_config->option<ConfigOptionInts>("temperature");
    const ConfigOptionInts* first_layer_temperature_config = filament_config->option<ConfigOptionInts>("first_layer_temperature");
    // The first layer keeps the filament's first-layer temperature (its normal one when unset),
    // because the object's print_temperature (bottom floor) would otherwise win on that layer.
    const int first_layer_temperature = first_layer_temperature_config->get_at(0) > 0 ?
        first_layer_temperature_config->get_at(0) : temperature_config->get_at(0);
    assert(temperature_config->size() >= 1);
    long nb_items_up = 1;
    if (!nb_up->GetValue().ToLong(&nb_items_up)) {
        nb_items_up = 2;
    }
    long nb_items_down = 1;
    if (!nb_down->GetValue().ToLong(&nb_items_down)) {
        nb_items_down = 2;
    }
    int16_t temperature = 5 * (temperature_config->get_at(0) / 5);
    long step_temp = 1;
    if (!steps->GetValue().ToLong(&step_temp)) {
        step_temp = 10;
    }
    size_t nb_items = 1 + nb_items_up + nb_items_down;
    //start at the highest temp
    temperature = temperature + step_temp * nb_items_up;
    
    /// --- scale ---
    //model is created for a 0.4 nozzle, scale xy with nozzle size.
    const ConfigOptionFloats* nozzle_diameter_config = printer_config->option<ConfigOptionFloats>("nozzle_diameter");
    assert(nozzle_diameter_config->size() > 0);
    float nozzle_diameter = nozzle_diameter_config->get_at(0);
    float xyzScale = nozzle_diameter / 0.4;
    //do scaling
    if (xyzScale < 0.9 || 1.1 < xyzScale) {
        model.objects()[objs_idx[0]].scale(xyzScale, xyzScale * 0.5, xyzScale);
    } else {
        xyzScale = 1;
        model.objects()[objs_idx[0]].scale(xyzScale, xyzScale * 0.5, xyzScale);
    }
    
    // it's rotated but not around the good origin: correct that
    double init_z_rotate_angle = Geometry::deg2rad(plat->config()->opt_float("init_z_rotate"));
    Matrix3d rot_matrix = Eigen::Quaterniond(Eigen::AngleAxisd(init_z_rotate_angle, Vec3d{0,0,1})).toRotationMatrix();
    auto     translate_from_rotation = [&rot_matrix, &model, &objs_idx](int idx, const Vec3d &translation) {
            ModelVolume *vol_parent = model.objects()[objs_idx[idx]].volumes[model.objects()[objs_idx[idx]].volumes.size()-2];
            ModelVolume *vol = model.objects()[objs_idx[idx]].volumes[model.objects()[objs_idx[idx]].volumes.size()-1];
            //Geometry::Transformation trsf = vol->get_transformation();
            //Vec3d rotxtrans = rot_matrix * translation;
            //Vec3d  offset_reste =  trsf.get_offset()- translation;
            //Vec3d  tot = rot_matrix * translation + trsf.get_offset()- translation;
            //Vec3d  tot2 =  translation + trsf.get_offset()- rot_matrix * translation;
            //trsf.set_offset( (rot_matrix *translation) - translation + trsf.get_offset());
            Geometry::Transformation trsf = vol->get_transformation();
            trsf.set_offset( (rot_matrix *translation) + vol_parent->get_offset());
            vol->set_transformation(trsf);
        };

    //add 8 others
    float zshift = (1 - xyzScale) / 2;
    if (temperature > 175 && temperature < 290 && temperature%5==0) {
        Vec3d translate{ 0 - xyzScale * 3.75, -xyzScale * 2.7, xyzScale * (0 * 10 - 2.45) };
        add_part(&model.objects()[objs_idx[0]], (boost::filesystem::path(Slic3r::resources_dir()) / "calibration" / "filament_temp" / ("t"+std::to_string(temperature)+".amf")).string(),
            translate, Vec3d{ xyzScale, xyzScale, xyzScale * 0.43 });
        translate_from_rotation(0, translate);
    }
    for (int16_t i = 1; i < nb_items; i++) {
        add_part(&model.objects()[objs_idx[0]], (boost::filesystem::path(Slic3r::resources_dir()) / "calibration" / "filament_temp" / ("Smart_compact_temperature_calibration_item.amf")).string(),
            Vec3d{ 0,0, i * 10 * xyzScale }, Vec3d{ xyzScale, xyzScale * 0.5, xyzScale });
        int sub_temp = temperature - i * step_temp;
        if (sub_temp > 175 && sub_temp < 290 && sub_temp % 5 == 0) {
            Vec3d translate{ 0 - xyzScale * 3.75, -xyzScale * 2.7, xyzScale * (0 * 10 - 2.45) };
            add_part(&model.objects()[objs_idx[0]], (boost::filesystem::path(Slic3r::resources_dir()) / "calibration" / "filament_temp" / ("t" + std::to_string(sub_temp) + ".amf")).string(),
                translate, Vec3d{ xyzScale, xyzScale, xyzScale * 0.43 });
            translate_from_rotation(0, translate);
        }
    }


    /// --- translate ---

    /// --- main config, please modify object config when possible ---
    DynamicPrintConfig new_print_config = *print_config; //make a copy
    new_print_config.set_deserialize_strict("print_plan_builder", "ordering.plan_builder.by_layer");
    
    // the object temperature is the first floor's
    model.objects()[objs_idx[0]].config.set_key_value("print_temperature", new ConfigOptionInt(temperature));
    model.objects()[objs_idx[0]].config.set_key_value("print_first_layer_temperature", new ConfigOptionInt(first_layer_temperature));

    /// --- custom config ---
    float brim_width = print_config->option<ConfigOptionFloat>("brim_width")->value;
    if (brim_width < nozzle_diameter * 8) {
        model.objects()[objs_idx[0]].config.set_key_value("brim_width", new ConfigOptionFloat(nozzle_diameter * 8));
    }
    if (PrintConfigDef::instance().get("brim_ears") != nullptr)
        model.objects()[objs_idx[0]].config.set_key_value("brim_ears", new ConfigOptionBool(false));
    model.objects()[objs_idx[0]].config.set_key_value("perimeters", new ConfigOptionInt(1));
    model.objects()[objs_idx[0]].config.set_key_value("extra_perimeters_on_overhangs", new ConfigOptionBool(true));
    model.objects()[objs_idx[0]].config.set_key_value("bottom_solid_layers", new ConfigOptionInt(2));
    model.objects()[objs_idx[0]].config.set_key_value("top_solid_layers", new ConfigOptionInt(3)); 
    model.objects()[objs_idx[0]].config.set_key_value("gap_fill_enabled", new ConfigOptionBool(false)); 
    model.objects()[objs_idx[0]].config.set_key_value("thin_perimeters", new ConfigOptionPercent(100));
    model.objects()[objs_idx[0]].config.set_key_value("layer_height", new ConfigOptionFloat(nozzle_diameter / 2));
    model.objects()[objs_idx[0]].config.set_key_value("fill_density", new ConfigOptionPercent(7));
    model.objects()[objs_idx[0]].config.set_deserialize_strict("solid_fill_pattern", "rectilinear");
    model.objects()[objs_idx[0]].config.set_key_value("infill_filled_solid", new ConfigOptionBool(true));
    model.objects()[objs_idx[0]].config.set_deserialize_strict("top_fill_pattern", "rectilinear");
    model.objects()[objs_idx[0]].config.set_key_value("infill_filled_bottom", new ConfigOptionBool(true));
    //disable ironing post-process, it only slow down things
    model.objects()[objs_idx[0]].config.set_key_value("ironing", new ConfigOptionBool(false));

    /// --- one height range per floor above the first carries its temperature ---
    // A range applies to layers whose middle is inside [low, high), so each floor
    // changes temperature at its own first layer. get_default_layer_config() gives
    // the layer_height (read by slicing and by the object list) and extruder (needed
    // to list the range) of a user-made range; it must run after the object's
    // layer_height is set. The last range extends one floor higher so a model top
    // above the last nominal floor keeps that floor's temperature (ranges are
    // clipped at the object height).
    ModelObject &tower = model.objects()[objs_idx[0]];
    for (int16_t i = 1; i < nb_items; i++) {
        const double low = double(i) * 10. * xyzScale;
        const double high = double(i + 1 < nb_items ? i + 1 : i + 2) * 10. * xyzScale;
        ModelConfig &floor_config = tower.layer_config_ranges[{ low, high }];
        floor_config.assign_config(this->gui_app->obj_list()->get_default_layer_config(int(objs_idx[0])));
        floor_config.set_key_value("print_temperature", new ConfigOptionInt(int(temperature - i * step_temp)));
    }

    //update plater
    //GLCanvas3D::set_warning_freeze(false);
    this->gui_app->get_tab(Preset::TYPE_FFF_PRINT)->load_config(new_print_config);
    plat->on_config_change(new_print_config);
    //this->gui_app->get_tab(Preset::TYPE_PRINTER)->load_config(new_printer_config);
    //plat->on_config_change(new_printer_config);
    plat->changed_objects(objs_idx);
    this->gui_app->get_tab(Preset::TYPE_FFF_PRINT)->update_dirty();
    //this->gui_app->get_tab(Preset::TYPE_PRINTER)->update_dirty();
    plat->is_preview_shown();
    //update everything, easier to code.
    ObjectList* obj = this->gui_app->obj_list();
    obj->update_after_undo_redo();
    freeze_gui.reset();

    plat->reslice();
}

} // namespace GUI
} // namespace Slic3r
