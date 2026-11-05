///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2017 - 2023 Oleksandra Iushchenko @YuSanka, Enrico Turri @enricoturri1966, Lukáš Matěna @lukasmatena, Vojtěch Bubník @bubnikv, David Kocík @kocikdav, Vojtěch Král @vojtechkral
///|/ Copyright (c) 2019 John Drake @foxox
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_PresetBundle_hpp_
#define slic3r_PresetBundle_hpp_

#include <array>
#include <memory>
#include <unordered_map>

#include <boost/filesystem/path.hpp>

#include "AppConfig.hpp"
#include "libslic3r/enum_bitmask.hpp"
#include "Preset.hpp"

/*
PresetBundle owns the profile collections and project configuration.
Its collections cover FFF print/filament/printer profiles, SLA profiles and physical printers.
Load user or vendor profiles, or import configuration with load_config_model()/load_config_file().
load_system_presets_from() accepts an explicit directory of vendor INI bundles.
A vendor bundle groups a manufacturer's profiles in one INI file.

After selecting profiles, full_config() builds their combined settings.
For project files, project_profile_sections() supplies separate INI sections
holding each selected profile's known metadata, such as inheritance.

To work on a reduced FFF catalog, collect names in PresetSubset and call subset().
Select the profiles on that copy, rebuild extruders_filaments with
update_multi_material_filament_presets(), then use full_config().
The copy keeps the profiles needed for selection. It does not reproduce SLA
or physical-printer selections; those collections remain at their defaults.

Construct a bundle, load the required profiles, then use its selected presets to compose full_config(). For project
saving, obtain project_profile_sections() as well so loading can restore metadata absent from the flat configuration.

When building a reduced bundle, keep the returned copy alive for as long as any
borrowed profile or vendor pointer into it is used. Rebuild its filament slots
after selection before composing the effective FFF configuration; copying a
catalogue alone does not select a new multi-material combination.
*/


namespace Slic3r {

// Bundle of Print + Filament + Printer presets.
class PresetBundle
{
public:
    PresetBundle();
    PresetBundle(const PresetBundle &rhs);
    PresetBundle& operator=(const PresetBundle &rhs);

    // Remove all the presets but the "-- default --".
    // Optionally remove all the files referenced by the presets from the user profile directory.
    void            reset(bool delete_files);

    // Reset this bundle, then load vendor INI files in filename order without changing data_dir().
    // Use the GUI's profile-loading and inheritance rules. The first loaded definition wins a duplicate name.
    // Return substitutions and accumulated bundle errors as a pair. Directory-access errors may throw.
    std::pair<PresetsConfigSubstitutions, std::string> load_system_presets_from(
        const std::string &vendor_directory, ForwardCompatibilitySubstitutionRule compatibility_rule);

    void            setup_directories();
    void            import_newer_configs(const std::string& from);

    struct PresetPreferences {
        std::string printer_model_id;// name of a preferred printer model
        std::string printer_variant; // name of a preferred printer variant
        std::string filament;        // name of a preferred filament preset
        std::string sla_material;    // name of a preferred sla_material preset
    };

    // Load ini files of all types (print, filament, printer) from Slic3r::data_dir() / presets.
    // Load selections (current print, current filaments, current printer) from config.ini
    // select preferred presets, if any exist
    PresetsConfigSubstitutions load_presets(AppConfig &config, ForwardCompatibilitySubstitutionRule rule,
                                            const PresetPreferences& preferred_selection = PresetPreferences());

    // Export selections (current print, current filaments, current printer) into config.ini
    void            export_selections(AppConfig &config);

    PresetCollection            fff_prints;
    PresetCollection            sla_prints;
    PresetCollection            filaments;
    PresetCollection            sla_materials;
    PresetCollection&           prints(PrinterTechnology pt)          { return pt == ptFFF ? this->fff_prints : this->sla_prints; }
    const PresetCollection&     prints(PrinterTechnology pt)    const { return pt == ptFFF ? this->fff_prints : this->sla_prints; }
	PresetCollection& 			materials(PrinterTechnology pt)       { return pt == ptFFF ? this->filaments : this->sla_materials; }
	const PresetCollection& 	materials(PrinterTechnology pt) const { return pt == ptFFF ? this->filaments : this->sla_materials; }
    PrinterPresetCollection     printers;
    PhysicalPrinterCollection   physical_printers;

    // Filament presets per extruder for a multi-extruder or multi-material print.
    // extruders_filaments.size() should be the same as printers.get_edited_preset().config.nozzle_diameter.size()
    std::vector<ExtruderFilaments> extruders_filaments;
    void cache_extruder_filaments_names();
    void reset_extruder_filaments();

    PresetCollection&           get_presets(Preset::Type preset_type);

    // The project configuration values are kept separated from the print/filament/printer preset,
    // they are being serialized / deserialized from / to the .amf, .3mf, .config, .gcode, 
    // and they are being used by slicing core.
    DynamicPrintConfig          project_config;

    // There will be an entry for each system profile loaded, 
    // and the system profiles will point to the VendorProfile instances owned by PresetBundle::vendors.
    VendorMap                   vendors;

    struct ObsoletePresets {
        std::vector<std::string> fff_prints;
        std::vector<std::string> sla_prints;
        std::vector<std::string> filaments;
        std::vector<std::string> sla_materials;
        std::vector<std::string> printers;
    };
    ObsoletePresets             obsolete_presets;

    std::set<std::string>       tmp_installed_presets;

    bool                        has_defauls_only() const 
        { return fff_prints.has_defaults_only() && filaments.has_defaults_only() && printers.has_defaults_only(); }

    DynamicPrintConfig          full_config() const;

    // Return the ini text of the project profile sections: one "[<type>:<name>]" section per selected profile
    // (print, each distinct filament, printer; or the SLA equivalents), holding the preset metadata (inherits,
    // compatible_*, default_*_profile) of the selected preset (its edited copy when it is the active one). Meant for the project files (3MF, AMF), whose flat
    // configuration does not carry these keys.
    std::string                 project_profile_sections() const;

    // Load user configuration and store it into the user profiles.
    // This method is called by the configuration wizard.
    void                        load_config_from_wizard(const std::string &name, DynamicPrintConfig config)
        { this->load_config_file_config(name, false, std::move(config)); }

    // Load configuration that comes from a model file containing configuration, such as 3MF et al.
    // This method is called by the Plater.
    // profile_sections is the text written by project_profile_sections(); each loaded preset receives the metadata
    // of the section named after its original profile name. An empty text brings no metadata: the presets keep the
    // metadata values found in the configuration itself, if any.
    void                        load_config_model(const std::string &name, DynamicPrintConfig config, const std::string &profile_sections = std::string())
        { this->load_config_file_config(name, true, std::move(config), profile_sections); }

    // Load an external config file containing the print, filament and printer presets.
    // Instead of a config file, a G-code may be loaded containing the full set of parameters.
    // In the future the configuration will likely be read from an AMF file as well.
    // If the file is loaded successfully, its print / filament / printer profiles will be activated.
    ConfigSubstitutions         load_config_file(const std::string &path, ForwardCompatibilitySubstitutionRule compatibility_rule, bool from_prusa = false);

    // Load a config bundle file, into presets and store the loaded presets into separate files
    // of the local configuration directory.
    // Load settings into the provided settings instance.
    // Activate the presets stored in the config bundle.
    // Returns the number of presets loaded successfully.
    enum LoadConfigBundleAttribute { 
        // Save the profiles, which have been loaded.
        SaveImported,
        // Delete all old config profiles before loading.
        ResetUserProfile,
        // Load a system config bundle.
        LoadSystem,
        LoadVendorOnly,
        //apply import rule from prusa
        ConvertFromPrusa,
    };
    using LoadConfigBundleAttributes = enum_bitmask<LoadConfigBundleAttribute>;
    // Load the config bundle based on the flags.
    // Don't do any config substitutions when loading a system profile, perform and report substitutions otherwise.
    std::pair<PresetsConfigSubstitutions, size_t> load_configbundle(
        const std::string &path, LoadConfigBundleAttributes flags, ForwardCompatibilitySubstitutionRule compatibility_rule);

    // Export a config bundle file containing all the presets and the names of the active presets.
    void                        export_configbundle(const std::string &path, bool export_system_settings = false, bool export_physical_printers = false);

    // Enable / disable the "- default -" preset.
    void                        set_default_suppressed(bool default_suppressed);

    // Set the filament preset name. As the name could come from the UI selection box, 
    // an optional "(modified)" suffix will be removed from the filament name.
    void                        set_filament_preset(size_t idx, const std::string &name);

    // Read out the number of extruders from an active printer preset,
    // update size and content of filament_presets.
    void                        update_multi_material_filament_presets();

    void                        update_filaments_compatible(PresetSelectCompatibleType select_other_filament_if_incompatible, int extruder_idx = -1);

    // Update the is_compatible flag of all print and filament presets depending on whether they are marked
    // as compatible with the currently selected printer (and print in case of filament presets).
    // Also updates the is_visible flag of each preset.
    // If select_other_if_incompatible is true, then the print or filament preset is switched to some compatible
    // preset if the current print or filament preset is not compatible.
    void                        update_compatible(PresetSelectCompatibleType select_other_print_if_incompatible, PresetSelectCompatibleType select_other_filament_if_incompatible);
    void                        update_compatible(PresetSelectCompatibleType select_other_if_incompatible) { this->update_compatible(select_other_if_incompatible, select_other_if_incompatible); }

    // Names of print, filament and printer profiles to request when copying a reduced FFF catalog.
    struct PresetSubset {
        std::set<std::string> prints;
        std::set<std::string> filaments;
        std::set<std::string> printers;
    };
    // Copy requested FFF profiles, resolved old names and parent chains into an independent bundle.
    // Also keep selection fallbacks, the required vendor records and project settings.
    // SLA and physical-printer collections remain at their defaults.
    // Select FFF profiles on the copy, then rebuild extruders_filaments before composing its configuration.
    PresetBundle subset(const PresetSubset &names) const;
    // Return filament names compatible with at least one extruder. Exclude default profiles.
    // Reuse ExtruderFilaments::update_compatible() to filter template profiles like the GUI.
    // printer and optional print come from this catalog, or copies preserving its vendor identity.
    // Do not change the catalog's flags, selected profiles or edited values.
    std::set<std::string> compatible_filament_names(const Preset &printer, const Preset *print) const;

    // Set the is_visible flag for printer vendors, printer models and printer variants
    // based on the user configuration.
    // If the "vendor" section is missing, enable all models and variants of the particular vendor.
    void                        load_installed_printers(const AppConfig &config);

    const std::string&          get_preset_name_by_alias(const Preset::Type& preset_type, const std::string& alias, int extruder_id = -1);

    // Save current preset of a provided type under a new name. If the name is different from the old one,
    // Unselected option would be reverted to the beginning values
    void                        save_changes_for_preset(const std::string& new_name, Preset::Type type, const std::vector<std::string>& unselected_options);
    // Transfer options form preset_from_name preset to preset_to_name preset and save preset_to_name preset as new new_name preset
    // Return false, if new preset wasn't saved
    bool                        transfer_and_save(Preset::Type type, const std::string& preset_from_name, const std::string& preset_to_name,
                                                  const std::string& new_name, const std::vector<std::string>& options);


    static std::array<Preset::Type, 3>  types_list(PrinterTechnology pt) {
        if (pt == ptFFF)
            return  { Preset::TYPE_PRINTER, Preset::TYPE_FFF_PRINT, Preset::TYPE_FFF_FILAMENT };
        return      { Preset::TYPE_PRINTER, Preset::TYPE_SLA_PRINT, Preset::TYPE_SLA_MATERIAL };
    }

private:
    std::pair<PresetsConfigSubstitutions, std::string> load_system_presets(ForwardCompatibilitySubstitutionRule compatibility_rule);
    // Merge one vendor's presets with the other vendor's presets, report duplicates.
    std::vector<std::string>    merge_presets(PresetBundle &&other);
    // Update renamed_from and alias maps of system profiles.
    void 						update_system_maps();
    // Update alias maps
    void 						update_alias_maps();

    // Set the is_visible flag for filaments and sla materials,
    // apply defaults based on enabled printers when no filaments/materials are installed.
    void                        load_installed_filaments(AppConfig &config);
    void                        load_installed_sla_materials(AppConfig &config);

    // Load selections (current print, current filaments, current printer) from config.ini
    // This is done just once on application start up.
    void                        load_selections(AppConfig &config, const PresetPreferences& preferred_selection = PresetPreferences());

    // Load print, filament & printer presets from a config. If it is an external config, then the name is extracted from the external path.
    // and the external config is just referenced, not stored into user profile directory.
    // If it is not an external config, then the config will be stored into the user profile directory.
    void                        load_config_file_config(const std::string &name_or_path, bool is_external, DynamicPrintConfig &&config,
                                                        const std::string &profile_sections = std::string());
    ConfigSubstitutions         load_config_file_config_bundle_dont_save(
        const std::string &path, const boost::property_tree::ptree &tree, ForwardCompatibilitySubstitutionRule compatibility_rule, bool from_prusa = false);

    DynamicPrintConfig          full_fff_config() const;
    DynamicPrintConfig          full_sla_config() const;
};

ENABLE_ENUM_BITMASK_OPERATORS(PresetBundle::LoadConfigBundleAttribute)

// Copies bed texture and model files to 'data_dir()\printer' folder, if needed
// and updates the config accordingly
extern void copy_bed_model_and_texture_if_needed(DynamicPrintConfig& config);

} // namespace Slic3r

#endif /* slic3r_PresetBundle_hpp_ */
