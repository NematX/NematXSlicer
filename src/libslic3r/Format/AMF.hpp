///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/ Copyright (c) Prusa Research 2017 - 2021 Vojtěch Bubník @bubnikv, Lukáš Matěna @lukasmatena, Enrico Turri @enricoturri1966
///|/
///|/ ported from lib/Slic3r/Format/AMF.pm:
///|/ Copyright (c) Prusa Research 2017 Vojtěch Bubník @bubnikv
///|/ Copyright (c) Slic3r 2012 - 2015 Alessandro Ranellucci @alranel
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Format_AMF_hpp_
#define slic3r_Format_AMF_hpp_

#include <string>

namespace Slic3r {

class Model;
class DynamicPrintConfig;
struct ConfigSubstitutionContext;

// Load the content of an amf file into the given model and configuration.
// When profile_sections is not null, it receives the text of the profile sections metadata (untouched when the file has none).
extern bool load_amf(const char* path, DynamicPrintConfig* config, ConfigSubstitutionContext* config_substitutions, Model* model, bool check_version,
                     std::string* profile_sections = nullptr);

struct OptionStoreAmf {
    bool fullpath_sources = true;
    bool export_config = true;
    bool export_modifiers = true;
    // Text of the profile sections metadata (PresetBundle::project_profile_sections()); not written when empty or when the config is not exported.
    std::string profile_sections;
    OptionStoreAmf& set_profile_sections(std::string text) { profile_sections = std::move(text); return *this; }
    OptionStoreAmf& set_fullpath_sources(bool use_fullpath_sources) { fullpath_sources = use_fullpath_sources; return *this; }
    OptionStoreAmf& set_export_config(bool use_export_config) { export_config = use_export_config; return *this; }
    OptionStoreAmf& set_export_modifiers(bool use_export_modifiers) { export_modifiers = use_export_modifiers; return *this; }
};

// Save the given model and the config data into an amf file.
// The model could be modified during the export process if meshes are not repaired or have no shared vertices
extern bool store_amf(std::string &path, Model *model, const DynamicPrintConfig *config, OptionStoreAmf options);

} // namespace Slic3r

#endif /* slic3r_Format_AMF_hpp_ */
