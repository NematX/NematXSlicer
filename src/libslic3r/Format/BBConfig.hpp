///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Format_BBconfig_hpp_
#define slic3r_Format_BBconfig_hpp_

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/miniz_extension.hpp"

#ifdef __APPLE__
    #include <boost/filesystem.hpp>
    #include <boost/nowide/fstream.hpp>

/*
Use read_json_file_bambu() to import a JSON settings file, or
convert_settings_from_bambu() for already-read key/value text. These functions
fill the caller's DynamicPrintConfig and report substitutions through its
configuration-loading context. The tables below are shared read-only data;
they are not the mutable configuration produced by an import.
*/
    typedef boost::filesystem::path std_path;
    typedef boost::nowide::ifstream std_ifstream;
    #define GET_STD_PATH_FOR_IFSTREAM(PARAM) PARAM.string()
#else
    #include <filesystem>
    #include <fstream>
    typedef std::filesystem::path std_path;
    typedef std::ifstream std_ifstream;
    #define GET_STD_PATH_FOR_IFSTREAM(PARAM) PARAM
#endif

namespace Slic3r {
struct ConfigSubstitutionContext;
class DynamicPrintConfig;
class Model;
class ModelConfigObject;

/*
Bambu / Orca <-> SuperSlicer translation tables
================================================

The tables below are the single source of truth for the correspondence between a Bambu (Orca) setting
and the SuperSlicer one. BBConfig.cpp reads them to import a Bambu project; BambuConfigExport.cpp reads
the same key table the other way round to write one. They are built once, on first use, by a function
local static, so concurrent readers (a GUI import and a background export) are safe and the tables never
grow when they are asked for again.
*/
namespace BBConfiguration {

// The kinds of preset a Bambu custom-variable key is stored in (flags, combined with |).
// All legacy flags evaluate to zero, so bit tests cannot route custom variables; firmware variables use explicit routing.
enum BBSettingType : uint8_t {
    bbstFFF_PRINT = 0 << 0,
    bbstFFF_FILAMENT = 0 << 1,
    bbstFFF_PRINTER = 0 << 2,
    bbstSLA_PRINT = 0 << 3,
    bbstSLA_MATERIAL = 0 << 4,
    bbstSLA_PRINTER = 0 << 5,
};

struct Tables
{
    // Bambu setting key -> SuperSlicer setting key. An empty target means "ignored on import".
    std::map<std::string, std::string> key_translation_map;
    // SuperSlicer setting key -> (Bambu value -> SuperSlicer value). Only an import table: several Bambu
    // values may give the same SuperSlicer value, so it is not reversible.
    std::map<std::string, std::map<std::string, std::string>> value_translation_map;
    // Syntactic forms of a custom G-code ("[name]" and "{...}" shapes), replaced as text, in order.
    std::vector<std::pair<std::string, std::string>> special_gcode_replace;
    // Bambu identifiers of a custom G-code that need another meaning than a renamed setting, replaced
    // only when they are a whole identifier.
    std::vector<std::pair<std::string, std::string>> identifier_gcode_replace;
    // Bambu keys that hold custom variables.
    std::map<std::string, BBSettingType> key_custom_settings_translation_map;
};

// Return the tables, built on the first call.
const Tables &tables();

} // namespace BBConfiguration

// Import JSON settings with scalar/vector and Prusa legacy conversions into the configuration.
bool read_json_file_bambu(const std_path &temp_file,
                          DynamicPrintConfig &         config,
                          ConfigSubstitutionContext &  config_substitutions,
                          bool                         with_phony);

std::map<std::string, std::string> read_ini_file_bambu(const std_path &temp_file);

// Import serialized settings into the print configuration without the JSON-only from_prusa pass.
bool convert_settings_from_bambu(std::map<std::string, std::string> bambu_settings_serialized,
                                 DynamicPrintConfig &               print_config,
                                 ConfigSubstitutionContext &        config_substitutions,
                                 bool                               with_phony);

// Import serialized object settings with the print configuration as parent, without from_prusa.
bool convert_settings_from_bambu(std::map<std::string, std::string> bambu_settings_serialized,
                                 const DynamicPrintConfig &         print_config,
                                 ModelConfigObject &                object_config,
                                 ConfigSubstitutionContext &        config_substitutions,
                                 bool                               with_phony);

std_path get_temp_file(Model &model);
std_path extract_file(Model &model, mz_zip_archive &archive, const mz_zip_archive_file_stat &stat);
} // namespace Slic3r

#endif /* slic3r_Format_BBconfig_hpp_ */
