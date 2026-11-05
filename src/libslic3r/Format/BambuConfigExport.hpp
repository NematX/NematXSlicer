///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Format_BambuConfigExport_hpp_
#define slic3r_Format_BambuConfigExport_hpp_

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

/*
SuperSlicer -> Bambu / Orca configuration export
================================================

A Bambu (Orca) project carries the whole configuration twice: in `Metadata/project_settings.config` (a
JSON file) and in the CONFIG_BLOCK comment of the G-code. Both are written from the same translation of
the SuperSlicer configuration, in two steps:

    BambuConfigTranslation translation = translate_config_to_bambu(config);
    std::string json = bambu_project_settings_json(translation, "02.08.01.55");
    auto        lines = bambu_config_block(translation);   // {key, value} pairs, written as `; key = value`

translate_config_to_bambu() keeps only what Orca can read: a setting is written when
  - its nature allows the G-code destination (ConfigBase::writes_key), so connection settings such as
    `print_host` and every preset-only setting never leave the machine, and it is enabled;
  - Orca defines a setting for it: through the import key table (BBConfig.cpp, read backwards), through
    a few explicit renames, or by keeping its name when Orca defines the same name;
  - its value can be written exactly in the Orca type of that setting (number, percent, bool as 0/1,
    point, enumeration name...). A value that cannot, such as a SuperSlicer enumeration value that Orca
    does not know, omits the setting and adds a warning.
The SuperSlicer settings without any Orca equivalent are omitted and counted in one warning. The values
of the import table are not reversed (it is a migration table, several Bambu values give one
SuperSlicer value): a small dedicated table gives the Orca value of the few SuperSlicer values whose name
differs, and the other values are written as they are when Orca accepts them.

Mixed vectors keep disabled slots as `nil` when Orca allows null values; fully disabled settings are omitted.
A scalar that Orca stores per filament is repeated for every extruder. JSON additionally excludes settings
whose nature forbids the Project destination; the CONFIG_BLOCK uses the G-code destination.

The G-code templates are written as they are: only the name of their setting changes.
*/

namespace Slic3r {

class ConfigBase;

// How the values of a setting are laid out in the output files.
enum class BambuValueKind : uint8_t {
    Scalar,        // one value: a JSON string, and `key = value` with the text escaped on one line
    NumericVector, // JSON array of strings; the CONFIG_BLOCK joins the values with ','
    StringVector   // JSON array of strings; the CONFIG_BLOCK joins the quoted-if-needed values with ';'
};

// The value(s) of one Orca setting, already serialized as Orca reads them.
struct BambuConfigEntry
{
    BambuValueKind           kind = BambuValueKind::Scalar;
    std::vector<std::string> values;
    bool                    project_allowed = true;
};

// The result of the translation: plain data, with no behavior.
struct BambuConfigTranslation
{
    // Orca key -> value(s), sorted by key (the order of both outputs).
    std::map<std::string, BambuConfigEntry> entries;
    // One line for each setting omitted because its value has no exact Orca form, then one line counting the
    // SuperSlicer settings that have no Orca equivalent.
    std::vector<std::string> warnings;
};

// Translate the configuration. Never throws for a value it cannot write: the setting is omitted and a warning added.
BambuConfigTranslation translate_config_to_bambu(const ConfigBase &config);

// Write Orca's `project_settings.config`: a flat JSON object with sorted keys, the headers `version` (the
// caller's), `name` ("project_settings") and `from` ("project"), strings for the scalars and arrays of strings
// for the vectors. A string that is not valid UTF-8 has its bad bytes replaced; the key stays.
// Serialize entries permitted in Project, replacing invalid UTF-8 bytes without dropping their keys.
std::string bambu_project_settings_json(const BambuConfigTranslation &translation, const std::string &version);

// Write the lines of Orca's CONFIG_BLOCK as {key, value}: sorted by key, templates on one line with their
// line breaks escaped, vectors joined with ',' (numbers, enumerations) or ';' (strings).
std::vector<std::pair<std::string, std::string>> bambu_config_block(const BambuConfigTranslation &translation);

// A SuperSlicer value that is written under another name in Orca.
struct BambuExportValueRule
{
    const char *ss_key;
    const char *ss_value;
    const char *orca_value;
};

// The dedicated table of renamed values, in a form that tests can check against Orca's enumerations.
const std::vector<BambuExportValueRule> &bambu_export_value_rules();

// Return the Orca key under which a SuperSlicer setting is written when its value needs no conversion, or
// nothing when Orca has no such setting. Settings with a conversion rule have their own key (see the cpp).
std::optional<std::string> bambu_export_key(const std::string &ss_key);

} // namespace Slic3r

#endif /* slic3r_Format_BambuConfigExport_hpp_ */
