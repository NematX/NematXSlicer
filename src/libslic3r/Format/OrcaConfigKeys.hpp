///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Format_OrcaConfigKeys_hpp_
#define slic3r_Format_OrcaConfigKeys_hpp_

#include <cstddef>
#include <string>

/*
Orca configuration keys
=======================

OrcaConfigKeys.cpp (generated, see its header for the source revision and the method) lists every FFF
setting that OrcaSlicer defines, with the type of its value, whether it can be nil and, for an
enumeration, the serialized names it accepts (joined by '|').

The Bambu configuration exporter (BambuConfigExport.cpp) uses the list for two jobs:
  - a SuperSlicer key without a renaming is written under its own name only when Orca defines a
    setting of that name (otherwise it would leak a SuperSlicer-only setting into the project file);
  - every value written is checked against the Orca type, and against the Orca enumeration when
    there is one, so that Orca never reads a value it cannot parse.

The list is plain data: it is searched by key with find_orca_config_key().
*/

namespace Slic3r {

// The type of an Orca setting value, named after Orca's ConfigOptionType (coBool -> Bool, coBools -> Bools...).
// A "Bools"-like type is a vector: one element per filament, extruder or extruder variant.
enum class OrcaOptionType : unsigned char {
    Bool, Bools, Int, Ints, Float, Floats, Percent, Percents, FloatOrPercent, FloatsOrPercents,
    String, Strings, Point, Points, PointsGroups, Enum, Enums
};

// One Orca setting. `enum_values` is empty unless the type is Enum or Enums; its names are separated by '|'.
struct OrcaConfigKey
{
    const char    *key;
    OrcaOptionType type;
    bool           nullable;
    const char    *enum_values;
};

// Return the Orca setting of that name, or nullptr when Orca does not define it.
const OrcaConfigKey *find_orca_config_key(const std::string &key);

// Return the number of settings of the list.
size_t orca_config_key_count();

// Return the immutable key array, valid for program lifetime and sized by orca_config_key_count().
const OrcaConfigKey *orca_config_keys();

// Return whether the type holds a vector of values.
inline bool orca_option_is_vector(OrcaOptionType type)
{
    switch (type) {
    case OrcaOptionType::Bools:
    case OrcaOptionType::Ints:
    case OrcaOptionType::Floats:
    case OrcaOptionType::Percents:
    case OrcaOptionType::FloatsOrPercents:
    case OrcaOptionType::Strings:
    case OrcaOptionType::Points:
    case OrcaOptionType::PointsGroups:
    case OrcaOptionType::Enums:
        return true;
    default:
        return false;
    }
}

} // namespace Slic3r

#endif /* slic3r_Format_OrcaConfigKeys_hpp_ */
