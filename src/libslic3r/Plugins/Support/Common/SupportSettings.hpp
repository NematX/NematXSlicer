///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Support settings shared by the support plugins: the enum options they test
   and the configuration keys they declare.

   Enums are read by their serialized value (decision Q4): the plugin side has
   no access to the host C++ enum types, and the serialized names are the
   stable form saved in project files. Each parse_* function turns one
   serialized value into a small plugin-side enum; an unknown value gives
   Unknown, so a newer host value never maps silently onto an old one.

       const Config config = object.config();
       if (support_style(config) == SupportStyle::Snug) ...

   Interface patterns are resolved to INFILL_PATTERN plugin ids, the strings
   resolve_pattern_id() of the STEP_SUPPORT_TOOLPATH payload expects.

   Key lists: a plugin declares in used_config_keys() every option its
   calculations read, so the host re-runs it when one of them changes. The
   grid family and the legacy toolpath algorithms declare the lists below with
   append_used_config_keys() instead of copying them; the lists cover the keys
   SupportParams reads, which those plugins will use once they are migrated to
   it (G1b, G2b). */
#ifndef plugins_support_common_settings_hpp_
#define plugins_support_common_settings_hpp_

#include <cstddef>
#include <cstdint>
#include <string>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"

namespace slic3r_api::Support::Common {

// support_material_style: "grid", "snug" or "organic".
enum class SupportStyle { Grid, Snug, Organic, Unknown };
// support_material_contact_distance_type: "filament", "plane" or "none" (soluble).
enum class ContactDistanceType { Filament, Plane, None, Unknown };
// support_material_pattern, the base pattern: "rectilinear", "rectilinear-grid" or "honeycomb".
enum class BasePattern { Rectilinear, RectilinearGrid, Honeycomb, Unknown };

// Turn one serialized value into its enum; any other value gives Unknown.
SupportStyle parse_support_style(const std::string &serialized);
ContactDistanceType parse_contact_distance_type(const std::string &serialized);
BasePattern parse_base_pattern(const std::string &serialized);

// Read the option from an object configuration and parse it; a missing option gives Unknown.
SupportStyle support_style(const Config &object_config);
ContactDistanceType contact_distance_type(const Config &object_config);
BasePattern base_pattern(const Config &object_config);

/*
Pattern plugin id of the sparse base, as the legacy toolpaths choose it:
honeycomb for the honeycomb base pattern, otherwise rectilinear when the base
is nearly full (density > 0.95) or wrapped in a sheath, and supportbase else.
*/
std::string base_pattern_plugin_id(BasePattern pattern, double base_density, bool with_sheath);

/*
Pattern plugin id of the top interface. serialized is the value of
support_material_top_interface_pattern; "auto" becomes concentric for soluble
support, rectilinear for a nearly full interface (density > 0.95) and
supportbase otherwise. Other values are pattern plugin ids already, except
"concentricgapfill", which no pattern plugin implements yet: it is returned
unchanged and resolve_pattern_id() then falls back with a warning.
*/
std::string top_interface_pattern_plugin_id(const std::string &serialized, bool soluble, double interface_density);

/*
Pattern plugin id of the bottom interface. serialized is the value of
support_material_bottom_interface_pattern; "auto" reuses the resolved top
pattern unless it is hilbertcurve, smooth or sawtooth, which are meant for top
surfaces, and then follows the same "auto" rule as the top.
*/
std::string bottom_interface_pattern_plugin_id(const std::string &serialized, const std::string &top_plugin_id,
                                               bool soluble, double interface_density);

// A borrowed array of used-key declarations owned by this library.
struct UsedKeyList
{
    const raw_used_config_key *keys = nullptr;
    size_t size = 0;
};

/*
Keys read by the support geometry part of SupportParams (heights, gaps,
flows, extruders) and by the grid family generator, including its
build-plate-only restriction. Every plugin using SupportParams declares them.
*/
UsedKeyList support_geometry_keys();
/*
Keys read by the path part of SupportParams (base and interface patterns,
sheath, angles, contact loops, first-layer density) and by the region settings
the legacy fillers read. Toolpath algorithms declare them in addition.
*/
UsedKeyList support_path_keys();

/*
Copy list at keys + offset when keys is not null and return offset plus the
list size, so used_config_keys() can chain several lists:

    int32_t count = append_used_config_keys(keys, 0, support_geometry_keys());
    count = append_used_config_keys(keys, count, support_path_keys());
    return count;
*/
int32_t append_used_config_keys(raw_used_config_key *keys, int32_t offset, UsedKeyList list) noexcept;

} // namespace slic3r_api::Support::Common

#endif // plugins_support_common_settings_hpp_
