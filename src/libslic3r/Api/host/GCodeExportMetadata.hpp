///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_host_GCodeExportMetadata_hpp_
#define slic3r_Api_host_GCodeExportMetadata_hpp_

#include <string>
#include <utility>
#include <vector>

/*
G-code export metadata
======================

This host-side module serializes export configuration entries and calculates
extrusion widths from the first Print object's active region settings, and formats
the measured print statistics. The configuration keys written are those whose
definition nature allows the G-code destination. The configuration filter is also
used by the G-code generator's saved settings.

The encode_gcode_export_* functions prepare configuration, width and statistics
values for the host's export payload. Keep their returned values alive while
building and lending the C arrays and string pointers: those pointers do not
take ownership. Exporters decide how to arrange these values in a file.
*/

namespace Slic3r {

class DynamicPrintConfig;
class Print;

// Append all keys that the G-code destination allows, except disabled optional settings, in key order; existing output entries are retained.
void encode_gcode_export_configuration(const DynamicPrintConfig &config,
                                       std::vector<std::pair<std::string, std::string>> &entries);

// Calculate widths from the first Print object's region settings and return owned G-code role labels/values with an empty pair after each region.
std::vector<std::pair<std::string, std::string>> encode_gcode_export_extrusion_widths(const Print &print);

// Format the measured statistics of the Print as owned (label, value) pairs in file order, using the configuration copy for the tool count (its nozzle diameters; measured tools beyond it are ignored) and filament properties; empty when the Print has no available statistics.
std::vector<std::pair<std::string, std::string>> encode_gcode_export_statistics(const Print &print,
                                                                               const DynamicPrintConfig &config);

} // namespace Slic3r

#endif // slic3r_Api_host_GCodeExportMetadata_hpp_
