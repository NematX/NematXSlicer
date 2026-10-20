///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "GCodeExportMetadata.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>

#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"

/*
G-code export metadata
======================

The file writer owns the final text, while this module serializes host
configuration, calculates extrusion widths from the first Print object's
active region settings and formats the statistics measured by the Reader.
The configuration keys written are those whose definition nature allows the G-code
destination (ConfigBase::writes_key()): host-only, preset-metadata and one-shot
settings are refused by their nature. This filter is shared by export metadata and
full-configuration output through encode_gcode_export_configuration().

Statistics follow the layout of the historical text file: one value per configured
tool for the filament lines (0.00 for a tool that extruded nothing), two decimals,
values separated by ", ". The per-tool length, volume, weight and cost are derived from
the measured volume of each tool and the filament diameter, density and price of
the configuration copy. The total cost is the filament cost only (no time cost),
and the lines "filament used [g]" and "filament cost" exist only when some tool has a
positive weight, respectively a positive weight and cost. The whole list is empty
when the Print has no available statistics.

Representative direct calls (branches are not a timeline):

    encode_gcode_export_configuration()
        |-- config.keys()
        |-- config.writes_key()
        `-- config.opt_serialize()

    encode_gcode_export_extrusion_widths()
        |-- format_extrusion_width()
        |-- region.flow()
        `-- support_material_flow()

    encode_gcode_export_statistics()
        |-- format_tool_values()
        `-- format_two_decimals()
*/

namespace Slic3r {
// Append configuration entries in sorted key order, excluding the keys that the G-code destination refuses and disabled optional settings.
void encode_gcode_export_configuration(const DynamicPrintConfig &config,
                                       std::vector<std::pair<std::string, std::string>> &entries);

// Calculate widths from the first Print object's active region settings and return owned labels and values.
std::vector<std::pair<std::string, std::string>> encode_gcode_export_extrusion_widths(const Print &print);

// Format the measured statistics as owned label/value pairs in file order; empty when they are unavailable.
std::vector<std::pair<std::string, std::string>> encode_gcode_export_statistics(const Print &print,
                                                                               const DynamicPrintConfig &config);

namespace {

// Format one calculated width with a locale-independent decimal point and return owned text.
std::string format_extrusion_width(float width);

// Format one number with two decimals and a locale-independent decimal point and return owned text.
std::string format_two_decimals(double value);

// Join per-tool values with two decimals, separated by ", ", and return owned text.
std::string format_tool_values(const std::vector<double> &values);

} // namespace

namespace {

// Format one width with a decimal point independent of the process locale and return owned text; allocation failures propagate.
std::string format_extrusion_width(float width)
{
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::fixed << std::setprecision(2) << width << "mm";
    return output.str();
}

// Format one value with two decimals and a decimal point independent of the process locale; allocation failures propagate.
std::string format_two_decimals(double value)
{
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::fixed << std::setprecision(2) << value;
    return output.str();
}

// Format each value with two decimals and join them with ", "; an empty list gives empty text.
std::string format_tool_values(const std::vector<double> &values)
{
    std::string output;
    for (size_t index = 0; index < values.size(); ++index) {
        if (index > 0)
            output += ", ";
        output += format_two_decimals(values[index]);
    }
    return output;
}

} // namespace

// Serialize every key that the G-code destination allows, except disabled optional options, from a borrowed config; append in sorted order and propagate serialization or allocation failures.
void encode_gcode_export_configuration(const DynamicPrintConfig &config,
                                      std::vector<std::pair<std::string, std::string>> &entries)
{
    const std::vector<std::string> keys = config.keys();
    entries.reserve(entries.size() + keys.size());
    for (const std::string &key : keys) {
        if (config.writes_key(key, ConfigWriteTarget::GCode) &&
            (config.option(key)->is_enabled() || !config.get_option_def(key)->is_optional))
            entries.emplace_back(key, config.opt_serialize(key));
    }
    entries.shrink_to_fit();
}

// Calculate widths from the first Print object's active region settings and return owned label/value pairs; empty prints yield no widths and flow-calculation failures propagate.
std::vector<std::pair<std::string, std::string>> encode_gcode_export_extrusion_widths(const Print &print)
{
    std::vector<std::pair<std::string, std::string>> entries;
    if (print.objects().empty())
        return entries;

    const PrintObject &first_object = print.object(0);
    const double layer_height = first_object.config().layer_height.value;
    const coord_t first_layer_height = print.get_min_first_layer_height();
    entries.reserve(print.num_print_regions() * 10);
    for (size_t region_id = 0; region_id < print.num_print_regions(); ++region_id) {
        const PrintRegion &region = print.print_region(region_id);
        entries.emplace_back("external perimeters extrusion width",
                             format_extrusion_width(region.flow(first_object, frExternalPerimeter, layer_height, 2).width()));
        entries.emplace_back("perimeters extrusion width",
                             format_extrusion_width(region.flow(first_object, frPerimeter, layer_height, 2).width()));
        entries.emplace_back("infill extrusion width",
                             format_extrusion_width(region.flow(first_object, frInfill, layer_height, 2).width()));
        entries.emplace_back("solid infill extrusion width",
                             format_extrusion_width(region.flow(first_object, frSolidInfill, layer_height, 2).width()));
        entries.emplace_back("top infill extrusion width",
                             format_extrusion_width(region.flow(first_object, frTopSolidInfill, layer_height, 2).width()));
        if (print.has_support_material()) {
            entries.emplace_back("support material extrusion width",
                                 format_extrusion_width(support_material_flow(&first_object).width()));
            entries.emplace_back("support material interface extrusion width",
                                 format_extrusion_width(support_material_interface_flow(&first_object).width()));
        }
        if (first_object.config().first_layer_extrusion_width.is_enabled())
            entries.emplace_back("first layer extrusion width",
                                 format_extrusion_width(region.flow(first_object, frPerimeter, unscaled(first_layer_height), 0).width()));
        if (first_object.config().first_layer_infill_extrusion_width.is_enabled())
            entries.emplace_back("first layer infill extrusion width",
                                 format_extrusion_width(region.flow(first_object, frSolidInfill, unscaled(first_layer_height), 0).width()));
        entries.emplace_back(std::string(), std::string());
    }
    return entries;
}

// Convert the measured volume of each tool into length, volume, weight and cost with the configuration copy's filament
// properties, then append the lines in file order. The tool count is the number of nozzle diameters of the configuration copy:
// a measured tool beyond it is ignored (it is in no per-tool line and no weight or cost total; the tower share is the Print's). The other tool vectors of the configuration
// should have as many values (a shorter vector reuses its first value). Returns nothing when the statistics are unavailable.
std::vector<std::pair<std::string, std::string>> encode_gcode_export_statistics(const Print &print,
                                                                               const DynamicPrintConfig &config)
{
    std::vector<std::pair<std::string, std::string>> entries;
    const PrintStatistics &statistics = print.print_statistics();
    if (!statistics.statistics_available)
        return entries;

    const size_t tool_count = config.option<ConfigOptionFloats>("nozzle_diameter")->get_values().size();
    const ConfigOptionFloats &diameters = *config.option<ConfigOptionFloats>("filament_diameter");
    const ConfigOptionFloats &densities = *config.option<ConfigOptionFloats>("filament_density");
    const ConfigOptionFloats &prices = *config.option<ConfigOptionFloats>("filament_cost");

    std::vector<double> millimeters(tool_count, 0.);
    std::vector<double> cubic_centimeters(tool_count, 0.);
    std::vector<double> grams(tool_count, 0.);
    std::vector<double> costs(tool_count, 0.);
    double total_grams = 0.;
    double total_cost = 0.;
    bool has_weight = false;
    bool has_cost = false;
    for (const std::pair<const extruder_index_t, double> &tool_volume : statistics.filament_stats) {
        const size_t tool = size_t(tool_volume.first);
        if (tool >= tool_count)
            continue;
        const double volume = tool_volume.second;
        millimeters[tool] = volume / (PI * std::pow(0.5 * diameters.get_at(tool), 2.));
        cubic_centimeters[tool] = volume * 0.001;
        grams[tool] = cubic_centimeters[tool] * densities.get_at(tool);
        costs[tool] = grams[tool] * prices.get_at(tool) * 0.001;
        total_grams += grams[tool];
        total_cost += costs[tool];
        has_weight |= grams[tool] > 0.;
        has_cost |= grams[tool] > 0. && costs[tool] > 0.;
    }

    entries.emplace_back(PrintStatistics::FilamentUsedMm, format_tool_values(millimeters));
    entries.emplace_back(PrintStatistics::FilamentUsedCm3, format_tool_values(cubic_centimeters));
    if (has_weight)
        entries.emplace_back(PrintStatistics::FilamentUsedG, format_tool_values(grams));
    if (has_cost)
        entries.emplace_back(PrintStatistics::FilamentCost, format_tool_values(costs));
    entries.emplace_back(PrintStatistics::TotalFilamentUsedG, format_two_decimals(total_grams));
    entries.emplace_back(PrintStatistics::TotalFilamentCost, format_two_decimals(total_cost));
    entries.emplace_back(PrintStatistics::TotalFilamentUsedWipeTower,
                         format_two_decimals(statistics.total_wipe_tower_filament_weight));
    entries.emplace_back("total toolchanges", std::to_string(statistics.total_toolchanges));
    entries.emplace_back("objects layers count", std::to_string(statistics.object_layer_count));
    entries.emplace_back("total layers count", std::to_string(statistics.total_layer_count));
    entries.emplace_back("has_support", statistics.has_support ? "1" : "0");
    return entries;
}

} // namespace Slic3r
