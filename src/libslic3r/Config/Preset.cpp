///|/ Copyright (c) SuperSlicer 2025 Durand rémi @supermerill
///|/ Copyright (c) Prusa Research 2017 - 2024 Oleksandra Iushchenko @YuSanka, Lukáš Matěna @lukasmatena, Tomáš Mészáros @tamasmeszaros, Lukáš Hejl @hejllukas, Vojtěch Bubník @bubnikv, Pavel Mikuš @Godrak, David Kocík @kocikdav, Enrico Turri @enricoturri1966, Vojtěch Král @vojtechkral
///|/ Copyright (c) 2021 Martin Budden
///|/ Copyright (c) 2021 Ilya @xorza
///|/ Copyright (c) 2019 John Drake @foxox
///|/ Copyright (c) 2018 Martin Loidl @LoidlM
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "Preset.hpp"

#include <cassert>

#include <nlohmann/json.hpp>

#include "AppConfig.hpp"
#include "Exception.hpp"
#include "I18N.hpp"
#include "Updater/RepositoryUpdater.hpp"

#ifdef _MSC_VER
    #define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
    #define NOMINMAX
#endif
    #include <Windows.h>
#endif /* _MSC_VER */


#include <algorithm>
#include <array>
#include <fstream>
#include <regex>
#include <stdexcept>
#include <unordered_map>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <boost/nowide/cenv.hpp>
#include <boost/nowide/convert.hpp>
#include <boost/nowide/cstdio.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/locale.hpp>
#include <boost/log/trivial.hpp>

#include "libslic3r.h"
#include "ConfigDef.hpp"
#include "PrintConfig.hpp"
#include "Utils.hpp"
#include "PlaceholderParser.hpp"
#include "GCode/Thumbnails.hpp"

#include "PresetBundle.hpp"

/*
Preset collections load, select and compare profiles for the GUI or callers without a GUI.
A Preset holds one profile's settings, identity and compatibility state.

Unrecognized FFF settings are preserved as text in JSON, not applied to slicing.
Known settings placed in the wrong profile type can also be preserved, but only
when their save permissions allow it. Validation checks the JSON shape and string
values; it cannot check the meaning of settings the application does not recognize.
Known profile metadata, such as inherits, stays separate.

A combined project configuration loses the source profile of unknown keys.
Key prefixes suggest a print, printer or filament group. Explicitly stored JSON
takes precedence over that guess. Composition later combines the saved groups.

External names use the source filename without repeatedly nesting file prefixes.
Reduced collections keep requested profiles, their parents and selection fallbacks.

Operation notes:
- Validate stored JSON, normalize settings, then remove misplaced keys
- preserve_unknown_preset_settings: retain permitted text under its profile type
- preserve_unknown_project_settings: infer groups from combined project keys
- external_preset_name: build a file-derived name with a collision number
- PresetCollection::assign_subset: copy the profiles needed for selection
- compatibility_context / printer_for_extruder: prepare compatibility variables

Representative direct calls (branches are not a timeline):

    PresetCollection::load_presets()
        |-- config.load_from_ini()
        |-- validate_preset_custom_settings()
        |-- Preset::normalize()
        |-- Preset::remove_invalid_keys()
        |-- preserve_unknown_preset_settings()
        `-- this->select_preset()

    preserve_unknown_preset_settings()
        |-- unknown_settings_json()
        |-- Json::object()
        |-- merge_json_objects()
        |-- parse_print_custom_setting()
        |-- captured.dump()
        `-- parse_filament_custom_setting()

    external_preset_name()
        |-- external_preset_base()
        `-- std::to_string()
*/

using boost::property_tree::ptree;

// todo: arc_fitting and mlin_gcode_resolution switch from print to printer

namespace Slic3r {

ConfigFileType guess_config_file_type(const ptree &tree)
{
    size_t app_config   = 0;
    size_t bundle       = 0;
    size_t config       = 0;
    for (const ptree::value_type &v : tree) {
        if (v.second.empty()) {
            if (v.first == "background_processing" ||
                v.first == "last_output_path" ||
                v.first == "no_controller" ||
                v.first == "no_defaults")
                ++ app_config;
            else if (v.first == "nozzle_diameter" ||
                v.first == "filament_diameter")
                ++ config;
        } else if (boost::algorithm::starts_with(v.first, "print:") ||
            boost::algorithm::starts_with(v.first, "filament:") ||
            boost::algorithm::starts_with(v.first, "printer:") ||
            v.first == "settings")
            ++ bundle;
        else if (v.first == "presets") {
            ++ app_config;
            ++ bundle;
        } else if (v.first == "recent") {
            for (auto &kvp : v.second)
                if (kvp.first == "config_directory" || kvp.first == "skein_directory")
                    ++ app_config;
        }
    }
    return (app_config > bundle && app_config > config) ? CONFIG_FILE_TYPE_APP_CONFIG :
           (bundle > config) ? CONFIG_FILE_TYPE_CONFIG_BUNDLE : CONFIG_FILE_TYPE_CONFIG;
}

/*static*/ std::string VendorProfile::get_http_url_rest(const std::string &config_update_rest)
{
    // Keep this legacy entry point for GUI callers while repository updaters
    // and vendor profiles share one URL normalization contract.
    return RepositoryUpdater::normalize_repository_rest_url(config_update_rest);
}

const std::regex VP_FOR_FILENAME("[^0-9a-zA-Z_\\-. ]");
std::string VendorProfile::usable_id() const {
    return std::regex_replace(id, VP_FOR_FILENAME, "-");
}

VendorProfile VendorProfile::from_ini(const boost::filesystem::path &path, bool load_all)
{
    const std::string id = path.stem().string();

    if (! boost::filesystem::exists(path)) {
        throw Slic3r::RuntimeError((boost::format("Cannot load Vendor Config Bundle `%1%`: File not found: `%2%`.") % id % path).str());
    }

    ptree tree;
    boost::nowide::ifstream ifs(path.string());
    boost::property_tree::read_ini(ifs, tree);
    return VendorProfile::from_ini(tree, id, load_all);
}

static const std::unordered_map<std::string, std::string> pre_family_model_map {{
    { "MK3",        "MK3" },
    { "MK3MMU2",    "MK3" },
    { "MK2.5",      "MK2.5" },
    { "MK2.5MMU2",  "MK2.5" },
    { "MK2S",       "MK2" },
    { "MK2SMM",     "MK2" },
    { "SL1",        "SL1" },
}};

VendorProfile VendorProfile::from_ini(const ptree &tree, const std::string &base_id, bool load_all)
{
    static const std::string printer_model_key = "printer_model:";
    static const std::string filaments_section = "default_filaments";
    static const std::string materials_section = "default_sla_materials";

    std::string id = base_id;

    VendorProfile res(id);

    // Helper to get compulsory fields
    auto get_or_throw = [&](const ptree &tree, const std::string &key) -> ptree::const_assoc_iterator
    {
        auto res = tree.find(key);
        if (res == tree.not_found()) {
            throw Slic3r::RuntimeError((boost::format("Vendor Config Bundle `%1%` is not valid: Missing secion or key: `%2%`.") % id % key).str());
        }
        return res;
    };

    // Load the header
    const auto &vendor_section = get_or_throw(tree, "vendor")->second;

    // fix id if set (can be useful when when getting from internet or loading from a stream)
    const auto id_node = vendor_section.find("id");
    if (id_node != vendor_section.not_found()) {
        std::string id_from_vendor = id_node->second.data();
        res.id = id = id_from_vendor;
    }

    // name, full_name and technologies
    res.name = get_or_throw(vendor_section, "name")->second.data();
    auto full_name_node = vendor_section.find("full_name");
    res.full_name = (full_name_node == vendor_section.not_found()) ? res.name : full_name_node->second.data();
    const auto technologies_field = vendor_section.get<std::string>("technologies", "");
    std::vector<std::string> technologies;
    if (Slic3r::unescape_strings_cstyle(technologies_field, technologies) && !technologies.empty()) {
        for (const std::string &technology : technologies) {
            PrinterTechnology tech = parse_printer_technology(technology);
            if (tech != ptUnknown) {
                res.technologies.push_back(tech);
            } else {
                BOOST_LOG_TRIVIAL(error)
                    << boost::format("Vendor bundle: `%1%`: Malformed technologies field: `%2%`") % id %
                        technologies_field;
            }
        }
    } else {
        //default to FFF if not present
        res.technologies.push_back(PrinterTechnology::ptFFF);
    }

    // description
    const auto description_node = vendor_section.find("description");
    if (description_node != vendor_section.not_found()) {
        res.description = description_node->second.data();
    }

    //it's now possible to have no version (only the vendor header)
    const auto config_version_node = vendor_section.find("config_version");
    if (config_version_node != vendor_section.not_found()) {
        auto config_version = Semver::parse(config_version_node->second.data());
        if (! config_version) {
            throw Slic3r::RuntimeError((boost::format("Vendor Config Bundle `%1%` is not valid: Cannot parse config_version: `%2%`.") % id % config_version_node->second.data()).str());
        } else {
            res.config_version = std::move(*config_version);
        }
    } else if(load_all) {
        throw Slic3r::RuntimeError((boost::format("Vendor Config Bundle `%1%` is not valid: Missing secion or key: `%2%`.") % id % "config_version").str());
    }

    // Load URLs
    const auto config_update_url = vendor_section.find("config_update_url");
    if (config_update_url != vendor_section.not_found()) {
        res.config_update_url = config_update_url->second.data();
    }
    
    const auto config_update_rest = vendor_section.find("config_update_rest");
    const auto config_update_github = vendor_section.find("config_update_github");
    if (config_update_rest != vendor_section.not_found()) {
        res.config_update_rest = config_update_rest->second.data();
    } else if (config_update_github != vendor_section.not_found()) {
        res.config_update_rest = config_update_github->second.data();
    }

    const auto changelog_url = vendor_section.find("changelog_url");
    if (changelog_url != vendor_section.not_found()) {
        res.changelog_url = changelog_url->second.data();
    }

    //slicer

    const auto slicer_name_node = vendor_section.find("slicer");
    if (slicer_name_node != vendor_section.not_found()) {
        res.slicer = slicer_name_node->second.data();
    }

    const auto slicer_version_node = vendor_section.find("slicer_version");
    if (slicer_version_node != vendor_section.not_found()) {
        auto slicer_version = Semver::parse(slicer_version_node->second.data());
        if (! slicer_version) {
            res.slicer_version = Semver::zero();
        } else {
            res.slicer_version = std::move(*slicer_version);
        }
    }

    //get family column size
    auto family_size = tree.find("family_size");
    if (family_size != tree.not_found()) {
        for (auto it = family_size->second.begin(); it != family_size->second.end(); ++it) {
            try {
                res.family_2_line_size[it->first] = atoi(it->second.data().c_str());
            } catch (const std::runtime_error & err) {
                printf("Error, can't set the line width for family %s:\n%s\n", it->first.c_str(), err.what());
            }
        }
    }

    const auto templates_profile = vendor_section.find("templates_profile");
    if (templates_profile != vendor_section.not_found()) {
        res.templates_profile = templates_profile->second.data() == "1";
    }

    if (! load_all) {
        return res;
    }

    // Load printer models
    for (auto &section : tree) {
        if (boost::starts_with(section.first, printer_model_key)) {
            VendorProfile::PrinterModel model;
            model.id = section.first.substr(printer_model_key.size());
            model.name = section.second.get<std::string>("name", model.id);

            const char *technology_fallback = boost::algorithm::starts_with(model.id, "SL") ? "SLA" : "FFF";

            auto technology_field = section.second.get<std::string>("technology", technology_fallback);
            if (! ConfigOptionEnum<PrinterTechnology>::from_string(technology_field, model.technology)) {
                BOOST_LOG_TRIVIAL(error) << boost::format("Vendor bundle: `%1%`: Invalid printer technology field: `%2%`") % id % technology_field;
                model.technology = ptFFF;
            }

            model.family = section.second.get<std::string>("family", std::string());
            if (model.family.empty() && res.name == "Prusa Research") {
                // If no family is specified, it can be inferred for known printers
                const auto from_pre_map = pre_family_model_map.find(model.id);
                if (from_pre_map != pre_family_model_map.end()) { model.family = from_pre_map->second; }
            }

            section.second.get<std::string>("variants", "");
            const auto variants_field = section.second.get<std::string>("variants", "");
            std::vector<std::string> variants;
            if (Slic3r::unescape_strings_cstyle(variants_field, variants)) {
                for (const std::string &variant_name : variants) {
                    if (model.variant(variant_name) == nullptr)
                        model.variants.emplace_back(VendorProfile::PrinterVariant(variant_name));
                }
            } else {
                BOOST_LOG_TRIVIAL(error) << boost::format("Vendor bundle: `%1%`: Malformed variants field: `%2%`") % id % variants_field;
            }
            auto default_materials_field = section.second.get<std::string>("default_materials", "");
            if (default_materials_field.empty())
            	default_materials_field = section.second.get<std::string>("default_filaments", "");
            if (Slic3r::unescape_strings_cstyle(default_materials_field, model.default_materials)) {
            	Slic3r::sort_remove_duplicates(model.default_materials);
            	if (! model.default_materials.empty() && model.default_materials.front().empty())
            		// An empty material was inserted into the list of default materials. Remove it.
            		model.default_materials.erase(model.default_materials.begin());
            } else {
                BOOST_LOG_TRIVIAL(error) << boost::format("Vendor bundle: `%1%`: Malformed default_materials field: `%2%`") % id % default_materials_field;
            }
            model.bed_model   = section.second.get<std::string>("bed_model", "");
            model.bed_texture = section.second.get<std::string>("bed_texture", "");
            model.bed_with_grid = section.second.get<std::string>("bed_with_grid", "") == "1";
            model.thumbnail   = section.second.get<std::string>("thumbnail", "");
            if (model.thumbnail.empty())
                model.thumbnail = model.id + "_thumbnail.png";

            //save it
            if (! model.id.empty() && ! model.variants.empty())
                res.models.emplace_back(std::move(model));
        }
    }

    // Load filaments and sla materials to be installed by default
    const auto filaments = tree.find(filaments_section);
    if (filaments != tree.not_found()) {
        for (auto &pair : filaments->second) {
            if (pair.second.data() == "1") {
                res.default_filaments.insert(pair.first);
            }
        }
    }
    const auto materials = tree.find(materials_section);
    if (materials != tree.not_found()) {
        for (auto &pair : materials->second) {
            if (pair.second.data() == "1") {
                res.default_sla_materials.insert(pair.first);
            }
        }
    }

    return res;
}

std::vector<std::string> VendorProfile::families() const
{
    std::vector<std::string> res;
    unsigned num_familiies = 0;

    for (auto &model : models) {
        if (std::find(res.begin(), res.end(), model.family) == res.end()) {
            res.push_back(model.family);
            num_familiies++;
        }
    }

    return res;
}

// Suffix to be added to a modified preset name in the combo box.
static std::string g_suffix_modified = " (modified)";
const std::string& Preset::suffix_modified()
{
    return g_suffix_modified;
}

void Preset::update_suffix_modified(const std::string& new_suffix_modified)
{
    g_suffix_modified = new_suffix_modified;
}
// Remove an optional "(modified)" suffix from a name.
// This converts a UI name to a unique preset identifier.
std::string Preset::remove_suffix_modified(const std::string &name)
{
    return boost::algorithm::ends_with(name, g_suffix_modified) ?
        name.substr(0, name.size() - g_suffix_modified.size()) :
        name;
}

// Update new extruder fields at the printer profile.
void Preset::normalize(DynamicPrintConfig &config)
{
    auto *nozzle_diameter = dynamic_cast<const ConfigOptionFloats*>(config.option("nozzle_diameter"));
    if (nozzle_diameter != nullptr)
        // Loaded the FFF Printer settings. Verify, that all extruder dependent values have enough values.
        config.set_num_extruders((unsigned int)nozzle_diameter->size());
    if (config.option("filament_diameter") != nullptr) {
        // This config contains single or multiple filament presets.
        // Ensure that the filament preset vector options contain the correct number of values.
        size_t n = (nozzle_diameter == nullptr) ? 1 : nozzle_diameter->size();
        const auto &defaults = FullPrintConfig::defaults();
        for (const std::string &key : PrintConfigDef::instance().option_keys(RAW_PRESET_TYPE_FFF_FILAMENT)) {
            if (key == "compatible_prints" || key == "compatible_printers")
                continue;
            auto *opt = config.option(key, false);
            auto *opt_default = defaults.option(key);
            /*assert(opt != nullptr);
            assert(opt->is_vector());*/
            if (opt != nullptr && opt->is_vector() && opt_default != nullptr) {
                static_cast<ConfigOptionVectorBase*>(opt)->resize(n, opt_default);
            }
        }
        // The following keys are mandatory for the UI, but they are not part of FullPrintConfig, therefore they are handled separately.
        for (const char *key : {"filament_settings_id"}) {
            auto *opt = config.option(key, false);
            assert(opt == nullptr || opt->type() == coStrings);
            if (opt != nullptr && opt->type() == coStrings)
                static_cast<ConfigOptionStrings*>(opt)->resize(n, std::string());
        }
    }
    auto *milling_diameter = dynamic_cast<const ConfigOptionFloats*>(config.option("milling_diameter"));
    if (milling_diameter != nullptr)
        // Loaded the FFF Printer settings. Verify, that all extruder dependent values have enough values.
        config.set_num_milling((unsigned int)milling_diameter->size());
    if (const auto *gap_fill_speed = config.option<ConfigOptionFloatOrPercent>("gap_fill_speed", false); gap_fill_speed && gap_fill_speed->value <= 0.) {
        // Legacy conversion. If the gap fill speed is zero, it means the gap fill is not enabled.
        // Set the new gap_fill_enabled value, so that it will show up in the UI as disabled.
        if (auto *gap_fill_enabled = config.option<ConfigOptionBool>("gap_fill_enabled", false); gap_fill_enabled)
            gap_fill_enabled->value = false;
    }
    handle_legacy_sla(config);
}

namespace {

using Json = nlohmann::json;

// Merge JSON objects recursively. A source value replaces the target value for the same key.
void merge_json_objects(Json &target, const Json &source);
// Parse a JSON object. An empty string gives an empty object.
// Invalid JSON or a non-object value raises ConfigurationError.
Json parse_custom_setting(const std::string &serialized);
// Parse stored print data. Only print/printer groups containing string values are accepted.
// Invalid structure raises ConfigurationError, as in parse_custom_setting().
Json parse_print_custom_setting(const std::string &serialized);
// Parse one filament's stored data as an object of string values.
// Invalid structure raises ConfigurationError, as in parse_custom_setting().
Json parse_filament_custom_setting(const std::string &serialized);
// Collect only unknown values, not replacements of known options.
Json unknown_settings_json(const ConfigSubstitutions &substitutions);
// Recognize the printer/extruder name prefixes available in a flat imported configuration.
bool starts_with_any(const std::string &key, std::initializer_list<const char *> prefixes);
// Split a quoted, C-escaped string list or a comma-separated value list.
// Keep each string in serialized form. A malformed quoted list stays as one unchanged value.
std::vector<std::string> split_raw_vector_value(const std::string &value);

void merge_json_objects(Json &target, const Json &source)
{
    for (Json::const_iterator item = source.begin(); item != source.end(); ++item) {
        Json &destination = target[item.key()];
        if (destination.is_object() && item.value().is_object())
            merge_json_objects(destination, item.value());
        else
            destination = item.value();
    }
}

Json parse_custom_setting(const std::string &serialized)
{
    if (serialized.empty())
        return Json::object();
    Json parsed;
    try {
        parsed = Json::parse(serialized);
    } catch (const Json::exception &error) {
        throw ConfigurationError(std::string("Invalid custom setting JSON: ") + error.what());
    }
    if (!parsed.is_object())
        throw ConfigurationError("Custom setting must be a JSON object");
    return parsed;
}

Json parse_print_custom_setting(const std::string &serialized)
{
    Json parsed = parse_custom_setting(serialized);
    for (Json::const_iterator origin = parsed.begin(); origin != parsed.end(); ++origin) {
        if ((origin.key() != "print" && origin.key() != "printer") || !origin.value().is_object())
            throw ConfigurationError("Print custom setting has an invalid origin");
        for (const Json &value : origin.value())
            if (!value.is_string())
                throw ConfigurationError("Print custom setting values must be strings");
    }
    return parsed;
}

Json parse_filament_custom_setting(const std::string &serialized)
{
    Json parsed = parse_custom_setting(serialized);
    for (const Json &value : parsed)
        if (!value.is_string())
            throw ConfigurationError("Filament custom setting values must be strings");
    return parsed;
}

Json unknown_settings_json(const ConfigSubstitutions &substitutions)
{
    Json result = Json::object();
    for (const ConfigSubstitution &substitution : substitutions)
        if (substitution.opt_def == nullptr && !substitution.old_name.empty())
            result[substitution.old_name] = substitution.old_value;
    return result;
}

bool starts_with_any(const std::string &key, std::initializer_list<const char *> prefixes)
{
    for (const char *prefix : prefixes)
        if (boost::starts_with(key, prefix))
            return true;
    return false;
}

std::vector<std::string> split_raw_vector_value(const std::string &value)
{
    if (!value.empty() && value.front() == '"') {
        std::vector<std::string> strings;
        if (!unescape_strings_cstyle(value, strings))
            return {value};
        std::vector<std::string> items;
        items.reserve(strings.size());
        for (const std::string &item : strings)
            items.push_back(escape_strings_cstyle({item}));
        return items;
    }
    std::vector<std::string> items;
    boost::split(items, value, boost::is_any_of(","));
    return items;
}

} // namespace

void preserve_unknown_project_settings(DynamicPrintConfig &config, const ConfigSubstitutions &substitutions)
{
    const Json unknown = unknown_settings_json(substitutions);
    if (unknown.empty())
        return;
    const ConfigOptionFloats *nozzles = config.option<ConfigOptionFloats>("nozzle_diameter");
    const size_t extruder_count = nozzles == nullptr ? 1 : std::max<size_t>(1, nozzles->size());
    Json print_groups = Json::object();
    std::vector<Json> filament_slots(extruder_count, Json::object());
    for (Json::const_iterator item = unknown.begin(); item != unknown.end(); ++item) {
        const std::string &key = item.key();
        const std::string value = item.value().get<std::string>();
        if (boost::starts_with(key, "filament_")) {
            const std::vector<std::string> items = split_raw_vector_value(value);
            if (items.size() == extruder_count) {
                for (size_t slot = 0; slot < extruder_count; ++slot)
                    filament_slots[slot][key] = items[slot];
                continue;
            }
        }
        const bool printer = starts_with_any(key, {
            "printer_", "firmware_", "machine_", "printhost_", "host_", "bed_", "thumbnails_", "gcode_",
            "extruder_", "retract_", "deretract_", "nozzle_", "tool_"});
        print_groups[printer ? "printer" : "print"][key] = value;
    }
    // The combined project no longer identifies the source profile of each key.
    // Keep explicitly stored groups when they conflict with the group inferred from a key prefix.
    if (!print_groups.empty()) {
        ConfigOptionString *setting = config.option<ConfigOptionString>("print_custom_setting", true);
        merge_json_objects(print_groups, parse_print_custom_setting(setting->value));
        setting->value = print_groups.dump();
    }
    bool any_filament = false;
    for (const Json &slot : filament_slots)
        any_filament = any_filament || !slot.empty();
    if (any_filament) {
        ConfigOptionStrings *setting = config.option<ConfigOptionStrings>("filament_custom_setting", true);
        std::vector<std::string> values = setting->get_values();
        values.resize(extruder_count);
        for (size_t slot = 0; slot < extruder_count; ++slot) {
            Json captured = filament_slots[slot];
            merge_json_objects(captured, parse_filament_custom_setting(values[slot]));
            values[slot] = captured.empty() ? std::string() : captured.dump();
        }
        setting->set(values);
    }
}

void preserve_unknown_preset_settings(DynamicPrintConfig &config, Preset::Type preset_type, const ConfigSubstitutions &substitutions)
{
    const Json unknown = unknown_settings_json(substitutions);
    if (unknown.empty())
        return;
    if (preset_type == Preset::TYPE_FFF_PRINT || preset_type == Preset::TYPE_PRINTER) {
        ConfigOptionString *setting = config.option<ConfigOptionString>("print_custom_setting", false);
        if (setting == nullptr)
            return;
        const char *origin = preset_type == Preset::TYPE_FFF_PRINT ? "print" : "printer";
        Json captured = Json::object({{origin, unknown}});
        merge_json_objects(captured, parse_print_custom_setting(setting->value));
        setting->value = captured.dump();
    } else if (preset_type == Preset::TYPE_FFF_FILAMENT) {
        ConfigOptionStrings *setting = config.option<ConfigOptionStrings>("filament_custom_setting", false);
        if (setting == nullptr)
            return;
        std::vector<std::string> values = setting->get_values();
        if (values.empty())
            values.emplace_back();
        Json captured = unknown;
        merge_json_objects(captured, parse_filament_custom_setting(values.front()));
        values.front() = captured.dump();
        setting->set(values);
    }
}

void validate_preset_custom_settings(const DynamicPrintConfig &config)
{
    const ConfigOptionString *print_setting = config.option<ConfigOptionString>("print_custom_setting");
    if (print_setting != nullptr)
        parse_print_custom_setting(print_setting->value);
    const ConfigOptionStrings *filament_setting = config.option<ConfigOptionStrings>("filament_custom_setting");
    if (filament_setting != nullptr)
        for (const std::string &value : filament_setting->get_values())
            parse_filament_custom_setting(value);
}

std::string print_custom_setting_for_origin(const DynamicPrintConfig &config, const char *origin)
{
    const ConfigOptionString *setting = config.option<ConfigOptionString>("print_custom_setting");
    if (setting == nullptr)
        return {};
    const Json parsed = parse_print_custom_setting(setting->value);
    const Json::const_iterator item = parsed.find(origin);
    return item == parsed.end() ? std::string() : Json{{origin, *item}}.dump();
}

std::string merge_print_custom_settings(const DynamicPrintConfig &print_config, const DynamicPrintConfig &printer_config)
{
    Json merged = Json::object();
    for (const DynamicPrintConfig *config : {&print_config, &printer_config}) {
        const ConfigOptionString *setting = config->option<ConfigOptionString>("print_custom_setting");
        if (setting != nullptr)
            merge_json_objects(merged, parse_print_custom_setting(setting->value));
    }
    return merged.empty() ? std::string() : merged.dump();
}

std::string Preset::remove_invalid_keys(DynamicPrintConfig &config, const DynamicPrintConfig &default_config,
                                        ConfigSubstitutions *preserved_substitutions)
{
    std::string incorrect_keys;
    for (const std::string &key : config.keys())
        if (! default_config.has(key)) {
            if (incorrect_keys.empty())
                incorrect_keys = key;
            else {
                incorrect_keys += ", ";
                incorrect_keys += key;
            }
            // Retained JSON can reach projects and G-code. Do not use it to bypass a known key's save restrictions.
            if (preserved_substitutions != nullptr && config.writes_key(key, ConfigWriteTarget::Local) &&
                config.writes_key(key, ConfigWriteTarget::Project) && config.writes_key(key, ConfigWriteTarget::GCode))
                preserved_substitutions->emplace_back(key, config.opt_serialize(key));
            config.erase(key);
        }
    return incorrect_keys;
}

void Preset::save()
{
    this->config.save(this->file, ConfigWriteTarget::Local);
}

// Return a label of this preset, consisting of a name and a "(modified)" suffix, if this preset is dirty.
std::string Preset::label() const
{
    return this->name + (this->is_dirty ? g_suffix_modified : "");
}

bool is_compatible_with_print(const PresetWithVendorProfile &preset, const PresetWithVendorProfile &active_print, const PresetWithVendorProfile &active_printer)
{
    // templates_profile vendor profiles should be decided as same vendor profiles
	if (preset.vendor != nullptr && preset.vendor != active_printer.vendor && !preset.vendor->templates_profile)
		// The current profile has a vendor assigned and it is different from the active print's vendor.
		return false;
    auto &condition             = preset.preset.compatible_prints_condition();
    auto *compatible_prints     = dynamic_cast<const ConfigOptionStrings*>(preset.preset.config.option("compatible_prints"));
    bool  has_compatible_prints = compatible_prints != nullptr && ! compatible_prints->empty();
    if (! has_compatible_prints && ! condition.empty()) {
        try {
            return PlaceholderParser::evaluate_boolean_expression(condition, active_print.preset.config);
        } catch (const std::runtime_error &err) {
            //FIXME in case of an error, return "compatible with everything".
            printf("Preset::is_compatible_with_print - parsing error of compatible_prints_condition %s:\n%s\n", active_print.preset.name.c_str(), err.what());
            return true;
        }
    }
    return preset.preset.is_default || active_print.preset.name.empty() || ! has_compatible_prints ||
        std::find(compatible_prints->get_values().begin(), compatible_prints->get_values().end(), active_print.preset.name) !=
            compatible_prints->get_values().end();
}

bool is_compatible_with_printer(const PresetWithVendorProfile &preset, const PresetWithVendorProfile &active_printer, const DynamicPrintConfig *extra_config)
{
    // templates_profile vendor profiles should be decided as same vendor profiles
	if (preset.vendor != nullptr && preset.vendor != active_printer.vendor && !preset.vendor->templates_profile)
		// The current profile has a vendor assigned and it is different from the active print's vendor.
		return false;
    auto &condition               = preset.preset.compatible_printers_condition();
    auto *compatible_printers     = dynamic_cast<const ConfigOptionStrings*>(preset.preset.config.option("compatible_printers"));
    bool  has_compatible_printers = compatible_printers != nullptr && ! compatible_printers->empty();
    if (! has_compatible_printers && ! condition.empty()) {
        try {
            return PlaceholderParser::evaluate_boolean_expression(condition, active_printer.preset.config, extra_config);
        } catch (const std::runtime_error &err) {
            //FIXME in case of an error, return "compatible with everything".
            printf("Preset::is_compatible_with_printer - parsing error of compatible_printers_condition %s:\n%s\n", active_printer.preset.name.c_str(), err.what());
            return true;
        }
    }
    return preset.preset.is_default || active_printer.preset.name.empty() || ! has_compatible_printers ||
        std::find(compatible_printers->get_values().begin(), compatible_printers->get_values().end(), active_printer.preset.name) !=
            compatible_printers->get_values().end();
}

DynamicPrintConfig compatibility_context(const Preset &printer)
{
    DynamicPrintConfig config;
    config.set_key_value("printer_preset", new ConfigOptionString(printer.name));
    const ConfigOption *opt = printer.config.option("nozzle_diameter");
    if (opt) {
        config.set_key_value("num_extruders", new ConfigOptionInt((int)static_cast<const ConfigOptionFloats*>(opt)->size()));
        config.set_key_value("extruders_count", new ConfigOptionInt((int) static_cast<const ConfigOptionFloats *>(opt)->size()));
    }
    opt = printer.config.option("milling_diameter");
    if (opt) {
        config.set_key_value("num_milling", new ConfigOptionInt((int)static_cast<const ConfigOptionFloats*>(opt)->size()));
        config.set_key_value("milling_count", new ConfigOptionInt((int) static_cast<const ConfigOptionFloats *>(opt)->size()));
    }
    return config;
}

bool is_compatible_with_printer(const PresetWithVendorProfile &preset, const PresetWithVendorProfile &active_printer)
{
    DynamicPrintConfig config = compatibility_context(active_printer.preset);
    return is_compatible_with_printer(preset, active_printer, &config);
}

void Preset::set_visible_from_appconfig(const AppConfig &app_config)
{
    if (vendor == nullptr) { return; }

    if (type == TYPE_PRINTER) {
        const std::string &model = config.opt_string("printer_model");
        const std::string &variant = config.opt_string("printer_variant");
        if (model.empty() || variant.empty())
            return;
        is_visible = app_config.get_variant(vendor->id, model, variant);
    } else if (type == TYPE_FFF_FILAMENT || type == TYPE_SLA_MATERIAL) {
    	const std::string &section_name = (type == TYPE_FFF_FILAMENT) ? AppConfig::SECTION_FILAMENTS : AppConfig::SECTION_MATERIALS;
        if (type == TYPE_FFF_FILAMENT && app_config.get_bool("no_templates") && vendor && vendor->templates_profile)
            is_visible = false;
    	else if (app_config.has_section(section_name)) {
    		// Check whether this profile is marked as "installed" in PrusaSlicer.ini,
    		// or whether a profile is marked as "installed", which this profile may have been renamed from.
	    	const std::map<std::string, std::string> &installed = app_config.get_section(section_name);
	    	auto has = [&installed](const std::string &name) {
	    		auto it = installed.find(name);
				return it != installed.end() && ! it->second.empty();
	    	};
	    	is_visible = has(this->name);
	    	for (auto it = this->renamed_from.begin(); ! is_visible && it != this->renamed_from.end(); ++ it)
	    		is_visible = has(*it);
	    }
        else 
			is_visible = false;
    }
}

PresetCollection::PresetCollection(Preset::Type type, const std::set<std::string> &keys, const Slic3r::StaticPrintConfig &defaults, const std::string &default_name) :
    m_type(type),
    m_edited_preset(type, "", false),
    m_saved_preset(type, "", false),
    m_idx_selected(0)
{
    // Insert just the default preset.
    this->add_default_preset(keys, defaults, default_name);
    m_edited_preset.config.apply(m_presets.front().config);
    update_saved_preset_from_current_preset();
}

void PresetCollection::reset(bool delete_files)
{
    if (m_presets.size() > m_num_default_presets) {
        if (delete_files) {
            // Erase the preset files.
            for (Preset &preset : m_presets)
                if (! preset.is_default && ! preset.is_external && ! preset.is_system)
                    boost::nowide::remove(preset.file.c_str());
        }
        // Don't use m_presets.resize() here as it requires a default constructor for Preset.
        m_presets.erase(m_presets.begin() + m_num_default_presets, m_presets.end());
        this->select_preset(0);
    }
    m_map_alias_to_profile_name.clear();
    m_map_system_profile_renamed.clear();
}

void PresetCollection::add_default_preset(const std::set<std::string> &keys, const Slic3r::StaticPrintConfig &defaults, const std::string &preset_name)
{
    // Insert just the default preset.
    m_presets.emplace_back(Preset::PresetFactory{this->type(), preset_name, true});
    if (keys.empty()) {
        m_presets.back().config.apply_only(defaults, defaults.keys());
    } else {
        m_presets.back().config.apply_only(defaults, keys);
    }
    m_presets.back().loaded = true;
    ++ m_num_default_presets;
}

// Load all presets found in dir_path.
// Throws an exception on error.
void PresetCollection::load_presets(
    const std::string &dir_path, const std::string &subdir, 
    PresetsConfigSubstitutions& substitutions, ForwardCompatibilitySubstitutionRule substitution_rule)
{
    // Don't use boost::filesystem::canonical() on Windows, it is broken in regard to reparse points,
    // see https://github.com/prusa3d/PrusaSlicer/issues/732
    boost::filesystem::path dir = boost::filesystem::absolute(boost::filesystem::path(dir_path) / subdir).make_preferred();
    m_dir_path = dir.string();
    std::string errors_cummulative;
    // Store the loaded presets into a new vector, otherwise the binary search for already existing presets would be broken.
    // (see the "Preset already present, not loading" message).
    std::deque<Preset> presets_loaded;
    for (auto &dir_entry : boost::filesystem::directory_iterator(dir))
        if (Slic3r::is_ini_file(dir_entry)) {
            std::string name = dir_entry.path().filename().string();
            // Remove the .ini suffix.
            name.erase(name.size() - 4);
            if (this->find_preset(name, false)) {
                // This happens when there's is a preset (most likely legacy one) with the same name as a system preset
                // that's already been loaded from a bundle.
                BOOST_LOG_TRIVIAL(warning) << "Preset already present, not loading: " << name;
                continue;
            }
            try {
                Preset preset(m_type, name, false);
                preset.file = dir_entry.path().string();
                // Load the preset file, apply preset values on top of defaults.
                try {
                    DynamicPrintConfig config;
                    ConfigSubstitutions config_substitutions = config.load_from_ini(preset.file, substitution_rule);
                    validate_preset_custom_settings(config);
                    // Find a default preset for the config. The PrintPresetCollection provides different default preset based on the "printer_technology" field.
                    const Preset &default_preset = this->default_preset_for(config);
                    preset.config = default_preset.config;
                    preset.config.apply(std::move(config));
                    Preset::normalize(preset.config);
                    // Report configuration fields, which are misplaced into a wrong group.
                    std::string incorrect_keys = Preset::remove_invalid_keys(preset.config, default_preset.config, &config_substitutions);
                    preserve_unknown_preset_settings(preset.config, m_type, config_substitutions);
                    // Preserving unknown text must not open a value-substitution dialog. Keep real known-value changes.
                    config_substitutions.erase(std::remove_if(config_substitutions.begin(), config_substitutions.end(),
                        [](const ConfigSubstitution &entry) { return entry.opt_def == nullptr; }), config_substitutions.end());
                    if ((substitution_rule == ForwardCompatibilitySubstitutionRule::Enable ||
                         substitution_rule == ForwardCompatibilitySubstitutionRule::EnableSystemSilent) && !config_substitutions.empty())
                        substitutions.push_back({preset.name, m_type, PresetConfigSubstitutions::Source::UserFile, preset.file, std::move(config_substitutions)});
                    if (! incorrect_keys.empty())
                        BOOST_LOG_TRIVIAL(error) << "Error in a preset file: The preset \"" <<
                            preset.file << "\" contains the following incorrect keys: " << incorrect_keys << ", which were removed";
                    preset.loaded = true;
                } catch (const std::ifstream::failure &err) {
                    throw Slic3r::RuntimeError(std::string("The selected preset cannot be loaded: ") + preset.file + "\n\tReason: " + err.what());
                } catch (const std::runtime_error &err) {
                    throw Slic3r::RuntimeError(std::string("Failed loading the preset file: ") + preset.file + "\n\tReason: " + err.what());
                }
                presets_loaded.emplace_back(preset);
            } catch (const std::runtime_error &err) {
                errors_cummulative += err.what();
                errors_cummulative += "\n";
            }
        }
    m_presets.insert(m_presets.end(), std::make_move_iterator(presets_loaded.begin()), std::make_move_iterator(presets_loaded.end()));
    std::sort(m_presets.begin() + m_num_default_presets, m_presets.end());
    if(this->type() == Preset::Type::TYPE_PRINTER)
        this->select_preset(first_visible_idx());
    if (! errors_cummulative.empty())
        throw Slic3r::RuntimeError(errors_cummulative);
}

namespace {

const std::array<const char *, 9> external_source_extensions {
    ".3mf", ".amf", ".zip", ".ini", ".gcode", ".gco", ".g", ".ngc", ".bgcode"};

// Find a prefix before " (" equal to file_name or ending in a recognized source extension.
// Return the first match. Otherwise keep original_name unchanged; no provenance is stored.
std::string external_preset_base(const std::string &file_name, const std::string &original_name);

std::string external_preset_base(const std::string &file_name, const std::string &original_name)
{
    if (original_name == file_name)
        return original_name;
    for (size_t pos = original_name.find(" ("); pos != std::string::npos; pos = original_name.find(" (", pos + 1)) {
        const std::string prefix = original_name.substr(0, pos);
        if (prefix == file_name || std::any_of(external_source_extensions.begin(), external_source_extensions.end(),
                [&prefix](const char *extension) { return boost::iends_with(prefix, extension); }))
            return prefix;
    }
    return original_name;
}

} // namespace

std::string external_preset_name(const std::string &file_name, const std::string &original_name, size_t index)
{
    const std::string base = original_name.empty() ? std::string() : external_preset_base(file_name, original_name);
    if (base.empty() || base == file_name)
        return index == 0 ? file_name : file_name + " (" + std::to_string(index) + ")";
    return file_name + " (" + base + (index == 0 ? std::string() : "-" + std::to_string(index)) + ")";
}

// Load a preset from an already parsed config file, insert it into the sorted sequence of presets
// and select it, losing previous modifications.
Preset& PresetCollection::load_preset(const std::string &path, const std::string &name, const DynamicPrintConfig &config, bool select)
{
    DynamicPrintConfig cfg(this->default_preset().config);
    cfg.apply_only(config, cfg.keys(), true);
    return this->load_preset(path, name, std::move(cfg), select);
}

// Are the values of the two profiles equal? The compared keys are the ones present in both configs whose nature is
// comparable (the same rule as DynamicConfig::equals()) and, if read_keys is set, which belong to that set. The loader
// deduces read_keys from the nature of the keys, not from the file: it holds the keys that a G-code destination accepts
// (default values included) and the keys of the profile section. read_keys is borrowed; nullptr means no restriction.
static bool profile_print_params_same(const DynamicPrintConfig &cfg_old, const DynamicPrintConfig &cfg_new,
                                      const std::set<t_config_option_key> *read_keys)
{
    for (const t_config_option_key &key : cfg_old.diff(cfg_new))
        if (cfg_old.compares_key(key) && (read_keys == nullptr || read_keys->count(key) > 0))
            return false;
    return true;
}

// Load a preset from an already parsed config file, insert it into the sorted sequence of presets
// and select it, losing previous modifications.
// Only a single profile could be edited at at the same time, which introduces complexity when loading
// filament profiles for multi-extruder printers.
ExternalPreset PresetCollection::load_external_preset(
    // Path to the profile source file (a G-code, an AMF or 3MF file, a config file)
    const std::string           &path,
    // Name of the profile, derived from the source file name.
    const std::string           &name,
    // Original name of the profile, extracted from the loaded config. Empty, if the name has not been stored.
    const std::string           &original_name,
    // Config to initialize the preset from. It may contain configs of all presets merged in a single dictionary!
    const DynamicPrintConfig    &combined_config,
    // Select the preset after loading?
    LoadAndSelect                select,
    // Keys that comparing and copying are restricted to, besides their nature: the keys that a G-code destination accepts and the
    // keys of the profile section (see Preset.hpp). Borrowed; nullptr means no restriction.
    const std::set<t_config_option_key> *read_keys)
{
    validate_preset_custom_settings(combined_config);
    // Load the preset over a default preset, so that the missing fields are filled in from the default preset.
    DynamicPrintConfig cfg(this->default_preset_for(combined_config).config);
    t_config_option_keys keys = cfg.keys();
    cfg.apply_only(combined_config, keys, true);
    // update_phony shouldn't be needed, still, it's best to be sure.
    cfg.update_phony({&combined_config});
    std::string                 &inherits = Preset::inherits(cfg);
    if (select == LoadAndSelect::Never) {
        // Some filament profile has been selected and modified already.
        // Check whether this profile is equal to the modified edited profile.
        const Preset &edited = this->get_edited_preset();
        if ((edited.name == original_name || edited.name == inherits) && profile_print_params_same(edited.config, cfg, read_keys))
            // Just point to that already selected and edited profile.
            return ExternalPreset(&(*this->find_preset_internal(edited.name)), false);
    }
    // Is there a preset already loaded with the name stored inside the config?
    std::deque<Preset>::iterator it       = this->find_preset_internal(original_name);
    bool                         found    = it != m_presets.end() && it->name == original_name;
    if (! found) {
    	// Try to match the original_name against the "renamed_from" profile names of loaded system profiles.
		it = this->find_preset_renamed(original_name);
		found = it != m_presets.end();
    }
    if (found && profile_print_params_same(it->config, cfg, read_keys) && it->is_visible) {
        // The preset exists and is visible and it matches the values stored inside config.
        if (select == LoadAndSelect::Always)
            this->select_preset(it - m_presets.begin());
        return ExternalPreset(&(*it), false);
    }
    if (! found && select != LoadAndSelect::Never && ! inherits.empty()) {
        // Try to use a system profile as a base to select the system profile
        // and override its settings with the loaded ones.
        assert(it == m_presets.end());
        it    = this->find_preset_internal(inherits);
        found = it != m_presets.end() && it->name == inherits;
        if (found && profile_print_params_same(it->config, cfg, read_keys)) {
            // The system preset exists and it matches the values stored inside config.
            if (select == LoadAndSelect::Always)
                this->select_preset(it - m_presets.begin());
            return ExternalPreset(&(*it), false);
        }
    }
    if (found) {
        if (select != LoadAndSelect::Never) {
            const size_t idx = it - m_presets.begin();
            // The newly selected preset can be activated AND have to be make as visible.
            bool is_installed = !m_presets[idx].is_visible;

            // Select the existing preset and override it with new values, so that
            // the differences will be shown in the preset editor against the referenced profile.
            this->select_preset(idx);

            // update dirty state only if it's needed
            if (!profile_print_params_same(it->config, cfg, read_keys)) {
                // The source config may contain keys from many possible preset types. Just copy those that relate to this preset
                // (the keys of cfg), and only the ones that the comparison above looks at: the comparable keys of read_keys.
                // A NORMAL key that the source lacks is copied with its default value; a LOCAL or PRESET_META key that is not
                // in the section keeps the value of the installed profile.
                keys.erase(std::remove_if(keys.begin(), keys.end(), [&cfg, read_keys](const t_config_option_key &key) {
                               return !cfg.compares_key(key) || (read_keys != nullptr && read_keys->count(key) == 0);
                           }), keys.end());

                this->get_edited_preset().config.apply_only(combined_config, keys, true);
                this->update_dirty();
                // Don't save the newly loaded project as a "saved into project" state.
                //update_saved_preset_from_current_preset();
                
                // the get_edited_preset can be 'not dirty' if it's exactly the same as a saved preset.
                //assert(this->get_edited_preset().is_dirty);
            }
            return ExternalPreset(&(*it), this->get_edited_preset().is_dirty, is_installed);
        }
        if (inherits.empty()) {
            // Update the "inherits" field.
            // There is a profile with the same name already loaded. Should we update the "inherits" field?
            inherits = it->vendor ? it->name : it->inherits();
        }
    }

    // The external preset does not match an internal preset, load the external preset.
    std::string new_name;
    for (size_t idx = 0;; ++ idx) {
        new_name = external_preset_name(name, original_name, idx);
        it = this->find_preset_internal(new_name);
        if (it == m_presets.end() || it->name != new_name)
            // Unique profile name. Insert a new profile.
            break;
        if (profile_print_params_same(it->config, cfg, read_keys)) {
            // The preset exists and it matches the values stored inside config.
            if (select == LoadAndSelect::Always)
                this->select_preset(it - m_presets.begin());
            return ExternalPreset(&(*it), false);
        }
        // Form another profile name.
    }
    // Insert a new profile.
    Preset &preset = this->load_preset(path, new_name, std::move(cfg), select == LoadAndSelect::Always);
    preset.is_external = true;
    if (this->m_idx_selected != size_t(-1) && &this->get_selected_preset() == &preset)
        this->get_edited_preset().is_external = true;

    return ExternalPreset(&preset, false);
}

Preset& PresetCollection::load_preset(const std::string &path, const std::string &name, DynamicPrintConfig &&config, bool select)
{
    validate_preset_custom_settings(config);
    auto it = this->find_preset_internal(name);
    if (it == m_presets.end() || it->name != name) {
        // The preset was not found. Create a new preset.
        it = m_presets.emplace(it, Preset::PresetFactory{m_type, name, false});
    }
    Preset &preset = *it;
    preset.file = path;
    preset.config = std::move(config);
    preset.loaded = true;
    preset.is_dirty = false;
    if (select)
        this->select_preset_by_name(name, true);
    return preset;
}

bool PresetCollection::save_current_preset(const std::string &new_name, bool detach)
{
    bool is_saved_as_new{ false };
    // 1) Find the preset with a new_name or create a new one,
    // initialize it with the edited config.
    auto it = this->find_preset_internal(new_name);
    if (it != m_presets.end() && it->name == new_name) {
        // Preset with the same name found.
        Preset &preset = *it;
        if (preset.is_default || preset.is_external || preset.is_system)
            // Cannot overwrite the default preset.
            return false;
        // Overwriting an existing preset.
        preset.config = std::move(m_edited_preset.config);
        // The newly saved preset will be activated -> make it visible.
        preset.is_visible = true;
        if (detach) {
            // Clear the link to the parent profile.
            preset.vendor = nullptr;
			preset.inherits().clear();
			preset.alias.clear();
			preset.renamed_from.clear();
        }
    } else {
        is_saved_as_new = true;
        // Creating a new preset.
        Preset       &preset   = *m_presets.insert(it, m_edited_preset);
        std::string  &inherits = preset.inherits();
        std::string   old_name = preset.name;
        preset.name = new_name;
        preset.file = this->path_from_name(new_name);
        preset.vendor = nullptr;
		preset.alias.clear();
        preset.renamed_from.clear();
        if (detach) {
        	// Clear the link to the parent profile.
        	inherits.clear();
        } else if (preset.is_system) {
            // Inheriting from a system preset.
            inherits = /* preset.vendor->name + "/" + */ old_name;
        } else if (inherits.empty()) {
            // Inheriting from a user preset. Link the new preset to the old preset.
            // inherits = old_name;
        } else {
            // Inherited from a user preset. Just maintain the "inherited" flag,
            // meaning it will inherit from either the system preset, or the inherited user preset.
        }
        preset.is_default  = false;
        preset.is_system   = false;
        preset.is_external = false;
        // The newly saved preset will be activated -> make it visible.
        preset.is_visible  = true;
        // Just system presets have aliases
        preset.alias.clear();
    }
    // 2) Activate the saved preset.
    this->select_preset_by_name(new_name, true);
    // 2) Store the active preset to disk.
    this->get_selected_preset().save();

    return is_saved_as_new;
}

Preset& PresetCollection::get_preset_with_name(const std::string& new_name, const Preset* initial_preset)
{
    // 1) Find the preset with a new_name or create a new one,
    // initialize it with the preset_to config.
    auto it = this->find_preset_internal(new_name);
    if (it != m_presets.end() && it->name == new_name) {
        // Preset with the same name found.
        Preset& preset = *it;
        if (!preset.is_default && !preset.is_external && !preset.is_system && initial_preset->name != new_name) {
            // Overwriting an existing preset if it isn't default/external/system or isn't an initial_preset
            preset.config = initial_preset->config;
            // The newly saved preset can be activated -> make it visible.
            preset.is_visible = true;
        }
        return preset;
    }

    const std::string selected_preset_name = this->get_selected_preset_name();

    // Creating a new preset.
    Preset& preset = *m_presets.insert(it, *initial_preset);
    std::string& inherits = preset.inherits();
    std::string old_name = preset.name;
    preset.name = new_name;
    preset.file = this->path_from_name(new_name);
    preset.vendor = nullptr;
    preset.alias.clear();
    preset.renamed_from.clear();
    if (preset.is_system) {
        // Inheriting from a system preset.
        inherits = old_name;
    }
    else if (inherits.empty()) {
        // Inheriting from a user preset. Link the new preset to the old preset.
        // inherits = old_name;
    }
    else {
        // Inherited from a user preset. Just maintain the "inherited" flag,
        // meaning it will inherit from either the system preset, or the inherited user preset.
    }
    preset.is_default = false;
    preset.is_system = false;
    preset.is_external = false;
    // The newly saved preset can be activated -> make it visible.
    preset.is_visible = true;
    // Just system presets have aliases
    preset.alias.clear();

    // sort printers and get new it
    std::sort(m_presets.begin(), m_presets.end());

    // set initial preset selection
    this->select_preset_by_name(selected_preset_name, true);

    it = this->find_preset_internal(new_name);
    assert(it != m_presets.end());

    return *it;
}

bool PresetCollection::delete_current_preset()
{
    const Preset &selected = this->get_selected_preset();
    if (selected.is_default)
        return false;
    if (! selected.is_external && ! selected.is_system) {
        // Erase the preset file.
        boost::nowide::remove(selected.file.c_str());
    }
    // Remove the preset from the list.
    m_presets.erase(m_presets.begin() + m_idx_selected);
    // Find the next visible preset.
    size_t new_selected_idx = m_idx_selected;
    if (new_selected_idx < m_presets.size())
        for (; new_selected_idx < m_presets.size() && ! m_presets[new_selected_idx].is_visible; ++ new_selected_idx) ;
    if (new_selected_idx == m_presets.size())
        for (--new_selected_idx; new_selected_idx > 0 && !m_presets[new_selected_idx].is_visible; --new_selected_idx);
    this->select_preset(new_selected_idx);
    return true;
}

bool PresetCollection::delete_preset(const std::string& name)
{
    if (name == this->get_selected_preset().name)
        return delete_current_preset();

    const std::string selected_preset_name = this->get_selected_preset_name();

    auto it = this->find_preset_internal(name);

    const Preset& preset = *it;
    if (preset.is_default)
        return false;
    if (!preset.is_external && !preset.is_system) {
        // Erase the preset file.
        boost::nowide::remove(preset.file.c_str());
    }
    m_presets.erase(it);

    // update selected preset
    // // supermerill: why reloading our preset? 
    //this->select_preset_by_name(selected_preset_name, true);

    return true;
}

const Preset* PresetCollection::get_selected_preset_parent() const
{
    if (this->get_selected_idx() == size_t(-1))
        // This preset collection has no preset activated yet. Only the get_edited_preset() is valid.
        return nullptr;

    const Preset 	  &selected_preset = this->get_selected_preset();
    if (selected_preset.is_system || selected_preset.is_default)
        return &selected_preset;

    const Preset 	  &edited_preset   = this->get_edited_preset();
    const std::string &inherits        = edited_preset.inherits();
    const Preset      *preset          = nullptr;
    if (inherits.empty()) {
        if (selected_preset.is_external)
            return nullptr;
        preset = &this->default_preset(m_type == Preset::Type::TYPE_PRINTER && edited_preset.printer_technology() == ptSLA ? 1 : 0);
    } else
        preset = this->find_preset(inherits, false);
    if (preset == nullptr) {
	    // Resolve the "renamed_from" field.
    	assert(! inherits.empty());
    	auto it = this->find_preset_renamed(inherits);
		if (it != m_presets.end())
			preset = &(*it);
    }
    return (preset == nullptr/* || preset->is_default*/ || preset->is_external) ? nullptr : preset;
}

const Preset* PresetCollection::get_preset_parent(const Preset& child) const
{
    const std::string &inherits = child.inherits();
    if (inherits.empty())
// 		return this->get_selected_preset().is_system ? &this->get_selected_preset() : nullptr;
        return nullptr;
    const Preset* preset = this->find_preset(inherits, false);
    if (preset == nullptr) {
    	auto it = this->find_preset_renamed(inherits);
		if (it != m_presets.end())
			preset = &(*it);
    }
    return
         // not found
        (preset == nullptr/* || preset->is_default */||
         // this should not happen, user profile should not derive from an external profile
         preset->is_external ||
         // this should not happen, however people are creative, see GH #4996
         preset == &child) ?
            nullptr :
            preset;
}

// Return vendor of the first parent profile, for which the vendor is defined, or null if such profile does not exist.
PresetWithVendorProfile PresetCollection::get_preset_with_vendor_profile(const Preset &preset) const
{
	const Preset		*p = &preset;
	const VendorProfile *v = nullptr;
	do {
		if (p->vendor != nullptr) {
			v = p->vendor;
			break;
		}
		p = this->get_preset_parent(*p);
	} while (p != nullptr);
	return PresetWithVendorProfile(preset, v);
}

const std::string& PresetCollection::get_preset_name_by_alias(const std::string& alias) const
{
	for (
		// Find the 1st profile name with the alias.
		auto it = Slic3r::lower_bound_by_predicate(m_map_alias_to_profile_name.begin(), m_map_alias_to_profile_name.end(), [&alias](auto &l){ return l.first < alias; });
		// Continue over all profile names with the same alias.
		it != m_map_alias_to_profile_name.end() && it->first == alias; ++ it)
		if (auto it_preset = this->find_preset_internal(it->second);
			it_preset != m_presets.end() && it_preset->name == it->second &&
            it_preset->is_visible && (it_preset->is_compatible || size_t(it_preset - m_presets.begin()) == m_idx_selected))
	        return it_preset->name;
    return alias;
}

const std::string* PresetCollection::get_preset_name_renamed(const std::string &old_name) const
{
	auto it_renamed = m_map_system_profile_renamed.find(old_name);
	if (it_renamed != m_map_system_profile_renamed.end())
		return &it_renamed->second;
	return nullptr;
}

const std::string& PresetCollection::get_suffix_modified() {
    return g_suffix_modified;
}

// Return a preset by its name. If the preset is active, a temporary copy is returned.
// If a preset is not found by its name, null is returned.
Preset* PresetCollection::find_preset(const std::string &name, bool first_visible_if_not_found, bool respect_active_preset /*= true*/)
{
    Preset key(m_type, name, false);
    auto it = this->find_preset_internal(name);
    // Ensure that a temporary copy is returned if the preset found is currently selected.
    return (it != m_presets.end() && it->name == key.name) ? &this->preset(it - m_presets.begin(), respect_active_preset) :
        first_visible_if_not_found ? &this->first_visible() : nullptr;
}

// Return index of the first visible preset. Certainly at least the '- default -' preset shall be visible.
size_t PresetCollection::first_visible_idx() const
{
    size_t idx = m_default_suppressed ? m_num_default_presets : 0;
    for (; idx < m_presets.size(); ++ idx)
        if (m_presets[idx].is_visible)
            break;
    if (idx == m_presets.size())
        idx = 0;
    return idx;
}

void PresetCollection::set_default_suppressed(bool default_suppressed)
{
    if (m_default_suppressed != default_suppressed) {
        m_default_suppressed = default_suppressed;
        bool default_visible = ! default_suppressed || m_idx_selected < m_num_default_presets;
        for (size_t i = 0; i < m_num_default_presets; ++ i)
            m_presets[i].is_visible = default_visible;
    }
}

size_t PresetCollection::update_compatible_internal(const PresetWithVendorProfile &active_printer, const PresetWithVendorProfile *active_print, PresetSelectCompatibleType unselect_if_incompatible)
{
    DynamicPrintConfig config = compatibility_context(active_printer.preset);
    bool some_compatible = false;
    std::vector<size_t> indices_of_template_presets;
    if(m_idx_selected < m_num_default_presets && unselect_if_incompatible != PresetSelectCompatibleType::Never)
        m_idx_selected = size_t(-1);
    for (size_t idx_preset = m_num_default_presets; idx_preset < m_presets.size(); ++ idx_preset) {
        bool    selected        = idx_preset == m_idx_selected;
        Preset &preset_selected = m_presets[idx_preset];
        Preset &preset_edited   = selected ? m_edited_preset : preset_selected;

        const PresetWithVendorProfile this_preset_with_vendor_profile = this->get_preset_with_vendor_profile(preset_edited);
        bool    was_compatible  = preset_edited.is_compatible;
        preset_edited.is_compatible = is_compatible_with_printer(this_preset_with_vendor_profile, active_printer, &config);
        some_compatible |= preset_edited.is_compatible;
	    if (active_print != nullptr)
	        preset_edited.is_compatible &= is_compatible_with_print(this_preset_with_vendor_profile, *active_print, active_printer);
        if (! preset_edited.is_compatible && selected &&
        	(unselect_if_incompatible == PresetSelectCompatibleType::Always || (unselect_if_incompatible == PresetSelectCompatibleType::OnlyIfWasCompatible && was_compatible)))
            m_idx_selected = size_t(-1);
        if (selected)
            preset_selected.is_compatible = preset_edited.is_compatible;
        if (preset_edited.vendor && preset_edited.vendor->templates_profile) {
            if (preset_selected.is_visible) {
                indices_of_template_presets.push_back(idx_preset);
            } else {
                preset_selected.is_compatible = false;
                if (selected)
                    m_idx_selected = size_t(-1);
            }
        }
    }
    // filter out template profiles where profile with same alias and compability exists
    if (!indices_of_template_presets.empty()) {
        for (size_t idx_preset = m_num_default_presets; idx_preset < m_presets.size(); ++idx_preset) {
            if (m_presets[idx_preset].vendor && !m_presets[idx_preset].vendor->templates_profile && m_presets[idx_preset].is_compatible) {
                std::string preset_alias = m_presets[idx_preset].alias;
                for (size_t idx_of_template_in_presets : indices_of_template_presets) {
                    if (m_presets[idx_of_template_in_presets].alias == preset_alias) {
                        // unselect selected template filament if there is non-template alias compatible
                        if (idx_of_template_in_presets == m_idx_selected && (unselect_if_incompatible == PresetSelectCompatibleType::Always || unselect_if_incompatible == PresetSelectCompatibleType::OnlyIfWasCompatible)) {
                            m_idx_selected = size_t(-1);
                        }
                        m_presets[idx_of_template_in_presets].is_compatible = false;
                        break;
                    }
                }
            }
        }
    }
    
    // Update visibility of the default profiles here if the defaults are suppressed, the current profile is not compatible and we don't want to select another compatible profile.
    if (m_idx_selected >= m_num_default_presets && m_default_suppressed)
	    for (size_t i = 0; i < m_num_default_presets; ++ i)
	        m_presets[i].is_visible = ! some_compatible;
    return m_idx_selected;
}

// Update a dirty flag of the current preset
// Return true if the dirty flag changed.
bool PresetCollection::update_dirty()
{
    bool was_dirty = this->get_selected_preset().is_dirty;
    bool is_dirty  = current_is_dirty();
    this->get_selected_preset().is_dirty = is_dirty;
    this->get_edited_preset().is_dirty = is_dirty;

    return was_dirty != is_dirty;
}

bool PresetCollection::is_dirty(const Preset *edited, const Preset *reference)
{
    // Only compares the comparable options existing in both configs.
    return edited != nullptr && reference != nullptr && ! reference->config.equals(edited->config, false);
}
template<class T>
void add_correct_opts_to_diff(const t_config_option_key &opt_key,
                              std::map<OptionKeyIdx, uint16_t> &vec,
                              const ConfigOption *option_cur,
                              const ConfigOption *option_init) {
    const T* opt_init = static_cast<const T*>(option_init);
    const T* opt_cur = static_cast<const T*>(option_cur);
    int opt_init_max_id = opt_init->size() - 1;
    // emplace the whole vector if size changed.
    if (opt_init->size() != opt_cur->size()) {
        vec.emplace(OptionKeyIdx::scalar(opt_key), PresetCollection::DIRTY_VECTOR_CHANGE_SIZE);
    }
    for (int32_t i = 0; i < int32_t(opt_cur->size()); i++) {
        // if (new one & we need to report new idx) or if hte value isn't the same.
        if (i >= opt_init->size()) {
            uint16_t status = PresetCollection::DIRTY_VECTOR_ADDED_IDX;
            if (opt_cur->get_at(i) == opt_init->get_at(0)) {
                status |= PresetCollection::DIRTY_VECTOR_SAME_AS_FIRST;
            }
            vec.emplace(OptionKeyIdx{opt_key, i}, status);
        } else if (opt_cur->get_at(i) != opt_init->get_at(i)) {
            vec.emplace(OptionKeyIdx{opt_key, i}, 0);
        }
    }
}

// Use deep_diff to correct return of changed options, considering individual options for each extruder.
inline std::map<OptionKeyIdx, uint16_t> deep_diff(const ConfigBase &config_this,
                                                  const ConfigBase &config_other,
                                                  bool ignore_phony) {
    std::map<OptionKeyIdx, uint16_t> diff;
    for (const t_config_option_key &opt_key : config_this.keys()) {
        const ConfigOption *this_opt  = config_this.option(opt_key);
        const ConfigOption *other_opt = config_other.option(opt_key);
        // dirty if both exist, the key is comparable, they aren't both phony and value is different.
        // A non comparable key is skipped before its vector is split by index.
        if (this_opt != nullptr && other_opt != nullptr && config_this.compares_key(opt_key)
            && (ignore_phony || !(this_opt->is_phony() && other_opt->is_phony()))
            && ((*this_opt != *other_opt) || (this_opt->is_phony() != other_opt->is_phony())))
        {
            switch (other_opt->type()) {
            case coInts:    add_correct_opts_to_diff<ConfigOptionInts       >(opt_key, diff, this_opt, other_opt);  break;
            case coBools:   add_correct_opts_to_diff<ConfigOptionBools      >(opt_key, diff, this_opt, other_opt);  break;
            case coFloats:  add_correct_opts_to_diff<ConfigOptionFloats     >(opt_key, diff, this_opt, other_opt);  break;
            case coStrings: add_correct_opts_to_diff<ConfigOptionStrings    >(opt_key, diff, this_opt, other_opt);  break;
            case coPercents:add_correct_opts_to_diff<ConfigOptionPercents   >(opt_key, diff, this_opt, other_opt);  break;
            case coFloatsOrPercents:add_correct_opts_to_diff<ConfigOptionFloatsOrPercents>(opt_key, diff, this_opt, other_opt);  break;
            case coPoints:  add_correct_opts_to_diff<ConfigOptionPoints     >(opt_key, diff, this_opt, other_opt);  break;
            case coGraphs:  add_correct_opts_to_diff<ConfigOptionGraphs     >(opt_key, diff, this_opt, other_opt);  break;
            default: diff.emplace(OptionKeyIdx::scalar(opt_key), 0); break;
            }
        }
    }
    return diff;
}

std::map<OptionKeyIdx, uint16_t> PresetCollection::dirty_options(const Preset *edited,
                                                         const Preset *reference,
                                                         const bool ignore_phony /*= false*/) {
    std::map<OptionKeyIdx, uint16_t> changed;
    if (edited != nullptr && reference != nullptr) {
        // Only compares the comparable options existing in both configs.
        changed = deep_diff(edited->config, reference->config, ignore_phony);
    }
    return changed;
}

// Select a new preset. This resets all the edits done to the currently selected preset.
// If the preset with index idx does not exist, a first visible preset is selected.
Preset& PresetCollection::select_preset(size_t idx)
{
    for (Preset &preset : m_presets)
        preset.is_dirty = false;
    if (idx >= m_presets.size())
        idx = first_visible_idx();
    m_idx_selected = idx;
    if (!m_presets[idx].is_visible)
        // The newly selected preset can be activated -> make it visible.
        m_presets[idx].is_visible = true;

    m_edited_preset = m_presets[idx];
    bool default_visible = ! m_default_suppressed || m_idx_selected < m_num_default_presets;
    for (size_t i = 0; i < m_num_default_presets; ++i)
        m_presets[i].is_visible = default_visible;
    return m_presets[idx];
}

bool PresetCollection::select_preset_by_name(const std::string &name_w_suffix, bool force)
{
    std::string name = Preset::remove_suffix_modified(name_w_suffix);
    // 1) Try to find the preset by its name.
    auto it = this->find_preset_internal(name);
    size_t idx = 0;
    if (it != m_presets.end() && it->name == name && it->is_visible)
        // Preset found by its name and it is visible.
        idx = it - m_presets.begin();
    else {
        // Find the first visible preset.
        for (size_t i = m_default_suppressed ? m_num_default_presets : 0; i < m_presets.size(); ++ i)
            if (m_presets[i].is_visible) {
                idx = i;
                break;
            }
        // If the first visible preset was not found, return the 0th element, which is the default preset.
    }

    // 2) Select the new preset.
    if (m_idx_selected != idx || force) {
        this->select_preset(idx);
        return true;
    }

    return false;
}

bool PresetCollection::select_preset_by_name_strict(const std::string &name)
{
    // 1) Try to find the preset by its name.
    auto it = this->find_preset_internal(name);
    size_t idx = (size_t)-1;
    if (it != m_presets.end() && it->name == name && it->is_visible)
        // Preset found by its name.
        idx = it - m_presets.begin();
    // 2) Select the new preset.
    if (idx != (size_t)-1) {
        this->select_preset(idx);
        return true;
    }
    m_idx_selected = idx;
    return false;
}

// Merge one vendor's presets with the other vendor's presets, report duplicates.
std::vector<std::string> PresetCollection::merge_presets(PresetCollection &&other, const VendorMap &new_vendors)
{
    std::vector<std::string> duplicates;
    for (Preset &preset : other.m_presets) {
        if (preset.is_default || preset.is_external)
            continue;
        Preset key(m_type, preset.name);
        auto it = std::lower_bound(m_presets.begin() + m_num_default_presets, m_presets.end(), key);
        if (it == m_presets.end() || it->name != preset.name) {
            if (preset.vendor != nullptr) {
                // Re-assign a pointer to the vendor structure in the new PresetBundle.
                auto it = new_vendors.find(preset.vendor->id);
                assert(it != new_vendors.end());
                preset.vendor = &it->second;
            }
            m_presets.emplace(it, std::move(preset));
        } else
            duplicates.emplace_back(std::move(preset.name));
    }
    return duplicates;
}

void PresetCollection::update_vendor_ptrs_after_copy(const VendorMap &new_vendors)
{
    const auto rebind = [&new_vendors](Preset &preset) {
        if (preset.vendor != nullptr) {
            assert(! preset.is_default && ! preset.is_external);
            // Re-assign a pointer to the vendor structure in the new PresetBundle.
            const VendorMap::const_iterator it = new_vendors.find(preset.vendor->id);
            assert(it != new_vendors.end());
            preset.vendor = &it->second;
        }
    };
    for (Preset &preset : m_presets)
        rebind(preset);
    rebind(m_edited_preset);
    rebind(m_saved_preset);
}

void PresetCollection::assign_subset(const PresetCollection &source, const std::set<std::string> &names)
{
    std::set<std::string> kept;
    // Retain every parent, including user profiles. A name already visited also stops an inheritance loop.
    const auto keep = [&source, &kept](const Preset *preset) {
        while (preset != nullptr && kept.insert(preset->name).second)
            preset = source.get_preset_parent(*preset);
    };
    for (const std::string &name : names) {
        const Preset *preset = source.find_preset(name, false, false);
        if (preset == nullptr) {
            const std::deque<Preset>::const_iterator renamed = source.find_preset_renamed(name);
            if (renamed != source.m_presets.end())
                preset = &*renamed;
        }
        keep(preset);
    }
    // Selection can consult the selected profile or the first visible profile as a fallback.
    // Keep both even when neither name was requested.
    if (source.m_idx_selected < source.m_presets.size() && source.m_idx_selected >= source.m_num_default_presets)
        keep(&source.m_presets[source.m_idx_selected]);
    if (const size_t first_visible = source.first_visible_idx(); first_visible >= source.m_num_default_presets)
        keep(&source.m_presets[first_visible]);
    m_type = source.m_type;
    m_default_suppressed = source.m_default_suppressed;
    m_num_default_presets = source.m_num_default_presets;
    m_dir_path = source.m_dir_path;
    m_map_alias_to_profile_name.clear();
    for (const std::pair<std::string, std::string> &alias : source.m_map_alias_to_profile_name)
        if (kept.count(alias.second) != 0)
            m_map_alias_to_profile_name.push_back(alias);
    m_map_system_profile_renamed.clear();
    for (const std::pair<const std::string, std::string> &renamed : source.m_map_system_profile_renamed)
        if (kept.count(renamed.second) != 0)
            m_map_system_profile_renamed.insert(renamed);
    m_presets.clear();
    for (size_t index = 0; index < source.m_presets.size(); ++index)
        if (index < source.m_num_default_presets || kept.count(source.m_presets[index].name) != 0)
            m_presets.push_back(source.m_presets[index]);
    m_idx_selected = 0;
    m_edited_preset = m_presets.front();
    m_saved_preset = m_presets.front();
    if (source.m_idx_selected < source.m_presets.size()) {
        const std::string &selected = source.m_presets[source.m_idx_selected].name;
        for (size_t index = 0; index < m_presets.size(); ++index)
            if (m_presets[index].name == selected) {
                m_idx_selected = index;
                m_edited_preset = source.m_edited_preset;
                m_saved_preset = source.m_saved_preset;
                break;
            }
    }
}

void PresetCollection::update_map_alias_to_profile_name()
{
	m_map_alias_to_profile_name.clear();
	for (const Preset &preset : m_presets)
		m_map_alias_to_profile_name.emplace_back(preset.alias, preset.name);
	std::sort(m_map_alias_to_profile_name.begin(), m_map_alias_to_profile_name.end(), [](auto &l, auto &r) { return l.first < r.first; });
}

void PresetCollection::update_map_system_profile_renamed()
{
	m_map_system_profile_renamed.clear();
	for (Preset &preset : m_presets)
		for (const std::string &renamed_from : preset.renamed_from) {
            const auto [it, success] = m_map_system_profile_renamed.insert(std::pair<std::string, std::string>(renamed_from, preset.name));
			if (! success)
                BOOST_LOG_TRIVIAL(error) << boost::format("Preset name \"%1%\" was marked as renamed from \"%2%\", though preset name \"%3%\" was marked as renamed from \"%2%\" as well.") % preset.name % renamed_from % it->second;
		}
}

std::string PresetCollection::name() const
{
    switch (this->type()) {
    case Preset::TYPE_FFF_PRINT:    return L("print");
    case Preset::TYPE_FFF_FILAMENT: return L("filament");
    case Preset::TYPE_SLA_PRINT:    return L("SLA print");
    case Preset::TYPE_SLA_MATERIAL: return L("SLA material");
    case Preset::TYPE_PRINTER:      return L("printer");
    default:                        return "invalid";
    }
}

std::string Preset::type_name(Type t) {
    switch (t) {
    case Preset::TYPE_FFF_PRINT:    return "print";
    case Preset::TYPE_FFF_FILAMENT: return "filament";
    case Preset::TYPE_SLA_PRINT:    return "sla_print";
    case Preset::TYPE_SLA_MATERIAL: return "sla_material";
    case Preset::TYPE_PRINTER:      return "printer";
    case Preset::TYPE_FREQUENT_FFF: return "freq_fff";
    case Preset::TYPE_FREQUENT_SLA: return "freq_sla";
    default:                        return "invalid";
    }
}

Preset::Type Preset::type_from_name(std::string name) { 
    if ("print" == name)
        return Preset::TYPE_FFF_PRINT;
    if ("filament" == name)
        return Preset::TYPE_FFF_FILAMENT;
    if ("sla_print" == name)
        return Preset::TYPE_SLA_PRINT;
    if ("sla_material" == name)
        return Preset::TYPE_SLA_MATERIAL;
    if ("printer" == name)
        return Preset::TYPE_PRINTER;
    if ("freq_fff" == name)
        return Preset::TYPE_FREQUENT_FFF;
    if ("freq_sla" == name)
        return Preset::TYPE_FREQUENT_SLA;
    return Preset::TYPE_INVALID;
}

std::string PresetCollection::section_name() const
{
    return Preset::type_name(this->type());
}

// Used for validating the "inherits" flag when importing user's config bundles.
// Returns names of all system presets including the former names of these presets.
std::vector<std::string> PresetCollection::system_preset_names() const
{
    size_t num = 0;
    for (const Preset &preset : m_presets)
        if (preset.is_system)
            ++ num;
    std::vector<std::string> out;
    out.reserve(num);
    for (const Preset &preset : m_presets)
        if (preset.is_system) {
            out.emplace_back(preset.name);
            out.insert(out.end(), preset.renamed_from.begin(), preset.renamed_from.end());
        }
    std::sort(out.begin(), out.end());
    return out;
}

// Generate a file path from a profile name. Add the ".ini" suffix if it is missing.
std::string PresetCollection::path_from_name(const std::string &new_name) const
{
    std::string file_name = boost::iends_with(new_name, ".ini") ? new_name : (new_name + ".ini");
    return (boost::filesystem::path(m_dir_path) / file_name).make_preferred().string();
}

const Preset& PrinterPresetCollection::default_preset_for(const DynamicPrintConfig &config) const
{
    const ConfigOptionEnumGeneric *opt_printer_technology = config.opt<ConfigOptionEnumGeneric>("printer_technology");
    return this->default_preset((opt_printer_technology == nullptr || opt_printer_technology->value == ptFFF) ? 0 : 1);
}

const Preset* PrinterPresetCollection::find_system_preset_by_model_and_variant(const std::string &model_id, const std::string& variant) const
{
    if (model_id.empty()) { return nullptr; }

    const auto it = std::find_if(cbegin(), cend(), [&](const Preset &preset) {
        if (!preset.is_system || preset.config.opt_string("printer_model") != model_id)
            return false;
        if (variant.empty())
            return true;
        return preset.config.opt_string("printer_variant") == variant;
    });

    return it != cend() ? &*it : nullptr;
}

bool  PrinterPresetCollection::only_default_printers() const
{
    for (const auto& printer : get_presets()) {
        if (! printer.is_default)
            return false;
    }
    return true;
}
// -------------------------
// ***  PhysicalPrinter  ***
// -------------------------

std::string PhysicalPrinter::separator()
{
    return " * ";
}

static std::vector<std::string> s_PhysicalPrinter_opts {
    "preset_name", // temporary option to compatibility with older Slicer
    "preset_names",
    "printer_technology",
    "host_type",
    "print_host",
    "printhost_apikey",
    "printhost_cafile",
    "printhost_client_cert",
    "printhost_client_cert_password",
    "printhost_port",
    "printhost_authorization_type",
    // HTTP digest authentization (RFC 2617)
    "printhost_user",
    "printhost_password",
    "printhost_ssl_ignore_revoke"
};

const std::vector<std::string>& PhysicalPrinter::printer_options()
{
    return s_PhysicalPrinter_opts;
}

static constexpr auto legacy_print_host_options = {
            "print_host",
            "printhost_apikey",
            "printhost_cafile",
            "printhost_port"
        };

std::vector<std::string> PhysicalPrinter::presets_with_print_host_information(const PrinterPresetCollection& printer_presets)
{
    std::vector<std::string> presets;
    for (const Preset& preset : printer_presets)
        if (has_print_host_information(preset.config))
            presets.emplace_back(preset.name);

    return presets;
}

bool PhysicalPrinter::has_print_host_information(const DynamicPrintConfig& config)
{
    for (const char *opt : legacy_print_host_options)
        if (!config.opt_string(opt).empty())
            return true;

    return false;
}

const std::set<std::string>& PhysicalPrinter::get_preset_names() const
{
    return preset_names;
}

bool PhysicalPrinter::has_empty_config() const
{
    return  config.opt_string("print_host"                      ).empty() &&
            config.opt_string("printhost_apikey"                ).empty() &&
            config.opt_string("printhost_cafile"                ).empty() &&
            config.opt_string("printhost_client_cert"           ).empty() && 
            config.opt_string("printhost_client_cert_password"  ).empty() && 
            config.opt_string("printhost_port"                  ).empty() &&
            config.opt_string("printhost_user"                  ).empty() &&
            config.opt_string("printhost_password"              ).empty() && 
            config.opt_string("printhost_port"                  ).empty();
}

// temporary workaround for compatibility with older Slicer
static void update_preset_name_option(const std::set<std::string>& preset_names, DynamicPrintConfig& config)
{
    std::string name;
    for (auto el : preset_names)
        name += el + ";";
    name.pop_back();
    config.set_key_value("preset_name", new ConfigOptionString(name));
}

void PhysicalPrinter::update_preset_names_in_config()
{
    if (!preset_names.empty()) {
        config.option<ConfigOptionStrings>("preset_names")->set(std::vector<std::string>(preset_names.begin(), preset_names.end()));

        // temporary workaround for compatibility with older Slicer
        update_preset_name_option(preset_names, config);
    }
}

void PhysicalPrinter::save(const std::string& file_name_from, const std::string& file_name_to)
{
    // rename the file
    boost::nowide::rename(file_name_from.data(), file_name_to.data());
    this->file = file_name_to;
    // save configuration
    this->config.save(this->file, ConfigWriteTarget::Local);
}

void PhysicalPrinter::update_from_preset(const Preset& preset)
{
    config.apply_only(preset.config, printer_options(), true);
    // add preset names to the options list
    preset_names.emplace(preset.name);
    update_preset_names_in_config();
}

void PhysicalPrinter::update_from_config(const DynamicPrintConfig& new_config)
{
    config.apply_only(new_config, printer_options(), false);

    const std::vector<std::string>& values = config.option<ConfigOptionStrings>("preset_names")->get_values();

    if (values.empty())
        preset_names.clear();
    else {
        for (const std::string& val : values)
            preset_names.emplace(val);
        // temporary workaround for compatibility with older Slicer
        update_preset_name_option(preset_names, config);
    }
}

void PhysicalPrinter::reset_presets()
{
    return preset_names.clear();
}

bool PhysicalPrinter::add_preset(const std::string& preset_name)
{
    return preset_names.emplace(preset_name).second;
}

bool PhysicalPrinter::delete_preset(const std::string& preset_name)
{
    return preset_names.erase(preset_name) > 0;
}

PhysicalPrinter::PhysicalPrinter(const std::string& name, const DynamicPrintConfig& default_config) :
    name(name), config(default_config)
{
    update_from_config(config);
}

PhysicalPrinter::PhysicalPrinter(const std::string& name, const DynamicPrintConfig &default_config, const Preset& preset) :
    name(name), config(default_config)
{
    update_from_preset(preset);
}

void PhysicalPrinter::set_name(const std::string& name)
{
    this->name = name;
}

std::string PhysicalPrinter::get_full_name(const std::string& preset_name) const
{
    return name + separator() + preset_name;
}

std::string PhysicalPrinter::get_short_name(std::string full_name)
{
    int pos = full_name.find(separator());
    if (pos > 0)
        boost::erase_tail(full_name, full_name.length() - pos);
    return full_name;
}

std::string PhysicalPrinter::get_preset_name(std::string name)
{
    int pos = name.find(separator());
    boost::erase_head(name, pos + 3);
    return Preset::remove_suffix_modified(name);
}


// -----------------------------------
// ***  PhysicalPrinterCollection  ***
// -----------------------------------

PhysicalPrinterCollection::PhysicalPrinterCollection( const std::vector<std::string>& keys, PresetBundle* preset_bundle)
    :m_preset_bundle_owner(preset_bundle)
{
    // Default config for a physical printer containing all key/value pairs of PhysicalPrinter::printer_options().
    for (const std::string &key : keys) {
        const ConfigOptionDef *opt = PrintConfigDef::instance().get(key);
        assert(opt);
        assert(opt->default_value);
        m_default_config.set_key_value(key, opt->default_value->clone());
    }
}

static void update_preset_names_if_were_renamed(std::set<std::string>& preset_names, const PrinterPresetCollection& printer_presets)
{
    std::set<std::string> new_names;

    bool was_renamed{ false };
    for (const std::string& preset_name : preset_names) {
        const Preset* preset = printer_presets.find_preset(preset_name);
        if (preset)
            new_names.emplace(preset_name);
        else {
            if (const std::string* new_name = printer_presets.get_preset_name_renamed(preset_name)) {
                BOOST_LOG_TRIVIAL(warning) << "Printer preset present " << preset_name << "was renamed to: " << *new_name << std::endl;
                new_names.emplace(*new_name);
                was_renamed = true;
            }
        }
    }

    if (was_renamed)
        preset_names = new_names;
}

// Load all printers found in dir_path.
// Throws an exception on error.
void PhysicalPrinterCollection::load_printers(
    const std::string& dir_path, const std::string& subdir, 
    PresetsConfigSubstitutions& substitutions, ForwardCompatibilitySubstitutionRule substitution_rule)
{
    // Don't use boost::filesystem::canonical() on Windows, it is broken in regard to reparse points,
    // see https://github.com/prusa3d/PrusaSlicer/issues/732
    boost::filesystem::path dir = boost::filesystem::absolute(boost::filesystem::path(dir_path) / subdir).make_preferred();
    m_dir_path = dir.string();
    std::string errors_cummulative;
    // Store the loaded printers into a new vector, otherwise the binary search for already existing presets would be broken.
    std::deque<PhysicalPrinter> printers_loaded;
    for (auto& dir_entry : boost::filesystem::directory_iterator(dir))
        if (Slic3r::is_ini_file(dir_entry)) {
            std::string name = dir_entry.path().filename().string();
            // Remove the .ini suffix.
            name.erase(name.size() - 4);
            if (PhysicalPrinter* found_printer = this->find_printer(name, false)) {
                // This happens when there's is a preset (most likely legacy one) with the same name as a system preset
                // that's already been loaded from a bundle.
                BOOST_LOG_TRIVIAL(warning) << "Printer already present, not loading: " << name;

                // But some of used printer_preset might have been renamed.
                // Check it and replace with new name(s) if it's needed
                update_preset_names_if_were_renamed(found_printer->preset_names, m_preset_bundle_owner->printers);

                continue;
            }
            try {
                PhysicalPrinter printer(name, this->default_config());
                printer.file = dir_entry.path().string();
                // Load the preset file, apply preset values on top of defaults.
                try {
                    DynamicPrintConfig config;
                    ConfigSubstitutions config_substitutions = config.load_from_ini(printer.file, substitution_rule);
                    if (! config_substitutions.empty())
                        substitutions.push_back({ name, Preset::Type::TYPE_PHYSICAL_PRINTER, PresetConfigSubstitutions::Source::UserFile, printer.file, std::move(config_substitutions) });
                    printer.update_from_config(config);
                    printer.loaded = true;
                    // Some of used printer_preset might have been renamed.
                    // Check it and replace with new name(s) if it's needed
                    update_preset_names_if_were_renamed(printer.preset_names, m_preset_bundle_owner->printers);
                }
                catch (const std::ifstream::failure& err) {
                    throw Slic3r::RuntimeError(std::string("The selected preset cannot be loaded: ") + printer.file + "\n\tReason: " + err.what());
                }
                catch (const std::runtime_error& err) {
                    throw Slic3r::RuntimeError(std::string("Failed loading the preset file: ") + printer.file + "\n\tReason: " + err.what());
                }
                printers_loaded.emplace_back(printer);
            }
            catch (const std::runtime_error& err) {
                errors_cummulative += err.what();
                errors_cummulative += "\n";
            }
        }
    m_printers.insert(m_printers.end(), std::make_move_iterator(printers_loaded.begin()), std::make_move_iterator(printers_loaded.end()));
    std::sort(m_printers.begin(), m_printers.end());
    if (!errors_cummulative.empty())
        throw Slic3r::RuntimeError(errors_cummulative);
}

void PhysicalPrinterCollection::load_printer(const std::string& path, const std::string& name, DynamicPrintConfig&& config, bool select, bool save/* = false*/)
{
    auto it = this->find_printer_internal(name);
    if (it == m_printers.end() || it->name != name) {
        // The preset was not found. Create a new preset.
        it = m_printers.emplace(it, PhysicalPrinter(name, config));
    }

    it->file = path;
    it->config = std::move(config);
    it->loaded = true;
    // Some of used printer_preset might have been renamed.
    // Check it and replace with new name(s) if it's needed
    update_preset_names_if_were_renamed(it->preset_names, m_preset_bundle_owner->printers);
    if (select)
        this->select_printer(*it);

    if (save)
        it->save();
}

// if there is saved user presets, contains information about "Print Host upload",
// Create default printers with this presets
// Note! "Print Host upload" options will be cleared after physical printer creations
void PhysicalPrinterCollection::load_printers_from_presets(PrinterPresetCollection& printer_presets)
{
    int cnt=0;
    for (Preset& preset: printer_presets) {
        DynamicPrintConfig& config = preset.config;
        for(const char* option : legacy_print_host_options) {
            if (!config.opt_string(option).empty()) {
                // check if printer with those "Print Host upload" options already exist
                PhysicalPrinter* existed_printer = find_printer_with_same_config(config);
                if (existed_printer)
                    // just add preset for this printer
                    existed_printer->add_preset(preset.name);
                else {
                    std::string new_printer_name = (boost::format("Printer %1%") % ++cnt ).str();
                    while (find_printer(new_printer_name))
                        new_printer_name = (boost::format("Printer %1%") % ++cnt).str();

                    // create new printer from this preset
                    PhysicalPrinter printer(new_printer_name, this->default_config(), preset);
                    printer.loaded = true;
                    save_printer(printer);
                }

                // erase "Print Host upload" information from the preset
                for (const char *opt : legacy_print_host_options)
                    config.opt_string(opt).clear();
                // save changes for preset
                preset.save();

                // update those changes for edited preset if it's equal to the preset
                Preset& edited = printer_presets.get_edited_preset();
                if (preset.name == edited.name) {
                    for (const char *opt : legacy_print_host_options)
                        edited.config.opt_string(opt).clear();
                }

                break;
            }
        }
    }
}

PhysicalPrinter* PhysicalPrinterCollection::find_printer( const std::string& name, bool case_sensitive_search)
{
    auto it = this->find_printer_internal(name, case_sensitive_search);

    // Ensure that a temporary copy is returned if the preset found is currently selected.
    auto is_equal_name = [name, case_sensitive_search](const std::string& in_name) {
        if (case_sensitive_search)
            return in_name == name;
        return boost::to_lower_copy<std::string>(in_name) == boost::to_lower_copy<std::string>(name);
    };

    if (it == m_printers.end() || !is_equal_name(it->name))
        return nullptr;
    return &this->printer(it - m_printers.begin());
}

std::deque<PhysicalPrinter>::iterator PhysicalPrinterCollection::find_printer_internal(const std::string& name, bool case_sensitive_search/* = true*/)
{
    if (case_sensitive_search)
        return Slic3r::lower_bound_by_predicate(m_printers.begin(), m_printers.end(), [&name](const auto& l) { return l.name < name;  });

    std::string low_name = boost::to_lower_copy<std::string>(name);

    size_t i = 0;
    for (const PhysicalPrinter& printer : m_printers) {
        if (boost::to_lower_copy<std::string>(printer.name) == low_name)
            break;
        i++;
    }
    if (i == m_printers.size())
        return m_printers.end();

    return m_printers.begin() + i;
}

PhysicalPrinter* PhysicalPrinterCollection::find_printer_with_same_config(const DynamicPrintConfig& config)
{
    for (const PhysicalPrinter& printer :*this) {
        bool is_equal = true;
        for (const char *opt : legacy_print_host_options)
            if (is_equal && printer.config.opt_string(opt) != config.opt_string(opt))
                is_equal = false;

        if (is_equal)
            return find_printer(printer.name);
    }
    return nullptr;
}

// Generate a file path from a profile name. Add the ".ini" suffix if it is missing.
std::string PhysicalPrinterCollection::path_from_name(const std::string& new_name) const
{
    std::string file_name = boost::iends_with(new_name, ".ini") ? new_name : (new_name + ".ini");
    return (boost::filesystem::path(m_dir_path) / file_name).make_preferred().string();
}

void PhysicalPrinterCollection::save_printer(PhysicalPrinter& edited_printer, const std::string& renamed_from/* = ""*/)
{
    // controll and update preset_names in edited_printer config
    edited_printer.update_preset_names_in_config();

    std::string name = renamed_from.empty() ? edited_printer.name : renamed_from;
    // 1) Find the printer with a new_name or create a new one,
    // initialize it with the edited config.
    auto it = this->find_printer_internal(name);
    if (it != m_printers.end() && it->name == name) {
        // Printer with the same name found.
        // Overwriting an existing preset.
        it->config = std::move(edited_printer.config);
        it->name = edited_printer.name;
        it->preset_names = edited_printer.preset_names;
        // sort printers and get new it
        std::sort(m_printers.begin(), m_printers.end());
        it = this->find_printer_internal(edited_printer.name);
    }
    else {
        // Creating a new printer.
        it = m_printers.emplace(it, edited_printer);
    }
    assert(it != m_printers.end());

    // 2) Save printer
    PhysicalPrinter& printer = *it;
    if (printer.file.empty())
        printer.file = this->path_from_name(printer.name);

    if (printer.file == this->path_from_name(printer.name))
        printer.save();
    else
        // if printer was renamed, we should rename a file and than save the config
        printer.save(printer.file, this->path_from_name(printer.name));

    // update idx_selected
    m_idx_selected = it - m_printers.begin();
}

bool PhysicalPrinterCollection::delete_printer(const std::string& name)
{
    auto it = this->find_printer_internal(name);
    if (it == m_printers.end())
        return false;

    const PhysicalPrinter& printer = *it;
    // Erase the preset file.
    boost::nowide::remove(printer.file.c_str());
    m_printers.erase(it);
    return true;
}

bool PhysicalPrinterCollection::delete_selected_printer()
{
    if (!has_selection())
        return false;
    const PhysicalPrinter& printer = this->get_selected_printer();

    // Erase the preset file.
    boost::nowide::remove(printer.file.c_str());
    // Remove the preset from the list.
    m_printers.erase(m_printers.begin() + m_idx_selected);
    // unselect all printers
    unselect_printer();

    return true;
}

bool PhysicalPrinterCollection::delete_preset_from_printers( const std::string& preset_name)
{
    std::vector<std::string> printers_for_delete;
    for (PhysicalPrinter& printer : m_printers) {
        if (printer.preset_names.size() == 1 && *printer.preset_names.begin() == preset_name)
            printers_for_delete.emplace_back(printer.name);
        else if (printer.delete_preset(preset_name))
            save_printer(printer);
    }

    if (!printers_for_delete.empty())
        for (const std::string& printer_name : printers_for_delete)
            delete_printer(printer_name);

    unselect_printer();
    return true;
}

void PhysicalPrinterCollection::rename_preset_in_printers(const std::string& old_preset_name, const std::string& new_preset_name)
{
    for (PhysicalPrinter& printer : m_printers)
        if (printer.delete_preset(old_preset_name)) {
            printer.add_preset(new_preset_name);
            printer.update_preset_names_in_config();
            printer.save();
        }
}

// Get list of printers which have more than one preset and "preset_names" preset is one of them
std::vector<std::string> PhysicalPrinterCollection::get_printers_with_preset(const std::string& preset_name, bool respect_only_preset /*= true*/)
{
    std::vector<std::string> printers;

    for (auto printer : m_printers) {
        if (!respect_only_preset && printer.preset_names.size() == 1)
            continue;
        if (printer.preset_names.find(preset_name) != printer.preset_names.end())
            printers.emplace_back(printer.name);
    }

    return printers;
}

// Get list of printers which has only "preset_names" preset
std::vector<std::string> PhysicalPrinterCollection::get_printers_with_only_preset(const std::string& preset_name)
{
    std::vector<std::string> printers;

    for (const PhysicalPrinter& printer : m_printers)
        if (printer.preset_names.size() == 1 && *printer.preset_names.begin() == preset_name)
            printers.emplace_back(printer.name);

    return printers;
}

std::string PhysicalPrinterCollection::get_selected_full_printer_name() const
{
    return (m_idx_selected == size_t(-1)) ? std::string() : this->get_selected_printer().get_full_name(m_selected_preset);
}

void PhysicalPrinterCollection::select_printer(const std::string& full_name)
{
    std::string printer_name = PhysicalPrinter::get_short_name(full_name);
    auto it = this->find_printer_internal(printer_name);
    if (it == m_printers.end()) {
        unselect_printer();
        return;
    }

    // update idx_selected
    m_idx_selected = it - m_printers.begin();

    // update name of the currently selected preset
    if (printer_name == full_name)
        // use first preset in the list
        m_selected_preset = *it->preset_names.begin();
    else
        m_selected_preset = it->get_preset_name(full_name);

    // Check if selected preset wasn't renamed and replace it with new name
    if (const std::string* new_name = m_preset_bundle_owner->printers.get_preset_name_renamed(m_selected_preset))
        m_selected_preset = *new_name;
}

void PhysicalPrinterCollection::select_printer(const std::string& printer_name, const std::string& preset_name)
{
    if (preset_name.empty())
        return select_printer(printer_name);
    return select_printer(printer_name + PhysicalPrinter::separator() + preset_name);
}

void PhysicalPrinterCollection::select_printer(const PhysicalPrinter& printer)
{
    return select_printer(printer.name);
}

bool PhysicalPrinterCollection::has_selection() const
{
    return m_idx_selected != size_t(-1);
}

void PhysicalPrinterCollection::unselect_printer()
{
    m_idx_selected = size_t(-1);
    m_selected_preset.clear();
}

bool PhysicalPrinterCollection::is_selected(PhysicalPrinterCollection::ConstIterator it, const std::string& preset_name) const
{
    return  m_idx_selected      == size_t(it - m_printers.begin()) &&
            m_selected_preset   == preset_name;
}

ExtruderFilaments::ExtruderFilaments(PresetCollection* filaments_collection, size_t extruder_id, std::string selected_name/* = std::string()*/)
: m_filaments (filaments_collection)
, m_extruder_id(extruder_id)
{
    const std::deque<Preset>& presets = m_filaments->get_presets();
    for (size_t id = 0; id < presets.size(); id ++)
        m_extr_filaments.emplace_back(&(presets[id]));

    select_filament(selected_name.empty() ? m_filaments->get_selected_preset_name() : selected_name);
}

const std::string& ExtruderFilaments::get_preset_name_by_alias(const std::string& alias) const
{
    const auto& aliases_map = m_filaments->map_alias_to_profile_name();
    for (
        // Find the 1st profile name with the alias.
        auto it = Slic3r::lower_bound_by_predicate(aliases_map.begin(), aliases_map.end(), [&alias](auto& l) { return l.first < alias; });
        // Continue over all profile names with the same alias.
        it != aliases_map.end() && it->first == alias; ++it)
        if (auto it_filament = find_filament_internal(it->second);
            it_filament != m_extr_filaments.end() && it_filament->preset->name == it->second &&
            it_filament->preset->is_visible && (it_filament->is_compatible || size_t(it_filament - m_extr_filaments.begin()) == m_idx_selected))
            return it_filament->preset->name;
    return alias;
}

bool ExtruderFilaments::select_filament(const std::string &name_w_suffix, bool force/*= false*/)
{
    std::string name = Preset::remove_suffix_modified(name_w_suffix);
    // 1) Try to find the preset by its name.
    auto it = this->find_filament_internal(name);
    size_t idx = 0;
    if (it != m_extr_filaments.end() && it->preset->name == name && it->preset->is_visible)
        // Preset found by its name and it is visible.
        idx = it - m_extr_filaments.begin();
    else {
        // Find the first visible preset.
        for (size_t i = 0; i < m_extr_filaments.size(); ++i)
            if (m_extr_filaments[i].preset->is_visible/* && m_extr_filaments[i].is_compatible*/) {
                idx = i;
                break;
            }
        // If the first visible preset was not found, return the 0th element, which is the default preset.
    }
    // 2) Select the new preset.
    if (m_idx_selected != idx || force) {
        this->select_filament(idx);
        return true;
    }

    return false;
}

Preset ExtruderFilaments::printer_for_extruder(const Preset &printer, size_t extruder_id)
{
    Preset printer_preset_adjusted = printer;
    if (extruder_id > 0 && !printer_preset_adjusted.config.opt_bool("single_extruder_multi_material")) {
        DynamicPrintConfig& active_printer_config = printer_preset_adjusted.config;
        for (const std::string& key : PrintConfigDef::instance().extruder_option_keys()) {
            if (key == "default_filament_profile")
                continue;// Ignore this field, because this parameter is not related to the extruder but to whole printer.
            ConfigOption *opt = active_printer_config.option(key, false);
            if (opt != nullptr && opt->is_vector())
                static_cast<ConfigOptionVectorBase*>(opt)->set_at(*opt, 0, extruder_id);
        }
    }
    return printer_preset_adjusted;
}

size_t ExtruderFilaments::update_compatible_internal(const PresetWithVendorProfile &active_printer,
                                                    const PresetWithVendorProfile *active_print,
                                                    PresetSelectCompatibleType unselect_if_incompatible)
{
    DynamicPrintConfig config = compatibility_context(active_printer.preset);
    const Preset printer_preset_adjusted = printer_for_extruder(active_printer.preset, m_extruder_id);
    PresetWithVendorProfile active_printer_adjusted(printer_preset_adjusted, active_printer.vendor);

    std::vector<size_t> indices_of_template_presets;
    indices_of_template_presets.reserve(m_extr_filaments.size());

    size_t num_default_presets = m_filaments->num_default_presets();
    for (size_t idx_preset = num_default_presets; idx_preset < m_extr_filaments.size(); ++idx_preset) {
        const bool    is_selected   = idx_preset == m_idx_selected;
        const Preset* preset        = m_extr_filaments[idx_preset].preset;
        Filament& extr_filament = m_extr_filaments[idx_preset];

        const PresetWithVendorProfile this_preset_with_vendor_profile = m_filaments->get_preset_with_vendor_profile(*preset);
        bool    was_compatible = extr_filament.is_compatible;
        extr_filament.is_compatible = is_compatible_with_printer(this_preset_with_vendor_profile, active_printer_adjusted, &config);
        if (active_print != nullptr)
            extr_filament.is_compatible &= is_compatible_with_print(this_preset_with_vendor_profile, *active_print, active_printer_adjusted);
        if (!extr_filament.is_compatible && is_selected &&
            (unselect_if_incompatible == PresetSelectCompatibleType::Always || (unselect_if_incompatible == PresetSelectCompatibleType::OnlyIfWasCompatible && was_compatible)))
            m_idx_selected = size_t(-1);
        if (preset->vendor && preset->vendor->templates_profile) {
            if (preset->is_visible)
                indices_of_template_presets.push_back(idx_preset);
            else {
                extr_filament.is_compatible = false;
                if (is_selected)
                    m_idx_selected = size_t(-1);
            }
        }
    }

    // filter out template profiles where profile with same alias and compability exists
    if (!indices_of_template_presets.empty()) {
        for (size_t idx = num_default_presets; idx < m_extr_filaments.size(); ++idx) {
            const Filament& filament    = m_extr_filaments[idx];
            const VendorProfile*    vendor      = filament.preset->vendor;
            if (vendor && !vendor->templates_profile && filament.is_compatible) {
                const std::string& preset_alias = filament.preset->alias;
                for (const auto& template_idx : indices_of_template_presets) {
                    if (m_extr_filaments[template_idx].preset->alias == preset_alias) {
                        m_extr_filaments[template_idx].is_compatible = false;
                        // unselect selected template filament if there is non-template alias compatible
                        if (template_idx == m_idx_selected && (unselect_if_incompatible != PresetSelectCompatibleType::Never))
                            m_idx_selected = size_t(-1);
                        break;
                    }
                }
            }
        }
    }

    return m_idx_selected;
}


namespace PresetUtils {
	const VendorProfile::PrinterModel* system_printer_model(const Preset &preset)
	{
		const VendorProfile::PrinterModel *out = nullptr;
		if (preset.vendor != nullptr) {
			const auto *printer_model = preset.config.opt<ConfigOptionString>("printer_model");
			if (printer_model != nullptr && ! printer_model->value.empty()) {
				auto it = std::find_if(preset.vendor->models.begin(), preset.vendor->models.end(), [printer_model](const VendorProfile::PrinterModel &pm) { return pm.id == printer_model->value; });
				if (it != preset.vendor->models.end())
					out = &(*it);
			}
		}
		return out;
	}

    std::string system_printer_bed_model(const Preset& preset)
    {
        std::string out;
        const VendorProfile::PrinterModel* pm = PresetUtils::system_printer_model(preset);
        if (pm != nullptr && !pm->bed_model.empty()) {
            out = Slic3r::data_dir() + "/vendor/" + preset.vendor->id + "/" + pm->bed_model;
            if (!boost::filesystem::exists(boost::filesystem::path(out)))
                out = Slic3r::resources_dir() + "/profiles/" + preset.vendor->id + "/" + pm->bed_model;
        }
        return out;
    }

    std::string system_printer_bed_texture(const Preset& preset)
    {
        std::string out;
        const VendorProfile::PrinterModel* pm = PresetUtils::system_printer_model(preset);
        if (pm != nullptr && !pm->bed_texture.empty()) {
            out = Slic3r::data_dir() + "/vendor/" + preset.vendor->id + "/" + pm->bed_texture;
            if (!boost::filesystem::exists(boost::filesystem::path(out)))
                out = Slic3r::resources_dir() + "/profiles/" + preset.vendor->id + "/" + pm->bed_texture;
        }
        return out;
    }

    bool vendor_profile_has_all_resources(const VendorProfile& vp)
    {
        namespace fs = boost::filesystem;

        std::string vendor_folder = Slic3r::data_dir()      + "/vendor/"   + vp.id + "/";
        std::string rsrc_folder   = Slic3r::resources_dir() + "/profiles/" + vp.id + "/";
        std::string cache_folder  = Slic3r::data_dir()      + "/cache/"    + vp.id + "/";
        for (const VendorProfile::PrinterModel& model : vp.models) {
            for (const std::string& res : { model.bed_texture, model.bed_model, model.thumbnail } ) {
                if (! res.empty()
                 && !fs::exists(fs::path(vendor_folder + res))
                 && !fs::exists(fs::path(rsrc_folder   + res))
                 && !fs::exists(fs::path(cache_folder  + res)))
                    return false;
            }
        }
        return true;
    }
    bool compare_vendor_profile_printers(const VendorProfile& vp_old, const VendorProfile& vp_new, std::vector<std::string>& new_printers)
    {
        for (const VendorProfile::PrinterModel& model : vp_new.models)
        {
            if (std::find_if(vp_old.models.begin(), vp_old.models.end(), [model](const VendorProfile::PrinterModel& pm) { return pm.id == model.id; }) == vp_old.models.end())
                new_printers.push_back(model.name);
        }
        return new_printers.empty();
    }
} // namespace PresetUtils

} // namespace Slic3r
