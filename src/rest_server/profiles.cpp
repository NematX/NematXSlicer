/*
 * REST profile catalog and isolated profile selection.
 *
 * The configured profile tree is loaded into a read-only bundle. Selection
 * requests copy that bundle, validate every requested profile, and build the
 * complete FFF configuration before the session mutation is committed.
 */

#include "profiles.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstddef>
#include <deque>
#include <stdexcept>
#include <string>
#include <utility>

#include "libslic3r/Config/ConfigOption.hpp"

namespace Slic3r::Rest {

namespace {

using Json = nlohmann::json;

void load_collection(
    PresetCollection &collection,
    const std::filesystem::path &profiles_directory,
    const char *subdirectory,
    PresetsConfigSubstitutions &substitutions)
{
    // Profile categories are optional so an empty root still has the built-in
    // PresetBundle defaults; PresetCollection requires existing directories.
    const std::filesystem::path directory = profiles_directory / subdirectory;
    std::error_code error;
    if (!std::filesystem::exists(directory, error)) {
        if (error)
            throw std::invalid_argument("Cannot inspect profile directory: " + error.message());
        return;
    }
    if (!std::filesystem::is_directory(directory, error) || error)
        throw std::invalid_argument(std::string("Profile path is not a directory: ") + directory.string());
    collection.load_presets(
        profiles_directory.string(),
        subdirectory,
        substitutions,
        ForwardCompatibilitySubstitutionRule::EnableSilent);
}

void register_fff_profiles(
    const PresetCollection &collection,
    std::size_t &next_id,
    std::unordered_map<std::string, std::string> &names_to_ids,
    std::unordered_map<std::string, std::string> &ids_to_names)
{
    for (const Preset &preset : collection()) {
        if (!preset.is_visible)
            continue;
        if (preset.printer_technology() != ptFFF)
            continue;
        const std::string id = "profile_" + std::to_string(next_id++);
        names_to_ids.emplace(preset.name, id);
        ids_to_names.emplace(id, preset.name);
    }
}

void select_first_visible_fff_printer(PresetBundle &bundle)
{
    // The engine's first visible printer is not guaranteed to be FFF.
    const std::deque<Preset> &printers = bundle.printers.get_presets();
    for (std::size_t index = 0; index < printers.size(); ++index) {
        const Preset &printer = printers[index];
        if (printer.is_visible && printer.printer_technology() == ptFFF) {
            bundle.printers.select_preset(index);
            return;
        }
    }
    throw std::invalid_argument("No visible FFF printer profile is available.");
}

void append_profiles(
    Json &target,
    const PresetCollection &collection,
    const std::unordered_map<std::string, std::string> &names_to_ids)
{
    // Installed visible profiles are listed; built-in defaults stay indexed so
    // the effective default selection can still be returned as an ID.
    for (const Preset &preset : collection) {
        const std::unordered_map<std::string, std::string>::const_iterator id = names_to_ids.find(preset.name);
        if (id == names_to_ids.end())
            continue;
        target.push_back({
            {"id", id->second},
            {"name", preset.alias.empty() ? preset.name : preset.alias}
        });
    }
}

const Preset *find_visible_preset(const PresetCollection &collection, const std::string &name)
{
    // A profile request must never fall back to the first visible preset.
    const Preset *preset = collection.find_preset(name, false, false);
    if (preset == nullptr || !preset->is_visible)
        return nullptr;
    return preset;
}

std::size_t extruder_count(const Preset &printer)
{
    // The printer's nozzle vector is the authoritative V1 extruder count.
    const ConfigOption *option = printer.config.option("nozzle_diameter");
    const ConfigOptionFloats *nozzle_diameter = dynamic_cast<const ConfigOptionFloats *>(option);
    if (nozzle_diameter == nullptr || nozzle_diameter->size() == 0)
        throw std::invalid_argument("The selected printer has no extruder definition.");
    return nozzle_diameter->size();
}

ProfileSelection effective_selection(const PresetBundle &bundle)
{
    // Read the effective names only after all compatibility and default logic ran.
    ProfileSelection selection;
    selection.print = bundle.fff_prints.get_selected_preset_name();
    selection.printer = bundle.printers.get_selected_preset_name();
    for (const ExtruderFilaments &filaments : bundle.extruders_filaments)
        selection.filaments.push_back(filaments.get_selected_preset_name());
    return selection;
}

bool select_filament_if_usable(ExtruderFilaments &slot, const std::string &name, bool require_compatible)
{
    for (std::size_t index = 0; index < slot().size(); ++index) {
        const Filament &filament = slot.filament(index);
        if (filament.preset->name == name && filament.preset->is_visible &&
            (!require_compatible || filament.is_compatible)) {
            slot.select_filament(index);
            return true;
        }
    }
    return false;
}

void select_filaments(PresetBundle &bundle, const std::vector<std::string> &filaments)
{
    for (std::size_t index = 0; index < filaments.size(); ++index)
        if (!select_filament_if_usable(bundle.extruders_filaments[index], filaments[index], true))
            throw std::invalid_argument("A selected filament profile is unavailable or incompatible.");
}

void select_default_filaments(PresetBundle &bundle)
{
    const ConfigOptionStrings *defaults =
        bundle.printers.get_edited_preset().config.option<ConfigOptionStrings>("default_filament_profile");
    const std::vector<std::string> preferred = defaults == nullptr ? std::vector<std::string>() : defaults->get_values();

    for (std::size_t index = 0; index < bundle.extruders_filaments.size(); ++index) {
        ExtruderFilaments &slot = bundle.extruders_filaments[index];
        bool selected = false;
        if (index < preferred.size() && !preferred[index].empty())
            if (const Preset *filament = find_visible_preset(bundle.filaments, preferred[index]); filament != nullptr)
                selected = select_filament_if_usable(slot, filament->name, true);

        for (const std::string &name : preferred) {
            if (selected)
                break;
            if (name.empty())
                continue;
            if (const Preset *filament = find_visible_preset(bundle.filaments, name); filament != nullptr)
                selected = select_filament_if_usable(slot, filament->name, true);
        }

        if (!selected) {
            const std::size_t first_profile = bundle.filaments.is_default_suppressed()
                ? bundle.filaments.num_default_presets()
                : 0;
            for (std::size_t candidate = first_profile; candidate < slot().size(); ++candidate) {
                const Filament &filament = slot.filament(candidate);
                if (filament.preset->is_visible && filament.is_compatible) {
                    slot.select_filament(candidate);
                    selected = true;
                    break;
                }
            }
            for (std::size_t candidate = 0; !selected && candidate < first_profile && candidate < slot().size(); ++candidate) {
                const Filament &filament = slot.filament(candidate);
                if (filament.preset->is_visible && filament.is_compatible) {
                    slot.select_filament(candidate);
                    selected = true;
                }
            }
        }

        if (!selected)
            throw std::invalid_argument("No compatible filament profile is available for the printer.");
    }
}

ProfileApplication apply_selection(PresetBundle bundle, const ProfileSelection &requested)
{
    // Validate the complete request on a private bundle before changing session state.
    const Preset *print = find_visible_preset(bundle.fff_prints, requested.print);
    const Preset *printer = find_visible_preset(bundle.printers, requested.printer);
    if (print == nullptr || printer == nullptr || printer->printer_technology() != ptFFF)
        throw std::invalid_argument("The selected print or printer profile is unavailable.");
    if (!requested.extruder.empty())
        throw std::invalid_argument("Independent extruder profiles are not available.");

    const std::size_t count = extruder_count(*printer);
    if (!requested.filaments.empty() && requested.filaments.size() != count)
        throw std::invalid_argument("The filament profile count does not match the printer.");
    for (const std::string &filament_name : requested.filaments) {
        const Preset *filament = find_visible_preset(bundle.filaments, filament_name);
        if (filament == nullptr)
            throw std::invalid_argument("The selected filament profile is unavailable.");
    }

    if (!bundle.printers.select_preset_by_name(requested.printer, true) ||
        !bundle.fff_prints.select_preset_by_name(requested.print, true))
        throw std::invalid_argument("The selected print or printer profile is unavailable.");
    bundle.extruders_filaments.clear();
    bundle.update_multi_material_filament_presets();
    if (bundle.extruders_filaments.size() != count)
        throw std::invalid_argument("The selected printer has an invalid extruder definition.");
    bundle.update_compatible(PresetSelectCompatibleType::Never);
    if (requested.filaments.empty())
        select_default_filaments(bundle);
    else
        select_filaments(bundle, requested.filaments);
    bundle.update_compatible(PresetSelectCompatibleType::Never);

    for (const ExtruderFilaments &filaments : bundle.extruders_filaments) {
        const Filament *selected = filaments.get_selected_filament();
        if (selected == nullptr || !selected->is_compatible)
            throw std::invalid_argument("A selected filament profile is incompatible with the printer.");
    }
    if (!bundle.fff_prints.get_edited_preset().is_compatible)
        throw std::invalid_argument("The selected print profile is incompatible with the printer.");

    ProfileApplication application;
    application.selection = effective_selection(bundle);
    application.configuration = bundle.full_config_secure();
    return application;
}

std::string resolve_profile_id(
    const std::unordered_map<std::string, std::string> &ids_to_names,
    const std::string &id,
    const char *category)
{
    const std::unordered_map<std::string, std::string>::const_iterator profile = ids_to_names.find(id);
    if (profile == ids_to_names.end())
        throw std::invalid_argument(std::string("The selected ") + category + " profile is unavailable.");
    return profile->second;
}

std::string profile_id_for_name(
    const std::unordered_map<std::string, std::string> &names_to_ids,
    const std::string &name,
    const char *category)
{
    const std::unordered_map<std::string, std::string>::const_iterator profile = names_to_ids.find(name);
    if (profile == names_to_ids.end())
        throw std::invalid_argument(std::string("The effective ") + category + " profile is unavailable.");
    return profile->second;
}

} // namespace

ProfileCatalog::ProfileCatalog(const std::filesystem::path &profiles_directory)
{
    PresetsConfigSubstitutions substitutions;
    load_collection(bundle_.fff_prints, profiles_directory, "print", substitutions);
    load_collection(bundle_.filaments, profiles_directory, "filament", substitutions);
    load_collection(bundle_.printers, profiles_directory, "printer", substitutions);
    select_first_visible_fff_printer(bundle_);
    bundle_.extruders_filaments.clear();
    bundle_.update_multi_material_filament_presets();
    bundle_.update_compatible(PresetSelectCompatibleType::Never);

    std::size_t next_id = 1;
    register_fff_profiles(bundle_.fff_prints, next_id, print_profile_ids_, print_profile_names_);
    register_fff_profiles(bundle_.printers, next_id, printer_profile_ids_, printer_profile_names_);
    register_fff_profiles(bundle_.filaments, next_id, filament_profile_ids_, filament_profile_names_);
}

nlohmann::json ProfileCatalog::list_profiles() const
{
    Json result{
        {"print", Json::array()},
        {"printer", Json::array()},
        {"extruder", Json::array()},
        {"filaments", Json::array()}
    };
    append_profiles(result["print"], bundle_.fff_prints, print_profile_ids_);
    append_profiles(result["printer"], bundle_.printers, printer_profile_ids_);
    append_profiles(result["filaments"], bundle_.filaments, filament_profile_ids_);
    return result;
}

ProfileApplication ProfileCatalog::defaults() const
{
    ProfileSelection requested;
    requested.print = bundle_.fff_prints.get_selected_preset_name();
    requested.printer = bundle_.printers.get_selected_preset_name();
    ProfileApplication application = apply_selection(bundle_, requested);
    application.selection.print = profile_id_for_name(print_profile_ids_, application.selection.print, "print");
    application.selection.printer = profile_id_for_name(printer_profile_ids_, application.selection.printer, "printer");
    for (std::string &filament : application.selection.filaments)
        filament = profile_id_for_name(filament_profile_ids_, filament, "filament");
    return application;
}

ProfileApplication ProfileCatalog::select(const ProfileSelection &requested) const
{
    ProfileSelection resolved = requested;
    resolved.print = resolve_profile_id(print_profile_names_, requested.print, "print");
    resolved.printer = resolve_profile_id(printer_profile_names_, requested.printer, "printer");
    for (std::string &filament : resolved.filaments)
        filament = resolve_profile_id(filament_profile_names_, filament, "filament");

    ProfileApplication application = apply_selection(bundle_, resolved);
    application.selection.print = profile_id_for_name(print_profile_ids_, application.selection.print, "print");
    application.selection.printer = profile_id_for_name(printer_profile_ids_, application.selection.printer, "printer");
    for (std::string &filament : application.selection.filaments)
        filament = profile_id_for_name(filament_profile_ids_, filament, "filament");
    return application;
}

} // namespace Slic3r::Rest
