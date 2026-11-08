/*
 * REST profile catalog and isolated profile selection.
 *
 * The configured profile tree is loaded into a read-only bundle. Selection
 * requests copy the part of that bundle they can read, validate every
 * requested profile, and build the complete FFF configuration before the
 * session mutation is committed.
 * Project profile sections are passed to the core and regenerated from the
 * loaded presets rather than copied from an untrusted archive.
 */

#include "profiles.hpp"

#include <nlohmann/json.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include <algorithm>
#include <cstddef>
#include <deque>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "libslic3r/Config/ConfigOption.hpp"

namespace Slic3r::Rest {

namespace {

using Json = nlohmann::json;

// Compose project-capable session settings without copying values reserved for a profile's local owner.
DynamicPrintConfig session_configuration(const PresetBundle &bundle);
// Retain the parents named by the loaded profiles' own sections; the core remains responsible for applying metadata.
void include_project_profile_parents(PresetBundle::PresetSubset &names, const std::string &profile_sections);

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
        throw std::invalid_argument(std::string("Profile path is not a directory: ") + directory.u8string());
    collection.load_presets(
        profiles_directory.u8string(),
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
    // Prefer installed printers; built-in defaults remain a fallback only when
    // the configured root has no usable FFF printer.
    for (std::size_t index = bundle.printers.num_default_presets(); index < printers.size(); ++index) {
        const Preset &printer = printers[index];
        if (printer.is_visible && printer.printer_technology() == ptFFF) {
            bundle.printers.select_preset(index);
            return;
        }
    }
    for (std::size_t index = 0; index < bundle.printers.num_default_presets(); ++index)
        if (printers[index].printer_technology() == ptFFF) {
            bundle.printers.select_preset(index);
            return;
        }
    throw std::invalid_argument("No visible FFF printer profile is available.");
}

void append_profiles(
    Json &target,
    const PresetCollection &collection,
    const std::unordered_map<std::string, std::string> &names_to_ids,
    const PresetWithVendorProfile *printer = nullptr,
    const PresetWithVendorProfile *print = nullptr,
    const DynamicPrintConfig *extra = nullptr,
    bool compatible_only = false)
{
    // Installed visible profiles are listed; built-in defaults stay indexed so
    // the effective default selection can still be returned as an ID.
    for (const Preset &preset : collection) {
        const std::unordered_map<std::string, std::string>::const_iterator id = names_to_ids.find(preset.name);
        if (id == names_to_ids.end())
            continue;
        Json entry{{"id", id->second}, {"name", preset.name}};
        if (preset.vendor != nullptr)
            entry["vendor"] = preset.vendor->name;
        if (printer != nullptr) {
            const PresetWithVendorProfile candidate = collection.get_preset_with_vendor_profile(preset);
            // An embedded printer has no vendor/name identity. Conditions still
            // apply, but explicit name lists cannot match an anonymous preset.
            const bool anonymous_printer = printer->preset.name.empty();
            const PresetWithVendorProfile evaluated(preset, anonymous_printer ? nullptr : candidate.vendor);
            const ConfigOptionStrings *printer_names = preset.config.option<ConfigOptionStrings>("compatible_printers");
            const ConfigOptionStrings *print_names = preset.config.option<ConfigOptionStrings>("compatible_prints");
            const bool compatible =
                !(anonymous_printer && printer_names != nullptr && !printer_names->empty()) &&
                is_compatible_with_printer(evaluated, *printer, extra) &&
                (print == nullptr ||
                    (!(print->preset.name.empty() && print_names != nullptr && !print_names->empty()) &&
                     is_compatible_with_print(evaluated, *print, *printer)));
            if (compatible_only && !compatible)
                continue;
            entry["compatible"] = compatible;
        }
        target.push_back(std::move(entry));
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
    application.configuration = session_configuration(bundle);
    application.profile_sections = bundle.project_profile_sections();
    return application;
}

DynamicPrintConfig session_configuration(const PresetBundle &bundle)
{
    DynamicPrintConfig configuration = bundle.full_config();
    for (const std::string &key : configuration.keys())
        if (!configuration.writes_key(key, ConfigWriteTarget::Project))
            configuration.erase(key);
    return configuration;
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

// True when the edited copy of the selected preset holds a value that changes
// the configuration. The engine's dirty flag is not used directly: loading a
// project also recomputes the phony width/spacing pair and the print_version
// stamp, which the flag reports as modifications although they change nothing.
bool edited_preset_modified(const PresetCollection &collection)
{
    const Preset &edited = collection.get_edited_preset();
    const Preset &selected = collection.get_selected_preset();
    for (const auto &[key, flags] : PresetCollection::dirty_options(&edited, &selected, true)) {
        if (key.key == "print_version")
            continue;
        const ConfigOption *edited_option = edited.config.option(key.key);
        const ConfigOption *selected_option = selected.config.option(key.key);
        if ((edited_option != nullptr && edited_option->is_phony()) ||
            (selected_option != nullptr && selected_option->is_phony()))
            continue;
        return true;
    }
    return false;
}

// Catalog ID of a preset selected after a project load, or "" when the
// project does not use the installed preset unchanged. The engine reports a
// difference in two ways: a new external preset (embedded profile, or a
// modified filament slot), or the installed preset selected with a modified
// edited copy. The edited copy is identified by name, so when two filament
// slots use the same installed preset and only one is modified, both report
// "": an unchanged slot may be shown as modified, never the reverse.
std::string installed_profile_id(
    const PresetCollection &collection,
    const std::string &name,
    const std::unordered_map<std::string, std::string> &names_to_ids)
{
    const Preset *preset = collection.find_preset(name, false, false);
    if (preset == nullptr || preset->is_external)
        return {};
    if (collection.get_edited_preset().name == name && edited_preset_modified(collection))
        return {};
    const std::unordered_map<std::string, std::string>::const_iterator profile = names_to_ids.find(name);
    return profile == names_to_ids.end() ? std::string() : profile->second;
}

} // namespace

ProfileCatalog::ProfileCatalog(const std::filesystem::path &profiles_directory)
{
    PresetsConfigSubstitutions substitutions;
    const std::filesystem::path vendor_directory = profiles_directory / "vendor";
    if (std::filesystem::exists(vendor_directory)) {
        try {
            // Match the GUI's zero-preset return for otherwise valid vendors
            // lacking models/variants. Diagnose the skipped file at startup.
            for (const std::filesystem::directory_entry &file : std::filesystem::directory_iterator(vendor_directory)) {
                if (!file.is_regular_file() || !boost::algorithm::iequals(file.path().extension().u8string(), ".ini"))
                    continue;
                try {
                    const VendorProfile vendor = VendorProfile::from_ini(boost::filesystem::path(file.path().u8string()));
                    if (!vendor.valid())
                        throw std::invalid_argument("invalid vendor metadata");
                    if (!vendor.templates_profile && (vendor.models.empty() || vendor.num_variants() == 0))
                        warnings_.push_back("profiles/vendor/" + file.path().filename().u8string() +
                            ": ignored, no printer model or template metadata");
                } catch (const std::exception &error) {
                    throw std::invalid_argument(file.path().u8string() + ": " + error.what());
                }
            }
            // The GUI accumulates load errors to report them after all vendors.
            // load_system_presets_from emits one "Vendor configuration file "
            // line per duplicate set; REST accepts only those nonfatal diagnostics.
            const std::pair<PresetsConfigSubstitutions, std::string> loaded = bundle_.load_system_presets_from(
                vendor_directory.u8string(), ForwardCompatibilitySubstitutionRule::EnableSilent);
            std::istringstream errors(loaded.second);
            std::string line;
            while (std::getline(errors, line)) {
                if (line.empty())
                    continue;
                if (line.rfind("Vendor configuration file ", 0) != 0)
                    throw std::invalid_argument(loaded.second);
                warnings_.push_back(line);
            }
            // System visibility here does not depend on GUI installation choices.
            PresetCollection *collections[] = {&bundle_.fff_prints, &bundle_.filaments, &bundle_.printers};
            for (PresetCollection *collection : collections)
                for (const Preset &preset : collection->get_presets())
                    if (preset.is_system)
                        collection->find_preset(preset.name, false)->is_visible = true;
        } catch (const std::exception &error) {
            throw std::invalid_argument(std::string("profiles/vendor: ") + error.what());
        }
    }
    load_collection(bundle_.fff_prints, profiles_directory, "print", substitutions);
    load_collection(bundle_.filaments, profiles_directory, "filament", substitutions);
    load_collection(bundle_.printers, profiles_directory, "printer", substitutions);
    select_first_visible_fff_printer(bundle_);
    if (!bundle_.vendors.empty()) {
        // Unlike loading a GUI AppConfig, a vendor-only tree has no saved print
        // selection. Choose a compatible installed print before seeding sessions.
        const Preset &printer = bundle_.printers.get_selected_preset();
        const PresetWithVendorProfile printer_vendor = bundle_.printers.get_preset_with_vendor_profile(printer);
        const DynamicPrintConfig extra = compatibility_context(printer);
        const ConfigOptionString *preferred = printer.config.option<ConfigOptionString>("default_print_profile");
        const Preset *selected = preferred == nullptr ? nullptr : bundle_.fff_prints.find_preset(preferred->value, false, false);
        if (selected == nullptr || !is_compatible_with_printer(bundle_.fff_prints.get_preset_with_vendor_profile(*selected), printer_vendor, &extra)) {
            selected = nullptr;
            for (const Preset &candidate : bundle_.fff_prints)
                if (candidate.is_visible && is_compatible_with_printer(bundle_.fff_prints.get_preset_with_vendor_profile(candidate), printer_vendor, &extra)) {
                    selected = &candidate;
                    break;
                }
        }
        if (selected != nullptr)
            bundle_.fff_prints.select_preset_by_name(selected->name, true);
    }
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
    return list_profiles({&bundle_.printers.get_selected_preset(), &bundle_.fff_prints.get_selected_preset()}, false);
}

const Preset *ProfileCatalog::printer_by_id(const std::string &id) const
{
    const std::unordered_map<std::string, std::string>::const_iterator found = printer_profile_names_.find(id);
    return found == printer_profile_names_.end() ? nullptr : bundle_.printers.find_preset(found->second, false, false);
}

const Preset *ProfileCatalog::print_by_id(const std::string &id) const
{
    const std::unordered_map<std::string, std::string>::const_iterator found = print_profile_names_.find(id);
    return found == print_profile_names_.end() ? nullptr : bundle_.fff_prints.find_preset(found->second, false, false);
}

nlohmann::json ProfileCatalog::list_profiles(const CompatibilityContext &context, bool compatible_only) const
{
    const Preset &printer = context.printer == nullptr ? bundle_.printers.get_selected_preset() : *context.printer;
    const Preset &print = context.print == nullptr ? bundle_.fff_prints.get_selected_preset() : *context.print;
    // Edited session copies keep their catalog identity, including the vendor
    // inherited by a user preset. Conditions read the copied live configuration.
    const Preset *printer_identity = bundle_.printers.find_preset(printer.name, false, false);
    const Preset *print_identity = bundle_.fff_prints.find_preset(print.name, false, false);
    const PresetWithVendorProfile printer_origin = bundle_.printers.get_preset_with_vendor_profile(
        printer_identity == nullptr ? printer : *printer_identity);
    const PresetWithVendorProfile print_origin = bundle_.fff_prints.get_preset_with_vendor_profile(
        print_identity == nullptr ? print : *print_identity);
    const PresetWithVendorProfile printer_vendor(printer, printer_origin.vendor);
    const PresetWithVendorProfile print_vendor(print, print_origin.vendor);
    const DynamicPrintConfig extra = compatibility_context(printer);
    Json result{
        {"print", Json::array()},
        {"printer", Json::array()},
        {"extruder", Json::array()},
        {"filaments", Json::array()}
    };
    append_profiles(result["print"], bundle_.fff_prints, print_profile_ids_, &printer_vendor, nullptr, &extra, compatible_only);
    append_profiles(result["printer"], bundle_.printers, printer_profile_ids_);
    append_profiles(result["filaments"], bundle_.filaments, filament_profile_ids_, &printer_vendor, &print_vendor, &extra, compatible_only);
    return result;
}

PresetBundle ProfileCatalog::working_bundle(const PresetBundle::PresetSubset &names) const
{
    return full_bundle_copies_ ? bundle_ : bundle_.subset(names);
}

PresetBundle::PresetSubset ProfileCatalog::selection_names(const ProfileSelection &requested) const
{
    // Unknown or unusable names are left out: apply_selection() rejects them
    // as it would on the full catalog.
    PresetBundle::PresetSubset names;
    names.printers = {requested.printer};
    names.prints = {requested.print};
    names.filaments.insert(requested.filaments.begin(), requested.filaments.end());
    const Preset *printer = bundle_.printers.find_preset(requested.printer, false, false);
    if (printer != nullptr && printer->printer_technology() == ptFFF) {
        // Also with explicit filaments: a requested template filament is
        // incompatible when a compatible system filament has its alias.
        const std::set<std::string> compatible = bundle_.compatible_filament_names(
            *printer, bundle_.fff_prints.find_preset(requested.print, false, false));
        names.filaments.insert(compatible.begin(), compatible.end());
    }
    return names;
}

ProfileApplication ProfileCatalog::defaults() const
{
    ProfileSelection requested;
    requested.print = bundle_.fff_prints.get_selected_preset_name();
    requested.printer = bundle_.printers.get_selected_preset_name();
    ProfileApplication application = apply_selection(working_bundle(selection_names(requested)), requested);
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

    ProfileApplication application = apply_selection(working_bundle(selection_names(resolved)), resolved);
    application.selection.print = profile_id_for_name(print_profile_ids_, application.selection.print, "print");
    application.selection.printer = profile_id_for_name(printer_profile_ids_, application.selection.printer, "printer");
    for (std::string &filament : application.selection.filaments)
        filament = profile_id_for_name(filament_profile_ids_, filament, "filament");
    return application;
}

ProfileApplication ProfileCatalog::load_project(DynamicPrintConfig project, const std::string &project_name,
    const std::string &profile_sections) const
{
    // The project names its presets and their parents; loading compares the
    // embedded values only with these presets (renamed names included, see
    // PresetBundle::subset). A surplus is harmless, so parents go to all sets.
    PresetBundle::PresetSubset names;
    if (const ConfigOptionString *print = project.option<ConfigOptionString>("print_settings_id"))
        names.prints.insert(print->value);
    if (const ConfigOptionString *printer = project.option<ConfigOptionString>("printer_settings_id"))
        names.printers.insert(printer->value);
    if (const ConfigOptionStrings *filaments = project.option<ConfigOptionStrings>("filament_settings_id"))
        names.filaments.insert(filaments->get_values().begin(), filaments->get_values().end());
    if (const ConfigOptionStrings *parents = project.option<ConfigOptionStrings>("inherits_cummulative"))
        for (const std::string &parent : parents->get_values())
            for (std::set<std::string> *category : {&names.prints, &names.printers, &names.filaments})
                category->insert(parent);
    include_project_profile_parents(names, profile_sections);
    // A preset embedded in the project gets a generated name; loading skips
    // the generated names that installed presets already use (or reuses an
    // identical one), so these presets are read too. A missing name, as for
    // the filament slots the project lacks, is an empty original name.
    const std::string file_name = boost::filesystem::path(project_name).filename().string();
    const std::pair<const PresetCollection *, std::set<std::string> *> categories[] = {
        {&bundle_.fff_prints, &names.prints}, {&bundle_.printers, &names.printers}, {&bundle_.filaments, &names.filaments}};
    for (const auto &[collection, category] : categories) {
        std::set<std::string> originals = *category;
        originals.insert(std::string());
        for (const std::string &original : originals)
            for (size_t index = 0;; ++index) {
                const std::string generated = external_preset_name(file_name, original, index);
                if (collection->find_preset(generated, false, false) == nullptr)
                    break;
                category->insert(generated);
            }
    }

    PresetBundle bundle = working_bundle(names);
    bundle.load_config_model(project_name, std::move(project), profile_sections);

    ProfileApplication application;
    application.selection.print = installed_profile_id(
        bundle.fff_prints, bundle.fff_prints.get_selected_preset_name(), print_profile_ids_);
    application.selection.printer = installed_profile_id(
        bundle.printers, bundle.printers.get_selected_preset_name(), printer_profile_ids_);
    for (const ExtruderFilaments &slot : bundle.extruders_filaments)
        application.selection.filaments.push_back(installed_profile_id(
            bundle.filaments, slot.get_selected_preset_name(), filament_profile_ids_));
    application.configuration = session_configuration(bundle);
    application.profile_sections = bundle.project_profile_sections();
    return application;
}

namespace {

void include_project_profile_parents(PresetBundle::PresetSubset &names, const std::string &profile_sections)
{
    if (profile_sections.empty())
        return;
    boost::property_tree::ptree parsed;
    try {
        std::istringstream input(profile_sections);
        boost::property_tree::read_ini(input, parsed);
    } catch (const boost::property_tree::ini_parser::ini_parser_error &) {
        // Invalid sections bring no metadata, as in the core's project loader.
        return;
    }
    const boost::property_tree::ptree &sections = parsed;
    const std::pair<const char *, std::set<std::string> *> categories[] = {
        {"print:", &names.prints}, {"printer:", &names.printers}, {"filament:", &names.filaments}};
    for (const std::pair<const char *, std::set<std::string> *> &category : categories) {
        const std::set<std::string> originals = *category.second;
        for (const std::string &name : originals) {
            const boost::property_tree::ptree::const_assoc_iterator section = sections.find(category.first + name);
            if (section == sections.not_found())
                continue;
            const boost::property_tree::ptree::const_assoc_iterator entry = section->second.find("inherits");
            if (entry == section->second.not_found())
                continue;
            ConfigOptionString parent;
            if (parent.deserialize(entry->second.data()) && !parent.value.empty())
                category.second->insert(parent.value);
        }
    }
}

} // namespace

} // namespace Slic3r::Rest
