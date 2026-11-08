/*
 * REST profile catalog and isolated profile selection.
 *
 * PresetBundle is loaded once from the configured profile directory. Requests
 * select profiles on a private reduced copy so one session cannot mutate
 * another session's effective configuration or the shared catalog.
 * Applications return the core-generated profile sections beside the flat
 * settings so callers can preserve them in project snapshots.
 */

#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>

#include <nlohmann/json_fwd.hpp>

#include "libslic3r/ContainerUtils.hpp"
#include "libslic3r/Config/PresetBundle.hpp"
#include "session.hpp"

namespace Slic3r::Rest {

struct ProfileApplication {
    ProfileSelection selection;
    DynamicPrintConfig configuration;
    std::string profile_sections;
};

// Preset's named constructor is protected. This temporary preset carries a
// session snapshot without inventing a catalog name or vendor identity.
struct SessionCompatibilityPreset : Preset {
    SessionCompatibilityPreset(Type category, const DynamicPrintConfig &configuration)
        : Preset(category, "")
    {
        config = configuration;
    }
};

class ProfileCatalog {
public:
    explicit ProfileCatalog(const std::filesystem::path &profiles_directory);

    nlohmann::json list_profiles() const;
    struct CompatibilityContext {
        // Embedded or modified session profiles supply their configuration in
        // temporary presets when there is no catalog ID to resolve.
        const Preset *printer = nullptr;
        const Preset *print = nullptr;
    };
    // Read-only compatibility checks; filament conditions see extruder zero.
    nlohmann::json list_profiles(const CompatibilityContext &context, bool compatible_only) const;
    const Preset *printer_by_id(const std::string &id) const;
    const Preset *print_by_id(const std::string &id) const;
    const std::vector<std::string> &warnings() const { return warnings_; }
    ProfileApplication defaults() const;
    ProfileApplication select(const ProfileSelection &requested) const;
    // Split a complete project configuration into print/printer/filament
    // presets on a private bundle copy, as the GUI does for a 3MF project, and
    // return the recomposed configuration. A profile of the selection keeps
    // its catalog ID only when the project uses the installed profile
    // unchanged; an embedded or modified profile is reported as "". The core applies
    // profile_sections to each matching profile and returns filtered sections under its loaded name.
    ProfileApplication load_project(DynamicPrintConfig project, const std::string &project_name,
        const std::string &profile_sections = std::string()) const;

private:
    // Tests compare reduced bundles with full copies through this accessor.
    friend struct ProfileCatalogTestAccess;

    // Private bundle for one request: a reduced copy of the catalog holding
    // `names`, which is much cheaper than a full copy for large catalogs.
    PresetBundle working_bundle(const PresetBundle::PresetSubset &names) const;
    // Names a selection of `printer` and `print` may read: both presets and
    // every filament compatible with one of the printer's extruders, which
    // default selection and template filtering compare.
    PresetBundle::PresetSubset selection_names(const ProfileSelection &requested) const;

    PresetBundle bundle_;
    // Test oracle only: work on full catalog copies, as before reduced bundles.
    bool full_bundle_copies_ = false;
    std::vector<std::string> warnings_;
    // IDs are generated per catalog and resolve only within this server instance.
    std::unordered_map<std::string, std::string> print_profile_ids_;
    std::unordered_map<std::string, std::string> print_profile_names_;
    std::unordered_map<std::string, std::string> printer_profile_ids_;
    std::unordered_map<std::string, std::string> printer_profile_names_;
    std::unordered_map<std::string, std::string> filament_profile_ids_;
    std::unordered_map<std::string, std::string> filament_profile_names_;
};

} // namespace Slic3r::Rest
