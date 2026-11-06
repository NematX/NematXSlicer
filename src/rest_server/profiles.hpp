/*
 * REST profile catalog and isolated profile selection.
 *
 * PresetBundle is loaded once from the configured profile directory. Requests
 * select profiles on a private copy so one session cannot mutate another
 * session's effective configuration or the shared catalog.
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

class ProfileCatalog {
public:
    explicit ProfileCatalog(const std::filesystem::path &profiles_directory);

    nlohmann::json list_profiles() const;
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
    PresetBundle bundle_;
    // IDs are generated per catalog and resolve only within this server instance.
    std::unordered_map<std::string, std::string> print_profile_ids_;
    std::unordered_map<std::string, std::string> print_profile_names_;
    std::unordered_map<std::string, std::string> printer_profile_ids_;
    std::unordered_map<std::string, std::string> printer_profile_names_;
    std::unordered_map<std::string, std::string> filament_profile_ids_;
    std::unordered_map<std::string, std::string> filament_profile_names_;
};

} // namespace Slic3r::Rest
