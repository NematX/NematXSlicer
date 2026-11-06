/*
 * REST profile catalog and isolated profile selection.
 *
 * PresetBundle is loaded once from the configured profile directory. Requests
 * select profiles on a private copy so one session cannot mutate another
 * session's effective configuration or the shared catalog.
 */

#pragma once

#include <filesystem>
#include <unordered_map>

#include <nlohmann/json_fwd.hpp>

#include "libslic3r/ContainerUtils.hpp"
#include "libslic3r/Config/PresetBundle.hpp"
#include "session.hpp"

namespace Slic3r::Rest {

struct ProfileApplication {
    ProfileSelection selection;
    DynamicPrintConfig configuration;
};

class ProfileCatalog {
public:
    explicit ProfileCatalog(const std::filesystem::path &profiles_directory);

    nlohmann::json list_profiles() const;
    ProfileApplication defaults() const;
    ProfileApplication select(const ProfileSelection &requested) const;

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
