// Flatten a read-only system bundle using the same inheritance loader as the GUI.
// Only concrete FFF presets are exported; phase 34 still serves flat user INIs.
#pragma once
#include "libslic3r/Config/PresetBundle.hpp"
#include <filesystem>

namespace RestServerTest {
inline void flatten_basic_profiles(const std::filesystem::path &source, const std::filesystem::path &destination)
{
    Slic3r::PresetBundle bundle;
    bundle.load_configbundle(source.u8string(), Slic3r::PresetBundle::LoadSystem,
        Slic3r::ForwardCompatibilitySubstitutionRule::Disable);
    const std::pair<const char *, Slic3r::PresetCollection *> collections[] = {
        {"print", &bundle.fff_prints}, {"printer", &bundle.printers}, {"filament", &bundle.filaments}};
    for (const std::pair<const char *, Slic3r::PresetCollection *> &category : collections) {
        const std::filesystem::path directory = destination / category.first;
        std::filesystem::create_directories(directory);
        for (const Slic3r::Preset &preset : category.second->get_presets()) {
            if (preset.is_default || !preset.is_system || preset.name.front() == '*')
                continue;
            Slic3r::DynamicPrintConfig config = preset.config;
            config.erase("inherits");
            config.save((directory / std::filesystem::u8path(preset.name + ".ini")).u8string());
        }
    }
}
} // namespace RestServerTest
