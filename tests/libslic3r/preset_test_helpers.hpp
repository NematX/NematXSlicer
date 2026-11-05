///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_preset_test_helpers_hpp_
#define slic3r_preset_test_helpers_hpp_

#include <filesystem>
#include <random>
#include <stdexcept>
#include <string>

/* TemporaryDirectory provides an isolated directory for profile-loading tests.
   It creates a new directory, never reuses an existing one, and removes its tree
   on destruction. A cleanup error does not replace a test failure with an exception.
   References returned by path() remain valid only while the directory object is alive.

   Typical use:
     PresetTest::TemporaryDirectory directory;
     const auto file = directory.path() / "profile.ini";
   Write the test's files under that path before loading profiles. */

namespace PresetTest {

class TemporaryDirectory
{
public:
    // Create a fresh directory without deleting anything if its random name already exists.
    TemporaryDirectory()
    {
        std::random_device random;
        path_ = std::filesystem::temp_directory_path() /
            ("superslicer_presets_" + std::to_string(random()) + "_" + std::to_string(random()));
        if (!std::filesystem::create_directory(path_))
            throw std::runtime_error("Cannot create a fresh preset test directory");
    }
    // Remove this fixture's files without masking a test failure with a cleanup exception.
    ~TemporaryDirectory()
    {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }
    TemporaryDirectory(const TemporaryDirectory &) = delete;
    TemporaryDirectory &operator=(const TemporaryDirectory &) = delete;
    // Borrow the fixture root until this directory is destroyed.
    const std::filesystem::path &path() const { return path_; }

private:
    std::filesystem::path path_;
};

} // namespace PresetTest

#endif
