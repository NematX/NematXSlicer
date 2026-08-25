// Configuration inventory uses isolated directories; preference checks use in-memory AppConfig methods without the GUI installation chooser.
#include <catch_main.hpp>
#include "libslic3r/Config/AppConfig.hpp"
#include "libslic3r/Semver.hpp"
#include "libslic3r_version.h"
#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>
#include <chrono>
#include <cstdlib>
#include <iterator>
#include <limits>
#include <stdexcept>

namespace {
namespace fs = boost::filesystem;

struct ConfigGlobals {
    const std::string data = Slic3r::has_data_dir() ? Slic3r::data_dir() : std::string{};
    const std::string resources = Slic3r::resources_dir();
    const fs::path install = Slic3r::install_path();

    // Restore borrowed process path settings and the timestamp-disabled Catch baseline; no files are written or removed.
    ~ConfigGlobals()
    {
        Slic3r::set_data_dir(data);
        Slic3r::set_resources_dir(resources);
        Slic3r::set_install_path(install);
        Slic3r::set_header_generate_with_date(false);
    }
};

// Write exact bytes beneath the caller's test directory; owns and closes the stream, throwing on open or write failure.
void write_file(const fs::path &path, const std::string &bytes)
{
    boost::nowide::ofstream out(path.string(), std::ios::binary);
    out.exceptions(std::ios::badbit | std::ios::failbit);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    out.close();
}

// Read an existing test file as exact bytes; owns and closes its stream and throws if opening or reading fails.
std::string read_file(const fs::path &path)
{
    boost::nowide::ifstream in(path.string(), std::ios::binary);
    if (!in) throw std::runtime_error("Cannot open test file: " + path.string());
    std::string bytes{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    if (in.bad()) throw std::runtime_error("Cannot read test file: " + path.string());
    return bytes;
}

// Compare normalized path spellings without requiring existence or accessing directories outside the test tree.
bool same_path(fs::path first, fs::path second)
{
    return first.lexically_normal().make_preferred() == second.lexically_normal().make_preferred();
}
}

TEST_CASE("Copied prerelease versions remain comparable", "[Version]")
{
    using Slic3r::Semver;
    const Semver copied_version = [] {
        const Semver source_version("2.7.63.4-beta-2026-08-19+build.19");
        return Semver(source_version);
    }();

    REQUIRE(copied_version == *Semver::parse("2.7.63.4-beta-2026-08-19"));
    REQUIRE(copied_version < *Semver::parse("2.7.63.4"));
    REQUIRE(copied_version.to_string() == "2.7.63.4-beta-2026-08-19+build.19");
}

TEST_CASE("Font size defaults preserve historical integer parsing", "[AppConfig][font_size]")
{
    struct HeaderDateRestorer {
        // Restore the runner's timestamp-disabled baseline on scope exit; no files or paths are changed.
        ~HeaderDateRestorer() { Slic3r::set_header_generate_with_date(false); }
    } restore;

    // Construction may discover FreeCAD read-only; a sentinel prevents repeated discovery during these defaults checks.
    Slic3r::AppConfig config(Slic3r::AppConfig::EAppMode::Editor);
    config.set("freecad_path", "font-size-test-unused");
    REQUIRE(config.get("font_size") == "0");
    config.set("font_pt_size", "-3");

    config.erase("", "font_size");
    REQUIRE(config.get("font_size").empty());
    config.set_defaults();
    CHECK(config.get("font_size") == "0");
    CHECK(config.get("font_pt_size") == "-3");

    for (const std::string value : {"", "0", "-1", "-12", "abc", "14", "14px"}) {
        CAPTURE(value);
        config.set("font_size", value);
        config.set_defaults();
        CHECK(config.get("font_size") == ((value == "14" || value == "14px") ? value : "0"));
        CHECK(config.get("font_pt_size") == "-3");
    }

    // A later preference write can be negative; this checks the callback's input, not GUI callback execution.
    config.set("font_size", "-12");
    CHECK(config.get_int("font_size") == -12);
}

TEST_CASE("NematX configuration roots preserve existing data", "[host_identity][AppConfig]")
{
    const char *output = std::getenv("HOST_CONFIG_TEST_ROOT");
    REQUIRE(output != nullptr);
    REQUIRE(fs::path(output).is_absolute());
    const fs::path sandbox = fs::path(output) /
        (std::to_string(Slic3r::get_current_pid()) + "-" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    const fs::path resources = sandbox / "portable-guard" / "a" / "b" / "resources";
    fs::create_directories(resources);
    REQUIRE_FALSE(fs::exists(resources / ".." / "configuration"));
    REQUIRE_FALSE(fs::exists(resources / ".." / ".." / ".." / "configuration"));
    const fs::path executable = sandbox / "current-executable";
    write_file(executable, "isolated executable identity\n");
    ConfigGlobals restore;
    Slic3r::set_data_dir("");
    Slic3r::set_resources_dir(resources.string());
    Slic3r::set_install_path(executable);
    // Viewer mode avoids unrelated editor-only defaults such as external FreeCAD discovery.
    Slic3r::AppConfig config(Slic3r::AppConfig::EAppMode::GCodeViewer);
    Slic3r::set_header_generate_with_date(false);
    const fs::path root = sandbox / "NematXSlicer";

    SECTION("Historical flat configuration is inventoried without an automatic GUI migration")
    {
        fs::create_directories(root);
        const fs::path ini = root / "NematXSlicer.ini";
        const std::string original = "# generated by NematXSlicer 2.7.63.4 on 2024-01-01\ncustom_marker = preserved\n";
        write_file(ini, original);
        REQUIRE_FALSE(fs::exists(root / "installed.ini"));
        REQUIRE(config.init_root_data_dir(root.string()));
        REQUIRE(same_path(config.get_root_data_dir(), root));
        const auto &entries = config.get_all_slicer_installed();
        REQUIRE(entries.size() == 1);
        CHECK(entries.front().version == Slic3r::Semver("2.7.63.4"));
        CHECK(entries.front().other_keys.at("legacy") == "1");
        CHECK(same_path(entries.front().get_config_path(config.get_root_data_dir()), root));
        CHECK_FALSE(Slic3r::has_data_dir());
        CHECK(config.data_dir().empty());
        CHECK(read_file(ini) == original);
        CHECK_FALSE(fs::exists(root / "installed.ini"));
        CHECK_FALSE(fs::exists(root / SLIC3R_BUILD_ID));
    }

    SECTION("Matching installed entry selects its existing relative configuration")
    {
        const fs::path selected = root / "historical-settings";
        fs::create_directories(selected);
        const fs::path ini = selected / "NematXSlicer.ini";
        const std::string original = "# preserved installed configuration\ncustom_marker = unchanged\n";
        write_file(ini, original);
        const fs::path old_selected = root / "nematx-2.7-settings";
        fs::create_directories(old_selected);
        const fs::path old_ini = old_selected / "NematXSlicer.ini";
        const std::string old_original = "# preserved NematXSlicer 2.7 installation\ncustom_marker = historical\n";
        write_file(old_ini, old_original);
        const std::string installed = std::string("[existing]\nversion = ") + SLIC3R_VERSION_FULL +
            "\nexe_path = " + executable.generic_string() +
            "\nexe_path_relative = 0\nconfig_path = historical-settings\nconfig_path_relative = 1\n" +
            "[nematx-2.7]\nversion = 2.7.63.4\nexe_path = " + executable.generic_string() +
            "\nexe_path_relative = 0\nconfig_path = nematx-2.7-settings\nconfig_path_relative = 1\n";
        write_file(root / "installed.ini", installed);
        REQUIRE(config.init_root_data_dir(root.string()));
        CHECK(same_path(config.get_root_data_dir(), root));
        REQUIRE(Slic3r::has_data_dir());
        CHECK(same_path(Slic3r::data_dir(), selected));
        CHECK(same_path(config.get_installation().get_config_path(config.get_root_data_dir()), selected));
        REQUIRE(config.get_all_slicer_installed().size() == 2);
        CHECK(config.get_all_slicer_installed().front().installed_name == "existing");
        CHECK(config.get_all_slicer_installed().back().installed_name == "nematx-2.7");
        CHECK(config.get_all_slicer_installed().back().version == Slic3r::Semver("2.7.63.4"));
        CHECK(same_path(config.get_all_slicer_installed().back().get_config_path(config.get_root_data_dir()), old_selected));
        CHECK(read_file(ini) == original);
        CHECK(read_file(old_ini) == old_original);
        CHECK(read_file(root / "installed.ini") == installed);
        CHECK_FALSE(fs::exists(root / SLIC3R_BUILD_ID));
    }

    SECTION("Historical installed entry alone is inventoried without automatic selection")
    {
        const fs::path selected = root / "nematx-2.7-settings";
        fs::create_directories(selected);
        const fs::path ini = selected / "NematXSlicer.ini";
        const std::string original = "# preserved NematXSlicer 2.7 configuration\ncustom_marker = old-only\n";
        write_file(ini, original);
        const std::string installed = std::string("[nematx-2.7]\nversion = 2.7.63.4\nexe_path = ") +
            executable.generic_string() +
            "\nexe_path_relative = 0\nconfig_path = nematx-2.7-settings\nconfig_path_relative = 1\n";
        write_file(root / "installed.ini", installed);
        REQUIRE(config.init_root_data_dir(root.string()));
        CHECK(same_path(config.get_root_data_dir(), root));
        const auto &entries = config.get_all_slicer_installed();
        REQUIRE(entries.size() == 1);
        CHECK(entries.front().installed_name == "nematx-2.7");
        CHECK(entries.front().version == Slic3r::Semver("2.7.63.4"));
        CHECK(same_path(entries.front().get_config_path(config.get_root_data_dir()), selected));
        CHECK_FALSE(Slic3r::has_data_dir());
        CHECK(config.data_dir().empty());
        CHECK(read_file(ini) == original);
        CHECK(read_file(root / "installed.ini") == installed);
        CHECK_FALSE(fs::exists(root / SLIC3R_BUILD_ID));
    }

    SECTION("Explicit data directory takes priority without creating a versioned root")
    {
        const fs::path selected = sandbox / "explicit-settings";
        fs::create_directories(selected);
        const fs::path ini = selected / "NematXSlicer.ini";
        const std::string original = "# explicit configuration remains untouched\n";
        write_file(ini, original);
        Slic3r::set_data_dir(selected.string());
        REQUIRE_FALSE(config.init_root_data_dir(root.string()));
        CHECK(same_path(Slic3r::data_dir(), selected));
        CHECK(same_path(config.data_dir(), selected));
        CHECK(config.get_installation().installed_name == "command_line");
        CHECK(config.get_root_data_dir().empty());
        CHECK_FALSE(fs::exists(root));
        CHECK(read_file(ini) == original);
    }
    CHECK(read_file(executable) == "isolated executable identity\n");
}
