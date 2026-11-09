#include <catch2/catch.hpp>

#include <string>

#include <boost/filesystem.hpp>

#include "libslic3r/Config/AppConfig.hpp"
#include "libslic3r/Utils.hpp"

/* Check application preferences in the core without initializing a GUI or
   reading an installed user profile. Font checks preserve positive integer
   prefixes while rejecting empty, invalid or nonpositive stored values.
   Datadir checks adopt an isolated path without creating it or inventorying a
   default root. Process path state and the runner's timestamp-free baseline are
   restored after each case, including when an assertion interrupts the test.

       TEST_CASE -> AppConfig::set_defaults(): normalize in-memory preferences
       TEST_CASE -> AppConfig::init_root_data_dir(): adopt the explicit process path
           `-- inspect ConfigurationEntry: verify copied path metadata
*/

namespace {

struct PreferenceGlobals {
    const std::string data = Slic3r::has_data_dir() ? Slic3r::data_dir() : std::string();

    // Restore the borrowed process path and the test runner's timestamp policy; no files are changed.
    ~PreferenceGlobals()
    {
        Slic3r::set_data_dir(data);
        Slic3r::set_header_generate_with_date(false);
    }
};

} // namespace

TEST_CASE("Host font preferences normalize nonpositive values without changing positive prefixes", "[plugins][host-preferences]")
{
    const PreferenceGlobals restore;
    Slic3r::AppConfig config(Slic3r::AppConfig::EAppMode::Editor);
    config.set("freecad_path", "font-preference-test-unused");
    CHECK(config.get("font_size") == "0");
    config.set("font_pt_size", "-3");
    for (const std::string &value : {"", "0", "-1", "-12", "abc", "14", "14px"}) {
        CAPTURE(value);
        config.set("font_size", value);
        config.set_defaults();
        CHECK(config.get("font_size") == (value == "14" || value == "14px" ? value : "0"));
        CHECK(config.get("font_pt_size") == "-3");
    }
}

TEST_CASE("Host configuration adopts an explicit datadir without inventory or filesystem writes", "[plugins][host-preferences]")
{
    const PreferenceGlobals restore;
    const boost::filesystem::path root = boost::filesystem::path("build") / "temp" /
        boost::filesystem::unique_path("host-preferences-%%%%-%%%%");
    const boost::filesystem::path explicit_path = root / "explicit";
    const boost::filesystem::path default_path = root / "unused-default";
    REQUIRE_FALSE(boost::filesystem::exists(root));
    Slic3r::set_data_dir(explicit_path.string());
    Slic3r::AppConfig config(Slic3r::AppConfig::EAppMode::Editor);
    CHECK_FALSE(config.init_root_data_dir(default_path.string()));
    const Slic3r::AppConfig::ConfigurationEntry installed = config.get_installation();
    CHECK(installed.config_path == explicit_path);
    CHECK(installed.installed_name == "command_line");
    CHECK(installed.other_keys.at("config_path") == explicit_path.string());
    CHECK(installed.other_keys.at("config_path_relative") == "0");
    CHECK(config.get_root_data_dir().empty());
    CHECK(config.get_all_slicer_installed().empty());
    CHECK(Slic3r::data_dir() == explicit_path.string());
    CHECK_FALSE(boost::filesystem::exists(root));
}
