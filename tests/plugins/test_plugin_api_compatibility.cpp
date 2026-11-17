// Validate the common ABI rules without loading a library. Package parsing
// must preserve literal header paths and never infer ABI from build versions.
#include <catch2/catch.hpp>
#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>
#include "libslic3r/Plugins/PluginApiCompatibility.hpp"

using namespace Slic3r;

extern "C" int test_plugin_api_headers_c(void);

// Require a host that consumes the policy, while keeping earlier layer editors compatible.
TEST_CASE("Retraction crossing policies publish layer-edit API 1.2",
          "[plugins][abi][top-surface-retraction]")
{
    const auto &contracts = host_plugin_api_contracts();
    const auto found = std::find_if(contracts.begin(), contracts.end(),
        [](const PluginApiRequirement &entry) { return entry.header == "steps/slic3r_step_layer_extrusion_edit.h"; });
    REQUIRE(found != contracts.end());
    CHECK(found->version.major == 1);
    CHECK(found->version.minor == 2);
    for (const char *version : {"1.1", "1.2", "1.3"}) {
        PluginPackageMetadata metadata;
        std::string error;
        REQUIRE(parse_plugin_package_metadata(std::string("[plugin]\npackage_version=1.0.0\nslicer_version=1.0.0\n"
            "[abi]\nslic3r_plugin_types.h=") + std::to_string(SLIC3R_PLUGIN_API_PLUGIN_TYPES_MAJOR) + "." +
            std::to_string(SLIC3R_PLUGIN_API_PLUGIN_TYPES_MINOR) +
            "\nsteps/slic3r_step_layer_extrusion_edit.h=" + version + "\n", metadata, error));
        CHECK(metadata.compatibility.compatible() == (std::string(version) != "1.3"));
    }
}

// Check the additive extrusion contract without changing public structure sizes or the two legacy flag bits.
TEST_CASE("Extrusion entity publishes minor 1.2 and refuses later requirements",
          "[plugins][abi][rotation]")
{
    const auto &contracts = host_plugin_api_contracts();
    const auto found = std::find_if(contracts.begin(), contracts.end(),
        [](const PluginApiRequirement &entry) { return entry.header == "slic3r_extrusion_entity.h"; });
    REQUIRE(found != contracts.end());
    CHECK(found->version.major == 1);
    CHECK(found->version.minor == 2);
    for (const char *version : {"1.0", "1.1", "1.2", "1.3"}) {
        PluginPackageMetadata metadata;
        std::string error;
        REQUIRE(parse_plugin_package_metadata(std::string("[plugin]\npackage_version=1.0.0\nslicer_version=1.0.0\n"
            "[abi]\nslic3r_plugin_types.h=") + std::to_string(SLIC3R_PLUGIN_API_PLUGIN_TYPES_MAJOR) + "." +
            std::to_string(SLIC3R_PLUGIN_API_PLUGIN_TYPES_MINOR) +
            "\nslic3r_extrusion_entity.h=" + version + "\n", metadata, error));
        CHECK(metadata.compatibility.compatible() == (std::string(version) != "1.3"));
    }
}

TEST_CASE("Package ABI diagnostics identify the manifest section key and invalid value", "[plugins][abi][repository]")
{
    const std::string declaration = GENERATE(std::string(""),
        std::string("[abi]\n"),
        std::string("[abi]\nslic3r_plugin_types.h=bad.version\n"),
        std::string("[abi]\nslic3r_plugin_types.h=3.0\nslic3r_plugin_types.h=3.0\n"));
    const boost::filesystem::path path = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("plugin-version-%%%%-%%%%.ini");
    {
        boost::nowide::ofstream stream(path.string());
        stream << "[plugin]\npackage_version=1.0.0\nslicer_version=1.0.0\n" << declaration;
    }
    PluginPackageMetadata metadata;
    std::string error;
    const bool parsed = read_plugin_package_metadata(path.string(), metadata, error);
    boost::filesystem::remove(path);
    REQUIRE(parsed);
    const std::string diagnostic = metadata.compatibility.message();
    CHECK(diagnostic.find(path.string()) != std::string::npos);
    CHECK(diagnostic.find("[abi]") != std::string::npos);
    CHECK(diagnostic.find("slic3r_plugin_types.h") != std::string::npos);
    if (declaration.empty())
        CHECK(diagnostic.find("Missing [abi] section") != std::string::npos);
    if (declaration.find("bad.version") != std::string::npos) {
        CHECK(diagnostic.find("bad.version") != std::string::npos);
        CHECK(diagnostic.find("major.minor") != std::string::npos);
    }
    if (declaration.find("3.0") != std::string::npos) {
        CHECK(diagnostic.find("line 6") != std::string::npos);
        CHECK(diagnostic.find("slic3r_plugin_types.h=3.0") != std::string::npos);
    }
}

TEST_CASE("Package ABI uses literal header names and ignores the build version", "[plugins][abi][repository]")
{
    for (const std::string &slicer : {std::string("1.0.0"), std::string("9999.0.0")}) {
        PluginPackageMetadata metadata;
        std::string error;
        REQUIRE(parse_plugin_package_metadata("[plugin]\npackage_version=1.0.0\nslicer_version=" + slicer +
            "\n[abi]\nslic3r_plugin_types.h=4.0\nsteps/slic3r_step_post_perimeter.h=1.0\nfuture.h=0.0\n", metadata, error));
        CHECK(metadata.compatibility.compatible());
        REQUIRE(metadata.abi.size() == 3);
        CHECK(metadata.abi[1].header == "steps/slic3r_step_post_perimeter.h");
        CHECK(metadata.slicer_version == slicer);
    }
}

// Parse test-owned package descriptions against the compiled header-version table without loading plugin code.
TEST_CASE("Orchestrator package requirements accept 3.5 and reject newer minors", "[plugins][abi][repository]")
{
    const std::string version = GENERATE(std::string("3.4"), std::string("3.5"), std::string("3.6"));
    PluginPackageMetadata metadata;
    std::string error;
    REQUIRE(parse_plugin_package_metadata("[plugin]\npackage_version=1.0.0\nslicer_version=2.7.63.0\n"
        "[abi]\nslic3r_plugin_types.h=4.0\nslic3r_orchestrator.h=" + version + "\n", metadata, error));
    CHECK(metadata.compatibility.compatible() == (version != "3.6"));
}

// Check additive data-tree requirements against the host's compiled version table.
TEST_CASE("Data tree package requirements accept 5.1 through 5.5 and reject 5.6", "[plugins][abi][data-tree]")
{
    const std::string version = GENERATE(std::string("5.1"), std::string("5.2"), std::string("5.3"),
        std::string("5.4"), std::string("5.5"), std::string("5.6"));
    PluginPackageMetadata metadata;
    std::string error;
    REQUIRE(parse_plugin_package_metadata("[plugin]\npackage_version=1.0.0\nslicer_version=2.7.63.0\n"
        "[abi]\nslic3r_plugin_types.h=4.0\nslic3r_data_tree.h=" + version + "\n", metadata, error));
    CHECK(metadata.compatibility.compatible() == (version != "5.6"));
}

TEST_CASE("Package ABI rejects missing invalid and conflicting requirements", "[plugins][abi][repository]")
{
    const std::string abi = GENERATE(std::string(""), std::string("[abi]\n"),
        std::string("[abi]\nslic3r_plugin_types.h=0.0\n"),
        std::string("[abi]\nslic3r_plugin_types.h=2.0\n"),
        std::string("[abi]\nslic3r_plugin_types.h=2.4\n"),
        std::string("[abi]\nslic3r_plugin_types.h=3.0\n"),
        std::string("[abi]\nslic3r_plugin_types.h=5.0\n"),
        std::string("[abi]\nslic3r_plugin_types.h=4.65535\n"),
        std::string("[abi]\nslic3r_plugin_types.h=4.0\nfuture.h=1.0\n"),
        std::string("[abi]\nslic3r_plugin_types.h=4.0\nslic3r_plugin_types.h=4.0\n"),
        std::string("[abi]\nslic3r_plugin_types.h=1.0.0\n"),
        std::string("[abi]\nslic3r_plugin_types.h=-1.0\n"),
        std::string("[abi]\nslic3r_plugin_types.h=65536.0\n"));
    PluginPackageMetadata metadata;
    std::string error;
    parse_plugin_package_metadata("[plugin]\npackage_version=1.0.0\nslicer_version=1.0.0\n" + abi, metadata, error);
    CHECK(metadata.compatibility.status == PluginApiCompatibilityStatus::Incompatible);
    CHECK_FALSE(metadata.compatibility.message().empty());
}

TEST_CASE("Clipper 2.3 rejects 1.x packages and accepts earlier 2.x minors", "[plugins][abi][clipper]")
{
    // The 2.x union takes a fill rule; minors add inward offsets, native polygon operations and ring detours.
    const std::vector<PluginApiRequirement> &host = host_plugin_api_contracts();
    const std::vector<PluginApiRequirement>::const_iterator clipper = std::find_if(host.begin(), host.end(),
        [](const PluginApiRequirement &contract) { return contract.header == "slic3r_clipper.h"; });
    REQUIRE(clipper != host.end());
    CHECK(clipper->version.major == 2);
    CHECK(clipper->version.minor == 3);

    for (const char *version : {"1.0", "1.3", "1.65535"}) {
        PluginPackageMetadata metadata;
        std::string error;
        parse_plugin_package_metadata(std::string("[plugin]\npackage_version=1.0.0\nslicer_version=1.0.0\n"
            "[abi]\nslic3r_plugin_types.h=4.0\nslic3r_clipper.h=") + version + "\n", metadata, error);
        CHECK(metadata.compatibility.status == PluginApiCompatibilityStatus::Incompatible);
        CHECK(metadata.compatibility.message().find("slic3r_clipper.h") != std::string::npos);
    }
    // Both the existing layout and the additive function extension are compatible.
    for (const char *version : {"2.0", "2.1", "2.2", "2.3", "2.4"}) {
        PluginPackageMetadata current;
        std::string error;
        REQUIRE(parse_plugin_package_metadata(std::string("[plugin]\npackage_version=1.0.0\nslicer_version=1.0.0\n"
            "[abi]\nslic3r_plugin_types.h=4.0\nslic3r_clipper.h=") + version + "\n", current, error));
        CHECK(current.compatibility.compatible() == (std::string(version) != "2.4"));
    }
}

TEST_CASE("Volume API 2.1 rejects 1.x packages and keeps 2.0 packages compatible", "[plugins][abi][volume]")
{
    // The host exposes slic3r_volume.h 2.1: the 2.0 slicing function plus the internal attractor name
    // and the support-geometry volume types.
    const std::vector<PluginApiRequirement> &host = host_plugin_api_contracts();
    const std::vector<PluginApiRequirement>::const_iterator volume = std::find_if(host.begin(), host.end(),
        [](const PluginApiRequirement &contract) { return contract.header == "slic3r_volume.h"; });
    REQUIRE(volume != host.end());
    CHECK(volume->version.major == 2);
    CHECK(volume->version.minor == 1);

    for (const char *version : {"1.0", "1.2", "1.65535"}) {
        PluginPackageMetadata metadata;
        std::string error;
        parse_plugin_package_metadata(std::string("[plugin]\npackage_version=1.0.0\nslicer_version=1.0.0\n"
            "[abi]\nslic3r_plugin_types.h=4.0\nslic3r_volume.h=") + version + "\n", metadata, error);
        CHECK(metadata.compatibility.status == PluginApiCompatibilityStatus::Incompatible);
        CHECK(metadata.compatibility.message().find("slic3r_volume.h") != std::string::npos);
    }
    // A 2.0 package and a package requiring the new types are accepted; a later minor is not.
    for (const char *version : {"2.0", "2.1", "2.2"}) {
        PluginPackageMetadata current;
        std::string error;
        REQUIRE(parse_plugin_package_metadata(std::string("[plugin]\npackage_version=1.0.0\nslicer_version=1.0.0\n"
            "[abi]\nslic3r_plugin_types.h=4.0\nslic3r_volume.h=") + version + "\n", current, error));
        CHECK(current.compatibility.compatible() == (std::string(version) != "2.2"));
    }
}

TEST_CASE("DLL and package validators share the same contract registry", "[plugins][abi][loader]")
{
    const std::vector<PluginApiRequirement> &host = host_plugin_api_contracts();
    std::vector<slic3r_major_minor_version> table;
    for (const PluginApiRequirement &contract : host)
        table.push_back(contract.version);
    CHECK(validate_plugin_api_table(table).compatible());
    CHECK(validate_plugin_api_requirements(host).compatible());
    table.resize(1);
    CHECK(validate_plugin_api_table(table).compatible());
    table.resize(host.size() + 10);
    CHECK(validate_plugin_api_table(table).compatible());
    table.back() = {1, 0};
    CHECK_FALSE(validate_plugin_api_table(table).compatible());
    table.assign(1, {0, 0});
    CHECK_FALSE(validate_plugin_api_table(table).compatible());
    table.assign(1, {4, 0});
    CHECK(validate_plugin_api_table(table).compatible());
    table.assign(1, {3, 0});
    CHECK_FALSE(validate_plugin_api_table(table).compatible());
    table.assign(1, {2, 0});
    CHECK_FALSE(validate_plugin_api_table(table).compatible());
    table.assign(1, {2, 4});
    CHECK_FALSE(validate_plugin_api_table(table).compatible());
    table.assign(1, {5, 0});
    CHECK_FALSE(validate_plugin_api_table(table).compatible());
    CHECK(PluginPackageMetadata{}.compatibility.status == PluginApiCompatibilityStatus::NotChecked);
}

TEST_CASE("Volume, run context and clipper headers compile as C", "[plugins][abi][c-headers]")
{
    REQUIRE(test_plugin_api_headers_c() == 1);
}

TEST_CASE("Fan policy and timing declare independent public ABI requirements", "[plugins][abi][fan-policy]")
{
    for (const char *header : {"slic3r_fan_policy.h", "slic3r_printing_layer_time.h"}) {
        INFO(header);
        PluginPackageMetadata metadata;
        std::string error;
        const std::string prefix = std::string("[plugin]\npackage_version=1.0.0\nslicer_version=1.0.0\n[abi]\nslic3r_plugin_types.h=") +
            std::to_string(SLIC3R_PLUGIN_API_PLUGIN_TYPES_MAJOR) + "." +
            std::to_string(SLIC3R_PLUGIN_API_PLUGIN_TYPES_MINOR) + "\n" + header + "=";
        const std::string major = std::string(header) == "slic3r_fan_policy.h" ? "2" : "1";
        REQUIRE(parse_plugin_package_metadata(prefix + major + ".0\n", metadata, error));
        CHECK(metadata.compatibility.compatible());
        REQUIRE(parse_plugin_package_metadata(prefix + major + ".1\n", metadata, error));
        CHECK_FALSE(metadata.compatibility.compatible());
        const std::string incompatible_major = major == "2" ? "1" : "2";
        REQUIRE(parse_plugin_package_metadata(prefix + incompatible_major + ".0\n", metadata, error));
        CHECK_FALSE(metadata.compatibility.compatible());
    }
}
