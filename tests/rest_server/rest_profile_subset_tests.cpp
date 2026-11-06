/*
 * Profile selection on reduced preset bundles must answer exactly as the
 * same catalog working on full bundle copies: same IDs, same configuration,
 * same errors and profile sections, for selections, defaults and project loading.
 */

#include <catch2/catch.hpp>
#include <nlohmann/json.hpp>

#include "engine.hpp"
#include "profiles.hpp"
#include "rest_profiles_test_access.hpp"
#include "rest_models_test_helpers.hpp"
#include "rest_server_fixture.hpp"
#include "rest_vendor_test_helpers.hpp"
#include <boost/nowide/cstdlib.hpp>

#include <algorithm>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

using namespace RestServerTest;
using Slic3r::Rest::ProfileApplication;
using Slic3r::Rest::ProfileCatalog;
using Slic3r::Rest::ProfileCatalogTestAccess;
using Slic3r::Rest::ProfileSelection;

namespace {

// A dual-extruder printer, a filament renamed from an old name, templates
// shadowed by a system alias, user presets inheriting through another user
// preset, and user presets holding the names "project.3mf" loads generate,
// next to the Basic and TestVendor bundles.
void write_subset_profiles(const std::filesystem::path &root)
{
    write_vendor_profiles(root);
    std::ofstream(root / "vendor/Subset.ini") << R"ini([vendor]
name = SubsetVendor
config_version = 1.0.0

[printer_model:Dual]
name = Dual
variants = 0.4

[printer:Dual 0.4]
printer_model = Dual
printer_variant = 0.4
nozzle_diameter = 0.4,0.6
extruder_offset = 0x0,0x0
bed_shape = 0x0,200x0,200x200,0x200
default_filament_profile = PLA @S
default_print_profile = Subset print

[print:Subset print]
layer_height = 0.2
compatible_printers_condition = nozzle_diameter[0]==0.4

[filament:PLA @S]
filament_type = PLA
compatible_printers_condition = nozzle_diameter[0]==0.4

[filament:PETG @S wide]
filament_type = PETG
compatible_printers_condition = nozzle_diameter[0]==0.6

[filament:Renamed PLA]
renamed_from = "Old PLA"
filament_type = PLA
temperature = 210
compatible_printers_condition = nozzle_diameter[0]==0.4
)ini";
    std::ofstream(root / "vendor/Templates.ini") << R"ini([vendor]
name = Templates
config_version = 1.0.0
templates_profile = 1

[filament:PLA]
filament_type = PLA

[filament:TPU]
filament_type = FLEX
)ini";
    std::filesystem::create_directories(root / "filament");
    std::filesystem::create_directories(root / "printer");
    std::ofstream(root / "filament/My PLA.ini") << "inherits = PLA @S\nfilament_type = PLA\n"
        "compatible_printers_condition = nozzle_diameter[0]==0.4\ntemperature = 199\n";
    std::ofstream(root / "filament/My PLA child.ini") << "inherits = My PLA\nfilament_type = PLA\n"
        "compatible_printers_condition = nozzle_diameter[0]==0.4\ntemperature = 198\n";
    std::ofstream(root / "filament/My old.ini") << "inherits = Old PLA\nfilament_type = PLA\n"
        "compatible_printers_condition = nozzle_diameter[0]==0.4\n";
    std::filesystem::create_directories(root / "print");
    std::ofstream(root / "print/project.3mf (Missing).ini") << "layer_height = 0.33\n";
    std::ofstream(root / "print/project.3mf (Missing-1).ini") << "layer_height = 0.34\n";
    std::ofstream(root / "filament/project.3mf (Gone).ini") << "temperature = 250\n";
    std::ofstream(root / "filament/project.3mf.ini") << "temperature = 251\n";
    std::ofstream(root / "printer/My dual.ini") << "inherits = Dual 0.4\nprinter_model = Dual\nprinter_variant = 0.4\n"
        "nozzle_diameter = 0.4,0.6\nextruder_offset = 0x0,0x0\ndefault_filament_profile = My PLA child\n"
        "default_print_profile = Subset print\n";
    std::ofstream(root / "printer/My dual child.ini") << "inherits = My dual\nprinter_model = Dual\nprinter_variant = 0.4\n"
        "nozzle_diameter = 0.4,0.6\nextruder_offset = 0x0,0x0\ndefault_filament_profile = PETG @S wide;PLA @S\n"
        "default_print_profile = Subset print\n";
}

struct Outcome {
    std::string error;
    ProfileApplication application;
};

Outcome run(const std::function<ProfileApplication()> &operation)
{
    try {
        return {{}, operation()};
    } catch (const std::exception &error) {
        return {error.what(), {}};
    }
}

// Runs one operation on reduced bundles and on full copies, and requires the
// same answer. Returns the reduced answer.
Outcome check_same(ProfileCatalog &catalog, const std::function<ProfileApplication(const ProfileCatalog &)> &operation)
{
    ProfileCatalogTestAccess::set_full_bundle_copies(catalog, false);
    const Outcome reduced = run([&]() { return operation(catalog); });
    ProfileCatalogTestAccess::set_full_bundle_copies(catalog, true);
    const Outcome full = run([&]() { return operation(catalog); });
    ProfileCatalogTestAccess::set_full_bundle_copies(catalog, false);

    CHECK(reduced.error == full.error);
    CHECK(reduced.application.selection.print == full.application.selection.print);
    CHECK(reduced.application.selection.printer == full.application.selection.printer);
    CHECK(reduced.application.selection.filaments == full.application.selection.filaments);
    CHECK(reduced.application.profile_sections == full.application.profile_sections);
    const Slic3r::DynamicPrintConfig &reduced_config = reduced.application.configuration;
    const Slic3r::DynamicPrintConfig &full_config = full.application.configuration;
    CHECK(reduced_config.keys() == full_config.keys());
    CHECK(reduced_config.diff(full_config).empty());
    return reduced;
}

std::vector<std::string> ids(const nlohmann::json &profiles)
{
    std::vector<std::string> result;
    for (const nlohmann::json &profile : profiles)
        result.push_back(profile.at("id").get<std::string>());
    return result;
}

std::string id_of(const nlohmann::json &profiles, const std::string &name)
{
    for (const nlohmann::json &profile : profiles)
        if (profile.at("name") == name)
            return profile.at("id").get<std::string>();
    return {};
}

std::size_t extruders(const ProfileCatalog &catalog, const std::string &printer_id)
{
    const Slic3r::Preset *printer = catalog.printer_by_id(printer_id);
    REQUIRE(printer != nullptr);
    return printer->config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter")->size();
}

ProfileSelection selection(const std::string &printer, const std::string &print, std::vector<std::string> filaments = {})
{
    ProfileSelection result;
    result.printer = printer;
    result.print = print;
    result.filaments = std::move(filaments);
    return result;
}

// Project variants of a selected configuration: unchanged, with modified
// print and filament values, and naming the filament by an old name.
std::vector<Slic3r::DynamicPrintConfig> project_variants(const Slic3r::DynamicPrintConfig &configuration)
{
    std::vector<Slic3r::DynamicPrintConfig> variants{configuration};
    Slic3r::DynamicPrintConfig modified = configuration;
    modified.set_deserialize_strict("layer_height", "0.15");
    modified.option<Slic3r::ConfigOptionInts>("temperature")->set_at(231, 0);
    variants.push_back(modified);
    Slic3r::DynamicPrintConfig renamed = configuration;
    Slic3r::ConfigOptionStrings *filaments = renamed.option<Slic3r::ConfigOptionStrings>("filament_settings_id");
    std::vector<std::string> names = filaments->get_values();
    if (std::find(names.begin(), names.end(), "Renamed PLA") != names.end()) {
        std::replace(names.begin(), names.end(), std::string("Renamed PLA"), std::string("Old PLA"));
        filaments->set(names);
        variants.push_back(renamed);
    }
    // Presets the catalog lacks, without parents: loading generates names,
    // which collide with installed user presets.
    for (const bool named : {true, false}) {
        Slic3r::DynamicPrintConfig missing = configuration;
        missing.set_deserialize_strict("print_settings_id", "Missing");
        Slic3r::ConfigOptionStrings *slots = missing.option<Slic3r::ConfigOptionStrings>("filament_settings_id");
        slots->set(named ? std::vector<std::string>(slots->size(), "Gone") : std::vector<std::string>());
        // Written only when a preset has a parent.
        missing.erase("inherits_cummulative");
        variants.push_back(missing);
    }
    return variants;
}

} // namespace

TEST_CASE("REST reduced preset bundles answer as full catalog copies", "[rest_vendor_subset]")
{
    ensure_engine_initialized();
    TempDirectory root;
    write_subset_profiles(root.path());
    ProfileCatalog catalog(root.path());
    REQUIRE(catalog.warnings().empty());
    const nlohmann::json listed = catalog.list_profiles();
    const std::vector<std::string> printers = ids(listed.at("printer"));
    const std::vector<std::string> prints = ids(listed.at("print"));
    const std::vector<std::string> filaments = ids(listed.at("filaments"));
    REQUIRE_FALSE(id_of(listed.at("printer"), "My dual child").empty());
    REQUIRE_FALSE(id_of(listed.at("filaments"), "PLA @Template").empty());

    check_same(catalog, [](const ProfileCatalog &candidate) { return candidate.defaults(); });

    std::size_t accepted = 0;
    std::size_t rejected = 0;
    std::vector<Slic3r::DynamicPrintConfig> projects;
    for (const std::string &printer : printers) {
        bool explicit_filaments = false;
        for (const std::string &print : prints) {
            // Default filaments, then, for the first print accepted with the
            // printer, each filament in every slot.
            std::vector<std::vector<std::string>> requests{{}};
            for (std::size_t index = 0; index < requests.size(); ++index) {
                const std::vector<std::string> request = requests[index];
                CAPTURE(printer, print, request);
                const Outcome outcome = check_same(catalog, [&](const ProfileCatalog &candidate) {
                    return candidate.select(selection(printer, print, request));
                });
                if (!outcome.error.empty()) {
                    ++rejected;
                    continue;
                }
                ++accepted;
                if (request.size() <= 1)
                    projects.push_back(outcome.application.configuration);
                if (request.empty() && !explicit_filaments) {
                    explicit_filaments = true;
                    for (const std::string &filament : filaments)
                        requests.emplace_back(extruders(catalog, printer), filament);
                }
            }
        }
    }
    // Both outcomes are exercised, including template and per-slot rejections.
    CHECK(accepted > 10);
    CHECK(rejected > 20);

    // Each slot falls back to the other preferred filament compatible with
    // its own nozzle.
    const Outcome dual = check_same(catalog, [&](const ProfileCatalog &candidate) {
        return candidate.select(selection(id_of(listed.at("printer"), "My dual child"),
            id_of(listed.at("print"), "Subset print")));
    });
    REQUIRE(dual.error.empty());
    CHECK(dual.application.selection.filaments == std::vector<std::string>{
        id_of(listed.at("filaments"), "PLA @S"), id_of(listed.at("filaments"), "PETG @S wide")});

    std::size_t project_count = 0;
    std::size_t collisions = 0;
    for (const Slic3r::DynamicPrintConfig &project : projects)
        for (const Slic3r::DynamicPrintConfig &variant : project_variants(project)) {
            CAPTURE(project_count);
            const Outcome loaded = check_same(catalog, [&](const ProfileCatalog &candidate) {
                return candidate.load_project(variant, "project.3mf");
            });
            ++project_count;
            // Generated names skip the installed presets that use them, unless
            // the project came from that very preset with identical values.
            // A recomposed print never is: it gains custom-setting metadata.
            const std::string print = project.opt_string("print_settings_id");
            if (loaded.error.empty() && variant.opt_string("print_settings_id") == "Missing" &&
                print.rfind("project.3mf", 0) != 0) {
                CHECK(loaded.application.configuration.opt_string("print_settings_id") == "project.3mf (Missing-2)");
                // Without a stored name, the first filament collides with the
                // user filament named after the file.
                const bool named = !variant.option<Slic3r::ConfigOptionStrings>("filament_settings_id")->empty();
                const std::string filament = project.option<Slic3r::ConfigOptionStrings>("filament_settings_id")->get_at(0);
                const std::string installed = named ? "project.3mf (Gone)" : "project.3mf";
                CHECK(loaded.application.configuration.option<Slic3r::ConfigOptionStrings>("filament_settings_id")
                    ->get_at(0) == (filament == installed ? installed : named ? "project.3mf (Gone-1)" : "project.3mf (1)"));
                ++collisions;
            }
        }
    CHECK(project_count > 20);
    CHECK(collisions > 10);
}

TEST_CASE("REST reduced project bundles retain parents named only by profile sections", "[rest_vendor_subset][rest_profile_sections]")
{
    ensure_engine_initialized();
    TempDirectory root;
    write_subset_profiles(root.path());
    std::ofstream(root.path() / "printer/My dual child.ini", std::ios::app)
        << "\nprinthost_password = catalog-secret-witness\n";
    ProfileCatalog catalog(root.path());
    const nlohmann::json listed = catalog.list_profiles();
    const ProfileApplication selected = catalog.select(selection(id_of(listed.at("printer"), "My dual child"),
        id_of(listed.at("print"), "Subset print"),
        {id_of(listed.at("filaments"), "My PLA child"), id_of(listed.at("filaments"), "PETG @S wide")}));
    CHECK_FALSE(selected.configuration.has("printhost_password"));
    for (const char *filament_parent : {"My PLA child", "Old PLA"}) {
        CAPTURE(filament_parent);
        Slic3r::DynamicPrintConfig project(selected.configuration);
        project.set_deserialize_strict("print_settings_id", "Embedded print");
        project.set_deserialize_strict("printer_settings_id", "Embedded printer");
        project.set_deserialize_strict("filament_settings_id", "Embedded A;Embedded B");
        project.erase("inherits_cummulative");
        const std::string sections = std::string(
            "[print:Embedded print]\ninherits = Subset print\n\n"
            "[printer:Embedded printer]\ninherits = My dual child\n\n"
            "[filament:Embedded A]\ninherits = ") + filament_parent +
            "\n\n[filament:Embedded B]\ninherits = PETG @S wide\n";
        const Outcome loaded = check_same(catalog, [&](const ProfileCatalog &candidate) {
            return candidate.load_project(project, "project.3mf", sections);
        });
        REQUIRE(loaded.error.empty());
        CHECK(loaded.application.configuration.opt_string("printer_settings_id") == "My dual child");
        CHECK_FALSE(loaded.application.profile_sections.empty());
        CHECK(loaded.application.profile_sections.find("catalog-secret-witness") == std::string::npos);
        CHECK_FALSE(loaded.application.configuration.has("printhost_password"));
    }
}

// Release check on a real vendor catalog (REST_VENDOR_PROFILES_DIR), sampled:
// each full-copy reference costs seconds there.
TEST_CASE("REST reduced preset bundles answer as full copies of a real catalog", "[.rest_vendor_subset_real]")
{
    ensure_engine_initialized();
    const char *source = boost::nowide::getenv("REST_VENDOR_PROFILES_DIR");
    REQUIRE(source != nullptr);
    TempDirectory root;
    std::filesystem::create_directories(root.path() / "vendor");
    for (const std::filesystem::directory_entry &entry : std::filesystem::directory_iterator(std::filesystem::u8path(source)))
        if (entry.is_regular_file() && entry.path().extension() == ".ini")
            std::filesystem::copy_file(entry.path(), root.path() / "vendor" / entry.path().filename());
    ProfileCatalog catalog(root.path());
    const nlohmann::json listed = catalog.list_profiles();
    const std::vector<std::string> printers = ids(listed.at("printer"));
    REQUIRE(printers.size() > 10);

    // One printer in N, plus the first multi-extruder printers.
    std::vector<std::string> sample;
    std::size_t multi = 0;
    const std::size_t step = printers.size() / 12 + 1;
    for (std::size_t index = 0; index < printers.size(); ++index) {
        const bool multiple = extruders(catalog, printers[index]) > 1;
        if (index % step == 0 || (multiple && multi < 4)) {
            sample.push_back(printers[index]);
            multi += multiple ? 1 : 0;
        }
    }

    check_same(catalog, [](const ProfileCatalog &candidate) { return candidate.defaults(); });
    std::size_t accepted = 0;
    std::size_t rejected = 0;
    std::size_t projects = 0;
    for (const std::string &printer : sample) {
        const Slic3r::Preset *printer_preset = catalog.printer_by_id(printer);
        const nlohmann::json compatible_prints = catalog.list_profiles({printer_preset, nullptr}, true).at("print");
        if (compatible_prints.empty())
            continue;
        const std::string print = compatible_prints.front().at("id").get<std::string>();
        const nlohmann::json compatibility = catalog.list_profiles({printer_preset, catalog.print_by_id(print)}, false);
        std::string compatible_filament;
        std::string incompatible_filament;
        for (const nlohmann::json &entry : compatibility.at("filaments")) {
            std::string &target = entry.at("compatible").get<bool>() ? compatible_filament : incompatible_filament;
            if (target.empty())
                target = entry.at("id").get<std::string>();
        }
        std::vector<std::vector<std::string>> requests{{}};
        for (const std::string &filament : {compatible_filament, incompatible_filament})
            if (!filament.empty())
                requests.emplace_back(extruders(catalog, printer), filament);
        for (const std::vector<std::string> &request : requests) {
            CAPTURE(printer_preset->name, print, request);
            const Outcome outcome = check_same(catalog, [&](const ProfileCatalog &candidate) {
                return candidate.select(selection(printer, print, request));
            });
            if (!outcome.error.empty()) {
                ++rejected;
                continue;
            }
            ++accepted;
            if (request.empty() && projects < 6)
                for (const Slic3r::DynamicPrintConfig &variant : project_variants(outcome.application.configuration)) {
                    check_same(catalog, [&](const ProfileCatalog &candidate) {
                        return candidate.load_project(variant, "project.3mf");
                    });
                    ++projects;
                }
        }
    }
    CHECK(accepted > 10);
    CHECK(rejected > 0);
    CHECK(projects > 0);
}
