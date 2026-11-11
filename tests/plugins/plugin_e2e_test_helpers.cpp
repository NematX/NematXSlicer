///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Build small, independent setting catalogues and STL inputs for plugin slicing
   and export tests. This helper does not itself run Print::process() or export.

   linear_cases() reads the targeted active plugins' used and defined keys. For
   each domain it applies only that domain's overrides to an unchanged baseline,
   compares effective serialized values, and deduplicates equivalent results.
   Each declared key must actually vary; listing an unchanged value is not coverage.
   Domains are independent settings or explicitly coupled groups, never combined
   into a Cartesian or pairwise matrix. enum_domain() reads an option's finite
   serialized enum values.

   Representative direct calls:

       linear_cases()
           |-- get_used_config_keys(), get_defined_config_keys()
           `-- apply_case() [fresh baseline copy for each candidate]

       stl_fixture()
           |-- make_cube(), merge() [cube or notched frame assembly]
           `-- stl_fixture(mesh) [also accepts a caller-supplied scene]
               |-- write_binary() [uniquely named temporary STL]
               |-- ReadSTLFile() [exercise the actual import path]
               `-- remove() [cleanup on success, attempted again on failure]

   Configurations and meshes are returned as owned values. Registry/config inputs
   are borrowed only for the call; activation and production objects are unchanged.
   Only stl_fixture() writes a temporary file; import and filesystem errors propagate.
*/
#include "plugin_e2e_test_helpers.hpp"
#include <map>
#include <set>
#include <stdexcept>
#include <boost/filesystem.hpp>
#include "gcode_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"

namespace Slic3r::Test::PluginE2E {
// Build a cube or notched frame, then return its STL round trip; write, import and cleanup errors propagate.
TriangleMesh stl_fixture(bool bridge_frame);
// Write an owned mesh copy and read it back; remove the file on success and attempt cleanup before propagating failures.
TriangleMesh stl_fixture(TriangleMesh mesh);

std::vector<Case> linear_cases(const Orchestrator &host, const std::vector<std::string> &plugins,
    const DynamicPrintConfig &baseline, const std::vector<Domain> &domains)
{
    std::set<std::string> declared, catalogued, varied, names, configurations;
    for (const std::string &id : plugins) {
        const Plugin *plugin = host.get_plugin(id);
        if (plugin == nullptr || !host.is_plugin_active(plugin))
            throw std::invalid_argument("The E2E baseline must activate plugin '" + id + "'.");
        for (const Plugin::UsedConfigKey &key : plugin->get_used_config_keys()) declared.insert(key.key);
        for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys()) declared.insert(key.key);
    }
    std::vector<Case> result{{"default", {}}};
    names.insert("default");
    for (const Domain &domain : domains) {
        if (domain.keys.empty() || domain.cases.empty())
            throw std::invalid_argument("An E2E setting domain has no keys or values.");
        const std::set<std::string> allowed(domain.keys.begin(), domain.keys.end());
        for (const std::string &key : allowed) {
            if (baseline.optptr(key) == nullptr)
                throw std::invalid_argument("No E2E baseline value exists for '" + key + "'.");
            catalogued.insert(key);
        }
        for (const Case &test : domain.cases) {
            if (test.name.empty() || !names.insert(test.name).second)
                throw std::invalid_argument("Duplicate or empty E2E case name: '" + test.name + "'.");
            std::set<std::string> written;
            for (const auto &[key, value] : test.settings)
                if (allowed.count(key) == 0 || !written.insert(key).second)
                    throw std::invalid_argument("E2E case '" + test.name + "' changes a key outside its group or repeats it: '" + key + "'.");
            const DynamicPrintConfig config = apply_case(baseline, test);
            std::vector<std::string> changed;
            for (const std::string &key : written)
                if (baseline.optptr(key)->serialize() != config.optptr(key)->serialize()) changed.push_back(key);
            if (changed.empty()) continue;
            // Sorted, delimited changed keys make equivalent cases match even
            // when override order or redundant baseline-valued entries differ.
            std::string signature;
            for (const std::string &key : changed) {
                varied.insert(key);
                signature += key + '\0' + config.optptr(key)->serialize() + '\0';
            }
            if (configurations.insert(signature).second) result.push_back(test);
        }
    }
    // Listing a key is not coverage: at least one effective value must differ
    // from baseline, or a new declaration would silently receive no exercise.
    for (const std::string &key : declared)
        if (catalogued.count(key) == 0 || varied.count(key) == 0)
            throw std::invalid_argument("Declare an E2E variation domain for setting '" + key + "'.");
    return result;
}

DynamicPrintConfig apply_case(const DynamicPrintConfig &baseline, const Case &test)
{
    DynamicPrintConfig result = baseline;
    for (const auto &[key, value] : test.settings) {
        const ConfigOption *option = result.optptr(key);
        if (!value.empty() && value.front() == '!' && option != nullptr && !option->can_be_disabled())
            throw std::invalid_argument("E2E setting '" + key + "' cannot be disabled.");
        result.set_deserialize_strict(key, value);
    }
    return result;
}

Domain enum_domain(const std::string &key)
{
    const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
    if (definition == nullptr || !definition->enum_def)
        throw std::invalid_argument("No enum values exist for E2E setting '" + key + "'.");
    Domain result{{key}, {}};
    for (const std::string &value : definition->enum_def->values())
        result.cases.push_back({key + "=" + value, {{key, value}}});
    return result;
}

TriangleMesh stl_fixture(bool bridge_frame)
{
    TriangleMesh mesh = make_cube(4., 4., .6);
    if (bridge_frame) {
        // A box assembly keeps import/slicing cheap while exposing an opening,
        // an unsupported roof, a notched edge and a higher top pad.
        mesh = make_cube(3., 10., .6);
        TriangleMesh right = make_cube(3., 10., .6), front = make_cube(6., 3., .6), back = make_cube(6., 3., .6);
        right.translate(9.f, 0.f, 0.f);front.translate(3.f, 0.f, 0.f);back.translate(3.f, 7.f, 0.f);
        mesh.merge(right);mesh.merge(front);mesh.merge(back);
        TriangleMesh roof = make_cube(12., 8., .4), ledge = make_cube(8., 2., .4), pad = make_cube(2., 2., .2);
        roof.translate(0.f, 0.f, .6f);ledge.translate(0.f, 8.f, .6f);pad.translate(0.f, 0.f, 1.f);
        mesh.merge(roof);mesh.merge(ledge);mesh.merge(pad);
    }
    return stl_fixture(mesh);
}

TriangleMesh stl_fixture(TriangleMesh mesh)
{
    boost::filesystem::path path = GCode::temporary_gcode_path();
    path.replace_extension(".stl");
    // Import the file, not the in-memory source mesh, so every suite exercises
    // the STL reader before reusing the same input across independent cases.
    try {
        if (!mesh.write_binary(path.string().c_str())) throw std::runtime_error("Cannot write the tiny E2E STL.");
        TriangleMesh loaded;
        if (!loaded.ReadSTLFile(path.string().c_str())) throw std::runtime_error("Cannot read the tiny E2E STL.");
        boost::filesystem::remove(path);
        return loaded;
    } catch (...) {
        boost::system::error_code ignored;
        boost::filesystem::remove(path, ignored);
        throw;
    }
}
}
