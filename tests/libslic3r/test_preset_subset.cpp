///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include <catch2/catch.hpp>

#include <fstream>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "preset_test_helpers.hpp"
#include "libslic3r/ContainerUtils.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PresetBundle.hpp"

/*
These tests compare a reduced FFF catalog with a copy of the full catalog.
The reduced copy keeps requested profiles, their parents and selection fallbacks.
Both copies must give the same compatibility, automatic selection and combined settings.
Queries must leave the source catalog unchanged. Profile vendor pointers must
refer to records owned by the copy, even after the source has been destroyed.

Operation notes:
- write_profiles: create vendor files, templates and multi-level parent chains
- load_bundle: load vendors before user profiles
- select: choose profiles and rebuild filament positions
- subset / compatible_filament_names: construct or query a reduced catalog
- Compare with the full copy; check source state and copied vendor ownership

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- write_profiles()
        |-- root.path()
        |-- load_bundle()
        |-- select()
        |-- bundle.filaments.select_preset_by_name()
        `-- bundle.subset()
*/

using namespace Slic3r;

namespace {

using TemporaryRoot = PresetTest::TemporaryDirectory;
using CatalogState = std::tuple<std::vector<std::tuple<std::string, bool, bool>>, std::string, std::string,
    std::string, std::string, std::vector<std::string>>;

// Write vendor/template profiles and user profiles whose parents span several levels or old names.
void write_profiles(const std::filesystem::path &root);
// Load system profiles before flat user profiles, as the GUI does.
void load_bundle(PresetBundle &bundle, const std::filesystem::path &root);
// Select a printer/print pair and rebuild its filament slots before evaluating compatibility.
void select(PresetBundle &bundle, const std::string &printer, const std::string &print);
// Collect actual names in collection order, including defaults.
std::vector<std::string> names_of(const PresetCollection &collection);
// Filter source names by the expected retained set, always keeping defaults.
std::vector<std::string> expected_names(const PresetCollection &source, const std::set<std::string> &kept);
// Find the named filament's compatibility state for this extruder. Fail the test if it is absent.
const Filament &slot_filament(const ExtruderFilaments &slot, const std::string &name);
// Resolve vendor identity through all parents, including renamed ones.
std::string vendor_id(const PresetCollection &collection, const std::string &name);
// Use the full catalog's normal evaluator to collect names compatible with at least one extruder.
std::set<std::string> extruder_compatible_names(const PresetBundle &source, const std::string &printer, const std::string &print);
// Snapshot the catalog's compatibility, visibility and selections before a read-only query.
CatalogState catalog_state(const PresetBundle &bundle);

void write_profiles(const std::filesystem::path &root)
{
    std::filesystem::create_directories(root / "vendor");
    std::filesystem::create_directories(root / "filament");
    std::filesystem::create_directories(root / "printer");
    std::ofstream(root / "vendor" / "SubsetVendor.ini") << R"ini([vendor]
name = Subset vendor
config_version = 1.0.0

[printer_model:Single]
name = Single
variants = 0.4

[printer_model:Dual]
name = Dual
variants = 0.4

[printer_model:Wide]
name = Wide
variants = 0.6

[printer:*common*]
bed_shape = 0x0,200x0,200x200,0x200
printer_variant = 0.4
default_filament_profile = PLA @A

[printer:Single 0.4]
inherits = *common*
printer_model = Single
nozzle_diameter = 0.4

[printer:Dual 0.4]
inherits = *common*
printer_model = Dual
nozzle_diameter = 0.4,0.6
extruder_offset = 0x0,0x0

[printer:Twin 0.4]
inherits = *common*
printer_model = Dual
nozzle_diameter = 0.4,0.4
extruder_offset = 0x0,0x0

[printer:MMU 0.4]
inherits = *common*
printer_model = Dual
single_extruder_multi_material = 1
nozzle_diameter = 0.4,0.6
extruder_offset = 0x0,0x0

[printer:Wide 0.6]
inherits = *common*
printer_model = Wide
printer_variant = 0.6
nozzle_diameter = 0.6

[print:Print 0.4]
layer_height = 0.2
compatible_printers_condition = nozzle_diameter[0]==0.4

[print:Print 0.6]
layer_height = 0.3
compatible_printers_condition = nozzle_diameter[0]==0.6

[filament:PLA @A]
filament_type = PLA
compatible_printers_condition = nozzle_diameter[0]==0.4

[filament:PETG @wide]
filament_type = PETG
compatible_printers_condition = nozzle_diameter[0]==0.6

[filament:ASA @none]
filament_type = ASA
compatible_printers_condition = nozzle_diameter[0]==1.0

[filament:Renamed PLA]
renamed_from = "Old PLA"
filament_type = PLA
compatible_printers_condition = nozzle_diameter[0]==0.4
)ini";
    std::ofstream(root / "vendor" / "OtherVendor.ini") << R"ini([vendor]
name = Other vendor
config_version = 1.0.0

[printer_model:Other]
name = Other
variants = 0.4

[printer:Other 0.4]
printer_model = Other
printer_variant = 0.4
nozzle_diameter = 0.4
default_filament_profile = Other PLA @O

[filament:Other PLA @O]
filament_type = PLA
compatible_printers_condition = nozzle_diameter[0]==0.4
)ini";
    std::ofstream(root / "vendor" / "Templates.ini") << R"ini([vendor]
name = Templates
config_version = 1.0.0
templates_profile = 1

[filament:PLA]
filament_type = PLA

[filament:TPU]
filament_type = FLEX
)ini";
    std::ofstream(root / "filament" / "My PLA.ini") <<
        "inherits = PLA @A\nfilament_type = PLA\ncompatible_printers_condition = nozzle_diameter[0]==0.4\n";
    std::ofstream(root / "filament" / "My old.ini") <<
        "inherits = Old PLA\nfilament_type = PLA\ncompatible_printers_condition = nozzle_diameter[0]==0.4\n";
    for (const auto &[child, parent] : {std::make_pair("My child", "My PLA"), std::make_pair("My old child", "My old"),
             std::make_pair("Other user", "Other PLA @O"), std::make_pair("Other child", "Other user")})
        std::ofstream(root / "filament" / (std::string(child) + ".ini")) << "inherits = " << parent <<
            "\nfilament_type = PLA\ncompatible_printers_condition = nozzle_diameter[0]==0.4\n";
    for (const auto &[child, parent] : {std::make_pair("My dual", "Dual 0.4"), std::make_pair("My dual child", "My dual")})
        std::ofstream(root / "printer" / (std::string(child) + ".ini")) << "inherits = " << parent <<
            "\nprinter_model = Dual\nprinter_variant = 0.4\nnozzle_diameter = 0.4,0.6\nextruder_offset = 0x0,0x0\n"
            "default_filament_profile = PLA @A\n";
}

void load_bundle(PresetBundle &bundle, const std::filesystem::path &root)
{
    const std::pair<PresetsConfigSubstitutions, std::string> loaded = bundle.load_system_presets_from(
        (root / "vendor").u8string(), ForwardCompatibilitySubstitutionRule::EnableSilent);
    REQUIRE(loaded.second.empty());
    PresetsConfigSubstitutions substitutions;
    bundle.filaments.load_presets(root.u8string(), "filament", substitutions, ForwardCompatibilitySubstitutionRule::EnableSilent);
    bundle.printers.load_presets(root.u8string(), "printer", substitutions, ForwardCompatibilitySubstitutionRule::EnableSilent);
}

void select(PresetBundle &bundle, const std::string &printer, const std::string &print)
{
    REQUIRE(bundle.printers.select_preset_by_name(printer, true));
    REQUIRE(bundle.fff_prints.select_preset_by_name(print, true));
    bundle.extruders_filaments.clear();
    bundle.update_multi_material_filament_presets();
    bundle.update_compatible(PresetSelectCompatibleType::Never);
}

std::vector<std::string> names_of(const PresetCollection &collection)
{
    std::vector<std::string> names;
    for (const Preset &preset : collection.get_presets())
        names.push_back(preset.name);
    return names;
}

std::vector<std::string> expected_names(const PresetCollection &source, const std::set<std::string> &kept)
{
    std::vector<std::string> names;
    for (size_t index = 0; index < source.get_presets().size(); ++index)
        if (index < source.num_default_presets() || kept.count(source.get_presets()[index].name) != 0)
            names.push_back(source.get_presets()[index].name);
    return names;
}

const Filament &slot_filament(const ExtruderFilaments &slot, const std::string &name)
{
    for (const Filament &filament : slot())
        if (filament.preset->name == name)
            return filament;
    FAIL("Filament " << name << " is missing from the extruder slot.");
    throw std::logic_error("unreachable");
}

std::string vendor_id(const PresetCollection &collection, const std::string &name)
{
    const Preset *preset = collection.find_preset(name, false, false);
    REQUIRE(preset != nullptr);
    const VendorProfile *vendor = collection.get_preset_with_vendor_profile(*preset).vendor;
    return vendor == nullptr ? std::string() : vendor->id;
}

std::set<std::string> extruder_compatible_names(const PresetBundle &source, const std::string &printer, const std::string &print)
{
    PresetBundle full = source;
    select(full, printer, print);
    std::set<std::string> names;
    for (const ExtruderFilaments &slot : full.extruders_filaments)
        for (const Filament &filament : slot())
            if (filament.is_compatible && !filament.preset->is_default)
                names.insert(filament.preset->name);
    return names;
}

CatalogState catalog_state(const PresetBundle &bundle)
{
    std::vector<std::tuple<std::string, bool, bool>> flags;
    for (const PresetCollection *collection : {&bundle.fff_prints, &bundle.filaments,
             static_cast<const PresetCollection *>(&bundle.printers)})
        for (const Preset &preset : collection->get_presets())
            flags.emplace_back(preset.name, preset.is_compatible, preset.is_visible);
    std::vector<std::string> slots;
    for (const ExtruderFilaments &slot : bundle.extruders_filaments)
        slots.push_back(slot.get_selected_preset_name());
    return {flags, bundle.fff_prints.get_selected_preset_name(), bundle.filaments.get_selected_preset_name(),
        bundle.printers.get_selected_preset_name(), bundle.filaments.get_edited_preset().name, slots};
}

} // namespace

TEST_CASE("Reduced preset collections keep named presets with ancestors and renamed targets", "[preset_subset]")
{
    TemporaryRoot root;
    write_profiles(root.path());
    PresetBundle bundle;
    load_bundle(bundle, root.path());
    select(bundle, "Single 0.4", "Print 0.4");
    REQUIRE(bundle.filaments.select_preset_by_name("PLA @A", true));
    PresetBundle::PresetSubset names;
    names.printers = {"My dual child"};
    names.filaments = {"My child", "My old child", "Old PLA", "Unknown"};
    const PresetBundle reduced = bundle.subset(names);
    REQUIRE(bundle.filaments.first_visible().name == "ASA @none");
    CHECK(names_of(reduced.filaments) == expected_names(bundle.filaments,
        {"My child", "My PLA", "PLA @A", "My old child", "My old", "Renamed PLA", "ASA @none"}));
    CHECK(names_of(reduced.fff_prints) == expected_names(bundle.fff_prints, {"Print 0.4"}));
    REQUIRE(bundle.printers.first_visible().name == "Dual 0.4");
    CHECK(names_of(reduced.printers) == expected_names(bundle.printers,
        {"My dual child", "My dual", "Dual 0.4", "Single 0.4"}));
    const Preset *pla = reduced.filaments.find_preset("PLA @A", false, false);
    REQUIRE(pla != nullptr);
    CHECK(pla->is_system);
    CHECK(pla->alias == "PLA");
    const Preset *renamed = reduced.filaments.find_preset("Renamed PLA", false, false);
    REQUIRE(renamed != nullptr);
    CHECK(renamed->renamed_from == std::vector<std::string>{"Old PLA"});
    const Preset *old_user = reduced.filaments.find_preset("My old", false, false);
    REQUIRE(old_user != nullptr);
    CHECK(reduced.filaments.get_preset_parent(*old_user) == renamed);
    CHECK(vendor_id(reduced.filaments, "My old child") == "SubsetVendor");
    CHECK(vendor_id(reduced.printers, "My dual child") == "SubsetVendor");
    for (const std::pair<std::string, std::string> &alias : const_cast<PresetBundle &>(reduced).filaments.map_alias_to_profile_name())
        CHECK(reduced.filaments.find_preset(alias.second, false, false) != nullptr);
    CHECK(reduced.filaments.get_selected_preset_name() == "PLA @A");
    CHECK(reduced.fff_prints.get_selected_preset_name() == "Print 0.4");
    CHECK(reduced.printers.get_selected_preset_name() == "Single 0.4");
    CHECK(reduced.extruders_filaments.empty());
    REQUIRE(pla->vendor != nullptr);
    CHECK(pla->vendor == &reduced.vendors.at(pla->vendor->id));
    CHECK(reduced.filaments.get_edited_preset().vendor == pla->vendor);
    CHECK(reduced.vendors.size() == 1);
    PresetBundle copy;
    {
        const PresetBundle temporary = bundle.subset(names);
        copy = temporary;
    }
    const VendorProfile *copied_vendor = copy.filaments.find_preset("PLA @A", false, false)->vendor;
    REQUIRE(copied_vendor != nullptr);
    CHECK(copied_vendor == &copy.vendors.at(copied_vendor->id));
    CHECK(copy.filaments.get_edited_preset().vendor == copied_vendor);
}

TEST_CASE("Compatible filament names follow the per-extruder ExtruderFilaments evaluation", "[preset_subset]")
{
    TemporaryRoot root;
    write_profiles(root.path());
    PresetBundle bundle;
    load_bundle(bundle, root.path());
    select(bundle, "Single 0.4", "Print 0.4");
    for (const std::pair<const char *, const char *> &selection :
             {std::make_pair("Single 0.4", "Print 0.4"), std::make_pair("Dual 0.4", "Print 0.4"),
              std::make_pair("Dual 0.4", "Print 0.6"), std::make_pair("Twin 0.4", "Print 0.4"),
              std::make_pair("MMU 0.4", "Print 0.4")}) {
        CAPTURE(selection.first, selection.second);
        const Preset *printer = bundle.printers.find_preset(selection.first, false, false);
        const Preset *print = bundle.fff_prints.find_preset(selection.second, false, false);
        REQUIRE(printer != nullptr);
        REQUIRE(print != nullptr);
        const std::set<std::string> names = bundle.compatible_filament_names(*printer, print);
        CHECK(names == extruder_compatible_names(bundle, selection.first, selection.second));
        CHECK(names.count("ASA @none") == 0);
        CHECK(names.count("TPU @Template") == 1);
        const size_t dual = std::string(selection.first) == "Dual 0.4" ? 1 : 0;
        CHECK(names.count("PETG @wide") == dual);
        CHECK(names.count("PLA @Template") == dual);
    }
    const Preset &single = *bundle.printers.find_preset("Single 0.4", false, false);
    const std::set<std::string> single_names = bundle.compatible_filament_names(single, nullptr);
    CHECK(single_names == std::set<std::string>{"My PLA", "My child", "My old", "My old child", "PLA @A",
        "Renamed PLA", "TPU @Template"});
}

TEST_CASE("Reduced bundles keep the vendor identity of multi-level inheritance chains", "[preset_subset]")
{
    TemporaryRoot root;
    write_profiles(root.path());
    PresetBundle bundle;
    load_bundle(bundle, root.path());
    select(bundle, "Single 0.4", "Print 0.4");
    const std::vector<std::string> chained = {"My child", "My old child", "Other child"};
    for (const char *printer : {"Single 0.4", "My dual child", "Other 0.4"}) {
        CAPTURE(printer);
        PresetBundle::PresetSubset names;
        names.printers = {printer};
        names.prints = {"Print 0.4"};
        names.filaments = {chained.begin(), chained.end()};
        PresetBundle reduced = bundle.subset(names);
        PresetBundle full = bundle;
        CHECK(vendor_id(reduced.printers, printer) == vendor_id(full.printers, printer));
        for (const std::string &name : chained) {
            CAPTURE(name);
            CHECK(vendor_id(reduced.filaments, name) == vendor_id(full.filaments, name));
            CHECK_FALSE(vendor_id(full.filaments, name).empty());
        }
        select(reduced, printer, "Print 0.4");
        select(full, printer, "Print 0.4");
        REQUIRE(reduced.extruders_filaments.size() == full.extruders_filaments.size());
        for (size_t slot = 0; slot < full.extruders_filaments.size(); ++slot)
            for (const std::string &name : chained) {
                CAPTURE(slot, name);
                CHECK(slot_filament(reduced.extruders_filaments[slot], name).is_compatible ==
                    slot_filament(full.extruders_filaments[slot], name).is_compatible);
            }
        const bool other = std::string(printer) == "Other 0.4";
        CHECK(slot_filament(reduced.extruders_filaments[0], "Other child").is_compatible == other);
        CHECK(slot_filament(reduced.extruders_filaments[0], "My child").is_compatible == !other);
    }
}

TEST_CASE("A reduced bundle selects filaments automatically as the full bundle", "[preset_subset]")
{
    TemporaryRoot root;
    write_profiles(root.path());
    PresetBundle bundle;
    load_bundle(bundle, root.path());
    select(bundle, "Single 0.4", "Print 0.4");
    REQUIRE(bundle.filaments.select_preset_by_name("ASA @none", true));
    size_t reselected = 0;
    for (const auto &[printer, print] : {std::make_pair("Single 0.4", "Print 0.4"),
             std::make_pair("Dual 0.4", "Print 0.4"), std::make_pair("My dual child", "Print 0.4"),
             std::make_pair("Wide 0.6", "Print 0.6"), std::make_pair("Other 0.4", "Print 0.4")})
        for (const char *start : {"", "ASA @none", "PETG @wide", "My child", "Other child"})
            for (const PresetSelectCompatibleType mode :
                     {PresetSelectCompatibleType::Always, PresetSelectCompatibleType::OnlyIfWasCompatible}) {
                CAPTURE(printer, print, start, int(mode));
                const Preset &printer_preset = *bundle.printers.find_preset(printer, false, false);
                PresetBundle::PresetSubset names;
                names.printers = {printer};
                names.prints = {print};
                names.filaments = bundle.compatible_filament_names(printer_preset,
                    bundle.fff_prints.find_preset(print, false, false));
                if (*start != '\0')
                    names.filaments.insert(start);
                PresetBundle reduced = bundle.subset(names);
                PresetBundle full = bundle;
                std::vector<std::string> initial;
                std::vector<std::vector<std::string>> selected;
                for (PresetBundle *candidate : {&full, &reduced}) {
                    REQUIRE(candidate->printers.select_preset_by_name(printer, true));
                    REQUIRE(candidate->fff_prints.select_preset_by_name(print, true));
                    candidate->extruders_filaments.clear();
                    candidate->update_multi_material_filament_presets();
                    for (ExtruderFilaments &slot : candidate->extruders_filaments)
                        if (*start != '\0')
                            slot.select_filament(start);
                    std::vector<std::string> slots;
                    for (const ExtruderFilaments &slot : candidate->extruders_filaments)
                        slots.push_back(slot.get_selected_preset_name());
                    if (candidate == &full)
                        initial = slots;
                    candidate->update_compatible(PresetSelectCompatibleType::Never, mode);
                    slots.clear();
                    for (const ExtruderFilaments &slot : candidate->extruders_filaments)
                        slots.push_back(slot.get_selected_preset_name() +
                            (slot.get_selected_filament()->is_compatible ? "" : " (incompatible)"));
                    selected.push_back(slots);
                }
                CHECK(selected[0] == selected[1]);
                for (size_t slot = 0; slot < initial.size(); ++slot)
                    if (selected[0][slot] != initial[slot])
                        ++reselected;
            }
    CHECK(reselected > 10);
}

TEST_CASE("Compatible filament names leave the catalog bundle unchanged", "[preset_subset]")
{
    TemporaryRoot root;
    write_profiles(root.path());
    PresetBundle bundle;
    load_bundle(bundle, root.path());
    select(bundle, "Dual 0.4", "Print 0.6");
    bundle.set_filament_preset(1, "PETG @wide");
    bundle.update_compatible(PresetSelectCompatibleType::Never);
    const CatalogState before = catalog_state(bundle);
    const PresetBundle &catalog = bundle;
    for (const char *printer_name : {"Single 0.4", "Dual 0.4"}) {
        const Preset &printer = *catalog.printers.find_preset(printer_name, false, false);
        for (const char *print_name : {"Print 0.4", "Print 0.6"})
            CHECK_FALSE(catalog.compatible_filament_names(printer,
                catalog.fff_prints.find_preset(print_name, false, false)).empty());
        CHECK_FALSE(catalog.compatible_filament_names(printer, nullptr).empty());
    }
    CHECK(catalog_state(bundle) == before);
}

TEST_CASE("A reduced bundle gives the same configuration as the full bundle", "[preset_subset]")
{
    TemporaryRoot root;
    write_profiles(root.path());
    PresetBundle bundle;
    load_bundle(bundle, root.path());
    select(bundle, "Single 0.4", "Print 0.4");
    const Preset &dual = *bundle.printers.find_preset("Dual 0.4", false, false);
    const Preset *print = bundle.fff_prints.find_preset("Print 0.6", false, false);
    PresetBundle::PresetSubset names;
    names.printers = {"Dual 0.4"};
    names.prints = {"Print 0.6"};
    names.filaments = bundle.compatible_filament_names(dual, print);
    PresetBundle reduced = bundle.subset(names);
    PresetBundle full = bundle;
    for (PresetBundle *candidate : {&reduced, &full}) {
        select(*candidate, "Dual 0.4", "Print 0.6");
        candidate->set_filament_preset(0, "My PLA");
        candidate->set_filament_preset(1, "PETG @wide");
        candidate->update_compatible(PresetSelectCompatibleType::Never);
    }
    const DynamicPrintConfig reduced_config = reduced.full_config();
    const DynamicPrintConfig full_config = full.full_config();
    CHECK(reduced_config.diff(full_config).empty());
    CHECK(reduced_config.opt_serialize("filament_settings_id") == full_config.opt_serialize("filament_settings_id"));
    for (size_t slot = 0; slot < 2; ++slot) {
        CAPTURE(slot);
        CHECK(reduced.extruders_filaments[slot].get_selected_filament()->is_compatible ==
            full.extruders_filaments[slot].get_selected_filament()->is_compatible);
    }
}
