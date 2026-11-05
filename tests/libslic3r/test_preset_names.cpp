///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include <catch2/catch.hpp>

#include "libslic3r/ContainerUtils.hpp"
#include "libslic3r/Config/Preset.hpp"

/*
These tests check names assigned to profiles loaded from external files.
Filename-shaped prefixes are reduced instead of nesting on every reload.
Other parenthesized text is kept. A collision number distinguishes names in use.
The extension cases cover the list recognized by external_preset_name().

Example tables and repeated reloads

Operation notes:
- external_preset_name: combine the file, profile name and collision number
- Compare the resulting name with the expected string

Representative direct calls (branches are not a timeline):

    TEST_CASE
        `-- Slic3r::external_preset_name()
*/

TEST_CASE("External preset names preserve source names without nesting", "[preset_names]")
{
    struct NameCase {
        const char *file_name;
        const char *original_name;
        size_t index;
        const char *expected;
    };
    const NameCase cases[] = {
        {"original.3mf", "", 0, "original.3mf"},
        {"original.3mf", "rest-print", 0, "original.3mf (rest-print)"},
        {"original.3mf", "original.3mf", 0, "original.3mf"},
        {"original.3mf", "original.3mf (original.3mf (original.3mf))", 0, "original.3mf"},
        {"original.3mf", "original.3mf (rest-print)", 0, "original.3mf"},
        {"saved.3mf", "original.3mf (rest-print)", 0, "saved.3mf (original.3mf)"},
        {"saved.3mf", "original.3mf", 0, "saved.3mf (original.3mf)"},
        {"c.3mf", "b.3mf (a.3mf)", 0, "c.3mf (b.3mf)"},
        {"original.3mf", "original.3mf", 1, "original.3mf (1)"},
        {"saved.3mf", "original.3mf (x)", 1, "saved.3mf (original.3mf-1)"},
        {"proj.3mf", "Prusament PLA (0.6 nozzle)", 0, "proj.3mf (Prusament PLA (0.6 nozzle))"},
        {"proj.3mf", "proj.3mf (Prusament PLA (0.6 nozzle))", 0, "proj.3mf"},
        {"other.3mf", "proj.3mf (Prusament PLA (0.6 nozzle))", 0, "other.3mf (proj.3mf)"},
        {"proj.3mf", "Generic PLA @MK4 0.4 (custom)", 0, "proj.3mf (Generic PLA @MK4 0.4 (custom))"},
        {"part (1).3mf", "part (1).3mf", 0, "part (1).3mf"},
        {"part (1).3mf", "part (1).3mf (rest-print)", 0, "part (1).3mf"},
        {"new.3mf", "other (1).3mf (x)", 0, "new.3mf (other (1).3mf)"},
        {"config.ini", "config.ini (x)", 0, "config.ini"},
        {"print.gcode", "proj.3MF (x)", 0, "print.gcode (proj.3MF)"},
        {"PROJ.3MF", "proj.3mf (x)", 0, "PROJ.3MF (proj.3mf)"},
        {"saved.3mf", "", 2, "saved.3mf (2)"},
    };
    for (const NameCase &test : cases) {
        CAPTURE(test.file_name, test.original_name, test.index);
        CHECK(Slic3r::external_preset_name(test.file_name, test.original_name, test.index) == test.expected);
    }
}

TEST_CASE("External preset names recognize all external source extensions", "[preset_names]")
{
    for (const char *extension : {".3mf", ".amf", ".zip", ".ini", ".gcode", ".gco", ".g", ".ngc", ".bgcode"}) {
        const std::string source = std::string("source") + extension;
        CAPTURE(source);
        CHECK(Slic3r::external_preset_name("saved.3mf", source + " (preset)", 0) == "saved.3mf (" + source + ")");
    }
}

TEST_CASE("External preset names stabilize on repeated loads of the same file", "[preset_names]")
{
    std::string original = "rest-print";
    for (size_t index = 0; index < 5; ++index) {
        CAPTURE(index);
        original = Slic3r::external_preset_name("original.3mf", original, 0);
        CHECK(original == (index == 0 ? "original.3mf (rest-print)" : "original.3mf"));
    }
}
