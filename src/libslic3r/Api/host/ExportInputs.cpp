///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "ExportInputs.hpp"

#include <cctype>
#include <istream>
#include <memory>
#include <ostream>
#include <string>

#include <boost/algorithm/string/case_conv.hpp>

#include "libslic3r/Exception.hpp"

/*
Command-line requester of the inputs of an export
=================================================

make_stream_export_inputs() returns an ExportInputsCallback that talks through two
streams, so the command line passes its standard input and output and a test passes
string streams. Each definition is asked in turn: the label and the tooltip, then the
proposed value between brackets, then one line is read. An empty line accepts the
proposed value. Any other line is converted by the option type of the definition; a
line that the type refuses is reported and the same question is asked again. A closed
input cannot be answered, so it throws instead of guessing a value. The command line
has no way to cancel: the callback always returns true.

String settings are taken as typed, because their serialized form would read the
backslashes of a Windows path as escape sequences. Every other type goes through
ConfigOption::deserialize(); booleans also accept yes/no/true/false.

Representative direct calls (branches are not a timeline):

    ask_form()
        `-- ask_definition()

    ask_definition()
        |-- proposed_text()
        |-- std::getline()
        |-- line.pop_back()
        `-- store_answer()

    store_answer()
        |-- definition.create_empty_option()
        |-- boost::to_lower_copy()
        |-- option->deserialize()
        |-- values.set_key_value()
        `-- option.release()
*/

namespace Slic3r {

// Ask the whole form, one definition after the other; return true.
static bool ask_form(std::istream &in, std::ostream &out, ExportInputForm &form);
// Ask one definition until a value is accepted; throw when the input closes.
static void ask_definition(std::istream &in, std::ostream &out, const ConfigOptionDef &definition, DynamicConfig &values);
// Return the text proposed to the user for an option (strings as stored, other types serialized).
static std::string proposed_text(const ConfigOptionDef &definition, const ConfigOption &option);
// Convert an answer into a new option of the type of the definition and store it under its key; false when refused. A string
// setting keeps the answer exactly as typed (no unescaping); the other types are read by their option type.
static bool store_answer(const ConfigOptionDef &definition, std::string text, DynamicConfig &values);

ExportInputsCallback make_stream_export_inputs(std::istream &in, std::ostream &out)
{
    return [&in, &out](ExportInputForm &form) { return ask_form(in, out, form); };
}

static bool ask_form(std::istream &in, std::ostream &out, ExportInputForm &form)
{
    for (const ConfigOptionDef &definition : form.definitions)
        ask_definition(in, out, definition, form.values);
    return true;
}

static void ask_definition(std::istream &in, std::ostream &out, const ConfigOptionDef &definition, DynamicConfig &values)
{
    const std::string label = definition.get_full_label().empty() ? definition.opt_key : definition.get_full_label();
    const ConfigOption *proposed = values.option(definition.opt_key);
    const std::string proposed_value = proposed == nullptr ? std::string() : proposed_text(definition, *proposed);
    for (;;) {
        out << label;
        if (!definition.tooltip.empty())
            out << " (" << definition.tooltip << ")";
        out << " [" << proposed_value << "]: " << std::flush;

        std::string line;
        if (!std::getline(in, line))
            throw RuntimeError("The standard input closed before the value of " + definition.opt_key + " was given.");
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        // An empty line keeps the proposed value, which is already in `values`.
        if (line.empty())
            return;
        if (store_answer(definition, line, values))
            return;
        out << "The value given for " << definition.opt_key << " is refused.\n";
    }
}

static std::string proposed_text(const ConfigOptionDef &definition, const ConfigOption &option)
{
    if (definition.type == coString)
        return static_cast<const ConfigOptionString &>(option).value;
    return option.serialize();
}

static bool store_answer(const ConfigOptionDef &definition, std::string text, DynamicConfig &values)
{
    std::unique_ptr<ConfigOption> option(definition.create_empty_option());
    if (definition.type == coString) {
        static_cast<ConfigOptionString &>(*option).value = std::move(text);
    } else {
        if (definition.type == coBool) {
            const std::string word = boost::to_lower_copy(text);
            if (word == "y" || word == "yes" || word == "true")
                text = "1";
            else if (word == "n" || word == "no" || word == "false")
                text = "0";
        }
        if (!option->deserialize(text))
            return false;
    }
    values.set_key_value(definition.opt_key, option.release());
    return true;
}

} // namespace Slic3r
