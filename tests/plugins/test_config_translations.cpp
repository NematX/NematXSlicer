#include <catch2/catch.hpp>

#include "libslic3r/Config/ConfigDef.hpp"

using namespace Slic3r;

// The GUI fills translated labels into the configuration definitions after the
// language is loaded. The fill recomputes everything from the raw fields, so it
// can run again after a language change and never touches the raw text used for
// parsing and serialization.
TEST_CASE("Config definitions store translated labels", "[config]")
{
    ConfigDef def;
    ConfigOptionDef *plain = def.add("plain_opt", coBool);
    plain->label      = "Plain label";
    plain->full_label = "Plain full label";
    plain->tooltip    = "Plain tooltip";

    ConfigOptionDef *plugin = def.add("plugin_opt", coInt);
    plugin->label              = "Plugin label";
    plugin->translation_domain = "test_plugin_domain";

    ConfigOptionDef *enum_opt = def.add("enum_opt", coEnum);
    enum_opt->label = "Enum option";
    enum_opt->set_enum_values(ConfigOptionDef::GUIType::select_close,
                              std::vector<std::pair<std::string, std::string>>{{"a", "Label A"}, {"b", "Label B"}});
    REQUIRE(enum_opt->enum_def.get() != nullptr);

    ConfigOptionDef *empty_opt = def.add("empty_opt", coBool);

    std::vector<std::pair<std::string, std::string>> calls;
    const auto translator = [&calls](const std::string &text, const std::string &domain) -> std::string {
        calls.emplace_back(text, domain);
        if (domain == "test_plugin_domain")
            return "FR " + text;
        if (text == "Plain label")
            return "FR Plain label";
        if (text == "Label A")
            return "FR Label A";
        // No translation available: the raw text is returned unchanged.
        return text;
    };

    const size_t updated = def.update_translations(translator);
    CHECK(updated == 3);

    // Translated text is stored next to the untouched original.
    CHECK(plain->label_translated == "FR Plain label");
    CHECK(plain->label == "Plain label");
    CHECK(plain->full_label_translated.empty());
    CHECK(plain->tooltip_translated.empty());
    CHECK(plain->translated_label() == "FR Plain label");
    CHECK(plain->translated_full_label() == "Plain full label");
    CHECK(plain->translated_tooltip() == "Plain tooltip");

    // The definition domain is forwarded to the translator.
    CHECK(plugin->label_translated == "FR Plugin label");
    bool plugin_domain_seen = false;
    for (const std::pair<std::string, std::string> &call : calls)
        if (call.second == "test_plugin_domain")
            plugin_domain_seen = true;
    CHECK(plugin_domain_seen);

    // Enum labels are translated per entry; untranslated ones fall back to raw
    // and the serialized values are never translated.
    CHECK(enum_opt->enum_def->translated_label(0) == "FR Label A");
    CHECK(enum_opt->enum_def->translated_label(1) == "Label B");
    CHECK(enum_opt->enum_def->labels()[0] == "Label A");
    CHECK(enum_opt->enum_def->values()[0] == "a");

    // Empty text is never sent to the translator and stays empty.
    CHECK(empty_opt->label_translated.empty());
    bool empty_text_seen = false;
    for (const std::pair<std::string, std::string> &call : calls)
        if (call.first.empty())
            empty_text_seen = true;
    CHECK_FALSE(empty_text_seen);

    // Recomputing from the originals is idempotent.
    CHECK(def.update_translations(translator) == 0);
    CHECK(plain->label_translated == "FR Plain label");
    CHECK(enum_opt->enum_def->translated_label(0) == "FR Label A");

    // A missing translation stores nothing and keeps the raw field.
    ConfigDef other;
    ConfigOptionDef *missing = other.add("missing_opt", coBool);
    missing->label = "Untranslated";
    CHECK(other.update_translations([](const std::string &text, const std::string &) { return text; }) == 0);
    CHECK(missing->label_translated.empty());
    CHECK(missing->translated_label() == "Untranslated");
}

// Fill stored translations, rebuild choices through public setters, then read current labels without translating again.
// Test-local descriptions own all data. Rebuilding or clearing must discard translations of previous labels.
TEST_CASE("Rebuilt enum definitions discard translated labels", "[config][translation]")
{
    ConfigDef definitions;
    ConfigOptionDef *option = definitions.add("rebuilt_enum", coEnum);
    option->set_enum_as_closed_for_scripted_enum({{"a", "Label A"}, {"b", "Label B"}});
    REQUIRE(definitions.update_translations([](const std::string &text, const std::string &) {
        return "Translated " + text;
    }) == 1);
    REQUIRE(option->enum_def->translated_label(0) == "Translated Label A");
    SECTION("Growing a scripted enum") {
        option->set_enum_as_closed_for_scripted_enum({{"x", "New X"}, {"y", "New Y"}, {"z", "New Z"}});
    }
    SECTION("Shrinking a scripted enum") {
        option->set_enum_as_closed_for_scripted_enum({{"x", "New X"}});
    }
    SECTION("Replacing with unlabelled values") {
        option->set_enum_values(ConfigOptionDef::GUIType::select_close, std::vector<std::string>{"x", "y", "z"});
    }
    SECTION("Replacing with explicit labels only") {
        option->set_enum_labels(ConfigOptionDef::GUIType::select_close, {"New X", "New Y", "New Z"});
    }
    SECTION("Clearing enum metadata") {
        option->enum_def->clear();
        CHECK(option->enum_def->translated_labels().empty());
    }
    CHECK(option->enum_def->translated_labels() == option->enum_def->labels());
    for (size_t i = 0; i < option->enum_def->labels().size(); ++i)
        CHECK(option->enum_def->translated_label(int(i)) == option->enum_def->label(int(i)));
}

// Refresh a test-local choice list without explicit labels using a function that would translate every input.
// Stored values must remain unchanged; window code performs display-only lookup for values not marked verbatim.
TEST_CASE("Enums without explicit labels keep serialized values outside the translation cache", "[config][translation]")
{
    ConfigDef definitions;
    ConfigOptionDef *option = definitions.add("unlabelled_enum", coEnum);
    option->set_enum_values(ConfigOptionDef::GUIType::select_close, std::vector<std::string>{"filament", "tool"});
    SECTION("Host enum") { option->untranslated = false; }
    SECTION("Verbatim enum") { option->untranslated = true; }
    size_t calls = 0;
    CHECK(definitions.update_translations([&calls](const std::string &text, const std::string &) {
        ++calls;
        return "Translated " + text;
    }) == 0);
    CHECK(calls == 0);
    CHECK_FALSE(option->enum_def->has_labels());
    CHECK(option->enum_def->translated_labels() == option->enum_def->values());
    CHECK(option->enum_def->translated_label(0) == "filament");
}
