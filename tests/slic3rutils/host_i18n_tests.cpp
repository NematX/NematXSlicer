// Headless catalogue routing checks: gettext domains identify the catalogue owning a message.
#include "slic3r/GUI/I18N.hpp"
#include <wx/init.h>
#include <wx/translation.h>
#include <iostream>

// Load local catalogues from argv[1] and check all domain-aware helpers; wx owns the installed translations and failures return nonzero without starting a GUI.
int main(int argc, char **argv)
{
    wxInitializer init;
    if (!init.IsOk() || argc != 2) return 2;
    wxFileTranslationsLoader::AddCatalogLookupPathPrefix(wxString::FromUTF8(argv[1]));
    auto *translations = new wxTranslations;
    wxTranslations::Set(translations);
    translations->SetLanguage("fr");
    if (!translations->AddCatalog("thirdparty") || !translations->AddCatalog(SLIC3R_APP_KEY)) return 3;
    int failures = 0;
    for (const std::string domain : {"Slic3r", SLIC3R_APP_KEY, "", "thirdparty"}) {
        const wxString prefix = domain == "thirdparty" ? "third-" : "app-";
        // Compare borrowed strings and count mismatches, printing diagnostics without altering translations.
        const auto check = [&](const wxString &actual, const wxString &expected) {
            if (actual != expected) { std::cerr << domain << ": " << actual.ToUTF8() << " != " << expected.ToUTF8() << '\n'; ++failures; }
        };
        using namespace Slic3r::GUI::I18N;
        check(translate_in_domain("item", domain), prefix + "one");
        check(translate_in_domain("item", "items", 1, domain), prefix + "one");
        check(translate_in_domain("item", "items", 2, domain), prefix + "many");
        check(translate_in_domain_with_context("item", "menu", domain), prefix + "context-one");
        check(translate_in_domain_with_context("item", "items", 1, "menu", domain), prefix + "context-one");
        check(translate_in_domain_with_context("item", "items", 2, "menu", domain), prefix + "context-many");
    }
    if (Slic3r::GUI::I18N::catalogue_domain("") != wxEmptyString ||
        Slic3r::GUI::I18N::catalogue_domain("thirdparty") != "thirdparty") ++failures;
    std::cout << "24 translation lookups; failures=" << failures << '\n';
    return failures ? 1 : 0;
}
