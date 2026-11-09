///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/ Copyright (c) Prusa Research 2018 - 2023 Oleksandra Iushchenko @YuSanka, Vojtěch Bubník @bubnikv, Vojtěch Král @vojtechkral
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef _
#define _(s)	Slic3r::GUI::I18N::translate((s))
#define _L(s)	Slic3r::GUI::I18N::translate((s))
#define _u8L(s)     Slic3r::GUI::I18N::translate_utf8((s))
#endif /* _ */

#ifndef _CTX
#define _CTX(s, ctx)	  Slic3r::GUI::I18N::translate((s), (ctx))
#define _CTX_utf8(s, ctx) Slic3r::GUI::I18N::translate_utf8((s), (ctx))
#endif /* _ */

#ifndef L
// !!! If you needed to translate some wxString,
// !!! please use _L(string)
// !!! _() - is a standard wxWidgets macro to translate
// !!! L() is used only for marking localizable string
// !!! It will be used in "xgettext" to create a Locating Message Catalog.
#define L(s) s
#endif /* L */

#ifndef L_CONTEXT
#define L_CONTEXT(s, context) s
#endif /* L */

#ifndef slic3r_GUI_I18N_hpp_
#define slic3r_GUI_I18N_hpp_

#include <string>

#include <wx/intl.h>
#include <wx/version.h>

#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r_version.h"

/* Translate GUI messages and option labels in the catalogue owned by their
   definition. The historical built-in domain Slic3r resolves to SLIC3R_APP_KEY;
   empty domains still use the default catalogue and plugin domains stay intact.
   Use translate_in_domain() for definition text and enum_display_label() for
   enum choices. UTF-8 wrappers return owned strings; wxWidgets stays in this
   GUI header rather than the core configuration API. */

namespace Slic3r { namespace GUI { 

namespace I18N {
	// Plugin-provided text carries its gettext domain with the owning
	// definition. Keeping the lookup here confines wxWidgets to the GUI layer.
	// Map only the historical built-in domain; preserve empty and third-party catalogue names.
	inline wxString catalogue_domain(const std::string &domain) {
		return wxString(domain == "Slic3r" ? SLIC3R_APP_KEY : domain.c_str(), wxConvUTF8);
	}
	// Return marked text verbatim; an empty catalogue name otherwise uses the default catalogue.
	inline wxString translate_in_domain(const char *s, const std::string &domain, bool untranslated = false) {
		if (untranslated) return wxString(s, wxConvUTF8);
		return domain.empty() ? wxGetTranslation(wxString(s, wxConvUTF8)) :
			wxGetTranslation(wxString(s, wxConvUTF8), catalogue_domain(domain));
	}
	// Translate borrowed UTF-8 text unless its setting requires verbatim display.
	inline wxString translate_in_domain(const std::string &s, const std::string &domain, bool untranslated = false) {
		return translate_in_domain(s.c_str(), domain, untranslated);
	}
	// Requires an enum option with def.enum_def set; defined labels return their stored translation, or the label itself if none is stored.
	// Otherwise return the value text translated in the option's translation domain, or as written when def.untranslated is set.
	inline wxString enum_display_label(const ConfigOptionDef &def, size_t index) {
		const ConfigOptionEnumDef &enum_def = *def.enum_def;
		return enum_def.has_labels() ? wxString::FromUTF8(enum_def.translated_label(index).c_str()) :
			translate_in_domain(enum_def.label(index), def.translation_domain, def.untranslated);
	}
	// Translate singular/plural UTF-8 text in its owning catalogue.
	inline wxString translate_in_domain(const char *s, const char *plural, unsigned int n, const std::string &domain) {
		return domain.empty() ? wxGetTranslation(wxString(s, wxConvUTF8), wxString(plural, wxConvUTF8), n) :
			wxGetTranslation(wxString(s, wxConvUTF8), wxString(plural, wxConvUTF8), n,
							 catalogue_domain(domain));
	}
	// Translate context-qualified UTF-8 text in its owning catalogue.
	inline wxString translate_in_domain_with_context(const char *s, const char *context, const std::string &domain) {
#if wxCHECK_VERSION(3, 1, 1)
		return wxGetTranslation(wxString(s, wxConvUTF8),
							catalogue_domain(domain),
							wxString(context, wxConvUTF8));
#else
		(void)context;
		return translate_in_domain(s, domain);
#endif
	}
	// Translate context-qualified plural text in its owning catalogue.
	inline wxString translate_in_domain_with_context(const char *s, const char *plural, unsigned int n,
	                                                 const char *context, const std::string &domain) {
#if wxCHECK_VERSION(3, 1, 1)
		return wxGetTranslation(wxString(s, wxConvUTF8), wxString(plural, wxConvUTF8), n,
							catalogue_domain(domain),
							wxString(context, wxConvUTF8));
#else
		(void)context;
		return translate_in_domain(s, plural, n, domain);
#endif
	}
	inline std::string translate_utf8_in_domain(const char *s, const std::string &domain) {
		return translate_in_domain(s, domain).ToUTF8().data();
	}
	// Return owned UTF-8 display text without catalogue lookup when marked verbatim.
	inline std::string translate_utf8_in_domain(const std::string &s, const std::string &domain, bool untranslated = false) {
		return translate_in_domain(s, domain, untranslated).ToUTF8().data();
	}
	inline std::string translate_utf8_in_domain(const char *s, const char *plural, unsigned int n, const std::string &domain) {
		return translate_in_domain(s, plural, n, domain).ToUTF8().data();
	}
	inline std::string translate_utf8_in_domain_with_context(const char *s, const char *context, const std::string &domain) {
		return translate_in_domain_with_context(s, context, domain).ToUTF8().data();
	}
	inline std::string translate_utf8_in_domain_with_context(const char *s, const char *plural, unsigned int n,
	                                                         const char *context, const std::string &domain) {
		return translate_in_domain_with_context(s, plural, n, context, domain).ToUTF8().data();
	}

	inline wxString translate(const char         *s) { return wxGetTranslation(wxString(s, wxConvUTF8)); }
	inline wxString translate(const wchar_t      *s) { return wxGetTranslation(s); }
	inline wxString translate(const std::string  &s) { return wxGetTranslation(wxString(s.c_str(), wxConvUTF8)); }
	inline wxString translate(const std::wstring &s) { return wxGetTranslation(s.c_str()); }
	inline wxString translate(const wxString     &s) { return wxGetTranslation(s); }

	inline wxString translate(const char         *s, const char 	    *plural, unsigned int n) { return wxGetTranslation(wxString(s, wxConvUTF8), wxString(plural, wxConvUTF8), n); }
	inline wxString translate(const wchar_t      *s, const wchar_t	    *plural, unsigned int n) { return wxGetTranslation(s, plural, n); }
	inline wxString translate(const std::string  &s, const std::string  &plural, unsigned int n) { return wxGetTranslation(wxString(s.c_str(), wxConvUTF8), wxString(plural.c_str(), wxConvUTF8), n); }
	inline wxString translate(const std::wstring &s, const std::wstring &plural, unsigned int n) { return wxGetTranslation(s.c_str(), plural.c_str(), n); }
	inline wxString translate(const wxString     &s, const wxString     &plural, unsigned int n) { return wxGetTranslation(s, plural, n); }

	inline std::string translate_utf8(const char         *s) { return wxGetTranslation(wxString(s, wxConvUTF8)).ToUTF8().data(); }
	inline std::string translate_utf8(const wchar_t      *s) { return wxGetTranslation(s).ToUTF8().data(); }
	inline std::string translate_utf8(const std::string  &s) { return wxGetTranslation(wxString(s.c_str(), wxConvUTF8)).ToUTF8().data(); }
	inline std::string translate_utf8(const std::wstring &s) { return wxGetTranslation(s.c_str()).ToUTF8().data(); }
	inline std::string translate_utf8(const wxString     &s) { return wxGetTranslation(s).ToUTF8().data(); }

	inline std::string translate_utf8(const char         *s, const char 	    *plural, unsigned int n) { return translate(s, plural, n).ToUTF8().data(); }
	inline std::string translate_utf8(const wchar_t      *s, const wchar_t	    *plural, unsigned int n) { return translate(s, plural, n).ToUTF8().data(); }
	inline std::string translate_utf8(const std::string  &s, const std::string  &plural, unsigned int n) { return translate(s, plural, n).ToUTF8().data(); }
	inline std::string translate_utf8(const std::wstring &s, const std::wstring &plural, unsigned int n) { return translate(s, plural, n).ToUTF8().data(); }
	inline std::string translate_utf8(const wxString     &s, const wxString     &plural, unsigned int n) { return translate(s, plural, n).ToUTF8().data(); }

#if wxCHECK_VERSION(3, 1, 1)
	#define _wxGetTranslation_ctx(S, CTX) wxGetTranslation((S), wxEmptyString, (CTX))
#else
	#define _wxGetTranslation_ctx(S, CTX) ((void)(CTX), wxGetTranslation((S)))
#endif

	inline wxString translate(const char *s, const char* ctx)         { return _wxGetTranslation_ctx(wxString(s, wxConvUTF8), ctx); }
	inline wxString translate(const wchar_t *s, const char* ctx)      { return _wxGetTranslation_ctx(s, ctx); }
	inline wxString translate(const std::string &s, const char* ctx)  { return _wxGetTranslation_ctx(wxString(s.c_str(), wxConvUTF8), ctx); }
	inline wxString translate(const std::wstring &s, const char* ctx) { return _wxGetTranslation_ctx(s.c_str(), ctx); }
	inline wxString translate(const wxString &s, const char* ctx)     { return _wxGetTranslation_ctx(s, ctx); }

	inline std::string translate_utf8(const char *s, const char* ctx)         { return _wxGetTranslation_ctx(wxString(s, wxConvUTF8), ctx).ToUTF8().data(); }
	inline std::string translate_utf8(const wchar_t *s, const char* ctx)      { return _wxGetTranslation_ctx(s, ctx).ToUTF8().data(); }
	inline std::string translate_utf8(const std::string &s, const char* ctx)  { return _wxGetTranslation_ctx(wxString(s.c_str(), wxConvUTF8), ctx).ToUTF8().data(); }
	inline std::string translate_utf8(const std::wstring &s, const char* ctx) { return _wxGetTranslation_ctx(s.c_str(), ctx).ToUTF8().data(); }
	inline std::string translate_utf8(const wxString &s, const char* ctx)     { return _wxGetTranslation_ctx(s, ctx).ToUTF8().data(); }

#undef _wxGetTranslation_ctx
} // namespace I18N

// Return translated std::string as a wxString
wxString	L_str(const std::string &str);

} // namespace GUI
} // namespace Slic3r

// Macro to function both as a marker for xgettext and to actually perform the translation.
#ifndef _L_PLURAL
//#define _L_PLURAL(s, plural, n) Slic3r::GUI::I18N::translate(s, plural, n) //doesn't want to compile
#define _L_PLURAL(s, plural, n) Slic3r::GUI::I18N::translate((s))
#endif /* L */

#endif /* slic3r_GUI_I18N_hpp_ */
