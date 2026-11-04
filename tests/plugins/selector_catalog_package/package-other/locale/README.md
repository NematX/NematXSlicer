# Translation directory for a different package

This directory represents resources belonging to package-other, a separate
installed-directory model used by the tests. A translation domain names a
catalogue of translated text; the application also records the directory of
the package that registered it.

The tests first call orchestrator_register_translation_catalog() while
package-other is the current package, then register a selector description from
package-alpha using that domain name. A selector is a setting choosing a plugin
from alternatives for one processing stage. The other package's catalogue does
not authorize translating this selector: its option and retained group text
must be marked for verbatim display, and a non-fatal warning is recorded.
An ordinary plugin option using that unavailable package/domain pair is rejected.
