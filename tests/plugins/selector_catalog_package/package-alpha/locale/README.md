# Translation directory for selector tests

This directory represents translation resources belonging to package-alpha, an
installed-directory model used by the tests. A translation domain is the name
identifying a catalogue of translated text; registering it records which package
directory is allowed to supply that text.

The tests first register this directory through
orchestrator_register_translation_catalog(), then create a selector, a setting
choosing a plugin from alternatives for one processing stage. They copy its
description into ConfigDef, a local collection of setting descriptions, and call
update_translations() with a deterministic translation function. No catalogue
binary or graphical interface is needed to check the forwarded domain.
