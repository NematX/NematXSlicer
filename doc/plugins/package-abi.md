# Package ABI Compatibility

Every installable plugin package contains a generated `version.ini`:

```ini
[plugin]
package_version = 1.0.0
slicer_version = 2.7.63.0-alpha+UNKNOWN

[abi]
slic3r_plugin_types.h = 4.1
slic3r_orchestrator.h = 3.5
slic3r_slicing_step.h = 2.4
slic3r_config_def.h = 1.6
slic3r_extrusion_entity.h = 1.0
steps/slic3r_step_post_perimeter.h = 1.0
steps/slic3r_step_gcode_export.h = 4.2
slic3r_gcode_thumbnail.h = 2.0
```

The slicer version records the build environment, not compatibility. It remains
part of archive and cache identifiers. ABI keys are literal, case-sensitive
paths relative to the public C API directory, not numeric contract IDs.

The vtable contract `slic3r_plugin_types.h` is mandatory. Required major versions
must match the host; required minor versions must not exceed it. `0.0` denotes
an unused contract. Unknown nonzero requirements, duplicate keys, invalid
versions and missing ABI declarations are incompatible.

The plugin-types 4.1 initialization callback returns an error identifier: zero
means success; nonzero results disable the instance with a diagnostic. The
orchestrator 3.5 interface supplies activation-independent registration lookup
and application-side logging. Package manifests and native version tables use
these public header macros; generated Python bindings derive their callback
return types and requirements from the same headers.
A contract whose structure is validated against its own `sizeof` (a vtable or
a result checked by `struct_size >= sizeof`) cannot gain a member in a minor
version: a table built before the change would be refused. Growing such a
structure is a major version. The G-code thumbnail service
(`slic3r_gcode_thumbnail.h`) is at 2.0 for this reason: its encoder table
gained the optional `encode_image` callback.

Plugin-types 4.1 includes the support-toolpath header identity
`SLIC3R_PLUGIN_API_STEP_SUPPORT_TOOLPATH` (58), naming
`steps/slic3r_step_support_toolpath.h`, and the exporter header identity
`SLIC3R_PLUGIN_API_STEP_GCODE_EXPORT` (59), naming
`steps/slic3r_step_gcode_export.h`. Its sized vtable layout is unchanged from 4.0.

Data-tree 5.0 removes the branch surface modifier and reserves bit 10. Support-toolpath
2.0 removes the branch role: branch bodies are sparse support, and the sheath algorithm
shares the sparse selector. Packages requiring data-tree 4.x or support-toolpath 1.x
must be rebuilt. Support-step remains 1.3; its publication payload is unchanged.

Data-tree 5.3 adds read-only access to the Print's default region configuration
without constructing the auxiliary object. Existing 5.x packages remain
compatible; the new query changes no structure layout or existing signature.

The G-code exporter step is at 4.2: QUERY requires a borrowed Print in its
payload, while the generic execution-context Print remains null. Exporter 3.x
packages are incompatible. QUERY reads settings from its configuration argument
and only computation-stable Print data (applied configuration and selected
services), never geometry or layers. It may acquire a service to inspect a
capability, but must stay quick, without rendering or writing, including on the
GUI thread. Compatibility cannot depend on request-only destination settings.
Compatible QUERY answers may append `action_label` (optional UTF-8 text without
ellipsis, translated by the GUI in the provider's domain) and
`supports_removable_media` (0 or 1). The host reads each field only when covered
by `struct_size`; a 4.0 provider retains an empty label and removable-media
support. The C++ provider view also defaults to removable-media support.
An empty label uses the GUI's ordinary export label. Incompatible answers do not
provide action information. The GUI adds the ellipsis and uses suitability only
for the FFF removable-media action, not as a restriction on chosen file paths.
Since 4.2 the EXPORT metadata ends with numeric statistics, appended flat after the
text `statistics` list and read only when `struct_size` covers the last of them
(`total_weight_g`); a 4.1 host structure carries none and the C++ view reads zeros.
They are the unrounded values from which the text is formatted, computed once with the
configuration copy of the export (the tools of its nozzle diameters): the estimated
time and the time of the Reader's layer 0 in seconds (nominal path time, no
acceleration; no silent-mode time exists), `layer_count` and `max_print_z` (millimeters,
without the Z offset) from the printing plan, and a per-tool array of length (mm),
volume (mm3) and weight (g) with their totals. The plan facts are zero until STEP_GCODE
has published them, also for a direct export of a reference it did not produce. The times and the material are valid if and
only if the numeric statistics exist and the text `statistics` list is not empty, else zero. All of it is host memory
borrowed for the synchronous EXPORT call. Plugins built for 4.1 keep working.

Native packages embed a pointer-free, versioned ASCII ABI record through
`slic3r_plugin_register_version.h`, using the same header-version macros as the
DLL export. Packaging extracts it without loading the library and fails if it
is absent, malformed, truncated or ambiguous. Keep this header last in the
export translation unit so all used C contracts are included.

The Python binding generator declares its complete C surface, including header
dependencies. The runtime package combines these requirements with its native
ones; repository Python packages conservatively require the whole generated
surface. This does **not** detect changes to the Python wrappers' own contracts.

Remote versions remain unchecked until downloaded. Cached incompatible versions
remain inspectable but cannot be selected for installation. Installation checks
the manifest before scheduling and again before publication; loading checks it
before executing native code or importing Python. Native DLL exports are still
validated separately: a compatible manifest does not override a bad binary.

The GUI displays compatible versions in green, incompatible versions in red,
and unchecked remote versions neutrally. The version chooser can download an
unchecked package and schedules it only after successful validation. Automatic
selection considers only verified compatible versions, ordered by package
version and then stable archive identifier.

## Embedded Changelogs

Packages may include an optional UTF-8 `changelog.json` next to `version.ini`:

```json
{
  "format_version": 1,
  "versions": {
    "1.0.1": ["Generate the package ABI manifest."],
    "1.0.0": ["Initial release."]
  }
}
```

Add `CHANGELOG path/to/changelog.json` to `slic3r_package_plugin()` or
`slic3r_package_python_plugin()`. Packaging validates and copies the document;
changing only this file republishes the native package on the next build too.
The document is limited to 1 MiB, uses unique keys, and maps exact package
versions (not slicer versions) to arrays of plain text notes. An empty array
intentionally supplies no notes for that version. Invalid files fail packaging.

The updater prefers the package's entry over repository history, including
when a repository response arrives later. Missing entries use the existing
repository fallback. An unread remote ZIP is not downloaded solely for notes.
The GUI displays both sources in the same changelog column, with provenance
in its tooltip. A malformed local file produces a path-specific diagnostic
and permits fallback; it does not prevent installation or change ABI status.
