# Package ABI Compatibility

Every installable plugin package contains a generated `version.ini`:

```ini
[plugin]
package_version = 1.0.0
slicer_version = 2.7.63.0-alpha+UNKNOWN

[abi]
slic3r_plugin_types.h = 4.1
slic3r_orchestrator.h = 3.5
slic3r_slicing_step.h = 2.2
slic3r_extrusion_entity.h = 1.0
steps/slic3r_step_post_perimeter.h = 1.0
steps/slic3r_step_gcode_export.h = 1.0
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

Plugin-types 4.1 includes the support-toolpath header identity
`SLIC3R_PLUGIN_API_STEP_SUPPORT_TOOLPATH` (58), naming
`steps/slic3r_step_support_toolpath.h`. Its sized vtable layout is unchanged from 4.0.

Data-tree 5.0 removes the branch surface modifier and reserves bit 10. Support-toolpath
2.0 removes the branch role: branch bodies are sparse support, and the sheath algorithm
shares the sparse selector. Packages requiring data-tree 4.x or support-toolpath 1.x
must be rebuilt. Support-step remains 1.3; its publication payload is unchanged.

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
