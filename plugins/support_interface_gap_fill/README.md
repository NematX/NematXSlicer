# Support interface gap fill

`nematx.support.interface_gap_fill` is a non-exclusive post-process provider at
`STEP_SUPPORT_TOOLPATH`, priority 300. It runs after the legacy support providers
and those of `nematx-support-interface-perimeters`, without replacing them or
calling their fillers.

Enable the object option `support_material_interface_gap_fill` (default false).
Only full-density interfaces with zero configured interface spacing are eligible.
Sparse passes, passes using base settings, missing providers and top contacts
with removable loops remain untouched. These two provider families are
recognized; unknown providers are skipped with a warning because their reserved
masks may differ.

The plugin subtracts useful path coverage from the final published surfaces,
filters residuals using the existing regional `gap_fill_*` thresholds, then uses
the SDK medial-axis extrusion factory. Non-bridged gaps use variable width;
bridges keep their nominal diameter and leave narrower gaps empty. Only new
paths are modulated against lower support slabs. They retain the interface role
and tool and follow the old tree behind a fixed ordering boundary. Existing
properties move with the old content, rather than leaking onto the additions.

Concentric paths are not regenerated: all additions follow the existing paths,
unlike the historical concentric implementation that interleaved gap fills.
Raft generation and general object gap fill are outside this package.

## Checks

The standalone target is `nematx_support_interface_gap_fill`; the focused SDK
test is `support_gap_fill_geometry`. `tests/test_gap_fill.py` runs bounded exports
and a linear catalogue of declared settings.

With host tests enabled, `nematx_support_gap_fill_host_tests` exercises repeated
changes on the same Print, object-local overrides and two-tool assignment. It
compares replayed trees with fresh Prints, rather than inferring invalidation
correctness from separate CLI invocations.

The export gate accepts `--embedded-defaults` to test the embedded provider
without loading the standalone package. `--compare-report` compares the enabled
deposition digest with a completed standalone report. Compilation of an embedded
object library alone is not evidence of embedded-export parity.
