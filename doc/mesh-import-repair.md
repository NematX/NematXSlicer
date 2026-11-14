# Mesh repair at import

Import repair is a per-volume operation, independent of the file format or GUI
preferences. Format loaders finalize their new volumes after decoding all face
annotations. The model file on disk is never rewritten by this operation.

## Painted and unpainted volumes

An unpainted volume uses the ADMesh import repair shared with STL input. Repair
works on a candidate mesh. A candidate with no usable faces does not replace the
decoded source. Accepted repairs refresh mesh statistics, the volume's convex
hull and the object's bounding-box caches.

A painted volume is validated without repair. This includes support painting,
material segmentation, seam painting and nonempty generic facet annotations.
Face indices, winding and painting associations are not changed by finalization.
An unpainted sibling in the same object is still repaired.

Painted does not mean valid or previously repaired. Remaining defects produce a
diagnostic naming the object and volume; the CLI logs it and the GUI can display
the same diagnostic as a warning. These warnings do not replace existing file
format errors for corrupt archives or unusable indices.

## Validation limits

`TriangleMesh::validate()` checks finite coordinates, in-range vertex indices,
triangle edges with identical endpoints, and unmatched oriented mesh edges.
It does not prove the absence of arbitrary geometric self-intersections.

This cleanup is not a general topology repair at use time. Slab chaining retains
its Debug assertions. Its existing Release path traces and omits residual open
chains; a nonempty result is therefore not proof of completeness for arbitrary
defective topology. The C painting-projection entry still returns `void` and has
no separate failure status. These error-reporting limits are not changed by the
shared cleanup or by the strict mesh constructor.

`Model::repair_imported_meshes()` applies the import policy to newly loaded
objects. `Model::mesh_validation_warnings()` inspects the current model without
mutating geometry or annotations. Direct users of format loaders receive the
same import policy as `Model::read_from_file()` and `Model::read_from_archive()`.

Import repair must not be used as a general repair of a selected painted patch:
such a patch may intentionally be an open surface, and its original indices
belong to the source volume's annotations.

## Working geometry after painting selection

`mesh_without_degenerate_faces(source, scratch)` borrows a clean source or makes
one filtered copy in the caller's scratch storage. Only exact zero-length face
edges are removed, including repeated indices and distinct indices at identical
positions. No vertices move and no boundary is closed. The source is unchanged.

Mesh sections, full projections, slab projections and selected painted triangle
patches all use this preparation after the caller has resolved its face selection.
API calls use the same native entries; the strict plugin mesh constructor still
copies accepted arrays without repairing or renumbering them.

## Output rings

`normalize_mesh_polygons()` removes exact duplicate neighbors and redundant
explicit closing points. A repeated vertex elsewhere in a ring splits it into
closed subpaths instead of deleting arbitrary vertices. Only rings with no area
(fewer than three distinct points or wholly collinear points) are discarded.
Independent rings are not merged, their winding is retained, and their coordinates
are not simplified or welded with a tolerance.

Zero signed area is not by itself proof that a ring has no geometric area: a
crossing ring can have oppositely wound lobes. General segment intersections stay
subject to the caller's existing fill-rule operations, not a new repair engine.

`normalize_mesh_expolygons()` applies the same ring operations while retaining
explicit exterior/hole ownership. A collapsed exterior removes its entire group.
If an exterior splits, Clipper reassigns its holes within that group without the
small-area filtering of the general surface wrappers. Point arrays are transferred
directly so Polygon's tolerant constructors cannot trim near-closing vertices.
This does not add unions to the raw-ring API.

Final native surfaces and API surface cleanups use this normalization around their
existing simplification. The raw mesh-slice API still adds no union, offset or
resolution-based simplification. Projection and slicing retain their distinct
geometric meanings; the existing section gap-closing tolerance is not extended to
projections by these changes.

## Focused verification

`[mesh-repair]` covers import repair, painted-source preservation, local mesh
preparation, exact rings and holes, and real-host Python parity. Existing
`[plugins][slice-volume][slicing-helper]` tests cover the native/API parameter
matrix, cancellation and error translation.

The opt-in `[mesh-repair-real-project]` case reads a temporary binary STL of
Senin's original affected volume through `ReadSTLFile(..., false)`. Set
`SLIC3R_MESH_REPAIR_QUALIFICATION_STL` to this file. The fixture must retain all
115166 original faces, including the 28 repeated-index faces. The test compares
the raw native projection with an explicitly filtered reference and checks that
the original mesh is unchanged. It does not run a CLI export or rely on import
repair to hide a use-time projection defect.
The fixture decoder restores exact shared vertex indexing from STL's per-face
points without removing or changing any faces.
