///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_BridgeGeometry_hpp_
#define slic3r_BridgeGeometry_hpp_
/* Widen bridge areas onto lower material and extend their ends for anchoring.
   BridgePreparation.cpp uses these helpers before generating parallel bridge rows.

   core is the input area for an expansion. domain is the area into which its ends
   may extend. axial_expand() grows core in the printing direction at angle, up to
   margin at each end, stopping at domain boundaries and holes. expand_bridge()
   first adds a uniform margin, clipped to domain and connected to core, then uses
   axial_expand() for the additional end margin. Only the uniform margin adds rows
   beside the original unsupported area.

   The boolean helpers combine or clip areas. XY coordinates and margin are scaled
   integers; angle is in radians. Inputs stay unchanged. Results belong to the
   caller's storage and must be destroyed before that storage.

   Example, in namespace slic3r_api::BridgeGeometry:
       StoredExPolygonCollection anchors = axial_expand(storage, core, domain, 0., scale_i(1.));

   geometry_key() and geometry_less() give the same geometry a stable hash and
   comparison order, so reordering input contours does not change bridge grouping.
*/
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
namespace slic3r_api { namespace BridgeGeometry {
// Return a normalized union in storage, or an owned empty collection for empty input.
StoredExPolygonCollection unite(storage_handle *storage, const ExPolygonCollection &areas);
// Return the part of a outside b in storage; empty b leaves an owned copy of a.
StoredExPolygonCollection subtract(storage_handle *storage, const ExPolygonCollection &a, const ExPolygonCollection &b);
// Return the common area in storage; an empty input produces an empty collection.
StoredExPolygonCollection intersect(storage_handle *storage, const ExPolygonCollection &a, const ExPolygonCollection &b);
// Extend the ends of each cross-section through core by margin, within domain
// union core. Do not cross holes or add rows beside core; margin <= 0 returns a copy.
// Malformed scanlines or more than one million temporary sweep pieces throw.
// Geometry/allocation failures and cancellation through the borrowed context propagate.
StoredExPolygonCollection axial_expand(storage_handle *storage, const ExPolygonCollection &core,
    const ExPolygonCollection &domain, double angle, coord_t margin, const plugin_run_context *ctx = nullptr);
// Add a round uniform margin within domain, retaining only components meeting core,
// then extend their ends axially. Zero uniform margin preserves axial_expand().
// Inputs are borrowed; results use storage. Geometry failures and cancellation propagate.
StoredExPolygonCollection expand_bridge(storage_handle *storage, const ExPolygonCollection &core,
    const ExPolygonCollection &domain, double angle, coord_t axial_margin, coord_t uniform_margin,
    const plugin_run_context *ctx = nullptr);
// Hash coordinates independently of contour start and collection order; winding
// and the actual coordinates remain part of the key.
uint64_t geometry_key(const ExPolygonCollection &areas);
// Compare those normalized coordinate sequences to order equal-hash candidates.
bool geometry_less(const ExPolygonCollection &a, const ExPolygonCollection &b);
}}
#endif
