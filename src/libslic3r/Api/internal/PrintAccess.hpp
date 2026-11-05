///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Api_internal_PrintAccess_hpp_
#define slic3r_Api_internal_PrintAccess_hpp_

#include "libslic3r/DataTreeFwd.hpp"
#include "libslic3r/Polygon.hpp"

/*
The host uses these accessors after an adhesion plugin has published its layers:
normalize their directions, then rebuild the first-layer hull. For example:

    ApiInternal::PrintAccess::rebuild_first_layer_convex_hull_after_skirt_brim(print);

The Print remains the owner of the outputs. This is a host implementation helper,
not an additional mutation interface for plugins.
*/

namespace Slic3r {

class ExtrusionEntity;
class Print;

namespace ApiInternal {

/*
Internal accessors for host-owned print outputs.

Plugin steps should normally mutate the print through C callback tables. The
host side of those callbacks still needs a narrow way to reach private Print
fields. Keep that access here instead of making brim, hull or state internals
public on Print.
*/
struct PrintAccess
{
    static void clear_brim(Print &print);
    static void clear_skirt(Print &print);
    static void normalize_skirt_brim_direction(Print &print);
    // Replace the owned plate-space hull with first-layer geometry, global adhesion and validated instance projections of local slices. Borrowed layers stay unchanged; invalid owners and allocation failures propagate.
    static void rebuild_first_layer_convex_hull_after_skirt_brim(Print &print);
};

} // namespace ApiInternal

} // namespace Slic3r

#endif // slic3r_Api_internal_PrintAccess_hpp_
