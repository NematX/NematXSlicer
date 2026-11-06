///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_BridgeSurfaceVolume_hpp_
#define slic3r_BridgeSurfaceVolume_hpp_
/* Estimate room for excess bridge material from lower fill surfaces and densities.
   No lower extrusion path is read. A solid or perimeter stops the downward search.

   Construct Index once per object, then call sections() for each straight bridge
   row. pitch is the row spacing divided by its recipe density, in mm. The current
   layer contributes its full height; lower sparse contributes only its empty part.
   Returned sections own their values. The index owns scratch geometry in the
   supplied plugin storage, which must outlive it. Unknown support density never
   counts as empty or full. Results estimate volume, not local collision clearance.

       BridgeSurfaceVolume::Index lower(object, context, run);
       auto pieces = lower.sections(a, b, bottom_z, layer_height, area, pitch, width, floor);
*/
#include <vector>
#include "BridgeClearance.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_finalize_slicing.h"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"

namespace slic3r_api { namespace BridgeSurfaceVolume {
class Index {
public:
    // Copy model classifications and lower Z intervals; missing support information
    // marks only its possible areas unknown. Allocation and invalid input errors propagate.
    Index(const Object &object, const run_ctx_finalize_slicing &context, const plugin_run_context *run);
    // Search down to the platform or a local solid, stopping as soon as the excess
    // fits. Keep nominal flow on unknown portions and never exceed incoming area.
    std::vector<BridgeClearance::SectionPiece> sections(c_point a, c_point b, double bottom_z,
        double layer_height, double nominal_area, double pitch, double width, double floor) const;
    // Hash all copied density/geometry/Z inputs for correction replay checks.
    uint64_t signature() const { return m_signature; }
private:
    struct Domain { StoredExPolygonCollection areas; double density; };
    struct Band { double bottom, top; std::vector<Domain> domains; };
    std::vector<Band> m_bands;
    storage_handle *m_storage;
    const plugin_run_context *m_run;
    uint64_t m_signature=1469598103934665603ull;
};
}}
#endif
