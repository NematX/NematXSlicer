///|/ Released under the GNU Affero General Public License, version 3 or later.
/* Perimeter-wrapped interface fill for support toolpath providers. Each island
   keeps its contours before its inner pattern; split islands may be reordered.
   The caller supplies a synchronous pattern callback and the effective flow.
   Example: fill_interface(storage, surface, settings, inner_pattern, output).
   Geometry and output views are borrowed only for the call. */
#ifndef nematx_interface_perimeters_hpp_
#define nematx_interface_perimeters_hpp_

#include "libslic3r/Plugins/Support/Common/PerimeterFill.hpp"

namespace nematx::support_interfaces {

struct Settings {
    unsigned int count = 0;
    double spacing = 0.;
    float encroachment = 0.5f;
    raw_extrusion_role role = RAW_EXTRUSION_ROLE_NONE;
    c_flow flow = {};
    bool monotonic = false;
};

// Append available contours and the inner pattern, keeping the pattern's complete extrusion tree.
// Empty offsets stop that branch. With a positive count, a failure leaves output unchanged;
// zero delegates directly to the pattern. Exceptions retain their original diagnostic.
void fill_interface(storage_handle *storage, const slic3r_api::Surface &surface, const Settings &settings,
                    const slic3r_api::Support::Common::InnerFill &inner, slic3r_api::MutableExtrusionEntity output);

}
#endif
