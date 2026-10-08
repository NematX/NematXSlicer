///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Derived support parameters, computed only through the plugin API.

   SupportParams turns the object, print and region settings into the values
   the support family and the support toolpaths share: layer heights rounded to
   the machine Z step, contact gaps, interface counts, the support flows, the
   XY gap and the patterns, densities and angles of the toolpaths. It gives the
   same values as the native derivation from the object settings (Slicing.cpp,
   support flow functions) and the native path settings of SupportParameters.cpp;
   tests/plugins/test_support_common.cpp compares both.

   Units follow the native classes: heights, gaps and extrusion lengths in
   millimeters as double, XY distances (gap_xy, resolution, min_height) scaled,
   angles in radians. Flows are c_flow values computed by the host
   (object_get_support_flow(), object_get_region_flow()), with their millimeter
   twins (c_flow_mm) for the computations the native code does in millimeters.

       const SupportParams params(Print(payload->print), Object(payload->object));
       const double spacing = unscaled(params.base_flow.spacing);

   Building one is cheap (a few config reads and host flow calls); keep one per
   object and per step run, not per layer. A plugin using it declares
   support_geometry_keys() and, for the path fields, support_path_keys(). */
#ifndef plugins_support_common_params_hpp_
#define plugins_support_common_params_hpp_

#include <cstddef>
#include <string>

#include "SupportSettings.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"

namespace slic3r_api::Support::Common {

struct SupportParams
{
    // Read the settings of a sliced object; its layers give the minimum layer height.
    SupportParams(const Print &print, const Object &object);

    // ---- Geometry (native Slicing.cpp derivation) ----
    // First layer, nominal support layer and thickest support layer heights, Z-step rounded (mm).
    double first_layer_height = 0, layer_height = 0, max_support_height = 0;
    // Vertical gaps above and below the support (mm), zero when soluble; machine Z step (mm).
    double gap_above = 0, gap_below = 0, z_step = 0;
    // Soluble support: contact distance type "none", no vertical gap.
    bool soluble = false;
    // Interface layers between the contact and the base, the contact itself excluded.
    size_t top_interfaces = 0, bottom_interfaces = 0;
    // First-layer, base and interface flows at the nominal heights; the interface flow is the
    // base flow when there is no interface layer. bottom_flow is the bottom-contact flow.
    c_flow first_flow = {}, base_flow = {}, interface_flow = {}, bottom_flow = {};
    // The same four flows in the host's millimeters. Computations that the native code does on the
    // float flow values (grid cell size, densities, widths in millimeters) read these, because the
    // scaled integers of c_flow are truncated.
    c_flow_mm first_flow_mm = {}, base_flow_mm = {}, interface_flow_mm = {}, bottom_flow_mm = {};
    // Expansion of the bottom interface, absolute (mm) or a percent of the bottom flow width.
    c_float_or_percent bottom_expansion = {};
    // Thinnest printable layer and XY gap to the object (scaled), and the internal resolution (scaled).
    coord_t min_height = 0, gap_xy = 0, resolution = SCALED_EPSILON;
    // Interface line density in (0, 1]; the base density when there is no interface layer.
    double interface_density = 0;

    // ---- Paths (native PathParams of the legacy toolpaths) ----
    SupportStyle style = SupportStyle::Unknown;
    ContactDistanceType distance_type = ContactDistanceType::Unknown;
    BasePattern base_pattern = BasePattern::Unknown;
    // Base line density in (0, 1].
    double base_density = 0;
    // Pattern plugin ids of the base and of the top and bottom interfaces ("auto" resolved).
    std::string base_pattern_id, top_pattern_id, bottom_pattern_id;
    // Sparse base wrapped in a perimeter (sheath), and removable loops around top contacts.
    bool with_sheath = false, interface_contact_loops = false;
    // Base and interface angles (radians, as float like the native fillers).
    float base_angle = 0.f, interface_angle = 0.f;
    // Height period of the quarter-turn base rotation (mm, 0 disables) and per-layer interface
    // angle increment (degrees).
    double angle_height = 0, interface_angle_increment = 0;
    // Line density of the first-layer flange, in (0, 1].
    double raft_first_layer_density = 0;
};

} // namespace slic3r_api::Support::Common

#endif // plugins_support_common_params_hpp_
