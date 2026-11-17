///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Derived support parameters, computed only through the plugin API.

   SupportParams turns the object, print and region settings into the values
   the support family and the support toolpaths share: layer heights rounded to
   the machine Z step, contact gaps, interface counts, the support flows, the
    XY gap and the patterns, densities and angles of the toolpaths. Values follow the
    native derivation (Slicing.cpp, support flow functions and SupportParameters.cpp),
    except gap_xy truncates to XY coordinates instead of snapping to layer Z: it may
    differ from native by less than SCALED_LAYER_Z_STEP. tests/plugins/test_support_common.cpp compares both.

   Units follow the native classes: heights, gaps and extrusion lengths in
   millimeters as double, XY distances (gap_xy, resolution, min_height) scaled,
   angles in radians except base_angle_degrees and interface_angle_increment (degrees). Flows are c_flow values computed by the host
   (object_get_support_flow(), object_get_region_flow()), with their millimeter
   twins (c_flow_mm) for the computations the native code does in millimeters.

       const SupportParams params(Print(payload->print), Object(payload->object));
       const double spacing = unscaled(params.base_flow.spacing);

   The family runner builds one SupportParams per object generation run and passes it down;
   each toolpath execution builds its own. Never rebuild it per layer. Construction costs a few
   config reads and host flow calls. A plugin using it declares
   support_geometry_keys() and, for the path fields, support_path_keys(). */
#ifndef plugins_support_common_params_hpp_
#define plugins_support_common_params_hpp_

#include <cstddef>
#include <string>
#include <vector>

#include "SupportSettings.hpp"
#include "libslic3r/Api/plugin/c/slic3r_plugin_run_context.h"
#include "libslic3r/Api/plugin/c/slic3r_volume.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"

namespace slic3r_api::Support::Common {

// Native check_z_step() on millimeters: round val to the nearest multiple of z_step, at 1e-8 mm.
// Contract: val must be finite and >= 0 (it is converted to uint64_t); val is returned unchanged when z_step <= EPSILON (1e-4 mm),
// that is when no machine Z step is set, not only when z_step is 0.
double check_z_step_mm(double val, double z_step);

struct SupportParams
{
    // Derive settings-based values even before slicing; min_height also needs the object's layers to account for their heights.
    SupportParams(const Print &print, const Object &object);

    // ---- Geometry (native Slicing.cpp derivation) ----
    // First layer, nominal support layer and thickest support layer heights, Z-step rounded (mm).
    double first_layer_height = 0, layer_height = 0, max_support_height = 0;
    // Thinnest layer of the support body (mm): its extruder's minimum, or with the current extruder the largest minimum of the object extruders.
    double support_min_height = 0;
    // Vertical gaps above and below the support (mm), zero when soluble; machine Z step (mm).
    double gap_above = 0, gap_below = 0, z_step = 0;
    // Soluble support: contact distance type "none", no vertical gap.
    bool soluble = false;
    // Interface layers between the contact and the base, the contact itself excluded.
    size_t top_interfaces = 0, bottom_interfaces = 0;
    // Raw configured counts, including the contact; a disabled bottom option follows the top count when used.
    int32_t top_interface_layers = 0, bottom_interface_layers = 0;
    // Whether the bottom count option itself is enabled, independently of its numeric value.
    bool bottom_interface_count_enabled = false;
    // Positive effective counts enable the geometry stacks; bottom geometry may exist without a top stack.
    bool top_interfaces_enabled = false, bottom_interfaces_enabled = false;
    // Publication and legacy paths use raw unclamped counts: top raw != 0 enables top surfaces; bottom surfaces require
    // top raw != 0 and (bottom option disabled OR bottom raw != 0), unlike the positive-count geometry flags above.
    bool top_published_interfaces_enabled = false, bottom_published_interfaces_enabled = false;
    // One-based configured tools; 0 selects the current tool.
    int32_t base_tool = 0, interface_tool = 0;
    // First-layer, base and interface flows at the nominal heights; the interface flow is the
    // base flow when there is no interface layer.
    c_flow first_flow = {}, base_flow = {}, interface_flow = {};
    // The same flows and the bottom-contact flow in the host's millimeters. Computations that the native code does on the
    // float flow values (grid cell size, densities, widths in millimeters) read these, because the
    // scaled integers of c_flow are truncated.
    c_flow_mm first_flow_mm = {}, base_flow_mm = {}, interface_flow_mm = {}, bottom_flow_mm = {};
    // Expansion of the bottom interface, absolute (mm) or a percent of the bottom flow width.
    c_float_or_percent bottom_expansion = {};
    // Thinnest printable layer and XY gap to the object (scaled), and the internal resolution (scaled).
    coord_t min_height = 0, gap_xy = 0, resolution = SCALED_EPSILON;
    // Contact mode: support_material_contact_xy_spacing is enabled and below gap_xy. The contact slabs then keep only
    // gap_xy_contact (scaled) from the object, and the cone of a contact, shrunk by contact_cone_step (scaled) per support
    // layer, joins the slabs of the next contact_cone_layers layers. Otherwise gap_xy_contact is gap_xy and the cones are off.
    bool contact_mode = false;
    coord_t gap_xy_contact = 0, contact_cone_step = 0;
    size_t contact_cone_layers = 0;
    // Interface line density in (0, 1]; the base density when there is no interface layer.
    double interface_density = 0;
    // Configured clear gap between sparse support lines (mm), shared by density calculation and the grid raster spacing.
    double base_spacing_gap = 0;

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
    // Unconverted base angle for consumers whose calculations start in degrees.
    double base_angle_degrees = 0;
    // Height period of the quarter-turn base rotation (mm, 0 disables) and per-layer interface
    // angle increment (degrees).
    double angle_height = 0, interface_angle_increment = 0;
    // Line density of the first-layer flange, in (0, 1].
    double raft_first_layer_density = 0;
};

// Returns borrowed object-layer views in increasing layer-id order.
std::vector<Layer> object_layers(const Object &object);
// Reports whether any island of the borrowed object has support demand.
bool object_has_support_demand(const Object &object);
// Whether the object prints support at all: support_material, or forced support layers (support_material_enforce_layers > 0),
// as the host's PrintObject::has_support(). Support enforcers, painted or volumes, add demand only then.
bool object_support_enabled(const Object &object);

/* ---- Support modifier volumes (enforcers, blockers and no-support zones) ----
   The demand modifiers slice the enforcers and blockers, and the support families slice the no-support zones, at the slice Z of
   every object layer. The helpers below are shared by them so that all of them slice the volumes alike. */

// The slice Z of every object layer (mm), the heights the modifier volumes are sliced at.
std::vector<float> layer_slice_zs(const Object &object);

// The slicing parameters of the modifier volumes: the centering transform of the object, the print resolution (at least EPSILON)
// and the regular slicing mode. Slice a volume with the transform multiplied by the matrix of the volume.
c_mesh_slicing_params support_modifier_slicing_params(const Print &print, const Object &object);

// Slice one modifier volume at the given Z values. base_params comes from support_modifier_slicing_params(). The result has one
// collection per Z (empty vector when the volume has no mesh), each cleaned with the resolution of base_params.
// run_ctx gives the cancellation checks and may be null.
std::vector<StoredExPolygonCollection> slice_support_modifier_volume(const Volume &volume,
                                                                     const std::vector<float> &slice_zs,
                                                                     const c_mesh_slicing_params &base_params,
                                                                     storage_handle *storage,
                                                                     const plugin_run_context *run_ctx);

// The union of every volume of the given type of the object, sliced like slice_support_modifier_volume() at the slice Z of every
// object layer: one collection per object layer, in storage. The result is an empty vector when the object has no such volume,
// and the union does not depend on the order of the volumes. run_ctx gives the cancellation checks and may be null.
std::vector<StoredExPolygonCollection> slice_support_volumes(const Print &print, const Object &object, raw_volume_type type,
                                                             storage_handle *storage, const plugin_run_context *run_ctx);
// The same union at the given slice Z values (mm, non-decreasing) instead of the object layers: one collection per Z.
std::vector<StoredExPolygonCollection> slice_support_volumes(const Print &print, const Object &object, raw_volume_type type,
                                                             const std::vector<float> &slice_zs, storage_handle *storage,
                                                             const plugin_run_context *run_ctx);

// What the support keeps clear of on one object layer: the layer slices, united with the no-support zones sliced for that layer
// (zones holds one collection per object layer, or nothing). Without a zone on the layer the operand is the slices themselves,
// so the clearance of an object without zones is unchanged. The operand borrows the layer and the zones until it is consumed.
ClipperOperand support_obstacle(const ClipperContext &clipper, const Layer &layer, size_t layer_idx,
                                const std::vector<StoredExPolygonCollection> &zones);

} // namespace slic3r_api::Support::Common

#endif // plugins_support_common_params_hpp_
