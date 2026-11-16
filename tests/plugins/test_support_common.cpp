#include "plugin_test_helpers.hpp"
#include "support_grid_test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_toolpath.h"
#include "libslic3r/Api/plugin/cpp/LayerHeightLimits.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"
#include "libslic3r/Plugins/Support/Common/PerimeterFill.hpp"
#include "libslic3r/Plugins/Support/Common/SupportDemand.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"
#include "libslic3r/Plugins/Support/Common/SupportPaths.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/Steps/InfillPatternService.hpp"

/* The Common support library rewrites, on the plugin API, values the native
   support code computes: the support geometry, the toolpath PathParams, the
   enum choices of the support settings, and FillWithPerimeter. These tests
   compare each one with the native code on the same inputs.

   - SupportParams runs on the configurations of the legacy-comparison harness
      (support_grid_test_helpers.hpp) and a few variants, after slicing; the mixed-nozzle
      bounds test derives settings-based values before slicing, without layer contributions to min_height.
   - The enum helpers run on every value of the native enum lists.
   - The smooth contact fill runs on a sliced print with non-default ironing settings and is
     compared with the native FillSmooth on the same surfaces.
   - The perimeter-wrapped fill calls its inner pattern through the
     STEP_SUPPORT_TOOLPATH pattern service, as a toolpath plugin does, and its
     printable paths are matched by geometry, multiplicity and effective properties.
     Perimeters must stay before their own fill; independent areas and inner islands
     have no canonical order or grouping. The honeycomb
     plugin leaves its clipped pieces unconnected where the native filler joins them,
     so for it only the perimeters are compared and the inner fill is measured.

   The flows come both as scaled integers (c_flow) and as the host's float
   millimeters (c_flow_mm); the derived densities read the millimeters, so
   every value is compared exactly.

       compare_perimeter_fill(): run the same recipe through native and plugin code
           `-- same_perimeter_fill(): match geometry and check perimeter-before-fill
               |-- collect_printed_paths(): read paths, inherited properties and ancestry
               |-- same_printed_path(): match points, orientation and physical properties
               `-- fixed_before(): check ordering permissions at the shared ancestor
*/

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
namespace Common = slic3r_api::Support::Common;

namespace {

// One harness configuration with extra overrides.
struct ParamsCase
{
    const char *style;
    bool soluble;
    int interfaces;
    bool same_tool;
    std::vector<std::pair<std::string, std::string>> overrides;
    // Options disabled as a whole after the overrides.
    std::vector<std::string> disabled = {};
};

// Native support geometry: nozzle limits, contact gaps and flows that the legacy support code derives
// from the object settings. Common::SupportParams has to give the same values through the plugin API.
struct NativeGridParams
{
    // Derive contact gaps, layer limits and geometry flows from the object's settings.
    explicit NativeGridParams(const PrintObject &object);
    double first_layer_height = 0, layer_height = 0, max_support_height = 0;
    double gap_above = 0, gap_below = 0, z_step = 0;
    bool soluble = false;
    size_t top_interfaces = 0, bottom_interfaces = 0;
    Flow first_flow, base_flow, interface_flow, bottom_flow;
    FloatOrPercent bottom_expansion;
    coord_t min_height = 0, gap_xy = 0, resolution = SCALED_EPSILON;
    double interface_density = 0;
};

// Resolve percent heights against the nozzle; zero uses the slicer's minimum default and the result is rounded to the machine Z step.
double native_min_layer_height(const PrintConfig &print, uint16_t id)
{
    const double nozzle = print.nozzle_diameter.get_at(id);
    const double height = print.min_layer_height.get_effective_value(nozzle, id);
    return check_z_step(slic3r_api::min_layer_height_mm(height, nozzle), print.z_step);
}
// Resolve percent heights against the nozzle; a disabled or zero limit uses three quarters of its diameter and respects the configured minimum.
double native_max_layer_height(const PrintConfig &print, uint16_t id)
{
    const double minimum = native_min_layer_height(print, id);
    const double nozzle = print.nozzle_diameter.get_at(id);
    const double maximum = print.max_layer_height.get_effective_value(nozzle, id);
    return check_z_step(std::max(minimum, slic3r_api::max_layer_height_mm(maximum, print.max_layer_height.is_enabled(), nozzle)),
                        print.z_step);
}

// Resolve nozzle limits and contact distances for an already sliced object; this instance owns its flows and leaves the object's settings unchanged.
NativeGridParams::NativeGridParams(const PrintObject &object)
{
    const PrintConfig &print = object.print()->config();
    const PrintObjectConfig &config = object.config();
    const std::set<extruder_index_t> extruders = object.object_extruders();
    z_step = print.z_step;
    soluble = config.support_material_contact_distance_type.value == zdNone;
    double min_nozzle = print.nozzle_diameter.get_at(0);
    first_layer_height = config.first_layer_height.value;
    if (config.first_layer_height.percent) {
        first_layer_height = std::numeric_limits<double>::max();
        for (extruder_index_t id : extruders) {
            const double nozzle = print.nozzle_diameter.get_at(id);
            min_nozzle = std::min(min_nozzle, nozzle);
            first_layer_height = std::min(first_layer_height, config.first_layer_height.get_effective_value(nozzle));
        }
    }
    first_layer_height = check_z_step(first_layer_height, z_step);
    const size_t base_id = config.support_material_extruder.value - 1;
    const size_t interface_id = config.support_material_interface_extruder.value - 1;
    const double interface_nozzle = print.nozzle_diameter.get_at(interface_id);
    const double base_max = std::min(native_max_layer_height(print, base_id),
        config.support_material_extruder.value > 0 ? print.nozzle_diameter.get_at(base_id) : min_nozzle);
    const double interface_max = std::min(native_max_layer_height(print, interface_id),
        config.support_material_interface_extruder.value > 0 ? interface_nozzle : min_nozzle);
    double minimum = 0, maximum = std::numeric_limits<double>::max();
    if (config.support_material_extruder.value > 0) {
        minimum = native_min_layer_height(print, base_id);
        maximum = base_max;
    }
    if (config.support_material_interface_extruder.value > 0) {
        minimum = std::max(minimum, native_min_layer_height(print, interface_id));
        maximum = std::min(maximum, interface_max);
    }
    max_support_height = maximum == std::numeric_limits<double>::max() ? 0 : std::min(maximum, base_max);
    for (extruder_index_t id : extruders) {
        minimum = std::max(minimum, native_min_layer_height(print, id));
        maximum = std::min(maximum, native_max_layer_height(print, id));
    }
    if (maximum == std::numeric_limits<double>::max()) maximum = config.layer_height.value;
    if (minimum == 0) minimum = config.layer_height.value;
    minimum = check_z_step(minimum, z_step);
    maximum = check_z_step(maximum, z_step);
    if (max_support_height == 0) max_support_height = maximum;
    max_support_height = check_z_step(max_support_height, z_step);
    layer_height = std::min(maximum, std::max(minimum, check_z_step(config.layer_height.value, z_step)));
    if (!soluble) {
        gap_above = check_z_step(config.support_material_contact_distance.get_effective_value(interface_nozzle), z_step);
        gap_below = check_z_step(config.support_material_bottom_contact_distance.get_effective_value(interface_nozzle), z_step);
        if (gap_below <= 0) gap_below = gap_above;
    }
    const int top = std::max(0, config.support_material_interface_layers.value);
    const int bottom = config.support_material_bottom_interface_layers.is_enabled() ?
        std::max(0, config.support_material_bottom_interface_layers.value) : top;
    top_interfaces = top > 0 ? size_t(top - 1) : 0;
    bottom_interfaces = bottom > 0 ? size_t(bottom - 1) : 0;
    first_flow = support_material_1st_layer_flow(&object, float(first_layer_height));
    base_flow = support_material_flow(&object, float(layer_height));
    interface_flow = support_material_interface_flow(&object, float(layer_height));
    if (top == 0) interface_flow = base_flow;
    resolution = scale_i(print.resolution_internal);
    min_height = std::numeric_limits<coord_t>::max();
    for (size_t id = 0; id < print.min_layer_height.size(); ++id) {
        const coord_t height = scale_to_layer_coord(print.min_layer_height.get_effective_value(print.nozzle_diameter.get_at(id), id));
        if (height > 0) min_height = std::min(min_height, height);
    }
    for (const Layer &layer : object.layers())
        if (layer.scaled_height() > 0) min_height = std::min(min_height, layer.scaled_height());
    double external_width = 0, bridge_ratio = 0;
    for (size_t id = 0; id < object.num_printing_regions(); ++id) {
        const PrintRegion &region = object.printing_region(id);
        external_width = std::max(external_width, double(region.flow(object, frExternalPerimeter, layer_height, 2).width()));
        bridge_ratio += region.config().bridge_flow_ratio.get_effective_value(1.);
    }
    gap_xy = scale_to_layer_coord(config.support_material_xy_spacing.get_effective_value(external_width));
    bridge_ratio /= object.num_printing_regions();
    bottom_flow = soluble ? interface_flow.with_flow_ratio(bridge_ratio) :
        Flow::bridging_flow(std::sqrt(bridge_ratio) * interface_flow.nozzle_diameter(), interface_flow.nozzle_diameter());
    bottom_expansion = boost::any_cast<FloatOrPercent>(config.support_material_bottom_interface_expansion.get_any());
    interface_density = std::min(1., interface_flow.spacing() / (config.support_material_interface_spacing.value + interface_flow.spacing()));
    if (top == 0)
        interface_density = std::min(1., base_flow.spacing() / (config.support_material_spacing.value + base_flow.spacing()));
}

// The harness combinations plus variants exercising percent heights, the current extruder,
// percent XY gaps, the base patterns, the "auto" interface patterns, disabled layer limits and
// interface counts, and object extruders with different nozzles.
std::vector<ParamsCase> params_cases();
// Compare every c_flow field with a native Flow, converted like the host converts it.
void check_flow(const c_flow &actual, const Flow &expected);
// Compare the millimeter flow with the native float values, exactly.
void check_flow_mm(const c_flow_mm &actual, const Flow &expected);
// Serialized name of a native InfillPattern value.
std::string pattern_name(InfillPattern pattern);
// Native pattern choice for one interface, as SupportParameters.cpp computes it.
InfillPattern native_auto_interface(bool soluble, double interface_density);
// One printable path with effective properties and its borrowed ancestry. Ancestry
// checks perimeter-before-fill constraints, not equality of grouping nodes.
struct PrintedPath {
    Points points;
    ExtrusionAttributes attributes;
    bool has_attributes = false, loop = false, reversible = false, sortable = false, loop_locked = true;
    std::vector<std::pair<const ExtrusionEntity *, size_t>> ancestors;
};
// Copy printable paths and inherited properties; owners remain borrowed through ancestry.
void collect_printed_paths(const ExtrusionEntity &entity, std::vector<PrintedPath> &paths,
    const ExtrusionAttributes *inherited = nullptr, bool loop = false, bool loop_reverse = true,
    bool loop_locked = true, std::vector<std::pair<const ExtrusionEntity *, size_t>> ancestors = {});
// Compare coordinates including cyclic loop starts and only permitted reversal.
bool same_printed_path(const PrintedPath &actual, const PrintedPath &expected);
// Check physical attributes with the existing one-scaled-unit dimension tolerance.
bool same_path_attributes(const PrintedPath &actual, const PrintedPath &expected);
// Whether the tree keeps this perimeter before this fill under later ordering/reversal.
bool fixed_before(const PrintedPath &perimeter, const PrintedPath &fill);
// Paths and length of the inner fills of a perimeter-wrapped tree, native and API.
struct InnerFillStats { size_t native_paths = 0, api_paths = 0; double native_length = 0, api_length = 0; };
// Match printable paths with multiplicity, independently of island/group order.
// Native FillWithPerimeter identifies perimeter paths and their zones; only its
// printable geometry/properties and perimeter-before-fill relations are required.
// Non-exact mode retains the documented honeycomb connection difference.
bool same_perimeter_fill(const ExtrusionEntity &actual, const ExtrusionEntity &expected,
                         bool exact, InnerFillStats &stats, std::string &error);
// Two disjoint areas with literal contours, a hole and open fill paths, owned by
// the returned tree. Each fixed area prints its perimeters before its fill.
ExtrusionEntity perimeter_comparison_fixture(bool reversible);
// Forward a pattern request of the test payload to the pattern service, as the host step does.
int32_t service_generate_pattern(const run_ctx_support_toolpath *ctx, infill_pattern_runtime_id pattern_id,
                                 const layer_handle *layer, const layer_island_handle *island,
                                 const layer_region_island_handle *region_island, const layer_region_handle *primary_region,
                                 const surface_handle *surface, const expolygon_collection_handle *no_overlap_areas,
                                 const raw_infill_pattern_params *params, extrusion_entity_handle *output);
// Recipe of the inner pattern equal to the native FillParams and Fill settings of the legacy toolpaths.
raw_infill_pattern_params inner_recipe(infill_pattern_runtime_id pattern_id, const Flow &flow, double spacing, float density,
                                       float angle, double z);
// c_flow of a native flow, as the host converts it.
c_flow to_api_flow(const Flow &flow);

// Builds owned parameter combinations and explicit option overrides, including an XY gap between Z-grid steps;
// no print is mutated here, and allocation failures propagate to Catch.
std::vector<ParamsCase> params_cases()
{
    std::vector<ParamsCase> cases;
    for (const char *style : {"grid", "snug"})
        for (const bool soluble : {false, true})
            for (const int interfaces : {0, 3})
                for (const bool same_tool : {false, true})
                    cases.push_back({style, soluble, interfaces, same_tool, {}});
    cases.push_back({"grid", false, 3, false, {{"first_layer_height", "75%"}}});
    cases.push_back({"grid", false, 3, false, {{"support_material_extruder", "0"}, {"support_material_interface_extruder", "0"}}});
    cases.push_back({"grid", false, 3, false, {{"support_material_xy_spacing", "60%"}, {"support_material_bottom_interface_expansion", "50%"}}});
    cases.push_back({"grid", false, 3, false, {{"support_material_xy_spacing", "0.123456"}}});
    cases.push_back({"grid", false, 3, false, {{"support_material_pattern", "honeycomb"}, {"support_material_with_sheath", "1"}}});
    cases.push_back({"snug", false, 3, false, {{"support_material_pattern", "rectilinear-grid"}, {"support_material_spacing", "0"}}});
    cases.push_back({"grid", false, 2, false, {{"support_material_top_interface_pattern", "auto"}, {"support_material_bottom_interface_pattern", "auto"}}});
    cases.push_back({"grid", true, 2, false, {{"support_material_top_interface_pattern", "hilbertcurve"}, {"support_material_bottom_interface_pattern", "auto"}}});
    cases.push_back({"grid", false, 2, false, {{"support_material_interface_spacing", "0"},
                                               {"support_material_angle", "30"}, {"support_material_interface_angle", "45"}}});
    cases.push_back({"grid", false, 3, false, {}, {"max_layer_height"}});
    cases.push_back({"grid", false, 3, false, {}, {"support_material_bottom_interface_layers"}});
    cases.push_back({"grid", false, 3, false, {{"support_material_bottom_interface_layers", "-1"}}});
    cases.push_back({"grid", false, 0, false, {{"support_material_bottom_interface_layers", "2"}}});
    cases.push_back({"grid", false, 3, false, {{"support_material_bottom_interface_layers", "0"}}});
    cases.push_back({"grid", false, 3, false, {{"nozzle_diameter", "0.6,0.4"}, {"first_layer_height", "75%"}, {"perimeter_extruder", "1"},
                                               {"infill_extruder", "2"}, {"solid_infill_extruder", "2"},
                                               {"support_material_extruder", "0"}, {"support_material_interface_extruder", "0"}}});
    cases.push_back({"grid", false, 3, false, {{"z_step", "0.04"}, {"layer_height", "0.15"}, {"min_layer_height", "0.08,0.05"},
                                               {"max_layer_height", "75%,0.3"}}});
    return cases;
}

void check_flow(const c_flow &actual, const Flow &expected)
{
    CHECK(actual.width == expected.scaled_width());
    CHECK(actual.spacing == expected.scaled_spacing());
    CHECK(actual.height == expected.scaled_height());
    CHECK(actual.nozzle_diameter == scale_i(expected.nozzle_diameter()));
    CHECK(actual.is_bridge == (expected.bridge() ? 1 : 0));
    CHECK(actual.spacing_ratio == expected.spacing_ratio());
    CHECK(actual.mm3_per_mm == expected.mm3_per_mm());
}

void check_flow_mm(const c_flow_mm &actual, const Flow &expected)
{
    CHECK(actual.width == double(expected.width()));
    CHECK(actual.spacing == double(expected.spacing()));
    CHECK(actual.height == double(expected.height()));
    CHECK(actual.nozzle_diameter == double(expected.nozzle_diameter()));
}

std::string pattern_name(InfillPattern pattern)
{
    // The static InfillPattern name map has no "supportbase", so the support patterns are listed here.
    switch (pattern) {
    case ipRectilinear: return "rectilinear";
    case ipMonotonic: return "monotonic";
    case ipConcentric: return "concentric";
    case ipSawtooth: return "sawtooth";
    case ipHilbertCurve: return "hilbertcurve";
    case ipSmooth: return "smooth";
    case ipSupportBase: return "supportbase";
    case ipHoneycomb: return "honeycomb";
    default: FAIL("Unexpected support pattern " << int(pattern)); return {};
    }
}

InfillPattern native_auto_interface(bool soluble, double interface_density)
{
    return soluble ? ipConcentric : interface_density > 0.95 ? ipRectilinear : ipSupportBase;
}

void collect_printed_paths(const ExtrusionEntity &entity, std::vector<PrintedPath> &paths,
    const ExtrusionAttributes *inherited, bool loop, bool loop_reverse, bool loop_locked,
    std::vector<std::pair<const ExtrusionEntity *, size_t>> ancestors)
{
    const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
    if (attributes == nullptr) attributes = inherited;
    if (entity.is_loop()) {
        loop = true;
        loop_reverse = loop_reverse && entity.can_reverse();
        loop_locked = loop_locked && !entity.can_sort();
    }
    if (entity.has_polyline()) {
        PrintedPath path;
        path.points = entity.polyline_ref().to_polyline().points;
        path.has_attributes = attributes != nullptr;
        if (attributes != nullptr) path.attributes = *attributes;
        path.loop = loop;
        path.reversible = entity.can_reverse() && (!loop || loop_reverse);
        path.sortable = entity.can_sort();
        path.loop_locked = loop_locked;
        path.ancestors = ancestors;
        paths.push_back(std::move(path));
    }
    for (size_t idx = 0; idx < entity.child_count(); ++idx) {
        ancestors.emplace_back(&entity, idx);
        collect_printed_paths(entity.child(idx), paths, attributes, loop, loop_reverse, loop_locked, ancestors);
        ancestors.pop_back();
    }
}

bool same_path_attributes(const PrintedPath &actual, const PrintedPath &expected)
{
    return actual.has_attributes && expected.has_attributes && actual.attributes.role == expected.attributes.role &&
        actual.attributes.no_seam == 0 && expected.attributes.no_seam == 0 &&
        actual.attributes.mm3_per_mm == expected.attributes.mm3_per_mm &&
        std::abs(actual.attributes.width - expected.attributes.width) <= unscaled(coord_t(1)) &&
        std::abs(actual.attributes.height - expected.attributes.height) <= unscaled(coord_t(1));
}

bool same_printed_path(const PrintedPath &actual, const PrintedPath &expected)
{
    if (!same_path_attributes(actual, expected) || actual.loop != expected.loop ||
        actual.reversible != expected.reversible || actual.sortable != expected.sortable ||
        actual.loop_locked != expected.loop_locked || actual.points.size() != expected.points.size()) return false;
    size_t size = expected.points.size();
    if (size == 0) return false;
    if (expected.loop) {
        if (actual.points.front() != actual.points.back() || expected.points.front() != expected.points.back()) return false;
        --size;
    }
    // A loop's first vertex is not a seam guarantee. Its winding is retained
    // unless the printable unit explicitly permits reversal.
    for (size_t start = 0; start < (expected.loop ? size : 1); ++start) {
        bool forward = true, reverse = expected.reversible;
        for (size_t i = 0; i < size; ++i) {
            forward = forward && actual.points[(start + i) % size] == expected.points[i];
            const size_t reversed = expected.loop ? (start + size - i) % size : size - i - 1;
            reverse = reverse && actual.points[reversed] == expected.points[i];
        }
        if (forward || reverse) return true;
    }
    return false;
}

bool fixed_before(const PrintedPath &perimeter, const PrintedPath &fill)
{
    for (size_t i = 0; i < std::min(perimeter.ancestors.size(), fill.ancestors.size()); ++i) {
        const std::pair<const ExtrusionEntity *, size_t> &a = perimeter.ancestors[i], &b = fill.ancestors[i];
        if (a.first != b.first) return false;
        if (a.second != b.second)
            return a.second < b.second && !a.first->can_sort() && !a.first->can_reverse();
    }
    return false;
}

bool same_perimeter_fill(const ExtrusionEntity &actual, const ExtrusionEntity &expected,
                         bool exact, InnerFillStats &stats, std::string &error)
{
    std::vector<PrintedPath> a, b;
    collect_printed_paths(actual, a);
    collect_printed_paths(expected, b);
    std::vector<bool> used(a.size(), false);
    std::vector<size_t> zones(b.size()), matches(b.size(), a.size());
    std::vector<bool> perimeter(b.size());
    const size_t area_depth = expected.can_sort() ? 1 : 0;
    const size_t zone_count = expected.can_sort() ? expected.child_count() : 1;
    std::vector<ExPolygon> boundaries(zone_count);
    // Only the native reference's documented builder layout labels the expected
    // perimeter paths. No actual grouping node is compared with that layout.
    for (size_t i = 0; i < b.size(); ++i) {
        if (b[i].ancestors.size() <= area_depth) { error = "Incomplete native perimeter fixture"; return false; }
        zones[i] = area_depth == 0 ? 0 : b[i].ancestors[0].second;
        perimeter[i] = b[i].ancestors[area_depth].second == 0;
        if (perimeter[i]) {
            Points ring = b[i].points;
            if (ring.size() < 4 || ring.front() != ring.back()) { error = "Invalid native perimeter loop"; return false; }
            ring.pop_back();
            Polygon polygon(std::move(ring));
            if (polygon.is_counter_clockwise()) boundaries[zones[i]].contour = std::move(polygon);
            else boundaries[zones[i]].holes.push_back(std::move(polygon));
        } else {
            ++stats.native_paths;
            stats.native_length += unscaled(Polyline(b[i].points).length());
        }
        if (!exact && !perimeter[i]) continue;
        for (size_t j = 0; j < a.size(); ++j)
            if (!used[j] && same_printed_path(a[j], b[i])) { matches[i] = j; used[j] = true; break; }
        if (matches[i] == a.size()) { error = "Missing or changed printable path " + std::to_string(i); return false; }
    }
    std::vector<size_t> actual_zones(a.size(), zone_count);
    std::vector<bool> actual_perimeter(a.size(), false);
    for (size_t i = 0; i < b.size(); ++i)
        if (matches[i] != a.size()) {
            actual_zones[matches[i]] = zones[i];
            actual_perimeter[matches[i]] = perimeter[i];
        }
    for (size_t j = 0; j < a.size(); ++j) {
        if (!used[j]) {
            if (exact) { error = "Extra or duplicated printable path"; return false; }
            // Honeycomb alone has a documented connection difference. Its inner
            // paths still need the right material properties and geometric zone.
            for (size_t i = 0; i < b.size(); ++i)
                if (perimeter[i] && same_printed_path(a[j], b[i])) { error = "Duplicated perimeter"; return false; }
            size_t holders = 0;
            for (size_t zone = 0; zone < boundaries.size(); ++zone) {
                const bool inside = a[j].points.size() >= 2 &&
                    diff_pl(Polylines{Polyline(a[j].points)}, ExPolygons{boundaries[zone]}).empty();
                if (inside) { actual_zones[j] = zone; ++holders; }
            }
            if (holders != 1 || !same_path_attributes(a[j], b.front()) || a[j].sortable || !a[j].reversible) {
                error = "Invalid honeycomb inner path"; return false;
            }
        }
        if (!actual_perimeter[j]) {
            ++stats.api_paths;
            stats.api_length += unscaled(Polyline(a[j].points).length());
            for (size_t i = 0; i < a.size(); ++i)
                if (actual_perimeter[i] && actual_zones[i] == actual_zones[j] && !fixed_before(a[i], a[j])) {
                    error = "Perimeter is not fixed before its own fill"; return false;
                }
        }
    }
    return true;
}

ExtrusionEntity perimeter_comparison_fixture(bool reversible)
{
    const ExtrusionAttributes attributes{ExtrusionRole::SupportMaterial, ExtrusionFlow{0.08, 0.44f, 0.2f}};
    ExtrusionEntity root(ExtrusionEntity::Children{}, true, true, false);
    for (const double x : {0., 20.}) {
        ExtrusionEntity area(ExtrusionEntity::Children{}, false, false, false);
        ExtrusionEntity perimeters(ExtrusionEntity::Children{}, true, true, false);
        Polygon contour{Point::new_scale(x, 0.), Point::new_scale(x + 10., 0.),
                        Point::new_scale(x + 10., 10.), Point::new_scale(x, 10.)};
        perimeters.append_child(ExtrusionLoop(ExtrusionPath(ArcPolyline(contour.split_at_index(0)), attributes, nullptr)));
        if (x == 0.) {
            Polygon hole{Point::new_scale(3., 3.), Point::new_scale(3., 4.), Point::new_scale(4., 4.), Point::new_scale(4., 3.)};
            perimeters.append_child(ExtrusionLoop(ExtrusionPath(ArcPolyline(hole.split_at_index(0)), attributes, nullptr)));
        }
        area.append_child(std::move(perimeters));
        ExtrusionEntity fill(ExtrusionEntity::Children{}, true, true, false);
        for (const double y : {2., 7.})
            fill.append_child(ExtrusionPath(ArcPolyline(Polyline{Point::new_scale(x + 1., y), Point::new_scale(x + 9., y)}),
                                            attributes, nullptr, reversible));
        area.append_child(std::move(fill));
        root.append_child(std::move(area));
    }
    return root;
}

int32_t service_generate_pattern(const run_ctx_support_toolpath *ctx, infill_pattern_runtime_id pattern_id,
                                 const layer_handle *layer, const layer_island_handle *island,
                                 const layer_region_island_handle *region_island, const layer_region_handle *primary_region,
                                 const surface_handle *surface, const expolygon_collection_handle *no_overlap_areas,
                                 const raw_infill_pattern_params *params, extrusion_entity_handle *output)
{
    run_ctx_infill_pattern payload = {};
    payload.print = ctx->print;
    payload.object = ctx->object;
    payload.layer = layer;
    payload.island = island;
    payload.region_island = region_island;
    payload.primary_region = primary_region;
    payload.surface = surface;
    payload.no_overlap_areas = no_overlap_areas;
    payload.params = params;
    payload.output = output;
    return static_cast<Steps::InfillPatternService *>(ctx->host_context)->generate_pattern(pattern_id, payload);
}

c_flow to_api_flow(const Flow &flow)
{
    c_flow out = {};
    out.width = flow.scaled_width();
    out.spacing = flow.scaled_spacing();
    out.height = flow.scaled_height();
    out.nozzle_diameter = scale_i(flow.nozzle_diameter());
    out.is_bridge = flow.bridge() ? 1 : 0;
    out.spacing_ratio = flow.spacing_ratio();
    out.mm3_per_mm = flow.mm3_per_mm();
    return out;
}

raw_infill_pattern_params inner_recipe(infill_pattern_runtime_id pattern_id, const Flow &flow, double spacing, float density,
                                       float angle, double z)
{
    raw_infill_pattern_params params = {};
    params.pattern_id = pattern_id;
    params.surface_type = raw_surface_type(stPosInternal | stDensSparse);
    params.extrusion_role = RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL;
    params.flow = to_api_flow(flow);
    params.density = density;
    params.flow_mult = 1.f;
    params.connection = C_INFILL_CONNECTION_CONNECTED;
    params.infill_flags = C_INFILL_FLAG_DONT_ADJUST;
    params.spacing = spacing;
    params.angle = angle;
    params.bridge_angle = -1.;
    params.z = z;
    params.layer_height = unscaled(flow.scaled_height());
    params.bridge_offset = -1;
    params.fill_resolution = scale_i(0.0125);
    params.anchor_length = 1000.f;
    params.anchor_length_max = 1000.f;
    // The legacy toolpaths center every pattern on a fixed 2 mm box around the origin.
    params.object_bounding_box.min = {-scale_i(1.), -scale_i(1.)};
    params.object_bounding_box.max = {scale_i(1.), scale_i(1.)};
    return params;
}

// One inner pattern of a perimeter-wrapped fill. exact: the pattern plugin reproduces the native inner
// Fill polylines. The honeycomb plugin keeps its clipped pieces apart where the native filler connects
// them along the border (accepted gap), so only its perimeters are compared exactly and its inner fill
// is measured.
struct InnerCase { const char *plugin_id; InfillPattern pattern; float density; float angle; bool exact; };

// Results gathered over several perimeter-wrapped fill comparisons.
struct PerimeterComparison
{
    size_t multi_area_trees = 0;
    InnerFillStats honeycomb;
};

// The test runtime leaves supportbase inactive; enable it for the sheath case and restore it.
struct ActiveSupportBase
{
    bool was_active = Orchestrator::instance().is_plugin_active("supportbase");
    ActiveSupportBase() { REQUIRE(Orchestrator::instance().set_plugin_active("supportbase", true)); }
    ~ActiveSupportBase() { Orchestrator::instance().set_plugin_active("supportbase", was_active); }
};

// A print and a pattern service reached through a STEP_SUPPORT_TOOLPATH payload, as a toolpath plugin sees it.
struct PatternServiceFixture
{
    PatternServiceFixture()
    {
        Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
        service = std::make_unique<Steps::InfillPatternService>(Orchestrator::instance(), print, 1,
                                                                Steps::InfillPatternService::SetupPatterns::Refuse);
        payload.print = reinterpret_cast<const print_handle *>(&print);
        payload.object = reinterpret_cast<const object_handle *>(&print.object(0));
        payload.host_context = service.get();
        payload.generate_pattern = &service_generate_pattern;
    }
    // The plugin runtime and its configuration caches must exist before the Print is built.
    bool runtime_ready = (Test::Plugins::ensure_plugin_test_runtime_initialized(), true);
    Print print;
    Model model;
    std::unique_ptr<Steps::InfillPatternService> service;
    run_ctx_support_toolpath payload = {};
};

// The first-layer flange (rectilinear, raft density) and the sheath around supportbase and honeycomb bases.
const std::vector<InnerCase> &inner_cases();
// Flow of the perimeter-wrapped fill comparisons.
Flow support_test_flow();
// Fill one surface natively and through the API and compare the trees (see InnerCase::exact).
void compare_perimeter_fill(PatternServiceFixture &fixture, const ExPolygon &surface, const InnerCase &inner,
                            PerimeterComparison &result);
// Copy of the native test_path() (ClipperUtils.cpp, not exported): a ring too small or too narrow to keep.
bool native_small_ring(const ClipperLib::Path &path);
// Rings the native perimeter intersection of FillWithPerimeter produces and its filter then removes.
size_t native_small_intersection_rings(const ExPolygon &surface, double spacing);

const std::vector<InnerCase> &inner_cases()
{
    static const std::vector<InnerCase> cases{{"rectilinear", ipRectilinear, 0.9f, float(M_PI / 2.), true},
                                              {"supportbase", ipSupportBase, 0.3f, 0.f, true},
                                              {"honeycomb", ipHoneycomb, 0.3f, 0.f, false}};
    return cases;
}

Flow support_test_flow()
{
    return Flow::new_from_width(0.44f, 0.4f, 0.2f, 1.f);
}

void compare_perimeter_fill(PatternServiceFixture &fixture, const ExPolygon &surface, const InnerCase &inner,
                            PerimeterComparison &result)
{
    const Flow flow = support_test_flow();
    const double spacing = flow.spacing();
    const double z = 0.2;
    const PrintRegionConfig region_config;
    // Native reference: the filler and parameters of the legacy fill, as SupportCommon.cpp sets them.
    std::unique_ptr<FillWithPerimeter> native_fill(new FillWithPerimeter(Fill::new_from_type(inner.pattern)));
    native_fill->ratio_fill_inside = 0.2f;
    native_fill->set_bounding_box(BoundingBox(Point(-scale_i(1.), -scale_i(1.)), Point(scale_i(1.), scale_i(1.))));
    native_fill->z = z;
    native_fill->angle = inner.angle;
    native_fill->link_max_length = 0;
    FillParams fill_params;
    fill_params.density = inner.density;
    fill_params.dont_adjust = true;
    fill_params.config = &region_config;
    fill_params.flow = flow;
    fill_params.role = ExtrusionRole::SupportMaterial;
    native_fill->init_spacing(spacing, fill_params);
    const Surface native_surface(stPosInternal | stDensSparse, surface);
    ExtrusionEntitiesPtr native_out;
    native_fill->fill_surface_extrusion(&native_surface, fill_params, native_out);
    std::vector<std::unique_ptr<ExtrusionEntity>> native_roots;
    for (ExtrusionEntity *entity : native_out)
        native_roots.emplace_back(entity);

    // API: the same surface and recipe, the inner pattern through the pattern service.
    PluginStorage storage;
    storage_handle *storage_value = reinterpret_cast<storage_handle *>(&storage);
    const raw_infill_pattern_params recipe = inner_recipe(fixture.service->resolve_pattern_id(inner.plugin_id), flow, spacing,
                                                          inner.density, inner.angle, z);
    Common::PerimeterFillSettings settings;
    settings.spacing = spacing;
    settings.ratio_fill_inside = 0.2f;
    settings.role = RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL;
    settings.flow = to_api_flow(flow);
    slic3r_api::StoredExtrusionEntity out(storage_value);
    const slic3r_api::ExPolygon area(reinterpret_cast<const expolygon_handle *>(&surface));
    Common::fill_with_perimeter(storage_value, slic3r_api::Surface(area, raw_surface_type(stPosInternal | stDensSparse)),
                                // Direct service calls have no run context; nullptr makes the cancellation check false.
                                settings, Common::pattern_service_fill(nullptr, fixture.payload, recipe.pattern_id, recipe, nullptr, nullptr, nullptr, nullptr),
                                out.mutable_view());
    const ExtrusionEntity &api_out = *reinterpret_cast<const ExtrusionEntity *>(out.handle());
    REQUIRE(api_out.empty() == native_roots.empty());
    if (native_roots.empty())
        return;
    REQUIRE(native_roots.size() == 1);
    const ExtrusionEntity &native_root = *native_roots.front();
    InnerFillStats measured;
    std::string error;
    const bool equal = same_perimeter_fill(api_out, native_root, inner.exact, measured, error);
    INFO(error);
    REQUIRE(equal);
    if (!inner.exact) {
        result.honeycomb.native_paths += measured.native_paths;
        result.honeycomb.api_paths += measured.api_paths;
        result.honeycomb.native_length += measured.native_length;
        result.honeycomb.api_length += measured.api_length;
    }
    if (native_root.child_count() > 0 && !native_root.child(0).is_leaf() && native_root.can_sort())
        ++result.multi_area_trees;
}

bool native_small_ring(const ClipperLib::Path &path)
{
    double area = ClipperLib::Area(path);
    ClipperLib::cInt max_dist_sqr = 0;
    const ClipperLib::IntPoint centroid = ClipperLib::Centroid(path, area);
    area = std::abs(area);
    for (const ClipperLib::IntPoint &pt : path) {
        const ClipperLib::cInt dx = std::abs(pt.x() - centroid.x()) & 0x3FFFFFFF;
        const ClipperLib::cInt dy = std::abs(pt.y() - centroid.y()) & 0x3FFFFFFF;
        max_dist_sqr = max_dist_sqr | (dx * dx + dy * dy);
    }
    return area < (2 * SCALED_EPSILON) * std::sqrt(max_dist_sqr);
}

size_t native_small_intersection_rings(const ExPolygon &surface, double spacing)
{
    // The first two steps of FillWithPerimeter::fill_surface_extrusion(), then the first pass of
    // intersection_ex() (clipper_do_polytree), whose output remove_small_areas() filters.
    const coord_t offset_for_overlap = scale_d(spacing / 2) * ((1 - 0.) / 2);
    const ExPolygons shrunk = offset2_ex(ExPolygons{surface}, scale_d(-spacing) / 2 - offset_for_overlap, offset_for_overlap,
                                         ClipperLib::jtMiter, scale_d(spacing) * 10);
    const ExPolygons limit = offset_ex(surface, scale_d(-spacing / 2));
    ClipperLib::Clipper clipper;
    for (const ExPolygon &expolygon : shrunk) {
        clipper.AddPath(expolygon.contour.points, ClipperLib::ptSubject, true);
        for (const Polygon &hole : expolygon.holes)
            clipper.AddPath(hole.points, ClipperLib::ptSubject, true);
    }
    for (const ExPolygon &expolygon : limit) {
        clipper.AddPath(expolygon.contour.points, ClipperLib::ptClip, true);
        for (const Polygon &hole : expolygon.holes)
            clipper.AddPath(hole.points, ClipperLib::ptClip, true);
    }
    ClipperLib::Paths rings;
    clipper.Execute(ClipperLib::ctIntersection, rings, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    return size_t(std::count_if(rings.begin(), rings.end(), native_small_ring));
}
} // namespace

// The explicit angle guard must retain the demand/grid difference at the threshold's 90-degree boundary.
TEST_CASE("Support overhang margins retain guarded and unguarded angle rules", "[plugins][support-common]")
{
    const coord_t height = scale_to_layer_coord(0.2);
    const coord_t width = scale_i(0.4);
    for (const bool guard : {false, true}) {
        CHECK(Common::overhang_margin(0, height, width, guard) == width / 2);
        CHECK(Common::overhang_margin(45, height, width, guard) ==
              scale_i(unscaled(height) / std::tan(PI * 46. / 180.)));
    }
    CHECK(Common::overhang_margin(89, height, width, true) == width / 2);
    CHECK(Common::overhang_margin(89, height, width, false) ==
          scale_i(unscaled(height) / std::tan(PI * 90. / 180.)));
    CHECK(Common::overhang_margin(90, height, width, true) == width / 2);
    // At 91 degrees the unguarded XY allowance is negative rather than a layer-Z conversion failure.
    CHECK(unscaled(height) / std::tan(PI * 91. / 180.) < 0.);
    CHECK(Common::overhang_margin(90, height, width, false) == scale_i(unscaled(height) / std::tan(PI * 91. / 180.)));
}

// Shared attributes and stored deposits must allow seam placement, with the physical flow values retained.
TEST_CASE("Support path attributes and deposits initialize no_seam to zero", "[plugins][support-common]")
{
    c_flow flow{};
    flow.width = scale_i(0.4);
    flow.height = scale_i(0.2);
    flow.mm3_per_mm = 0.08;
    const slic3r_api::EPropertyAttributes attributes = Common::path_attributes(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL, flow);
    CHECK(attributes.no_seam == 0);
    CHECK(attributes.mm3_per_mm() == flow.mm3_per_mm);
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const ExPolygon area(Polygon::new_scale({Vec2d(0, 0), Vec2d(1, 0), Vec2d(1, 1), Vec2d(0, 1)}));
    slic3r_api::StoredExtrusionEntity output(handle);
    Common::append_deposit(handle, output.mutable_view(), slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&area)), flow);
    REQUIRE(output.readonly().child_count() == 1);
    const slic3r_api::EPropertyAttributes *stored = output.readonly().child(0).get(slic3r_api::EPropertyAttributes::key);
    REQUIRE(stored != nullptr);
    CHECK(stored->no_seam == 0);
    CHECK(stored->mm3_per_mm() == flow.mm3_per_mm);
}

// Ordered copies must preserve ring starts and orientation, including appends after existing destination rings.
TEST_CASE("Support ring copies preserve contour then holes and area order", "[plugins][support-common]")
{
    const Polygon contour = Polygon::new_scale({Vec2d(0, 0), Vec2d(100, 0), Vec2d(100, 100), Vec2d(0, 100)});
    const Polygon hole = Polygon::new_scale({Vec2d(10, 10), Vec2d(10, 20), Vec2d(20, 20), Vec2d(20, 10)});
    const Polygon second_hole = Polygon::new_scale({Vec2d(30, 30), Vec2d(30, 40), Vec2d(40, 40), Vec2d(40, 30)});
    const Polygon other = Polygon::new_scale({Vec2d(200, 0), Vec2d(300, 0), Vec2d(300, 100), Vec2d(200, 100)});
    const ExPolygons areas{ExPolygon(contour, Polygons{hole, second_hole}), ExPolygon(other)};
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const slic3r_api::ExPolygonCollection view(reinterpret_cast<const expolygon_collection_handle *>(&areas));
    const slic3r_api::StoredPolygonCollection rings = Common::to_polygons(handle, view);
    CHECK(SG::native(rings) == Polygons{contour, hole, second_hole, other});
    slic3r_api::StoredPolygonCollection appended(handle);
    appended.push_back(slic3r_api::Polygon(reinterpret_cast<const polygon_handle *>(&other)));
    Common::append_polygons(appended, view.at(0));
    Common::append_polygons(appended, view);
    CHECK(SG::native(appended) == Polygons{other, contour, hole, second_hole, contour, hole, second_hole, other});
    const ExPolygons empty;
    const slic3r_api::ExPolygonCollection empty_view(reinterpret_cast<const expolygon_collection_handle *>(&empty));
    CHECK(Common::to_polygons(handle, empty_view).empty());
    // A near-closing last vertex survives host flattening but native individual insertion removes it on move.
    ExPolygons tiny(1);
    tiny.front().contour.points = {Point(0, 0), Point(1000, 0), Point(1000, 1000), Point(0, 10)};
    const slic3r_api::ExPolygon tiny_view(reinterpret_cast<const expolygon_handle *>(&tiny.front()));
    slic3r_api::StoredPolygonCollection expected(handle);
    expected.push_back(tiny_view.contour());
    for (const slic3r_api::Polygon ring : tiny_view.holes())
        expected.push_back(ring);
    CHECK(SG::native(Common::expolygon_rings(handle, tiny_view)) == SG::native(expected));
    const std::vector<slic3r_api::Polygon> tiny_rings = Common::polygon_rings(tiny_view);
    REQUIRE(tiny_rings.size() == 1);
    CHECK(tiny_rings.front().size() == 4);
    CHECK(expected.at(0).size() == 3);
    const slic3r_api::ExPolygonCollection tiny_areas(reinterpret_cast<const expolygon_collection_handle *>(&tiny));
    CHECK(Common::to_polygons(handle, tiny_areas).at(0).size() == 4);
}

TEST_CASE("Support params equal the native grid support parameters", "[plugins][support-common]")
{
    for (const ParamsCase &test : params_cases()) {
        CAPTURE(test.style, test.soluble, test.interfaces, test.same_tool, test.overrides.size(), test.disabled.size());
        DynamicPrintConfig config = SG::config(test.style, test.soluble, test.interfaces, test.same_tool);
        for (const std::pair<std::string, std::string> &item : test.overrides)
            config.set_deserialize_strict(item.first, item.second);
        for (const std::string &key : test.disabled) {
            REQUIRE(config.option(key) != nullptr);
            config.option(key)->set_enabled(false);
        }
        Print print;
        Model model;
        Test::init_print({SG::slab()}, print, model, config);
        Orchestrator &host = Orchestrator::instance();
        Steps::StepLayerHeightGeneration::prepare(print);
        Steps::StepLayerHeightGeneration::run_step(host, print);
        Steps::StepSlicing::run_step(host, print);
        const PrintObject &object = print.object(0);

        const NativeGridParams native(object);
        const Common::SupportParams params(slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
                                           slic3r_api::Object(reinterpret_cast<const object_handle *>(&object)));
        CHECK(params.first_layer_height == native.first_layer_height);
        CHECK(params.layer_height == native.layer_height);
        CHECK(params.max_support_height == native.max_support_height);
        CHECK(params.gap_above == native.gap_above);
        CHECK(params.gap_below == native.gap_below);
        CHECK(params.z_step == native.z_step);
        CHECK(params.soluble == native.soluble);
        CHECK(params.top_interfaces == native.top_interfaces);
        CHECK(params.bottom_interfaces == native.bottom_interfaces);
        check_flow(params.first_flow, native.first_flow);
        check_flow(params.base_flow, native.base_flow);
        check_flow(params.interface_flow, native.interface_flow);
        CHECK(params.bottom_expansion.value == native.bottom_expansion.value);
        CHECK((params.bottom_expansion.percent != 0) == native.bottom_expansion.percent);
        CHECK(params.min_height == native.min_height);
        // Native snaps this XY quantity to a Z step; the plugin uses exact XY truncation instead.
        const slic3r_api::Object object_view = SG::object_view(object);
        double external_width = 0.;
        for (uint32_t region = 0; region < object_view.print_region_count(); ++region)
            external_width = std::max(external_width, object_view.region_flow_mm(region,
                raw_extrusion_role(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER), params.layer_height, 2).width);
        CHECK(params.gap_xy == scale_i(object_view.config().get("support_material_xy_spacing").get_effective_value(external_width)));
        CHECK(std::abs(params.gap_xy - native.gap_xy) <= SCALED_LAYER_Z_STEP);
        CHECK(params.resolution == native.resolution);
        // The millimeter flows are the native float values, so the densities are exact.
        check_flow_mm(params.first_flow_mm, native.first_flow);
        check_flow_mm(params.base_flow_mm, native.base_flow);
        check_flow_mm(params.interface_flow_mm, native.interface_flow);
        check_flow_mm(params.bottom_flow_mm, native.bottom_flow);
        CHECK(params.interface_density == native.interface_density);

        // Path settings, as the legacy support code (SupportParameters.cpp) derives them.
        const PrintObjectConfig &object_config = object.config();
        const int32_t top_count = object_config.support_material_interface_layers.value;
        const int32_t bottom_count = object_config.support_material_bottom_interface_layers.value;
        const bool bottom_count_enabled = object_config.support_material_bottom_interface_layers.is_enabled();
        CHECK(params.top_interface_layers == top_count);
        CHECK(params.bottom_interface_layers == bottom_count);
        CHECK(params.bottom_interface_count_enabled == bottom_count_enabled);
        CHECK(params.top_interfaces_enabled == (top_count > 0));
        CHECK(params.bottom_interfaces_enabled == ((bottom_count_enabled ? bottom_count : top_count) > 0));
        CHECK(params.top_published_interfaces_enabled == (top_count != 0));
        CHECK(params.bottom_published_interfaces_enabled == (top_count != 0 && (!bottom_count_enabled || bottom_count != 0)));
        CHECK(params.base_tool == object_config.support_material_extruder.value);
        CHECK(params.interface_tool == object_config.support_material_interface_extruder.value);
        CHECK(params.base_angle_degrees == object_config.support_material_angle.value);
        Common::SlabSpacing spacing;
        spacing.base_step = 123;
        spacing.interface_step = 456;
        Common::set_interface_counts(spacing, params);
        CHECK(spacing.interface_layers == std::max(0, top_count));
        CHECK(spacing.bottom_interface_layers == std::max(0, bottom_count_enabled ? bottom_count : top_count));
        CHECK(spacing.base_step == 123);
        CHECK(spacing.interface_step == 456);
        // An explicit zero enables the option but disables both bottom interface roles and their slab counts.
        if (std::find(test.overrides.begin(), test.overrides.end(),
                std::pair<std::string, std::string>{"support_material_bottom_interface_layers", "0"}) != test.overrides.end()) {
            REQUIRE(bottom_count_enabled);
            CHECK_FALSE(params.bottom_interfaces_enabled);
            CHECK_FALSE(params.bottom_published_interfaces_enabled);
            CHECK(params.bottom_interfaces == 0);
            CHECK(spacing.bottom_interface_layers == 0);
        }
        const std::vector<slic3r_api::Layer> layers = Common::object_layers(SG::object_view(object));
        REQUIRE(layers.size() == object.layer_count());
        for (size_t id = 0; id < layers.size(); ++id)
            CHECK(layers[id].print_z() == object.layer(id).scaled_print_z());
        CHECK_FALSE(Common::object_has_support_demand(SG::object_view(object)));
        const double native_density = std::min(1., native.base_flow.spacing() /
            (object_config.support_material_spacing.value + native.base_flow.spacing()));
        CHECK(params.base_density == native_density);
        const InfillPattern base_pattern = object_config.support_material_pattern.value == smpHoneycomb ? ipHoneycomb :
            native_density > 0.95 || object_config.support_material_with_sheath.value ? ipRectilinear : ipSupportBase;
        InfillPattern top_pattern = object_config.support_material_top_interface_pattern.value;
        InfillPattern bottom_pattern = object_config.support_material_bottom_interface_pattern.value;
        if (top_pattern == ipAuto)
            top_pattern = native_auto_interface(native.soluble, native.interface_density);
        if (bottom_pattern == ipAuto)
            bottom_pattern = top_pattern != ipHilbertCurve && top_pattern != ipSmooth && top_pattern != ipSawtooth ? top_pattern :
                native_auto_interface(native.soluble, native.interface_density);
        CHECK(params.base_pattern_id == pattern_name(base_pattern));
        CHECK(params.top_pattern_id == pattern_name(top_pattern));
        CHECK(params.bottom_pattern_id == pattern_name(bottom_pattern));
        CHECK(params.with_sheath == object_config.support_material_with_sheath.value);
        CHECK(params.interface_contact_loops == object_config.support_material_interface_contact_loops.value);
        CHECK(params.base_angle == Geometry::deg2rad(float(object_config.support_material_angle.value)));
        CHECK(params.interface_angle == Geometry::deg2rad(float(object_config.support_material_interface_angle.value)));
        CHECK(params.angle_height == object_config.support_material_angle_height.value);
        CHECK(params.interface_angle_increment == object_config.support_material_interface_angle_increment.value);
        CHECK(params.raft_first_layer_density == Approx(object_config.raft_first_layer_density.get_effective_value(1.)).epsilon(1e-12));
        CHECK(params.style == (std::string(test.style) == "grid" ? Common::SupportStyle::Grid : Common::SupportStyle::Snug));
    }
}

// Build parameters for locally owned two-tool prints without generating layers; construction failures or differences from native bounds fail the test.
TEST_CASE("Support params apply native layer height order with mixed nozzles", "[plugins][support-common]")
{
    for (const bool inverted : {false, true}) {
        CAPTURE(inverted);
        DynamicPrintConfig config = SG::config("grid", false, 3, false);
        config.set_deserialize_strict("nozzle_diameter", "0.8,0.2");
        config.set_deserialize_strict("min_layer_height", inverted ? "50%,25%" : "12.5%,25%");
        config.set_deserialize_strict("max_layer_height", "75%,75%");
        config.set_deserialize_strict("layer_height", "0.2");
        config.set_deserialize_strict("first_layer_height", "0.4");
        config.set_deserialize_strict("z_step", "0.01");
        config.set_deserialize_strict("support_material_extruder", "1");
        config.set_deserialize_strict("support_material_interface_extruder", "2");
        Print print;
        Model model;
        ModelObject *model_object = model.add_object();
        model_object->add_volume(SG::slab());
        model_object->add_instance();
        model_object->ensure_on_bed();
        print.auto_assign_extruders(model_object);
        print.apply(model, config);
        // Native layer-profile generation requires ordered bounds, but parameter derivation accepts either order.
        const PrintObject &object = print.object(0);
        const SlicingParameters &native = object.slicing_parameters();
        REQUIRE(native.min_layer_height == Approx(inverted ? 0.4 : 0.1));
        REQUIRE(native.max_layer_height == Approx(0.15));
        REQUIRE((native.min_layer_height > native.max_layer_height) == inverted);
        const Common::SupportParams params = SG::params(print, object);
        CHECK(params.layer_height == native.layer_height);
        CHECK(params.layer_height == native.max_layer_height);
        CHECK(params.layer_height == Approx(0.15));
    }
}

TEST_CASE("Support params use the unset minimum layer height of small nozzles", "[plugins][support-common]")
{
    // With min_layer_height at 0 on 0.05 mm nozzles, the support parameters and the host slicing parameters both use the shared unset minimum.
    DynamicPrintConfig config = SG::config("grid", false, 3, false);
    config.set_deserialize_strict("nozzle_diameter", "0.05,0.05");
    config.set_deserialize_strict("min_layer_height", "0,0");
    config.set_deserialize_strict("max_layer_height", "0.04,0.04");
    config.set_deserialize_strict("layer_height", "0.025");
    config.set_deserialize_strict("first_layer_height", "0.04");
    config.set_deserialize_strict("z_step", "0");
    config.set_deserialize_strict("support_material_extruder", "1");
    config.set_deserialize_strict("support_material_interface_extruder", "2");
    Print print;
    Model model;
    ModelObject *model_object = model.add_object();
    model_object->add_volume(SG::slab());
    model_object->add_instance();
    model_object->ensure_on_bed();
    print.auto_assign_extruders(model_object);
    print.apply(model, config);
    const PrintObject &object = print.object(0);
    const SlicingParameters &native = object.slicing_parameters();
    REQUIRE(native.min_layer_height == Approx(slic3r_api::min_layer_height_mm(0., 0.05)));
    REQUIRE(native.layer_height == Approx(0.025));
    const Common::SupportParams params = SG::params(print, object);
    CHECK(params.layer_height == native.layer_height);
    CHECK(params.support_min_height == Approx(slic3r_api::min_layer_height_mm(0., 0.05)));
}

TEST_CASE("Support enum helpers read every serialized value", "[plugins][support-common]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto values = [](const char *key) {
        const ConfigOptionDef *def = PrintConfigDef::instance().get(key);
        REQUIRE(def != nullptr);
        REQUIRE(def->enum_def.get() != nullptr);
        return def->enum_def->values();
    };
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const slic3r_api::Config view(ApiHost::to_config_handle(&config));

    for (const std::string &value : values("support_material_style")) {
        CAPTURE(value);
        config.set_deserialize_strict("support_material_style", value);
        const SupportMaterialStyle native = config.opt_enum<SupportMaterialStyle>("support_material_style");
        const Common::SupportStyle expected = native == smsGrid ? Common::SupportStyle::Grid :
            native == smsSnug ? Common::SupportStyle::Snug : native == smsTree ? Common::SupportStyle::Tree :
            native == smsOrganic ? Common::SupportStyle::Organic : native == smsFilled ? Common::SupportStyle::Filled :
            native == smsGridMerged ? Common::SupportStyle::GridMerged : Common::SupportStyle::Unknown;
        CHECK(expected != Common::SupportStyle::Unknown);
        CHECK(Common::support_style(view) == expected);
    }
    for (const std::string &value : values("support_material_contact_distance_type")) {
        CAPTURE(value);
        config.set_deserialize_strict("support_material_contact_distance_type", value);
        const SupportZDistanceType native = config.opt_enum<SupportZDistanceType>("support_material_contact_distance_type");
        const Common::ContactDistanceType expected = native == zdFilament ? Common::ContactDistanceType::Filament :
            native == zdPlane ? Common::ContactDistanceType::Plane : native == zdNone ? Common::ContactDistanceType::None :
            Common::ContactDistanceType::Unknown;
        CHECK(expected != Common::ContactDistanceType::Unknown);
        CHECK(Common::contact_distance_type(view) == expected);
    }
    for (const std::string &value : values("support_material_pattern")) {
        CAPTURE(value);
        config.set_deserialize_strict("support_material_pattern", value);
        const SupportMaterialPattern native = config.opt_enum<SupportMaterialPattern>("support_material_pattern");
        const Common::BasePattern expected = native == smpRectilinear ? Common::BasePattern::Rectilinear :
            native == smpRectilinearGrid ? Common::BasePattern::RectilinearGrid :
            native == smpHoneycomb ? Common::BasePattern::Honeycomb : Common::BasePattern::Unknown;
        CHECK(expected != Common::BasePattern::Unknown);
        const Common::BasePattern pattern = Common::base_pattern(view);
        CHECK(pattern == expected);
        // The base pattern id follows the native PathParams choice at both sides of the 0.95 density threshold.
        for (const double density : {0.3, 0.96})
            for (const bool sheath : {false, true}) {
                const InfillPattern native_id = native == smpHoneycomb ? ipHoneycomb :
                    density > 0.95 || sheath ? ipRectilinear : ipSupportBase;
                CHECK(Common::base_pattern_plugin_id(pattern, density, sheath) == pattern_name(native_id));
            }
    }
    CHECK(Common::parse_support_style("tree") == Common::SupportStyle::Tree);
    CHECK(Common::parse_support_style("filled") == Common::SupportStyle::Filled);
    CHECK(Common::parse_contact_distance_type("") == Common::ContactDistanceType::Unknown);
    CHECK(Common::parse_base_pattern("lightning") == Common::BasePattern::Unknown);

    // Interface patterns: every top value, then every bottom value after every top value, soluble or
    // not, on both sides of the 0.95 interface density threshold.
    const std::vector<std::string> top_values = values("support_material_top_interface_pattern");
    const std::vector<std::string> bottom_values = values("support_material_bottom_interface_pattern");
    for (const bool soluble : {false, true})
        for (const double density : {0.4, 0.96})
            for (const std::string &top : top_values) {
                CAPTURE(soluble, density, top);
                if (top == "concentricgapfill") {
                    // Listed in the option but absent from the native InfillPattern map, so no native
                    // config holds it; the helper passes it on for the pattern service to report.
                    CHECK(Common::top_interface_pattern_plugin_id(top, soluble, density) == top);
                    continue;
                }
                config.set_deserialize_strict("support_material_top_interface_pattern", top);
                InfillPattern native_top = config.opt_enum<InfillPattern>("support_material_top_interface_pattern");
                if (native_top == ipAuto)
                    native_top = native_auto_interface(soluble, density);
                const std::string top_id = Common::top_interface_pattern_plugin_id(top, soluble, density);
                CHECK(top_id == pattern_name(native_top));
                for (const std::string &bottom : bottom_values) {
                    CAPTURE(bottom);
                    config.set_deserialize_strict("support_material_bottom_interface_pattern", bottom);
                    InfillPattern native_bottom = config.opt_enum<InfillPattern>("support_material_bottom_interface_pattern");
                    if (native_bottom == ipAuto)
                        native_bottom = native_top != ipHilbertCurve && native_top != ipSmooth && native_top != ipSawtooth ?
                            native_top : native_auto_interface(soluble, density);
                    CHECK(Common::bottom_interface_pattern_plugin_id(bottom, top_id, soluble, density) == pattern_name(native_bottom));
                }
            }
}

TEST_CASE("Support plugins declare their keys from the Common library", "[plugins][support-common]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Common::UsedKeyList geometry = Common::support_geometry_keys();
    const Common::UsedKeyList path = Common::support_path_keys();
    const auto declares = [](const Plugin &plugin, const Common::UsedKeyList &list) {
        for (size_t idx = 0; idx < list.size; ++idx) {
            const std::string key = list.keys[idx].key;
            CAPTURE(plugin.get_id(), key);
            CHECK(std::any_of(plugin.get_used_config_keys().begin(), plugin.get_used_config_keys().end(),
                              [&key](const Plugin::UsedConfigKey &used) { return used.key == key; }));
        }
    };
    const Plugin *family = Orchestrator::instance().get_plugin("support.family.grid");
    REQUIRE(family != nullptr);
    declares(*family, geometry);
    // The merged grid style also reads the shared overhang limit.
    CHECK(family->get_used_config_keys().size() == geometry.size + 1);
    CHECK(std::any_of(family->get_used_config_keys().begin(), family->get_used_config_keys().end(),
                      [](const Plugin::UsedConfigKey &used) { return used.key == "support_max_overhang"; }));
    // The filled family reads raft_layers only to reject a raft; other families and common key lists do not declare it.
    const auto declares_raft = [](const Plugin &plugin) {
        return std::any_of(plugin.get_used_config_keys().begin(), plugin.get_used_config_keys().end(),
                           [](const Plugin::UsedConfigKey &used) { return used.key == "raft_layers"; });
    };
    for (size_t idx = 0; idx < geometry.size; ++idx)
        CHECK(std::string(geometry.keys[idx].key) != "raft_layers");
    for (size_t idx = 0; idx < path.size; ++idx)
        CHECK(std::string(path.keys[idx].key) != "raft_layers");
    for (const char *id : {"support.family.grid", "support.family.tree", "support.demand.bridge_removal"}) {
        const Plugin *plugin = Orchestrator::instance().get_plugin(id);
        REQUIRE(plugin != nullptr);
        CAPTURE(id);
        CHECK_FALSE(declares_raft(*plugin));
    }
    for (const char *id : {"support.toolpath.legacy_sparse", "support.toolpath.legacy_bottom", "support.toolpath.legacy_top",
                           "support.toolpath.legacy_bottom_contact", "support.toolpath.legacy_top_contact"}) {
        const Plugin *toolpath = Orchestrator::instance().get_plugin(id);
        REQUIRE(toolpath != nullptr);
        declares(*toolpath, geometry);
        declares(*toolpath, path);
        CHECK(toolpath->get_used_config_keys().size() == geometry.size + path.size);
    }
    // A null buffer returns the count; a buffer receives the lists one after the other.
    std::vector<raw_used_config_key> keys(geometry.size + path.size);
    CHECK(Common::append_used_config_keys(nullptr, 0, geometry) == int32_t(geometry.size));
    const int32_t count = Common::append_used_config_keys(keys.data(), 0, geometry);
    CHECK(Common::append_used_config_keys(keys.data(), count, path) == int32_t(keys.size()));
    CHECK(std::string(keys.back().key) == path.keys[path.size - 1].key);
}

TEST_CASE("Perimeter comparison accepts independent permutations and grouping", "[plugins][support-common][support-comparison]")
{
    const ExtrusionEntity expected = perimeter_comparison_fixture(true);
    ExtrusionEntity actual(expected);
    SECTION("independent areas and fill paths") {
        std::swap(actual.children()[0], actual.children()[1]);
        for (const ExtrusionEntityUPtr &area : actual.children())
            std::reverse(area->child(1).children().begin(), area->child(1).children().end());
    }
    SECTION("different grouping with all perimeters before their fills") {
        actual = ExtrusionEntity(ExtrusionEntity::Children{}, false, false, false);
        for (const size_t index : {1u, 0u}) actual.append_child(expected.child(index).child(0));
        for (const size_t index : {0u, 1u}) {
            ExtrusionEntity wrapper(ExtrusionEntity::Children{}, false, false, false);
            wrapper.append_child(expected.child(index).child(1));
            actual.append_child(std::move(wrapper));
        }
    }
    SECTION("independent fill islands inside a fixed area") {
        ExtrusionEntity &area = actual.child(0);
        const ExtrusionEntity fill(area.child(1));
        area.remove_child(1);
        area.append_child(fill.child(1));
        area.append_child(fill.child(0));
    }
    SECTION("cyclic loop start") {
        ExtrusionEntity &path = actual.child(0).child(0).child(0).child(0);
        Points points = path.polyline_ref().to_polyline().points;
        points.pop_back();
        std::rotate(points.begin(), points.begin() + 1, points.end());
        points.push_back(points.front());
        path.set_polyline(ArcPolyline(Polyline(std::move(points))));
    }
    SECTION("permitted open path reversal") {
        actual.child(0).child(1).child(0).reverse();
    }
    SECTION("effective attributes inherited through a wrapper") {
        ExtrusionEntity &fill = actual.child(0).child(1);
        fill.add_property(*fill.child(0).get_property<ExtrusionAttributes>());
        for (const ExtrusionEntityUPtr &path : fill.children()) path->remove_property<ExtrusionAttributes>();
    }
    InnerFillStats stats;
    std::string error;
    const bool equal = same_perimeter_fill(actual, expected, true, stats, error);
    INFO(error);
    CHECK(equal);
    CHECK(stats.api_paths == 4);
    CHECK(stats.api_paths == stats.native_paths);
}

TEST_CASE("Perimeter comparison rejects lost geometry properties and printing constraints", "[plugins][support-common][support-comparison]")
{
    const ExtrusionEntity expected = perimeter_comparison_fixture(false);
    ExtrusionEntity actual(expected);
    ExtrusionEntity &fill = actual.child(0).child(1);
    SECTION("missing path") { fill.remove_child(0); }
    SECTION("duplicated path") { fill.append_child(fill.child(0)); }
    SECTION("changed coordinate") {
        Points points = fill.child(0).polyline_ref().to_polyline().points;
        points.front().x() += 1;
        fill.child(0).set_polyline(ArcPolyline(Polyline(std::move(points))));
    }
    SECTION("changed role") { fill.child(0).get_property<ExtrusionAttributes>()->role = ExtrusionRole::SupportMaterialInterface; }
    SECTION("changed flow") { fill.child(0).get_property<ExtrusionAttributes>()->mm3_per_mm *= 2.; }
    SECTION("changed width") { fill.child(0).get_property<ExtrusionAttributes>()->width += 0.01f; }
    SECTION("changed height") { fill.child(0).get_property<ExtrusionAttributes>()->height += 0.01f; }
    SECTION("changed seam permission") { fill.child(0).get_property<ExtrusionAttributes>()->no_seam = 1; }
    SECTION("forbidden open path reversal") { fill.child(0).reverse(); }
    SECTION("changed reversal permission") { fill.child(0).set_can_sort_reverse(false, true); }
    SECTION("reversed perimeter winding") { actual.child(0).child(0).child(0).child(0).reverse(); }
    SECTION("fill before its own perimeter") { std::swap(actual.child(0).children()[0], actual.child(0).children()[1]); }
    SECTION("sortable perimeter and fill") { actual.child(0).set_can_sort_reverse(true, true); }
    SECTION("reversible perimeter and fill") { actual.child(0).set_can_sort_reverse(false, true); }
    SECTION("broken continuous path") {
        const ExtrusionEntity path(fill.child(0));
        const ExtrusionAttributes attributes = *path.get_property<ExtrusionAttributes>();
        const Points points = path.polyline_ref().to_polyline().points;
        fill.remove_child(0);
        const Point middle = (points.front() + points.back()) / 2;
        for (const Polyline &piece : Polylines{Polyline{points.front(), middle}, Polyline{middle, points.back()}})
            fill.append_child(ExtrusionPath(ArcPolyline(piece), attributes, nullptr, false));
    }
    InnerFillStats stats;
    std::string error;
    CHECK_FALSE(same_perimeter_fill(actual, expected, true, stats, error));
    CHECK_FALSE(error.empty());
}

TEST_CASE("Perimeter-wrapped support fill equals the native FillWithPerimeter", "[plugins][support-common]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ActiveSupportBase active_supportbase;
    PatternServiceFixture fixture;

    // A rectangle, an L with a hole, and two squares joined by a neck the perimeter inset splits.
    std::vector<ExPolygon> surfaces;
    surfaces.emplace_back(Polygon({Point(0, 0), Point(scale_i(10.), 0), Point(scale_i(10.), scale_i(6.)), Point(0, scale_i(6.))}));
    ExPolygon l_shape(Polygon({Point(0, 0), Point(scale_i(12.), 0), Point(scale_i(12.), scale_i(4.)), Point(scale_i(4.), scale_i(4.)),
                               Point(scale_i(4.), scale_i(12.)), Point(0, scale_i(12.))}));
    Polygon hole({Point(scale_i(1.), scale_i(1.)), Point(scale_i(1.), scale_i(2.5)), Point(scale_i(2.5), scale_i(2.5)), Point(scale_i(2.5), scale_i(1.))});
    hole.make_clockwise();
    l_shape.holes.push_back(hole);
    surfaces.push_back(l_shape);
    surfaces.emplace_back(Polygon({Point(0, 0), Point(scale_i(6.), 0), Point(scale_i(6.), scale_i(2.8)), Point(scale_i(6.4), scale_i(2.8)),
                                   Point(scale_i(6.4), 0), Point(scale_i(12.4), 0), Point(scale_i(12.4), scale_i(6.)),
                                   Point(scale_i(6.4), scale_i(6.)), Point(scale_i(6.4), scale_i(3.2)), Point(scale_i(6.), scale_i(3.2)),
                                   Point(scale_i(6.), scale_i(6.)), Point(0, scale_i(6.))}));

    PerimeterComparison result;
    for (const InnerCase &inner : inner_cases())
        for (size_t surface_idx = 0; surface_idx < surfaces.size(); ++surface_idx) {
            CAPTURE(inner.plugin_id, surface_idx);
            compare_perimeter_fill(fixture, surfaces[surface_idx], inner, result);
        }
    // The neck case must exercise the sortable root with one node per area.
    CHECK(result.multi_area_trees > 0);
    // Characterisation of the accepted honeycomb gap: the plugin leaves more, shorter paths.
    CAPTURE(result.honeycomb.native_paths, result.honeycomb.native_length, result.honeycomb.api_paths, result.honeycomb.api_length);
    CHECK(result.honeycomb.api_paths > result.honeycomb.native_paths);
    CHECK(result.honeycomb.api_length < result.honeycomb.native_length);
}

// The native intersection_ex() drops the rings too small or too narrow to keep before its union
// pass; the helper must drop the same ones. Seeded star shapes are searched for surfaces whose
// perimeter intersection leaves such rings, and those are compared with the native filler.
TEST_CASE("Perimeter-wrapped support fill drops the small intersection rings like native", "[plugins][support-common]")
{
    PatternServiceFixture fixture;
    const double spacing = support_test_flow().spacing();
    std::mt19937 rng(20261008);
    std::uniform_real_distribution<double> radius(0.3, 6.);
    std::vector<ExPolygon> residue_surfaces;
    size_t removed_rings = 0;
    for (size_t shape = 0; shape < 5000 && residue_surfaces.size() < 4; ++shape) {
        const size_t vertices = 5 + shape % 60;
        Polygon star;
        for (size_t idx = 0; idx < vertices; ++idx) {
            const double angle = 2. * M_PI * double(idx) / double(vertices);
            const double r = radius(rng);
            star.points.emplace_back(scale_i(r * std::cos(angle)), scale_i(r * std::sin(angle)));
        }
        star.make_counter_clockwise();
        const ExPolygon surface(star);
        if (const size_t removed = native_small_intersection_rings(surface, spacing); removed > 0) {
            residue_surfaces.push_back(surface);
            removed_rings += removed;
        }
    }
    CAPTURE(residue_surfaces.size(), removed_rings);
    REQUIRE(residue_surfaces.size() == 4);
    CHECK(removed_rings >= residue_surfaces.size());
    PerimeterComparison result;
    for (size_t surface_idx = 0; surface_idx < residue_surfaces.size(); ++surface_idx) {
        CAPTURE(surface_idx);
        compare_perimeter_fill(fixture, residue_surfaces[surface_idx], inner_cases().front(), result);
    }
}
// Evidence for the sheath gap: with the same recipe, the honeycomb plugin gives the pieces of the native
// unconnected honeycomb, ignores the connection field, and the native connected honeycomb joins them.
TEST_CASE("Honeycomb plugin pieces equal the native honeycomb without connection", "[plugins][support-common]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    REQUIRE(host.is_plugin_active("honeycomb"));
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
    Steps::InfillPatternService service(host, print, 1, Steps::InfillPatternService::SetupPatterns::Refuse);
    run_ctx_support_toolpath payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.object = reinterpret_cast<const object_handle *>(&print.object(0));
    payload.host_context = &service;
    payload.generate_pattern = &service_generate_pattern;

    const ExPolygon area(Polygon({Point(0, 0), Point(scale_i(10.), 0), Point(scale_i(10.), scale_i(6.)), Point(0, scale_i(6.))}));
    const Flow flow = Flow::new_from_width(0.44f, 0.4f, 0.2f, 1.f);
    const double spacing = flow.spacing();
    const float density = 0.3f;
    const PrintRegionConfig region_config;

    // Native honeycomb with the Fill members and FillParams the sheath gives its inner filler.
    const auto native_paths = [&](InfillConnection connection) {
        std::unique_ptr<Fill> fill(Fill::new_from_type(ipHoneycomb));
        fill->set_bounding_box(BoundingBox(Point(-scale_i(1.), -scale_i(1.)), Point(scale_i(1.), scale_i(1.))));
        fill->z = 0.2;
        fill->angle = 0.f;
        fill->link_max_length = 0;
        FillParams params;
        params.density = density;
        params.dont_adjust = true;
        params.flow = flow;
        params.role = ExtrusionRole::SupportMaterial;
        params.connection = connection;
        params.config = &region_config;
        fill->init_spacing(spacing, params);
        const Surface surface(stPosInternal | stDensSparse, area);
        return fill->fill_surface(&surface, params);
    };
    // Plugin honeycomb through the pattern service with the recipe of the perimeter-wrapped fill.
    const auto plugin_paths = [&](c_infill_connection connection) {
        raw_infill_pattern_params recipe = inner_recipe(service.resolve_pattern_id("honeycomb"), flow, spacing, density, 0.f, 0.2);
        recipe.connection = connection;
        PluginStorage storage;
        storage_handle *storage_value = reinterpret_cast<storage_handle *>(&storage);
        slic3r_api::StoredSurfaceCollection surfaces(storage_value);
        surfaces.append(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&area)), raw_surface_type(stPosInternal | stDensSparse));
        ExtrusionEntityCollection output;
        REQUIRE(service_generate_pattern(&payload, recipe.pattern_id, nullptr, nullptr, nullptr, nullptr,
                                         surface_collection_at(surfaces.handle(), 0), nullptr, &recipe,
                                         reinterpret_cast<extrusion_entity_handle *>(&output)) == 1);
        Polylines lines;
        for (const ArcPolyline &line : output.as_polylines())
            lines.push_back(line.to_polyline());
        return lines;
    };
    const auto total_length = [](const Polylines &lines) {
        double length = 0;
        for (const Polyline &line : lines)
            length += unscaled(line.length());
        return length;
    };

    const Polylines native_connected = native_paths(InfillConnection::icConnected);
    const Polylines native_unconnected = native_paths(InfillConnection::icNotConnected);
    const Polylines plugin_connected = plugin_paths(C_INFILL_CONNECTION_CONNECTED);
    const Polylines plugin_unconnected = plugin_paths(C_INFILL_CONNECTION_NOT_CONNECTED);
    INFO("Honeycomb on a 10 x 6 mm rectangle: native connected " << native_connected.size() << " paths, "
         << total_length(native_connected) << " mm; native unconnected " << native_unconnected.size() << " paths, "
         << total_length(native_unconnected) << " mm; plugin " << plugin_connected.size() << " paths, "
         << total_length(plugin_connected) << " mm");
    // The plugin output does not depend on the connection field.
    CHECK(plugin_connected == plugin_unconnected);
    // It holds the same pieces as the native unconnected honeycomb (the native chains them in another order).
    CHECK(plugin_connected.size() == native_unconnected.size());
    CHECK(total_length(plugin_connected) == Approx(total_length(native_unconnected)).epsilon(1e-9));
    // Connecting joins pieces along the border: fewer, longer paths.
    CHECK(native_connected.size() < native_unconnected.size());
}

namespace {

// Length, volume, path count and absolute x and y extents of the extrusion paths of one role, summed over a tree.
// The extents tell the direction of the lines: a quarter turn of a fill swaps them, while its length does not change.
struct RoleTotal
{
    size_t paths = 0;
    double length = 0, volume = 0, dx = 0, dy = 0;
};
using RoleTotals = std::map<uint16_t, RoleTotal>;

// Add the leaves of a native tree to totals, keyed by the role bits of each path.
void add_native_totals(const ExtrusionEntity &entity, RoleTotals &totals)
{
    if (entity.has_polyline()) {
        const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
        REQUIRE(attributes != nullptr);
        const double length = unscaled(entity.polyline_ref().length());
        RoleTotal &total = totals[uint16_t(attributes->role)];
        ++total.paths;
        total.length += length;
        total.volume += length * attributes->mm3_per_mm;
        const Polyline line = entity.polyline_ref().to_polyline();
        for (size_t index = 1; index < line.points.size(); ++index) {
            total.dx += unscaled(std::abs(double(line.points[index].x() - line.points[index - 1].x())));
            total.dy += unscaled(std::abs(double(line.points[index].y() - line.points[index - 1].y())));
        }
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        add_native_totals(*child, totals);
}

// Add the leaves of a plugin-API tree to totals, keyed by the role bits of each path.
void add_api_totals(const slic3r_api::ExtrusionEntity &entity, RoleTotals &totals)
{
    if (entity.has_polyline()) {
        const slic3r_api::EPropertyAttributes *attributes = entity.get(slic3r_api::EPropertyAttributes::key);
        REQUIRE(attributes != nullptr);
        const std::vector<c_point> points = entity.points();
        double length = 0;
        RoleTotal &total = totals[uint16_t(attributes->extrusion_role())];
        for (size_t index = 1; index < points.size(); ++index) {
            const double dx = double(points[index].x - points[index - 1].x), dy = double(points[index].y - points[index - 1].y);
            length += std::hypot(dx, dy);
            total.dx += unscaled(std::abs(dx));
            total.dy += unscaled(std::abs(dy));
        }
        ++total.paths;
        total.length += unscaled(length);
        total.volume += unscaled(length) * attributes->mm3_per_mm();
    }
    for (uint32_t index = 0; index < entity.child_count(); ++index)
        add_api_totals(entity.child(index), totals);
}

// A sliced cube whose region carries non-default ironing settings: the native FillSmooth and the smooth
// plugin both read them from the region of the surface.
struct SmoothPrint
{
    SmoothPrint()
    {
        DynamicPrintConfig config = SG::config("grid", false, 3, false);
        config.set_deserialize_strict({{"fill_smooth_width", "40%"}, {"fill_smooth_distribution", "30%"}});
        Test::init_print({Test::mesh(Test::TestMesh::cube_20x20x20)}, print, model, config);
        Orchestrator &host = Orchestrator::instance();
        Steps::StepLayerHeightGeneration::prepare(print);
        Steps::StepLayerHeightGeneration::run_step(host, print);
        Steps::StepSlicing::run_step(host, print);
        service = std::make_unique<Steps::InfillPatternService>(host, print, 1, Steps::InfillPatternService::SetupPatterns::Refuse);
        payload.print = reinterpret_cast<const print_handle *>(&print);
        payload.object = reinterpret_cast<const object_handle *>(&print.object(0));
        payload.host_context = service.get();
        payload.generate_pattern = &service_generate_pattern;
    }
    Print print;
    Model model;
    std::unique_ptr<Steps::InfillPatternService> service;
    run_ctx_support_toolpath payload = {};
};

} // namespace

// There is no layer-level smooth variant: the legacy support generator asserts at SupportCommon.cpp:2389-2390 that each
// support entity has one non-mixed support role, and an ironed contact mixes the interface and ironing roles in one
// entity. The native FillSmooth is the filler the legacy calls for that contact, so the smooth plugin is compared with it
// on the same surfaces, recipe and region settings, and each role must print paths within one percent in length, volume
// and line direction.
// Narrow contacts, areas narrower than two line spacings, are a known difference that is not carried over: the native
// deposit pass leaves them unfilled, because the legacy never sets no-overlap areas and its gap fill (FillBase.cpp:285)
// is cut by an empty set, which is a legacy defect. The plugin prints monotonic lines there, so the shapes below are wide.
TEST_CASE("Smooth support contact fill equals the native FillSmooth", "[plugins][support-common]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    SmoothPrint print_case;
    const PrintObject &object = print_case.print.object(0);
    const LayerRegion &region = object.layer(0).region(0);
    const PrintRegion &print_region = object.printing_region(0);
    const Flow flow = support_test_flow();
    const double spacing = flow.spacing();
    const float density = 1.f;
    const float angle = float(M_PI / 12.);
    const double z = 0.2;

    // A rectangle and an L with a hole, the contact shapes of the perimeter comparisons.
    std::vector<ExPolygon> surfaces;
    surfaces.emplace_back(Polygon({Point(0, 0), Point(scale_i(10.), 0), Point(scale_i(10.), scale_i(6.)), Point(0, scale_i(6.))}));
    ExPolygon l_shape(Polygon({Point(0, 0), Point(scale_i(12.), 0), Point(scale_i(12.), scale_i(4.)), Point(scale_i(4.), scale_i(4.)),
                               Point(scale_i(4.), scale_i(12.)), Point(0, scale_i(12.))}));
    Polygon hole({Point(scale_i(1.), scale_i(1.)), Point(scale_i(1.), scale_i(2.5)), Point(scale_i(2.5), scale_i(2.5)), Point(scale_i(2.5), scale_i(1.))});
    hole.make_clockwise();
    l_shape.holes.push_back(hole);
    surfaces.push_back(l_shape);

    RoleTotals native_totals, api_totals;
    for (const ExPolygon &surface : surfaces) {
        FillParams params;
        params.density = density;
        params.dont_adjust = true;
        params.config = &print_region.config();
        params.flow = flow;
        params.role = ExtrusionRole::SupportMaterialInterface;
        std::unique_ptr<Fill> native_fill(Fill::new_from_type(ipSmooth));
        native_fill->set_bounding_box(BoundingBox(Point(-scale_i(1.), -scale_i(1.)), Point(scale_i(1.), scale_i(1.))));
        native_fill->z = z;
        native_fill->angle = angle;
        native_fill->init_spacing(spacing, params);
        const Surface native_surface(stModSupport | stPosTop | stDensSolid, surface);
        ExtrusionEntitiesPtr native_out;
        native_fill->fill_surface_extrusion(&native_surface, params, native_out);
        for (ExtrusionEntity *entity : native_out) {
            std::unique_ptr<ExtrusionEntity> owned(entity);
            add_native_totals(*owned, native_totals);
        }

        // The same surface and recipe through the smooth plugin, with the region of the layer as its primary region.
        raw_infill_pattern_params recipe = inner_recipe(print_case.service->resolve_pattern_id("smooth"), flow, spacing, density, angle, z);
        recipe.extrusion_role = RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
        recipe.surface_type = raw_surface_type(RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_POS_TOP | RAW_SURFACE_TYPE_DENS_SOLID);
        recipe.infill_flags = C_INFILL_FLAG_DONT_ADJUST | C_INFILL_FLAG_CAN_ANGLE_CROSS;
        // No plugin run owns this direct service call; a null context is never stopping.
        const Common::InnerFill inner = Common::pattern_service_fill(nullptr, print_case.payload, recipe.pattern_id, recipe, nullptr, nullptr,
                                                                     nullptr, reinterpret_cast<const layer_region_handle *>(&region));
        PluginStorage storage;
        storage_handle *storage_value = reinterpret_cast<storage_handle *>(&storage);
        const slic3r_api::ExPolygon area(reinterpret_cast<const expolygon_handle *>(&surface));
        slic3r_api::StoredSurfaceCollection surface_collection(storage_value);
        surface_collection.append(area, recipe.surface_type);
        slic3r_api::StoredExtrusionEntity out(storage_value);
        inner(surface_collection_at(surface_collection.handle(), 0), out.mutable_view());
        add_api_totals(out.readonly(), api_totals);
    }

    // Both passes of the contact print: the deposit in the interface role and the ironing pass.
    CHECK(api_totals[uint16_t(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE)].length > 0);
    CHECK(api_totals[uint16_t(RAW_EXTRUSION_ROLE_IRONING_INFILL)].length > 0);
    CHECK(api_totals.size() == native_totals.size());
    for (const RoleTotals::value_type &item : native_totals) {
        CAPTURE(item.first, item.second.length, item.second.volume);
        const RoleTotals::const_iterator found = api_totals.find(item.first);
        REQUIRE(found != api_totals.end());
        CHECK(found->second.length == Approx(item.second.length).epsilon(0.01));
        CHECK(found->second.volume == Approx(item.second.volume).epsilon(0.01));
        CHECK(found->second.dx == Approx(item.second.dx).epsilon(0.01).margin(1e-6));
        CHECK(found->second.dy == Approx(item.second.dy).epsilon(0.01).margin(1e-6));
    }
}
