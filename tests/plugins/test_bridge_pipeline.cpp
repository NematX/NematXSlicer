///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Check that slicing and G-code export preserve bridge flow corrections and
   keep the pieces of each bridge path connected.

   Pipeline cases build small box assemblies, compare source and printable bridge
   sections, and replay checkpoints. The E2E case reuses a cube and a notched frame
   loaded from STL, plus a taller sparse cube, while varying one setting or an
    explicitly coupled group. Full density uses a compatible full-fill pattern.
    Depth-budget cases use 0.1 mm layers, 20% sparse and actual mesh cavities;
    their independent volume oracle uses the nozzle's circular nominal section.

   Representative direct calls in a pipeline TEST_CASE:

       TEST_CASE
           |-- bridge_mesh(), bridge_config() [controlled model and settings]
           |-- Print::process() [run actual slicing providers]
           |-- measure_object() [inspect source extrusions]
           |   `-- measure_tree()
           |-- export_code()
           |   `-- Orchestrator::export_gcode_cli() [reference and final file exporter]
           |-- measure_plan() [inspect the plan after export's edits]
           |   `-- measure_tree()
           `-- check_export() [local emitted sections and relative-E volume]

   These sibling calls are separate test operations. Measurement never feeds
   expected flow back into the production correction; visit_paths() supplies
   inherited attributes to test callbacks and does not export or alter geometry.

   E2E TEST_CASE also calls linear_cases(), stl_fixture() and apply_case() before
   running each independent configuration through process/export. They are test
   setup helpers, not calls made by Print::process(). measure_all_bridges() then
   inspects every bridge, including compensation-disabled rows.

   Depth-budget cases call depth_budget_config(), cavity_mesh(), Print::process()
   and check_depth_budget() as separate operations. check_depth_budget() directly
   compares source sections and surface area with an analytic empty-height budget;
   it does not call measure_object() or export_code(). Export is checked separately
   with measure_all_bridges() and check_export().

   Model/Print fixtures own their trees; measurements traverse const native views.
   Coordinates/lengths are in mm, areas in mm2 and volumes in mm3. The export oracle
   assumes the controlled relative-E, filament and retraction settings supplied by
   bridge_config(). Atomic piece continuity is checked on the tree separately from
   the export's uninterrupted-move count. Activation is restored at each scope exit.
*/
#include <catch2/catch.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "gcode_test_helpers.hpp"
#include "plugin_e2e_test_helpers.hpp"
#include "support_grid_test_helpers.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/properties/BridgeProperties.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "libslic3r/Steps/StepFinalizeSlicing.hpp"

namespace {
using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
namespace GC = Slic3r::Test::GCode;

// Save all activation flags so each integration case can select a bridge feature
// and ordering provider without changing the test runtime of following cases.
class Activation {
public:
    // Save the complete activation set, then select this case's feature and orderer.
    Activation(bool enabled, const char *ordering = "ordering.extrusion_tree.advanced", bool experimental = true)
    {
        Orchestrator &host = Orchestrator::instance();
        for (Plugin *plugin : host.active_plugins())
            previous.push_back(plugin);
        for (const char *id : {"surface.bridge_preparation", "infill.recipe.bridge_preparation", "bridge.finalize_surface_volume"})
            REQUIRE(host.set_plugin_active(id, enabled));
        REQUIRE(host.set_plugin_active("bridge.finalize_volume", enabled && experimental));
        REQUIRE(host.set_plugin_active(ordering, true));
    }
    // Restore that set and cancellation state even when a test assertion throws.
    ~Activation()
    {
        Orchestrator &host = Orchestrator::instance();
        host.clear_active_plugins();
        for (Plugin *plugin : previous)
            host.set_plugin_active(plugin, true);
        host.reset_plugin_cancel();
    }
private:
    std::vector<Plugin *> previous;
};

// Bounds of one straight printable section in mm, with its area in mm2.
struct PrintedPiece {
    double ax, ay, bx, by, z, area;
};
// Aggregate path lengths/volumes and the section records used by the export oracle.
struct Measure {
    double length = 0., volume = 0., nominal = 0.;
    size_t pieces = 0, corrected = 0, unknown = 0, atomic = 0, joins = 0;
    std::vector<PrintedPiece> paths;
};
// Measure one bridge layer in mm2/mm3, independently of its stored volume budget.
struct DepthBudget {
    double area=0.,length=0.,volume=0.,nominal=0.;
    size_t pieces=0;
};
// A foreign ordering provider records lifecycle entry but deliberately publishes
// nothing. The host must refuse marked strands before entering this provider.
class OrderingProbe final : public slic3r_api::PluginBase {
public:
    // Borrow the test host for this process-held probe's lifetime.
    explicit OrderingProbe(orchestrator_handle *host) : PluginBase(host) {}
    mutable size_t setups = 0, runs = 0;
private:
    // Give the test's deliberately uncertified provider a stable lookup id.
    const char *id_impl() const noexcept override { return "test.bridge.foreign_ordering"; }
    // Enter the same dispatch step as real extrusion-tree orderers.
    slicing_step_t step_impl() const noexcept override { return STEP_ORDERING; }
    // Compete with real providers rather than running as an unrelated post-process.
    const char *exclusive_group_impl() const noexcept override { return "ordering.extrusion_tree"; }
    // Use a valid priority within the extrusion-ordering step's range.
    int32_t priority_impl() const noexcept override { return 1050; }
    // Keep the probe independent of production ordering implementations.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Record setup entry so early rejection can be distinguished from a run failure.
    void setup_run_impl(const plugin_run_context *) const override { ++setups; }
    // Record run entry without publishing or modifying the printable plan.
    void run_impl(const plugin_run_context *) const override { ++runs; }
};
// Return an owned two-pillar/roof mesh in mm with a genuine unsupported center.
TriangleMesh bridge_mesh();
// Use real production providers, precise relative E and no calibration or retraction
// so the bridge's emitted filament volume is independently measurable.
DynamicPrintConfig bridge_config(bool supports, const char *ordering = "ordering.extrusion_tree.advanced");
// Configure exactly two top/bottom layers and a 0.4 mm round bridge at 100% flow.
// Zero margins keep the measured bridge domain separate from solid anchor material.
DynamicPrintConfig depth_budget_config();
// Build a 20x10 block with a real central 10x10 cavity, in complete 0.1 mm layers.
// Side pillars keep the object's bottom on the platform even with floor_layers=0.
TriangleMesh cavity_mesh(size_t layers,size_t floor_layers,size_t gap_layers);
// Check actual source paths against the analytic empty-height budget and surface area.
// Require density-estimate stamps, positive deposition and the independent local section;
// unsupported material or unexpected roles cannot silently disappear from the measurement.
DepthBudget check_depth_budget(const PrintObject &object,size_t layer_index,bool internal,double equivalent_height);
// Visit physical leaves with their inherited attributes; missing deposition attributes fail the test.
void visit_paths(const ExtrusionEntity &root,
    const std::function<void(const ExtrusionEntity &, const ExtrusionAttributes &)> &visit,
    const ExtrusionAttributes *inherited = nullptr);
// Validate collective orientation, exact piece joins and stamped attributes;
// accumulate physical volume without mutating the tree or checking input signatures.
void measure_tree(const ExtrusionEntity &root, Measure &result, double z);
// Measure only the original object trees, not the ordered clones of the same paths.
Measure measure_object(const PrintObject &object);
// Measure the current plan; call after export to include its ordering, layer edits
// and simplification rather than an earlier printable clone.
Measure measure_plan(const Printing::PrintingPlan &plan);
// Measure every bridge leaf, including compensation-disabled cases, while
// retaining continuity checks for atomic strands that carry finalization stamps.
Measure measure_all_bridges(const Printing::PrintingPlan &plan);
// Return exported G-code as owned text and clean its temporary output pair on
// success/failure. Export may update the plan; exporter and filesystem errors propagate.
std::string export_code(Print &print);
// Remove only comments and surrounding whitespace when comparing replayed printer commands.
std::vector<std::string> normalized_commands(const std::string &code);
// Match each relative-E bridge move to a planned straight section and check total
// volume plus at least the expected uninterrupted-join count. Requires bridge_config()
// settings; exact piece-to-piece continuity is checked separately by measure_tree().
void check_export(const std::string &code, const Measure &expected);

TriangleMesh bridge_mesh()
{
    TriangleMesh mesh = make_cube(4., 6., 1.2);
    TriangleMesh other = make_cube(4., 6., 1.2);
    other.translate(8., 0., 0.);
    mesh.merge(other);
    TriangleMesh roof = make_cube(12., 6., .4);
    roof.translate(0.f, 0.f, 1.2f);
    mesh.merge(roof);
    return mesh;
}

DynamicPrintConfig bridge_config(bool supports, const char *ordering)
{
    DynamicPrintConfig config = SG::config("grid", false, 1, true);
    config.set_deserialize_strict({{"support_material", supports ? "1" : "0"}, {"bridge_angle", "0"},
        {"bridged_infill_margin", "1"}, {"bridge_flow_ratio", "150%"}, {"fill_density", "30%"},
        {"support_material_contact_distance", "0.05"}, {"support_material_interface_spacing", "0"},
        {"perimeters", "2"}, {"seam_visibility", "0"}, {"step_gcode_plugin", "gcode.printing_plan_file_writer"},
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"}, {"machine_limits_usage", "ignore"},
        {"use_relative_e_distances", "1"}, {"retract_length", "0,0"}, {"retract_restart_extra", "0,0"},
        {"wipe", "0,0"}, {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
         {"gcode_precision_e", "5"}, {"bridge_volume_compensation", "bridge.finalize_volume"}});
    const std::string selector = Steps::plugin_exclusive_group_option_key(
        Orchestrator::instance(), STEP_ORDERING, "ordering.extrusion_tree");
    REQUIRE_FALSE(selector.empty());
    config.set_deserialize_strict(selector, ordering);
    return config;
}

DynamicPrintConfig depth_budget_config()
{
    DynamicPrintConfig config=bridge_config(false);
    config.set_deserialize_strict({{"nozzle_diameter","0.4"},{"layer_height","0.1"},{"first_layer_height","0.1"},
        {"min_layer_height","0.05"},{"max_layer_height","0.3"},{"perimeters","1"},
        {"fill_density","20%"},{"fill_pattern","rectilinear"},{"top_solid_layers","2"},{"bottom_solid_layers","2"},
        {"top_solid_min_thickness","0"},{"bottom_solid_min_thickness","0"},{"solid_infill_below_area","0"},
        {"infill_dense","0"},{"bridge_type","nozzle"},{"bridge_flow_ratio","100%"},{"bridge_fill_pattern","rectilinear"},
        {"bridge_angle","0"},{"bridged_infill_margin","0"},{"bridged_infill_uniform_margin","0"},{"internal_bridge_expansion","0"},
        {"infill_overlap","0"},{"bridge_volume_compensation","bridge.finalize_surface_volume"},{"bridge_volume_compensation_enabled","1"},
        {"filament_diameter","1.75"},{"extrusion_multiplier","1"},{"retract_length","0"},{"retract_restart_extra","0"},{"wipe","0"}});
    return config;
}

TriangleMesh cavity_mesh(size_t layers,size_t floor_layers,size_t gap_layers)
{
    REQUIRE(floor_layers+gap_layers<layers);
    TriangleMesh mesh=make_cube(5.,10.,double(layers)*.1);
    TriangleMesh right=make_cube(5.,10.,double(layers)*.1);
    right.translate(15.f,0.f,0.f);mesh.merge(right);
    if (floor_layers>0) {
        TriangleMesh floor=make_cube(10.,10.,double(floor_layers)*.1);
        floor.translate(5.f,0.f,0.f);mesh.merge(floor);
    }
    const double roof_bottom=double(floor_layers+gap_layers)*.1;
    TriangleMesh roof=make_cube(10.,10.,double(layers)*.1-roof_bottom);
    roof.translate(5.f,0.f,float(roof_bottom));mesh.merge(roof);
    return mesh;
}

DepthBudget check_depth_budget(const PrintObject &object,size_t layer_index,bool internal,double equivalent_height)
{
    const Layer &layer=object.layer(layer_index);
    DepthBudget measured;
    for (const LayerSliceIsland &island : layer.islands())
        for (const LayerRegionIsland &region : island.regions_islands()) {
            for (const Surface &surface : region.fill_surfaces()) {
                const slic3r_api::BridgeSurfaceInfo *info=surface.get_property<slic3r_api::BridgeSurfaceInfo>();
                if (info==nullptr || info->core_rule!=(internal ? 2u : 1u)) continue;
                CHECK(surface.has_mod_bridge());CHECK(surface.has_pos_bottom()!=internal);
                measured.area+=surface.expolygon.area()*SCALING_FACTOR*SCALING_FACTOR;
            }
            for (const auto &[role,root] : region.extrusion_buckets())
                visit_paths(root,[&](const ExtrusionEntity &leaf,const ExtrusionAttributes &attributes) {
                    if (attributes.role!=(internal ? ExtrusionRole::InternalBridgeInfill : ExtrusionRole::BridgeInfill)) return;
                    const slic3r_api::BridgeVolumeInfo *stamp=leaf.get_property<slic3r_api::BridgeVolumeInfo>();
                    REQUIRE(stamp!=nullptr);CHECK(stamp->confidence==BRIDGE_CONFIDENCE_DENSITY_ESTIMATE);
                    // A 0.4 mm nozzle at 100% bridge flow makes a circular thread;
                    // rectilinear bridge pitch is 0.4 mm, independent of the correction.
                    const double nominal=PI*.4*.4/4.;
                    const double expected=std::min(nominal,.4*equivalent_height);
                    CHECK(stamp->nominal.width==Approx(.4).margin(1e-7));
                    CHECK(stamp->nominal.height==Approx(.4).margin(1e-7));
                    CHECK(stamp->nominal.mm3_per_mm==Approx(nominal).margin(1e-8));
                    CHECK(attributes.width==stamp->nominal.width);
                    CHECK(attributes.mm3_per_mm==Approx(expected).margin(1e-8));
                    const double length=unscaled(leaf.polyline_ref().length());
                    measured.length+=length;measured.volume+=length*attributes.mm3_per_mm;
                    measured.nominal+=length*nominal;++measured.pieces;
                });
        }
    REQUIRE(measured.area>0.);CHECK(measured.area<=100.+1e-6);REQUIRE(measured.pieces>0);
    REQUIRE(measured.volume>0.);
    INFO("area="<<measured.area<<" volume="<<measured.volume<<" nominal="<<measured.nominal<<" height budget="<<equivalent_height);
    CHECK(measured.volume<=equivalent_height*measured.area+1e-6);
    if (equivalent_height<PI*.4/4.) CHECK(measured.volume<measured.nominal-1e-6);
    else CHECK(measured.volume==Approx(measured.nominal).margin(1e-6));
    return measured;
}

void visit_paths(const ExtrusionEntity &root,
    const std::function<void(const ExtrusionEntity &, const ExtrusionAttributes &)> &visit,
    const ExtrusionAttributes *inherited)
{
    const ExtrusionAttributes *attributes = root.get_property<ExtrusionAttributes>();
    if (attributes == nullptr)
        attributes = inherited;
    if (root.has_polyline()) {
        REQUIRE(attributes != nullptr);
        visit(root, *attributes);
    }
    for (const ExtrusionEntityUPtr &child : root.children())
        if (child)
            visit_paths(*child, visit, attributes);
}

void measure_tree(const ExtrusionEntity &root, Measure &result, double z)
{
    if (const slic3r_api::BridgeAtomicStrand *strand = root.get_property<slic3r_api::BridgeAtomicStrand>()) {
        REQUIRE(strand->atomic_orientation == 1);
        CHECK_FALSE(root.can_sort());
        std::vector<const ExtrusionEntity *> leaves;
        visit_paths(root, [&](const ExtrusionEntity &leaf, const ExtrusionAttributes &) { leaves.push_back(&leaf); });
        REQUIRE(leaves.size() > 1);
        for (size_t idx = 1; idx < leaves.size(); ++idx)
            CHECK(leaves[idx - 1]->last_point() == leaves[idx]->first_point());
        result.joins += leaves.size() - 1;
        ++result.atomic;
    }
    if (const slic3r_api::BridgeVolumeInfo *stamp = root.get_property<slic3r_api::BridgeVolumeInfo>()) {
        const ExtrusionAttributes *attributes = root.get_property<ExtrusionAttributes>();
        REQUIRE(attributes != nullptr);
        REQUIRE(root.has_polyline());
        const double length = unscaled(root.polyline_ref().length());
        CHECK((attributes->role == ExtrusionRole::BridgeInfill || attributes->role == ExtrusionRole::InternalBridgeInfill));
        CHECK(attributes->mm3_per_mm == stamp->effective_area_mm2);
        CHECK(attributes->height == stamp->effective_height_mm);
        CHECK(attributes->width == stamp->nominal.width);
        CHECK(attributes->mm3_per_mm <= stamp->nominal.mm3_per_mm);
        result.length += length;
        result.volume += length * attributes->mm3_per_mm;
        result.nominal += length * stamp->nominal.mm3_per_mm;
        result.paths.push_back({unscaled(root.first_point().x()), unscaled(root.first_point().y()),
            unscaled(root.last_point().x()), unscaled(root.last_point().y()), z, attributes->mm3_per_mm});
        ++result.pieces;
        if (attributes->mm3_per_mm < stamp->nominal.mm3_per_mm - 1e-7)
            ++result.corrected;
        if (stamp->confidence == BRIDGE_CONFIDENCE_UNKNOWN)
            ++result.unknown;
    }
    for (const ExtrusionEntityUPtr &child : root.children())
        if (child)
            measure_tree(*child, result, z);
}

Measure measure_object(const PrintObject &object)
{
    Measure result;
    for (const Layer &layer : object.layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const auto &[role, root] : region.extrusion_buckets())
                    measure_tree(root, result, layer.unscaled_print_z());
    return result;
}

Measure measure_plan(const Printing::PrintingPlan &plan)
{
    Measure result;
    for (const Printing::PrintingGroup &group : plan.groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions)
                    if (extrusion.root)
                        measure_tree(*extrusion.root, result, unscaled(layer.print_z));
    return result;
}

Measure measure_all_bridges(const Printing::PrintingPlan &plan)
{
    Measure result = measure_plan(plan);
    result.length = 0.;result.volume = 0.;result.pieces = 0;result.paths.clear();
    for (const Printing::PrintingGroup &group : plan.groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions)
                    if (extrusion.root)
                        visit_paths(*extrusion.root, [&](const ExtrusionEntity &leaf, const ExtrusionAttributes &attributes) {
                            if (attributes.role != ExtrusionRole::BridgeInfill && attributes.role != ExtrusionRole::InternalBridgeInfill) return;
                            const double length = unscaled(leaf.polyline_ref().length());
                            result.length += length;result.volume += length * attributes.mm3_per_mm;++result.pieces;
                            result.paths.push_back({unscaled(leaf.first_point().x()), unscaled(leaf.first_point().y()),
                                unscaled(leaf.last_point().x()), unscaled(leaf.last_point().y()), unscaled(layer.print_z), attributes.mm3_per_mm});
                        });
    return result;
}

std::string export_code(Print &print)
{
    const boost::filesystem::path path = GC::temporary_gcode_path();
    try {
        const std::string exported = Orchestrator::instance().export_gcode_cli(print, path.string());
        const std::string code = GC::read_text_file(exported);
        GC::remove_output_pair(path);
        return code;
    } catch (...) {
        GC::remove_output_pair(path);
        throw;
    }
}

std::vector<std::string> normalized_commands(const std::string &code)
{
    std::vector<std::string> result;
    std::istringstream input(code);
    std::string line;
    while (std::getline(input, line)) {
        line = line.substr(0, line.find(';'));
        const size_t first = line.find_first_not_of(" \t\r");
        if (first != std::string::npos)
            result.push_back(line.substr(first, line.find_last_not_of(" \t\r") - first + 1));
    }
    return result;
}

void check_export(const std::string &code, const Measure &expected)
{
    double x = 0., y = 0., z = 0., emitted_e = 0.;
    size_t bridge_moves = 0, continuations = 0;
    bool previous_bridge = false, interrupted = false;
    std::string type, line;
    std::istringstream input(code);
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (line.rfind(";TYPE:", 0) == 0)
            type = line.substr(6);
        std::istringstream words(line.substr(0, line.find(';')));
        std::string word;
        words >> word;
        if (word != "G0" && word != "G1")
            continue;
        double nx = x, ny = y, nz = z, e = 0.;
        bool has_e = false;
        while (words >> word) {
            if (word.size() < 2)
                continue;
            if (word[0] == 'X') nx = std::stod(word.substr(1));
            if (word[0] == 'Y') ny = std::stod(word.substr(1));
            if (word[0] == 'Z') nz = std::stod(word.substr(1));
            if (word[0] == 'E') { e = std::stod(word.substr(1)); has_e = true; }
        }
        const bool moving = std::hypot(nx - x, ny - y) > 1e-6;
        const bool bridge = (type == "Bridge infill" || type == "Internal bridge infill") && has_e && e > 0. && moving;
        if (bridge) {
            bool matched = false;
            // Project onto a matching-Z planned section so local flow is checked,
            // not merely a total volume that could hide misplaced compensation.
            for (const PrintedPiece &piece : expected.paths) {
                if (std::abs(piece.z - nz) > .0006) continue;
                const double dx = piece.bx - piece.ax, dy = piece.by - piece.ay;
                const double length = std::hypot(dx, dy);
                const double ux = dx / length, uy = dy / length;
                const double first = (x - piece.ax) * ux + (y - piece.ay) * uy;
                const double last = (nx - piece.ax) * ux + (ny - piece.ay) * uy;
                if (first < -.001 || last > length + .001 || last < first ||
                    std::abs((x - piece.ax) * uy - (y - piece.ay) * ux) > .001 ||
                    std::abs((nx - piece.ax) * uy - (ny - piece.ay) * ux) > .001) continue;
                const double emitted = e * PI * 1.75 * 1.75 / 4.;
                CHECK(emitted == Approx(piece.area * std::hypot(nx - x, ny - y)).margin(piece.area * .0015 + .00002));
                matched = true;
                break;
            }
            REQUIRE(matched);
            emitted_e += e;
            ++bridge_moves;
            if (previous_bridge && !interrupted && nz == z)
                ++continuations;
            previous_bridge = true;
            interrupted = false;
        } else if (moving || nz != z || (has_e && e < 0.)) {
            previous_bridge = false;
            interrupted = true;
        }
        x = nx; y = ny; z = nz;
    }
    REQUIRE(bridge_moves > 0);
    CHECK(continuations >= expected.joins);
    const double filament_area = PI * 1.75 * 1.75 / 4.;
    // Relative E is rounded per move; the total tolerance grows only with that move count.
    CHECK(emitted_e * filament_area == Approx(expected.volume).margin(double(bridge_moves) * filament_area * .000006));
}
}

TEST_CASE("Internal bridge volume follows 0.18 and 0.26 area budgets before returning to nominal",
    "[plugins][bridge-pipeline][bridge-internal][bridge-surface-volume][bridge-depth-budget]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Activation active(true,"ordering.extrusion_tree.advanced",false);
    const DynamicPrintConfig config=depth_budget_config();
    size_t first_unlimited=0;
    for (size_t layers=4;layers<=8;++layers) {
        INFO("10x10 square / "<<layers<<" layers");
        Model model;Print print;
        Test::init_print({make_cube(10.,10.,double(layers)*.1)},print,model,config);print.process();
        const PrintObject &object=print.object(0);REQUIRE(object.layer_count()==layers);
        for (size_t i=0;i<layers;++i) {
            CHECK(object.layer(i).unscaled_height()==Approx(.1).margin(1e-6));
            CHECK(object.layer(i).unscaled_print_z()==Approx(double(i+1)*.1).margin(1e-6));
        }
        const size_t bridge_index=layers-2;
        if (layers==4) {
            for (const LayerSliceIsland &island : object.layer(bridge_index).islands())
                for (const LayerRegionIsland &region : island.regions_islands()) {
                    for (const Surface &surface : region.fill_surfaces()) {
                        CHECK_FALSE(surface.has_mod_bridge());CHECK(surface.get_property<slic3r_api::BridgeSurfaceInfo>()==nullptr);
                    }
                    for (const auto &[role,root] : region.extrusion_buckets())
                        visit_paths(root,[](const ExtrusionEntity &,const ExtrusionAttributes &attributes) {
                            CHECK_FALSE(attributes.extrusion_role().is_bridge());
                        });
                }
            CHECK(measure_object(object).pieces==0);
            const std::string code=export_code(print);
            REQUIRE_FALSE(normalized_commands(code).empty());
            CHECK(code.find(";TYPE:Bridge infill")==std::string::npos);
            CHECK(code.find(";TYPE:Internal bridge infill")==std::string::npos);
            continue;
        }
        const double budget=.1+double(layers-4)*.08;
        const DepthBudget measured=check_depth_budget(object,bridge_index,true,budget);
        // The second top layer is on solid, so it must not inherit a bridge role.
        for (const LayerSliceIsland &island : object.layer(layers-1).islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces()) CHECK_FALSE(surface.has_mod_bridge());
        const double source_volume=measure_object(object).volume;
        const std::string code=export_code(print);REQUIRE(print.printing_plan()!=nullptr);
        const Measure printable=measure_all_bridges(*print.printing_plan());
        CHECK(printable.volume==Approx(source_volume).margin(1e-6));
        check_export(code,printable);
        if (budget>=PI*.4/4. && first_unlimited==0) first_unlimited=layers;
        if (const char *path=std::getenv("SLIC3R_BRIDGE_BUDGET_REPORT")) {
            std::ofstream report(path,std::ios::app);REQUIRE(report.good());
            report<<"internal\t"<<layers<<'\t'<<bridge_index+1<<'\t'<<budget<<'\t'<<measured.area<<'\t'<<measured.volume<<'\t'<<measured.nominal<<'\n';
            report.close();REQUIRE(report.good());
        }
    }
    CHECK(first_unlimited==7);
}

TEST_CASE("External bridge volume stops at the platform or the cavity floor before returning to nominal",
    "[plugins][bridge-pipeline][bridge-surface-volume][bridge-depth-budget]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Activation active(true,"ordering.extrusion_tree.advanced",false);
    const DynamicPrintConfig config=depth_budget_config();
    struct Case { size_t layers,floor_layers,gap_layers; };
    for (const Case test : {Case{6,0,1},Case{6,0,2},Case{6,0,3},Case{6,1,1},Case{6,1,2},Case{6,1,3},Case{5,2,1}}) {
        INFO("20x10 block / "<<test.layers<<" layers / floor="<<test.floor_layers<<" / cavity="<<test.gap_layers);
        Model model;Print print;
        Test::init_print({cavity_mesh(test.layers,test.floor_layers,test.gap_layers)},print,model,config);print.process();
        const PrintObject &object=print.object(0);REQUIRE(object.layer_count()==test.layers);
        const size_t bridge_index=test.floor_layers+test.gap_layers;
        // This must be actual space outside the model, not an internal void/sparse surface.
        for (size_t i=0;i<test.layers;++i) {
            double area=0.;for (const LayerSliceIsland &island : object.layer(i).islands()) area+=island.get_slice().area()*SCALING_FACTOR*SCALING_FACTOR;
            const bool removed=i>=test.floor_layers && i<bridge_index;
            CHECK(area==Approx(removed ? 100. : 200.).margin(1e-5));
            CHECK(object.layer(i).unscaled_height()==Approx(.1).margin(1e-6));
        }
        const double budget=.1+double(test.gap_layers)*.1;
        const DepthBudget measured=check_depth_budget(object,bridge_index,false,budget);
        const double source_volume=measure_object(object).volume;
        const std::string code=export_code(print);REQUIRE(print.printing_plan()!=nullptr);
        const Measure printable=measure_all_bridges(*print.printing_plan());
        CHECK(printable.volume==Approx(source_volume).margin(1e-6));
        check_export(code,printable);
        if (const char *path=std::getenv("SLIC3R_BRIDGE_BUDGET_REPORT")) {
            std::ofstream report(path,std::ios::app);REQUIRE(report.good());
            report<<"external-floor-"<<test.floor_layers<<'\t'<<test.layers<<'\t'<<bridge_index+1<<'\t'<<budget<<'\t'<<measured.area<<'\t'<<measured.volume<<'\t'<<measured.nominal<<'\n';
            report.close();REQUIRE(report.good());
        }
    }
}

TEST_CASE("Default surface volume restores internal bridge role speed acceleration fan and replay", "[plugins][bridge-pipeline][bridge-internal][bridge-surface-volume]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Activation active(true,"ordering.extrusion_tree.advanced",false);
    DynamicPrintConfig config=bridge_config(false);
    config.set_deserialize_strict({{"bridge_volume_compensation","bridge.finalize_surface_volume"},{"top_solid_layers","1"},{"bottom_solid_layers","1"},
        {"top_solid_min_thickness","0"},{"bottom_solid_min_thickness","0"},{"internal_bridge_speed","31"},{"bridge_speed","19"},
        {"default_acceleration","1000"},{"internal_bridge_acceleration","567"},{"internal_bridge_fan_speed","80,80"},{"bridge_fan_speed","20,20"},
        {"slowdown_below_layer_time","0,0"},{"fan_below_layer_time","0,0"},{"full_fan_speed_layer","0,0"},
        {"disable_fan_first_layers","0,0"},{"fan_always_on","1,1"}});
    Model model;Print print;
    Test::init_print({make_cube(12.,8.,1.6)},print,model,config);print.process();
    size_t surfaces=0,paths=0;
    for (const Layer &layer : print.object(0).layers())
        for (const LayerSliceIsland &island : layer.islands()) for (const LayerRegionIsland &region : island.regions_islands()) {
            for (const Surface &surface : region.fill_surfaces())
                if (const slic3r_api::BridgeSurfaceInfo *info=surface.get_property<slic3r_api::BridgeSurfaceInfo>()) {
                    CHECK(info->core_rule==2);CHECK(surface.has_pos_top());++surfaces;
                }
            for (const auto &[role,root] : region.extrusion_buckets())
                visit_paths(root,[&](const ExtrusionEntity &leaf,const ExtrusionAttributes &attributes) {
                    if (attributes.role!=ExtrusionRole::InternalBridgeInfill) return;
                    const slic3r_api::BridgeVolumeInfo *stamp=leaf.get_property<slic3r_api::BridgeVolumeInfo>();REQUIRE(stamp!=nullptr);
                    CHECK(stamp->confidence==BRIDGE_CONFIDENCE_DENSITY_ESTIMATE);++paths;
                });
        }
    REQUIRE(surfaces>0);REQUIRE(paths>0);
    const std::string code=export_code(print);REQUIRE(print.printing_plan()!=nullptr);
    CHECK(code.find(";TYPE:Internal bridge infill")!=std::string::npos);
    CHECK((code.find("M204 S567")!=std::string::npos || code.find("M204 P567")!=std::string::npos));
    CHECK(code.find("M106 S204")!=std::string::npos);
    bool internal=false,internal_speed=false;double feed=0.;
    std::istringstream lines(code);std::string line;
    while (std::getline(lines,line)) {
        if (line.rfind(";TYPE:",0)==0) internal=line.find("Internal bridge infill")!=std::string::npos;
        if (line.rfind("G1 ",0)!=0) continue;
        std::istringstream words(line.substr(0,line.find(';')));std::string word;bool deposit=false;
        while (words>>word) {
            if (word.size()>1 && word[0]=='F') feed=std::stod(word.substr(1));
            if (word.size()>1 && word[0]=='E' && std::stod(word.substr(1))>0.) deposit=true;
        }
        if (internal && deposit && std::abs(feed-1860.)<.1) internal_speed=true;
    }
    CHECK(internal_speed);check_export(code,measure_all_bridges(*print.printing_plan()));
    const std::vector<std::string> commands=normalized_commands(code);
    print.request_replay_from_worker(Steps::Checkpoint::Print);print.process();
    CHECK(normalized_commands(export_code(print))==commands);
    print.request_replay_from_worker(Steps::Checkpoint::Slicing);print.process();
    CHECK(normalized_commands(export_code(print))==commands);
    CHECK(Orchestrator::instance().incomplete_activation_groups().count("bridge_preparation")==0);
}

TEST_CASE("Bridge volume alternatives use native exclusive selection and singleton choices", "[plugins][bridge-selector]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Activation active(true);
    Orchestrator &host=Orchestrator::instance();
    Plugin *surface=host.get_plugin("bridge.finalize_surface_volume"),*experimental=host.get_plugin("bridge.finalize_volume");
    REQUIRE(surface!=nullptr);REQUIRE(experimental!=nullptr);
    CHECK(surface->get_exclusive_group()=="bridge_volume");CHECK(experimental->get_exclusive_group()=="bridge_volume");
    CHECK(surface->get_priority()==experimental->get_priority());
    CHECK(surface->get_exclusive_group_priority()<experimental->get_exclusive_group_priority());
    CHECK(Steps::plugin_exclusive_group_option_key(host,STEP_FINALIZE_SLICING,"bridge_volume")=="bridge_volume_compensation");
    DynamicPrintConfig config=DynamicPrintConfig::full_print_config();
    DynamicConfig absent;
    CHECK(Steps::selected_active_plugin_from_group(host,STEP_FINALIZE_SLICING,"bridge_volume",&absent)==surface);
    CHECK(Steps::selected_active_plugin_from_group(host,STEP_FINALIZE_SLICING,"bridge_volume",nullptr)==surface);
    for (const char *choice : {"bridge.finalize_surface_volume","bridge.finalize_volume"}) {
        config.set_deserialize_strict("bridge_volume_compensation",choice);
        const std::vector<Plugin *> selected=Steps::selected_or_active_plugins_for_step(host,STEP_FINALIZE_SLICING,&config);
        REQUIRE(selected.size()==1);CHECK(selected.front()->get_id()==choice);
    }
    // The native option catalogue contains alternatives only while several members are active.
    for (Plugin *only : {surface,experimental}) {
        REQUIRE(host.set_plugin_active(surface,only==surface));REQUIRE(host.set_plugin_active(experimental,only==experimental));
        config.set_deserialize_strict("bridge_volume_compensation",only==surface ? experimental->get_id() : surface->get_id());
        const std::vector<Plugin *> selected=Steps::selected_or_active_plugins_for_step(host,STEP_FINALIZE_SLICING,&config);
        REQUIRE(selected.size()==1);CHECK(selected.front()==only);
        for (const Steps::StepExclusivePluginGroup &entry : Steps::active_exclusive_plugin_groups(host))
            CHECK_FALSE((entry.group.step==STEP_FINALIZE_SLICING && entry.group.group_id=="bridge_volume"));
    }
    REQUIRE(host.set_plugin_active(surface,true));REQUIRE(host.set_plugin_active(experimental,true));
    size_t groups=0;
    for (const Steps::StepExclusivePluginGroup &entry : Steps::active_exclusive_plugin_groups(host))
        if (entry.group.step==STEP_FINALIZE_SLICING && entry.group.group_id=="bridge_volume") {
            CHECK(entry.group.option_key_storage=="bridge_volume_compensation");CHECK(entry.plugins.size()==2);++groups;
        }
    CHECK(groups==1);
}

TEST_CASE("Legacy bridge model profiles migrate through INI loading", "[plugins][bridge-selector][config]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const std::pair<std::string,std::string> &model : {
        std::pair<std::string,std::string>{"surface_density_v1","bridge.finalize_surface_volume"},
        {"conservative_clearance_v1","bridge.finalize_volume"},
        {"bridge.finalize_surface_volume","bridge.finalize_surface_volume"},
        {"bridge.finalize_volume","bridge.finalize_volume"}, {"disabled",""}}) {
        INFO(model.first);
        DynamicPrintConfig loaded;
        const ConfigSubstitutions substitutions=loaded.load_from_ini_string("bridge_volume_compensation = "+model.first+"\n",
            ForwardCompatibilitySubstitutionRule::Disable);
        CHECK(substitutions.empty());
        if (model.first=="disabled") {
            REQUIRE(loaded.optptr("bridge_volume_compensation_enabled")!=nullptr);
            CHECK_FALSE(loaded.opt_bool("bridge_volume_compensation_enabled"));
            CHECK_FALSE(loaded.has("bridge_volume_compensation"));
        } else {
            REQUIRE(loaded.optptr("bridge_volume_compensation")!=nullptr);
            CHECK(loaded.optptr("bridge_volume_compensation")->serialize()==model.second);
        }
    }
}

TEST_CASE("An unavailable experimental choice resolves to the sole active surface provider", "[plugins][bridge-pipeline][bridge-surface-volume][bridge-selector]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Activation active(true,"ordering.extrusion_tree.advanced",false);
    DynamicPrintConfig legacy;
    legacy.load_from_ini_string("bridge_volume_compensation = conservative_clearance_v1\n",ForwardCompatibilitySubstitutionRule::Disable);
    DynamicPrintConfig config=bridge_config(false);config.apply(legacy);
    Model model;Print print;Test::init_print({bridge_mesh()},print,model,config);
    REQUIRE_NOTHROW(print.process());
    REQUIRE(measure_object(print.object(0)).pieces>0);
    for (const Layer &layer : print.object(0).layers()) for (const LayerSliceIsland &island : layer.islands())
        for (const LayerRegionIsland &region : island.regions_islands()) for (const auto &[role,root] : region.extrusion_buckets())
            visit_paths(root,[](const ExtrusionEntity &leaf,const ExtrusionAttributes &) {
                if (const slic3r_api::BridgeVolumeInfo *stamp=leaf.get_property<slic3r_api::BridgeVolumeInfo>())
                    CHECK((stamp->confidence==BRIDGE_CONFIDENCE_DENSITY_ESTIMATE || stamp->confidence==BRIDGE_CONFIDENCE_DENSITY_FLOOR_LIMITED));
            });
    const std::string code=export_code(print);REQUIRE(print.printing_plan()!=nullptr);
    check_export(code,measure_all_bridges(*print.printing_plan()));
}

TEST_CASE("Real bridge pipelines preserve native circular flow and export local volume", "[plugins][bridge-pipeline][bridge-volume]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const char *ordering : {"ordering.extrusion_tree.advanced", "ordering.extrusion_tree.xy_serpentine", "ordering.extrusion_tree.default"}) {
        INFO(ordering);
        Activation active(true, ordering);
        Model model;
        Print print;
        Test::init_print({bridge_mesh()}, print, model, bridge_config(false, ordering));
        print.process();
        const Measure source = measure_object(print.object(0));
        REQUIRE(source.pieces > 0);
        CHECK(source.corrected > 0);
        CHECK(source.unknown == 0);
        CHECK(source.volume < source.nominal);
        REQUIRE(source.atomic > 0);
        const std::string code = export_code(print);
        REQUIRE(print.printing_plan() != nullptr);
        const Measure final = measure_plan(*print.printing_plan());
        CHECK(final.length == Approx(source.length).margin(1e-6));
        CHECK(final.volume == Approx(source.volume).margin(1e-7));
        CHECK(final.pieces == source.pieces);
        check_export(code, final);
    }
}

TEST_CASE("Bridge compensation regenerates nominal infill on Slicing replay and survives Print replay", "[plugins][bridge-pipeline][bridge-volume][checkpoint]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Activation active(true);
    Model model;
    Print print;
    Test::init_print({bridge_mesh()}, print, model, bridge_config(false));
    size_t slicing = 0, finalizing = 0;
    print.status().set_callback([&](const PrintBase::SlicingStatus &status) {
        if ((status.flags & PrintBase::SlicingStatus::MAIN_STATE) == 0) return;
        if (status.main_text == "StepSlicing") ++slicing;
        if (status.main_text == "Finalizing slicing") ++finalizing;
    });
    print.process();
    const Measure original = measure_object(print.object(0));
    REQUIRE(original.corrected > 0);
    const std::vector<std::string> commands = normalized_commands(export_code(print));
    print.request_replay_from_worker(Steps::Checkpoint::Print);
    print.process();
    CHECK(normalized_commands(export_code(print)) == commands);
    CHECK(measure_object(print.object(0)).volume == original.volume);
    CHECK(slicing == 1);CHECK(finalizing == 1);
    DynamicPrintConfig changed = print.full_print_config();
    changed.set_deserialize_strict("bridge_volume_compensation_enabled", "0");
    CHECK(print.apply(model, changed) == Print::APPLY_STATUS_INVALIDATED);
    REQUIRE(print.current_checkpoint() == Steps::Checkpoint::Slicing);
    CHECK(print.should_execute_step(STEP_SUPPORT_DEMAND));
    CHECK(print.should_execute_step(STEP_INFILL));
    CHECK_FALSE(print.should_execute_step(STEP_SLICING));
    print.process();
    CHECK(measure_object(print.object(0)).pieces == 0);
    double nominal_volume = 0.;
    for (const Layer &layer : print.object(0).layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const auto &[role, root] : region.extrusion_buckets())
                    visit_paths(root, [&](const ExtrusionEntity &leaf, const ExtrusionAttributes &attributes) {
                        if (attributes.role == ExtrusionRole::BridgeInfill || attributes.role == ExtrusionRole::InternalBridgeInfill)
                            nominal_volume += unscaled(leaf.polyline_ref().length()) * attributes.mm3_per_mm;
                    });
    CHECK(nominal_volume == Approx(original.nominal).margin(1e-6));
    changed.set_deserialize_strict("bridge_volume_compensation_enabled", "1");
    REQUIRE(print.apply(model, changed) == Print::APPLY_STATUS_INVALIDATED);
    print.process();
    CHECK(measure_object(print.object(0)).volume == Approx(original.volume).margin(1e-7));
    CHECK(slicing == 1);CHECK(finalizing == 3);
    CHECK(normalized_commands(export_code(print)) == commands);
    changed.set_deserialize_strict("bridge_volume_compensation", "bridge.finalize_surface_volume");
    REQUIRE(print.apply(model, changed) == Print::APPLY_STATUS_INVALIDATED);
    REQUIRE(print.current_checkpoint() == Steps::Checkpoint::Slicing);
    print.process();
    REQUIRE(measure_object(print.object(0)).pieces > 0);
    changed.set_deserialize_strict("bridge_volume_compensation", "bridge.finalize_volume");
    REQUIRE(print.apply(model, changed) == Print::APPLY_STATUS_INVALIDATED);
    print.process();
    CHECK(measure_object(print.object(0)).volume == Approx(original.volume).margin(1e-7));
    CHECK(slicing == 1);CHECK(finalizing == 5);
    CHECK(normalized_commands(export_code(print)) == commands);
}

TEST_CASE("Real generated supports complete occupation beneath prepared bridges", "[plugins][bridge-pipeline][bridge-support][bridge-volume]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Activation active(true);
    Model model;
    Print print;
    Test::init_print({bridge_mesh()}, print, model, bridge_config(true));
    print.process();
    const PrintObject &object = print.object(0);
    const slic3r_api::SupportPipelineInfo *state = object.get_property<slic3r_api::SupportPipelineInfo>();
    REQUIRE(state != nullptr);
    CHECK(state->demand_computed == 1);CHECK(state->surfaces_generated == 1);CHECK(state->material_generated == 1);
    REQUIRE_FALSE(object.auxiliary_layers().empty());
    double support_volume = 0.;
    for (const Layer &layer : object.auxiliary_layers())
        for (const auto &[role, measure] : SG::measure_layer(layer))
            support_volume += measure.volume;
    REQUIRE(support_volume > 0.);
    size_t supported_core_pieces = 0;
    for (const Layer &layer : object.layers())
        for (const LayerSliceIsland &island : layer.islands()) {
            ExPolygons lower;
            for (const LayerSliceIsland::Link &linked : island.overlaps_below)
                lower.push_back(linked.to->get_slice());
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const auto &[role, root] : region.extrusion_buckets())
                    visit_paths(root, [&](const ExtrusionEntity &leaf, const ExtrusionAttributes &attributes) {
                        const slic3r_api::BridgeVolumeInfo *stamp = leaf.get_property<slic3r_api::BridgeVolumeInfo>();
                        if (stamp == nullptr || attributes.mm3_per_mm >= stamp->nominal.mm3_per_mm - 1e-7) return;
                        const Point middle = (leaf.first_point() + leaf.last_point()) / 2;
                        for (const ExPolygon &area : lower)
                            if (area.contains(middle)) return;
                        for (const Layer &support : object.auxiliary_layers())
                            if (std::abs(layer.unscaled_print_z() - support.unscaled_print_z() - stamp->available_height_mm) < 1e-6) {
                                ++supported_core_pieces;
                                break;
                            }
                    });
        }
    CHECK(supported_core_pieces > 0);
    const Measure measured = measure_object(object);
    CHECK(measured.unknown == 0);
    const std::string code = export_code(print);
    REQUIRE(print.printing_plan() != nullptr);
    check_export(code, measure_plan(*print.printing_plan()));
}

TEST_CASE("A print without bridges has identical commands with preparation enabled or disabled", "[plugins][bridge-pipeline][bridge-preparation]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    std::vector<std::string> reference;
    for (const bool enabled : {false, true}) {
        Activation active(enabled);
        Model model;
        Print print;
        Test::init_print({make_cube(4., 4., .6)}, print, model, bridge_config(false));
        print.process();
        CHECK(measure_object(print.object(0)).pieces == 0);
        const std::vector<std::string> commands = normalized_commands(export_code(print));
        REQUIRE_FALSE(commands.empty());
        if (enabled) CHECK(commands == reference); else reference = commands;
    }
}

TEST_CASE("Unmodeled post-infill flow remains nominal instead of inventing a bridge diameter", "[plugins][bridge-pipeline][bridge-volume]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Activation active(true);
    Model model;
    Print print;
    Test::init_print({bridge_mesh()}, print, model, bridge_config(false));
    SG::prefix(print);
    size_t changed = 0;
    double volume = 0.;
    // Change area alone, leaving dimensions intact. Inferring a new circular
    // diameter would overwrite the upstream producer's different physical model.
    for (Layer &layer : print.object(0).layers())
        for (LayerSliceIsland &island : layer.islands())
            for (LayerRegionIsland &region : island.regions_islands())
                for (const auto &[role, root] : region.extrusion_buckets())
                    visit_paths(root, [&](const ExtrusionEntity &leaf, const ExtrusionAttributes &attributes) {
                        if (attributes.role != ExtrusionRole::BridgeInfill && attributes.role != ExtrusionRole::InternalBridgeInfill) return;
                        ExtrusionAttributes modified = attributes;
                        modified.mm3_per_mm *= 1.1;
                        const_cast<ExtrusionEntity &>(leaf).get_or_add_property<ExtrusionAttributes>() = modified;
                        volume += unscaled(leaf.polyline_ref().length()) * modified.mm3_per_mm;
                        ++changed;
                    });
    REQUIRE(changed > 0);
    SG::family(print);
    SG::toolpaths(print);
    Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(), print);
    const Measure measured = measure_object(print.object(0));
    CHECK(measured.pieces == changed);
    CHECK(measured.unknown == changed);
    CHECK(measured.corrected == 0);
    CHECK(measured.volume == volume);
}

TEST_CASE("An older solid layer constrains the free bridge core below an empty immediate layer", "[plugins][bridge-pipeline][bridge-volume]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Activation active(true);
    TriangleMesh mesh = make_cube(12., 6., .2);
    // At roof Z=.6 the immediate lower layer has an empty core, but the base at
    // Z=.2 limits clearance to .4. Slice adjacency alone cannot find that obstacle.
    TriangleMesh left = make_cube(4., 6., .4), right = make_cube(4., 6., .4), roof = make_cube(12., 6., .4);
    right.translate(8.f, 0.f, 0.f);roof.translate(0.f, 0.f, .4f);
    mesh.merge(left);mesh.merge(right);mesh.merge(roof);
    DynamicPrintConfig config = bridge_config(false);
    config.set_deserialize_strict("fill_density", "100%");
    Model model;
    Print print;
    Test::init_print({mesh}, print, model, config);
    print.process();
    const PrintObject &object = print.object(0);
    size_t checked = 0;
    for (const Layer &layer : object.layers()) {
        if (std::abs(layer.unscaled_print_z() - .6) > 1e-6) continue;
        for (const LayerSliceIsland &island : layer.islands()) {
            ExPolygons lower;
            for (const LayerSliceIsland::Link &link : island.overlaps_below)
                lower.push_back(link.to->get_slice());
            ExPolygons cores;
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces())
                    if (surface.get_property<slic3r_api::BridgeSurfaceInfo>() != nullptr)
                        append(cores, diff_ex(ExPolygons{surface.expolygon}, lower));
            const ExPolygons interior = offset_ex(cores, -scale_i(.6));
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const auto &[role, root] : region.extrusion_buckets())
                    visit_paths(root, [&](const ExtrusionEntity &leaf, const ExtrusionAttributes &attributes) {
                        const slic3r_api::BridgeVolumeInfo *stamp = leaf.get_property<slic3r_api::BridgeVolumeInfo>();
                        if (stamp == nullptr) return;
                        const Point middle = (leaf.first_point() + leaf.last_point()) / 2;
                        bool inside = false;
                        for (const ExPolygon &area : interior) inside = inside || area.contains(middle);
                        if (!inside) return;
                        CHECK(stamp->confidence == BRIDGE_CONFIDENCE_CERTIFIED);
                        CHECK(stamp->available_height_mm == Approx(.4).margin(1e-6));
                        CHECK(attributes.height == Approx(.4).margin(1e-6));
                        const double expected = attributes.width * .4 - (1. - PI / 4.) * .4 * .4;
                        CHECK(attributes.mm3_per_mm == Approx(expected).margin(1e-7));
                        ++checked;
                    });
        }
    }
    REQUIRE(checked > 0);
}

TEST_CASE("Foreign ordering is refused before its lifecycle only for atomic bridges", "[plugins][bridge-pipeline][bridge-ordering]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    static OrderingProbe probe(reinterpret_cast<orchestrator_handle *>(&host));
    if (host.get_plugin("test.bridge.foreign_ordering") == nullptr)
        REQUIRE(host.register_plugin(probe.c_instance()));
    for (const bool bridged : {true, false}) {
        Activation active(true);
        Model model;
        Print print;
        Test::init_print({bridged ? bridge_mesh() : make_cube(4., 4., .6)}, print, model, bridge_config(false));
        print.process();
        for (const char *id : {"ordering.extrusion_tree.advanced", "ordering.extrusion_tree.xy_serpentine", "ordering.extrusion_tree.default"})
            REQUIRE(host.set_plugin_active(id, false));
        REQUIRE(host.set_plugin_active("test.bridge.foreign_ordering", true));
        probe.setups = 0;probe.runs = 0;
        // This direct dispatch has a new provider set, unlike the preceding
        // slicing prefix. Give progress the exact units it is about to observe.
        std::vector<std::string> units;
        for (const Plugin *plugin : Steps::selected_or_active_plugins_for_step(host, STEP_ORDERING, &print.full_print_config()))
            units.push_back(plugin->get_id());
        print.pipeline_progress().initialize({{STEP_ORDERING, units}});
        print.pipeline_progress().begin_step(STEP_ORDERING, "Ordering bridge test");
        if (bridged) {
            REQUIRE(measure_object(print.object(0)).atomic > 0);
            CHECK_THROWS_WITH(Steps::StepExtrusionOrdering::run_step(host, print), Catch::Contains("atomic"));
            CHECK(probe.setups == 0);CHECK(probe.runs == 0);
        } else {
            REQUIRE_NOTHROW(Steps::StepExtrusionOrdering::run_step(host, print));
            CHECK(probe.setups > 0);CHECK(probe.runs > 0);
            REQUIRE_NOTHROW(print.pipeline_progress().verify_step_complete(STEP_ORDERING));
        }
    }
}

TEST_CASE("Bridge preparation reports a fixed workload with and without bridges", "[plugins][bridge-performance]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    std::ostringstream report;
    report << "fixture\tenabled\tslicing_ms\tpaths\tsegments\tbridge_pieces\tatomic_strands\tunknown_pieces\n";
    for (const bool bridged : {false, true})
        for (const bool enabled : {false, true}) {
            Activation active(enabled);
            Model model;
            Print print;
            Test::init_print({bridged ? bridge_mesh() : make_cube(4., 4., .6)}, print, model, bridge_config(false));
            // Time only Print::process(), not the checks or report writes that follow it.
            const std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            print.process();
            const double milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
            size_t paths = 0, segments = 0;
            const PrintObject &object = print.object(0);
            for (const Layer &layer : object.layers())
                for (const LayerSliceIsland &island : layer.islands())
                    for (const LayerRegionIsland &region : island.regions_islands())
                        for (const auto &[role, root] : region.extrusion_buckets())
                            visit_paths(root, [&](const ExtrusionEntity &leaf, const ExtrusionAttributes &) {
                                ++paths;
                                segments += leaf.polyline_ref().size() - 1;
                            });
            const Measure measured = measure_object(object);
            REQUIRE(paths > 0);
            CHECK(measured.unknown == 0);
            CHECK((measured.pieces > 0) == (bridged && enabled));
            report << (bridged ? "two_pillars" : "cube") << '\t' << enabled << '\t' << milliseconds << '\t'
                << paths << '\t' << segments << '\t' << measured.pieces << '\t' << measured.atomic << '\t' << measured.unknown << '\n';
        }
    if (const char *path = std::getenv("SLIC3R_BRIDGE_PERFORMANCE_REPORT")) {
        std::ofstream output(path);
        REQUIRE(output.good());
        output << report.str();
        output.close();
        REQUIRE(output.good());
    }
}

TEST_CASE("Fresh profiles activate the complete bridge feature with compensation on", "[plugins][bridge-preparation][bridge-pipeline][bridge-selector]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    std::ifstream input(std::string(TEST_DATA_DIR) + "/../../src/plugins/default_activated.ini.in");
    REQUIRE(input.good());
    std::ostringstream contents;
    contents << input.rdbuf();
    for (const char *id : {"surface.bridge_preparation", "infill.recipe.bridge_preparation", "bridge.finalize_surface_volume", "bridge_detector.default"})
        CHECK(contents.str().find(std::string(id) + " = 1") != std::string::npos);
    CHECK(contents.str().find("bridge.finalize_volume = 0") != std::string::npos);
    const DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    REQUIRE(config.optptr("bridge_volume_compensation") != nullptr);
    CHECK(config.optptr("bridge_volume_compensation")->serialize() == "bridge.finalize_surface_volume");
    const ConfigOptionDef *definition = PrintConfigDef::instance().get("bridge_volume_compensation");
    REQUIRE(definition != nullptr);
    CHECK(definition->invalidates_step == STEP_FINALIZE_SLICING);
    CHECK(definition->container_type == ConfigOptionContainerType::Project);
    CHECK(definition->contains_plugin_id == 1);
    REQUIRE(definition->enum_def);
    CHECK(definition->enum_def->values() == std::vector<std::string>{"bridge.finalize_surface_volume", "bridge.finalize_volume"});
    CHECK(Steps::checkpoint_before_step(definition->invalidates_step) == Steps::Checkpoint::Slicing);
    REQUIRE(config.optptr("bridge_volume_compensation_enabled") != nullptr);
    CHECK(config.opt_bool("bridge_volume_compensation_enabled"));
    const ConfigOptionDef *enabled = PrintConfigDef::instance().get("bridge_volume_compensation_enabled");
    REQUIRE(enabled != nullptr);
    CHECK(enabled->container_type == ConfigOptionContainerType::Region);
    CHECK(enabled->invalidates_step == STEP_FINALIZE_SLICING);
    REQUIRE(config.optptr("bridged_infill_uniform_margin") != nullptr);
    CHECK(config.optptr("bridged_infill_uniform_margin")->serialize() == "25%");
    const ConfigOptionDef *uniform = PrintConfigDef::instance().get("bridged_infill_uniform_margin");
    const ConfigOptionDef *axial = PrintConfigDef::instance().get("bridged_infill_margin");
    REQUIRE(uniform != nullptr);REQUIRE(axial != nullptr);
    CHECK(uniform->type == coFloatOrPercent);
    CHECK(uniform->ratio_over == axial->ratio_over);
    CHECK(uniform->min == 0.);
    CHECK(uniform->invalidates_step == axial->invalidates_step);
}

TEST_CASE("Bridge and ensuring declared settings pass a linear STL slicing and export matrix", "[plugins][plugin-e2e][bridge-e2e][bridge-preparation]")
{
    namespace E2E = Slic3r::Test::PluginE2E;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Activation active(true);
    Orchestrator &host = Orchestrator::instance();
    const std::vector<std::string> ids{"surface.bridge_preparation", "infill.recipe.bridge_preparation", "bridge.finalize_surface_volume", "bridge.finalize_volume", "ensuring"};
    const DynamicPrintConfig defaults = DynamicPrintConfig::full_print_config();
    DynamicPrintConfig baseline = bridge_config(false);
    // Keep the fast fixture's printer/export setup, but use actual default
    // values for every setting declared by the tested providers.
    for (const std::string &id : ids)
        for (const Plugin::UsedConfigKey &key : host.get_plugin(id)->get_used_config_keys()) {
            REQUIRE(defaults.optptr(key.key) != nullptr);
            baseline.set_key_value(key.key, defaults.optptr(key.key)->clone());
        }
    baseline.set_deserialize_strict("bridge_volume_compensation", defaults.optptr("bridge_volume_compensation")->serialize());
    std::vector<E2E::Domain> domains;
    const auto add_values = [&](const std::string &key, const std::vector<std::string> &values) {
        E2E::Domain domain{{key}, {}};
        for (const std::string &value : values) domain.cases.push_back({key + "=" + value, {{key, value}}});
        domains.push_back(std::move(domain));
    };
    add_values("bridge_angle", {"!0", "0", "30", "90"});
    add_values("bridged_infill_margin", {"0", "0.5", "200%"});
    add_values("bridged_infill_uniform_margin", {"0", "0.5", "100%"});
    add_values("bridge_precision", {"0.01", "50%"});
    domains.push_back(E2E::enum_domain("bridge_type"));
    add_values("bridge_flow_ratio", {"50%", "100%", "150%"});
    // Grid is not a valid 100% pattern (FFFPrintConfig::validate). Couple only
    // that boundary to rectilinear; keep other density changes independent.
    domains.push_back({{"fill_density","fill_pattern"}, {
        {"fill_density=0%",{{"fill_density","0%"}}}, {"fill_density=5%",{{"fill_density","5%"}}},
        {"full density with a full-capable pattern",{{"fill_density","100%"},{"fill_pattern","rectilinear"}}}}});
    add_values("internal_bridge_min_width", {"0", "0.5", "500%"});
    add_values("internal_bridge_expansion", {"0", "1"});
    add_values("fill_angle", {"0", "30"});
    add_values("fill_angle_increment", {"0", "90"});
    add_values("fill_angle_cross", {"0", "1"});
    add_values("fill_angle_template", {"", "0,45"});
    // A solid receiver isolates zero perimeters from a lack of material below the anchor.
    domains.push_back({{"perimeters", "fill_density", "fill_pattern"}, {
        {"no perimeters on a full receiver", {{"perimeters", "0"}, {"fill_density", "100%"}, {"fill_pattern", "rectilinear"}}},
        {"one perimeter", {{"perimeters", "1"}}}, {"four perimeters", {{"perimeters", "4"}}}}});
    add_values("nozzle_diameter", {"0.25", "0.6"});
    // Selecting tool 2 requires a matching nozzle and per-tool height-limit vectors.
    domains.push_back({{"solid_infill_extruder", "nozzle_diameter", "min_layer_height", "max_layer_height"}, {
        {"bridge on the second unequal nozzle", {{"solid_infill_extruder", "2"}, {"nozzle_diameter", "0.4,0.6"},
            {"min_layer_height", "5%,5%"}, {"max_layer_height", "75%,75%"}}}}});
    domains.push_back(E2E::enum_domain("bridge_volume_compensation"));
    add_values("bridge_volume_compensation_enabled", {"0", "1"});
    add_values("min_layer_height", {"0", "0.05", "25%"});
    add_values("resolution_internal", {"0.001", "0.1"});
    domains.push_back(E2E::enum_domain("bridge_fill_pattern"));
    add_values("min_feature_size", {"0", "50%"});
    add_values("min_bead_width", {"0.05", "100%"});
    add_values("wall_transition_filter_deviation", {"0", "50%"});
    add_values("wall_transition_length", {"0.001", "200%"});
    add_values("wall_transition_angle", {"1", "59"});
    add_values("wall_distribution_count", {"2", "4"});
    const std::vector<E2E::Case> cases = E2E::linear_cases(host, ids, baseline, domains);
    CHECK(cases.size() <= 70);
    // Omitting a declared key must fail before a partial catalogue can report success.
    for (const std::string key : {"resolution_internal", "bridged_infill_uniform_margin", "internal_bridge_min_width", "fill_angle_template", "bridge_volume_compensation_enabled"}) {
        std::vector<E2E::Domain> incomplete = domains;
        incomplete.erase(std::remove_if(incomplete.begin(), incomplete.end(), [&](const E2E::Domain &domain) {
            return domain.keys == std::vector<std::string>{key};
        }), incomplete.end());
        CHECK_THROWS_WITH(E2E::linear_cases(host, ids, baseline, incomplete), Catch::Contains(key));
    }
    std::ofstream report;
    if (const char *path = std::getenv("SLIC3R_E2E_REPORT")) {
        report.open(path);
        REQUIRE(report.good());
        report << "event\tfixture\tcase\tmilliseconds\tbridge_pieces\n";
    }
    for (int fixture=0;fixture<3;++fixture) {
        TriangleMesh mesh = E2E::stl_fixture(fixture==1);
        // The taller STL cube leaves sparse between its bottom and top shells,
        // so internal settings are exercised on actual internal bridge output.
        if (fixture==2) mesh.scale(3.f);
        const std::string fixture_name=fixture==0 ? "solid cube" : fixture==1 ? "notched frame" : "sparse cube";
        for (const E2E::Case &test : cases) {
            INFO(fixture_name << " / " << test.name);
            // Flush the start record so even a native assertion identifies the active case.
            if (report.is_open()) report << "started\t" << fixture << '\t' << test.name << "\t0\t0" << std::endl;
            const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            Model model;
            Print print;
            Test::init_print({mesh}, print, model, E2E::apply_case(baseline, test));
            print.process();
            const std::string code = export_code(print);
            REQUIRE_FALSE(normalized_commands(code).empty());
            REQUIRE(print.printing_plan() != nullptr);
            const Measure measured = measure_all_bridges(*print.printing_plan());
            CHECK(measured.unknown == 0);
            if (fixture==1) {
                REQUIRE(measured.pieces > 0);
                check_export(code, measured);
            } else if (fixture==0) CHECK(measured.pieces == 0);
            else {
                if (test.name=="default") REQUIRE(measured.pieces>0);
                if (measured.pieces>0) check_export(code,measured);
            }
            if (report.is_open()) {
                const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
                report << "ended\t" << fixture << '\t' << test.name << '\t' << ms << '\t' << measured.pieces << std::endl;
                REQUIRE(report.good());
            }
        }
    }
}
