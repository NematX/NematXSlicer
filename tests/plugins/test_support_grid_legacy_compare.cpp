#include "support_grid_test_helpers.hpp"
#include "support_grid_legacy_slabs.hpp"
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Format/3mf.hpp"
#include "libslic3r/Support/SupportMaterial.hpp"

/* Comparison runs one object-slicing prefix, then the new grid family; its layers are
   detached and the native legacy generator runs on the same object. Auxiliary-layer contours, heights
   and print Z are measured before extrusion roles can affect the comparison.
    Per-Z symmetric difference measures geometry rather than polygon order.
    Original native slabs supply the set of printable thicknesses at each Z.
    Path lengths and volumes are summed by role and extruder at each Z.
    A fixed one-percent tolerance allows integer-coordinate clipping differences
    without hiding a change in density, flow or tool selection.
    Its region surface caches are reconstructed from current raw slices and
    island fill surfaces because production no longer fills those caches. Its
    concrete extrusion classes are rebuilt from generic tree leaves with the
    same geometry and effective flow attributes, preserving loop boundaries.
    Single-layer print Z groups reuse their per-layer overlap areas while retaining both sets of checks. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
using SG::PathMeasures;
using SG::measure_layer;
using SG::prepare_legacy_inputs;

namespace {
// Slice and compare native/family support for one model, configuration and optional pattern variant.
// The print owns native layers and retains detached plugin layers locally; report I/O and generation failures fail the test.
void compare(int model_id, const char *style, bool soluble, int interfaces, bool same_tool, const char *variant);
// Report both borrowed layer collections after checks, temporarily swapping them into the object and restoring ownership on failure.
void record_layer_comparison(const std::string &section, PrintObject &object, LayerUPtrs &plugin_layers,
                             const std::vector<SG::LayerSurfaceAreas> &published);
// Run one model through every style, contact-distance and interface-count combination.
void run_comparison(int model_id, bool same_tool = false);
// Check published geometry and contact spacing independently of the native comparison; on_object false
// means support may not rest on the object (build plate only).
void check_family(const PrintObject &object, const slic3r_api::Support::Common::GeneratedSupport &support,
                  const slic3r_api::Support::Common::SupportParams &params, int model_id, bool on_object = true);

// Check that every published leaf carries a positive flow whose height is its layer's thickness.
void check_published_paths(const ExtrusionEntity &entity, const Layer &layer);
// Hash published auxiliary layers including surface types, the contact bit and the partition into surfaces.
uint64_t published_hash(const PrintObject &object);
// Hashes of what the native generator reads from the object layers. geometry: per layer the print Z, the scaled height, the ids of the
// lower and upper layers (-1 when absent), the slices, the raw slice of every region, and per region island the extruder, the indices
// (in the layer) of its regions and its fill surfaces (type and polygons). extrusions: per region island and per perimeter, gap fill,
// infill, ironing, milling, support and support interface bucket, whether it exists, its leaf count and, per leaf in tree order, the
// effective role, width, height, mm3_per_mm and every point.
struct ObjectFingerprint {
    uint64_t geometry = 0, extrusions = 0;
};
// Fingerprint the object layers (not the auxiliary ones), to prove that the support steps leave them unmodified.
ObjectFingerprint object_fingerprint(const PrintObject &object);
// Count the polyline leaves of an extrusion tree.
size_t count_extrusion_leaves(const ExtrusionEntity &entity);
// Mix into hash the effective attributes (role, width, height, mm3_per_mm; the nearest ancestor's when a leaf has none, -1 when no
// attributes exist) and the points of every polyline leaf of the tree, in tree order.
void mix_extrusion_leaves(uint64_t &hash, const ExtrusionEntity &entity, const ExtrusionAttributes *attributes = nullptr);
// Append one comparison result line to the file named by SLIC3R_SUPPORT_COMPARE_TABLE, when it is set.
void record_comparison(const std::string &section, double slice, double length, double volume, uint64_t hash);
struct InterfaceShape;
struct InterfaceReport;
struct InterfaceComparison;
// Accumulate closed-ring and open-line lengths from a borrowed extrusion tree.
void add_interface_shape(const ExtrusionEntity &entity, bool in_loop, InterfaceShape &shape);
// Read the top/bottom side from a borrowed layer's typed surfaces.
const char *interface_side(const Layer &layer);
// Report borrowed layers by print Z, excluding the first layer and borrowing fallback side names.
InterfaceReport report_interface_layers(const LayerRefs &layers, const char *who, const std::map<coord_t, std::string> &known_sides = {});
// Assert matching per-Z path shapes in two borrowed reports.
void compare_interface_reports(const InterfaceReport &plugin, const InterfaceReport &legacy, const std::string &label);
// Generate and compare owned interface reports for a borrowed print and label.
std::vector<InterfaceComparison> check_interface_patterns_of(Print &print, const std::string &label);
// Compare contact patterns for one model and contact-distance setting.
void check_interface_pattern(int model_id, bool soluble);
// Compare filament-distance bridge contacts and the margin-only contact area.
void check_bridge_contact_pattern();

// Walk the owned tree and check width, height and flow per leaf; the layer stays owned by the caller.
void check_published_paths(const ExtrusionEntity &entity, const Layer &layer)
{
    if (entity.has_polyline()) {
        const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
        REQUIRE(attributes != nullptr);
        CHECK(attributes->width > 0);
        CHECK(attributes->mm3_per_mm > 0);
        CHECK(attributes->height == Approx(layer.unscaled_height()));
        return;
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) check_published_paths(*child, layer);
}

// FNV-1a over published layer geometry, surface types and the partition into surfaces, in publication order:
// equal hashes mean equal published layers including the contact bit and surface partition.
uint64_t published_hash(const PrintObject &object)
{
    uint64_t hash = 1469598103934665603ull;
    const auto mix = [&hash](int64_t value) {
        for (int byte = 0; byte < 8; ++byte) {
            hash ^= uint64_t(value >> (byte * 8)) & 0xff;
            hash *= 1099511628211ull;
        }
    };
    for (const Layer &layer : object.auxiliary_layers()) {
        mix(layer.scaled_print_z());
        mix(layer.scaled_height());
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &work : island.regions_islands()) {
                mix(work.extruder_id());
                for (const Surface &surface : work.fill_surfaces()) {
                    mix(int64_t(surface.surface_type));
                    for (const Point &point : surface.expolygon.contour.points) {
                        mix(point.x());
                        mix(point.y());
                    }
                    for (const Polygon &hole : surface.expolygon.holes)
                        for (const Point &point : hole.points) {
                            mix(point.x());
                            mix(point.y());
                        }
                }
            }
    }
    return hash;
}

// Count the polyline leaves of an extrusion tree.
size_t count_extrusion_leaves(const ExtrusionEntity &entity)
{
    size_t count = entity.has_polyline() ? 1 : 0;
    for (const ExtrusionEntityUPtr &child : entity.children()) count += count_extrusion_leaves(*child);
    return count;
}

// FNV-1a over the 8 bytes of a value, as published_hash() does.
void mix_value(uint64_t &hash, int64_t value)
{
    for (int byte = 0; byte < 8; ++byte) {
        hash ^= uint64_t(value >> (byte * 8)) & 0xff;
        hash *= 1099511628211ull;
    }
}

// Mix the bits of a floating point value, so that the smallest change of a width, height or flow changes the hash.
void mix_double(uint64_t &hash, double value)
{
    int64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    mix_value(hash, bits);
}

// Walk the tree like measure_paths(): a leaf takes the attributes of its nearest ancestor that has some, unless it has its own.
void mix_extrusion_leaves(uint64_t &hash, const ExtrusionEntity &entity, const ExtrusionAttributes *attributes)
{
    if (const ExtrusionAttributes *local = entity.get_property<ExtrusionAttributes>()) attributes = local;
    if (entity.has_polyline()) {
        if (attributes == nullptr) {
            mix_value(hash, -1);
        } else {
            mix_value(hash, int64_t(attributes->role));
            mix_double(hash, double(attributes->width));
            mix_double(hash, double(attributes->height));
            mix_double(hash, attributes->mm3_per_mm);
        }
        const ArcPolyline &polyline = entity.polyline_ref();
        mix_value(hash, int64_t(polyline.size()));
        for (size_t idx = 0; idx < polyline.size(); ++idx) {
            mix_value(hash, polyline.get_point(idx).x());
            mix_value(hash, polyline.get_point(idx).y());
        }
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) mix_extrusion_leaves(hash, *child, attributes);
}

// Walk the object layers (the auxiliary ones are not read) and hash what ObjectFingerprint describes; nothing is modified.
ObjectFingerprint object_fingerprint(const PrintObject &object)
{
    ObjectFingerprint result{1469598103934665603ull, 1469598103934665603ull};
    const auto mix = mix_value;
    const auto mix_polygon = [&](const Polygon &polygon) {
        mix(result.geometry, int64_t(polygon.points.size()));
        for (const Point &point : polygon.points) {
            mix(result.geometry, point.x());
            mix(result.geometry, point.y());
        }
    };
    const auto mix_expolygon = [&](const ExPolygon &expolygon) {
        mix_polygon(expolygon.contour);
        mix(result.geometry, int64_t(expolygon.holes.size()));
        for (const Polygon &hole : expolygon.holes) mix_polygon(hole);
    };
    const auto mix_expolygons = [&](const ExPolygons &expolygons) {
        mix(result.geometry, int64_t(expolygons.size()));
        for (const ExPolygon &expolygon : expolygons) mix_expolygon(expolygon);
    };
    for (const Layer &layer : object.layers()) {
        mix(result.geometry, layer.scaled_print_z());
        mix(result.geometry, layer.scaled_height());
        mix(result.geometry, layer.lower_layer != nullptr ? int64_t(layer.lower_layer->id()) : int64_t(-1));
        mix(result.geometry, layer.upper_layer != nullptr ? int64_t(layer.upper_layer->id()) : int64_t(-1));
        mix_expolygons(layer.lslices());
        for (const LayerRegion &region : layer.regions())
            mix_expolygons(region.get_raw_slices());
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &work : island.regions_islands()) {
                mix(result.geometry, int64_t(work.extruder_id()));
                // The regions of the island, as their indices in the layer.
                mix(result.geometry, int64_t(work.regions().size()));
                int64_t region_index = 0;
                for (const LayerRegion &region : layer.regions()) {
                    if (work.regions().count(&region) != 0)
                        mix(result.geometry, region_index);
                    ++region_index;
                }
                mix(result.geometry, int64_t(work.fill_surfaces().size()));
                for (const Surface &surface : work.fill_surfaces()) {
                    mix(result.geometry, int64_t(surface.surface_type));
                    mix_expolygon(surface.expolygon);
                }
                // The role index in this fixed list identifies the role in the hash.
                int64_t role_index = 0;
                for (const ExtrusionRole role : {LayerRegionIsland::PERIMETERS, LayerRegionIsland::GAP_FILLS, LayerRegionIsland::INFILLS,
                                                 LayerRegionIsland::IRONINGS, LayerRegionIsland::MILLS, LayerRegionIsland::SUPPORT,
                                                 LayerRegionIsland::SUPPORT_INTERFACE}) {
                    mix(result.extrusions, role_index++);
                    if (!work.has_extrusion(role)) {
                        mix(result.extrusions, -1);
                        continue;
                    }
                    mix(result.extrusions, int64_t(count_extrusion_leaves(work.extrusion(role))));
                    mix_extrusion_leaves(result.extrusions, work.extrusion(role));
                }
            }
    }
    return result;
}

// The table lets a refactoring of the family prove that every comparison gives the same deviations
// as before; nothing is written unless the variable is set.
void record_comparison(const std::string &section, double slice, double length, double volume, uint64_t hash)
{
    const char *path = std::getenv("SLIC3R_SUPPORT_COMPARE_TABLE");
    if (path == nullptr || path[0] == 0)
        return;
    std::ofstream out(path, std::ios::app);
    INFO("comparison table " << path);
    REQUIRE(out.is_open());
    out << section << " slice=" << slice << " length=" << length << " volume=" << volume << " hash=" << std::hex << hash << std::dec << "\n";
    out.flush();
    REQUIRE(out.good());
}
// Apply one comparison variant's extra settings on top of the shared configuration; an empty variant keeps the defaults.
// There is no brim variant: the legacy support still cuts the brim area out of the first layer, while the plugin ignores the brim and the skirt, so the two diverge by design.
void apply_variant(DynamicPrintConfig &config, const char *variant)
{
    if (variant == nullptr || variant[0] == 0) return;
    if (strcmp(variant, "concentric") == 0)
        config.set_deserialize_strict({{"support_material_top_interface_pattern", "concentric"},
                                       {"support_material_bottom_interface_pattern", "concentric"}});
    else if (strcmp(variant, "auto_sparse") == 0)
        // A positive interface spacing makes the automatic top pattern resolve to the sparse
        // support pattern. The bottom pattern stays explicit: its automatic value would also
        // resolve to the sparse pattern, which the shared supportbase filler rejects for the
        // bridging flow of a non-soluble bottom contact in both implementations.
        config.set_deserialize_strict({{"support_material_top_interface_pattern", "auto"},
                                       {"support_material_bottom_interface_pattern", "rectilinear"},
                                       {"support_material_interface_spacing", "2"}});
    else if (strcmp(variant, "contact_loops") == 0)
        config.set_deserialize_strict({{"support_material_interface_contact_loops", "1"}});
    else if (strcmp(variant, "honeycomb") == 0)
        config.set_deserialize_strict({{"support_material_pattern", "honeycomb"}});
    else if (strcmp(variant, "no_sheath") == 0)
        config.set_deserialize_strict({{"support_material_with_sheath", "0"}});
    else if (strcmp(variant, "rectilinear_grid") == 0)
        config.set_deserialize_strict({{"support_material_pattern", "rectilinear-grid"}});    else if (strcmp(variant, "filament") == 0)
        config.set_deserialize_strict({{"support_material_contact_distance_type", "filament"}});
    else if (strncmp(variant, "bridge_", 7) == 0)
        // The bridge height of the object decides the contact Z only with a filament contact distance.
        // A bridge flow ratio below one exercises its square root in the nozzle and flow types.
        config.set_deserialize_strict({{"support_material_contact_distance_type", "filament"},
                                       {"bridge_type", variant + 7}, {"bridge_flow_ratio", "80%"}});
    else if (strcmp(variant, "feet") == 0)
        config.set_deserialize_strict({{"raft_first_layer_expansion", "2"}});
    else if (strcmp(variant, "interface_increment") == 0)
        // A turn of 30 degrees per interface id: at 90 degrees consecutive ids give perpendicular lines and ids two apart give
        // parallel lines (180 degrees); at 30 degrees consecutive ids differ by 30 degrees and ids two apart by 60 degrees.
        config.set_deserialize_strict({{"support_material_interface_angle_increment", "30"}});
    else if (strcmp(variant, "closing") == 0)
        config.set_deserialize_strict({{"support_material_closing_radius", "1"}});
    else if (strcmp(variant, "buildplate_only") == 0)
        config.set_deserialize_strict({{"support_material_buildplate_only", "1"}});
    else
        FAIL("Unknown comparison variant " << variant);
}

// Slice one locally owned print for the supplied variant, with one shared prefix: run the plugin family and its toolpaths, check that the object
// layers are unmodified, detach the plugin support layers, then run the native generator on the same object, and compare geometry, slab heights
// and paths per Z; report failed one-percent checks and write a table only when requested.
void compare(int model_id, const char *style, bool soluble, int interfaces, bool same_tool, const char *variant)
{
    DYNAMIC_SECTION("model=" << model_id << " style=" << style << " soluble=" << soluble << " interfaces=" << interfaces
                    << " same_tool=" << same_tool << " variant=" << (variant != nullptr ? variant : "")) {
        TriangleMesh mesh;
        if (model_id == 0) mesh = SG::slab();
        else if (model_id == 1) { mesh = Test::mesh(Test::TestMesh::cube_with_hole); mesh.rotate_x(float(M_PI / 2)); }
        else if (model_id == 2) mesh = Test::mesh(Test::TestMesh::bridge);
        else if (model_id == 3) {
            mesh = make_cube(4., 8., 5.);
            Transform3d shear = Transform3d::Identity();
            shear.linear()(0, 2) = 1.;
            mesh.transform(shear);
        }
        else if (model_id == 4) mesh = SG::slab(true);
        else {
            // The roof of the slab overhangs both the bed and a low platform under its far end.
            mesh = SG::slab();
            TriangleMesh platform = make_cube(4., 8., 0.8);
            platform.translate(8.f, 0.f, 0.f);
            mesh.merge(platform);
        }        CAPTURE(model_id, style, soluble, interfaces, same_tool, variant);
        DynamicPrintConfig config = SG::config(style, soluble, interfaces, same_tool);
        // Every snug comparison starts its interfaces at 45 degrees (the increment adds to it), set before the variant so that a
        // variant keeps the last word:
        // - snug gets a diagonal starting angle next to the 90 degrees of the grid comparisons, so two interface angles are covered;
        // - no line runs parallel to the straight edges of the cube hole, where it would amplify the small demand difference
        //   of snug soluble (see the contact loops comparison comment): whole lines would enter or leave;
        // - the rectilinear-grid base of the snug comparisons uses the interface angle every second layer, so it turns with it.
        if (strcmp(style, "snug") == 0)
            config.set_deserialize_strict({{"support_material_interface_angle", "45"}});
        apply_variant(config, variant);
        if (model_id == 3) {
            // The lateral shift equals layer height. Enforcing its layers gives both implementations the full unsupported strip;
            // angle detection differs because native detect_overhangs expands its threshold strip back to the object's edge.
            config.set_deserialize_strict({{"support_material_threshold", "60"}, {"support_material_enforce_layers", "100"}});
        }
        // One print serves both implementations: the family and its toolpaths run first, their layers are then detached
        // and the native generator runs on the same object, which the support steps leave unmodified (checked below).
        Print print;
        Model model;
        Test::init_print({mesh}, print, model, config);
        SG::prefix(print);
        PrintObject &object = print.object(0);
        REQUIRE(object.auxiliary_layers().empty());
        const ObjectFingerprint prefix_fingerprint = object_fingerprint(object);
        SG::family(print);
        std::vector<SG::LayerSurfaceAreas> published;
        if (std::getenv("SLIC3R_SUPPORT_LAYER_TABLE") != nullptr)
            for (const Layer &layer : object.auxiliary_layers()) published.push_back(SG::layer_surface_areas(layer));
        const SG::Generated generated_support(print, object);
        check_family(object, generated_support.support, SG::params(print, object), model_id,
                     variant == nullptr || strcmp(variant, "buildplate_only") != 0);
        SG::toolpaths(print);
        const ObjectFingerprint support_fingerprint = object_fingerprint(object);
        REQUIRE(support_fingerprint.geometry == prefix_fingerprint.geometry);
        REQUIRE(support_fingerprint.extrusions == prefix_fingerprint.extrusions);
        const char *table_path = std::getenv("SLIC3R_SUPPORT_COMPARE_TABLE");
        const bool write_table = table_path != nullptr && table_path[0] != 0;
        // Hashed while the object still holds the layers of the family.
        const uint64_t plugin_hash = write_table ? published_hash(object) : 0;

        // The plugin layers stay alive in this vector, declared after the print and the model that outlive it, until the
        // end of the section. They reference the object only through their own pointers and destroy nothing of it.
        LayerUPtrs plugin_layers = std::move(object.mutable_auxiliary_layers());
        REQUIRE(object.auxiliary_layers().empty());
        prepare_legacy_inputs(object);
        PrintObjectSupportMaterial generator(&object, std::make_shared<SlicingParameters>(object.slicing_parameters()));
        SG::NativeSlabs native_slabs;
        SG::native_slabs(object, generator, native_slabs);
        const LayerRefs generated = make_ref_view<Layer>(plugin_layers);
        const LayerRefs legacy = object.auxiliary_layers();
        REQUIRE_FALSE(generated.empty());
        REQUIRE_FALSE(legacy.empty());
        CAPTURE(generated.size(), legacy.size());
        std::map<coord_t, ExPolygons> slices_by_z;
        std::map<coord_t, std::set<coord_t>> heights_by_z, native_heights_by_z;
        std::map<coord_t, PathMeasures> paths_by_z;
        for (const Layer &layer : generated) {
            const coord_t z = layer.scaled_print_z();
            slices_by_z[z] = union_ex(slices_by_z[z], layer.lslices());
            heights_by_z[z].insert(layer.scaled_height());
            const PathMeasures layer_paths = measure_layer(layer);
            PathMeasures &z_paths = paths_by_z[z];
            for (const auto &[key, paths] : layer_paths) {
                z_paths[key].length += paths.length;
                z_paths[key].volume += paths.volume;
                z_paths[key].count += paths.count;
            }
        }
        // The heights recorded by native_slabs() before layer installation and toolpaths changed the slabs.
        for (const std::pair<coord_t, coord_t> &slab : native_slabs.heights)
            native_heights_by_z[slab.first].insert(slab.second);
        REQUIRE(slices_by_z.size() == legacy.size());
        CHECK(heights_by_z == native_heights_by_z);
        CHECK(heights_by_z.begin()->second.size() == 1);
        CHECK(std::count_if(generated.begin(), generated.end(), [&](const Layer &layer) {
            return layer.scaled_print_z() == generated.front().scaled_print_z();
        }) == 1);
        double maximum_difference = 0, maximum_length_difference = 0, maximum_volume_difference = 0;
        // Honeycomb pieces are not connected by the plugin: its paths are recorded, not compared.
        const bool honeycomb = variant != nullptr && strcmp(variant, "honeycomb") == 0;
        size_t honeycomb_legacy_paths = 0, honeycomb_plugin_paths = 0;
        double honeycomb_legacy_length = 0, honeycomb_plugin_length = 0;
        for (size_t id = 0; id < legacy.size(); ++id) {
            CAPTURE(id);
            const coord_t z = legacy[id].scaled_print_z();
            REQUIRE(slices_by_z.count(z) == 1);
            CHECK(heights_by_z.at(z) == native_heights_by_z.at(z));
            const ExPolygons &actual_slices = slices_by_z.at(z);
            const ExPolygons difference = union_ex(diff_ex(actual_slices, legacy[id].lslices()),
                                                   diff_ex(legacy[id].lslices(), actual_slices));
            const double reference_area = area(legacy[id].lslices());
            const double ratio = reference_area > 0 ? area(difference) / reference_area : area(difference);
            maximum_difference = std::max(maximum_difference, ratio);
            INFO("symmetric difference / legacy area = " << ratio);
            CHECK(ratio < 0.01);
            /* Exception: the family publishes one auxiliary layer per (print Z, thickness), so a print Z
               can carry several of them. Each is filled at its own thickness, while the native generator
               installed a single merged layer of that Z and filled it once at the smallest thickness.
               Their path totals therefore cannot be compared at such a Z. The slice union and the
               thickness set above still have to match, and the check below proves that every published
               surface was filled, so the exception cannot hide a lost fill. */
            const size_t family_layers = size_t(std::count_if(generated.begin(), generated.end(),
                [z](const Layer &layer) { return layer.scaled_print_z() == z; }));
            if (family_layers > 1) {
                for (const Layer &layer : generated) {
                    if (layer.scaled_print_z() != z) continue;
                    // A published surface must have been filled by its role somewhere in this layer:
                    // a sparse surface by the sparse role (SUPPORT bucket), and a dense top or bottom
                    // surface by its own role, which writes the interface bucket when interface layers
                    // are enabled on that side and the support bucket when it prints it with base
                    // settings. The role writes into the region island of its own tool, which can be a
                    // sibling of the island holding the surface, so the check is per layer.
                    const bool top_interface_layers = object.config().support_material_interface_layers.value != 0;
                    const bool bottom_interface_layers = object.config().support_material_bottom_interface_layers.is_enabled() ?
                        object.config().support_material_bottom_interface_layers.value != 0 : top_interface_layers;
                    bool dense_needs_interface = false, dense_surface = false, sparse_surface = false;
                    bool dense_paths = false, sparse_paths = false, interface_paths = false;
                    for (const LayerSliceIsland &island : layer.islands())
                        for (const LayerRegionIsland &region : island.regions_islands()) {
                            for (const Surface &surface : region.fill_surfaces()) {
                                const SurfaceType type = SurfaceType(surface.surface_type & ~stModContact);
                                if (type == (stModSupport | stPosInternal | stDensSparse)) sparse_surface = true;
                                else {
                                    dense_surface = true;
                                    if ((type & stPosTop) != 0 ? top_interface_layers : bottom_interface_layers)
                                        dense_needs_interface = true;
                                }
                            }
                            dense_paths = dense_paths || region.has_extrusion(LayerRegionIsland::SUPPORT);
                            interface_paths = interface_paths || region.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE);
                            sparse_paths = sparse_paths || region.has_extrusion(LayerRegionIsland::SUPPORT);
                            for (const ExtrusionRole bucket : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE})
                                if (region.has_extrusion(bucket)) check_published_paths(region.extrusion(bucket), layer);
                        }
                    const bool dense_ok = dense_needs_interface ? interface_paths : dense_paths;
                    if (dense_surface) CHECK(dense_ok);
                    if (sparse_surface) CHECK(sparse_paths);
                }
                continue;
            }
            const PathMeasures &actual = paths_by_z.at(z);
            const PathMeasures expected = measure_layer(legacy[id]);
            CHECK(actual.size() == expected.size());
            for (const auto &[key, reference] : expected) {
                CAPTURE(key.first, key.second, reference.length, reference.volume);
                const PathMeasures::const_iterator found = actual.find(key);
                REQUIRE(found != actual.end());
                const double length_difference = reference.length > 0 ? std::abs(found->second.length - reference.length) / reference.length : found->second.length;
                const double volume_difference = reference.volume > 0 ? std::abs(found->second.volume - reference.volume) / reference.volume : found->second.volume;
                maximum_length_difference = std::max(maximum_length_difference, length_difference);
                maximum_volume_difference = std::max(maximum_volume_difference, volume_difference);
                CAPTURE(length_difference, volume_difference);
                if (!honeycomb) {
                    CHECK(length_difference < 0.01);
                    CHECK(volume_difference < 0.01);
                }
            }
            if (honeycomb) {
                for (const PathMeasures::value_type &item : expected) {
                    honeycomb_legacy_paths += item.second.count;
                    honeycomb_legacy_length += item.second.length;
                }
                for (const PathMeasures::value_type &item : actual) {
                    honeycomb_plugin_paths += item.second.count;
                    honeycomb_plugin_length += item.second.length;
                }
            }
        }
        // Report the largest measured deviation only when this section fails.
        INFO("largest deviations: slice=" << maximum_difference << " length=" << maximum_length_difference << " volume=" << maximum_volume_difference);
        if (std::getenv("SLIC3R_SUPPORT_LAYER_TABLE") != nullptr) {
            std::ostringstream section;
            section << "grid model=" << model_id << " style=" << style << " soluble=" << soluble << " interfaces=" << interfaces
                    << " same_tool=" << same_tool << " variant=" << (variant != nullptr ? variant : "") << " object=0";
            record_layer_comparison(section.str(), object, plugin_layers, published);
        }
        if (write_table) {
            std::ostringstream section;
            section << "model=" << model_id << " style=" << style << " soluble=" << soluble << " interfaces=" << interfaces
                    << " same_tool=" << same_tool << " variant=" << (variant != nullptr ? variant : "");
            if (honeycomb)
                section << " legacy_paths=" << honeycomb_legacy_paths << " plugin_paths=" << honeycomb_plugin_paths
                        << " legacy_length=" << honeycomb_legacy_length << " plugin_length=" << honeycomb_plugin_length;
            record_comparison(section.str(), maximum_difference, maximum_length_difference, maximum_volume_difference,
                              plugin_hash);
        }
    }
}

void record_layer_comparison(const std::string &section, PrintObject &object, LayerUPtrs &plugin_layers,
                             const std::vector<SG::LayerSurfaceAreas> &published)
{
    if (std::getenv("SLIC3R_SUPPORT_LAYER_TABLE") == nullptr) return;
    SG::record_layer_report(section + " native", object,
        std::vector<SG::LayerSurfaceAreas>(object.auxiliary_layers().size()), "", false);
    // Expose retained plugin layers without changing their lifetime or native layer ownership.
    struct LayerSwap {
        LayerUPtrs &current, &saved;
        // Return the two layer collections to their original owners.
        ~LayerSwap() { current.swap(saved); }
    };
    object.mutable_auxiliary_layers().swap(plugin_layers);
    const LayerSwap restore{object.mutable_auxiliary_layers(), plugin_layers};
    SG::record_layer_report(section + " plugin", object, published, "");
}

// Run one model through every style, contact-distance and interface-count combination.
void run_comparison(int model_id, bool same_tool)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("grid", "snug");
    const bool soluble = GENERATE(false, true);
    const int interfaces = GENERATE(0, 3);
    compare(model_id, style, soluble, interfaces, same_tool, "");
}

// Check borrowed slabs without changing them; contacts identify adjacent layers, model_id selects roof/platform expectations, and geometry/spacing failures are reported as checks.
void check_family(const PrintObject &object, const slic3r_api::Support::Common::GeneratedSupport &support,
                  const slic3r_api::Support::Common::SupportParams &p, int model_id, bool on_object)
{
    using namespace slic3r_api::Support::Common;
    REQUIRE_FALSE(object.auxiliary_layers().empty());
    bool top = false, bottom = false, base = false;
    std::set<size_t> ids;
    coord_t previous_z = 0;
    for (const Layer &layer : object.auxiliary_layers()) {
        CHECK(scale_to_layer_coord(layer.unscaled_print_z()) == layer.scaled_print_z());
        CHECK(layer.scaled_print_z() >= previous_z);
        previous_z = layer.scaled_print_z();
        CHECK(ids.insert(layer.id()).second);
        CAPTURE(layer.scaled_print_z(), layer.scaled_height());
        CHECK(layer.scaled_height() >= p.min_height);
        // With a filament contact distance, a bridging bottom contact is printed as a bridge thread, one
        // bottom-flow height thick, which may exceed the largest support layer height (native rule too).
        bool bridging_bottom = false;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &work : island.regions_islands())
                for (const Surface &surface : work.fill_surfaces())
                    bridging_bottom = bridging_bottom || (surface.surface_type & ~stModContact) == (stModSupport | stPosBottom | stDensSolid | stModBridge);
        const bool bridge_thread = p.distance_type == slic3r_api::Support::Common::ContactDistanceType::Filament &&
            bridging_bottom && layer.scaled_height() == scale_to_layer_coord(p.bottom_flow_mm.height);
        CHECK((layer.unscaled_height() <= p.max_support_height + EPSILON || bridge_thread));
        CHECK_FALSE(layer.has_extrusions());
        REQUIRE(layer.get_property<LayerSupportProperty>() != nullptr);
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &work : island.regions_islands())
                for (const Surface &surface : work.fill_surfaces()) {
                    const SurfaceType type = SurfaceType(surface.surface_type & ~stModContact);
                    CHECK((type == (stModSupport | stPosTop | stDensSolid) ||
                           type == (stModSupport | stPosBottom | stDensSolid) ||
                           type == (stModSupport | stPosBottom | stDensSolid | stModBridge) ||
                           type == (stModSupport | stPosInternal | stDensSparse)));
                    top = top || bool(type & stPosTop);
                    bottom = bottom || bool(type & stPosBottom);
                    base = base || bool(type & stPosInternal);
                }
    }
    bool expected_top = false, expected_bottom = false, expected_base = false;
    for (const GeneratorLayer *contact : support.top) {
        // A contact sits one gap below the object above it. With a filament contact distance, a layer with
        // bridges also gets a contact one gap below its bridges, whose height is the average bridge height of
        // the layer regions (native LayerRegion::bridging_height_avg_mm()).
        const Layer &above = object.layer(contact->object_above);
        const coord_t gap_z = above.scaled_bottom_z() - scale_to_layer_coord(p.gap_above);
        coord_t bridge_z = gap_z;
        if (p.distance_type == slic3r_api::Support::Common::ContactDistanceType::Filament) {
            double total = 0;
            for (const LayerRegion &region : above.regions()) total += region.bridging_height_avg_mm();
            bridge_z = above.scaled_print_z() - scale_to_layer_coord(total / above.region_count()) - scale_to_layer_coord(p.gap_above);
        }
        CAPTURE(contact->print_z, gap_z, bridge_z);
        CHECK((contact->print_z == gap_z || contact->print_z == bridge_z));        if (contact->polygons.empty()) continue;
        expected_top = true;
        bool published = false;
        for (const Layer &layer : object.auxiliary_layers()) {
            if (layer.scaled_print_z() != contact->print_z || layer.scaled_height() != contact->height) continue;
            Polygons contours, base_contours;
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &work : island.regions_islands())
                    for (const Surface &surface : work.fill_surfaces())
                        if ((surface.surface_type & ~stModContact) == (stModSupport | stPosTop | stDensSolid))
                            polygons_append(contours, surface.expolygon);
                        else if ((surface.surface_type & ~stModContact) == (stModSupport | stPosInternal | stDensSparse))
                            polygons_append(base_contours, surface.expolygon);
            // With no interface layer on that side the family publishes a mergeable contact as base,
            // so the contact area then lies inside the sparse surface instead of the top one.
            if (contours.empty() && object.config().support_material_interface_layers.value == 0)
                CHECK(diff_ex(SG::native(contact->polygons), base_contours).empty());
            else
                CHECK(diff_ex(SG::native(contact->polygons), contours).empty());
            published = true;
        }
        CHECK(published);
    }
    for (const GeneratorLayer *contact : support.bottom) {
        CHECK(contact->bottom_z == object.layer(contact->object_below).scaled_print_z() + scale_to_layer_coord(p.gap_below));
        if (contact->polygons.empty()) continue;
        expected_bottom = true;
        bool published = false;
        for (const Layer &layer : object.auxiliary_layers()) {
            if (layer.scaled_print_z() != contact->print_z || layer.scaled_height() != contact->height) continue;
            CHECK(layer.scaled_print_z() - layer.scaled_height() == contact->bottom_z);
            Polygons contours, base_contours;
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &work : island.regions_islands())
                    for (const Surface &surface : work.fill_surfaces())
                        if ((surface.surface_type & ~stModContact) == (stModSupport | stPosBottom | stDensSolid | (p.soluble ? 0 : stModBridge)))
                            polygons_append(contours, surface.expolygon);
                        else if ((surface.surface_type & ~stModContact) == (stModSupport | stPosInternal | stDensSparse))
                            polygons_append(base_contours, surface.expolygon);
            // Same rule as the top side: a contact typed as base lies inside the sparse surface.
            if (contours.empty() && !(object.config().support_material_bottom_interface_layers.is_enabled() ? object.config().support_material_bottom_interface_layers.value != 0 : object.config().support_material_interface_layers.value != 0))
                CHECK(diff_ex(SG::native(contact->polygons), base_contours).empty());
            else
                CHECK(diff_ex(SG::native(contact->polygons), contours).empty());
            published = true;
        }
        CHECK(published);
    }
    for (const GeneratorLayer *layer : support.interfaces) {
        expected_top = expected_top || (layer->type == LayerType::TopInterface && !layer->polygons.empty());
        expected_bottom = expected_bottom || (layer->type == LayerType::BottomInterface && !layer->polygons.empty());
    }
    for (const GeneratorLayer *layer : support.base) expected_base = expected_base || !layer->polygons.empty();
    // Horizontal roofs retain dense tops. On the 45-degree side, XY object trimming can remove each narrow contact strip entirely;
    // remaining dense interfaces, if any, still have to agree with the published surface types below.
    const bool top_interface_layers = object.config().support_material_interface_layers.value != 0;
    const bool bottom_interface_layers = object.config().support_material_bottom_interface_layers.is_enabled() ?
        object.config().support_material_bottom_interface_layers.value != 0 : top_interface_layers;
    if (model_id != 3) CHECK(expected_top);
    CHECK(expected_base);
    // Support resting on the object exists on the models with an object surface below an overhang,
    // unless support is restricted to the build plate.
    CHECK(expected_bottom == (on_object && (model_id == 1 || model_id == 4 || model_id == 5)));
    // A contact is published as top or bottom, or as base when no interface layer is configured on
    // its side and it can merge with the base.
    const bool top_ok = top == expected_top || (expected_top && !top_interface_layers && base);
    const bool bottom_ok = bottom == expected_bottom || (expected_bottom && !bottom_interface_layers && base);
    const bool base_ok = base == expected_base || (expected_base && !top_interface_layers && !bottom_interface_layers);
    CHECK(top_ok);
    CHECK(bottom_ok);
    CHECK(base_ok);
    // The three published kinds must not overlap: an area published twice would be extruded twice.
    // Check within each auxiliary layer and across the layers sharing one print Z.
    {
        std::map<std::pair<coord_t, coord_t>, std::array<Polygons, 3>> per_layer;
        std::map<coord_t, std::array<Polygons, 3>> per_z;
        std::map<coord_t, size_t> layers_per_z;
        std::map<coord_t, std::array<double, 3>> overlap_per_z;
        for (const Layer &layer : object.auxiliary_layers()) {
            ++layers_per_z[layer.scaled_print_z()];
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &work : island.regions_islands())
                    for (const Surface &surface : work.fill_surfaces()) {
                        const SurfaceType type = SurfaceType(surface.surface_type & ~stModContact);
                        const size_t kind = type == (stModSupport | stPosTop | stDensSolid) ? 0 :
                            (type == (stModSupport | stPosBottom | stDensSolid) ||
                             type == (stModSupport | stPosBottom | stDensSolid | stModBridge)) ? 1 : 2;
                        polygons_append(per_layer[{layer.scaled_print_z(), layer.scaled_height()}][kind], surface.expolygon);
                        polygons_append(per_z[layer.scaled_print_z()][kind], surface.expolygon);
                    }
        }
        for (const auto &[key, kinds] : per_layer)
            for (size_t a = 0; a < 3; ++a)
                for (size_t b = a + 1; b < 3; ++b) {
                    CAPTURE(key.first, key.second, a, b);
                    const double overlap = area(intersection_ex(kinds[a], kinds[b]));
                    // The three unordered kind pairs (0,1), (0,2), (1,2) use slots 0, 1, 2.
                    if (layers_per_z.at(key.first) == 1)
                        overlap_per_z[key.first][a + b - 1] = overlap;
                    CHECK(overlap < 1e3);
                }
        for (const auto &[z, kinds] : per_z)
            for (size_t a = 0; a < 3; ++a)
                for (size_t b = a + 1; b < 3; ++b) {
                    CAPTURE(z, a, b);
                    // With one layer at this Z the polygons are identical to those already checked above.
                    const double overlap = layers_per_z.at(z) == 1 ? overlap_per_z.at(z)[a + b - 1] :
                        area(intersection_ex(kinds[a], kinds[b]));
                    CHECK(overlap < 1e3);
                }
    }
}
}

// Compare a roof on a narrow post through eight independent native/family sections with geometry and contact assertions.
TEST_CASE("Grid legacy comparison: overhanging slab", "[plugins][support-grid][legacy-compare][slow]") { run_comparison(0); }
// Compare eight native/family combinations on a rotated cube with a hole and contacts above object surfaces.
TEST_CASE("Grid legacy comparison: cube with hole", "[plugins][support-grid][legacy-compare][slow]") { run_comparison(1); }
// Compare a bridge through eight independent native/family sections, preserving family geometry checks.
TEST_CASE("Grid legacy comparison: bridge", "[plugins][support-grid][legacy-compare][slow]") { run_comparison(2); }
// Compare the enforced demand of a sheared cube through eight sections with both soluble and separated contacts.
TEST_CASE("Grid legacy comparison: 45 degree overhang", "[plugins][support-grid][legacy-compare][slow]") { run_comparison(3); }
// Compare a roof above a low platform through eight sections, verifying family bottom contacts before native execution.
TEST_CASE("Grid legacy comparison: object above platform", "[plugins][support-grid][legacy-compare][slow]") { run_comparison(4); }
// Compare the same configurations with one support tool for both roles, exercising the native cross-role merges.
TEST_CASE("Grid legacy comparison: same support tool", "[plugins][support-grid][legacy-compare][support-fusion][slow]") { run_comparison(4, true); }
// Compare a soluble overhang with one support tool, where contacts and interfaces share their slab.
TEST_CASE("Grid legacy comparison: same support tool soluble", "[plugins][support-grid][legacy-compare][support-fusion][slow]") { run_comparison(3, true); }
// Compare the bridge model with one support tool: its shared slabs exercise the interface part
// joining the bottom contact, for a soluble and a non-soluble interface.
TEST_CASE("Grid legacy comparison: bridge with same support tool", "[plugins][support-grid][legacy-compare][support-fusion][slow]") { run_comparison(2, true); }
// Compare explicit concentric contact patterns on both interfaces, including the bridging bottom
// contact, which the native filler also prints with the configured contact pattern.
TEST_CASE("Grid legacy comparison: concentric interfaces", "[plugins][support-grid][legacy-compare][support-pattern][slow]") {
    for (const char *style : {"grid", "snug"}) compare(1, style, false, 3, false, "concentric");
}
// Compare automatic contact patterns resolving to the sparse support pattern through a positive interface spacing.
TEST_CASE("Grid legacy comparison: automatic sparse interfaces", "[plugins][support-grid][legacy-compare][support-pattern][slow]") {
    for (const char *style : {"grid", "snug"}) compare(1, style, false, 3, false, "auto_sparse");
}
// Compare top contact loops around the overhang. The snug soluble combination is left out: the
// family demand differs slightly from the native overhang detection there, and the loop length
// follows the contact contour, so that documented difference is amplified beyond one percent.
TEST_CASE("Grid legacy comparison: contact loops", "[plugins][support-grid][legacy-compare][support-pattern][slow]") {
    for (const char *style : {"grid", "snug"})
        for (const bool soluble : {false, true}) {
            if (strcmp(style, "snug") == 0 && soluble) continue;
            compare(1, style, soluble, 3, false, "contact_loops");
        }
}
// Compare the honeycomb sparse pattern and the base without sheath. The plugin leaves honeycomb pieces unconnected
// (accepted gap): honeycomb sections compare slices and heights and only record path counts and lengths. On the cube
// with a hole and three interfaces the native pattern gives 105 paths of 5364 mm against 186 paths of 4706 mm for grid,
// and 60 paths of 3995 mm against 141 paths of 3446 mm for snug.
TEST_CASE("Grid legacy comparison: honeycomb and sheath", "[plugins][support-grid][legacy-compare][support-pattern][slow]") {
    for (const char *style : {"grid", "snug"})
        for (const char *variant : {"honeycomb", "no_sheath"}) compare(1, style, false, 3, false, variant);
}
// Compare the rectilinear-grid base pattern, whose angle alternates per print Z, so the native and
// the family layer counts must agree even when one print Z carries several auxiliary layers.
TEST_CASE("Grid legacy comparison: rectilinear grid pattern", "[plugins][support-grid][legacy-compare][support-pattern][slow]") {
    for (const char *style : {"grid", "snug"}) compare(1, style, false, 3, false, "rectilinear_grid");
}
// Compare a filament contact distance, whose bridge contacts take the object's bridge height, on a
// roof and on a bridge.
TEST_CASE("Grid legacy comparison: filament contact distance", "[plugins][support-grid][legacy-compare][support-settings][slow]") {
    for (const int model_id : {1, 2})
        for (const char *style : {"grid", "snug"}) compare(model_id, style, false, 3, false, "filament");
}
// Compare the three bridge types, which give the height of the bridges above a bridging contact.
TEST_CASE("Grid legacy comparison: bridge types", "[plugins][support-grid][legacy-compare][support-settings][slow]") {
    for (const char *style : {"grid", "snug"})
        for (const char *variant : {"bridge_flow", "bridge_height", "bridge_nozzle"}) compare(2, style, false, 3, false, variant);
}
// Compare an expanded first-layer foot of the support columns.
TEST_CASE("Grid legacy comparison: foot expansion", "[plugins][support-grid][legacy-compare][support-settings][slow]") {
    for (const int model_id : {0, 1})
        for (const char *style : {"grid", "snug"}) compare(model_id, style, false, 3, false, "feet");
}
// Compare a positive interface angle increment on the cube with a hole and on the roof over bed and platform (bottom contact on the
// platform, columns from the bed with the first-layer flange, base-only layers between interface Z). Models 1, 4 and 5 all have bottom
// contacts and contacts-only layers. On model 5 only the bottom contact and the interfaces on the platform are non-square: the area
// under the roof is the 8 x 8 mm square, where lengths are nearly independent of the line orientation. The comparison covers lengths
// and volumes per Z, so it exercises the increment and the interface id numbering of the plugin and the native generator together;
// the orientation itself is checked for the plugin by the built-in support toolpath test, and not for the native generator.
TEST_CASE("Grid legacy comparison: interface angle increment", "[plugins][support-grid][legacy-compare][support-settings][slow]") {
    for (const int model_id : {1, 5})
        for (const char *style : {"grid", "snug"}) compare(model_id, style, false, 3, false, "interface_increment");
}
// Compare a snug closing radius other than the default.
TEST_CASE("Grid legacy comparison: snug closing radius", "[plugins][support-grid][legacy-compare][support-settings][slow]") {
    for (const int model_id : {0, 1}) compare(model_id, "snug", false, 3, false, "closing");
}
// Compare a roof overhanging the bed and a low platform, the model of the build-plate restriction below.
TEST_CASE("Grid legacy comparison: roof over bed and platform", "[plugins][support-grid][legacy-compare][support-settings][slow]") {
    for (const char *style : {"grid", "snug"}) compare(5, style, false, 3, false, "");
}
// The same roof with support restricted to the build plate, snug style.
TEST_CASE("Grid legacy comparison: build plate only restriction, snug", "[plugins][support-grid][legacy-compare][support-settings][slow]") {
    compare(5, "snug", false, 3, false, "buildplate_only");
}
// The same roof with support restricted to the build plate, grid style: the native and the family both cut
// their grid contacts and columns by the area the object covers below each layer, so neither puts support on the platform.
TEST_CASE("Grid legacy comparison: build plate only restriction, grid", "[plugins][support-grid][legacy-compare][support-settings][slow]") {
    compare(5, "grid", false, 3, false, "buildplate_only");
}

namespace {
// The interface paths of one layer, split by shape: a loop (an ExtrusionLoop, or a polyline that ends where it starts) is a concentric ring, any other
// path is an open line of a rectilinear or sparse pattern. Lengths are in millimetres.
struct InterfaceShape {
    double closed = 0, open = 0;
    // 'concentric' when most of the length is closed rings, 'rectilinear' when most is open lines, 'mixed' otherwise.
    const char *shape() const
    {
        const double total = closed + open;
        return closed >= 0.8 * total ? "concentric" : closed <= 0.2 * total ? "rectilinear" : "mixed";
    }
};

// Borrow the tree and recursively add each leaf's millimetre length to the caller's closed/open totals;
// loop membership or coincident endpoints selects closed, no geometry changes, and traversal failures propagate.
void add_interface_shape(const ExtrusionEntity &entity, bool in_loop, InterfaceShape &shape)
{
    if (entity.has_polyline()) {
        const ArcPolyline &line = entity.polyline_ref();
        const bool closed = in_loop || (line.size() > 2 && line.front() == line.back());
        (closed ? shape.closed : shape.open) += unscaled(line.length());
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        add_interface_shape(*child, in_loop || entity.is_loop(), shape);
}

// Borrow a layer and scan its typed surfaces, returning a static top/bottom/both/unknown label without mutation;
// surface-access failures propagate and no ownership transfers.
const char *interface_side(const Layer &layer)
{
    bool top = false, bottom = false;
    for (const LayerSliceIsland &island : layer.islands())
        for (const LayerRegionIsland &region : island.regions_islands())
            for (const Surface &surface : region.fill_surfaces()) {
                top = top || (surface.surface_type & stPosTop) != 0;
                bottom = bottom || (surface.surface_type & stPosBottom) != 0;
            }
    return top && bottom ? "both" : top ? "top" : bottom ? "bottom" : "?";
}

// The interface paths of the layers of one print Z (layers of one Z with different heights are added together): their shape, the side and the height.
struct ZInterface {
    InterfaceShape shape;
    std::string side;
    double height = 0;
};

// The report of one generator: one entry per print Z that has interface paths, in print Z order, and the text of the same lines. The first layer is
// skipped (its flange is rectilinear with a perimeter by design). A layer whose side is unknown ("?", the native layers carry no typed surfaces) takes
// the side that `known_sides` holds for its print Z, when it has one.
struct InterfaceReport {
    std::string text;
    std::map<coord_t, ZInterface> by_z;
    // Layers and layers not concentric per side, as text.
    std::string summary() const
    {
        std::map<std::string, std::pair<size_t, size_t>> per_side;
        for (const auto &[z, entry] : by_z) {
            std::pair<size_t, size_t> &counts = per_side[entry.side];
            ++counts.first;
            counts.second += std::string(entry.shape.shape()) != "concentric" ? 1 : 0;
        }
        std::ostringstream out;
        for (const auto &[side, counts] : per_side)
            out << side << ": " << counts.first << " (" << counts.second << " not concentric)  ";
        return out.str();
    }
    // Copy the side names into an owned map keyed by print Z.
    std::map<coord_t, std::string> sides_by_z() const
    {
        std::map<coord_t, std::string> sides;
        for (const auto &[z, entry] : by_z)
            sides[z] = entry.side;
        return sides;
    }
};
// Borrows layers, label and fallback side names, groups path measures by print Z and skips the first layer;
// returns an owned report without changing the inputs; allocation and view failures propagate.
InterfaceReport report_interface_layers(const LayerRefs &layers, const char *who, const std::map<coord_t, std::string> &known_sides)
{
    InterfaceReport report;
    for (const Layer &layer : layers) {
        if (layer.scaled_bottom_z() <= 0)
            continue;
        InterfaceShape shape;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                if (region.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE))
                    add_interface_shape(region.extrusion(LayerRegionIsland::SUPPORT_INTERFACE), false, shape);
        if (shape.closed + shape.open <= 0)
            continue;
        std::string side = interface_side(layer);
        if (side == "?") {
            const std::map<coord_t, std::string>::const_iterator known = known_sides.find(layer.scaled_print_z());
            if (known != known_sides.end())
                side = known->second;
        }
        ZInterface &entry = report.by_z[layer.scaled_print_z()];
        entry.shape.closed += shape.closed;
        entry.shape.open += shape.open;
        entry.side = entry.side.empty() ? side : entry.side;
        entry.height = layer.unscaled_height();
        std::ostringstream line;
        line << "  " << who << " z=" << layer.unscaled_print_z() << " height=" << layer.unscaled_height() << " side=" << side << " closed="
             << shape.closed << " open=" << shape.open << " -> " << shape.shape() << "\n";
        report.text += line.str();
    }
    return report;
}

// Print both reports, then check the plugin against the native generator print Z by print Z: the same layers have interface paths and each has the same
// shape. Inputs are borrowed and unchanged; a print Z present on one side only fails the test, and allocation/output failures propagate.
void compare_interface_reports(const InterfaceReport &plugin, const InterfaceReport &legacy, const std::string &label)
{
    UNSCOPED_INFO("interface layers of the plugin (" << plugin.summary() << "):\n" << plugin.text
                  << "interface layers of the legacy generator (" << legacy.summary() << "):\n" << legacy.text);
    std::cout << "[interface-pattern] " << label << "\nplugin " << plugin.summary() << "\n" << plugin.text << "legacy " << legacy.summary() << "\n"
              << legacy.text;
    std::set<coord_t> zs;
    for (const auto &[z, entry] : plugin.by_z)
        zs.insert(z);
    for (const auto &[z, entry] : legacy.by_z)
        zs.insert(z);
    for (const coord_t z : zs) {
        INFO(label << " print z " << unscaled(z));
        const std::map<coord_t, ZInterface>::const_iterator plugin_entry = plugin.by_z.find(z), legacy_entry = legacy.by_z.find(z);
        if (plugin_entry == plugin.by_z.end() || legacy_entry == legacy.by_z.end()) {
            FAIL_CHECK("the interface layer exists only in the " << (plugin_entry == plugin.by_z.end() ? "legacy generator" : "plugin"));
            continue;
        }
        CHECK(std::string(plugin_entry->second.shape.shape()) == std::string(legacy_entry->second.shape.shape()));
    }
}

// The interface reports of both generators for one object.
struct InterfaceComparison {
    InterfaceReport plugin, legacy;
};

// Runs the family and its toolpaths, then compares the interface shapes of the plugin against the native generator layer by layer for every object,
// and returns owned reports for each object. The print and label are borrowed; the print keeps the native auxiliary layers,
// detached plugin layers are destroyed locally, and generation/allocation failures propagate or fail the test.
std::vector<InterfaceComparison> check_interface_patterns_of(Print &print, const std::string &label)
{
    SG::family(print);
    std::vector<std::vector<SG::LayerSurfaceAreas>> published(print.objects().size());
    if (std::getenv("SLIC3R_SUPPORT_LAYER_TABLE") != nullptr)
        for (size_t index = 0; index < print.objects().size(); ++index)
            for (const Layer &layer : print.object(index).auxiliary_layers())
                published[index].push_back(SG::layer_surface_areas(layer));
    SG::toolpaths(print);
    std::vector<InterfaceComparison> comparisons;
    for (size_t object_idx = 0; object_idx < print.objects().size(); ++object_idx) {
        PrintObject &object = print.object(object_idx);
        LayerUPtrs plugin_layers = std::move(object.mutable_auxiliary_layers());
        prepare_legacy_inputs(object);
        PrintObjectSupportMaterial generator(&object, std::make_shared<SlicingParameters>(object.slicing_parameters()));
        SG::NativeSlabs native_slabs;
        SG::native_slabs(object, generator, native_slabs);
        const InterfaceReport plugin_report = report_interface_layers(make_ref_view<Layer>(plugin_layers), "plugin");
        const InterfaceReport legacy_report = report_interface_layers(object.auxiliary_layers(), "legacy", plugin_report.sides_by_z());
        compare_interface_reports(plugin_report, legacy_report, label + " object " + std::to_string(object_idx));
        REQUIRE_FALSE(plugin_report.by_z.empty());
        record_layer_comparison("grid pattern=" + label + " object=" + std::to_string(object_idx), object,
                                plugin_layers, published[object_idx]);
        comparisons.push_back(InterfaceComparison{plugin_report, legacy_report});
    }
    return comparisons;
}

// Slice one model (0: a roof over the bed; 5: the same roof partly over a low platform, so support also lands on the object) with three top and three
// bottom interface layers, both contact patterns concentric and no contact loops, and compare the plugin with the native generator.
void check_interface_pattern(int model_id, bool soluble)
{
    DYNAMIC_SECTION("model=" << model_id << " soluble=" << soluble) {
        TriangleMesh mesh;
        if (model_id == 0) {
            mesh = SG::slab();
        } else {
            // The roof of the slab overhangs both the bed and a low platform under its far end, so part of the support lands on the object.
            mesh = SG::slab();
            TriangleMesh platform = make_cube(4., 8., 0.8);
            platform.translate(8.f, 0.f, 0.f);
            mesh.merge(platform);
        }
        DynamicPrintConfig config = SG::config("grid", soluble, 3, false);
        config.set_deserialize_strict({{"support_material_top_interface_pattern", "concentric"},
                                       {"support_material_bottom_interface_pattern", "concentric"},
                                       {"support_material_interface_contact_loops", "0"}});
        Print print;
        Model model;
        Test::init_print({mesh}, print, model, config);
        SG::prefix(print);
        check_interface_patterns_of(print, "model=" + std::to_string(model_id) + " soluble=" + std::to_string(soluble));
        if (model_id == 5)
            CHECK(!print.object(0).auxiliary_layers().empty());
    }
}

// A bridge with a filament contact distance: under a layer with bridges the family places a second top contact, a bridging height under the top of the
// layer, next to the normal one. Both are contacts of the same object layer, and the toolpaths must print the layer of the bridge contact as a contact
// (concentric here) as the native generator does, whichever object layer is at the contact distance above it.
void check_bridge_contact_pattern()
{
    SECTION("bridge with a filament contact distance") {
        DynamicPrintConfig config = SG::config("grid", false, 3, false);
        config.set_deserialize_strict({{"support_material_top_interface_pattern", "concentric"},
                                       {"support_material_bottom_interface_pattern", "concentric"},
                                       {"support_material_interface_contact_loops", "0"},
                                       {"support_material_contact_distance_type", "filament"},
                                       {"bridge_type", "nozzle"}, {"bridge_flow_ratio", "80%"}});
        Print print;
        Model model;
        Test::init_print({Test::mesh(Test::TestMesh::bridge)}, print, model, config);
        SG::prefix(print);
        // The precondition: some object layer has two top contacts (the normal one and the bridge one).
        const SG::Generated generated(print, print.object(0));
        const slic3r_api::Support::Common::SupportParams geometry = SG::params(print, print.object(0));
        std::map<size_t, size_t> contacts_per_layer;
        for (const slic3r_api::Support::Common::GeneratorLayer *slab : generated.support.top)
            if (!slab->polygons.empty())
                ++contacts_per_layer[slab->object_above];
        const bool has_bridge_contact = std::any_of(contacts_per_layer.begin(), contacts_per_layer.end(),
            [](const std::pair<const size_t, size_t> &entry) { return entry.second > 1; });
        REQUIRE(has_bridge_contact);
        // The remaining contact margin lies outside the slices of the object layer above the gap
        // (bottom Z 5.0, contact print Z 4.8 plus the 0.2 gap), so intersecting those slices finds no contact;
        // the published contact bit still selects the contact pattern for the margin area.
        const slic3r_api::Support::Common::GeneratorLayer *margin_contact = nullptr;
        for (const slic3r_api::Support::Common::GeneratorLayer *slab : generated.support.top)
            if (slab->type == slic3r_api::Support::Common::LayerType::TopContact && slab->print_z == scale_i(4.8) && !slab->polygons.empty()) {
                const Layer *above = nullptr;
                for (const Layer &object_layer : print.object(0).layers())
                    if (object_layer.scaled_bottom_z() == slab->print_z + scale_i(geometry.gap_above))
                        above = &object_layer;
                REQUIRE(above != nullptr);
                CHECK(above == &print.object(0).layer(slab->object_above));
                CHECK(intersection_ex(SG::native(slab->polygons), above->lslices()).empty());
                margin_contact = slab;
            }
        REQUIRE(margin_contact != nullptr);
        const std::vector<InterfaceComparison> comparisons = check_interface_patterns_of(print, "bridge with a filament contact distance");
        REQUIRE(comparisons.size() == 1);
        for (const InterfaceReport *report : {&comparisons.front().plugin, &comparisons.front().legacy}) {
            const std::map<coord_t, ZInterface>::const_iterator layer = report->by_z.find(scale_i(4.8));
            REQUIRE(layer != report->by_z.end());
            CHECK(std::string(layer->second.shape.shape()) == "concentric");
        }
    }
}
} // namespace

// The configured interface pattern applies to the contact layers only, as in the native generator; the intermediate interface layers stay rectilinear.
// The family must give every layer the same shape as the native generator, layer by layer.
TEST_CASE("Grid legacy comparison: interface pattern follows the native contact layers", "[plugins][support-grid][legacy-compare][support-pattern]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const int model_id : {0, 5})
        for (const bool soluble : {false, true})
            check_interface_pattern(model_id, soluble);
    check_bridge_contact_pattern();
    SECTION("concentric top contacts and rectilinear bottom contacts on model 5") {
        TriangleMesh mesh = SG::slab();
        TriangleMesh platform = make_cube(4., 8., 0.8);
        platform.translate(8.f, 0.f, 0.f);
        mesh.merge(platform);
        DynamicPrintConfig config = SG::config("grid", false, 3, false);
        config.set_deserialize_strict({{"support_material_top_interface_pattern", "concentric"},
                                       {"support_material_bottom_interface_pattern", "rectilinear"},
                                       {"support_material_interface_contact_loops", "0"}});
        Print print;
        Model model;
        Test::init_print({mesh}, print, model, config);
        SG::prefix(print);
        SG::family(print);
        std::vector<SG::LayerSurfaceAreas> published;
        if (std::getenv("SLIC3R_SUPPORT_LAYER_TABLE") != nullptr)
            for (const Layer &layer : print.object(0).auxiliary_layers()) published.push_back(SG::layer_surface_areas(layer));
        SG::toolpaths(print);
        PrintObject &object = print.object(0);
        const InterfaceReport plugin = report_interface_layers(object.auxiliary_layers(), "plugin");
        std::set<coord_t> top_contacts, bottom_contacts;
        // Contact bits and position bits identify each side independently of the selected path pattern.
        for (const Layer &layer : object.auxiliary_layers()) {
            if (layer.scaled_bottom_z() <= 0) continue;
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region : island.regions_islands())
                    for (const Surface &surface : region.fill_surfaces()) {
                        if ((surface.surface_type & stModContact) == 0) continue;
                        if ((surface.surface_type & stPosTop) != 0) top_contacts.insert(layer.scaled_print_z());
                        if ((surface.surface_type & stPosBottom) != 0) bottom_contacts.insert(layer.scaled_print_z());
                    }
        }
        REQUIRE_FALSE(top_contacts.empty());
        REQUIRE_FALSE(bottom_contacts.empty());
        for (const coord_t z : top_contacts) {
            CAPTURE(z);
            REQUIRE(plugin.by_z.count(z) == 1);
            CHECK(plugin.by_z.at(z).side == "top");
            CHECK(std::string(plugin.by_z.at(z).shape.shape()) == "concentric");
        }
        for (const coord_t z : bottom_contacts) {
            CAPTURE(z);
            REQUIRE(plugin.by_z.count(z) == 1);
            CHECK(plugin.by_z.at(z).side == "bottom");
            CHECK(std::string(plugin.by_z.at(z).shape.shape()) == "rectilinear");
        }
        LayerUPtrs plugin_layers = std::move(object.mutable_auxiliary_layers());
        prepare_legacy_inputs(object);
        PrintObjectSupportMaterial generator(&object, std::make_shared<SlicingParameters>(object.slicing_parameters()));
        SG::NativeSlabs native_slabs;
        SG::native_slabs(object, generator, native_slabs);
        const InterfaceReport legacy = report_interface_layers(object.auxiliary_layers(), "legacy", plugin.sides_by_z());
        compare_interface_reports(plugin, legacy, "model 5 distinct top and bottom patterns");
        record_layer_comparison("grid pattern=model 5 distinct top and bottom patterns object=0", object, plugin_layers, published);
    }
}

// The same comparison on a project file (SLIC3R_SUPPORT_3MF names a 3mf with its configuration), so a project's own support setup is compared as a
// whole: for every print Z with interface paths, the plugin and the native generator print their shape, side and height. Run it with
//   plugins_tests "[support-pattern-file]" with SLIC3R_SUPPORT_3MF=<path>
// (the case is hidden: its tag starts with '.', so it runs only when selected). The configuration of the file is applied over the support test
// configuration, and the support toolpath providers of the family are selected as in the other comparisons.
TEST_CASE("Support interface pattern matches the native contact layers on a project file", "[.][support-pattern-file]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *path = std::getenv("SLIC3R_SUPPORT_3MF");
    if (path == nullptr || path[0] == 0) {
        WARN("SLIC3R_SUPPORT_3MF is not set: no project file to compare");
        return;
    }
    Model model;
    DynamicPrintConfig loaded_config;
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::EnableSilent};
    std::string sections;
    REQUIRE(load_3mf(path, loaded_config, substitutions, &model, false, false, &sections));
    REQUIRE(model.objects().size() > 0);
    // The support test configuration, the file over it, then the toolpath providers of the family again (the file holds the defaults).
    DynamicPrintConfig config = SG::config("grid", false, 3, false);
    config.apply(loaded_config, true);
    // 0 means "the current extruder" for the plugin family. The native generator only takes a one-based extruder (it throws for 0, and nothing in the
    // slicer calls it any more), so the comparison uses extruder 1, which is the same extruder for a file with a single extruder.
    for (const char *key : {"support_material_extruder", "support_material_interface_extruder"})
        if (config.opt_int(key) == 0)
            config.set_deserialize_strict({{key, "1"}});
    for (const std::pair<const char *, const char *> &provider : {
        std::pair<const char *, const char *>{"support_toolpath_sparse", "support.toolpath.legacy_sparse"},
        {"support_toolpath_bottom_interface", "support.toolpath.legacy_bottom"},
        {"support_toolpath_top_interface", "support.toolpath.legacy_top"}}) {
        const std::string key = Steps::plugin_exclusive_group_option_key(Orchestrator::instance(), STEP_SUPPORT_TOOLPATH, provider.first);
        if (!key.empty())
            config.set_deserialize_strict({{key, std::string(provider.second)}});
    }
    Print print;
    for (ModelObject &object : model.objects())
        print.auto_assign_extruders(&object);
    print.apply(model, config);
    const std::pair<PrintBase::PrintValidationError, std::string> validation = print.validate();
    UNSCOPED_INFO("print validation: " << validation.second);
    REQUIRE(validation.first == PrintBase::PrintValidationError::pveNone);
    print.set_status_silent();
    SG::prefix(print);
    check_interface_patterns_of(print, std::string("file ") + path);
}
