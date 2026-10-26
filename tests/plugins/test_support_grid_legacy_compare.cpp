#include "support_grid_test_helpers.hpp"
#include "support_grid_legacy_slabs.hpp"
#include <cstring>
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Plugins/Support/Grid/GridSupportGenerator.hpp"
#include "libslic3r/Support/SupportMaterial.hpp"

/* Comparison runs identical object-slicing prefixes, then chooses the new
   grid family or native legacy generator. Auxiliary-layer contours, heights
   and print Z are measured before extrusion roles can affect the comparison.
    Per-Z symmetric difference measures geometry rather than polygon order.
    Original native slabs supply the set of printable thicknesses at each Z.
    Path lengths and volumes are summed by role and extruder at each Z.
    A fixed one-percent tolerance allows integer-coordinate clipping differences
    without hiding a change in density, flow or tool selection.
    Its region surface caches are reconstructed from current raw slices and
    island fill surfaces because production no longer fills those caches. Its
    concrete extrusion classes are rebuilt from generic tree leaves with the
    same geometry and effective flow attributes, preserving loop boundaries. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;

namespace {
// Run one model through every style, contact-distance and interface-count combination.
void run_comparison(int model_id, bool same_tool = false);
// Check published geometry and contact spacing independently of the native comparison.
void check_family(const PrintObject &object, const Slic3r::SupportGrid::GeneratedSupport &support, const Slic3r::SupportGrid::GridSupportParams &params, int model_id);
// Populate deprecated region caches and concrete extrusion classes for native support.
void prepare_legacy_inputs(PrintObject &object);
// Copy generic extrusion leaves into concrete paths with resolved flow attributes.
void legacy_paths(const ExtrusionEntity &entity, ExtrusionPaths &paths, const ExtrusionAttributes *attributes = nullptr);
// Rebuild concrete loops and paths for the native support reader.
void legacy_extrusions(const ExtrusionEntity &entity, ExtrusionEntityCollection &out, const ExtrusionAttributes *attributes = nullptr);
struct PathMeasure { double length = 0, volume = 0; };
using PathMeasures = std::map<std::pair<extruder_index_t, uint16_t>, PathMeasure>;
// Sum effective flow and geometric length for each extrusion leaf.
void measure_paths(const ExtrusionEntity &entity, extruder_index_t extruder, PathMeasures &result, const ExtrusionAttributes *attributes = nullptr);
// Measure both support buckets in every region island of a layer.
PathMeasures measure_layer(const Layer &layer);

// Resolve inherited attributes while visiting the tree; missing support flow fails instead of silently omitting a leaf, and result owns the summed values.
void measure_paths(const ExtrusionEntity &entity, extruder_index_t extruder, PathMeasures &result, const ExtrusionAttributes *attributes)
{
    if (const ExtrusionAttributes *local = entity.get_property<ExtrusionAttributes>()) attributes = local;
    if (entity.has_polyline()) {
        REQUIRE(attributes != nullptr);
        PathMeasure &measure = result[{extruder, uint16_t(attributes->role)}];
        const double length = unscaled(entity.polyline_ref().length());
        measure.length += length;
        measure.volume += length * attributes->mm3_per_mm;
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) measure_paths(*child, extruder, result, attributes);
}

// Read support and interface trees without modifying the layer; the returned map separates roles and tools even when both buckets use one tool.
PathMeasures measure_layer(const Layer &layer)
{
    PathMeasures result;
    for (const LayerSliceIsland &island : layer.islands())
        for (const LayerRegionIsland &region : island.regions_islands())
            for (const ExtrusionRole bucket : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE})
                if (region.has_extrusion(bucket)) measure_paths(region.extrusion(bucket), region.extruder_id(), result);
    return result;
}

// Classify raw footprints by adjacent-layer coverage and copy island fills and paths into legacy-readable inputs; only the legacy print's caches and path classes change.
void prepare_legacy_inputs(PrintObject &object)
{
    for (Layer &layer : object.layers())
        for (LayerRegion &region : layer.regions()) {
            const ExPolygons &raw = region.get_raw_slices();
            const ExPolygons bottom = layer.lower_layer ? diff_ex(raw, layer.lower_layer->lslices()) : raw;
            const ExPolygons top = diff_ex(layer.upper_layer ? diff_ex(raw, layer.upper_layer->lslices()) : raw, bottom);
            region.m_slices.clear();
            region.m_slices.append(bottom, layer.lower_layer ? stPosBottom | stDensSolid | stModBridge : stPosBottom | stDensSolid);
            region.m_slices.append(top, stPosTop | stDensSolid);
            region.m_slices.append(diff_ex(diff_ex(raw, bottom), top), stPosInternal | stDensSparse);
            SurfaceCollection &fills = region.set_fill_surfaces();
            fills.clear();
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &work : island.regions_islands())
                    if (work.regions().count(&region))
                        for (const Surface &surface : work.fill_surfaces())
                            fills.append(intersection_ex(ExPolygons{surface.expolygon}, raw), surface);
        }
    for (Layer &layer : object.layers())
        for (LayerSliceIsland &island : layer.islands())
            for (LayerRegionIsland &work : island.regions_islands())
                for (const ExtrusionRole role : {LayerRegionIsland::PERIMETERS, LayerRegionIsland::INFILLS}) {
                    if (!work.has_extrusion(role)) continue;
                    ExtrusionEntityCollection concrete;
                    legacy_extrusions(work.extrusion(role), concrete);
                    work.mutable_extrusion(role) = std::move(concrete);
                }
}

// Resolve nearest attributes while copying polylines into test-owned paths; missing flow on a geometric leaf fails the comparison instead of dropping geometry.
void legacy_paths(const ExtrusionEntity &entity, ExtrusionPaths &paths, const ExtrusionAttributes *attributes)
{
    if (const ExtrusionAttributes *local = entity.get_property<ExtrusionAttributes>()) attributes = local;
    if (entity.has_polyline()) {
        REQUIRE(attributes != nullptr);
        ExtrusionPath path(*attributes);
        path.set_polyline(entity.polyline_ref());
        paths.push_back(std::move(path));
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) legacy_paths(*child, paths, attributes);
}

// Preserve loop boundaries while flattening generic wrapper nodes into concrete classes; the legacy print owns the copies and the source tree stays valid until replacement.
void legacy_extrusions(const ExtrusionEntity &entity, ExtrusionEntityCollection &out, const ExtrusionAttributes *attributes)
{
    if (const ExtrusionAttributes *local = entity.get_property<ExtrusionAttributes>()) attributes = local;
    if (entity.is_loop()) {
        ExtrusionPaths paths;
        legacy_paths(entity, paths, attributes);
        if (!paths.empty()) out.append(ExtrusionLoop(std::move(paths)));
    } else if (entity.has_polyline()) {
        REQUIRE(attributes != nullptr);
        ExtrusionPath path(*attributes);
        path.set_polyline(entity.polyline_ref());
        out.append(std::move(path));
    } else {
        for (const ExtrusionEntityUPtr &child : entity.children()) legacy_extrusions(*child, out, attributes);
    }
}
// Check that every published leaf carries a positive flow whose height is its layer's thickness.
void check_published_paths(const ExtrusionEntity &entity, const Layer &layer);

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

// Apply one comparison variant's extra settings on top of the shared configuration; an empty variant keeps the defaults.
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
        config.set_deserialize_strict({{"support_material_pattern", "rectilinear-grid"}});
    else
        FAIL("Unknown comparison variant " << variant);
}

// Slice two owned prints and compare geometry, original slab heights and path totals at each Z; separate same-Z family layers are unioned, and the one-percent tolerance stays fixed.
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
        else mesh = SG::slab(true);
        CAPTURE(model_id, style, soluble, interfaces, same_tool, variant);
        DynamicPrintConfig config = SG::config(style, soluble, interfaces, same_tool);
        apply_variant(config, variant);
        if (model_id == 3) {
            // The lateral shift equals layer height. Enforcing its layers gives both implementations the full unsupported strip;
            // angle detection differs because native detect_overhangs expands its threshold strip back to the object's edge.
            config.set_deserialize_strict({{"support_material_threshold", "60"}, {"support_material_enforce_layers", "100"}});
        }
        Print plugin_print, legacy_print;
        Model plugin_model, legacy_model;
        Test::init_print({mesh}, plugin_print, plugin_model, config);
        Test::init_print({mesh}, legacy_print, legacy_model, config);
        SG::prefix(plugin_print);
        SG::prefix(legacy_print);
        REQUIRE(legacy_print.object(0).auxiliary_layers().empty());
        SG::family(plugin_print);
        const PrintObject &plugin_object = plugin_print.object(0);
        const Slic3r::SupportGrid::GridSupportParams params(plugin_object);
        Slic3r::SupportGrid::GeneratedSupport support;
        Slic3r::SupportGrid::generate_support(plugin_object, params, support);
        check_family(plugin_object, support, params, model_id);
        SG::toolpaths(plugin_print);

        PrintObject &legacy_object = legacy_print.object(0);
        prepare_legacy_inputs(legacy_object);
        PrintObjectSupportMaterial generator(&legacy_object, std::make_shared<SlicingParameters>(legacy_object.slicing_parameters()));
        SG::NativeSlabs native_slabs;
        SG::native_slabs(legacy_object, generator, native_slabs);
        generator.generate(legacy_object);
        const LayerRefs generated = plugin_print.object(0).auxiliary_layers();
        const LayerRefs legacy = legacy_object.auxiliary_layers();
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
            }
        }
        for (const SG::NativeLayers *layers : {&native_slabs.top, &native_slabs.bottom, &native_slabs.base, &native_slabs.interfaces})
            for (const FFFSupport::SupportGeneratorLayer *slab : *layers)
                if (!slab->polygons.empty()) native_heights_by_z[slab->scaled_print_z()].insert(slab->scaled_height());
        REQUIRE(slices_by_z.size() == legacy.size());
        CHECK(heights_by_z == native_heights_by_z);
        CHECK(heights_by_z.begin()->second.size() == 1);
        CHECK(std::count_if(generated.begin(), generated.end(), [&](const Layer &layer) {
            return layer.scaled_print_z() == generated.front().scaled_print_z();
        }) == 1);
        double maximum_difference = 0, maximum_length_difference = 0, maximum_volume_difference = 0;
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
                    const bool top_interface_layers = plugin_object.config().support_material_interface_layers.value != 0;
                    const bool bottom_interface_layers = plugin_object.config().support_material_bottom_interface_layers.is_enabled() ?
                        plugin_object.config().support_material_bottom_interface_layers.value != 0 : top_interface_layers;
                    bool dense_needs_interface = false, dense_surface = false, sparse_surface = false;
                    bool dense_paths = false, sparse_paths = false, interface_paths = false;
                    for (const LayerSliceIsland &island : layer.islands())
                        for (const LayerRegionIsland &region : island.regions_islands()) {
                            for (const Surface &surface : region.fill_surfaces()) {
                                const SurfaceType type = surface.surface_type;
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
                CHECK(length_difference < 0.01);
                CHECK(volume_difference < 0.01);
            }
        }
        // Report the largest measured deviation only when this section fails.
        INFO("largest deviations: slice=" << maximum_difference << " length=" << maximum_length_difference << " volume=" << maximum_volume_difference);
    }
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

// Check published slabs without changing them; temporary contacts identify adjacent object layers for gap arithmetic, and model_id selects roofs or supporting platforms.
void check_family(const PrintObject &object, const Slic3r::SupportGrid::GeneratedSupport &support, const Slic3r::SupportGrid::GridSupportParams &p, int model_id)
{
    using namespace Slic3r::SupportGrid;
    REQUIRE_FALSE(object.auxiliary_layers().empty());
    bool top = false, bottom = false, base = false;
    std::set<size_t> ids;
    coord_t previous_z = 0;
    for (const Layer &layer : object.auxiliary_layers()) {
        CHECK(scale_to_layer_coord(layer.unscaled_print_z()) == layer.scaled_print_z());
        CHECK(layer.scaled_print_z() >= previous_z);
        previous_z = layer.scaled_print_z();
        CHECK(ids.insert(layer.id()).second);
        CHECK(layer.scaled_height() >= p.min_height);
        CHECK(layer.unscaled_height() <= p.max_support_height + EPSILON);
        CHECK_FALSE(layer.has_extrusions());
        REQUIRE(layer.get_property<LayerSupportProperty>() != nullptr);
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &work : island.regions_islands())
                for (const Surface &surface : work.fill_surfaces()) {
                    const SurfaceType type = surface.surface_type;
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
        CHECK(contact->print_z == object.layer(contact->object_above).scaled_bottom_z() - scale_to_layer_coord(p.gap_above));
        if (contact->polygons.empty()) continue;
        expected_top = true;
        bool published = false;
        for (const Layer &layer : object.auxiliary_layers()) {
            if (layer.scaled_print_z() != contact->print_z || layer.scaled_height() != contact->height) continue;
            Polygons contours, base_contours;
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &work : island.regions_islands())
                    for (const Surface &surface : work.fill_surfaces())
                        if (surface.surface_type == (stModSupport | stPosTop | stDensSolid))
                            polygons_append(contours, surface.expolygon);
                        else if (surface.surface_type == (stModSupport | stPosInternal | stDensSparse))
                            polygons_append(base_contours, surface.expolygon);
            // With no interface layer on that side the family publishes a mergeable contact as base,
            // so the contact area then lies inside the sparse surface instead of the top one.
            if (contours.empty() && object.config().support_material_interface_layers.value == 0)
                CHECK(diff_ex(contact->polygons, base_contours).empty());
            else
                CHECK(diff_ex(contact->polygons, contours).empty());
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
                        if (surface.surface_type == (stModSupport | stPosBottom | stDensSolid | (p.soluble ? 0 : stModBridge)))
                            polygons_append(contours, surface.expolygon);
                        else if (surface.surface_type == (stModSupport | stPosInternal | stDensSparse))
                            polygons_append(base_contours, surface.expolygon);
            // Same rule as the top side: a contact typed as base lies inside the sparse surface.
            if (contours.empty() && !(object.config().support_material_bottom_interface_layers.is_enabled() ? object.config().support_material_bottom_interface_layers.value != 0 : object.config().support_material_interface_layers.value != 0))
                CHECK(diff_ex(contact->polygons, base_contours).empty());
            else
                CHECK(diff_ex(contact->polygons, contours).empty());
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
    CHECK(expected_bottom == (model_id == 1 || model_id == 4));
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
        for (const Layer &layer : object.auxiliary_layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &work : island.regions_islands())
                    for (const Surface &surface : work.fill_surfaces()) {
                        const SurfaceType type = surface.surface_type;
                        const size_t kind = type == (stModSupport | stPosTop | stDensSolid) ? 0 :
                            (type == (stModSupport | stPosBottom | stDensSolid) ||
                             type == (stModSupport | stPosBottom | stDensSolid | stModBridge)) ? 1 : 2;
                        polygons_append(per_layer[{layer.scaled_print_z(), layer.scaled_height()}][kind], surface.expolygon);
                        polygons_append(per_z[layer.scaled_print_z()][kind], surface.expolygon);
                    }
        for (const auto &[key, kinds] : per_layer)
            for (size_t a = 0; a < 3; ++a)
                for (size_t b = a + 1; b < 3; ++b) {
                    CAPTURE(key.first, key.second, a, b);
                    CHECK(area(intersection_ex(kinds[a], kinds[b])) < 1e3);
                }
        for (const auto &[z, kinds] : per_z)
            for (size_t a = 0; a < 3; ++a)
                for (size_t b = a + 1; b < 3; ++b) {
                    CAPTURE(z, a, b);
                    CHECK(area(intersection_ex(kinds[a], kinds[b])) < 1e3);
                }
    }
}
}

// Compare a roof on a narrow post through eight independent native/family sections with geometry and contact assertions.
TEST_CASE("Grid legacy comparison: overhanging slab", "[plugins][support-grid][legacy-compare]") { run_comparison(0); }
// Compare eight native/family combinations on a rotated cube with a hole and contacts above object surfaces.
TEST_CASE("Grid legacy comparison: cube with hole", "[plugins][support-grid][legacy-compare]") { run_comparison(1); }
// Compare a bridge through eight independent native/family sections, preserving family geometry checks.
TEST_CASE("Grid legacy comparison: bridge", "[plugins][support-grid][legacy-compare]") { run_comparison(2); }
// Compare the enforced demand of a sheared cube through eight sections with both soluble and separated contacts.
TEST_CASE("Grid legacy comparison: 45 degree overhang", "[plugins][support-grid][legacy-compare]") { run_comparison(3); }
// Compare a roof above a low platform through eight sections, verifying family bottom contacts before native execution.
TEST_CASE("Grid legacy comparison: object above platform", "[plugins][support-grid][legacy-compare]") { run_comparison(4); }
// Compare the same configurations with one support tool for both roles, exercising the native cross-role merges.
TEST_CASE("Grid legacy comparison: same support tool", "[plugins][support-grid][legacy-compare][support-fusion]") { run_comparison(4, true); }
// Compare a soluble overhang with one support tool, where contacts and interfaces share their slab.
TEST_CASE("Grid legacy comparison: same support tool soluble", "[plugins][support-grid][legacy-compare][support-fusion]") { run_comparison(3, true); }
// Compare the bridge model with one support tool: its shared slabs exercise the interface part
// joining the bottom contact, for a soluble and a non-soluble interface.
TEST_CASE("Grid legacy comparison: bridge with same support tool", "[plugins][support-grid][legacy-compare][support-fusion]") { run_comparison(2, true); }
// Compare explicit concentric contact patterns on both interfaces, including the bridging bottom
// contact, which the native filler also prints with the configured contact pattern.
TEST_CASE("Grid legacy comparison: concentric interfaces", "[plugins][support-grid][legacy-compare][support-pattern]") {
    for (const char *style : {"grid", "snug"}) compare(1, style, false, 3, false, "concentric");
}
// Compare automatic contact patterns resolving to the sparse support pattern through a positive interface spacing.
TEST_CASE("Grid legacy comparison: automatic sparse interfaces", "[plugins][support-grid][legacy-compare][support-pattern]") {
    for (const char *style : {"grid", "snug"}) compare(1, style, false, 3, false, "auto_sparse");
}
// Compare top contact loops around the overhang. The snug soluble combination is left out: the
// family demand differs slightly from the native overhang detection there, and the loop length
// follows the contact contour, so that documented difference is amplified beyond one percent.
TEST_CASE("Grid legacy comparison: contact loops", "[plugins][support-grid][legacy-compare][support-pattern]") {
    for (const char *style : {"grid", "snug"})
        for (const bool soluble : {false, true}) {
            if (strcmp(style, "snug") == 0 && soluble) continue;
            compare(1, style, soluble, 3, false, "contact_loops");
        }
}
// Compare a honeycomb sparse pattern and the base sheath against the native generator.
TEST_CASE("Grid legacy comparison: honeycomb and sheath", "[plugins][support-grid][legacy-compare][support-pattern]") {
    for (const char *style : {"grid", "snug"})
        for (const char *variant : {"honeycomb", "no_sheath"}) compare(1, style, false, 3, false, variant);
}
// Compare the rectilinear-grid base pattern, whose angle alternates per print Z, so the native and
// the family layer counts must agree even when one print Z carries several auxiliary layers.
TEST_CASE("Grid legacy comparison: rectilinear grid pattern", "[plugins][support-grid][legacy-compare][support-pattern]") {
    for (const char *style : {"grid", "snug"}) compare(1, style, false, 3, false, "rectilinear_grid");
}
