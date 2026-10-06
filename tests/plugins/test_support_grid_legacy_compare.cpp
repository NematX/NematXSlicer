#include "support_grid_test_helpers.hpp"
#include <iostream>
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Plugins/Support/Grid/GridSupportGenerator.hpp"
#include "libslic3r/Support/SupportMaterial.hpp"

/* Comparison runs identical object-slicing prefixes, then chooses the new
   grid family or native legacy generator. Auxiliary-layer contours, heights
   and print Z are measured before extrusion roles can affect the comparison.
   Per-layer symmetric difference measures geometry rather than polygon order.
    The legacy generator also generates paths, which these tests do not compare.
    Its region surface caches are reconstructed from current raw slices and
    island fill surfaces because production no longer fills those caches. Its
    concrete extrusion classes are rebuilt from generic tree leaves with the
    same geometry and effective flow attributes, preserving loop boundaries. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;

namespace {
// Run one model through every style, contact-distance and interface-count combination.
void run_comparison(int model_id);
// Check published geometry and contact spacing independently of the native comparison.
void check_family(const PrintObject &object, const Slic3r::SupportGrid::GeneratedSupport &support, const Slic3r::SupportGrid::GridSupportParams &params, int model_id);
// Populate deprecated region caches and concrete extrusion classes for native support.
void prepare_legacy_inputs(PrintObject &object);
// Copy generic extrusion leaves into concrete paths with resolved flow attributes.
void legacy_paths(const ExtrusionEntity &entity, ExtrusionPaths &paths, const ExtrusionAttributes *attributes = nullptr);
// Rebuild concrete loops and paths for the native support reader.
void legacy_extrusions(const ExtrusionEntity &entity, ExtrusionEntityCollection &out, const ExtrusionAttributes *attributes = nullptr);

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
// Slice two owned prints for one model and compare native output after independent family checks; each parameter section keeps exact Z/thickness and the fixed XY tolerance.
void run_comparison(int model_id)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("grid", "snug");
    const bool soluble = GENERATE(false, true);
    const int interfaces = GENERATE(0, 3);
    DYNAMIC_SECTION("model=" << model_id << " style=" << style << " soluble=" << soluble << " interfaces=" << interfaces) {
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
        CAPTURE(model_id, style, soluble, interfaces);
        DynamicPrintConfig config = SG::config(style, soluble, interfaces);
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

        PrintObject &legacy_object = legacy_print.object(0);
        prepare_legacy_inputs(legacy_object);
        std::cout << "support native run model=" << model_id << " style=" << style << " soluble=" << soluble
                  << " interfaces=" << interfaces << std::endl;
        PrintObjectSupportMaterial generator(&legacy_object, std::make_shared<SlicingParameters>(legacy_object.slicing_parameters()));
        generator.generate(legacy_object);
        const LayerRefs generated = plugin_print.object(0).auxiliary_layers();
        const LayerRefs legacy = legacy_object.auxiliary_layers();
        REQUIRE_FALSE(generated.empty());
        REQUIRE_FALSE(legacy.empty());
        CAPTURE(generated.size(), legacy.size());
        REQUIRE(generated.size() == legacy.size());
        double maximum_difference = 0;
        for (size_t id = 0; id < legacy.size(); ++id) {
            CAPTURE(id);
            CHECK(generated[id].scaled_print_z() == legacy[id].scaled_print_z());
            CHECK(generated[id].scaled_height() == legacy[id].scaled_height());
            const ExPolygons difference = union_ex(diff_ex(generated[id].lslices(), legacy[id].lslices()),
                                                   diff_ex(legacy[id].lslices(), generated[id].lslices()));
            const double reference_area = area(legacy[id].lslices());
            const double ratio = reference_area > 0 ? area(difference) / reference_area : area(difference);
            maximum_difference = std::max(maximum_difference, ratio);
            INFO("symmetric difference / legacy area = " << ratio);
            CHECK(ratio < 0.01);
        }
        // A successful run reports the largest measured deviation for each parameter combination.
        std::cout << "support comparison model=" << model_id << " style=" << style << " soluble=" << soluble
                  << " interfaces=" << interfaces << " layers=" << legacy.size() << " max_difference=" << maximum_difference << '\n';
    }
}

// Check published slabs without changing them; temporary contacts identify adjacent object layers for gap arithmetic, and model_id selects roofs or supporting platforms.
void check_family(const PrintObject &object, const Slic3r::SupportGrid::GeneratedSupport &support, const Slic3r::SupportGrid::GridSupportParams &p, int model_id)
{
    using namespace Slic3r::SupportGrid;
    REQUIRE_FALSE(object.auxiliary_layers().empty());
    bool top = false, bottom = false, base = false;
    for (const Layer &layer : object.auxiliary_layers()) {
        CHECK(scale_to_layer_coord(layer.unscaled_print_z()) == layer.scaled_print_z());
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
            if (layer.scaled_print_z() != contact->print_z) continue;
            Polygons contours;
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &work : island.regions_islands())
                    for (const Surface &surface : work.fill_surfaces())
                        if (surface.surface_type == (stModSupport | stPosTop | stDensSolid))
                            polygons_append(contours, surface.expolygon);
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
            if (layer.scaled_print_z() != contact->print_z) continue;
            CHECK(layer.scaled_print_z() - layer.scaled_height() == contact->bottom_z);
            Polygons contours;
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &work : island.regions_islands())
                    for (const Surface &surface : work.fill_surfaces())
                        if (surface.surface_type == (stModSupport | stPosBottom | stDensSolid | (p.soluble ? 0 : stModBridge)))
                            polygons_append(contours, surface.expolygon);
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
    if (model_id != 3) CHECK(expected_top);
    CHECK(expected_base);
    CHECK(expected_bottom == (model_id == 1 || model_id == 4));
    CHECK(top == expected_top);
    CHECK(bottom == expected_bottom);
    CHECK(base == expected_base);
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
