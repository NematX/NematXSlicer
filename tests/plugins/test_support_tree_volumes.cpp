#include "polygon_metrics_test_helpers.hpp"
#include "support_grid_test_helpers.hpp"
#include "support_tree_test_helpers.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeElements.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeGeometry.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeSettings.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeVolumes.hpp"

#include <algorithm>
#include <atomic>
#include <exception>
#include <mutex>
#include <random>
#include <stdexcept>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <boost/core/null_deleter.hpp>
#include <boost/log/core.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <oneapi/tbb/parallel_for.h>
#include <oneapi/tbb/task_group.h>

/* TreeVolumes tests. TreeVolumes caches the collision, avoidance, placeable and wall restriction areas that the tree routing reads.
   precalculate() fills the caches for the keys (radius, layer) a tree normally requests; any other key is computed when it is first
   requested, by whichever thread asks, while other threads read and fill the same caches. The tests check what that lazy path
   must guarantee:
   - an area computed on demand equals the one of a precalculated reference (deterministic, one thread);
   - the placeable areas of radius 0 are placeable areas also when nothing was precalculated;
   - a reference returned by a getter stays valid and unchanged when its cache grows to a later layer;
   - many threads requesting keys that are not precalculated all get the reference areas (stress test);
   - a lazy computation checks the cancellation of the context given to the constructor and throws PluginCancelled, also from a cancelled TBB group, without publishing a partial area;
   - the support blockers: one box, a curved blocker (cylinder) and two overlapping boxes are sliced like the native ones, within the
      resolution of the slicing, and enter the collision;
   - physical Z gaps include only closer object layers (within Z rounding tolerance), and model-root surfaces use the last layer
     at or below the bottom-gap boundary. A one-layer ledge makes these choices observable independently of branch routing.
   After precalculate(), a lazy computation logs an error line "Had to calculate ...". The tests capture the log to assert that a
   request really took the lazy path, so that they do not silently test the caches only. */

using namespace Slic3r;
namespace PM = Slic3r::Test::PolygonMetrics;
namespace SG = Slic3r::Test::SupportGrid;
namespace ST = Slic3r::Test::SupportTree;
namespace TF = Slic3r::TreeFamily;

namespace {
using Volumes = TF::TreeVolumes;

// Builds a sliced post with one thin ledge in the selected height band, then checks a probe point under the ledge only.
// Prints and polygon storage are local owners; slicing and allocation failures propagate to Catch.
void check_physical_gap_ledge(bool variable, double band_z, const char *uniform_gap = "0.25");

// Check a positive-XY collision ramp below a single-layer ledge, either in a thin band or next to the object's top.
void check_collision_ramp(bool virtual_top);

// Slices an owned post twice to place a one-layer ledge at a real slice Z, then checks a point under the ledge only against all collision and placeable
// windows. variable selects a thin/thick profile, band_z selects its band and uniform_gap supplies the uniform-case clearance in mm;
// slicing and allocation failures propagate to Catch.
void check_physical_gap_ledge(bool variable, double band_z, const char *uniform_gap)
{
    DynamicPrintConfig config = SG::config("tree", false, 3);
    config.set_deserialize_strict({{"support_material_xy_spacing", "0"},
        {"support_material_contact_distance", variable ? "0.2" : uniform_gap},
        {"support_material_bottom_contact_distance", variable ? "0.2" : uniform_gap}});
    const std::vector<coordf_t> profile = variable ?
        std::vector<coordf_t>{0., 0.05, 2., 0.05, 2.01, 0.3, 6., 0.3} : std::vector<coordf_t>{};
    Print seed;
    Model seed_model;
    if (variable)
        SG::init_print_with_profile(seed, seed_model, make_cube(2., 2., 6.), profile, config);
    else
        Test::init_print({make_cube(2., 2., 6.)}, seed, seed_model, config);
    SG::prefix(seed);
    size_t j = 0;
    while (j < seed.object(0).layer_count() && seed.object(0).layer(j).unscaled_print_z() < band_z)
        ++j;
    REQUIRE(j + 5 < seed.object(0).layer_count());
    TriangleMesh mesh = make_cube(2., 2., 6.);
    TriangleMesh ledge = make_cube(4., 2., 0.01);
    ledge.translate(0., 0., seed.object(0).layer(j).slice_z - 0.005);
    mesh.merge(ledge);
    Print print;
    Model model;
    if (variable)
        SG::init_print_with_profile(print, model, std::move(mesh), profile, config);
    else
        Test::init_print({mesh}, print, model, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() == seed.object(0).layer_count());
    const BoundingBox bounds = get_extents(object.layer(j).lslices());
    const Point probe(bounds.max.x() - scale_i(0.5), (bounds.min.y() + bounds.max.y()) / 2);
    const slic3r_api::Support::Common::SupportParams params = SG::params(print, object);
    const TF::TreeSettings settings = ST::tree_settings(print, object, params);
    REQUIRE(settings.support_rests_on_model);
    PluginStorage storage;
    Volumes volumes(SG::object_view(object), SG::print_view(print), settings, SG::handle(storage), nullptr);
    size_t above_hits = 0, below_hits = 0, placeable_hits = 0;
    for (size_t layer = 0; layer < object.layer_count(); ++layer) {
        CAPTURE(variable, band_z, j, layer);
        // Only the ledge layer contains this point; the post is two millimetres narrower.
        bool object_contains = false;
        for (const ExPolygon &polygon : object.layer(layer).lslices())
            object_contains = object_contains || polygon.contains(probe);
        REQUIRE(object_contains == (layer == j));
        bool collision_contains = false;
        for (const Polygon &polygon : SG::native(volumes.getCollision(0, TF::LayerIndex(layer), true)))
            if (polygon.contains(probe))
                collision_contains = !collision_contains;
        const coord_t eps = scale_to_layer_coord(EPSILON);
        const bool expected = layer == j || (j > layer ?
            object.layer(j).scaled_bottom_z() + eps < object.layer(layer).scaled_print_z() + scale_to_layer_coord(params.gap_above) :
            object.layer(j).scaled_print_z() > object.layer(layer).scaled_bottom_z() - scale_to_layer_coord(params.gap_below) + eps);
        CHECK(collision_contains == expected);
        above_hits += expected && j > layer;
        below_hits += expected && j < layer;
        // The last surface at or below the physical bottom-gap boundary is the only possible model-root surface.
        int surface = -1;
        for (size_t below = 0; below < layer; ++below)
            if (object.layer(below).scaled_print_z() <= object.layer(layer).scaled_bottom_z() - scale_to_layer_coord(params.gap_below) + eps)
                surface = int(below);
        bool placeable_contains = false;
        for (const Polygon &polygon : SG::native(volumes.getPlaceableAreas(0, TF::LayerIndex(layer))))
            if (polygon.contains(probe))
                placeable_contains = !placeable_contains;
        CHECK(placeable_contains == (surface == int(j)));
        placeable_hits += placeable_contains;
    }
    REQUIRE(above_hits > 0);
    REQUIRE(below_hits > 0);
    REQUIRE(placeable_hits > 0);
    if (!variable) {
        // At h=0.2, gaps 0.08, 0.25 and 0.5 include respectively one, two and three neighbours on either side.
        const uint32_t count = std::string(uniform_gap) == "0.08" ? 1u : std::string(uniform_gap) == "0.5" ? 3u : 2u;
        for (size_t layer = 0; layer < object.layer_count(); ++layer) {
            CHECK(settings.z_distance_top_layers[layer] == count);
            CHECK(settings.z_distance_bottom_layers[layer] == std::min<size_t>(count, layer));
        }
        CHECK(above_hits == count);
        CHECK(below_hits == count);
        if (std::string(uniform_gap) == "0.08")
            CHECK(std::round(params.gap_above / 0.2) == 0.);
        if (std::string(uniform_gap) == "0.5")
            CHECK(std::round(params.gap_above / 0.2) == 3.);
    } else if (band_z < 2.) {
        // In the 0.05 band, gap=0.2 includes clearances 0, 0.05, 0.10, 0.15, but not 0.20.
        CHECK(settings.z_distance_top_layers[j] == 4);
        CHECK(settings.z_distance_bottom_layers[j] == 4);
    } else {
        // In the 0.3 band, only clearance zero is below gap=0.2.
        CHECK(settings.z_distance_top_layers[j] == 1);
        CHECK(settings.z_distance_bottom_layers[j] == 1);
    }
}

// Slices locally owned posts to put a ledge at one real slice Z, then probes between the correct ramp and a one-layer ramp.
// virtual_top selects uniform layers with a virtual extension; storage owns both comparison caches and failures reach Catch.
void check_collision_ramp(bool virtual_top)
{
    DynamicPrintConfig config = SG::config("tree", false, 3);
    config.set_deserialize_strict({{"support_material_xy_spacing", "0.4"},
        {"support_material_contact_distance", virtual_top ? "0.6" : "0.2"},
        {"support_material_bottom_contact_distance", virtual_top ? "0.6" : "0.2"}});
    const std::vector<coordf_t> profile{0., 0.05, 2., 0.05, 2.01, 0.3, 6., 0.3};
    Print seed;
    Model seed_model;
    if (virtual_top)
        Test::init_print({make_cube(2., 2., 6.)}, seed, seed_model, config);
    else
        SG::init_print_with_profile(seed, seed_model, make_cube(2., 2., 6.), profile, config);
    SG::prefix(seed);
    size_t j = virtual_top ? seed.object(0).layer_count() - 1 : 1;
    while (!virtual_top && seed.object(0).layer(j).unscaled_print_z() < 1.)
        ++j;
    TriangleMesh mesh = make_cube(2., 2., 6.);
    TriangleMesh ledge = make_cube(4., 2., 0.01);
    ledge.translate(0., 0., seed.object(0).layer(j).slice_z - 0.005);
    mesh.merge(ledge);
    Print print;
    Model model;
    if (virtual_top)
        Test::init_print({mesh}, print, model, config);
    else
        SG::init_print_with_profile(print, model, std::move(mesh), profile, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() == seed.object(0).layer_count());
    const TF::TreeSettings settings = ST::tree_settings(print, object, SG::params(print, object));
    const TF::LayerIndex layer = TF::LayerIndex(j - 1);
    REQUIRE(settings.z_distance_top_layers[layer] == (virtual_top ? 3u : 4u));
    if (virtual_top) {
        // 0.6 on 0.2 layers is the smallest simple gap whose near-top collision boundary differs when virtual layers are ignored.
        CHECK(settings.z_distance_top_layers[j] == 3);
        CHECK(settings.z_distance_top_layers[j - 2] == 3);
        CHECK(settings.z_distance_bottom_layers[j] == 3);
    }
    const coord_t radius = settings.min_radius;
    const coord_t xy = settings.xy_min_distance;
    REQUIRE(radius > 0);
    REQUIRE(xy > 0);
    const BoundingBox bounds = get_extents(object.layer(j).lslices());
    // At i=1, n=4 reaches 3xy/4 and n=3 reaches 2xy/3; n=1 reaches only xy/2. The radius shifts all boundaries equally.
    const coord_t extra = virtual_top ? 7 * xy / 12 : 5 * xy / 8;
    const Point probe(bounds.max.x() + radius + extra, (bounds.min.y() + bounds.max.y()) / 2);
    PluginStorage storage;
    Volumes volumes(SG::object_view(object), SG::print_view(print), settings, SG::handle(storage), nullptr);
    bool correct_contains = false;
    for (const Polygon &polygon : SG::native(volumes.getCollision(radius, layer, true)))
        if (polygon.contains(probe))
            correct_contains = !correct_contains;
    CHECK(correct_contains);
    TF::TreeSettings truncated = settings;
    truncated.z_distance_top_layers[layer] = 1;
    Volumes one_layer(SG::object_view(object), SG::print_view(print), truncated, SG::handle(storage), nullptr);
    bool truncated_contains = false;
    for (const Polygon &polygon : SG::native(one_layer.getCollision(radius, layer, true)))
        if (polygon.contains(probe))
            truncated_contains = !truncated_contains;
    CHECK_FALSE(truncated_contains);
}

// Layers that precalculate() computes for the volumes under test, the layer at which they are requested (not precalculated), and the
// layers that precalculate() computes for the reference, which answers from its caches.
constexpr TF::LayerIndex precalculated_layers = 2;
constexpr TF::LayerIndex lazy_layer = 4;
constexpr TF::LayerIndex reference_layers = 8;

// Collects the text of the Boost.Log records emitted while it is alive, so a test can tell whether TreeVolumes took its lazy path.
class LogCapture {
    using Backend = boost::log::sinks::text_ostream_backend;
    using Sink    = boost::log::sinks::synchronous_sink<Backend>;
public:
    LogCapture()
    {
        boost::shared_ptr<Backend> backend = boost::make_shared<Backend>();
        backend->add_stream(boost::shared_ptr<std::ostream>(&m_text, boost::null_deleter()));
        backend->auto_flush(true);
        m_sink = boost::make_shared<Sink>(backend);
        boost::log::core::get()->add_sink(m_sink);
    }
    ~LogCapture() { boost::log::core::get()->remove_sink(m_sink); }
    LogCapture(const LogCapture &) = delete;
    LogCapture &operator=(const LogCapture &) = delete;

    // Number of times the text appears in the records captured so far.
    size_t count(const std::string &needle) const
    {
        // The sink writes into m_text under this lock.
        const Sink::locked_backend_ptr lock = m_sink->locked_backend();
        const std::string text = m_text.str();
        size_t found = 0;
        for (size_t at = text.find(needle); at != std::string::npos; at = text.find(needle, at + needle.size()))
            ++ found;
        return found;
    }

private:
    std::ostringstream      m_text;
    boost::shared_ptr<Sink> m_sink;
};

// The first object of a sliced print, with the tree settings read from it. The default XY spacing of 3 mm makes the difference
// between the regular and the minimum XY distance large, so the radii with and without the minimum XY distance fall into
// different buckets.
struct Scene {
    // The storage owns the areas of the volumes made from the scene, so it is declared first and destroyed last.
    PluginStorage    storage;
    Print            print;
    Model            model;
    TF::TreeSettings settings;

    explicit Scene(bool build_plate_only = false, const char *xy_spacing = "3")
    {
        DynamicPrintConfig config = SG::config("tree", false, 3);
        config.set_deserialize_strict({{"support_material_xy_spacing", xy_spacing}});
        if (build_plate_only)
            config.set_deserialize_strict({{"support_material_buildplate_only", "1"}});
        Test::init_print({SG::slab(true)}, print, model, config);
        SG::prefix(print);
        settings = Slic3r::Test::SupportTree::tree_settings(print, print.object(0), SG::params(print, print.object(0)));
    }
    const PrintObject &object() const { return print.object(0); }
    // New volumes over the first object, owning their areas in the storage of the scene; the context (nullptr never cancels) is the one they check.
    Volumes volumes(const plugin_run_context *context = nullptr) { return Volumes(SG::object_view(object()), SG::print_view(print), settings, SG::handle(storage), context); }
};

// One area that TreeVolumes can give: its kind and the arguments of the getter.
enum class Kind { Collision, Avoidance, Placeable, Wall };
struct Request {
    Kind                 kind;
    coord_t              radius;
    TF::LayerIndex       layer;
    bool                 min_xy_dist = true;
    Volumes::AvoidanceType type      = Volumes::AvoidanceType::Fast;
    bool                 to_model    = false;
};

// Calls the getter of the request.
const slic3r_api::StoredPolygonCollection &fetch(const Volumes &volumes, const Request &request)
{
    switch (request.kind) {
    case Kind::Collision: return volumes.getCollision(request.radius, request.layer, request.min_xy_dist);
    case Kind::Avoidance: return volumes.getAvoidance(request.radius, request.layer, request.type, request.to_model, request.min_xy_dist);
    case Kind::Placeable: return volumes.getPlaceableAreas(request.radius, request.layer);
    case Kind::Wall:      return volumes.getWallRestriction(request.radius, request.layer, request.min_xy_dist);
    }
    return volumes.getCollision(request.radius, request.layer, request.min_xy_dist);
}

// The start of the log line that the getter writes when it has to compute the request, with the radius rounded as the getter does.
std::string lazy_message(const Volumes &volumes, const Request &request)
{
    const std::string key = std::to_string(volumes.ceilRadius(request.radius, request.min_xy_dist)) + " and layer " + std::to_string(request.layer) + ",";
    switch (request.kind) {
    case Kind::Collision: return "Had to calculate collision at radius " + key;
    case Kind::Avoidance: return std::string(request.to_model ? "Had to calculate Avoidance to model at radius " : "Had to calculate Avoidance at radius ") + key;
    // The placeable areas and the wall restrictions round the radius without the XY distance delta.
    case Kind::Placeable: return "Had to calculate Placeable Areas at radius " + std::to_string(volumes.ceilRadius(request.radius, true)) + " and layer " + std::to_string(request.layer) + ",";
    case Kind::Wall:      return "Had to calculate Wall restrictions at radius " + std::to_string(volumes.ceilRadius(request.radius, true)) + " and layer " + std::to_string(request.layer) + ",";
    }
    return {};
}

// A readable form of the request for the failure messages.
std::string describe(const Request &request)
{
    static const char *const kinds[] = { "collision", "avoidance", "placeable", "wall restriction" };
    static const char *const types[] = { "slow", "fast safe", "fast" };
    std::string text = std::string(kinds[int(request.kind)]) + " radius " + std::to_string(request.radius) + " layer " + std::to_string(request.layer);
    if (request.kind == Kind::Avoidance)
        text += std::string(" ") + types[int(request.type)] + (request.to_model ? " to model" : " to plate");
    if (request.kind != Kind::Placeable)
        text += request.min_xy_dist ? " min xy" : " regular xy";
    return text;
}

// The areas are computed by the same deterministic code from the same outlines, so equal areas have the same canonical rings (the
// rings are sorted and start at their smallest point, so that the order of a Clipper result does not matter).
using Rings = std::vector<ST::Ring>;
Rings rings(const slic3r_api::StoredPolygonCollection &area) { return ST::rings_of_view(area); }
bool same(const slic3r_api::StoredPolygonCollection &a, const slic3r_api::StoredPolygonCollection &b) { return rings(a) == rings(b); }
bool same(const slic3r_api::StoredPolygonCollection &a, const Rings &b) { return rings(a) == b; }
}

// Exercises one-layer ledges in thin and thick real-Z bands and in the conservative fractional-gap uniform case.
TEST_CASE("Tree collision and placeable windows respect physical Z gaps", "[plugins][support-tree-volumes][support-tree-physical-gap]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    check_physical_gap_ledge(true, 1.);
    check_physical_gap_ledge(true, 4.);
    check_physical_gap_ledge(false, 2.);
    check_collision_ramp(false);
    check_collision_ramp(true);
}

// Uniform windows and one-layer ledge collisions remain physical for gaps below half a layer and above a whole layer.
TEST_CASE("Tree windows include physical neighbours for small and large Z gaps", "[plugins][support-tree-volumes][support-tree-gap-sizes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *gap = GENERATE("0.08", "0.5");
    CAPTURE(gap);
    check_physical_gap_ledge(false, 2., gap);
}

// The support blockers of the object are avoided like the object. The volumes slice them with the helper shared with the demand
// modifiers, at the print resolution, where the native PrintObject::slice_support_blockers() uses the default resolution: a contour
// lies within the resolution of the native one, so the slices are compared with the native ones by the area between them, which is
// bounded by the perimeter times the resolution. The blocker volumes stand beside the post, under the roof.
namespace {
// A sliced print of the scene with the given blocker meshes (object coordinates, as the slab: the roof is at z 4 to 5).
void make_blocker_print(Print &print, Model &model, const std::vector<TriangleMesh> &blockers)
{
    DynamicPrintConfig config = SG::config("tree", false, 3);
    config.set_deserialize_strict({{"support_material_xy_spacing", "3"}});
    Test::init_print({SG::slab(true)}, print, model, config);
    if (! blockers.empty()) {
        for (ModelObject &model_object : model.objects())
            for (const TriangleMesh &blocker : blockers)
                model_object.add_volume(TriangleMesh(blocker), ModelVolumeType::SUPPORT_BLOCKER, false);
        print.apply(model, config);
        print.validate();
        print.set_status_silent();
    }
    SG::prefix(print);
}

// Slices the blocker volumes of the print with the helper TreeVolumes uses, takes the union of the volumes layer by layer with the
// native clipper, and checks on every layer that the area between this union and the native slice_support_blockers() stays within
// the bound of the layer. The bound is 1e-3 mm2 when nothing is simplified (boxes: only rounding differs) and the perimeter times
// the resolution of the slicing when the outline is curved (the cleaning of the outline moves it by up to the resolution).
// Returns the number of layers that have a native slice.
size_t compare_blocker_slices(const Print &print, storage_handle *run_storage, bool curved)
{
    size_t sliced_layers = 0;
    const slic3r_api::Object object = SG::object_view(print.object(0));
    const std::vector<ExPolygons> native = print.object(0).slice_support_blockers();
    const std::vector<float> zs = slic3r_api::Support::Common::layer_slice_zs(object);
    const c_mesh_slicing_params params = slic3r_api::Support::Common::support_modifier_slicing_params(SG::print_view(print), object);
    const double resolution = scale_d(params.resolution);
    std::vector<Polygons> api(native.size());
    for (uint32_t idx = 0; idx < object.volume_count(); ++ idx) {
        if (object.volume(idx).type() != RAW_VOLUME_TYPE_SUPPORT_BLOCKER)
            continue;
        const std::vector<slic3r_api::StoredExPolygonCollection> slices =
            slic3r_api::Support::Common::slice_support_modifier_volume(object.volume(idx), zs, params, run_storage, nullptr);
        REQUIRE(slices.size() == native.size());
        for (size_t layer_idx = 0; layer_idx < slices.size(); ++ layer_idx) {
            const slic3r_api::StoredPolygonCollection rings = TF::to_polygons(run_storage, slices[layer_idx]);
            append(api[layer_idx], SG::native(rings));
        }
    }
    for (size_t layer_idx = 0; layer_idx < native.size(); ++ layer_idx) {
        const Polygons native_rings = to_polygons(native[layer_idx]);
        const Polygons united = union_(api[layer_idx]);
        const double difference_mm2 = PM::sym_diff_mm2(united, native_rings);
        const double bound_mm2 = curved ? PM::mm2(total_length(native_rings) * resolution) : 1e-3;
        CAPTURE(layer_idx, difference_mm2, bound_mm2);
        CHECK(difference_mm2 <= bound_mm2);
        sliced_layers += native_rings.empty() ? 0 : 1;
    }
    return sliced_layers;
}
// A box of the given size with its minimum corner at (x, y, z), in object coordinates.
TriangleMesh blocker_box(double x, double y, double z, double sx, double sy, double sz)
{
    TriangleMesh box = make_cube(sx, sy, sz);
    box.translate(x, y, z);
    return box;
}

// The collision of radius 0 on a layer, copied into the run storage, for the scene with the given blockers.
slic3r_api::StoredPolygonCollection blocker_collision(storage_handle *run_storage, const std::vector<TriangleMesh> &blockers, TF::LayerIndex layer)
{
    Print print;
    Model model;
    make_blocker_print(print, model, blockers);
    const TF::TreeSettings settings = ST::tree_settings(print, print.object(0), SG::params(print, print.object(0)));
    Volumes volumes(SG::object_view(print.object(0)), SG::print_view(print), settings, run_storage, nullptr);
    slic3r_api::StoredPolygonCollection copy(run_storage);
    copy.copy_from(volumes.getCollision(0, layer, true));
    return copy;
}

}

// One box: its collision contains the collision of the object alone and is larger by the box.
TEST_CASE("Support blockers are part of the collision", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    // The storage owns the collections below and the areas of the volumes, so it is declared first.
    PluginStorage storage;
    storage_handle *run_storage = SG::handle(storage);
    constexpr TF::LayerIndex layer = 8;
    const slic3r_api::StoredPolygonCollection without = blocker_collision(run_storage, {}, layer);
    const slic3r_api::StoredPolygonCollection with = blocker_collision(run_storage, { blocker_box(5., 2., 1., 3., 3., 2.) }, layer);
    CHECK(PM::mm2(TF::polygons_area(with) - TF::polygons_area(without)) > 5.);
    const slic3r_api::StoredPolygonCollection lost = TF::polygons_diff(run_storage, without, with);
    CHECK(PM::mm2(TF::polygons_area(lost)) < 1e-3);

    Print print;
    Model model;
    make_blocker_print(print, model, { blocker_box(5., 2., 1., 3., 3., 2.) });
    CHECK(compare_blocker_slices(print, run_storage, false) > 0);
}

// A curved blocker: the slices follow the native ones within the resolution along the whole contour.
TEST_CASE("Curved support blockers are sliced within the resolution of the native slices", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    PluginStorage storage;
    Print print;
    Model model;
    TriangleMesh cylinder = make_cylinder(1.5, 2.);
    cylinder.translate(6.5, 4., 1.);
    make_blocker_print(print, model, { cylinder });
    CHECK(compare_blocker_slices(print, SG::handle(storage), true) > 0);
}

// Two overlapping blockers are united: the slices equal the native union, and the collision equals the one of a single blocker that
// has the shape of the two (the offset of the union is the union of the offsets for boxes, up to the resolution).
TEST_CASE("Overlapping support blockers are united like the native ones", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    PluginStorage storage;
    storage_handle *run_storage = SG::handle(storage);
    constexpr TF::LayerIndex layer = 8;
    TriangleMesh first = blocker_box(5., 2., 1., 3., 3., 2.);
    TriangleMesh second = blocker_box(6.5, 3.5, 1., 3., 3., 2.);
    TriangleMesh both = first;
    both.merge(second);

    Print print;
    Model model;
    make_blocker_print(print, model, { first, second });
    CHECK(compare_blocker_slices(print, run_storage, false) > 0);

    const slic3r_api::StoredPolygonCollection two_volumes = blocker_collision(run_storage, { first, second }, layer);
    const slic3r_api::StoredPolygonCollection one_volume = blocker_collision(run_storage, { both }, layer);
    const slic3r_api::StoredPolygonCollection only_first = blocker_collision(run_storage, { first }, layer);
    // The collision of the merged mesh and the one of the two volumes differ by rounding along the contour: perimeter x resolution.
    const double resolution = scale_d(SG::print_view(print).config().get("resolution").get_float());
    const double bound = PM::mm2(total_length(SG::native(one_volume)) * resolution);
    const slic3r_api::StoredPolygonCollection extra = TF::polygons_diff(run_storage, two_volumes, one_volume);
    const slic3r_api::StoredPolygonCollection missing = TF::polygons_diff(run_storage, one_volume, two_volumes);
    CHECK(PM::mm2(TF::polygons_area(extra)) <= bound);
    CHECK(PM::mm2(TF::polygons_area(missing)) <= bound);
    // The second blocker counts: its part outside of the first one enlarges the collision.
    CHECK(PM::mm2(TF::polygons_area(two_volumes) - TF::polygons_area(only_first)) > 2.);
}
// The placeable areas of radius 0 are computed together with the collision of radius 0. A cold cache must not hand the collision back
// as placeable areas: the first call, a second call and the precalculated reference give the same areas.
TEST_CASE("Cold radius 0 placeable areas are placeable areas and not collisions", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Scene scene;
    Volumes reference = scene.volumes();
    reference.precalculate(reference_layers);
    // A layer where the placeable areas exist and differ from the collision, so that mixing them up shows.
    TF::LayerIndex layer = -1;
    for (TF::LayerIndex idx = 2; idx <= reference_layers && layer < 0; ++ idx) {
        const slic3r_api::StoredPolygonCollection &placeable = reference.getPlaceableAreas(0, idx);
        if (placeable.size() > 0 && ! same(placeable, reference.getCollision(0, idx, true)))
            layer = idx;
    }
    REQUIRE(layer >= 0);
    const Rings expected  = rings(reference.getPlaceableAreas(0, layer));
    const Rings collision = rings(reference.getCollision(0, layer, true));

    Volumes cold = scene.volumes();
    const Rings first  = rings(cold.getPlaceableAreas(0, layer));
    const Rings second = rings(cold.getPlaceableAreas(0, layer));
    CHECK_FALSE(first == collision);
    CHECK(first == expected);
    CHECK(second == expected);
}

// Without resting on the model nobody places a branch on it, so no placeable area exists. The request still has to return, as it did
// before the placeable areas of radius 0 were re-read after their computation: it gives the collision.
TEST_CASE("Radius 0 placeable areas without resting on the model return the collision", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Scene scene(true);
    REQUIRE_FALSE(scene.settings.support_rests_on_model);
    Volumes volumes = scene.volumes();
    const Rings placeable = rings(volumes.getPlaceableAreas(0, lazy_layer));
    Volumes other = scene.volumes();
    CHECK(placeable == rings(other.getCollision(0, lazy_layer, true)));
}

// The caches promise that a returned reference stays valid while they grow. Hold one, ask the same cache for a later layer, which
// makes it grow, and check that the same object is still there with the same content.
TEST_CASE("References from the volume caches survive the growth of the cache", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Scene scene;
    const coord_t radius = scene.settings.branch_radius;
    const std::vector<Request> requests{
        Request{ Kind::Collision, radius, 1 },
        Request{ Kind::Avoidance, radius, 1 },
        Request{ Kind::Placeable, radius, 1 },
        Request{ Kind::Wall,      radius, 1 } };
    for (const Request &small : requests) {
        CAPTURE(describe(small));
        Request large = small;
        large.layer   = 6;
        Volumes volumes = scene.volumes();
        const slic3r_api::StoredPolygonCollection &held    = fetch(volumes, small);
        const slic3r_api::StoredPolygonCollection *address = &held;
        const Rings content = rings(held);
        // The later layer is not cached yet: the cache grows to hold it.
        fetch(volumes, large);
        const slic3r_api::StoredPolygonCollection &again = fetch(volumes, small);
        // The content is only read when the object is still the one that was returned: a destroyed one must not be touched.
        const bool stable = &again == address;
        CHECK(stable);
        if (stable)
            CHECK(rings(held) == content);
    }
}

// A branch without radius and without XY distance has no step to propagate its avoidance with, so its avoidance is its collision.
// The request has to return: nothing used to be published for such a key, and the getter computed it again for ever.
TEST_CASE("The avoidance of a branch that cannot move is its collision", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Scene scene(false, "0");
    REQUIRE(scene.settings.xy_min_distance == 0);
    REQUIRE(scene.settings.xy_distance == 0);
    for (Volumes::AvoidanceType type : { Volumes::AvoidanceType::Slow, Volumes::AvoidanceType::FastSafe, Volumes::AvoidanceType::Fast })
        for (bool to_model : { false, true }) {
            Request request{ Kind::Avoidance, 0, lazy_layer, true, type, to_model };
            CAPTURE(describe(request));
            // Nothing is precalculated, and radius 0 is not a key that precalculate() computes.
            Volumes volumes = scene.volumes();
            const slic3r_api::StoredPolygonCollection &avoidance = fetch(volumes, request);
            const slic3r_api::StoredPolygonCollection &collision = volumes.getCollision(0, lazy_layer, true);
            // To the model, the places where a branch can stand are free.
            const slic3r_api::StoredPolygonCollection expected = to_model ?
                TF::polygons_diff(SG::handle(scene.storage), collision, volumes.getPlaceableAreas(0, lazy_layer)) :
                TF::polygons_union(SG::handle(scene.storage), collision);
            if (type == Volumes::AvoidanceType::Fast)
                CHECK(same(avoidance, expected));
            else {
                // The slow and the holefree avoidances start from the holefree collision, which is not public: it contains the
                // plain collision.
                const slic3r_api::StoredPolygonCollection missing = TF::polygons_diff(SG::handle(scene.storage), expected, avoidance);
                CHECK(TF::polygons_area(missing) <= TF::sqr(scale_d(0.01)));
            }
        }
}

// With a branch diameter equal to the tip diameter and no difference between the regular and the minimum XY distance, no radius is
// smaller than the radius until which a branch widens, so precalculate() gives no key to the holefree collisions. An empty list of
// keys must be a no-op: its size is the grainsize of the TBB loop, which TBB rejects when it is not positive.
TEST_CASE("Precalculation without holefree collision keys completes", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print            print;
    Model            model;
    DynamicPrintConfig config = SG::config("tree", false, 3);
    config.set_deserialize_strict({{"support_material_xy_spacing", "0"},
                                   {"support_tree_branch_diameter", "0.42"},
                                   {"support_tree_tip_diameter", "0.42"}});
    Test::init_print({SG::slab(true)}, print, model, config);
    SG::prefix(print);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, print.object(0), SG::params(print, print.object(0)));
    REQUIRE(settings.xy_distance == settings.xy_min_distance);
    REQUIRE(settings.min_radius == settings.branch_radius);
    REQUIRE(settings.increase_radius_until_radius == settings.branch_radius);

    PluginStorage storage;
    Volumes volumes(SG::object_view(print.object(0)), SG::print_view(print), settings, SG::handle(storage), nullptr);
    volumes.precalculate(reference_layers);
    CHECK(volumes.getCollision(settings.branch_radius, 1, true).size() > 0);
}

// Every kind of area, requested for a layer that precalculate() did not compute (asserted from the log), equals the area of a
// reference that answers from its caches. One fresh volumes object per request, so that every request is lazy.
TEST_CASE("Areas computed on demand equal the precalculated areas", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Scene scene;
    const coord_t tip = scene.settings.min_radius, branch = scene.settings.branch_radius;
    std::vector<Request> requests;
    for (coord_t radius : { tip, branch })
        for (bool min_xy_dist : { true, false }) {
            requests.push_back(Request{ Kind::Collision, radius, lazy_layer, min_xy_dist });
            requests.push_back(Request{ Kind::Wall,      radius, lazy_layer, min_xy_dist });
        }
    // Radius 0 is computed with the collisions, the other radii from it.
    for (coord_t radius : { coord_t(0), tip, branch })
        requests.push_back(Request{ Kind::Placeable, radius, lazy_layer });
    for (Volumes::AvoidanceType type : { Volumes::AvoidanceType::Slow, Volumes::AvoidanceType::FastSafe, Volumes::AvoidanceType::Fast })
        for (bool to_model : { false, true })
            for (bool min_xy_dist : { true, false })
                requests.push_back(Request{ Kind::Avoidance, tip, lazy_layer, min_xy_dist, type, to_model });

    const LogCapture capture;
    Volumes reference = scene.volumes();
    reference.precalculate(reference_layers);
    for (const Request &request : requests) {
        CAPTURE(describe(request));
        const size_t lazy_before = capture.count("Had to calculate");
        const Rings expected = rings(fetch(reference, request));
        REQUIRE(capture.count("Had to calculate") == lazy_before);

        Volumes volumes = scene.volumes();
        volumes.precalculate(precalculated_layers);
        const std::string message = lazy_message(volumes, request);
        const size_t before = capture.count(message);
        const Rings actual = rings(fetch(volumes, request));
        // The request took the lazy path, otherwise the comparison would test the caches only.
        REQUIRE(capture.count(message) > before);
        CHECK(actual == expected);
    }
}

// The cancellation state of a fake host: it counts the is_cancelled calls and reports a cancellation from the call number cancel_from
// (0: never) on, or at once when forced. The counter is atomic as the computations run on several threads. When group is set, the
// call number group_from also cancels that TBB group (as a failing sibling task would); the call then reports a host cancellation
// only when group_reports_cancel is set.
struct CancelHost {
    std::atomic<int>  calls{ 0 };
    std::atomic<int>  cancel_from{ 0 };
    std::atomic<bool> forced{ false };
    tbb::task_group_context *group = nullptr;
    int               group_from = 0;
    bool              group_reports_cancel = false;

    plugin_run_context context()
    {
        plugin_run_context out = {};
        out.host_context = reinterpret_cast<plugin_host_context *>(this);
        out.is_cancelled = [](plugin_host_context *host_context) {
            CancelHost &host = *reinterpret_cast<CancelHost *>(host_context);
            const int call = ++ host.calls;
            if (host.group != nullptr && call >= host.group_from) {
                host.group->cancel_group_execution();
                return host.group_reports_cancel ? 1 : 0;
            }
            return (host.forced || (host.cancel_from > 0 && call >= host.cancel_from)) ? 1 : 0;
        };
        return out;
    }
};

// Requests for every kind of area on a layer that precalculate() does not compute. The Slow and FastSafe avoidances, which need the
// holefree collisions, are asked for the radius of a tip, as only the radii below the increase radius have holefree collisions.
std::vector<Request> lazy_requests(const TF::TreeSettings &settings)
{
    const coord_t tip = settings.min_radius, branch = settings.branch_radius;
    std::vector<Request> requests{
        Request{ Kind::Collision, branch, lazy_layer },
        Request{ Kind::Wall,      branch, lazy_layer },
        Request{ Kind::Placeable, branch, lazy_layer },
        Request{ Kind::Avoidance, branch, lazy_layer, true, Volumes::AvoidanceType::Fast, false },
        Request{ Kind::Avoidance, branch, lazy_layer, true, Volumes::AvoidanceType::Fast, true } };
    for (Volumes::AvoidanceType type : { Volumes::AvoidanceType::Slow, Volumes::AvoidanceType::FastSafe })
        for (bool to_model : { false, true })
            requests.push_back(Request{ Kind::Avoidance, tip, lazy_layer, true, type, to_model });
    return requests;
}

// The lazy computations check the context given to the constructor: a request for an area that precalculate() did not compute
// throws PluginCancelled once the host cancels, whatever the call at which it does. A computation cut at any point publishes no
// partial area: once the host stops cancelling, the same request returns the area of the reference.
TEST_CASE("A cancelled context interrupts the lazy computation of an area", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Scene scene;
    const std::vector<Request> requests = lazy_requests(scene.settings);

    Volumes reference = scene.volumes();
    reference.precalculate(reference_layers);

    for (const Request &request : requests) {
        CAPTURE(describe(request));
        const Rings expected = rings(fetch(reference, request));

        // Without cancellation, the request makes some number of checks.
        CancelHost counter;
        const plugin_run_context counting_context = counter.context();
        {
            Volumes volumes = scene.volumes(&counting_context);
            volumes.precalculate(precalculated_layers);
            counter.calls = 0;
            CHECK(rings(fetch(volumes, request)) == expected);
        }
        const int total = counter.calls;
        REQUIRE(total > 1);

        // The host cancels from the k-th check on: the first checks always interrupt the request, a later one may find the request
        // done (the checks run on several threads, so their order varies): the request is cancelled exactly when the host had cancelled
        // by the time it ended (calls >= cancel_from). Either way no partial area is kept.
        for (const int from : { 1, 2, total / 4, total / 2, total - 1, total }) {
            CAPTURE(from, total);
            CancelHost host;
            const plugin_run_context context = host.context();
            Volumes volumes = scene.volumes(&context);
            volumes.precalculate(precalculated_layers);
            host.calls = 0;
            host.cancel_from = std::max(from, 1);
            bool cancelled = false;
            try {
                fetch(volumes, request);
            } catch (const slic3r_api::PluginCancelled &) {
                cancelled = true;
            }
            if (from <= 2)
                CHECK(cancelled);
            CHECK(cancelled == (host.calls.load() >= host.cancel_from.load()));
            host.cancel_from = 0;
            CHECK(rings(fetch(volumes, request)) == expected);
        }
    }
}

// What a call ends with: no exception, the cancellation of the plugin, the error of a cancelled TBB group, or something else.
enum class Outcome { Returned, PluginCancelled, GroupCancelled, Other };
template<class Fn> Outcome outcome_of(Fn &&fn)
{
    try {
        fn();
    } catch (const slic3r_api::PluginCancelled &) {
        return Outcome::PluginCancelled;
    } catch (const std::runtime_error &) {
        return Outcome::GroupCancelled;
    } catch (...) {
        return Outcome::Other;
    }
    return Outcome::Returned;
}

// When the TBB group that runs a call is cancelled, TBB skips the parallel loops nested in the call without any exception. A getter
// whose computation is skipped must throw instead of reading its cache again, which misses again, forever (the stack overflows). The
// group is cancelled on purpose from inside a one-index parallel_for, on one thread, and the missing areas are requested there. In the
// first variant the host reports the cancellation too (PluginCancelled); in the second only the group is cancelled, as when another
// task of the group failed (the call must still end, with the error of the group). Outside the group the same volumes still work.
TEST_CASE("A TBB group that is cancelled interrupts the lazy computation of an area", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Scene scene;
    const std::vector<Request> requests = lazy_requests(scene.settings);

    Volumes reference = scene.volumes();
    reference.precalculate(reference_layers);

    for (const bool host_cancelled : { true, false }) {
        CAPTURE(host_cancelled);
        CancelHost host;
        const plugin_run_context context = host.context();
        Volumes volumes = scene.volumes(&context);
        volumes.precalculate(precalculated_layers);

        std::vector<Outcome> outcomes(requests.size(), Outcome::Returned);
        tbb::task_group_context group;
        tbb::parallel_for(0, 1, [&](int) {
            group.cancel_group_execution();
            host.forced = host_cancelled;
            for (size_t idx = 0; idx < requests.size(); ++ idx)
                outcomes[idx] = outcome_of([&] { fetch(volumes, requests[idx]); });
        }, group);
        host.forced = false;

        for (size_t idx = 0; idx < requests.size(); ++ idx) {
            CAPTURE(describe(requests[idx]));
            CHECK(outcomes[idx] == (host_cancelled ? Outcome::PluginCancelled : Outcome::GroupCancelled));
            // Nothing partial was published: outside the cancelled group the area is the one of the reference.
            CHECK(rings(fetch(volumes, requests[idx])) == rings(fetch(reference, requests[idx])));
        }
    }
}

// The group may also be cancelled in the middle of a computation, by the k-th check of the host (a sibling task failing at any point):
// the loops already running are cut short, the later ones are skipped, and the request ends, either complete (the cancellation came
// after its last check) or by an exception, without hanging, crashing or publishing a partial area. A host that also reports the
// cancellation ends it with PluginCancelled; one that does not, with the error of the group. Outside the group the area is the one
// of the reference.
TEST_CASE("A TBB group that is cancelled during a lazy computation leaves no partial area", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Scene scene;
    const std::vector<Request> requests = lazy_requests(scene.settings);

    Volumes reference = scene.volumes();
    reference.precalculate(reference_layers);

    for (const Request &request : requests) {
        CAPTURE(describe(request));
        const Rings expected = rings(fetch(reference, request));

        // Without cancellation, the request makes some number of checks.
        CancelHost counter;
        const plugin_run_context counting_context = counter.context();
        {
            Volumes volumes = scene.volumes(&counting_context);
            volumes.precalculate(precalculated_layers);
            counter.calls = 0;
            CHECK(rings(fetch(volumes, request)) == expected);
        }
        const int total = counter.calls;
        REQUIRE(total > 1);

        for (const bool host_reports : { true, false })
            for (const int from : { 1, 2, total / 4, total / 2, total - 1, total }) {
                CAPTURE(host_reports, from, total);
                tbb::task_group_context group;
                CancelHost host;
                const plugin_run_context context = host.context();
                Volumes volumes = scene.volumes(&context);
                volumes.precalculate(precalculated_layers);
                host.calls = 0;
                host.group = &group;
                host.group_from = std::max(from, 1);
                host.group_reports_cancel = host_reports;
                Outcome outcome = Outcome::Returned;
                tbb::parallel_for(0, 1, [&](int) { outcome = outcome_of([&] { fetch(volumes, request); }); }, group);
                // The first checks always fall inside the computation, so it cannot have returned.
                if (from <= 2)
                    CHECK(outcome == (host_reports ? Outcome::PluginCancelled : Outcome::GroupCancelled));
                else
                    CHECK(outcome != Outcome::Other);
                if (! host_reports)
                    CHECK(outcome != Outcome::PluginCancelled);
                host.group = nullptr;
                CHECK(rings(fetch(volumes, request)) == expected);
            }
    }
}// The routing asks for the wider base of a branch near the build plate with the minimum XY distance, and caches hold exactly the
// radii that precalculate() computes: the foot target of every layer must be covered, or the routing computes it on demand from its
// parallel loops. The XY spacing of the scene is large, so the minimum XY bucket of the base is not the regular one.
TEST_CASE("Precalculation covers the wider base of a branch at the minimum XY distance", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Scene scene;
    const LogCapture capture;
    constexpr TF::LayerIndex top_layer = 10;
    Volumes volumes = scene.volumes();
    volumes.precalculate(top_layer);
    size_t widened_layers = 0;
    for (TF::LayerIndex layer = 1; layer <= top_layer; ++ layer) {
        if (scene.settings.foot_target[layer] <= double(scene.settings.branch_radius))
            continue;
        const coord_t radius = coord_t(scene.settings.foot_target[layer]);
        ++ widened_layers;
        CAPTURE(layer, radius);
        const size_t lazy_before = capture.count("Had to calculate");
        volumes.getCollision(radius, layer, true);
        volumes.getAvoidance(radius, layer, Volumes::AvoidanceType::Fast, false, true);
        volumes.getAvoidance(radius, layer, Volumes::AvoidanceType::Fast, true, true);
        CHECK(capture.count("Had to calculate") == lazy_before);
        if (widened_layers == 1) {
            // The regular XY variant of the same foot layer is covered as well.
            volumes.getCollision(radius, layer, false);
            volumes.getAvoidance(radius, layer, Volumes::AvoidanceType::Fast, false, false);
            CHECK(capture.count("Had to calculate") == lazy_before);
        }
    }
    // The scene has a base to cover on the layers considered.
    CHECK(widened_layers > 0);
}
// A merge is refused when the merged collision radius needs a higher cache step than both branches, with the cache step of the XY
// variant of the merge (ceilRadius() differs between the minimum and the regular XY distance).
TEST_CASE("Tree merge is refused only when the merged radius crosses a higher cache step", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Scene scene;
    const Volumes volumes = scene.volumes();
    for (const bool use_min_xy_dist : { true, false }) {
        CAPTURE(use_min_xy_dist);
        const coord_t low = scene.settings.branch_radius;
        const coord_t low_ceil = volumes.ceilRadius(low, use_min_xy_dist);
        // The first radius, in steps of 0.01 mm, that needs a higher cache step than the branch radius.
        coord_t high = low;
        while (volumes.ceilRadius(high, use_min_xy_dist) == low_ceil)
            high += scale_i(0.01);
        const coord_t high_ceil = volumes.ceilRadius(high, use_min_xy_dist);
        REQUIRE(high_ceil > low_ceil);
        // Within one step: a slightly larger merged radius in the same step is accepted.
        CHECK_FALSE(TF::merge_crosses_cache_step(volumes.ceilRadius(high - scale_i(0.01), use_min_xy_dist), low_ceil, low_ceil));
        CHECK_FALSE(TF::merge_crosses_cache_step(low_ceil, low_ceil, low_ceil));
        // A higher step than both parents: refused.
        CHECK(TF::merge_crosses_cache_step(high_ceil, low_ceil, low_ceil));
        // One parent already in the higher step: accepted, from either side.
        CHECK_FALSE(TF::merge_crosses_cache_step(high_ceil, low_ceil, high_ceil));
        CHECK_FALSE(TF::merge_crosses_cache_step(high_ceil, high_ceil, low_ceil));
        // A smaller merged step is accepted too.
        CHECK_FALSE(TF::merge_crosses_cache_step(low_ceil, high_ceil, high_ceil));
    }
}
// Many threads request keys that are not precalculated at once, in their own order. Whatever the interleaving of the lazy
// computations, every thread must get the reference areas and no computation may fail. The interleavings come from the timing of
// the threads, so this complements the deterministic tests above and cannot prove their absence; each iteration starts from fresh
// caches to give the race many chances. The error lines of the lazy path widen the windows and are also what the test checks for.
// It takes about 8 s in Debug, so it is also tagged [slow]: the repository runs the slow cases when Plugins/Support (Common, Grid,
// Tree) change and before each commit.
TEST_CASE("Concurrent requests for areas that are not precalculated agree with the reference", "[plugins][support-tree-routing][tree-volumes][slow]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    constexpr int iterations = 40, thread_count = 8;
    Scene scene;
    const coord_t radius = scene.settings.branch_radius;
    std::vector<Request> requests;
    for (TF::LayerIndex layer = lazy_layer - 1; layer <= lazy_layer + 1; ++ layer) {
        requests.push_back(Request{ Kind::Collision, radius, layer, true });
        requests.push_back(Request{ Kind::Collision, radius, layer, false });
        requests.push_back(Request{ Kind::Avoidance, radius, layer, true, Volumes::AvoidanceType::Fast, false });
        requests.push_back(Request{ Kind::Avoidance, radius, layer, true, Volumes::AvoidanceType::Slow, false });
        requests.push_back(Request{ Kind::Avoidance, radius, layer, true, Volumes::AvoidanceType::Fast, true });
        requests.push_back(Request{ Kind::Placeable, radius, layer });
        requests.push_back(Request{ Kind::Wall,      radius, layer, false });
        requests.push_back(Request{ Kind::Wall,      radius, layer, true });
    }

    const LogCapture capture;
    Volumes reference = scene.volumes();
    reference.precalculate(reference_layers);
    std::vector<Rings> expected;
    for (const Request &request : requests)
        expected.push_back(rings(fetch(reference, request)));

    int completed = 0;
    for (int iteration = 0; iteration < iterations; ++ iteration) {
        Volumes volumes = scene.volumes();
        volumes.precalculate(precalculated_layers);
        std::atomic<int>  ready{ 0 };
        std::atomic<bool> go{ false };
        std::mutex        failures_mutex;
        std::vector<std::string> failures;
        std::vector<std::thread> threads;
        for (int thread = 0; thread < thread_count; ++ thread)
            threads.emplace_back([&, thread] {
                // Each thread asks for all the keys in its own order.
                std::vector<size_t> order(requests.size());
                for (size_t idx = 0; idx < order.size(); ++ idx)
                    order[idx] = idx;
                std::mt19937 random(unsigned(iteration * 131 + thread));
                std::shuffle(order.begin(), order.end(), random);
                ++ ready;
                while (! go.load())
                    std::this_thread::yield();
                for (size_t idx : order) {
                    try {
                        if (! same(fetch(volumes, requests[idx]), expected[idx])) {
                            std::lock_guard<std::mutex> lock(failures_mutex);
                            failures.push_back("different area: " + describe(requests[idx]));
                        }
                    } catch (const std::exception &error) {
                        std::lock_guard<std::mutex> lock(failures_mutex);
                        failures.push_back(std::string(error.what()) + ": " + describe(requests[idx]));
                    }
                }
            });
        // Release all the threads together.
        while (ready.load() < thread_count)
            std::this_thread::yield();
        go = true;
        for (std::thread &thread : threads)
            thread.join();
        std::string all_failures;
        for (const std::string &failure : failures)
            all_failures += failure + "\n";
        INFO("iteration " << iteration << ":\n" << all_failures);
        CHECK(failures.empty());
        ++ completed;
        if (! failures.empty())
            break;
    }

    // Every iteration started from fresh caches, so the first request of each kind took the lazy path.
    CHECK(capture.count("Had to calculate collision at radius ") >= size_t(completed));
    CHECK(capture.count("Had to calculate Avoidance at radius ") >= size_t(completed));
    CHECK(capture.count("Had to calculate Avoidance to model at radius ") >= size_t(completed));
    CHECK(capture.count("Had to calculate Placeable Areas at radius ") >= size_t(completed));
    CHECK(capture.count("Had to calculate Wall restrictions at radius ") >= size_t(completed));
}
