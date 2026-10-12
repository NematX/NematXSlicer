#include "support_grid_test_helpers.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeSettings.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeVolumes.hpp"

#include <algorithm>
#include <atomic>
#include <exception>
#include <mutex>
#include <random>
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

/* TreeVolumes tests. TreeVolumes caches the collision, avoidance, placeable and wall restriction areas that the tree routing reads.
   precalculate() fills the caches for the keys (radius, layer) a tree normally requests; any other key is computed when it is first
   requested, by whichever thread asks, while other threads read and fill the same caches. The tests check what that lazy path
   must guarantee:
   - an area computed on demand equals the one of a precalculated reference (deterministic, one thread);
   - the placeable areas of radius 0 are placeable areas also when nothing was precalculated;
   - a reference returned by a getter stays valid and unchanged when its cache grows to a later layer;
   - many threads requesting keys that are not precalculated all get the reference areas (stress test).
   After precalculate(), a lazy computation logs an error line "Had to calculate ...". The tests capture the log to assert that a
   request really took the lazy path, so that they do not silently test the caches only. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
namespace TF = Slic3r::TreeFamily;

namespace {
using Volumes = TF::TreeVolumes;

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
        settings = TF::TreeSettings(print.object(0), SG::params(print, print.object(0)));
    }
    const PrintObject &object() const { return print.object(0); }
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
const Polygons &fetch(const Volumes &volumes, const Request &request)
{
    switch (request.kind) {
    case Kind::Collision: return volumes.getCollision(request.radius, request.layer, request.min_xy_dist);
    case Kind::Avoidance: return volumes.getAvoidance(request.radius, request.layer, request.type, request.to_model, request.min_xy_dist);
    case Kind::Placeable: return volumes.getPlaceableAreas(request.radius, request.layer, [] {});
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

// The areas are computed by the same deterministic code from the same outlines, so equal areas are identical polygons.
bool same(const Polygons &a, const Polygons &b) { return a == b; }
}

// The placeable areas of radius 0 are computed together with the collision of radius 0. A cold cache must not hand the collision back
// as placeable areas: the first call, a second call and the precalculated reference give the same areas.
TEST_CASE("Cold radius 0 placeable areas are placeable areas and not collisions", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Scene scene;
    Volumes reference(scene.object(), scene.settings);
    reference.precalculate(reference_layers, [] {});
    // A layer where the placeable areas exist and differ from the collision, so that mixing them up shows.
    TF::LayerIndex layer = -1;
    for (TF::LayerIndex idx = 2; idx <= reference_layers && layer < 0; ++ idx) {
        const Polygons &placeable = reference.getPlaceableAreas(0, idx, [] {});
        if (! placeable.empty() && ! same(placeable, reference.getCollision(0, idx, true)))
            layer = idx;
    }
    REQUIRE(layer >= 0);
    const Polygons expected  = reference.getPlaceableAreas(0, layer, [] {});
    const Polygons collision = reference.getCollision(0, layer, true);

    Volumes cold(scene.object(), scene.settings);
    const Polygons first  = cold.getPlaceableAreas(0, layer, [] {});
    const Polygons second = cold.getPlaceableAreas(0, layer, [] {});
    CHECK_FALSE(same(first, collision));
    CHECK(same(first, expected));
    CHECK(same(second, expected));
}

// Without resting on the model nobody places a branch on it, so no placeable area exists. The request still has to return, as it did
// before the placeable areas of radius 0 were re-read after their computation: it gives the collision.
TEST_CASE("Radius 0 placeable areas without resting on the model return the collision", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Scene scene(true);
    REQUIRE_FALSE(scene.settings.support_rests_on_model);
    Volumes volumes(scene.object(), scene.settings);
    const Polygons placeable = volumes.getPlaceableAreas(0, lazy_layer, [] {});
    Volumes other(scene.object(), scene.settings);
    CHECK(same(placeable, other.getCollision(0, lazy_layer, true)));
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
        Volumes volumes(scene.object(), scene.settings);
        const Polygons &held    = fetch(volumes, small);
        const Polygons *address = &held;
        const Polygons  content = held;
        // The later layer is not cached yet: the cache grows to hold it.
        fetch(volumes, large);
        const Polygons &again = fetch(volumes, small);
        // The content is only read when the object is still the one that was returned: a destroyed one must not be touched.
        const bool stable = &again == address;
        CHECK(stable);
        if (stable)
            CHECK(same(held, content));
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
            Volumes volumes(scene.object(), scene.settings);
            const Polygons avoidance = fetch(volumes, request);
            const Polygons collision = volumes.getCollision(0, lazy_layer, true);
            // To the model, the places where a branch can stand are free.
            const Polygons expected = to_model ? diff(collision, volumes.getPlaceableAreas(0, lazy_layer, [] {})) : collision;
            if (type == Volumes::AvoidanceType::Fast)
                CHECK(same(avoidance, expected));
            else
                // The slow and the holefree avoidances start from the holefree collision, which is not public: it contains the
                // plain collision.
                CHECK(area(diff(expected, avoidance)) <= sqr(scale_d(0.01)));
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
    const TF::TreeSettings settings(print.object(0), SG::params(print, print.object(0)));
    REQUIRE(settings.xy_distance == settings.xy_min_distance);
    REQUIRE(settings.min_radius == settings.branch_radius);
    REQUIRE(settings.increase_radius_until_radius == settings.branch_radius);

    Volumes volumes(print.object(0), settings);
    volumes.precalculate(reference_layers, [] {});
    CHECK(! volumes.getCollision(settings.branch_radius, 1, true).empty());
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
    Volumes reference(scene.object(), scene.settings);
    reference.precalculate(reference_layers, [] {});
    for (const Request &request : requests) {
        CAPTURE(describe(request));
        const size_t lazy_before = capture.count("Had to calculate");
        const Polygons expected = fetch(reference, request);
        REQUIRE(capture.count("Had to calculate") == lazy_before);

        Volumes volumes(scene.object(), scene.settings);
        volumes.precalculate(precalculated_layers, [] {});
        const std::string message = lazy_message(volumes, request);
        const size_t before = capture.count(message);
        const Polygons actual = fetch(volumes, request);
        // The request took the lazy path, otherwise the comparison would test the caches only.
        REQUIRE(capture.count(message) > before);
        CHECK(same(actual, expected));
    }
}

// The routing asks for the wider base of a branch near the build plate with the minimum XY distance, and caches hold exactly the
// radii that precalculate() computes: every radius of the base must be covered, or the routing computes it on demand from its
// parallel loops. The XY spacing of the scene is large, so the minimum XY bucket of the base is not the regular one.
TEST_CASE("Precalculation covers the wider base of a branch at the minimum XY distance", "[plugins][support-tree-routing][tree-volumes]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Scene scene;
    const LogCapture capture;
    constexpr TF::LayerIndex top_layer = 10;
    Volumes volumes(scene.object(), scene.settings);
    volumes.precalculate(top_layer, [] {});
    size_t widened_layers = 0;
    for (TF::LayerIndex layer = 1; layer <= top_layer; ++ layer) {
        const coord_t radius = scene.settings.recommendedMinRadius(layer);
        if (radius <= 0)
            continue;
        ++ widened_layers;
        CAPTURE(layer, radius);
        const size_t lazy_before = capture.count("Had to calculate");
        volumes.getCollision(radius, layer, true);
        volumes.getAvoidance(radius, layer, Volumes::AvoidanceType::Fast, false, true);
        volumes.getAvoidance(radius, layer, Volumes::AvoidanceType::Fast, true, true);
        CHECK(capture.count("Had to calculate") == lazy_before);
    }
    // The scene has a base to cover on the layers considered.
    CHECK(widened_layers > 0);
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
    Volumes reference(scene.object(), scene.settings);
    reference.precalculate(reference_layers, [] {});
    std::vector<Polygons> expected;
    for (const Request &request : requests)
        expected.push_back(fetch(reference, request));

    int completed = 0;
    for (int iteration = 0; iteration < iterations; ++ iteration) {
        Volumes volumes(scene.object(), scene.settings);
        volumes.precalculate(precalculated_layers, [] {});
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
