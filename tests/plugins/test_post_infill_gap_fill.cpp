#include <catch2/catch.hpp>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerIslandAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepPostInfillGeneration.hpp"
#include "libslic3r/Surface.hpp"

/* Exercise the built-in post-infill pass on small stamped surfaces in a real
   sliced object. Explicit pattern paths make residual widths reproducible and
   separate preservation of old content from variable-width generation.

       GapFixture -> sliced cube and perimeter-owned free areas
           add_surface -> source recipe, optionally one inherited-flow path
           post -> normal STEP_POST_INFILL dispatch and cleanup
           check_gaps -> source scope, allowed domain and generated attributes

   The publication test instead calls the real pattern generator on several
   empty solid surfaces, so it covers recipe survival between publications. */
namespace {
using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;
constexpr const char *k_gap = "infill.post_process.gap_fill";

// Limit each run to named providers and restore the shared runtime afterward.
class ActiveGapPlugins {
public:
    explicit ActiveGapPlugins(std::initializer_list<const char *> ids)
    {
        auto &orch = Orchestrator::instance();
        m_old.assign(orch.active_plugins().begin(), orch.active_plugins().end());
        orch.clear_active_plugins();
        for (const char *id : ids)
            REQUIRE(orch.set_plugin_active(id, true));
    }
    // Restore activation so the next fixture sees the original shared runtime.
    ~ActiveGapPlugins()
    {
        auto &orch = Orchestrator::instance();
        orch.clear_active_plugins();
        for (auto *plugin : m_old) orch.set_plugin_active(plugin, true);
    }
private:
    std::vector<Plugin *> m_old;
};

// Own a prepared print; the selected region remains stable during post-infill.
struct GapFixture {
    PreparedPerimeterPrint prepared;
    LayerRegionIsland *region = nullptr;
    Layer *layer = nullptr;

    // Use explicit limits so the narrow residual is not suppressed by defaults.
    explicit GapFixture(std::initializer_list<std::pair<std::string, std::string>> overrides = {})
    {
        DynamicPrintConfig config = perimeter_config({
            {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
            {"perimeters", "1"}, {"infill_filled_solid", "1"}, {"infill_filled_top", "1"},
            {"infill_filled_bottom", "1"}, {"gap_fill_enabled", "1"},
            {"gap_fill_min_area", "0"}, {"gap_fill_min_length", "0"},
            {"gap_fill_min_width", "0.1"}, {"gap_fill_max_width", "0.8"}, {"gap_fill_extension", "0"}});
        for (const auto &entry : overrides)
            config.set_deserialize(entry.first, entry.second);
        prepare_cube_print(prepared, config);
        ActiveGapPlugins active({SIMPLE_PERIMETER_GENERATOR});
        Steps::StepGeneratePerimeter::run_step(Orchestrator::instance(), prepared.print);
        layer = &prepared.print.objects().front().layers()[2];
        region = &layer->islands().front().regions_island(0);
        region->set_fill_surfaces().clear();
        region->mutable_extrusion(LayerRegionIsland::INFILLS).clear();
    }

    // Stamp a narrow surface and optionally add an inherited-flow old path.
    uint64_t add_surface(SurfaceType type, bool old_path = false, float mult = 1.f, double y = 0.)
    {
        const ExPolygon area = rectangle_expolygon(-4., y - (old_path ? .4 : .15), 4., y + (old_path ? .4 : .15));
        region->set_fill_surfaces().surfaces.emplace_back(type, area);
        const uint64_t id = region->fill_surfaces().surfaces.back().id();
        ExtrusionEntity tree(ExtrusionEntity::Children{}, true, false, false);
        auto &recipe = tree.get_or_add_property<ExtrusionPropertyInfill>();
        recipe.source_surface_id = id;
        recipe.surface_type = uint16_t(type);
        recipe.height = scale_i(.2);
        recipe.spacing = scale_i(.45 - .2 * (1. - PI / 4.));
        recipe.spacing_ratio = 1.f;
        recipe.density = 1.f;
        recipe.flow_mult = mult;
        if (old_path) {
            auto &a = tree.get_or_add_property<ExtrusionAttributes>();
            a.role = ExtrusionRole::SolidInfill;
            a.width = .45f;
            a.height = .2f;
            a.mm3_per_mm = .2 * (.45 - .2 * (1. - PI / 4.));
            tree.get_or_add_property<ExtrusionPropertyLoopRole>().perimeter_idx = 7;
            ExtrusionEntity path(true);
            path.set_polyline(ArcPolyline(Points{Point(scale_i(-4.), scale_i(y - .2)), Point(scale_i(4.), scale_i(y - .2))}));
            tree.append_child(std::move(path));
        }
        region->mutable_extrusion(LayerRegionIsland::INFILLS).append_child(std::move(tree));
        return id;
    }

    // Invoke the host step with only the requested pass enabled.
    void post(bool correct_volume = false)
    {
        ActiveGapPlugins active({k_gap});
        if (correct_volume)
            REQUIRE(Orchestrator::instance().set_plugin_active("infill.post_process.full_fill_volume", true));
        Steps::StepPostInfillGeneration::run_step(Orchestrator::instance(), prepared.print);
    }
};

// Measure the effective flow, including attributes stored on ancestor scopes.
double inherited_volume(const ExtrusionEntity &tree, const ExtrusionAttributes *inherited = nullptr)
{
    const auto *a = tree.get_property<ExtrusionAttributes>();
    if (!a) a = inherited;
    double volume = 0.;
    if (tree.has_polyline()) {
        REQUIRE(a != nullptr);
        volume = unscaled(tree.polyline_ref().length()) * a->mm3_per_mm;
    }
    for (const auto &child : tree.children())
        volume += inherited_volume(*child, a);
    return volume;
}

// Check generated leaves and sum volume, resolving inherited attributes.
double check_gaps(const ExtrusionEntity &tree, double min_y, double max_y,
    const ExtrusionAttributes *inherited = nullptr)
{
    CHECK(tree.get_property<ExtrusionPropertyLoopRole>() == nullptr);
    const auto *a = tree.get_property<ExtrusionAttributes>();
    if (!a) a = inherited;
    double volume = 0.;
    if (tree.has_polyline()) {
        REQUIRE(a != nullptr);
        CHECK(a->role == ExtrusionRole::GapFill);
        CHECK(a->mm3_per_mm > 0.);
        CHECK(a->width > 0.f);
        CHECK(a->height == Approx(.2));
        for (const Point &p : tree.polyline_ref().to_polyline().points) {
            CHECK(unscaled(p.y()) >= min_y - EPSILON);
            CHECK(unscaled(p.y()) <= max_y + EPSILON);
            CHECK(std::abs(unscaled(p.x())) <= 4. + EPSILON);
        }
        volume += unscaled(tree.polyline_ref().length()) * a->mm3_per_mm;
    }
    for (const auto &child : tree.children())
        volume += check_gaps(*child, min_y, max_y, a);
    return volume;
}
} // namespace

TEST_CASE("Infill gap fill completes a stamped surface without changing its old content", "[plugins][infill][gap-fill]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    GapFixture f;
    const uint64_t id = f.add_surface(stPosInternal | stDensSolid, true);
    auto &bucket = f.region->mutable_extrusion(LayerRegionIsland::INFILLS);
    const auto old_id = bucket.child(0).child(0).get_id();
    const double before = inherited_volume(bucket);
    f.post();
    REQUIRE(bucket.child_count() == 1);
    const auto &tree = bucket.child(0);
    REQUIRE(tree.get_property<ExtrusionPropertyInfill>() != nullptr);
    CHECK(tree.get_property<ExtrusionPropertyInfill>()->source_surface_id == id);
    REQUIRE(tree.child_count() == 2);
    CHECK_FALSE(tree.can_sort());
    const auto &old = tree.child(0);
    CHECK(old.child(0).get_id() == old_id);
    CHECK(inherited_volume(old) == Approx(before).epsilon(1e-12));
    REQUIRE(old.get_property<ExtrusionPropertyLoopRole>() != nullptr);
    CHECK(old.get_property<ExtrusionPropertyLoopRole>()->perimeter_idx == 7);
    CHECK(old.get_property<ExtrusionPropertyInfill>() == nullptr);
    CHECK(check_gaps(tree.child(1), 0., .4) > 0.);
    CHECK(inherited_volume(bucket) > before);
}

TEST_CASE("Infill gap fill respects activation and excludes sparse and bridge surfaces", "[plugins][infill][gap-fill]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto type = GENERATE(stPosTop | stDensSolid, stPosBottom | stDensSolid,
        stPosInternal | stDensSolid, stPosInternal | stDensSparse, stPosBottom | stDensSolid | stModBridge);
    const bool enabled = GENERATE(false, true);
    GapFixture f({{"infill_filled_top", enabled ? "1" : "0"}, {"infill_filled_bottom", enabled ? "1" : "0"},
        {"infill_filled_solid", enabled ? "1" : "0"}});
    f.add_surface(type);
    f.post();
    const bool expected = enabled && (type & stDensSolid) && !(type & stModBridge);
    CHECK(f.region->has_extrusion(LayerRegionIsland::INFILLS) == expected);
    if (expected)
        CHECK(check_gaps(f.region->extrusion(LayerRegionIsland::INFILLS), -.15, .15) > 0.);
}

TEST_CASE("Infill gap fill thresholds and global switch suppress additions", "[plugins][infill][gap-fill]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto setting = GENERATE(std::make_pair("gap_fill_enabled", "0"),
        std::make_pair("gap_fill_min_width", "0.5"), std::make_pair("gap_fill_max_width", "0.05"),
        std::make_pair("gap_fill_min_area", "100"), std::make_pair("gap_fill_min_length", "100"));
    INFO(setting.first);
    GapFixture f({{setting.first, setting.second}});
    f.add_surface(stPosInternal | stDensSolid);
    f.post();
    CHECK_FALSE(f.region->has_extrusion(LayerRegionIsland::INFILLS));
}

TEST_CASE("Infill gap fill leaves sparse bridge and unstamped paths intact", "[plugins][infill][gap-fill]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    GapFixture f;
    f.add_surface(stPosInternal | stDensSparse, true, 1.f, -2.);
    f.add_surface(stPosBottom | stDensSolid | stModBridge, true, 1.f, 0.);
    f.add_surface(stPosInternal | stDensSolid, true, 1.f, 2.);
    auto &bucket = f.region->mutable_extrusion(LayerRegionIsland::INFILLS);
    bucket.child(2).remove_property<ExtrusionPropertyInfill>();
    const double before = inherited_volume(bucket);
    std::vector<uint64_t> ids;
    for (const auto &child : bucket.children()) ids.push_back(child->child(0).get_id());
    f.post();
    REQUIRE(bucket.child_count() == 3);
    CHECK(inherited_volume(bucket) == Approx(before).epsilon(1e-12));
    for (size_t c = 0; c < bucket.child_count(); ++c) {
        REQUIRE(bucket.child(c).child_count() == 1);
        CHECK(bucket.child(c).child(0).get_id() == ids[c]);
        CHECK(bucket.child(c).get_property<ExtrusionPropertyLoopRole>() != nullptr);
    }
}

TEST_CASE("Infill gap fill applies the source multiplier once", "[plugins][infill][gap-fill]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    GapFixture f;
    f.add_surface(stPosInternal | stDensSolid, false, 1.f, -1.);
    f.add_surface(stPosInternal | stDensSolid, false, .7f, 1.);
    f.post();
    const auto &bucket = f.region->extrusion(LayerRegionIsland::INFILLS);
    REQUIRE(bucket.child_count() == 2);
    const double a = check_gaps(bucket.child(0), -1.15, -.85);
    const double b = check_gaps(bucket.child(1), .85, 1.15);
    CHECK(a > 0.);
    CHECK(b == Approx(a * .7).epsilon(1e-5));
}

TEST_CASE("Full fill correction measures the pattern and its added gaps together", "[plugins][infill][gap-fill]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    GapFixture f;
    f.add_surface(stPosInternal | stDensSolid, true);
    auto &tree = f.region->mutable_extrusion(LayerRegionIsland::INFILLS).child(0);
    tree.get_property<ExtrusionPropertyInfill>()->area_to_fill_mm2 = 6.4;
    f.post();
    const auto &completed = f.region->extrusion(LayerRegionIsland::INFILLS).child(0);
    const double before = inherited_volume(completed);
    const double target = 6.4 * .2;
    const double ratio = std::max(.8, std::min(1.3, target / before));
    const double old_volume = inherited_volume(completed.child(0));
    ActiveGapPlugins active({"infill.post_process.full_fill_volume"});
    Steps::StepPostInfillGeneration::run_step(Orchestrator::instance(), f.prepared.print);
    CHECK(inherited_volume(completed) == Approx(before * ratio).epsilon(1e-6));
    CHECK(inherited_volume(completed.child(0)) == Approx(old_volume * ratio).epsilon(1e-6));
}

TEST_CASE("Infill generation preserves multiple successful empty solid recipes", "[plugins][infill][gap-fill]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    GapFixture f({{"solid_fill_pattern", "rectilinear"}, {"top_fill_pattern", "rectilinear"}});
    f.add_surface(stPosInternal | stDensSolid, false, 1.f, -1.);
    f.add_surface(stPosInternal | stDensSolid, false, 1.f, 1.);
    f.region->mutable_extrusion(LayerRegionIsland::INFILLS).clear();
    ActiveGapPlugins active({"infill.generator.default", "rectilinear"});
    Steps::StepGenerateInfill::run_step(Orchestrator::instance(), f.prepared.print);
    auto &bucket = f.region->mutable_extrusion(LayerRegionIsland::INFILLS);
    REQUIRE(bucket.child_count() == 2);
    for (const auto &child : bucket.children()) {
        REQUIRE(child->get_property<ExtrusionPropertyInfill>() != nullptr);
        CHECK(child->empty());
    }
    f.post();
    CHECK(f.region->has_extrusion(LayerRegionIsland::INFILLS));
    CHECK(check_gaps(bucket.child(0), -1.15, -.85) > 0.);
    CHECK(check_gaps(bucket.child(1), .85, 1.15) > 0.);
}

TEST_CASE("Infill gap fill clips modifier settings within one shared region island", "[plugins][infill][gap-fill]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool disable = GENERATE(false, true);
    GapFixture f;
    f.add_surface(stPosInternal | stDensSolid);
    LayerRegion &left = f.layer->region(0);
    ApiInternal::LayerRegionAccess::slices_mutable(left) = {rectangle_expolygon(-10., -10., 0., 10.)};
    PrintRegionConfig right_config = left.region().config();
    right_config.set_deserialize_strict(disable ? "infill_filled_solid" : "gap_fill_min_width", disable ? "0" : "0.5");
    ApiInternal::LayerAccess::add_region(*f.layer, f.prepared.print.register_print_region(right_config));
    LayerRegion &right = f.layer->region(f.layer->region_count() - 1);
    ApiInternal::LayerRegionAccess::slices_mutable(right) = {rectangle_expolygon(0., -10., 10., 10.)};
    f.layer->island(0).fill_regions(*f.layer);
    auto &shared = f.layer->island(0).get_or_add_region_island({&left, &right});
    shared.set_fill_surfaces() = std::move(f.region->set_fill_surfaces());
    shared.mutable_extrusion(LayerRegionIsland::INFILLS) = std::move(f.region->mutable_extrusion(LayerRegionIsland::INFILLS));
    f.region = &shared;
    f.post();
    REQUIRE(shared.has_extrusion(LayerRegionIsland::INFILLS));
    const auto &tree = shared.extrusion(LayerRegionIsland::INFILLS);
    CHECK(check_gaps(tree, -.15, .15) > 0.);
    std::vector<const ExtrusionEntity *> pending{&tree};
    while (!pending.empty()) {
        const auto *node = pending.back();
        pending.pop_back();
        if (node->has_polyline())
            for (const Point &p : node->polyline_ref().to_polyline().points)
                CHECK(unscaled(p.x()) <= EPSILON);
        for (const auto &child : node->children())
            pending.push_back(child.get());
    }
}

TEST_CASE("Infill gap fill respects the perimeter-free mask including holes", "[plugins][infill][gap-fill]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    GapFixture f;
    f.add_surface(stPosInternal | stDensSolid);
    ExPolygon allowed = rectangle_expolygon(-3., -.15, 3., .15);
    allowed.holes.push_back(rectangle_expolygon(-.5, -.1, .5, .1).contour);
    allowed.holes.back().reverse();
    ApiInternal::LayerIslandAccess::infill_free_areas_mutable(f.layer->island(0)) = {allowed};
    f.post();
    REQUIRE(f.region->has_extrusion(LayerRegionIsland::INFILLS));
    const auto &tree = f.region->extrusion(LayerRegionIsland::INFILLS);
    CHECK(check_gaps(tree, -.15, .15) > 0.);
    std::vector<const ExtrusionEntity *> pending{&tree};
    while (!pending.empty()) {
        const auto *node = pending.back();
        pending.pop_back();
        if (node->has_polyline()) {
            CHECK(diff_pl(node->polyline_ref().to_polyline(), allowed).empty());
            for (const Point &p : node->polyline_ref().to_polyline().points) {
                CHECK(std::abs(unscaled(p.x())) <= 3. + EPSILON);
                CHECK_FALSE((std::abs(unscaled(p.x())) < .5 - EPSILON && std::abs(unscaled(p.y())) < .1 - EPSILON));
            }
        }
        for (const auto &child : node->children())
            pending.push_back(child.get());
    }
}
